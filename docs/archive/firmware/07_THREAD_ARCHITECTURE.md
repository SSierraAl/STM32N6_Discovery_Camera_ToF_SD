# Thread Architecture

All facts below are verified against `Src/main.c` (`main_freertos`, task
creation) and `Src/app_thread.c` (task bodies, IPC).

## 1. RTOS setup

* FreeRTOS, tick rate **1 kHz** (`configTICK_RATE_HZ 1000`, 1 tick = 1 ms).
* **Every task is statically allocated** (`xTaskCreateStatic` + static stack
  and TCB buffers) — no dynamic heap for tasks; chosen for unattended field
  stability.
* `configMINIMAL_STACK_SIZE = 1024 words = 4 KB`.
* Creation order in `main_freertos()`: I2C arbiter mutex → main → storage →
  camera → sensor → btn (conditional) → `vTaskStartScheduler()`.

## 2. Task table (current code)

| Task | FreeRTOS name | Entry point | Stack | Priority | Created when |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Main init | `main` | `main_thread_fct` (main.c:1008) | 1024 w (4 KB) | IDLE+1 | always |
| Storage | `storage` | `storage_task` (app_thread.c:974) | 6144 w (24 KB) | IDLE+2 | `!TEST_TOF_MODE` |
| Camera | `camera` | `camera_task` (app_thread.c:848) | 4096 w (16 KB) | IDLE+3 | `!TEST_TOF_MODE` |
| Sensor | `sensor` | `sensor_task` (app_thread.c:353) | 4096 w (16 KB) | IDLE+4 (highest) | `CAPTURE_MODE != 0` |
| Button | `btn` | `btn_thread_fct` (main.c:766) | 4096 w (16 KB) | IDLE+3 | `CAPTURE_MODE == 0` only |

Priorities are relative (only the idle task runs lower). `sensor_task` is the
highest so a detection is never delayed behind SD writes. The production
configuration (`CAPTURE_MODE 4`, `TEST_TOF_MODE 0`) runs **four tasks**:
main, storage, camera, sensor. `btn` does not exist in production.

## 3. Task lifecycle

* Every worker task spins on `while (!system_ready) vTaskDelay(500 ms);`
  before touching hardware.
* `main_thread_fct` performs all board initialization (doc 06 §4), sets
  `system_ready = 1`, and terminates itself.
* `sensor_task` additionally runs the ToF bring-up after `system_ready`:
  `VL53L5CX_Init(&hi2c1)` → `VL53L5CX_Configure(4x4, 30 ms, 15 Hz)` →
  `VL53L5CX_StartRanging()` → `VL53L5CX_LearnBaseline()` (15 settle frames +
  20 samples ≈ 2.5 s). If the primary baseline is not ready it parks in
  `SENSOR_STATE_STOPPED` forever.

## 4. IPC objects (created by `IPC_Init`, app_thread.c:81)

| Object | Type / depth | Producer | Consumer | Payload | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `camera_cmd_queue` | queue, 4 × `CameraCmd_t` | sensor_task | camera_task | `CAM_CMD_SNAP` (+ unused legacy cmds) | "take a burst" trigger |
| `storage_cmd_queue` | queue, 8 × `StorageCmd_t` | camera_task | storage_task | PSRAM pointer, size, W/H, format, `snap_id`, RTC stamp | one job per captured frame |
| `sensor_event_queue` | queue, 4 × `CameraEvent_typed` | (reserved) | sensor_task | camera events | created for notifications; the mode-4 flow uses the semaphores/flags below instead |
| `camera_ready_sem` | binary | camera_task | sensor_task | — | burst finished (success **or** failure) |
| `storage_done_sem` | counting, max 8 | storage_task | sensor_task | — | one token per persisted frame (success or failure) |

Shared flags (`app_thread.c`):

| Flag | Owner | Meaning |
| :--- | :--- | :--- |
| `system_ready` | main_thread | boot gate for all workers |
| `g_capture_busy` | sensor_task | 1 while a full capture+storage cycle is in progress; new triggers ignored |
| `g_sensor_state` | sensor_task | `SENSOR_STATE_IDLE / RUNNING / PAUSED / STOPPED` |
| `g_last_batch_frames` | camera_task | frames actually captured (0 → sensor skips storage wait) |
| `g_last_storage_rc` | storage_task | last SD write result |
| `g_next_snap_id` | camera_task | monotonic ID assigned **before** dispatch; a failed write leaves an ID gap |
| `g_sd_img_base_block` | storage_task | logical write cursor, starts at `SD_SNAP_BASE_BLOCK` (3072) |

## 5. What each task does

### 5.1 `sensor_task` (ToF monitoring + capture orchestration)

Loop (single-sensor path, the active one):

1. If `g_sensor_state == PAUSED` → delay 50 ms, continue (used during
   in-loop baseline refresh).
2. Cooldown countdown (one step per loop iteration).
3. `VL53L5CX_Update()` — waits for the next ToF frame (~67 ms at 15 Hz) and
   runs the detection engine (doc 08 §1). Inside `Update`, the **periodic
   baseline re-learn** (every 1000 frames, quiet-gated) can restart the
   sensor and re-learn all zones.
4. `WS2812_OffWatchdog()` — every second when no flash is active, re-sends a
   full OFF frame (WS2812 has no electrical readback; this repairs a
   mis-decoded LED state). Silent.
5. Baseline-refresh requests from the detector (persistent stable drift) →
   `StopRanging`, 50 ms, `StartRanging`, 200 ms, `LearnBaseline`, cooldown 5,
   then a 3 s re-arm holdoff (only level/fast events accepted during it).
6. On detection (`insect_detected && cooldown == 0 && !g_capture_busy`):
   * `g_capture_busy = 1`, `g_sensor_state = PAUSED`, take result
     (`VL53L5CX_GetResult`: trigger source + affected zones),
   * GREEN off / RED on,
   * `WS2812_FlashStart(WS2812_ILLUMINATION_COLOR, WS2812_ILLUMINATION_BRIGHTNESS)`,
   * update camera-activation counters (adaptive refresh),
   * **block** in `Capture_RequestSnapshot(60000)` — the whole pipeline
     (doc 07 §6.2),
   * `VL53L5CX_ZoneDetectorAfterCapture()` (drops stale comparisons, keeps
     live latches),
   * RED off / GREEN on,
   * if activation max reached → full baseline refresh + 3 s holdoff,
     otherwise re-arm ranging and open a fresh 30 s window,
   * `g_sensor_state = RUNNING`, `g_capture_busy = 0`, cooldown = 5.
7. `vTaskDelay(5 ms)` — loop period ≈ one ToF frame + 5 ms.

While blocked in `Capture_RequestSnapshot` the ToF sensor keeps ranging
autonomously (AUTONOMOUS mode) but no results are evaluated.

### 5.2 `camera_task` (acquisition)

1. Polls `camera_cmd_queue` with a 20 ms timeout (modes 1/2 also service the
   ISP in the idle loop; mode 4 does not need it).
2. On `CAM_CMD_SNAP` (mode 4): `CAM_CallbackBatchSnap(batch_buf, frame_size)`
   — wake sensor → start pipe → 8 warmup frames via DCMIPP frame-event
   callback → copy 4 frames into `batch_buf` slots → stop pipe once → standby.
3. Gives `camera_ready_sem` **on success and on failure** (so the sensor task
   always unblocks).
4. On success: one `RTC_Stamp` via `RTC_CaptureStamp()`, assigns
   `snap_id = g_next_snap_id .. +rc`, and for each frame builds a
   `StorageCmd_t` whose `capture_tick` is the exact DCMIPP completion tick
   (`CAM_GetCallbackFrameTick(f)`) → `storage_cmd_queue`.
   If the storage queue is full a frame is dropped (logged, not fatal).

### 5.3 `storage_task` (SD writes)

1. `xQueueReceive(storage_cmd_queue, portMAX_DELAY)` — idle-waits for work.
2. `SD_StoreRawImage(...)`: waits for card-ready (CMD13, max 5 s), then writes
   the 64-byte header + raw YUV422 in PSRAM-staged batches of up to 1024
   blocks (512 KB), with D-cache clean before every DMA write and a
   CMD13-poll + `SD_BATCH_RECOVERY_GAP_MS` gap between batches (doc 08 §5).
   All writes pass through the journal shim, which reserves the physical
   append range first (doc 08 §5.3).
3. On failure: `SD_Reinit()` (abort → deinit → SDMMC2 peripheral reset →
   reinit with validated clock → 4-bit bus) and **one retry of the same
   image** (the logical cursor was restored, so the retry lands in the same
   place). Still failing → image lost, another `SD_Reinit()` leaves the card
   in a clean state.
4. Gives `storage_done_sem` exactly once per job — success **or** failure.

### 5.4 `btn_thread_fct` (mode 0 only — not present in production)

Polls PC13 (`BTN_POLL_DELAY_MS 10`, debounce `BTN_DEBOUNCE_MS 20`), runs a
manual full-resolution capture and stores it with a separate static
`SD_StoreRawImage` copy in main.c (logical base block 1000, its own cursor).

### 5.5 `main_thread_fct` (init only)

See doc 06 §4. It never touches the camera/ToF/SD after `system_ready`.

## 6. Synchronization rules

1. **Startup gate.** No worker runs before `system_ready = 1`; hardware init
   is therefore single-threaded and race-free.
2. **One capture at a time.** `g_capture_busy` is set by `sensor_task` before
   the request and cleared only after **all storage tokens** are back. New
   ToF triggers during the cycle are ignored by design (the insect is being
   photographed).
3. **Capture budget.** `Capture_RequestSnapshot(60000)` bounds the whole
   camera+SD cycle to 60 s; on timeout it returns `-1` and the fail-safe
   `WS2812_FlashStop()` still runs on every return path
   (`Capture_ReturnWithIlluminationOff`).
4. **Shared I2C1.** Recursive mutex (`I2C_Arbiter_Init` before tasks). Both
   the camera middleware handle and the ToF raw `hi2c1` handle must take it;
   camera pre-init in the main thread additionally removes the boot-time
   conflict entirely.
5. **D-cache discipline.** `SCB_CleanDCache_by_Addr` before every SD DMA write
   (PSRAM staging buffer) and before journal metadata writes;
   `SCB_InvalidateDCache_by_Addr` before the CPU reads DCMIPP-DMA'd frame data
   or journal records.
6. **One-wire light bus.** WS2812 has no acknowledgement; the driver sends
   redundant complete frames (3× OFF, 1 retry) and the idle watchdog
   re-transmits OFF once per second (doc 08 §4).
7. **ID semantics.** `snap_id` gaps mean a frame was dropped or its write
   failed twice — expected and logged, not an error state.

## 7. Task sets per configuration

| Configuration | Tasks created |
| :--- | :--- |
| **Production** (`CAPTURE_MODE 4`, `TEST_TOF_MODE 0`) | main, storage, camera, sensor |
| ToF commissioning (`TEST_TOF_MODE 1`) | main, sensor — camera and SD are never initialized |
| Manual camera (`CAPTURE_MODE 0`) | main, storage, camera, btn — ToF disabled, no sensor task |

