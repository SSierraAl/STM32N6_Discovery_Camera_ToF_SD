# Algorithms — How the System Works

Every value below is the current field configuration (`Inc/app_config.h` and
`Src/vl53l5cx_detection.c`). Tuning guidance lives in `09_TUNING_PARAMETERS.md`.

## 1. ToF detection (VL53L5CX, production 4×4 path)

### 1.1 Measurement

* Sensor: single VL53L5CX @ 0x29, **4×4 grid (16 zones)**, AUTONOMOUS ranging
  mode, integration 30 ms, **15 Hz**, target order **CLOSEST** (a tiny floor
  insect is closer than the floor; STRONGEST would pick the floor).
* Each frame yields per zone: signal return rate (kcps), distance (mm),
  motion value. Motion is produced inside the sensor by ST's motion
  indicator plugin (10-frame temporal accumulation, min 1 zone, extra noise 0;
  the plugin natively monitors the 400–1500 mm band).
* Zones with signal below `VL53L5CX_DET_MIN_SIGNAL` (500 kcps) are **blind**:
  excluded from baseline learning and from detection.

### 1.2 Baseline

`VL53L5CX_LearnBaseline()` discards 15 warm-up frames
(`VL53L5CX_DET_BASELINE_SETTLE_FRAMES`) then collects 20 samples
(`VL53L5CX_DET_BASELINE_SAMPLES`) per zone. Learning is always done with the
trap empty; the refresh mechanisms in §2 guarantee it stays that way.

### 1.3 Per-frame processing

1. **Common-mode removal**: a shift that affects many zones coherently
   (trap vibration, temperature) is subtracted before any per-zone test —
   global movement of the box is not an insect.
2. Per zone, two local residuals are computed:
   * `local_signal` — signal drop vs baseline, in %,
   * `local_distance` — distance change **toward the sensor**, in mm.
3. The residuals are tested against the evidence classes below.

### 1.4 Evidence classes (what may fire a photo)

| Class | Condition (per frame) | Frames to fire | Notes |
| :--- | :--- | :--- | :--- |
| Strong level | `local_distance ≥ 5 mm` (or signal > 3 % combined with distance) | **1** | Immediate trigger |
| Fast edge | signal ≥ 2 % **and** distance 3–4 mm | 2 | Needs confirmation in the following frame; catches fast small objects |
| Floor sustain | floor zone, distance ≥ 1 mm toward sensor **and** signal ≥ 2 % | 4 | For slow insects that never make a sharp edge |
| Floor score | floor zone qualifying (signal ≥ 2 % + distance ≥ 1 mm) | 3 (score +4/frame, trigger at 12) | Score survives a blocked interval (cooldown) without resetting to zero |
| Micro-persistence | floor zone, evidence = signal ≥ 1 % **or** distance ≥ 2 mm | ≈18 net (see below) | Slow same-zone accumulator for a settled tiny insect |
| Weak track | 2–3 zones in a 5 s window, each with recent per-zone motion ≥ 1 % / 1 mm | — | **Disabled** (`VL53L5CX_DET_WEAK_TRACK_ENABLED 0`): cross-zone 2 % noise tripped it in field logs |

"Floor zone" = a zone within `VL53L5CX_DET_FLOOR_DEPTH_BAND_MM` (30 mm) in
depth of the farthest valid zone — i.e. the trap floor, not the close walls.

**Micro-persistence math** (the anti-1-frame-spike filter): score starts at 0,
`+2` on an evidence frame, `−1` on a quiet frame, frozen while the level
policy is off (cooldown/holdoff), triggers at 32 and fires a level event with
SIGNAL source. A settled insect (evidence in ~90 % of frames) nets ≈ +1.7/frame
→ fires in ≈ 1.2–1.3 s. Random empty-box noise (evidence in ~20 % of frames,
hopping between zones) nets ≈ −0.4/frame → stays pinned near 0. The per-frame
evidence bit is OR-ed into the raw zone mask, so a present insect is never
quietly re-baselined while it sits there.

### 1.5 Latching and de-duplication

* A detected zone stays **latched** while the same signal offset persists
  (cleared after 3 clear frames; a zone must be quiet 15 frames at < 4 % to
  drop out completely), so one insect = one photo, not one per frame.
* A weak capture may escalate **once** to a strong capture in the same zone;
  strong captures remain one-per-evidence-episode.
* After every capture, `VL53L5CX_ZoneDetectorAfterCapture()` discards stale
  comparisons (the task was blocked ~8 s) but preserves live latches — a full
  reset here would re-trigger the same insect after every photo.

### 1.6 Trigger output

`VL53L5CX_GetResult()` returns `trigger_source` (1 SIGNAL, 2 MOTION, 3 S+M,
4 DISTANCE, 5 S+D, 6 M+D, 7 S+M+D) plus affected zones and their drops. With
`VL53L5CX_DET_EVENT_TRACE 1` every accepted capture logs one compact
`TOFEVT ... micro=<mask> ... z=<zone>:<sig>:<dist>` line independent of
`PERF_DEBUG_LEVEL`.

### 1.7 Cooldown

5 sensor-task iterations after each capture or baseline refresh
(≈0.35 s at 15 Hz). In `TEST_TOF_MODE` the cooldown is 30 (~2 s) for manual
observation.

## 2. Baseline adaptation (how the detector stays calibrated for months)

Slow environmental change (dust, temperature, a permanently blocking object)
shifts a zone's baseline. If the new baseline is not re-learned, the shift
itself becomes "evidence" and the trap photographs nothing repeatedly. Four
mechanisms bound that problem:

| # | Mechanism | Where | Rule | Cost |
| :--- | :--- | :--- | :--- | :--- |
| 1 | **Periodic re-learn ("recursive baseline")** | inside `VL53L5CX_Update()` | Every 1000 Update frames (~66 s) restart the sensor and re-learn **all** zones — **only while the trap is quiet**: skipped while an insect event is active or while any zone is latched/blocked, and retried on the next frame if the sensor is busy | ~2.5 s ToF blind (15 settle + 20 sample frames) |
| 2 | **Per-zone stable-plateau recentering** | zone detector (`vl53l5cx_detection.c`) | A zone whose residual stays small (≤ 1 % signal **and** ≤ 1 mm distance per frame) accumulates stable frames inside a 12 s window; 8 or more stable frames recenter **that zone's** baseline only. Counting inside the window (instead of requiring unbroken stability) tolerates normal 1 %/1 mm jitter | 1 zone corrected; bounds a static-bias episode to about one photo |
| 3 | **Drift refresh (detector-requested)** | `sensor_task` | The detector sets a refresh request when coherent persistent drift is seen → `StopRanging`, 50 ms, `StartRanging`, 200 ms, `LearnBaseline`, cooldown 5, then 3 s holdoff during which only level/fast events are accepted | full re-learn |
| 4 | **Camera-activation refresh** | `sensor_task` | 3 completed camera activations inside a rolling 30 s window force one full baseline refresh + 3 s holdoff (high-sensitivity override; the generic values are 2 in 5 s for other modes). Bounds any false-trigger loop to 3 photos | full re-learn |

Current build runs **1 and 4 together** (1 re-baselines slowly drifting zones
between events; 4 stops burst false-triggering). Manual refresh (USER button
→ `VL53L5CX_RefreshBaseline_Manual`) is available in `TEST_TOF_MODE` builds.

Internal recentering constants (code constants in `vl53l5cx_detection.c`,
not in `app_config.h`): `TOF_TEST_SETTLE_MS 12000`,
`TOF_TEST_STABLE_MIN_FRAMES 8`, `TOF_TEST_STABLE_WINDOW_FRAMES 180`,
`TOF_TEST_CLEAR_FRAMES 3`, `TOF_TEST_QUIET_FRAMES 15`,
`TOF_TEST_QUIET_SIGNAL_PCT 4`, `TOF_TEST_SCENE_SETTLE_MS 5000`.

## 3. Camera acquisition (mode 4, CALLBACK-BATCH)

1. **Boot**: `CAM_CallbackInit()` runs the full CMW-IMX335 init + warmup once,
   then parks the sensor in **standby** (sensor draws no power, pipe stopped).
2. **Trigger**: `CAM_CallbackBatchSnap(batch_buf, frame_size)` (one production
   frame = 2,531,328 B):
   1. wake the sensor (~20 ms), apply manual exposure 15 ms + gain 12 dB
      (AEC disabled in manual mode),
   2. start the DCMIPP pipe (30 FPS, 1296×972 YUV422 — the sensor still reads
      full resolution; "binning" here is DCMIPP down-scaling, not sensor
      binning, so rolling-shutter readout is unchanged),
   3. wait for frame events from the **DCMIPP ISR callback**
      (`CAM_NotifyFrameEvent` → `g_frame_event_count`): 8 warmup frames are
      discarded (first frames after wake are not trustworthy),
   4. for 4 frames: wait for the frame-complete event → copy that frame from
      its ping-pong buffer (`capture_buf`/`save_buf`) into the next
      `batch_buf` slot. The pipe **never stops between frames** → no tearing,
      all 4 frames sharp,
   5. stop the pipe once, return the sensor to standby.
3. The exact HAL tick of each frame completion is kept and handed to the
   storage queue, so the 4 stored frames carry their true sub-second order
   (see §6).
4. Total trigger→burst ≈ 20 ms + 12 × 33 ms ≈ 0.4 s.

Other modes: 0 = full init/warmup/deinit per button press (2592×1944);
1 = pipe always running, one frame copied on trigger; 2 = stop/restart
between frames (tearing — deprecated).

## 4. Illumination (WS2812 light ring)

* 12 WS2812 LEDs, driven by TIM1 PWM + GPDMA1. Waveform constants are
  hardware-validated in `ws2812.h` (period 500, "1" = 80, "0" = 30,
  1700 reset entries). Wire format GRB; the API takes 0xRRGGBB and converts
  once.
* **Brightness is RGB scaling, not PWM duty** — the LEDs are physically
  on for the whole exposure window, so brightness is perfectly in sync with
  even short exposures and raising it has no flicker cost.
* **Reliable strobe**: a finished DMA transfer only proves the waveform was
  sent, not that the LEDs decoded it. The ON frame is sent with 1 retry; the
  OFF frame is sent **3 times** with 1 retry because leaving the light on is
  the unsafe failure mode. Transfers never overlap.
* **Capture coupling** (`WS2812_MODE 1`, production):
  `WS2812_FlashStart(color, brightness)` on detection → light covers the
  entire wake+warmup+burst+SD-write cycle → `WS2812_FlashStop()` right after
  `camera_ready_sem`, plus a fail-safe re-stop on every
  `Capture_RequestSnapshot` return path.
* **Idle watchdog**: `WS2812_OffWatchdog()` re-sends a full OFF frame once
  per second while no flash is active (called from `sensor_task`), silently
  repairing a pixel that mis-decoded an earlier OFF.
* Color: `0xC8B080` warm white (max illumination for the camera is pure
  white; warm white reduces visible blue flash). `WS2812_MODE 0` = light on
  for the whole cycle via `TurnOn/TurnOff`; `WS2812_MODE 2` = brief
  indicator flash only (`WS2812_INDICATOR_MS`).

## 5. Storage (SD card)

### 5.1 Image record format (`sd_image_header_t`, exactly 64 bytes, enforced by `_Static_assert`)

| Offset (bytes) | Field | Value / meaning |
| :--- | :--- | :--- |
| 0 | `magic` | `0x49444745` (`SD_HEADER_TAG`) |
| 4 | `width` | 1296 (production) |
| 8 | `height` | 972 (production) |
| 12 | `pixel_format` | 0 = YUV422 (2 B/px); 2 = 1 B/px |
| 16 | `data_size` | width × height × bpp (must match, checked by journal) |
| 20 | `timestamp` | UTC seconds (DS3231) |
| 24 | `checksum` | reserved |
| 28 | `snap_id` | monotonic capture ID (gaps = dropped/failed frames) |
| 32 | `time_tag` | `0x31435254` (`RAW_TIME_TAG`) |
| 36 | `time_flags` | bit 0 = RTC valid, bit 1 = post-capture RTC sample |
| 40 | `capture_tick` | exact DCMIPP frame-completion tick (sub-second frame order) |
| 44 | `reserved[20]` | zero |

Followed by the raw YUV422 frame bytes. No filesystem: the host decoder
(`datalogger.py` / SD viewer) reads these records directly.

### 5.2 Batched write algorithm (`SD_StoreRawImage`)

Images are appended from the logical cursor `g_sd_img_base_block`
(starts at `SD_SNAP_BASE_BLOCK` = 3072):

1. `SD_WaitForReady()` — poll **CMD13** until the card reports TRANSFER state
   (max 5 s). The HAL software state is meaningless here: it is READY while
   the card's NAND is still programming, and writing then causes
   `STA=0x5000` (data CRC timeout).
2. Fill the 512 KB PSRAM staging buffer: header + first chunk, then up to
   1024 blocks of image data.
3. `SCB_CleanDCache_by_Addr` → `HAL_SD_WriteBlocks` (one 512 KB transfer).
4. Repeat for the remainder; between batches: CMD13 wait +
   `SD_BATCH_RECOVERY_GAP_MS` fixed gap (0 ms currently — the gap exists for
   slow cards where CMD13 "ready" is still optimistic).

One 2.5 MB frame ≈ 4,945 blocks ≈ 5 batch transfers.

### 5.3 Persistent A/B append journal (`sd_storage_journal.c`)

The app writes with **logical** block addresses that restart at 3072 after
every MCU reset. A transparent shim (installed by `raw_image.h`, which is
included first in `main.c` and `app_thread.c`, renaming `HAL_SD_Init` /
`HAL_SD_WriteBlocks` to the `SDJ_*` variants) translates them into a
**persistent physical append position**, so photos from previous sessions
are never overwritten:

* Journal records live in two 512-byte slots: **block A = 3070**,
  **block B = 3071**. Record = 28 bytes: magic `"SDJ1"`, version 1,
  sequence, `next_block`, `next_snap_id`, `snap_base`, CRC32 (verified).
* **Reserve first, write second**: before an image's data blocks are touched,
  the successor record (sequence+1, new `next_block`, new `next_snap_id`) is
  written to the *other* slot with **read-after-write verification**. Power
  loss can therefore leave an unused or partially written block range, but
  an older image is never clobbered.
* A/B slots are re-read **before every new image** — a hot-swapped card or a
  journal edited on the PC can never inherit the old card's append pointer.
* Boot/recovery: newest valid sequence wins; if neither slot is valid
  (new/cleared card) a fresh journal is created at block 3072 / ID 0.
  If the append position would exceed the card size the image is refused
  ("Card full") — `SD_MAX_SNAPSHOTS` is only a safety constant.
* Writes to blocks **below 3072** (boot benchmark) pass through unmapped.
* `SDJ_HAL_SD_Init` (used by `SD_Reinit`) resets journal state, forces the
  validated `ClockDiv=4`, and retries up to 3 times with an SDMMC2 peripheral
  reset between attempts.

### 5.4 Failure handling

`storage_task`: write failure → `SD_Reinit()` (peripheral-level reset +
reinit + 4-bit bus config, preserving the logical cursor) → **one retry of
the same image** → if it fails again the image is lost (logged
`[SD] FAIL ... image LOST after retry`) and the card is left re-initialized.
A `storage_done_sem` token is given in every case, so the capture pipeline
never deadlocks on a dead card.

### 5.5 Capacity

Blocks per production image: `(64 + 2,531,328 + 511) / 512 = 4,945`
(≈ 2.53 MB). A 32 GB card (≈ 64,000,000 blocks) holds roughly **12,900
production images** before the journal refuses the next append.

## 6. Timestamps (DS3231 RTC)

* DS3231 on I2C, battery-backed UTC, 1-second resolution.
* Provisioning: set `RTC_SET_ON_BOOT 1` + `RTC_SET_UTC "<iso>"`, flash once,
  set back to 0 — normal boots then always preserve the battery-backed time
  (no build-time clock).
* Each capture takes one UTC sample (`RTC_CaptureStamp`, post-capture) and
  each of the 4 burst frames stores its own DCMIPP completion tick in
  `capture_tick`, so the frames keep their true sub-second ordering inside
  the 64-byte header.

## 7. Host-side tooling (offline only, not part of the firmware)

| Tool | Consumes | Used for |
| :--- | :--- | :--- |
| `datalogger.py` | `ALLPARAM` console frames | full per-zone parameter logging during calibration |
| `zone_monitor.py` | `ZFRAME` console frames | compact per-zone live monitoring |
| `analysis.py` | saved logs/images | offline statistics |
| raw SD viewer | SD card records (§5.1) | decoding the YUV422 images |


