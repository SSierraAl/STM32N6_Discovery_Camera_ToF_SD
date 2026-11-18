# Capture Modes Reference Guide
## Detailed Comparison of All Operating Modes

---

## Table of Contents

1. [Mode Overview](#1-mode-overview)
2. [Mode 0: On-Demand (Button)](#2-mode-0-on-demand-button)
3. [Mode 1: Continuous Camera + ToF](#3-mode-1-continuous-camera--tof)
4. [Mode 2: Batch (Stop/Restart)](#4-mode-2-batch-stoprestart)
5. [Mode 4: Callback-Batch (Zero-Copy, Recommended)](#5-mode-4-callback-batch-zero-copy-recommended)
6. [Mode Comparison Matrix](#6-mode-comparison-matrix)
7. [Mode Selection Flowchart](#7-mode-selection-flowchart)
8. [Troubleshooting by Mode](#8-troubleshooting-by-mode)

---

## 1. Mode Overview

Set the capture mode in `app_config.h`:
```c
#define CAPTURE_MODE  4  /* 0 = On-Demand | 1 = Continuous | 2 = Batch | 4 = Callback-Batch */
```

| Mode | Name | Trigger | ToF During Capture | Camera State Between Captures | Frames per Trigger | Resolution |
|------|------|---------|-------------------|-------------------------------|-------------------|------------|
| **0** | On-Demand | USER Button | OFF (task not created) | Powers off completely | 1 | Full (2592×1944) |
| **1** | Continuous | ToF Detection | PAUSED during capture | Streaming (never stops) | 1 | Binned (1296×972) |
| **2** | Batch | ToF Detection | PAUSED during capture | Streaming (never stops) | 3 | Binned (1296×972) |
| **4** | Callback-Batch | ToF Detection | PAUSED during capture | Standby (powers down) | 4 | Binned (1296×972) |

**Mode 3 was removed** (was Standby-Batch, replaced by Callback-Batch mode 4).

---

## 2. Mode 0: On-Demand (Button)

### Purpose
Manual single-frame capture triggered by pressing the USER button (PC13). Maximum resolution, no ToF sensor, simplest operation.

### Configuration
```c
#define CAPTURE_MODE            0
#define CAM_BINNING             0     /* Full resolution: 2592×1944 */
```

### Tasks Created
| Task | Active? | Notes |
|------|---------|-------|
| main_thread | Yes | Board init, then self-destructs |
| btn_thread | Yes | Polls USER button (PC13) |
| camera_task | Yes | Idle (no commands sent to it) |
| storage_task | Yes | Handles SD writes from btn_thread |
| sensor_task | **NO** | Not created when CAPTURE_MODE == 0 |

### Flow Diagram
```
[BOOT]
  main_thread:
    → Hardware init (clocks, PSRAM, SD, I2C, UART)
    → SD card init + benchmark
    → IPC_Init() (queues, semaphores)
    → NO camera pre-init (camera stays off)
    → system_ready = 1
    → vTaskDelete()

[BUTTON PRESS]
  btn_thread:
    → WS2812_TurnOn()
    → RED LED on
    │
    → CAM_CaptureSingleFrame(capture_buf, ...)
      │
      ├── CMW_CAMERA_Init()          ← Full init every time (I2C, sensor, DCMIPP)
      ├── CMW_CAMERA_Start()         ← Continuous mode
      ├── Wait warmup frames (11)    ← Discarded (~363ms)
      ├── Wait 1 more frame          ← Captured (~33ms)
      ├── HAL_DCMIPP_CSI_PIPE_Stop() ← Stop pipe
      ├── HAL_Delay(5ms)             ← Pipeline drain
      ├── SCB_InvalidateDCache()     ← Make image visible to CPU
      └── CMW_CAMERA_DeInit()        ← Power down camera
      │
    → SD_StoreRawImage(capture_buf, ...)
      ├── Write header (64 bytes)
      ├── Write image data (9,677,376 bytes = 2592×1944×2)
      └── Advance g_sd_img_base_block
      │
    → WS2812_TurnOff()
    → GREEN LED on
    → Wait for button release
```

### Timing Breakdown (Full Resolution 2592×1944)
| Phase | Duration | Percentage |
|-------|----------|------------|
| Camera init | ~100ms | 1.0% |
| Warmup (11 frames) | ~363ms | 3.6% |
| Capture (1 frame) | ~33ms | 0.3% |
| Camera deinit | ~23ms | 0.2% |
| **Camera total** | **~519ms** | **5.1%** |
| SD write (10 MB, 19684 blocks) | ~9500ms | 93.8% |
| **Total** | **~10019ms** | **100%** |

### SD Card Usage (per image)
| Resolution | Bytes | Blocks | Batches (64 blk) | Write Time |
|-----------|-------|--------|-------------------|------------|
| 2592×1944 | 9,677,376 | 18,908 | 296 | ~9500ms |
| 1296×972 | 2,519,424 | 4,921 | 77 | ~2500ms |

### Pros and Cons
| ✅ Pros | ❌ Cons |
|---------|---------|
| Maximum resolution (2592×1944) | Very slow (10s per capture) |
| No ToF sensor needed (saves power) | Manual trigger only |
| Simplest operation | Camera init overhead every time |
| No I2C conflicts | Not suitable for fast-moving subjects |
| PSRAM efficient (one buffer) | Large SD writes |

### Use Cases
- Static object photography
- Maximum detail requirements
- Calibration/test shots
- When ToF sensor is unavailable

---

## 3. Mode 1: Continuous Camera + ToF

### Purpose
Camera runs continuously at lower resolution. ToF detects insect → camera freezes one frame → SD write. Fastest capture, both sensors always on.

### Configuration
```c
#define CAPTURE_MODE            1
#define CAM_BINNING             1     /* Auto-set: binned 1296×972 */
```

### Tasks Created
| Task | Active? | Notes |
|------|---------|-------|
| main_thread | Yes | Board init + **camera pre-init** |
| sensor_task | Yes | ToF monitoring, sends capture requests |
| camera_task | Yes | **Services ISP continuously** + handles snaps |
| storage_task | Yes | SD card writes |
| btn_thread | No | Not created for mode 1 |

### Flow Diagram
```
[BOOT]
  main_thread:
    → Hardware init
    → SD card init + benchmark
    → IPC_Init()
    │
    → CAM_ContinuousStart(capture_buf, ...)
      │
      ├── CMW_CAMERA_Init()          ← Init camera (I2C, sensor, DCMIPP)
      ├── CMW_CAMERA_Start()         ← Continuous mode
      ├── Wait warmup frames (11)    ← Discarded
      └── Pipe LEFT RUNNING          ← Key: camera never stops
      │
    → system_ready = 1
    → vTaskDelete()
    │
  sensor_task:
    → VL53L5CX_Init() → StartRanging() → LearnBaseline()
    → Wait 5 seconds (ToF stabilization)
    → Enter monitoring loop

[TOF DETECTION]
  sensor_task:
    → VL53L5CX_IsInsectDetected() == true
    → g_capture_busy = 1
    → g_sensor_state = PAUSED        ← ToF stops polling
    → Capture_RequestSnapshot(60000)
      │
      ├── camera_cmd_queue: CAM_CMD_SNAP
      ├── Wait camera_ready_sem      ← Blocks until camera done
      ├── Wait storage_done_sem      ← Blocks until SD done
      └── Return 0 (success)
    │
    → g_sensor_state = RUNNING       ← ToF resumes
    → cooldown = 30                  ← Skip next 30 readings

[CAMERA SNAP]
  camera_task (receives CAM_CMD_SNAP):
    → CAM_ContinuousSnap(save_buf, frame_size)
      │
      ├── Wait for frame-ready event ← Next completed frame
      │   (pipe still running!)
      ├── SCB_InvalidateDCache()
      └── memcpy(save_buf, capture_buf, frame_size)
      │
    → storage_cmd_queue: STORAGE_CMD_SAVE

[STORAGE]
  storage_task:
    → SD_StoreRawImage(save_buf, ...)
```

### Timing Breakdown (1296×972, single frame)
| Phase | Duration | Notes |
|-------|----------|-------|
| ToF detection → camera_cmd | ~5ms | sensor_task priority 4 |
| Camera: wait 1 frame | ~33ms | Next frame completes |
| Camera: memcpy to save_buf | ~700ms | PSRAM-to-PSRAM copy (slow!) |
| **Camera total** | **~738ms** | |
| SD write (1 image) | ~2500ms | 77 batches × (33ms write + 15ms gap) |
| Cooldown | ~150ms | 30 ToF readings × 5ms |
| **Total cycle** | **~3388ms** | |

### I2C Bus Behavior
```
Time axis:
sensor_task:  VL53L5CX_Update() → VL53L5CX_GetResult() → IsInsectDetected()
                  │               │                        │
                  └── I2C read ───┴── I2C read ────────────┘
                          (every 5-10ms)

camera_task:  CAM_IspUpdate()
                  │
                  └── I2C write (auto exposure/gain, every ~20ms)

I2C Arbiter:  Mutex serializes all I2C operations
              ┌─────────────────────────────────────────┐
              │ sensor_task takes mutex → does I2C →   │
              │ releases mutex                          │
              │ camera_task takes mutex → does I2C →    │
              │ releases mutex                          │
              └─────────────────────────────────────────┘
```

### Pros and Cons
| ✅ Pros | ❌ Cons |
|---------|---------|
| Fastest capture (33ms to frame) | Lower resolution (1296×972) |
| Camera always warm (no re-init) | Requires I2C arbitration |
| ToF continuous monitoring | High power consumption |
| Good for fast-moving insects | 700ms memcpy bottleneck |

### Use Cases
- Insect capture (fast movement)
- Continuous monitoring scenarios
- When minimum capture latency matters

---

## 4. Mode 2: Batch (Stop/Restart)

### Purpose
Camera runs continuously. ToF detection → capture 3 frames using Stop→memcpy→Restart for each frame. More data per detection but slower per-frame.

### Configuration
```c
#define CAPTURE_MODE            2
#define BATCH_FRAMES            3     /* Frames per detection event */
```

### Tasks Created
| Task | Active? | Notes |
|------|---------|-------|
| main_thread | Yes | Board init only (no camera pre-init) |
| sensor_task | Yes | ToF monitoring |
| camera_task | Yes | Starts pipe, then handles batch snaps |
| storage_task | Yes | SD card writes (3 images) |
| btn_thread | No | Not created for mode 2 |

### Flow Diagram
```
[BOOT]
  main_thread:
    → Hardware init (NO camera pre-init)
    → system_ready = 1
    → vTaskDelete()
    │
  camera_task:
    → CAM_ContinuousStart(capture_buf, ...)
      → Full init + warmup + leave pipe RUNNING

[TOF DETECTION → CAPTURE]
  camera_task (receives CAM_CMD_SNAP):
    → CAM_ContinuousBatchSnap(batch_buf, frame_size)
      │
      └── For each of BATCH_FRAMES (3):
          ├── HAL_DCMIPP_CSI_PIPE_Stop()     ← Stop pipe
          ├── for(volatile i; i<100; i++);    ← Brief drain
          ├── SCB_InvalidateDCache()          ← Make visible
          ├── memcpy(batch_buf[i], capture_buf, frame_size)  ← Copy
          ├── SCB_CleanDCache()               ← Prepare for SD
          └── CAM_CapturePipe_Start()         ← Restart pipe
          │
      └── Return 3

  → storage_cmd_queue: × 3 STORAGE_CMD_SAVE

[STORAGE]
  storage_task:
    → SD_StoreRawImage(image #1)
    → SD_StoreRawImage(image #2)
    → SD_StoreRawImage(image #3)
```

### Timing Breakdown (1296×972, 3 frames)
| Phase | Duration | Notes |
|-------|----------|-------|
| Stop + memcpy + restart × 3 | ~3×(738ms) = ~2214ms | Each frame: stop(23ms) + memcpy(700ms) + start(5ms) + wait(33ms)... |
| **Camera total** | **~2214ms** | |
| SD write × 3 images | ~7500ms | 3 × 2500ms |
| Cooldown | ~150ms | |
| **Total cycle** | **~9864ms** | |

### Pros and Cons
| ✅ Pros | ❌ Cons |
|---------|---------|
| 3 frames per detection (more data) | Stop/Restart causes tearing |
| Camera always warm | Very slow capture (2.2s) |
| Simple logic | I2C conflict with ToF |
| Good for slower insects | Insect can escape during capture |

### Use Cases
- Slower-moving subjects
- When 3+ frames needed for analysis
- Development/testing

---

## 5. Mode 4: Callback-Batch (Zero-Copy, Recommended)

### Purpose
**PRODUCTION MODE.** Camera initialized at boot, put in standby. ToF detection → wake camera → start double-buffered pipe → capture 4 frames with **ZERO memcpy** (hardware DMA directly to final buffer) → stop → standby.

### Configuration
```c
#define CAPTURE_MODE              4
#define CALLBACK_WARMUP_FRAMES    11  /* Frames to discard after wake */
#define CALLBACK_FRAMES           4   /* Frames to keep per trigger */
```

### Tasks Created
| Task | Active? | Notes |
|------|---------|-------|
| main_thread | Yes | Board init + **camera callback init** |
| sensor_task | Yes | ToF monitoring |
| camera_task | Yes | ISP servicing + callback batch snaps |
| storage_task | Yes | SD card writes (4 images) |
| btn_thread | No | Not created for mode 4 |

### Flow Diagram (Detailed)
```
[BOOT]
  main_thread:
    → Hardware init
    → SD card init + benchmark
    → IPC_Init()
    │
    → CAM_CallbackInit(capture_buf, ...)
      │
      ├── CMW_CAMERA_Init()
      ├── CMW_CAMERA_Start(capture_buf, CONTINUOUS)
      ├── Wait warmup frames (11)
      ├── HAL_DCMIPP_CSI_PIPE_Stop()
      ├── IMX335 → STANDBY (I2C: 0x3000 = 0x01)
      ├── g_frame_event_count = 0
      └── g_callback_ready = 1
      │
    → system_ready = 1
    → vTaskDelete()

[TOF DETECTION → CAPTURE]
  camera_task (receives CAM_CMD_SNAP):
    → CAM_CallbackBatchSnap(batch_buf, frame_size)
      │
      ├── IMX335 → STREAMING (I2C: 0x3000 = 0x00)
      ├── HAL_Delay(35ms) ← Sensor wake time
      │
      ├── CMW_CAMERA_DoubleBufferStart(capture_buf, save_buf)
      │   → DCMIPP configured for ping-pong DMA:
      │     Address 0 → capture_buf (scratch)
      │     Address 1 → save_buf (scratch)
      │   → Pipe running, alternating buffers
      │
      ├── Re-apply exposure/gain (I2C via I2C mutex)
      │
      ├── total = WARMUP(11) + FRAMES(4) = 15
      ├── frame_seq = 0
      │
      └── For k = 0 to 14:
          │
          ├── CAM_WaitNextFrameReady(120ms)
          │   → Waits for g_frame_event_count to increment
          │   → ISR fires when DCMIPP finishes DMA to PSRAM
          │   → Also calls CAM_IspUpdate() (I2C via mutex)
          │   → frame_seq++ (tracks completed frames)
          │
          ├── out_idx = k - 11
          │
          ├── If k >= 11 (capture frames):
          │   ├── dest = batch_buf[out_idx × frame_size]
          │   ├── SCB_InvalidateDCache(dest, frame_size)
          │   │   → This frame was DMA'd DIRECTLY into batch_buf!
          │   │   → No memcpy needed! (zero-copy)
          │   ├── captured++
          │   │
          │   └── ARM 2 frames ahead:
          │       arm_idx = out_idx + 2
          │       If arm_idx < 4:
          │         parity = (frame_seq - 1) % 2
          │         HAL_DCMIPP_PIPE_SetMemoryAddress(
          │           pipe, parity ? ADDR_1 : ADDR_0,
          │           batch_buf[arm_idx × frame_size])
          │         → Next time DCMIPP wants this address,
          │           it will DMA directly into final buffer
          │
          └── If k < 11 (warmup frames):
              → Frame DMA'd into capture_buf or save_buf (scratch)
              → Discarded, no action needed
              │
      ├── HAL_DCMIPP_CSI_PIPE_Stop()
      ├── IMX335 → STANDBY (I2C: 0x3000 = 0x01)
      └── Return captured (4)
      │
    → xSemaphoreGive(camera_ready_sem)
    → For each of 4 frames:
        storage_cmd_queue: STORAGE_CMD_SAVE

[STORAGE]
  storage_task:
    → SD_StoreRawImage(batch_buf[0], ...)  # Image #0
    → SD_StoreRawImage(batch_buf[1], ...)  # Image #1
    → SD_StoreRawImage(batch_buf[2], ...)  # Image #2
    → SD_StoreRawImage(batch_buf[3], ...)  # Image #3

[SYNC]
  sensor_task (Capture_RequestSnapshot):
    → Wait camera_ready_sem ← Camera finished
    → Wait storage_done_sem × 4 ← All 4 images on SD
    → g_capture_busy = 0
    → g_sensor_state = RUNNING
    → cooldown = 30
```

### Zero-Copy Explained
```
Traditional approach (Mode 2):
  capture_buf ← DCMIPP DMA (continuous)
       │
       │ memcpy (CPU, 2.4MB, ~700ms)  ← SLOW!
       │
  batch_buf[0] ← final destination

Callback-Batch approach (Mode 4):
  Frame 0: DCMIPP → capture_buf (scratch, discarded)
  Frame 1: DCMIPP → save_buf (scratch, discarded)
  ...
  Frame 11 (warmup last): DCMIPP → capture_buf
       │
       │ At this point, reprogram Address 0:
       │ HAL_DCMIPP_PIPE_SetMemoryAddress(ADDR_0, batch_buf[0])
       │
  Frame 12 (capture[0]): DCMIPP → batch_buf[0] ← DIRECT!
  Frame 13 (capture[1]): DCMIPP → batch_buf[1] ← DIRECT!
  Frame 14 (capture[2]): DCMIPP → batch_buf[2] ← DIRECT!
  Frame 15 (capture[3]): DCMIPP → batch_buf[3] ← DIRECT!
  
  NO memcpy needed for capture frames!
  DCMIPP hardware DMA writes directly to final destination.
```

### Timing Breakdown (1296×972, 4 frames)
| Phase | Duration | Notes |
|-------|----------|-------|
| IMX335 wake from standby | 35ms | I2C + HAL_Delay |
| Exposure/gain reconfig | 3ms | I2C write |
| Warmup (11 frames) | 363ms | Hardware DMA to scratch buffers |
| Capture (4 frames, zero-copy) | 132ms | Hardware DMA directly to batch_buf |
| Pipe stop + standby | 23ms | |
| **Camera total** | **~556ms** | **No memcpy overhead!** |
| SD write × 4 images | ~10,000ms | 4 × 2500ms |
| Cooldown | ~150ms | |
| **Total cycle** | **~10,706ms** | |

### Memory Usage
| Buffer | Size | Location | Purpose |
|--------|------|----------|---------|
| capture_buf | 2.5MB | PSRAM | Warmup scratch (double-buffer ping) |
| save_buf | 2.5MB | PSRAM | Warmup scratch (double-buffer pong) |
| batch_buf | 10MB | PSRAM | 4 × 2.5MB final capture destination |
| sd_batch_buf | 32KB | PSRAM | SD card batch write staging |
| **Total** | **~15MB** | | Fits in 16MB PSRAM |

### Pros and Cons
| ✅ Pros | ❌ Cons |
|---------|---------|
| **ZERO memcpy** (hardware DMA direct) | Camera must warm up every trigger (35ms) |
| ALL frames sharp (no pipe restart) | Lower resolution (1296×972) |
| Fastest capture for all frames | Complex memory management |
| Camera in standby between captures (power save) | ToF paused during entire cycle |
| 4 frames per detection | Highest PSRAM usage |
| Production-tested, most reliable mode | |

### Use Cases
- **Primary production mode** for insect capture
- When frame quality is critical (no tearing)
- Multiple frames needed for analysis
- Best balance of speed, quality, and reliability

---

## 6. Mode Comparison Matrix

### Performance Comparison
| Metric | Mode 0 | Mode 1 | Mode 2 | Mode 4 |
|--------|--------|--------|--------|--------|
| **Resolution** | 2592×1944 | 1296×972 | 1296×972 | 1296×972 |
| **Frames per trigger** | 1 | 1 | 3 | 4 |
| **Camera latency** | 519ms | 33ms | 2214ms | 556ms |
| **SD latency (total)** | 9500ms | 2500ms | 7500ms | 10000ms |
| **Total cycle** | 10019ms | 3388ms | 9864ms | 10706ms |
| **Image quality** | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ |
| **PSRAM usage** | 9.7MB | 12MB | 12.5MB | 15MB |

### Feature Comparison
| Feature | Mode 0 | Mode 1 | Mode 2 | Mode 4 |
|---------|--------|--------|--------|--------|
| ToF sensor | ❌ | ✅ | ✅ | ✅ |
| LED sync | ❌ | ✅ | ✅ | ✅ |
| I2C arbitration | ❌ | ✅ | ✅ | ✅ |
| Double buffering | ❌ | ✅ | ❌ | ✅ |
| Zero-copy capture | ❌ | ❌ | ❌ | ✅ |
| Standby between captures | ✅ (full off) | ❌ | ❌ | ✅ |
| Button trigger | ✅ | ❌ | ❌ | ❌ |
| Auto trigger | ❌ | ✅ | ✅ | ✅ |
| SD recovery + retry | ✅ | ✅ | ✅ | ✅ |
| Camera retry on I2C fail | ✅ | ✅ | ✅ | ✅ |
| ToF warmup (5s) | N/A | ✅ | ✅ | ✅ |

### Reliability Comparison
| Issue | Mode 0 | Mode 1 | Mode 2 | Mode 4 |
|-------|--------|--------|--------|--------|
| I2C bus corruption | None (ToF off) | Mitigated (mutex) | Mitigated (mutex) | Mitigated (mutex) |
| SD card CRC errors | Possible (gap) | Possible (gap) | Possible (gap) | Possible (gap) |
| Pipe restart tearing | N/A | N/A | ✅ YES | None |
| Camera init failure | Retry (3x) | Retry (3x) | Retry (3x) | Retry (3x) |
| ToF false boot capture | N/A | Mitigated (5s warmup) | Mitigated (5s warmup) | Mitigated (5s warmup) |
| Bad SD sector | Recovery + skip | Recovery + skip | Recovery + skip | Recovery + skip |

---

## 7. Mode Selection Flowchart

```
Start: What do you need?
│
├─ Manual control? (button press, not autonomous)
│  └─→ MODE 0 (On-Demand)
│      Maximum resolution, simple, slow
│
├─ Autonomous insect detection?
│  │
│  ├─ Fastest single-frame capture needed?
│  │  └─→ MODE 1 (Continuous)
│  │      33ms to frame, lower resolution, continuous monitoring
│  │
│  ├─ Multiple frames per detection?
│  │  │
│  │  ├─ Frame quality critical (no tearing)?
│  │  │  └─→ MODE 4 (Callback-Batch) ⭐ RECOMMENDED
│  │  │      4 frames, zero-copy, all sharp, production use
│  │  │
│  │  └─ Just need more data, quality less important?
│  │     └─→ MODE 2 (Batch)
│  │         3 frames, stop/restart (may have tearing)
│  │
│  └─ Other requirements?
│     See comparison matrix above
```

---

## 8. Troubleshooting by Mode

### Mode 0 Specific
| Symptom | Cause | Fix |
|---------|-------|-----|
| Camera init fails (ret=-7) | I2C corrupted from previous run | Retry logic handles this (3 attempts) |
| SD write fails | Full card or bad sector | SD_Reinit() + retry; check card |
| Image too dark | Full resolution needs more light | Increase CAM_EXPOSURE_VALUE, WS2812 brightness |

### Mode 1 Specific
| Symptom | Cause | Fix |
|---------|-------|-----|
| I2C conflict (camera init fails) | ToF using I2C1 at same time | I2C arbiter should handle; check mutex |
| memcpy too slow (700ms) | PSRAM bandwidth limitation | Acceptable; mode 4 eliminates this |
| Camera stream corrupted | ISP update timing | CAM_IspUpdate() in camera_task loop |

### Mode 2 Specific
| Symptom | Cause | Fix |
|---------|-------|-----|
| First frame sharp, others blurry | Stop/Restart tearing | Use MODE 4 instead |
| Capture takes too long (2.2s) | 3 × (stop + memcpy + restart) | Reduce BATCH_FRAMES or use MODE 4 |
| I2C conflict with ToF | Continuous camera + ToF | I2C arbiter should handle |

### Mode 4 Specific
| Symptom | Cause | Fix |
|---------|-------|-----|
| First capture at boot (no insect) | ToF not stabilized | 5-second warmup built-in |
| Image has wrong brightness | Exposure not re-applied after wake | Already handled in CAM_CallbackBatchSnap |
| Double buffer address not updated | ARM 2 frames ahead logic | Verify g_frame_event_count parity |
| Only 1-2 of 4 images on SD | SD write timeout | Increase SD_BATCH_RECOVERY_GAP_MS |

---

*Capture Modes Reference — Updated July 2026*