# CALLBACK-BATCH Mode (CAPTURE_MODE = 4) — Complete System Flow Analysis

## Overview

CALLBACK-BATCH mode is the optimized capture mode for insect detection on the STM32N6 Discovery board. It combines **standby power management**, **hardware double-buffering**, and **zero-copy DMA** to capture 3 sharp frames (~650ms) without the tearing or glitches of previous modes.

**Key achievement:** All 3 frames are captured from a continuously running DCMIPP pipeline with zero CPU memcpy overhead — the hardware DMA writes each frame directly into its final PSRAM slot.

---

## Architecture at a Glance

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        BOOT SEQUENCE                                    │
│                                                                         │
│  1. main_thread: Init all peripherals                                   │
│     ├── PSRAM (XSPI NOR, 16MB)                                          │
│     ├── SD card (SDMMC2, 4-bit)                                         │
│     ├── I2C1 (VL53L5CX shared bus)                                      │
│     ├── WS2812 (TIM1 PWM + GPDMA1)                                      │
│     └── Camera (IMX335 + DCMIPP) → STANDBY                              │
│  2. system_ready = 1                                                    │
│  3. Tasks start: sensor_task, camera_task, storage_task                 │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                       IDLE STATE                                        │
│                                                                         │
│  Camera: IMX335 in STANDBY (reg 0x3000 = 0x01)                         │
│           DCMIPP pipe STOPPED                                           │
│           I2C1 bus FREE for ToF sensor                                  │
│                                                                         │
│  ToF sensor: VL53L5CX actively ranging every 400ms                     │
│              sensor_task polls VL53L5CX_Update()                        │
│              Checks for insect detection                                │
│                                                                         │
│  LEDs: OFF (except GREEN status LED)                                    │
│  WS2812 illumination: OFF                                               │
│                                                                         │
│  Power consumption: LOW (camera in standby, no DMA active)              │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                              INSECT DETECTED
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    CAPTURE SEQUENCE (~650ms camera)                     │
│                                                                         │
│  ┌─ sensor_task ────────────────────────────────────────────────────┐   │
│  │ 1. VL53L5CX_IsInsectDetected() → true                            │   │
│  │ 2. g_sensor_state = SENSOR_STATE_PAUSED (ToF stops ranging)      │   │
│  │ 3. WS2812_FlashStart() → LEDs ON at 80% brightness               │   │
│  │ 4. Capture_RequestSnapshot(60000) → send CAM_CMD_SNAP via queue  │   │
│  │ 5. Wait for camera_ready_sem + storage_done_sem × 3              │   │
│  │ 6. WS2812_FlashStop() → LEDs OFF                                 │   │
│  │ 7. g_sensor_state = SENSOR_STATE_RUNNING (ToF resumes)           │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─ camera_task ────────────────────────────────────────────────────┐   │
│  │ 1. Receive CAM_CMD_SNAP from camera_cmd_queue                    │   │
│  │ 2. CAM_CallbackBatchSnap(batch_buf, frame_size):                 │   │
│  │    │                                                             │   │
│  │    │  Step 1: Wake sensor                                        │   │
│  │    │    IMX335_REG_MODE_SELECT = 0x00 (STREAMING)               │   │
│  │    │    HAL_Delay(20ms) — sensor stabilization                   │   │
│  │    │                                                             │   │
│  │    │  Step 2: Start DCMIPP double-buffer mode                    │   │
│  │    │    CMW_CAMERA_DoubleBufferStart(pipe, capture_buf,          │   │
│  │    │                               save_buf, CONTINUOUS)        │   │
│  │    │    Hardware now ping-pongs DMA between 2 buffers            │   │
│  │    │                                                             │   │
│  │    │  Step 3: Re-apply exposure/gain                             │   │
│  │    │    SetExposure(80µs), SetGain(500)                          │   │
│  │    │                                                             │   │
│  │    │  Step 4+5: Unified warmup + capture loop (18 frames total)  │   │
│  │    │    for k = 0 to 17:                                         │   │
│  │    │      CAM_WaitNextFrameReady() → ISR callback fires          │   │
│  │    │                                                             │   │
│  │    │      if k < 15 (warmup):                                    │   │
│  │    │        DMA writes to scratch buffers, discarded             │   │
│  │    │                                                             │   │
│  │    │      if k >= 15 (capture):                                  │   │
│  │    │        DMA wrote DIRECTLY to batch_buf[out_idx]             │   │
│  │    │        SCB_InvalidateDCache() — no memcpy needed            │   │
│  │    │        captured++                                           │   │
│  │    │                                                             │   │
│  │    │      Arm 2 frames ahead:                                    │   │
│  │    │        HAL_DCMIPP_PIPE_SetMemoryAddress() → redirect next   │   │
│  │    │        DMA destination to batch_buf[out_idx+2]              │   │
│  │    │                                                             │   │
│  │    │  Step 6: Stop pipe + Return to standby                      │   │
│  │    │    HAL_DCMIPP_CSI_PIPE_Stop()                               │   │
│  │    │    IMX335_REG_MODE_SELECT = 0x01 (STANDBY)                 │   │
│  │    │                                                             │   │
│  │ 3. xSemaphoreGive(camera_ready_sem)                               │   │
│  │ 4. For each frame: send STORAGE_CMD_SAVE to storage_cmd_queue     │   │
│  │    (batch_buf[0..2], 2.5MB each)                                  │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─ storage_task ───────────────────────────────────────────────────┐   │
│  │ For each of 3 frames:                                             │   │
│  │   1. Receive STORAGE_CMD_SAVE from storage_cmd_queue             │   │
│  │   2. SD_StoreRawImage():                                         │   │
│  │      ├── Build 64-byte header (magic, dims, checksum, timestamp) │   │
│  │      ├── Write in 64-block batches (32KB each)                   │   │
│  │      ├── Wait for card ready between batches                     │   │
│  │      └── Total: ~77 HAL_SD_WriteBlocks calls per frame           │   │
│  │   3. xSemaphoreGive(storage_done_sem)                            │   │
│  │                                                                   │   │
│  │   Each frame: ~1900ms (SD card bottleneck)                       │   │
│  │   Total storage: ~5700ms for 3 frames                            │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                       BACK TO IDLE STATE                                │
│                                                                         │
│  Camera: IMX335 in STANDBY (0x3000 = 0x01)                             │
│           DCMIPP pipe STOPPED                                           │
│           I2C1 bus FREE for ToF sensor                                  │
│                                                                         │
│  ToF sensor: Resumed ranging (400ms period)                             │
│  LEDs: OFF                                                              │
│  System ready for next detection                                        │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Detailed Timing Breakdown

### Camera Capture (from your log: 652ms total)

| Phase | Duration | Details |
|-------|----------|---------|
| Wake sensor | ~20ms | I2C write 0x3000=0x00 + HAL_Delay(20) |
| DCMIPP start | ~5ms | DoubleBufferStart + pipe init |
| Exposure/gain | ~4ms | I2C writes + CAM_IspUpdate() |
| Warmup frames (15) | ~520ms | 15 × 33ms per frame |
| Capture frames (3) | ~107ms | 3 × 33ms per frame (zero-copy) |
| Pipe stop + standby | ~23ms | CSI_PIPE_Stop + I2C write 0x3000=0x01 |

### Storage (from your log: ~5700ms for 3 frames)

| Phase | Per Frame | Total (×3) |
|-------|-----------|------------|
| SD card write | ~1900ms | ~5700ms |
| Wait (card ready) | 60ms | 180ms |
| DMA transfer | 432ms | 1296ms |
| Inter-batch gap | 3420ms | 10260ms |

### Total Cycle Time: ~6895ms

```
Total: 6895ms
├─ Camera capture:    652ms   (9.5%)   ← FAST
├─ SD storage:       5700ms   (82.7%)  ← BOTTLENECK
└─ IPC overhead:      543ms   (7.8%)
```

---

## Buffer Layout in PSRAM (16MB total)

| Buffer | Size | Purpose |
|--------|------|---------|
| `capture_buf` | 2.5MB | DCMIPP DMA destination (ping-pong buffer 0) |
| `save_buf` | 2.5MB | DCMIPP DMA destination (ping-pong buffer 1) |
| `batch_buf` | 7.5MB | Final frame storage (3 × 2.5MB slots) |
| `sd_batch_buf` | 32KB | SD card write staging (64 blocks) |
| **Total camera+SD** | **~12.5MB** | |
| **Remaining** | **~3.5MB** | FreeRTOS stacks, OS, etc. |

---

## The Zero-Copy DMA Trick

### Problem with Previous Approaches
- **Mode 3 (STANDBY-BATCH):** Stop → memcpy → Restart between frames
  - memcpy of 2.5MB takes ~5-10ms per frame
  - Restart causes tearing on frames 2 & 3 (insufficient warmup)
- **Mode 4 initial attempt:** Continuous pipe + memcpy
  - DMA overwrites `capture_buf` while memcpy is still reading it
  - Result: glitched/torn images

### Solution: Hardware Double-Buffering with Address Reprogramming
1. Start DCMIPP with `DoubleBufferStart(capture_buf, save_buf)` — hardware alternates DMA destinations automatically
2. During warmup, frames land in `capture_buf`/`save_buf` (scratch buffers)
3. **2 frames ahead** of each capture, call `HAL_DCMIPP_PIPE_SetMemoryAddress()` to redirect the next DMA destination to `batch_buf[out_idx]`
4. By the time the frame is captured, DCMIPP is already pointing at the correct final destination
5. When frame event fires: the frame is ALREADY in `batch_buf` — just invalidate D-Cache, no memcpy needed

### Why This Works
- DCMIPP has 2 address registers (ADDR0 and ADDR1), alternating each frame
- Address register writes are instant (1 CPU cycle)
- The address being reprogrammed was used 1 frame ago, and won't be used again for 2 frames (~66ms @30fps)
- Plenty of time margin: 66ms >> 1 cycle

---

## When is Each Component Active?

### Camera (IMX335 + DCMIPP)

| State | When | Register | I2C Usage | Power |
|-------|------|----------|-----------|-------|
| STANDBY | Idle (between triggers) | 0x3000 = 0x01 | NONE | LOW |
| WAKE | Trigger → +20ms | 0x3000 = 0x00 | Brief (1 I2C write) | HIGH |
| STREAMING | +20ms → +652ms | 0x3000 = 0x00 | NONE | HIGH |
| STANDBY | +652ms → next trigger | 0x3000 = 0x01 | Brief (1 I2C write) | LOW |

### ToF Sensor (VL53L5CX)

| State | When | Ranging | I2C Usage |
|-------|------|---------|-----------|
| RUNNING | Idle + post-capture | Every 400ms | Active |
| PAUSED | During capture (652ms) | STOPPED | NONE |
| RUNNING | After capture complete | Every 400ms | Active |

### WS2812 LEDs

| State | When | Brightness | Power |
|-------|------|------------|-------|
| OFF | Idle | 0% | NONE |
| ON | During capture + SD write | 80% | HIGH (brief flash) |
| OFF | After SD write complete | 0% | NONE |

### FreeRTOS Tasks

| Task | Priority | Active During |
|------|----------|---------------|
| `sensor_task` | IDLE+4 (highest) | Always (polls ToF every 5ms) |
| `camera_task` | IDLE+3 | On trigger (receives queue msg) |
| `storage_task` | IDLE+2 | During SD writes (receives queue msg) |
| `main_thread` | IDLE+1 | Boot only (then deletes itself) |

---

## Key Registers and I2C Timing

### IMX335 Mode Register (0x3000)
```c
#define IMX335_MODE_STANDBY     0x01  // Camera off, I2C released
#define IMX335_MODE_STREAMING   0x00  // Camera active, streaming CSI-2
```

### I2C1 Bus Usage Over Time
```
Boot:      [Camera init I2C] → [Standby write] → I2C FREE
Idle:      [ToF ranging I2C...] (continuous, every 400ms)
Trigger:   ToF PAUSED → [Camera wake I2C: 0x3000=0x00, +20ms delay]
Capture:   No I2C needed (exposure/gain via I2C, then CSI-2 streaming)
End:       [Camera standby I2C: 0x3000=0x01] → I2C FREE
Resume:    ToF RUNNING (I2C for ranging again)
```

**Critical:** Camera I2C and ToF I2C share the same bus (I2C1). They never conflict because:
1. During idle: Camera is in standby (no I2C), ToF owns the bus
2. During capture: ToF is paused (no I2C), Camera owns the bus briefly at wake/end

---

## Configuration Parameters (app_config.h)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CAPTURE_MODE` | 4 | Callback-batch mode |
| `SNAP_WIDTH` | 1296 | Image width (2x2 binned from 2592) |
| `SNAP_HEIGHT` | 972 | Image height (2x2 binned from 1944) |
| `SNAP_FPS` | 30 | Camera frame rate |
| `CALLBACK_WARMUP_FRAMES` | 15 | Frames discarded after wake |
| `CALLBACK_FRAMES` | 3 | Frames captured per trigger |
| `CAM_EXPOSURE_VALUE` | 80µs | Shutter speed (fast, freezes motion) |
| `CAM_GAIN_VALUE` | 500 | Analog gain (low, less noise) |
| `WS2812_ILLUMINATION_BRIGHTNESS` | 80% | LED flash brightness |
| `WS2812_MODE` | 1 | CAPTURE mode (LED on during capture+SD) |

---

## Debugging Checklist

### If frames are glitched/torn:
1. Check D-Cache invalidation: `SCB_InvalidateDCache_by_Addr()` is called on `batch_buf` slots before storage reads them
2. Verify `CALLBACK_WARMUP_FRAMES` is sufficient (try 18 if green tint appears)
3. Check that `HAL_DCMIPP_PIPE_SetMemoryAddress()` is called 2 frames ahead

### If frames are blurry:
1. Reduce `CAM_EXPOSURE_VALUE` (currently 80µs, min is 8µs)
2. Increase `WS2812_ILLUMINATION_BRIGHTNESS` (currently 80%)
3. Check exposure readback in logs: `[CAM] Standby-wake exposure=XX gain=XX`

### If standby wake fails:
1. Check I2C1 is free (ToF not actively communicating)
2. Verify 20ms delay after wake is sufficient
3. Check `[CAM] Continuous start failed after callback wake` in logs

### If SD write fails:
1. Increase `SD_RETRY_DELAY_MS` or `SD_MAX_RETRIES`
2. Check SD card speed class (Class 10 or UHS-I recommended)
3. Look for `[SD] CRC error` or `[SD] timeout` in logs

---

## Performance Improvement Ideas

1. **Reduce warmup frames** (currently 15): If images are stable with 8-10 warmup frames, save ~200ms
2. **SD card parallel write**: Write frames to SD while capturing next detection (pipeline overlap)
3. **Higher SD clock**: Current SDMMC2 at HCLK/2 — try faster divisor
4. **Smaller resolution**: 640x480 at 60fps would reduce SD write time significantly (~0.6MB vs 2.5MB per frame)

---

## File Reference

| File | Role |
|------|------|
| `Src/app_cam.c` | Camera driver, CALLBACK-BATCH implementation |
| `Inc/app_cam.h` | Camera API declarations |
| `Src/app_thread.c` | FreeRTOS tasks (sensor, camera, storage) |
| `Src/main.c` | Boot sequence, buffer allocation |
| `Inc/app_config.h` | All tunable parameters |
| `Src/vl53l5cx_detection.c` | Insect detection logic |
| `Src/ws2812.c` | LED illumination control |