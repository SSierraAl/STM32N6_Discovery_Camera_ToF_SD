# SD Card Storage Analysis & Capture Modes Guide

## 1. SD Card Storage: Bottleneck Analysis

### Performance Profile (from your timing report)

Your 1296x972 (2.4 MB) image takes ~2000ms to write. The breakdown reveals:

| Component | Time | Percentage |
|-----------|------|-----------|
| SD Card Ready Wait | 128ms | 2.3% |
| DMA Transfer | 787ms | 14.4% |
| **Inter-batch Gap** | **4560ms** | **83.3%** |

### Root Cause: The 15ms vTaskDelay

In `app_thread.c` (storage_task → SD_StoreRawImage), there is a deliberate `vTaskDelay(pdMS_TO_TICKS(15))` between each batch write:

```c
// Line ~260 in app_thread.c
t0 = HAL_GetTick();
vTaskDelay(pdMS_TO_TICKS(15));  // <-- THIS IS THE BOTTLENECK
uint32_t gap_ms = HAL_GetTick() - t0;
```

With **77 batches per image**, this adds up to: 77 × 15ms = **~1155ms of pure delay**.

**Why it exists:** Without this delay, the SD card returns CRC errors because it needs time between multi-block write operations for internal flash programming. The delay was added as a workaround.

### Why Some Images Store and Others Don't

The intermittent failure ("some images stored, others not") has two likely causes:

1. **SD Card Timeout (2000ms):** `SD_WaitForReady()` waits up to 2000ms for the card to be ready before each batch. If the card is slow (common with cheaper SDXC cards), it times out → `SD_StoreRawImage` returns -1 → image is lost.

2. **Race Condition (fixed):** Previously, `g_last_batch_frames` could be overwritten by a second capture before the first caller read it. This is now prevented because `Capture_RequestSnapshot()` blocks until ALL storage is complete (waits on `storage_done_sem` for each frame).

### Reliability Fix Already Applied

A **retry-after-reinit** mechanism was added to `storage_task`:
```c
if (rc != 0) {
    printf("[SD] FAIL — attempting recovery + retry...\n");
    if (SD_Reinit() == 0) {
        rc = SD_StoreRawImage(...);  // Try once more
    }
}
```
This catches transient SD card hiccups (timeout/CRC) and retries the SAME image.

### Possible Further Optimizations (Not Critical)

| Option | Expected Gain | Risk |
|--------|--------------|------|
| Reduce 15ms delay to 5-10ms | ~300-500ms faster per image | Medium — may cause CRC on some cards |
| Increase SD_BATCH_WRITE_BLOCKS from 64 → 128 | Fewer batches = less gap overhead | High — more CRC errors likely |
| Use SDCARD_TRANSFER state check instead of fixed delay | Adaptive wait, no wasted time | Low — good improvement |
| Higher-end SD card (U3/V30 class) | Faster internal programming | No code change needed |

---

## 2. Capture Modes Overview

### Mode 0: ON-DEMAND (Button-Triggered, Full Resolution)
```c
#define CAPTURE_MODE 0
```
- **ToF:** DISABLED (sensor_task not created)
- **Camera:** OFF until button press
- **Resolution:** 2592×1944 (full 5MP, CAM_BINNING=0)
- **Trigger:** USER button (PC13) via btn_thread
- **Flow:**
  1. Button pressed → WS2812 illumination ON
  2. Full CMW_CAMERA_Init + warmup (11 frames) + capture
  3. Save to SD → illumination OFF
  4. Camera fully deinitialized → waits for next press

**Files modified:**
- `main.c`: sensor_task creation gated with `#if CAPTURE_MODE != 0`
- `main.c`: btn_thread created only in mode 0
- `app_thread.c`: camera_task falls through to `#else` branch (CAM_CaptureSingleFrame)

---

### Mode 1: CONTINUOUS (Camera + ToF Always Running, Low Resolution)
```c
#define CAPTURE_MODE 1
```
- **ToF:** ENABLED (sensor_task runs continuously)
- **Camera:** ALWAYS ON (pipe running continuously at 1296×972, CAM_BINNING=1)
- **Trigger:** ToF insect detection → CAM_ContinuousSnap
- **Flow:**
  1. At boot: camera initialized in main_thread (before system_ready=1)
  2. Pipe runs continuously with double-buffering (capture_buf + capture_buf2)
  3. ToF detects insect → pauses ranging → triggers capture
  4. CAM_ContinuousSnap: stop pipe → copy frame to save_buf → restart pipe
  5. save_buf queued to storage_task → SD write
  6. ToF resumes after storage completes

**I2C Mutex Protection:**
Both camera and ToF share I2C1. Without serialization, concurrent I2C transactions corrupt the bus protocol (the historical "CMW_CAMERA_Init fails ret=-7" issue).

**Solution implemented (i2c_arbiter module):**
- Global FreeRTOS mutex `g_i2c1_mutex` created before scheduler starts
- **Camera side:** `cmw_io.h` macros redirect BSP_I2C1_ReadReg16/WriteReg16 → `I2C_Arbiter_Camera*` wrappers (take mutex → call BSP → release)
- **ToF side:** `platform.c` HAL_I2C_* calls wrapped with mutex take/give
- This catches ALL I2C traffic including ISP background process (AE/AWB)

**Files modified:**
- `i2c_arbiter.c/.h`: New module (mutex + camera wrappers)
- `platform.c`: ToF I2C functions wrapped
- `cmw_io.h`: Macros redirected to locked wrappers
- `main.c`: Camera init moved to main_thread (before system_ready=1)
- `main.c`: `capture_buf2[]` allocated for double-buffering
- `app_thread.c`: camera_task no longer calls CAM_ContinuousStart (already running)

---

### Mode 2: BATCH (Camera Always Running, Multiple Frames per Detection)
```c
#define CAPTURE_MODE 2
```
- **ToF:** ENABLED
- **Camera:** ALWAYS ON (1296×972, CAM_BINNING=1)
- **Trigger:** ToF → captures BATCH_FRAMES (3) via stop/copy/restart each frame
- **NOTE:** Has I2C conflict warning (same as mode 1, but less severe since camera stops between frames)

---

### Mode 3: STANDBY-BATCH — REMOVED
```
~DELETED~
```
All standby-batch code has been removed:
- ~~`CAM_StandbyInit()`~~ deleted from app_cam.c
- ~~`CAM_StandbyBatchSnap()`~~ deleted from app_cam.c
- ~~`CAM_IsStandbyReady()`~~ deleted from app_cam.c
- Prototypes removed from app_cam.h

---

### Mode 4: CALLBACK-BATCH (Proven, Working — TOUCHED NOTHING)
```c
#define CAPTURE_MODE 4
```
- **ToF:** ENABLED (paused only during capture)
- **Camera:** STANDBY between triggers (sensor powered, pipe stopped)
- **Trigger:** ToF → wake → start pipe → 11 warmup + 4 capture frames (zero-copy DMA) → stop → standby
- **Key Feature:** Uses DCMIPP frame event callback (ISR-driven) — NO stop/restart between frames = ALL frames sharp
- **Status:** ✅ WORKING — No changes made to this mode

---

## 3. File Changes Summary

| File | Change | Purpose |
|------|--------|---------|
| `i2c_arbiter.c` | **NEW** | I2C1 mutex + camera-side locked wrappers |
| `i2c_arbiter.h` | **NEW** | Mutex declaration + API |
| `platform.c` | MODIFIED | ToF HAL_I2C calls wrapped with mutex |
| `cmw_io.h` | MODIFIED | Camera macros redirected to locked wrappers |
| `main.c` | MODIFIED | Mode 1 camera init in main_thread; sensor_task gated for mode 0; capture_buf2 for mode 1 |
| `app_thread.c` | MODIFIED | CAM_ContinuousStart removed from camera_task (mode 1); SD retry-after-reinit |
| `app_cam.c` | MODIFIED | Standby-batch functions deleted (~150 lines) |
| `app_cam.h` | MODIFIED | Standby-batch prototypes deleted |
| `app_config.h` | MODIFIED | Mode 3 doc removed; binning conditional updated (mode 1 included) |
| `Makefile` | MODIFIED | i2c_arbiter.c added to C_SOURCES |

## 4. How to Switch Modes

Edit `app_config.h`:
```c
#ifndef CAPTURE_MODE
#define CAPTURE_MODE            0  // Change to 0, 1, 2, or 4
#endif
```

Then rebuild and flash.