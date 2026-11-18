# IMX335 Binning Exploration Plan

## Overview

This document outlines the plan to explore **pixel binning** on the Sony IMX335 camera sensor. Binning combines adjacent pixels to create larger "super-pixels", reducing resolution but significantly improving:
- **Readout speed** (less rolling shutter distortion)
- **Low-light performance** (more photons per super-pixel)
- **Frame rate** (smaller data to process)

## Current Configuration

| Parameter | Value |
|-----------|-------|
| Resolution | 2592 × 1944 (5MPX) |
| Frame Rate | 30 FPS |
| Readout Time | ~32ms/frame |
| Rolling Shutter Effect | ~25ms skew (top-to-bottom) |
| Bandwidth | 10MB/frame (YUV422) |

## IMX335 Binning Modes

The IMX335 sensor supports these binning configurations via the CMW middleware:

| Mode | Output Res | Readout | Effective Pixel | Data/Frame | Use Case |
|------|-----------|---------|-----------------|------------|----------|
| **1×1 (None)** | 2592×1944 | 32ms | 1.55µm | 10MB | Species ID (current) |
| **2×2 Area** | 1296×972 | ~8ms | 3.1µm | 2.5MB | Motion detection |
| **2×2 Binning** | 1296×972 | ~8ms | 3.1µm | 2.5MB | Low light capture |
| **4×4 Area** | 648×486 | ~2ms | 6.2µm | 0.6MB | Preview/trigger |

**Note:** "Area" mode sums pixels (better SNR), "Binning" mode averages pixels (linear response).

## Implementation Plan

### Step 1: Add Binning Config to app_config.h

```c
/* Camera binning mode:
   0 = 1x1 (full resolution 2592x1944 - current)
   1 = 2x2 area (1296x972 - faster readout)
   2 = 4x4 area (648x486 - fastest) */
#define CAM_BINNING_MODE     0

/* Auto-calculate resolution based on binning */
#if CAM_BINNING_MODE == 0
    #define SNAP_WIDTH       2592
    #define SNAP_HEIGHT      1944
#elif CAM_BINNING_MODE == 1
    #define SNAP_WIDTH       1296
    #define SNAP_HEIGHT      972
#elif CAM_BINNING_MODE == 2
    #define SNAP_WIDTH       648
    #define SNAP_HEIGHT      486
#endif
```

### Step 2: Modify app_cam.c to Support Binning

**File:** `Src/app_cam.c` → `CAM_Init()` function

**Current code:**
```c
cam_conf.width  = SENSOR_WIDTH;   // 0 = full
cam_conf.height = SENSOR_HEIGHT;  // 0 = full
```

**Modified code:**
```c
#if CAM_BINNING_MODE == 0
    cam_conf.width  = 0;  /* Full 2592x1944 */
    cam_conf.height = 0;
#elif CAM_BINNING_MODE == 1
    cam_conf.width  = 1296;  /* 2x2 binned */
    cam_conf.height = 972;
#elif CAM_BINNING_MODE == 2
    cam_conf.width  = 648;   /* 4x4 binned */
    cam_conf.height = 486;
#endif
```

**Also update:** `CMW_CAMERA_SetExposureMode()` may need different exposure ranges for binned modes.

### Step 3: Update Buffer Sizes

The frame buffer size is auto-calculated in `app_config.h`:
```c
#define SNAP_FRAME_SIZE      (SNAP_WIDTH * SNAP_HEIGHT * 2)  // YUV422 = 2bpp
```

This automatically adjusts:
- 1×1: 10,077,696 bytes (current)
- 2×2: 2,519,424 bytes (4× smaller)
- 4×4: 629,856 bytes (16× smaller)

**No code changes needed** — buffers scale automatically.

### Step 4: Adjust SD Storage Parameters

Smaller frames = fewer SD blocks = faster writes:

| Mode | Frame Size | SD Blocks | Batches (64/batch) | Est. SD Time |
|------|-----------|-----------|-------------------|--------------|
| 1×1 | 10MB | 19,684 | 308 | ~9.5s |
| 2×2 | 2.5MB | 4,922 | 77 | ~2.4s |
| 4×4 | 0.6MB | 1,231 | 19 | ~0.6s |

**Potential:** With 2×2 binning, SD write drops from 9.5s to 2.4s — **75% faster**!

### Step 5: Update Illumination Timing

With faster readout, the illumination duration can be reduced:

```c
/* Illumination duration based on binning mode */
#if CAM_BINNING_MODE == 0
    #define WS2812_ILLUMINATION_MS  500  // Full res: 185ms init + 363ms warmup + 33ms capture
#elif CAM_BINNING_MODE == 1
    #define WS2812_ILLUMINATION_MS  150  // 2x2: ~8ms readout, less warmup needed
#elif CAM_BINNING_MODE == 2
    #define WS2812_ILLUMINATION_MS  50   // 4x4: ~2ms readout, minimal warmup
#endif
```

### Step 6: Testing Procedure

1. **Build with CAM_BINNING_MODE = 0** (baseline, current config)
   - Verify 5MPX images still work correctly
   - Record timing: camera ~570ms, SD ~9.5s, total ~10.7s

2. **Build with CAM_BINNING_MODE = 1** (2×2 binning)
   - Verify 1296×972 images capture correctly
   - Expected timing: camera ~150ms, SD ~2.4s, total ~2.6s
   - **Check:** Can you still identify insect species at this resolution?

3. **Build with CAM_BINNING_MODE = 2** (4×4 binning)
   - Verify 648×486 images
   - Expected timing: camera ~50ms, SD ~0.6s, total ~0.7s
   - **Use case:** Motion detection/preview only (too low res for ID)

## Rolling Shutter Impact

Binning directly reduces rolling shutter distortion:

| Mode | Readout Time | Insect Speed (1m/s) | Row Skew |
|------|-------------|---------------------|----------|
| 1×1 | 25ms | 250µm displacement | Severe |
| 2×2 | 6ms | 60µm displacement | Moderate |
| 4×4 | 1.5ms | 15µm displacement | Minimal |

**Key Insight:** 2×2 binning reduces rolling shutter artifact by 4× while maintaining ~1.3MPX resolution (enough for many insect IDs).

## Hybrid Strategy (Recommended)

For best results, consider a **two-stage capture**:

```
1. ToF detects insect
2. Quick 4×4 binning preview (2ms) → confirm insect in frame
3. Full 1×1 resolution capture (32ms) → species ID
4. SD write of both images
```

**Requires:** Dual buffer allocation + camera mode switch during capture.

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| CMW doesn't support binning API | Cannot implement | Test with `CAM_Init()` first |
| Resolution too low for ID | Cannot classify species | Use hybrid strategy |
| SD write still slow | No improvement | Binning reduces frame size 4× |
| Illumination timing wrong | Dark/overexposed images | Adjust WS2812_ILLUMINATION_MS |

## Next Steps

1. [ ] Confirm CMW IMX335 driver supports `cam_conf.width/height` for binning
2. [ ] Implement config changes in `app_config.h`
3. [ ] Modify `CAM_Init()` in `app_cam.c`
4. [ ] Test 2×2 mode first (best balance of speed/resolution)
5. [ ] If 2×2 works, test 4×4 for preview mode
6. [ ] Consider hybrid strategy if single mode insufficient

## References

- Sony IMX335 datasheet: Section 8.3 (Binning/Readout modes)
- CMW Camera Middleware: `CMW_CameraInit_t` structure documentation
- STM32N6 DCMIPP: Hardware crop/scaling capabilities

---

**Document Version:** 1.0  
**Last Updated:** 2026-07-09  
**Author:** Cline (AI Assistant)