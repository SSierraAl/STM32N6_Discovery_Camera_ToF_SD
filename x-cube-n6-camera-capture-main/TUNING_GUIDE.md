# Parameter Tuning Guide
## How to Adjust Every Configuration Parameter

---

## Table of Contents

1. [Quick Start](#1-quick-start)
2. [Camera Parameters](#2-camera-parameters)
3. [SD Card Parameters](#3-sd-card-parameters)
4. [ToF Detection Parameters](#4-tof-detection-parameters)
5. [LED Illumination Parameters](#5-led-illumination-parameters)
6. [Capture Mode Parameters](#6-capture-mode-parameters)
7. [Performance Debug Parameters](#7-performance-debug-parameters)
8. [Tuning Scenarios](#8-tuning-scenarios)
9. [Parameter Dependencies](#9-parameter-dependencies)

---

## 1. Quick Start

### Where to Find Parameters
All application parameters are in **one file**: `Inc/app_config.h`

```
Inc/app_config.h
├── SECTION 1:  CAMERA CAPTURE PARAMETERS
├── SECTION 1B: CAMERA IMAGE QUALITY
├── SECTION 2:  MEMORY / BUFFER SIZES (auto-calculated)
├── SECTION 3:  SD CARD STORAGE PARAMETERS
├── SECTION 4:  CAPTURE MODE SELECTION
├── SECTION 5D: CALLBACK-BATCH PARAMETERS (Mode 4)
├── SECTION 5B: BATCH PARAMETERS (Mode 2)
├── SECTION 5:  BUTTON / LED PARAMETERS
├── SECTION 7:  PERFORMANCE DEBUG / TIMING
└── SECTION 8:  WS2812 ILLUMINATION CONFIGURATION
```

ToF-specific parameters are in: `Inc/vl53l5cx_detection.h`

### Parameters You'll Tune Most Often
| Parameter | When to Change | Typical Adjustment |
|-----------|---------------|-------------------|
| `CAPTURE_MODE` | Switching operating mode | 0, 1, 2, or 4 |
| `CAM_EXPOSURE_VALUE` | Images too dark or blurry | Decrease for speed, increase for brightness |
| `CAM_GAIN_VALUE` | Images too noisy or too dark | Increase for brightness, decrease for less noise |
| `SD_BATCH_RECOVERY_GAP_MS` | SD write failures | Increase from 15 → 20ms |
| `VL53L5CX_DET_THRESHOLD_PCT` | Too many or too few detections | Increase to be more selective |
| `CALLBACK_FRAMES` | Need more/fewer images per detection | 1-8 (more = more PSRAM used) |

---

## 2. Camera Parameters

### `CAPTURE_MODE` — Operating Mode
```c
#define CAPTURE_MODE  4
```
| Value | Mode | Description |
|-------|------|-------------|
| 0 | On-Demand | Button trigger, full resolution, no ToF |
| 1 | Continuous | Camera always running, ToF trigger, 1 frame |
| 2 | Batch | Camera always running, ToF trigger, 3 frames |
| 4 | Callback-Batch | ⭐ Recommended. Standby→wake→4 frames zero-copy |

**Impact**: Changes which tasks are created, which buffers are allocated, and the entire capture flow.

---

### `CAM_BINNING` — Resolution Selection
```c
#if CAPTURE_MODE == 1 || CAPTURE_MODE == 2 || CAPTURE_MODE == 4
#define CAM_BINNING  1   /* 2x2 binning: 1296×972 */
#else
#define CAM_BINNING  0   /* Full resolution: 2592×1944 */
#endif
```
| Value | Resolution | Frame Size | Use Case |
|-------|-----------|------------|----------|
| 0 | 2592×1944 | 9,677,376 bytes (9.2 MB) | Maximum detail, static subjects |
| 1 | 1296×972 | 2,519,424 bytes (2.4 MB) | Fast capture, insects |

**Auto-set**: Binning is automatically set based on CAPTURE_MODE. Modes 1,2,4 use binning=1. Mode 0 uses binning=0.

**Manual override**: Change the `#if` condition or define `CAM_BINNING` directly.

**Impact on SD write time**:
- Full res: ~9500ms per image (296 batches)
- Binned: ~2500ms per image (77 batches)

---

### `SNAP_WARMUP_FRAMES` — Camera Settling Frames
```c
#define SNAP_WARMUP_FRAMES  11
```
| Value | Effect | Risk |
|-------|--------|------|
| 5-7 | Faster capture, less warmup | First frames may be green-tinted |
| 8-11 | Balanced (recommended) | Good quality, reasonable speed |
| 12-20 | Maximum quality | Slower capture, insect can move |

**Why warmup matters**: After camera power-on or standby wake, the IMX335 sensor outputs unstable frames (green tint, wrong exposure). These must be discarded before capturing.

**Time cost**: Each warmup frame = ~33ms at 30 FPS. 11 warmup frames = 363ms.

---

### `SNAP_FPS` — Camera Frame Rate
```c
#define SNAP_FPS  30
```
| Value | Frame Period | Use Case |
|-------|-------------|----------|
| 30 | 33.3ms | Default, good balance |
| 15 | 66.7ms | Longer exposure (brighter), slower capture |
| 60 | 16.7ms | Shorter exposure, faster (may be too dark) |

**Note**: Not all resolutions support all FPS values. Check IMX335 datasheet.

---

## 2B. Camera Image Quality Parameters

### `CAM_EXPOSURE_MODE` — Exposure Control
```c
#define CAM_EXPOSURE_MODE  1   /* 0=Auto | 1=Manual | 2=Freeze */
```
| Value | Mode | Description |
|-------|------|-------------|
| 0 | AUTO | Sensor decides exposure automatically. Can be slow in low light → motion blur |
| 1 | MANUAL | Fixed exposure + gain. Fastest shutter, repeatable results. **Recommended for Mode 4** |
| 2 | FREEZE | Uses last auto value. Good for consistent lighting |

**Why MANUAL for Mode 4**: In callback-batch mode, the camera wakes from standby each trigger. AUTO exposure takes multiple frames to converge, causing inconsistent brightness across the batch. MANUAL pins values, giving identical exposure every trigger.

---

### `CAM_EXPOSURE_VALUE` — Shutter Speed (Manual Mode Only)
```c
#define CAM_EXPOSURE_VALUE  30   /* Microseconds */
```
| Value | Shutter Effect | When to Use |
|-------|---------------|-------------|
| 8-50 | Ultra-fast (freezes everything) | Bright LED illumination, fast insects |
| 50-200 | Fast (recommended range) | Good balance of speed and brightness |
| 200-1000 | Medium | Dimmer lighting, slower subjects |
| 1000-5000 | Slow | Very dim, but risk of motion blur |
| 5000+ | Very slow | Only for static objects |

**Current setting (30μs)**: Extremely fast shutter. Requires bright LED illumination and moderate gain.

**Tuning procedure**:
1. Start at 200μs, gain 2000
2. If images blurry → decrease exposure (faster shutter)
3. If images too dark → increase gain OR increase LED brightness
4. Target: sharpest image that is still bright enough

---

### `CAM_GAIN_VALUE` — Sensor Amplification (Manual Mode Only)
```c
#define CAM_GAIN_VALUE  8   /* Internal gain (0-72000, ×1000 representation) */
```
| Value | Effect | When to Use |
|-------|--------|-------------|
| 1-10 | Minimal gain (clean signal) | Very bright illumination |
| 10-100 | Low gain | Bright LED, good balance |
| 100-4000 | Medium gain | Moderate lighting |
| 4000-8000 | High gain | Dim lighting (noisy images) |
| 8000+ | Very high gain | Last resort (very noisy) |

**Current setting (8)**: Extremely low gain. Combined with 30μs exposure and 20% LED brightness, this produces a clean but potentially dark image.

**Trade-off**: Higher gain = brighter image but more visible noise/grain. Lower gain = cleaner image but may be too dark.

**Tuning procedure**:
1. Set exposure first (determines shutter speed)
2. Then adjust gain to achieve desired brightness
3. Prefer higher exposure + lower gain over lower exposure + higher gain (better signal-to-noise)

---

### `WS2812_ILLUMINATION_BRIGHTNESS` — LED Brightness
```c
#define WS2812_ILLUMINATION_BRIGHTNESS  20  /* Percentage (0-100%) */
```
| Value | LED Output | Camera Effect |
|-------|-----------|---------------|
| 5-10 | Very dim | Need high gain, risk of blur |
| 15-25 | Low-moderate | Balanced (current setting) |
| 25-50 | Moderate | Good brightness, moderate power |
| 50-75 | Bright | May need lower gain |
| 75-100 | Maximum | Best signal, highest power |

**Current setting (20%)**: Relatively low. Combined with 30μs exposure and gain 8, the image may be dark. If so, increase this value.

**Relationship**: LED brightness × exposure time = total light collected. If you increase one, you can decrease the other.

---

### `CAM_BRIGHTNESS`, `CAM_CONTRAST`, `CAM_ANTI_FLICKER`
```c
#define CAM_BRIGHTNESS    0   /* -128 to +127 */
#define CAM_CONTRAST      0   /* Small positive values only */
#define CAM_ANTI_FLICKER  0   /* 0=off, 1=50Hz, 2=60Hz, 3=auto */
```
These are rarely tuned. Leave at defaults unless:
- **Brightness**: Image is consistently too light/dark across all conditions
- **Contrast**: Image looks "flat" (try value 10-20)
- **Anti-flicker**: Horizontal banding under AC-powered lights (set to 1 for EU 50Hz, 2 for US 60Hz)

---

## 3. SD Card Parameters

### `SD_BATCH_WRITE_BLOCKS` — Write Batch Size
```c
#define SD_BATCH_WRITE_BLOCKS  64
```
| Value | HAL Calls per Image (1296×972) | Reliability |
|-------|-------------------------------|-------------|
| 16 | 308 | Most reliable, slowest (more overhead) |
| 32 | 154 | Good balance |
| 64 | 77 | Default, reliable |
| 128 | 39 | Fastest, may cause CRC errors |

**Impact**: Larger batches = fewer HAL calls = less overhead BUT more data per write = higher chance of CRC error if card can't keep up.

**Recommendation**: Keep at 64. Only change if experimenting.

---

### `SD_BATCH_RECOVERY_GAP_MS` — Inter-Batch Delay ⭐
```c
#define SD_BATCH_RECOVERY_GAP_MS  15
```
**THE MOST IMPORTANT SD PARAMETER.** This is the minimum delay (ms) between SD write batches.

| Value | Effect | When to Use |
|-------|--------|-------------|
| 0 | No delay, writes back-to-back | Fastest, likely CRC errors |
| 5-10 | Minimal delay | May work on fast cards |
| **15** | **Default** | **Good balance of speed and reliability** |
| 20 | Conservative delay | **Use if you see STA=0x5000 errors** |
| 25-30 | Maximum delay | Slow cards, old cards, bad sectors |
| 50+ | Very safe | Development/testing only |

**How to diagnose**:
- See `STA=0x5000` in console? → **Increase this value** (try 20, then 25)
- SD writes reliable but too slow? → **Decrease this value** (try 10, then 5)
- Sweet spot: lowest value with zero errors over 100+ captures

**Time impact**: A 1296×972 image = 77 batches. Changing from 15ms → 20ms adds ~385ms per image.

---

### `SD_SNAP_BASE_BLOCK` — First Image Location
```c
#define SD_SNAP_BASE_BLOCK  3072
```
Leaves blocks 0-3071 free (1.5 MB). Do not change unless you need to reserve space for other data.

---

## 4. ToF Detection Parameters

### `VL53L5CX_DET_RESOLUTION` — ToF Grid Size
```c
#define VL53L5CX_DET_RESOLUTION  4   /* 4×4 or 8×8 grid */
```
| Value | Zones | Sensitivity | Use Case |
|-------|-------|-------------|----------|
| 4 | 16 zones (4×4) | Higher (larger zones = more photons per zone) | **Default, recommended** |
| 8 | 64 zones (8×8) | Lower (smaller zones) | Fine-grained zone analysis |

---

### `VL53L5CX_DET_THRESHOLD_PCT` — Signal Drop Threshold
```c
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_THRESHOLD_PCT  15   /* 8×8: 15% signal drop = detection */
#else
#define VL53L5CX_DET_THRESHOLD_PCT  6    /* 4×4: 6% signal drop = detection */
#endif
```
**What it does**: When an insect flies through a ToF zone, the reflected light signal drops (object is closer than baseline). This threshold determines how big the drop must be to trigger.

| Value | Effect | Risk |
|-------|--------|------|
| 3-5 | Very sensitive | False triggers from noise, ambient light changes |
| 6-10 | Moderate sensitivity | Good balance (current 4×4 = 6%) |
| 10-15 | Less sensitive | May miss small insects |
| 15-25 | Very selective | Only large/very close objects detected |

**Tuning**: If getting false detections (no insect present) → **increase**. If missing real insects → **decrease**.

---

### `VL53L5CX_DET_MOTION_THRESH` — Motion Indicator Threshold
```c
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_MOTION_THRESH  100
#else
#define VL53L5CX_DET_MOTION_THRESH  40
#endif
```
The VL53L5CX has a built-in motion indicator (0-255). Values above this threshold contribute to detection.

**Tuning**: Similar to `THRESHOLD_PCT`. Increase if too sensitive, decrease if not sensitive enough.

---

### `VL53L5CX_DET_MIN_AFFECTED_ZONES` — Minimum Zones for Detection
```c
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_MIN_AFFECTED_ZONES  2
#else
#define VL53L5CX_DET_MIN_AFFECTED_ZONES  1
#endif
```
**What it does**: Require at least N zones to show anomalies before triggering. Prevents false triggers from single noisy zones.

**Tuning**: If false triggers from noise → increase to 2. If missing small insects → keep at 1.

---

### `VL53L5CX_DET_MIN_SIGNAL` — Signal Quality Gate
```c
#define VL53L5CX_DET_MIN_SIGNAL  500
```
Zones with signal below 500 kcps are excluded from detection (noisy/blind zones). Prevents false triggers from unreliable zones.

**Tuning**: If false triggers in dark areas → increase. If missing detections in normal areas → decrease (but risk more noise).

---

### `VL53L5CX_DET_ADAPTIVE_REFRESH` — Baseline Update
```c
#define VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED   1
#define VL53L5CX_DET_REFRESH_WINDOW_SECS        10
#define VL53L5CX_DET_MAX_DETECTIONS             5
```
After 5 detections within a 10-second window, the ToF sensor re-learns its baseline. This prevents "adaptation fatigue" where the sensor slowly adjusts to a permanently altered scene.

**Tuning**: Usually leave at defaults. If sensor becomes less sensitive over time → decrease `MAX_DETECTIONS` or `WINDOW_SECS`.

---

## 5. LED Illumination Parameters

### `WS2812_MODE` — LED Behavior
```c
#define WS2812_MODE  1   /* 0=Always On | 1=Capture | 2=Indicator */
```
| Value | Mode | Description |
|-------|------|-------------|
| 0 | ALWAYS_ON | LEDs on continuously. Power hungry, not recommended. |
| 1 | CAPTURE | ⭐ Recommended. LEDs on during capture cycle only (FlashStart/FlashStop). |
| 2 | INDICATOR | Brief flash on detection only. No illumination for camera. |

---

### `WS2812_ILLUMINATION_COLOR` — LED Color
```c
#define WS2812_ILLUMINATION_COLOR  0xA0FF28  /* RGB format */
```
| Color | Value | Use Case |
|-------|-------|----------|
| White | 0xFFFFFF | Maximum illumination |
| Green-tinted | 0xA0FF28 | Current setting, good for insects |
| Red | 0xFF0000 | Least disruptive to insects (camera needs more gain) |
| Blue | 0x0000FF | Some insects less sensitive |

**Recommendation**: White (0xFFFFFF) for maximum illumination, or current green-tinted for insect-friendly operation.

---

## 6. Capture Mode Parameters

### Mode 4: Callback-Batch
```c
#define CALLBACK_WARMUP_FRAMES  11   /* Discard after wake */
#define CALLBACK_FRAMES         4    /* Keep per trigger */
#define CALLBACK_WAKE_TIMEOUT_MS 1000
```

| Parameter | Current | Range | Effect of Changing |
|-----------|---------|-------|-------------------|
| `CALLBACK_WARMUP_FRAMES` | 11 | 5-15 | Fewer = faster but risk of tinted frames |
| `CALLBACK_FRAMES` | 4 | 1-8 | More = more data, more PSRAM, longer SD write |
| `CALLBACK_WAKE_TIMEOUT_MS` | 1000 | 500-2000 | Timeout for wake + warmup + capture |

**PSRAM impact of CALLBACK_FRAMES**:
| Frames | PSRAM Used | Fits? |
|--------|-----------|-------|
| 1 | 2.5MB | ✅ Yes |
| 2 | 5MB | ✅ Yes |
| 3 | 7.5MB | ✅ Yes |
| 4 | 10MB | ✅ Yes (current) |
| 5 | 12.5MB | ✅ Yes |
| 6 | 15MB | ⚠️ Tight |
| 7-8 | 17.5-20MB | ❌ Overflows 16MB PSRAM |

### Mode 2: Batch
```c
#define BATCH_FRAMES  3   /* Frames per detection */
```
Same PSRAM constraints as CALLBACK_FRAMES. Max ~5 frames safely in 16MB PSRAM.

---

## 7. Performance Debug Parameters

### `PERF_DEBUG_LEVEL` — Console Output Verbosity
```c
#define PERF_DEBUG_LEVEL  2   /* 0=Minimal | 1=Standard | 2=Verbose | 3=Debug */
```
| Level | Output | Use Case |
|-------|--------|----------|
| 0 | Only total time per snapshot | Production, clean logs |
| 1 | Phase breakdown (camera, SD) | Default debugging |
| 2 | Sub-phase timing + per-batch SD | Performance analysis |
| 3 | Everything + per-block analysis | Deep bottleneck hunting |

### `PERF_PRINT_SUMMARY` — Full Report After Each Capture
```c
#define PERF_PRINT_SUMMARY  1   /* 0=Off | 1=On */
```
When enabled, prints the full timing report table after each capture cycle.

### `PERF_SD_BATCH_PRINT_EVERY` — Per-Batch SD Timing
```c
#define PERF_SD_BATCH_PRINT_EVERY  0  /* 0=Off, N=print every Nth batch */
```
When >0, prints timing for every Nth SD batch. Useful for seeing if specific batches are slow.

---

## 8. Tuning Scenarios

### Scenario 1: "Images are too dark"
```
Step 1: Increase WS2812_ILLUMINATION_BRIGHTNESS (20% → 50%)
Step 2: If still dark, increase CAM_EXPOSURE_VALUE (30 → 200)
Step 3: If still dark, increase CAM_GAIN_VALUE (8 → 2000)
Step 4: Verify LED color is white (0xFFFFFF) for maximum output
```

### Scenario 2: "Images are blurry (motion)"
```
Step 1: Decrease CAM_EXPOSURE_VALUE (30 → 8) for faster shutter
Step 2: Increase WS2812_ILLUMINATION_BRIGHTNESS to compensate for less light
Step 3: Increase CAM_GAIN_VALUE if image too dark after step 1
Step 4: Verify SNAP_FPS = 30 (higher FPS = shorter frame period = less rolling shutter)
```

### Scenario 3: "SD write failures (STA=0x5000)"
```
Step 1: Increase SD_BATCH_RECOVERY_GAP_MS (15 → 20)
Step 2: If still failing, try (20 → 25)
Step 3: If still failing, try reducing SD_BATCH_WRITE_BLOCKS (64 → 32)
Step 4: If all else fails, replace SD card (may have bad sectors)
```

### Scenario 4: "Too many false detections (ToF triggers with nothing there)"
```
Step 1: Increase VL53L5CX_DET_THRESHOLD_PCT (6 → 10)
Step 2: Increase VL53L5CX_DET_MIN_AFFECTED_ZONES (1 → 2)
Step 3: Increase VL53L5CX_DET_MIN_SIGNAL (500 → 1000)
Step 4: Check environment for IR light sources or reflective surfaces
```

### Scenario 5: "Missing real detections (insect flies through, no trigger)"
```
Step 1: Decrease VL53L5CX_DET_THRESHOLD_PCT (6 → 3)
Step 2: Decrease VL53L5CX_DET_MOTION_THRESH (40 → 20)
Step 3: Set VL53L5CX_DET_MIN_AFFECTED_ZONES to 1
Step 4: Verify VL53L5CX_DET_MIN_SIGNAL not too high (500 is good)
```

### Scenario 6: "First frames after boot are green-tinted"
```
Step 1: Increase SNAP_WARMUP_FRAMES (11 → 15)
Step 2: Increase CALLBACK_WARMUP_FRAMES (11 → 15) for Mode 4
Step 3: Verify camera is given enough time after power-on (>500ms)
```

### Scenario 7: "Want to maximize images per hour"
```
The bottleneck is SD writing (~2.5s per image). To minimize:
Step 1: Use smallest resolution that works (1296×972, not 2592×1944)
Step 2: Reduce CALLBACK_FRAMES (4 → 2) for fewer images per cycle
Step 3: Reduce SD_BATCH_RECOVERY_GAP_MS (15 → 10) if card allows
Step 4: Use a faster SD card (U3/V30 rated)
```

---

## 9. Parameter Dependencies

### Critical Relationships
```
CAPTURE_MODE determines:
  ├── CAM_BINNING (auto-set: mode 0 = 0, modes 1,2,4 = 1)
  ├── Tasks created (sensor_task: modes 1,2,4; btn_thread: mode 0)
  ├── Camera pre-init (modes 1,4 in main_thread; mode 2 in camera_task)
  └── Buffers allocated (save_buf: modes 1,4; batch_buf: modes 2,4)

CAM_EXPOSURE_VALUE + CAM_GAIN_VALUE + WS2812_ILLUMINATION_BRIGHTNESS:
  └── Together determine image brightness
      Increase one → can decrease another
      Best quality: high exposure + high LED + low gain

CALLBACK_FRAMES × SNAP_FRAME_SIZE:
  └── Must fit in PSRAM (16MB total, ~15MB available for buffers)
      4 × 2.5MB = 10MB ← current usage
      6 × 2.5MB = 15MB ← maximum safe
      8 × 2.5MB = 20MB ← OVERFLOW ❌

SD_BATCH_RECOVERY_GAP_MS × SD_BATCH_WRITE_BLOCKS:
  └── Together determine SD write reliability and speed
      Larger gap + smaller batches = more reliable, slower
      Smaller gap + larger batches = faster, less reliable

VL53L5CX_DET_THRESHOLD_PCT + VL53L5CX_DET_MIN_AFFECTED_ZONES:
  └── Together determine detection sensitivity
      Higher threshold + more zones = less sensitive, fewer false triggers
      Lower threshold + fewer zones = more sensitive, more false triggers
```

---

*Parameter Tuning Guide — Updated July 2026*