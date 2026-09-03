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
├── SECTION 8:  WS2812 ILLUMINATION CONFIGURATION
├── SECTION 8B: TOF TEST MODE (on-site validation)
└── SECTION 9:  TOF DETECTION (VL53L5CX)
```

ToF-specific parameters are in: `Inc/app_config.h` (SECTION 9, above).

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

All ToF parameters live in the **SECTION 9 "ToF Detection (VL53L5CX)"** block of `Inc/app_config.h` (moved from `vl53l5cx_detection.h`, which now includes `app_config.h`). Several parameters are resolution-dependent via `#if VL53L5CX_DET_RESOLUTION == 8` — the code blocks below show both values. Every change requires a firmware rebuild + flash; there is no runtime configuration.

### Where to change what — master reference

| You want to change… | Parameter | Where | Default (4×4 / 8×8) |
|---|---|---|---|
| Signal drop trigger level | `VL53L5CX_DET_THRESHOLD_PCT` | `app_config.h` §9 | 6 / 15 (% of baseline) |
| Motion trigger level | `VL53L5CX_DET_MOTION_THRESH` | `app_config.h` §9 | 60 / 100 (plugin motion units) |
| Zones needed for a detection | `VL53L5CX_DET_MIN_AFFECTED_ZONES` | `app_config.h` §9 | 1 / 2 |
| Minimum signal floor | `VL53L5CX_DET_MIN_SIGNAL` | `app_config.h` §9 | 500 (both) |
| Grid size | `VL53L5CX_DET_RESOLUTION` | `app_config.h` §9 | 4 |
| Integration time (primary) | `VL53L5CX_DET_INTEGRATION_MS` | `app_config.h` §9 → passed to `VL53L5CX_Configure()` from `Src/app_thread.c` | 30 / 800 ms |
| Ranging frequency (primary) | `VL53L5CX_DET_RANGING_FREQ_HZ` | same as above | 15 Hz (both) |
| Ranging mode (primary) | `VL53L5CX_RANGING_MODE` | `app_config.h` §9 → applied in `VL53L5CX_Configure()` (`Src/vl53l5cx_detection.c`) | 3 = AUTONOMOUS (both) |
| Baseline samples (primary) | `VL53L5CX_DET_BASELINE_SAMPLES` | `app_config.h` §9 | 20 / 30 |
| Sensor motion plugin (global flag / accumulation / noise) | `VL53L5CX_DET_MOTION_MIN_ZONES`, `…_MOTION_PERSIST_FRAMES`, `…_MOTION_EXTRA_NOISE` (guardian: `VL53L5CX_EXT_MOTION_*`) | `app_config.h` §9 → applied to the sensor in `VL53L5CX_Configure()` / `VL53L5CX_External_Configure()` | 1 / 16 / 0 (independent per sensor) |
| Baseline refresh | `VL53L5CX_DET_PERIODIC_RESTART_*`, `VL53L5CX_DET_ADAPTIVE_REFRESH_*` | `app_config.h` §9 | adaptive: 15 s window, max 3 detections |
| Single vs dual (guardian) mode | `VL53L5CX_DUAL_SENSOR` | `app_config.h` §9 | 0 |
| Guardian wake duration / confirmation | `VL53L5CX_DUAL_WAKE_DURATION_MS`, `VL53L5CX_DUAL_CONFIRM_FRAMES` | `app_config.h` §9 | 5000 ms / 2 frames |
| Debug UART output | `VL53L5CX_DET_DEBUG_ZFRAME*`, `VL53L5CX_DET_DEBUG_ALLPARAM*` | `app_config.h` §9 | ZFRAME on, ALLPARAM off |
| I2C addresses | — **hardcoded, not a define** | `Src/vl53l5cx_detection.c`: primary `0x29` in `VL53L5CX_Init()`, external `0x31` (7-bit, = `0x62` 8-bit) in `VL53L5CX_External_Init()` | — |
| External (guardian) sensor timing | `VL53L5CX_EXT_INTEGRATION_MS`, `VL53L5CX_EXT_RANGING_FREQ_HZ`, `VL53L5CX_EXT_RANGING_MODE` | `app_config.h` §9 EXT block → applied in `VL53L5CX_External_Configure()` | 800 ms / 15 Hz / 1 (CONTINUOUS) — independent of the primary |
| External (guardian) detection thresholds | `VL53L5CX_EXT_THRESHOLD_PCT`, `VL53L5CX_EXT_MOTION_THRESH`, `VL53L5CX_EXT_MIN_AFFECTED_ZONES`, `VL53L5CX_EXT_MIN_SIGNAL` | `app_config.h` §9 EXT block → `VL53L5CX_External_Update()` / `VL53L5CX_External_LearnBaseline()` | 6 / 60 / 1 / 500 (4×4) — independent of the primary |
| Python tuning variables | `MOTION_THRESH`, `MIN_AFFECTED_ZONES`, … | top of `vl53l5cx_analysis.py` (workspace root) | see "Translating a vl53l5cx_analysis.py sweep into the firmware" below |
| Preview thresholds live (host overlay only) | `THRESHOLD_PCT`, `MOTION_THRESH`, `MIN_AFFECTED_ZONES`, `MIN_SIGNAL` | top of `vl53l5cx_zone_monitor.py` (workspace root) | 6 / 60 / 1 / 500 (4×4) — mirrors the §9 defines; does NOT change the firmware |

### How a trigger is computed (exact logic)

For every zone `z` of the current frame (`VL53L5CX_Update()`; `VL53L5CX_External_Update()` runs the identical logic with the guardian's own `VL53L5CX_EXT_*` values):

1. **Baseline gate:** the zone must have a valid baseline (`s_zone_valid[z]`).
2. **Signal channel** (additionally gated by the fresh-data status `VL53L5CX_STATUS_OK(status)` — only status 5, 6 or 9; status 0 = stale data is rejected — see `05_ToF_Detection_System.md` §7):
   the current `signal_per_spad` must be ≥ `VL53L5CX_DET_MIN_SIGNAL` (500), then

   `drop% = |baseline − current| × 100 / baseline`

   → the zone signal-triggers when `drop% > VL53L5CX_DET_THRESHOLD_PCT` (**strictly greater**).
3. **Motion channel** — **not** status-gated on the primary sensor (it is gate-restricted on the external guardian): the sensor-side ST plugin computes a per-zone 32-bit motion value (programmed by default to see movement between 400 and 1500 mm) → the zone motion-triggers when `motion_value >= VL53L5CX_DET_MOTION_THRESH` (**greater-or-equal**).
4. **Frame decision:** count every zone that triggered on *either* channel (`affected_count`). A detection is latched for the frame when `affected_count >= VL53L5CX_DET_MIN_AFFECTED_ZONES` (4×4: 1, 8×8: 2); `trigger_source` then records which channel(s) fired (SIGNAL / MOTION / BOTH).

The sensor-side plugin parameters (`MOTION_MIN_ZONES`, `PERSIST_FRAMES`, `EXTRA_NOISE`) shape the raw motion values of step 3 and the plugin's *global* flag (the datalog `MOTION` lines) — they are **not** part of the step-4 frame decision.

### `VL53L5CX_DET_RESOLUTION` — ToF Grid Size
```c
#define VL53L5CX_DET_RESOLUTION  4   /* 4×4 or 8×8 grid */
```
| Value | Zones | Sensitivity | Use Case |
|-------|-------|-------------|----------|
| 4 | 16 zones (4×4) | Higher (larger zones = more photons per zone) | **Default, recommended** |
| 8 | 64 zones (8×8) | Lower (smaller zones) | Fine-grained zone analysis |

---

### `VL53L5CX_RANGING_MODE` — Primary Ranging Mode
```c
#define VL53L5CX_RANGING_MODE  3   /* 1 = CONTINUOUS, 3 = AUTONOMOUS */
```
The VL53L5CX ULD driver implements **only two** ranging modes (raw driver codes, see `vl53l5cx_api.h`):

| Value | Driver macro | Behavior |
|-------|--------------|----------|
| 3 (default) | `VL53L5CX_RANGING_MODE_AUTONOMOUS` | The precise integration time from `VL53L5CX_DET_INTEGRATION_MS` is used; results stream at ≈ `VL53L5CX_DET_RANGING_FREQ_HZ`. |
| 1 | `VL53L5CX_RANGING_MODE_CONTINUOUS` | Integration time is forced to the **sensor maximum** — highest sensitivity, but `VL53L5CX_DET_INTEGRATION_MS` is then ignored and the effective frame rate drops below the requested frequency. |

**How to change**: edit `VL53L5CX_RANGING_MODE` in `app_config.h` §9 → rebuild/flash. Verify on the console: `VL53L5CX_Configure()` prints `[ToF] Configured: res=…, int=…ms, freq=…Hz, mode=…`.

**Scope**: primary sensor only. The **external guardian** has its own mode define `VL53L5CX_EXT_RANGING_MODE` (default 1 = CONTINUOUS — the previous hardcoded behavior).

---

### `VL53L5CX_DET_INTEGRATION_MS` / `VL53L5CX_DET_RANGING_FREQ_HZ` — Sensor Timing
```c
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_INTEGRATION_MS   800
#else
#define VL53L5CX_DET_INTEGRATION_MS   30
#endif
#define VL53L5CX_DET_RANGING_FREQ_HZ  15
```
| | |
|---|---|
| **What it does** | Integration time = how long the sensor integrates light per zone per frame. More time → higher signal (kcps/spad) → more margin on the signal channel, better low-light / long-range detection. |
| **Driver limit** | 2 … 1000 ms. Values outside the range are rejected (`VL53L5CX_STATUS_INVALID_PARAM`) and the sensor keeps its previous integration time. |
| **Frequency trade-off** | The requested `…FREQ_HZ` is only achievable while `integration + processing < 1/frequency`. 30 ms @ 15 Hz (4×4) is honored; with 800 ms integration (8×8) the effective rate is far below 15 Hz. |
| **Interaction with mode** | In CONTINUOUS mode (`VL53L5CX_RANGING_MODE = 1`) the integration time is forced to the sensor maximum and `VL53L5CX_DET_INTEGRATION_MS` is ignored. |
| **Where it is applied** | `Src/app_thread.c` passes both values to `VL53L5CX_Configure()` at startup (the external guardian keeps its own `VL53L5CX_EXT_INTEGRATION_MS` / `VL53L5CX_EXT_RANGING_FREQ_HZ`, default 800 ms / 15 Hz). |

**Tuning**: weak/missing detections at distance or in low light → raise integration (30 → 60 → 100 → 300 ms). Detections too slow for fast insects (object vanishes between frames) → lower integration or raise the frequency.

---

### `VL53L5CX_DET_THRESHOLD_PCT` — Signal Drop Threshold
```c
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_THRESHOLD_PCT  15   /* 8×8: 15% signal drop = detection */
#else
#define VL53L5CX_DET_THRESHOLD_PCT  6    /* 4×4: 6% signal drop = detection */
#endif
```
**What it does**: When an insect flies through a ToF zone, the reflected light signal drops (object is closer than baseline). For each valid zone the firmware computes `drop% = |baseline − current| × 100 / baseline` and the zone signal-triggers when `drop% > VL53L5CX_DET_THRESHOLD_PCT` (**strictly greater**). A zone is only evaluated on the signal channel when it has a valid baseline, a fresh-data status (5/6/9) and a current signal ≥ `VL53L5CX_DET_MIN_SIGNAL`.

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
#define VL53L5CX_DET_MOTION_THRESH  60
#endif
```
The ST motion indicator plugin runs inside the **sensor's GO2** and computes a per-zone 32-bit motion value (programmed by default to report movement between **400 and 1500 mm** — objects outside that window produce motion values of 0); a zone is motion-triggered when the value is **at or above** (`>=`) this threshold. On the primary sensor the motion channel is evaluated independently of ranging status and baseline validity (gate-restricted on the external guardian), so it catches fast objects that the signal channel can miss.

Three plugin-level defines shape the raw motion values the plugin produces — `MOTION_MIN_ZONES`, `MOTION_PERSIST_FRAMES` and `MOTION_EXTRA_NOISE` — each documented in its own subsection below. They are applied to the sensor at configure time; after changing any of them, re-datalog and re-check this threshold (the values on the other side of it have moved).

**Tuning**: `MOTION_THRESH` is the main knob — increase if motion false-positives, decrease if missing fast insects. `EXTRA_NOISE` / `PERSIST_FRAMES` raise the sensor-side noise floor (less jitter, less sensitivity).

---

### `VL53L5CX_DET_MOTION_MIN_ZONES` — ST Motion Plugin: Global-Motion Zones
```c
#define VL53L5CX_DET_MOTION_MIN_ZONES  1   /* same for 4×4 and 8×8 (plugin default) */
```
**What it does**: `min_nb_for_global_detection` in the ST motion plugin — how many zones must show motion before the plugin raises its *global* motion flag (visible in the datalog `MOTION`/`DETF` lines and in `VL53L5CX_MotionTest()` output). The firmware trigger itself uses the per-zone values against `MOTION_THRESH` + `VL53L5CX_DET_MIN_AFFECTED_ZONES`, so this define does **not** change the trigger — only the plugin's global flag.

**Tuning**: Leave at 1. Raise (2–3) only to make the plugin's global flag more selective (e.g. for cleaner datalog MOTION lines).

---

### `VL53L5CX_DET_MOTION_PERSIST_FRAMES` — ST Motion Plugin: Temporal Persistence
```c
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  16   /* same for 4×4 and 8×8 (plugin default) */
```
**What it does**: `nb_of_temporal_accumulations` in the ST motion plugin (plugin default: 16) — the sensor-side temporal accumulation used when the plugin computes each zone's motion value. Higher = smoother values (single-frame spikes die out) but slower reaction to real motion.

**Tuning**: 16 is the plugin default and the current firmware setting. Lower (4–8) if slow movers are missed; raise (20–30) if you see single-frame motion spikes in the `ZFRAME` motion field. **After any change, re-datalog and re-check `VL53L5CX_DET_MOTION_THRESH`** — the values on the other side of the threshold have moved.

---

### `VL53L5CX_DET_MOTION_EXTRA_NOISE` — ST Motion Plugin: Extra Noise Sigma
```c
#define VL53L5CX_DET_MOTION_EXTRA_NOISE  0   /* same for 4×4 and 8×8 (plugin default) */
```
**What it does**: `extra_noise_sigma` in the ST motion plugin (plugin default: 0) — extra noise margin added to the plugin's internal estimate, effectively a sensor-side noise floor that keeps the reported per-zone motion values conservative.

**Tuning**: Raise (25–100) if the per-zone motion field in `ZFRAME` jitters without insects present (noisy installation, small 8×8 zones); keep at 0 for maximum sensitivity. As with `PERSIST_FRAMES`, re-datalog and re-check `VL53L5CX_DET_MOTION_THRESH` afterwards.

---

### `VL53L5CX_DET_MIN_AFFECTED_ZONES` — Minimum Zones for Detection
```c
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_MIN_AFFECTED_ZONES  2
#else
#define VL53L5CX_DET_MIN_AFFECTED_ZONES  1
#endif
```
**What it does**: The firmware frame decision — after the per-zone loop, a detection is latched when the number of zones that triggered on *either* channel (signal **or** motion) in the current frame satisfies `affected_count >= VL53L5CX_DET_MIN_AFFECTED_ZONES`. At 4×4 (default) a single anomalous zone is enough; at 8×8 two zones are required. Prevents false triggers from single noisy zones.

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
#define VL53L5CX_DET_PERIODIC_RESTART_ENABLED   0
#define VL53L5CX_DET_PERIODIC_RESTART_INTERVAL  500
#define VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED   1
#define VL53L5CX_DET_REFRESH_WINDOW_SECS        15
#define VL53L5CX_DET_MAX_DETECTIONS             3
```
Two mutually exclusive refresh modes exist (periodic: every N frames; adaptive: detection-based — adaptive is currently enabled). After 3 detections within a 15-second window, the ToF sensor re-learns its baseline. This prevents "adaptation fatigue" where the sensor slowly adjusts to a permanently altered scene.

**Tuning**: Usually leave at defaults. If sensor becomes less sensitive over time → decrease `MAX_DETECTIONS` or `WINDOW_SECS`.

---

### `VL53L5CX_DET_BASELINE_SAMPLES` — Baseline Learning Samples
```c
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_BASELINE_SAMPLES  30
#else
#define VL53L5CX_DET_BASELINE_SAMPLES  20
#endif
```
Number of frames averaged while learning the per-zone signal baseline at startup. More samples = steadier baseline but slower start. The external (guardian) sensor uses its own count: `VL53L5CX_SENSOR2_BASELINE_SAMPLES` (60).

**Tuning**: Leave at defaults. Lower only if boot-time learning is too slow.

---

### Translating a vl53l5cx_analysis.py sweep into the firmware

`vl53l5cx_analysis.py` (workspace root, next to `vl53l5cx_datalogger.py`) replays the recorded per-zone motion values and, for each candidate pair (threshold T, min-zones M), counts the frames that would trip the firmware motion trigger:

```
triggered_frame = (number of zones with motion >= T) >= M
```

| vl53l5cx_analysis.py variable (top of file) | Firmware define to edit (`app_config.h` §9) |
|---|---|
| `MOTION_THRESH` (marked on plots) | `VL53L5CX_DET_MOTION_THRESH` |
| `MIN_AFFECTED_ZONES` (marked on plots) | `VL53L5CX_DET_MIN_AFFECTED_ZONES` |
| `CANDIDATE_THRESHOLDS` | sweep space for T |
| `CANDIDATE_MIN_ZONES` | sweep space for M |
| `MOTION_MIN_ZONES` / `PERSIST_FRAMES` / `EXTRA_NOISE` | `VL53L5CX_DET_MOTION_MIN_ZONES` / `VL53L5CX_DET_MOTION_PERSIST_FRAMES` / `VL53L5CX_DET_MOTION_EXTRA_NOISE` — **annotation only**: these run inside the sensor, so changing them needs a firmware edit + a new datalog (the sweep cannot predict their effect from an old CSV) |

**What the sweep does NOT cover:** the signal channel (`VL53L5CX_DET_THRESHOLD_PCT`, `VL53L5CX_DET_MIN_SIGNAL`), the fresh-data status gate (5/6/9), and the baseline. To tune the signal threshold, use the signal box plots / heatmap from the same report (or compare the `baseline` vs `signal` columns of the CSV against the drop % you want to count as a hit).

**Workflow:**
1. Set `VL53L5CX_DET_DEBUG_ALLPARAMS 1` in `app_config.h` §9, rebuild, flash.
2. `python vl53l5cx_datalogger.py` (workspace root) → records `datalog\tof_datalog_<timestamp>.csv` next to the script (launch-folder independent; `RESOLUTION` must match the firmware). Record an empty scene first, then your target insect. `E` exports the session summary (`*_summary.csv`), `S` stops and closes with a console recap.
3. From the workspace root: `python vl53l5cx_analysis.py` (auto-detects the newest datalog in `datalog\`, or pass a specific CSV) → open the sweep table (`motion_threshold_sweep.csv` + plot 13); pick (T, M) with few `events` on the empty-scene frames that still catches the insect.
4. Edit the two defines in `app_config.h` §9, rebuild, flash.
5. Keep the same values in the `vl53l5cx_analysis.py` header block so the next report marks the new firmware point.

> **Live preview (optional, any time):** the `DETECTION THRESHOLDS` block at the top of `vl53l5cx_zone_monitor.py` (workspace root) mirrors the same §9 defines and drives the labeled threshold lines, the heatmap title, and a per-frame host trigger prediction. Change the values there while the sensor streams to *preview* a candidate setting in real time — it is host-side only; the firmware still runs with the values in `app_config.h`.

### `VL53L5CX_DUAL_SENSOR` — Dual Sensor (Guardian) Mode
```c
#define VL53L5CX_DUAL_SENSOR              1    /* 0 = single sensor, 1 = dual (guardian) — current */
#define VL53L5CX_PRIMARY_ADDRESS          0x29 /* camera ToF (7-bit) */
#define VL53L5CX_EXTERNAL_ADDRESS         0x62 /* external ToF (8-bit; 7-bit 0x31) */
#define VL53L5CX_DUAL_WAKE_DURATION_MS    5000 /* primary active window after wake (ms) */
#define VL53L5CX_DUAL_CONFIRM_FRAMES      2    /* consecutive external detections needed */
#define VL53L5CX_SENSOR2_BASELINE_SAMPLES 60
```
When `DUAL_SENSOR = 1`, the external "guardian" sensor runs continuously and the camera ToF stays in ST sleep to save power. The guardian must see `CONFIRM_FRAMES` consecutive detections before waking the primary, which then runs for `WAKE_DURATION_MS` and returns to sleep.

| Parameter | What it does | Tuning |
|-----------|-------------|--------|
| `VL53L5CX_DUAL_CONFIRM_FRAMES` | Consecutive external detections needed to wake the primary | Lower to 1 = faster wake, more false wakes |
| `VL53L5CX_DUAL_WAKE_DURATION_MS` | How long the primary stays awake after a confirmed wake | Raise if capture needs more time; lower to save power |
| `VL53L5CX_SENSOR2_BASELINE_SAMPLES` | Guardian baseline samples | Fewer = faster boot |

**Tuning**: Primary never wakes → lower `CONFIRM_FRAMES` to 1 and check the `EXT,ZFRAME` serial output. Too many false wakes → raise to 3-4. The I2C addresses are fixed at power-up (the external is re-addressed from 0x29 → 0x62) — do not change them without updating the power-up sequence.

### `VL53L5CX_EXT_*` — Independent EXT (Guardian) Configuration

With `DUAL_SENSOR = 1`, the guardian's configuration is **fully independent** of the primary's: `app_config.h` §9 has a dedicated EXT block (active only in dual mode). Every default equals the value the guardian used before the block existed (hardcoded timing + shared DET thresholds), so an untouched block changes nothing:

```c
/* app_config.h §9 — EXT block (dual mode only) */
#define VL53L5CX_EXT_INTEGRATION_MS        800  /* ms, driver limit 2..1000 (was hardcoded) */
#define VL53L5CX_EXT_RANGING_FREQ_HZ       15   /* Hz (was hardcoded) */
#define VL53L5CX_EXT_RANGING_MODE          1    /* 1 = CONTINUOUS, 3 = AUTONOMOUS (was hardcoded) */
#define VL53L5CX_EXT_THRESHOLD_PCT         6    /* 4×4 (8×8: 15) — signal drop % strictly greater than */
#define VL53L5CX_EXT_MOTION_THRESH         60   /* 4×4 (8×8: 100) — per-zone motion value >= this */
#define VL53L5CX_EXT_MIN_AFFECTED_ZONES    1    /* 4×4 (8×8: 2) — zones needed for a frame trigger */
#define VL53L5CX_EXT_MIN_SIGNAL            500  /* kcps/spad floor (baseline learning + detection) */
#define VL53L5CX_EXT_MOTION_MIN_ZONES      1    /* plugin min_nb_for_global_detection */
#define VL53L5CX_EXT_MOTION_PERSIST_FRAMES 16   /* plugin nb_of_temporal_accumulations */
#define VL53L5CX_EXT_MOTION_EXTRA_NOISE    0    /* plugin extra_noise_sigma */
```

| Parameter | Where it is applied | Notes |
|---|---|---|
| `…_EXT_INTEGRATION_MS` / `…_EXT_RANGING_FREQ_HZ` / `…_EXT_RANGING_MODE` | `VL53L5CX_External_Configure()` | Independent of `VL53L5CX_DET_INTEGRATION_MS` / `…_FREQ_HZ` / `VL53L5CX_RANGING_MODE`. Console: `[EXT] Configured: …` |
| `…_EXT_THRESHOLD_PCT` / `…_EXT_MOTION_THRESH` / `…_EXT_MIN_AFFECTED_ZONES` / `…_EXT_MIN_SIGNAL` | `VL53L5CX_External_Update()` (guardian trigger → wakes the primary) + `VL53L5CX_External_LearnBaseline()` (signal floor) | Console: `[EXT] Detection: …` |
| `…_EXT_MOTION_MIN_ZONES` / `…_EXT_MOTION_PERSIST_FRAMES` / `…_EXT_MOTION_EXTRA_NOISE` | ST motion plugin inside the guardian (via DCI) | Shapes the raw motion values; after changing, re-check the `EXT,ZFRAME` motion column |

**Not independent** (still shared/hardcoded for the guardian): grid resolution (`VL53L5CX_DET_RESOLUTION` sizes the zone arrays and both the `ZFRAME` / `EXT,ZFRAME` layouts), target order (CLOSEST), sharpener (10 %), and the baseline-refresh policy (the `PERIODIC` / `ADAPTIVE` flags are common; the guardian keeps its own counters). The baseline sample count was already independent (`VL53L5CX_SENSOR2_BASELINE_SAMPLES`).

**Tuning**: the guardian drives the wake decisions, so tune it from `EXT,ZFRAME` — `vl53l5cx_zone_monitor.py`'s EXT tab now uses these values for its overlay lines, heatmap title and host-trigger prediction (mirrored in the script's `EXT_*` variables). Too many false wakes → raise `VL53L5CX_EXT_THRESHOLD_PCT` / `VL53L5CX_EXT_MOTION_THRESH` / `VL53L5CX_EXT_MIN_AFFECTED_ZONES` (or `DUAL_CONFIRM_FRAMES`). Wake too slow → lower them and/or raise `VL53L5CX_EXT_INTEGRATION_MS`.

### `TEST_TOF_MODE` — ToF-Only Test Mode (on-site validation)
```c
#define TEST_TOF_MODE    1    /* app_config.h §8B: 1 = ToF-only (no camera/SD), 0 = production */
#define TEST_TOF_LED_MS  300  /* RED LED indication duration (ms) */
```
With `TEST_TOF_MODE = 1` the build is **ToF-only**: at boot the SD card and the camera are **never initialized** (and `camera_task` / `storage_task` are not created), so no photo can be taken and the card is never touched. A detection only lights the RED board LED for `TEST_TOF_LED_MS` (the WS2812 strip stays OFF) and the console prints the **affected zone numbers** with their drop values. Everything else (baseline learning, periodic refresh, signal + motion trigger rules, 30-frame cooldown) is unchanged.
The USER button (PC13) re-learns the baseline(s) **on demand**: one press runs the same refresh procedure as the auto-refresh (stop-ranging → 50 ms → start-ranging → 200 ms → re-learn) on the primary, and — with `DUAL_SENSOR = 1` — on the external guardian as well (the primary is woken first if parked in ST sleep, then parked back to sleep; the RED LED stays on for the whole refresh). Use it to re-baseline after repositioning the sensor or after a lighting change without a power cycle — and if you disable `ADAPTIVE_REFRESH` / `PERIODIC_RESTART` for a clean validation, it becomes the *only* baseline-refresh path.
Use it on-site to validate sensor position/orientation and to calibrate the detection thresholds in this section. **revert to 0 for production**

---

### ToF troubleshooting — symptom → knob

| Symptom | First check | Knob to turn (direction) |
|---|---|---|
| No trigger, insect clearly in FOV | ZFRAME: are zone signals sane (≫ `MIN_SIGNAL`)? Status 5/6/9? | `THRESHOLD_PCT` (↓), `MOTION_THRESH` (↓), `MIN_AFFECTED_ZONES` (↓ to 1), `INTEGRATION_MS` (↑ if signal is weak) |
| No trigger at distance / in low light | Signal values in the CSV box plot | `INTEGRATION_MS` (↑: 30 → 100 → 300 ms), or `VL53L5CX_RANGING_MODE` → 1 (CONTINUOUS = max integration, lower frame rate) |
| Triggers with nothing there | Which channel fired? (`trigger_source` in the DETF line / console) | `THRESHOLD_PCT` (↑) and/or `MOTION_THRESH` (↑); `MIN_AFFECTED_ZONES` (↑ to 2); `MIN_SIGNAL` (↑); look for IR light sources / reflective surfaces; enable the periodic baseline refresh |
| Motion values jitter frame-to-frame, no insects | ZFRAME motion column | plugin `EXTRA_NOISE` (↑) and/or `PERSIST_FRAMES` (↑) — then re-datalog and re-tune `MOTION_THRESH` |
| Motion channel dead (all motion 0 although objects move) | Distance to the sensor | The plugin is programmed for the **400–1500 mm** window by default; outside it, no motion values are produced (signal channel still works) |
| Only the first frame after boot/wake is accepted (or none) | Console status values | The status gate accepts only 5/6/9 — 0 (stale data) is correctly rejected; if fresh frames never arrive: integration too long for the requested frequency, or ranging not started (`StartRanging` failed in the console log) |
| Sensitivity slowly decays over a session | Console "Adaptive refresh" lines | `ADAPTIVE_REFRESH` `MAX_DETECTIONS` (↓) or enable `PERIODIC_RESTART` (1) with `INTERVAL` (500) |
| Dual mode: guardian sees it, primary never wakes | Console `[EXT] Detection confirmed!` lines | `DUAL_CONFIRM_FRAMES` (↓ to 1), `DUAL_WAKE_DURATION_MS` (↑) |
| Dual mode: false wakes | `EXT,ZFRAME` (uses the independent `VL53L5CX_EXT_*` defines) | `VL53L5CX_EXT_THRESHOLD_PCT` / `VL53L5CX_EXT_MOTION_THRESH` (↑), `VL53L5CX_EXT_MIN_AFFECTED_ZONES` (↑), `DUAL_CONFIRM_FRAMES` (↑ to 3–4) |
| "Primary ToF baseline not ready!" at boot (task stops) | LearnBaseline console output | Field of view blocked? Raise `INTEGRATION_MS`, lower `MIN_SIGNAL`, raise `BASELINE_SAMPLES` (→ 30) |
| No ZFRAME/ALLPARAM on UART | `VL53L5CX_DET_DEBUG_*` flags in `app_config.h` §9 | set `DEBUG_ZFRAME` (or `DEBUGALLPARAMS`) to 1 and rebuild |
| Validating sensor position/orientation on-site (no photos wanted) | Console `>>> TEST:` zone list | `TEST_TOF_MODE` (`app_config.h` §8B) → 1: camera/SD not initialized at all; RED LED on for `TEST_TOF_LED_MS` + per-zone console output; USER button (PC13) re-learns the baseline(s) on demand. Revert to 0 for production |

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
*(full symptom→knob table: §4 "ToF troubleshooting")*
```
Step 1: Increase VL53L5CX_DET_THRESHOLD_PCT (6 → 10)
Step 2: Increase VL53L5CX_DET_MIN_AFFECTED_ZONES (1 → 2)
Step 3: Increase VL53L5CX_DET_MIN_SIGNAL (500 → 1000)
Step 4: Check environment for IR light sources or reflective surfaces
```

### Scenario 5: "Missing real detections (insect flies through, no trigger)"
*(full symptom→knob table: §4 "ToF troubleshooting")*
```
Step 1: Decrease VL53L5CX_DET_THRESHOLD_PCT (6 → 3)
Step 2: Decrease VL53L5CX_DET_MOTION_THRESH (60 → 30)
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

### Scenario 8: "Validate ToF placement on-site (no capture)"
```
Step 1: Set TEST_TOF_MODE = 1 (app_config.h §8B) and rebuild
**It is recommended to also put VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED and VL53L5CX_DET_PERIODIC_RESTART_ENABLED to 0** in order to validate the detections on-site.
Step 2: Mount the unit at its final position and power-cycle it once
        (baseline learning runs at boot — it must learn the real site).
        After any repositioning or lighting change, one USER button (PC13)
        press re-learns the baseline(s) on demand (primary + external
        guardian in dual mode) — no power cycle needed.
Step 3: Move a target (insect, hand, card) across the detection area at
        different speeds — watch the RED LED and the console
        (no camera, no SD card: neither is even initialized)
Step 4: Read the ">>> TEST:" lines: trigger source (SIGNAL/MOTION) and the
        affected zone numbers (0-15 on the 4x4 grid) show WHERE in the FOV
        the target was — rotate/reposition the sensor until zones cluster
        where the target is expected to cross
Step 5: Tune thresholds here (Scenarios 4-5) and re-test
Step 6: Set TEST_TOF_MODE = 0 for the production build
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

*Parameter Tuning Guide — Updated September 2026*