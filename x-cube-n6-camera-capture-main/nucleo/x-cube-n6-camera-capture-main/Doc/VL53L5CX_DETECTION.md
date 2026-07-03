# VL53L5CX Insect Detection Module

Modular API for the VL53L5CX 8x8 Time-of-Flight sensor, providing zone-based signal-drop detection for insect monitoring with automatic camera capture.

## Hardware

| Signal        | Pin   | Function                          |
|---------------|-------|-----------------------------------|
| I2C1 SCL      | PC1   | I2C clock                         |
| I2C1 SDA      | PH9   | I2C data                          |
| PWR_EN        | PD0   | Power enable (active HIGH)        |
| I2C_RST       | PE7   | Reset (active LOW pulse)          |
| LPn           | PD6   | Low-power disable (active HIGH)   |

Sensor I2C address: **0x29**

## Quick Start

```c
#include "vl53l5cx_detection.h"

// --- In main_thread (startup) ---
VL53L5CX_I2C_Init();       // Board-specific I2C config (in main.c)
VL53L5CX_PowerUp();        // GPIO power sequence

// --- In detection task ---
if (VL53L5CX_Init(&hi2c1) != 0) {
    // Sensor not detected — handle error
}

// Set resolution: VL53L5CX_RESOLUTION_4X4 (16 zones) or VL53L5CX_RESOLUTION_8X8 (64 zones)
VL53L5CX_Configure(VL53L5CX_RESOLUTION_8X8, 800, 15);
VL53L5CX_StartRanging();
VL53L5CX_LearnBaseline();

while (1) {
    if (VL53L5CX_Update()) {
        if (VL53L5CX_IsInsectDetected()) {
            VL53L5CX_DetectionResult_t r = VL53L5CX_GetResult();
            printf("Insect! %u zones affected\n", r.affected_count);
        }
    }
}
```

## How It Works

1. **Baseline Learning** — At startup, the module captures `BASELINE_SAMPLES` (default 10) frames and computes the average signal-per-SPAD and distance for each zone.

2. **Signal-Drop Detection** — On every new frame, the current signal is compared against the learned baseline. If the signal drops by more than `THRESHOLD_PCT` (default 6%), the zone is flagged as affected.

3. **Adaptive Baseline (optional)** — When `ADAPTIVE_BASELINE_ENABLED = 1`, non-affected zones slowly update their baseline using an exponential moving average (EMA), compensating for temperature drift and ambient light changes.

## Configuration

Edit `Inc/vl53l5cx_detection.h`:

### Resolution Selector

```c
#define VL53L5CX_DET_RESOLUTION   8    // 4 = 4x4 (16 zones), 8 = 8x8 (64 zones)
```

Changing this value automatically sets `VL53L5CX_DET_NUM_ZONES` to 16 or 64. All arrays, loops, and debug output adapt to the selected resolution.

### Core Parameters

| Define                           | Default | Description                              |
|----------------------------------|---------|------------------------------------------|
| `VL53L5CX_DET_RESOLUTION`        | 8       | 4 = 4x4 mode, 8 = 8x8 mode              |
| `VL53L5CX_DET_NUM_ZONES`        | 64      | Auto-derived from RESOLUTION             |
| `VL53L5CX_DET_BASELINE_SAMPLES`  | 10      | Frames averaged for baseline             |
| `VL53L5CX_DET_THRESHOLD_PCT`     | 6       | Signal drop % to trigger detection       |
| `VL53L5CX_DET_MOTION_THRESH`     | 20      | Hardware motion indicator threshold      |
| `VL53L5CX_DET_ADAPTIVE_ENABLED`  | 0       | 1 = enable EMA baseline drift correction |
| `VL53L5CX_DET_EMA_DIVIDER`       | 256     | Smoothing factor (larger = slower)       |

### Debug / Python Monitor Parameters

| Define                                | Default | Description                                           |
|---------------------------------------|---------|-------------------------------------------------------|
| `VL53L5CX_DET_DEBUG_FRAME_INTERVAL`   | 1       | Print ZFRAME every N frames. **Must be ≥ 1 for Python plots to receive data.** Set to 0 to disable all serial output. |
| `VL53L5CX_DET_DEBUG_ALLPARAMS`        | 0       | Print detailed human-readable table + ALLPARAM line every N frames. |
| `VL53L5CX_DET_DEBUG_ALLPARAM_INT`     | 10      | Interval (frames) between ALLPARAM detailed output.   |
| `VL53L5CX_DET_DEBUG_EXTENDED_ZFRAME`  | 1       | **0** = legacy ZFRAME (4 fields/zone). **1** = extended ZFRAME (12 fields/zone + temp + motion). |

#### Debug Configuration Presets

| Goal | FRAME_INTERVAL | ALLPARAMS | EXTENDED_ZFRAME |
|------|----------------|-----------|-----------------|
| **No debug output** (production) | 0 | 0 | 0 |
| **Python plots, minimal serial** | 1 | 0 | 0 |
| **Python plots, all parameters** | 1 | 0 | 1 |
| **Full debug + console tables** | 1 | 1 | 1 |

#### Important: Python Monitor Requires FRAME_INTERVAL ≥ 1

If you set `VL53L5CX_DET_DEBUG_FRAME_INTERVAL = 0`, **no ZFRAME lines are sent over serial**, so the Python monitor receives zero data and the plots remain empty. To have working plots:

- Keep `FRAME_INTERVAL = 1` (or any value ≥ 1)
- Set `ALLPARAMS = 0` to reduce console spam (optional)
- Set `EXTENDED_ZFRAME = 0` for minimal serial bandwidth (legacy 4-field format), or `1` for all parameters

The Python script (`vl53l5cx_zone_monitor.py`) auto-detects both legacy and extended ZFRAME formats.

#### Serial Bandwidth Impact

At 115200 baud and FRAME_INTERVAL=1 (~67ms at 15Hz):

| Mode | Fields | Bytes/line | Bytes/sec |
|------|--------|-----------|-----------|
| Legacy ZFRAME 4x4 | ~65 | ~400 | ~6000 |
| Legacy ZFRAME 8x8 | ~125 | ~750 | ~11000 |
| Extended ZFRAME 4x4 | ~210 | ~1200 | ~18000 |
| Extended ZFRAME 8x8 | ~825 | ~4500 | ~67000 |
| ALLPARAMS (extra) | — | ~2000/table + MOTION ~100 | every 10 frames |

## Zone Mask

Zone masking is **disabled** in the current build — all zones participate in detection. The `is_zone_enabled()` helper always returns 1.

To re-enable zone masking in the future, modify `is_zone_enabled()` in `vl53l5cx_detection.c` to check a bitmask against the zone index.

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `VL53L5CX_Init(&hi2c)` | Detect sensor, init API. Returns 0 on success. |
| `VL53L5CX_PowerUp()` | GPIO power sequence (PWR_EN → RST → LPn) |
| `VL53L5CX_PowerDown()` | Cut power (PWR_EN = LOW) |

### Configuration

| Function | Description |
|----------|-------------|
| `VL53L5CX_Configure(res, int_ms, freq_hz)` | Set resolution, integration time, frequency |
| `VL53L5CX_StartRanging()` | Begin continuous ranging |
| `VL53L5CX_StopRanging()` | Stop ranging |

### Data Access

| Function | Description |
|----------|-------------|
| `VL53L5CX_WaitForDataReady(timeout_ms)` | Block until new frame. Returns 1 if ready. |
| `VL53L5CX_GetData()` | Read latest frame. Returns 0 on success. |
| `VL53L5CX_GetZoneData(z, &sig, &dist, &stat)` | Get signal/distance/status for zone z |
| `VL53L5CX_GetBaselineData(z, &sig, &dist)` | Get learned baseline for zone z |
| `VL53L5CX_IsZoneValid(z)` | Returns 1 if zone has valid baseline |
| `VL53L5CX_IsBaselineReady()` | Returns 1 if baseline learning completed |

### Baseline

| Function | Description |
|----------|-------------|
| `VL53L5CX_LearnBaseline()` | Capture BASELINE_SAMPLES frames, compute averages |
| `VL53L5CX_ResetBaseline()` | Clear all baseline data |

### Detection

| Function | Description |
|----------|-------------|
| `VL53L5CX_Update()` | Process new frame + run detection. Returns 1 if processed. |
| `VL53L5CX_IsInsectDetected()` | Returns 1 if insect detected last frame |
| `VL53L5CX_GetResult()` | Returns `VL53L5CX_DetectionResult_t` with details |

**DetectionResult_t struct:**
```c
typedef struct {
    uint8_t  insect_detected;       // 1 if insect detected
    uint8_t  affected_count;        // Number of affected zones
    uint8_t  affected_zones[64];    // Zone indices (size = NUM_ZONES)
    uint32_t affected_drop[64];     // Signal drop % per zone
    uint8_t  valid_measurements;    // Valid zone readings this frame
} VL53L5CX_DetectionResult_t;
```

### Debug / Diagnostics

| Function | Description |
|----------|-------------|
| `VL53L5CX_PrintAllZoneParams()` | Full table: dist, signal, baseline, drop%, status, ambient, sigma, reflectance, spads, targets |
| `VL53L5CX_PrintZFrame()` | Compact CSV line for Python visualization |
| `VL53L5CX_ScanI2CBus()` | Scan I2C1, print all responding addresses |

### Legacy Test Functions

| Function | Description |
|----------|-------------|
| `VL53L5CX_Validate()` | Quick sensor presence check |
| `VL53L5CX_ReadingTest()` | Baseline + infinite monitoring loop |
| `VL53L5CX_MotionTest()` | Baseline + motion indicator + infinite loop |

## Debug Output Formats

### Extended ZFRAME (EXTENDED_ZFRAME = 1)
```
ZFRAME,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...,motion0,...,motionN
```
- `temp` = silicon temperature in °C
- 12 fields per zone: sig, base, dist, bdist, ambient, sigma, reflectance, status, spads, targets, drop%, valid
- Motion data appended at end (N = NUM_ZONES values)

### Legacy ZFRAME (EXTENDED_ZFRAME = 0)
```
ZFRAME,sig0,base0,dist0,bdist0,sig1,base1,dist1,bdist1,...
```
- 4 fields per zone: signal, baseline_signal, distance, baseline_distance

### ALLPARAM (detailed, every N frames)
```
ALLPARAM,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...
```
Same 12-field format as extended ZFRAME but prefixed with `ALLPARAM`.

### MOTION
```
MOTION,global1,global2,status,nb_detected,nb_agg,motion0,...,motionN
```
Hardware motion indicator values (N = NUM_ZONES values).

### DET (Detection event)
```
DET,1,Z5:15,Z9:12
```
Capture #1, zones Z5 (15% drop) and Z9 (12% drop).

## Python Monitor

The `vl53l5cx_zone_monitor.py` script provides real-time visualization:

```bash
# Install dependencies
pip install pyserial pyqtgraph numpy

# Run (edit SERIAL_PORT in the script first)
python vl53l5cx_zone_monitor.py
```

**Configuration** (must match firmware):
```python
SERIAL_PORT = 'COM6'     # Your serial port
RESOLUTION  = 8          # Must match VL53L5CX_DET_RESOLUTION
```

**Tabs:**
- **Signal & Distance** — Current vs baseline signal and distance per zone
- **Detection Metrics** — Signal drop %, ambient noise, range sigma
- **Advanced** — Reflectance %, SPADs enabled, motion indicator
- **Heatmaps** — 8×8 grid views of drop %, distance, motion, reflectance

**Data saving:** Set `SAVE_DATA = True` to log all frames to CSV in `zone_data/` folder.

**Keyboard:** Press `R` to reset all plot data.

## Files

| File | Description |
|------|-------------|
| `Inc/vl53l5cx_detection.h` | Header + configuration constants |
| `Src/vl53l5cx_detection.c` | Full implementation |
| `vl53l5cx_zone_monitor.py` | Python real-time visualization script |

## Tuning Tips

- **Too many false positives** → Increase `THRESHOLD_PCT`
- **Missed detections** → Decrease `THRESHOLD_PCT` or increase integration time
- **Baseline drift over hours** → Enable `ADAPTIVE_BASELINE_ENABLED`
- **Sensor not detected** → Call `VL53L5CX_ScanI2CBus()` to verify I2C connectivity
- **8x8 too much serial bandwidth** → Use legacy ZFRAME (EXTENDED=0) or increase FRAME_INTERVAL