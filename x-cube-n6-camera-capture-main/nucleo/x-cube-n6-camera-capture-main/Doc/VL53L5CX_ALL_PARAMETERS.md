# VL53L5CX All Parameters - Extended Monitoring

## Overview

The VL53L5CX time-of-flight sensor provides rich per-zone data. This document describes all enabled parameters, the serial output format, and how the Python monitor reads them.

The firmware is configured for **8x8 resolution (64 zones)** by default. Change `VL53L5CX_DET_RESOLUTION` in `vl53l5cx_detection.h` to switch between 4x4 (16 zones) and 8x8 (64 zones).

## Enabled Parameters

### Per-Zone Parameters (64 zones in 8x8 resolution)

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `signal_per_spad` | uint32_t | kcps/spad | Signal returned to the sensor |
| `baseline_signal` | uint32_t | kcps/spad | Learned baseline signal |
| `distance_mm` | int16_t | mm | Measured distance |
| `baseline_distance` | uint16_t | mm | Learned baseline distance |
| `ambient_per_spad` | uint32_t | kcps/spad | Ambient light noise |
| `range_sigma_mm` | uint16_t | mm | Measurement uncertainty (sigma) |
| `reflectance` | uint8_t | % | Estimated surface reflectance |
| `target_status` | uint8_t | - | Status: 0/5/9 = valid ranging |
| `nb_spads_enabled` | uint32_t | count | Number of SPADs active |
| `nb_target_detected` | uint8_t | count | Number of targets detected |
| `signal_drop_pct` | uint32_t | % | Computed signal drop from baseline |
| `zone_valid` | uint8_t | flag | 1 = zone has valid baseline |
| `motion_indicator` | uint32_t | power | Motion detection power per zone |

### Global Parameters

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `silicon_temp_degc` | int8_t | °C | Internal sensor temperature |
| `motion_global_1` | uint32_t | - | Global motion indicator 1 |
| `motion_global_2` | uint32_t | - | Global motion indicator 2 |
| `motion_status` | uint8_t | - | Motion detector status |
| `nb_detected_aggregates` | uint8_t | - | Number of detected motion clusters |
| `nb_aggregates` | uint8_t | - | Total number of motion aggregates |

## Serial Output Format

### Extended ZFRAME (every frame)

**8x8 mode (64 zones):**
```
ZFRAME,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...,sig63,...,valid63,motion0,...,motion63
```

- **Total fields**: 1 (temp) + 64×12 (zone data) + 64 (motion) = **833 comma-separated values**
- **Per zone** (12 fields): sig, base, dist, bdist, amb, sigma, refl, status, spads, targs, drop, valid
- **Example**: `ZFRAME,35,1200,1300,150,155,50,12,45,0,2500,1,8,1,...`

**4x4 mode (16 zones):**
```
ZFRAME,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...,sig15,...,valid15,motion0,...,motion15
```

- **Total fields**: 1 (temp) + 16×12 (zone data) + 16 (motion) = **209 comma-separated values**

### ALLPARAM (every 10 frames, detailed)

```
ALLPARAM,temp,sig0,base0,dist0,bdist0,amb0,sigma0,refl0,status0,spads0,targs0,drop0,valid0,...
```

Same structure as extended ZFRAME (12 fields per zone). Additionally prints a human-readable table to the console.

### MOTION (every 10 frames)

**8x8 mode:**
```
MOTION,global1,global2,status,nb_detected,nb_agg,motion0,motion1,...,motion63
```

**4x4 mode:**
```
MOTION,global1,global2,status,nb_detected,nb_agg,motion0,motion1,...,motion15
```

### DET (on detection event)

```
DET,capture_num,Z3:15,Z7:12
```

### Legacy ZFRAME (when extended mode disabled)

**8x8 mode:**
```
ZFRAME,sig0,base0,dist0,bdist0,...,sig63,base63,dist63,bdist63
```
- **Total fields**: 1 + 64×4 = **257 comma-separated values**

**4x4 mode:**
```
ZFRAME,sig0,base0,dist0,bdist0,...,sig15,base15,dist15,bdist15
```
- **Total fields**: 1 + 16×4 = **65 comma-separated values**

Python parser auto-detects format by field count.

## Configuration (vl53l5cx_detection.h)

```c
/* Resolution: 4 = 4x4 (16 zones), 8 = 8x8 (64 zones) */
#define VL53L5CX_DET_RESOLUTION       8

/* Frame interval for ZFRAME output (0 = disable) */
#define VL53L5CX_DET_DEBUG_FRAME_INTERVAL 1

/* Enable detailed ALLPARAM table output */
#define VL53L5CX_DET_DEBUG_ALLPARAMS    0
#define VL53L5CX_DET_DEBUG_ALLPARAM_INT 10

/* Enable extended ZFRAME with all parameters */
#define VL53L5CX_DET_DEBUG_EXTENDED_ZFRAME 1

/* Adaptive baseline tracking */
#define VL53L5CX_DET_ADAPTIVE_ENABLED 0
#define VL53L5CX_DET_EMA_DIVIDER      256

/* Detection thresholds */
#define VL53L5CX_DET_THRESHOLD_PCT    6   /* Signal drop % */
#define VL53L5CX_DET_MOTION_THRESH    20  /* Motion power */
```

## Python Monitor (vl53l5cx_zone_monitor.py)

### Tabs

1. **Signal & Distance** - Signal per SPAD (current vs baseline), Distance (current vs baseline)
2. **Detection Metrics** - Signal Drop %, Ambient Noise, Range Sigma
3. **Advanced** - Reflectance %, SPADs Enabled, Motion Indicator
4. **Heatmaps** - 8×8 heatmaps for Drop, Distance, Motion, Reflectance + Temperature display

### Configuration

```python
SERIAL_PORT = 'COM6'   # Your serial port
RESOLUTION  = 8        # MUST match firmware VL53L5CX_DET_RESOLUTION
```

### Parsers

- `parse_extended_zframe_line()` - Handles extended format (12 fields/zone + temp + motion)
- `parse_legacy_zframe_line()` - Handles legacy format (4 fields/zone, backward compatible)
- `parse_allparam_line()` - Handles ALLPARAM output
- `parse_motion_line()` - Handles MOTION output

### Requirements

```bash
pip install pyserial pyqtgraph numpy
```

### Usage

```bash
python vl53l5cx_zone_monitor.py
```

Press `R` to reset all data buffers.

## Target Status Codes

| Code | Meaning |
|------|---------|
| 0    | Ranging successful |
| 1    | Minimal measurement |
| 2    | No road |
| 3    | Sigma phase test |
| 4    | No early peak  |
| 5    | Ranging valid (sigma check) |
| 6    | Ranging valid (SRL) |
| 7    | Wrapped target |
| 8    | BitErrorCorrectable |
| 9    | Ranging valid (raw check) |

Status 0, 5, and 9 are considered valid for detection.

## Reflectance Estimation

The reflectance value is given as a percentage (0-255). It represents the estimated reflectivity of the target surface. Higher values indicate more reflective surfaces.

## Range Sigma

The range_sigma_mm value represents the uncertainty of the distance measurement in mm. Lower values indicate more confident/accurate measurements.

## SPADs Enabled

The number of Single-Photon Avalanche Diodes (SPADs) enabled for the measurement. The sensor dynamically adjusts this based on ambient light conditions. More SPADs = better signal in low light.

## Motion Indicator

The motion indicator uses frame-to-frame comparison to detect movement. Each zone has a motion power value. The global indicators provide aggregate motion detection across all zones.

## Notes

- All parameters are conditionally compiled. If a parameter is disabled in the VL53L5CX API (e.g., `VL53L5CX_DISABLE_AMBIENT_PER_SPAD`), the value will be 0.
- The extended ZFRAME is backward compatible: the Python parser tries extended format first, then falls back to legacy format.
- Temperature is the silicon internal temperature, not ambient temperature.
- Zone masking is disabled in the current build — all zones participate in detection.
- 8x8 mode produces significantly more serial data than 4x4. At 115200 baud with extended ZFRAME every frame, expect ~67 KB/s of serial output.