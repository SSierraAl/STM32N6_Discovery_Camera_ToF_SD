# VL53L5CX ToF Sensor Detection Module

## Overview

This module provides insect/presence detection using the ST VL53L5CX Time-of-Flight (ToF) depth sensor. It supports both **single sensor** and **dual sensor** modes. The detection logic compares live signal data against a learned baseline, triggering when significant signal drop or motion is detected across multiple zones.

**Files:**
- `Inc/vl53l5cx_detection.h` — Public API, configuration defines, data structures
- `Src/vl53l5cx_detection.c` — Full implementation

---

## Hardware

| Component | Sensor | I2C Address | Location | Role |
|-----------|--------|-------------|----------|------|
| **Primary** | VL53L5CX | 0x29 | Integrated in camera module | High-accuracy depth sensing (sleeps when idle in dual mode) |
| **External** | VL53L5CX | 0x62 | Placed before camera FOV | Continuous guardian monitoring |

### Power Control Pins (Dual Mode)

| Pin | Function |
|-----|----------|
| `PD0` (PWR_EN) | Main power enable |
| `PE7` (I2C_RST) | I2C bus reset |
| `PD6` (LPn external) | External sensor power-down (active low) |
| `PQ5` (LPn camera) | Camera ToF power-down (active low) |

---

## Sensor Target Status Codes

From the VL53L5CX datasheet:

| Code | Meaning | Confidence |
|------|---------|------------|
| **0** | Ranging data not updated | — |
| **1** | Signal rate too low on SPAD array | <50% |
| **2** | Target phase | <50% |
| **3** | Sigma estimator too high | <50% |
| **4** | Target consistency failed | <50% |
| **5** | **Range valid** | **100%** |
| **6** | Wrap-around not performed (first range) | **~50%** |
| **7** | Rate consistency failed | <50% |
| **8** | Signal rate too low for current target | <50% |
| **9** | **Range valid with large pulse** (merged target) | **~50%** |
| **10** | Range valid, no target at previous range | <50% |
| **11** | Measurement consistency failed | <50% |
| **12** | Target blurred by sharpener | <50% |
| **13** | Target detected but inconsistent | <50% |
| **255** | No target detected | — |

**This module accepts zones with status `0`, `5`, or `9`:**
- Status `5`: Full confidence — always used
- Status `9`: Partial confidence — used (large pulse / merged target still provides valid depth)
- Status `0`: Data not updated — used as fallback (keeps previous valid reading alive during brief sensor hiccups)

All other statuses are silently skipped for that zone.

---

## Configuration

All configurable parameters are in `vl53l5cx_detection.h`:

### Mode Selection

```c
#define VL53L5CX_DUAL_SENSOR  0  // Single mode (default ST behavior)
#define VL53L5CX_DUAL_SENSOR  1  // Dual mode (external guardian + primary sleeps)
```

### Dual Mode Timers

```c
#define VL53L5CX_DUAL_WAKE_DURATION_MS    5000   // Primary stays awake after wake (ms)
#define VL53L5CX_DUAL_MONITOR_AFTER_WAKE_MS 1000 // Primary detection window after wake
#define VL53L5CX_DUAL_CONFIRM_FRAMES         3   // External confirmation before waking primary
```

### Detection Thresholds

```c
#define VL53L5CX_DET_NUM_ZONES             16    // 4x4 grid = 16 zones
#define VL53L5CX_DET_MIN_SIGNAL            50    // Min signal per SPAD (below = ignore zone)
#define VL53L5CX_DET_THRESHOLD_PCT          6    // Signal drop % to trigger zone alarm
#define VL53L5CX_DET_MIN_AFFECTED_ZONES     3    // Zones needed for global detection
#define VL53L5CX_DET_BASELINE_SAMPLES      20    // Samples averaged for baseline
#define VL53L5CX_SENSOR2_BASELINE_SAMPLES  15    // External sensor baseline samples
```

### Motion Indicator

```c
#define VL53L5CX_DET_MOTION_THRESH         100   // Motion value to trigger zone alarm
#define VL53L5CX_DET_MOTION_MIN_ZONES       2    // Global motion threshold zones
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  2    // Temporal accumulation before motion triggers
#define VL53L5CX_DET_MOTION_EXTRA_NOISE    10    // Extra noise sigma
```

### Baseline Refresh

```c
#define VL53L5CX_DET_PERIODIC_RESTART_ENABLED   1  // Enable periodic restart
#define VL53L5CX_DET_PERIODIC_RESTART_INTERVAL 300 // Frames between periodic restarts
#define VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED   1  // Enable adaptive refresh
#define VL53L5CX_DET_REFRESH_WINDOW_SECS       30  // Seconds in adaptive window
#define VL53L5CX_DET_MAX_DETECTIONS            10  // Detections in window before refresh
```

### Debug Output

```c
#define VL53L5CX_DET_DEBUG_ZFRAME      1  // Enable ZFRAME debug lines
#define VL53L5CX_DET_DEBUG_ZFRAME_INT  20 // ZFRAME every N calls to Update()
#define VL53L5CX_DET_DEBUG_ALLPARAMS   0  // Enable ALLPARAM debug lines
#define VL53L5CX_DET_DEBUG_ALLPARAM_INT 50 // ALLPARAM every N calls
```

---

## Single Sensor Mode (`DUAL_SENSOR = 0`)

Standard operation. The camera ToF sensor runs continuously.

```
+-----------+     +-----------+     +-----------+
| Init      | --> | Learn     | --> | Continuous|
| Configure |     | Baseline  |     | Update()  |
+-----------+     +-----------+     +-----------+
                                                   |
                                          +--------+v--------+
                                          | Zone trigger?    |
                                          +--------+--------+
                                                   |
                                     +-------------+-------------+
                                     |                           |
                               YES   |                       NO  |
                                     |                           |
                              +v-----+-----+              +v-----+-----+
                              | Capture +   |              | Loop back  |
                              | LED/RGB     |              | to Update()|
                              | Refresh     |              +------------+
                              +-------------+
```

### API Flow

```c
/* In sensor_task (or main loop) */
VL53L5CX_Init(&hi2c1);
VL53L5CX_Configure(VL53L5CX_RESOLUTION_4X4, 800, 15);  // 4x4, 800ms integration, 15Hz
VL53L5CX_StartRanging();
VL53L5CX_LearnBaseline();  // Takes ~2 seconds (20 samples + settle)

while (1) {
    if (VL53L5CX_Update()) {
        if (VL53L5CX_IsInsectDetected()) {
            res = VL53L5CX_GetResult();
            /* res.trigger_source: 1=SIGNAL, 2=MOTION, 3=BOTH */
            /* Trigger capture, LEDs, etc. */
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

---

## Dual Sensor Mode (`DUAL_SENSOR = 1`)

Reversed roles: external sensor (guardian) is always ON. Primary camera ToF sleeps by default and only wakes when the guardian detects something.

### Architecture

```
                    CONTINUOUSLY ON
    +-----------------------------+
    |  External ToF (0x62)        |
    |  VL53L5CX_External_Update() |
    +------------+----------------+
                 |
        Detects motion or
        signal drop >= 6%
        in >= 3 zones
                 |
                 v
    +----------------------------+
    | Wake Primary (VL53L5CX)    |
    | VL53L5CX_Primary_Wake()    |
    +------------+---------------+
                 |
        Primary active for
        DUAL_WAKE_DURATION_MS
                 |
         +-------+-------+
         |               |
   Primary detects   Timeout expires
    insect in zone         |
         |                 |
         |            Return to sleep
         |            VL53L5CX_Primary_Sleep()
         |                 |
         |                 v
         |        +------------------+
         |        | External resumes |
         |        | MONITORING state |
         |        +------------------+
         v
    Capture + LED
    + baseline refresh
```

### API Flow

```c
/* In sensor_task init */
VL53L5CX_Init(&hi2c1);              // Primary at 0x29
VL53L5CX_Configure(...);
VL53L5CX_StartRanging();

/* Init external */
VL53L5CX_External_Init();
VL53L5CX_External_Configure();
VL53L5CX_External_StartRanging();

/* Learn baselines (sensor is active from StartRanging) */
VL53L5CX_LearnBaseline();             // Primary baseline
VL53L5CX_External_LearnBaseline();    // External baseline

/* Put primary to sleep — both baselines are learned */
VL53L5CX_Primary_Sleep();

while (1) {
    /* External always monitoring */
    if (VL53L5CX_External_GetState() == EXTERNAL_STATE_MONITORING) {
        VL53L5CX_External_Update();
    }

    /* Check if primary wake timeout expired */
    VL53L5CX_Primary_CheckWakeTimeout();

    /* When primary is awake (woken by external), also poll it */
    if (VL53L5CX_Primary_IsActive()) {
        if (VL53L5CX_Update()) {
            if (VL53L5CX_IsInsectDetected()) {
                /* Capture, LED, etc. */
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
}
```

### State Machines

#### External Sensor States
```
IDLE -> MONITORING -> DETECTED -> WAITING -> MONITORING -> ...
                              |
                              +-- (confirm frames) --> WAKES PRIMARY
```

#### Primary Sensor States
```
SLEEP <--returning-- ACTIVE <--waking-- SLEEP
   ^                          |
   +-- timeout expired -------+
```

---

## Detection Logic

### Per-Zone Evaluation (each `Update()` call)

For each of the 16 zones in the 4x4 grid:

1. **Skip invalid zones**: If `s_zone_valid[z]` is 0 (no baseline), skip
2. **Check target status**: Only process zones with status `0`, `5`, or `9`
3. **Min signal check**: If `signal_per_spad < DET_MIN_SIGNAL` (50), skip
4. **Calculate signal drop**: `abs(current - baseline) * 100 / baseline`
5. **Signal trigger**: If `drop_pct > DET_THRESHOLD_PCT` (6%), zone is triggered
6. **Motion trigger**: If `motion_value >= DET_MOTION_THRESH` (100), zone is triggered
7. **Zone alarm**: If either signal OR motion triggered, record zone
8. **Global alarm**: If `affected_zones >= DET_MIN_AFFECTED_ZONES` (3), insect is detected

### Trigger Source Encoding

```c
#define VL53L5CX_TRIG_SIGNAL   1  // Signal drop only
#define VL53L5CX_TRIG_MOTION   2  // Motion indicator only
#define VL53L5CX_TRIG_BOTH     3  // Both signal drop AND motion
```

### Baseline Refresh

Two mechanisms keep baselines accurate over time:

**Periodic Refresh** (`DET_PERIODIC_RESTART_ENABLED`): Every `DET_PERIODIC_RESTART_INTERVAL` (300) frames:
1. Stop ranging
2. Restart ranging
3. Relearn baseline (20 samples)

**Adaptive Refresh** (`DET_ADAPTIVE_REFRESH_ENABLED`): Every `DET_REFRESH_WINDOW_SECS` (30) seconds, check if detections exceeded `DET_MAX_DETECTIONS` (10). If so, relearn baseline. This handles environments where many detections corrupt the baseline over time.

---

## Debug Serial Protocol

### ZFRAME (primary sensor)
```
ZFRAME,<temp>,<sig0>,<dist0>,<base_sig0>,<base_dist0>,<motion0>,...
```
16 zones × 5 values = 82 comma-separated fields total.

### ZFRAME (external sensor, dual mode only)
```
EXT,ZFRAME,<temp>,<sig0>,<dist0>,<base_sig0>,<base_dist0>,<motion0>,...
```
Same format, prefixed with `EXT,`.

### ALLPARAM (primary sensor)
```
ALLPARAM,<temp>,<cur_sig0>,<base_sig0>,<cur_dist0>,<base_dist0>,<drop_pct0>,...
```
Includes computed drop percentage.

### ALLPARAM (external sensor)
```
EXT,ALLPARAM,<temp>,...
```

### BASELINE
```
BASELINE,<base_sig0>,<base_dist0>,<base_sig1>,<base_dist1>,...
EXT,BASELINE,...
```

### Detection Events
```
>>> INSECT DETECTED [SIGNAL]!
>>> INSECT DETECTED [MOTION]!
>>> INSECT DETECTED [SIGNAL+MOTION]!
>>> INSECT DETECTED (Primary, woken by external) [SIGNAL]!
[EXT] Detection confirmed! Waking primary sensor...
```

---

## Python Monitor

`vl53l5cx_zone_monitor.py` — Real-time visualization of ToF data.

**Features:**
- Dark theme (Fusion style)
- Dual tabs: "S1 (Primary)" and "EXT (Guardian)"
- 4 line plots per tab: Signal, Distance, Drop%, Motion
- 1 heatmap per tab: Signal drop %
- Threshold indicator lines (6% drop, 40 motion)
- Status labels with color coding (green OK, red alarm)
- Temperature display (green <45°C, orange 45-60°C, red >60°C)
- Global detection counter
- `R` key resets all data

**Dependencies:** `pip install pyserial pyqtgraph numpy`

**Usage:** `python vl53l5cx_zone_monitor.py`

---

## Integration

### In `app_thread.c` → `sensor_task()`:

```c
#if VL53L5CX_DUAL_SENSOR
    // External sensor init
    VL53L5CX_External_Init();
    VL53L5CX_External_Configure();
    VL53L5CX_External_StartRanging();

    // Learn both baselines, then sleep primary
    VL53L5CX_LearnBaseline();
    VL53L5CX_External_LearnBaseline();
    VL53L5CX_Primary_Sleep();
#else
    // Standard single sensor init
    VL53L5CX_LearnBaseline();
#endif

while (1) {
#if VL53L5CX_DUAL_SENSOR
    // Poll external continuously
    if (VL53L5CX_External_GetState() == EXTERNAL_STATE_MONITORING)
        VL53L5CX_External_Update();
    VL53L5CX_Primary_CheckWakeTimeout();

    // Poll primary when awake
    if (VL53L5CX_Primary_IsActive() && VL53L5CX_Update()) {
        if (VL53L5CX_IsInsectDetected()) { /* capture */ }
    }
#else
    if (VL53L5CX_Update() && VL53L5CX_IsInsectDetected()) { /* capture */ }
#endif
}
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| External sensor never refreshes baseline | `VL53L5CX_External_Update()` not called | Verify `EXTERNAL_STATE_MONITORING` is set |
| Primary sensor stuck in SLEEP | Wake not triggered | Check `VL53L5CX_DUAL_CONFIRM_FRAMES` and detection threshold |
| Zones show status != 0,5,9 | Sensor not tracking properly | Increase integration time, check lens cleanliness |
| Signal drop always 0% | Baseline not learned | Call `VL53L5CX_LearnBaseline()` with clear FOV |
| Too many false detections | Threshold too low | Increase `VL53L5CX_DET_THRESHOLD_PCT` or `MIN_AFFECTED_ZONES` |
| No detections at all | Threshold too high | Decrease thresholds, verify `VL53L5CX_IsBaselineReady()` |
| I2C collisions | Both sensors at same address | Power external first, change address to 0x62 before powering primary |

---

## Design Philosophy

1. **Modular** — `DUAL_SENSOR` define switches between single and dual mode. When `DUAL_SENSOR=0`, code follows standard VL53L5CX flow.
2. **Self-calibrating** — Baselines are relearned periodically and adaptively, keeping detection accurate over changing environments.
3. **Energy-efficient** — In dual mode, the camera ToF (power-hungry) sleeps 95%+ of the time. Only the external guardian runs continuously.
4. **Confidence-based** — Trigger source (signal/motion/both) is reported for each detection, allowing downstream logic to prioritize.
5. **Zone-aware** — Detection requires N zones (not just 1) to trigger, reducing false positives from single-zone noise.