# Project MASSIF 
**Monitoring Automatisé et Systèmes de Surveillance Intelligents de la biodiversité des insectes dans les écosystèmes Forestiers français**

---

# ToF Detection System (VL53L5CX)

## 1. ToF Sensor Overview
The VL53L5CX 520 nm Direct Time-of-Flight (DToF) sensor acts as the autonomous, event-driven trigger for the entire camera system. Unlike the camera, the ToF sensor does not capture optical images; its sole responsibility is to continuously measure the observation area and raise a detection event when an insect breaks the learned signal baseline.

The detection module compares live per-zone signal data against a learned baseline and triggers when a significant **signal drop** and/or **motion** is seen in enough zones. Two operating modes are supported:

- **Single sensor mode** (`VL53L5CX_DUAL_SENSOR = 0`, default): the camera ToF runs continuously.
- **Dual sensor mode** (`VL53L5CX_DUAL_SENSOR = 1`): an external "guardian" ToF is always on; the camera ToF stays in ST sleep by default and is woken only when the guardian confirms a detection.

**Files:**

| File | Role |
|------|------|
| `Inc/vl53l5cx_detection.h` | Public API, all configuration `#defines`, data structures |
| `Src/vl53l5cx_detection.c` | Full implementation (init, power, baseline, detection, dual mode, debug) |
| `Src/app_thread.c` | `sensor_task()` — owns the ToF polling loop and trigger orchestration |
| `Src/main.c` | `VL53L5CX_I2C_Init()` + `VL53L5CX_PowerUp()` in the main thread, before `system_ready` |
| `Src/platform.c` | VL53L5CX low-level I2C (every transfer wrapped by the arbiter mutex) |
| `Src/i2c_arbiter.c/.h` | `g_i2c1_mutex` — serializes I2C1 traffic between camera and ToF |
| `zone_monitor.py` | Real-time ToF visualization (project root) |
| `datalogger.py` | CSV data logging for offline tuning |
| `analysis.py` | Motion-parameter sweep analysis (kept in sync with the header) |

*Thread ownership:* all ToF polling and trigger logic is strictly owned by `sensor_task`. The ToF and the camera share the physical I2C1 bus; the arbiter mutex (`g_i2c1_mutex`) serializes their traffic.

### Ranging grid
The sensor divides its field of view into a matrix of ranging zones. The grid size is selectable via `VL53L5CX_DET_RESOLUTION`: **4x4 (16 zones)** by default for speed, or **8x8 (64 zones)** for finer spatial detail. Every zone independently reports measured distance (mm), returned signal strength (per SPAD), target status, and a motion-indicator value.

---

## 2. Hardware

| Component | Sensor | I2C Address | Location | Role |
|-----------|--------|-------------|----------|------|
| **Primary** | VL53L5CX | 0x29 | Integrated in camera module | High-accuracy depth sensing (sleeps when idle in dual mode) |
| **External** | VL53L5CX | 0x62 | Placed before camera FOV | Continuous guardian monitoring (dual mode only) |

*Address convention:* the primary's 0x29 is the 7-bit address (8-bit 0x52, the VL53L5CX default). The external sensor starts at the same 0x29 and is **re-addressed to 0x62 (8-bit; 7-bit 0x31)** during the dual power-up sequence, so both devices can share the bus.

### Power Control Pins (Dual Mode)

| Pin | Function |
|-----|----------|
| `PD0` (PWR_EN) | Main power enable |
| `PE7` (I2C_RST) | I2C bus reset |
| `PD6` (LPn external) | External sensor power-down (active low) |
| `PQ5` (LPn camera) | Camera ToF power-down (active low) |

### Power-up sequence — `VL53L5CX_PowerUp()`
Called from the main thread (`main.c`) immediately before `system_ready = 1`, so no task can touch I2C before the sensors are alive. The sequence first runs `VL53L5CX_ScanI2CBus()` (probes 7-bit addresses 0x00–0x7F, 10 ms timeout each).

**Dual mode** (`VL53L5CX_DUAL_SENSOR = 1`) — executed only if exactly **3** I2C devices are enumerated:
1. Configure PD0, PE7, PD6, PQ5 as push-pull outputs.
2. `PD0 = SET` (main power on).
3. I2C reset pulse on `PE7` (RESET → SET → RESET).
4. `PQ5 = RESET` (camera ToF off), `PD6 = SET` (external ToF on).
5. Re-address the external ToF from 0x29 → 0x62 (`vl53l5cx_set_i2c_address`) while the camera ToF is still off.
6. `PQ5 = SET` (camera ToF on).

**Single mode** (`VL53L5CX_DUAL_SENSOR = 0`):
1. `PD0 = RESET`, `PE7 = SET`, `PD6 = RESET` — external/aux power controls de-asserted; the active sensor is the ToF at 0x29.

`VL53L5CX_PowerDown()` de-asserts `PD0` (main power off).

## 3. Sensor Target Status Codes

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

**Status sets accepted by the code:**

| Where | Accepted statuses | Why |
|-------|-------------------|-----|
| Primary live detection (`VL53L5CX_Update`) | **5, 6, 9** | 5 = valid (100%); 6 = first range, wrap-around not yet performed (~50%); 9 = valid with large pulse / merged target (~50%) |
| Baseline learning (`VL53L5CX_LearnBaseline`) | **0, 5, 9** | 0 (data not updated) keeps the previous valid reading alive during brief sensor hiccups |
| External guardian detection (`VL53L5CX_External_Update`) | **0, 5, 9** | The guardian only needs to see a change, not a perfect range |
| Debug output (`ZFRAME` / `EXT,ZFRAME`) | **0, 5, 9** | Zones with any other status are printed as `0,0` |

All other statuses are silently skipped for that zone.

---

## 4. Configuration

All tunables live in `Inc/vl53l5cx_detection.h`. Values below are the current defaults.

### 4.1 Mode and grid selection

```c
#define VL53L5CX_DUAL_SENSOR      0   // 0 = single sensor (default ST behavior), 1 = dual sensor
#define VL53L5CX_DET_RESOLUTION   4   // 4 = 4x4 grid (16 zones), 8 = 8x8 grid (64 zones)
```

### 4.2 Dual-mode timers

```c
#define VL53L5CX_DUAL_WAKE_DURATION_MS    5000  // primary stays awake after wake (ms)
#define VL53L5CX_DUAL_CONFIRM_FRAMES       2    // consecutive external detection frames before waking primary
#define VL53L5CX_SENSOR2_BASELINE_SAMPLES  15   // external sensor baseline samples (fewer → faster init)
```

### 4.3 Detection thresholds (resolution-dependent)

| Define | 4×4 (default) | 8×8 | Meaning |
|--------|---------------|-----|---------|
| `VL53L5CX_DET_NUM_ZONES` | 16 | 64 | Grid size |
| `VL53L5CX_DET_BASELINE_SAMPLES` | 20 | 30 | Frames averaged for the baseline |
| `VL53L5CX_DET_THRESHOLD_PCT` | 6 | 15 | Signal drop % that triggers a zone alarm |
| `VL53L5CX_DET_MOTION_THRESH` | 60 | 100 | Motion value that triggers a zone alarm |
| `VL53L5CX_DET_MIN_AFFECTED_ZONES` | 1 | 2 | Zones needed for a global detection |
| `VL53L5CX_DET_MOTION_MIN_ZONES` | 1 | 2 | ST motion plugin: zones for global motion |
| `VL53L5CX_DET_MOTION_PERSIST_FRAMES` | 1 | 3 | ST motion plugin: temporal accumulation |
| `VL53L5CX_DET_MOTION_EXTRA_NOISE` | 0 | 50 | ST motion plugin: extra noise sigma |
| `VL53L5CX_DET_MIN_SIGNAL` | 500 | 500 | Per-SPAD signal gate (below → zone ignored) |

The 8×8 grid uses higher thresholds because smaller zones collect fewer photons and are noisier.

### 4.4 Baseline refresh — enable ONE mode only

```c
/* MODE 1: Periodic (frame-based) — DISABLED by default */
#define VL53L5CX_DET_PERIODIC_RESTART_ENABLED   0
#define VL53L5CX_DET_PERIODIC_RESTART_INTERVAL  500  /* refresh every N Update() frames */

/* MODE 2: Adaptive (detection-based) — ENABLED by default */
#define VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED   1
#define VL53L5CX_DET_REFRESH_WINDOW_SECS        15   /* real-time sliding window (seconds) */
#define VL53L5CX_DET_MAX_DETECTIONS             3    /* max detections per window before refresh */
```

The header explicitly states to pick one mode at a time. The frame counter is a static incremented on every `VL53L5CX_Update()` call (it is *not* reset by detections); the adaptive window is built on `xTaskGetTickCount()` (real time, not frame count). Blocking waits inside the module use `vTaskDelay()` (never `HAL_Delay()`), and the counters are only touched by `sensor_task`, so no mutex is needed.

### 4.5 Debug output

```c
/* DEBUG MODE 1: compact ZFRAME (for zone_monitor.py real-time plots) */
#define VL53L5CX_DET_DEBUG_ZFRAME     1
#define VL53L5CX_DET_DEBUG_ZFRAME_INT 1   /* emit ZFRAME every N Update() frames */

/* DEBUG MODE 2: ALLPARAM (for datalogger.py + analysis.py) */
#define VL53L5CX_DET_DEBUG_ALLPARAMS    0
#define VL53L5CX_DET_DEBUG_ALLPARAM_INT 5   /* emit ALLPARAM every N Update() frames */
```

Both modes can be enabled simultaneously:
- Real-time only → keep ZFRAME, disable ALLPARAM
- Lower UART bandwidth → disable ZFRAME, keep ALLPARAM

### 4.6 Ranging configuration applied by `sensor_task`

| | Primary 4×4 (default) | Primary 8×8 | External (dual mode) |
|---|---|---|---|
| Resolution | `VL53L5CX_RESOLUTION_4X4` | `VL53L5CX_RESOLUTION_8X8` | 4×4 (fixed) |
| Integration time | **30 ms** | 800 ms | 800 ms |
| Ranging frequency | 15 Hz | 15 Hz | 15 Hz |
| Target order | `STRONGEST` | `STRONGEST` | `CLOSEST` |
| Ranging mode | `AUTONOMOUS` | `AUTONOMOUS` | `CONTINUOUS` |
| Sharpener | 10% | 10% | 10% |

The ST motion indicator plugin is initialized inside `VL53L5CX_Configure()` (and `VL53L5CX_External_Configure()`) with the `MOTION_*` defines above, unless the driver is built with `VL53L5CX_DISABLE_MOTION_INDICATOR`; the plugin's `min_nb_for_global_detection`, `nb_of_temporal_accumulations` and `extra_noise_sigma` fields are overwritten with those defines after `vl53l5cx_motion_indicator_init()`, on both sensors. `VL53L5CX_MotionTest()` (manual hook, currently commented out in `main.c`) prints the global motion indicators and every zone's motion value against `VL53L5CX_DET_MOTION_THRESH`.

---

## 5. Single Sensor Mode (`DUAL_SENSOR = 0`)

Standard operation: the camera ToF sensor runs continuously.

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
                              | cooldown=30 |              +------------+
                              +-------------+
```

### API flow

```c
/* main.c — main thread, before tasks start */
VL53L5CX_I2C_Init();
VL53L5CX_PowerUp();          /* single-mode sequence */

/* app_thread.c — sensor_task() */
VL53L5CX_Init(&hi2c1);       /* is_alive retry 10×300 ms, init retry 3×200 ms */
VL53L5CX_Configure(VL53L5CX_RESOLUTION_4X4, 30, 15);  /* 4x4, 30 ms integration, 15 Hz */
VL53L5CX_StartRanging();
VL53L5CX_LearnBaseline();    /* 20 samples + 5 settle frames */

while (1) {
    if (VL53L5CX_Update()) {
        if (VL53L5CX_IsInsectDetected() && cooldown == 0) {
            VL53L5CX_DetectionResult_t res = VL53L5CX_GetResult();
            /* res.trigger_source: 1=SIGNAL, 2=MOTION, 3=BOTH */
            /* trigger capture, LEDs, set cooldown = 30 */
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
}
```

---

## 6. Dual Sensor Mode (`DUAL_SENSOR = 1`)

When dual mode is enabled, the **external sensor becomes the always-on "guardian"**, and the **primary camera ToF stays in ST sleep by default** to save power. The primary is only woken when the guardian confirms a detection, then runs for a fixed window (`VL53L5CX_DUAL_WAKE_DURATION_MS`) and goes back to sleep.

```
+--------------+     +-----------+     +-----------+
| Init         | --> | Learn     | --> | Guardian  |
| Both Sensors |     | Both      |     | MONITORING|
| (guardian    |     | Baselines |     | Primary   |
|  @ 0x62)     |     |           |     | in SLEEP  |
+--------------+     +-----------+     +-----------+
                                               |
                                      +--------+v--------+
                                      | Guardian sees    |
                                      | detection?       |
                                      +--------+--------+
                                               |
                                   +----------+----------+
                                   |                     |
                             YES   |                     NO |
                                   |                     |
                          +v-------+-------+            +v-------+
                          | CONFIRM_FRAMES  |            | back to|
                          | (2) consecutive |            | monitoring
                          | detections      |            +--------+
                          +-------+---------+
                                  |
                     +------------+-------------+
                     |                         |
              +v-----+-----+           +v-------+-------+
              | Primary runs |          | Wake window   |
              | detection loop|         | (5000 ms)     |
              | 5000 ms          |        expires       |
              +-----------------+           +-----------+
```

### Initialization (inside `sensor_task`)

```c
VL53L5CX_Init(&hi2c1);          /* primary @ 0x29 — fatal on failure */
VL53L5CX_Configure(VL53L5CX_RESOLUTION_4X4, 30, 15);
VL53L5CX_StartRanging();

if (VL53L5CX_External_Init() == 0) {   /* external @ 0x62 (7-bit 0x31) */
    VL53L5CX_External_Configure();
    VL53L5CX_External_StartRanging();  /* state → EXTERNAL_STATE_MONITORING */
}
/* on failure: warning only → graceful degradation, primary-only mode */

VL53L5CX_LearnBaseline();        /* primary baseline (fatal if no valid zones) */
VL53L5CX_External_LearnBaseline(); /* external baseline: 15 samples + 3 settle */
VL53L5CX_Primary_SleepAtStartup();  /* park the camera ToF in ST sleep */
```

**Notes:**
- `VL53L5CX_Init()` fails the task if the primary is not alive. An `External_Init()` failure only prints `[EXT] Warning: External sensor initialization failed. Primary-only mode.` and the task continues.
- `Primary_SleepAtStartup()` is required at init: the primary is physically ranging at that moment, but the module's state still holds the initial `PRIMARY_STATE_SLEEP` value (a plain `Primary_Sleep()` would treat it as "already asleep" and no-op). The startup variant first forces the state to `ACTIVE`, then runs the real sleep sequence.
- A missing **primary** baseline parks the task in `SENSOR_STATE_STOPPED` ("Primary ToF baseline not ready!"). The **external** baseline is optional; a warning is printed if it fails.

### Main loop (inside `sensor_task`)

```c
while (1) {
    if (g_sensor_state == SENSOR_STATE_PAUSED) { vTaskDelay(50); continue; }

    if (cooldown > 0) cooldown--;

    if (VL53L5CX_External_GetState() == EXTERNAL_STATE_MONITORING)
        VL53L5CX_External_Update();        /* guardian always polling */

    VL53L5CX_Primary_CheckWakeTimeout();   /* → SLEEP after 5000 ms */

    if (VL53L5CX_Primary_IsActive()) {
        VL53L5CX_Update();                 /* primary detection */
        if (VL53L5CX_IsInsectDetected() && cooldown == 0) {
            if (g_capture_busy) continue;  /* busy gate */
            /* LEDs → Capture_RequestSnapshot(60000) → cooldown = 30 */
        }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
}
```

### Wake logic (inside `VL53L5CX_External_Update`)
- Each external detection while the primary is sleeping increments a confirm counter; any non-detecting frame resets it to 0.
- When the count reaches `VL53L5CX_DUAL_CONFIRM_FRAMES` (2), the external state goes to `EXTERNAL_STATE_DETECTED` and `VL53L5CX_Primary_Wake()` is called.
- `VL53L5CX_Primary_CheckWakeTimeout()` returns the primary to sleep once `VL53L5CX_DUAL_WAKE_DURATION_MS` (5000 ms) has elapsed since wake, and resets the external state to `EXTERNAL_STATE_MONITORING`.

### External state machine

```
+---------------------------+
| EXTERNAL_STATE_MONITORING |  (always-on guardian)
+---------------------------+
          | Detection confirmed (2 consecutive frames)
          v
+---------------------------+
| EXTERNAL_STATE_DETECTED   |  → VL53L5CX_Primary_Wake()
+---------------------------+
          | Primary active window elapsed (5000 ms)
          v
+---------------------------+
| EXTERNAL_STATE_MONITORING |  (loop back)
+---------------------------+
```

### Primary state machine & power management

```
+-----------------+
| PRIMARY_SLEEP   |  (default, ST sleep)
+-----------------+
          | VL53L5CX_Primary_Wake()
          v
+-----------------+
| PRIMARY_WAKEUP  |  (power mode VL53L5CX_POWER_MODE_WAKEUP)
+-----------------+
          | 200 ms → StartRanging → 200 ms
          v
+-----------------+
| PRIMARY_ACTIVE  |  (ranging, detection enabled)
+-----------------+
          | 5000 ms elapsed (CheckWakeTimeout)
          v
+-----------------+
| PRIMARY_RETURNING|
+-----------------+
          | StopRanging → 50 ms → POWER_MODE_SLEEP
          v
+-----------------+
| PRIMARY_SLEEP   |
+-----------------+
```

- `VL53L5CX_Primary_Sleep()` only acts from `ACTIVE`/`RETURNING`: stop ranging → 50 ms → `vl53l5cx_set_power_mode(POWER_MODE_SLEEP)`. The sensor keeps its firmware and configuration in sleep, so waking is fast.
- `VL53L5CX_Primary_Wake()` skips if already active: set `POWER_MODE_WAKEUP` → 200 ms → `StartRanging` → 200 ms → `ACTIVE`, recording `HAL_GetTick()` for the timeout.
- `VL53L5CX_IsPrimaryActive()` returns non-zero in `ACTIVE`/`WAKEUP`/`RETURNING`.

---

## 7. Detection Logic

### Per-zone evaluation (every `VL53L5CX_Update()` call)

1. Wait for data ready (`VL53L5CX_WaitForDataReady`, 1000 ms timeout) and fetch ranging data.
2. For each of the 16 (or 64) zones:
    1. **Signal channel (gated):** the zone must have a learned baseline (`s_zone_valid`), a target status of 0/5/9 (both sensors), and a current signal present and ≥ `VL53L5CX_DET_MIN_SIGNAL` (500) — otherwise it is skipped for the signal channel.
    2. **Zero-baseline guard:** if the current signal is 0 and the baseline is below `VL53L5CX_DET_MIN_SIGNAL`, the baseline is zeroed for that zone (prevents ghost zones from re-triggering forever).
    3. **Signal trigger:** drop = `|current − baseline| × 100 / baseline`; triggered when the drop exceeds `VL53L5CX_DET_THRESHOLD_PCT` (6% at 4×4).
    4. **Motion channel (ST plugin):** a zone is motion-triggered when its per-zone motion value (indexed through the plugin's `map_id[]`) is ≥ `VL53L5CX_DET_MOTION_THRESH` (60 at 4×4). On the **primary** sensor this channel is evaluated for *every* zone — independently of baseline validity and ranging status (the plugin reports motion even for status 255 zones and for zones that were empty at boot); on the **external (guardian)** sensor it sits inside the same gates as the signal channel.
    5. A zone is in alarm if **either** channel fires; the zone index plus the drop % (signal) or the raw motion value (motion-only) is recorded in the result.
3. **Global alarm:** if `affected_count ≥ VL53L5CX_DET_MIN_AFFECTED_ZONES` (1 at 4×4), `insect_detected` is set.

### Trigger source encoding

```c
#define VL53L5CX_TRIG_SIGNAL  0x01  // signal drop only
#define VL53L5CX_TRIG_MOTION  0x02  // motion indicator only
#define VL53L5CX_TRIG_BOTH    0x03  // both channels
```

`VL53L5CX_GetResult()` returns the `VL53L5CX_DetectionResult_t`: `insect_detected`, `trigger_source`, `affected_count`, `affected_zones[]`, `affected_drop[]` (drop % when the signal channel fired, raw motion value 0-255 when motion-only), and `valid_measurements` (zones with signal ≥ `VL53L5CX_DET_MIN_SIGNAL` this frame).

### Baseline learning (`VL53L5CX_LearnBaseline`)

1. `VL53L5CX_ResetBaseline()` clears the arrays and the ready flag.
2. Averages `VL53L5CX_DET_BASELINE_SAMPLES` valid frames (20 at 4×4, 30 at 8×8); a zone is accumulated only when its status is in {0,5,9} and its signal is ≥ `VL53L5CX_DET_MIN_SIGNAL`.
3. 5 settle frames are discarded afterwards (external sensor: 3) so the first live detection is not compared against stale state.
4. Sets `s_baseline_ready = 1` and prints a `BASELINE,` line. `VL53L5CX_IsBaselineReady()` reports the state.

### Baseline refresh

**Periodic** (MODE 1, disabled by default): every `VL53L5CX_DET_PERIODIC_RESTART_INTERVAL` frames → stop ranging, 50 ms, restart, 200 ms, re-learn baseline.

**Adaptive** (MODE 2, default): a sliding real-time window of `VL53L5CX_DET_REFRESH_WINDOW_SECS` (15 s) built on `xTaskGetTickCount()`. If more than `VL53L5CX_DET_MAX_DETECTIONS` (3) detections occur inside the window, the baseline is re-learned — the "adaptation fatigue" guard for environments that have permanently changed (e.g. an insect landed on the lens). The same mechanism runs independently for the external sensor in dual mode.

Refresh procedure: `vl53l5cx_stop_ranging()` → 50 ms → `vl53l5cx_start_ranging()` → 200 ms → `LearnBaseline()`.

### Trigger orchestration & synchronization safeguards (in `sensor_task`)

1. **The busy gate (`g_capture_busy`):** if a capture/SD write is already in flight, the new detection is dropped (no queue buildup, no re-entrant triggers).
2. **I2C arbitration:** the ToF and the camera share the physical I2C1 bus; every ToF I2C transfer in `platform.c` takes `g_i2c1_mutex`, so ToF polling never collides with camera register access.
3. **The cooldown phase:** after a successful trigger, `cooldown = 30` frames. During cooldown, detections are ignored — a dead time while the insect leaves and the LED afterglow decays.
4. **State handshake:** on trigger, the task sets `SENSOR_STATE_PAUSED`, drives the LEDs, issues `Capture_RequestSnapshot(60000)`, restores `SENSOR_STATE_RUNNING` and restarts ranging.

---

## 8. Debug Serial Protocol (UART1 @ 115200)

The current firmware build emits the following machine-readable lines, all prefixed with an `EXT,` variant for the external sensor in dual mode where applicable:

### ZFRAME (primary) — DEBUG MODE 1
```
ZFRAME,<temp>,<sig0>,<dist0>,<base_sig0>,<base_dist0>,<motion0>,<sig1>,...
```
5 fields per zone (signal, distance, baseline signal, baseline distance, motion), plus the temperature. At 4×4: 1 + 5×16 = 81 numeric fields after the `ZFRAME` token (82 tokens). At 8×8: 321 numeric fields. Zones whose status is not 0/5/9 are printed as `0,0`.

### EXT,ZFRAME (external, dual mode)
Same layout with the `EXT,` prefix; emitted right after the primary ZFRAME using the same interval counter.

### ALLPARAM (primary) — DEBUG MODE 2
```
ALLPARAM,<temp>,<cur_sig0>,<base_sig0>,<cur_dist0>,<base_dist0>,<drop_pct0>,...
```
5 fields per zone, including the computed drop percentage. `EXT,ALLPARAM` is the dual-mode external variant (emitted every `VL53L5CX_DET_DEBUG_ALLPARAM_INT` frames).

> **Note on the Python tools:** `zone_monitor.py` and `datalogger.py` also implement parsers for an **extended 12-field ALLPARAM layout** (adds ambient, sigma, reflectance, status, spads, targets, drop, valid) and for `MOTION,...` / `DETF,...` / `DET,...` event lines used by an extended/optional debug firmware build. The current build emits only the 5-field format above; the extra parsers are kept for forward/backward compatibility.

### BASELINE
```
BASELINE,<sig0>,<dist0>,<sig1>,<dist1>,...
```
Printed once per `VL53L5CX_LearnBaseline()` (primary only in the current build; the monitor also accepts an `EXT,`-prefixed variant).

### Human-readable event lines
```
[ToF] Configured: res=4, int=30ms, freq=15Hz
[ToF] Ranging started
[BASELINE] Learning 20 samples + 5 settle frames...
[BASELINE] Done. Valid zones: 14/16
[ToF] Adaptive refresh: 4 detections in 15s
[ToF] Window: 1 detections in 15s — no refresh
[EXT] Detection confirmed! Waking primary sensor...
[PRIMARY] Waking from sleep
[PRIMARY] Active until 123456 ms
[PRIMARY] Wake timeout expired (5000 ms), returning to sleep
[PRIMARY] Entering sleep mode
>>> INSECT DETECTED [SIGNAL]!
>>> INSECT DETECTED [MOTION]!
>>> INSECT DETECTED [SIGNAL+MOTION]!
>>> INSECT DETECTED (Primary, woken by external) [SIGNAL]!
```
The `>>> INSECT DETECTED` lines are emitted under `#if PERF_DEBUG_LEVEL >= 1`.

---

## 9. Python Tools

### zone_monitor.py (project root)
Real-time visualization:
- Set `SERIAL_PORT` (e.g. `COM7`) and `BAUD_RATE` (115200) at the top of the file.
- `RESOLUTION` **must** match `VL53L5CX_DET_RESOLUTION` in the header (4 or 8); the monitor derives 16/64 zones from it and warns on resolution mismatch.
- `DUAL_SENSOR_MODE = True` when the firmware has `VL53L5CX_DUAL_SENSOR = 1`; external lines are routed from the `EXT,` prefix (older `S2,` builds are also accepted).
- Per-sensor tabs (S1 primary, S2 guardian): compact view (signal vs baseline, drop %, motion, per-zone heatmap) and detailed view (per-zone plots).
- GLOBAL view: temperature, motion/detection state, live detection counter.
- Threshold indicator lines (6% drop, motion) are drawn for reference.
- `R` key resets all data.

### datalogger.py
Saves every frame to `tof_datalog_<timestamp>.csv` (per-zone signal, distance, baseline, motion, drop %, reflectance columns plus MOTION/DETF globals when present); the output feeds `analysis.py`.

### analysis.py
Motion-parameter sweep/tuning tool; the parameter block at the top must stay in sync with `vl53l5cx_detection.h`.

### vl53l5cx_zone_monitor.py (workspace root)
Original monitor (S1 / EXT tabs), kept for reference. `zone_monitor.py` is the current tool.

**Dependencies:** `pip install pyserial pyqtgraph numpy`

---

## 10. Integration

### main.c (main thread)
```c
VL53L5CX_I2C_Init();   // I2C1 — board-specific init, stays in main.c
VL53L5CX_PowerUp();    // single/dual power sequence + I2C bus scan
system_ready = 1;      // gate: FreeRTOS tasks only run after this
```

### app_thread.c → sensor_task()
- Single mode: `Init → Configure(res, 30, 15) → StartRanging → LearnBaseline → Update()` loop with the busy gate + cooldown + LED/capture handshake described in §7.
- Dual mode: adds `External_Init/Configure/StartRanging → External_LearnBaseline → Primary_SleepAtStartup` at startup, and `External_Update + Primary_CheckWakeTimeout + Primary_IsActive` gating in the loop (see §6).

---

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `[ToF] ERROR: Sensor not detected!` at boot | ToF not powered / I2C problem | Check the `VL53L5CX_PowerUp()` sequence and the bus scan output (0x29 must appear) |
| Dual power-up does nothing | `ScanI2CBus()` did not enumerate 3 devices | Fix wiring; both ToFs + camera must be visible on the bus before the power sequence |
| `[EXT] ERROR: Sensor not detected at 0x62!` | Address change failed at power-up | Confirm `[OK] External ToF address changed to 0x62` is printed; otherwise both ToFs collide at 0x29 |
| External sensor init failed (warning only) | Degraded to primary-only mode | Check external power pins (PD6) and the re-addressing log |
| `[ERROR] Primary ToF baseline not ready!` (task stopped) | No valid zones during learning | Clear the field of view, raise integration time, compare `VL53L5CX_DET_MIN_SIGNAL` (500) with the actual ZFRAME signal values |
| Primary stuck in SLEEP (dual mode) | Wake never confirmed | Lower `VL53L5CX_DUAL_CONFIRM_FRAMES` (2 → 1); check `EXT,ZFRAME` for external detections |
| Zones printed as 0 / invalid status | Sensor not tracking | Increase integration time, clean the lens, check ambient light |
| Signal drop always 0% | Baseline not learned or zone invalid | Verify `VL53L5CX_IsBaselineReady()`, re-learn with a clear FOV |
| Too many false detections | Thresholds too low | Raise `DET_THRESHOLD_PCT`, `DET_MOTION_THRESH`, `DET_MIN_AFFECTED_ZONES` (see TUNING_GUIDE.md §8) |
| No detections at all | Thresholds too high / signal below gate | Lower thresholds; compare `ZFRAME` signal against `DET_MIN_SIGNAL` (500) |
| Monitor shows no data / resolution mismatch | `zone_monitor.py` `RESOLUTION` ≠ firmware | Sync the `RESOLUTION` and `DUAL_SENSOR_MODE` constants |
| I2C collisions / bus errors | Camera + ToF share I2C1 | The arbiter (`g_i2c1_mutex`) already guards ToF transfers; verify both ToF and camera I2C wrappers take the mutex |
| Re-triggers on the same insect | Cooldown too short | Increase the `cooldown = 30` value in `sensor_task` |

---

## 12. Design Philosophy

1. **Modular** — the `VL53L5CX_DUAL_SENSOR` define switches between single and dual mode; with `= 0` the code follows the standard VL53L5CX flow.
2. **Resolution-aware** — 4×4/8×8 selectable via `VL53L5CX_DET_RESOLUTION`; all thresholds scale per resolution.
3. **Self-calibrating** — adaptive (and optional periodic) baseline refresh keeps detection accurate as the environment changes.
4. **Energy-efficient** — in dual mode the camera ToF stays in ST sleep almost all the time; only the guardian runs continuously.
5. **Confidence-based** — the trigger source (signal / motion / both) is reported for every detection, letting downstream logic weight each trigger.
6. **Robust** — graceful degradation (external init failure → primary only), retry loops at init, I2C arbitration, busy gate and cooldown against re-entrancy.
