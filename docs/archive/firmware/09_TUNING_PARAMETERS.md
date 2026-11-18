# Tuning Parameters — `Inc/app_config.h`

`app_config.h` is the **single source of truth** for system behavior. Every
parameter below has its **current field value** (this build), its meaning, and
what to change and why. Change → rebuild → flash. ToF parameters changed
without an empty-box validation pass can silently reintroduce false triggers:
collect a 10–15 min empty-trap log first (`[ToF]`/`[ADAPT]`/`TOFEVT` lines).

Legend: **TUNE** = regularly adjusted, **FIX** = change only with reason,
**DEBUG** = 0 in production.

## 1. Operating mode

| Parameter | Current | Type | Meaning |
| :--- | :--- | :--- | :--- |
| `CAPTURE_MODE` | 4 | FIX | 0 = button/manual, 1 = continuous, 2 = batch (tearing), **4 = callback-batch (production)** |
| `TEST_TOF_MODE` | 0 | FIX | 1 = ToF-only commissioning build: camera + SD never initialized, no photos, RED LED on detection, USER button re-learns baseline |
| `CAM_BINNING` | 1 (auto in mode 4) | FIX | 1 = 1296×972 DCMIPP-scaled output, 0 = 2592×1944 (full). Not sensor binning |
| `VL53L5CX_DUAL_SENSOR` | 0 | FIX | 1 = guardian ToF wakes a sleeping primary (see §9) |
| `VL53L5CX_DET_HIGH_SENS_CAMERA` | 1 | FIX | Use the validated per-zone 4×4 detector in camera builds (0 = legacy camera-only temporal filter) |
| `VL53L5CX_DET_HIGH_SENS_TEST` | 1 | FIX | Same detector in TEST builds |
| `TEST_TOF_LED_MS` | 300 | FIX | RED LED duration per detection in TEST mode |
| `VL53L5CX_DET_ZONE_SURVEY` | 0 | DEBUG | 1 = temporary 4×4 zone survey (periodic zone snapshots every `VL53L5CX_DET_ZONE_LOG_INTERVAL_MS`) |

## 2. Camera & image quality

| Parameter | Current | Type | Meaning / tuning |
| :--- | :--- | :--- | :--- |
| `SNAP_FPS` | 30 | FIX | Pipe rate; one frame = 33 ms |
| `SNAP_WARMUP_FRAMES` | 8 | TUNE | Frames discarded after power-on/wake. Green tint on first captures → raise |
| `SNAP_TIMEOUT_MS` | 200 | FIX | Max wait for warmup+capture |
| `CAM_EXPOSURE_MODE` | 1 (MANUAL) | TUNE | 0 = ISP auto (AEC), 1 = manual. Manual is required for repeatable insect-freeze shots |
| `CAM_EXPOSURE_VALUE` | 15000 µs | TUNE | Shutter. **Lower = less motion blur, more noise** (compensate with LED brightness/gain). Raise = brighter, more blur |
| `CAM_GAIN_VALUE` | 12000 mdB | TUNE | 12 dB. Quantized in 300 mdB steps; analog range ends at 30000. Raise only after LED brightness is maxed |
| `CAM_BRIGHTNESS` / `CAM_CONTRAST` | 0 / 0 | DEBUG | ISP adjustments, leave at 0 unless bench-proven |
| `CAM_ANTI_FLICKER` | 0 | DEBUG | 1 = 50 Hz, 2 = 60 Hz — only for AC-lit banding |
| `CAM_SENSOR_REG_DEBUG` | 1 | DEBUG | Physical I2C register readback after each burst (via arbiter, pipe stopped). Set 0 for final timing measurements |

## 3. Mode-4 capture burst

| Parameter | Current | Type | Meaning / tuning |
| :--- | :--- | :--- | :--- |
| `CALLBACK_WARMUP_FRAMES` | 8 | TUNE | Frames discarded after wake (ISR-callback counted). Green/garbage first frames → raise; each frame costs 33 ms of insect travel |
| `CALLBACK_FRAMES` | 4 | TUNE | Frames per detection. More = higher chance of one sharp frame, but +33 ms each and +2.5 MB SD each. PSRAM: 4×2.5 = 10 MB of the 16 MB |
| `CALLBACK_WAKE_TIMEOUT_MS` | 1000 | FIX | Max wait for wake+warmup+burst |
| `BATCH_FRAMES` | 4 (auto) | — | Equals `CALLBACK_FRAMES` in mode 4 |

## 4. SD storage

| Parameter | Current | Type | Meaning / tuning |
| :--- | :--- | :--- | :--- |
| `SD_SNAP_BASE_BLOCK` | 3072 | FIX | Logical start of the image area. Journal slots A/B sit at 3070/3071; the journal keeps the real append position across reboots (doc 08 §5.3) |
| `SD_BATCH_WRITE_BLOCKS` | 1024 (512 KB) | TUNE | Blocks per `HAL_SD_WriteBlocks` transfer (staged through PSRAM). Lower (64–256) if your card errors; higher = fewer HAL calls on good cards |
| `SD_BATCH_RECOVERY_GAP_MS` | 0 | TUNE | Fixed gap between batch transfers, after the CMD13 wait. `[SD] FAIL ... STA=0x5000` (data CRC) on a slow card → 15, then 30 |
| `SD_MAX_SNAPSHOTS` | 15000 | FIX | Safety constant (≈ 32 GB card at half resolution); the journal's "Card full" check is authoritative |
| `SD_WRITE_TIMEOUT_MS` / `SD_READ_TIMEOUT_MS` | 5000 | FIX | HAL transfer timeouts |
| `SD_CARD_READY_TIMEOUT_MS` | 5000 | FIX | CMD13 card-ready wait per batch |

## 5. Illumination (WS2812)

| Parameter | Current | Type | Meaning / tuning |
| :--- | :--- | :--- | :--- |
| `WS2812_MODE` | 1 | FIX | 0 = light on for whole capture (TurnOn), **1 = flash tied to capture (production)**, 2 = indicator flash only |
| `WS2812_ILLUMINATION_BRIGHTNESS` | 100 | TUNE | 0–100 %. RGB scaling, always in sync with exposure. Dark images → raise this before raising gain |
| `WS2812_ILLUMINATION_COLOR` | 0xC8B080 | TUNE | 0xRRGGBB warm white. 0xFFFFFF = maximum light on the sensor (less gain needed); warm white reduces the visible blue flash. Driver converts to the GRB wire order once |
| `WS2812_INDICATOR_MS` | 0 | FIX | Post-capture confirmation flash (mode 2) |
| `WS2812_ILLUMINATION_MS` | 500 | — | Legacy blocking-illumination duration (mode 0 path); mode 1 uses FlashStart/FlashStop around the real pipeline |
| `WS2812_NUM_LEDS` | 12 (ws2812.h) | FIX | Must match the physical strip |

## 6. ToF sensor core (applies to all detector modes)

| Parameter | Current | Type | Meaning / tuning |
| :--- | :--- | :--- | :--- |
| `VL53L5CX_DET_RESOLUTION` | 4 | FIX | 4 = 4×4 grid (16 zones) — the validated production grid; 8 = 8×8 (64 zones, noisier, needs its own thresholds) |
| `VL53L5CX_RANGING_MODE` | 3 (AUTONOMOUS) | FIX | 1 = CONTINUOUS (max integration, slow rate); 3 = autonomous 15 Hz |
| `VL53L5CX_DET_INTEGRATION_MS` | 30 | TUNE | 2–1000 ms. Longer = better weak-signal range, but it changes noise and drift behavior — re-validate on an empty trap after any change |
| `VL53L5CX_DET_RANGING_FREQ_HZ` | 15 | FIX | ~67 ms frame period; every frame-based timer in the detector scales with it |
| `VL53L5CX_DET_TARGET_ORDER` | 1 (CLOSEST) | FIX | 2 = STRONGEST. Must stay CLOSEST: a floor insect is closer than the floor it stands on |
| `VL53L5CX_DET_BASELINE_SAMPLES` | 20 | TUNE | Learning samples per zone; more = more stable baseline, longer boot learn |
| `VL53L5CX_DET_BASELINE_SETTLE_FRAMES` | 15 | FIX | Warm-up frames discarded before learning |
| `VL53L5CX_DET_MIN_SIGNAL` | 500 kcps | TUNE | Zones below this are blind (excluded from learning + detection). Raise to silence noisy zones; never above the weakest real floor zone's signal (check ZFRAME/ALLPARAM) |
| `VL53L5CX_DET_THRESHOLD_PCT` | 3 | TUNE | Legacy level threshold (signal drop %) |
| `VL53L5CX_DET_MOTION_THRESH` | 35 | TUNE | Legacy motion threshold (plugin output) |
| `VL53L5CX_DET_MIN_AFFECTED_ZONES` | 1 | FIX | Legacy multi-zone rule |
| `VL53L5CX_DET_MOTION_MIN_ZONES` | 1 | FIX | ST motion plugin: zones required for its GLOBAL flag (datalog column only, not the trigger path) |
| `VL53L5CX_DET_MOTION_PERSIST_FRAMES` | 10 | TUNE | Plugin temporal accumulation: higher = smoother, slower motion values |
| `VL53L5CX_DET_MOTION_EXTRA_NOISE` | 0 | TUNE | Plugin noise margin: higher = less sensitive |

## 7. ToF local detector (evidence thresholds — decide "what is an insect")

The parameters below guard the weak floor zones (8–11). Full math: doc 08 §1.

| Parameter | Current | Type | Meaning / tuning |
| :--- | :--- | :--- | :--- |
| `VL53L5CX_DET_LOCAL_DIST_STRONG_MM` | 5 | TUNE | Immediate-trigger distance toward sensor. Raise = fewer false triggers, risk missing tiny insects |
| `VL53L5CX_DET_LOCAL_DIST_WEAK_MM` | 3 | TUNE | Weak-track distance entry level |
| `VL53L5CX_DET_LOCAL_SIGNAL_WEAK_PCT` | 2 | TUNE | Signal-drop entry for weak classes |
| `VL53L5CX_DET_WEAK_TRACK_ENABLED` | 0 | FIX | Cross-zone track, 5 s window, 2–3 zones. **Disabled: tripped on 2 % noise in field logs** |
| `VL53L5CX_DET_FAST_EDGE_SIGNAL_PCT` | 2 | TUNE | Fast-edge entry signal (2-frame confirmation) |
| `VL53L5CX_DET_FAST_EDGE_DISTANCE_MM` | 4 | TUNE | Fast-edge distance |
| `VL53L5CX_DET_FAST_BASELINE_SIGNAL_PCT` / `_DISTANCE_MM` | 2 / 3 | TUNE | Fast-edge baseline comparison levels |
| `VL53L5CX_DET_WEAK_MOTION_SIGNAL_PCT` / `_DISTANCE_MM` / `_MEMORY_FRAMES` | 1 / 1 / 3 | TUNE | Per-zone motion requirement inside a track (signal % / mm / look-back frames) |
| `VL53L5CX_DET_FLOOR_PROTRUSION_MIN_MM` | 1 | TUNE | Minimum floor-zone distance toward sensor for floor sustain/score |
| `VL53L5CX_DET_FLOOR_HOLD_FRAMES` | 4 | TUNE | Floor-sustain persistence at 15 Hz (≈ 270 ms) |
| `VL53L5CX_DET_FLOOR_SIGNAL_SCORE_HIT` | 4 | TUNE | Score per qualifying floor frame |
| `VL53L5CX_DET_FLOOR_SIGNAL_SCORE_TRIGGER` | 12 | TUNE | = 3 qualifying frames (≈ 200 ms). **Main knob against empty-floor false triggers (try 16/20)** |
| `VL53L5CX_DET_FLOOR_DEPTH_BAND_MM` | 30 | TUNE | Depth band around the farthest valid zone that counts as "floor" |
| `VL53L5CX_DET_MICRO_PERSIST_ENABLED` | 1 | TUNE | Slow same-zone accumulator for settled tiny insects |
| `VL53L5CX_DET_MICRO_SIGNAL_PCT` | 1 | TUNE | Micro evidence: signal drop ≥ 1 %… |
| `VL53L5CX_DET_MICRO_DISTANCE_MM` | 2 | TUNE | …or distance ≥ 2 mm |
| `VL53L5CX_DET_MICRO_HIT` | 2 | TUNE | Score gain per evidence frame |
| `VL53L5CX_DET_MICRO_DECAY` | 1 | TUNE | Score loss per quiet frame (2 = stricter) |
| `VL53L5CX_DET_MICRO_TRIGGER` | 32 | TUNE | Fire level: settled insect ≈ 1.2–1.3 s to fire; random 20 % noise never reaches it. 24 = faster, 48 = more patient |
| `VL53L5CX_DET_REARM_INTERVAL_MS` | 0 (OFF) | FIX | Debounced rearm between captures in the same zone. 0 = disabled (a settled insect must not be re-baselined while present). If re-enabled: ≥ 20000 |

Recentering internals (code constants in `vl53l5cx_detection.c`, doc 08 §2):
12 s stable window, 8 stable frames (≤ 1 % and ≤ 1 mm per frame) recenter a
single zone — tuned with the detector, not in `app_config.h`.

## 8. Baseline refresh policy (doc 08 §2)

The config header says "pick ONE mode", but the current field build runs
**periodic (primary) + activation refresh (burst backstop)** together:

| Parameter | Current | Type | Meaning / tuning |
| :--- | :--- | :--- | :--- |
| `VL53L5CX_DET_PERIODIC_RESTART_ENABLED` | 1 | TUNE | Full re-learn inside `VL53L5CX_Update()`, quiet-gated (never while an insect is present) |
| `VL53L5CX_DET_PERIODIC_RESTART_INTERVAL` | 1000 frames (~66 s) | TUNE | 500 = faster re-baselining of a drifting zone (≈ 33 s, more blind time); 2000–3000 = less blind time, slower correction of recurring static-bias episodes |
| `VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED` | 1 | TUNE | Count completed camera activations and force a refresh after the max |
| `VL53L5CX_DET_REFRESH_WINDOW_SECS` | 5 | FIX | Generic activation window (other detector modes) |
| `VL53L5CX_DET_MAX_DETECTIONS` | 2 | FIX | Generic activation max (other modes) |
| `VL53L5CX_DET_HIGH_SENS_REFRESH_WINDOW_SECS` | 30 | TUNE | High-sensitivity override, **active in this build**: window in which activations are counted |
| `VL53L5CX_DET_HIGH_SENS_MAX_DETECTIONS` | 3 | TUNE | Activations in the window that force a full refresh. Bounds any false-trigger loop to 3 photos |
| `VL53L5CX_DET_HIGH_SENS_REARM_HOLDOFF_SECS` | 3 | TUNE | After an activation refresh: weak tracks discarded, only level/fast events accepted for N s (ToF stays active) |

## 9. Dual sensor (currently disabled — `VL53L5CX_DUAL_SENSOR 0`)

| Parameter | Current | Meaning |
| :--- | :--- | :--- |
| `DUAL_WAKE_DURATION_MS` | 5000 | How long the primary ToF stays awake after a guardian wake |
| `DUAL_BASELINE_MODE` | 1 (QUICK_WAKE) | 0 = no refresh, 1 = re-learn on wake, 2 = re-learn before sleep |
| `VL53L5CX_EXT_THRESHOLD_PCT` | 6 | Guardian signal-drop threshold |
| `VL53L5CX_EXT_MOTION_THRESH` | 60 | Guardian motion threshold |
| `VL53L5CX_EXT_MIN_AFFECTED_ZONES` | 1 | Guardian zone rule |
| `VL53L5CX_EXT_MIN_SIGNAL` | 500 | Guardian blind-zone cutoff (kcps) |
| `VL53L5CX_EXT_MOTION_CONFIRM_FRAMES` | 2 | Motion confirmation frames |
| `VL53L5CX_EXT_BASELINE_SAMPLES` | 30 | Guardian baseline samples |
| `VL53L5CX_EXT_RANGING_FREQ_HZ` / `_INTEGRATION_MS` / `_RANGING_MODE` | 15 / 30 / 1 (CONTINUOUS) | Guardian timing (continuous for max integration while it is the always-on sensor) |
| `VL53L5CX_EXT_MOTION_*` (1 / 16 / 0) | — | Guardian motion plugin settings (min zones / persist frames / extra noise) |

## 10. Debug & logging (all should be 0 in a long-unattended run except noted)

| Parameter | Current | Meaning |
| :--- | :--- | :--- |
| `PERF_DEBUG_LEVEL` | 0 | 0 = quiet, 1 = standard (`[CAM]`, `[SD]`, `[LIGHT]`, per-phase timings), 2 = verbose, 3 = all |
| `PERF_SD_BATCH_PRINT_EVERY` | 0 | With `PERF_DEBUG_LEVEL ≥ 2`: print an SD progress line every N batches (0 = off) |
| `VL53L5CX_DET_EVENT_TRACE` | **1 (keep)** | One compact `TOFEVT` line per accepted capture, independent of `PERF_DEBUG_LEVEL` |
| `VL53L5CX_DET_CAL_TRACE` | 0 | Per-zone residual trace (`[CAL z=n sig=.. drop=.. dist=.. motion=..]`) — 1 **only** during a calibration session; 100 ms interval |
| `VL53L5CX_DET_CAL_TRACE_INTERVAL_MS` | 100 | CAL trace rate limit |
| `VL53L5CX_DET_DEBUG_ZFRAME` / `_INT` | 0 | Per-frame 16-zone compact log for `zone_monitor.py` (INT = ms rate limit) |
| `VL53L5CX_DET_DEBUG_ALLPARAM` / `_INT` | 0 | Full per-zone parameter frames for `datalogger.py` (INT 5 ms = ~200 frames/s) |
| `VL53L5CX_DET_ZONE_LOG_INTERVAL_MS` | 2000 | Zone-snapshot interval in survey builds (`VL53L5CX_DET_ZONE_SURVEY 1`) |
| `RTC_PRINT_TIME` | 1 | Print UTC date/time on boot |
| `RTC_SET_ON_BOOT` | 0 | 1 = force the RTC to `RTC_SET_UTC` at every boot (provisioning only, then back to 0) |
| `RTC_SET_UTC` | "2026-01-01T00:00:00Z" | Value used when `RTC_SET_ON_BOOT 1` |

## 11. Symptom → first parameter to touch

| Symptom (from logs / photos) | First knob | Direction |
| :--- | :--- | :--- |
| Photo of empty trap (false trigger) | `VL53L5CX_DET_MICRO_TRIGGER`, `VL53L5CX_DET_FLOOR_SIGNAL_SCORE_TRIGGER` | raise (48 / 16–20); verify with `CAL_TRACE 1` which zone fires |
| Missed small settled insect | `VL53L5CX_DET_MICRO_TRIGGER`, `..._FLOOR_SIGNAL_SCORE_TRIGGER` | lower (24 / 8–10) |
| Motion blur on insect | `CAM_EXPOSURE_VALUE` | lower (8–10 ms), compensate with `WS2812_ILLUMINATION_BRIGHTNESS` / `CAM_GAIN_VALUE` |
| Dark / noisy image | `WS2812_ILLUMINATION_BRIGHTNESS`, then `CAM_EXPOSURE_VALUE`, then `CAM_GAIN_VALUE` | raise, in that order |
| Green tint on first captured frame | `CALLBACK_WARMUP_FRAMES` (and `SNAP_WARMUP_FRAMES` for mode 0) | raise by 2–4 |
| SD `[FAIL] STA=0x5000` (CRC) | `SD_BATCH_RECOVERY_GAP_MS` | 0 → 15 → 30; also lower `SD_BATCH_WRITE_BLOCKS` |
| Recurring 1–2 min static-bias photo episodes | `VL53L5CX_DET_PERIODIC_RESTART_INTERVAL` | lower (500) |
| Too much ToF blind time | `VL53L5CX_DET_PERIODIC_RESTART_INTERVAL` | raise (2000–3000) |
| CMW camera init `ret=-7` at boot | — | keep camera pre-init inside the main thread (do not move it into `camera_task`); check the I2C arbiter |
| Light stays on after capture | — | driver bug path: OFF watchdog covers it; check `WS2812` DMA log lines |

