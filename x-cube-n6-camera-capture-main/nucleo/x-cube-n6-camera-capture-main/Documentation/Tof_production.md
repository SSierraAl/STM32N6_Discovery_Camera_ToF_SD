# 4Ã—4 ToF detection and field calibration

## Scope and version

This guide describes the **single sensor, 4Ã—4, high sensitivity camera** path in
`extratof_tuning`, based on `c11aaa3` (45 ms integration), followed by
`KAN-36-periodic-baseline-recovery.patch` and the configuration patch that adds
this guide. Parameters are in
`x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Inc/app_config.h`.
Detection is implemented in `Src/vl53l5cx_detection.c`; photo and adaptive
refresh control are in `Src/app_thread.c`. No parameter below changes LED color.

The code also supports test mode, legacy camera mode, 8Ã—8 and dual sensor mode.
The local residual detector and the new millisecond periodic timer are selected
only when `VL53L5CX_DUAL_SENSOR=0`, `VL53L5CX_DET_RESOLUTION=4`,
`VL53L5CX_DET_HIGH_SENS_CAMERA=1` and `TEST_TOF_MODE=0`. The related local
detector settings also affect the equivalent 4Ã—4 `HIGH_SENS_TEST` path.
Other modes retain their original frame counter for periodic refresh.

## Operating sequence

1. At startup, the sensor starts ranging, discards 15 settling frames, then
   accumulates 20 baseline frames. A zone needs at least half the collection
   frames valid. `BASELINE` prints mean signal and distance for each zone.
2. Every fresh frame yields a signed distance and signal change from its zone
   baseline. The **median across valid zones** is subtracted from each zone:
   a coherent movement affecting most zones tends to cancel, while a local
   change stays visible. A zone must have valid status and enough signal.
3. A local signal change of at least 3%, or a local distance change of at
   least 5 mm, creates immediate level evidence. Two frame fast edges can
   detect smaller rapid changes. Floor signal, protrusion and same-zone micro
   scores can detect slower weaker changes. Cross-zone weak tracking is
   disabled in the present configuration (`VL53L5CX_DET_WEAK_TRACK_ENABLED=0`).
   The ST motion plugin is computed but **does not directly trigger** this
   4Ã—4 high sensitivity camera detector.
4. An event is gated by per-zone latches, capture cooldown and a brief
   post-refresh recovery policy. `TOFEVT` records an accepted event, then
   the camera requests a snapshot and SD storage. A zone can escalate once
   from a weak photo to independently strong evidence; its latch otherwise
   prevents repeated photos while the same evidence persists.
5. After every capture, frame-edge history is cleared but latches persist.
   A rolling 30 s window counts all camera activations. On the third capture,
   the full adaptive baseline is learned. The camera pipeline time occurs
   *before* the window opens for the next capture.
6. A separate periodic full baseline runs about 67 s **after completion of
   the most recent full baseline** by default. It is scheduled by elapsed
   time, not by number of new frames. It also runs when a zone is latched or
   blocked, then resets its own timer. A full manual, scene, startup or
   three-photo baseline also resets that timer. A per-zone recenter does not.
7. Quiet zones gradually follow slow signal drift. A persistent stationary
   zone can be recentered independently after a stable 12 s plateau. A
   coherent shift in at least 12 zones for 5 s can request a full baseline.

### Three different clocks

| Mechanism | Present value | Starts/restarts | What happens |
| --- | ---: | --- | --- |
| Full periodic baseline | 66,666 ms (about 67 s) | End of **any full** baseline | Full stop, restart, settle and learn, regardless of latches |
| Photo-count baseline | 3 captures within rolling 30 s | Window after each completed capture | Full learn on capture 3; window expires after 30 s without another photo |
| Per-zone stationary recenter | 12,000 ms plus 8 stable frames | Start of local stable evidence | Only that zone's baseline may move; full timer stays unchanged |
| Coherent scene refresh | 5,000 ms | Continuous global scene shift | Requests a full baseline if capture policy permits |

These are independent. The periodic interval restarts after a **full** learn;
an insect that is stationary for hours may become part of a learned baseline.
Its later movement can still create new evidence, but detection is not guaranteed
if the target produces less change than noise. A full learn during vibration
can also learn the vibrating scene. The next scheduled learn can recover after
the vibration stops; this is why testing the interval in the actual trap matters.
The periodic learn does **not** reset the separate photo-count window: that
window expires after its own quiet time or resets at its own adaptive/scene
refresh.

## The fastest controls: baseline cadence and capture behavior

All names below are `#define` settings in `Inc/app_config.h`. Keep the suffix
units (`_MS`, `_SECS`, `_FRAMES`, `_PCT`, `_MM`) straight.

| Setting | Current | Increase it | Decrease it |
| --- | ---: | --- | --- |
| `VL53L5CX_DET_PERIODIC_RESTART_ENABLED` | 1 | Binary switch: 1 enables periodic refresh | 0 disables it, including the new camera timer |
| `VL53L5CX_DET_PERIODIC_CAMERA_INTERVAL_MS` | derived: 66,666 ms | Full learns less often; fewer interruptions, slower recovery from a contaminated baseline | Full learns more often; more interruptions and more risk of learning a present object |
| `VL53L5CX_DET_PERIODIC_RESTART_INTERVAL` | 1000 frames | Longer period in *other modes* and affects the default camera expression | Shorter period; use the camera interval explicitly for camera mode |
| `VL53L5CX_DET_BASELINE_SETTLE_FRAMES` | 15 frames | More sensor settling before averaging; longer pause | Faster refresh, greater warm-up offset risk |
| `VL53L5CX_DET_BASELINE_SAMPLES` (4Ã—4) | 20 frames | Averages more readings, longer pause, may learn a moving object | Faster baseline, less averaging |
| `VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED` | 1 | Binary switch: 1 enables photo-count refresh | 0 stops three-photo refresh; periodic can still run |
| `VL53L5CX_DET_HIGH_SENS_MAX_DETECTIONS` | 3 photos | Allows more photos before full baseline | Learns sooner; may erase an object before it moves |
| `VL53L5CX_DET_HIGH_SENS_REFRESH_WINDOW_SECS` | 30 s | Distant photos count toward one group | Requires photos closer together to reach the limit |
| `VL53L5CX_DET_HIGH_SENS_REARM_HOLDOFF_SECS` | 3 s | Withholds weak tracks longer after a full baseline | Reopens weak tracks earlier; more noise risk |
| `VL53L5CX_DET_HIGH_SENS_CAPTURE_COOLDOWN_FRAMES` | 5 frames | Waits more fresh ToF frames after camera/SD or stable-scene baseline before another photo; latch state still updates | Allows photos sooner; an unstable restarted sensor can retrigger |

To choose **5 minutes**, change only the camera interval:

```c
#define VL53L5CX_DET_PERIODIC_CAMERA_INTERVAL_MS (5UL * 60UL * 1000UL)
```

For 10 minutes use `10UL * 60UL * 1000UL`. Current default deliberately
remains ~67 s. The configuration is compiled into firmware, not changed at
runtime. Changing frequency from 15 Hz also changes the *default expression*;
an explicit minute expression is independent of frequency. Allow roughly
35 ranging frames for a full learn (15 settling + 20 collection), **at least**
about 2.3 s at 15 Hz, plus the 50 ms restart wait, 200 ms warm-up wait,
IÂ²C and console time. Missed frames/timeouts can extend this pause.

## Sensitivity controls

These controls determine whether a small, slow or fast object is seen. A lower
threshold generally increases both sensitivity and false positives. Change
**one variable at a time** and compare against both empty-box vibration logs
and an insect in the same physical zones.

| Setting | Current | Role / field trade-off |
| --- | ---: | --- |
| `VL53L5CX_DET_INTEGRATION_MS` (4Ã—4) | 45 ms | Longer sensing window can improve signal; must fit chosen ranging period. Keep 45 ms while calibrating detection. |
| `VL53L5CX_DET_RANGING_FREQ_HZ` | 15 Hz | Frame period ~67 ms; changing it changes real duration of all frame counters and the default periodic interval. |
| `VL53L5CX_DET_MIN_SIGNAL` | 500 | Zones below this signal are excluded from baseline and local detection. Lowering can admit unstable zones. |
| `VL53L5CX_DET_TARGET_ORDER` | 1 (closest) | A small floor object may be closer than the floor; 2 chooses strongest and may favor the floor. |
| `VL53L5CX_DET_THRESHOLD_PCT` (4Ã—4) | 3% | Immediate local signal level. Lower catches weak objects, risks one-frame noise. |
| `VL53L5CX_DET_LOCAL_DIST_STRONG_MM` | 5 mm | Immediate local distance level. Lower can admit vibration residuals. |
| `VL53L5CX_DET_LOCAL_DIST_WEAK_MM` | 3 mm | Weak distance candidate; needs additional support. |
| `VL53L5CX_DET_LOCAL_SIGNAL_WEAK_PCT` | 2% | Weak local signal candidate, floor score and support for distance paths. At 1%, empty-box logs have shown noise. |
| `VL53L5CX_DET_FAST_EDGE_SIGNAL_PCT` | 2% | Fresh frame-to-frame signal edge; requires local baseline support and following-frame confirmation. |
| `VL53L5CX_DET_FAST_EDGE_DISTANCE_MM` | 4 mm | Fresh distance edge, with baseline and signal support. |
| `VL53L5CX_DET_FAST_BASELINE_SIGNAL_PCT` / `...DISTANCE_MM` | 2% / 3 mm | Local reference support for fast paths. |
| `VL53L5CX_DET_FLOOR_DEPTH_BAND_MM` | 30 mm | Zones near the farthest valid baseline distance are treated as floor. Decrease if wall zones interfere; inspect `BASELINE`. |
| `VL53L5CX_DET_FLOOR_PROTRUSION_MIN_MM` | 1 mm | Requires a floor zone to move toward sensor, with weak signal support. |
| `VL53L5CX_DET_FLOOR_HOLD_FRAMES` | 4 frames | Consecutive protrusion evidence; at 15 Hz roughly 267 ms of frames. |
| `VL53L5CX_DET_FLOOR_SIGNAL_SCORE_HIT` / `...TRIGGER` | 4 / 12 | Floor 2% score accumulates toward capture; lower trigger gives faster response and more false photos. |
| `VL53L5CX_DET_MICRO_PERSIST_ENABLED` | 1 | Same-zone weak floor persistence path. Disable only to isolate it during a controlled field trial. |
| `VL53L5CX_DET_MICRO_SIGNAL_PCT` / `...DISTANCE_MM` | 1% / 2 mm | Per-frame weak evidence thresholds; evaluate against empty-box trace. |
| `VL53L5CX_DET_MICRO_HIT` / `...DECAY` / `...TRIGGER` | 2 / 1 / 32 | Same-zone persistence score. Reducing trigger accelerates both tiny detections and sustained noise triggers. |
| `VL53L5CX_DET_WEAK_TRACK_ENABLED` | 0 | Cross-zone weak path disabled after false tracks; enabling changes detection behavior substantially. |
| `VL53L5CX_DET_LOCAL_TRACK_WINDOW_MS` / `...MIN_ZONES` / `...MAX_ZONES` | 5000 ms / 2 / 3 | Relevant only when cross-zone weak tracking is enabled. |
| `VL53L5CX_DET_MOTION_THRESH` | 35 | ST motion indicator threshold used by drift/stability logic in this 4Ã—4 path; it is **not** a direct photo trigger. |

## Baseline protection and drift controls

The following values were previously inside the detector source; they are now
adjustable beside the other settings. **Their defaults have not changed.**

| Setting | Current | If increased / important condition |
| --- | ---: | --- |
| `VL53L5CX_DET_ZONE_CLEAR_FRAMES` | 3 frames | Latch needs longer raw-free run before another photo; must remain <=255. |
| `VL53L5CX_DET_ZONE_QUIET_FRAMES` | 15 frames | Quiet-zone signal tracks drift later; must remain <=255. |
| `VL53L5CX_DET_ZONE_QUIET_SIGNAL_PCT` | 4% | Allows adaptation with larger signal deviation if no raw candidate: can absorb real changes. |
| `VL53L5CX_DET_ZONE_QUIET_DISTANCE_MM` | 3 mm | Allows quiet signal adaptation across a larger distance deviation. |
| `VL53L5CX_DET_ZONE_DRIFT_DIVISOR` | 32 | Larger divisor moves signal baseline more slowly each qualifying frame; cannot be zero. Integer division can produce zero step for very small offsets. |
| `VL53L5CX_DET_STABLE_PLATEAU_MS` | 12,000 ms | Waits longer before recentering a stationary raw candidate, including a settled insect; can use minutes, e.g. `2UL * 60UL * 1000UL`. |
| `VL53L5CX_DET_STABLE_MIN_FRAMES` | 8 frames | Demands more stable readings inside window; must be <= window cap. |
| `VL53L5CX_DET_STABLE_WINDOW_FRAMES` | 180 frames | Caps stable frame counter; cannot exceed 255 without changing its C storage type. Despite its name it is a **count cap**, not an elapsed-time window. |
| `VL53L5CX_DET_STABLE_MAX_SIGNAL_JITTER_PCT` / `...DISTANCE_JITTER_MM` | 1% / 1 mm | More jitter accepted as a stationary plateau; can recenter noise or moving target. |
| `VL53L5CX_DET_SCENE_SETTLE_MS` | 5,000 ms | Coherent displacement must persist longer before requesting full learn; minutes possible but can leave shifted baselines active longer. |
| `VL53L5CX_DET_SCENE_MIN_ZONES` | 12 | Needs at least 12 same-direction zones; if fewer than 12 zones are valid, this automatic scene mechanism cannot fire. |
| `VL53L5CX_DET_SCENE_DISTANCE_MM` / `...SIGNAL_PCT` | 4 mm / 7% | Larger coherent shift needed; lower values can treat vibration as an environment change. |
| `VL53L5CX_DET_REARM_INTERVAL_MS` | 0 (off) | Enabling repeat photos from a continuously latched zone can restore cascades of false triggers. |

The quiet-zone drift adjustment changes **signal** gradually; it does not
automatically correct that zone's floor distance. The plateau recenter can
update signal and/or distance and release its latch. The global scene request
requires the scene condition to persist and no qualifying motion flag; the
sensor task only executes it when cooldown is zero and no new event is being
captured. A manual PC13 refresh initiates a full baseline independently.

## Reading field logs

| Output | How to read it |
| --- | --- |
| `[BASELINE] ... Valid zones` and `BASELINE,...` | Initial or refreshed per-zone reference. A missing floor zone remains blind to local detection until a valid baseline is learned. |
| `[ADAPT] Camera activation 1/3`, `2/3`, `3/3` | Count of photo requests within the configured window. |
| `[ADAPT] Maximum activations reached` | Full baseline from the photo count. |
| `[ToF] Periodic refresh since last baseline...` and `done` | Full timed refresh; next deadline is measured from completion. |
| `[ADAPT] Zone ... baseline` | Only that zone was recentered after a stable plateau. |
| `[ADAPT] Persistent stable drift` | Full baseline requested by coherent scene shift. |
| `TOFEVT` | Accepted detector event; `src=1` signal, `src=4` distance, `src=5` both. `strong`, `raw`, `latched`, `micro`, `track` are hexadecimal zone masks, with bit `1<<z`. The displayed `z:ld_mm:ls_pct` is the magnitude of **local** residual. |
| `TOFMISS` | A candidate was policy-blocked; it is not a photograph. |
| `TOFCAL` | Signed per-zone local residuals and frame changes; enable `VL53L5CX_DET_CAL_TRACE=1` temporarily when calibrating. Negative `sd_mm` means closer to sensor. UART output can affect timing. |
| `NOISEMETRIC` | Optional common-mode / MAD diagnostic, disabled in the current high sensitivity camera build. If enabled, it is observation only and does not veto a photo. |

The `class` printed with a camera activation is a **bit mask**: level `1`,
fast edge `2`, weak track `4`, escalation of an already latched scene `8`.
For example, `class=9` means level plus latched escalation. It does not change
the three-photo count: every camera activation counts.

## A controlled field procedure

1. Keep a copy of the known working commit, and confirm `c11aaa3` plus the
   periodic patch are installed. Confirm the ToF configuration reports
   45 ms integration and 15 Hz ranging.
2. With an empty, stationary trap, record `BASELINE`, 10 minutes of `TOFEVT`
   and periodic refresh timestamps. Then repeat during representative
   vibration. Note false photos per hour and valid baseline zones.
3. Run a small slow target and a large fast target through **each relevant
   physical zone**. Record zone, passage time, detection time and whether a
   photo was stored. Do not infer detection from LED alone.
4. First vary only `VL53L5CX_DET_PERIODIC_CAMERA_INTERVAL_MS` (e.g. 2, 5,
   then 10 minutes) if frequent full learns may absorb an insect or interrupt
   capture. Compare both vibration recovery and detection after the target
   remains stationary. Longer periods also delay recovery from a bad learn.
5. If small stationary targets vanish, compare `TOFCAL` empty-box and target
   residuals before changing `...MICRO_TRIGGER`, `...LOCAL_SIGNAL_WEAK_PCT`
   or plateau timing. Raising `...STABLE_PLATEAU_MS` delays learning that
   stationary target, at the cost of keeping persistent noise active longer.
6. Keep the best setting and repeat both empty-box and target trials. Collect
   logs after the periodic refresh as well as immediately after startup.

**Practical limit:** this sensor cannot guarantee discrimination when an
insect's per-zone change matches vibration and measurement noise. Extending
timers does not create more signal; only field measurements can establish
whether the smallest target is resolvable in this geometry.

## Applying the two patches on Windows

If your branch is at `c11aaa3`, apply the periodic patch **first**, then the
configuration/guide patch; both are email patches for `git am`. If the first
patch is already present, apply only the second. Check your working tree is
clean before starting.

```powershell
git status --short
git log -3 --oneline
git am "$HOME\Downloads\KAN-36-periodic-baseline-recovery.patch"
git am "$HOME\Downloads\KAN-36-configurable-tof-baselines.patch"
git push origin extratof_tuning
```

If `git am` says a previous operation is in progress, inspect `git status`
before proceeding. `git am --abort` cancels that incomplete operation; do not
delete Git's rebase directory by hand. The attached rollback patch targets an
older state and is not part of this two-patch sequence.