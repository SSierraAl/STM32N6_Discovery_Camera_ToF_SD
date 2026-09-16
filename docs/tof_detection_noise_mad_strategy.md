# ToF Insect Detection Strategy: Global Noise Rejection with Median/MAD

## Purpose

This document defines a detection architecture for the VL53L5CX insect passage sensor that preserves maximum sensitivity to very small insects while reducing false positives caused by mechanical vibration, wind, trap movement, sensor noise, and long-term drift.

The design goal is deliberately asymmetric: **missing a real insect is more costly than accepting some false positives**. Noise rejection must therefore operate as a conservative second-stage classifier and must not replace or desensitize the existing raw detector.

## Current constraints

The current production detector uses a 4x4 ToF grid and intentionally sensitive thresholds. Any zone may trigger. A single affected zone is sufficient for a raw detection. The detector uses both signal change relative to a learned per-zone baseline and the ST motion-indicator output.

This sensitivity is necessary because small insects may only perturb one zone and their signal can be close to the sensor noise floor. Increasing the minimum number of affected zones, increasing the signal threshold, or requiring multiple consecutive frames could reduce false positives but would also increase the probability of missing small or fast insects.

The system also has two distinct disturbances that must not be confused:

1. **Fast common motion / vibration**: tens to hundreds of milliseconds. Often affects many zones coherently.
2. **Slow sensor/environment drift**: seconds to hours. Must remain handled by adaptive baseline logic.

The proposed architecture therefore separates raw detection, fast common-mode estimation, and slow baseline adaptation.

## High-level architecture

```text
                     4x4 ToF frame
                      16 zones
                         |
            +------------+-------------+
            |            |             |
            v            v             v
        RAW insect   GLOBAL NOISE    BASELINE
         detector     estimator       tracker
            |            |             |
            |            v             |
            |       vibration /        |
            |       common motion      |
            |            |             |
            +------------+-------------+
                         |
                         v
                  DECISION LAYER
                         |
                +--------+--------+
                |                 |
                v                 v
            TAKE PHOTO         DISCARD
```

The first implementation is **observation-only**. It calculates and prints the noise metrics but does not discard any detection. This allows thresholds to be derived from real trap data before enabling a veto.

## 1. Preserve the existing raw detector

The current sensitive detector remains the first stage:

```text
signal_triggered = signal_drop > current signal threshold
motion_triggered = motion_value >= current motion threshold
raw_candidate    = signal_triggered OR motion_triggered
```

The current `MIN_AFFECTED_ZONES = 1` behavior is preserved.

All 16 zones remain eligible to trigger. The previous center-row-only experiment remains commented and is not part of this strategy.

## 2. Signed per-zone changes

For every valid zone `z`, compute signed changes in addition to the existing absolute signal-drop metric.

### Distance change

```text
d[z] = current_distance[z] - baseline_distance[z]
```

The sign is preserved because rigid mechanical movement often causes many zones to move in the same direction.

### Signal change

```text
s[z] = 100 * (current_signal[z] - baseline_signal[z]) / baseline_signal[z]
```

This is separate from the existing absolute signal-drop percentage used by the raw detector.

### Motion indicator

```text
m[z] = motion_indicator[z]
```

The ST motion indicator is useful not only for insect detection but also as evidence that a large portion of the scene moved simultaneously.

## 3. Robust common-mode estimate using the median

Because an insect may appear in any of the 16 zones, no fixed row is reserved as a reference region. Instead, the common scene movement is estimated from all valid zones using the median.

```text
Gd = median(d[z])
Gs = median(s[z])
Gm = median(m[z])
```

The median is preferred over the mean because one or a few insect-affected zones are outliers and should not significantly change the global estimate.

Example:

```text
Distance deltas with vibration + insect:

 -5  -6  -5  -6
 -6  -5 -25  -5
 -5  -6  -6  -5
 -6  -5  -6  -5

Gd ~= -5.5 mm
```

The insect zone does not dominate the common-mode estimate.

## 4. Median Absolute Deviation (MAD)

The Median Absolute Deviation measures how dispersed the zones are around the global median.

```text
MADd = median(abs(d[z] - Gd))
MADs = median(abs(s[z] - Gs))
MADm = median(abs(m[z] - Gm))
```

Interpretation:

- low `MAD` + non-zero global shift: coherent whole-system movement is likely;
- high `MAD`: the frame is spatially heterogeneous, so a local object or chaotic noise may be present;
- low global shift + one strong residual: compatible with a localized insect.

MAD provides a dynamic estimate of the current noise distribution instead of assuming a fixed noise level.

## 5. Spatial coherence

A vibration event should often move many valid zones similarly. For diagnostic purposes, define a tolerance around the median:

```text
distance_tolerance = max(min_distance_tolerance, 3 * MADd)
signal_tolerance   = max(min_signal_tolerance,   3 * MADs)
```

Then count the zones close to the global model:

```text
coherent_distance = count(abs(d[z] - Gd) <= distance_tolerance)
coherent_signal   = count(abs(s[z] - Gs) <= signal_tolerance)
```

A high coherent-zone ratio is evidence of common-mode motion.

These tolerance formulas are initially **diagnostic only**. They do not reject frames in the observation commit.

## 6. Local residuals

For each zone, remove the instantaneous global component:

```text
Rd[z] = d[z] - Gd
Rs[z] = s[z] - Gs
```

These residuals answer the key question:

> Does this zone contain a change that cannot be explained by the movement affecting the rest of the sensor?

### Vibration only

```text
Gd = -6 mm
candidate zone d = -7 mm
Rd = -1 mm
```

The candidate closely follows common motion.

### Vibration plus insect

```text
Gd = -6 mm
candidate zone d = -14 mm
Rd = -8 mm
```

A strong local residual survives common-mode subtraction, so the system should remain able to trigger even during vibration.

## 7. Motion coverage

Count how many zones exceed the existing motion threshold:

```text
motion_coverage = count(m[z] >= current motion threshold)
```

Examples:

```text
1/16  -> compatible with a local insect
3/16  -> still compatible with a localized object
13/16 -> strong evidence of whole-system movement
```

Motion coverage is evidence for the future vibration classifier. It is not a hard insect filter by itself.

## 8. Spatial footprint

For the zones that satisfy the existing raw detector, count how many sensor rows and columns are represented.

```text
affected_rows
affected_columns
```

A small insect is normally spatially compact. A rigid movement may activate several rows and columns simultaneously.

Again, this is supporting evidence and should not independently veto a detection.

## 9. Future conservative veto logic

After observation data has been collected, the intended classifier is asymmetric:

```text
RAW candidate
     |
     v
Is global vibration strong?
     / \
   NO   YES
   |     |
   v     v
TRIGGER  Does the candidate have a local residual?
              / \
            YES  NO
             |    |
             v    v
          TRIGGER Is the global pattern highly coherent?
                       / \
                     NO  YES
                     |    |
                     v    v
                  TRIGGER DISCARD
```

A candidate should be discarded only when all of the following are strongly supported:

1. substantial common-mode movement is present;
2. many zones agree with that common motion;
3. the raw candidate does not contain a meaningful local residual.

Ambiguous cases continue to trigger the camera.

## 10. Slow drift remains a separate adaptive process

The fast common-mode estimator must not replace the current adaptive baseline.

The intended time scales are:

```text
insect event       : milliseconds
vibration estimate : every ToF frame
noise statistics   : frames to seconds
baseline drift     : seconds to minutes/hours
```

A future slow common-mode state can be maintained with an exponential moving average:

```text
Gslow(t) = Gslow(t-1) + alpha_slow * (Gd(t) - Gslow(t-1))
```

Then the rapid component is approximately:

```text
vibration(t) = Gd(t) - Gslow(t)
```

The long-term baseline should update only on trusted clean measurements. Adaptation should be frozen for candidate zones and during uncertain frames so that an insect is not learned as background.

The existing hard/adaptive baseline refresh remains a fallback during early development.

## 11. Observation-mode UART metrics

The first implementation prints a compact `NOISEMETRIC` record only when the existing detector already reports an insect candidate.

Planned fields:

```text
NOISEMETRIC,
 temp=<sensor temperature>,
 trig=<existing trigger source>,
 affected=<existing affected-zone count>,
 validD=<zones used for distance model>,
 Gd=<median signed distance delta>,
 MADd=<distance MAD>,
 cohD=<distance-coherent zones>,
 validS=<zones used for signal model>,
 Gs=<median signed signal delta percent>,
 MADs=<signal MAD>,
 cohS=<signal-coherent zones>,
 validM=<motion zones>,
 Gm=<median motion>,
 MADm=<motion MAD>,
 motionCov=<zones above existing motion threshold>,
 rows=<affected rows>,
 cols=<affected columns>,
 maxRd=<largest distance residual among raw candidate zones>,
 maxRs=<largest signal residual among raw candidate zones>
```

No field in this diagnostic record changes the current trigger result.

## 12. Validation plan

Collect labeled events in at least four conditions:

### A. Quiet / no insect

Goal: characterize normal sensor noise and drift.

### B. Vibration / no insect

Create realistic trap movement from wind, handling, attachment vibration, or deliberate mechanical excitation.

Goal: characterize `Gd`, `Gs`, `Gm`, MAD, coherence, motion coverage, rows, and columns during false positives.

### C. Insect / quiet

Use insects spanning the smallest expected size up to larger specimens.

Goal: identify the minimum local residual and spatial footprint of true events.

### D. Insect + vibration

This is the critical validation case.

Goal: verify that common-mode vibration can be present while a local residual remains detectable.

For each captured event, associate the UART `NOISEMETRIC` line with the stored image so the event can later be labeled as insect, vibration, both, or uncertain.

## 13. Criteria before enabling rejection

Do not enable an automatic vibration veto until field data demonstrates a clear region where vibration-only events separate from true insects.

The initial threshold should be deliberately conservative. If an event falls into an overlap/uncertain region, the camera should still trigger.

A useful target is therefore not maximum false-positive removal. The target is:

> Remove only high-confidence mechanical false positives while preserving the current sensitivity to single-zone and weak insect passages.

## 14. Implementation stages

### Stage 1 - Observation only

- Preserve all current thresholds.
- Preserve all current trigger decisions.
- Preserve all 16 active zones.
- Preserve adaptive baseline refresh.
- Compute common-mode median and MAD metrics.
- Print metrics on existing detections.

### Stage 2 - Offline analysis

- Label field events from images.
- Compare vibration-only and insect distributions.
- Select candidate coherence/residual thresholds from real data.

### Stage 3 - Conservative vibration veto

- Add the veto behind a dedicated configuration flag.
- Reject only high-confidence vibration-only events.
- Keep ambiguous events as camera triggers.

### Stage 4 - Adaptive noise and drift model

- Maintain fast vibration/noise statistics.
- Maintain a separate slow drift estimate.
- Freeze adaptation on candidate/uncertain zones.
- Retain the existing baseline refresh as a fallback until the adaptive tracker is validated.

## Safety principle

At every stage, the system should fail toward **taking an extra image**, not toward silently discarding a possible small insect.
