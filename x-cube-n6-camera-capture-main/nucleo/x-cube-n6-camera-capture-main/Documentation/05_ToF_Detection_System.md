# Project MASSIF 
**Monitoring Automatisé et Systèmes de Surveillance Intelligents de la biodiversité des insectes dans les écosystèmes Forestiers français**

---

## 1. ToF Sensor Overview
The VL53L5CX 520 nm Direct Time-of-Flight (DToF) sensor acts as the autonomous, event-driven trigger for the entire camera system. Unlike the camera, the ToF sensor does not capture optical images; its sole responsibility is to continuously measure the observation area and raise a hardware event when an insect breaks the physical baseline.

*   **Ranging Grid:** The sensor divides its field of view into a matrix of ranging zones. By default, it uses a 4x4 grid (16 zones) for speed, but can be configured for 8x8 (64 zones) for higher spatial detail.
*   **Zone Data:** Every zone independently reports its measured distance (mm), returned signal strength (per SPAD), target status, and a proprietary motion-indicator value (0-255).
*   **Thread Ownership:** The entire ToF polling and trigger logic is strictly owned by the `sensor_task`.

---

## 2. Baseline Learning Logic
Before the system can detect an insect, it must learn the geometric profile of the "empty" environment (the background).

1.  **Initialization Order:** The ToF sensor is initialized before `system_ready` is asserted to prevent I2C conflicts with the camera's boot sequence.
2.  **Baseline Accumulation (`VL53L5CX_LearnBaseline`):** The system clears previous arrays and averages `VL53L5CX_DET_BASELINE_SAMPLES` valid frames (10 frames for a 4x4 grid) to create the empty-scene baseline. 
3.  **Signal Quality Gating:** The algorithm strictly accepts only zones with a valid target status (0, 5, or 9) and a returned signal strength at or above `VL53L5CX_DET_MIN_SIGNAL` (500 kcps). Zones with no usable return are explicitly excluded to prevent false triggers from blind or extremely noisy spatial areas.
4.  **Boot Stabilization Window:** Immediately after ranging starts, the sensor enforces a mandatory 5-second stabilization window. Readings inside this window are deliberately discarded to prevent spurious false-triggers before the sensor's internal algorithms fully stabilize.

---

## 3. Detection Algorithm (`VL53L5CX_Update`)
The `sensor_task` loops continuously, fetching frames and running the detection algorithm on every valid zone. Detection relies on two independent evidence channels:

*   **Channel 1: Signal Collapse:** When an insect cross between the sensor and the background, it intercepts the back-scatter, causing the live per-zone signal strength to drop. If this drop is greater than `VL53L5CX_DET_THRESHOLD_PCT` (default 6% for 4x4) below the learned baseline, the zone is flagged.
*   **Channel 2: Motion Indicator:** The ST API calculates a motion value (0-255). If this value meets or exceeds `VL53L5CX_DET_MOTION_THRESH` (default 40), the zone is flagged.

**The Trigger Rule:** If the total count of flagged zones reaches the `VL53L5CX_DET_MIN_AFFECTED_ZONES` threshold (1 for a 4x4 grid), the `insect_detected` flag is set, and the trigger sequence begins.

---

## 4. Baseline Maintenance (Adaptation Modes)
Environmental conditions (like sunlight shifts or a leaf falling into view) can alter the background. The firmware provides two baseline maintenance strategies to prevent the sensor from becoming permanently blinded.

*   **Periodic Refresh:** Forces a baseline refresh every N frames purely based on time, regardless of detection activity (default is off).
*   **Adaptive Refresh (Production Default):** Utilizes a 10-second sliding window. If more than 5 detections (`VL53L5CX_DET_MAX_DETECTIONS`) occur within this brief window, the system assumes "adaptation fatigue". This means the environment has permanently changed (e.g., an insect landed permanently on the lens), forcing the ToF sensor to re-learn its baseline to re-stabilize the system.

---

## 5. Trigger Orchestration & Synchronization Safeguards
When an insect is successfully detected, `sensor_task` orchestrates a strictly guarded sequence to hand control over to the camera pipeline without causing I2C bus collisions.

1.  **The Busy Gate (`g_capture_busy`):** The task first checks the volatile busy lock. If a capture is already actively writing to the SD card, the new ToF detection is deliberately dropped (no queue buildup or re-entrant triggers allowed).
2.  **I2C Arbitration:** Because both the ToF sensor and the camera share the physical `I2C1` bus, the `g_i2c1_mutex` strictly locks the bus so the ToF's continuous polling never collides with the camera's wake-up routine.
3.  **The Cooldown Phase:** Once the SD write completes and the camera returns to standby, the `sensor_task` resumes ranging but enforces a strict 30-frame cooldown (roughly 1 second at 30 Hz). This guaranteed dead-time prevents the system from rapidly re-triggering on the exact same insect as it flies away, or triggering on residual LED afterglow.