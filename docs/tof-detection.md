# ToF detection and spatial analysis

The default automatic capture configuration uses a single VL53L5CX in 4 × 4 mode. The detector identifies local changes in distance and reflected signal and requests a camera capture. The ToF zone grid and the camera image grid are separate. The camera determines the crop from the captured image. [System operation and performance](how-it-works.md)

## Ranging and reference acquisition

`sensor_task()` configures the sensor for autonomous ranging, the closest-target order, 45 ms integration and a requested rate of 15 frames/s. At startup the detector performs 15 settling attempts followed by 30 baseline sampling attempts. Each zone averages its accepted samples independently. A zone needs at least 15 accepted samples to be marked valid. An empty passage during acquisition establishes the trap geometry used in subsequent comparisons.

`VL53L5CX_Update()` supplies the current ranging data. The high-sensitivity 4 × 4 branch calls `VL53L5CX_TestDetectionStep()` for each fresh frame, including during cooldown so the detector can maintain its zone state. The candidate calculation requires a valid baseline, an accepted target status, a positive distance and a signal of at least `VL53L5CX_DET_MIN_SIGNAL`.

## Spatial analysis within one frame

For each valid zone, the detector calculates a signed distance change in millimeters and a signed percentage change in return signal relative to that zone's baseline. The median is calculated separately for distance and signal using valid zones from the same frame.

| Quantity | Calculation |
| --- | --- |
| Distance change | Current distance − baseline distance |
| Signal change | 100 × (current signal − baseline signal) / baseline signal, with integer truncation |
| Signed local residual | Zone change − median change across valid zones |
| Local evidence magnitude | Absolute value of the signed local residual |
| Frame-to-frame change | Difference from the previous valid measurement in the same zone |

For example, if most zones move 3 mm farther and one zone moves 2 mm closer, the median is approximately +3 mm. The zone of interest has a signed residual of −5 mm. The shared shift is removed, and the local change reaches the default 5 mm distance threshold for a level candidate. Actual capture still depends on event permissions and the zone latch.

The floor mask identifies valid zones whose baseline distance is within 30 mm of the farthest calibrated zone. This geometric test limits the persistence paths intended for insects on the floor. The protrusion test uses a negative signed distance residual because the floor-facing sensor measures a shorter distance when a target stands above the floor. Most level and edge tests use residual magnitudes.

## Candidate paths

| Path | Required evidence in the default configuration |
| --- | --- |
| Level change | A local signal change of at least 3% or a local distance change of at least 5 mm supplies a candidate without a second-frame confirmation. |
| Fast signal change | A 2% frame-to-frame signal change with weak local signal evidence sets a pending candidate. Weak local signal evidence in the same zone on the next frame confirms the fast path. |
| Fast distance change | A change of at least 4 mm from the preceding frame with local distance evidence of at least 3 mm sets a pending candidate. On the following frame, the fast path requires at least 3 mm local distance evidence and at least 2% local signal evidence in that zone. Strong 5 mm distance evidence has its own level path. |
| Floor protrusion | Weak signal evidence and a signed local distance of −1 mm or closer in a floor zone produce a candidate after four consecutive qualifying frames. |
| Persistent floor signal | Weak signal evidence in a floor zone adds four points per frame. The candidate threshold is 12. Quiet frames decay the score when level events are allowed. |
| Micro-persistence | A floor-zone residual of at least 1% signal or 2 mm distance adds two points per frame, up to 32. A quiet frame removes one point when level events are allowed. This path can supply a candidate from weak evidence held in a single zone. |

The firmware represents each set of zones as a 16-bit mask with bit `1 << z` for zone `z`. The optional weak spatial track combines newly active floor zones supported by recent motion or a confirmed floor protrusion. Its default criteria are two neighboring zones (diagonals included) or three distinct zones within 5 s, with a maximum of three zones. `VL53L5CX_DET_WEAK_TRACK_ENABLED=0` disables this trigger path in the checked-in configuration. The optional trigger-zone mask is also disabled.

## Trigger gating and baseline updates

1. The sensor task selects the event classes allowed for the frame. During the post-capture frame cooldown, the detector updates its internal state with capture disabled. Once the counter reaches zero, the post-refresh restriction can still limit events to the level and fast-edge classes during the three-second recovery interval.
2. The detector combines the permitted candidate masks. A latch suppresses another event from the same persistent evidence. A stronger level change can trigger in a zone previously latched by weaker evidence. Three clear frames release the latch under the default clear rule.
3. An accepted event requests a camera image. The sensor task waits for the request, including any SD storage. The default configuration then refreshes the ToF baseline and sets a five-frame cooldown. A camera decision that discards the image still counts as a request.
4. Between events, quiet zones can gradually update their signal reference. A stable local change can recenter that zone after 12 s if the frame count and jitter limits are met. A persistent, same-direction change meeting the scene-wide threshold in at least 12 zones for 5 s can request a full refresh when the motion condition permits it. The periodic refresh is scheduled after five minutes without a full baseline update.

A baseline refresh can include a target still in the passage. Controlled tests therefore distinguish cases where a target remains stationary, leaves and returns, or crosses before the next eligible trigger. These cases exercise different detection and reference-update paths.

## Diagnostic records

| Record | Interpretation |
| --- | --- |
| `TOFEVT` | Accepted detector event. Zone masks identify the evidence and the latch state. Storage is decided later from the camera image. |
| `TOFMISS` | Candidate blocked by the current trigger policy. |
| `TOFCAL` | Per-zone calibration data when enabled, including signed local residuals. |
| `[ADAPT]` | Baseline updates and post-capture recovery. |
| `[CAM][SELECT]` | Image classification and storage selection for a completed exposure. |

Bench comparisons report empty-passage triggers per minute, usable images per controlled passage, and the interval before a second passage can trigger. Tests covering different positions on the ToF grid can reveal zones with missing or weak returns under the installed geometry. The ToF detector can respond to an insect that leaves the camera field before exposure, so interpretation uses both the ToF and image selection records.

[ToF detector implementation](../x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Src/vl53l5cx_detection.c) · [Sensor task](../x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Src/app_thread.c) · [Configuration](../x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Inc/app_config.h)
