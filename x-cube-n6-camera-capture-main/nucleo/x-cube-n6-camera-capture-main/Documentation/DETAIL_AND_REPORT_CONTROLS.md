# Shared exposure, fine detail and report controls

## Current baseline

The user's board run confirmed that motion blur improved at 5000 us exposure.
Keep `CAM_EXPOSURE_MODE=1`, `CAM_EXPOSURE_VALUE=5000` and the user's adjusted
`CAM_GAIN_VALUE=10000`. The driver quantizes the last value to 9900 mdB (gain
register 33), exactly as in the supplied readback.

These settings now apply to capture modes 0, 1, 2 and 4. IMX335 ISP automatic
exposure is disabled whenever this shared manual policy is selected. AUTO
re-enables it in every mode. The legacy one-init batch helper also obeys AUTO
instead of unconditionally writing manual values. Frame sizes, frame counts,
warmup, DMA destinations, ToF, illumination and storage algorithms are unchanged.
Only Mode 4 has the post-standby physical register diagnostic; cross-mode
compilation is not cross-mode hardware validation.

## Fine detail: what to compare next

No new smoothing or sharpening filter was introduced. The existing pipeline
includes Bayer demosaicing followed by output scaling. In Mode 4 the camera
reads 2592x1944 and DCMIPP outputs 1296x972. Fine features can be lost during
that size reduction. The log cannot prove that this is the cause of the
reported slightly smooth appearance; focus and residual motion are alternatives.

1. Put a stationary fine-detail target at the insect flight plane. Examine
   exported images at 100% pixel scale, avoiding an enlarged/interpolated preview.
   Keep exposure, gain, illumination and decoder unchanged.
2. Check lens focus at that exact working distance and across the intended
   flight depth. If stationary features are also soft, do not attribute all
   softness to motion or shorten exposure without evidence.
3. Compare an existing full-resolution Mode 0 single shot with Mode 4. With
   the shared exposure/gain this is a more useful resolution comparison. Mode 0
   also differs in capture timing, so use a stationary target and stable light.
   Compare the same physical feature, not just the apparent sharpness of two
   differently magnified previews. If more detail exists at full resolution,
   downsizing is a contributor.
4. Do not simply switch the four-frame burst to full resolution: each frame
   would require four times the payload and corresponding PSRAM/SD capacity.
   A full-resolution burst or unscaled crop needs a separate buffer/field-of-view
   review. Current burst dimensions are retained.
5. Keep the successful 5 ms setting for now. Reducing gain only helps noise if
   brightness remains usable; it does not restore detail lost to scaling or
   focus. Leave demosaic coefficients unchanged until a repeatable target
   comparison is available. Aggressive sharpening can exaggerate noise and
   outlines rather than reveal actual insect features.

ST's [ISP tuning guide](https://wiki.st.com/stm32mcu/wiki/ISP%3AHow_to_tune_ISP_using_the_STM32_ISP_IQTune)
and [IQTune tool](https://www.st.com/en/development-tools/stm32-isp-iqtune.html)
describe controlled sensor/lens ISP tuning. A stationary and moving image crop
would allow a more specific next change than guessing new filter coefficients.

## Report switches in Inc/app_config.h

| Setting | Default | Meaning |
| --- | --- | --- |
| `PERF_PRINT_SUMMARY` | 1 | Master report switch. 0 suppresses the report and summary fallback; timing measurements remain available. |
| `PERF_REPORT_DETAIL` | 2 | 0: one-line result; 1: camera phases, group totals and rates table; 2: additionally show SD subphases and counters. |
| `PERF_DEBUG_LEVEL` | 1 | Application camera/storage verbosity, independent of table detail. |
| `PERF_CAMERA_FRAME_LOG` | 0 | Mode 4 per-frame warmup/capture logs. If enabled, still prints only after DMA stops. |
| `CAM_SENSOR_REG_DEBUG` | 1 | Physical IMX335 register diagnostic after standby. Independent of the table. |
| `PERF_TRACK_SD_WAIT_TIME` | 1 | Record detailed SD wait/write/gap durations; disabling does not disable storage wall time. |
| `PERF_PRINT_STATS` | 0 | Enable running statistics when the summary is enabled and the window is nonzero. |
| `PERF_STATS_WINDOW` | 10 | Running statistics window size; zero disables statistics. |

For a detailed table with fewer camera/storage messages, use summary=1,
detail=2, debug=0, frame-log=0. Set sensor-reg-debug=0 too when physical
readback is no longer needed. These settings do not mute independent ToF
operational messages. To disable the table alone, set summary=0. The table
describes the instrumented task capture path; this change does not add that
report to the separate legacy button-only main.c storage path.

The ASCII table is 71 columns wide for serial terminals. Camera phase rows
are components of the camera total; SD detail rows are components of storage
wall time. Do not add those sections to the group totals a second time.
Percentages are rounded independently and can sum to 100.1%.

## Supplied board report and tests

The latest run reconciles: 634 ms camera + 3997 ms storage + 12 ms other =
4643 ms. Storage contains 893 ms in HAL calls plus 3104 ms remaining work.
That remainder is not evidence that the SD card itself takes 3104 ms; it
includes copying, checksum, cache, logging, recovery and untracked waits.
The measured payload rates are 2.40 MiB/s for storage and 2.07 MiB/s for the
cycle. Per-frame log suppression should reduce part of the 55 ms camera tail,
but the amount requires another board measurement.

`python tests/run_perf_tests.py` runs production accounting/printing code with
a fake HAL clock. It includes the latest board numbers, four-frame payloads,
retries, rollover, zero/missing/invalid timings, fixed table widths, compact
output, report-off behavior, statistics and independent verbosity controls.
Generated text and executables go under ignored `.validation/`.

ARM syntax checks cover camera, task and report sources plus IMX335 middleware
for all four modes; preprocessing checks AUTO/MANUAL AEC policy and shared
exposure/gain values. No board was flashed in this change, and fine-detail
improvement has not been claimed without image comparison.
