# IMX335 image quality review and bench procedure

Update: the subsequent [exposure and timing validation](EXPOSURE_AND_TIMING_VALIDATION.md)
sets Mode 4 to a 5 ms manual comparison and replaces the old timing accounting.
The AUTO default discussed below describes the earlier diagnostic stage.

Review date: 2026-09-08. Target: the NUCLEO-N657X0-Q project in this directory.
The goal is to retain fine detail on moving insects without unnecessarily increasing noise.
Image quality improvements need bench confirmation; no board was flashed during this review.

## Camera pipeline

1. `Src/main.c` initializes Mode 4 using `CAM_CallbackInit()`.
2. `Src/app_cam.c`: `CAM_Init()` requests the sensor's default dimensions (0/0).
   `Lib/Camera_Middleware/sensors/cmw_imx335.c` supports 2592x1944 and selects RAW10.
   The IMX335 sends Bayer data through CSI-2 to DCMIPP.
3. The IMX335 middleware starts the STM32 ISP. The tuning header
   `Inc/imx335_isp_param_conf.h` configures black level, demosaicing, AE,
   AWB/color correction, gamma and statistics. `CAM_IspUpdate()` runs the software
   ISP algorithms while waiting for camera events. Sensor changes travel back
   through middleware callbacks and the I2C arbiter.
4. DCMIPP Pipe 1 crops/scales and converts to packed YUV422. `CAM_BINNING=1`
   selects 1296x972 output, not physical IMX335 binning. Sensor readout timing
   and rolling-shutter distortion therefore do not improve just by using this flag.
5. At boot the camera warms up, then the pipe stops and the sensor enters standby.
6. The capture task in `Src/app_thread.c` handles detection/illumination and calls
   `CAM_CallbackBatchSnap()`. That function wakes the sensor, waits 35 ms,
   starts double-buffered capture, discards 11 frames, and retains four frames.
   It reassigns DMA destinations two frames ahead directly into PSRAM batch slots.
7. After frame completion, the pipe stops and the sensor returns to standby.
   The caller stores the raw frames using the existing SD path. Storage format,
   ToF detection and illumination settings were not changed.

## Findings and implemented changes

| Finding | Change / implication |
| --- | --- |
| Manual writes competed with enabled ISP AE; the IMX335 has no exposure-mode hook | Mode 4 manual policy now disables `AECAlgo.enable` in its ISP tuning header. AUTO leaves it enabled. Other modes retain the previous ISP setting. |
| `CAM_GAIN_VALUE=8` is interpreted as mdB and divided by 300 | It programs zero gain, not 8x. Corrected comments. Existing exposure/gain numbers are preserved and inactive in the new AUTO default. |
| `CMW_CAMERA_GetExposure/GetGain` return cached values | Added physical VMAX, SHS1 and gain register readings at boot standby and post-batch standby, using `CMW_I2C_READREG16`. |
| Proposed diagnostic before pipe stop could delay shutdown while DMA still targets final buffers | Diagnostics run after stop/standby. Existing per-frame verbose logs are buffered as timing values and printed after stop too. Batch duration excludes those deferred prints. |
| Configuration claimed all continuous frames were sharp and 200 us guaranteed negligible blur | Removed those claims. Continuous capture addresses restart tearing; it cannot guarantee motion sharpness. |
| FREEZE does not have an IMX335 implementation | Mode 4 rejects that configuration at compile time rather than silently behaving as AUTO. Manual tuning values are range-checked. |

Mode 4 now defaults to AUTO for a diagnostic baseline. This is not a claim that
AUTO minimizes motion blur. It establishes what exposure/gain the existing ISP
chooses under the current lighting, without competing application writes.
Static sensor settings, AWB, statistics area, sensor driver, frame dimensions,
frame count, warmup count, ToF, illumination and storage remain unchanged.

Readback limitations: the two log entries are end-state register snapshots, not
metadata for individual photographs. The sensor/ISP can delay application of AE
changes (the tuning file specifies a three-frame sensor delay). `exposure_est_us`
uses the driver's nominal 135000 lines/s; it is explicitly an estimate, not a
measurement of HMAX/clock timing. Raw register values and integration lines are
provided so changes in sensor timing do not become hidden assumptions. The
diagnostic is read-only but adds task time and I2C traffic after the batch;
disable `CAM_SENSOR_REG_DEBUG` for final throughput measurements.

## Controlled tuning sequence

1. Build and run the default Mode 4 AUTO configuration. Collect at least ten
   batches under the normal illumination, plus `INIT_STANDBY` and
   `AFTER_BATCH_STANDBY` log lines. Include a stationary fine-detail target at
   the insect flight plane. Compare stationary detail with moving detail using
   the same raw-image decoder and display scale.
2. If stationary detail is soft too, check lens focus at the actual working
   distance, depth of field, and decoder YUV packing before shortening exposure.
   Increasing sharpening cannot reconstruct missing motion detail and may
   accentuate the noise already reported.
3. For a manual comparison set `CAM_EXPOSURE_MODE=1` and choose an exposure from
   the AUTO readback, initially using the same gain in mdB (rounded to 300 mdB).
   Then halve exposure between runs until moving details stop improving.
   This policy now disables AE; inspect physical post-batch readings for
   stability across triggers. Do not simply switch to manual with the preserved
   8 us / 8 mdB experiment values and expect similar brightness.
4. Keep illumination fixed. If maintaining signal level requires increasing
   gain when exposure is halved, about +6000 mdB is a first comparison step,
   not a noise-free substitute for light. The local driver advertises analog
   gain through 30000 mdB; total gain extends to 72000 mdB. Prefer the longest
   exposure that meets the detail requirement, with the lowest usable gain.
5. Compare all four images, not just the best one. Record usable-frame fraction,
   fine-detail visibility, noise, clipping and first-versus-last brightness/color.
   Noisy images with a high edge score are not necessarily sharper photographs.
6. After exposure is stable, assess wake latency separately. Eleven discarded
   frames at 30 fps cost about 367 ms, plus the 35 ms wake delay and the first
   retained frame. First-frame availability is roughly 0.4-0.44 seconds after
   wake, before caller/trigger overhead. Short exposure does not fix that delay.
   A later experiment can reduce warmup gradually, but must validate first-frame
   color/exposure and the DMA schedule. This implementation needs at least two
   warmup frames to arm both first retained buffers; do not set it to zero.
   Keeping the sensor streaming is another later option with a power tradeoff.

For lateral motion, approximate blur in output pixels is
`blur_px = speed_m_s * exposure_s * image_width_px / scene_width_m`.
For example, a 100 mm scene at 1296 pixels and motion at 1 m/s gives about
2.6 pixels of blur at 200 us; a one-pixel budget would require about 77 us.
These are illustrative assumptions, not measurements of this setup. Insect
speed and field width are still needed to compute a useful exposure target.

## Remaining review items

- The existing sensor exposure driver reads four bytes starting at the 20-bit
  VMAX register, uses floating-point conversion and relaxes SHS1 to 1 with an
  incorrect comment about minimum exposure. Smaller SHS1 increases integration
  time. A separate driver change should mask VMAX, bound integration before
  subtraction, preserve the documented shutter limits and release register hold
  on write errors. It was intentionally not changed in this diagnostic stage.
- DMA buffer parity assumes each awaited event corresponds to one sequential
  frame. Deferred logging reduces one source of delay; it does not prove that
  missed frame events or task preemption cannot cause buffer reuse. Validate
  frame order/tearing on hardware under load before reducing warmup.
- Modes 0/1/2 still have their previous capture and exposure behavior, including
  their existing stop/restart and cached-readback limitations. This patch does
  not claim to fix those paths.
- AUTO may settle differently during boot and illuminated capture; a short
  burst cannot be assumed to have fully converged AE/AWB.

## Validation

ARM GCC syntax checks used the Makefile's Nucleo include paths, CPU flags and
sensor defines for both `Src/app_cam.c` and `sensors/cmw_imx335.c` in Modes 0,
1, 2, Mode 4 AUTO, Mode 4 MANUAL, and Mode 4 with sensor diagnostics disabled.
These checks cover compilation branches, not linking, sensor timing or image
quality. No full firmware link, flashing or hardware test was performed.

## Primary references

- [ST IMX335 driver](https://github.com/STMicroelectronics/stm32-imx335/blob/main/imx335.c): gain units and exposure-mode capability; local driver differs from upstream.
- [ST camera middleware](https://github.com/STMicroelectronics/stm32-mw-camera): sensor/ISP integration.
- [ST DCMIPP application note AN6211](https://www.st.com/resource/en/application_note/an6211-introduction-to-digital-camera-interface-pixel-pipeline-for-stm32-mcus-stmicroelectronics.pdf): ISP pipeline and tuning architecture.
