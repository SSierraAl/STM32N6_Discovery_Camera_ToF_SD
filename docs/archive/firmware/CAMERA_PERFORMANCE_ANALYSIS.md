# Camera Performance Analysis: Reducing Motion Blur for Fast-Moving Objects

## Current Configuration (app_config.h)

| Parameter | Current Value | Effect |
|-----------|--------------|--------|
| `SNAP_FPS` | 30 | 33.3ms per frame |
| `CAM_EXPOSURE_MODE` | 1 (MANUAL) | Fixed exposure, not auto |
| `CAM_EXPOSURE_VALUE` | 1000 µs | Very short exposure |
| `CAM_GAIN_VALUE` | 600 | Moderate gain |
| `SNAP_WARMUP_FRAMES` | 11 | 333ms warmup delay |
| `CAPTURE_MODE` | 1 (CONTINUOUS) | Camera always running |

**Current timing:** Camera=122ms + SD=195ms = Total=343ms per capture

---

## Why Motion Blur Occurs

Motion blur is caused by **long exposure time** (the sensor integration time during which light accumulates on each pixel). A moving object will appear blurred because its image shifts across multiple pixels during this integration window.

### Key insight:
- At 30 FPS, each frame period = 33.3ms
- Exposure of 1000µs = 1ms = ~3% of frame time
- **However**, the IMX335 sensor may ignore your exposure setting in certain conditions

---

## Why Changing Exposure May Not Work

### 1. CMW CAMERA MIDDLEWARE overrides exposure after initialization

In `app_cam.c` `CAM_InitAndStartContinuous()` (line ~430):

```c
/* ---- Apply camera quality settings ---- */
ret_expo_mode = CMW_CAMERA_SetExposureMode(MANUAL/AUTO/FREEZE);

if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
}
```

**PROBLEM:** The `CMW_CAMERA_SetExposureMode()` call happens BEFORE the exposure value is set. But the IMX335 sensor driver (`cmw_imx335.c`) may have internal logic that:

- Resets exposure to defaults when switching modes
- Clamps exposure values to valid ranges
- Ignores manual exposure if auto-focus/ae is still active

### 2. The IMX335 sensor has rolling shutter

The IMX335 uses a **rolling shutter** readout, meaning each row of pixels is exposed at a slightly different time. For fast-moving objects, this causes:
- **Geometric distortion** (leaning/tilt effect)
- **Motion smear** across rows
- No single "fastest" exposure — even 1µs exposure still has rolling shutter delay

### 3. FPS limits minimum exposure

At 30 FPS with a rolling shutter:
- Maximum theoretical exposure = 33.3ms (full frame)
- Rolling shutter adds per-row delay that can span the entire frame
- This means even short exposure settings may not eliminate all blur

---

## Recommended Settings for Fastest Shutter (Minimum Blur)

### Option A: Ultra-Fast Exposure (Bright Environment)
```c
#define SNAP_FPS             60        // 16.7ms frame time
#define CAM_EXPOSURE_MODE    1         // MANUAL
#define CAM_EXPOSURE_VALUE   100       // 100µs (0.1ms!) - minimum blur
#define CAM_GAIN_VALUE       2000      // Max gain to compensate for short exposure
#define SNAP_WARMUP_FRAMES   5         // Fewer warmup frames
```
**Result:** Extremely short exposure = frozen motion. Image will be noisy but sharp.

### Option B: Balanced Fast Exposure (Indoor)
```c
#define SNAP_FPS             60        // 16.7ms frame time
#define CAM_EXPOSURE_MODE    1         // MANUAL
#define CAM_EXPOSURE_VALUE   500       // 500µs (0.5ms)
#define CAM_GAIN_VALUE       1500      // High gain to compensate
#define SNAP_WARMUP_FRAMES   5         // Fewer warmup frames
```
**Result:** Good balance of sharpness and noise.

### Option C: Auto-Exposure with Freeze (Dynamic Lighting)
```c
#define SNAP_FPS             60
#define CAM_EXPOSURE_MODE    2         // FREEZE - uses last auto value
#define CAM_GAIN_VALUE       1000
#define SNAP_WARMUP_FRAMES   8
```
**Note:** First capture uses AUTO to establish baseline, then FREEZE locks it.

---

## Verify Exposure Actually Changed

Add this debugging code in `app_cam.c` after setting exposure:

```c
int32_t actual_exposure, actual_gain;
CMW_CAMERA_GetExposure(&actual_exposure);
CMW_CAMERA_GetGain(&actual_gain);
printf("[CAM] SET exposure=%ld, gain=%ld -> GOT exposure=%ld, gain=%ld\n",
       (long)CAM_EXPOSURE_VALUE, (long)CAM_GAIN_VALUE,
       (long)actual_exposure, (long)actual_gain);
```

If SET != GOT, the sensor is NOT using your value.

---

## Additional Blur Reduction Techniques

### 1. Increase Frame Rate to 60 FPS
- Halves the maximum rolling shutter delay
- Reduces motion smear by 2x
- May require lower resolution (640x480 at 60fps instead of 480x480)

### 2. Increase Illumination
- Shorter exposure = less light = need more illumination
- Add brighter WS2812 LEDs or increase pulse duration
- External strobe light (LED flash triggered at capture time)

### 3. Use Strobe Flash
- Trigger a brief (1ms) high-power LED flash at capture time
- Most light arrives during the flash, effectively "freezing" motion
- This is how sports photography works

### 4. Reduce Resolution
- Lower resolution = faster readout = less rolling shutter effect
- 320x240 reads out 4x faster than 640x480

---

## Capture Latency Breakdown

| Step | Current Time | Can Improve? |
|------|-------------|--------------|
| ToF detection | 67ms (15Hz) | Yes: increase to 30Hz |
| DCMIPP stop | <1ms | No |
| memcpy 460KB | ~120ms | No: PSRAM bandwidth limited |
| DCMIPP restart | <1ms | No |
| SD write 901 blocks | ~195ms | No: SD card speed |
| **Total** | **~343ms** | **Best: ~200ms** |

The 122ms "Camera" time is mostly the memcpy (116ms) plus stop/restart overhead. This is NOT the exposure time — it's the time to extract the frame from the always-running camera.

---

## Next Steps

1. **Test with 60 FPS** — change `SNAP_FPS` to 60
2. **Set minimum exposure** — try `CAM_EXPOSURE_VALUE = 100`
3. **Increase gain** — set `CAM_GAIN_VALUE = 2000` to compensate
4. **Verify exposure registers** — add the debug readback code
5. **Add strobe flash** — trigger WS2812 at exact capture moment
6. **Reduce warmup frames** — change `SNAP_WARMUP_FRAMES = 5` for faster init

The most impactful change for blur reduction is **exposure time**, but you MUST verify the sensor actually accepts it. The CMW middleware may be silently clamping it.