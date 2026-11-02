# Multi-Resolution Insect Detection & Capture Strategy — v2.0

> **Last updated**: 2026-10-30
> **Status**: Phase 1 complete, Phase 2 partial, Phase 3–5 pending

---

## 1. Overview & Goal

Implement a **three-stage multi-resolution capture pipeline** on the STM32N6570-DK (Discovery) for insect photography. The pipeline captures photos at three different resolutions per trigger event:

| Stage | Resolution     | Purpose                         | Frames |
|-------|----------------|---------------------------------|--------|
| 1     | 640×480        | Fast low-res detection + insect sizing by `IF_Process()` | 1      |
| 2     | 1296×972       | Full-frame capture (1.3 MP, DCMIPP-downscaled from 2592×1944) | 2–4    |
| 3     | ROI@2592×1944  | High-res cropped capture via sensor AREA3 windowing       | 2–4    |

All processing is on-device. No PC involvement.

---

## 2. Repository Structure

```
Discovery/
├── MULTI_RES_STRATEGY.md                 ← THIS FILE
├── Image_filter.py                        Python reference prototype
└── x-cube-n6-camera-capture-main/
    └── nucleo/
        └── x-cube-n6-camera-capture-main/
            ├── Inc/
            │   ├── app_cam.h             [MODIFIED] +CAM_SetSensorROI, CAM_WriteSensorRegMulti
            │   ├── app_config.h           [TO BE MODIFIED] +IF_* detection params
            │   ├── image_filter.h         [CREATED] Detection data structures & API
            │   └── raw_image.h            [TO BE MODIFIED] +stage metadata
            ├── Src/
            │   ├── app_cam.c             [MODIFIED] +CAM_SetSensorROI, CAM_SetFullFrame, helpers
            │   ├── app_thread.c           [TO BE MODIFIED] +three-stage state machine
            │   └── image_filter.c         [CREATED] Full detection algorithm
            └── Lib/
                └── Camera_Middleware/
                    └── sensors/
                        └── imx335/
                            ├── imx335_reg.h   Read-only reference for AREA3 addresses
                            └── imx335.c       Read-only reference

External reference project:
MASSIF-ROI-Windowing/                        Proven AREA3 register sequence
  ├── Src/roi_test.c                          App_ChangeROI() reference
  └── Src/app_cam.c                           DCMIPP crop integration pattern
```

---

## 3. Three-Stage Pipeline Architecture

```
Trigger (ToF detection or button)
  │
  ▼
┌──────────────────────────────────────────────────────────────────────┐
│ STAGE 1: Low-res Fast Detection @ 640×480                            │
│                                                                      │
│  1. Deinit camera (if running)                                       │
│  2. CAM_Init() with conf.capture_width=640, capture_height=480       │
│     → DCMIPP downscales 2592×1944→640×480 (sensor stays full-res)    │
│  3. Start pipe, discard 2 warmup frames, capture 1 frame             │
│  4. Call IF_Process(frame, &result):                                  │
│     - Downscale 640×480 YUV422 → 80×60 grid (8×8 block avg of Y)     │
│     - Compute residual = |frame - bg|, removing global mean shift     │
│     - Classify: CLEAN / DIFFERENT (insect) / BROAD (lighting change)  │
│     - Hot-block detection on 4×4 grid cells → bounding box            │
│     - Masked EMA background update (hot blocks excluded)              │
│     - Fallback check: map bbox to sensor px → too large for ROI?     │
│  5. If NO insect → return to standby                                 │
│  6. If insect → save bounding box, proceed to Stage 2                │
└──────────────────────────────────────────────────────────────────────┘
  │ insect detected
  ▼
┌──────────────────────────────────────────────────────────────────────┐
│ STAGE 2: 1.3 MP Full-Frame Capture @ 1296×972                       │
│                                                                      │
│  1. CAM_Deinit()                                                     │
│  2. CAM_CallbackInit(1296×972) → full init + warmup + standby       │
│  3. CAM_CallbackBatchSnap() → 2–4 frames (zero-copy DMA)             │
│  4. Save all frames to SD with header metadata (Stage=2)             │
└──────────────────────────────────────────────────────────────────────┘
  │
  ▼
┌──────────────────────────────────────────────────────────────────────┐
│ FALLBACK CHECK                                                       │
│                                                                      │
│  IF_CalcROI(&result, &roi_x, &roi_y, &roi_w, &roi_h) returns:        │
│   0 → insect fits in ROI → proceed to Stage 3                        │
│  -1 → insect too large (≥IF_ROI_MAX_SZ_W=600 or ≥IF_ROI_MAX_SZ_H=440)│
│       → skip Stage 3, done                                           │
└──────────────────────────────────────────────────────────────────────┘
  │ fits in ROI
  ▼
┌──────────────────────────────────────────────────────────────────────┐
│ STAGE 3: Max-Resolution ROI Capture @ sensor-native pixels            │
│                                                                      │
│  1. CAM_Deinit()                                                      │
│  2. CAM_SetSensorROI(roi_x, roi_y, roi_w, roi_h)                      │
│     → Standby → Hold → AREA3 regs → VMAX → SHUTTER → Stream          │
│  3. CAM_Init() with conf.capture_width=roi_w, capture_height=roi_h   │
│     → DCMIPP pipe configured for the new smaller window              │
│  4. CAM_CallbackBatchSnap() → 2–4 frames at max-res ROI             │
│  5. Save all frames to SD (linked to Stage 2 via shared event_id)    │
│  6. CAM_Deinit() + CAM_SetFullFrame() → restore to full sensor       │
│  7. CAM_CallbackInit(1296×972) → return to standby for next trigger │
└──────────────────────────────────────────────────────────────────────┘
  │
  ▼
Return to standby / low-power state
```

### Key Architectural Decision: Why No Sensor Reconfig for Stage 1

For Stage 1 (640×480), we **do NOT** change the IMX335 output size. The sensor stays at 2592×1944. DCMIPP hardware downscales to 640×480. Rationale:

| Approach | Pros | Cons |
|----------|------|------|
| Sensor reconfig to 640×480 | Saves MIPI bandwidth | Complex timing; risk of instability; need full mode register set |
| DCMIPP downscale (chosen) | Sensor stays in known state; proven working; fast reinit | Higher MIPI bandwidth (doesn't matter for 1 frame) |

Stage 3 **requires** AREA3 windowing because the ROI is dynamic and the MIPI bandwidth / PSRAM savings for 4 frames at 2592×1944 would be prohibitive.

---

## 4. Phase 1 — Image Filter C Port ✅ DONE

### Files Created

| File | Lines | Status |
|------|-------|--------|
| `Inc/image_filter.h` | 85 | Complete |
| `Src/image_filter.c` | 307 | Complete |

### Data Structures (`image_filter.h:42–55`)

```c
typedef struct {
    uint8_t  grid[60][80];              // downsampled luma (80×60)
} IF_Grid_t;

typedef struct {
    uint8_t  hot_map[15][20];           // 1 = hot block (20×15 hot blocks)
    uint16_t bbox_x, bbox_y;            // top-left in detection-px (640×480 space)
    uint16_t bbox_w, bbox_h;            // size in detection-px
    uint8_t  hot_count;                 // number of hot blocks
    uint8_t  hot_strong_count;          // blocks ≥ 2× threshold (strong evidence)
    uint8_t  has_insect;                // 0 or 1
    uint8_t  fallback;                  // 1 = insect too large for Stage 3 ROI
} IF_Result_t;
```

### Detection Algorithm Parameters (`image_filter.h:6–36`)

| Define | Value | Meaning |
|--------|-------|---------|
| `IF_DETECT_W` | 640 | Detection frame width (pixels) |
| `IF_DETECT_H` | 480 | Detection frame height (pixels) |
| `IF_DS` | 8 | Downsample factor: 8×8 pixels → 1 grid cell |
| `IF_GRID_W` | 80 | Grid width: 640/8 |
| `IF_GRID_H` | 60 | Grid height: 480/8 |
| `IF_HOT_DS` | 4 | Hot-block factor: 4×4 grid cells → 1 hot block |
| `IF_HOT_W` | 20 | Hot blocks wide: 80/4 |
| `IF_HOT_H` | 15 | Hot blocks high: 60/4 |
| `IF_T_PIXEL` | 24 | Per-cell residual threshold: |diff|≥24 → "changed pixel" |
| `IF_T_BLOCK` | 10 | Block-mean residual ≥10 → "hot block" (≥2× = "strong") |
| `IF_EMA_SHIFT` | 4 | EMA adaptation rate: bg += diff >> 4 (faster than 3) |
| `IF_GLOBAL_MEAN` | 8 | Global mean shift ≥8 → consider scene-wide change |
| `IF_GLOBAL_MAX_HOT` | 6 | ... AND hot blocks ≤6 → it's a lighting change, not insect |
| `IF_MAX_HOT_OBJECT` | 40 | Localized object has ≤40 hot blocks |
| `IF_MIN_OBJECT_BPS` | 5 | Object must change ≥0.05 % of grid cells |
| `IF_CLEAN_MAX_BPS` | 25 | CLEAN class: changed cells ≤0.25 % |
| `IF_CLEAN_MAX_DIFF` | 16 | CLEAN class: max cell diff ≤16 |
| `IF_ROI_MARGIN` | 20 | Extra detection-px around bbox for ROI calculation |
| `IF_ROI_MAX_SZ_W` | 600 | Max insect width in sensor-px for Stage 3 eligibility |
| `IF_ROI_MAX_SZ_H` | 440 | Max insect height in sensor-px for Stage 3 eligibility |

### Algorithm Pipeline (`image_filter.c:81–251`)

```
IF_Process(yuv422_frame, &result):
  1. Downscale(frame) → 80×60 grid          [L81–115]
  2. Compute grid mean & bg mean              [L118–128]
  3. Global mean shift removal                [L130]
  4. Residual = |grid - bg - global_shift|     [L135–144]
  5. changed_pixel count, max_diff            [L145–146]
  6. Hot blocks: 4×4 grid blocks, mean≥IF_T_BLOCK → hot  [L148–168]
  7. Bounding box from hot blocks             [L171–208]
     - Prefer strong blocks (≥2× threshold) for tighter bbox
     - Fall back to any hot block if no strong
  8. Classification (CLEAN / DIFFERENT / BROAD) [L210–228]
  9. Masked EMA background update             [L231–242]
     - Skip cells whose hot block is flagged
     - Prevents learning insect into background
  10. Fallback check: map bbox to sensor px   [L245–250]
```

### Classification Logic (`image_filter.c:214–228`)

```
IF (changed_bps ≤ 25 AND max_diff ≤ 16)        → CLEAN (no insect, update bg)
ELSE IF (hot_count == 0 AND changed_bps < 5)    → Noise (no insect, update bg)
ELSE IF (hot_strong_count > 0 AND
         hot_count ≤ 40 AND changed_bps ≥ 5)    → DIFFERENT → insect!
ELSE IF (hot_count > 0 AND
         hot_count ≤ 40 AND changed_bps ≥ 5)    → DIFFERENT weak → insect!
ELSE                                             → BROAD (lighting change, update bg)
```

### ROI Calculation (`image_filter.c:258–307`)

```
IF_CalcROI(&detection, &roi_x, &roi_y, &roi_w, &roi_h):
  1. Map bbox → sensor pixel coordinates (2592×1944)
     sensor_w = bbox_w * 2592 / 640
     sensor_h = bbox_h * 1944 / 480
     sensor_cx = (bbox_x + bbox_w/2) * 2592 / 640
     sensor_cy = (bbox_y + bbox_h/2) * 1944 / 480
  2. If sensor_w > 600 OR sensor_h > 440 → return -1 (too large)
  3. ROI = insect + margin centered on insect_cx/cy
  4. Clamp to sensor bounds (0..2592, 0..1944)
  5. Even alignment (YUV422: & ~1)
  6. Minimum ROI size: 64×64
  7. Return 0 (success)
```

### First-Frame Handling (`image_filter.c:86–92`)

On the very first call to `IF_Process()`, the current frame is snapped as the initial background and `has_insect=0` is returned. This avoids a false-positive on the first frame when no background exists yet.

### Verification Notes

The C implementation has NOT yet been validated against the Python prototype. Validation task (Phase 5) should:
1. Capture 10 test images from the camera at 640×480
2. Process both with `Image_filter.py` and `IF_Process()`
3. Compare: has_insect classification must match; bounding box must be within ±1 grid cell

---

## 5. Phase 2 — Sensor Reconfiguration Helpers ✅ PARTIAL

### Files Modified

| File | Lines | Changes |
|------|-------|---------|
| `Inc/app_cam.h` | 255 | Added `CAM_WriteSensorRegMulti()`, `CAM_SetSensorROI()`, `CAM_SetFullFrame()` declarations |
| `Src/app_cam.c` | 1030 | Added multi-byte write helper, 24/32-bit write helpers, AREA3 sequence implementation |

### Functions Implemented (`app_cam.c:49–212`)

#### `CAM_WriteSensorRegMulti(uint16_t reg, const uint8_t *data, uint16_t len)` — `app_cam.c:49`

Wraps `CMW_I2C_WRITEREG16()` for multi-byte register writes. Used for AREA3 2-byte X/Y start/end registers.

#### `CAM_SetSensorROI(uint16_t x_start, uint16_t y_start, uint16_t width, uint16_t height)` — `app_cam.c:92`

Full AREA3 register sequence with 3× retry per register write:

| Step | Register | Value | Delay | Description |
|------|----------|-------|-------|-------------|
| 1 | `0x3000` | `0x01` | 20 ms | STANDBY mode |
| 2 | `0x3001` | `0x01` | — | Set HOLD |
| 3 | `0x3074` | x_start (2B LE) | — | AREA3 X start address |
| 4 | `0x3076` | y_start (2B LE) | — | AREA3 Y start address |
| 5 | `0x3078` | x_end (2B LE) | — | AREA3 X end address |
| 6 | `0x307A` | y_end (2B LE) | — | AREA3 Y end address |
| 7 | `0x3030` | VMAX (4B LE) | — | Frame length = height + 100 |
| 8 | `0x3058` | SHUTTER (3B LE) | — | Exposure = VMAX − 100 |
| 9 | `0x3001` | `0x00` | 30 ms | Clear HOLD (apply) |
| 10 | `0x3000` | `0x00` | 50 ms | STREAMING mode |

Validation: x_start+width ≤ 2592, y_start+height ≤ 1944, min 16×4. Returns 0 on success, −1 on I2C failure.

#### `CAM_SetFullFrame(void)` — `app_cam.c:209`

Convenience: calls `CAM_SetSensorROI(0, 0, 2592, 1944)`.

### Missing from Phase 2

- **`CAM_InitWithDimensions(w, h)`** — A wrapper that deinits and reinits with new capture width/height. Currently each caller must manually `CAM_Deinit()` then build a `CAM_conf_t` and call `CAM_Init()`. For Stage 1 and Stage 3 this pattern must be added to `app_thread.c`.

---

## 6. Phase 3 — Three-Stage Pipeline Integration 🔴 PENDING

### File: `Src/app_thread.c` — Three-Stage State Machine

New state machine:

```c
typedef enum {
    STATE_IDLE,
    STATE_STAGE1_DETECT,
    STATE_STAGE2_CAPTURE,
    STATE_STAGE3_ROI,
    STATE_STAGE3_SKIP,
    STATE_RESTORE
} capture_state_t;
```

### Stage 1 Integration (State: `STATE_STAGE1_DETECT`)

```c
// In camera_task command handler, case CAM_CAPTURE_IMAGE:
capture_state = STATE_STAGE1_DETECT;

// 1. Configure 640×480 detection capture
CAM_conf_t conf = {
    .capture_width = IF_DETECT_W,     // 640
    .capture_height = IF_DETECT_H,    // 480
    .fps = SNAP_FPS,
    .dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1,
    .is_rgb_swap = 0,
};
CAM_Init(&conf);
CMW_CAMERA_SetExposureMode(...);
if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
}

// 2. Start pipe, warmup 2 frames, capture 1
extern uint8_t detection_frame[IF_DETECT_FRAME_SZ];  // ≈614KB in PSRAM
CAM_CapturePipe_Start(detection_frame, CMW_MODE_CONTINUOUS);
CAM_WaitNextFrameReady(200, NULL);    // warmup 1
CAM_WaitNextFrameReady(200, NULL);    // warmup 2
CAM_WaitNextFrameReady(200, NULL);    // capture frame
HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);

// 3. Process detection
IF_Result_t result;
IF_Process(detection_frame, &result);

if (result.has_insect) {
    // Save bounding box info → metadata for SD header
    capture_state = STATE_STAGE2_CAPTURE;
    g_detection_result = result;       // static global
} else {
    // No insect → done
    CAM_Deinit();
    capture_state = STATE_IDLE;
}
```

**IMPORTANT**: The `detection_frame` buffer must be statically allocated in PSRAM (614 KB). It can overlap with `capture_buf` since Stage 1 and Stage 2 run sequentially.

### Stage 2 Integration (State: `STATE_STAGE2_CAPTURE`)

```c
capture_state = STATE_STAGE2_CAPTURE;

// 1. Deinit from Stage 1
CAM_Deinit();

// 2. Reinit with 1296×972 (Callback-Batch mode)
CAM_CallbackInit(batch_buf, CALLBACK_BUF_SIZE, SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS);

// 3. Capture batch
int captured = CAM_CallbackBatchSnap(batch_buf, SNAP_FRAME_SIZE);
if (captured > 0) {
    // Send to storage task with Stage=2 metadata
    // Include event_id = g_event_counter
}

// 4. Check if Stage 3 is eligible
uint16_t roi_x, roi_y, roi_w, roi_h;
if (IF_CalcROI(&g_detection_result, &roi_x, &roi_y, &roi_w, &roi_h) == 0) {
    capture_state = STATE_STAGE3_ROI;   // proceed
} else {
    capture_state = STATE_STAGE3_SKIP;  // insect too large
}
```

### Stage 3 Integration (State: `STATE_STAGE3_ROI`)

```c
capture_state = STATE_STAGE3_ROI;

// 1. CAM_Deinit()
CAM_Deinit();    // stops pipe, sensor goes to standby inside DeInit

// 2. Write AREA3 registers directly
HAL_Delay(10);
CAM_SetSensorROI(roi_x, roi_y, roi_w, roi_h);

// 3. Reinit DCMIPP pipe with ROI dimensions
CAM_conf_t roi_conf = {
    .capture_width = roi_w,
    .capture_height = roi_h,
    .fps = SNAP_FPS,
    .dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1,
    .is_rgb_swap = 0,
};
CAM_Init(&roi_conf);

// 4. Reapply exposure (are these correct after AREA3?)
CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL : ...);
if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
}

// 5. Capture batch (use callback-batch or single-frame)
// Need roi_frame_size = roi_w * roi_h * 2
size_t roi_frame_size = (size_t)roi_w * (size_t)roi_h * 2;
uint32_t roi_batch_size = roi_frame_size * CALLBACK_FRAMES;

// Option A: Reuse batch_buf for Stage 3 (safe since Stage 2 is done)
// batch_buf is ~2.5MB; roi_batch_size is similar
CAM_CallbackInit(batch_buf, roi_batch_size, roi_w, roi_h, SNAP_FPS);
int roi_captured = CAM_CallbackBatchSnap(batch_buf, roi_frame_size);

// 6. Save with Stage=3 metadata (same event_id as Stage 2)
```

### Restore (State: `STATE_RESTORE`)

```c
capture_state = STATE_RESTORE;

// 1. Deinit ROI pipe
CAM_Deinit();

// 2. Restore full sensor
CAM_SetFullFrame();

// 3. Reinit in 1296×972 standby mode
CAM_CallbackInit(batch_buf, CALLBACK_BUF_SIZE, SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS);

capture_state = STATE_IDLE;
```

### Fallback (State: `STATE_STAGE3_SKIP`)

```c
capture_state = STATE_STAGE3_SKIP;

printf("[CAPTURE] Stage 3 skipped: insect %ux%u exceeds ROI limit\n",
       g_detection_result.bbox_w, g_detection_result.bbox_h);

// Already in standby from Stage 2 completion
// Just transition to idle
capture_state = STATE_IDLE;
```

### Key Risks in Phase 3

1. **`CAM_Deinit()` + `CAM_SetSensorROI()` ordering**: The `CAM_Deinit()` function calls `CMW_CAMERA_DeInit()` which puts the sensor in standby and stops the pipe. After this, `CAM_SetSensorROI()` does its own standby→hold→write→stream cycle. The double-standby at the start of `CAM_SetSensorROI()` is harmless (sensor is already in standby from `CAM_Deinit()`).

2. **Exposure after AREA3**: After `CAM_SetSensorROI()` changes VMAX and SHUTTER, the previously-set exposure value (which was for a different frame size) may no longer be optimal. The exposure/gain must be re-applied after `CAM_Init()` post-AREA3.

3. **Double buffer conflict in Stage 3**: `CAM_CallbackBatchSnap()` uses `capture_buf` and `save_buf` as double-buffer scratch. These are sized for 1296×972 (2.5 MB each). For Stage 3 at e.g. 640×480, they're oversized but still work. For large ROIs (up to 600×440), they still fit in 2.5 MB.

4. **Batch buffer size for Stage 3**: `batch_buf` is `CALLBACK_FRAMES * SNAP_FRAME_SIZE = 2 * 1296 * 972 * 2 = 5,038,848 bytes`. For Stage 3 at max ROI (600×440): `2 * 600 * 440 * 2 = 1,056,000 bytes`. It fits easily.

5. **`CAM_WaitNextFrameReady()` is static**: This function (`app_cam.c:472`) is `static`. Stage 3 integration may need to expose it or duplicate the pattern.

---

## 7. Phase 4 — SD Storage Extensions 🔴 PENDING

### File: `Inc/raw_image.h`

Need to extend the SD image header to include stage identification and bounding box metadata.

Current header (in `raw_image.h`):

```c
typedef struct {
    uint32_t magic;
    uint32_t frame_size;
    uint32_t width;
    uint32_t height;
    uint32_t event_id;         // ← likely already exists
    uint32_t timestamp;
    uint8_t  reserved[40];     // ← use some of these
} sd_image_header_t;
```

Changes needed:

| Field | Offset | Size | Description |
|-------|--------|------|-------------|
| `stage` | after `timestamp` | 4 | 1, 2, or 3 |
| `roi_x` | +4 | 2 | AREA3 X start (Stage 3 only) |
| `roi_y` | +6 | 2 | AREA3 Y start |
| `roi_w` | +8 | 2 | AREA3 width |
| `roi_h` | +10 | 2 | AREA3 height |
| `bbox_x` | +12 | 2 | Detection bounding box X |
| `bbox_y` | +14 | 2 | Detection bounding box Y |
| `bbox_w` | +16 | 2 | Detection bounding box width |
| `bbox_h` | +18 | 2 | Detection bounding box height |
| (reserved) | +20 | 20 | Reduced reserved |

### File: `Src/app_thread.c` — Storage Task

The storage task needs to know what stage it's saving. Options:

**Option A**: Separate buffers per stage with stage metadata embedded.
```
g_stage_event_id    — shared counter incremented per trigger
g_stage_num         — 2 or 3 (set by capture state machine)
g_stage_roi_x/y/w/h — set by Stage 3 path
g_bbox_x/y/w/h      — set by IF_Process()
```

**Option B**: Unique SD filename prefixes:
- Stage 2: `EVT{id}_S2_{frame}.raw`
- Stage 3: `EVT{id}_S3_{frame}.raw`

---

## 8. Phase 5 — Configuration Defines 🔴 PENDING

### File: `Inc/app_config.h` (or `Inc/image_filter.h`)

The detection parameters are already in `Inc/image_filter.h`. app_config.h needs no IF_* defines. But app_config.h needs:

```c
// Detection frame buffer
#define DETECT_BUF_SIZE     (IF_DETECT_W * IF_DETECT_H * 2)  // 614,400

// Stage 3 buffer (reuse batch_buf)
// batch_buf is CALLBACK_FRAMES * SNAP_FRAME_SIZE
// For Stage 3: roi_frame_size ≤ 600*440*2 = 528,000 per frame
// 2 frames → 1,056,000 bytes — fits in batch_buf (5 MB)
```

---

## 9. PSRAM Budget

| Buffer | Resolution | Bytes | Stage |
|--------|-----------|-------|-------|
| `detection_frame` | 640×480 YUV422 | 614,400 | Stage 1 only (reusable) |
| `capture_buf` | 1296×972 YUV422 | 2,519,424 | Stage 2 & 3 (reused) |
| `save_buf` | 1296×972 YUV422 | 2,519,424 | Stage 2 & 3 (reused) |
| `batch_buf` | 1296×972 × 2 frames | 5,038,848 | Stage 2 & 3 (reused, same buffer) |
| **Peak total** | — | **~8.2 MB** | Well within 16 MB PSRAM |

Buffers are serialized: only one stage active at a time. No overlap.

---

## 10. Complete Task List (Detailed)

### 🔲 Task 1: Integrate Phase 3 State Machine in `Src/app_thread.c`

**Prerequisites**: Phase 1 (done), Phase 2 (done)

**Files to modify**: `Src/app_thread.c`

**Steps**:

| # | Step | Details | Estimated effort |
|---|------|---------|-----------------|
| 1.1 | Add capture states enum | `STATE_IDLE`, `STATE_STAGE1_DETECT`, `STATE_STAGE2_CAPTURE`, `STATE_STAGE3_ROI`, `STATE_STAGE3_SKIP`, `STATE_RESTORE` | 10 min |
| 1.2 | Add `g_detection_result` global | Store `IF_Result_t` from Stage 1 for use in Stage 3 and metadata | 5 min |
| 1.3 | Add `g_event_counter` global | Monotonically incremented per trigger, used as shared event_id | 5 min |
| 1.4 | Add detection frame buffer | `static uint8_t detection_frame[IF_DETECT_FRAME_SZ]` in PSRAM | 10 min |
| 1.5 | Implement Stage 1 in `camera_task` | Deinit → Init(640×480) → Start → Warmup 2 → Capture 1 → IF_Process() | 2 hours |
| 1.6 | Implement Stage 2 in `camera_task` | Deinit → CallbackInit(1296×972) → CallbackBatchSnap → Send to storage | 1 hour |
| 1.7 | Implement Stage 3 in `camera_task` | Deinit → SetSensorROI → Init(roi_w,roi_h) → CallbackBatchSnap → Send to storage | 2 hours |
| 1.8 | Implement fallback & restore | Restore to 1296×972 standby after Stage 3 or skip | 30 min |
| 1.9 | Implement `__attribute__((section(".psram")))` | Ensure `detection_frame` is placed in PSRAM, not SRAM | 10 min |
| 1.10 | Compile test | Build without errors | 15 min |

**Verification**: Debug printf traces show the state transitions. `IF_Process()` result printed to UART.

### 🔲 Task 2: Extend SD Storage in `Inc/raw_image.h`

**Prerequisites**: Task 1

**Steps**:

| # | Step | Details | Effort |
|---|------|---------|--------|
| 2.1 | Add stage/roi/bbox fields to `sd_image_header_t` | Use reserved bytes, add typed struct members | 20 min |
| 2.2 | Update storage task to populate fields | In storage task, read g_stage_num, g_roi, g_bbox globals | 30 min |
| 2.3 | Update SD naming or metadata ID | Map event_id + stage to unique file naming | 15 min |
| 2.4 | Build test | Verify SD files have correct metadata | 15 min |

**Verification**: Read saved files from SD, verify stage and bounding box metadata.

### 🔲 Task 3: Stage 1 Detection Validation

**Prerequisites**: Task 1

**Steps**:

| # | Step | Details | Effort |
|---|------|---------|--------|
| 3.1 | Capture 10 test images | Use current capture mode to get 10× 640×480 frames (or use Python script) | 1 hour |
| 3.2 | Process with Python prototype | Run `Image_filter.py` on each, record classification + bbox | 30 min |
| 3.3 | Process with C implementation | Build test harness or use printf to compare | 1 hour |
| 3.4 | Fix discrepancies | If classification differs, tune thresholds or fix algorithm | 2 hours |

**Verification**: ≥90 % classification match, bbox within ±1 grid cell (8 px).

### 🔲 Task 4: Stage 3 AREA3 Validation

**Prerequisites**: Task 1

**Steps**:

| # | Step | Details | Effort |
|---|------|---------|--------|
| 4.1 | Verify sensor register readback | After CAM_SetSensorROI(), read back AREA3/VMAX/SHUTTER via I2C, print to UART | 1 hour |
| 4.2 | Verify ROI frame dimensions | Captured frame at known ROI: check width/height match | 30 min |
| 4.3 | Test 10 consecutive ROI switches | Toggle AREA3 10×, verify no I2C errors, no hangs | 1 hour |
| 4.4 | Test edge-case ROI positions | Corner ROIs (0,0), (2000,1500), full-frame, minimal (64×64) | 1 hour |

**Verification**: 50 consecutive switches without failure. Captured frames at correct resolution.

### 🔲 Task 5: Full End-to-End Pipeline Test

**Prerequisites**: Tasks 1–4

**Steps**:

| # | Step | Details | Effort |
|---|------|---------|--------|
| 5.1 | Manual trigger test | Press button → verify Stage 1→2→3 sequence via UART | 30 min |
| 5.2 | SD card output check | Read SD, verify Stage 2 + Stage 3 files, correct dimensions | 30 min |
| 5.3 | Fallback test | Test with large background change → verify Stage 3 skipped | 30 min |
| 5.4 | Exposure/gain check | Compare exposure between Stage 2 and Stage 3 frames | 30 min |
| 5.5 | Timing measurement | Print total pipeline duration (trigger → all frames saved) | 15 min |

**Verification**: 10 consecutive successful pipelines, all frames on SD.

### 🔲 Task 6: ToF Trigger Integration

**Prerequisites**: Task 5

**Steps**:

| # | Step | Details | Effort |
|---|------|---------|--------|
| 6.1 | Replace test trigger command | Wire `CAM_CAPTURE_IMAGE` from ToF detection into new pipeline | 1 hour |
| 6.2 | Disable legacy pipeline | Remove or guard old single-stage capture code | 30 min |
| 6.3| Verify ToF→3-stage flow | ToF detects insect → Stage 1→2→3→SD → standby | 1 hour |

---

## 11. Implementation Order & Dependencies

```
Phase 1 (image_filter C port)           ✅ DONE
Phase 2 (AREA3 helpers)                 ✅ DONE
        │
        ▼
Task 1: Phase 3 state machine          ⬜ PENDING (CRITICAL PATH)
        │
        ├──▶ Task 2: SD metadata        ⬜ PENDING (needs Task 1's globals)
        ├──▶ Task 3: Detection validate ⬜ PENDING (needs Task 1 to run)
        ├──▶ Task 4: AREA3 validate     ⬜ PENDING (needs Task 1 to run)
        │
        ▼
Task 5: Full E2E test                  ⬜ PENDING (needs 1–4)
        │
        ▼
Task 6: ToF trigger integration        ⬜ PENDING (needs 5)
```

---

## 12. Risks & Mitigations

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| CAM_Init() after CAM_SetSensorROI() overwrites AREA3 registers | Medium | High | Test this immediately in Task 4. If CMW_CAMERA_Init() overwrites AREA3, we may need a `CAM_InitNoSensor()` variant that only sets up DCMIPP |
| Double-standby → streaming glitch | Low | Medium | The `CAM_Deinit()` → `CAM_SetSensorROI()` sequence does standby→hold→write→stream. If Deinit already left sensor in standby, the first step of SetSensorROI is a no-op. Verified in reference project. |
| detection_frame PSRAM placement | Low | High | Must use `__attribute__((section(".psram")))` or linker section. If placed in SRAM, will overflow 640KB SRAM. |
| batch_buf reuse across stages corrupts Stage 2 data | Low | High | Stage 3 batch_buf reuse only happens AFTER Stage 2 data is copied to SD. Safe by design. |
| AREA3 SHUTTER calc wrong for exposure | Medium | Medium | SHUTTER = VMAX − 100 formula from reference project. If ROI is very small (e.g., 64×64), VMAX = 164, SHUTTER = 64. At 30 FPS with HMAX=4500, this gives ~330 µs exposure. Automatically handled. |
| ToF triggers during pipeline | Low | Medium | Serialize in app_thread.c: ignore CAM_CAPTURE_IMAGE while not in STATE_IDLE. |

---

## 13. Key Code Patterns Reference

### Pattern 1: 640×480 Single-Frame Capture for Stage 1

```c
#include "image_filter.h"

static IF_Result_t     g_detection_result;
static uint8_t         detection_frame[IF_DETECT_FRAME_SZ]
    __attribute__((section(".psram")));

void capture_stage1(void)
{
    CAM_conf_t conf = {
        .capture_width = IF_DETECT_W,
        .capture_height = IF_DETECT_H,
        .fps = SNAP_FPS,
        .dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1,
        .is_rgb_swap = 0,
    };
    CAM_Init(&conf);

    DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();

    CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1
        ? CMW_EXPOSUREMODE_MANUAL : CMW_EXPOSUREMODE_AUTO);
    if (CAM_EXPOSURE_MODE == 1) {
        CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
        CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
    }

    CAM_CapturePipe_Start(detection_frame, CMW_MODE_CONTINUOUS);

    CAM_WaitNextFrameReady(200, NULL);  // warmup
    CAM_WaitNextFrameReady(200, NULL);  // warmup
    CAM_WaitNextFrameReady(200, NULL);  // capture

    HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);

    IF_Process(detection_frame, &g_detection_result);
}
```

### Pattern 2: AREA3 ROI for Stage 3

```c
void capture_stage3(uint16_t roi_x, uint16_t roi_y,
                   uint16_t roi_w, uint16_t roi_h)
{
    CAM_Deinit();
    HAL_Delay(10);

    if (CAM_SetSensorROI(roi_x, roi_y, roi_w, roi_h) != 0) {
        printf("[CAPTURE] Stage 3: AREA3 failed, skipping\n");
        return;
    }

    // DCMIPP pipe with ROI dimensions
    extern uint8_t batch_buf[];
    size_t frame_size = (size_t)roi_w * (size_t)roi_h * 2;
    size_t batch_size = frame_size * CALLBACK_FRAMES;

    CAM_CallbackInit(batch_buf, batch_size, roi_w, roi_h, SNAP_FPS);
    int n = CAM_CallbackBatchSnap(batch_buf, frame_size);
    // ... save n frames
}
```

### Pattern 3: Fallback Check

```c
if (g_detection_result.has_insect) {
    uint16_t rx, ry, rw, rh;
    if (IF_CalcROI(&g_detection_result, &rx, &ry, &rw, &rh) == 0) {
        capture_stage3(rx, ry, rw, rh);
    } else {
        printf("[CAPTURE] Insect too large for ROI — saving Stage 2 only\n");
    }
}
```

---

## 14. First Compile Checklist

Before the first build of the integrated pipeline:

- [ ] `detection_frame[]` placed in PSRAM section
- [ ] `IF_Init()` called once at boot
- [ ] `image_filter.h` included in `app_thread.c`
- [ ] New `capture_state` variable added
- [ ] State transitions in correct order
- [ ] `CAM_WaitNextFrameReady()` accessible (or made non-static)
- [ ] `batch_buf` large enough for max ROI frames
- [ ] `extern IF_Result_t g_detection_result;` in app_thread.c
- [ ] No duplicate `CAM_Deinit()` / `CAM_Init()` calls
- [ ] Exposure re-applied after each CAM_Init()
- [ ] Event counter increments per trigger
