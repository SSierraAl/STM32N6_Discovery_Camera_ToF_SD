/**
 ******************************************************************************
 * @file    app_cam.h
 * @author  GPM Application Team (adapted for standalone snapshot mode)
 *
 * @brief   Camera abstraction layer for the CMW-IMX335 sensor on NUCLEO-N657X0.
 *
 *   This module wraps the Camera Middleware (CMW) and DCMIPP hardware to
 *   provide simple init/capture/deinit functions. It is used by both the
 *   legacy UVC pipeline (when app_run() is called) and the standalone
 *   snapshot capture (CAM_CaptureSingleFrame).
 *
 *   The IMX335 sensor outputs YUV422 frames via the CSI-2 interface. The
 *   DCMIPP ISP processes and converts the pixel data into a user-selectable
 *   format before DMA transfer into PSRAM.
 ******************************************************************************
 */
#ifndef APP_CAM_H
#define APP_CAM_H

#include <stdint.h>

/* ================================================================
   CAMERA CONFIGURATION STRUCTURE
   ================================================================ */

/**
 * @brief Camera initialization parameters.
 *
 *   Passed to CAM_Init() to configure resolution, frame rate, and
 *   pixel format. The DCMIPP ISP is programmed accordingly.
 */
typedef struct {
    int capture_width;
    int capture_height;
    int fps;
    int dcmipp_output_format;
    int is_rgb_swap;
} CAM_conf_t;

/* ================================================================
   PUBLIC API — BASIC CAMERA CONTROL
   ================================================================ */

/** Returns 0 on success, -1 on failure */
int CAM_Init(CAM_conf_t *conf);
int CAM_CapturePipe_Start(uint8_t *capture_pipe_dst, uint32_t cam_mode);
void CAM_IspUpdate(void);
void CAM_Deinit(void);

/* ================================================================
   PUBLIC API — STANDALONE SINGLE-FRAME CAPTURE
   ================================================================ */

/**
 * @brief  Capture a single frame (full init + warmup + capture + deinit cycle).
 *
 * @param  buf            Pre-allocated buffer (>= width*height*2 bytes for YUV422)
 * @param  buf_size       Size of buf in bytes
 * @param  width          Capture width (e.g. 1024)
 * @param  height         Capture height (e.g. 768)
 * @param  fps            Sensor frame rate (e.g. 30)
 * @param  warmup_frames  Number of frames to discard before saving (default: 8 if <=0)
 * @return 0 on success, -1 on timeout or buffer too small
 */
int CAM_CaptureSingleFrame(uint8_t *buf, int buf_size, int width, int height, int fps, int warmup_frames);

/**
 * @brief  Backward-compatible wrapper with default warmup = 8 frames.
 */
int CAM_CaptureSingleFrame_DefaultWarmup(uint8_t *buf, int buf_size, int width, int height, int fps);

/* ================================================================
   PUBLIC API — CONTINUOUS MODE (CAPTURE_MODE = 1)
   Camera always running, snapshot on trigger with zero warmup delay.
   ================================================================ */

/**
 * @brief  Start continuous camera capture (init + warmup done once at boot).
 *
 * @param  buf            Pre-allocated buffer (>= width*height*2 bytes for YUV422)
 * @param  buf_size       Size of buf in bytes
 * @param  width          Capture width (e.g. 2592)
 * @param  height         Capture height (e.g. 1944)
 * @param  fps            Sensor frame rate (e.g. 30)
 * @return 0 on success, -1 on timeout or buffer too small
 */
int CAM_ContinuousStart(uint8_t *buf, int buf_size, int width, int height, int fps);

/**
 * @brief  Snap the current frame from the continuous capture buffer.
 *
 *   Atomically: Stop DCMIPP pipe → memcpy to dest_buf → Restart pipe.
 *   This prevents buffer corruption (camera never writes while we copy).
 *
 * @param  dest_buf       Destination buffer (>= width*height*2 bytes)
 * @param  frame_size     Exact frame size in bytes (width*height*2)
 * @return 0 on success, -1 on error
 */
int CAM_ContinuousSnap(uint8_t *dest_buf, uint32_t frame_size);

/**
 * @brief  Stop continuous capture and deinit camera (shutdown).
 */
int CAM_ContinuousStop(void);

/* ================================================================
   PUBLIC API — BATCH CAPTURE MODE (CAPTURE_MODE = 2)
   One init+warmup+deinit cycle, N consecutive frames captured rapidly.
   ================================================================ */

/**
 * @brief  Capture multiple frames with ONE init+warmup+deinit cycle.
 *
 *   Strategy:
 *     1. Init camera (once)
 *     2. Start continuous capture
 *     3. Warmup: discard SNAP_WARMUP_FRAMES (once)
 *     4. Capture frame_count consecutive frames (~33ms each at 30fps)
 *     5. Stop & deinit camera (once)
 *
 *   Much faster than calling CAM_CaptureSingleFrame N times:
 *     CAM_CaptureBatchFrames:  ~400ms + N×33ms
 *     N×CAM_CaptureSingleFrame: N×(~664ms)
 *
 * @param  batch_buf    Pre-allocated buffer ≥ frame_count × frame_size bytes
 * @param  frame_size   Size of one frame in bytes (width*height*2)
 * @param  frame_count  Number of frames to capture (e.g., 3)
 * @param  width        Capture width
 * @param  height       Capture height
 * @param  fps          Sensor frame rate
 * @return number of frames captured (0..frame_count), -1 on error
 */
int CAM_CaptureBatchFrames(uint8_t *batch_buf, int frame_size, int frame_count,
                            int width, int height, int fps);

/**
 * @brief  Capture BATCH_FRAMES quickly from continuous camera into batch_buf.
 *
 *   For each frame: stop pipe → copy → restart pipe → wait 1 frame.
 *   LEDs should stay ON during this entire call.
 *   Only used when camera is already running in continuous mode.
 *
 * @param  batch_buf    Pre-allocated buffer ≥ BATCH_FRAMES × frame_size bytes
 * @param  frame_size   Size of one frame in bytes (width*height*2)
 * @return number of frames captured (0..BATCH_FRAMES), -1 on error
 */
int CAM_ContinuousBatchSnap(uint8_t *batch_buf, uint32_t frame_size);

/* ================================================================
   PUBLIC API — CALLBACK-BATCH MODE (CAPTURE_MODE = 4)
   Callback-driven continuous batch capture — NO Stop/Restart between frames.

   Uses the DCMIPP frame event callback (ISR-driven, g_frame_event_count)
   to know EXACTLY when each frame is DMA'd. The pipe runs continuously
   through the entire batch, eliminating the Stop/Restart tearing that
   causes blurry frames in STANDBY-BATCH mode.

   Flow on trigger:
     Wake sensor (20ms) → Start continuous pipe → Warmup via callback →
     For each frame: wait for callback → memcpy → repeat →
     Stop pipe ONCE → Return to standby

   ALL frames are sharp because the DCMIPP pipe never restarts mid-batch.
   ================================================================ */

/**
 * @brief  Initialize camera and put in standby mode (called ONCE at boot).
 *
 *   Same as CAM_StandbyInit: full init + warmup + standby.
 *
 * @param  buf            Pre-allocated buffer for capture
 * @param  buf_size       Size of buf in bytes
 * @param  width          Capture width
 * @param  height         Capture height
 * @param  fps            Sensor frame rate
 * @return 0 on success, -1 on error
 */
int CAM_CallbackInit(uint8_t *buf, int buf_size, int width, int height, int fps);

/**
 * @brief  Wake from standby, capture CALLBACK_FRAMES continuously, return to standby.
 *
 *   KEY DIFFERENCE from CAM_StandbyBatchSnap:
 *   - Does NOT Stop/Restart the DCMIPP pipe between frames
 *   - Uses frame event callback (g_frame_event_count, ISR-driven) to know
 *     exactly when each frame's DMA is complete
 *   - Copies each frame to batch_buf immediately after callback fires
 *   - Stops the pipe ONLY ONCE after all frames are captured
 *
 *   This eliminates the Stop/Restart tearing that caused frames 2&3 to be
 *   blurry in STANDBY-BATCH mode. All frames come from a continuously
 *   running, perfectly synchronized DCMIPP pipeline.
 *
 * @param  batch_buf    Pre-allocated buffer ≥ CALLBACK_FRAMES × frame_size
 * @param  frame_size   Size of one frame in bytes
 * @return number of frames captured (0..CALLBACK_FRAMES), -1 on error
 */
int CAM_CallbackBatchSnap(uint8_t *batch_buf, uint32_t frame_size);
/** Rectangle in native pixels of the oriented image, with exclusive right/bottom edges. */
typedef struct {
  uint16_t x, y, width, height;
} CAM_BurstWindow;

/**
 * @brief Locate a target using the project-supplied detector.
 * @param source Read-only, tightly packed native YUYV frame.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @param scratch Temporary CPU workspace, separate from source.
 * @param scratch_bytes Workspace capacity in bytes.
 * @param box Output rectangle before adding any crop margin.
 * @return 1 for a box, 0 for no target, negative for a detector error.
 */
int CAM_DetectExternal(const uint8_t *source, uint16_t width, uint16_t height,
                       uint8_t *scratch, uint32_t scratch_bytes, CAM_BurstWindow *box);

/**
 * @brief Wake, warm up, capture one native YUYV frame, and return to standby.
 * @param buf 32-byte-aligned PSRAM destination for camera DMA.
 * @param size Destination capacity in bytes.
 * @param tick Output HAL tick in ms at snapshot completion, valid on success.
 * @return 1 when a frame is ready for CPU reads, 0 on failure.
 */
int CAM_CaptureFullFrame(uint8_t *buf, uint32_t size, uint32_t *tick);

/**
 * @brief Locate optical foreground and prepare a native crop window.
 * @param source Read-only native YUYV frame after camera DMA has stopped.
 * @param scratch CPU workspace (at least CAM_BURST_CROP_BYTES, aligned to 32 bytes).
 * @param capacity Workspace and eventual crop output capacity in bytes.
 * @param object Foreground box before crop margin, or zero when unavailable.
 * @param window Padded YUYV window, or zero when unavailable or oversized.
 * @param components Output number of confirmed components (zero when unknown).
 * @return Static diagnostic string describing the localization outcome.
 */
const char *CAM_LocalizeFrame(const uint8_t *source, uint8_t *scratch, uint32_t capacity,
                              CAM_BurstWindow *object, CAM_BurstWindow *window, uint32_t *components);

/** Return HAL tick recorded when a kept Mode-4 frame completed. */
uint32_t CAM_GetCallbackFrameTick(uint32_t index);

/**
 * @brief  Check if camera is initialized and ready for callback batch.
 * @return 1 if ready, 0 if not
 */
int CAM_IsCallbackReady(void);

/* Called by CMW frame callback when a full frame transfer is complete. */
void CAM_NotifyFrameEvent(void);

/* ================================================================
   PUBLIC API — FRAME COUNTER (used by vsync IRQ handler)
   ================================================================ */

void CAM_CountVsyncFrame(void);
void CAM_ResetFrameCounter(int wait_frames);
uint32_t CAM_GetFrameCount(void);

/* ================================================================
   MULTI-RESOLUTION / ROI API  (Stage 1 + Stage 3)
   ================================================================ */

/**
 * @brief  Multi-byte register write to IMX335 via I2C (16-bit address).
 * @param  reg    16-bit register address (e.g. 0x3074)
 * @param  data   Pointer to data bytes (little-endian)
 * @param  len    Number of bytes to write (1..4)
 * @return 0 on success, -1 on error
 */
int CAM_WriteSensorRegMulti(uint16_t reg, const uint8_t *data, uint16_t len);

/**
 * @brief  Reconfigure IMX335 output window via AREA3 registers.
 *         Sequence: STANDBY → HOLD → write AREA3 + VMAX + SHUTTER → clear HOLD → STREAMING.
 *         After this call, the sensor outputs only (width × height) pixels.
 *         Caller must re-init DCMIPP with matching dimensions.
 *
 * @param  x_start  Window X start (0–2591, even)
 * @param  y_start  Window Y start (0–1943, even)
 * @param  width    Window width (16–2592, even)
 * @param  height   Window height (4–1944, even)
 * @return 0 on success, -1 on error
 */
int CAM_SetSensorROI(uint16_t x_start, uint16_t y_start,
                     uint16_t width, uint16_t height);

/**
 * @brief  Convenience: set sensor to full-frame output (2592×1944).
 *         Equivalent to CAM_SetSensorROI(0, 0, 2592, 1944).
 */
int CAM_SetFullFrame(void);

/**
 * @brief  Put camera sensor into low-power standby mode (writes 0x01 to 0x3000).
 *         The sensor stays powered; call CAM_CallbackBatchSnap to wake it again.
 */
void CAM_Standby(void);

/**
 * @brief  Reconfigure DCMIPP pipe for a given resolution on-the-fly.
 *         Uses PipeState reset (no DeInit→Init) — call only when pipe is stopped.
 *         After this, CAM_CallbackBatchSnap uses the new dimensions.
 *         Returns 0 on success, -1 on error.
 */
int CAM_SetPipeResolution(uint16_t width, uint16_t height,
                           uint16_t offset_x, uint16_t offset_y);

/**
 * @brief  Reset callback-ready flag so the next CAM_CallbackBatchSnap
 *         re-initializes the camera pipeline cleanly (after Stage 3's
 *         DeInit→Init). Call between capture cycles in camera_task.
 */
void CAM_ResetCallbackReady(void);

/**
 * @brief  Stage 3 AREA3 ROI capture.
 *         Sensor enters standby → AREA3 → DCMIPP init → warmup → stop →
 *         full-frame → standby. Buffer must hold roi_w*roi_h*2 bytes.
 *         Returns captured frame count (>0) on success, -1 on error.
 */
int CAM_CaptureROI(uint8_t *buf, uint32_t buf_size,
                   uint16_t roi_x, uint16_t roi_y,
                   uint16_t roi_w, uint16_t roi_h);

#endif /* APP_CAM_H */
