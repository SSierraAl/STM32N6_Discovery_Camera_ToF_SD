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
void CAM_CapturePipe_Start(uint8_t *capture_pipe_dst, uint32_t cam_mode);
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
   PUBLIC API — FRAME COUNTER (used by vsync IRQ handler)
   ================================================================ */

void CAM_CountVsyncFrame(void);
void CAM_ResetFrameCounter(int wait_frames);
uint32_t CAM_GetFrameCount(void);

#endif /* APP_CAM_H */