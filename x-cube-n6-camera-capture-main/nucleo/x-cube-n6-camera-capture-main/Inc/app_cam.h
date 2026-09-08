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
 ******************************************************************************
 */
#ifndef APP_CAM_H
#define APP_CAM_H

#include <stdint.h>

/* ================================================================
   CAMERA CONFIGURATION STRUCTURE
   ================================================================ */

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

int CAM_Init(CAM_conf_t *conf);
int CAM_CapturePipe_Start(uint8_t *capture_pipe_dst, uint32_t cam_mode);
void CAM_IspUpdate(void);
void CAM_Deinit(void);

/* ================================================================
   PUBLIC API — STANDALONE SINGLE-FRAME CAPTURE
   ================================================================ */

int CAM_CaptureSingleFrame(uint8_t *buf, int buf_size, int width, int height, int fps, int warmup_frames);
int CAM_CaptureSingleFrame_DefaultWarmup(uint8_t *buf, int buf_size, int width, int height, int fps);

/* ================================================================
   PUBLIC API — CONTINUOUS MODE (CAPTURE_MODE = 1)
   ================================================================ */

int CAM_ContinuousStart(uint8_t *buf, int buf_size, int width, int height, int fps);
int CAM_ContinuousSnap(uint8_t *dest_buf, uint32_t frame_size);
int CAM_ContinuousStop(void);

/* ================================================================
   PUBLIC API — BATCH CAPTURE MODE (CAPTURE_MODE = 2)
   ================================================================ */

int CAM_CaptureBatchFrames(uint8_t *batch_buf, int frame_size, int frame_count,
                           int width, int height, int fps);
int CAM_ContinuousBatchSnap(uint8_t *batch_buf, uint32_t frame_size);

/* ================================================================
   PUBLIC API — CALLBACK-BATCH MODE (CAPTURE_MODE = 4)
   ================================================================ */

/** Initialize Mode 4 once at boot and park the IMX335 in standby. */
int CAM_CallbackInit(uint8_t *buf, int buf_size, int width, int height, int fps);

/**
 * Wake from standby, capture CALLBACK_FRAMES continuously with zero-copy DMA,
 * stop the pipe once, and return to standby.
 */
int CAM_CallbackBatchSnap(uint8_t *batch_buf, uint32_t frame_size);

/**
 * Return the HAL tick at which the requested kept Mode-4 frame was observed
 * complete by the camera task (immediately after the DCMIPP frame event).
 * This performs no I2C transaction and is used only to attach per-frame
 * timing metadata after the batch has finished.
 *
 * @param index  Kept frame index, 0..CALLBACK_FRAMES-1
 * @return HAL_GetTick() value for that frame, or 0 if unavailable/invalid.
 */
uint32_t CAM_GetCallbackFrameTick(uint32_t index);

int CAM_IsCallbackReady(void);

/* Called by CMW frame callback when a full frame transfer is complete. */
void CAM_NotifyFrameEvent(void);

/* ================================================================
   PUBLIC API — FRAME COUNTER
   ================================================================ */

void CAM_CountVsyncFrame(void);
void CAM_ResetFrameCounter(int wait_frames);
uint32_t CAM_GetFrameCount(void);

#endif /* APP_CAM_H */
