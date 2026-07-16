/**
 * ******************************************************************************
 * @file    app_cam.c
 * @author  GPM Application Team
 *
 * ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * ******************************************************************************
 */
#include <assert.h>
#include "cmw_camera.h"
#include "app_cam.h"
#include "app_config.h"
#include "perf_debug.h"
#include "debug_color.h"
#include "stm32n6xx.h"
#include "stm32n6xx_hal.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"

/* External reference to shared perf timer */
extern PerfTimer_t g_perf_timer;

/* External reference to capture buffer (defined in main.c) */
extern uint8_t capture_buf[];

/* Define sensor orientation */
#if CAMERA_SELFY == 1
#define SENSOR_IMX335_FLIP CMW_MIRRORFLIP_MIRROR
#define SENSOR_VD66GY_FLIP CMW_MIRRORFLIP_FLIP
#define SENSOR_VD55G1_FLIP CMW_MIRRORFLIP_FLIP
#define SENSOR_VD1943_FLIP CMW_MIRRORFLIP_MIRROR
#else
#define SENSOR_IMX335_FLIP CMW_MIRRORFLIP_NONE
#define SENSOR_VD66GY_FLIP CMW_MIRRORFLIP_FLIP_MIRROR
#define SENSOR_VD55G1_FLIP CMW_MIRRORFLIP_FLIP_MIRROR
#define SENSOR_VD1943_FLIP CMW_MIRRORFLIP_NONE
#endif

/* Define sensor width x height size. 0x0 means full frame */
#define SENSOR_WIDTH     0
#define SENSOR_HEIGHT    0

static const char *sensor_names[] = {
  "CMW_UNKNOWN",
  "CMW_VD66GY",
  "CMW_IMX335",
  "CMW_VD55G1",
  "CMW_VD1943",
};

static CMW_Sensor_Name_t sensor;
static int is_sensor_valid = 0;

static int CAM_getFlipMode(CMW_Sensor_Name_t sensor)
{
  int sensor_mirror_flip = CMW_MIRRORFLIP_NONE;
  int sensor_name_idx = 0;

  switch (sensor) {
  case CMW_VD66GY_Sensor:
    sensor_mirror_flip = SENSOR_VD66GY_FLIP;
    sensor_name_idx = 1;
    break;
  case CMW_IMX335_Sensor:
    sensor_mirror_flip = SENSOR_IMX335_FLIP;
    sensor_name_idx = 2;
    break;
  case CMW_VD55G1_Sensor:
    sensor_mirror_flip = SENSOR_VD55G1_FLIP;
    sensor_name_idx = 3;
    break;
  case CMW_VD1943_Sensor:
    sensor_mirror_flip = SENSOR_VD1943_FLIP;
    sensor_name_idx = 4;
    break;
  default:
    assert(0);
  }
  printf("Detected %s\n", sensor_names[sensor_name_idx]);

  return sensor_mirror_flip;
}
// MARK: Camera capture functions
static int CAM_FormatToBpp(int dcmipp_output_format)
{
  int bpp = 0;

  switch (dcmipp_output_format)
  {
  case DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1:
    bpp = 1;
    break;
  case DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1:
  case DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1:
    bpp = 2;
    break;
  case DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1:
    bpp = 3;
    break;
  default:
    assert(0);
  }

  return bpp;
}

/* Keep display output aspect ratio using crop area */
static void CAM_InitCropConfig(CMW_Manual_roi_area_t *roi, int sensor_width, int sensor_height, CAM_conf_t *conf)
{
  const float ratiox = (float)sensor_width / conf->capture_width;
  const float ratioy = (float)sensor_height / conf->capture_height;
  const float ratio = MIN(ratiox, ratioy);

  assert(ratio >= 1);
  assert(ratio < 64);

  roi->width = (uint32_t) MIN(conf->capture_width * ratio, sensor_width);
  roi->height = (uint32_t) MIN(conf->capture_height * ratio, sensor_height);
  roi->offset_x = (sensor_width - roi->width + 1) / 2;
  roi->offset_y = (sensor_height - roi->height + 1) / 2;
}

static void CAM_EnableYuv(uint32_t Pipe)
{
  DCMIPP_ColorConversionConfTypeDef color_conf = {
    .ClampOutputSamples = ENABLE,
    .OutputSamplesType = DCMIPP_CLAMP_YUV,
    .RR = 131, .RG = -119, .RB = -12, .RA = 128,
    .GR =  55, .GG =  183, .GB =  18, .GA =   0,
    .BR = -30, .BG = -101, .BB = 131, .BA = 128,
  };
  int ret;

  /* only pipe1 can do yuv */
  assert(Pipe == DCMIPP_PIPE1);
  ret = HAL_DCMIPP_PIPE_SetYUVConversionConfig(CMW_CAMERA_GetDCMIPPHandle(), Pipe, &color_conf);
  assert(ret == HAL_OK);
  ret = HAL_DCMIPP_PIPE_EnableYUVConversion(CMW_CAMERA_GetDCMIPPHandle(), Pipe);
  assert(ret == HAL_OK);
}

static void DCMIPP_PipeInitCapture(CAM_conf_t *cam_conf, int sensor_width, int sensor_height, CAM_conf_t *conf)
{
  CMW_DCMIPP_Conf_t dcmipp_conf;
  uint32_t hw_pitch;
  int ret;

  assert(conf->capture_width >= conf->capture_height);

  dcmipp_conf.output_width = conf->capture_width;
  dcmipp_conf.output_height = conf->capture_height;
  dcmipp_conf.output_format = cam_conf->dcmipp_output_format;
  dcmipp_conf.output_bpp = CAM_FormatToBpp(cam_conf->dcmipp_output_format);
  dcmipp_conf.mode = CMW_Aspect_ratio_manual_roi;
  dcmipp_conf.enable_swap = cam_conf->is_rgb_swap;
  dcmipp_conf.enable_gamma_conversion = 0;
  CAM_InitCropConfig(&dcmipp_conf.manual_conf, sensor_width, sensor_height, conf);
  ret = CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dcmipp_conf, &hw_pitch);
  assert(ret == HAL_OK);
  assert(hw_pitch == dcmipp_conf.output_width * dcmipp_conf.output_bpp);

  if (cam_conf->dcmipp_output_format == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
    CAM_EnableYuv(DCMIPP_PIPE1);
}

int CAM_Init(CAM_conf_t *conf)
{
  CMW_CameraInit_t cam_conf;
  int ret;

  if (!is_sensor_valid) {
    is_sensor_valid = 1;
    ret = CMW_CAMERA_GetSensorName(&sensor);
    assert(ret == CMW_ERROR_NONE);
  }

  cam_conf.width = SENSOR_WIDTH;
  cam_conf.height = SENSOR_HEIGHT;
  cam_conf.fps = conf->fps;
  cam_conf.mirror_flip = CAM_getFlipMode(sensor);

  ret = CMW_CAMERA_Init(&cam_conf, NULL);
  assert(ret == CMW_ERROR_NONE);

  /* CMW_CAMERA_Init update width height */
  assert(cam_conf.width);
  assert(cam_conf.height);
  DCMIPP_PipeInitCapture(conf, cam_conf.width, cam_conf.height, conf);
}

void CAM_SetSliceROI(uint16_t y_start, uint16_t height)
{
  CMW_DCMIPP_Conf_t conf;
  /* Reuse existing pipe config, change only manual_conf.offset_y and height */
  conf.manual_conf.offset_y = y_start;
  conf.manual_conf.height = height;
  CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &conf, NULL);
}


void CAM_CapturePipe_Start(uint8_t *capture_pipe_dst, uint32_t cam_mode)
{
  int ret;

  ret = CMW_CAMERA_Start(DCMIPP_PIPE1, capture_pipe_dst, cam_mode);
  assert(ret == CMW_ERROR_NONE);
}

void CAM_IspUpdate(void)
{
  int ret;

  ret = CMW_CAMERA_Run();
  assert(ret == CMW_ERROR_NONE);
}

void CAM_Deinit()
{
  int ret;

  ret = CMW_CAMERA_DeInit();
  assert(ret == HAL_OK);
}

void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe)
{
  /* FIXME : Need to tune sensor/ipplug so we can remove this implementation */
}

/* ---------- Frame counter for standalone warmup ----------
   The vsync interrupt fires once per captured frame. We count frames
   here so we can discard exactly N warmup frames instead of guessing
   with a time-based delay. */
static volatile uint32_t g_frame_count = 0;
/* ISR/task-shared countdown; updated from VSYNC callback and polled by tasks. */
static volatile int      g_wait_frames = 0;  /* target: frames to wait before done */

/**
 * @brief Called from CMW_CAMERA_PIPE_VsyncEventCallback on each frame completion.
 *        Increments the frame counter; when enough frames arrive, sets g_wait_frames = 0. */
void CAM_CountVsyncFrame(void)
{
  if (g_wait_frames > 0) {
    g_frame_count++;
    if (g_frame_count >= (uint32_t)g_wait_frames)
      g_wait_frames = 0;  /* signal: warmup done */
  }
}

/**
 * @brief Reset the frame counter and set the target number of frames to wait.
 *        Call this before starting a standalone capture. */
void CAM_ResetFrameCounter(int wait_frames)
{
  g_frame_count = 0;
  g_wait_frames = wait_frames;
}

/**
 * @brief Get the current frame count (for debugging). */
uint32_t CAM_GetFrameCount(void)
{
  return g_frame_count;
}

/**
 * @brief Wait for a number of VSYNC frames, with timeout.
 * @param frame_count Number of frames to wait for.
 * @param timeout_ms  Timeout in milliseconds.
 * @param poll_ticks  Delay between polls.
 * @return 0 on success, -1 on timeout.
 */
static int CAM_WaitFrames(int frame_count, uint32_t timeout_ms, TickType_t poll_ticks)
{
  uint32_t deadline = HAL_GetTick() + timeout_ms;
  CAM_ResetFrameCounter(frame_count);
  while (g_wait_frames != 0) {
    CAM_IspUpdate();
    vTaskDelay(poll_ticks);
    uint32_t now = HAL_GetTick();
    /* Signed delta comparison keeps timeout check valid across tick wraparound. */
    if ((int32_t)(now - deadline) >= 0) {
      return -1;
    }
  }
  return 0;
}

/**
 * @brief Capture a single frame WITHOUT UVC streaming.
 *
 * Strategy (frame-counted warmup, NOT time-based):
 *   1. Init camera sensor + DCMIPP pipe (YUV422, requested resolution)
 *   2. Start CONTINUOUS capture into buf (each frame overwrites buf)
 *   3. Wait for exactly `warmup_frames + 1' frame completions:
 *      - `warmup_frames' are discarded (sensor stabilization)
 *      - the final frame is kept in buf
 *   4. Stop DCMIPP (buf holds the clean, stabilized frame)
 *   5. Deinit camera (power down sensor + DCMIPP)
 *
 * @param buf            Pre-allocated buffer (≥ width*height*2 bytes)
 * @param buf_size       Size of buf in bytes
 * @param width          Capture width  (e.g. 640)
 * @param height         Capture height (e.g. 480)
 * @param fps            Sensor frame-rate (e.g. 30)
 * @param warmup_frames  Number of frames to discard before keeping one (default: 8)
 * @return 0 on success, -1 on error (timeout)
 */
int CAM_CaptureSingleFrame(uint8_t *buf, int buf_size, int width, int height, int fps, int warmup_frames)
{
  /* ---- sanity checks ---- */
  int min_size = width * height * 2;  /* YUV422 = 2 bpp */

#if PERF_DEBUG_LEVEL >= 2
  printf("[CAM] Enter: %dx%d, min_buf=%d, avail=%d\n", width, height, min_size, buf_size);
#endif

  if (buf == NULL || buf_size <= 0) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] FAIL: buf NULL or size 0\n");
#endif
    return -1;
  }
  if (buf_size < min_size) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] FAIL: buffer too small (%d < %d)\n", buf_size, min_size);
#endif
    return -1;
  }
  if (warmup_frames < 1) warmup_frames = 8;

  /* ---- PERF: Mark camera init start ---- */
  PERF_MARK(g_perf_timer, CAM_INIT);

  /* ---- 1. Init camera ---- */
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Init camera %dx%d@%d YUV422 ...\n", width, height, fps);
#endif

  CAM_conf_t conf = {0};
  conf.capture_width        = width;
  conf.capture_height       = height;
  conf.fps                  = fps;
  conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
  conf.is_rgb_swap          = 0;

  CAM_Init(&conf);

  /* ---- PERF: Mark exposure config ---- */
  PERF_MARK(g_perf_timer, CAM_EXPO);

  /* ---- Apply camera quality settings from app_config.h ----
     CRITICAL: Must be AFTER CAM_Init() so sensor defaults don't override */
  int32_t ret_expo_mode;

  ret_expo_mode = CMW_CAMERA_SetExposureMode(
      CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL :
      CAM_EXPOSURE_MODE == 2 ? CMW_EXPOSUREMODE_AUTOFREEZE :
                               CMW_EXPOSUREMODE_AUTO);

  if (CAM_EXPOSURE_MODE == 1) {
    int32_t ret_expo = CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    int32_t ret_gain = CMW_CAMERA_SetGain(CAM_GAIN_VALUE);

    /* Read back to verify */
    int32_t readback_expo, readback_gain;
    CMW_CAMERA_GetExposure(&readback_expo);
    CMW_CAMERA_GetGain(&readback_gain);

#if PERF_DEBUG_LEVEL >= 2
    printf("[CAM] SetExposure(%ld)->rc=%ld, SetGain(%ld)->rc=%ld\n",
           (long)CAM_EXPOSURE_VALUE, (long)ret_expo,
           (long)CAM_GAIN_VALUE, (long)ret_gain);
    printf("[CAM] Readback: exposure=%ld, gain=%ld\n", (long)readback_expo, (long)readback_gain);
#elif PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Exposure=%ld Gain=%ld (rc=%ld)\n",
           (long)readback_expo, (long)readback_gain, (long)ret_expo);
#endif
  } else {
#if PERF_DEBUG_LEVEL >= 2
    printf("[CAM] SetExposureMode=%ld rc=%ld\n", (long)CAM_EXPOSURE_MODE, (long)ret_expo_mode);
#endif
  }

#if PERF_DEBUG_LEVEL >= 2
  printf("[CAM] Camera init done\n");
#endif

  DCMIPP_HandleTypeDef *hhandle = CMW_CAMERA_GetDCMIPPHandle();

  /* ---- PERF: Mark warmup start ---- */
  PERF_MARK(g_perf_timer, CAM_WARMUP);

  /* Reset frame counter */
  CAM_ResetFrameCounter(warmup_frames + 1);

  /* ---- 2. Start CONTINUOUS capture into buf ---- */
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Start continuous capture (warmup=%d + 1)...\n", warmup_frames);
#endif

  CAM_CapturePipe_Start(buf, CMW_MODE_CONTINUOUS);

#if PERF_DEBUG_LEVEL >= 3
  printf("[CAM] Capture started. Waiting %d frames...\n", warmup_frames + 1);
#endif

  /* ---- 3. Wait for enough frames ---- */
  uint32_t t0 = HAL_GetTick();
  while (g_wait_frames != 0) {
    CAM_IspUpdate();
    vTaskDelay(pdMS_TO_TICKS(5));

    if (HAL_GetTick() - t0 > 5000) {
#if PERF_DEBUG_LEVEL >= 1
      printf("[CAM] TIMEOUT! Got %lu/%d frames in %lu ms\n",
             (unsigned long)g_frame_count, warmup_frames + 1,
             (unsigned long)(HAL_GetTick() - t0));
#endif
      HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);
      CAM_Deinit();
      return -1;
    }
  }

#if PERF_DEBUG_LEVEL >= 2
  printf("[CAM] Got %lu frames in %lu ms (~%lu ms/frame)\n",
         (unsigned long)g_frame_count,
         (unsigned long)(HAL_GetTick() - t0),
         g_frame_count > 0 ? (HAL_GetTick() - t0) / g_frame_count : 0);
#endif

  /* ---- PERF: Mark frame captured ---- */
  PERF_MARK(g_perf_timer, CAM_SNAP);

  /* ---- 4. Stop DCMIPP — buf has the clean frame ---- */
  PERF_MARK(g_perf_timer, CAM_STOP);
  HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);
  HAL_Delay(5);

  /* ---- 5. Deinit camera ---- */
  PERF_MARK(g_perf_timer, CAM_DEINIT);

#if PERF_DEBUG_LEVEL >= 2
  printf("[CAM] Deinit camera ...\n");
#endif

  CAM_Deinit();

#if PERF_DEBUG_LEVEL >= 3
  printf("[CAM] Done. buf=%p size=%d\n", (void*)buf, min_size);
#endif

  return 0;
}

/* Backward-compatible wrapper with default warmup = 8 frames */
int CAM_CaptureSingleFrame_DefaultWarmup(uint8_t *buf, int buf_size, int width, int height, int fps)
{
  return CAM_CaptureSingleFrame(buf, buf_size, width, height, fps, 8);
}

/**
 * @brief Capture multiple frames with ONE init+warmup+deinit cycle.
 *
 *   Much faster than calling CAM_CaptureSingleFrame N times:
 *     - Single warmup (8 frames discarded once)
 *     - N consecutive frames captured rapidly (~33ms each at 30fps)
 *     - Single deinit at end
 *
 *   Total time: ~1 init + 8 warmup + N capture + 1 deinit ≈ 400ms + N×33ms
 *   vs N×CAM_CaptureSingleFrame: N×(400ms + 8×33ms) ≈ N×664ms
 *
 * @param batch_buf    Pre-allocated buffer ≥ frame_count × frame_size
 * @param frame_size   Size of one frame in bytes
 * @param frame_count  Number of frames to capture (e.g., 3)
 * @param width        Capture width
 * @param height       Capture height
 * @param fps          Sensor frame rate
 * @return number of frames captured (0..frame_count), -1 on error
 */
int CAM_CaptureBatchFrames(uint8_t *batch_buf, int frame_size, int frame_count,
                           int width, int height, int fps)
{
  int min_size = width * height * 2;  /* YUV422 = 2 bpp */
  if (batch_buf == NULL || frame_size < min_size || frame_count < 1) return -1;

  /* 1. Init camera */
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Batch init %dx%d@%d YUV422 (%d frames)...\n", width, height, fps, frame_count);
#endif

  CAM_conf_t conf = {0};
  conf.capture_width = width;
  conf.capture_height = height;
  conf.fps = fps;
  conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
  conf.is_rgb_swap = 0;
  CAM_Init(&conf);

  /* 2. Apply exposure */
  CMW_CAMERA_SetExposureMode(CMW_EXPOSUREMODE_MANUAL);
  CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
  CMW_CAMERA_SetGain(CAM_GAIN_VALUE);

  DCMIPP_HandleTypeDef *hhandle = CMW_CAMERA_GetDCMIPPHandle();

   /* 3. Start continuous capture */
   CAM_CapturePipe_Start(capture_buf, CMW_MODE_CONTINUOUS);

  /* Re-apply exposure after start */
  if (CAM_EXPOSURE_MODE == 1) {
      CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
      CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
      CAM_IspUpdate();
  }

  /* 4. Warmup: discard SNAP_WARMUP_FRAMES */
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Warmup: discarding %d frames...\n", SNAP_WARMUP_FRAMES);
#endif
  CAM_ResetFrameCounter(SNAP_WARMUP_FRAMES);
  uint32_t t0 = HAL_GetTick();
  while (g_wait_frames != 0) {
    CAM_IspUpdate();
    vTaskDelay(pdMS_TO_TICKS(5));
    if (HAL_GetTick() - t0 > 5000) {
#if PERF_DEBUG_LEVEL >= 1
      printf("[CAM] WARMUP TIMEOUT!\n");
#endif
      HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);
      CAM_Deinit();
      return -1;
    }
  }

  /* 5. Capture frame_count frames */
  int captured = 0;
  for (int i = 0; i < frame_count; i++) {
    /* Wait for one frame */
    CAM_ResetFrameCounter(1);
    t0 = HAL_GetTick();
    while (g_wait_frames != 0) {
      CAM_IspUpdate();
      vTaskDelay(pdMS_TO_TICKS(5));
      if (HAL_GetTick() - t0 > 500) break;
    }

    /* Copy to batch buffer */
    uint8_t *dest = batch_buf + (i * frame_size);
    SCB_InvalidateDCache_by_Addr((uint32_t *)capture_buf, frame_size);
    memcpy(dest, capture_buf, frame_size);
    SCB_CleanDCache_by_Addr((uint32_t *)dest, frame_size);
    captured++;
  }

  /* 6. Stop and deinit */
  HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);
  CAM_Deinit();

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Batch: %d/%d frames in %lu ms\n", captured, frame_count,
         (unsigned long)(HAL_GetTick() - t0));
#endif

  return captured;
}

/* ================================================================
   CONTINUOUS MODE IMPLEMENTATION (CAPTURE_MODE = 1)
   ================================================================ */

/**
 * @brief  Internal helper: Init camera + apply quality settings + warmup.
 *
 *   Shared between CAM_CaptureSingleFrame (on-demand) and
 *   CAM_ContinuousStart (continuous).  This function performs the
 *   sensor init, exposure/gain configuration, warmup discard, and
 *   leaves the capture pipe RUNNING in continuous mode.
 *
 * @param  buf            Pre-allocated buffer for capture
 * @param  buf_size       Size of buf in bytes
 * @param  width          Capture width
 * @param  height         Capture height
 * @param  fps            Sensor frame rate
 * @param  warmup_frames  Number of frames to discard (use 0 to skip warmup)
 * @return 0 on success, -1 on error
 */
static int CAM_InitAndStartContinuous(uint8_t *buf, int buf_size,
                                       int width, int height, int fps,
                                       int warmup_frames)
{
  int min_size = width * height * 2;

  if (buf == NULL || buf_size <= 0) return -1;
  if (buf_size < min_size) return -1;
  if (warmup_frames < 0) warmup_frames = 0;

  /* ---- 1. Init camera ---- */
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Init camera %dx%d@%d YUV422 ...\n", width, height, fps);
#endif

  CAM_conf_t conf = {0};
  conf.capture_width        = width;
  conf.capture_height       = height;
  conf.fps                  = fps;
  conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
  conf.is_rgb_swap          = 0;

  CAM_Init(&conf);

  /* ---- Apply camera quality settings ---- */
  int32_t ret_expo_mode;
  ret_expo_mode = CMW_CAMERA_SetExposureMode(
      CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL :
      CAM_EXPOSURE_MODE == 2 ? CMW_EXPOSUREMODE_AUTOFREEZE :
                               CMW_EXPOSUREMODE_AUTO);

  if (CAM_EXPOSURE_MODE == 1) {
    int32_t ret_expo = CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    int32_t ret_gain = CMW_CAMERA_SetGain(CAM_GAIN_VALUE);

    int32_t readback_expo, readback_gain;
    CMW_CAMERA_GetExposure(&readback_expo);
    CMW_CAMERA_GetGain(&readback_gain);

#if PERF_DEBUG_LEVEL >= 2
    printf("[CAM] SetExposure(%ld)->rc=%ld, SetGain(%ld)->rc=%ld\n",
           (long)CAM_EXPOSURE_VALUE, (long)ret_expo,
           (long)CAM_GAIN_VALUE, (long)ret_gain);
    printf("[CAM] Readback: exposure=%ld, gain=%ld\n", (long)readback_expo, (long)readback_gain);
#endif
  } else {
#if PERF_DEBUG_LEVEL >= 2
    printf("[CAM] SetExposureMode=%ld rc=%ld\n", (long)CAM_EXPOSURE_MODE, (long)ret_expo_mode);
#endif
  }

  DCMIPP_HandleTypeDef *hhandle = CMW_CAMERA_GetDCMIPPHandle();

  /* ---- 2. Start CONTINUOUS capture into buf ---- */
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Start continuous capture ...\n");
#endif

  CAM_CapturePipe_Start(buf, CMW_MODE_CONTINUOUS);

  /* ---- CRITICAL: Re-apply manual exposure AFTER ISP starts ----
     The ISP library's auto-exposure engine may overwrite sensor exposure
     during ISP_Start() / CAM_IspUpdate(). For MANUAL mode, we must force
     the exposure value again after the camera pipe is running. */
  if (CAM_EXPOSURE_MODE == 1) {
    int32_t ret_expo2 = CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    int32_t ret_gain2 = CMW_CAMERA_SetGain(CAM_GAIN_VALUE);

    /* Force ISP to process the new values */
    CAM_IspUpdate();

    int32_t readback_expo2, readback_gain2;
    CMW_CAMERA_GetExposure(&readback_expo2);
    CMW_CAMERA_GetGain(&readback_gain2);

#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Post-start exposure=%ld, gain=%ld (rc=%ld,%ld)\n",
           (long)readback_expo2, (long)readback_gain2, (long)ret_expo2, (long)ret_gain2);
#endif
  }

  /* ---- 3. Wait for warmup frames (if requested) ---- */
  if (warmup_frames > 0) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Warmup: discarding %d frames...\n", warmup_frames);
#endif
    CAM_ResetFrameCounter(warmup_frames);

    uint32_t t0 = HAL_GetTick();
    while (g_wait_frames != 0) {
      CAM_IspUpdate();
      vTaskDelay(pdMS_TO_TICKS(5));

      if (HAL_GetTick() - t0 > 5000) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[CAM] WARMUP TIMEOUT! Got %lu/%d frames\n",
               (unsigned long)g_frame_count, warmup_frames);
#endif
        HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);
        CAM_Deinit();
        return -1;
      }
    }
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Warmup done (%lu frames). Camera RUNNING.\n",
           (unsigned long)g_frame_count);
#endif
  } else {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] No warmup. Camera RUNNING.\n");
#endif
  }

  return 0;
}

/**
 * @brief  Start continuous camera capture (init + warmup done once at boot).
 *
 *   After this call the camera pipe is running in CONTINUOUS mode,
 *   constantly overwriting `buf` with fresh frames.  Use
 *   CAM_ContinuousSnap() to safely extract a frame on demand.
 *
 * @return 0 on success, -1 on timeout or buffer too small
 */
int CAM_ContinuousStart(uint8_t *buf, int buf_size, int width, int height, int fps)
{
  return CAM_InitAndStartContinuous(buf, buf_size, width, height, fps, SNAP_WARMUP_FRAMES);
}

/**
 * @brief  Snap the current frame from the continuous capture buffer.
 *
 *   Strategy (FIXED for torn-frame glitch):
 *     1. STOP the DCMIPP pipe briefly (halts sensor → DMA → capture_buf)
 *     2. Invalidate D-Cache for capture_buf (read fresh DMA data)
 *     3. memcpy capture_buf → dest_buf (guaranteed clean frame, no tearing)
 *     4. Clean D-Cache for dest_buf (SD write sees it)
 *     5. RESTART the DCMIPP pipe (continuous capture resumes)
 *
 *   At 30 FPS, one frame period ≈ 33ms. Stopping for ~5ms copy is only
 *   ~15% of a frame period — the sensor simply drops 0-1 frames during
 *   the copy, which is imperceptible.
 *
 *   This eliminates the "row jump"/glitch artifact caused by DCMIPP
 *   overwriting capture_buf rows while we're reading them.
 *
 * @return 0 on success, -1 on error
 */
int CAM_ContinuousSnap(uint8_t *dest_buf, uint32_t frame_size)
{
    if (!dest_buf) return -1;

    uint32_t t0 = HAL_GetTick();

    DCMIPP_HandleTypeDef *hhandle = CMW_CAMERA_GetDCMIPPHandle();

    /* Wait for NEXT frame after trigger so we never store a stale frame. */
    if (CAM_WaitFrames(1, 200, pdMS_TO_TICKS(2)) != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[CAM] ContinuousSnap timeout waiting next VSYNC\n");
#endif
        return -1;
    }

    /* --- Step 1: STOP DCMIPP pipe to prevent torn frames --- */
    HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);

    /* Small delay to ensure DCMIPP has fully stopped and DMA is idle */
    for (volatile int i = 0; i < 100; i++);

    /* --- Step 2: D-Cache coherency --- */
    SCB_InvalidateDCache_by_Addr((uint32_t *)capture_buf, frame_size);

    /* --- Step 3: memcpy — guaranteed no tearing since pipe is stopped --- */
    memcpy(dest_buf, capture_buf, frame_size);

    /* --- Step 4: Clean destination — push CPU-written data to PSRAM for SD DMA --- */
    SCB_CleanDCache_by_Addr((uint32_t *)dest_buf, frame_size);

    /* --- Step 5: RESTART the DCMIPP pipe --- */
    CMW_CAMERA_Start(DCMIPP_PIPE1, capture_buf, CMW_MODE_CONTINUOUS);

    uint32_t elapsed = HAL_GetTick() - t0;

#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] ContinuousSnap: stop+copy+restart in %lu ms\n", (unsigned long)elapsed);
#endif

    return 0;
}

/**
 * @brief  Stop continuous capture and deinit camera (shutdown).
 */
int CAM_ContinuousStop(void)
{
  DCMIPP_HandleTypeDef *hhandle = CMW_CAMERA_GetDCMIPPHandle();
  if (hhandle) {
    HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);
  }
  CAM_Deinit();
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Continuous mode stopped.\n");
#endif
  return 0;
}

/* ================================================================
   BATCH CAPTURE MODE (CAPTURE_MODE = 2)
   ================================================================ */

/**
 * @brief  Capture BATCH_FRAMES quickly from continuous camera into batch_buf.
 *
 *   Strategy:
 *     The camera is running in CONTINUOUS mode, overwriting capture_buf.
 *     On each grab:
 *       1. STOP pipe
 *       2. Invalidate D-Cache for capture_buf
 *       3. memcpy capture_buf → batch_buf[frame_index * frame_size]
 *       4. RESTART pipe (immediately, next frame arrives ~33ms at 30FPS)
 *       5. Wait for 1 new frame to complete (poll g_frame_count)
 *       6. Repeat for BATCH_FRAMES
 *
 *   Total grab time: BATCH_FRAMES × (~33ms + copy_time) ≈ 3 × 38ms = 114ms
 *   LEDs stay ON during this entire window.
 *
 * @param  batch_buf    Pre-allocated buffer ≥ BATCH_FRAMES × frame_size
 * @param  frame_size   Size of one frame in bytes
 * @return number of frames successfully captured (0..BATCH_FRAMES), -1 on error
 */
int CAM_ContinuousBatchSnap(uint8_t *batch_buf, uint32_t frame_size)
{
#if CAPTURE_MODE != 2
    (void)batch_buf; (void)frame_size;
    return -1;
#endif
    if (!batch_buf) return -1;

    DCMIPP_HandleTypeDef *hhandle = CMW_CAMERA_GetDCMIPPHandle();
    if (!hhandle) return -1;

    uint32_t t0 = HAL_GetTick();
    uint8_t frames_captured = 0;

#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] BatchSnap: grabbing %d frames...\n", BATCH_FRAMES);
#endif

    /* Ensure first copied frame is fresh after trigger. */
    if (CAM_WaitFrames(1, 200, pdMS_TO_TICKS(2)) != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[CAM] BatchSnap timeout waiting first fresh VSYNC\n");
#endif
        return -1;
    }

    for (uint8_t i = 0; i < BATCH_FRAMES; i++) {
        uint8_t *dest = batch_buf + (i * frame_size);

        /* --- Step 1: STOP DCMIPP pipe --- */
        HAL_DCMIPP_PIPE_Stop(hhandle, DCMIPP_PIPE1);
        for (volatile int d = 0; d < 100; d++);

        /* --- Step 2: Invalidate D-Cache (read fresh DMA data) --- */
        SCB_InvalidateDCache_by_Addr((uint32_t *)capture_buf, frame_size);

        /* --- Step 3: memcpy capture_buf → batch slot --- */
        memcpy(dest, capture_buf, frame_size);

        /* --- Step 4: Clean destination for SD DMA later --- */
        SCB_CleanDCache_by_Addr((uint32_t *)dest, frame_size);

        /* --- Step 5: RESTART pipe immediately --- */
        CMW_CAMERA_Start(DCMIPP_PIPE1, capture_buf, CMW_MODE_CONTINUOUS);

        frames_captured++;

        /* --- Step 6: Wait for 1 new frame before next grab (except last) --- */
        if (i < BATCH_FRAMES - 1) {
            if (CAM_WaitFrames(1, 200, pdMS_TO_TICKS(2)) != 0) {
                break;
            }
        }
    }

    uint32_t elapsed = HAL_GetTick() - t0;

#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] BatchSnap: %d frames in %lu ms (%lu ms/frame)\n",
           (unsigned long)frames_captured,
           (unsigned long)elapsed,
           frames_captured > 0 ? elapsed / frames_captured : 0);
#endif

    return (int)frames_captured;
}
