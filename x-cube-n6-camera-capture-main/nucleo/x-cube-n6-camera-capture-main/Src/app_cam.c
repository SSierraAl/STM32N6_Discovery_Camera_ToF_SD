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
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "cmsis_compiler.h"
#include "cmw_camera.h"
#include "cmw_io.h"
#include "app_cam.h"
#include "app_config.h"
#include "perf_debug.h"
#include "debug_color.h"
#include "stm32n6xx.h"
#include "stm32n6xx_hal.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"

/* IMX335 I2C address and standby/streaming registers */
#ifndef CAMERA_IMX335_ADDRESS
#define CAMERA_IMX335_ADDRESS  0x34U
#endif
#define IMX335_REG_MODE_SELECT  0x3000U
#define IMX335_MODE_STREAMING   0x00
#define IMX335_MODE_STANDBY     0x01

/** Helper: Write a single byte to an IMX335 register via I2C1 (16-bit addr). */
static int CAM_WriteSensorReg(uint16_t reg, uint8_t value)
{
  return CMW_I2C_WRITEREG16(CAMERA_IMX335_ADDRESS, reg, &value, 1);
}

extern PerfTimer_t g_perf_timer;
extern uint8_t capture_buf[];

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

#define SENSOR_WIDTH     0
#define SENSOR_HEIGHT    0

static const char *sensor_names[] = {
  "CMW_UNKNOWN", "CMW_VD66GY", "CMW_IMX335", "CMW_VD55G1", "CMW_VD1943",
};
static CMW_Sensor_Name_t sensor;
static int is_sensor_valid = 0;
static int g_cam_ready = 0;
/* Separate ready flag for CALLBACK-BATCH mode (CAPTURE_MODE = 4). */
static int g_callback_ready = 0;

static int CAM_getFlipMode(CMW_Sensor_Name_t s)
{
  int mode = CMW_MIRRORFLIP_NONE;
  int idx = 0;
  switch (s) {
    case CMW_VD66GY_Sensor:    mode = SENSOR_VD66GY_FLIP; idx = 1; break;
    case CMW_IMX335_Sensor:    mode = SENSOR_IMX335_FLIP; idx = 2; break;
    case CMW_VD55G1_Sensor:    mode = SENSOR_VD55G1_FLIP; idx = 3; break;
    case CMW_VD1943_Sensor:    mode = SENSOR_VD1943_FLIP; idx = 4; break;
    default: assert(0);
  }
  printf("Detected %s\n", sensor_names[idx]);
  return mode;
}

static int CAM_FormatToBpp(int fmt)
{
  switch (fmt) {
    case DCMIPP_PIXEL_PACKER_FORMAT_MONO_Y8_G8_1: return 1;
    case DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1:
    case DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1: return 2;
    case DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1: return 3;
    default: assert(0); return 0;
  }
}

static void CAM_InitCropConfig(CMW_Manual_roi_area_t *roi, int sw, int sh, CAM_conf_t *conf)
{
  float rx = (float)sw / conf->capture_width;
  float ry = (float)sh / conf->capture_height;
  float r = MIN(rx, ry);
  assert(r >= 1 && r < 64);
  roi->width  = (uint32_t)MIN(conf->capture_width * r, sw);
  roi->height = (uint32_t)MIN(conf->capture_height * r, sh);
  roi->offset_x = (sw - roi->width + 1) / 2;
  roi->offset_y = (sh - roi->height + 1) / 2;
}

static void CAM_EnableYuv(uint32_t Pipe)
{
  DCMIPP_ColorConversionConfTypeDef cc = {
    .ClampOutputSamples = ENABLE, .OutputSamplesType = DCMIPP_CLAMP_YUV,
    .RR = 131, .RG = -119, .RB = -12, .RA = 128,
    .GR = 55, .GG = 183, .GB = 18, .GA = 0,
    .BR = -30, .BG = -101, .BB = 131, .BA = 128,
  };
  assert(Pipe == DCMIPP_PIPE1);
  assert(HAL_DCMIPP_PIPE_SetYUVConversionConfig(CMW_CAMERA_GetDCMIPPHandle(), Pipe, &cc) == HAL_OK);
  assert(HAL_DCMIPP_PIPE_EnableYUVConversion(CMW_CAMERA_GetDCMIPPHandle(), Pipe) == HAL_OK);
}

static void DCMIPP_PipeInitCapture(CAM_conf_t *cam_conf, int sw, int sh, CAM_conf_t *conf)
{
  CMW_DCMIPP_Conf_t dc;
  uint32_t hw_pitch;
  assert(conf->capture_width >= conf->capture_height);
  dc.output_width  = conf->capture_width;
  dc.output_height = conf->capture_height;
  dc.output_format = cam_conf->dcmipp_output_format;
  dc.output_bpp    = CAM_FormatToBpp(cam_conf->dcmipp_output_format);
  dc.mode          = CMW_Aspect_ratio_manual_roi;
  dc.enable_swap   = cam_conf->is_rgb_swap;
  dc.enable_gamma_conversion = 0;
  CAM_InitCropConfig(&dc.manual_conf, sw, sh, conf);
  assert(CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dc, &hw_pitch) == HAL_OK);
  assert(hw_pitch == dc.output_width * dc.output_bpp);
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
  assert(cam_conf.width && cam_conf.height);
  DCMIPP_PipeInitCapture(conf, cam_conf.width, cam_conf.height, conf);
  return ret; // Return the actual initialization result
}

void CAM_SetSliceROI(uint16_t y_start, uint16_t height)
{
  CMW_DCMIPP_Conf_t conf;
  conf.manual_conf.offset_y = y_start;
  conf.manual_conf.height = height;
  CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &conf, NULL);
}

int CAM_CapturePipe_Start(uint8_t *dst, uint32_t mode)
{
  int ret = CMW_CAMERA_Start(DCMIPP_PIPE1, dst, mode);
#if PERF_DEBUG_LEVEL >= 1
  if (ret != CMW_ERROR_NONE) {
    printf("[CAM] Start failed mode=%lu ret=%d\n", (unsigned long)mode, ret);
  }
#endif
  return ret;
}

static int CAM_CapturePipe_StartRetry(uint8_t *dst, uint32_t mode, uint8_t retries)
{
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();

  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    if (CAM_CapturePipe_Start(dst, mode) == CMW_ERROR_NONE) {
      return 0;
    }

    if (h) {
      HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
    }
    HAL_Delay(2);
  }

  return -1;
}

void CAM_IspUpdate(void)
{
  assert(CMW_CAMERA_Run() == CMW_ERROR_NONE);
}

void CAM_Deinit(void)
{
  assert(CMW_CAMERA_DeInit() == HAL_OK);
}

void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe) { (void)pipe; }

static volatile uint32_t g_frame_count = 0;
static volatile int g_wait_frames = 0;
static volatile uint32_t g_frame_event_count = 0;
static volatile uint32_t g_vsync_count = 0;

void CAM_NotifyFrameEvent(void)
{
  g_frame_event_count++;
}

/**
 * @brief  Block until DCMIPP confirms a full frame has been DMA'd into
 *         capture_buf (CMW_CAMERA_PIPE_FrameEventCallback -> CAM_NotifyFrameEvent).
 *
 *   IMPORTANT: This must gate ONLY on the frame-complete event, not VSYNC.
 *   VSYNC marks the START of the next frame (sensor row-sync) and can fire
 *   slightly before DCMIPP finishes writing the previous frame to PSRAM.
 *   Gating on "vsync OR frame_event" (as before) could let the caller stop
 *   the pipe and memcpy a frame that is still being written -> partial/black
 *   or torn images, worse for a moving subject.
 * @return 0 on success, -1 on timeout (elapsed_ms optionally reports actual wait)
 */
static int CAM_WaitNextFrameReady(uint32_t timeout_ms, uint32_t *elapsed_ms)
{
  uint32_t start_evt = g_frame_event_count;
  uint32_t t0 = HAL_GetTick();

  while (g_frame_event_count == start_evt) {
    CAM_IspUpdate();
    vTaskDelay(pdMS_TO_TICKS(1));
    if (HAL_GetTick() - t0 > timeout_ms) {
      if (elapsed_ms) *elapsed_ms = HAL_GetTick() - t0;
      return -1;
    }
  }
  if (elapsed_ms) *elapsed_ms = HAL_GetTick() - t0;
  return 0;
}

void CAM_CountVsyncFrame(void)
{
  g_vsync_count++;
  if (g_wait_frames > 0) {
    g_frame_count++;
    if (g_frame_count >= (uint32_t)g_wait_frames)
      g_wait_frames = 0;
  }
}

void CAM_ResetFrameCounter(int wait_frames)
{
  g_frame_count = 0;
  g_wait_frames = wait_frames;
}

uint32_t CAM_GetFrameCount(void) { return g_frame_count; }

/* ================================================================
   ON-DEMAND SINGLE FRAME
   ================================================================ */
int CAM_CaptureSingleFrame(uint8_t *buf, int buf_size, int width, int height, int fps, int warmup_frames)
{
  int min_size = width * height * 2;
  if (buf == NULL || buf_size <= 0 || buf_size < min_size) return -1;
  if (warmup_frames < 1) warmup_frames = 8;

  PERF_MARK(g_perf_timer, CAM_INIT);
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Init camera %dx%d@%d YUV422 ...\n", width, height, fps);
#endif

  CAM_conf_t conf = {0};
  conf.capture_width = width;
  conf.capture_height = height;
  conf.fps = fps;
  conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
  conf.is_rgb_swap = 0;
  CAM_Init(&conf);

  PERF_MARK(g_perf_timer, CAM_EXPO);
  CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL :
                             CAM_EXPOSURE_MODE == 2 ? CMW_EXPOSUREMODE_AUTOFREEZE : CMW_EXPOSUREMODE_AUTO);
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
    int32_t re, rg;
    CMW_CAMERA_GetExposure(&re);
    CMW_CAMERA_GetGain(&rg);
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Exposure=%ld Gain=%ld\n", (long)re, (long)rg);
#endif
  }

  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  PERF_MARK(g_perf_timer, CAM_WARMUP);
  CAM_ResetFrameCounter(warmup_frames + 1);
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Start continuous (warmup=%d+1)...\n", warmup_frames);
#endif
  CAM_CapturePipe_Start(buf, CMW_MODE_CONTINUOUS);

  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
  }

  uint32_t t0 = HAL_GetTick();
  while (g_wait_frames != 0) {
    CAM_IspUpdate();
    vTaskDelay(pdMS_TO_TICKS(5));
    if (HAL_GetTick() - t0 > 5000) {
#if PERF_DEBUG_LEVEL >= 1
      printf("[CAM] TIMEOUT %lu/%d\n", (unsigned long)g_frame_count, warmup_frames+1);
#endif
      HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
      CAM_Deinit();
      return -1;
    }
  }

  PERF_MARK(g_perf_timer, CAM_SNAP);
  PERF_MARK(g_perf_timer, CAM_STOP);
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  HAL_Delay(5);
  SCB_InvalidateDCache_by_Addr((uint32_t*)buf, min_size);
  PERF_MARK(g_perf_timer, CAM_DEINIT);
  CAM_Deinit();
  return 0;
}

int CAM_CaptureSingleFrame_DefaultWarmup(uint8_t *buf, int bs, int w, int h, int fps)
{
  return CAM_CaptureSingleFrame(buf, bs, w, h, fps, 8);
}

/* ================================================================
   BATCH (ONE INIT, N FRAMES, ONE DEINIT)
   ================================================================ */
int CAM_CaptureBatchFrames(uint8_t *batch_buf, int frame_size, int frame_count, int width, int height, int fps)
{
  int min_size = width * height * 2;
  if (!batch_buf || frame_size < min_size || frame_count < 1) return -1;

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Batch init %dx%d@%d (%d frames)...\n", width, height, fps, frame_count);
#endif
  CAM_conf_t conf = {0};
  conf.capture_width = width;
  conf.capture_height = height;
  conf.fps = fps;
  conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
  conf.is_rgb_swap = 0;
  CAM_Init(&conf);

  CMW_CAMERA_SetExposureMode(CMW_EXPOSUREMODE_MANUAL);
  CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
  CMW_CAMERA_SetGain(CAM_GAIN_VALUE);

  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  CAM_CapturePipe_Start(capture_buf, CMW_MODE_CONTINUOUS);
  CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
  CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
  CAM_IspUpdate();

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Warmup %d frames...\n", SNAP_WARMUP_FRAMES);
#endif
  CAM_ResetFrameCounter(SNAP_WARMUP_FRAMES);
  uint32_t t0 = HAL_GetTick();
  while (g_wait_frames != 0) {
    CAM_IspUpdate();
    vTaskDelay(pdMS_TO_TICKS(5));
    if (HAL_GetTick() - t0 > 5000) { HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0); CAM_Deinit(); return -1; }
  }

  int captured = 0;
  for (int i = 0; i < frame_count; i++) {
    CAM_ResetFrameCounter(1);
    t0 = HAL_GetTick();
    while (g_wait_frames != 0) {
      CAM_IspUpdate();
      vTaskDelay(pdMS_TO_TICKS(5));
      if (HAL_GetTick() - t0 > 500) break;
    }
    uint8_t *dest = batch_buf + (i * frame_size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)capture_buf, frame_size);
    memcpy(dest, capture_buf, frame_size);
    SCB_CleanDCache_by_Addr((uint32_t*)dest, frame_size);
    captured++;
  }
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_Deinit();
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Batch: %d/%d in %lu ms\n", captured, frame_count, (unsigned long)(HAL_GetTick()-t0));
#endif
  return captured;
}

/* ================================================================
   CONTINUOUS MODE INTERNAL HELPER
   ================================================================ */
static int CAM_InitAndStartContinuous(uint8_t *buf, int buf_size, int width, int height, int fps, int warmup_frames)
{
  int min_size = width * height * 2;
  if (!buf || buf_size <= 0 || buf_size < min_size) return -1;
  if (warmup_frames < 0) warmup_frames = 0;

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Init %dx%d@%d YUV422...\n", width, height, fps);
#endif
  CAM_conf_t conf = {0};
  conf.capture_width = width;
  conf.capture_height = height;
  conf.fps = fps;
  conf.dcmipp_output_format = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
  conf.is_rgb_swap = 0;
  CAM_Init(&conf);

  CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL :
                             CAM_EXPOSURE_MODE == 2 ? CMW_EXPOSUREMODE_AUTOFREEZE : CMW_EXPOSUREMODE_AUTO);
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
#if PERF_DEBUG_LEVEL >= 2
    int32_t re, rg; CMW_CAMERA_GetExposure(&re); CMW_CAMERA_GetGain(&rg);
    printf("[CAM] exposure=%ld gain=%ld\n", (long)re, (long)rg);
#endif
  }

  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Start continuous...\n");
#endif
  CAM_CapturePipe_Start(buf, CMW_MODE_CONTINUOUS);

  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
    CAM_IspUpdate();
#if PERF_DEBUG_LEVEL >= 1
    int32_t re, rg; CMW_CAMERA_GetExposure(&re); CMW_CAMERA_GetGain(&rg);
    printf("[CAM] Post-start exp=%ld gain=%ld\n", (long)re, (long)rg);
#endif
  }

  if (warmup_frames > 0) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Warmup %d frames...\n", warmup_frames);
#endif
    CAM_ResetFrameCounter(warmup_frames);
    uint32_t t0 = HAL_GetTick();
    while (g_wait_frames != 0) {
      CAM_IspUpdate();
      vTaskDelay(pdMS_TO_TICKS(5));
      if (HAL_GetTick() - t0 > 5000) {
        HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0); CAM_Deinit(); return -1;
      }
    }
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Warmup done (%lu frames). RUNNING.\n", (unsigned long)g_frame_count);
#endif
  }
  return 0;
}

int CAM_ContinuousStart(uint8_t *buf, int bs, int w, int h, int fps)
{
  return CAM_InitAndStartContinuous(buf, bs, w, h, fps, SNAP_WARMUP_FRAMES);
}

int CAM_ContinuousSnap(uint8_t *dest, uint32_t fs)
{
  if (!dest) return -1;
  uint32_t t0 = HAL_GetTick();
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  for (volatile int i = 0; i < 100; i++);
  SCB_InvalidateDCache_by_Addr((uint32_t*)capture_buf, fs);
  memcpy(dest, capture_buf, fs);
  SCB_CleanDCache_by_Addr((uint32_t*)dest, fs);
  CAM_CapturePipe_Start(capture_buf, CMW_MODE_CONTINUOUS);
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] ContinuousSnap in %lu ms\n", (unsigned long)(HAL_GetTick()-t0));
#endif
  return 0;
}

int CAM_ContinuousStop(void)
{
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  if (h) HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_Deinit();
  return 0;
}

/* ================================================================
   BATCH MODE (CAPTURE_MODE = 2)
   ================================================================ */
int CAM_ContinuousBatchSnap(uint8_t *batch_buf, uint32_t frame_size)
{
#if CAPTURE_MODE != 2
  (void)batch_buf; (void)frame_size; return -1;
#endif
  if (!batch_buf) return -1;
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  if (!h) return -1;

  uint32_t t0 = HAL_GetTick();
  uint8_t captured = 0;
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] BatchSnap: %d frames...\n", BATCH_FRAMES);
#endif

  for (uint8_t i = 0; i < BATCH_FRAMES; i++) {
    uint8_t *dest = batch_buf + (i * frame_size);
    HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
    for (volatile int d = 0; d < 100; d++);
    SCB_InvalidateDCache_by_Addr((uint32_t*)capture_buf, frame_size);
    memcpy(dest, capture_buf, frame_size);
    SCB_CleanDCache_by_Addr((uint32_t*)dest, frame_size);
    CAM_CapturePipe_Start(capture_buf, CMW_MODE_CONTINUOUS);
    captured++;
    if (i < BATCH_FRAMES - 1) {
      CAM_ResetFrameCounter(1);
      uint32_t tw = HAL_GetTick();
      while (g_wait_frames != 0) {
        CAM_IspUpdate();
        vTaskDelay(pdMS_TO_TICKS(2));
        if (HAL_GetTick() - tw > 200) break;
      }
    }
  }

  uint32_t elapsed = HAL_GetTick() - t0;
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] BatchSnap: %d frames in %lu ms\n", (unsigned long)captured, (unsigned long)elapsed);
#endif
  return (int)captured;
}

/* ================================================================
   STANDBY-BATCH MODE (CAPTURE_MODE = 3)
   ================================================================ */
int CAM_StandbyInit(uint8_t *buf, int buf_size, int width, int height, int fps)
{
  uint32_t t0 = HAL_GetTick();
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Standby init: full init + warmup + standby...\n");
#endif

  int rc = CAM_InitAndStartContinuous(buf, buf_size, width, height, fps, SNAP_WARMUP_FRAMES);
  if (rc != 0) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Standby init FAILED rc=%d\n", rc);
#endif
    return -1;
  }

  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);

  g_cam_ready = 1;
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Standby init done in %lu ms.\n", (unsigned long)(HAL_GetTick() - t0));
#endif
  return 0;
}

/**
 * @brief  Wake from standby, capture BATCH_FRAMES, return to standby.
 */
int CAM_StandbyBatchSnap(uint8_t *batch_buf, uint32_t frame_size)
{
#if CAPTURE_MODE != 3
  (void)batch_buf; (void)frame_size; return -1;
#endif
  if (!batch_buf || !g_cam_ready) return -1;

  PERF_MARK(g_perf_timer, CAM_INIT);
  uint32_t t0 = HAL_GetTick();
  uint8_t captured = 0;
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  if (!h) return -1;

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Standby-Batch: wakeup + %d frames...\n", BATCH_FRAMES);
#endif

  /* Step 1: Wake sensor */
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STREAMING);
  HAL_Delay(35);

  /* Step 2: Start continuous pipe (stable path for this project). */
  if (CAM_CapturePipe_StartRetry(capture_buf, CMW_MODE_CONTINUOUS, 2) != 0) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Continuous start failed after standby wake\n");
#endif
    goto standby_exit;
  }

  /* Step 3: Re-apply exposure */
  PERF_MARK(g_perf_timer, CAM_EXPO);
  CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL :
                             CAM_EXPOSURE_MODE == 2 ? CMW_EXPOSUREMODE_AUTOFREEZE : CMW_EXPOSUREMODE_AUTO);
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
    CAM_IspUpdate();
#if PERF_DEBUG_LEVEL >= 1
    int32_t re = -1, rg = -1;
    CMW_CAMERA_GetExposure(&re);
    CMW_CAMERA_GetGain(&rg);
    printf("[CAM] Standby-wake exposure=%ld gain=%ld (target %d/%d)\n",
           (long)re, (long)rg, (int)CAM_EXPOSURE_VALUE, (int)CAM_GAIN_VALUE);
#endif
  }

  /* Step 4: Warmup after wakeup (discard N completed frames). */
  PERF_MARK(g_perf_timer, CAM_WARMUP);
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Standby warmup %d frames...\n", STANDBY_WARMUP_FRAMES);
#endif
  for (uint8_t w = 0; w < STANDBY_WARMUP_FRAMES; w++) {
    uint32_t frame_ms = 0;
    if (CAM_WaitNextFrameReady(120, &frame_ms) != 0) {
#if PERF_DEBUG_LEVEL >= 1
      printf("[CAM] Standby WARMUP TIMEOUT!\n");
#endif
      goto standby_exit;
    }
#if PERF_DEBUG_LEVEL >= 2
    printf("[CAM] warmup[%u] frame in %lu ms\n", (unsigned)w, (unsigned long)frame_ms);
#endif
  }

  /* Step 5: Capture BATCH_FRAMES from continuous stream with frame-ready gating. */
  while (captured < BATCH_FRAMES) {
    uint32_t frame_ms = 0;
    if (CAM_WaitNextFrameReady(120, &frame_ms) != 0) {
#if PERF_DEBUG_LEVEL >= 1
      printf("[CAM] Frame wait timeout at index %u\n", (unsigned)captured);
#endif
      break;
    }

    /* NOTE: Stop must happen immediately after the frame-ready event, with
       nothing (not even a printf) in between. capture_buf is being
       continuously overwritten until Stop() actually lands, so any delay
       here risks copying a frame Stop() only partially caught. */
    HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
    HAL_Delay(2);

    uint8_t *dest = batch_buf + (captured * frame_size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)capture_buf, frame_size);
    memcpy(dest, capture_buf, frame_size);
    SCB_CleanDCache_by_Addr((uint32_t*)dest, frame_size);
#if PERF_DEBUG_LEVEL >= 2
    printf("[CAM] capture[%u] frame in %lu ms\n", (unsigned)captured, (unsigned long)frame_ms);
#endif
    captured++;

    if (captured < BATCH_FRAMES) {
      if (CAM_CapturePipe_StartRetry(capture_buf, CMW_MODE_CONTINUOUS, 2) != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[CAM] Restart failed at index %u\n", (unsigned)captured);
#endif
        break;
      }
    }
  }

  PERF_MARK(g_perf_timer, CAM_SNAP);

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Standby-Batch: %d frames in %lu ms\n",
         (unsigned long)captured, (unsigned long)(HAL_GetTick()-t0));
#endif

standby_exit:
  /* Step 6: Return to standby */
  PERF_MARK(g_perf_timer, CAM_STOP);
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
  PERF_MARK(g_perf_timer, CAM_DEINIT);
  return (int)captured;
}

int CAM_IsStandbyReady(void)
{
  return g_cam_ready;
}

/* ================================================================
   CALLBACK-BATCH MODE (CAPTURE_MODE = 4)
   Callback-driven continuous batch capture — NO Stop/Restart between frames.

   Uses g_frame_event_count (incremented by DCMIPP frame event ISR callback)
   to know EXACTLY when each frame DMA is complete. The pipe runs continuously
   through the entire batch, eliminating Stop/Restart tearing.
   ================================================================ */

int CAM_CallbackInit(uint8_t *buf, int buf_size, int width, int height, int fps)
{
  uint32_t t0 = HAL_GetTick();
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch init: full init + warmup + standby...\n");
#endif

  /* Reuse the proven standby init path. */
  int rc = CAM_InitAndStartContinuous(buf, buf_size, width, height, fps, SNAP_WARMUP_FRAMES);
  if (rc != 0) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Callback init FAILED rc=%d\n", rc);
#endif
    return -1;
  }

  /* Stop pipe and put sensor in standby (same as standby mode). */
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);

  /* Reset frame event counter so captures start from a known state. */
  g_frame_event_count = 0;
  g_callback_ready = 1;

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch init done in %lu ms.\n", (unsigned long)(HAL_GetTick() - t0));
#endif
  return 0;
}

/**
 * @brief  Wake from standby, capture CALLBACK_FRAMES continuously, return to standby.
 *
 *   TRUE zero-copy, no-restart capture. The DCMIPP pipe is Started exactly
 *   ONCE for the whole batch and never stopped/restarted. Instead of
 *   memcpy-ing each frame out of a scratch buffer (which was measured to
 *   cost hundreds of ms per 2.4MB frame — PSRAM-to-PSRAM CPU copies are
 *   slow), we reprogram the DCMIPP's own ping-pong destination registers
 *   (HAL_DCMIPP_PIPE_SetMemoryAddress) 2 FRAMES AHEAD so the hardware DMAs
 *   each wanted frame DIRECTLY into its final batch_buf[] slot. No CPU
 *   copy, no race: an address is only ever reprogrammed right after the
 *   frame that was using it completes, and that same physical address
 *   isn't touched again for a full 2-frame period (~66 ms @30fps) — far
 *   longer than a register write takes.
 *
 *   Sequencing (k = frame index since Start, 0-based):
 *     k = 0 .. CALLBACK_WARMUP_FRAMES-1        -> warmup, discarded (lands in
 *                                                 capture_buf/save_buf scratch)
 *     k = CALLBACK_WARMUP_FRAMES .. (W+N-1)    -> the N=CALLBACK_FRAMES frames
 *                                                 we keep, DMA'd straight into
 *                                                 batch_buf[k-W]
 *   Two frames before each kept frame is due, we redirect whichever
 *   physical address is about to free up to point at that frame's final
 *   batch_buf slot — so by the time the hardware actually captures it, the
 *   destination is already correct.
 */
int CAM_CallbackBatchSnap(uint8_t *batch_buf, uint32_t frame_size)
{
#if CAPTURE_MODE != 4
  (void)batch_buf; (void)frame_size; return -1;
#endif
  if (!batch_buf || !g_callback_ready) return -1;

  extern uint8_t save_buf[]; /* unused in mode 4 otherwise — reused as scratch warmup buffer */
  uint8_t *const raw_buf[2] = { capture_buf, save_buf };

  PERF_MARK(g_perf_timer, CAM_INIT);
  uint32_t t0 = HAL_GetTick();
  uint8_t captured = 0;
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  if (!h) return -1;

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch: wakeup + %d frames (zero-copy DMA, no restart)...\n", CALLBACK_FRAMES);
#endif

  /* ---------------------------------------------------------------
     Step 1: Wake sensor from standby.
     --------------------------------------------------------------- */
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STREAMING);
  HAL_Delay(35);

  /* ---------------------------------------------------------------
     Step 2: Start continuous pipe ONCE with HARDWARE double buffering.
     Addresses start out pointing at scratch raw_buf[0]/raw_buf[1]; they
     get reprogrammed to batch_buf slots as the warmup tail approaches.
     --------------------------------------------------------------- */
  if (CMW_CAMERA_DoubleBufferStart(DCMIPP_PIPE1, raw_buf[0], raw_buf[1], CMW_MODE_CONTINUOUS) != CMW_ERROR_NONE) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Double-buffer start failed after callback wake\n");
#endif
    goto callback_exit;
  }

  /* ---------------------------------------------------------------
     Step 3: Re-apply exposure/gain after wake.
     --------------------------------------------------------------- */
  PERF_MARK(g_perf_timer, CAM_EXPO);
  CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL :
                             CAM_EXPOSURE_MODE == 2 ? CMW_EXPOSUREMODE_AUTOFREEZE : CMW_EXPOSUREMODE_AUTO);
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
    CAM_IspUpdate();
  }

  /* ---------------------------------------------------------------
     Step 4 + 5 unified: walk through warmup AND capture frames in one
     loop so the "arm 2 frames ahead" bookkeeping is simple/uniform.
     k in [0, CALLBACK_WARMUP_FRAMES) are discarded; k in
     [CALLBACK_WARMUP_FRAMES, CALLBACK_WARMUP_FRAMES+CALLBACK_FRAMES) are
     the frames we keep (out_idx = k - CALLBACK_WARMUP_FRAMES).
     --------------------------------------------------------------- */
  PERF_MARK(g_perf_timer, CAM_WARMUP);
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback warmup %d + capture %d frames...\n", CALLBACK_WARMUP_FRAMES, CALLBACK_FRAMES);
#endif
  {
    const uint32_t total = (uint32_t)CALLBACK_WARMUP_FRAMES + (uint32_t)CALLBACK_FRAMES;
    uint32_t frame_seq = 0;

    for (uint32_t k = 0; k < total; k++) {
      uint32_t frame_ms = 0;

      if (k == CALLBACK_WARMUP_FRAMES) {
        PERF_MARK(g_perf_timer, CAM_SNAP);
      }

      if (CAM_WaitNextFrameReady(120, &frame_ms) != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[CAM] Frame wait timeout at k=%lu\n", (unsigned long)k);
#endif
        break;
      }
      frame_seq++;

      int32_t out_idx = (int32_t)k - (int32_t)CALLBACK_WARMUP_FRAMES;
      if (out_idx >= 0) {
        /* This frame was DMA'd directly into batch_buf[out_idx] — no copy
           needed, just drop any stale CPU cache lines before storage reads it. */
        uint8_t *dest = batch_buf + ((uint32_t)out_idx * frame_size);
        SCB_InvalidateDCache_by_Addr((uint32_t*)dest, frame_size);
        captured++;
#if PERF_DEBUG_LEVEL >= 2
        printf("[CAM] capture[%d] frame in %lu ms (zero-copy)\n", out_idx, (unsigned long)frame_ms);
#endif
      }
#if PERF_DEBUG_LEVEL >= 2
      else {
        printf("[CAM] warmup[%lu] frame in %lu ms\n", (unsigned long)k, (unsigned long)frame_ms);
      }
#endif

      /* Arm the address that JUST freed (used by the frame that just
         completed) for reuse 2 frames from now, if that future frame is
         one we want to keep. */
      int32_t arm_idx = out_idx + 2;
      if (arm_idx >= 0 && arm_idx < CALLBACK_FRAMES) {
        uint32_t parity = (frame_seq - 1U) % 2U;
        HAL_DCMIPP_PIPE_SetMemoryAddress(h, DCMIPP_PIPE1,
                                          parity == 0U ? DCMIPP_MEMORY_ADDRESS_0 : DCMIPP_MEMORY_ADDRESS_1,
                                          (uint32_t)(batch_buf + ((uint32_t)arm_idx * frame_size)));
      }
    }
  }

  /* ---------------------------------------------------------------
     Step 6: Stop pipe ONCE after ALL frames captured.
     --------------------------------------------------------------- */
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch: %d/%d frames in %lu ms (continuous, zero-copy)\n",
         (unsigned)captured, CALLBACK_FRAMES, (unsigned long)(HAL_GetTick() - t0));
#endif

callback_exit:
  /* Return to standby. */
  PERF_MARK(g_perf_timer, CAM_STOP);
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
  PERF_MARK(g_perf_timer, CAM_DEINIT);
  return (int)captured;
}

int CAM_IsCallbackReady(void)
{
  return g_callback_ready;
}
