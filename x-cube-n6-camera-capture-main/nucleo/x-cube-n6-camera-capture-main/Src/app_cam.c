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
static int g_callback_ready = 0;
#if CAPTURE_MODE == 4
/* Timing metadata only: one HAL tick per kept frame. No I2C is performed in
   the callback capture loop. */
static uint32_t g_callback_capture_ticks[CALLBACK_FRAMES] = {0};
#endif

#if CAM_SENSOR_REG_DEBUG && (CAPTURE_MODE == 4)
static void CAM_DumpImx335AeState(const char *tag)
{
  uint8_t v[3] = {0}, s[3] = {0}, g[2] = {0};
  if (sensor != CMW_IMX335_Sensor) return;

  int rv = CMW_I2C_READREG16(CAMERA_IMX335_ADDRESS, 0x3030U, v, sizeof(v));
  int rs = CMW_I2C_READREG16(CAMERA_IMX335_ADDRESS, 0x3058U, s, sizeof(s));
  int rg = CMW_I2C_READREG16(CAMERA_IMX335_ADDRESS, 0x30E8U, g, sizeof(g));
  if (rv || rs || rg) {
    printf("[IMX335][AE][%s] read failed: VMAX=%d SHS1=%d GAIN=%d\n", tag, rv, rs, rg);
    return;
  }
  uint32_t vmax = v[0] | ((uint32_t)v[1] << 8) | (((uint32_t)v[2] & 0x0FU) << 16);
  uint32_t shs1 = s[0] | ((uint32_t)s[1] << 8) | (((uint32_t)s[2] & 0x0FU) << 16);
  uint32_t gain = g[0] | (((uint32_t)g[1] & 0x07U) << 8);
  uint32_t lines = vmax > shs1 ? vmax - shs1 : 0U;
  uint32_t exposure_est_us = (uint32_t)(((uint64_t)lines * 1000000U + 67500U) / 135000U);
  printf("[IMX335][AE][%s] policy=%d VMAX=%lu SHS1=%lu lines=%lu "
         "exposure_est_us=%lu (nominal 135000 lines/s) gain_reg=%lu gain_mdB=%lu\n",
         tag, CAM_EXPOSURE_MODE, (unsigned long)vmax, (unsigned long)shs1,
         (unsigned long)lines, (unsigned long)exposure_est_us,
         (unsigned long)gain, (unsigned long)(gain * 300U));
}
#endif

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
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Full field=%lux%lu+%lu+%lu output=%dx%d\n",
         (unsigned long)dc.manual_conf.width, (unsigned long)dc.manual_conf.height,
         (unsigned long)dc.manual_conf.offset_x, (unsigned long)dc.manual_conf.offset_y,
         conf->capture_width, conf->capture_height);
#endif
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
    for (int attempt = 1; attempt <= 3; attempt++) {
      ret = CMW_CAMERA_GetSensorName(&sensor);
      if (ret == CMW_ERROR_NONE) break;
      printf("[CAM] I2C not ready (attempt %d/3, ret=%d)\n", attempt, ret);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (ret != CMW_ERROR_NONE) {
      printf("[CAM] FATAL: Cannot detect sensor after 3 retries (ret=%d)\n", ret);
      assert(0);
    }
  }

  cam_conf.width = SENSOR_WIDTH;
  cam_conf.height = SENSOR_HEIGHT;
  cam_conf.fps = conf->fps;
  cam_conf.mirror_flip = CAM_getFlipMode(sensor);
  ret = CMW_CAMERA_Init(&cam_conf, NULL);
  if (ret != CMW_ERROR_NONE) {
    printf("[CAM] Init failed (ret=%d), retrying with delay...\n", ret);
    vTaskDelay(pdMS_TO_TICKS(500));
    ret = CMW_CAMERA_Init(&cam_conf, NULL);
  }
  if (ret != CMW_ERROR_NONE) {
    printf("[CAM] FATAL: Init failed after retry (ret=%d)\n", ret);
    assert(0);
  }
  assert(cam_conf.width && cam_conf.height);
  DCMIPP_PipeInitCapture(conf, cam_conf.width, cam_conf.height, conf);
  return ret;
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
    if (CAM_CapturePipe_Start(dst, mode) == CMW_ERROR_NONE) return 0;
    if (h) HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
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

#if CAPTURE_MODE == 0
static volatile uint32_t g_single_capture_errors;
#endif
void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe)
{
#if CAPTURE_MODE == 0
  if (pipe == DCMIPP_PIPE1) ++g_single_capture_errors;
#else
  (void)pipe;
#endif
}

static volatile uint32_t g_frame_count = 0;
static volatile int g_wait_frames = 0;
static volatile uint32_t g_frame_event_count = 0;
static volatile uint32_t g_vsync_count = 0;

void CAM_NotifyFrameEvent(void)
{
  g_frame_event_count++;
}

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
    if (g_frame_count >= (uint32_t)g_wait_frames) g_wait_frames = 0;
  }
}

void CAM_ResetFrameCounter(int wait_frames)
{
  g_frame_count = 0;
  g_wait_frames = wait_frames;
}

uint32_t CAM_GetFrameCount(void) { return g_frame_count; }

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
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Full-field single: %dx%d, %d warmups reuse one buffer\n", width, height, warmup_frames);
#endif
#if CAPTURE_MODE == 0
  g_single_capture_errors = 0;
#endif
  if (CAM_CapturePipe_Start(buf, CMW_MODE_CONTINUOUS) != CMW_ERROR_NONE) {
    CAM_Deinit();
    return -1;
  }
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
  }
  for (int frame = 0; frame <= warmup_frames; ++frame) {
    if (frame == warmup_frames) PERF_MARK(g_perf_timer, CAM_SNAP);
    if (CAM_WaitNextFrameReady(SNAP_TIMEOUT_MS, NULL) != 0) {
      HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
      CAM_Deinit();
      return -1;
    }
  }

  PERF_MARK(g_perf_timer, CAM_STOP);
  HAL_StatusTypeDef stop_rc = HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
#if CAPTURE_MODE == 0
  if (g_single_capture_errors || stop_rc != HAL_OK) {
    printf("[CAM] Single rejected: pipe errors=%lu stop=%d\n",
           (unsigned long)g_single_capture_errors, (int)stop_rc);
    CAM_Deinit();
    return -1;
  }
#else
  if (stop_rc != HAL_OK) { CAM_Deinit(); return -1; }
#endif
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

  CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL : CMW_EXPOSUREMODE_AUTO);
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
  }

  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  CAM_CapturePipe_Start(capture_buf, CMW_MODE_CONTINUOUS);
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
  }
  CAM_IspUpdate();
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Warmup %d frames...\n", SNAP_WARMUP_FRAMES);
#endif
  CAM_ResetFrameCounter(SNAP_WARMUP_FRAMES);
  uint32_t t0 = HAL_GetTick();
  while (g_wait_frames != 0) {
    CAM_IspUpdate();
    vTaskDelay(pdMS_TO_TICKS(5));
    if (HAL_GetTick() - t0 > 5000) {
      HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
      CAM_Deinit();
      return -1;
    }
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
        HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
        CAM_Deinit();
        return -1;
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
  printf("[CAM] BatchSnap: %lu frames in %lu ms\n", (unsigned long)captured, (unsigned long)elapsed);
#endif
  return (int)captured;
}

int CAM_CallbackInit(uint8_t *buf, int buf_size, int width, int height, int fps)
{
  uint32_t t0 = HAL_GetTick();
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch init: full init + warmup + standby...\n");
#endif
  int rc = CAM_InitAndStartContinuous(buf, buf_size, width, height, fps, SNAP_WARMUP_FRAMES);
  if (rc != 0) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Callback init FAILED rc=%d\n", rc);
#endif
    return -1;
  }

  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
#if CAM_SENSOR_REG_DEBUG && (CAPTURE_MODE == 4)
  CAM_DumpImx335AeState("INIT_STANDBY");
#endif
  g_frame_event_count = 0;
  g_callback_ready = 1;
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch init done in %lu ms.\n", (unsigned long)(HAL_GetTick() - t0));
#endif
  return 0;
}

int CAM_CallbackBatchSnap(uint8_t *batch_buf, uint32_t frame_size)
{
#if PERF_CAMERA_FRAME_LOG
  uint32_t frame_times[CALLBACK_WARMUP_FRAMES + CALLBACK_FRAMES];
  uint32_t completed_frames = 0;
#endif
#if CAPTURE_MODE != 4
  (void)batch_buf; (void)frame_size; return -1;
#endif
  if (!batch_buf || !g_callback_ready) return -1;

#if CAPTURE_MODE == 4
  memset(g_callback_capture_ticks, 0, sizeof(g_callback_capture_ticks));
#endif
  extern uint8_t save_buf[];
  uint8_t *const raw_buf[2] = { capture_buf, save_buf };

  PERF_MARK(g_perf_timer, CAM_INIT);
  uint32_t t0 = HAL_GetTick();
  uint8_t captured = 0;
  DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
  if (!h) return -1;

#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch: wakeup + %d frames (zero-copy DMA, no restart)...\n", CALLBACK_FRAMES);
#endif
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STREAMING);
  HAL_Delay(35);

  if (CMW_CAMERA_DoubleBufferStart(DCMIPP_PIPE1, raw_buf[0], raw_buf[1], CMW_MODE_CONTINUOUS) != CMW_ERROR_NONE) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Double-buffer start failed after callback wake\n");
#endif
    goto callback_exit;
  }

  PERF_MARK(g_perf_timer, CAM_EXPO);
  CMW_CAMERA_SetExposureMode(CAM_EXPOSURE_MODE == 1 ? CMW_EXPOSUREMODE_MANUAL :
                             CAM_EXPOSURE_MODE == 2 ? CMW_EXPOSUREMODE_AUTOFREEZE : CMW_EXPOSUREMODE_AUTO);
  if (CAM_EXPOSURE_MODE == 1) {
    CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
    CMW_CAMERA_SetGain(CAM_GAIN_VALUE);
    CAM_IspUpdate();
  }

  PERF_MARK(g_perf_timer, CAM_WARMUP);
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback warmup %d + capture %d frames...\n", CALLBACK_WARMUP_FRAMES, CALLBACK_FRAMES);
#endif
  {
    const uint32_t total = (uint32_t)CALLBACK_WARMUP_FRAMES + (uint32_t)CALLBACK_FRAMES;
    uint32_t frame_seq = 0;

    for (uint32_t k = 0; k < total; k++) {
      uint32_t frame_ms = 0;
      if (k == CALLBACK_WARMUP_FRAMES) PERF_MARK(g_perf_timer, CAM_SNAP);

      if (CAM_WaitNextFrameReady(120, &frame_ms) != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[CAM] Frame wait timeout at k=%lu\n", (unsigned long)k);
#endif
        break;
      }
      frame_seq++;
#if PERF_CAMERA_FRAME_LOG
      frame_times[completed_frames++] = frame_ms;
#endif

      int32_t out_idx = (int32_t)k - (int32_t)CALLBACK_WARMUP_FRAMES;
      if (out_idx >= 0) {
        uint8_t *dest = batch_buf + ((uint32_t)out_idx * frame_size);
        SCB_InvalidateDCache_by_Addr((uint32_t*)dest, frame_size);
        /* The frame-complete event already fired; record the task-visible
           completion tick now. This is metadata only and adds no I2C/UART
           traffic to the critical DMA loop. */
        g_callback_capture_ticks[(uint32_t)out_idx] = HAL_GetTick();
        captured++;
      }

      int32_t arm_idx = out_idx + 2;
      if (arm_idx >= 0 && arm_idx < CALLBACK_FRAMES) {
        uint32_t parity = (frame_seq - 1U) % 2U;
        HAL_DCMIPP_PIPE_SetMemoryAddress(h, DCMIPP_PIPE1,
                                        parity == 0U ? DCMIPP_MEMORY_ADDRESS_0 : DCMIPP_MEMORY_ADDRESS_1,
                                        (uint32_t)(batch_buf + ((uint32_t)arm_idx * frame_size)));
      }
    }
  }

callback_exit:
  PERF_MARK(g_perf_timer, CAM_STOP);
  HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
  PERF_MARK(g_perf_timer, CAM_DEINIT);
#if PERF_DEBUG_LEVEL >= 1
  uint32_t capture_elapsed_ms = HAL_GetTick() - t0;
#endif
#if PERF_CAMERA_FRAME_LOG
  for (uint32_t k = 0; k < completed_frames; k++) {
    printf("[CAM] %s[%lu] wait=%lu ms\n",
           k < CALLBACK_WARMUP_FRAMES ? "warmup" : "capture",
           (unsigned long)(k < CALLBACK_WARMUP_FRAMES ? k : k - CALLBACK_WARMUP_FRAMES),
           (unsigned long)frame_times[k]);
  }
#endif
#if PERF_DEBUG_LEVEL >= 1
  printf("[CAM] Callback-Batch: %d/%d frames in %lu ms (continuous, zero-copy)\n",
         (unsigned)captured, CALLBACK_FRAMES, (unsigned long)capture_elapsed_ms);
#endif
#if CAM_SENSOR_REG_DEBUG && (CAPTURE_MODE == 4)
  if (captured > 0U) CAM_DumpImx335AeState("AFTER_BATCH_STANDBY");
#endif
  return (int)captured;
}

uint32_t CAM_GetCallbackFrameTick(uint32_t index)
{
#if CAPTURE_MODE == 4
  if (index < CALLBACK_FRAMES) return g_callback_capture_ticks[index];
#else
  (void)index;
#endif
  return 0U;
}

int CAM_IsCallbackReady(void)
{
  return g_callback_ready;
}
