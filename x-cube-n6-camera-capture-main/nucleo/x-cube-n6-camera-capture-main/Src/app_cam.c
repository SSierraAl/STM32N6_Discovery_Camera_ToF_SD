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

/* Multi-byte register write helper */
int CAM_WriteSensorRegMulti(uint16_t reg, const uint8_t *data, uint16_t len)
{
  return CMW_I2C_WRITEREG16(CAMERA_IMX335_ADDRESS, reg, (uint8_t *)data, len);
}

/* ================================================================
   IMX335 AREA3 / ROI register sequence
   ================================================================
   Full standby->hold->write->stream sequence with retry logic.
   Reference: MASSIF-ROI-Windowing project, App_ChangeROI().
   ================================================================ */

#define IMX335_REG_HOLD       0x3001U
#define IMX335_REG_VMAX       0x3030U
#define IMX335_REG_SHUTTER    0x3058U
#define AREA3_ST_ADR_X        0x3074U
#define AREA3_ST_ADR_Y        0x3076U
#define AREA3_END_ADR_X       0x3078U
#define AREA3_END_ADR_Y       0x307AU
#define IMX335_SENSOR_W       2592
#define IMX335_SENSOR_H       1944

/** Helper: write 3-byte (24-bit) value to sensor, little-endian. */
static int CAM_WriteSensorReg24(uint16_t reg, uint32_t value)
{
  uint8_t d[3];
  d[0] = (uint8_t)(value & 0xFFU);
  d[1] = (uint8_t)((value >> 8) & 0xFFU);
  d[2] = (uint8_t)((value >> 16) & 0xFFU);
  return CMW_I2C_WRITEREG16(CAMERA_IMX335_ADDRESS, reg, d, 3);
}

/** Helper: write 2-byte (16-bit) value to sensor, little-endian. */
static int CAM_WriteSensorReg16(uint16_t reg, uint32_t value)
{
  uint8_t d[2];
  d[0] = (uint8_t)(value & 0xFFU);
  d[1] = (uint8_t)((value >> 8) & 0xFFU);
  return CMW_I2C_WRITEREG16(CAMERA_IMX335_ADDRESS, reg, d, 2);
}

/** Helper: write 4-byte (32-bit) value to sensor, little-endian. */
static int CAM_WriteSensorReg32(uint16_t reg, uint32_t value)
{
  uint8_t d[4];
  d[0] = (uint8_t)(value & 0xFFU);
  d[1] = (uint8_t)((value >> 8) & 0xFFU);
  d[2] = (uint8_t)((value >> 16) & 0xFFU);
  d[3] = (uint8_t)((value >> 24) & 0xFFU);
  return CMW_I2C_WRITEREG16(CAMERA_IMX335_ADDRESS, reg, d, 4);
}

int CAM_SetSensorROI(uint16_t x_start, uint16_t y_start,
                     uint16_t width, uint16_t height)
{
  uint16_t x_end, y_end;
  uint32_t vmax, shutter;
  int      retry, ret;
  uint8_t  data[4];

  /* Validate */
  if (width < 16 || height < 4) return -1;
  if ((uint32_t)x_start + width > IMX335_SENSOR_W) return -1;
  if ((uint32_t)y_start + height > IMX335_SENSOR_H) return -1;

  x_end = x_start + width - 1;
  y_end = y_start + height - 1;

  /* VMAX = full-frame 30fps value = 0x1194 = 4500 lines.
     IMPORTANT: IMX335 VMAX is a 16-bit register at 0x3030-0x3031.
     Do NOT use CAM_WriteSensorReg32 (4 bytes) — it corrupts 0x3032-0x3033!
     line_time = 1/(30*4500) = 7.407 us at 30fps.  */
  vmax   = 0x1194UL;  /* Fixed: full-frame 30fps from imx335.c framerate_30fps_regs */
  /* SHUTTER from target exposure (CAM_EXPOSURE_VALUE µs):
     shutter = exposure_us * fps * vmax / 1000000  */
  shutter = (uint32_t)((unsigned long long)CAM_EXPOSURE_VALUE *
                      (unsigned long long)SNAP_FPS *
                      (unsigned long long)vmax / 1000000ULL);
  if (shutter < 9) shutter = 9;
  if (shutter > vmax - 4) shutter = vmax - 4;

  {
    /* Actual exposure in microseconds for diagnostic */
    uint32_t exposure_us = (uint32_t)((unsigned long long)shutter *
                                     1000000ULL /
                                     ((unsigned long long)SNAP_FPS * vmax));
    printf("[CAM] SetSensorROI: (%u,%u) %ux%u → VMAX=%lu SHUTTER=%lu → exposure=%lu us (target=%lu us)\n",
           x_start, y_start, width, height,
           (unsigned long)vmax, (unsigned long)shutter,
           (unsigned long)exposure_us, (unsigned long)CAM_EXPOSURE_VALUE);
  }

  /* Step 1: STANDBY (0x3000 = 0x01) with retry */
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;
  HAL_Delay(20);

  /* Step 2: Set HOLD (0x3001 = 0x01) with retry */
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorReg(IMX335_REG_HOLD, 0x01);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;

  /* Step 3: Write AREA3 registers (little-endian 16-bit) with retry */
  data[0] = (uint8_t)(x_start & 0xFFU);
  data[1] = (uint8_t)((x_start >> 8) & 0xFFU);
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorRegMulti(AREA3_ST_ADR_X, data, 2);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;

  data[0] = (uint8_t)(y_start & 0xFFU);
  data[1] = (uint8_t)((y_start >> 8) & 0xFFU);
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorRegMulti(AREA3_ST_ADR_Y, data, 2);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;

  data[0] = (uint8_t)(x_end & 0xFFU);
  data[1] = (uint8_t)((x_end >> 8) & 0xFFU);
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorRegMulti(AREA3_END_ADR_X, data, 2);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;

  data[0] = (uint8_t)(y_end & 0xFFU);
  data[1] = (uint8_t)((y_end >> 8) & 0xFFU);
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorRegMulti(AREA3_END_ADR_Y, data, 2);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;

  /* Step 4: Write VMAX (16-bit at 0x3030-0x3031, NOT 32-bit! 0x3032-0x3033 are OTHER registers.) */
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorReg16(IMX335_REG_VMAX, vmax);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;

  /* Step 5: Write SHUTTER (24-bit) with retry */
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorReg24(IMX335_REG_SHUTTER, shutter);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;

  /* Step 6: Clear HOLD to apply (0x3001 = 0x00) */
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorReg(IMX335_REG_HOLD, 0x00);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;
  HAL_Delay(30);

  /* Step 7: STREAMING (0x3000 = 0x00) */
  for (retry = 0; retry < 3; retry++) {
    ret = CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STREAMING);
    if (ret == 0) break;
    HAL_Delay(5);
  }
  if (ret) return -1;
  HAL_Delay(50);

  printf("[CAM] Sensor ROI configured: %dx%d from (%u,%u)\n",
         width, height, x_start, y_start);
  return 0;
}

int CAM_SetFullFrame(void)
{
  return CAM_SetSensorROI(0, 0, IMX335_SENSOR_W, IMX335_SENSOR_H);
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
#if CAPTURE_MODE == 4
static uint32_t g_callback_capture_ticks[CALLBACK_FRAMES] = {0};
#endif

#if CAM_SENSOR_REG_DEBUG && (CAPTURE_MODE == 4)
/* Call only after pipe stop + standby, from the capture task. No ISP update
   runs between these reads. Do not insert I2C/UART work in the DMA loop. */
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
  /* Same nominal line period as the current driver: 1e6/(4500*30).
     Deliberately do not infer line timing from requested SNAP_FPS.
     This is an estimate, not a timing measurement or per-frame metadata. */
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
#if CAM_NATIVE_ROI
  assert(CAM_NATIVE_ROI_X + conf->capture_width <= sw);
  assert(CAM_NATIVE_ROI_Y + conf->capture_height <= sh);
  roi->width = conf->capture_width;
  roi->height = conf->capture_height;
  roi->offset_x = CAM_NATIVE_ROI_X;
  roi->offset_y = CAM_NATIVE_ROI_Y;
#else
  float rx = (float)sw / conf->capture_width;
  float ry = (float)sh / conf->capture_height;
  float r = MIN(rx, ry);
  assert(r >= 1 && r < 64);
  roi->width  = (uint32_t)MIN(conf->capture_width * r, sw);
  roi->height = (uint32_t)MIN(conf->capture_height * r, sh);
  roi->offset_x = (sw - roi->width + 1) / 2;
  roi->offset_y = (sh - roi->height + 1) / 2;
#endif
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
  dc.output_width  = conf->capture_width;
  dc.output_height = conf->capture_height;
  dc.output_format = cam_conf->dcmipp_output_format;
  dc.output_bpp    = CAM_FormatToBpp(cam_conf->dcmipp_output_format);
  dc.mode          = CMW_Aspect_ratio_manual_roi;
  dc.enable_swap   = cam_conf->is_rgb_swap;
  dc.enable_gamma_conversion = 0;
#if CAM_NATIVE_ROI
  assert(sensor == CMW_IMX335_Sensor);
#else
  assert(conf->capture_width >= conf->capture_height);
#endif
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

  /* Camera I2C can become corrupted if the board was power-cycled or the
     program was interrupted while the camera was active. Retry up to 3 times
     with I2C bus reset between attempts instead of asserting. */
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
  assert(CMW_CAMERA_GetDCMIPPHandle() != NULL);
  assert(CMW_CAMERA_Run() == CMW_ERROR_NONE);
}

void CAM_Deinit(void)
{
  assert(CMW_CAMERA_DeInit() == HAL_OK);
}

#if CAPTURE_MODE == 0
static volatile uint32_t g_single_capture_errors;
#endif
void CMW_CAMERA_PIPE_ErrorCallback(uint32_t pipe) {
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
static int CAM_WaitFrameAfter(uint32_t start_evt, uint32_t timeout_ms, uint32_t *elapsed_ms)
{
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

static int CAM_WaitNextFrameReady(uint32_t timeout_ms, uint32_t *elapsed_ms)
{
  return CAM_WaitFrameAfter(g_frame_event_count, timeout_ms, elapsed_ms);
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
  /* Frame completion, not VSYNC: full-size writes must finish before stop.
     No extra warmup buffers are allocated. Mode 4 uses its unchanged path. */
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
  printf("[CAM] BatchSnap: %lu frames in %lu ms\n", (unsigned long)captured, (unsigned long)elapsed);
#endif
  return (int)captured;
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

#if CAM_SENSOR_REG_DEBUG && (CAPTURE_MODE == 4)
  CAM_DumpImx335AeState("INIT_STANDBY");
#endif

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
  uint32_t last_event = g_frame_event_count;
  if (CMW_CAMERA_DoubleBufferStart(DCMIPP_PIPE1, raw_buf[0], raw_buf[1], CMW_MODE_CONTINUOUS) != CMW_ERROR_NONE) {
#if PERF_DEBUG_LEVEL >= 1
    printf("[CAM] Double-buffer start failed after callback wake\n");
#endif
    CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
    return -1;
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

      if (CAM_WaitFrameAfter(last_event, 120, &frame_ms) != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[CAM] Frame wait timeout at k=%lu\n", (unsigned long)k);
#endif
        break;
      }
      last_event++;
      if (g_frame_event_count != last_event) {
        printf("[CAM] Missed frame completion in callback burst\n");
        captured = 0;
        break;
      }
      frame_seq++;
#if PERF_CAMERA_FRAME_LOG
      frame_times[completed_frames++] = frame_ms;
#endif

      int32_t out_idx = (int32_t)k - (int32_t)CALLBACK_WARMUP_FRAMES;
      /* Retarget the completed hardware slot before doing any cache work.
         After its final retained frame, send further DMA back to scratch. */
      int32_t arm_idx = out_idx + 2;
      uint32_t slot = (frame_seq - 1U) % 2U;
      if (arm_idx >= 0) {
        uint8_t *next = arm_idx < CALLBACK_FRAMES ?
                        batch_buf + (uint32_t)arm_idx * frame_size : raw_buf[slot];
        uint32_t address_slot = slot == 0U ? DCMIPP_MEMORY_ADDRESS_0 : DCMIPP_MEMORY_ADDRESS_1;
        if (HAL_DCMIPP_PIPE_SetMemoryAddress(h, DCMIPP_PIPE1, address_slot,
                                            (uint32_t)next) != HAL_OK) {
          printf("[CAM] Failed to arm callback burst DMA slot\n");
          captured = 0;
          break;
        }
      }

      if (out_idx >= 0) {
        uint8_t *dest = batch_buf + (uint32_t)out_idx * frame_size;
        SCB_InvalidateDCache_by_Addr((uint32_t *)dest, frame_size);
        g_callback_capture_ticks[(uint32_t)out_idx] = HAL_GetTick();
        captured++;
      }
    }
  }

  /* ---------------------------------------------------------------
     Step 6: Stop pipe ONCE after ALL frames captured.
     --------------------------------------------------------------- */
  /* Return to standby. */
  PERF_MARK(g_perf_timer, CAM_STOP);
  if (HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0) != HAL_OK) {
    printf("[CAM] Callback burst stop failed, capture disabled until reboot\n");
    g_callback_ready = 0;
    captured = 0;
  }
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
  PERF_MARK(g_perf_timer, CAM_DEINIT);
#if PERF_DEBUG_LEVEL >= 1
  uint32_t capture_elapsed_ms = HAL_GetTick() - t0;
#endif
#if PERF_CAMERA_FRAME_LOG
  /* UART output must not delay rearming or stopping the DMA pipe. */
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

/* ================================================================
   ROW DIAGNOSTIC — print mean Y of key rows to detect dark/top rows.
   Samples the Y component (byte 0 of each pixel pair) across a row.
   ================================================================ */
static void CAM_DumpRowStats(const uint8_t *buf, uint16_t w, uint16_t h,
                              const char *label)
{
  const uint32_t pitch = (uint32_t)w * 2;  /* YUV422: 2 bytes/pixel */
  /* Key row indices: first, quarter, half, three-quarter, last */
  uint32_t rows[] = {0, h/4, h/2, 3*h/4, h > 0 ? h-1 : 0};
  uint32_t sample_pix[] = {0, w/4, w/2, 3*w/4, w > 0 ? w-1 : 0};
  int nr = sizeof(rows)/sizeof(rows[0]);
  int np = sizeof(sample_pix)/sizeof(sample_pix[0]);

  for (int ri = 0; ri < nr; ri++) {
    /* Compute mean Y across the row by sampling at 5 x-positions */
    uint32_t sum_y = 0;
    for (int pi = 0; pi < np; pi++) {
      uint32_t off = rows[ri] * pitch + sample_pix[pi] * 2;
      sum_y += buf[off];
    }
    printf("[CAM] %s row=%lu meanY=%u\n",
           label, (unsigned long)rows[ri], (unsigned)(sum_y / np));
  }
}

/* ================================================================
   CAM_ResetCallbackReady  —  Allow re-init for second capture.
   After Stage 3's CAM_CaptureROI does DeInit→Init, the DCMIPP
   handle is refreshed but g_callback_ready remains set from boot's
   CAM_CallbackInit.  Reset it so CAM_CallbackBatchSnap re-initializes
   cleanly on the next capture cycle.
   ================================================================ */
void CAM_ResetCallbackReady(void)
{
  g_callback_ready = 0;
}

/* ================================================================
   CAM_CaptureROI  —  Stage 3 AREA3 ROI capture after Stage 1 detection.
   Performs a FULL CMW_CAMERA_DeInit → Init cycle so the DCMIPP pipe
   is completely clean before reconfiguring for the ROI crop.  Pipe-
   only crop changes without full re-init produce corrupted frames
   (repeated/dark/green columns) because the DCMIPP state machine
   does not cleanly transition between different crop sizes.
   Programs sensor AREA3 registers for the window, then uses the
   callback-batch approach to capture CALLBACK_FRAMES frames.
   Returns number of frames captured, or -1 on error.
   ================================================================ */
int CAM_CaptureROI(uint8_t *buf, uint32_t buf_size,
                   uint16_t roi_x, uint16_t roi_y,
                   uint16_t roi_w, uint16_t roi_h)
{
  uint32_t roi_frame_sz = (uint32_t)roi_w * roi_h * 2;
  if (!buf || buf_size < roi_frame_sz) return -1;

  uint32_t max_frames = buf_size / roi_frame_sz;
  if (max_frames > CALLBACK_FRAMES) max_frames = CALLBACK_FRAMES;
  if (max_frames == 0) return -1;

  uint32_t captured = 0;

  printf("[CAM] Stage 3: AREA3 (%u,%u) %ux%u native 2592x1944 capture %lu frames\n",
         roi_x, roi_y, roi_w, roi_h, (unsigned long)max_frames);

  /* ---------------------------------------------------------------
     Step 0: Full DeInit->Init to reset DCMIPP + CSI receiver cleanly.
     --------------------------------------------------------------- */
  {
    CMW_CameraInit_t cam_conf;
    CMW_CAMERA_DeInit();
    HAL_Delay(50);
    cam_conf.width  = SENSOR_WIDTH;
    cam_conf.height = SENSOR_HEIGHT;
    cam_conf.fps = SNAP_FPS;
    cam_conf.mirror_flip = CAM_getFlipMode(sensor);
    if (CMW_CAMERA_Init(&cam_conf, NULL) != CMW_ERROR_NONE) {
      printf("[CAM] Stage 3: CMW_CAMERA_Init failed\n");
      return -1;
    }
  }

  /* Set exposure/gain after Init's minimum — avoids Y=16 from CMW_CAMERA_Init. */
  CMW_CAMERA_SetExposure(CAM_EXPOSURE_VALUE);
  CMW_CAMERA_SetGain(CAM_GAIN_VALUE);

  /* ---------------------------------------------------------------
     Step 1: Configure DCMIPP pipe for 1:1 at the ROI size first.
     Offset (0,0) since AREA3 handles the spatial crop.
     --------------------------------------------------------------- */
  {
    uint32_t hw_pitch;
    CMW_DCMIPP_Conf_t dc = {0};
    extern DCMIPP_HandleTypeDef hcamera_dcmipp;
    dc.output_width      = roi_w;
    dc.output_height     = roi_h;
    dc.output_format     = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
    dc.output_bpp        = 2;
    dc.mode              = CMW_Aspect_ratio_manual_roi;
    dc.enable_swap       = 0;
    dc.enable_gamma_conversion = 0;
    dc.manual_conf.width  = roi_w;
    dc.manual_conf.height = roi_h;
    dc.manual_conf.offset_x = 0;
    dc.manual_conf.offset_y = 0;
    hcamera_dcmipp.PipeState[DCMIPP_PIPE1] = HAL_DCMIPP_PIPE_STATE_RESET;
    if (CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &dc, &hw_pitch) != CMW_ERROR_NONE) {
      printf("[CAM] Stage 3: SetPipeConfig failed\n");
      CMW_CAMERA_DeInit();
      return -1;
    }
    hcamera_dcmipp.PipeState[DCMIPP_PIPE1] = HAL_DCMIPP_PIPE_STATE_READY;
    if (dc.output_format == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
      CAM_EnableYuv(DCMIPP_PIPE1);
  }

  /* ---------------------------------------------------------------
     Step 2: Program sensor AREA3 (AREA1) registers for the ROI window.
     CRITICAL: Do this BEFORE starting the DCMIPP pipe, while sensor
     is in the init standby state.  CAM_SetSensorROI does:
       STANDBY → HOLD → write AREA1+VMAX+SHUTTER → STREAMING.
     The DCMIPP pipe is NOT running yet, so the CSI receiver stays clean.
     --------------------------------------------------------------- */
  if (CAM_SetSensorROI(roi_x, roi_y, roi_w, roi_h) != 0) {
    printf("[CAM] Stage 3: CAM_SetSensorROI failed\n");
    CMW_CAMERA_DeInit();
    return -1;
  }

  /* ---------------------------------------------------------------
     Step 3: Start double-buffer continuous capture.  The sensor is
     already configured for the AREA3 window and streaming.
     Camera_Drv.Start() writes streaming mode (0x3000=0x00) which is
     redundant but harmless — it does NOT reset the AREA1 registers.
     --------------------------------------------------------------- */
  {
    extern uint8_t save_buf[];
    if (CMW_CAMERA_DoubleBufferStart(DCMIPP_PIPE1, capture_buf, save_buf, CMW_MODE_CONTINUOUS) != CMW_ERROR_NONE) {
      printf("[CAM] Stage 3: DoubleBufferStart failed\n");
      CMW_CAMERA_DeInit();
      return -1;
    }
  }

  /* Double-check actual exposure/gain after AREA3 override. */
  {
    uint32_t actual_exposure = 0;
    CMW_CAMERA_GetExposure(&actual_exposure);
    printf("[CAM] Stage 3: ROI active — CMW exposure=%lu us (target=%lu us)\n",
           (unsigned long)actual_exposure, (unsigned long)CAM_EXPOSURE_VALUE);
  }

  /* ---------------------------------------------------------------
     Step 4: Walk through warmup AND capture frames in one loop.
     k in [0, WARMUP) are discarded; k in [WARMUP, WARMUP+CAPTURE)
     are saved into successive batch_buf slots.  DMA addresses are
     armed 2 frames ahead for zero-copy continuous capture.
     --------------------------------------------------------------- */
  {
    DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
    const uint32_t total = (uint32_t)CALLBACK_WARMUP_FRAMES + max_frames;

    for (uint32_t k = 0; k < total; k++) {
      if (CAM_WaitNextFrameReady(120, NULL) != 0) {
        printf("[CAM] Stage 3: frame wait timeout at k=%lu\n", (unsigned long)k);
        break;
      }

      int32_t out_idx = (int32_t)k - (int32_t)CALLBACK_WARMUP_FRAMES;
      if (out_idx >= 0) {
        uint8_t *dest = buf + ((uint32_t)out_idx * roi_frame_sz);
        SCB_InvalidateDCache_by_Addr((uint32_t*)dest, roi_frame_sz);
        captured++;
        CAM_DumpRowStats(dest, roi_w, roi_h, "Stage3_ROI");
      }

      /* Arm the address that JUST freed for reuse 2 frames from now. */
      int32_t arm_idx = out_idx + 2;
      if (arm_idx >= 0 && arm_idx < (int32_t)max_frames) {
        uint32_t parity = k % 2U;
        HAL_DCMIPP_PIPE_SetMemoryAddress(h, DCMIPP_PIPE1,
                                          parity == 0U ? DCMIPP_MEMORY_ADDRESS_0 : DCMIPP_MEMORY_ADDRESS_1,
                                          (uint32_t)(buf + ((uint32_t)arm_idx * roi_frame_sz)));
      }
    }

    printf("[CAM] Stage 3: Captured %d/%lu ROI frames\n", (int)captured, (unsigned long)max_frames);
  }

  /* ---------------------------------------------------------------
     Step 5: Stop pipe.  Caller restores full-frame.
     --------------------------------------------------------------- */
  {
    DCMIPP_HandleTypeDef *h = CMW_CAMERA_GetDCMIPPHandle();
    HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
  }

  return (int)captured;
}

void CAM_Standby(void)
{
  CAM_WriteSensorReg(IMX335_REG_MODE_SELECT, IMX335_MODE_STANDBY);
}

int CAM_SetPipeResolution(uint16_t width, uint16_t height,
                           uint16_t offset_x, uint16_t offset_y)
{
  extern DCMIPP_HandleTypeDef hcamera_dcmipp;
  uint32_t hw_pitch;
  CMW_DCMIPP_Conf_t sdc = {0};
  sdc.output_width      = width;
  sdc.output_height     = height;
  sdc.output_format     = DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1;
  sdc.output_bpp        = 2;
  sdc.mode              = CMW_Aspect_ratio_manual_roi;
  sdc.enable_swap       = 0;
  sdc.enable_gamma_conversion = 0;
  sdc.manual_conf.width  = width;
  sdc.manual_conf.height = height;
  sdc.manual_conf.offset_x = offset_x;
  sdc.manual_conf.offset_y = offset_y;
  hcamera_dcmipp.PipeState[DCMIPP_PIPE1] = HAL_DCMIPP_PIPE_STATE_RESET;
  if (CMW_CAMERA_SetPipeConfig(DCMIPP_PIPE1, &sdc, &hw_pitch) != CMW_ERROR_NONE) {
    printf("[CAM] SetPipeResolution %ux%u failed\n", width, height);
    return -1;
  }
  hcamera_dcmipp.PipeState[DCMIPP_PIPE1] = HAL_DCMIPP_PIPE_STATE_READY;
  if (sdc.output_format == DCMIPP_PIXEL_PACKER_FORMAT_YUV422_1)
    CAM_EnableYuv(DCMIPP_PIPE1);
  printf("[CAM] Pipe reconfigured: %ux%u offset %u,%u\n", width, height, offset_x, offset_y);
  return 0;
}
