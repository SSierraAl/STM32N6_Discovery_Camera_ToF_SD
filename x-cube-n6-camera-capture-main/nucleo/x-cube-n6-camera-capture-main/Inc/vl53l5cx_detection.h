/**
 * *****************************************************************************
 * @file    vl53l5cx_detection.h
 * @brief   VL53L5CX ToF Sensor - Modular Detection API
 *
 *           Provides initialization, configuration, baseline learning,
 *           and insect detection for the VL53L5CX 4x4/8x8 ToF sensor.
 *
 *           Usage:
 *             VL53L5CX_Init(&hi2c1);
 *             VL53L5CX_Configure(VL53L5CX_RESOLUTION_4X4, 800, 15);
 *             VL53L5CX_StartRanging();
 *             VL53L5CX_LearnBaseline();
 *             while (1) {
 *                 if (VL53L5CX_Update()) {
 *                     uint8_t insect = VL53L5CX_IsInsectDetected();
 *                     // ...
 *                 }
 *             }
 *
 * *****************************************************************************
 */

#ifndef VL53L5CX_DETECTION_H
#define VL53L5CX_DETECTION_H

#include <stdint.h>
#include "stm32n6xx_hal.h"
#include "vl53l5cx_api.h"

/* ================================================================
   Configuration Constants
   ================================================================ */

#define VL53L5CX_DET_NUM_ZONES        16      /* 4x4 resolution = 16 zones */
#define VL53L5CX_DET_BASELINE_SAMPLES 10      /* Frames to learn baseline */
#define VL53L5CX_DET_THRESHOLD_PCT    6       /* Signal drop % for detection */
#define VL53L5CX_DET_MOTION_THRESH    20      /* Motion indicator threshold */

/* Adaptive baseline filter:
   When enabled, slowly updates baseline using EMA for zones NOT
   affected by detection. Compensates for natural drift (temperature,
   ambient light) without chasing real objects. */
#define VL53L5CX_DET_ADAPTIVE_ENABLED 0
#define VL53L5CX_DET_EMA_DIVIDER      256     /* Smoothing factor */

/* Debug output for Python visualization:
   Prints ZFRAME line every N frames. Set to 0 to disable. */
#define VL53L5CX_DET_DEBUG_FRAME_INTERVAL 2

/* Debug: Print ALL ToF parameters per zone (detailed table)
   Set to 1 to enable, 0 to disable. */
#define VL53L5CX_DET_DEBUG_ALLPARAMS    0
#define VL53L5CX_DET_DEBUG_ALLPARAM_INT 10

/* Zone mask presets */
#define VL53L5CX_DET_ZONES_ALL        0xFFFF
#define VL53L5CX_DET_ZONES_CENTER_4   0x0660
#define VL53L5CX_DET_ZONES_TOP_ROW    0x000F
#define VL53L5CX_DET_ZONES_BOTTOM_ROW 0xF000
#define VL53L5CX_DET_ZONES_LEFT_COL   0x1111
#define VL53L5CX_DET_ZONES_RIGHT_COL  0x8888
#define VL53L5CX_DET_ZONES_CORNERS    0x8009

/* ================================================================
   Detection Result Structure
   ================================================================ */

typedef struct {
    uint8_t  insect_detected;       /* 1 if insect detected this frame */
    uint8_t  affected_count;        /* Number of affected zones */
    uint8_t  affected_zones[16];    /* Zone indices */
    uint32_t affected_drop[16];     /* Drop % per affected zone */
    uint8_t  valid_measurements;    /* Number of valid zone readings */
} VL53L5CX_DetectionResult_t;

/* ================================================================
   Public API
   ================================================================ */

/* --- Initialization --- */
int  VL53L5CX_Init(I2C_HandleTypeDef *hi2c);
void VL53L5CX_PowerUp(void);
void VL53L5CX_PowerDown(void);

/* --- Configuration --- */
void VL53L5CX_Configure(uint8_t resolution, int integration_ms, int freq_hz);
void VL53L5CX_StartRanging(void);
void VL53L5CX_StopRanging(void);

/* --- Data Access --- */
int  VL53L5CX_WaitForDataReady(uint32_t timeout_ms);
int  VL53L5CX_GetData(void);
void VL53L5CX_GetZoneData(uint8_t zone, uint32_t *signal, uint16_t *distance, uint8_t *status);
void VL53L5CX_GetBaselineData(uint8_t zone, uint32_t *signal, uint16_t *distance);
int  VL53L5CX_IsZoneValid(uint8_t zone);
int  VL53L5CX_IsBaselineReady(void);

/* --- Baseline Management --- */
void VL53L5CX_LearnBaseline(void);
void VL53L5CX_ResetBaseline(void);

/* --- Detection --- */
int  VL53L5CX_Update(void);                          /* Returns 1 if new data processed */
int  VL53L5CX_IsInsectDetected(void);                /* Returns 1 if insect detected */
VL53L5CX_DetectionResult_t VL53L5CX_GetResult(void); /* Get last detection result */

/* --- Zone Mask --- */
void VL53L5CX_SetZoneMask(uint16_t mask);
void VL53L5CX_EnableZoneFilter(uint8_t enable);
void VL53L5CX_PrintZoneMask(void);
uint16_t VL53L5CX_GetZoneMask(void);

/* --- Debug / Diagnostics --- */
void VL53L5CX_PrintAllZoneParams(void);
void VL53L5CX_PrintZFrame(void);
int  VL53L5CX_ScanI2CBus(void);                      /* Returns number of devices found */

/* --- Legacy Test Functions --- */
void VL53L5CX_Validate(void);
void VL53L5CX_ReadingTest(void);
void VL53L5CX_MotionTest(void);

#endif /* VL53L5CX_DETECTION_H */