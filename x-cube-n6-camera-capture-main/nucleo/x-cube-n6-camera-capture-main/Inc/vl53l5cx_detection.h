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

/* Resolution selector:
   - 4 = VL53L5CX_RESOLUTION_4X4  (16 zones, faster, lower granularity)
   - 8 = VL53L5CX_RESOLUTION_8X8  (64 zones, slower, higher granularity) */
#define VL53L5CX_DET_RESOLUTION       8

#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_NUM_ZONES        64
#elif VL53L5CX_DET_RESOLUTION == 4
#define VL53L5CX_DET_NUM_ZONES        16
#else
#error "VL53L5CX_DET_RESOLUTION must be 4 or 8"
#endif

#define VL53L5CX_DET_BASELINE_SAMPLES 10      /* Frames to learn baseline */
#define VL53L5CX_DET_THRESHOLD_PCT    6       /* Signal drop % for detection */
#define VL53L5CX_DET_MOTION_THRESH    40      /* Motion indicator threshold (per zone) */

/* Adaptive baseline filter:
   When enabled, slowly updates baseline using EMA for zones NOT
   affected by detection. Compensates for natural drift (temperature,
   ambient light) without chasing real objects. */
#define VL53L5CX_DET_ADAPTIVE_ENABLED 0
#define VL53L5CX_DET_EMA_DIVIDER      256     /* Smoothing factor */

/* Debug output for Python visualization:
   Set to 0 because debug prints are now in main.c insect_detection_task.
   The task calls VL53L5CX_PrintZFrame() and VL53L5CX_PrintAllZoneParams() directly. */
#define VL53L5CX_DET_DEBUG_FRAME_INTERVAL 0

/* Debug: Print ALL ToF parameters per zone (detailed table)
   Set to 1 to enable, 0 to disable. Only affects console verbosity. */
#define VL53L5CX_DET_DEBUG_ALLPARAMS    1
#define VL53L5CX_DET_DEBUG_ALLPARAM_INT 10

/* Enable extended ZFRAME with ALL parameters:
   1 = 209 fields per line (all params + motion)
   0 = 65 fields per line (legacy: sig, base, dist, bdist only)
   Python auto-detects both formats. */
#define VL53L5CX_DET_DEBUG_EXTENDED_ZFRAME 1

/* ================================================================
   Detection Result Structure
   ================================================================ */

typedef struct {
    uint8_t  insect_detected;       /* 1 if insect detected this frame */
    uint8_t  affected_count;        /* Number of affected zones */
    uint8_t  affected_zones[VL53L5CX_DET_NUM_ZONES];  /* Zone indices */
    uint32_t affected_drop[VL53L5CX_DET_NUM_ZONES];   /* Drop % per affected zone */
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

/* --- Debug / Diagnostics --- */
void VL53L5CX_PrintAllZoneParams(void);
void VL53L5CX_PrintZFrame(void);
int  VL53L5CX_ScanI2CBus(void);                      /* Returns number of devices found */

/* --- Legacy Test Functions --- */
void VL53L5CX_Validate(void);
void VL53L5CX_ReadingTest(void);
void VL53L5CX_MotionTest(void);

#endif /* VL53L5CX_DETECTION_H */
