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
#define VL53L5CX_DET_RESOLUTION       4

#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_NUM_ZONES        64
#elif VL53L5CX_DET_RESOLUTION == 4
#define VL53L5CX_DET_NUM_ZONES        16
#else
#error "VL53L5CX_DET_RESOLUTION must be 4 or 8"
#endif

/* Resolution-specific baseline samples:
   8x8 zones are smaller and noisier, need more samples for stable baseline. */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_BASELINE_SAMPLES 30      /* 8x8: more samples for stability */
#else
#define VL53L5CX_DET_BASELINE_SAMPLES 10      /* 4x4: 10 samples enough */
#endif

/* Resolution-specific thresholds:
   8x8 zones are 1/4 the area of 4x4 zones, so signal is noisier and
   motion indicator is more sensitive. Use higher thresholds for 8x8. */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_THRESHOLD_PCT    15      /* 8x8: higher signal drop threshold (noisier) */
#define VL53L5CX_DET_MOTION_THRESH    100     /* 8x8: higher motion threshold (more sensitive) */
#define VL53L5CX_DET_MIN_AFFECTED_ZONES 1     /* 8x8: require 2 zones (reduce false triggers) */
#else
#define VL53L5CX_DET_THRESHOLD_PCT     6      /* 4x4: lower signal drop threshold */
#define VL53L5CX_DET_MOTION_THRESH    40      /* 4x4: lower motion threshold */
#define VL53L5CX_DET_MIN_AFFECTED_ZONES 1     /* 4x4: single zone triggers */
#endif

/* Motion indicator tuning (applied at ST API level):
   These are set in VL53L5CX_Configure() after motion init.

   - MOTION_MIN_ZONES: min zones that must detect motion for global trigger.
     ST API: min_nb_for_global_detection. Set to 1 to disable.
   - MOTION_PERSIST_FRAMES: frames to accumulate before motion fires.
     ST API: nb_of_temporal_accumulations. Set to 1 to disable.
   - MOTION_EXTRA_NOISE_SIGMA: extra noise floor to ignore small fluctuations.
     ST API: extra_noise_sigma. Higher = more tolerant of ambient noise.
     Set to 0 to disable. */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_MOTION_MIN_ZONES       2     /* 8x8: require 2 zones for ST motion API */
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  3     /* 8x8: more persistence (3 frames) */
#define VL53L5CX_DET_MOTION_EXTRA_NOISE     50    /* 8x8: add noise floor */
#else
#define VL53L5CX_DET_MOTION_MIN_ZONES       1     /* 4x4: single zone for ST motion API */
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  2     /* 4x4: 2 frame persistence */
#define VL53L5CX_DET_MOTION_EXTRA_NOISE     0     /* 4x4: no extra noise */
#endif

/* Adaptive baseline filter:
   When enabled, slowly updates baseline using EMA for zones NOT
   affected by detection. Compensates for natural drift (temperature,
   ambient light) without chasing real objects. */
#define VL53L5CX_DET_ADAPTIVE_ENABLED 0
#define VL53L5CX_DET_EMA_DIVIDER      256     /* Smoothing factor */

/* Debug output (legacy, unused - prints are in main.c insect_task) */
#define VL53L5CX_DET_DEBUG_FRAME_INTERVAL 0

/* Streamlined ZFRAME format (always enabled):
   Sends only the variables needed for insect detection:
     ZFRAME,temp,sig0,base0,dist0,motion0,sig1,base1,dist1,motion1,...
   Per zone: 4 fields (signal, baseline_signal, distance, motion)
   Total for 8x8: 1 + 64*4 = 257 values (vs. 1 + 64*12 + 64 = 1345 in extended)
   Python parses this compact format for all plots and heatmaps. */
#define VL53L5CX_DET_ZFRAME_COMPACT 1

/* ================================================================
   Detection Result Structure
   ================================================================ */

/* Trigger source flags */
#define VL53L5CX_TRIG_SIGNAL  0x01  /* Signal drop triggered detection */
#define VL53L5CX_TRIG_MOTION  0x02  /* Motion indicator triggered detection */
#define VL53L5CX_TRIG_BOTH    0x03  /* Both signal and motion triggered */

typedef struct {
    uint8_t  insect_detected;       /* 1 if insect detected this frame */
    uint8_t  trigger_source;        /* VL53L5CX_TRIG_SIGNAL / MOTION / BOTH */
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
