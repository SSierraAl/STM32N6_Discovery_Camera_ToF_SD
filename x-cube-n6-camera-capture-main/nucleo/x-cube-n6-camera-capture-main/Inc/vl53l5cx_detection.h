/**
 * *****************************************************************************
 * @file    vl53l5cx_detection.h
 * @brief   VL53L5CX ToF Sensor - Clean Detection API
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

/* Resolution-specific baseline samples */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_BASELINE_SAMPLES 30
#else
#define VL53L5CX_DET_BASELINE_SAMPLES 10
#endif

/* Resolution-specific detection thresholds */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_THRESHOLD_PCT      15      /* 8x8: higher signal drop threshold */
#define VL53L5CX_DET_MOTION_THRESH      100     /* 8x8: higher motion threshold */
#define VL53L5CX_DET_MIN_AFFECTED_ZONES 2       /* 8x8: require 2 zones */
#else
#define VL53L5CX_DET_THRESHOLD_PCT      6       /* 4x4: lower signal drop threshold */
#define VL53L5CX_DET_MOTION_THRESH      40      /* 4x4: lower motion threshold */
#define VL53L5CX_DET_MIN_AFFECTED_ZONES 1       /* 4x4: single zone triggers */
#endif

/* Motion indicator tuning (ST API level) */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_MOTION_MIN_ZONES       2
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  3
#define VL53L5CX_DET_MOTION_EXTRA_NOISE     50
#else
#define VL53L5CX_DET_MOTION_MIN_ZONES       1
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  2
#define VL53L5CX_DET_MOTION_EXTRA_NOISE     0
#endif

/* Minimum signal threshold:
   Zones with signal below this are skipped during baseline learning
   and detection. Prevents false triggers from noisy/blind zones. */
#define VL53L5CX_DET_MIN_SIGNAL 500

/* ================================================================
   Baseline Refresh Modes (choose ONE)
   ================================================================

   MODE 1 — PERIODIC (time-based):
     Refreshes baseline every N frames regardless of detections.
     Use when you want predictable, fixed-interval refreshes.
     Example: every 500 frames (~33s at 15Hz).

   MODE 2 — ADAPTIVE (detection-based):
     Counts detections in a sliding window. If more than MAX_DETECTIONS
     occur within the window, baseline is refreshed.
     Use when environment is mostly stable but occasionally changes.

   IMPORTANT: Only enable ONE mode at a time.
   - For periodic: set PERIODIC_RESTART_ENABLED=1, ADAPTIVE_REFRESH_ENABLED=0
   - For adaptive: set PERIODIC_RESTART_ENABLED=0, ADAPTIVE_REFRESH_ENABLED=1

   Frame counting method:
     A static counter increments on EVERY call to VL53L5CX_Update().
     It is NOT reset by detections. It wraps at the window/interval size.
     FreeRTOS safety: the counter is only read/written by the ToF task
     (no shared access), so no mutex is needed.
     vTaskDelay() is used for blocking waits (not HAL_Delay), keeping
     the RTOS scheduler responsive.
   ================================================================ */

/* --- MODE 1: Periodic Restart --- */
#define VL53L5CX_DET_PERIODIC_RESTART_ENABLED   0
#define VL53L5CX_DET_PERIODIC_RESTART_INTERVAL  500  /* Refresh every N frames */

/* --- MODE 2: Adaptive Refresh (time-based) --- */
#define VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED   1
#define VL53L5CX_DET_REFRESH_WINDOW_SECS        10   /* Real-time window in seconds */
#define VL53L5CX_DET_MAX_DETECTIONS             5    /* Max detections in window before refresh */

/* ================================================================
   Debug Output Configuration
   ================================================================ */

/* DEBUG MODE 1: Compact ZFRAME (for zone_monitor.py real-time plots)
   Format: ZFRAME,temp,sig0,dist0,base_sig0,base_dist0,motion0,...
   Per zone: 5 fields (signal, distance, baseline_signal, baseline_distance, motion)
   Enables: Signal vs Baseline plot, Distance plot, Motion plot, Drop% heatmap */
#define VL53L5CX_DET_DEBUG_ZFRAME     0
#define VL53L5CX_DET_DEBUG_ZFRAME_INT 1    /* Emit ZFRAME every N frames in Update() */

/* DEBUG MODE 2: ALLPARAM (for datalogger.py + analysis.py)
   Format: ALLPARAM,temp,sig0,base_sig0,dist0,base_dist0,amb0,sigma0,reflect0,...
   Per zone: 12 fields (all parameters)
   Enables: Full parameter logging, detailed analysis, all heatmaps */
#define VL53L5CX_DET_DEBUG_ALLPARAMS    0
#define VL53L5CX_DET_DEBUG_ALLPARAM_INT 50   /* Emit ALLPARAM every N frames */

/* NOTE: Both ZFRAME and ALLPARAM can be enabled simultaneously.
   - ZFRAME: emitted every frame (compact, for real-time monitoring)
   - ALLPARAM: emitted every 50 frames (detailed, for datalogging + analysis)
   - For lower UART bandwidth: disable ZFRAME, keep ALLPARAM
   - For real-time only: keep ZFRAME, disable ALLPARAM */

/* ================================================================
   Detection Result Structure
   ================================================================ */

/* Trigger source flags */
#define VL53L5CX_TRIG_SIGNAL  0x01
#define VL53L5CX_TRIG_MOTION  0x02
#define VL53L5CX_TRIG_BOTH    0x03

typedef struct {
    uint8_t  insect_detected;
    uint8_t  trigger_source;
    uint8_t  affected_count;
    uint8_t  affected_zones[VL53L5CX_DET_NUM_ZONES];
    uint32_t affected_drop[VL53L5CX_DET_NUM_ZONES];
    uint8_t  valid_measurements;
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
int  VL53L5CX_Update(void);
int  VL53L5CX_IsInsectDetected(void);
VL53L5CX_DetectionResult_t VL53L5CX_GetResult(void);

/* --- Debug / Diagnostics --- */
void VL53L5CX_PrintAllZoneParams(void);
void VL53L5CX_PrintZFrame(void);
void VL53L5CX_PrintBaselineFrame(void);
int  VL53L5CX_ScanI2CBus(void);

/* --- Legacy Test Functions --- */
void VL53L5CX_Validate(void);
void VL53L5CX_ReadingTest(void);
void VL53L5CX_MotionTest(void);

#endif /* VL53L5CX_DETECTION_H */
