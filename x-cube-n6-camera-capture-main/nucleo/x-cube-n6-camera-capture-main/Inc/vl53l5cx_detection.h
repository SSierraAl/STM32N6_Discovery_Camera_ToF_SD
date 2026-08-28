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
    Dual Sensor Configuration
    ================================================================ */

/** Enable dual ToF sensor mode.
    0 = Single sensor mode: camera ToF always on, standard VL53L5CX_Update() loop
    1 = Dual sensor mode:
        - External sensor (secondary, placed before FOV) is continuously ON
        - Camera ToF (primary sensor) is in SLEEP mode by default
        - When external sensor detects motion/signal drop, it wakes the camera ToF
        - Camera ToF stays active for DUAL_WAKE_DURATION_MS, then returns to sleep */
#define VL53L5CX_DUAL_SENSOR              0

/** I2C addresses for dual sensor mode.
    Primary (camera ToF):    always at 0x29, near camera, sleep mode by default
    External (guardian):     at 0x62, before FOV, continuously monitoring */
#define VL53L5CX_PRIMARY_ADDRESS          0x29
#define VL53L5CX_EXTERNAL_ADDRESS         0x62

/** Camera ToF wake duration (ms).
    When the external sensor detects something, the camera ToF is woken up
    and remains active for this duration before returning to sleep mode. */
#define VL53L5CX_DUAL_WAKE_DURATION_MS    5000

/** External sensor detection confirmation frames.
    Number of consecutive detection frames required from the external sensor
    before waking the camera ToF. Prevents false triggers from momentary noise. */
#define VL53L5CX_DUAL_CONFIRM_FRAMES      2

/** Secondary sensor baseline samples (can be fewer for faster init). */
#define VL53L5CX_SENSOR2_BASELINE_SAMPLES 15

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
#define VL53L5CX_DET_BASELINE_SAMPLES 20
#endif

/* Resolution-specific detection thresholds */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_THRESHOLD_PCT      15      /* 8x8: higher signal drop threshold */
#define VL53L5CX_DET_MOTION_THRESH      100     /* 8x8: higher motion threshold */
#define VL53L5CX_DET_MIN_AFFECTED_ZONES 2       /* 8x8: require 2 zones */
#else
#define VL53L5CX_DET_THRESHOLD_PCT      6       /* 4x4: lower signal drop threshold */
#define VL53L5CX_DET_MOTION_THRESH      60      /* 4x4: lower motion threshold */
#define VL53L5CX_DET_MIN_AFFECTED_ZONES 1       /* 4x4: single zone triggers */
#endif

/* Motion indicator tuning (ST API level) */
#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_MOTION_MIN_ZONES       2
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  3
#define VL53L5CX_DET_MOTION_EXTRA_NOISE     50
#else
#define VL53L5CX_DET_MOTION_MIN_ZONES       1
#define VL53L5CX_DET_MOTION_PERSIST_FRAMES  1
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
#define VL53L5CX_DET_REFRESH_WINDOW_SECS        15   /* Real-time window in seconds */
#define VL53L5CX_DET_MAX_DETECTIONS             3    /* Max detections in window before refresh */

/* ================================================================
   Debug Output Configuration
   ================================================================ */

/* DEBUG MODE 1: Compact ZFRAME (for zone_monitor.py real-time plots)
   Format: ZFRAME,temp,sig0,dist0,base_sig0,base_dist0,motion0,...
   Per zone: 5 fields (signal, distance, baseline_signal, baseline_distance, motion)
   Enables: Signal vs Baseline plot, Distance plot, Motion plot, Drop% heatmap */
#define VL53L5CX_DET_DEBUG_ZFRAME     1
#define VL53L5CX_DET_DEBUG_ZFRAME_INT 1    /* Emit ZFRAME every N frames in Update() */

/* DEBUG MODE 2: ALLPARAM (for datalogger.py + analysis.py)
   Format: ALLPARAM,temp,sig0,base_sig0,dist0,base_dist0,amb0,sigma0,reflect0,...
   Per zone: 12 fields (all parameters)
   Enables: Full parameter logging, detailed analysis, all heatmaps */
#define VL53L5CX_DET_DEBUG_ALLPARAMS    0
#define VL53L5CX_DET_DEBUG_ALLPARAM_INT 5   /* Emit ALLPARAM every N frames */

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

/** Unified power-up: selects single or dual sensor sequence automatically.
    When VL53L5CX_DUAL_SENSOR==1: powers both ToF devices, re-addresses external to 0x62.
    When VL53L5CX_DUAL_SENSOR==0: powers single external ToF at 0x29.
    Call this from main(). */
void VL53L5CX_PowerUp(void);

/** Power down the ToF sensor. */
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

/* ================================================================
    Primary Sensor State Machine (Dual Mode)
    ================================================================
    In dual sensor mode, the PRIMARY sensor (camera ToF) alternates between
    sleep and active states based on external sensor detections.
    ================================================================ */

typedef enum {
    PRIMARY_STATE_SLEEP,      // In ST sleep mode (VL53L5CX_POWER_MODE_SLEEP)
    PRIMARY_STATE_WAKING,     // Wake sequence in progress
    PRIMARY_STATE_ACTIVE,     // Ranging active, detecting
    PRIMARY_STATE_RETURNING   // Timer expired, returning to sleep
} PrimaryState_t;

/* ================================================================
    External Sensor (Secondary) State Machine (Dual Mode)
    ================================================================
    The external sensor is always ON, continuously monitoring for motion.
    ================================================================ */

typedef enum {
    EXTERNAL_STATE_IDLE,          // Not running
    EXTERNAL_STATE_MONITORING,    // Continuously monitoring
    EXTERNAL_STATE_DETECTED,      // Detection triggered, waking primary
    EXTERNAL_STATE_WAITING        // Waiting for primary to complete
} ExternalState_t;

/* ================================================================
    Primary Sensor Power Management API (Dual Mode)
    ================================================================
    These functions control the camera ToF sleep/wake cycle.
    Only active when VL53L5CX_DUAL_SENSOR == 1.
    ================================================================ */

/** Put primary sensor (camera ToF) into ST sleep mode.
    Stops ranging, enters VL53L5CX_POWER_MODE_SLEEP.
    Firmware and configuration are retained. */
void VL53L5CX_Primary_Sleep(void);

/** Startup variant of Primary_Sleep(): at init the primary is physically
    ranging while the state machine still holds its initial SLEEP value, so
    plain Primary_Sleep() would be a no-op. Marks the state ACTIVE first,
    then runs the real sleep sequence. Call once at init instead of
    Primary_Sleep(). */
void VL53L5CX_Primary_SleepAtStartup(void);

/** Wake primary sensor from ST sleep mode.
    Enters VL53L5CX_POWER_MODE_WAKEUP, restarts ranging.
    Firmware and configuration retained — no re-init needed. */
void VL53L5CX_Primary_Wake(void);

/** Get current primary sensor state. */
PrimaryState_t VL53L5CX_Primary_GetState(void);

/** Check if primary sensor is currently active (not sleeping).
    @return 1 if active, 0 if sleeping */
int VL53L5CX_Primary_IsActive(void);

/* ================================================================
    External Sensor (Secondary) API (Dual Mode)
    ================================================================
    All functions below are only active when VL53L5CX_DUAL_SENSOR == 1.
    The external sensor is the guardian VL53L5CX at I2C address 0x62,
    continuously monitoring for motion/signal drop.
    ================================================================ */

/** One-time initialization after PowerUp().
    Runs vl53l5cx_is_alive() + vl53l5cx_init().
    @return 0 on success, -1 on failure */
int  VL53L5CX_External_Init(void);

/** Configure external sensor (resolution, integration time, frequency). */
void VL53L5CX_External_Configure(void);

/** Start continuous ranging on external sensor. */
void VL53L5CX_External_StartRanging(void);

/** Stop ranging on external sensor. */
void VL53L5CX_External_StopRanging(void);

/** Learn baseline for external sensor. */
void VL53L5CX_External_LearnBaseline(void);

/** Update external sensor data and run detection.
    If detection occurs, wakes the primary sensor.
    @return 1 if data ready, 0 if not */
int  VL53L5CX_External_Update(void);

/** Check if external sensor detected an insect.
    @return 1 if detected, 0 if not */
int  VL53L5CX_External_IsInsectDetected(void);

/** Get current external sensor state. */
ExternalState_t VL53L5CX_External_GetState(void);

/** Get external sensor baseline ready flag.
    @return 1 if baseline is ready, 0 if not */
int  VL53L5CX_External_IsBaselineReady(void);

/** Print ZFRAME for external sensor (EXT prefix). */
void VL53L5CX_External_PrintZFrame(void);

/** Print ALLPARAM for external sensor (EXT prefix). */
void VL53L5CX_External_PrintAllZoneParams(void);

/** Handle primary sensor wake timer.
    Called each loop iteration. When the wake duration expires,
    puts the primary sensor back to sleep. */
void VL53L5CX_Primary_CheckWakeTimeout(void);

#endif /* VL53L5CX_DETECTION_H */
