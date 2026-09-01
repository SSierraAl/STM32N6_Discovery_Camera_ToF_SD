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

/* Tunables (mode, addresses, resolution, thresholds, motion plugin,
   baseline refresh, debug output) live in app_config.h -
   "ToF Detection (VL53L5CX)" section. */
#include "app_config.h"

/* ================================================================
    Configuration Constants
    (all tunables in app_config.h; NUM_ZONES derived from RESOLUTION)
    ================================================================ */

#if VL53L5CX_DET_RESOLUTION == 8
#define VL53L5CX_DET_NUM_ZONES        64
#elif VL53L5CX_DET_RESOLUTION == 4
#define VL53L5CX_DET_NUM_ZONES        16
#else
#error "VL53L5CX_DET_RESOLUTION must be 4 or 8"
#endif

/* Per-target statuses accepted by the signal channel (baseline learning,
   detection, debug output) - only FRESH-data statuses:
     5 = VALID            : fresh, full-confidence measurement.
     6 = FIRST_RANGE      : fresh measurement right after ranging (re)start
                            (boot, wake, baseline restart). Wrap-around is
                            not checked yet, so ST rates it ~50% confidence -
                            same class as 9, but the data is current.
     9 = VALID_WITH_LARGE : fresh, valid, large pulse / merged target.
   0 = DATA_NOT_UPDATED is deliberately NOT accepted: on such frames the
   values being read are the last ones measured (carried data), and acting
   on them would just re-evaluate the previous frame's verdict (a stale
   drop would re-trigger).
   All other statuses (7 no-return, 8 no-target, saturation, error...) are
   likewise rejected: acting on them means reacting to missing data.
   NOTE: the primary sensor's motion channel is status-independent. */
#define VL53L5CX_STATUS_OK_FILT(st) \
    ((st) == 5 || (st) == 6 || (st) == 9)


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
