/**
 * @file    vlc5_tof.h
 * @brief   VLC5 (VL53L5CX) Time-of-Flight Sensor Module
 *
 *   Encapsulates all VL53L5CX sensor management into a clean, reusable module.
 *   Provides initialization, calibration, and insect detection as a FreeRTOS
 *   task similar to the button handling pattern.
 *
 *   Usage:
 *     1. Call VLC5_Init() from main thread after I2C + GPIO are ready
 *     2. Call VLC5_Calibrate() from main thread or before starting monitoring
 *     3. Create VLC5_DetectionTask() as a FreeRTOS task
 *
 *   Detection callback:
 *     Register a callback with VLC5_SetDetectionCallback() that will be called
 *     whenever an insect (signal drop) is detected. The callback triggers
 *     camera capture + SD storage.
 *
 *   Configuration parameters are defined below and can be tuned.
 ******************************************************************************
 */

#ifndef VLC5_TOF_H
#define VLC5_TOF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * CONFIGURATION — Tune these for your setup
 * ================================================================ */

/** Number of baseline samples to average per zone */
#define VLC5_BASELINE_SAMPLES      10

/** Signal drop percentage threshold for insect detection */
#define VLC5_INSECT_THRESHOLD_PCT  10

/** Number of zones in 4x4 resolution (16 total) */
#define VLC5_NUM_ZONES             16

/** Integration time in milliseconds (longer = more accurate, slower) */
#define VLC5_INTEGRATION_TIME_MS   800

/** Ranging frequency in Hz */
#define VLC5_RANGING_FREQ_HZ       15

/** Cooldown frames after a capture before next detection allowed */
#define VLC5_CAPTURE_COOLDOWN_FRAMES 60  /* ~4 seconds at 15 Hz */

/* ================================================================
 * CALIBRATION RESULT STRUCTURE
 * ================================================================ */

/** Calibration data returned after baseline learning */
typedef struct {
    uint32_t  signal[VLC5_NUM_ZONES];     /* Average signal per zone (kcps/spad) */
    uint16_t  distance[VLC5_NUM_ZONES];   /* Average distance per zone (mm) */
    uint8_t   valid_zones;           /* Number of valid zones */
    bool      ready;                 /* true if calibration succeeded */
} VLC5_Calibration_t;

/* ================================================================
 * DETECTION CALLBACK TYPE
 * ================================================================ */

/**
 * @brief Callback invoked when an insect is detected.
 * @param zone_count Number of affected zones
 * @param zones Array of affected zone indices
 * @param drops Array of signal drop percentages per zone
 */
typedef void (*VLC5_DetectionCallback_t)(uint8_t zone_count,
                                          const uint8_t *zones,
                                          const uint32_t *drops);

/* ================================================================
 * PUBLIC FUNCTIONS
 * ================================================================ */

/**
 * @brief Initialize the VL53L5CX sensor (I2C communication + sensor init).
 * @note  Must be called after I2C1 peripheral is initialized.
 * @return 0 on success, non-zero error code on failure.
 */
uint8_t VLC5_Init(void);

/**
 * @brief Check if the sensor is alive.
 * @param is_alive Output: 1 if alive, 0 if not.
 * @return 0 on success.
 */
uint8_t VLC5_IsAlive(uint8_t *is_alive);

/**
 * @brief Calibrate the sensor by learning baseline signal over multiple samples.
 *        This blocks for ~baseline_samples / ranging_freq_hz seconds.
 * @param cal Output calibration data (can be NULL if not needed).
 * @return 0 on success, non-zero if not enough valid zones.
 * @note  Call this from the main thread or before starting detection task.
 */
uint8_t VLC5_Calibrate(VLC5_Calibration_t *cal);

/**
 * @brief Get the current calibration data.
 * @return Pointer to calibration struct (read-only).
 */
const VLC5_Calibration_t *VLC5_GetCalibration(void);

/**
 * @brief Check if calibration is ready.
 * @return true if calibration completed successfully.
 */
bool VLC5_IsCalibrated(void);

/**
 * @brief Read the latest ranging data from the sensor.
 * @param results Output VL53L5CX_ResultsData struct.
 * @return 0 if data was read successfully.
 */
uint8_t VLC5_ReadData(VL53L5CX_ResultsData *results);

/**
 * @brief Wait until new data is ready (with FreeRTOS delay).
 * @return 0 when data is ready.
 */
uint8_t VLC5_WaitDataReady(void);

/**
 * @brief Set the callback invoked on insect detection.
 * @param cb Callback function pointer.
 */
void VLC5_SetDetectionCallback(VLC5_DetectionCallback_t cb);

/**
 * @brief FreeRTOS task for continuous insect detection.
 *        Monitors ToF zones and triggers callback on detection.
 * @param arg Unused argument.
 * @note  This task runs forever. Create it with xTaskCreateStatic().
 */
void VLC5_DetectionTask(void *arg);

/**
 * @brief Stop continuous ranging.
 */
void VLC5_StopRanging(void);

/**
 * @brief Start continuous ranging.
 * @return 0 on success.
 */
uint8_t VLC5_StartRanging(void);

#ifdef __cplusplus
}
#endif

#endif /* VLC5_TOF_H */