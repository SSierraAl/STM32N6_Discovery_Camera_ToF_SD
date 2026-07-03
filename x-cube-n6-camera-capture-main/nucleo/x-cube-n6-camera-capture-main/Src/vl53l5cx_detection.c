/**
 * *****************************************************************************
 * @file    vl53l5cx_detection.c
 * @brief   VL53L5CX ToF Sensor - Modular Detection API Implementation
 *
 *           All VL53L5CX functions are here. Include vl53l5cx_detection.h
 *           in your code and call the public API directly.
 *
 * *****************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include "vl53l5cx_detection.h"
#include "platform.h"
#include "vl53l5cx_plugin_motion_indicator.h"
#include "FreeRTOS.h"
#include "task.h"

/* ================================================================
   Internal State
   ================================================================ */

static I2C_HandleTypeDef *s_hi2c = NULL;
static VL53L5CX_Configuration  s_dev;
static VL53L5CX_ResultsData    s_results;

static uint32_t  s_baseline_signal[VL53L5CX_DET_NUM_ZONES] = {0};
static uint16_t  s_baseline_distance[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_zone_valid[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_baseline_ready = 0;

static VL53L5CX_DetectionResult_t s_last_result = {0};
static uint8_t s_last_insect_detected = 0;

/* ================================================================
   Internal Helpers
   ================================================================ */

/* Zone mask disabled — all zones always enabled */
static inline int is_zone_enabled(uint8_t z)
{
    (void)z;
    return 1;
}

/* ================================================================
   Initialization
   ================================================================ */

/**
 * @brief  Initialize VL53L5CX sensor
 * @param  hi2c  Pointer to I2C handle
 * @return 0 on success, -1 if sensor not detected, -2 if init failed
 */
int VL53L5CX_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t is_alive;
    int init_status;

    s_hi2c = hi2c;
    s_dev.platform.address = 0x29;

    /* Retry sensor detection (up to 10 attempts, 300ms each) */
    printf("[ToF] Waiting for sensor to respond...\n");
    for (uint8_t retry = 0; retry < 10; retry++) {
        init_status = vl53l5cx_is_alive(&s_dev, &is_alive);
        if (is_alive && init_status == 0) break;
        printf("[ToF] Not ready (retry %d/10)...\n", retry + 1);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    if (!is_alive || init_status != 0) {
        printf("[ToF] ERROR: Sensor not detected!\n");
        return -1;
    }

    /* Init sensor API (up to 3 retries) */
    for (uint8_t r = 0; r < 3; r++) {
        init_status = vl53l5cx_init(&s_dev);
        if (init_status == 0) break;
        printf("[ToF] Init retry %d/3...\n", r + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (init_status != 0) {
        printf("[ToF] ERROR: Init failed (status=%d)!\n", init_status);
        return -2;
    }

    printf("[ToF] Sensor initialized successfully!\n");
    return 0;
}

/**
 * @brief  Power up the VL53L5CX sensor (GPIO sequence)
 *
 *         Pin Mapping:
 *           - PD0  → PWR_EN  (Power Enable, active HIGH)
 *           - PE7  → I2C_RST (Reset, active LOW pulse)
 *           - PD6  → LPn     (Low Power disable, active HIGH)
 */
void VL53L5CX_PowerUp(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Initialize GPIO pins */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_0;  /* PWR_EN */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;  /* I2C_RST */
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;  /* LPn */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    printf("\n=== ToF Sensor Power Up ===\n");

    /* Step 1: Enable power */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(10);

    /* Step 2: Reset pulse (active LOW) */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(10);

    /* Step 3: Disable Low Power mode */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(100);

    printf("[OK] Sensor power-up complete\n\n");
}

/**
 * @brief  Power down the VL53L5CX sensor
 */
void VL53L5CX_PowerDown(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    printf("[ToF] Sensor powered down\n");
}

/* ================================================================
   Configuration
   ================================================================ */

void VL53L5CX_Configure(uint8_t resolution, int integration_ms, int freq_hz)
{
    vl53l5cx_set_resolution(&s_dev, resolution);
    vl53l5cx_set_integration_time_ms(&s_dev, integration_ms);
    vl53l5cx_set_ranging_frequency_hz(&s_dev, freq_hz);
    vl53l5cx_set_target_order(&s_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    vl53l5cx_set_sharpener_percent(&s_dev, 10);
    vl53l5cx_set_ranging_mode(&s_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    printf("[ToF] Configured: res=%d, int=%dms, freq=%dHz\n",
           resolution, integration_ms, freq_hz);
}

void VL53L5CX_StartRanging(void)
{
    vl53l5cx_start_ranging(&s_dev);
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("[ToF] Ranging started\n");
}

void VL53L5CX_StopRanging(void)
{
    vl53l5cx_stop_ranging(&s_dev);
    printf("[ToF] Ranging stopped\n");
}

/* ================================================================
   Data Access
   ================================================================ */

int VL53L5CX_WaitForDataReady(uint32_t timeout_ms)
{
    uint8_t is_ready = 0;
    TickType_t start = xTaskGetTickCount();

    while (!is_ready) {
        vl53l5cx_check_data_ready(&s_dev, &is_ready);
        if (is_ready) return 1;
        if (xTaskGetTickCount() - start > pdMS_TO_TICKS(timeout_ms)) return 0;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return 0;
}

int VL53L5CX_GetData(void)
{
    return vl53l5cx_get_ranging_data(&s_dev, &s_results);
}

void VL53L5CX_GetZoneData(uint8_t zone, uint32_t *signal, uint16_t *distance, uint8_t *status)
{
    uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * zone;
    if (signal)   *signal   = s_results.signal_per_spad[idx];
    if (distance) *distance  = s_results.distance_mm[idx];
    if (status)   *status    = s_results.target_status[idx];
}

void VL53L5CX_GetBaselineData(uint8_t zone, uint32_t *signal, uint16_t *distance)
{
    if (zone < VL53L5CX_DET_NUM_ZONES) {
        if (signal)   *signal   = s_baseline_signal[zone];
        if (distance) *distance  = s_baseline_distance[zone];
    }
}

int VL53L5CX_IsZoneValid(uint8_t zone)
{
    if (zone < VL53L5CX_DET_NUM_ZONES) return s_zone_valid[zone];
    return 0;
}

int VL53L5CX_IsBaselineReady(void)
{
    return s_baseline_ready;
}

/* ================================================================
   Baseline Management
   ================================================================ */

void VL53L5CX_ResetBaseline(void)
{
    memset(s_baseline_signal, 0, sizeof(s_baseline_signal));
    memset(s_baseline_distance, 0, sizeof(s_baseline_distance));
    memset(s_zone_valid, 0, sizeof(s_zone_valid));
    s_baseline_ready = 0;
    printf("[ToF] Baseline reset\n");
}

void VL53L5CX_LearnBaseline(void)
{
    uint8_t is_ready;

    VL53L5CX_ResetBaseline();

    printf("[BASELINE] Learning %d samples...\n", VL53L5CX_DET_BASELINE_SAMPLES);

    for (uint8_t i = 0; i < VL53L5CX_DET_BASELINE_SAMPLES; i++) {
        if (!VL53L5CX_WaitForDataReady(1000)) continue;
        if (VL53L5CX_GetData() != 0) continue;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            if (!is_zone_enabled(z)) continue;
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results.target_status[idx] == 0 ||
                s_results.target_status[idx] == 5 ||
                s_results.target_status[idx] == 9) {
                s_baseline_signal[z] += s_results.signal_per_spad[idx];
                s_baseline_distance[z] += s_results.distance_mm[idx];
                s_zone_valid[z] = 1;
            }
        }
        printf("  [%d/%d]\r", i + 1, VL53L5CX_DET_BASELINE_SAMPLES);
    }

    /* Compute averages */
    uint8_t valid_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (is_zone_enabled(z) && s_zone_valid[z]) {
            s_baseline_signal[z] /= VL53L5CX_DET_BASELINE_SAMPLES;
            s_baseline_distance[z] /= VL53L5CX_DET_BASELINE_SAMPLES;
            valid_count++;
        }
    }

    s_baseline_ready = 1;
    printf("\n[BASELINE] Done. Valid zones: %d/%d\n", valid_count, VL53L5CX_DET_NUM_ZONES);
}

/* ================================================================
   Detection
   ================================================================ */

/**
 * @brief  Update: wait for data, process, run detection
 * @return 1 if new data was processed, 0 if timeout
 */
int VL53L5CX_Update(void)
{
    if (!VL53L5CX_WaitForDataReady(1000)) return 0;
    if (VL53L5CX_GetData() != 0) return 0;

    /* Reset detection result */
    s_last_insect_detected = 0;
    s_last_result.insect_detected = 0;
    s_last_result.affected_count = 0;
    s_last_result.valid_measurements = 0;

    /* Check each enabled zone */
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (!is_zone_enabled(z)) continue;
        if (!s_zone_valid[z]) continue;

        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

        if (s_results.target_status[idx] != 0 &&
            s_results.target_status[idx] != 5 &&
            s_results.target_status[idx] != 9)
            continue;

        s_last_result.valid_measurements++;

        /* Compute signal drop */
        uint32_t signal_drop = 0;
        if (s_baseline_signal[z] > 0) {
            int32_t diff = (int32_t)s_baseline_signal[z] - (int32_t)s_results.signal_per_spad[idx];
            if (diff < 0) diff = -diff;
            signal_drop = (uint32_t)diff * 100 / s_baseline_signal[z];
        }

        if (signal_drop > VL53L5CX_DET_THRESHOLD_PCT) {
            uint8_t k = s_last_result.affected_count;
            s_last_result.affected_zones[k] = (uint8_t)z;
            s_last_result.affected_drop[k] = signal_drop;
            s_last_result.affected_count++;
            s_last_insect_detected = 1;
            s_last_result.insect_detected = 1;
        }
    }

    /* Adaptive baseline update (only for non-affected zones) */
#if VL53L5CX_DET_ADAPTIVE_ENABLED
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (!is_zone_enabled(z)) continue;
        if (!s_zone_valid[z]) continue;

        uint8_t is_affected = 0;
        for (uint8_t k = 0; k < s_last_result.affected_count; k++) {
            if (s_last_result.affected_zones[k] == (uint8_t)z) {
                is_affected = 1;
                break;
            }
        }
        if (is_affected) continue;

        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        if (s_results.target_status[idx] != 0 &&
            s_results.target_status[idx] != 5 &&
            s_results.target_status[idx] != 9) continue;

        if (s_baseline_signal[z] > 0) {
            int32_t diff = (int32_t)s_results.signal_per_spad[idx] - (int32_t)s_baseline_signal[z];
            s_baseline_signal[z] += diff / VL53L5CX_DET_EMA_DIVIDER;
        }
        if (s_baseline_distance[z] > 0) {
            int32_t diff = (int32_t)s_results.distance_mm[idx] - (int32_t)s_baseline_distance[z];
            s_baseline_distance[z] += diff / VL53L5CX_DET_EMA_DIVIDER;
        }
    }
#endif

    /* Debug output */
#if VL53L5CX_DET_DEBUG_FRAME_INTERVAL > 0
    static uint32_t debug_counter = 0;
    debug_counter++;
    if (debug_counter >= VL53L5CX_DET_DEBUG_FRAME_INTERVAL) {
        debug_counter = 0;
        VL53L5CX_PrintZFrame();
    }
#endif

#if VL53L5CX_DET_DEBUG_ALLPARAMS > 0
    static uint32_t allparam_counter = 0;
    allparam_counter++;
    if (allparam_counter >= VL53L5CX_DET_DEBUG_ALLPARAM_INT) {
        allparam_counter = 0;
        VL53L5CX_PrintAllZoneParams();
    }
#endif

    return 1;
}

int VL53L5CX_IsInsectDetected(void)
{
    return s_last_insect_detected;
}

VL53L5CX_DetectionResult_t VL53L5CX_GetResult(void)
{
    return s_last_result;
}

/* ================================================================
   Debug / Diagnostics
   ================================================================ */

/**
 * @brief  PrintAllZoneParams - prints a detailed table of ALL ToF parameters
 *
 * Format (ALLPARAM header):
 *   ALLPARAM,temp,sig,base_sig,dist,base_dist,ambient,sigma,reflect,status,spads,targets,drop%
 *   Then one line per zone (16 zones):
 *   Z0: sig,base_sig,dist,base_dist,ambient,sigma,reflect,status,spads,targets,drop%
 *   Z1: ...
 *   ...
 *   Z15: ...
 *   Motion line:
 *   MOTION:global1,global2,status,nb_detected,nb_agg,motion0,motion1,...,motion15
 */
void VL53L5CX_PrintAllZoneParams(void)
{
    int8_t temp = s_results.silicon_temp_degc;

    printf("\n=== VL53L5CX All Zone Parameters (temp=%d°C) ===\n", temp);
    printf("%3s | %6s | %7s | %8s | %7s | %8s | %6s | %7s | %6s | %7s | %6s | %6s\n",
           "Z", "status", "dist", "signal", "b_dist", "b_signal", "ambient",
           "sigma", "reflect", "spads", "targs", "drop%");
    printf("-----|--------|----------|----------|---------|------------|----------|--------|---------|----------|--------|------\n");

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

        uint32_t cur_sig    = s_results.signal_per_spad[idx];
        int16_t  cur_dist   = s_results.distance_mm[idx];
        uint8_t  cur_stat   = s_results.target_status[idx];
        uint32_t cur_ambient = 0;
        uint16_t cur_sigma  = 0;
        uint8_t  cur_reflect = 0;
        uint32_t cur_spads  = 0;
        uint8_t  cur_targets = 0;

#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
        cur_ambient = s_results.ambient_per_spad[idx];
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
        cur_sigma = s_results.range_sigma_mm[idx];
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
        cur_reflect = s_results.reflectance[idx];
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
        cur_spads = s_results.nb_spads_enabled[idx];
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
        cur_targets = s_results.nb_target_detected[idx];
#endif

        uint32_t base_sig = s_baseline_signal[z];
        uint16_t base_dist = s_baseline_distance[z];

        uint32_t drop_pct = 0;
        if (base_sig > 0) {
            int32_t diff = (int32_t)base_sig - (int32_t)cur_sig;
            if (diff < 0) diff = -diff;
            drop_pct = (uint32_t)diff * 100 / base_sig;
        }

        printf("Z%2d | %6d | %7dmm | %8d | %7dmm | %9d | %8d | %6d | %7d | %7d | %6d | %5d%%\n",
               z, cur_stat, cur_dist, cur_sig, base_dist, base_sig,
               cur_ambient, cur_sigma, cur_reflect, cur_spads, cur_targets, drop_pct);
    }

    /* Print motion indicator data */
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    printf("\n--- Motion Indicator ---\n");
    printf("Global1: %lu, Global2: %lu, Status: %d, Detected: %d, Aggregates: %d\n",
           (unsigned long)s_results.motion_indicator.global_indicator_1,
           (unsigned long)s_results.motion_indicator.global_indicator_2,
           s_results.motion_indicator.status,
           s_results.motion_indicator.nb_of_detected_aggregates,
           s_results.motion_indicator.nb_of_aggregates);
    printf("Motion[0:%d]: ", VL53L5CX_DET_NUM_ZONES - 1);
    for (int m = 0; m < VL53L5CX_DET_NUM_ZONES; m++) {
        printf("%lu ", (unsigned long)s_results.motion_indicator.motion[m]);
    }
    printf("\n");
#endif

    /* Also emit machine-parseable ALLPARAM block for Python */
    printf("ALLPARAM,");
    printf("%d,", temp);
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint32_t cur_sig    = s_results.signal_per_spad[idx];
        int16_t  cur_dist   = s_results.distance_mm[idx];
        uint8_t  cur_stat   = s_results.target_status[idx];
        uint32_t cur_ambient = 0;
        uint16_t cur_sigma  = 0;
        uint8_t  cur_reflect = 0;
        uint32_t cur_spads  = 0;
        uint8_t  cur_targets = 0;

#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
        cur_ambient = s_results.ambient_per_spad[idx];
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
        cur_sigma = s_results.range_sigma_mm[idx];
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
        cur_reflect = s_results.reflectance[idx];
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
        cur_spads = s_results.nb_spads_enabled[idx];
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
        cur_targets = s_results.nb_target_detected[idx];
#endif

        uint32_t base_sig = s_baseline_signal[z];
        uint16_t base_dist = s_baseline_distance[z];
        uint32_t drop_pct = 0;
        if (base_sig > 0) {
            int32_t diff = (int32_t)base_sig - (int32_t)cur_sig;
            if (diff < 0) diff = -diff;
            drop_pct = (uint32_t)diff * 100 / base_sig;
        }

        if (z > 0) printf(",");
        printf("%lu,%lu,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
               (unsigned long)cur_sig,
               (unsigned long)base_sig,
               cur_dist,
               base_dist,
               (unsigned long)cur_ambient,
               (unsigned long)cur_sigma,
               (unsigned long)cur_reflect,
               (unsigned long)cur_stat,
               (unsigned long)cur_spads,
               (unsigned long)cur_targets,
               (unsigned long)drop_pct,
               (unsigned long)(s_zone_valid[z] ? 1 : 0));
    }
    printf("\r\n");

    /* Emit MOTION line */
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    printf("MOTION,%lu,%lu,%d,%d,%d,",
           (unsigned long)s_results.motion_indicator.global_indicator_1,
           (unsigned long)s_results.motion_indicator.global_indicator_2,
           s_results.motion_indicator.status,
           s_results.motion_indicator.nb_of_detected_aggregates,
           s_results.motion_indicator.nb_of_aggregates);
    for (int m = 0; m < VL53L5CX_DET_NUM_ZONES; m++) {
        if (m > 0) printf(",");
        printf("%lu", (unsigned long)s_results.motion_indicator.motion[m]);
    }
    printf("\r\n");
#else
    printf("MOTION,0,0,0,0,0");
    for (int m = 0; m < VL53L5CX_DET_NUM_ZONES; m++) {
        printf(",0");
    }
    printf("\r\n");
#endif
}

/**
 * @brief  PrintZFrame - emits ZFRAME line for Python monitoring
 *
 * EXTENDED FORMAT (when VL53L5CX_DET_DEBUG_EXTENDED_ZFRAME == 1):
 *   ZFRAME,temp,sig0,base0,dist0,bdist0,amb0,sigmm0,refl0,status0,spads0,targs0,drop0,valid0,...,sig15,...,valid15,motion0,...,motion15
 *
 * LEGACY FORMAT (when VL53L5CX_DET_DEBUG_EXTENDED_ZFRAME == 0):
 *   ZFRAME,sig0,base0,dist0,bdist0,...,sig15,base15,dist15,bdist15
 */
void VL53L5CX_PrintZFrame(void)
{
#if VL53L5CX_DET_DEBUG_EXTENDED_ZFRAME
    /* Extended ZFRAME with ALL parameters */
    int8_t temp = s_results.silicon_temp_degc;
    printf("ZFRAME,%d,", temp);

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

        uint32_t cur_sig    = 0;
        int16_t  cur_dist   = 0;
        uint8_t  cur_stat   = 0;
        uint32_t cur_ambient = 0;
        uint16_t cur_sigma  = 0;
        uint8_t  cur_reflect = 0;
        uint32_t cur_spads  = 0;
        uint8_t  cur_targets = 0;
        uint8_t  cur_valid  = 0;

        if (s_zone_valid[z] && is_zone_enabled(z) &&
            (s_results.target_status[idx] == 0 ||
             s_results.target_status[idx] == 5 ||
             s_results.target_status[idx] == 9)) {
            cur_sig    = s_results.signal_per_spad[idx];
            cur_dist   = s_results.distance_mm[idx];
            cur_stat   = s_results.target_status[idx];
            cur_valid  = 1;

#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
            cur_ambient = s_results.ambient_per_spad[idx];
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
            cur_sigma = s_results.range_sigma_mm[idx];
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
            cur_reflect = s_results.reflectance[idx];
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
            cur_spads = s_results.nb_spads_enabled[idx];
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
            cur_targets = s_results.nb_target_detected[idx];
#endif
        }

        uint32_t base_sig = s_baseline_signal[z];
        uint16_t base_dist = s_baseline_distance[z];
        uint32_t drop_pct = 0;
        if (base_sig > 0) {
            int32_t diff = (int32_t)base_sig - (int32_t)cur_sig;
            if (diff < 0) diff = -diff;
            drop_pct = (uint32_t)diff * 100 / base_sig;
        }

        if (z > 0) printf(",");
        printf("%lu,%lu,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
               (unsigned long)cur_sig,
               (unsigned long)base_sig,
               cur_dist,
               base_dist,
               (unsigned long)cur_ambient,
               (unsigned long)cur_sigma,
               (unsigned long)cur_reflect,
               (unsigned long)cur_stat,
               (unsigned long)cur_spads,
               (unsigned long)cur_targets,
               (unsigned long)drop_pct,
               (unsigned long)cur_valid);
    }

    /* Append motion indicator data */
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    printf(",");
    for (int m = 0; m < VL53L5CX_DET_NUM_ZONES; m++) {
        if (m > 0) printf(",");
        printf("%lu", (unsigned long)s_results.motion_indicator.motion[m]);
    }
#else
    for (int m = 0; m < VL53L5CX_DET_NUM_ZONES; m++) {
        printf(",0");
    }
#endif

    printf("\r\n");

#else
    /* Legacy ZFRAME format (backward compatible) */
    printf("ZFRAME,");
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t cur_sig = 0, cur_dist = 0;
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        if (s_zone_valid[z] && is_zone_enabled(z) &&
            (s_results.target_status[idx] == 0 ||
             s_results.target_status[idx] == 5 ||
             s_results.target_status[idx] == 9)) {
            cur_sig  = s_results.signal_per_spad[idx];
            cur_dist = s_results.distance_mm[idx];
        }
        if (z > 0) printf(",");
        printf("%lu,%lu,%lu,%lu",
                (unsigned long)cur_sig,
                (unsigned long)s_baseline_signal[z],
                (unsigned long)cur_dist,
                (unsigned long)s_baseline_distance[z]);
    }
    printf("\r\n");
#endif
}

int VL53L5CX_ScanI2CBus(void)
{
    if (!s_hi2c) return 0;
    uint8_t found = 0;
    for (uint8_t addr = 0; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(s_hi2c, addr << 1, 1, 10) == HAL_OK) {
            printf("  I2C device at 0x%02X\n", addr);
            found++;
        }
    }
    return found;
}

/* ================================================================
   Legacy Test Functions (kept for backward compatibility)
   ================================================================ */

void VL53L5CX_Validate(void)
{
    uint8_t is_alive;
    int st;

    s_dev.platform.address = 0x29;
    st = vl53l5cx_is_alive(&s_dev, &is_alive);
    printf("ALIVE %i and status %i\r\n", is_alive, st);

    if (!is_alive || st) {
        printf("Sensor NOT detected\r\n");
    } else {
        printf("Sensor detected\r\n");
        st = vl53l5cx_init(&s_dev);
        if (st) {
            printf("Init FAIL\r\n");
        } else {
            printf("Init OK\r\n");
            vl53l5cx_start_ranging(&s_dev);
        }
    }
    HAL_Delay(1000);

    if (s_hi2c) {
        for (uint8_t addr = 0; addr < 128; addr++) {
            if (HAL_I2C_IsDeviceReady(s_hi2c, addr << 1, 1, 10) == HAL_OK) {
                printf("I2C device found at 0x%02X\r\n", addr);
            }
        }
    }
}

void VL53L5CX_ReadingTest(void)
{
    uint8_t is_ready;
    uint8_t i;

    vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_4X4);
    VL53L5CX_ResetBaseline();
    vl53l5cx_set_integration_time_ms(&s_dev, 800);
    vl53l5cx_set_ranging_frequency_hz(&s_dev, 15);
    vl53l5cx_set_target_order(&s_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    vl53l5cx_set_sharpener_percent(&s_dev, 10);
    vl53l5cx_set_ranging_mode(&s_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

    vl53l5cx_start_ranging(&s_dev);
    VL53L5CX_WaitMs(&s_dev.platform, 200);
    printf("Ranging started...\r\n");

    printf("\r\n[BASELINE] Taking %d samples per zone...\r\n", VL53L5CX_DET_BASELINE_SAMPLES);

    for (i = 0; i < VL53L5CX_DET_BASELINE_SAMPLES; i++) {
        do {
            vl53l5cx_check_data_ready(&s_dev, &is_ready);
            if (!is_ready) vTaskDelay(pdMS_TO_TICKS(10));
        } while (!is_ready);

        if (vl53l5cx_get_ranging_data(&s_dev, &s_results) != 0)
            continue;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results.target_status[idx] == 0 ||
                s_results.target_status[idx] == 5 ||
                s_results.target_status[idx] == 9) {
                s_baseline_signal[z] += s_results.signal_per_spad[idx];
                s_baseline_distance[z] += s_results.distance_mm[idx];
                s_zone_valid[z] = 1;
            }
        }
        printf("  [%d/%d]\r", i + 1, VL53L5CX_DET_BASELINE_SAMPLES);
    }

    printf("\r\n[BASELINE] Zone | Distance(mm) | Signal(kcps/spad)\r\n");
    printf("----------|----------------|----------------------\r\n");

    uint8_t valid_zone_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (s_zone_valid[z]) {
            s_baseline_signal[z] /= VL53L5CX_DET_BASELINE_SAMPLES;
            s_baseline_distance[z] /= VL53L5CX_DET_BASELINE_SAMPLES;
            valid_zone_count++;
            printf("  Z%2d    |    %8d     |    %10d\r\n",
                   z, s_baseline_distance[z], s_baseline_signal[z]);
        }
    }

    printf("\r\n[BASELINE] Valid zones: %d/%d\r\n",
           valid_zone_count, VL53L5CX_DET_NUM_ZONES);
    printf("[BASELINE] Insect threshold: >%d%% drop in any zone\r\n",
           VL53L5CX_DET_THRESHOLD_PCT);
    s_baseline_ready = 1;

    printf("\r\n[MONITORING] Watching %d zones for insect passage...\r\n",
           valid_zone_count);
    printf("  (Wave your hand or blow air in front of the sensor)\r\n\n");

    while (1) {
        do {
            vl53l5cx_check_data_ready(&s_dev, &is_ready);
            if (!is_ready) vTaskDelay(pdMS_TO_TICKS(10));
        } while (!is_ready);

        if (vl53l5cx_get_ranging_data(&s_dev, &s_results) != 0)
            continue;

        uint8_t insect_found = 0;
        uint8_t affected_zones[VL53L5CX_DET_NUM_ZONES] = {0};
        uint32_t affected_drop[VL53L5CX_DET_NUM_ZONES] = {0};
        uint8_t affected_count = 0;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            if (!s_zone_valid[z]) continue;
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results.target_status[idx] != 0 &&
                s_results.target_status[idx] != 5 &&
                s_results.target_status[idx] != 9) continue;

            uint32_t signal_drop = 0;
            if (s_baseline_signal[z] > 0) {
                int32_t diff = (int32_t)s_baseline_signal[z] -
                               (int32_t)s_results.signal_per_spad[idx];
                if (diff < 0) diff = -diff;
                signal_drop = (uint32_t)diff * 100 / s_baseline_signal[z];
            }

            if (signal_drop > VL53L5CX_DET_THRESHOLD_PCT) {
                affected_zones[affected_count] = (uint8_t)z;
                affected_drop[affected_count] = signal_drop;
                affected_count++;
                insect_found = 1;
            }
        }

        if (insect_found) {
            char zone_list[64] = {0};
            for (uint8_t k = 0; k < affected_count; k++) {
                char tmp[16];
                sprintf(tmp, "Z%u", affected_zones[k]);
                if (k == 0) strcpy(zone_list, tmp);
                else { strcat(zone_list, ","); strcat(zone_list, tmp); }
            }
            printf(">>> INSECT DETECTED! Zones: [%s] (%u zone(s))\r\n",
                   zone_list, affected_count);
            for (uint8_t k = 0; k < affected_count; k++) {
                uint8_t z = affected_zones[k];
                uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
                printf("    Z%2d: dist=%4dmm  signal=%6d  baseline=%6d  (drop=%u%%)\r\n",
                       z, s_results.distance_mm[idx], s_results.signal_per_spad[idx],
                       s_baseline_signal[z], affected_drop[k]);
            }
            printf("\r\n");
        } else {
            uint16_t min_d = 9999, max_d = 0;
            uint32_t min_s = 999999, max_s = 0;
            uint8_t frame_valid = 0;
            for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
                if (!s_zone_valid[z]) continue;
                uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
                if (s_results.target_status[idx] != 0 &&
                    s_results.target_status[idx] != 5 &&
                    s_results.target_status[idx] != 9) continue;
                frame_valid++;
                if (s_results.distance_mm[idx] < min_d) min_d = s_results.distance_mm[idx];
                if (s_results.distance_mm[idx] > max_d) max_d = s_results.distance_mm[idx];
                if (s_results.signal_per_spad[idx] < min_s) min_s = s_results.signal_per_spad[idx];
                if (s_results.signal_per_spad[idx] > max_s) max_s = s_results.signal_per_spad[idx];
            }
            if (frame_valid > 0) {
                printf(". zones=%2d  dist=%4d-%4dmm  signal=%5d-%5d\r\n",
                       frame_valid, min_d, max_d, min_s, max_s);
            }
        }
    }
}

void VL53L5CX_MotionTest(void)
{
    VL53L5CX_Motion_Configuration motion_config;
    uint8_t ready;

    printf("\r\n=== VL53L5CX Insect + Motion Detection (4x4 zones) ===\r\n");

    vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_4X4);
    vl53l5cx_set_integration_time_ms(&s_dev, 800);
    vl53l5cx_set_ranging_frequency_hz(&s_dev, 15);
    vl53l5cx_set_target_order(&s_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    vl53l5cx_set_sharpener_percent(&s_dev, 10);
    vl53l5cx_set_ranging_mode(&s_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

    int st = vl53l5cx_motion_indicator_init(&s_dev, &motion_config, VL53L5CX_RESOLUTION_4X4);
    if (st) printf("[WARN] Motion indicator init failed: %d\r\n", st);

    vl53l5cx_motion_indicator_set_distance_motion(&s_dev, &motion_config, 1000, 2000);
    vl53l5cx_start_ranging(&s_dev);
    VL53L5CX_WaitMs(&s_dev.platform, 200);

    printf("\r\n[BASELINE] Learning %d frames...\r\n", VL53L5CX_DET_BASELINE_SAMPLES);

    uint32_t baseline_signal_zone[VL53L5CX_DET_NUM_ZONES] = {0};
    uint16_t baseline_distance_zone[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t  valid_baseline_zones[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t  baseline_samples_zone[VL53L5CX_DET_NUM_ZONES] = {0};

    for (uint8_t sample = 0; sample < VL53L5CX_DET_BASELINE_SAMPLES; sample++) {
        uint32_t timeout = 0;
        while (!ready && timeout < 100) {
            vl53l5cx_check_data_ready(&s_dev, &ready);
            if (!ready) { VL53L5CX_WaitMs(&s_dev.platform, 10); timeout++; }
        }
        if (ready) {
            vl53l5cx_get_ranging_data(&s_dev, &s_results);
            for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
                if (!is_zone_enabled(z)) continue;
                uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
                uint8_t s = s_results.target_status[idx];
                if (s == 0 || s == 5 || s == 9) {
                    uint16_t dist = s_results.distance_mm[idx];
                    if (dist > 20 && dist < 4000) {
                        valid_baseline_zones[z] = 1;
                        baseline_signal_zone[z] += s_results.signal_per_spad[idx];
                        baseline_distance_zone[z] = dist;
                        baseline_samples_zone[z]++;
                    }
                }
            }
        }
        ready = 0;
        VL53L5CX_WaitMs(&s_dev.platform, 70);
    }

    uint8_t valid_zone_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (baseline_samples_zone[z] > 0) {
            baseline_signal_zone[z] /= baseline_samples_zone[z];
            valid_zone_count++;
        } else {
            valid_baseline_zones[z] = 0;
        }
    }

    printf("\r\n[BASELINE] Valid Zones: %d/%d\r\n", valid_zone_count, VL53L5CX_DET_NUM_ZONES);
    printf("[BASELINE] Insect threshold: >%d%% signal drop\r\n", VL53L5CX_DET_THRESHOLD_PCT);
    printf("[BASELINE] Motion threshold: >%d motion power\r\n", VL53L5CX_DET_MOTION_THRESH);
    printf("\r\n[MONITORING] Watching %d zones...\r\n\r\n", valid_zone_count);

    uint32_t frame_count = 0, insect_count = 0, motion_count = 0;

    for (;;) {
        uint32_t timeout = 0;
        while (!ready && timeout < 100) {
            vl53l5cx_check_data_ready(&s_dev, &ready);
            if (!ready) { VL53L5CX_WaitMs(&s_dev.platform, 10); timeout++; }
        }
        if (!ready) { VL53L5CX_WaitMs(&s_dev.platform, 20); continue; }

        vl53l5cx_get_ranging_data(&s_dev, &s_results);
        frame_count++;

        uint8_t insect_found = 0, motion_found = 0;
        uint8_t insect_zone_count = 0, motion_zone_count = 0;
        uint8_t valid_measurements = 0;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            if (!is_zone_enabled(z)) continue;
            if (!valid_baseline_zones[z]) continue;

            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results.target_status[idx] != 0 &&
                s_results.target_status[idx] != 5 &&
                s_results.target_status[idx] != 9) continue;

            uint16_t dist = s_results.distance_mm[idx];
            if (dist < 20 || dist > 4000) continue;
            valid_measurements++;

            if (baseline_signal_zone[z] > 0) {
                int32_t diff = (int32_t)baseline_signal_zone[z] -
                               (int32_t)s_results.signal_per_spad[idx];
                if (diff > 0) {
                    uint32_t drop = (uint32_t)diff * 100 / baseline_signal_zone[z];
                    if (drop > VL53L5CX_DET_THRESHOLD_PCT) {
                        insect_zone_count++;
                        insect_found = 1;
                    }
                }
            }

            if (s_results.motion_indicator.motion[motion_config.map_id[z]] >= VL53L5CX_DET_MOTION_THRESH) {
                motion_zone_count++;
                motion_found = 1;
            }
        }

        if (insect_found && valid_measurements > 5) {
            printf(">>> INSECT DETECTED! Frame %d (%d zones)\r\n",
                   frame_count, insect_zone_count);
            insect_count++;
        }
        if (motion_found) {
            printf(">>> MOTION DETECTED! Frame %d (%d zones)\r\n",
                   frame_count, motion_zone_count);
            motion_count++;
        }
        if (!insect_found && !motion_found && valid_measurements > 0 && frame_count % 50 == 0) {
            printf(". frame=%d zones=%d (insect:%d motion:%d)\r\n",
                   frame_count, valid_measurements, insect_count, motion_count);
        }

        VL53L5CX_WaitMs(&s_dev.platform, 20);
    }
}
