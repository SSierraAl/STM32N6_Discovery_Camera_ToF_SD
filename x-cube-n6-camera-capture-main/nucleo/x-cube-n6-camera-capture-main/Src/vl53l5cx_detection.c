/**
 * *****************************************************************************
 * @file    vl53l5cx_detection.c
 * @brief   VL53L5CX ToF Sensor - Clean Detection API Implementation
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
static VL53L5CX_Motion_Configuration s_motion_config;
static uint8_t s_motion_initialized = 0;

static uint32_t  s_baseline_signal[VL53L5CX_DET_NUM_ZONES] = {0};
static uint16_t  s_baseline_distance[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_zone_valid[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_baseline_ready = 0;

static VL53L5CX_DetectionResult_t s_last_result = {0};
static uint8_t s_last_insect_detected = 0;

/* ================================================================
    Dual Sensor Mode State (External = Guardian, Primary = Camera ToF)
    ================================================================
    When VL53L5CX_DUAL_SENSOR == 1:
      - External sensor (at 0x62): always ON, continuously monitoring
      - Primary sensor (at 0x29): sleep mode by default, wakes on detection
    ================================================================ */
#if VL53L5CX_DUAL_SENSOR
// External (guardian) sensor state
static VL53L5CX_Configuration  s_dev_ext;
static VL53L5CX_ResultsData    s_results_ext;
static VL53L5CX_Motion_Configuration s_motion_config_ext;
static uint8_t s_motion_initialized_ext = 0;

static uint32_t  s_baseline_signal_ext[VL53L5CX_DET_NUM_ZONES] = {0};
static uint16_t  s_baseline_distance_ext[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_zone_valid_ext[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_baseline_ready_ext = 0;

static VL53L5CX_DetectionResult_t s_last_result_ext = {0};
static uint8_t s_last_insect_detected_ext = 0;
static ExternalState_t s_external_state = EXTERNAL_STATE_IDLE;

// Primary sensor state management
static PrimaryState_t s_primary_state = PRIMARY_STATE_SLEEP;
static uint32_t s_primary_wake_time = 0;
static uint8_t s_primary_detection_confirm = 0;
#endif

/* ================================================================
   Initialization
   ================================================================ */

int VL53L5CX_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t is_alive;
    int init_status;

    s_hi2c = hi2c;
    s_dev.platform.address = 0x29;
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

void VL53L5CX_PowerUp(void)
{
    int i2cdevices = 0;
    i2cdevices = VL53L5CX_ScanI2CBus();
#if VL53L5CX_DUAL_SENSOR
    if(i2cdevices == 3) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        __HAL_RCC_GPIOQ_CLK_ENABLE();

        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

        GPIO_InitStruct.Pin = GPIO_PIN_0;  /* PWR_EN */
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_7;  /* I2C_RST */
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_6;  /* LPn external */
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_5;  /* LPn camera */
        HAL_GPIO_Init(GPIOQ, &GPIO_InitStruct);

        printf("\n=== ToF Dual Sensor Power Up ===\n");

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(10);

        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_Delay(10);

        /* Power external ToF first (camera still off) */
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(10);

        /* Change external ToF address from 0x29 to 0x62 so camera ToF can use 0x29 */
        s_dev_ext.platform.address = 0x29;
        uint8_t addr_st = vl53l5cx_set_i2c_address(&s_dev_ext, 0x62);
        if (addr_st != 0) {
            printf("[WARN] Address change failed (status=%d)\n", addr_st);
        } else {
            printf("[OK] External ToF address changed to 0x62\n");
        }

        /* Now power camera ToF (primary) */
        HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(100);

        printf("[OK] Dual sensor power-up complete\n\n");
    }
#else
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(10);
    printf("[OK] Sensor power-up complete\n\n");
#endif
}

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
    vl53l5cx_set_target_order(&s_dev, VL53L5CX_TARGET_ORDER_STRONGEST);
    vl53l5cx_set_sharpener_percent(&s_dev, 10);
    vl53l5cx_set_ranging_mode(&s_dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    int motion_st = vl53l5cx_motion_indicator_init(&s_dev, &s_motion_config, resolution);
    if (motion_st) {
        printf("[ToF] WARN: Motion indicator init failed: %d\n", motion_st);
        s_motion_initialized = 0;
    } else {
        s_motion_config.min_nb_for_global_detection = VL53L5CX_DET_MOTION_MIN_ZONES;
        s_motion_config.nb_of_temporal_accumulations = VL53L5CX_DET_MOTION_PERSIST_FRAMES;
        s_motion_config.extra_noise_sigma = VL53L5CX_DET_MOTION_EXTRA_NOISE;
        s_motion_initialized = 1;
        printf("[ToF] Motion indicator enabled\n");
    }
#else
    s_motion_initialized = 0;
#endif

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
    VL53L5CX_ResetBaseline();
    const uint8_t baseline_samples = VL53L5CX_DET_BASELINE_SAMPLES;
    const uint8_t settle_frames = 5;

    printf("[BASELINE] Learning %d samples + %d settle frames...\n",
           baseline_samples, settle_frames);

    for (uint8_t i = 0; i < baseline_samples; i++) {
        if (!VL53L5CX_WaitForDataReady(1000)) continue;
        if (VL53L5CX_GetData() != 0) continue;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results.target_status[idx] == 0 ||
                s_results.target_status[idx] == 5 ||
                s_results.target_status[idx] == 9) {
                if (s_results.signal_per_spad[idx] < VL53L5CX_DET_MIN_SIGNAL) {
                    continue;
                }
                s_baseline_signal[z] += s_results.signal_per_spad[idx];
                s_baseline_distance[z] += s_results.distance_mm[idx];
                s_zone_valid[z] = 1;
            }
        }
        printf("  [BASELINE %d/%d]\r", i + 1, baseline_samples);
    }

    uint8_t valid_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (s_zone_valid[z]) {
            s_baseline_signal[z] /= baseline_samples;
            s_baseline_distance[z] /= baseline_samples;
            valid_count++;
        }
    }

    for (uint8_t i = 0; i < settle_frames; i++) {
        if (!VL53L5CX_WaitForDataReady(1000)) continue;
        if (VL53L5CX_GetData() != 0) continue;
        printf("  [SETTLE %d/%d]\r", i + 1, settle_frames);
    }

    s_baseline_ready = 1;
    printf("\n[BASELINE] Done. Valid zones: %d/%d\n", valid_count, VL53L5CX_DET_NUM_ZONES);
    VL53L5CX_PrintBaselineFrame();
}

/* ================================================================
   Detection
   ================================================================ */

int VL53L5CX_Update(void)
{
    if (!VL53L5CX_WaitForDataReady(1000)) return 0;
    if (VL53L5CX_GetData() != 0) return 0;

    s_last_insect_detected = 0;
    s_last_result.insect_detected = 0;
    s_last_result.trigger_source = 0;
    s_last_result.affected_count = 0;
    s_last_result.valid_measurements = 0;

    uint8_t frame_trig_signal = 0;
    uint8_t frame_trig_motion = 0;

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint8_t status = s_results.target_status[idx];

        int signal_triggered = 0;
        uint32_t signal_drop = 0;

        /* SIGNAL: original gates, unchanged (zone_valid, status 0/5/9,
           signal present, >= MIN, baseline drop > THRESH_PCT). */
        if (s_zone_valid[z] &&
            (status == 0 || status == 5 || status == 9)) {
            if (s_results.signal_per_spad[idx] == 0 &&
                s_baseline_signal[z] > 0 &&
                s_baseline_signal[z] < VL53L5CX_DET_MIN_SIGNAL) {
                s_baseline_signal[z]   = 0;
                s_baseline_distance[z] = 0;
            }

            if (s_results.signal_per_spad[idx] != 0 &&
                s_results.signal_per_spad[idx] >= VL53L5CX_DET_MIN_SIGNAL) {
                s_last_result.valid_measurements++;

                if (s_baseline_signal[z] > 0) {
                    int32_t diff = (int32_t)s_baseline_signal[z] - (int32_t)s_results.signal_per_spad[idx];
                    if (diff < 0) diff = -diff;
                    signal_drop = (uint32_t)diff * 100 / s_baseline_signal[z];
                }

                signal_triggered = (signal_drop > VL53L5CX_DET_THRESHOLD_PCT);
            }
        }

        /* MOTION: any zone whose motion surpasses the threshold is a motion
           detection — independent of status and signal. In practice the
           threshold is surpassed with status 5 AND status 255 (the motion
           plugin reports per-zone motion independently of the ranging
           status, and zones that were empty at boot are zone_valid=0). */
        int motion_triggered = 0;
        uint32_t motion_val = 0;
        if (s_motion_initialized) {
            motion_val = s_results.motion_indicator.motion[s_motion_config.map_id[z]];
            motion_triggered = (motion_val >= VL53L5CX_DET_MOTION_THRESH);
        }

        if (signal_triggered || motion_triggered) {
            uint8_t k = s_last_result.affected_count;
            s_last_result.affected_zones[k] = (uint8_t)z;
            s_last_result.affected_drop[k] = signal_triggered ? signal_drop : motion_val;
            s_last_result.affected_count++;

            if (signal_triggered) frame_trig_signal = 1;
            if (motion_triggered) frame_trig_motion = 1;
        }

        if (s_last_result.affected_count >= VL53L5CX_DET_MIN_AFFECTED_ZONES) {

            s_last_insect_detected = 1;
            s_last_result.insect_detected = 1;
            s_last_result.trigger_source = (frame_trig_signal | frame_trig_motion)
                ? (frame_trig_signal && frame_trig_motion ? VL53L5CX_TRIG_BOTH
                                                          : (frame_trig_signal ? VL53L5CX_TRIG_SIGNAL : VL53L5CX_TRIG_MOTION))
                : 0;
        }
    }

    /* BASELINE REFRESH */
#if VL53L5CX_DET_PERIODIC_RESTART_ENABLED > 0
    {
        static uint32_t frame_counter = 0;
        frame_counter++;
        if (frame_counter >= VL53L5CX_DET_PERIODIC_RESTART_INTERVAL) {
            frame_counter = 0;
            printf("[ToF] Periodic refresh...\n");
            vl53l5cx_stop_ranging(&s_dev);
            vTaskDelay(pdMS_TO_TICKS(50));
            vl53l5cx_start_ranging(&s_dev);
            vTaskDelay(pdMS_TO_TICKS(200));
            VL53L5CX_LearnBaseline();
            printf("[ToF] Periodic refresh done.\n");
        }
    }
#endif

#if VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED > 0
    {
        static uint32_t window_start = 0;
        static uint8_t  detection_count = 0;
        if (window_start == 0)
            window_start = xTaskGetTickCount();

        if (s_last_insect_detected)
            detection_count++;

        if ((xTaskGetTickCount() - window_start) >= pdMS_TO_TICKS(VL53L5CX_DET_REFRESH_WINDOW_SECS * 1000)) {
            window_start = xTaskGetTickCount();
            if (detection_count > VL53L5CX_DET_MAX_DETECTIONS) {
                printf("[ToF] Adaptive refresh: %d detections in %ds\n",
                       detection_count, VL53L5CX_DET_REFRESH_WINDOW_SECS);
                vl53l5cx_stop_ranging(&s_dev);
                vTaskDelay(pdMS_TO_TICKS(50));
                vl53l5cx_start_ranging(&s_dev);
                vTaskDelay(pdMS_TO_TICKS(200));
                VL53L5CX_LearnBaseline();
                printf("[ToF] Adaptive refresh done.\n");
            } else {
                printf("[ToF] Window: %d detections in %ds — no refresh\n",
                       detection_count, VL53L5CX_DET_REFRESH_WINDOW_SECS);
            }
            detection_count = 0;
        }
    }
#endif

    /* Debug output */
#if VL53L5CX_DET_DEBUG_ZFRAME > 0
    static uint32_t zframe_counter = 0;
    zframe_counter++;
    if (zframe_counter >= VL53L5CX_DET_DEBUG_ZFRAME_INT) {
        zframe_counter = 0;
        VL53L5CX_PrintZFrame();
#if VL53L5CX_DUAL_SENSOR
        VL53L5CX_External_PrintZFrame();
#endif
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

void VL53L5CX_PrintAllZoneParams(void)
{
    int8_t temp = s_results.silicon_temp_degc;
    printf("ALLPARAM,%d,", temp);
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint32_t cur_sig  = s_results.signal_per_spad[idx];
        int16_t  cur_dist = s_results.distance_mm[idx];
        uint32_t base_sig  = s_baseline_signal[z];
        uint16_t base_dist = s_baseline_distance[z];
        uint32_t drop_pct = 0;
        if (base_sig > 0) {
            int32_t diff = (int32_t)base_sig - (int32_t)cur_sig;
            if (diff < 0) diff = -diff;
            drop_pct = (uint32_t)diff * 100 / base_sig;
        }
        if (z > 0) printf(",");
        printf("%lu,%lu,%d,%d,%lu",
               (unsigned long)cur_sig, (unsigned long)base_sig,
               cur_dist, base_dist, (unsigned long)drop_pct);
    }
    printf("\r\n");
}

void VL53L5CX_PrintZFrame(void)
{
    int8_t temp = s_results.silicon_temp_degc;
    printf("ZFRAME,%d", temp);
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t cur_sig  = 0;
        uint16_t cur_dist = 0;
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint8_t status = s_results.target_status[idx];
        if (status == 0 || status == 5 || status == 9) {
            cur_sig  = s_results.signal_per_spad[idx];
            cur_dist = s_results.distance_mm[idx];
        }
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        uint32_t motion = s_results.motion_indicator.motion[z];
#else
        uint32_t motion = 0;
#endif
        printf(",%lu,%lu,%lu,%lu,%lu",
               (unsigned long)cur_sig, (unsigned long)cur_dist,
               (unsigned long)s_baseline_signal[z],
               (unsigned long)s_baseline_distance[z],
               (unsigned long)motion);
    }
    printf("\r\n");
}

void VL53L5CX_PrintBaselineFrame(void)
{
    printf("BASELINE,");
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (z > 0) printf(",");
        printf("%lu,%lu",
               (unsigned long)s_baseline_signal[z],
               (unsigned long)s_baseline_distance[z]);
    }
    printf("\r\n");
}

int VL53L5CX_ScanI2CBus(void)
{
    extern I2C_HandleTypeDef hi2c1;
    I2C_HandleTypeDef *hi2c = s_hi2c ? s_hi2c : &hi2c1;
    if (!hi2c) return 0;
    uint8_t found = 0;
    for (uint8_t addr = 0; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 10) == HAL_OK) {
            printf("  I2C device at 0x%02X\n", addr);
            found++;
        }
    }
    return found;
}

/* ================================================================
   Legacy Test Functions
   ================================================================ */

void VL53L5CX_Validate(void)
{
    printf("=== Validate ===\n");
    uint8_t alive;
    int status = vl53l5cx_is_alive(&s_dev, &alive);
    printf("Alive: %s (0x%02X)\n", alive ? "YES" : "NO", (uint8_t)status);
    printf("Temp: %d C\n", s_results.silicon_temp_degc);
}

void VL53L5CX_ReadingTest(void)
{
    printf("=== Reading Test ===\n");
    uint8_t alive;
    if (vl53l5cx_is_alive(&s_dev, &alive) != 0 || !alive) {
        printf("Sensor not alive!\n");
        return;
    }

    vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_4X4);
    vl53l5cx_set_integration_time_ms(&s_dev, 800);
    vl53l5cx_set_ranging_frequency_hz(&s_dev, 15);
    vl53l5cx_start_ranging(&s_dev);

    for (int i = 0; i < 10; i++) {
        uint8_t ready = 0;
        TickType_t start = xTaskGetTickCount();
        while (!ready) {
            vl53l5cx_check_data_ready(&s_dev, &ready);
            if (xTaskGetTickCount() - start > pdMS_TO_TICKS(1000)) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!ready) { printf("  Frame %d: TIMEOUT\n", i + 1); continue; }
        if (vl53l5cx_get_ranging_data(&s_dev, &s_results) != 0) {
            printf("  Frame %d: READ_ERROR\n", i + 1);
            continue;
        }

        float total_sig = 0, avg_dist = 0, zone_cnt = 0;
        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results.target_status[idx] == 0 ||
                s_results.target_status[idx] == 5 ||
                s_results.target_status[idx] == 9) {
                total_sig += s_results.signal_per_spad[idx];
                avg_dist += s_results.distance_mm[idx];
                zone_cnt++;
            }
        }

        float avg_sig = zone_cnt > 0 ? total_sig / zone_cnt : 0;
        avg_dist = zone_cnt > 0 ? avg_dist / zone_cnt : 0;
        printf("  Frame %2d: zones=%02d, avg_sig=%.0f spads, avg_dist=%.0f mm, temp=%d C\n",
               i + 1, (int)zone_cnt, avg_sig, avg_dist, s_results.silicon_temp_degc);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vl53l5cx_stop_ranging(&s_dev);
}

void VL53L5CX_MotionTest(void)
{
    printf("=== Motion Test ===\n");
    if (!VL53L5CX_WaitForDataReady(1000) || VL53L5CX_GetData() != 0) return;

    printf("  Global1=%lu  Global2=%lu  Status=%d\n",
           (unsigned long)s_results.motion_indicator.global_indicator_1,
           (unsigned long)s_results.motion_indicator.global_indicator_2,
           s_results.motion_indicator.status);
    printf("  nb_of_detected=%d  nb_of_aggregates=%d\n",
           s_results.motion_indicator.nb_of_detected_aggregates,
           s_results.motion_indicator.nb_of_aggregates);

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        if (s_results.target_status[idx] == 0 ||
            s_results.target_status[idx] == 5 ||
            s_results.target_status[idx] == 9) {
            uint32_t motion_val = s_results.motion_indicator.motion[s_motion_config.map_id[z]];
            printf("  Zone %2d -> motion=%lu (thresh=%d) %s\n",
                   z, (unsigned long)motion_val, VL53L5CX_DET_MOTION_THRESH,
                   motion_val >= VL53L5CX_DET_MOTION_THRESH ? "DETECTED" : "");
        }
    }
}

/* ================================================================
   Dual Sensor Mode Implementation
   ================================================================
   When VL53L5CX_DUAL_SENSOR == 1:
     - External sensor (at 0x62): always ON, continuously monitoring
     - Primary sensor (at 0x29): sleep mode by default, wakes on detection

   Flow:
     1. External sensor runs VL53L5CX_External_Update() continuously
     2. When external detects motion/signal drop -> wakes primary
     3. Primary stays active for VL53L5CX_DUAL_WAKE_DURATION_MS
     4. Primary returns to sleep after timeout
     5. External continues monitoring throughout
   ================================================================ */

#if VL53L5CX_DUAL_SENSOR

/* ================================================================
   Primary Sensor Power Management (Camera ToF Sleep/Wake)
   ================================================================ */

void VL53L5CX_Primary_Sleep(void)
{
    if (s_primary_state != PRIMARY_STATE_ACTIVE &&
        s_primary_state != PRIMARY_STATE_RETURNING) return;

    printf("[PRIMARY] Entering sleep mode\n");
    vl53l5cx_stop_ranging(&s_dev);
    vTaskDelay(pdMS_TO_TICKS(50));
    vl53l5cx_set_power_mode(&s_dev, VL53L5CX_POWER_MODE_SLEEP);
    s_primary_state = PRIMARY_STATE_SLEEP;
    s_primary_wake_time = 0;
}

void VL53L5CX_Primary_SleepAtStartup(void)
{
    /* At startup the primary is physically ranging (it was started before
       baseline learning) but s_primary_state still holds its initial SLEEP
       value, which would make VL53L5CX_Primary_Sleep() return immediately.
       Mark the state ACTIVE so the real stop-ranging + sleep sequence runs
       and the camera ToF is actually in ST sleep by default, as designed. */
    s_primary_state = PRIMARY_STATE_ACTIVE;
    VL53L5CX_Primary_Sleep();
}

void VL53L5CX_Primary_Wake(void)
{
    if (s_primary_state == PRIMARY_STATE_ACTIVE) return;

    printf("[PRIMARY] Waking from sleep\n");
    s_primary_state = PRIMARY_STATE_WAKING;

    /* Wake from ST sleep mode */
    vl53l5cx_set_power_mode(&s_dev, VL53L5CX_POWER_MODE_WAKEUP);
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Restart ranging (no re-init needed - config retained) */
    vl53l5cx_start_ranging(&s_dev);
    vTaskDelay(pdMS_TO_TICKS(200));

    s_primary_state = PRIMARY_STATE_ACTIVE;
    s_primary_wake_time = HAL_GetTick();
    printf("[PRIMARY] Active until %lu ms\n",
           (unsigned long)(s_primary_wake_time + VL53L5CX_DUAL_WAKE_DURATION_MS));
}

PrimaryState_t VL53L5CX_Primary_GetState(void)
{
    return s_primary_state;
}

int VL53L5CX_Primary_IsActive(void)
{
    return (s_primary_state == PRIMARY_STATE_ACTIVE) ? 1 : 0;
}

void VL53L5CX_Primary_CheckWakeTimeout(void)
{
    if (s_primary_state != PRIMARY_STATE_ACTIVE) return;

    uint32_t elapsed = HAL_GetTick() - s_primary_wake_time;
    if (elapsed >= VL53L5CX_DUAL_WAKE_DURATION_MS) {
        printf("[PRIMARY] Wake timeout expired (%lu ms), returning to sleep\n",
               (unsigned long)elapsed);
        s_primary_state = PRIMARY_STATE_RETURNING;
        s_external_state = EXTERNAL_STATE_WAITING;
        VL53L5CX_Primary_Sleep();
        s_external_state = EXTERNAL_STATE_MONITORING;
    }
}

/* ================================================================
   External Sensor (Guardian) API
   ================================================================ */

int VL53L5CX_External_Init(void)
{
    uint8_t is_alive;
    int init_status;

    s_dev_ext.platform.address = 0x31;

    printf("[EXT] Waiting for sensor at 0x%02X...\n", VL53L5CX_EXTERNAL_ADDRESS);
    for (uint8_t retry = 0; retry < 5; retry++) {
        init_status = vl53l5cx_is_alive(&s_dev_ext, &is_alive);
        if (is_alive && init_status == 0) break;
        printf("[EXT] Not ready (retry %d/5)...\n", retry + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!is_alive || init_status != 0) {
        printf("[EXT] ERROR: Sensor not detected at 0x%02X!\n", VL53L5CX_EXTERNAL_ADDRESS);
        s_external_state = EXTERNAL_STATE_IDLE;
        return -1;
    }

    for (uint8_t r = 0; r < 3; r++) {
        init_status = vl53l5cx_init(&s_dev_ext);
        if (init_status == 0) break;
        printf("[EXT] Init retry %d/3...\n", r + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (init_status != 0) {
        printf("[EXT] ERROR: Init failed (status=%d)!\n", init_status);
        s_external_state = EXTERNAL_STATE_IDLE;
        return -2;
    }

    s_external_state = EXTERNAL_STATE_IDLE;
    printf("[EXT] Initialized successfully!\n");
    return 0;
}

void VL53L5CX_External_Configure(void)
{
    vl53l5cx_set_resolution(&s_dev_ext, VL53L5CX_RESOLUTION_4X4);
    vl53l5cx_set_integration_time_ms(&s_dev_ext, 800);
    vl53l5cx_set_ranging_frequency_hz(&s_dev_ext, 15);
    vl53l5cx_set_target_order(&s_dev_ext, VL53L5CX_TARGET_ORDER_CLOSEST);
    vl53l5cx_set_sharpener_percent(&s_dev_ext, 10);
    vl53l5cx_set_ranging_mode(&s_dev_ext, VL53L5CX_RANGING_MODE_CONTINUOUS);

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    int motion_st = vl53l5cx_motion_indicator_init(&s_dev_ext, &s_motion_config_ext,
        VL53L5CX_RESOLUTION_4X4);
    if (motion_st) {
        printf("[EXT] WARN: Motion indicator init failed: %d\n", motion_st);
        s_motion_initialized_ext = 0;
    } else {
        s_motion_config_ext.min_nb_for_global_detection = VL53L5CX_DET_MOTION_MIN_ZONES;
        s_motion_config_ext.nb_of_temporal_accumulations = VL53L5CX_DET_MOTION_PERSIST_FRAMES;
        s_motion_config_ext.extra_noise_sigma = VL53L5CX_DET_MOTION_EXTRA_NOISE;
        s_motion_initialized_ext = 1;
        printf("[EXT] Motion indicator enabled\n");
    }
#else
    s_motion_initialized_ext = 0;
#endif

    printf("[EXT] Configured: res=4x4, int=800ms, freq=15Hz\n");
}

void VL53L5CX_External_StartRanging(void)
{
    vl53l5cx_start_ranging(&s_dev_ext);
    vTaskDelay(pdMS_TO_TICKS(200));
    s_external_state = EXTERNAL_STATE_MONITORING;
    printf("[EXT] Ranging started\n");
}

void VL53L5CX_External_StopRanging(void)
{
    vl53l5cx_stop_ranging(&s_dev_ext);
    printf("[EXT] Ranging stopped\n");
}

void VL53L5CX_External_LearnBaseline(void)
{
    memset(s_baseline_signal_ext, 0, sizeof(s_baseline_signal_ext));
    memset(s_baseline_distance_ext, 0, sizeof(s_baseline_distance_ext));
    memset(s_zone_valid_ext, 0, sizeof(s_zone_valid_ext));
    s_baseline_ready_ext = 0;

    const uint8_t baseline_samples = VL53L5CX_SENSOR2_BASELINE_SAMPLES;
    const uint8_t settle_frames = 3;

    printf("[EXT] Baseline: %d samples + %d settle...\n", baseline_samples, settle_frames);

    for (uint8_t i = 0; i < baseline_samples; i++) {
        uint8_t is_ready = 0;
        TickType_t start = xTaskGetTickCount();

        while (!is_ready) {
            vl53l5cx_check_data_ready(&s_dev_ext, &is_ready);
            if (is_ready) break;
            if (xTaskGetTickCount() - start > pdMS_TO_TICKS(1000)) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!is_ready) continue;

        if (vl53l5cx_get_ranging_data(&s_dev_ext, &s_results_ext) != 0) continue;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results_ext.target_status[idx] == 0 ||
                s_results_ext.target_status[idx] == 5 ||
                s_results_ext.target_status[idx] == 9) {
                if (s_results_ext.signal_per_spad[idx] < VL53L5CX_DET_MIN_SIGNAL) continue;
                s_baseline_signal_ext[z] += s_results_ext.signal_per_spad[idx];
                s_baseline_distance_ext[z] += s_results_ext.distance_mm[idx];
                s_zone_valid_ext[z] = 1;
            }
        }
        printf("  [EXT BASELINE %d/%d]\r", i + 1, baseline_samples);
    }

    uint8_t valid_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (s_zone_valid_ext[z]) {
            s_baseline_signal_ext[z] /= baseline_samples;
            s_baseline_distance_ext[z] /= baseline_samples;
            valid_count++;
        }
    }

    for (uint8_t i = 0; i < settle_frames; i++) {
        uint8_t is_ready = 0;
        TickType_t start = xTaskGetTickCount();
        while (!is_ready) {
            vl53l5cx_check_data_ready(&s_dev_ext, &is_ready);
            if (is_ready) break;
            if (xTaskGetTickCount() - start > pdMS_TO_TICKS(1000)) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        (void)vl53l5cx_get_ranging_data(&s_dev_ext, &s_results_ext);
        printf("  [EXT SETTLE %d/%d]\r", i + 1, settle_frames);
    }

    s_baseline_ready_ext = 1;
    printf("\n[EXT] Baseline done. Valid zones: %d/%d\n", valid_count, VL53L5CX_DET_NUM_ZONES);
}

int VL53L5CX_External_Update(void)
{
    if (s_external_state != EXTERNAL_STATE_MONITORING) return 0;

    if (vl53l5cx_get_ranging_data(&s_dev_ext, &s_results_ext) != 0) return 0;

    s_last_insect_detected_ext = 0;
    s_last_result_ext.insect_detected = 0;
    s_last_result_ext.trigger_source = 0;
    s_last_result_ext.affected_count = 0;
    s_last_result_ext.valid_measurements = 0;

    uint8_t frame_trig_signal = 0;
    uint8_t frame_trig_motion = 0;

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (!s_zone_valid_ext[z]) continue;

        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

        if (s_results_ext.target_status[idx] != 0 &&
            s_results_ext.target_status[idx] != 5 &&
            s_results_ext.target_status[idx] != 9)
            continue;

        if (s_results_ext.signal_per_spad[idx] == 0) continue;
        if (s_results_ext.signal_per_spad[idx] < VL53L5CX_DET_MIN_SIGNAL) continue;

        s_last_result_ext.valid_measurements++;

        uint32_t signal_drop = 0;
        if (s_baseline_signal_ext[z] > 0) {
            int32_t diff = (int32_t)s_baseline_signal_ext[z] - (int32_t)s_results_ext.signal_per_spad[idx];
            if (diff < 0) diff = -diff;
            signal_drop = (uint32_t)diff * 100 / s_baseline_signal_ext[z];
        }

        int signal_triggered = (signal_drop > VL53L5CX_DET_THRESHOLD_PCT);

        int motion_triggered = 0;
        if (s_motion_initialized_ext) {
            uint32_t motion_val = s_results_ext.motion_indicator.motion[s_motion_config_ext.map_id[z]];
            motion_triggered = (motion_val >= VL53L5CX_DET_MOTION_THRESH);
        }

        if (signal_triggered || motion_triggered) {
            uint8_t k = s_last_result_ext.affected_count;
            s_last_result_ext.affected_zones[k] = (uint8_t)z;
            s_last_result_ext.affected_drop[k] = signal_triggered ? signal_drop :
                s_results_ext.motion_indicator.motion[s_motion_config_ext.map_id[z]];
            s_last_result_ext.affected_count++;

            if (signal_triggered) frame_trig_signal = 1;
            if (motion_triggered) frame_trig_motion = 1;
        }

        if (s_last_result_ext.affected_count >= VL53L5CX_DET_MIN_AFFECTED_ZONES) {
            s_last_insect_detected_ext = 1;
            s_last_result_ext.insect_detected = 1;
            s_last_result_ext.trigger_source = (frame_trig_signal | frame_trig_motion)
                ? (frame_trig_signal && frame_trig_motion ? VL53L5CX_TRIG_BOTH
                                                          : (frame_trig_signal ? VL53L5CX_TRIG_SIGNAL : VL53L5CX_TRIG_MOTION))
                : 0;
        }
    }

    /* ---- BASELINE REFRESH for external sensor ----
       Periodic restart keeps the guardian baseline accurate. */
#if VL53L5CX_DET_PERIODIC_RESTART_ENABLED > 0
    {
        static uint32_t ext_frame_counter = 0;
        ext_frame_counter++;
        if (ext_frame_counter >= VL53L5CX_DET_PERIODIC_RESTART_INTERVAL) {
            ext_frame_counter = 0;
            printf("[EXT] Periodic refresh...\n");
            vl53l5cx_stop_ranging(&s_dev_ext);
            vTaskDelay(pdMS_TO_TICKS(50));
            vl53l5cx_start_ranging(&s_dev_ext);
            vTaskDelay(pdMS_TO_TICKS(200));
            VL53L5CX_External_LearnBaseline();
            printf("[EXT] Periodic refresh done.\n");
        }
    }
#endif

#if VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED > 0
    {
        static uint32_t ext_window_start = 0;
        static uint8_t  ext_detection_count = 0;
        if (ext_window_start == 0)
            ext_window_start = xTaskGetTickCount();

        if (s_last_insect_detected_ext)
            ext_detection_count++;

        if ((xTaskGetTickCount() - ext_window_start) >= pdMS_TO_TICKS(VL53L5CX_DET_REFRESH_WINDOW_SECS * 1000)) {
            ext_window_start = xTaskGetTickCount();
            if (ext_detection_count > VL53L5CX_DET_MAX_DETECTIONS) {
                printf("[EXT] Adaptive refresh: %d detections in %ds\n",
                       ext_detection_count, VL53L5CX_DET_REFRESH_WINDOW_SECS);
                vl53l5cx_stop_ranging(&s_dev_ext);
                vTaskDelay(pdMS_TO_TICKS(50));
                vl53l5cx_start_ranging(&s_dev_ext);
                vTaskDelay(pdMS_TO_TICKS(200));
                VL53L5CX_External_LearnBaseline();
                printf("[EXT] Adaptive refresh done.\n");
            } else {
                printf("[EXT] Window: %d detections in %ds — no refresh\n",
                       ext_detection_count, VL53L5CX_DET_REFRESH_WINDOW_SECS);
            }
            ext_detection_count = 0;
        }
    }
#endif

    /* If detection and primary is sleeping, wake it up */
    if (s_last_insect_detected_ext && s_primary_state == PRIMARY_STATE_SLEEP) {
        s_primary_detection_confirm++;
        if (s_primary_detection_confirm >= VL53L5CX_DUAL_CONFIRM_FRAMES) {
            printf("[EXT] Detection confirmed! Waking primary sensor...\n");
            s_external_state = EXTERNAL_STATE_DETECTED;
            VL53L5CX_Primary_Wake();
            s_primary_detection_confirm = 0;
        }
    } else if (!s_last_insect_detected_ext) {
        s_primary_detection_confirm = 0;
    }

    return 1;
}

int VL53L5CX_External_IsInsectDetected(void)
{
    return s_last_insect_detected_ext;
}

ExternalState_t VL53L5CX_External_GetState(void)
{
    return s_external_state;
}

int VL53L5CX_External_IsBaselineReady(void)
{
    return s_baseline_ready_ext;
}

void VL53L5CX_External_PrintZFrame(void)
{
    int8_t temp = s_results_ext.silicon_temp_degc;
    printf("EXT,ZFRAME,%d", temp);
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t cur_sig  = 0;
        uint16_t cur_dist = 0;
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint8_t status = s_results_ext.target_status[idx];

        if (status == 0 || status == 5 || status == 9) {
            cur_sig  = s_results_ext.signal_per_spad[idx];
            cur_dist = s_results_ext.distance_mm[idx];
        }

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        uint32_t motion = s_results_ext.motion_indicator.motion[z];
#else
        uint32_t motion = 0;
#endif

        printf(",%lu,%lu,%lu,%lu,%lu",
               (unsigned long)cur_sig, (unsigned long)cur_dist,
               (unsigned long)s_baseline_signal_ext[z],
               (unsigned long)s_baseline_distance_ext[z],
               (unsigned long)motion);
    }
    printf("\r\n");
}

void VL53L5CX_External_PrintAllZoneParams(void)
{
    int8_t temp = s_results_ext.silicon_temp_degc;
    printf("EXT,ALLPARAM,%d,", temp);
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint32_t cur_sig  = s_results_ext.signal_per_spad[idx];
        int16_t  cur_dist = s_results_ext.distance_mm[idx];
        uint32_t base_sig  = s_baseline_signal_ext[z];
        uint16_t base_dist = s_baseline_distance_ext[z];
        uint32_t drop_pct = 0;
        if (base_sig > 0) {
            int32_t diff = (int32_t)base_sig - (int32_t)cur_sig;
            if (diff < 0) diff = -diff;
            drop_pct = (uint32_t)diff * 100 / base_sig;
        }
        if (z > 0) printf(",");
        printf("%lu,%lu,%d,%d,%lu",
               (unsigned long)cur_sig, (unsigned long)base_sig,
               cur_dist, base_dist, (unsigned long)drop_pct);
    }
    printf("\r\n");
}

#endif /* VL53L5CX_DUAL_SENSOR */
