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
   Secondary Sensor (Camera ToF) State
   ================================================================ */
#if VL53L5CX_DUAL_SENSOR
static VL53L5CX_Configuration  s_dev2;
static VL53L5CX_ResultsData    s_results2;
static VL53L5CX_Motion_Configuration s_motion_config2;
static uint8_t s_motion_initialized2 = 0;

static uint32_t  s_baseline_signal2[VL53L5CX_DET_NUM_ZONES] = {0};
static uint16_t  s_baseline_distance2[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_zone_valid2[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t   s_baseline_ready2 = 0;

static VL53L5CX_DetectionResult_t s_last_result2 = {0};
static uint8_t s_last_insect_detected2 = 0;

static Sensor2State_t s_sensor2_state = SENSOR2_STATE_SLEEP;
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

/** Unified power-up function.
    When VL53L5CX_DUAL_SENSOR==1: powers both ToF devices, re-addresses external to 0x62.
    When VL53L5CX_DUAL_SENSOR==0: powers single external ToF at 0x29.
    Call this from main() instead of VL53L5CX_PowerUpDevices() or VL53L5CX_PowerUp(). */
void VL53L5CX_PowerUp(void)
{
	int i2cdevices = 0;
	i2cdevices = VL53L5CX_ScanI2CBus();
#if VL53L5CX_DUAL_SENSOR

	if(i2cdevices==3){
		GPIO_InitTypeDef GPIO_InitStruct = {0};

		//__HAL_RCC_GPIOD_CLK_ENABLE();
		//__HAL_RCC_GPIOE_CLK_ENABLE();
		__HAL_RCC_GPIOQ_CLK_ENABLE();

		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

		/* Dual sensor: need PD0(PWR_EN), PE7(I2C_RST), PD6(LPn_ext), PQ5(LPn_cam) */
		GPIO_InitStruct.Pin = GPIO_PIN_0;  /* PWR_EN */
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_7;  /* I2C_RST */
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_6;  /* LPn external */
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_5;  /* LPn camera */
		HAL_GPIO_Init(GPIOQ, &GPIO_InitStruct);

		printf("\n=== ToF Dual Sensor Power Up ===\n");

		/* Power enable */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_Delay(10);

		/* I2C reset sequence */
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
		s_dev2.platform.address = 0x29;
		uint8_t addr_st = vl53l5cx_set_i2c_address(&s_dev2, 0x62);
		if (addr_st != 0) {
			printf("[WARN] Address change failed (status=%d), external may conflict!\n", addr_st);
		} else {
			printf("[OK] External ToF address changed to 0x62\n");
		}

		/* Now power camera ToF */
		HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);

		printf("[OK] Dual sensor power-up complete\n\n");

	}
#else
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = GPIO_PIN_0;  /* PWR_EN */
    GPIO_InitStruct.Pin = GPIO_PIN_6;  /* LPn external */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
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

    /* Initialize motion indicator plugin */
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
        printf("[ToF] Motion indicator enabled (min_zones=%d, persist=%d, extra_noise=%d)\n",
               VL53L5CX_DET_MOTION_MIN_ZONES,
               VL53L5CX_DET_MOTION_PERSIST_FRAMES,
               VL53L5CX_DET_MOTION_EXTRA_NOISE);
    }
#else
    s_motion_initialized = 0;
    printf("[ToF] Motion indicator disabled (compile flag)\n");
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
    printf("[ToF] Baseline reset (signal + distance)\n");
}

void VL53L5CX_LearnBaseline(void)
{
    VL53L5CX_ResetBaseline();

    const uint8_t baseline_samples = VL53L5CX_DET_BASELINE_SAMPLES;
    const uint8_t settle_frames = 5;

    printf("[BASELINE] Learning %d samples + %d settle frames...\n",
           baseline_samples, settle_frames);

    /* ---- Phase 1: Collect baseline samples ---- */
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

    /* Compute averages */
    uint8_t valid_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (s_zone_valid[z]) {
            s_baseline_signal[z] /= baseline_samples;
            s_baseline_distance[z] /= baseline_samples;
            valid_count++;
        }
    }

    /* ---- Phase 2: Settle frames ---- */
    for (uint8_t i = 0; i < settle_frames; i++) {
        if (!VL53L5CX_WaitForDataReady(1000)) continue;
        if (VL53L5CX_GetData() != 0) continue;
        printf("  [SETTLE %d/%d]\r", i + 1, settle_frames);
    }

    s_baseline_ready = 1;
    printf("\n[BASELINE] Done. Valid zones: %d/%d\n", valid_count, VL53L5CX_DET_NUM_ZONES);

    /* Emit baseline frame for Python monitor */
    VL53L5CX_PrintBaselineFrame();
}

/* ================================================================
   Detection
   ================================================================ */

int VL53L5CX_Update(void)
{
    if (!VL53L5CX_WaitForDataReady(1000)) return 0;
    if (VL53L5CX_GetData() != 0) return 0;

    /* Reset detection result */
    s_last_insect_detected = 0;
    s_last_result.insect_detected = 0;
    s_last_result.trigger_source = 0;
    s_last_result.affected_count = 0;
    s_last_result.valid_measurements = 0;

    /* Track which methods triggered across all zones */
    uint8_t frame_trig_signal = 0;
    uint8_t frame_trig_motion = 0;

    /* Check each zone */
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (!s_zone_valid[z]) continue;

        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

        if (s_results.target_status[idx] != 0 &&
            s_results.target_status[idx] != 5 &&
            s_results.target_status[idx] != 9)
            continue;

        /* Clear stale baselines for unreliable zones */
        if (s_results.signal_per_spad[idx] == 0 &&
            s_baseline_signal[z] > 0 &&
            s_baseline_signal[z] < VL53L5CX_DET_MIN_SIGNAL) {
            s_baseline_signal[z]   = 0;
            s_baseline_distance[z] = 0;
        }

        /* Skip zero-signal zones */
        if (s_results.signal_per_spad[idx] == 0)
            continue;

        /* Skip zones with signal below minimum threshold */
        if (s_results.signal_per_spad[idx] < VL53L5CX_DET_MIN_SIGNAL)
            continue;

        s_last_result.valid_measurements++;

        /* Compute signal drop */
        uint32_t signal_drop = 0;
        if (s_baseline_signal[z] > 0) {
            int32_t diff = (int32_t)s_baseline_signal[z] - (int32_t)s_results.signal_per_spad[idx];
            if (diff < 0) diff = -diff;
            signal_drop = (uint32_t)diff * 100 / s_baseline_signal[z];
        }

        /* Check signal drop threshold */
        int signal_triggered = (signal_drop > VL53L5CX_DET_THRESHOLD_PCT);

        /* Check motion indicator (fixed threshold) */
        int motion_triggered = 0;
        if (s_motion_initialized) {
            uint32_t motion_val = s_results.motion_indicator.motion[s_motion_config.map_id[z]];
            motion_triggered = (motion_val >= VL53L5CX_DET_MOTION_THRESH);
        }

        /* Detection triggers if EITHER signal drop OR motion exceeds threshold */
        if (signal_triggered || motion_triggered) {
            uint8_t k = s_last_result.affected_count;
            s_last_result.affected_zones[k] = (uint8_t)z;
            s_last_result.affected_drop[k] = signal_triggered ? signal_drop :
                s_results.motion_indicator.motion[s_motion_config.map_id[z]];
            s_last_result.affected_count++;

            if (signal_triggered) frame_trig_signal = 1;
            if (motion_triggered) frame_trig_motion = 1;
        }

        /* Only set insect_detected if enough zones are affected */
        if (s_last_result.affected_count >= VL53L5CX_DET_MIN_AFFECTED_ZONES) {
            s_last_insect_detected = 1;
            s_last_result.insect_detected = 1;
            s_last_result.trigger_source = (frame_trig_signal | frame_trig_motion)
                ? (frame_trig_signal && frame_trig_motion ? VL53L5CX_TRIG_BOTH
                                                          : (frame_trig_signal ? VL53L5CX_TRIG_SIGNAL : VL53L5CX_TRIG_MOTION))
                : 0;
        }
    }

    /* ================================================================
       BASELINE REFRESH — Two modes (choose ONE in header)
       ================================================================ */

    /* --- MODE 1: Periodic Restart (fixed interval) --- */
#if VL53L5CX_DET_PERIODIC_RESTART_ENABLED > 0
    {
        static uint32_t frame_counter = 0;
        frame_counter++;

        if (frame_counter >= VL53L5CX_DET_PERIODIC_RESTART_INTERVAL) {
            frame_counter = 0;
            printf("[ToF] Periodic refresh (every %d frames)...\n",
                   VL53L5CX_DET_PERIODIC_RESTART_INTERVAL);

            vl53l5cx_stop_ranging(&s_dev);
            vTaskDelay(pdMS_TO_TICKS(50));
            vl53l5cx_start_ranging(&s_dev);
            vTaskDelay(pdMS_TO_TICKS(200));
            VL53L5CX_LearnBaseline();

            printf("[ToF] Periodic refresh done.\n");
        }
    }
#endif

    /* --- MODE 2: Adaptive Refresh (time-based, detection-rate) --- */
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
                printf("[ToF] Adaptive refresh: %d detections in %ds (max=%d)\n",
                       detection_count, VL53L5CX_DET_REFRESH_WINDOW_SECS,
                       VL53L5CX_DET_MAX_DETECTIONS);

                vl53l5cx_stop_ranging(&s_dev);
                vTaskDelay(pdMS_TO_TICKS(50));
                vl53l5cx_start_ranging(&s_dev);
                vTaskDelay(pdMS_TO_TICKS(200));
                VL53L5CX_LearnBaseline();

                printf("[ToF] Adaptive refresh done.\n");
            } else {
                printf("[ToF] Window: %d detections in %ds (max=%d) — no refresh\n",
                       detection_count, VL53L5CX_DET_REFRESH_WINDOW_SECS,
                       VL53L5CX_DET_MAX_DETECTIONS);
            }
            detection_count = 0;
        }
    }
#endif

    /* Debug output: ZFRAME (compact, for real-time monitoring) */
#if VL53L5CX_DET_DEBUG_ZFRAME > 0
    static uint32_t zframe_counter = 0;
    zframe_counter++;
    if (zframe_counter >= VL53L5CX_DET_DEBUG_ZFRAME_INT) {
        zframe_counter = 0;
        VL53L5CX_PrintZFrame();
#if VL53L5CX_DUAL_SENSOR
        VL53L5CX_Sensor2_PrintZFrame();
#endif
    }
#endif

    /* Debug output: ALLPARAM (detailed, for datalogging + analysis) */
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

    printf("ALLPARAM,");
    printf("%d,", temp);
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint32_t cur_sig     = s_results.signal_per_spad[idx];
        int16_t  cur_dist    = s_results.distance_mm[idx];
        uint8_t  cur_stat    = s_results.target_status[idx];
        uint32_t cur_ambient = 0;
        uint16_t cur_sigma   = 0;
        uint8_t  cur_reflect = 0;
        uint32_t cur_spads   = 0;
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

        uint32_t base_sig  = s_baseline_signal[z];
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

    /* MOTION line */
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

void VL53L5CX_PrintZFrame(void)
{
    int8_t temp = s_results.silicon_temp_degc;
    printf("ZFRAME,%d", temp);

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t cur_sig  = 0;
        uint16_t cur_dist = 0;
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint8_t status = s_results.target_status[idx];

        if (status == 6 || status == 5 || status == 9) {
            cur_sig  = s_results.signal_per_spad[idx];
            cur_dist = s_results.distance_mm[idx];
        }

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        uint32_t motion = s_results.motion_indicator.motion[z];
#else
        uint32_t motion = 0;
#endif

        /* ZFRAME format: sig, dist, base_sig, base_dist, motion */
        printf(",%lu,%lu,%lu,%lu,%lu",
               (unsigned long)cur_sig,
               (unsigned long)cur_dist,
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
   Secondary Sensor API (Dual Mode)
   ================================================================ */

#if VL53L5CX_DUAL_SENSOR

int VL53L5CX_Sensor2_Init(void)
{
    uint8_t is_alive;
    int init_status;

    if (s_hi2c == NULL) return -1;

    s_dev2.platform.address = VL53L5CX_SECONDARY_ADDRESS;

    printf("[S2] Waiting for sensor to respond...\n");
    for (uint8_t retry = 0; retry < 5; retry++) {
        init_status = vl53l5cx_is_alive(&s_dev2, &is_alive);
        if (is_alive && init_status == 0) break;
        printf("[S2] Not ready (retry %d/5)...\n", retry + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (!is_alive || init_status != 0) {
        printf("[S2] ERROR: Sensor not detected at 0x%02X!\n", VL53L5CX_SECONDARY_ADDRESS);
        s_sensor2_state = SENSOR2_STATE_SLEEP;
        return -1;
    }

    for (uint8_t r = 0; r < 3; r++) {
        init_status = vl53l5cx_init(&s_dev2);
        if (init_status == 0) break;
        printf("[S2] Init retry %d/3...\n", r + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (init_status != 0) {
        printf("[S2] ERROR: Init failed (status=%d)!\n", init_status);
        s_sensor2_state = SENSOR2_STATE_SLEEP;
        return -2;
    }

    s_sensor2_state = SENSOR2_STATE_READY;
    printf("[S2] Initialized successfully!\n");
    return 0;
}

void VL53L5CX_Sensor2_Configure(void)
{
    vl53l5cx_set_resolution(&s_dev2, VL53L5CX_RESOLUTION_4X4);
    vl53l5cx_set_integration_time_ms(&s_dev2, 800);
    vl53l5cx_set_ranging_frequency_hz(&s_dev2, 15);
    vl53l5cx_set_target_order(&s_dev2, VL53L5CX_TARGET_ORDER_CLOSEST);
    vl53l5cx_set_sharpener_percent(&s_dev2, 10);
    vl53l5cx_set_ranging_mode(&s_dev2, VL53L5CX_RANGING_MODE_CONTINUOUS);

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    int motion_st = vl53l5cx_motion_indicator_init(&s_dev2, &s_motion_config2,
        VL53L5CX_RESOLUTION_4X4);
    if (motion_st) {
        printf("[S2] WARN: Motion indicator init failed: %d\n", motion_st);
        s_motion_initialized2 = 0;
    } else {
        s_motion_config2.min_nb_for_global_detection = VL53L5CX_DET_MOTION_MIN_ZONES;
        s_motion_config2.nb_of_temporal_accumulations = VL53L5CX_DET_MOTION_PERSIST_FRAMES;
        s_motion_config2.extra_noise_sigma = VL53L5CX_DET_MOTION_EXTRA_NOISE;
        s_motion_initialized2 = 1;
        printf("[S2] Motion indicator enabled\n");
    }
#else
    s_motion_initialized2 = 0;
#endif

    printf("[S2] Configured: res=4x4, int=800ms, freq=15Hz\n");
}

void VL53L5CX_Sensor2_StartRanging(void)
{
    vl53l5cx_start_ranging(&s_dev2);
    vTaskDelay(pdMS_TO_TICKS(200));
    s_sensor2_state = SENSOR2_STATE_MONITORING;
    printf("[S2] Ranging started\n");
}

void VL53L5CX_Sensor2_StopRanging(void)
{
    vl53l5cx_stop_ranging(&s_dev2);
    printf("[S2] Ranging stopped\n");
}

void VL53L5CX_Sensor2_LearnBaseline(void)
{
    memset(s_baseline_signal2, 0, sizeof(s_baseline_signal2));
    memset(s_baseline_distance2, 0, sizeof(s_baseline_distance2));
    memset(s_zone_valid2, 0, sizeof(s_zone_valid2));
    s_baseline_ready2 = 0;

    const uint8_t baseline_samples = VL53L5CX_SENSOR2_BASELINE_SAMPLES;
    const uint8_t settle_frames = 3;

    printf("[S2] Baseline: %d samples + %d settle...\n", baseline_samples, settle_frames);

    for (uint8_t i = 0; i < baseline_samples; i++) {
        uint8_t is_ready = 0;
        TickType_t start = xTaskGetTickCount();

        while (!is_ready) {
            vl53l5cx_check_data_ready(&s_dev2, &is_ready);
            if (is_ready) break;
            if (xTaskGetTickCount() - start > pdMS_TO_TICKS(1000)) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!is_ready) continue;

        if (vl53l5cx_get_ranging_data(&s_dev2, &s_results2) != 0) continue;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (s_results2.target_status[idx] == 0 ||
                s_results2.target_status[idx] == 5 ||
                s_results2.target_status[idx] == 9) {
                if (s_results2.signal_per_spad[idx] < VL53L5CX_DET_MIN_SIGNAL) continue;
                s_baseline_signal2[z] += s_results2.signal_per_spad[idx];
                s_baseline_distance2[z] += s_results2.distance_mm[idx];
                s_zone_valid2[z] = 1;
            }
        }
        printf("  [S2 BASELINE %d/%d]\r", i + 1, baseline_samples);
    }

    uint8_t valid_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (s_zone_valid2[z]) {
            s_baseline_signal2[z] /= baseline_samples;
            s_baseline_distance2[z] /= baseline_samples;
            valid_count++;
        }
    }

    for (uint8_t i = 0; i < settle_frames; i++) {
        uint8_t is_ready = 0;
        TickType_t start = xTaskGetTickCount();

        while (!is_ready) {
            vl53l5cx_check_data_ready(&s_dev2, &is_ready);
            if (is_ready) break;
            if (xTaskGetTickCount() - start > pdMS_TO_TICKS(1000)) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        (void)vl53l5cx_get_ranging_data(&s_dev2, &s_results2);
        printf("  [S2 SETTLE %d/%d]\r", i + 1, settle_frames);
    }

    s_baseline_ready2 = 1;
    printf("\n[S2] Baseline done. Valid zones: %d/%d\n", valid_count, VL53L5CX_DET_NUM_ZONES);
}

int VL53L5CX_Sensor2_RunDetection(void)
{
    if (vl53l5cx_get_ranging_data(&s_dev2, &s_results2) != 0) return 0;

    s_last_insect_detected2 = 0;
    s_last_result2.insect_detected = 0;
    s_last_result2.trigger_source = 0;
    s_last_result2.affected_count = 0;
    s_last_result2.valid_measurements = 0;

    uint8_t frame_trig_signal = 0;
    uint8_t frame_trig_motion = 0;

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (!s_zone_valid2[z]) continue;

        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

        if (s_results2.target_status[idx] != 0 &&
            s_results2.target_status[idx] != 5 &&
            s_results2.target_status[idx] != 9)
            continue;

        if (s_results2.signal_per_spad[idx] == 0) continue;
        if (s_results2.signal_per_spad[idx] < VL53L5CX_DET_MIN_SIGNAL) continue;

        s_last_result2.valid_measurements++;

        /* Compute signal drop */
        uint32_t signal_drop = 0;
        if (s_baseline_signal2[z] > 0) {
            int32_t diff = (int32_t)s_baseline_signal2[z] - (int32_t)s_results2.signal_per_spad[idx];
            if (diff < 0) diff = -diff;
            signal_drop = (uint32_t)diff * 100 / s_baseline_signal2[z];
        }

        int signal_triggered = (signal_drop > VL53L5CX_DET_THRESHOLD_PCT);

        /* Check motion (fixed threshold) */
        int motion_triggered = 0;
        if (s_motion_initialized2) {
            uint32_t motion_val = s_results2.motion_indicator.motion[s_motion_config2.map_id[z]];
            motion_triggered = (motion_val >= VL53L5CX_DET_MOTION_THRESH);
        }

        if (signal_triggered || motion_triggered) {
            uint8_t k = s_last_result2.affected_count;
            s_last_result2.affected_zones[k] = (uint8_t)z;
            s_last_result2.affected_drop[k] = signal_triggered ? signal_drop :
                s_results2.motion_indicator.motion[s_motion_config2.map_id[z]];
            s_last_result2.affected_count++;

            if (signal_triggered) frame_trig_signal = 1;
            if (motion_triggered) frame_trig_motion = 1;
        }

        if (s_last_result2.affected_count >= VL53L5CX_DET_MIN_AFFECTED_ZONES) {
            s_last_insect_detected2 = 1;
            s_last_result2.insect_detected = 1;
            s_last_result2.trigger_source = (frame_trig_signal | frame_trig_motion)
                ? (frame_trig_signal && frame_trig_motion ? VL53L5CX_TRIG_BOTH
                                                          : (frame_trig_signal ? VL53L5CX_TRIG_SIGNAL : VL53L5CX_TRIG_MOTION))
                : 0;
        }
    }

    return s_last_insect_detected2;
}

int VL53L5CX_Sensor2_IsInsectDetected(void)
{
    return s_last_insect_detected2;
}

Sensor2State_t VL53L5CX_Sensor2_GetState(void)
{
    return s_sensor2_state;
}

int VL53L5CX_Sensor2_IsBaselineReady(void)
{
    return s_baseline_ready2;
}

void VL53L5CX_Sensor2_PrintZFrame(void)
{
    int8_t temp = s_results2.silicon_temp_degc;
    printf("S2,ZFRAME,%d", temp);

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t cur_sig  = 0;
        uint16_t cur_dist = 0;
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint8_t status = s_results2.target_status[idx];

        if (status == 6 || status == 5 || status == 9) {
            cur_sig  = s_results2.signal_per_spad[idx];
            cur_dist = s_results2.distance_mm[idx];
        }

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        uint32_t motion = s_results2.motion_indicator.motion[z];
#else
        uint32_t motion = 0;
#endif

        /* S2,ZFRAME format: sig, dist, base_sig, base_dist, motion */
        printf(",%lu,%lu,%lu,%lu,%lu",
               (unsigned long)cur_sig,
               (unsigned long)cur_dist,
               (unsigned long)s_baseline_signal2[z],
               (unsigned long)s_baseline_distance2[z],
               (unsigned long)motion);
    }
    printf("\r\n");
}

void VL53L5CX_Sensor2_PrintAllZoneParams(void)
{
    int8_t temp = s_results2.silicon_temp_degc;
    printf("S2,ALLPARAM,%d,", temp);

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint32_t cur_sig  = s_results2.signal_per_spad[idx];
        int16_t  cur_dist = s_results2.distance_mm[idx];
        uint8_t  cur_stat = s_results2.target_status[idx];

        uint32_t base_sig  = s_baseline_signal2[z];
        uint16_t base_dist = s_baseline_distance2[z];
        uint32_t drop_pct = 0;
        if (base_sig > 0) {
            int32_t diff = (int32_t)base_sig - (int32_t)cur_sig;
            if (diff < 0) diff = -diff;
            drop_pct = (uint32_t)diff * 100 / base_sig;
        }

        if (z > 0) printf(",");
        printf("%lu,%lu,%d,%d,%lu,%lu,%lu",
               (unsigned long)cur_sig,
               (unsigned long)base_sig,
               cur_dist,
               base_dist,
               (unsigned long)cur_stat,
               (unsigned long)drop_pct,
               (unsigned long)(s_zone_valid2[z] ? 1 : 0));
    }
    printf("\r\n");
}

#endif /* VL53L5CX_DUAL_SENSOR */
