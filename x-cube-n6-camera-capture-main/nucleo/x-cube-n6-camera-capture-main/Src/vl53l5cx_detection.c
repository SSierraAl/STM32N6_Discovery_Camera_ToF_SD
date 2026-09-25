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
#include <stdint.h>
#include <string.h>
#include "vl53l5cx_detection.h"
/* The source wrapper leaves printf intact; baseline and ToF diagnostics
   remain visible after a clean rebuild. */
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

#if !VL53L5CX_DUAL_SENSOR && (VL53L5CX_DET_RESOLUTION == 4) && \
    ((TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_TEST) || \
     (!TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_CAMERA))
/* Keep a detected zone latched while the same signal offset persists.
   The short edge test still accepts another object in that zone immediately.
   Zone-specific recalibration happens only after a long stationary plateau. */
#define TOF_TEST_CLEAR_FRAMES       3U
#define TOF_TEST_QUIET_FRAMES      15U
#define TOF_TEST_QUIET_SIGNAL_PCT   4U
#define TOF_TEST_SETTLE_MS          12000U
#define TOF_TEST_STABLE_MIN_FRAMES      8U
/* Recentering counts stable frames inside this window instead of requiring
   TOF_TEST_SETTLE_MS of unbroken stability: a static plateau with normal
   1 %/1 mm sensor jitter (field: zones 10, 8) still reaches the count,
   while a moving insect breaks the per-frame gate. The window must stay
   below any re-arm interval or a biased zone re-photographs forever. */
#define TOF_TEST_STABLE_WINDOW_FRAMES 180U
#define TOF_TEST_SCENE_SETTLE_MS     5000U
static uint16_t s_test_latched = 0U;
/* A weak capture may escalate once to a strong capture in the same zone.
   Strong captures remain one-per-evidence-episode. */
static uint16_t s_test_latched_strong = 0U;
static uint16_t s_test_strong_escalation_pending = 0U;
static uint32_t s_test_latched_since[16] = {0};
static uint8_t s_test_clear[16] = {0};
static uint32_t s_test_prev_sig[16] = {0};
static uint16_t s_test_prev_dist[16] = {0};
static uint8_t s_test_prev_valid[16] = {0};
static uint32_t s_test_stable_since[16] = {0};
static uint32_t s_test_stable_sig[16] = {0};
static uint32_t s_test_stable_dist[16] = {0};
static uint8_t s_test_stable_frames[16] = {0};
static uint32_t s_test_scene_since = 0U;
static uint8_t s_test_refresh_requested = 0U;
static uint16_t s_test_track_mask = 0U;
static uint16_t s_test_track_signal_mask = 0U;
static uint16_t s_test_track_distance_mask = 0U;
static uint32_t s_test_track_signal_value[16] = {0};
static uint32_t s_test_track_distance_value[16] = {0};
static uint32_t s_test_track_since = 0U;
static uint16_t s_test_weak_active_mask = 0U;
static uint8_t s_test_recent_motion[16] = {0};
static uint16_t s_test_fast_pending_signal = 0U;
static uint16_t s_test_fast_pending_distance = 0U;
static uint16_t s_test_floor_pending_mask = 0U;
static uint8_t s_test_floor_hold_frames[16] = {0};
static uint8_t s_test_floor_signal_score[16] = {0};
static uint16_t s_test_blocked_mask = 0U;
static uint8_t s_test_micro_score[16] = {0};
static uint8_t s_test_last_event_class = 0U;

static uint32_t TestAbsDiff(uint32_t a, uint32_t b)
{
    return a >= b ? a - b : b - a;
}

static uint32_t TestAbsI32(int32_t value)
{
    return (uint32_t)(value < 0 ? -(int64_t)value : value);
}

static int32_t TestMedianI32(const int32_t *values, uint8_t count)
{
    int32_t sorted[16];
    if (count == 0U) return 0;
    for (uint8_t i = 0U; i < count; i++) sorted[i] = values[i];
    for (uint8_t i = 1U; i < count; i++) {
        int32_t key = sorted[i];
        int j = (int)i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    if (count & 1U) return sorted[count / 2U];
    return (int32_t)(((int64_t)sorted[count / 2U - 1U] +
                      (int64_t)sorted[count / 2U]) / 2LL);
}

static uint8_t TestCountBits16(uint16_t mask)
{
    uint8_t count = 0U;
    while (mask) {
        count += (uint8_t)(mask & 1U);
        mask >>= 1;
    }
    return count;
}

static uint8_t TestHasAdjacentPair(uint16_t mask)
{
    for (uint8_t a = 0U; a < 16U; a++) {
        if (!(mask & (uint16_t)(1U << a))) continue;
        const int ar = a / 4;
        const int ac = a % 4;
        for (uint8_t b = (uint8_t)(a + 1U); b < 16U; b++) {
            if (!(mask & (uint16_t)(1U << b))) continue;
            int dr = ar - (int)(b / 4U);
            int dc = ac - (int)(b % 4U);
            if (dr < 0) dr = -dr;
            if (dc < 0) dc = -dc;
            if (dr <= 1 && dc <= 1) return 1U;
        }
    }
    return 0U;
}

static void TestResetLocalTrack(void)
{
    s_test_track_mask = 0U;
    s_test_track_signal_mask = 0U;
    s_test_track_distance_mask = 0U;
    memset(s_test_track_signal_value, 0, sizeof(s_test_track_signal_value));
    memset(s_test_track_distance_value, 0, sizeof(s_test_track_distance_value));
    s_test_track_since = 0U;
}

static void TestResetDetectionState(void)
{
    s_test_latched = 0U;
    s_test_latched_strong = 0U;
    s_test_strong_escalation_pending = 0U;
    memset(s_test_latched_since, 0, sizeof(s_test_latched_since));
    memset(s_test_clear, 0, sizeof(s_test_clear));
    memset(s_test_prev_sig, 0, sizeof(s_test_prev_sig));
    memset(s_test_prev_dist, 0, sizeof(s_test_prev_dist));
    memset(s_test_prev_valid, 0, sizeof(s_test_prev_valid));
    memset(s_test_stable_since, 0, sizeof(s_test_stable_since));
    memset(s_test_stable_sig, 0, sizeof(s_test_stable_sig));
    memset(s_test_stable_dist, 0, sizeof(s_test_stable_dist));
    memset(s_test_stable_frames, 0, sizeof(s_test_stable_frames));
    s_test_scene_since = 0U;
    s_test_refresh_requested = 0U;
    s_test_weak_active_mask = 0U;
    memset(s_test_recent_motion, 0, sizeof(s_test_recent_motion));
    s_test_fast_pending_signal = 0U;
    s_test_fast_pending_distance = 0U;
    s_test_floor_pending_mask = 0U;
    memset(s_test_floor_hold_frames, 0, sizeof(s_test_floor_hold_frames));
    memset(s_test_floor_signal_score, 0, sizeof(s_test_floor_signal_score));
    memset(s_test_micro_score, 0, sizeof(s_test_micro_score));
    s_test_blocked_mask = 0U;
    s_test_last_event_class = 0U;
    TestResetLocalTrack();
}

void VL53L5CX_ZoneDetectorAfterCapture(void)
{
    /* The sensor task does not read frames while camera/SD work blocks it.
       The first subsequent frame must not be compared against that stale
       pre-capture frame. Preserve latches so the same insect does not cause
       another capture solely because the camera finished. */
    memset(s_test_prev_valid, 0, sizeof(s_test_prev_valid));
    memset(s_test_stable_since, 0, sizeof(s_test_stable_since));
    memset(s_test_stable_frames, 0, sizeof(s_test_stable_frames));
    s_test_fast_pending_signal = 0U;
    s_test_fast_pending_distance = 0U;
    s_test_floor_pending_mask = 0U;
    memset(s_test_floor_hold_frames, 0, sizeof(s_test_floor_hold_frames));
    memset(s_test_floor_signal_score, 0, sizeof(s_test_floor_signal_score));
    memset(s_test_micro_score, 0, sizeof(s_test_micro_score));
    s_test_scene_since = 0U;
    TestResetLocalTrack();
}
#endif

#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && (VL53L5CX_DET_RESOLUTION == 4)
static uint32_t s_last_frame_tick = 0U;
#endif

/* Observation-only diagnostics for the vibration/noise study.
   The source wrapper selects the mode-specific setting; keep the fallback
   when this file is built without that wrapper. */
#ifndef VL53L5CX_DET_DEBUG_NOISE_METRICS
#define VL53L5CX_DET_DEBUG_NOISE_METRICS  1
#endif

#if VL53L5CX_DET_DEBUG_NOISE_METRICS > 0
static uint32_t VL53L5CX_AbsDiffI32(int32_t a, int32_t b)
{
    int64_t d = (int64_t)a - (int64_t)b;
    if (d < 0) d = -d;
    return (uint32_t)d;
}

static uint32_t VL53L5CX_AbsDiffU32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static int32_t VL53L5CX_MedianI32(const int32_t *values, uint8_t count)
{
    if (count == 0) return 0;

    int32_t sorted[VL53L5CX_DET_NUM_ZONES];
    for (uint8_t i = 0; i < count; i++) sorted[i] = values[i];

    for (uint8_t i = 1; i < count; i++) {
        int32_t key = sorted[i];
        int j = (int)i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    if (count & 1U) return sorted[count / 2U];
    return (int32_t)(((int64_t)sorted[count / 2U - 1U] +
                      (int64_t)sorted[count / 2U]) / 2LL);
}

static uint32_t VL53L5CX_MedianU32(const uint32_t *values, uint8_t count)
{
    if (count == 0) return 0;

    uint32_t sorted[VL53L5CX_DET_NUM_ZONES];
    for (uint8_t i = 0; i < count; i++) sorted[i] = values[i];

    for (uint8_t i = 1; i < count; i++) {
        uint32_t key = sorted[i];
        int j = (int)i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    if (count & 1U) return sorted[count / 2U];
    return (uint32_t)(((uint64_t)sorted[count / 2U - 1U] +
                       (uint64_t)sorted[count / 2U]) / 2ULL);
}

static uint8_t VL53L5CX_CountBits16(uint16_t mask)
{
    uint8_t count = 0;
    while (mask) {
        count += (uint8_t)(mask & 1U);
        mask >>= 1;
    }
    return count;
}
#endif

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
#if VL53L5CX_DUAL_SENSOR
    /* Preserve the validated dual-sensor startup path exactly. The current
       dual implementation still uses the scan result in its startup gate.
       In single-sensor mode the scan was diagnostic-only, so skipping it
       avoids probing unrelated/reserved addresses on shared I2C1 (camera,
       ToF and RTC) without changing ToF power-up or detection behavior. */
    int i2cdevices = VL53L5CX_ScanI2CBus();
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
    uint8_t active_target_order = 0U;
    vl53l5cx_set_resolution(&s_dev, resolution);
    vl53l5cx_set_integration_time_ms(&s_dev, integration_ms);
    vl53l5cx_set_ranging_frequency_hz(&s_dev, freq_hz);
    int target_st = vl53l5cx_set_target_order(&s_dev,
                                               VL53L5CX_DET_TARGET_ORDER);
    vl53l5cx_set_sharpener_percent(&s_dev, 10);
    /* VL53L5CX_RANGING_MODE (app_config.h): 1 = CONTINUOUS (integration
       forced to sensor maximum), 3 = AUTONOMOUS (precise integration). */
    vl53l5cx_set_ranging_mode(&s_dev, VL53L5CX_RANGING_MODE);

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    int motion_st = vl53l5cx_motion_indicator_init(&s_dev, &s_motion_config, resolution);
    if (motion_st) {
        printf("[ToF] WARN: Motion indicator init failed: %d\n", motion_st);
        s_motion_initialized = 0;
    } else {
        s_motion_config.min_nb_for_global_detection = VL53L5CX_DET_MOTION_MIN_ZONES;
        s_motion_config.nb_of_temporal_accumulations = VL53L5CX_DET_MOTION_PERSIST_FRAMES;
        s_motion_config.extra_noise_sigma = VL53L5CX_DET_MOTION_EXTRA_NOISE;
        /* The plugin only writes its configuration to the sensor during
           init/set_resolution, so re-apply it here — otherwise the defines
           above would have no effect on the sensor. */
        motion_st = vl53l5cx_motion_indicator_set_resolution(&s_dev, &s_motion_config, resolution);
        if (motion_st) {
            printf("[ToF] WARN: Motion indicator re-apply failed: %d (plugin defaults remain)\n",
                   motion_st);
        }
        s_motion_initialized = 1;
        printf("[ToF] Motion indicator enabled (global_zones=%d, accum=%d, noise_sigma=%d)\n",
               VL53L5CX_DET_MOTION_MIN_ZONES, VL53L5CX_DET_MOTION_PERSIST_FRAMES,
               VL53L5CX_DET_MOTION_EXTRA_NOISE);
    }
#else
    s_motion_initialized = 0;
#endif

    target_st |= vl53l5cx_get_target_order(&s_dev, &active_target_order);
    printf("[ToF] Configured: res=%d, int=%dms, freq=%dHz, mode=%d, target=%s(%u), target_st=%d\n",
           resolution, integration_ms, freq_hz, (int)VL53L5CX_RANGING_MODE,
           active_target_order == VL53L5CX_TARGET_ORDER_CLOSEST ?
               "CLOSEST" : (active_target_order == VL53L5CX_TARGET_ORDER_STRONGEST ?
                   "STRONGEST" : "UNKNOWN"),
           (unsigned)active_target_order, target_st);
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
#if !VL53L5CX_DUAL_SENSOR && (VL53L5CX_DET_RESOLUTION == 4) && \
    ((TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_TEST) || \
     (!TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_CAMERA))
    TestResetDetectionState();
#endif
    printf("[ToF] Baseline reset\n");
}

void VL53L5CX_LearnBaseline(void)
{
    VL53L5CX_ResetBaseline();
    const uint8_t baseline_samples = VL53L5CX_DET_BASELINE_SAMPLES;
    const uint8_t settle_frames = VL53L5CX_DET_BASELINE_SETTLE_FRAMES;
    const uint16_t min_valid_frames = baseline_samples / 2;

    /* Per-zone accumulation with per-zone OK-frame counts.
       Right after power-on some zones only have OK status on PART of the
       samples (the sensor's internal baseline engine is still converging).
       Dividing by the TOTAL sample count (previous behavior) biased those
       baselines low; since the drop test is |base-cur|/base, a 50%-low
       baseline reads as a ~100% drop -> fake detection on EVERY frame.
       A zone is only valid if it was OK on at least half the samples. */
    uint32_t sum_signal[VL53L5CX_DET_NUM_ZONES] = {0};
    uint32_t sum_distance[VL53L5CX_DET_NUM_ZONES] = {0};
    uint16_t ok_frames[VL53L5CX_DET_NUM_ZONES] = {0};
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && (VL53L5CX_DET_RESOLUTION == 4)
    int16_t min_distance[VL53L5CX_DET_NUM_ZONES];
    int16_t max_distance[VL53L5CX_DET_NUM_ZONES] = {0};
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++)
        min_distance[z] = INT16_MAX;
#endif

    printf("[BASELINE] Learning %d samples + %d settle frames...\n",
           baseline_samples, settle_frames);

    /* Let the ranging engine converge before measuring the reference. This
       used to run after accumulation, which made every restart vulnerable to
       the 2-10 %% warm-up offsets observed in field logs. */
    for (uint8_t i = 0; i < settle_frames; i++) {
        if (!VL53L5CX_WaitForDataReady(1000)) continue;
        if (VL53L5CX_GetData() != 0) continue;
#if !(TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY)
        printf("  [SETTLE %d/%d]\r", i + 1, settle_frames);
#endif
    }

    for (uint8_t i = 0; i < baseline_samples; i++) {
        if (!VL53L5CX_WaitForDataReady(1000)) continue;
        if (VL53L5CX_GetData() != 0) continue;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
            if (VL53L5CX_STATUS_OK_FILT(s_results.target_status[idx])) {
                if (s_results.signal_per_spad[idx] < VL53L5CX_DET_MIN_SIGNAL) {
                    continue;
                }
                sum_signal[z]   += s_results.signal_per_spad[idx];
                sum_distance[z] += s_results.distance_mm[idx];
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && (VL53L5CX_DET_RESOLUTION == 4)
                if (s_results.distance_mm[idx] < min_distance[z])
                    min_distance[z] = s_results.distance_mm[idx];
                if (s_results.distance_mm[idx] > max_distance[z])
                    max_distance[z] = s_results.distance_mm[idx];
#endif
                ok_frames[z]++;
            }
        }
#if !(TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY)
        printf("  [BASELINE %d/%d]\r", i + 1, baseline_samples);
#endif
    }

    uint8_t valid_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (ok_frames[z] >= min_valid_frames) {
            s_baseline_signal[z]   = sum_signal[z] / ok_frames[z];
            s_baseline_distance[z] = (uint16_t)(sum_distance[z] / ok_frames[z]);
            s_zone_valid[z] = 1;
            valid_count++;
        }
    }

    s_baseline_ready = 1;
    /* Baseline samples and settle frames must never remain in either temporal
       detector history. The next live frames will prime fresh history. */
    VL53L5CX_ResetDetectionFilterState();
    printf("\n[BASELINE] Done. Valid zones: %d/%d\n", valid_count, VL53L5CX_DET_NUM_ZONES);
    VL53L5CX_PrintBaselineFrame();
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && (VL53L5CX_DET_RESOLUTION == 4)
    /* Baseline is already per zone. Sample count and observed distance span
       identify unreliable/mixed zones before a floor mask is chosen. */
    printf("TOFBASE,t=%lu,format=z:valid:n:distance_mm:signal:span_mm",
           (unsigned long)HAL_GetTick());
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        int span = ok_frames[z] ? (int)max_distance[z] - (int)min_distance[z] : 0;
        printf(",%u:%u:%u:%u:%lu:%u", (unsigned)z, (unsigned)s_zone_valid[z],
               (unsigned)ok_frames[z], (unsigned)s_baseline_distance[z],
               (unsigned long)s_baseline_signal[z], (unsigned)span);
    }
    printf("\r\n");
#endif
}

/* ================================================================
   Detection
   ================================================================ */

int VL53L5CX_Update(void)
{
    if (!VL53L5CX_WaitForDataReady(1000)) return 0;
    if (VL53L5CX_GetData() != 0) return 0;
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && (VL53L5CX_DET_RESOLUTION == 4)
    s_last_frame_tick = HAL_GetTick();
#endif

    s_last_insect_detected = 0;
    s_last_result.insect_detected = 0;
    s_last_result.trigger_source = 0;
    s_last_result.affected_count = 0;
    s_last_result.valid_measurements = 0;

    uint8_t frame_trig_signal = 0;
    uint8_t frame_trig_motion = 0;

#if VL53L5CX_DET_DEBUG_NOISE_METRICS > 0
    /* Observation-only arrays. They never feed back into the detector. */
    int32_t noise_dist_delta[VL53L5CX_DET_NUM_ZONES] = {0};
    int32_t noise_signal_delta_pct[VL53L5CX_DET_NUM_ZONES] = {0};
    uint32_t noise_motion[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t noise_dist_valid[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t noise_signal_valid[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t noise_motion_valid[VL53L5CX_DET_NUM_ZONES] = {0};
#endif

    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {


        // IT IS POSSIBLE TO DELETE THE FOLLOWING BLOCK OF CODE IF YOU WANT TO TRIGGER ON ALL ZONES, NOT JUST THE MIDDLE ROWS
        //Test to explore the trigger based only on the middle rows
        /*const int zone_row = z / VL53L5CX_DET_RESOLUTION;
        const int center_row_1 = (VL53L5CX_DET_RESOLUTION / 2) - 1;
        const int center_row_2 =  VL53L5CX_DET_RESOLUTION / 2;

        if ((zone_row != center_row_1) &&
            (zone_row != center_row_2)) {
            continue;
        }*/
        // End of snippet

        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint8_t status = s_results.target_status[idx];

        int signal_triggered = 0;
        uint32_t signal_drop = 0;

#if VL53L5CX_DET_DEBUG_NOISE_METRICS > 0
        /* Signed distance/signal deltas are collected from trustworthy
           baseline zones. This is diagnostic only; the existing detector
           below continues to use its original absolute signal-drop logic. */
        if (s_zone_valid[z] && VL53L5CX_STATUS_OK_FILT(status)) {
            uint16_t cur_dist = s_results.distance_mm[idx];
            uint32_t cur_sig = s_results.signal_per_spad[idx];

            if (s_baseline_distance[z] > 0 && cur_dist > 0) {
                noise_dist_delta[z] = (int32_t)cur_dist - (int32_t)s_baseline_distance[z];
                noise_dist_valid[z] = 1;
            }

            if (s_baseline_signal[z] > 0 &&
                cur_sig != 0 && cur_sig >= VL53L5CX_DET_MIN_SIGNAL) {
                int64_t numerator = ((int64_t)cur_sig - (int64_t)s_baseline_signal[z]) * 100LL;
                noise_signal_delta_pct[z] = (int32_t)(numerator / (int64_t)s_baseline_signal[z]);
                noise_signal_valid[z] = 1;
            }
        }
#endif

        /* SIGNAL: gates (zone_valid, VL53L5CX_STATUS_OK = 5/6/9,
           signal present, >= MIN, baseline drop > THRESH_PCT). */
        if (s_zone_valid[z] && VL53L5CX_STATUS_OK_FILT(status)) {
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
#if VL53L5CX_DET_DEBUG_NOISE_METRICS > 0
            noise_motion[z] = motion_val;
            noise_motion_valid[z] = 1;
#endif
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

#if VL53L5CX_DET_DEBUG_NOISE_METRICS > 0
    /* Stage-1 observation mode: characterize common motion and local residuals
       only when the EXISTING detector already decided to trigger. Nothing in
       this block can clear or set s_last_insect_detected. */
    if (s_last_insect_detected) {
        int32_t dist_values[VL53L5CX_DET_NUM_ZONES];
        int32_t sig_values[VL53L5CX_DET_NUM_ZONES];
        uint32_t motion_values[VL53L5CX_DET_NUM_ZONES];
        uint8_t dist_count = 0, sig_count = 0, motion_count = 0;
        uint8_t motion_coverage = 0;

        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            if (noise_dist_valid[z]) dist_values[dist_count++] = noise_dist_delta[z];
            if (noise_signal_valid[z]) sig_values[sig_count++] = noise_signal_delta_pct[z];
            if (noise_motion_valid[z]) {
                motion_values[motion_count++] = noise_motion[z];
                if (noise_motion[z] >= VL53L5CX_DET_MOTION_THRESH) motion_coverage++;
            }
        }

        int32_t global_dist = VL53L5CX_MedianI32(dist_values, dist_count);
        int32_t global_sig = VL53L5CX_MedianI32(sig_values, sig_count);
        uint32_t global_motion = VL53L5CX_MedianU32(motion_values, motion_count);

        uint32_t dist_dev[VL53L5CX_DET_NUM_ZONES];
        uint32_t sig_dev[VL53L5CX_DET_NUM_ZONES];
        uint32_t motion_dev[VL53L5CX_DET_NUM_ZONES];
        for (uint8_t i = 0; i < dist_count; i++)
            dist_dev[i] = VL53L5CX_AbsDiffI32(dist_values[i], global_dist);
        for (uint8_t i = 0; i < sig_count; i++)
            sig_dev[i] = VL53L5CX_AbsDiffI32(sig_values[i], global_sig);
        for (uint8_t i = 0; i < motion_count; i++)
            motion_dev[i] = VL53L5CX_AbsDiffU32(motion_values[i], global_motion);

        uint32_t mad_dist = VL53L5CX_MedianU32(dist_dev, dist_count);
        uint32_t mad_sig = VL53L5CX_MedianU32(sig_dev, sig_count);
        uint32_t mad_motion = VL53L5CX_MedianU32(motion_dev, motion_count);

        /* Coherence is diagnostic only. 3*MAD scales with the observed frame
           dispersion; small floors avoid a zero-width band on very quiet data. */
        uint32_t dist_tol = mad_dist * 3U;
        uint32_t sig_tol = mad_sig * 3U;
        if (dist_tol < 2U) dist_tol = 2U;
        if (sig_tol < 1U) sig_tol = 1U;

        uint8_t coherent_dist = 0, coherent_sig = 0;
        for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
            if (noise_dist_valid[z] &&
                VL53L5CX_AbsDiffI32(noise_dist_delta[z], global_dist) <= dist_tol)
                coherent_dist++;
            if (noise_signal_valid[z] &&
                VL53L5CX_AbsDiffI32(noise_signal_delta_pct[z], global_sig) <= sig_tol)
                coherent_sig++;
        }

        uint16_t row_mask = 0;
        uint16_t col_mask = 0;
        uint32_t max_local_dist = 0;
        uint32_t max_local_sig = 0;

        for (uint8_t i = 0; i < s_last_result.affected_count; i++) {
            uint8_t z = s_last_result.affected_zones[i];
            uint8_t row = (uint8_t)(z / VL53L5CX_DET_RESOLUTION);
            uint8_t col = (uint8_t)(z % VL53L5CX_DET_RESOLUTION);
            if (row < 16U) row_mask |= (uint16_t)(1U << row);
            if (col < 16U) col_mask |= (uint16_t)(1U << col);

            if (noise_dist_valid[z]) {
                uint32_t residual = VL53L5CX_AbsDiffI32(noise_dist_delta[z], global_dist);
                if (residual > max_local_dist) max_local_dist = residual;
            }
            if (noise_signal_valid[z]) {
                uint32_t residual = VL53L5CX_AbsDiffI32(noise_signal_delta_pct[z], global_sig);
                if (residual > max_local_sig) max_local_sig = residual;
            }
        }

        printf("NOISEMETRIC,temp=%d,trig=%u,affected=%u,validD=%u,Gd=%ld,MADd=%lu,cohD=%u/%u,validS=%u,Gs=%ld,MADs=%lu,cohS=%u/%u,validM=%u,Gm=%lu,MADm=%lu,motionCov=%u/%u,rows=%u,cols=%u,maxRd=%lu,maxRs=%lu\r\n",
               (int)s_results.silicon_temp_degc,
               (unsigned)s_last_result.trigger_source,
               (unsigned)s_last_result.affected_count,
               (unsigned)dist_count,
               (long)global_dist,
               (unsigned long)mad_dist,
               (unsigned)coherent_dist,
               (unsigned)dist_count,
               (unsigned)sig_count,
               (long)global_sig,
               (unsigned long)mad_sig,
               (unsigned)coherent_sig,
               (unsigned)sig_count,
               (unsigned)motion_count,
               (unsigned long)global_motion,
               (unsigned long)mad_motion,
               (unsigned)motion_coverage,
               (unsigned)motion_count,
               (unsigned)VL53L5CX_CountBits16(row_mask),
               (unsigned)VL53L5CX_CountBits16(col_mask),
               (unsigned long)max_local_dist,
               (unsigned long)max_local_sig);
    }
#endif

    /* BASELINE REFRESH */
#if VL53L5CX_DET_PERIODIC_RESTART_ENABLED > 0
    {
        static uint32_t frame_counter = 0;
        frame_counter++;
        if (frame_counter >= VL53L5CX_DET_PERIODIC_RESTART_INTERVAL) {
            /* Quiet gate: never re-learn while an insect event or a
               latched/blocked candidate zone is active, or the animal gets
               baked into the baseline. The counter is not reset on skip,
               so the refresh retries on the very next frame. */
            uint8_t tof_busy = (uint8_t)(s_last_insect_detected != 0);
#if !VL53L5CX_DUAL_SENSOR && (VL53L5CX_DET_RESOLUTION == 4) && \
    ((TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_TEST) || \
     (!TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_CAMERA))
            tof_busy |= (s_test_latched != 0U) || (s_test_blocked_mask != 0U);
#endif
            if (!tof_busy) {
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
    }
#endif

    /* Debug output */
#if VL53L5CX_DET_DEBUG_ZFRAME > 0
    static uint32_t zframe_counter = 0;
    zframe_counter++;
    if (zframe_counter >= VL53L5CX_DET_DEBUG_ZFRAME_INT) {
        zframe_counter = 0;
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

#if !VL53L5CX_DUAL_SENSOR && (VL53L5CX_DET_RESOLUTION == 4) && \
    ((TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_TEST) || \
     (!TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_CAMERA))
int VL53L5CX_TestDetectionStep(uint8_t event_policy)
{
    const uint32_t now = HAL_GetTick();
    uint16_t signal_mask = 0U;
    uint16_t strong_distance_mask = 0U;
    uint16_t fast_signal_mask = 0U;
    uint16_t fast_distance_mask = 0U;
    uint16_t fast_signal_candidate_mask = 0U;
    uint16_t fast_distance_candidate_mask = 0U;
    uint16_t fast_distance_support_mask = 0U;
    uint16_t fast_edge_mask = 0U;
    uint16_t weak_signal_mask = 0U;
    uint16_t weak_distance_mask = 0U;
    uint16_t weak_motion_mask = 0U;
    uint16_t floor_mask = 0U;
    uint16_t floor_protrusion_mask = 0U;
    uint16_t floor_hold_mask = 0U;
    uint16_t floor_signal_hold_mask = 0U;
#if VL53L5CX_DET_MICRO_PERSIST_ENABLED
    uint16_t micro_evidence_mask = 0U;
    uint16_t micro_hold_mask = 0U;
#endif
    uint16_t level_event_mask = 0U;
    uint16_t candidate_event_mask = 0U;
    uint16_t event_mask = 0U;
    uint16_t event_signal_mask = 0U;
    uint16_t event_distance_mask = 0U;
    uint16_t track_event_mask = 0U;
    uint16_t raw_mask = 0U;
    uint8_t new_evidence = 0U;
    uint8_t distance_up = 0U, distance_down = 0U;
    uint8_t signal_up = 0U, signal_down = 0U;
    uint8_t motion_active = 0U;
    VL53L5CX_DetectionResult_t res = VL53L5CX_GetResult();
    int32_t distance_delta[16] = {0};
    int32_t signal_delta_pct[16] = {0};
    int32_t distance_values[16];
    int32_t signal_values[16];
    uint32_t local_distance[16] = {0};
    int32_t signed_local_distance[16] = {0};
    uint32_t local_signal[16] = {0};
    int32_t signed_local_signal[16] = {0};
    uint32_t frame_distance[16] = {0};
    uint32_t frame_signal[16] = {0};
    uint8_t local_valid[16] = {0};
    uint8_t distance_count = 0U;
    uint8_t signal_count = 0U;
    uint16_t farthest_baseline_distance = 0U;

    /* Build robust common-mode references from this same frame. A vibration
       or slow scene shift moves most zones together; a small insect changes
       only one or a few zones and therefore remains after median subtraction. */
    for (uint8_t z = 0U; z < 16U; z++) {
        const uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        const uint32_t signal = s_results.signal_per_spad[idx];
        const int16_t distance = s_results.distance_mm[idx];
        const uint32_t base = s_baseline_signal[z];
        const uint16_t base_dist = s_baseline_distance[z];
        if (s_zone_valid[z] && base > 0U && base_dist > farthest_baseline_distance)
            farthest_baseline_distance = base_dist;
        const uint8_t valid = (uint8_t)(s_zone_valid[z] &&
            VL53L5CX_STATUS_OK_FILT(s_results.target_status[idx]) &&
            signal >= VL53L5CX_DET_MIN_SIGNAL && distance > 0 &&
            base > 0U && base_dist > 0U);
        if (!valid) continue;

        local_valid[z] = 1U;
        distance_delta[z] = (int32_t)distance - (int32_t)base_dist;
        signal_delta_pct[z] = (int32_t)((((int64_t)signal - (int64_t)base) *
                                          100LL) / (int64_t)base);
        distance_values[distance_count++] = distance_delta[z];
        signal_values[signal_count++] = signal_delta_pct[z];
    }

    const int32_t common_distance = TestMedianI32(distance_values, distance_count);
    const int32_t common_signal = TestMedianI32(signal_values, signal_count);

    for (uint8_t z = 0U; z < 16U; z++) {
        if (s_test_recent_motion[z] > 0U) s_test_recent_motion[z]--;
        if (!local_valid[z]) {
            s_test_floor_hold_frames[z] = 0U;
            s_test_floor_signal_score[z] = 0U;
            continue;
        }
        const uint16_t bit = (uint16_t)(1U << z);
        const uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        const uint32_t signal = s_results.signal_per_spad[idx];
        const uint16_t distance = (uint16_t)s_results.distance_mm[idx];
        signed_local_distance[z] = distance_delta[z] - common_distance;
        local_distance[z] = TestAbsI32(signed_local_distance[z]);
        signed_local_signal[z] = signal_delta_pct[z] - common_signal;
        local_signal[z] = TestAbsI32(signed_local_signal[z]);
        if ((uint32_t)s_baseline_distance[z] +
                VL53L5CX_DET_FLOOR_DEPTH_BAND_MM >=
            (uint32_t)farthest_baseline_distance) {
            floor_mask |= bit;
        }

        if (s_test_prev_valid[z]) {
            frame_signal[z] = (s_test_prev_sig[z] > 0U) ?
                (uint32_t)(((uint64_t)TestAbsDiff(signal, s_test_prev_sig[z]) * 100U) /
                           s_test_prev_sig[z]) : 0U;
            frame_distance[z] = TestAbsDiff((uint32_t)distance,
                                             (uint32_t)s_test_prev_dist[z]);
            if (frame_signal[z] >= VL53L5CX_DET_WEAK_MOTION_SIGNAL_PCT ||
                frame_distance[z] >= VL53L5CX_DET_WEAK_MOTION_DISTANCE_MM) {
                s_test_recent_motion[z] = VL53L5CX_DET_WEAK_MOTION_MEMORY_FRAMES;
            }
        }
        if (s_test_recent_motion[z] > 0U) weak_motion_mask |= bit;

        /* Preserve the tested single-zone signal sensitivity, but apply it
           to the local residual instead of a trap-wide common change. */
        if (local_signal[z] >= VL53L5CX_DET_THRESHOLD_PCT)
            signal_mask |= bit;
        else if (local_signal[z] >= VL53L5CX_DET_LOCAL_SIGNAL_WEAK_PCT)
            weak_signal_mask |= bit;

        if (local_distance[z] >= VL53L5CX_DET_LOCAL_DIST_STRONG_MM)
            strong_distance_mask |= bit;
        else if (local_distance[z] >= VL53L5CX_DET_LOCAL_DIST_WEAK_MM)
            weak_distance_mask |= bit;
        if (local_distance[z] >= VL53L5CX_DET_FAST_BASELINE_DISTANCE_MM)
            fast_distance_support_mask |= bit;

        /* An insect on the calibrated floor shortens the distance to the
           opposite-mounted sensor. Keep this direction; absolute deltas alone
           cannot separate a slow protrusion from arbitrary vibration. */
        if ((floor_mask & bit) && (weak_signal_mask & bit) &&
            signed_local_distance[z] <=
                -(int32_t)VL53L5CX_DET_FLOOR_PROTRUSION_MIN_MM)
            floor_protrusion_mask |= bit;

        if (floor_protrusion_mask & bit) {
            if (s_test_floor_hold_frames[z] < VL53L5CX_DET_FLOOR_HOLD_FRAMES)
                s_test_floor_hold_frames[z]++;
            if (s_test_floor_hold_frames[z] >= VL53L5CX_DET_FLOOR_HOLD_FRAMES)
                floor_hold_mask |= bit;
        } else {
            s_test_floor_hold_frames[z] = 0U;
        }

        /* The 500 ms trace can miss short 2% peaks that the 15 Hz detector
           sees. A score already saturated in a policy-blocked frame remains
           eligible for this frame before its normal quiet-frame decay. */
        if (s_test_floor_signal_score[z] >=
            VL53L5CX_DET_FLOOR_SIGNAL_SCORE_TRIGGER)
            floor_signal_hold_mask |= bit;
        if ((floor_mask & bit) && (weak_signal_mask & bit)) {
            const uint8_t room = (uint8_t)(
                VL53L5CX_DET_FLOOR_SIGNAL_SCORE_TRIGGER -
                s_test_floor_signal_score[z]);
            if (room <= VL53L5CX_DET_FLOOR_SIGNAL_SCORE_HIT)
                s_test_floor_signal_score[z] =
                    VL53L5CX_DET_FLOOR_SIGNAL_SCORE_TRIGGER;
            else
                s_test_floor_signal_score[z] +=
                    VL53L5CX_DET_FLOOR_SIGNAL_SCORE_HIT;
        } else if (s_test_floor_signal_score[z] > 0U &&
                   (event_policy & VL53L5CX_TEST_EVENT_ALLOW_LEVEL)) {
            s_test_floor_signal_score[z]--;
        }
        if (s_test_floor_signal_score[z] >=
            VL53L5CX_DET_FLOOR_SIGNAL_SCORE_TRIGGER)
            floor_signal_hold_mask |= bit;

#if VL53L5CX_DET_MICRO_PERSIST_ENABLED
        /* Sub-threshold micro-persistence: a settled insect holds ~1% /
           1-2 mm in one floor zone for seconds, while empty-box noise is a
           1-frame spike that hops zones. Evidence feeds raw_mask (baseline
           protection + latch hold); only the slow same-zone score may fire.
           The score freezes while LEVEL is policy-blocked, like the floor
           score above. */
        if ((floor_mask & bit) &&
            (local_signal[z] >= VL53L5CX_DET_MICRO_SIGNAL_PCT ||
             local_distance[z] >= VL53L5CX_DET_MICRO_DISTANCE_MM)) {
            micro_evidence_mask |= bit;
            const uint8_t room = (uint8_t)(VL53L5CX_DET_MICRO_TRIGGER -
                                           s_test_micro_score[z]);
            if (room <= VL53L5CX_DET_MICRO_HIT)
                s_test_micro_score[z] = VL53L5CX_DET_MICRO_TRIGGER;
            else
                s_test_micro_score[z] = (uint8_t)(s_test_micro_score[z] +
                                                  VL53L5CX_DET_MICRO_HIT);
        } else if (s_test_micro_score[z] > 0U &&
                   (event_policy & VL53L5CX_TEST_EVENT_ALLOW_LEVEL)) {
            s_test_micro_score[z] = (uint8_t)(s_test_micro_score[z] -
                                              VL53L5CX_DET_MICRO_DECAY);
        }
        if (s_test_micro_score[z] >= VL53L5CX_DET_MICRO_TRIGGER)
            micro_hold_mask |= bit;
#endif

        /* Kinematic path: require both a fresh per-zone edge and a local
           baseline residual. Coherent vibration is removed by the common
           median above; slow drift lacks the frame-to-frame edge. */
        if (frame_signal[z] >= VL53L5CX_DET_FAST_EDGE_SIGNAL_PCT &&
            local_signal[z] >= VL53L5CX_DET_FAST_BASELINE_SIGNAL_PCT)
            fast_signal_candidate_mask |= bit;
        if (frame_distance[z] >= VL53L5CX_DET_FAST_EDGE_DISTANCE_MM &&
            local_distance[z] >= VL53L5CX_DET_FAST_BASELINE_DISTANCE_MM)
            fast_distance_candidate_mask |= bit;
    }

    /* The normal level path accepts a 3 %% local signal change in one frame.
       The more sensitive 2 %% fast-signal path must still be present in the
       following frame. Distance residuals of 3-4 mm are indistinguishable
       from the observed ToF noise, so admit them only when a following frame
       retains both >=3 mm distance and >=2 %% same-zone signal evidence. A
       local >=5 mm residual remains strong and immediate. */
    const uint16_t signal_support_mask = (uint16_t)(signal_mask |
                                                     weak_signal_mask);
    weak_distance_mask &= signal_support_mask;
    fast_signal_mask = (uint16_t)(s_test_fast_pending_signal & weak_signal_mask);
    fast_distance_mask = (uint16_t)(s_test_fast_pending_distance &
        fast_distance_support_mask & signal_support_mask &
        ~strong_distance_mask);
    s_test_fast_pending_signal = (uint16_t)(fast_signal_candidate_mask & weak_signal_mask);
    s_test_fast_pending_distance = (uint16_t)(fast_distance_candidate_mask &
                                               ~strong_distance_mask);
    fast_edge_mask = (uint16_t)(fast_signal_mask | fast_distance_mask);
    level_event_mask = (uint16_t)(signal_mask | strong_distance_mask |
                                  floor_hold_mask | floor_signal_hold_mask);
#if VL53L5CX_DET_MICRO_PERSIST_ENABLED
    level_event_mask = (uint16_t)(level_event_mask | micro_hold_mask);
#endif
    event_mask = level_event_mask;
    event_signal_mask = (uint16_t)(signal_mask | floor_hold_mask |
                                   floor_signal_hold_mask);
    event_distance_mask = (uint16_t)(strong_distance_mask | floor_hold_mask);
#if VL53L5CX_DET_MICRO_PERSIST_ENABLED
    event_signal_mask = (uint16_t)(event_signal_mask | micro_hold_mask);
#endif

    /* Weak evidence that is neither persistent nor repeated is not enough for
       a photo by itself. Retain it long enough for a slow tiny insect to cross
       into another zone. Two neighbouring zones (including diagonals), or any
       three distinct zones, form a spatial track. */
    if (s_test_track_since != 0U &&
        (now - s_test_track_since) > VL53L5CX_DET_LOCAL_TRACK_WINDOW_MS) {
        TestResetLocalTrack();
    }
    const uint16_t weak_mask = (uint16_t)((weak_signal_mask |
                                            weak_distance_mask) & floor_mask);
    /* A slow floor protrusion is admitted after two consecutive frames even
       when its frame-to-frame edge is too small for recent_motion. */
    const uint16_t confirmed_floor_mask = (uint16_t)(floor_protrusion_mask &
                                                       s_test_floor_pending_mask);
    s_test_floor_pending_mask = floor_protrusion_mask;
    /* A stationary weak deviation is drift, not a track. Keep a zone active
       only after it has shown a recent edge; clearing the weak level rearms
       that zone for a later insect. */
    s_test_weak_active_mask &= weak_mask;
    if (s_test_latched != 0U) {
        /* After a photo, a stationary insect or vibration can spread weakly
           into neighbouring zones. Those zones are the same uninterrupted
           scene, not a second spatial track. Mark every current weak zone as
           already active and discard the track until the photographed scene
           has genuinely cleared. Independent >=3 %% signal and >=5 mm level
           events remain enabled in every zone. */
        s_test_weak_active_mask |= weak_mask;
        TestResetLocalTrack();
    } else {
        const uint16_t track_admission_mask = (uint16_t)(weak_motion_mask |
                                                          confirmed_floor_mask);
        const uint16_t new_weak_mask = (uint16_t)(weak_mask & track_admission_mask &
                                                   ~s_test_weak_active_mask);
        s_test_weak_active_mask |= new_weak_mask;
        if (new_weak_mask != 0U) {
            if (s_test_track_since == 0U) s_test_track_since = now;
            s_test_track_mask |= new_weak_mask;
            s_test_track_signal_mask |= (uint16_t)(weak_signal_mask & new_weak_mask);
            s_test_track_distance_mask |= (uint16_t)(weak_distance_mask & new_weak_mask);
            for (uint8_t z = 0U; z < 16U; z++) {
                const uint16_t bit = (uint16_t)(1U << z);
                if (!(new_weak_mask & bit)) continue;
                s_test_track_signal_value[z] = local_signal[z];
                s_test_track_distance_value[z] = local_distance[z];
            }
        }
    }
    const uint8_t tracked_zones = TestCountBits16(s_test_track_mask);
    if (VL53L5CX_DET_WEAK_TRACK_ENABLED > 0 &&
        tracked_zones >= VL53L5CX_DET_LOCAL_TRACK_MIN_ZONES &&
        tracked_zones <= VL53L5CX_DET_LOCAL_TRACK_MAX_ZONES &&
        (TestHasAdjacentPair(s_test_track_mask) || tracked_zones >= 3U)) {
        track_event_mask = s_test_track_mask;
    }

    /* Apply independent gates. In recovery, strong levels and fast edges stay
       armed while weak tracks are withheld until the sensor settles. */
    candidate_event_mask = (uint16_t)(event_mask | fast_edge_mask |
                                       track_event_mask);
    const uint16_t level_mask = event_mask;
    const uint16_t level_signal_mask = event_signal_mask;
    const uint16_t level_distance_mask = event_distance_mask;
    event_mask = 0U;
    event_signal_mask = 0U;
    event_distance_mask = 0U;
    if (event_policy & VL53L5CX_TEST_EVENT_ALLOW_LEVEL) {
        event_mask |= level_mask;
        event_signal_mask |= level_signal_mask;
        event_distance_mask |= level_distance_mask;
    }
    if (event_policy & VL53L5CX_TEST_EVENT_ALLOW_FAST) {
        event_mask |= fast_edge_mask;
        event_signal_mask |= fast_signal_mask;
        event_distance_mask |= fast_distance_mask;
    }
    if (event_policy & VL53L5CX_TEST_EVENT_ALLOW_TRACK) {
        event_mask |= track_event_mask;
        event_signal_mask |= s_test_track_signal_mask;
        event_distance_mask |= s_test_track_distance_mask;
    }

    /* A zone normally generates one capture per uninterrupted evidence
       episode. Permit exactly one escalation when a zone photographed from
       weak evidence later becomes independently strong. This recovers a real
       insect entering a noise-latched zone without re-opening the periodic
       strong-plateau loop. Require the escalation in two consecutive frames;
       policy-blocked frames may establish the first one. */
    const uint16_t strong_level_mask = (uint16_t)(signal_mask |
                                                   strong_distance_mask);
    const uint16_t strong_escalation_candidate = (uint16_t)(
        strong_level_mask & s_test_latched &
        (uint16_t)~s_test_latched_strong);
    const uint16_t strong_escalation_mask = (uint16_t)(
        event_mask & strong_escalation_candidate &
        s_test_strong_escalation_pending);
    s_test_strong_escalation_pending = strong_escalation_candidate;
    event_mask = (uint16_t)((event_mask & (uint16_t)~s_test_latched) |
                            strong_escalation_mask);
    event_signal_mask &= event_mask;
    event_distance_mask &= event_mask;

    /* raw_mask keeps the anti-loop/adaptation state aware of all current
       local evidence, while event_mask contains only evidence strong enough
       to request a new photo now. Motion-only candidates are deliberately
       excluded: the ST motion plugin's supported range starts at 400 mm,
       outside this 40-110 mm box geometry. */
    raw_mask = (uint16_t)(signal_mask | strong_distance_mask | fast_edge_mask |
                          weak_signal_mask | weak_distance_mask);
#if VL53L5CX_DET_MICRO_PERSIST_ENABLED
    /* Micro evidence keeps a settling insect out of quiet baseline drift
       and holds its zone latch until the insect leaves. */
    raw_mask = (uint16_t)(raw_mask | micro_evidence_mask);
#endif
    s_test_blocked_mask &= raw_mask;

    for (uint8_t z = 0U; z < 16U; z++) {
        const uint16_t bit = (uint16_t)(1U << z);
        const uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        const uint32_t signal = s_results.signal_per_spad[idx];
        const int16_t distance = s_results.distance_mm[idx];
        const uint32_t base = s_baseline_signal[z];
        const uint16_t base_dist = s_baseline_distance[z];
        const uint8_t valid = (uint8_t)(s_zone_valid[z] &&
            VL53L5CX_STATUS_OK_FILT(s_results.target_status[idx]) &&
            signal >= VL53L5CX_DET_MIN_SIGNAL && distance > 0);
        uint32_t motion = 0U;
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        if (s_motion_initialized)
            motion = s_results.motion_indicator.motion[s_motion_config.map_id[z]];
#endif
        if (motion >= VL53L5CX_DET_MOTION_THRESH) motion_active = 1U;

        /* Clear an old event even if the target becomes temporarily invalid.
           Motion is allowed to trigger on invalid ranging zones. */
#if VL53L5CX_DET_REARM_INTERVAL_MS > 0
        /* Bounded re-arm: release the zone latch once the interval has
           elapsed while the insect's level evidence persists, so it can
           be photographed again. A static object is instead absorbed by
           the 12 s stable-plateau recentering (bounded to 2 photos). */
        if ((s_test_latched & bit) != 0U && s_test_latched_since[z] != 0U &&
            (now - s_test_latched_since[z]) >= VL53L5CX_DET_REARM_INTERVAL_MS) {
            s_test_latched = (uint16_t)(s_test_latched & (uint16_t)~bit);
            s_test_latched_strong &= (uint16_t)~bit;
        }
#endif
        if (!(raw_mask & bit)) {
            if (s_test_clear[z] < TOF_TEST_QUIET_FRAMES) s_test_clear[z]++;
            if (s_test_clear[z] >= TOF_TEST_CLEAR_FRAMES) {
                s_test_latched = (uint16_t)(s_test_latched & (uint16_t)~bit);
                s_test_latched_strong &= (uint16_t)~bit;
                s_test_latched_since[z] = 0U;
            }
        } else {
            s_test_clear[z] = 0U;
        }
        const uint8_t latched = (uint8_t)((s_test_latched & bit) != 0U);

        if (!valid) {
            if (event_mask & bit) new_evidence = 1U;
            s_test_prev_valid[z] = 0U;
            s_test_stable_since[z] = 0U;
            s_test_stable_frames[z] = 0U;
            continue;
        }

        const uint32_t signal_change = (base > 0U) ?
            (uint32_t)(((uint64_t)TestAbsDiff(signal, base) * 100U) / base) : 0U;
        const uint32_t distance_change = TestAbsDiff((uint32_t)distance, base_dist);
        if (base_dist > 0U && distance >= (int32_t)base_dist + 4) distance_up++;
        if (base_dist > 0U && (int32_t)distance + 4 <= base_dist) distance_down++;
        if (base > 0U && signal >= base && signal_change >= 7U) signal_up++;
        if (base > 0U && signal < base && signal_change >= 7U) signal_down++;

        const uint32_t frame_signal_pct = frame_signal[z];
        const uint32_t frame_distance_mm = frame_distance[z];
        if (event_mask & bit) new_evidence = 1U;

        /* A quiet zone follows slow signal drift without changing its floor
           distance. Never learn an unphotographed raw candidate as baseline. */
        if (!latched && !(raw_mask & bit) && s_test_clear[z] >= TOF_TEST_QUIET_FRAMES &&
            signal_change <= TOF_TEST_QUIET_SIGNAL_PCT &&
            distance_change <= 3U && motion < VL53L5CX_DET_MOTION_THRESH && base) {
            int32_t diff = (int32_t)signal - (int32_t)base;
            s_baseline_signal[z] = (uint32_t)((int32_t)base + diff / 32);
        }

        /* Re-center any stationary per-zone plateau, including a persistent
           2 %% weak candidate that never produced a photo. Entry movement is
           still detected first; a stable-frame count inside the settle window
           may become the new empty-box reference. */
        if ((raw_mask & bit) && motion < VL53L5CX_DET_MOTION_THRESH &&
            s_test_prev_valid[z] && frame_signal_pct <= 1U &&
            frame_distance_mm <= 1U) {
            if (s_test_stable_since[z] == 0U) {
                s_test_stable_since[z] = now;
                s_test_stable_sig[z] = signal;
                s_test_stable_dist[z] = (uint32_t)distance;
                s_test_stable_frames[z] = 1U;
            } else {
                s_test_stable_sig[z] =
                    (uint32_t)(((uint64_t)s_test_stable_sig[z] * 7U + signal) / 8U);
                s_test_stable_dist[z] =
                    (uint32_t)((s_test_stable_dist[z] * 7U + (uint32_t)distance) / 8U);
                if (s_test_stable_frames[z] < TOF_TEST_STABLE_WINDOW_FRAMES)
                    s_test_stable_frames[z]++;
                if ((now - s_test_stable_since[z]) >= TOF_TEST_SETTLE_MS &&
                    s_test_stable_frames[z] >= TOF_TEST_STABLE_MIN_FRAMES) {
                    const uint32_t old_signal = s_baseline_signal[z];
                    const uint16_t old_distance = s_baseline_distance[z];
                    if (signal_change >= VL53L5CX_DET_LOCAL_SIGNAL_WEAK_PCT)
                        s_baseline_signal[z] = s_test_stable_sig[z];
                    if (distance_change >= VL53L5CX_DET_LOCAL_DIST_WEAK_MM)
                        s_baseline_distance[z] = (uint16_t)s_test_stable_dist[z];
                    printf("[ADAPT] Zone %u baseline sig %lu->%lu dist %u->%u (stable window: %u frames, max jitter 1%%/1mm)\n",
                           (unsigned)z, (unsigned long)old_signal,
                           (unsigned long)s_baseline_signal[z],
                           (unsigned)old_distance,
                           (unsigned)s_baseline_distance[z],
                           (unsigned)s_test_stable_frames[z]);
                    s_test_latched = (uint16_t)(s_test_latched & (uint16_t)~bit);
                    s_test_latched_strong &= (uint16_t)~bit;
                    s_test_latched_since[z] = 0U;
                    s_test_weak_active_mask &= (uint16_t)~bit;
                    s_test_blocked_mask &= (uint16_t)~bit;
                    s_test_stable_since[z] = 0U;
                    s_test_stable_frames[z] = 0U;
                }
            }
        } else {
            s_test_stable_since[z] = 0U;
            s_test_stable_frames[z] = 0U;
        }

        s_test_prev_sig[z] = signal;
        s_test_prev_dist[z] = (uint16_t)distance;
        s_test_prev_valid[z] = 1U;
    }

    /* Widespread persistent distance or signal shift is a scene change,
       unlike a one-zone insect. Request one bounded full baseline refresh. */
    if (!motion_active && (distance_up >= 12U || distance_down >= 12U ||
                           signal_up >= 12U || signal_down >= 12U)) {
        TestResetLocalTrack();
        if (s_test_scene_since == 0U) s_test_scene_since = now;
        if ((now - s_test_scene_since) >= TOF_TEST_SCENE_SETTLE_MS)
            s_test_refresh_requested = 1U;
    } else {
        s_test_scene_since = 0U;
    }

    const uint16_t blocked_current_mask = (uint16_t)(
        (candidate_event_mask & ~event_mask) & raw_mask);
#if VL53L5CX_DET_EVENT_TRACE > 0
    const uint16_t newly_blocked_mask = (uint16_t)(blocked_current_mask &
                                                   ~s_test_blocked_mask);
    if (newly_blocked_mask != 0U) {
        printf("TOFMISS,t=%lu,policy=%u,strong=%04X,fast=%04X,track=%04X,blocked=%04X\r\n",
               (unsigned long)now, (unsigned)event_policy,
               (unsigned)level_event_mask, (unsigned)fast_edge_mask,
               (unsigned)track_event_mask, (unsigned)newly_blocked_mask);
    }
#endif
    s_test_blocked_mask |= blocked_current_mask;

#if VL53L5CX_DET_CAL_TRACE > 0
    /* Diagnostic only: expose values below the event thresholds so a tiny
       insect can be calibrated from evidence rather than by lowering limits
       blindly. Negative sd means the zone moved toward the opposite-mounted
       sensor. No mask or detector state is changed by this trace. */
    static uint32_t cal_trace_tick = 0U;
    if (cal_trace_tick == 0U ||
        (now - cal_trace_tick) >= VL53L5CX_DET_CAL_TRACE_INTERVAL_MS) {
        cal_trace_tick = now;
        printf("TOFCAL,t=%lu,policy=%u,cd=%ld,cs=%ld,floor=%04X,fmt=z:sd_mm:ss_pct:fd_mm:fs_pct,z=",
               (unsigned long)now, (unsigned)event_policy,
               (long)common_distance, (long)common_signal,
               (unsigned)floor_mask);
        uint8_t printed = 0U;
        for (uint8_t z = 0U; z < 16U; z++) {
            if (!local_valid[z]) continue;
            if (printed != 0U) printf(";");
            printf("%u:%ld:%ld:%lu:%lu", (unsigned)z,
                   (long)signed_local_distance[z],
                   (long)signed_local_signal[z],
                   (unsigned long)frame_distance[z],
                   (unsigned long)frame_signal[z]);
            printed = 1U;
        }
        printf("\r\n");
    }
#endif

    if (new_evidence) {
        const uint16_t strong_event_mask = strong_level_mask;
        const uint16_t latched_before = s_test_latched;
        s_test_last_event_class = 0U;
        if (event_mask & level_event_mask)
            s_test_last_event_class |= VL53L5CX_TEST_EVENT_CLASS_LEVEL;
        if (event_mask & fast_edge_mask)
            s_test_last_event_class |= VL53L5CX_TEST_EVENT_CLASS_FAST_EDGE;
        if (event_mask & track_event_mask)
            s_test_last_event_class |= VL53L5CX_TEST_EVENT_CLASS_WEAK_TRACK;
        if ((event_mask & level_event_mask & strong_event_mask &
             latched_before) != 0U)
            s_test_last_event_class |= VL53L5CX_TEST_EVENT_CLASS_LATCHED_SCENE;
        s_test_latched |= event_mask;
        s_test_latched_strong |= (uint16_t)(event_mask & strong_event_mask);
        s_last_insect_detected = 1U;
        s_last_result.insect_detected = 1U;
        s_last_result.trigger_source = 0U;
        s_last_result.affected_count = 0U;
        s_last_result.valid_measurements = res.valid_measurements;
        if (event_mask & event_signal_mask)
            s_last_result.trigger_source |= VL53L5CX_TRIG_SIGNAL;
        if (event_mask & event_distance_mask)
            s_last_result.trigger_source |= VL53L5CX_TRIG_DISTANCE;
        for (uint8_t z = 0U; z < 16U; z++) {
            const uint16_t bit = (uint16_t)(1U << z);
            if (!(event_mask & bit)) continue;
            /* Stamp (and restart, on a re-arm re-fire) the per-zone latch
               timer used by the bounded re-arm. */
            s_test_latched_since[z] = now;
            uint32_t event_distance = local_distance[z];
            uint32_t event_signal = local_signal[z];
            if (s_test_track_distance_value[z] > event_distance)
                event_distance = s_test_track_distance_value[z];
            if (s_test_track_signal_value[z] > event_signal)
                event_signal = s_test_track_signal_value[z];
            const uint8_t k = s_last_result.affected_count++;
            s_last_result.affected_zones[k] = z;
            s_last_result.affected_drop[k] = (event_distance_mask & bit) ?
                event_distance : event_signal;
        }
#if VL53L5CX_DET_EVENT_TRACE > 0
        /* Zone entries are zone:local_distance_mm:local_signal_percent. */
        printf("TOFEVT,t=%lu,src=%u,v=%u,strong=%04X,track=%04X,raw=%04X,latched=%04X"
#if VL53L5CX_DET_MICRO_PERSIST_ENABLED
               ",micro=%04X"
#endif
               ",cd=%ld,cs=%ld,fmt=z:ld_mm:ls_pct,z=",
               (unsigned long)now, (unsigned)s_last_result.trigger_source,
               (unsigned)s_last_result.valid_measurements,
               (unsigned)strong_event_mask, (unsigned)track_event_mask,
               (unsigned)raw_mask, (unsigned)latched_before
#if VL53L5CX_DET_MICRO_PERSIST_ENABLED
              , (unsigned)micro_hold_mask
#endif
              , (long)common_distance, (long)common_signal);
        for (uint8_t i = 0U; i < s_last_result.affected_count; i++) {
            const uint8_t z = s_last_result.affected_zones[i];
            uint32_t event_distance = local_distance[z];
            uint32_t event_signal = local_signal[z];
            if (s_test_track_distance_value[z] > event_distance)
                event_distance = s_test_track_distance_value[z];
            if (s_test_track_signal_value[z] > event_signal)
                event_signal = s_test_track_signal_value[z];
            if (i > 0U) printf(";");
            printf("%u:%lu:%lu", (unsigned)z,
                   (unsigned long)event_distance,
                   (unsigned long)event_signal);
        }
        printf("\r\n");
#endif
        TestResetLocalTrack();
        return 1;
    }
    s_last_insect_detected = 0U;
    s_last_result.insect_detected = 0U;
    s_last_result.trigger_source = 0U;
    s_last_result.affected_count = 0U;
    s_test_last_event_class = 0U;
    return 0;
}

uint8_t VL53L5CX_TestGetLastEventClass(void)
{
    return s_test_last_event_class;
}

int VL53L5CX_TestTakeBaselineRefreshRequest(void)
{
    if (!s_test_refresh_requested) return 0;
    s_test_refresh_requested = 0U;
    return 1;
}
#endif

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
        if (VL53L5CX_STATUS_OK_FILT(status)) {
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

#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && (VL53L5CX_DET_RESOLUTION == 4)
void VL53L5CX_PrintZoneSnapshot(const char *reason)
{
    /* s_results is the frame VL53L5CX_Update() just read. No new ranging or
       I2C transaction is needed; zone indices are row-major (0..15 in 4x4). */
    printf("TOFZONE,frame_t=%lu,reason=%s,temp=%d,raw=%u/%u,format=z:valid:status:distance_mm:signal:motion",
           (unsigned long)s_last_frame_tick, reason, (int)s_results.silicon_temp_degc,
           (unsigned)s_last_result.trigger_source,
           (unsigned)s_last_result.affected_count);
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint32_t motion = 0U;
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        if (s_motion_initialized)
            motion = s_results.motion_indicator.motion[s_motion_config.map_id[z]];
#endif
        printf(",%u:%u:%u:%d:%lu:%lu", (unsigned)z,
               (unsigned)s_zone_valid[z], (unsigned)s_results.target_status[idx],
               (int)s_results.distance_mm[idx],
               (unsigned long)s_results.signal_per_spad[idx], (unsigned long)motion);
    }
    printf("\r\n");
}
#endif

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
            if (VL53L5CX_STATUS_OK_FILT(s_results.target_status[idx])) {
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
        if (VL53L5CX_STATUS_OK_FILT(s_results.target_status[idx])) {
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
     Baseline refresh across the cycle is selected by
     VL53L5CX_DUAL_BASELINE_MODE (app_config.h): QUICK_WAKE
     re-learns on wake, PRE_SLEEP just before sleep,
     NO_REFRESH disables the cycle refresh entirely
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

    /* Baseline refresh on wake (VL53L5CX_DUAL_BASELINE_MODE, app_config.h):
       after ST sleep the sensor's internal baseline engine is cold again
       and the first frames can be biased (field observation). QUICK_WAKE
       re-learns BEFORE entering ACTIVE, so the full
       VL53L5CX_DUAL_WAKE_DURATION_MS window is spent detecting against a
       fresh reference; PRE_SLEEP defers the learn to just before sleep
       instead; NO_REFRESH learns nothing on wake (fastest wake) — the
       baseline then comes from boot init, periodic/adaptive refresh and
       the manual button only. The state stays WAKING meanwhile, which
       every state-machine consumer already treats as "not active":
       CheckWakeTimeout() acts on ACTIVE only, Primary_Sleep() on
       ACTIVE/RETURNING only, and the guardian wake condition requires
       SLEEP — so nothing can race. */
#if VL53L5CX_DUAL_BASELINE_MODE == VL53L5CX_BASELINE_QUICK_WAKE
    printf("[PRIMARY] Post-wake re-learn (quick)\n");
    VL53L5CX_LearnBaseline();
    s_last_insect_detected = 0;  /* drop any stale pre-wake detection */
#endif

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
    /* Independent EXT (guardian) configuration — VL53L5CX_EXT_* block
       in app_config.h section 9. The defaults there equal the previous
       hardcoded values, so an untouched block keeps the old behavior.
       Target order (CLOSEST) and sharpener (10%) stay hardcoded. */
    vl53l5cx_set_resolution(&s_dev_ext, VL53L5CX_RESOLUTION_4X4);
    vl53l5cx_set_integration_time_ms(&s_dev_ext, VL53L5CX_EXT_INTEGRATION_MS);
    vl53l5cx_set_ranging_frequency_hz(&s_dev_ext, VL53L5CX_EXT_RANGING_FREQ_HZ);
    vl53l5cx_set_target_order(&s_dev_ext, VL53L5CX_TARGET_ORDER_CLOSEST);
    vl53l5cx_set_sharpener_percent(&s_dev_ext, 10);
    vl53l5cx_set_ranging_mode(&s_dev_ext, VL53L5CX_EXT_RANGING_MODE);

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
    int motion_st = vl53l5cx_motion_indicator_init(&s_dev_ext, &s_motion_config_ext,
        VL53L5CX_RESOLUTION_4X4);
    if (motion_st) {
        printf("[EXT] WARN: Motion indicator init failed: %d\n", motion_st);
        s_motion_initialized_ext = 0;
    } else {
        s_motion_config_ext.min_nb_for_global_detection = VL53L5CX_EXT_MOTION_MIN_ZONES;
        s_motion_config_ext.nb_of_temporal_accumulations = VL53L5CX_EXT_MOTION_PERSIST_FRAMES;
        s_motion_config_ext.extra_noise_sigma = VL53L5CX_EXT_MOTION_EXTRA_NOISE;
        /* Re-apply to the sensor (see VL53L5CX_Configure for why). */
        motion_st = vl53l5cx_motion_indicator_set_resolution(&s_dev_ext, &s_motion_config_ext,
            VL53L5CX_RESOLUTION_4X4);
        if (motion_st) {
            printf("[EXT] WARN: Motion indicator re-apply failed: %d (plugin defaults remain)\n",
                   motion_st);
        }
        s_motion_initialized_ext = 1;
        printf("[EXT] Motion indicator enabled (global_zones=%d, accum=%d, noise_sigma=%d)\n",
               VL53L5CX_EXT_MOTION_MIN_ZONES, VL53L5CX_EXT_MOTION_PERSIST_FRAMES,
               VL53L5CX_EXT_MOTION_EXTRA_NOISE);
    }
#else
    s_motion_initialized_ext = 0;
#endif

    printf("[EXT] Configured: res=4x4, int=%dms, freq=%dHz, mode=%d (target=CLOSEST, sharpener=10%%)\n",
           VL53L5CX_EXT_INTEGRATION_MS, VL53L5CX_EXT_RANGING_FREQ_HZ,
           (int)VL53L5CX_EXT_RANGING_MODE);
    printf("[EXT] Detection: drop>%d%%, motion>=%d, zones>=%d, min_signal=%d (VL53L5CX_EXT_* in app_config.h section 9)\n",
           VL53L5CX_EXT_THRESHOLD_PCT, VL53L5CX_EXT_MOTION_THRESH,
           VL53L5CX_EXT_MIN_AFFECTED_ZONES, VL53L5CX_EXT_MIN_SIGNAL);
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

    const uint8_t baseline_samples = VL53L5CX_EXT_BASELINE_SAMPLES;
    const uint8_t settle_frames = 3;
    const uint16_t min_valid_frames = baseline_samples / 2;

    /* Per-zone OK counts (same rationale as VL53L5CX_LearnBaseline):
       dividing by the total sample count biases zones that are only OK
       on part of the samples, causing fake detections. */
    uint32_t sum_signal[VL53L5CX_DET_NUM_ZONES] = {0};
    uint32_t sum_distance[VL53L5CX_DET_NUM_ZONES] = {0};
    uint16_t ok_frames[VL53L5CX_DET_NUM_ZONES] = {0};

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
            if (VL53L5CX_STATUS_OK_FILT(s_results_ext.target_status[idx])) {
                if (s_results_ext.signal_per_spad[idx] < VL53L5CX_EXT_MIN_SIGNAL) continue;
                sum_signal[z]   += s_results_ext.signal_per_spad[idx];
                sum_distance[z] += s_results_ext.distance_mm[idx];
                ok_frames[z]++;
            }
        }
        printf("  [EXT BASELINE %d/%d]\r", i + 1, baseline_samples);
    }

    uint8_t valid_count = 0;
    for (int z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (ok_frames[z] >= min_valid_frames) {
            s_baseline_signal_ext[z]   = sum_signal[z] / ok_frames[z];
            s_baseline_distance_ext[z] = (uint16_t)(sum_distance[z] / ok_frames[z]);
            s_zone_valid_ext[z] = 1;
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

        if (!VL53L5CX_STATUS_OK_FILT(s_results_ext.target_status[idx]))
            continue;

        if (s_results_ext.signal_per_spad[idx] == 0) continue;
        if (s_results_ext.signal_per_spad[idx] < VL53L5CX_EXT_MIN_SIGNAL) continue;

        s_last_result_ext.valid_measurements++;

        uint32_t signal_drop = 0;
        if (s_baseline_signal_ext[z] > 0) {
            int32_t diff = (int32_t)s_baseline_signal_ext[z] - (int32_t)s_results_ext.signal_per_spad[idx];
            if (diff < 0) diff = -diff;
            signal_drop = (uint32_t)diff * 100 / s_baseline_signal_ext[z];
        }

        int signal_triggered = (signal_drop > VL53L5CX_EXT_THRESHOLD_PCT);

        int motion_triggered = 0;
        if (s_motion_initialized_ext) {
            uint32_t motion_val = s_results_ext.motion_indicator.motion[s_motion_config_ext.map_id[z]];
            motion_triggered = (motion_val >= VL53L5CX_EXT_MOTION_THRESH);
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

        if (s_last_result_ext.affected_count >= VL53L5CX_EXT_MIN_AFFECTED_ZONES) {
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

#if VL53L5CX_DET_DEBUG_ZFRAME > 0
    static uint32_t zframe_counter = 0;
    zframe_counter++;
    if (zframe_counter >= VL53L5CX_DET_DEBUG_ZFRAME_INT) {
        zframe_counter = 0;
        VL53L5CX_External_PrintZFrame();
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

        if (VL53L5CX_STATUS_OK_FILT(status)) {
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

/* ================================================================
   Manual Baseline Refresh (button-triggered, TEST_TOF_MODE)
   ================================================================
   Same procedure as the documented periodic/adaptive refresh:
   stop-ranging -> 50 ms -> start-ranging -> 200 ms -> re-learn.
   In dual mode the camera ToF (primary) is parked in ST sleep by
   default, so it is woken first and put back to sleep afterwards.
   The wake follows VL53L5CX_DUAL_BASELINE_MODE: with QUICK_WAKE it
   already re-learns on wake, so the documented procedure below is
   the final re-learn; with PRE_SLEEP / NO_REFRESH the wake does
   not learn, so this procedure is the only re-learn of the press.
   The external (guardian) baseline is refreshed in place.
   Blocking for a few seconds — only call from sensor_task.
   ================================================================ */

void VL53L5CX_RefreshBaseline_Manual(void)
{
#if VL53L5CX_DUAL_SENSOR
    printf("[BTN] Refreshing PRIMARY baseline (waking from sleep)...\n");
    /* Wake the camera ToF if it is parked in ST sleep. Already-active
       states (mid detection cycle) simply skip the wake sequence (and
       any on-wake re-learn inside VL53L5CX_Primary_Wake). */
    if (!VL53L5CX_Primary_IsActive())
        VL53L5CX_Primary_Wake();
#else
    printf("[BTN] Refreshing PRIMARY baseline...\n");
#endif

    /* Documented refresh procedure (same as periodic/adaptive refresh) */
    vl53l5cx_stop_ranging(&s_dev);
    vTaskDelay(pdMS_TO_TICKS(50));
    vl53l5cx_start_ranging(&s_dev);
    vTaskDelay(pdMS_TO_TICKS(200));
    VL53L5CX_LearnBaseline();
    s_last_insect_detected = 0;  /* drop any stale pre-press detection */

#if VL53L5CX_DUAL_SENSOR
    /* Park the camera ToF back in ST sleep (default dual-mode state) —
       same state dance as VL53L5CX_Primary_CheckWakeTimeout(). */
    s_external_state = EXTERNAL_STATE_WAITING;
    VL53L5CX_Primary_Sleep();
    s_external_state = EXTERNAL_STATE_MONITORING;
    s_primary_detection_confirm = 0;

    /* External guardian: always on, refresh in place. */
    if (VL53L5CX_External_GetState() != EXTERNAL_STATE_IDLE) {
        printf("[BTN] Refreshing EXTERNAL (guardian) baseline...\n");
        vl53l5cx_stop_ranging(&s_dev_ext);
        vTaskDelay(pdMS_TO_TICKS(50));
        vl53l5cx_start_ranging(&s_dev_ext);
        vTaskDelay(pdMS_TO_TICKS(200));
        VL53L5CX_External_LearnBaseline();
        s_last_insect_detected_ext = 0;
    }
#endif

    printf("[BTN] Baseline refresh done\n");
}
