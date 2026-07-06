

/**
 * @file    vlc5_tof.c
 * @brief   VLC5 (VL53L5CX) Time-of-Flight Sensor Module — Implementation
 *
 *   Wraps all VL53L5CX operations (init, calibration, detection) into a
 *   clean module. The DetectionTask runs as a FreeRTOS task, similar to
 *   the button handler pattern.
 ******************************************************************************
 */

#include "vlc5_tof.h"

/* ---- STM32 HAL / BSP ---- */
#include "stm32n6xx_hal.h"

/* ---- VL53L5CX ST API ---- */
#include "platform.h"
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_motion_indicator.h"

/* ---- FreeRTOS ---- */
#include "FreeRTOS.h"
#include "task.h"

/* ---- BSP for LEDs ---- */
#include "stm32n6570_discovery.h"

/* ---- WS2812 illumination ---- */
#include "ws2812.h"

/* ================================================================
 * EXTERNAL PERIPHERAL HANDLES (declared in main.c)
 * ================================================================ */
extern I2C_HandleTypeDef hi2c1;

/* ================================================================
 * PRIVATE VARIABLES
 * ================================================================ */

/** Sensor configuration and results (module-private) */
static VL53L5CX_Configuration  g_vlc5_dev;
static VL53L5CX_ResultsData    g_vlc5_results;

/** Calibration data */
static VLC5_Calibration_t g_vlc5_calibration;

/** Detection callback */
static VLC5_DetectionCallback_t g_vlc5_detection_cb = NULL;

/** Sensor ready flag */
static bool g_vlc5_ready = false;

/** Minimum valid zone count for calibration */
#define VLC5_MIN_VALID_ZONES  2

/* ================================================================
 * PRIVATE FUNCTIONS
 * ================================================================ */

static void VLC5_GPIO_Init(void);
static void VLC5_PowerUp(void);
static uint8_t VLC5_DetectInsect(const VL53L5CX_ResultsData *results,
                                  uint8_t *affected_zones,
                                  uint32_t *affected_drops,
                                  uint8_t *affected_count);

/* ================================================================
 * GPIO + POWER
 * ================================================================ */

/**
 * @brief Configure GPIO pins used by the VL53L5CX sensor.
 *   PD0  = PWR_EN  (power enable)
 *   PD6  = LPn     (low-power / sleep control)
 *   PH5  = RST     (hardware reset)
 */
static void VLC5_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* PD0 - PWR_EN */
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);

    /* PD6 - LPn */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    /* PH5 - RST */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);

    printf("[VLC5] GPIO control pins configured\n");
}

/**
 * @brief Power-up sequence for the VL53L5CX sensor.
 */
static void VLC5_PowerUp(void)
{
    printf("[VLC5] Powering up sensor...\n");

    /* Ensure PWR_EN is HIGH */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(20);

    /* Hardware reset: pull RST low, then high */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(20);

    /* Bring LPn HIGH (normal operation) */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(100);

    printf("[VLC5] Power-up complete\n");
}

/* ================================================================
 * PUBLIC: INITIALIZATION
 * ================================================================ */

uint8_t VLC5_Init(void)
{
    uint8_t status;
    uint8_t is_alive;

    /* Initialize GPIO and power up */
    VLC5_GPIO_Init();
    VLC5_PowerUp();

    /* Configure platform address */
    g_vlc5_dev.platform.address = 0x29;

    /* Check if sensor is alive */
    status = vl53l5cx_is_alive(&g_vlc5_dev, &is_alive);
    if (!is_alive || status) {
        printf("[VLC5] ERROR: Sensor not detected! (status=%d, alive=%d)\n",
               status, is_alive);
        return 1;
    }
    printf("[VLC5] Sensor detected!\n");

    /* Initialize the sensor */
    status = vl53l5cx_init(&g_vlc5_dev);
    if (status) {
        printf("[VLC5] ERROR: Init failed (status=%d)\n", status);
        return 2;
    }
    printf("[VLC5] Sensor initialized\n");

    /* Configure resolution */
    status = vl53l5cx_set_resolution(&g_vlc5_dev, VL53L5CX_RESOLUTION_4X4);
    if (status) {
        printf("[VLC5] WARNING: Set resolution failed (status=%d)\n", status);
    }

    /* Reset calibration data */
    memset(&g_vlc5_calibration, 0, sizeof(g_vlc5_calibration));

    g_vlc5_ready = true;
    printf("[VLC5] Init complete (ready=true)\n");
    return 0;
}

uint8_t VLC5_IsAlive(uint8_t *is_alive)
{
    if (!g_vlc5_ready) return 1;
    return vl53l5cx_is_alive(&g_vlc5_dev, is_alive);
}

/* ================================================================
 * PUBLIC: CALIBRATION
 * ================================================================ */

uint8_t VLC5_StartRanging(void)
{
    uint8_t status;

    if (!g_vlc5_ready) return 1;

    /* Configure ranging parameters */
    status = vl53l5cx_set_integration_time_ms(&g_vlc5_dev, VLC5_INTEGRATION_TIME_MS);
    if (status) return status;

    status = vl53l5cx_set_ranging_frequency_hz(&g_vlc5_dev, VLC5_RANGING_FREQ_HZ);
    if (status) return status;

    status = vl53l5cx_set_target_order(&g_vlc5_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    if (status) return status;

    status = vl53l5cx_set_sharpener_percent(&g_vlc5_dev, 10);
    if (status) return status;

    status = vl53l5cx_set_ranging_mode(&g_vlc5_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    if (status) return status;

    /* Start ranging */
    vl53l5cx_start_ranging(&g_vlc5_dev);
    VL53L5CX_WaitMs(&g_vlc5_dev.platform, 200);

    printf("[VLC5] Continuous ranging started (%d Hz, %d ms integration)\n",
           VLC5_RANGING_FREQ_HZ, VLC5_INTEGRATION_TIME_MS);
    return 0;
}

void VLC5_StopRanging(void)
{
    if (!g_vlc5_ready) return;
    vl53l5cx_stop_ranging(&g_vlc5_dev);
    printf("[VLC5] Ranging stopped\n");
}

/**
 * @brief Calibrate the sensor by learning the baseline signal level.
 *
 *   This function:
 *     1. Starts continuous ranging
 *     2. Takes VLC5_BASELINE_SAMPLES readings
 *     3. Averages signal and distance per zone
 *     4. Stores calibration data
 *
 *   Blocks for approximately: BASELINE_SAMPLES / RANGING_FREQ_HZ seconds.
 *
 * @param cal  Output calibration data (may be NULL).
 * @return 0 on success, 1 if not enough valid zones.
 */
uint8_t VLC5_Calibrate(VLC5_Calibration_t *cal)
{
    uint8_t is_ready;
    uint32_t signal_acc[VLC5_NUM_ZONES] = {0};
    uint16_t dist_acc[VLC5_NUM_ZONES]   = {0};
    uint8_t  zone_valid[VLC5_NUM_ZONES] = {0};

    printf("[VLC5] Starting calibration (%d samples)...\n", VLC5_BASELINE_SAMPLES);

    /* Start ranging if not already running */
    VLC5_StartRanging();

    /* Accumulate baseline samples */
    for (uint8_t s = 0; s < VLC5_BASELINE_SAMPLES; s++)
    {
        /* Wait for data ready */
        do {
            vl53l5cx_check_data_ready(&g_vlc5_dev, &is_ready);
            if (!is_ready) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        } while (!is_ready);

        /* Read data */
        if (vl53l5cx_get_ranging_data(&g_vlc5_dev, &g_vlc5_results) != 0) {
            continue;
        }

        /* Accumulate per-zone signal and distance */
        for (int z = 0; z < VLC5_NUM_ZONES; z++)
        {
            uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

            /* Accept only valid target statuses */
            if (g_vlc5_results.target_status[idx] == 0 ||
                g_vlc5_results.target_status[idx] == 5 ||
                g_vlc5_results.target_status[idx] == 9)
            {
                signal_acc[z] += g_vlc5_results.signal_per_spad[idx];
                dist_acc[z]   += g_vlc5_results.distance_mm[idx];
                zone_valid[z]  = 1;
            }
        }

        printf("[VLC5]   Sample %d/%d\r", s + 1, VLC5_BASELINE_SAMPLES);
    }

    printf("\n");

    /* Compute averages and populate calibration struct */
    uint8_t valid_count = 0;

    for (int z = 0; z < VLC5_NUM_ZONES; z++)
    {
        if (zone_valid[z])
        {
            g_vlc5_calibration.signal[z]   = signal_acc[z] / VLC5_BASELINE_SAMPLES;
            g_vlc5_calibration.distance[z] = dist_acc[z] / VLC5_BASELINE_SAMPLES;
            valid_count++;
        }
        else
        {
            g_vlc5_calibration.signal[z]   = 0;
            g_vlc5_calibration.distance[z] = 0;
        }
    }

    g_vlc5_calibration.valid_zones = valid_count;
    g_vlc5_calibration.ready       = (valid_count >= VLC5_MIN_VALID_ZONES);

    /* Print calibration results */
    printf("[VLC5] Calibration Results:\n");
    printf("[VLC5]   Valid zones: %d/%d\n", valid_count, VLC5_NUM_ZONES);

    for (int z = 0; z < VLC5_NUM_ZONES; z++)
    {
        if (zone_valid[z])
        {
            printf("[VLC5]   Z%2d: dist=%4d mm  signal=%6d kcps/spad\n",
                   z, g_vlc5_calibration.distance[z], g_vlc5_calibration.signal[z]);
        }
    }

    if (g_vlc5_calibration.ready)
    {
        printf("[VLC5] Calibration OK (threshold >%d%% signal drop)\n",
               VLC5_INSECT_THRESHOLD_PCT);
    }
    else
    {
        printf("[VLC5] Calibration FAILED: only %d valid zones (need >= %d)\n",
               valid_count, VLC5_MIN_VALID_ZONES);
    }

    /* Optionally copy to caller buffer */
    if (cal != NULL)
    {
        memcpy(cal, &g_vlc5_calibration, sizeof(VLC5_Calibration_t));
    }

    return g_vlc5_calibration.ready ? 0 : 1;
}

const VLC5_Calibration_t *VLC5_GetCalibration(void)
{
    return &g_vlc5_calibration;
}

bool VLC5_IsCalibrated(void)
{
    return g_vlc5_calibration.ready;
}

/* ================================================================
 * PUBLIC: DATA READING
 * ================================================================ */

uint8_t VLC5_WaitDataReady(void)
{
    uint8_t is_ready;

    do {
        vl53l5cx_check_data_ready(&g_vlc5_dev, &is_ready);
        if (!is_ready) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } while (!is_ready);

    return 0;
}

uint8_t VLC5_ReadData(VL53L5CX_ResultsData *results)
{
    if (!g_vlc5_ready) return 1;

    uint8_t status = vl53l5cx_get_ranging_data(&g_vlc5_dev, &g_vlc5_results);
    if (status == 0 && results != NULL)
    {
        memcpy(results, &g_vlc5_results, sizeof(VL53L5CX_ResultsData));
    }
    return status;
}

/* ================================================================
 * PUBLIC: CALLBACK
 * ================================================================ */

void VLC5_SetDetectionCallback(VLC5_DetectionCallback_t cb)
{
    g_vlc5_detection_cb = cb;
}

/* ================================================================
 * PRIVATE: INSECT DETECTION ALGORITHM
 * ================================================================ */

/**
 * @brief Analyze a single ranging frame for insect detection.
 *
 *   Compares current signal per zone against the calibrated baseline.
 *   A signal drop greater than VLC5_INSECT_THRESHOLD_PCT triggers detection.
 *
 * @param results        Current ranging data.
 * @param affected_zones Output array of affected zone indices.
 * @param affected_drops Output array of signal drop percentages.
 * @param affected_count Output number of affected zones.
 * @return 1 if insect detected, 0 otherwise.
 */
static uint8_t VLC5_DetectInsect(const VL53L5CX_ResultsData *results,
                                  uint8_t *affected_zones,
                                  uint32_t *affected_drops,
                                  uint8_t *affected_count)
{
    *affected_count = 0;

    for (int z = 0; z < VLC5_NUM_ZONES; z++)
    {
        /* Skip invalid zones */
        if (g_vlc5_calibration.signal[z] == 0) continue;

        uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;

        /* Check target status */
        if (results->target_status[idx] != 0 &&
            results->target_status[idx] != 5 &&
            results->target_status[idx] != 9)
        {
            continue;
        }

        /* Calculate signal drop percentage */
        int32_t diff = (int32_t)g_vlc5_calibration.signal[z] -
                       (int32_t)results->signal_per_spad[idx];
        if (diff < 0) diff = -diff;

        uint32_t signal_drop = (uint32_t)diff * 100 / g_vlc5_calibration.signal[z];

        if (signal_drop > VLC5_INSECT_THRESHOLD_PCT)
        {
            affected_zones[*affected_count] = (uint8_t)z;
            affected_drops[*affected_count] = signal_drop;
            (*affected_count)++;
        }
    }

    return (*affected_count > 0) ? 1 : 0;
}

/* ================================================================
 * PUBLIC: DETECTION TASK (FreeRTOS)
 * ================================================================ */

/**
 * @brief Main detection task — runs forever.
 *
 *   Monitors ToF zones for insect detection. When detected:
 *     1. Turns off GREEN LED
 *     2. Turns on RED LED
 *     3. Flashes WS2812 illumination
 *     4. Calls detection callback (camera capture + SD store)
 *     5. Enters cooldown period
 *     6. Restores GREEN LED
 */
void VLC5_DetectionTask(void *arg)
{
    (void)arg;

    uint8_t is_ready;
    uint8_t affected_zones[VLC5_NUM_ZONES];
    uint32_t affected_drops[VLC5_NUM_ZONES];
    uint8_t affected_count;

    printf("\n============================================\n");
    printf("  VLC5 DETECTION TASK (VL53L5CX)\n");
    printf("  4x4 zone-based signal drop detection\n");
    printf("  Auto-capture on detection\n");
    printf("============================================\n\n");

    /* Wait for sensor to be ready */
    while (!g_vlc5_ready)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Wait for calibration to complete */
    while (!VLC5_IsCalibrated())
    {
        printf("[VLC5] Waiting for calibration...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Verify calibration has enough zones */
    if (g_vlc5_calibration.valid_zones < VLC5_MIN_VALID_ZONES)
    {
        printf("[VLC5] ERROR: Not enough valid zones (%d/%d)\n",
               g_vlc5_calibration.valid_zones, VLC5_MIN_VALID_ZONES);
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    printf("[VLC5] Monitoring %d zones. Waiting for insect...\n",
           g_vlc5_calibration.valid_zones);

    uint32_t cooldown_frames = 0;
    volatile uint8_t is_capturing = 0;

    /* GREEN LED = monitoring state */
    BSP_LED_On(LED_GREEN);

    /* Main detection loop */
    while (1)
    {
        /* During capture, just drain data without detecting */
        if (is_capturing)
        {
            VLC5_WaitDataReady();
            vl53l5cx_get_ranging_data(&g_vlc5_dev, &g_vlc5_results);
            continue;
        }

        /* Wait for new data */
        VLC5_WaitDataReady();

        if (vl53l5cx_get_ranging_data(&g_vlc5_dev, &g_vlc5_results) != 0)
        {
            continue;
        }

        /* Decrement cooldown */
        if (cooldown_frames > 0) cooldown_frames--;

        /* Run insect detection algorithm */
        if (VLC5_DetectInsect(&g_vlc5_results, affected_zones,
                               affected_drops, &affected_count))
        {
            /* Only trigger if not in cooldown */
            if (cooldown_frames == 0)
            {
                printf("[VLC5] INSECT DETECTED! (%d zones)\n", affected_count);

                /* Print affected zone details */
                for (uint8_t k = 0; k < affected_count; k++)
                {
                    uint8_t z = affected_zones[k];
                    uint8_t idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
                    printf("[VLC5]   Z%2d: dist=%4d mm  signal=%6d  "
                           "baseline=%6d  (drop=%lu%%)\n",
                           z,
                           g_vlc5_results.distance_mm[idx],
                           g_vlc5_results.signal_per_spad[idx],
                           g_vlc5_calibration.signal[z],
                           (unsigned long)affected_drops[k]);
                }

                /* ---- LED + Illumination sequence ---- */
                BSP_LED_Off(LED_GREEN);
                BSP_LED_On(LED_RED);
                WS2812_TurnOn();
                vTaskDelay(pdMS_TO_TICKS(100));
                WS2812_TurnOff();

                /* ---- Call detection callback ---- */
                if (g_vlc5_detection_cb != NULL)
                {
                    is_capturing = 1;
                    g_vlc5_detection_cb(affected_count,
                                        affected_zones,
                                        affected_drops);
                    is_capturing = 0;
                }

                /* ---- Restore monitoring state ---- */
                BSP_LED_Off(LED_RED);
                BSP_LED_On(LED_GREEN);

                /* Enter cooldown */
                cooldown_frames = VLC5_CAPTURE_COOLDOWN_FRAMES;
            }
        }
    }
}