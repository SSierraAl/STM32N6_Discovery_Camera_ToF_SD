/**
 * @file    app_thread.c
 * @brief   Multi-threaded Camera + Sensor + Storage
 */

#include <stdio.h>
#include <string.h>
#include "app_thread.h"
#include "app_config.h"
#include "app_cam.h"
#include "perf_debug.h"
#include "vl53l5cx_detection.h"
#include "ws2812.h"
#include "stm32n6xx_hal.h"
#include "stm32n6570_discovery.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern uint8_t capture_buf[];
#if CAPTURE_MODE == 1
extern uint8_t save_buf[];
#endif
#if CAPTURE_MODE == 2 || CAPTURE_MODE == 4
extern uint8_t batch_buf[];
#endif

QueueHandle_t    camera_cmd_queue   = NULL;
QueueHandle_t    storage_cmd_queue  = NULL;
QueueHandle_t    sensor_event_queue = NULL;
SemaphoreHandle_t camera_ready_sem  = NULL;
SemaphoreHandle_t storage_done_sem  = NULL;
volatile SensorState_t g_sensor_state = SENSOR_STATE_IDLE;

static volatile int g_capture_busy = 0;
static volatile int g_last_storage_rc = 0;
static volatile int g_last_batch_frames = 0;
static uint32_t g_snap_count = 0;
static uint32_t g_sd_img_base_block;
#define SD_IMG_HEADER_SIZE  64
#define SD_BLOCK_SIZE       512
static uint32_t g_debug_frame_count = 0;

typedef struct {
    uint32_t magic; uint32_t width; uint32_t height;
    uint32_t pixel_format; uint32_t data_size; uint32_t timestamp;
    uint32_t checksum; uint32_t snap_id; uint8_t reserved[28];
} sd_image_header_t;
#define SD_HEADER_TAG  0x49444745U

extern SD_HandleTypeDef hsd1;
extern uint8_t sd_batch_buf[];
PerfTimer_t g_perf_timer;

/* ==================== IPC ==================== */
BaseType_t IPC_Init(void)
{
    camera_cmd_queue = xQueueCreate(CAMERA_CMD_QUEUE_LEN, sizeof(CameraCmd_t));
    if (!camera_cmd_queue) return pdFALSE;
    storage_cmd_queue = xQueueCreate(STORAGE_CMD_QUEUE_LEN, sizeof(StorageCmd_t));
    if (!storage_cmd_queue) return pdFALSE;
    sensor_event_queue = xQueueCreate(SENSOR_EVENT_QUEUE_LEN, sizeof(CameraEvent_typed));
    if (!sensor_event_queue) return pdFALSE;
    camera_ready_sem = xSemaphoreCreateBinary();
    if (!camera_ready_sem) return pdFALSE;
    storage_done_sem = xSemaphoreCreateCounting(STORAGE_CMD_QUEUE_LEN, 0);
    if (!storage_done_sem) return pdFALSE;
    xSemaphoreTake(camera_ready_sem, 0);
    xSemaphoreTake(storage_done_sem, 0);
    g_sd_img_base_block = SD_SNAP_BASE_BLOCK;
    printf("[IPC] OK\n");
    return pdTRUE;
}

void IPC_Deinit(void)
{
    if (camera_cmd_queue) vQueueDelete(camera_cmd_queue);
    if (storage_cmd_queue) vQueueDelete(storage_cmd_queue);
    if (sensor_event_queue) vQueueDelete(sensor_event_queue);
    if (camera_ready_sem) vSemaphoreDelete(camera_ready_sem);
    if (storage_done_sem) vSemaphoreDelete(storage_done_sem);
}

int Capture_IsBusy(void) { return g_capture_busy; }

BaseType_t Capture_RequestSnapOnly(void)
{
    CameraCmd_t cmd = {0}; cmd.type = CAM_CMD_SNAP;
    return xQueueSend(camera_cmd_queue, &cmd, pdMS_TO_TICKS(100));
}

/* ==================== SD AUTO-RECOVERY ==================== */
/* NOTE: Not static — called from main.c (btn_thread) as well as storage_task. */
int SD_Reinit(void)
{
#if PERF_DEBUG_LEVEL >= 1
    printf("[SD] === SD RECOVERY STARTED ===\n");
#endif
    uint32_t saved_block = g_sd_img_base_block;
    HAL_SD_Abort(&hsd1);
    HAL_SD_DeInit(&hsd1);
    vTaskDelay(pdMS_TO_TICKS(100));
    __HAL_RCC_SDMMC2_FORCE_RESET();
    vTaskDelay(pdMS_TO_TICKS(10));
    __HAL_RCC_SDMMC2_RELEASE_RESET();
    vTaskDelay(pdMS_TO_TICKS(10));
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_SDMMC2;
    PeriphClkInit.Sdmmc2ClockSelection  = RCC_SDMMC2CLKSOURCE_HCLK;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    hsd1.Instance             = SDMMC2;
    hsd1.Init.ClockEdge       = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave  = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide         = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv        = 2;
    hsd1.State                = HAL_SD_STATE_RESET;
    HAL_SD_MspInit(&hsd1);
    HAL_StatusTypeDef status = HAL_SD_Init(&hsd1);
    if (status == HAL_OK) {
        status = HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);
        if (status == HAL_OK) { hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B; }
    }
    g_sd_img_base_block = saved_block;
    if (status == HAL_OK) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] Recovery OK! Next write from block %lu\n", (unsigned long)g_sd_img_base_block);
#endif
        return 0;
    } else {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] Recovery FAILED (HAL=0x%08lX)\n", (unsigned long)status);
#endif
        return -1;
    }
}

/* ==================== SD STORE ==================== */
/** Wait for SD card to be ready for next write.
    MUST use HAL_SD_GetCardState() (sends CMD13 to the card) to query the
    ACTUAL card state. HAL_SD_GetState() only returns the HAL driver's
    software flag, which is already READY after HAL_SD_WriteBlocks returns
    (blocking call), even though the card's internal NAND flash is still
    erasing/programming. Writing while the card is PROGRAMMING causes
    STA=0x5000 (Data CRC timeout) errors. */
static int SD_WaitForReady(void)
{
    uint32_t wait_ms = 0;
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER && wait_ms < 5000) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_ms++;
    }
    if (wait_ms >= 5000) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] timeout before next batch (card still PROGRAMMING after %lu ms)\n",
               (unsigned long)wait_ms);
#endif
        return -1;
    }
    return 0;
}

static int SD_StoreRawImage(const uint8_t *img_buf, uint32_t img_size, uint32_t w, uint32_t h, uint32_t pixel_format, uint32_t snap_id)
{
    uint32_t base = g_sd_img_base_block;
    uint32_t batch_blocks = SD_BATCH_WRITE_BLOCKS;
    uint32_t batch_size = batch_blocks * SD_BLOCK_SIZE;
    uint32_t local_batch_count = 0;
    uint32_t i, checksum = 0;

    if (hsd1.SdCard.BlockNbr > 0) {
        uint32_t total_blocks = (SD_IMG_HEADER_SIZE + img_size + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
        if ((base + total_blocks - 1) >= hsd1.SdCard.BlockNbr) {
#if PERF_DEBUG_LEVEL >= 1
            printf("[SD] OVERFLOW\n");
#endif
            return -1;
        }
    }

    for (i = 0; i < img_size; i++) checksum ^= img_buf[i];

    sd_image_header_t hdr;
    hdr.magic = SD_HEADER_TAG; hdr.width = w; hdr.height = h;
    hdr.pixel_format = pixel_format; hdr.data_size = img_size;
    hdr.timestamp = HAL_GetTick(); hdr.checksum = checksum;
    hdr.snap_id = snap_id; memset(hdr.reserved, 0, sizeof(hdr.reserved));

#if PERF_DEBUG_LEVEL >= 1
    printf("[SD] #%" PRIu32 " %lux%lu %lu bytes (%.3f MB)\n",
           snap_id, (unsigned long)w, (unsigned long)h, img_size, img_size / 1048576.0f);
#endif

    PERF_MARK(g_perf_timer, STORAGE);

    uint32_t t_wait = HAL_GetTick();
    if (SD_WaitForReady() != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] Not ready for first write\n");
#endif
        return -1;
    }
    uint32_t wait_ms = HAL_GetTick() - t_wait;

    memset(sd_batch_buf, 0, batch_size);
    memcpy(sd_batch_buf, &hdr, SD_IMG_HEADER_SIZE);
    uint32_t first_chunk = SD_BLOCK_SIZE - SD_IMG_HEADER_SIZE;
    if (first_chunk > img_size) first_chunk = img_size;
    memcpy(sd_batch_buf + SD_IMG_HEADER_SIZE, img_buf, first_chunk);

    uint32_t img_offset = first_chunk;
    uint32_t batch_blk  = 1;
    while (img_offset < img_size && batch_blk < batch_blocks) {
        uint32_t chunk = SD_BLOCK_SIZE;
        if (chunk > img_size - img_offset) chunk = img_size - img_offset;
        memcpy(sd_batch_buf + batch_blk * SD_BLOCK_SIZE, img_buf + img_offset, chunk);
        if (chunk < SD_BLOCK_SIZE)
            memset(sd_batch_buf + batch_blk * SD_BLOCK_SIZE + chunk, 0, SD_BLOCK_SIZE - chunk);
        img_offset += chunk;
        batch_blk++;
    }

    PERF_MARK(g_perf_timer, CACHE_CLEAN);
    SCB_CleanDCache_by_Addr((uint32_t *)sd_batch_buf, batch_blk * SD_BLOCK_SIZE);
    PERF_MARK(g_perf_timer, SD_WRITE);

    uint32_t t0 = HAL_GetTick();
    HAL_StatusTypeDef st = HAL_SD_WriteBlocks(&hsd1, sd_batch_buf, base, batch_blk, HAL_MAX_DELAY);
    uint32_t write_ms = HAL_GetTick() - t0;

    if (st != HAL_OK) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] header FAIL block %lu HAL=0x%08lX STA=0x%08lX\n",
               (unsigned long)base, (unsigned long)st, (unsigned long)SDMMC2->STA);
#endif
        return -1;
    }

    Perf_SD_RecordBatch(&g_perf_timer, wait_ms, write_ms, 0);
    local_batch_count++;
    uint32_t current_block = base + batch_blk;

    while (img_offset < img_size) {
        uint32_t remaining_bytes = img_size - img_offset;
        uint32_t remaining_blocks_full = (remaining_bytes + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
        uint32_t blocks_in_batch = (remaining_blocks_full < batch_blocks) ? remaining_blocks_full : batch_blocks;

        for (uint32_t b = 0; b < blocks_in_batch; b++) {
            uint32_t src = img_offset + b * SD_BLOCK_SIZE;
            uint32_t dst = b * SD_BLOCK_SIZE;
            if (src + SD_BLOCK_SIZE <= img_size) {
                memcpy(sd_batch_buf + dst, img_buf + src, SD_BLOCK_SIZE);
            } else {
                uint32_t partial = img_size - src;
                memcpy(sd_batch_buf + dst, img_buf + src, partial);
                memset(sd_batch_buf + dst + partial, 0, SD_BLOCK_SIZE - partial);
            }
        }

        SCB_CleanDCache_by_Addr((uint32_t *)sd_batch_buf, blocks_in_batch * SD_BLOCK_SIZE);

        /* HYBRID WAIT (adaptive + minimum gap):
           1. Poll HAL_SD_GetCardState until the card reports TRANSFER ready
              (may take 0-5000ms depending on card speed and internal flash state)
           2. THEN wait 20ms for the card's internal NAND flash erase/program
              cycles to complete. The CMD13 "ready" status is optimistic — the
              card's host controller reports ready before the NAND flash is
              actually done. Without this gap, STA=0x5000 (Data CRC timeout)
              occurs on many SDXC cards when the next write arrives too early.
           This combination: adaptive wait handles slow cards, fixed gap handles
           the CMD13-vs-NAND timing mismatch that causes CRC errors. */
        t0 = HAL_GetTick();
        if (SD_WaitForReady() != 0) {
#if PERF_DEBUG_LEVEL >= 1
            printf("[SD] timeout before block %lu\n", (unsigned long)current_block);
#endif
            return -1;
        }
        wait_ms = HAL_GetTick() - t0;

        /* Minimum inter-batch recovery gap (configurable via SD_BATCH_RECOVERY_GAP_MS in app_config.h) */
        t0 = HAL_GetTick();
        vTaskDelay(pdMS_TO_TICKS(SD_BATCH_RECOVERY_GAP_MS));
        uint32_t gap_ms = HAL_GetTick() - t0;

        t0 = HAL_GetTick();
        st = HAL_SD_WriteBlocks(&hsd1, sd_batch_buf, current_block, blocks_in_batch, HAL_MAX_DELAY);
        write_ms = HAL_GetTick() - t0;

        if (st != HAL_OK) {
#if PERF_DEBUG_LEVEL >= 1
            printf("[SD] FAIL block %lu HAL=0x%08lX STA=0x%08lX\n",
                   (unsigned long)current_block, (unsigned long)st, (unsigned long)SDMMC2->STA);
#endif
            return -1;
        }

        Perf_SD_RecordBatch(&g_perf_timer, wait_ms, write_ms, gap_ms);
        local_batch_count++;
#if PERF_DEBUG_LEVEL >= 2
        if (PERF_SD_BATCH_PRINT_EVERY > 0 && (local_batch_count % PERF_SD_BATCH_PRINT_EVERY) == 0) {
            uint32_t done_bytes = img_offset + blocks_in_batch * SD_BLOCK_SIZE;
            float pct = (100.0f * done_bytes) / img_size;
            printf("[SD] #%lu wait=%lums write=%lums gap=%lums | %.1f%%\n",
               (unsigned long)local_batch_count, (unsigned long)wait_ms, (unsigned long)write_ms, (unsigned long)gap_ms, pct);
        }
#elif PERF_DEBUG_LEVEL >= 1
        if ((local_batch_count % 64) == 0) {
            uint32_t done_bytes = img_offset + blocks_in_batch * SD_BLOCK_SIZE;
            float pct = (100.0f * done_bytes) / img_size;
            printf("[SD] ... %.1f%%\n", pct);
        }
#endif
        img_offset += blocks_in_batch * SD_BLOCK_SIZE;
        current_block += blocks_in_batch;
    }

    g_sd_img_base_block = current_block;
    PERF_MARK(g_perf_timer, DONE);
#if PERF_DEBUG_LEVEL >= 1
    printf("[SD] OK blocks %lu..%lu (%lu batches)\n",
           (unsigned long)base, (unsigned long)(current_block - 1), (unsigned long)local_batch_count);
#endif
    return 0;
}

/* ==================== SENSOR TASK ==================== */
void sensor_task(void *arg)
{
    (void)arg;
    extern volatile int system_ready;
    while (!system_ready) vTaskDelay(pdMS_TO_TICKS(500));

    if (VL53L5CX_Init(&hi2c1) != 0) {
        g_sensor_state = SENSOR_STATE_STOPPED;
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

#if VL53L5CX_DET_RESOLUTION == 8
    VL53L5CX_Configure(VL53L5CX_RESOLUTION_8X8, VL53L5CX_DET_INTEGRATION_MS, VL53L5CX_DET_RANGING_FREQ_HZ);
#else
    VL53L5CX_Configure(VL53L5CX_RESOLUTION_4X4, VL53L5CX_DET_INTEGRATION_MS, VL53L5CX_DET_RANGING_FREQ_HZ);
#endif

    VL53L5CX_StartRanging();

#if VL53L5CX_DUAL_SENSOR
    /* Initialize external (guardian) sensor */
    if (VL53L5CX_External_Init() != 0) {
        printf("[WARN] External ToF init failed, continuing with primary only\n");
    } else {
        VL53L5CX_External_Configure();
        VL53L5CX_External_StartRanging();
    }
#endif

    /* Learn primary sensor baseline (sensor is active from StartRanging above) */
    VL53L5CX_LearnBaseline();

#if VL53L5CX_DUAL_SENSOR
    /* Learn external sensor baseline */
    if (VL53L5CX_External_GetState() != EXTERNAL_STATE_IDLE) {
        VL53L5CX_External_LearnBaseline();
    }

    /* NOW put primary sensor to sleep (both baselines are learned).
       Use the startup variant: plain Primary_Sleep() is a no-op here because
       the state machine still holds its initial SLEEP value while the sensor
       is physically ranging. */
    VL53L5CX_Primary_SleepAtStartup();
#endif

    /* Only fail if PRIMARY sensor baseline is not ready (external is optional) */
    if (!VL53L5CX_IsBaselineReady()) {
        g_sensor_state = SENSOR_STATE_STOPPED;
        printf("[ERROR] Primary ToF baseline not ready!\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    g_sensor_state = SENSOR_STATE_RUNNING;
    printf("[SENSOR] Running\n");
    uint32_t capture_count = 0, cooldown = 0;

    while (1) {
        if (g_sensor_state == SENSOR_STATE_PAUSED) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

#if VL53L5CX_DUAL_SENSOR
        /* ---- DUAL SENSOR MODE ----
           External sensor is always ON, monitoring for motion/signal drop.
           Primary sensor (camera ToF) is in sleep mode by default.
           When external detects something, it wakes the primary. */

        /* Cooldown countdown, one step per loop iteration (same as single-sensor
           mode). Without this, cooldown latches at 30 after the first capture
           and blocks every subsequent trigger, even when raw data shows the
           signal drop above threshold. */
        if (cooldown > 0) cooldown--;

        /* Update external (guardian) sensor continuously */
        if (VL53L5CX_External_GetState() == EXTERNAL_STATE_MONITORING) {
            (void)VL53L5CX_External_Update();
        }

        /* Check primary sensor wake timeout */
        VL53L5CX_Primary_CheckWakeTimeout();

        /* When primary is active, also update it for detection */
        if (VL53L5CX_Primary_IsActive()) {
            if (!VL53L5CX_Update()) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            /* Check primary sensor detection while it's awake */
            if (VL53L5CX_IsInsectDetected() && cooldown == 0) {
                if (g_capture_busy) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
                g_capture_busy = 1;
                g_sensor_state = SENSOR_STATE_PAUSED;
                VL53L5CX_DetectionResult_t res = VL53L5CX_GetResult();

#if PERF_DEBUG_LEVEL >= 1
                const char* trig_str;
                if (res.trigger_source == 1) trig_str = "SIGNAL";
                else if (res.trigger_source == 2) trig_str = "MOTION";
                else if (res.trigger_source == 3) trig_str = "SIGNAL+MOTION";
                else trig_str = "UNKNOWN";
                printf(">>> INSECT DETECTED (Primary, woken by external) [%s]!\n", trig_str);
#endif

#if TEST_TOF_MODE
                /* === ON-SITE TEST MODE: no camera, no SD — RED LED only === */
                printf(">>> TEST: %u zone(s) affected, %u valid\r\n",
                       (unsigned)res.affected_count, (unsigned)res.valid_measurements);
                for (int a = 0; a < res.affected_count; a++)
                    printf("    zone %u -> %lu\r\n", (unsigned)res.affected_zones[a],
                           (unsigned long)res.affected_drop[a]);
                BSP_LED_Off(LED_GREEN); BSP_LED_On(LED_RED);
                vTaskDelay(pdMS_TO_TICKS(TEST_TOF_LED_MS));
                BSP_LED_Off(LED_RED); BSP_LED_On(LED_GREEN);
                g_sensor_state = SENSOR_STATE_RUNNING;
                g_capture_busy = 0;
                cooldown = 30;
                continue;
#else
                BSP_LED_Off(LED_GREEN); BSP_LED_On(LED_RED);
#if WS2812_MODE == 1
                WS2812_FlashStart(WS2812_ILLUMINATION_COLOR, WS2812_ILLUMINATION_BRIGHTNESS);
#elif WS2812_MODE == 0
                WS2812_TurnOn();
#else
                WS2812_TurnOn();
                vTaskDelay(pdMS_TO_TICKS(WS2812_INDICATOR_MS));
                WS2812_TurnOff();
#endif

                PerfTimer_t t;
                PERF_START(t);
                int rc = Capture_RequestSnapshot(60000);
                PERF_STOP(t);
                BSP_LED_Off(LED_RED); BSP_LED_On(LED_GREEN);
                VL53L5CX_StartRanging();
                g_sensor_state = SENSOR_STATE_RUNNING;
                g_capture_busy = 0;
                cooldown = 30;

                if (rc == 0) {
                    capture_count++;
#if PERF_PRINT_SUMMARY
                    Perf_PrintSummary(&g_perf_timer, capture_count);
                    Perf_UpdateStats(&g_perf_timer);
#else
                    printf("Snapshot #%lu SAVED (%lu ms)\n", (unsigned long)capture_count, (unsigned long)Perf_TotalElapsed(&t));
#endif
                } else {
#if PERF_DEBUG_LEVEL >= 1
                    printf("[SENSOR] Capture FAILED rc=%d\n", rc);
#endif
                }
#endif /* TEST_TOF_MODE */
            }
        }
#else
        /* ---- SINGLE SENSOR MODE ----
           Standard VL53L5CX_Update() loop as before. */

        /* Update primary sensor */
        if (!VL53L5CX_Update()) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        g_debug_frame_count++;
        if (g_debug_frame_count >= 1) g_debug_frame_count = 0;
        if (cooldown > 0) cooldown--;

        /* Check primary sensor detection */
        if (VL53L5CX_IsInsectDetected() && cooldown == 0) {
            if (g_capture_busy) continue;
            g_capture_busy = 1;
            g_sensor_state = SENSOR_STATE_PAUSED;
            VL53L5CX_DetectionResult_t res = VL53L5CX_GetResult();

#if PERF_DEBUG_LEVEL >= 1
            const char* trig_str;
            if (res.trigger_source == 1) trig_str = "SIGNAL";
            else if (res.trigger_source == 2) trig_str = "MOTION";
            else if (res.trigger_source == 3) trig_str = "SIGNAL+MOTION";
            else trig_str = "UNKNOWN";
            printf(">>> INSECT DETECTED [%s]!\n", trig_str);
#endif

#if TEST_TOF_MODE
            /* === ON-SITE TEST MODE: no camera, no SD — RED LED only ===
               Zone list shows WHERE in the FOV the target was, so sensor
               position/orientation can be validated. The WS2812 strip is
               not used in test mode. */
            printf(">>> TEST: %u zone(s) affected, %u valid\r\n",
                   (unsigned)res.affected_count, (unsigned)res.valid_measurements);
            for (int a = 0; a < res.affected_count; a++)
                printf("    zone %u -> %lu\r\n", (unsigned)res.affected_zones[a],
                       (unsigned long)res.affected_drop[a]);
            BSP_LED_Off(LED_GREEN); BSP_LED_On(LED_RED);
            vTaskDelay(pdMS_TO_TICKS(TEST_TOF_LED_MS));
            BSP_LED_Off(LED_RED); BSP_LED_On(LED_GREEN);
            g_sensor_state = SENSOR_STATE_RUNNING;
            g_capture_busy = 0;
            cooldown = 30;
            continue;
#else
            BSP_LED_Off(LED_GREEN); BSP_LED_On(LED_RED);
#if WS2812_MODE == 1
            WS2812_FlashStart(WS2812_ILLUMINATION_COLOR, WS2812_ILLUMINATION_BRIGHTNESS);
#elif WS2812_MODE == 0
            WS2812_TurnOn();
#else
            WS2812_TurnOn();
            vTaskDelay(pdMS_TO_TICKS(WS2812_INDICATOR_MS));
            WS2812_TurnOff();
#endif

            PerfTimer_t t;
            PERF_START(t);
            int rc = Capture_RequestSnapshot(60000);
            PERF_STOP(t);
            BSP_LED_Off(LED_RED); BSP_LED_On(LED_GREEN);
            VL53L5CX_StartRanging();
            g_sensor_state = SENSOR_STATE_RUNNING;
            g_capture_busy = 0;
            cooldown = 30;

            if (rc == 0) {
                capture_count++;
#if PERF_PRINT_SUMMARY
                Perf_PrintSummary(&g_perf_timer, capture_count);
                Perf_UpdateStats(&g_perf_timer);
#else
                printf("Snapshot #%lu SAVED (%lu ms)\n", (unsigned long)capture_count, (unsigned long)Perf_TotalElapsed(&t));
#endif
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[SENSOR] Capture FAILED rc=%d\n", rc);
#endif
            }
#endif /* TEST_TOF_MODE */
        }
#endif /* VL53L5CX_DUAL_SENSOR */

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


/* ==================== CAMERA TASK ==================== */
void camera_task(void *arg)
{
    (void)arg;
    extern volatile int system_ready;
    while (!system_ready) vTaskDelay(pdMS_TO_TICKS(500));

    /* NOTE:
       - CAPTURE_MODE==1: camera already initialized in main_thread (pipe running).
         camera_task only services ISP in idle loop and handles snap commands.
       - CAPTURE_MODE==2: camera_task starts the continuous pipe here. */
#if CAPTURE_MODE == 2
    CAM_ContinuousStart(capture_buf, MAX_SNAP_FRAME_SIZE, SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS);
#endif

    uint32_t frame_size = (uint32_t)SNAP_WIDTH * SNAP_HEIGHT * 2UL;

    while (1) {
        CameraCmd_t cmd = {0};
#if CAPTURE_MODE == 1 || CAPTURE_MODE == 2
        CAM_IspUpdate();
#endif
        if (xQueueReceive(camera_cmd_queue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) continue;

        if (cmd.type == CAM_CMD_SNAP) {
            Perf_Start(&g_perf_timer);
            g_perf_timer.sd_total_wait_ms = 0;
            g_perf_timer.sd_total_write_ms = 0;
            g_perf_timer.sd_total_gap_ms = 0;
            g_perf_timer.sd_batch_count = 0;
            g_perf_timer.sd_max_batch_ms = 0;
            g_perf_timer.sd_max_wait_ms = 0;
            int rc = -1;
            g_last_batch_frames = 0;

#if CAPTURE_MODE == 1
            extern uint8_t save_buf[];
            rc = CAM_ContinuousSnap(save_buf, frame_size);
            if (rc == 0) {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] OK %lu ms\n", (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_START, PERF_PHASE_CAM_DEINIT));
#endif
                xSemaphoreGive(camera_ready_sem);
                StorageCmd_t sc = {0};
                sc.type = STORAGE_CMD_SAVE; sc.image_buf = save_buf;
                sc.image_size = frame_size; sc.width = SNAP_WIDTH; sc.height = SNAP_HEIGHT;
                sc.pixel_format = 0; sc.snap_id = g_snap_count;
                xQueueSend(storage_cmd_queue, &sc, pdMS_TO_TICKS(1000));
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] FAIL\n");
#endif
                xSemaphoreGive(camera_ready_sem);
            }
#elif CAPTURE_MODE == 2 || CAPTURE_MODE == 4
            extern uint8_t batch_buf[];
#if CAPTURE_MODE == 2
            rc = CAM_ContinuousBatchSnap(batch_buf, frame_size);
#else
            /* CAPTURE_MODE == 4: Callback-Batch (continuous, NO stop/restart) */
            rc = CAM_CallbackBatchSnap(batch_buf, frame_size);
#endif
            if (rc > 0) {
                g_last_batch_frames = rc;
#if PERF_DEBUG_LEVEL >= 1
#if CAPTURE_MODE == 4
                printf("[CAM] Callback-Batch: %d frames captured (continuous)\n", rc);
#else
                printf("[CAM] Batch: %d frames captured\n", rc);
#endif
#endif
                xSemaphoreGive(camera_ready_sem);
                for (int f = 0; f < rc; f++) {
                    StorageCmd_t sc = {0};
                    sc.type = STORAGE_CMD_SAVE;
                    sc.image_buf = batch_buf + (f * frame_size);
                    sc.image_size = frame_size;
                    sc.width = SNAP_WIDTH; sc.height = SNAP_HEIGHT;
                    sc.pixel_format = 0; sc.snap_id = g_snap_count + (uint32_t)f;
                    if (xQueueSend(storage_cmd_queue, &sc, pdMS_TO_TICKS(1000)) != pdTRUE) {
#if PERF_DEBUG_LEVEL >= 1
                        printf("[CAM] Storage queue full, dropping frame %d\n", f);
#endif
                    }
                }
            } else {
#if PERF_DEBUG_LEVEL >= 1
#if CAPTURE_MODE == 4
                printf("[CAM] Callback-Batch FAIL\n");
#else
                printf("[CAM] Batch FAIL\n");
#endif
#endif
                xSemaphoreGive(camera_ready_sem);
            }
#else
            rc = CAM_CaptureSingleFrame(capture_buf, MAX_SNAP_FRAME_SIZE, SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS, SNAP_WARMUP_FRAMES);
            if (rc == 0) {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] OK %lu ms\n", (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_START, PERF_PHASE_CAM_DEINIT));
#endif
                xSemaphoreGive(camera_ready_sem);
                StorageCmd_t sc = {0};
                sc.type = STORAGE_CMD_SAVE; sc.image_buf = capture_buf;
                sc.image_size = frame_size; sc.width = SNAP_WIDTH; sc.height = SNAP_HEIGHT;
                sc.pixel_format = 0; sc.snap_id = g_snap_count;
                xQueueSend(storage_cmd_queue, &sc, pdMS_TO_TICKS(1000));
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] FAIL\n");
#endif
                xSemaphoreGive(camera_ready_sem);
            }
#endif
        }
    }
}

/* ==================== STORAGE TASK ==================== */
void storage_task(void *arg)
{
    (void)arg;
    extern volatile int system_ready;
    while (!system_ready) vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        StorageCmd_t cmd = {0};
        if (xQueueReceive(storage_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) continue;
        if (cmd.type == STORAGE_CMD_SAVE) {
            PERF_MARK(g_perf_timer, STORAGE);
            int rc = SD_StoreRawImage(cmd.image_buf, cmd.image_size, cmd.width, cmd.height, cmd.pixel_format, cmd.snap_id);
            if (rc != 0) {
                /* A single transient card hiccup (timeout / CRC) previously meant this
                   image was silently lost. Reinit the peripheral and retry the SAME
                   image ONCE before giving up, since g_sd_img_base_block was restored
                   to the block this image was supposed to start at. */
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] FAIL — attempting recovery + retry...\n");
#endif
                if (SD_Reinit() == 0) {
                    rc = SD_StoreRawImage(cmd.image_buf, cmd.image_size, cmd.width, cmd.height, cmd.pixel_format, cmd.snap_id);
                }
            }
            g_last_storage_rc = rc;
            PERF_MARK(g_perf_timer, DONE);
            if (rc == 0) {
                g_snap_count++;
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] OK %lu ms\n", (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_STORAGE, PERF_PHASE_DONE));
#endif
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] FAIL %lu ms — image LOST after retry\n", (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_STORAGE, PERF_PHASE_DONE));
#endif
                SD_Reinit();
            }
            xSemaphoreGive(storage_done_sem);
        }
    }
}

/* ==================== PIPELINE ==================== */
int Capture_RequestSnapshot(uint32_t timeout_ms)
{
    uint32_t t_start = HAL_GetTick();
    CameraCmd_t cmd = {0}; cmd.type = CAM_CMD_SNAP;
    g_last_storage_rc = 0;
    while (xSemaphoreTake(storage_done_sem, 0) == pdTRUE) {
        /* Drain stale completion tokens from previous capture cycles. */
    }
    if (xQueueSend(camera_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) return -2;

    TickType_t ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
    if (xSemaphoreTake(camera_ready_sem, ticks) != pdTRUE) return -1;

#if WS2812_MODE == 1
    WS2812_FlashStop();
#elif WS2812_MODE == 0
    WS2812_FlashStop();
#endif

#if CAPTURE_MODE == 2 || CAPTURE_MODE == 4
    /* Wait for ALL BATCH_FRAMES to be stored before returning */
    int frames_to_wait = g_last_batch_frames > 0 ? g_last_batch_frames : BATCH_FRAMES;
    for (int i = 0; i < frames_to_wait; i++) {
        if (timeout_ms > 0) {
            uint32_t elapsed = HAL_GetTick() - t_start;
            if (elapsed >= timeout_ms) return -1;
            ticks = pdMS_TO_TICKS(timeout_ms - elapsed);
        } else { ticks = portMAX_DELAY; }
        if (xSemaphoreTake(storage_done_sem, ticks) != pdTRUE) return -1;
    }
#else
    if (timeout_ms > 0) {
        uint32_t elapsed = HAL_GetTick() - t_start;
        if (elapsed >= timeout_ms) return -1;
        ticks = pdMS_TO_TICKS(timeout_ms - elapsed);
    } else { ticks = portMAX_DELAY; }
    if (xSemaphoreTake(storage_done_sem, ticks) != pdTRUE) return -1;
#endif
    if (g_last_storage_rc != 0) return -1;
    return 0;
}
