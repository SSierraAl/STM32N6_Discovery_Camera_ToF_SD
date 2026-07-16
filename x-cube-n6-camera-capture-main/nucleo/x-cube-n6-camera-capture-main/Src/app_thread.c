/**
 * @file    app_thread.c
 * @brief   Multi-threaded Camera + Sensor + Storage
 *
 *   SD STORE: Based on the PROVEN working implementation from main.c.
 *   Key: Must check BOTH card state AND HAL state. Must wait for card
 *   to be truly ready before each batch (not just rely on fixed delay).
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
#if CAPTURE_MODE == 2
extern uint8_t batch_buf[];
#endif

QueueHandle_t    camera_cmd_queue   = NULL;
QueueHandle_t    storage_cmd_queue  = NULL;
QueueHandle_t    sensor_event_queue = NULL;
SemaphoreHandle_t camera_ready_sem  = NULL;
SemaphoreHandle_t storage_done_sem  = NULL;
volatile SensorState_t g_sensor_state = SENSOR_STATE_IDLE;

static volatile int g_capture_busy = 0;
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

/* Shared performance timer */
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
    storage_done_sem = xSemaphoreCreateBinary();
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

/** Reinitialize the SD card after a fatal error.
    Preserves g_sd_img_base_block so existing photos are NOT overwritten.
    Returns 0 on success, -1 on failure. */
static int SD_Reinit(void)
{
#if PERF_DEBUG_LEVEL >= 1
    printf("[SD] === SD RECOVERY STARTED ===\n");
#endif

    /* Save current block offset BEFORE reinit (preserve existing photos!) */
    uint32_t saved_block = g_sd_img_base_block;

    /* Pause sensor task to prevent new capture requests during recovery */
    g_sensor_state = SENSOR_STATE_PAUSED;

    /* Deinit current SD */
    HAL_SD_Abort(&hsd1);
    HAL_SD_DeInit(&hsd1);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Full peripheral reset */
    __HAL_RCC_SDMMC2_FORCE_RESET();
    vTaskDelay(pdMS_TO_TICKS(10));
    __HAL_RCC_SDMMC2_RELEASE_RESET();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Reconfigure clock */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_SDMMC2;
    PeriphClkInit.Sdmmc2ClockSelection  = RCC_SDMMC2CLKSOURCE_HCLK;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /* Reinit SD */
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
        if (status == HAL_OK) {
            hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
        }
    }

    /* Restore saved block offset — CRITICAL: don't overwrite existing photos! */
    g_sd_img_base_block = saved_block;

    /* Resume sensor task */
    g_sensor_state = SENSOR_STATE_RUNNING;

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

/* ==================== SD STORE (PROVEN from main.c) ==================== */

/** Wait for SD card ready.
    This is the PROVEN approach from main.c that works reliably. */
static int SD_WaitForReady(void)
{
    uint32_t wait_ms = 0;
    uint32_t last_print = 0;

    /* CRITICAL: Must check BOTH conditions. HAL_SD_GetCardState() checks
       the card hardware state, while hsd1.State tracks the HAL driver
       state machine. Both must be ready. */
    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER && wait_ms < 2000) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_ms++;
    }

    if (wait_ms >= 2000) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] timeout before next batch\n");
#endif
        return -1;
    }
    return 0;
}

/** Store raw image to SD card.
    This implementation is based on the PROVEN working code from main.c.
    Changes: Added timing markers and progress prints only. */
static int SD_StoreRawImage(const uint8_t *img_buf, uint32_t img_size,
                            uint32_t w, uint32_t h, uint32_t pixel_format,
                            uint32_t snap_id)
{
    uint32_t         base = g_sd_img_base_block;
    uint32_t         batch_blocks = SD_BATCH_WRITE_BLOCKS;
    uint32_t         batch_size = batch_blocks * SD_BLOCK_SIZE;
    uint32_t         i, checksum = 0;

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
           snap_id, (unsigned long)w, (unsigned long)h, img_size,
           img_size / 1048576.0f);
#endif

    /* ---- PERF: Reset SD batch counters ---- */
    g_perf_timer.sd_total_wait_ms = 0;
    g_perf_timer.sd_total_write_ms = 0;
    g_perf_timer.sd_total_gap_ms = 0;
    g_perf_timer.sd_batch_count = 0;
    g_perf_timer.sd_max_batch_ms = 0;
    g_perf_timer.sd_max_wait_ms = 0;

    PERF_MARK(g_perf_timer, STORAGE);

    /* ---- Wait for card + HAL ready BEFORE first write ---- */
    uint32_t t_wait = HAL_GetTick();
    if (SD_WaitForReady() != 0) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] Not ready for first write\n");
#endif
        return -1;
    }
    uint32_t wait_ms = HAL_GetTick() - t_wait;

    /* ---- Step 1: Write header + first chunk ---- */
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
    uint32_t current_block = base + batch_blk;

    /* ---- Step 2: Write remaining image data in full batches ---- */
    while (img_offset < img_size) {
        uint32_t remaining_bytes = img_size - img_offset;
        uint32_t remaining_blocks_full = (remaining_bytes + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
        uint32_t blocks_in_batch = (remaining_blocks_full < batch_blocks)
                                    ? remaining_blocks_full : batch_blocks;

        /* Fill the batch buffer */
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

        /* Wait for card ready (PROVEN: checks card state via HAL) */
        t0 = HAL_GetTick();
        if (SD_WaitForReady() != 0) {
#if PERF_DEBUG_LEVEL >= 1
            printf("[SD] timeout before block %lu\n", (unsigned long)current_block);
#endif
            return -1;
        }
        wait_ms = HAL_GetTick() - t0;

        /* CRITICAL: SDXC cards need significant time between multi-block writes
           for internal flash programming and wear leveling. With 128-block batches
           (64KB each), 20ms is the minimum tested gap that is stable.
           10ms causes DCRCFAIL at random batches (~65% through). */
        t0 = HAL_GetTick();
        vTaskDelay(pdMS_TO_TICKS(15));
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

        /* Progress print (throttled) */
        uint32_t batch_num = g_perf_timer.sd_batch_count;
#if PERF_DEBUG_LEVEL >= 2
        if (batch_num % PERF_SD_BATCH_PRINT_EVERY == 0) {
            uint32_t done_bytes = img_offset + blocks_in_batch * SD_BLOCK_SIZE;
            float pct = (100.0f * done_bytes) / img_size;
            printf("[SD] #%lu wait=%lums write=%lums gap=%lums | %.1f%%\n",
                   (unsigned long)batch_num,
                   (unsigned long)wait_ms,
                   (unsigned long)write_ms,
                   (unsigned long)gap_ms, pct);
        }
#elif PERF_DEBUG_LEVEL >= 1
        if (batch_num % 64 == 0) {
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
           (unsigned long)base, (unsigned long)(current_block - 1),
           (unsigned long)g_perf_timer.sd_batch_count);
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
    VL53L5CX_Configure(VL53L5CX_RESOLUTION_8X8, 800, 15);
#else
    VL53L5CX_Configure(VL53L5CX_RESOLUTION_4X4, 800, 15);
#endif
    VL53L5CX_StartRanging();
    VL53L5CX_LearnBaseline();
    if (!VL53L5CX_IsBaselineReady()) {
        g_sensor_state = SENSOR_STATE_STOPPED;
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    g_sensor_state = SENSOR_STATE_RUNNING;
    printf("[SENSOR] Running\n");

    uint32_t capture_count = 0, cooldown = 0;

    while (1) {
        if (g_sensor_state == SENSOR_STATE_PAUSED) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        if (!VL53L5CX_Update()) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        g_debug_frame_count++;
        if (g_debug_frame_count >= 1) g_debug_frame_count = 0;
        if (cooldown > 0) cooldown--;

        if (VL53L5CX_IsInsectDetected() && cooldown == 0) {
            if (g_capture_busy) continue;
            g_capture_busy = 1;
            g_sensor_state = SENSOR_STATE_PAUSED;
            (void)VL53L5CX_GetResult();

#if PERF_DEBUG_LEVEL >= 1
            printf(">>> INSECT DETECTED!\n");
#endif
            BSP_LED_Off(LED_GREEN); BSP_LED_On(LED_RED);

#if WS2812_MODE == 1  /* CAPTURE mode: illuminate DURING full capture + SD write */
            /* Turn LEDs ON before capture, keep them ON until snapshot is saved to SD */
            WS2812_FlashStart(WS2812_ILLUMINATION_COLOR,
                              WS2812_ILLUMINATION_BRIGHTNESS);
#elif WS2812_MODE == 0  /* ALWAYS_ON */
            WS2812_TurnOn();
#else  /* INDICATOR only */
            WS2812_TurnOn();
            vTaskDelay(pdMS_TO_TICKS(WS2812_INDICATOR_MS));
            WS2812_TurnOff();
#endif

            PerfTimer_t t;
            PERF_START(t);

            int rc = Capture_RequestSnapshot(60000);

            PERF_STOP(t);

            /* Turn off illumination AFTER capture + SD write is complete */
#if WS2812_MODE == 1
            WS2812_FlashStop();
#elif WS2812_MODE == 0
            WS2812_TurnOff();
#endif
            BSP_LED_Off(LED_RED); BSP_LED_On(LED_GREEN);
            g_sensor_state = SENSOR_STATE_RUNNING;
            g_capture_busy = 0;
            cooldown = 30;

            if (rc == 0) {
                capture_count++;
#if PERF_PRINT_SUMMARY
                PERF_PRINT(t, capture_count);
                PERF_STATS(t);
#else
                printf("Snapshot #%lu SAVED (%lu ms)\n",
                       (unsigned long)capture_count, (unsigned long)Perf_TotalElapsed(&t));
#endif
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[SENSOR] Capture FAILED rc=%d\n", rc);
#endif
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ==================== CAMERA TASK ==================== */
void camera_task(void *arg)
{
    (void)arg;
    extern volatile int system_ready;
    while (!system_ready) vTaskDelay(pdMS_TO_TICKS(500));

#if CAPTURE_MODE == 1 || CAPTURE_MODE == 2
    CAM_ContinuousStart(capture_buf, MAX_SNAP_FRAME_SIZE, SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS);
#endif
    uint32_t frame_size = (uint32_t)SNAP_WIDTH * SNAP_HEIGHT * 2UL;

    while (1) {
        CameraCmd_t cmd = {0};
#if CAPTURE_MODE == 1
        CAM_IspUpdate();
#endif
        if (xQueueReceive(camera_cmd_queue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) continue;

        if (cmd.type == CAM_CMD_SNAP) {
            PERF_MARK(g_perf_timer, START);
            int rc = -1;
            const uint8_t *frame_to_save = NULL;

#if CAPTURE_MODE == 1
            extern uint8_t save_buf[];
            rc = CAM_ContinuousSnap(save_buf, frame_size);
            if (rc == 0) frame_to_save = save_buf;
#elif CAPTURE_MODE == 2
            int captured_frame_count = CAM_ContinuousBatchSnap(batch_buf, frame_size);
            rc = (captured_frame_count > 0) ? 0 : -1;
            if (rc == 0) {
                uint32_t frame_offset = 0;
                for (int frame_idx = 0; frame_idx < captured_frame_count; frame_idx++) {
                    const uint8_t *frame = batch_buf + frame_offset;
                    uint32_t snap_id = g_snap_count;
                    if (SD_StoreRawImage(frame, frame_size, SNAP_WIDTH, SNAP_HEIGHT, 0, snap_id) == 0) {
                        g_snap_count++;
                        frame_offset += frame_size;
                    } else {
                        rc = -1;
                        break;
                    }
                }
            }
#else
            rc = CAM_CaptureSingleFrame(capture_buf, MAX_SNAP_FRAME_SIZE,
                                        SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS, SNAP_WARMUP_FRAMES);
            if (rc == 0) frame_to_save = capture_buf;
#endif

            if (rc == 0) {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] OK %lu ms\n",
                       (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_START, PERF_PHASE_CAM_DEINIT));
#endif
                xSemaphoreGive(camera_ready_sem);
#if CAPTURE_MODE == 2
                xSemaphoreGive(storage_done_sem);
#else
                StorageCmd_t sc = {0};
                sc.type = STORAGE_CMD_SAVE; sc.image_buf = frame_to_save;
                sc.image_size = frame_size; sc.width = SNAP_WIDTH; sc.height = SNAP_HEIGHT;
                sc.pixel_format = 0; sc.snap_id = g_snap_count;
                xQueueSend(storage_cmd_queue, &sc, pdMS_TO_TICKS(1000));
#endif
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] FAIL\n");
#endif
                xSemaphoreGive(camera_ready_sem);
#if CAPTURE_MODE == 2
                xSemaphoreGive(storage_done_sem);
#endif
            }
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
            int rc = SD_StoreRawImage(cmd.image_buf, cmd.image_size,
                                      cmd.width, cmd.height, cmd.pixel_format, cmd.snap_id);
            PERF_MARK(g_perf_timer, DONE);
            if (rc == 0) {
                g_snap_count++;
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] OK %lu ms\n",
                       (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_STORAGE, PERF_PHASE_DONE));
#endif
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] FAIL %lu ms — attempting recovery...\n",
                       (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_STORAGE, PERF_PHASE_DONE));
#endif
                /* Auto-recover: reinit SD card, preserve existing photos */
                int rec = SD_Reinit();
                if (rec == 0) {
#if PERF_DEBUG_LEVEL >= 1
                    printf("[SD] Recovery OK — next capture will retry.\n");
#endif
                }
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
    if (xQueueSend(camera_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) return -2;

    TickType_t ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
    if (xSemaphoreTake(camera_ready_sem, ticks) != pdTRUE) return -1;

    if (timeout_ms > 0) {
        uint32_t elapsed = HAL_GetTick() - t_start;
        if (elapsed >= timeout_ms) return -1;
        ticks = pdMS_TO_TICKS(timeout_ms - elapsed);
    } else { ticks = portMAX_DELAY; }

    if (xSemaphoreTake(storage_done_sem, ticks) != pdTRUE) return -1;
    return 0;
}
