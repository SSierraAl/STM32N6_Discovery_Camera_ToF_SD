#include "raw_image.h"
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
#include "image_filter.h"
#include "utils.h"

/* Detection frame buffer (640x480 YUV422 = 614 KB) in PSRAM */
static uint8_t detection_frame[IF_DETECT_FRAME_SZ] ALIGN_32 IN_PSRAM;

/* Bound repeated camera activations in every detector mode. This includes the
   high-sensitivity 4x4 camera path: after the configured number of photos, a
   full baseline refresh clears weak-track noise instead of allowing an
   unbounded capture loop. */
#if VL53L5CX_DET_ADAPTIVE_REFRESH_ENABLED > 0
#define TOF_CAPTURE_ACTIVATION_REFRESH 1
#if !VL53L5CX_DUAL_SENSOR && !TEST_TOF_MODE && \
    (VL53L5CX_DET_RESOLUTION == 4) && VL53L5CX_DET_HIGH_SENS_CAMERA
#define TOF_CAPTURE_REFRESH_WINDOW_SECS VL53L5CX_DET_HIGH_SENS_REFRESH_WINDOW_SECS
#define TOF_CAPTURE_MAX_DETECTIONS      VL53L5CX_DET_HIGH_SENS_MAX_DETECTIONS
#define TOF_CAPTURE_REARM_HOLDOFF_SECS  VL53L5CX_DET_HIGH_SENS_REARM_HOLDOFF_SECS
#define TOF_CAPTURE_COOLDOWN_FRAMES      VL53L5CX_DET_HIGH_SENS_CAPTURE_COOLDOWN_FRAMES
#else
#define TOF_CAPTURE_REFRESH_WINDOW_SECS VL53L5CX_DET_REFRESH_WINDOW_SECS
#define TOF_CAPTURE_MAX_DETECTIONS      VL53L5CX_DET_MAX_DETECTIONS
#define TOF_CAPTURE_REARM_HOLDOFF_SECS  0
#define TOF_CAPTURE_COOLDOWN_FRAMES      5U
#endif
#else
#define TOF_CAPTURE_ACTIVATION_REFRESH 0
#define TOF_CAPTURE_REARM_HOLDOFF_SECS  0
#define TOF_CAPTURE_COOLDOWN_FRAMES      5U
#endif

extern I2C_HandleTypeDef hi2c1;
extern uint8_t capture_buf[];
#if CAPTURE_MODE == 1
extern uint8_t save_buf[];
#endif
#if CAPTURE_MODE == 4
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
static volatile int g_stage3_crops = 0;
/* Camera task reserves IDs before dispatch; failed writes leave an ID gap. */
static uint32_t g_next_snap_id = 0;
static uint32_t g_sd_img_base_block;
#define SD_IMG_HEADER_SIZE  64
#define SD_BLOCK_SIZE       512
static uint32_t g_debug_frame_count = 0;



extern SD_HandleTypeDef hsd1;
extern uint8_t sd_batch_buf[];
PerfTimer_t g_perf_timer;

/* ================================================================
   MULTI-RESOLUTION CAPTURE STATE (Phase 3)
   ================================================================ */
typedef enum {
    CAPTURE_STAGE_IDLE = 0,
    CAPTURE_STAGE1_DETECT,       /* 640×480 downscale + image_filter */
    CAPTURE_STAGE2_FULLFRAME,    /* 1296×972 full-frame save */
    CAPTURE_STAGE3_ROI,          /* AREA3 ROI capture */
    CAPTURE_STAGE_RESTORE,       /* Restore full-frame */
    CAPTURE_STAGE_DONE,
} CaptureStage_t;

static volatile CaptureStage_t g_capture_stage = CAPTURE_STAGE_IDLE;
static IF_Result_t g_detection_result;
static int g_stage1_detected = 0;

/* Software downscale: 1296×972 YUYV -> 640×480 YUYV (2× decimate + center crop)
   1296/2 = 648, center-crop to 640 (offset 4 pixel pairs from left)
   972/2  = 486, center-crop to 480 (offset 3 rows from top) */
static void downscale_1296x972_to_640x480(const uint8_t *src, uint8_t *dst)
{
    int src_row_bytes = 1296 * 2;
    int dst_row_bytes = 640 * 2;
    printf("[MULTI] Downscale 1296x972 -> 640x480 (2x decimate + center crop)\n");
    for (int r = 0; r < 480; r++) {
        int src_r = r * 2 + 3;
        for (int g = 0; g < 320; g++) {
            int src_g = g + 4;
            int si = src_r * src_row_bytes + src_g * 4;
            int di = r * dst_row_bytes + g * 4;
            dst[di + 0] = src[si + 0];
            dst[di + 1] = src[si + 1];
            dst[di + 2] = src[si + 2];
            dst[di + 3] = src[si + 3];
        }
    }
    SCB_InvalidateDCache_by_Addr((uint32_t *)dst, IF_DETECT_FRAME_SZ);
    printf("[MULTI] Downscale done (%d bytes)\n", IF_DETECT_FRAME_SZ);
}

/* DS3231 gives UTC at one-second resolution. Mode 4 keeps that UTC anchor
   and replaces only capture_tick with the exact completion tick of each frame. */
static RTC_Stamp RTC_StampForFrame(RTC_Stamp stamp, uint32_t frame_tick)
{
    if (frame_tick != 0U) stamp.capture_tick = frame_tick;
    return stamp;
}

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

static int SD_StoreRawImage(const uint8_t *img_buf, uint32_t img_size, uint32_t w, uint32_t h, uint32_t pixel_format, uint32_t snap_id, RTC_Stamp stamp)
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

    uint32_t checksum_start = HAL_GetTick();
    for (i = 0; i < img_size; i++) checksum ^= img_buf[i];
    g_perf_timer.sd_checksum_ms += HAL_GetTick() - checksum_start;

    sd_image_header_t hdr = {0};
    hdr.magic = SD_HEADER_TAG; hdr.width = w; hdr.height = h;
    hdr.pixel_format = pixel_format; hdr.data_size = img_size;
    RawImage_SetStamp(&hdr, stamp); hdr.checksum = checksum;
    hdr.snap_id = snap_id; memset(hdr.reserved, 0, sizeof(hdr.reserved));

#if PERF_DEBUG_LEVEL >= 1
    printf("[SD] #%" PRIu32 " %lux%lu %lu bytes (%.3f MB)\n",
           snap_id, (unsigned long)w, (unsigned long)h, img_size, img_size / 1048576.0f);
#endif

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

    Perf_SD_RecordBatch(&g_perf_timer, wait_ms, write_ms, 0, st == HAL_OK ? batch_blk : 0);
    if (st != HAL_OK) {
#if PERF_DEBUG_LEVEL >= 1
        printf("[SD] header FAIL block %lu HAL=0x%08lX STA=0x%08lX\n",
               (unsigned long)base, (unsigned long)st, (unsigned long)SDMMC2->STA);
#endif
        return -1;
    }

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

        Perf_SD_RecordBatch(&g_perf_timer, wait_ms, write_ms, gap_ms, st == HAL_OK ? blocks_in_batch : 0);
        if (st != HAL_OK) {
#if PERF_DEBUG_LEVEL >= 1
            printf("[SD] FAIL block %lu HAL=0x%08lX STA=0x%08lX\n",
                   (unsigned long)current_block, (unsigned long)st, (unsigned long)SDMMC2->STA);
#endif
            return -1;
        }

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
    /* Cycle completion belongs to Capture_RequestSnapshot, after all frames. */
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
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY
    printf("[TOF TEST] sensor_task v3 started\n");
#endif

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

    /* Announce the selected sleep/wake baseline refresh strategy so
       the console makes it obvious when the cycle refresh is off. */
#if VL53L5CX_DUAL_BASELINE_MODE == VL53L5CX_BASELINE_NO_REFRESH
    printf("[PRIMARY] Baseline mode: NO_REFRESH - cycle refresh DISABLED\n");
#elif VL53L5CX_DUAL_BASELINE_MODE == VL53L5CX_BASELINE_PRE_SLEEP
    printf("[PRIMARY] Baseline mode: PRE_SLEEP - re-learn before sleep\n");
#else
    printf("[PRIMARY] Baseline mode: QUICK_WAKE - re-learn on wake\n");
#endif
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
#if TOF_CAPTURE_ACTIVATION_REFRESH
    uint8_t consecutive_captures = 0;
    uint8_t consecutive_window_active = 0;
    TickType_t consecutive_window_start = 0;
#endif
#if TOF_CAPTURE_REARM_HOLDOFF_SECS > 0
    uint8_t capture_rearm_holdoff_active = 0;
    TickType_t capture_rearm_holdoff_start = 0;
#endif
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && !VL53L5CX_DUAL_SENSOR && \
    (VL53L5CX_DET_RESOLUTION == 4) && \
    (VL53L5CX_DET_ZONE_LOG_INTERVAL_MS > 0)
    TickType_t zone_log_last = xTaskGetTickCount();
#endif

    while (1) {
        if (g_sensor_state == SENSOR_STATE_PAUSED) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

#if TEST_TOF_MODE
        /* ---- Manual baseline refresh via USER button (PC13) ----
           TEST build only: one press = one full refresh of the primary
           baseline and, in dual mode, the external guardian baseline as
           well (see VL53L5CX_RefreshBaseline_Manual()). Debounced and
           edge-triggered — re-arms once the button is released. The RED
           LED stays on for the whole refresh (a few seconds). */
        static uint8_t btn_refresh_armed = 1;
        if (btn_refresh_armed &&
            HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
            vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
                btn_refresh_armed = 0;
                printf("[BTN] PC13 pressed — refreshing baseline(s)...\n");
                BSP_LED_Off(LED_GREEN);
                BSP_LED_On(LED_RED);
                VL53L5CX_RefreshBaseline_Manual();
                BSP_LED_Off(LED_RED);
                BSP_LED_On(LED_GREEN);
                while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
                    vTaskDelay(pdMS_TO_TICKS(50));
                btn_refresh_armed = 1;
            }
        }
#endif

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

#if TOF_CAPTURE_ACTIVATION_REFRESH
            if (consecutive_window_active &&
                (xTaskGetTickCount() - consecutive_window_start) >=
                    pdMS_TO_TICKS(TOF_CAPTURE_REFRESH_WINDOW_SECS * 1000UL)) {
                consecutive_captures = 0;
                consecutive_window_active = 0;
                printf("[ADAPT] Window expired, counter reset\n");
            }
#endif

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
                else if (res.trigger_source == 4) trig_str = "DISTANCE";
                else if (res.trigger_source == 5) trig_str = "SIGNAL+DISTANCE";
                else if (res.trigger_source == 6) trig_str = "MOTION+DISTANCE";
                else if (res.trigger_source == 7) trig_str = "SIGNAL+MOTION+DISTANCE";
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

#if TOF_CAPTURE_ACTIVATION_REFRESH
                if (consecutive_window_active) {
                    consecutive_captures++;
                } else {
                    consecutive_captures = 1;
                }
                consecutive_window_active = 0;
                printf("[ADAPT] Camera activation %u/%u\n",
                       (unsigned)consecutive_captures,
                       (unsigned)TOF_CAPTURE_MAX_DETECTIONS);
#endif

                PerfTimer_t t;
                PERF_START(t);
                int rc = Capture_RequestSnapshot(60000);
                PERF_STOP(t);
                BSP_LED_Off(LED_RED); BSP_LED_On(LED_GREEN);

                /* Wait for Stage 3 (ROI capture + SD save) to complete.
                   g_capture_busy is cleared by camera callback after Stage 3
                   finishes all captures and saves (lines 1135-1137).  Timeout
                   after 30 s to prevent deadlock if capture failed. */
                {
                    TickType_t wstart = xTaskGetTickCount();
                    while (g_capture_busy) {
                        if ((xTaskGetTickCount() - wstart) > pdMS_TO_TICKS(30000)) break;
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                }

#if TOF_CAPTURE_ACTIVATION_REFRESH
                if (consecutive_captures >= TOF_CAPTURE_MAX_DETECTIONS) {
                    printf("[ADAPT] Maximum activations reached, refreshing baseline\n");
                    VL53L5CX_StopRanging();
                    vTaskDelay(pdMS_TO_TICKS(50));
                    VL53L5CX_StartRanging();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    VL53L5CX_LearnBaseline();
                    consecutive_captures = 0;
                    consecutive_window_active = 0;
                    printf("[ADAPT] Baseline refresh complete\n");
                } else {
                    VL53L5CX_StartRanging();
                    consecutive_window_start = xTaskGetTickCount();
                    consecutive_window_active = 1;
                    printf("[ADAPT] Window opened for %lu s\n",
                           (unsigned long)TOF_CAPTURE_REFRESH_WINDOW_SECS);
                }
#else
                VL53L5CX_StartRanging();
#endif
                g_sensor_state = SENSOR_STATE_RUNNING;
                g_capture_busy = 0;
                cooldown = 5;

                if (rc == 0) {
                    capture_count++;
#if PERF_PRINT_SUMMARY
                    Perf_PrintSummary(&g_perf_timer, capture_count);
                    Perf_UpdateStats(&g_perf_timer);
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

#if !TEST_TOF_MODE && (WS2812_MODE == 0 || WS2812_MODE == 1)
        /* WS2812 has no readback. While the camera is idle, periodically
           overwrite any misdecoded residual colour with redundant OFF frames. */
        WS2812_OffWatchdog();
#endif

        g_debug_frame_count++;
        if (g_debug_frame_count >= 1) g_debug_frame_count = 0;
        if (cooldown > 0) cooldown--;

#if TOF_CAPTURE_REARM_HOLDOFF_SECS > 0
        if (capture_rearm_holdoff_active &&
            (xTaskGetTickCount() - capture_rearm_holdoff_start) >=
                pdMS_TO_TICKS(TOF_CAPTURE_REARM_HOLDOFF_SECS * 1000UL)) {
            capture_rearm_holdoff_active = 0;
            /* Discard weak evidence accumulated while photos were blocked.
               Preserve the learned baseline and the currently active weak
               zones so persistent 2% noise is not treated as a new edge. */
            VL53L5CX_ZoneDetectorAfterCapture();
            printf("[ADAPT] Camera re-armed; weak track cleared\n");
        }
#endif

#if TOF_CAPTURE_ACTIVATION_REFRESH
        if (consecutive_window_active &&
            (xTaskGetTickCount() - consecutive_window_start) >=
                pdMS_TO_TICKS(TOF_CAPTURE_REFRESH_WINDOW_SECS * 1000UL)) {
            consecutive_captures = 0;
            consecutive_window_active = 0;
            printf("[ADAPT] Window expired, counter reset\n");
        }
#endif

        /* Evaluate every fresh frame, including cooldown, so a persistent
           zone remains latched in both the ToF test and camera modes. */
#if !VL53L5CX_DUAL_SENSOR && (VL53L5CX_DET_RESOLUTION == 4) && \
    ((TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_TEST) || \
     (!TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_CAMERA))
        uint8_t tof_event_policy = 0U;
        if (cooldown == 0)
            tof_event_policy = VL53L5CX_TEST_EVENT_ALLOW_ALL;
#if TOF_CAPTURE_REARM_HOLDOFF_SECS > 0
        if (capture_rearm_holdoff_active && cooldown == 0)
            tof_event_policy = (VL53L5CX_TEST_EVENT_ALLOW_LEVEL |
                                VL53L5CX_TEST_EVENT_ALLOW_FAST);
#endif
        const int insect_detected = VL53L5CX_TestDetectionStep(tof_event_policy);
#else
        const int insect_detected = VL53L5CX_IsInsectDetected();
#endif
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && \
    (VL53L5CX_DET_RESOLUTION == 4) && \
    (VL53L5CX_DET_ZONE_LOG_INTERVAL_MS > 0)
        TickType_t zone_log_now = xTaskGetTickCount();
        if ((zone_log_now - zone_log_last) >=
            pdMS_TO_TICKS(VL53L5CX_DET_ZONE_LOG_INTERVAL_MS)) {
            zone_log_last = zone_log_now;
            VL53L5CX_PrintZoneSnapshot("periodic");
        }
#endif
#if !VL53L5CX_DUAL_SENSOR && (VL53L5CX_DET_RESOLUTION == 4) && \
    ((TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_TEST) || \
     (!TEST_TOF_MODE && VL53L5CX_DET_HIGH_SENS_CAMERA))
        if (cooldown == 0 && !insect_detected &&
            (VL53L5CX_TakeBaselineRefreshRequest() ||
             VL53L5CX_TestTakeBaselineRefreshRequest())) {
#else
        if (cooldown == 0 && VL53L5CX_TakeBaselineRefreshRequest()) {
#endif
#if TEST_TOF_MODE && VL53L5CX_DET_ZONE_SURVEY && \
    (VL53L5CX_DET_RESOLUTION == 4)
            VL53L5CX_PrintZoneSnapshot("drift");
#endif
            printf("[ADAPT] Persistent stable drift, refreshing baseline\n");
            g_sensor_state = SENSOR_STATE_PAUSED;
            VL53L5CX_StopRanging();
            vTaskDelay(pdMS_TO_TICKS(50));
            VL53L5CX_StartRanging();
            vTaskDelay(pdMS_TO_TICKS(200));
            VL53L5CX_LearnBaseline();
#if TOF_CAPTURE_ACTIVATION_REFRESH
            consecutive_captures = 0;
            consecutive_window_active = 0;
#endif
            g_sensor_state = SENSOR_STATE_RUNNING;
            cooldown = TOF_CAPTURE_COOLDOWN_FRAMES;
            printf("[ADAPT] Stable-drift baseline refresh complete\n");
#if TOF_CAPTURE_REARM_HOLDOFF_SECS > 0
            capture_rearm_holdoff_start = xTaskGetTickCount();
            capture_rearm_holdoff_active = 1;
            printf("[ADAPT] Level/track recovery for %lu s; ToF remains active\n",
                   (unsigned long)TOF_CAPTURE_REARM_HOLDOFF_SECS);
#endif
            continue;
        }

        /* Check primary sensor detection */
        if (insect_detected && cooldown == 0) {
            if (g_capture_busy) continue;
            g_capture_busy = 1;
            g_sensor_state = SENSOR_STATE_PAUSED;
            VL53L5CX_DetectionResult_t res = VL53L5CX_GetResult();

#if PERF_DEBUG_LEVEL >= 1
            const char* trig_str;
            if (res.trigger_source == 1) trig_str = "SIGNAL";
            else if (res.trigger_source == 2) trig_str = "MOTION";
            else if (res.trigger_source == 3) trig_str = "SIGNAL+MOTION";
            else if (res.trigger_source == 4) trig_str = "DISTANCE";
            else if (res.trigger_source == 5) trig_str = "SIGNAL+DISTANCE";
            else if (res.trigger_source == 6) trig_str = "MOTION+DISTANCE";
            else if (res.trigger_source == 7) trig_str = "SIGNAL+MOTION+DISTANCE";
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
#if VL53L5CX_DET_ZONE_SURVEY && (VL53L5CX_DET_RESOLUTION == 4)
            VL53L5CX_PrintZoneSnapshot("trigger");
#if VL53L5CX_DET_ZONE_LOG_INTERVAL_MS > 0
            zone_log_last = xTaskGetTickCount();
#endif
#endif
            g_sensor_state = SENSOR_STATE_RUNNING;
            g_capture_busy = 0;
            cooldown = 30;
            continue;
#else
            /* Keep the event class for diagnostics, but count every completed
               camera activation. Three captures with no 30 s quiet gap force
               a baseline refresh, regardless of which detector path fired. */
#if !VL53L5CX_DUAL_SENSOR && (VL53L5CX_DET_RESOLUTION == 4) && \
    VL53L5CX_DET_HIGH_SENS_CAMERA
            const uint8_t tof_event_class = VL53L5CX_TestGetLastEventClass();
#else
            const uint8_t tof_event_class = 0U;
#endif
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

#if TOF_CAPTURE_ACTIVATION_REFRESH
            if (consecutive_window_active) {
                consecutive_captures++;
            } else {
                consecutive_captures = 1;
            }
            consecutive_window_active = 0;
            printf("[ADAPT] Camera activation %u/%u class=%u\n",
                   (unsigned)consecutive_captures,
                   (unsigned)TOF_CAPTURE_MAX_DETECTIONS,
                   (unsigned)tof_event_class);
#endif

            PerfTimer_t t;
            PERF_START(t);
            int rc = Capture_RequestSnapshot(60000);
            PERF_STOP(t);
#if VL53L5CX_DET_HIGH_SENS_CAMERA && (VL53L5CX_DET_RESOLUTION == 4)
            VL53L5CX_ZoneDetectorAfterCapture();
#endif
            /* sensor_task was blocked throughout camera/SD work. Do not compare
               the first new ToF sample with stale pre-capture history.
               ZoneDetectorAfterCapture already clears those comparisons while
               preserving active latches; a full reset here caused the same
               persistent evidence to trigger again after every photo. */
            BSP_LED_Off(LED_RED); BSP_LED_On(LED_GREEN);

            /* Wait for Stage 3 (ROI capture + SD save) to complete.
               g_capture_busy is cleared by camera callback after Stage 3
               finishes all captures and saves.  Timeout after 30 s to
               prevent deadlock if capture failed. */
            {
                TickType_t wstart = xTaskGetTickCount();
                while (g_capture_busy) {
                    if ((xTaskGetTickCount() - wstart) > pdMS_TO_TICKS(30000)) break;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

#if TOF_CAPTURE_ACTIVATION_REFRESH
            if (consecutive_captures >= TOF_CAPTURE_MAX_DETECTIONS) {
                printf("[ADAPT] Maximum activations reached, refreshing baseline\n");
                VL53L5CX_StopRanging();
                vTaskDelay(pdMS_TO_TICKS(50));
                VL53L5CX_StartRanging();
                vTaskDelay(pdMS_TO_TICKS(200));
                VL53L5CX_LearnBaseline();
                consecutive_captures = 0;
                consecutive_window_active = 0;
                printf("[ADAPT] Baseline refresh complete\n");
#if TOF_CAPTURE_REARM_HOLDOFF_SECS > 0
                capture_rearm_holdoff_start = xTaskGetTickCount();
                capture_rearm_holdoff_active = 1;
                printf("[ADAPT] Level/track recovery for %lu s; ToF remains active\n",
                       (unsigned long)TOF_CAPTURE_REARM_HOLDOFF_SECS);
#endif
            } else {
                vTaskDelay(pdMS_TO_TICKS(800)); /* let temp sensor settle before resume */
                VL53L5CX_StartRanging();
                consecutive_window_start = xTaskGetTickCount();
                consecutive_window_active = 1;
                printf("[ADAPT] Window opened for %lu s\n",
                       (unsigned long)TOF_CAPTURE_REFRESH_WINDOW_SECS);
            }
#else
            vTaskDelay(pdMS_TO_TICKS(800));
            VL53L5CX_StartRanging();
#endif
            g_sensor_state = SENSOR_STATE_RUNNING;
            g_capture_busy = 0;
            cooldown = TOF_CAPTURE_COOLDOWN_FRAMES;

            if (rc == 0) {
                capture_count++;
#if PERF_PRINT_SUMMARY
                Perf_PrintSummary(&g_perf_timer, capture_count);
                Perf_UpdateStats(&g_perf_timer);
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


    uint32_t frame_size = (uint32_t)SNAP_WIDTH * SNAP_HEIGHT * 2UL;

    while (1) {
        CameraCmd_t cmd = {0};
#if CAPTURE_MODE == 1
        CAM_IspUpdate();
#endif
        if (xQueueReceive(camera_cmd_queue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) continue;

        if (cmd.type == CAM_CMD_SNAP) {
            PERF_MARK(g_perf_timer, CAM_INIT);
            int rc = -1;
            g_last_batch_frames = 0;

#if CAPTURE_MODE == 1
            extern uint8_t save_buf[];
            rc = CAM_ContinuousSnap(save_buf, frame_size);
            PERF_MARK(g_perf_timer, CAM_END);
            if (rc == 0) {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] OK %lu ms\n", (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_START, PERF_PHASE_CAM_DEINIT));
#endif
                xSemaphoreGive(camera_ready_sem);
                StorageCmd_t sc = {0};
                sc.type = STORAGE_CMD_SAVE; sc.image_buf = save_buf;
                sc.image_size = frame_size; sc.width = SNAP_WIDTH; sc.height = SNAP_HEIGHT;
                sc.pixel_format = 0; sc.snap_id = g_next_snap_id++;
                sc.stamp = RTC_CaptureStamp();
                xQueueSend(storage_cmd_queue, &sc, pdMS_TO_TICKS(1000));
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] FAIL\n");
#endif
                xSemaphoreGive(camera_ready_sem);
            }
#elif CAPTURE_MODE == 4
            extern uint8_t batch_buf[];
#if CAPTURE_MODE == 4
            /* CAPTURE_MODE == 4: Callback-Batch (continuous, NO stop/restart) */
            rc = CAM_CallbackBatchSnap(batch_buf, frame_size);
            /* PIPE1 is stopped inside CAM_CallbackBatchSnap + sensor in standby */
            PERF_MARK(g_perf_timer, CAM_END);
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

                /* ================================================================
                   STAGE 1: Multi-resolution object detection (Phase 3)
                   Takes the first captured frame, downscales to 640×480, runs
                   image_filter detection pipeline, prints comprehensive results.
                   ================================================================ */
                g_capture_stage = CAPTURE_STAGE1_DETECT;
                printf("\n========== [MULTI] === STAGE 1: Low-Res Detection === ==========\n");
                printf("[MULTI] Frame %dx%d YUV422 -> downscale -> IF_Process\n",
                       SNAP_WIDTH, SNAP_HEIGHT);
                uint32_t t_detect = HAL_GetTick();
                downscale_1296x972_to_640x480(batch_buf, detection_frame);

                /* Y-plane stats: verify camera data is real */
                {
                    uint32_t y_sum = 0;
                    uint8_t y_min = 255, y_max = 0;
                    int y_n = 0;
                    for (int y = 0; y < IF_DETECT_H; y += 8) {
                        for (int x = 0; x < IF_DETECT_W; x += 8) {
                            uint8_t yv = detection_frame[y * IF_DETECT_W * 2 + x * 2];
                            y_sum += yv; y_n++;
                            if (yv < y_min) y_min = yv;
                            if (yv > y_max) y_max = yv;
                        }
                    }
                    printf("[MULTI] Y-plane (mean/min/max): %u / %u / %u\n",
                           (unsigned)(y_sum / y_n), y_min, y_max);
                }

                /* Grid heatmap (80x60 cells, subsampled 4x -> 20x15 chars) */
                {
                    IF_Grid_t grid;
                    IF_Downscale(detection_frame, &grid);
                    uint32_t g_sum = 0;
                    uint8_t g_min = 255, g_max = 0;
                    for (int gy = 0; gy < IF_GRID_H; gy++) {
                        for (int gx = 0; gx < IF_GRID_W; gx++) {
                            uint8_t gv = grid.grid[gy][gx];
                            g_sum += gv;
                            if (gv < g_min) g_min = gv;
                            if (gv > g_max) g_max = gv;
                        }
                    }
                    printf("[MULTI] Grid 80x60 (mean/min/max): %u / %u / %u\n",
                           (unsigned)(g_sum / (IF_GRID_H * IF_GRID_W)), g_min, g_max);
                    printf("[MULTI] Heatmap ('.'=dark, '%%'=bright):\n");
                    for (int gy = 0; gy < IF_GRID_H; gy += 4) {
                        printf("[MULTI] ");
                        for (int gx = 0; gx < IF_GRID_W; gx += 4) {
                            uint32_t cs = 0; int cc = 0;
                            for (int dy = 0; dy < 4 && gy+dy < IF_GRID_H; dy++)
                                for (int dx = 0; dx < 4 && gx+dx < IF_GRID_W; dx++)
                                    { cs += grid.grid[gy+dy][gx+dx]; cc++; }
                            uint8_t a = (uint8_t)(cs / cc);
                            putchar(a < 32 ? ' ' : a < 64 ? '.' : a < 96 ? ':' :
                                    a < 128 ? '-' : a < 160 ? '=' : a < 192 ? '+' :
                                    a < 224 ? '#' : '%');
                        }
                        putchar('\n');
                    }
                }

                IF_Process(detection_frame, &g_detection_result);
                uint32_t detect_ms = HAL_GetTick() - t_detect;
                printf("[MULTI] Detection took %lu ms\n", (unsigned long)detect_ms);
                printf("[MULTI] Result: has_insect=%s  hot_count=%u  strong=%u  bbox=(%u,%u) %ux%u  fallback=%u\n",
                       g_detection_result.has_insect ? "YES" : "no",
                       g_detection_result.hot_count,
                       g_detection_result.hot_strong_count,
                       g_detection_result.bbox_x, g_detection_result.bbox_y,
                       g_detection_result.bbox_w, g_detection_result.bbox_h,
                       g_detection_result.fallback);
                if (g_detection_result.has_insect) {
                    g_stage1_detected = 1;
                    printf("[MULTI] >>> STAGE 1: INSECT DETECTED <<<\n");
                } else {
                    g_stage1_detected = 0;
                    printf("[MULTI] STAGE 1: No insect detected (fallback: save full-frame)\n");
                }

                /* ---------------------------------------------------------------
                   STAGE 2: Save batch frames to SD (read from batch_buf).
                   Must happen BEFORE Stage 3 which REUSES batch_buf for AREA3.
                   --------------------------------------------------------------- */
                g_capture_stage = CAPTURE_STAGE2_FULLFRAME;
                printf("[MULTI] >>> STAGE 2: Save %d batch frames (%ux%u)\n",
                       rc, SNAP_WIDTH, SNAP_HEIGHT);
                xSemaphoreGive(camera_ready_sem);
                RTC_Stamp stamp = RTC_CaptureStamp();
                uint32_t first_id = g_next_snap_id;
                g_next_snap_id += (uint32_t)rc;
                for (int f = 0; f < rc; f++) {
                    StorageCmd_t sc = {0};
                    sc.type = STORAGE_CMD_SAVE;
                    sc.image_buf = batch_buf + (f * frame_size);
                    sc.image_size = frame_size;
                    sc.width = SNAP_WIDTH; sc.height = SNAP_HEIGHT;
                    sc.pixel_format = 0; sc.snap_id = first_id + (uint32_t)f;
#if CAPTURE_MODE == 4
                    sc.stamp = RTC_StampForFrame(stamp, CAM_GetCallbackFrameTick((uint32_t)f));
#else
                    sc.stamp = stamp;
#endif
                    if (xQueueSend(storage_cmd_queue, &sc, pdMS_TO_TICKS(1000)) != pdTRUE) {
#if PERF_DEBUG_LEVEL >= 1
                        printf("[CAM] Storage queue full, dropping frame %d\n", f);
#endif
                    }
                }
                /* Wait for storage to finish reading batch frames before
                   Stage 3 overwrites batch_buf with the crop output. */
                for (int i = 0; i < rc; i++) {
                    xSemaphoreTake(storage_done_sem, pdMS_TO_TICKS(5000));
                }

                /* ---------------------------------------------------------------
                   STAGE 3: Software crop each batch frame from the buffer.
                   Only when insect detected — skip fallback crops.
                   --------------------------------------------------------------- */
                /* ---------------------------------------------------------------
                    STAGE 3: Sensor-level AREA3 + full DCMIPP DeInit->Init for
                    clean native-res crop capture.  Uses CAM_CaptureROI() which
                    does the full DeInit->Init->AREA3->Configure->Capture cycle
                    to avoid corrupted frames from stale DCMIPP state.
                    --------------------------------------------------------------- */
                if (g_detection_result.has_insect) {
                    uint16_t roi_x, roi_y, roi_w, roi_h;
                    int roi_ok = IF_CalcROI(&g_detection_result, &roi_x, &roi_y, &roi_w, &roi_h);
                    if (roi_ok != 0) {
                        printf("[MULTI] >>> Stage 3: IF_CalcROI failed, using full SNAP center\n");
                        roi_w = SNAP_WIDTH; roi_h = SNAP_HEIGHT;
                        roi_x = (uint16_t)((2592U - SNAP_WIDTH) / 2) & ~1U;
                        roi_y = (uint16_t)((1944U - SNAP_HEIGHT) / 2) & ~1U;
                    } else {
                        printf("[MULTI] >>> Stage 3: AREA3 (%u,%u) %ux%u @ native 2592x1944", roi_x, roi_y, roi_w, roi_h);
                    }
                    printf(" %ux%u native, capturing %d frames\n", roi_w, roi_h, rc);

                    /* CAM_CaptureROI internally does: DeInit->Init->AREA3->PipeConfig->Capture->Stop.
                       Returns captured frame count or -1 on error. */
                    int area3_frames = CAM_CaptureROI(batch_buf, BATCH_BUF_SIZE, roi_x, roi_y, roi_w, roi_h);
                    if (area3_frames > 0) {
                        for (int f = 0; f < area3_frames; f++) {
                            uint32_t roi_frame_sz = (uint32_t)roi_w * roi_h * 2;
                            uint8_t *frame = batch_buf + (uint32_t)f * roi_frame_sz;
                            SCB_InvalidateDCache_by_Addr((uint32_t*)frame, roi_frame_sz);
                            StorageCmd_t sc3 = {0};
                            sc3.type = STORAGE_CMD_SAVE;
                            sc3.image_buf = frame;
                            sc3.image_size = roi_frame_sz;
                            sc3.width = roi_w; sc3.height = roi_h;
                            sc3.pixel_format = 0;
                            sc3.snap_id = g_next_snap_id++;
                            sc3.stamp = RTC_CaptureStamp();
                            xQueueSend(storage_cmd_queue, &sc3, pdMS_TO_TICKS(1000));
                            printf("[MULTI] >>> Stage 3 frame %d: native AREA3 %ux%u snap=%lu\n",
                                   f, roi_w, roi_h, (unsigned long)(g_next_snap_id-1));
                            xSemaphoreTake(storage_done_sem, pdMS_TO_TICKS(5000));
                        }
                    } else {
                        printf("[MULTI] >>> Stage 3: CAM_CaptureROI returned %d\n", area3_frames);
                    }

                    /* Restore: full AREA3 + SNAP pipe config */
                    CAM_SetFullFrame();
                    uint16_t snap_off_x = (uint16_t)((2592U - SNAP_WIDTH) / 2) & ~1U;
                    uint16_t snap_off_y = (uint16_t)((1944U - SNAP_HEIGHT) / 2) & ~1U;
                    CAM_SetPipeResolution(SNAP_WIDTH, SNAP_HEIGHT, snap_off_x, snap_off_y);
                    CAM_Standby();
                }
                printf("=============================================================\n\n");
                g_capture_stage = CAPTURE_STAGE_DONE;
                g_detection_result.has_insect = 0;
                g_capture_busy = 0;

#if WS2812_MODE == 1 || WS2812_MODE == 0
                /* Turn off flash AFTER Stage 3 completes.
                   CRITICAL: camera_ready_sem is given at Stage 2 to unblock SD storage,
                   so Capture_RequestSnapshot would turn light off too early.  We stop
                   it HERE, in the camera task, after all 3 stages finish. */
                WS2812_FlashStop();
#endif
            } else {
#if PERF_DEBUG_LEVEL >= 1
#if CAPTURE_MODE == 4
                printf("[CAM] Callback-Batch FAIL\n");
#else
                printf("[CAM] Batch FAIL\n");
#endif
#endif
                g_capture_busy = 0;
                xSemaphoreGive(camera_ready_sem);
            }
#else
            rc = CAM_CaptureSingleFrame(capture_buf, MAX_SNAP_FRAME_SIZE, SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS, SNAP_WARMUP_FRAMES);
            PERF_MARK(g_perf_timer, CAM_END);
            if (rc == 0) {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] OK %lu ms\n", (unsigned long)Perf_PhaseElapsed(&g_perf_timer, PERF_PHASE_START, PERF_PHASE_CAM_DEINIT));
#endif
                xSemaphoreGive(camera_ready_sem);
                StorageCmd_t sc = {0};
                sc.type = STORAGE_CMD_SAVE; sc.image_buf = capture_buf;
                sc.image_size = frame_size; sc.width = SNAP_WIDTH; sc.height = SNAP_HEIGHT;
                sc.pixel_format = 0; sc.snap_id = g_next_snap_id++;
                sc.stamp = RTC_CaptureStamp();
                xQueueSend(storage_cmd_queue, &sc, pdMS_TO_TICKS(1000));
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[CAM] FAIL\n");
#endif
                g_capture_busy = 0;
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
            uint32_t storage_start = HAL_GetTick();
            int rc = SD_StoreRawImage(cmd.image_buf, cmd.image_size, cmd.width, cmd.height, cmd.pixel_format, cmd.snap_id, cmd.stamp);
            if (rc != 0) {
                /* A single transient card hiccup (timeout / CRC) previously meant this
                   image was silently lost. Reinit the peripheral and retry the SAME
                   image ONCE before giving up, since g_sd_img_base_block was restored
                   to the block this image was supposed to start at. */
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] FAIL — attempting recovery + retry...\n");
#endif
                if (SD_Reinit() == 0) {
                    rc = SD_StoreRawImage(cmd.image_buf, cmd.image_size, cmd.width, cmd.height, cmd.pixel_format, cmd.snap_id, cmd.stamp);
                }
            }
            if (rc != 0) g_last_storage_rc = rc;
#if PERF_DEBUG_LEVEL >= 1
            uint32_t storage_elapsed = HAL_GetTick() - storage_start;
#endif
            if (rc == 0) {
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] OK %lu ms\n", (unsigned long)storage_elapsed);
#endif
            } else {
#if PERF_DEBUG_LEVEL >= 1
                printf("[SD] FAIL %lu ms — image LOST after retry\n", (unsigned long)storage_elapsed);
#endif
                SD_Reinit();
            }
            Perf_StorageComplete(&g_perf_timer, HAL_GetTick() - storage_start, cmd.image_size, rc == 0);
            xSemaphoreGive(storage_done_sem);
        }
    }
}

/* ==================== PIPELINE ==================== */
static int Capture_ReturnWithIlluminationOff(int result)
{
#if WS2812_MODE == 0 || WS2812_MODE == 1
    /* Final fail-safe cleanup for every success/error/timeout return path.
       The normal camera-complete path still turns the ring off immediately;
       this second cleanup catches a misdecoded OFF frame after SD activity. */
    WS2812_FlashStop();
#endif
    return result;
}

int Capture_RequestSnapshot(uint32_t timeout_ms)
{
    Perf_Start(&g_perf_timer);
    uint32_t t_start = g_perf_timer.start_tick;
    CameraCmd_t cmd = {0}; cmd.type = CAM_CMD_SNAP;
    g_last_storage_rc = 0;
    while (xSemaphoreTake(storage_done_sem, 0) == pdTRUE) {
        /* Drain stale completion tokens from previous capture cycles. */
    }
    if (xQueueSend(camera_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return Capture_ReturnWithIlluminationOff(-2);
    }

    TickType_t ticks = (timeout_ms > 0) ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
    if (xSemaphoreTake(camera_ready_sem, ticks) != pdTRUE) {
        return Capture_ReturnWithIlluminationOff(-1);
    }

    /* NOTE: WS2812_FlashStop is called by the camera task AFTER Stage 3
       completes (see camera_task loop).  We removed it from here because
       camera_ready_sem is given during Stage 2, which is BEFORE Stage 3. */

#if CAPTURE_MODE == 4
    /* For CAPTURE_MODE == 4: camera_task already waits for ALL storage tokens
       (Stage 2 batch-frames + Stage 3 AREA3 frames) inside its success path.
       We NO LONGER wait here — the camera task handles every storage_done_sem
       token.  The only remaining need is the error check below. */
    if (g_last_batch_frames <= 0) {
        /* camera_task signals camera_ready_sem on capture failure as well.
           Do not wait for storage tokens that will never be produced. */
        return Capture_ReturnWithIlluminationOff(-1);
    }
    /* NO extra xSemaphoreTake(storage_done_sem) here — camera_task already
       consumed ALL storage tokens (Stage 2 + Stage 3). */
#else
    if (timeout_ms > 0) {
        uint32_t elapsed = HAL_GetTick() - t_start;
        if (elapsed >= timeout_ms) return Capture_ReturnWithIlluminationOff(-1);
        ticks = pdMS_TO_TICKS(timeout_ms - elapsed);
    } else { ticks = portMAX_DELAY; }
    if (xSemaphoreTake(storage_done_sem, ticks) != pdTRUE)
        return Capture_ReturnWithIlluminationOff(-1);
#endif
    Perf_Stop(&g_perf_timer);
    if (g_last_storage_rc != 0) return Capture_ReturnWithIlluminationOff(-1);
#if CAPTURE_MODE == 4
    /* For CAPTURE_MODE == 4, the camera task calls WS2812_FlashStop AFTER
       Stage 3 completes (line 1170).  We must NOT turn the light off here —
       Stage 3 still needs illumination for the AREA3 native-res crop.
       The camera task handles both Stage 2 save AND Stage 3 capture +
       light-off internally. */
    return 0;
#else
    return Capture_ReturnWithIlluminationOff(0);
#endif
}
