/**
 * ******************************************************************************
 * @file    app_thread.c
 * @brief   Multi-threaded Camera + Sensor + Storage implementation
 *
 *   Three dedicated threads with clean separation of concerns:
 *     1. SENSOR_TASK  - VL53L5CX ToF monitoring & insect detection
 *     2. CAMERA_TASK  - Camera acquisition (continuous/snapshot modes)
 *     3. STORAGE_TASK - SD card raw image storage
 * ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include "app_thread.h"
#include "app_config.h"
#include "app_cam.h"
#include "vl53l5cx_detection.h"
#include "ws2812.h"
#include "stm32n6xx_hal.h"
#include "stm32n6570_discovery.h"  /* BSP_LED_On/Off, LED_GREEN/LED_RED */
#include "main.h"

/* ================================================================
   EXTERNAL REFERENCES
   ================================================================ */

extern I2C_HandleTypeDef hi2c1;
extern uint8_t capture_buf[];    /* Main camera buffer (PSRAM) */

#if CAPTURE_MODE == 1
extern uint8_t save_buf[];       /* Save buffer for continuous mode (PSRAM) */
#endif

/* ================================================================
   IPC OBJECTS (queues, semaphores)
   ================================================================ */

QueueHandle_t    camera_cmd_queue    = NULL;
QueueHandle_t    storage_cmd_queue   = NULL;
QueueHandle_t    sensor_event_queue  = NULL;

SemaphoreHandle_t camera_ready_sem   = NULL;
SemaphoreHandle_t storage_done_sem   = NULL;

volatile SensorState_t g_sensor_state = SENSOR_STATE_IDLE;

/* Capture busy flag */
static volatile int g_capture_busy = 0;

/* Snapshot counter (preserved from original code) */
static uint32_t g_snap_count = 0;

/* SD image base block tracking */
static uint32_t g_sd_img_base_block;
#define SD_IMG_HEADER_SIZE  64
#define SD_BLOCK_SIZE       512

/* Debug output counters */
static uint32_t g_debug_frame_count = 0;

/* ================================================================
   SD CARD IMAGE HEADER (matches main.c)
   ================================================================ */

typedef struct {
    uint32_t magic;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t data_size;
    uint32_t timestamp;
    uint32_t checksum;
    uint32_t snap_id;
    uint8_t  reserved[28];
} sd_image_header_t;

#define SD_HEADER_TAG       0x49444745U
#define SD_BATCH_WRITE_BLOCKS  64

/* External SD handle and batch buffer references */
extern SD_HandleTypeDef hsd1;
extern uint8_t sd_batch_buf[];

/* ================================================================
   IPC_Init / IPC_Deinit
   ================================================================ */

BaseType_t IPC_Init(void)
{
    /* Create queues */
    camera_cmd_queue = xQueueCreate(CAMERA_CMD_QUEUE_LEN, sizeof(CameraCmd_t));
    if (!camera_cmd_queue) return pdFALSE;

    storage_cmd_queue = xQueueCreate(STORAGE_CMD_QUEUE_LEN, sizeof(StorageCmd_t));
    if (!storage_cmd_queue) return pdFALSE;

    sensor_event_queue = xQueueCreate(SENSOR_EVENT_QUEUE_LEN, sizeof(CameraEvent_typed));
    if (!sensor_event_queue) return pdFALSE;

    /* Create binary semaphores (initially given = 0 / not taken) */
    camera_ready_sem = xSemaphoreCreateBinary();
    if (!camera_ready_sem) return pdFALSE;

    storage_done_sem = xSemaphoreCreateBinary();
    if (!storage_done_sem) return pdFALSE;

    /* Ensure semaphores start in "not signaled" state */
    xSemaphoreTake(camera_ready_sem, 0);
    xSemaphoreTake(storage_done_sem, 0);

    /* Initialize SD base block */
    g_sd_img_base_block = SD_SNAP_BASE_BLOCK;

    printf("[IPC] Queues and semaphores created successfully\n");
    return pdTRUE;
}

void IPC_Deinit(void)
{
    if (camera_cmd_queue)    { vQueueDelete(camera_cmd_queue);    camera_cmd_queue    = NULL; }
    if (storage_cmd_queue)   { vQueueDelete(storage_cmd_queue);   storage_cmd_queue   = NULL; }
    if (sensor_event_queue)  { vQueueDelete(sensor_event_queue);  sensor_event_queue  = NULL; }
    if (camera_ready_sem)    { vSemaphoreDelete(camera_ready_sem); camera_ready_sem   = NULL; }
    if (storage_done_sem)    { vSemaphoreDelete(storage_done_sem); storage_done_sem   = NULL; }
}

/* ================================================================
   HIGH-LEVEL API
   ================================================================ */

int Capture_IsBusy(void)
{
    return g_capture_busy;
}

BaseType_t Capture_RequestSnapOnly(void)
{
    CameraCmd_t cmd = {0};
    cmd.type = CAM_CMD_SNAP;

    if (xQueueSend(camera_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        printf("[CAPTURE] Snap request failed (queue full)\n");
        return pdFALSE;
    }
    return pdTRUE;
}

/**
 * @brief  SD_StoreRawImage — moved here from main.c for the storage task.
 *         Uses batched writes for performance.
 */
static int SD_StoreRawImage(const uint8_t *img_buf, uint32_t img_size,
                            uint32_t w, uint32_t h, uint32_t pixel_format,
                            uint32_t snap_id)
{
    uint32_t base = g_sd_img_base_block;
    uint32_t total_blocks = (SD_IMG_HEADER_SIZE + img_size + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
    uint32_t i, checksum = 0;

    /* Overflow protection */
    if (hsd1.SdCard.BlockNbr > 0) {
        uint32_t last_block = base + total_blocks - 1;
        if (last_block >= hsd1.SdCard.BlockNbr) {
            printf("[SD Store] OVERFLOW! block %lu >= card max %lu\n",
                   (unsigned long)last_block, (unsigned long)hsd1.SdCard.BlockNbr);
            return -1;
        }
    }

    /* Compute XOR checksum */
    for (i = 0; i < img_size; i++)
        checksum ^= img_buf[i];

    /* Build header */
    sd_image_header_t hdr;
    hdr.magic        = SD_HEADER_TAG;
    hdr.width        = w;
    hdr.height       = h;
    hdr.pixel_format = pixel_format;
    hdr.data_size    = img_size;
    hdr.timestamp    = HAL_GetTick();
    hdr.checksum     = checksum;
    hdr.snap_id      = snap_id;
    memset(hdr.reserved, 0, sizeof(hdr.reserved));

    uint32_t batch_size = SD_BATCH_WRITE_BLOCKS * SD_BLOCK_SIZE;

    printf("[SD Store] %lux%lu fmt=%lu %lu bytes %lu blocks (batch=%lu)\n",
           (unsigned long)w, (unsigned long)h, pixel_format,
           img_size, total_blocks, (unsigned long)SD_BATCH_WRITE_BLOCKS);

    /* --- Step 1: Write header block + first image chunk --- */
    memset((void *)sd_batch_buf, 0, batch_size);
    memcpy((void *)sd_batch_buf, (const void *)&hdr, SD_IMG_HEADER_SIZE);
    uint32_t first_chunk = SD_BLOCK_SIZE - SD_IMG_HEADER_SIZE;
    if (first_chunk > img_size) first_chunk = img_size;
    memcpy((void *)(sd_batch_buf + SD_IMG_HEADER_SIZE), (const void *)img_buf, first_chunk);

    uint32_t img_offset = first_chunk;
    uint32_t batch_blk  = 1;
    while (img_offset < img_size && batch_blk < SD_BATCH_WRITE_BLOCKS) {
        uint32_t chunk = SD_BLOCK_SIZE;
        if (chunk > img_size - img_offset) chunk = img_size - img_offset;
        memcpy((void *)(sd_batch_buf + batch_blk * SD_BLOCK_SIZE),
               (const void *)(img_buf + img_offset), chunk);
        if (chunk < SD_BLOCK_SIZE)
            memset((void *)(sd_batch_buf + batch_blk * SD_BLOCK_SIZE + chunk),
                   0, SD_BLOCK_SIZE - chunk);
        img_offset += chunk;
        batch_blk++;
    }

    HAL_StatusTypeDef st = HAL_SD_WriteBlocks(&hsd1, sd_batch_buf, base, batch_blk, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        printf("[SD Store] header batch FAIL (HAL=0x%08lX)\n", (unsigned long)st);
        return -1;
    }

    uint32_t current_block = base + batch_blk;

    /* --- Step 2: Write remaining image data in full batches --- */
    while (img_offset < img_size) {
        uint32_t remaining_bytes = img_size - img_offset;
        uint32_t remaining_blocks_full = (remaining_bytes + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
        uint32_t blocks_in_batch = (remaining_blocks_full < SD_BATCH_WRITE_BLOCKS)
                                    ? remaining_blocks_full : SD_BATCH_WRITE_BLOCKS;

        for (uint32_t b = 0; b < blocks_in_batch; b++) {
            uint32_t src = img_offset + b * SD_BLOCK_SIZE;
            uint32_t dst = b * SD_BLOCK_SIZE;
            if (src + SD_BLOCK_SIZE <= img_size) {
                memcpy((void *)(sd_batch_buf + dst), (const void *)(img_buf + src), SD_BLOCK_SIZE);
            } else {
                uint32_t partial = img_size - src;
                memcpy((void *)(sd_batch_buf + dst), (const void *)(img_buf + src), partial);
                memset((void *)(sd_batch_buf + dst + partial), 0, SD_BLOCK_SIZE - partial);
            }
        }

        /* Wait for SD card ready */
        {
            uint32_t wait_ms = 0;
            while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER && wait_ms < 2000) {
                vTaskDelay(pdMS_TO_TICKS(1));
                wait_ms++;
            }
            if (wait_ms >= 2000) {
                printf("[SD Store] Card timeout before block %lu (state=%lu)\n",
                       (unsigned long)current_block,
                       (unsigned long)HAL_SD_GetCardState(&hsd1));
            }
        }

        st = HAL_SD_WriteBlocks(&hsd1, sd_batch_buf, current_block, blocks_in_batch, HAL_MAX_DELAY);
        if (st != HAL_OK) {
            printf("[SD Store] batch FAIL at block %lu (HAL=0x%08lX STA=0x%08lX)\n",
                   (unsigned long)current_block, (unsigned long)st, (unsigned long)SDMMC2->STA);
            return -1;
        }

        img_offset += blocks_in_batch * SD_BLOCK_SIZE;
        current_block += blocks_in_batch;
    }

    printf("[SD Store] OK (blocks %lu..%lu)\n",
           (unsigned long)base, (unsigned long)(current_block - 1));

    /* Advance base block for next snapshot */
    g_sd_img_base_block = current_block;

    return 0;
}


/* ================================================================
   SENSOR TASK
   ================================================================ */

void sensor_task(void *arg)
{
    (void)arg;

    printf("\n============================================\n");
    printf("  SENSOR TASK (VL53L5CX)\n");
    printf("============================================\n\n");

    /* Wait for system ready signal from main */
    extern volatile int system_ready;
    while (!system_ready) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Initialize ToF sensor */
    if (VL53L5CX_Init(&hi2c1) != 0) {
        printf("[SENSOR] VL53L5CX_Init failed! Scanning I2C bus...\n");
        VL53L5CX_ScanI2CBus();
        /* Don't delete task — keep alive for button capture */
        g_sensor_state = SENSOR_STATE_STOPPED;
        while (1) {
            printf("[SENSOR] ToF unavailable. Press button for camera-only mode.\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    /* Configure resolution, integration time, frequency */
#if VL53L5CX_DET_RESOLUTION == 8
    VL53L5CX_Configure(VL53L5CX_RESOLUTION_8X8, 800, 15);
    printf("[SENSOR] Resolution: 8x8 (64 zones)\n");
#else
    VL53L5CX_Configure(VL53L5CX_RESOLUTION_4X4, 800, 15);
    printf("[SENSOR] Resolution: 4x4 (16 zones)\n");
#endif

    /* Start continuous ranging */
    VL53L5CX_StartRanging();

    /* Learn baseline */
    VL53L5CX_LearnBaseline();

    if (!VL53L5CX_IsBaselineReady()) {
        printf("[SENSOR] Baseline learning failed!\n");
        g_sensor_state = SENSOR_STATE_STOPPED;
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    g_sensor_state = SENSOR_STATE_RUNNING;

    printf("[SENSOR] Monitoring zones for insect passage...\n");
#if CAPTURE_MODE == 1
    printf("[SENSOR] Camera mode: CONTINUOUS\n");
#else
    printf("[SENSOR] Camera mode: ON-DEMAND\n");
#endif
    printf("[SENSOR] Auto-capture on detection enabled.\n\n");

    uint32_t capture_count = 0;
    uint32_t cooldown_frames = 0;
    const uint32_t COOLDOWN_FRAMES_VALUE = 30;

    while (1) {
        /* If sensor is paused, wait briefly and skip ToF reads */
        if (g_sensor_state == SENSOR_STATE_PAUSED) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (g_sensor_state == SENSOR_STATE_STOPPED) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Normal operation: update ToF sensor */
        if (!VL53L5CX_Update()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Continuous debug output */
        g_debug_frame_count++;
        if (g_debug_frame_count >= 1) {
            g_debug_frame_count = 0;
            //VL53L5CX_PrintZFrame();
        }

        /* Check cooldown */
        if (cooldown_frames > 0) cooldown_frames--;

        /* ---- INSECT DETECTED? ---- */
        if (VL53L5CX_IsInsectDetected() && cooldown_frames == 0) {
            /* Guard: skip if a capture is already in progress */
            if (g_capture_busy) {
                printf("    [SKIP] Capture already in progress, ignoring detection.\n");
                continue;
            }

            g_capture_busy = 1;

            VL53L5CX_DetectionResult_t result = VL53L5CX_GetResult();

            const char *trig_str;
            switch (result.trigger_source) {
                case VL53L5CX_TRIG_SIGNAL: trig_str = "signal"; break;
                case VL53L5CX_TRIG_MOTION: trig_str = "motion"; break;
                case VL53L5CX_TRIG_BOTH:   trig_str = "signal+motion"; break;
                default:                    trig_str = "unknown"; break;
            }

            printf(">>> INSECT DETECTED! Trigger: %s (%u zones)\n",
                   trig_str, result.affected_count);

            /* ---- STEP 0: LEDs + Illumination ---- */
            BSP_LED_Off(LED_GREEN);
            BSP_LED_On(LED_RED);
            WS2812_TurnOn();
            WS2812_TurnOff();

            /* ---- STEP 1: PAUSE SENSOR (stop I2C + ToF reads) ---- */
            g_sensor_state = SENSOR_STATE_PAUSED;
            printf("    [SENSOR] PAUSED — handing off to camera+storage\n");

            /* ---- STEP 2: TRIGGER CAPTURE via the threaded pipeline ---- */
            uint32_t t_total_start = HAL_GetTick();
            int capture_rc = Capture_RequestSnapshot(5000);

            if (capture_rc == 0) {
                capture_count++;
                uint32_t t_total_end = HAL_GetTick();
                printf("    >>> Snapshot #%lu SAVED (total %lu ms)\n",
                       (unsigned long)capture_count,
                       (unsigned long)(t_total_end - t_total_start));
            } else {
                printf("    [ERROR] Capture pipeline failed (rc=%d)\n", capture_rc);
            }

            /* ---- STEP 3: Restore LEDs + Resume Sensor ---- */
            BSP_LED_Off(LED_RED);
            BSP_LED_On(LED_GREEN);
            g_sensor_state = SENSOR_STATE_RUNNING;
            g_capture_busy = 0;
            cooldown_frames = COOLDOWN_FRAMES_VALUE;

            printf("    [SENSOR] RESUMED — Cooldown: %d frames (%.1f s)\n\n",
                   COOLDOWN_FRAMES_VALUE, (float)COOLDOWN_FRAMES_VALUE / 15.0f);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


/* ================================================================
   CAMERA TASK
   ================================================================ */

void camera_task(void *arg)
{
    (void)arg;

    printf("\n============================================\n");
    printf("  CAMERA TASK\n");
    printf("============================================\n\n");

    /* Wait for system ready */
    extern volatile int system_ready;
    while (!system_ready) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

#if CAPTURE_MODE == 1
    /* CONTINUOUS MODE: Start camera at boot */
    printf("[CAMERA] Starting continuous capture at boot...\n");
    int cam_rc = CAM_ContinuousStart(capture_buf, MAX_SNAP_FRAME_SIZE,
                                     SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS);
    if (cam_rc != 0) {
        printf("[CAMERA] ContinuousStart failed (rc=%d)!\n", cam_rc);
    } else {
        printf("[CAMERA] Continuous capture RUNNING\n");
    }
#endif

    uint32_t frame_size = (uint32_t)SNAP_WIDTH * SNAP_HEIGHT * 2UL;

    while (1) {
        CameraCmd_t cmd = {0};

        /* In continuous mode, keep the ISP updated */
#if CAPTURE_MODE == 1
        CAM_IspUpdate();
#endif

        /* Wait for commands (check immediately, then block) */
        if (xQueueReceive(camera_cmd_queue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }

        printf("[CAMERA] Received command: type=%d\n", cmd.type);

        switch (cmd.type) {
        case CAM_CMD_SNAP: {
            uint32_t t0 = HAL_GetTick();
            int rc = -1;
            const uint8_t *frame_to_save = NULL;

#if CAPTURE_MODE == 1
            /* CONTINUOUS MODE: Stop pipe → copy → restart */
            printf("[CAMERA] [CONTINUOUS] Snapping current frame...\n");
            extern uint8_t save_buf[];
            rc = CAM_ContinuousSnap(save_buf, frame_size);
            if (rc == 0) {
                frame_to_save = save_buf;
            }
#else
            /* ON-DEMAND MODE: Full init + capture + deinit */
            printf("[CAMERA] [ON-DEMAND] Full camera init + capture...\n");
            rc = CAM_CaptureSingleFrame(capture_buf, MAX_SNAP_FRAME_SIZE,
                                        SNAP_WIDTH, SNAP_HEIGHT,
                                        SNAP_FPS, SNAP_WARMUP_FRAMES);
            if (rc == 0) {
                frame_to_save = capture_buf;
            }
#endif

            uint32_t elapsed = HAL_GetTick() - t0;

            if (rc == 0) {
                printf("[CAMERA] Snap OK in %lu ms\n", (unsigned long)elapsed);

                /* Signal camera ready */
                xSemaphoreGive(camera_ready_sem);

                /* Forward to storage task */
                StorageCmd_t storage_cmd = {0};
                storage_cmd.type         = STORAGE_CMD_SAVE;
                storage_cmd.image_buf    = frame_to_save;
                storage_cmd.image_size   = frame_size;
                storage_cmd.width        = SNAP_WIDTH;
                storage_cmd.height       = SNAP_HEIGHT;
                storage_cmd.pixel_format = 0;  /* YUV422 */
                storage_cmd.snap_id      = g_snap_count;

                if (xQueueSend(storage_cmd_queue, &storage_cmd, pdMS_TO_TICKS(1000)) != pdTRUE) {
                    printf("[CAMERA] Storage queue full! Frame dropped.\n");
                }
            } else {
                printf("[CAMERA] Snap FAILED (rc=%d)\n", rc);

                /* Signal failure so sensor doesn't block forever */
                CameraEvent_typed evt = {0};
                evt.type       = CAM_EVENT_SNAP_FAILED;
                evt.error_code = rc;
                evt.elapsed_ms = elapsed;
                xQueueSend(sensor_event_queue, &evt, 0);

                /* Still give the semaphore so sensor task unblocks */
                xSemaphoreGive(camera_ready_sem);
            }
            break;
        }

        case CAM_CMD_START_CONTINUOUS: {
            printf("[CAMERA] Starting continuous capture...\n");
            int rc = CAM_ContinuousStart(capture_buf, MAX_SNAP_FRAME_SIZE,
                                         SNAP_WIDTH, SNAP_HEIGHT, SNAP_FPS);
            CameraEvent_typed evt = {0};
            if (rc == 0) {
                evt.type = CAM_EVENT_CONTINUOUS_STARTED;
                printf("[CAMERA] Continuous capture started\n");
            } else {
                evt.type       = CAM_EVENT_ERROR;
                evt.error_code = rc;
                printf("[CAMERA] ContinuousStart failed (rc=%d)\n", rc);
            }
            evt.elapsed_ms = HAL_GetTick();
            xQueueSend(sensor_event_queue, &evt, 0);
            break;
        }

        case CAM_CMD_STOP_CONTINUOUS: {
            printf("[CAMERA] Stopping continuous capture...\n");
            CAM_ContinuousStop();
            CameraEvent_typed evt = {0};
            evt.type = CAM_EVENT_CONTINUOUS_STOPPED;
            xQueueSend(sensor_event_queue, &evt, 0);
            break;
        }

        case CAM_CMD_CAPTURE_SINGLE: {
            uint32_t t0 = HAL_GetTick();
            printf("[CAMERA] Single capture %lux%lu @%lu fps...\n",
                   (unsigned long)cmd.width, (unsigned long)cmd.height,
                   (unsigned long)cmd.fps);

            int rc = CAM_CaptureSingleFrame(capture_buf, MAX_SNAP_FRAME_SIZE,
                                            cmd.width, cmd.height,
                                            cmd.fps, cmd.warmup_frames);

            uint32_t elapsed = HAL_GetTick() - t0;
            CameraEvent_typed evt = {0};

            if (rc == 0) {
                evt.type = CAM_EVENT_SNAP_READY;
                evt.elapsed_ms = elapsed;
                printf("[CAMERA] Single capture OK in %lu ms\n", (unsigned long)elapsed);

                /* Forward to storage if buffer provided */
                if (cmd.dest_buf) {
                    StorageCmd_t storage_cmd = {0};
                    storage_cmd.type         = STORAGE_CMD_SAVE;
                    storage_cmd.image_buf    = capture_buf;
                    storage_cmd.image_size   = frame_size;
                    storage_cmd.width        = cmd.width;
                    storage_cmd.height       = cmd.height;
                    storage_cmd.pixel_format = 0;
                    storage_cmd.snap_id      = g_snap_count;
                    xQueueSend(storage_cmd_queue, &storage_cmd, pdMS_TO_TICKS(1000));
                }
            } else {
                evt.type       = CAM_EVENT_SNAP_FAILED;
                evt.error_code = rc;
                evt.elapsed_ms = elapsed;
                printf("[CAMERA] Single capture FAILED (rc=%d)\n", rc);
            }

            xSemaphoreGive(camera_ready_sem);
            xQueueSend(sensor_event_queue, &evt, 0);
            break;
        }

        case CAM_CMD_GET_STATUS:
            printf("[CAMERA] Status: continuous=%s\n",
#if CAPTURE_MODE == 1
                   "running"
#else
                   "on-demand"
#endif
            );
            break;

        default:
            printf("[CAMERA] Unknown command: %d\n", cmd.type);
            break;
        }
    }
}


/* ================================================================
   STORAGE TASK
   ================================================================ */

void storage_task(void *arg)
{
    (void)arg;

    printf("\n============================================\n");
    printf("  STORAGE TASK\n");
    printf("============================================\n\n");

    /* Wait for system ready */
    extern volatile int system_ready;
    while (!system_ready) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    printf("[STORAGE] Waiting for save commands...\n\n");

    while (1) {
        StorageCmd_t cmd = {0};

        /* Block until a command arrives */
        if (xQueueReceive(storage_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (cmd.type) {
        case STORAGE_CMD_SAVE: {
            printf("[STORAGE] Save request: %lux%lu %lu bytes (snap_id=%lu)\n",
                   (unsigned long)cmd.width, (unsigned long)cmd.height,
                   (unsigned long)cmd.image_size, (unsigned long)cmd.snap_id);

            uint32_t t0 = HAL_GetTick();

            /* Perform the SD write (blocking — this is the storage task's job) */
            int rc = SD_StoreRawImage(cmd.image_buf, cmd.image_size,
                                      cmd.width, cmd.height,
                                      cmd.pixel_format, cmd.snap_id);

            uint32_t elapsed = HAL_GetTick() - t0;

            if (rc == 0) {
                g_snap_count++;
                printf("[STORAGE] Save OK (%lu ms) — total snaps: %lu\n",
                       (unsigned long)elapsed, (unsigned long)g_snap_count);

                StorageEvent_t evt = {0};
                evt.type      = STORAGE_EVENT_SAVE_DONE;
                evt.elapsed_ms = elapsed;
                // Not sending to any queue since camera_task already signaled
            } else {
                printf("[STORAGE] Save FAILED (%lu ms)\n", (unsigned long)elapsed);

                StorageEvent_t evt = {0};
                evt.type       = STORAGE_EVENT_SAVE_FAILED;
                evt.error_code = rc;
                evt.elapsed_ms = elapsed;
            }

            /* Signal storage done — unblocks sensor task */
            xSemaphoreGive(storage_done_sem);
            break;
        }

        case STORAGE_CMD_GET_STATUS:
            printf("[STORAGE] Next block: %lu, Snaps saved: %lu\n",
                   (unsigned long)g_sd_img_base_block,
                   (unsigned long)g_snap_count);
            break;

        case STORAGE_CMD_GET_FREE_SPACE:
            if (hsd1.SdCard.BlockNbr > 0) {
                uint32_t blocks_per_snap = (SD_IMG_HEADER_SIZE +
                    (uint32_t)SNAP_WIDTH * SNAP_HEIGHT * 2UL + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;
                uint32_t remaining = hsd1.SdCard.BlockNbr - g_sd_img_base_block;
                uint32_t snaps_left = remaining / blocks_per_snap;
                printf("[STORAGE] ~%lu snapshots remaining (%lu blocks free)\n",
                       (unsigned long)snaps_left, (unsigned long)remaining);
            }
            break;

        default:
            printf("[STORAGE] Unknown command: %d\n", cmd.type);
            break;
        }
    }
}


/* ================================================================
   Capture_RequestSnapshot — Full capture pipeline
   ================================================================ */

int Capture_RequestSnapshot(uint32_t timeout_ms)
{
    uint32_t t_start = HAL_GetTick();

    /* ---- STEP 1: Send snapshot command to camera task ---- */
    CameraCmd_t cam_cmd = {0};
    cam_cmd.type = CAM_CMD_SNAP;

    if (xQueueSend(camera_cmd_queue, &cam_cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        printf("[PIPELINE] Failed to send snap command to camera\n");
        return -2;
    }

    /* ---- STEP 2: Wait for camera to finish the snap ---- */
    TickType_t ticks_left;
    if (timeout_ms > 0) {
        uint32_t elapsed = HAL_GetTick() - t_start;
        if (elapsed >= timeout_ms) return -1;
        ticks_left = pdMS_TO_TICKS(timeout_ms - elapsed);
    } else {
        ticks_left = portMAX_DELAY;
    }

    if (xSemaphoreTake(camera_ready_sem, ticks_left) != pdTRUE) {
        printf("[PIPELINE] Timeout waiting for camera snap\n");
        return -1;
    }

    /* ---- STEP 3: Wait for storage to finish the SD write ---- */
    if (timeout_ms > 0) {
        uint32_t elapsed = HAL_GetTick() - t_start;
        if (elapsed >= timeout_ms) return -1;
        ticks_left = pdMS_TO_TICKS(timeout_ms - elapsed);
    } else {
        ticks_left = portMAX_DELAY;
    }

    if (xSemaphoreTake(storage_done_sem, ticks_left) != pdTRUE) {
        printf("[PIPELINE] Timeout waiting for storage write\n");
        return -1;
    }

    printf("[PIPELINE] Capture complete (snap + save)\n");
    return 0;
}