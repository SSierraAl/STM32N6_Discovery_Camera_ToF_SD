/**
 * @file    app_sdcard.c
 * @brief   SD Card helper functions for camera frame storage
 * @date    June 2026
 */

#include "app_sdcard.h"
#include "app_filex.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * Local Variables
 * ========================================================================== */

static char g_session_dir[32] = {0};
static APP_SDCard_Stats_t g_stats = {0};

/* Mutex for thread-safe SD operations */

static StaticSemaphore_t g_sd_mutex_buffer;
static SemaphoreHandle_t g_sd_mutex = NULL;

/* ============================================================================
 * Initialization (called from app_filex.c)
 * ========================================================================== */

void APP_SDCard_Init(void)
{
    /* Create mutex for thread safety */
    g_sd_mutex = xSemaphoreCreateMutexStatic(&g_sd_mutex_buffer);

    /* Get session directory name */
    uint32_t tick = HAL_GetTick();
    sprintf(g_session_dir, "/SESSION_%010lu", tick);

    memset(&g_stats, 0, sizeof(g_stats));
}

/* ============================================================================
 * Helper: Build full file path
 * ========================================================================== */

static void build_filepath(char *filepath, size_t max_len, 
                          uint32_t frame_number, const char *format)
{
    snprintf(filepath, max_len, "%s/%s", 
             g_session_dir, format);
    
    /* Replace %lu placeholder if format contains it */
    char temp[64];
    snprintf(temp, sizeof(temp), format, frame_number);
    snprintf(filepath, max_len, "%s/%s", g_session_dir, temp);
}

/* ============================================================================
 * Frame Storage Functions
 * ========================================================================== */

uint32_t APP_SDCard_StoreFrame_RAW(uint32_t frame_number, 
                                   const uint8_t *buffer, 
                                   uint32_t size)
{
    if (!buffer || !size) {
        return 1;  /* Invalid parameters */
    }

    if (!MX_FileX_IsSD_Ready()) {
        return 2;  /* SD not ready */
    }

    char filepath[64];
    build_filepath(filepath, sizeof(filepath), frame_number, SD_RAW_FORMAT);

    /* Take mutex */
    if (g_sd_mutex) {
        xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    }

    /* Write file */
    UINT status = MX_FileX_Write(filepath, (VOID *)buffer, size);

    /* Release mutex */
    if (g_sd_mutex) {
        xSemaphoreGive(g_sd_mutex);
    }

    if (status == FX_SUCCESS) {
        g_stats.frames_stored++;
        g_stats.total_bytes += size;
        return 0;  /* Success */
    } else {
        g_stats.errors++;
        return 3;  /* Write failed */
    }
}

uint32_t APP_SDCard_StoreFrame_JPEG(uint32_t frame_number,
                                    const uint8_t *buffer,
                                    uint32_t size)
{
    if (!buffer || !size) {
        return 1;
    }

    if (!MX_FileX_IsSD_Ready()) {
        return 2;
    }

    char filepath[64];
    build_filepath(filepath, sizeof(filepath), frame_number, SD_JPEG_FORMAT);

    if (g_sd_mutex) {
        xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    }

    UINT status = MX_FileX_Write(filepath, (VOID *)buffer, size);

    if (g_sd_mutex) {
        xSemaphoreGive(g_sd_mutex);
    }

    if (status == FX_SUCCESS) {
        g_stats.frames_stored++;
        g_stats.total_bytes += size;
        return 0;
    } else {
        g_stats.errors++;
        return 3;
    }
}

uint32_t APP_SDCard_StoreFrameMetadata(uint32_t frame_number,
                                       const char *metadata)
{
    if (!metadata) {
        return 1;
    }

    if (!MX_FileX_IsSD_Ready()) {
        return 2;
    }

    char filepath[64];
    build_filepath(filepath, sizeof(filepath), frame_number, SD_METADATA_FORMAT);

    if (g_sd_mutex) {
        xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    }

    UINT status = MX_FileX_Write(filepath, (VOID *)metadata, strlen(metadata));

    if (g_sd_mutex) {
        xSemaphoreGive(g_sd_mutex);
    }

    return (status == FX_SUCCESS) ? 0 : 3;
}

/* ============================================================================
 * Query Functions
 * ========================================================================== */

int APP_SDCard_HasSpace(uint32_t frame_size)
{
    uint32_t free_space = MX_FileX_GetFreeSpace();
    
    /* Keep 1MB reserve */
    #define SD_RESERVE_BYTES (1024 * 1024)
    
    if (free_space < (frame_size + SD_RESERVE_BYTES)) {
        return 0;  /* Not enough space */
    }
    
    return 1;  /* Has space */
}

uint32_t APP_SDCard_GetSessionDir(char *buffer, uint32_t size)
{
    if (!buffer || !size) {
        return 1;
    }

    strncpy(buffer, g_session_dir, size - 1);
    buffer[size - 1] = '\0';
    return 0;
}

uint32_t APP_SDCard_GetFreeSpace(void)
{
    uint32_t free = MX_FileX_GetFreeSpace();
    g_stats.sd_free_bytes = free;
    return free;
}

/* ============================================================================
 * Statistics
 * ========================================================================== */

void APP_SDCard_GetStats(APP_SDCard_Stats_t *stats)
{
    if (!stats) {
        return;
    }

    if (g_sd_mutex) {
        xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    }

    memcpy(stats, &g_stats, sizeof(APP_SDCard_Stats_t));
    stats->sd_free_bytes = MX_FileX_GetFreeSpace();

    if (g_sd_mutex) {
        xSemaphoreGive(g_sd_mutex);
    }
}

void APP_SDCard_ResetStats(void)
{
    if (g_sd_mutex) {
        xSemaphoreTake(g_sd_mutex, portMAX_DELAY);
    }

    memset(&g_stats, 0, sizeof(g_stats));

    if (g_sd_mutex) {
        xSemaphoreGive(g_sd_mutex);
    }
}

/* ============================================================================
 * End of File
 * ========================================================================== */
