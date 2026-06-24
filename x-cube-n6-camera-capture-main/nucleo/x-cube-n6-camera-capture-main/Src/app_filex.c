/**
 * @file    app_filex.c
 * @brief   FileX middleware initialization for FreeRTOS
 * @note    Integrates with STM32 SDMMC2 hardware
 * @date    June 2026
 */
#include "FreeRTOS.h"
#include "task.h"

#include "app_filex.h"
#include "main.h"
#include "fx_stm32_sd_driver.h"
#include <stdio.h>

/* ============================================================================
 * Global Variables
 * ========================================================================== */

FX_MEDIA    g_fx_media_sd;
FX_MEDIA    g_fx_media_sram;

static int  g_filex_initialized = 0;

/* Buffer for FileX FX_MEDIA sector cache (SD card) */
ALIGN_32BYTES(uint32_t fx_sd_media_memory[FX_STM32_SD_DEFAULT_SECTOR_SIZE / sizeof(uint32_t)]);

/* ============================================================================
 * Internal Helper Functions
 * ========================================================================== */

/**
 * @brief Get human-readable error message
 */
static const char* filex_error_string(UINT error)
{
    switch (error) {
        case FX_SUCCESS:                     return "FX_SUCCESS";
        case FX_BOOT_ERROR:                  return "FX_BOOT_ERROR";
        case FX_MEDIA_INVALID:               return "FX_MEDIA_INVALID";
        case FX_NOT_FOUND:                   return "FX_NOT_FOUND";
        case FX_NOT_A_FILE:                  return "FX_NOT_A_FILE";
        case FX_NOT_DIRECTORY:               return "FX_NOT_DIRECTORY";
        case FX_ALREADY_CREATED:             return "FX_ALREADY_CREATED";
        case FX_WRITE_PROTECT:               return "FX_WRITE_PROTECT";
        case FX_NO_MORE_SPACE:               return "FX_NO_MORE_SPACE";
        case FX_INVALID_NAME:                return "FX_INVALID_NAME";
        case FX_IO_ERROR:                    return "FX_IO_ERROR";
        case FX_FILE_CORRUPT:                return "FX_FILE_CORRUPT";
        default:                             return "UNKNOWN_ERROR";
    }
}

/* ============================================================================
 * FileX System Initialization (called once at startup)
 * ========================================================================== */

UINT MX_FileX_Init(void)
{
    UINT status = FX_SUCCESS;

    if (g_filex_initialized) {
        FILEX_PRINT("Already initialized\n");
        return FX_SUCCESS;
    }

    FILEX_PRINT("Initializing FileX...\n");

    /* Initialize FileX system */
    fx_system_initialize();
    FILEX_PRINT("✓ fx_system_initialize() complete\n");

    /* Wait for SDMMC to be ready (optional, depends on your hardware init order) */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Open SD card media using the standard FileX driver interface */
    FILEX_PRINT("Opening SD card media...\n");
    status = fx_media_open(&g_fx_media_sd,
                            "SD_CARD",
                            fx_stm32_sd_driver,
                            (VOID *)FX_NULL,
                            (VOID *)fx_sd_media_memory,
                            sizeof(fx_sd_media_memory));

    if (status != FX_SUCCESS) {
        FILEX_PRINT("✗ SD card open failed: %s (0x%X)\n",
                    filex_error_string(status), status);
        return status;
    }

    FILEX_PRINT("✓ SD card mounted successfully\n");
    FILEX_PRINT("  Total sectors: %lu\n", g_fx_media_sd.fx_media_total_sectors);
    FILEX_PRINT("  Bytes per sector: %lu\n", g_fx_media_sd.fx_media_bytes_per_sector);

    /* Create SESSION directory for organized storage */
    uint32_t tick = HAL_GetTick();
    char session_dir[32];
    sprintf(session_dir, "/SESSION_%010lu", tick);

    FILEX_PRINT("Creating session directory: %s\n", session_dir);
    status = fx_directory_create(&g_fx_media_sd, session_dir);
    
    if (status == FX_ALREADY_CREATED) {
        FILEX_PRINT("✓ Session directory already exists\n");
    } else if (status != FX_SUCCESS) {
        FILEX_PRINT("✗ Failed to create session directory: %s (0x%X)\n",
                    filex_error_string(status), status);
        /* Non-fatal - continue anyway */
    } else {
        FILEX_PRINT("✓ Session directory created\n");
    }

    g_filex_initialized = 1;
    FILEX_PRINT("✓ FileX initialization complete\n");

    return FX_SUCCESS;
}

/* ============================================================================
 * FileX System Deinitialization
 * ========================================================================== */

UINT MX_FileX_DeInit(void)
{
    UINT status;

    if (!g_filex_initialized) {
        return FX_SUCCESS;
    }

    FILEX_PRINT("Deinitializing FileX...\n");

    /* Close media */
    status = fx_media_close(&g_fx_media_sd);
    if (status != FX_SUCCESS) {
        FILEX_PRINT("✗ Failed to close SD media: %s (0x%X)\n",
                    filex_error_string(status), status);
        return status;
    }

    g_filex_initialized = 0;
    FILEX_PRINT("✓ FileX deinitialized\n");

    return FX_SUCCESS;
}

/* ============================================================================
 * Query Functions
 * ========================================================================== */

int MX_FileX_IsSD_Ready(void)
{
    if (!g_filex_initialized) {
        return 0;
    }

    /* Check if media is still valid */
    if (g_fx_media_sd.fx_media_id != FX_MEDIA_ID) {
        return 0;  /* Media handle corrupted */
    }

    return 1;  /* Ready */
}

ULONG MX_FileX_GetFreeSpace(void)
{
    if (!MX_FileX_IsSD_Ready()) {
        return 0;
    }

    /* Calculate free space in bytes */
    ULONG available_clusters = g_fx_media_sd.fx_media_available_clusters;
    ULONG cluster_size = g_fx_media_sd.fx_media_bytes_per_sector * 
                         g_fx_media_sd.fx_media_sectors_per_cluster;

    return available_clusters * cluster_size;
}

/* ============================================================================
 * File Operations (Simple Wrapper)
 * ========================================================================== */

/**
 * @brief Write data to a file on SD card
 * @param filename  Path and filename (e.g., "/SESSION_xxx/frame_001.raw")
 * @param data      Pointer to data buffer
 * @param size      Number of bytes to write
 * @return          0 on success, non-zero on error
 */
UINT MX_FileX_Write(const char *filename, VOID *data, ULONG size)
{
    UINT status;
    FX_FILE file_handle;

    if (!MX_FileX_IsSD_Ready()) {
        FILEX_PRINT("✗ SD card not ready\n");
        return FX_MEDIA_INVALID;
    }

    /* Create/open file */
    status = fx_file_create(&g_fx_media_sd, (CHAR *)filename);
    if (status != FX_SUCCESS && status != FX_ALREADY_CREATED) {
        FILEX_PRINT("✗ Failed to create file %s: %s\n", 
                    filename, filex_error_string(status));
        return status;
    }

    /* Open file */
    status = fx_file_open(&g_fx_media_sd, &file_handle, (CHAR *)filename, 
                          FX_OPEN_FOR_WRITE);
    if (status != FX_SUCCESS) {
        FILEX_PRINT("✗ Failed to open file %s: %s\n", 
                    filename, filex_error_string(status));
        return status;
    }

    /* Seek to end */
    status = fx_file_seek(&file_handle, file_handle.fx_file_current_file_size);
    if (status != FX_SUCCESS) {
        fx_file_close(&file_handle);
        return status;
    }

    /* Write data */
    status = fx_file_write(&file_handle, data, size);
    if (status != FX_SUCCESS) {
        FILEX_PRINT("✗ Failed to write to file %s: %s\n", 
                    filename, filex_error_string(status));
        fx_file_close(&file_handle);
        return status;
    }

    /* Close file */
    status = fx_file_close(&file_handle);
    if (status != FX_SUCCESS) {
        FILEX_PRINT("✗ Failed to close file %s: %s\n", 
                    filename, filex_error_string(status));
        return status;
    }

    return FX_SUCCESS;
}

/**
 * @brief Read data from a file on SD card
 */
UINT MX_FileX_Read(const char *filename, VOID *data, ULONG size, ULONG *bytes_read)
{
    UINT status;
    FX_FILE file_handle;

    if (!MX_FileX_IsSD_Ready()) {
        return FX_MEDIA_INVALID;
    }

    /* Open file for read */
    status = fx_file_open(&g_fx_media_sd, &file_handle, (CHAR *)filename,
                          FX_OPEN_FOR_READ);
    if (status != FX_SUCCESS) {
        return status;
    }

    /* Read data */
    status = fx_file_read(&file_handle, data, size, bytes_read);

    /* Close file */
    fx_file_close(&file_handle);

    return status;
}

/* ============================================================================
 * End of File
 * ========================================================================== */
