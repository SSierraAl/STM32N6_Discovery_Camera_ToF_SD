**
 * @file    app_filex.h
 * @brief   FileX and SDMMC integration for FreeRTOS
 * @note    Configured for x-cube-n6-camera-capture-main
 * @date    June 2026
 */

#ifndef APP_FILEX_H
#define APP_FILEX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include FreeRTOS FIRST before FileX drivers that depend on it */
#include "FreeRTOS.h"
#include "semphr.h"

#include "fx_api.h"
#include "fx_stm32_sd_driver.h"

/* ============================================================================
 * FileX Media Definitions
 * ========================================================================== */

#define FX_MEDIA_NAME_SIZE          64
#define FX_MEDIA_BUFFER_SIZE        (512)
#define FX_MAX_MEDIA_COUNT          2

/* Media instance IDs */
#define FX_MEDIA_SD_ID              0
#define FX_MEDIA_SRAM_ID            1

/* ============================================================================
 * Global Media Instances
 * ========================================================================== */

extern FX_MEDIA    g_fx_media_sd;      /* SD card media */
extern FX_MEDIA    g_fx_media_sram;    /* SRAM media (optional) */

/* ============================================================================
 * FileX Initialization Function
 * ========================================================================== */

/**
 * @brief Initialize FileX and mount SD card
 * @note  Call from FreeRTOS task context (not interrupt)
 * @return 0 on success, non-zero on error
 */
UINT MX_FileX_Init(void);

/**
 * @brief Deinitialize FileX and close media
 * @return 0 on success
 */
UINT MX_FileX_DeInit(void);

/**
 * @brief Check if SD card is mounted and ready
 * @return 1 if ready, 0 if not
 */
int MX_FileX_IsSD_Ready(void);

/**
 * @brief Get free space on SD card (in bytes)
 * @return Available bytes, or 0 if error
 */
ULONG MX_FileX_GetFreeSpace(void);

/* ============================================================================
 * Debug/Logging
 * ========================================================================== */

#include <stdio.h>

#define FILEX_DEBUG_ENABLED 1

#if FILEX_DEBUG_ENABLED
  #define FILEX_PRINT(fmt, ...) \
    printf("[FileX] " fmt, ##__VA_ARGS__)
#else
  #define FILEX_PRINT(fmt, ...) do {} while(0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_FILEX_H */
