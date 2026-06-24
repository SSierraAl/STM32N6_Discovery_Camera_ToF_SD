/**
 * @file    app_sdcard.h
 * @brief   SD Card helper functions for camera frame storage
 * @note    Built on top of FileX
 * @date    June 2026
 */

#ifndef APP_SDCARD_H
#define APP_SDCARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "fx_api.h"

/* ============================================================================
 * Frame Storage Constants
 * ========================================================================== */

#define SD_FILENAME_MAX          64
#define SD_RAW_FORMAT            "frame_%06lu.raw"
#define SD_JPEG_FORMAT           "frame_%06lu.jpg"
#define SD_METADATA_FORMAT       "frame_%06lu.txt"

/* ============================================================================
 * Frame Storage Functions
 * ========================================================================== */

/**
 * @brief Store a raw frame to SD card
 * @param frame_number  Sequential frame counter
 * @param buffer        Pointer to frame data
 * @param size          Size of frame in bytes
 * @return              0 on success, non-zero on error
 */
uint32_t APP_SDCard_StoreFrame_RAW(uint32_t frame_number, 
                                   const uint8_t *buffer, 
                                   uint32_t size);

/**
 * @brief Store a JPEG-compressed frame to SD card
 * @param frame_number  Sequential frame counter
 * @param buffer        Pointer to JPEG data
 * @param size          Size of JPEG in bytes
 * @return              0 on success, non-zero on error
 */
uint32_t APP_SDCard_StoreFrame_JPEG(uint32_t frame_number,
                                    const uint8_t *buffer,
                                    uint32_t size);

/**
 * @brief Store frame metadata (timestamp, resolution, etc.)
 * @param frame_number  Sequential frame counter
 * @param metadata      Metadata string (e.g., "Resolution: 1024x768\n")
 * @return              0 on success, non-zero on error
 */
uint32_t APP_SDCard_StoreFrameMetadata(uint32_t frame_number,
                                       const char *metadata);

/**
 * @brief Check if SD card has enough space for a frame
 * @param frame_size    Size of frame to store (bytes)
 * @return              1 if enough space, 0 if not enough space
 */
int APP_SDCard_HasSpace(uint32_t frame_size);

/**
 * @brief Get current session directory name
 * @param buffer        Pointer to store directory name
 * @param size          Size of buffer
 * @return              0 on success, non-zero on error
 */
uint32_t APP_SDCard_GetSessionDir(char *buffer, uint32_t size);

/**
 * @brief Get free space on SD card in bytes
 * @return              Free bytes available
 */
uint32_t APP_SDCard_GetFreeSpace(void);

/**
 * @brief Initialize SD card storage system
 * @note  Must be called after MX_FileX_Init()
 */
void APP_SDCard_Init(void);

/* ============================================================================
 * Statistics and Monitoring
 * ========================================================================== */

typedef struct {
    uint32_t frames_stored;         /* Total frames written */
    uint32_t total_bytes;           /* Total bytes written */
    uint32_t errors;                /* Number of write errors */
    uint32_t sd_free_bytes;         /* Current free space */
} APP_SDCard_Stats_t;

/**
 * @brief Get current SD card statistics
 * @param stats         Pointer to stats structure
 */
void APP_SDCard_GetStats(APP_SDCard_Stats_t *stats);

/**
 * @brief Reset statistics counters
 */
void APP_SDCard_ResetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SDCARD_H */
