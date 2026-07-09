/**
 * @file    app_dma2d.h
 * @brief   DMA2D Hardware Accelerator for Image Processing
 * 
 * Provides:
 * - Fast M2M memory copy (2-5x faster than CPU memcpy)
 * - Hardware pixel format conversion (YUV422 -> RGB565/RGB888)
 * - Non-blocking transfer with callback support
 */

#ifndef APP_DMA2D_H
#define APP_DMA2D_H

#include <stdint.h>
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_dma2d.h"

/* DMA2D Transfer Modes */
typedef enum {
    DMA2D_COPY_ONLY,      /* Simple memory copy */
    DMA2D_COPY_CONVERT,   /* Copy with format conversion */
    DMA2D_FILL_RECT       /* Fill rectangular region with color */
} DMA2D_TransferMode_t;

/* Pixel Format Identifiers */
typedef enum {
    FMT_RGB565  = DMA2D_INPUT_RGB565,
    FMT_RGB888  = DMA2D_INPUT_RGB888,
    FMT_ARGB8888 = DMA2D_INPUT_ARGB8888,
    FMT_YCBCR   = DMA2D_INPUT_YCBCR,
    FMT_L8      = DMA2D_INPUT_L8,
    FMT_A8      = DMA2D_INPUT_A8
} DMA2D_PixelFormat_t;

/* DMA2D Transfer Complete Callback Type */
typedef void (*DMA2D_Callback_t)(void *userdata);

/**
 * @brief Initialize DMA2D peripheral
 * @return 0 on success, -1 on failure
 */
int DMA2D_Init(void);

/**
 * @brief Fast memory-to-memory copy using DMA2D
 * @param src Source buffer address
 * @param dst Destination buffer address  
 * @param width Image width in pixels
 * @param height Image height in pixels
 * @param bytes_per_pixel Bytes per pixel (2 for RGB565/YUV422, 3 for RGB888, 4 for ARGB8888)
 * @return 0 on success, -1 on failure
 */
int DMA2D_Copy(void *src, void *dst, uint32_t width, uint32_t height, uint32_t bytes_per_pixel);

/**
 * @brief Copy with pixel format conversion
 * @param src Source buffer
 * @param dst Destination buffer
 * @param width Width in pixels
 * @param height Height in pixels
 * @param src_fmt Source pixel format (DMA2D_PixelFormat_t)
 * @param dst_fmt Destination pixel format (DMA2D_PixelFormat_t)
 * @return 0 on success, -1 on failure
 */
int DMA2D_CopyConvert(void *src, void *dst, uint32_t width, uint32_t height, 
                      DMA2D_PixelFormat_t src_fmt, DMA2D_PixelFormat_t dst_fmt);

/**
 * @brief Non-blocking copy with callback
 * @param src Source buffer
 * @param dst Destination buffer
 * @param width Width in pixels
 * @param height Height in pixels
 * @param bytes_per_pixel Bytes per pixel
 * @param callback Function to call on transfer complete
 * @param userdata User data passed to callback
 * @return 0 on success, -1 on failure
 */
int DMA2D_CopyIT(void *src, void *dst, uint32_t width, uint32_t height, 
                 uint32_t bytes_per_pixel, DMA2D_Callback_t callback, void *userdata);

/**
 * @brief Wait for ongoing DMA2D transfer to complete
 * @param timeout_ms Timeout in milliseconds
 * @return 0 on success, -1 on timeout
 */
int DMA2D_WaitForComplete(uint32_t timeout_ms);

/**
 * @brief Check if DMA2D is busy
 * @return 1 if busy, 0 if ready
 */
int DMA2D_IsBusy(void);

/**
 * @brief Fill rectangular region with color
 * @param dst Destination buffer
 * @param x X coordinate
 * @param y Y coordinate  
 * @param width Fill width in pixels
 * @param height Fill height in pixels
 * @param color Color value (depends on output format)
 * @param bytes_per_pixel Bytes per pixel
 */
void DMA2D_FillRect(void *dst, uint32_t x, uint32_t y, uint32_t width, uint32_t height,
                    uint32_t color, uint32_t bytes_per_pixel);

#endif /* APP_DMA2D_H */