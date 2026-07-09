/**
 * @file    app_dma2d.c
 * @brief   DMA2D Hardware Accelerator Implementation
 * 
 * Uses DMA2D peripheral for:
 * - Fast memory-to-memory copy (M2M mode)
 * - Hardware pixel format conversion (M2M_PFC mode)
 * - Non-blocking transfers with interrupt support
 */

#include <stdio.h>
#include <string.h>
#include "app_dma2d.h"
#include "FreeRTOS.h"
#include "task.h"

/* DMA2D Global Handle */
static DMA2D_HandleTypeDef hdma2d;

/* Callback support for interrupt-driven transfers */
static DMA2D_Callback_t g_dma2d_callback = NULL;
static void *g_dma2d_userdata = NULL;
static SemaphoreHandle_t g_dma2d_done_sem = NULL;

/**
 * @brief DMA2D Transfer Complete Callback (HAL)
 */
void HAL_DMA2D_XferCpltCallback(DMA2D_HandleTypeDef *hdma2d)
{
    if (g_dma2d_done_sem != NULL) {
        xSemaphoreGiveFromISR(g_dma2d_done_sem, NULL);
    }
    if (g_dma2d_callback != NULL) {
        g_dma2d_callback(g_dma2d_userdata);
    }
}

/**
 * @brief DMA2D Transfer Error Callback (HAL)
 */
void HAL_DMA2D_XferErrorCallback(DMA2D_HandleTypeDef *hdma2d)
{
    if (g_dma2d_done_sem != NULL) {
        xSemaphoreGiveFromISR(g_dma2d_done_sem, NULL);
    }
}

/**
 * @brief Initialize DMA2D peripheral
 */
int DMA2D_Init(void)
{
    /* Create semaphore for non-blocking operations */
    g_dma2d_done_sem = xSemaphoreCreateBinary();
    if (g_dma2d_done_sem == NULL) {
        return -1;
    }
    xSemaphoreTake(g_dma2d_done_sem, 0); /* Start empty */

    /* DMA2D Hardware Initialization */
    hdma2d.Instance = DMA2D;
    hdma2d.Init.Mode = DMA2D_M2M;                    /* Default: Memory-to-Memory */
    hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;     /* Default output: 2 bytes/pixel */
    hdma2d.Init.OutputOffset = 0;
    hdma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;
    hdma2d.Init.RedBlueSwap = DMA2D_RB_REGULAR;
    hdma2d.Init.BytesSwap = DMA2D_BYTES_REGULAR;
    hdma2d.Init.LineOffsetMode = DMA2D_LOM_PIXELS;

    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
        return -1;
    }

    /* Configure foreground layer (input) */
    hdma2d.LayerCfg[1].InputOffset = 0;
    hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
    hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[1].InputAlpha = 0;
    hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
    hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
    hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;

    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
        return -1;
    }

    /* Enable DMA2D Interrupt */
    HAL_NVIC_SetPriority(DMA2D_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);

    printf("[DMA2D] Initialized\n");
    return 0;
}

/**
 * @brief Fast memory-to-memory copy
 */
int DMA2D_Copy(void *src, void *dst, uint32_t width, uint32_t height, uint32_t bytes_per_pixel)
{
    if (src == NULL || dst == NULL || width == 0 || height == 0) {
        return -1;
    }

    /* Select input/output format based on bytes_per_pixel */
    uint32_t input_fmt, output_fmt;
    
    switch (bytes_per_pixel) {
        case 2:
            input_fmt = DMA2D_INPUT_RGB565;    /* 2-byte format (YUV422 passthrough) */
            output_fmt = DMA2D_OUTPUT_RGB565;
            break;
        case 3:
            input_fmt = DMA2D_INPUT_RGB888;
            output_fmt = DMA2D_OUTPUT_RGB888;
            break;
        case 4:
            input_fmt = DMA2D_INPUT_ARGB8888;
            output_fmt = DMA2D_OUTPUT_ARGB8888;
            break;
        default:
            return -1; /* Unsupported format */
    }

    /* Configure DMA2D for M2M transfer */
    hdma2d.Init.Mode = DMA2D_M2M;
    hdma2d.Init.ColorMode = output_fmt;
    hdma2d.Init.OutputOffset = 0;
    
    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
        return -1;
    }

    /* Configure input layer */
    hdma2d.LayerCfg[1].InputColorMode = input_fmt;
    hdma2d.LayerCfg[1].InputOffset = 0;
    
    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
        return -1;
    }

    /* Start transfer (blocking) */
    if (HAL_DMA2D_Start(&hdma2d, (uint32_t)src, (uint32_t)dst, width, height) != HAL_OK) {
        return -1;
    }

    /* Wait for completion */
    if (HAL_DMA2D_PollForTransfer(&hdma2d, 1000) != HAL_OK) {
        return -1;
    }

    return 0;
}

/**
 * @brief Copy with pixel format conversion
 */
int DMA2D_CopyConvert(void *src, void *dst, uint32_t width, uint32_t height,
                      DMA2D_PixelFormat_t src_fmt, DMA2D_PixelFormat_t dst_fmt)
{
    if (src == NULL || dst == NULL || width == 0 || height == 0) {
        return -1;
    }

    /* Configure DMA2D for M2M_PFC mode (with format conversion) */
    hdma2d.Init.Mode = DMA2D_M2M_PFC;
    
    /* Map destination format to output mode */
    switch (dst_fmt) {
        case FMT_RGB565:
            hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
            break;
        case FMT_RGB888:
            hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB888;
            break;
        case FMT_ARGB8888:
            hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
            break;
        default:
            return -1;
    }
    
    hdma2d.Init.OutputOffset = 0;
    
    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
        return -1;
    }

    /* Configure input layer with source format */
    hdma2d.LayerCfg[1].InputColorMode = src_fmt;
    hdma2d.LayerCfg[1].InputOffset = 0;
    hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
    
    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
        return -1;
    }

    /* Start conversion transfer */
    if (HAL_DMA2D_Start(&hdma2d, (uint32_t)src, (uint32_t)dst, width, height) != HAL_OK) {
        return -1;
    }

    /* Wait for completion */
    if (HAL_DMA2D_PollForTransfer(&hdma2d, 1000) != HAL_OK) {
        return -1;
    }

    return 0;
}

/**
 * @brief Non-blocking copy with callback
 */
int DMA2D_CopyIT(void *src, void *dst, uint32_t width, uint32_t height,
                 uint32_t bytes_per_pixel, DMA2D_Callback_t callback, void *userdata)
{
    if (src == NULL || dst == NULL || width == 0 || height == 0) {
        return -1;
    }

    /* Save callback info */
    g_dma2d_callback = callback;
    g_dma2d_userdata = userdata;

    /* Empty the semaphore before starting */
    if (g_dma2d_done_sem != NULL) {
        xSemaphoreTake(g_dma2d_done_sem, 0);
    }

    /* Configure transfer (same as DMA2D_Copy) */
    uint32_t input_fmt, output_fmt;
    
    switch (bytes_per_pixel) {
        case 2:
            input_fmt = DMA2D_INPUT_RGB565;
            output_fmt = DMA2D_OUTPUT_RGB565;
            break;
        case 3:
            input_fmt = DMA2D_INPUT_RGB888;
            output_fmt = DMA2D_OUTPUT_RGB888;
            break;
        case 4:
            input_fmt = DMA2D_INPUT_ARGB8888;
            output_fmt = DMA2D_OUTPUT_ARGB8888;
            break;
        default:
            return -1;
    }

    hdma2d.Init.Mode = DMA2D_M2M;
    hdma2d.Init.ColorMode = output_fmt;
    hdma2d.Init.OutputOffset = 0;
    
    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
        return -1;
    }

    hdma2d.LayerCfg[1].InputColorMode = input_fmt;
    hdma2d.LayerCfg[1].InputOffset = 0;
    
    if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
        return -1;
    }

    /* Start interrupt-driven transfer */
    if (HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)src, (uint32_t)dst, width, height) != HAL_OK) {
        return -1;
    }

    return 0;
}

/**
 * @brief Wait for DMA2D transfer to complete
 */
int DMA2D_WaitForComplete(uint32_t timeout_ms)
{
    if (g_dma2d_done_sem == NULL) {
        return -1;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(g_dma2d_done_sem, ticks) != pdTRUE) {
        return -1; /* Timeout */
    }

    return 0;
}

/**
 * @brief Check if DMA2D is busy
 */
int DMA2D_IsBusy(void)
{
    return (HAL_DMA2D_GetState(&hdma2d) == HAL_DMA2D_STATE_BUSY) ? 1 : 0;
}

/**
 * @brief Fill rectangular region with color
 */
void DMA2D_FillRect(void *dst, uint32_t x, uint32_t y, uint32_t width, uint32_t height,
                    uint32_t color, uint32_t bytes_per_pixel)
{
    if (dst == NULL || width == 0 || height == 0) {
        return;
    }

    /* Select output format */
    uint32_t output_fmt;
    switch (bytes_per_pixel) {
        case 2:
            output_fmt = DMA2D_OUTPUT_RGB565;
            break;
        case 3:
            output_fmt = DMA2D_OUTPUT_RGB888;
            break;
        case 4:
            output_fmt = DMA2D_OUTPUT_ARGB8888;
            break;
        default:
            return;
    }

    /* Configure DMA2D for R2M mode (Register-to-Memory) */
    hdma2d.Init.Mode = DMA2D_R2M;
    hdma2d.Init.ColorMode = output_fmt;
    hdma2d.Init.OutputOffset = 0;
    
    if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
        return;
    }

    /* Start fill operation */
    if (HAL_DMA2D_Start(&hdma2d, color, (uint32_t)dst, width, height) != HAL_OK) {
        return;
    }

    /* Wait for completion */
    HAL_DMA2D_PollForTransfer(&hdma2d, 1000);
}

/**
 * @brief DMA2D Interrupt Handler
 */
void DMA2D_IRQHandler(void)
{
    HAL_DMA2D_IRQHandler(&hdma2d);
}