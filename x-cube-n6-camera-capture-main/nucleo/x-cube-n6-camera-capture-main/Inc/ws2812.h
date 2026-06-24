/**
 * @file    ws2812.h
 * @brief   WS2812 RGB LED Driver - Minimal Version
 * @date    2026
 *
 * This is a minimal WS2812 driver for STM32 using TIM1_CH2 with DMA.
 * It includes debug configuration output for easy troubleshooting.
 *
 * DEPENDENCIES:
 *   - TIM1_CH2 configured with DMA (GPDMA1_Channel2)
 *   - PD7 configured as TIM1_CH2 alternate function
 *   - HAL_TIM_MODULE_ENABLED in stm32n6xx_hal_conf.h
 */

#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
   CONFIGURATION - Customize these for your hardware
   ================================================================ */

/** Number of WS2812 LEDs in the chain (MUST match your hardware) */
#define WS2812_NUM_LEDS             2

/** Default brightness (0-100%) */
#define WS2812_BRIGHTNESS_DEFAULT   10

/** Timer period (from your working configuration) */
#define WS2812_PERIOD               500

/** PWM values (from your working configuration) */
#define WS2812_PWM_ONE              80
#define WS2812_PWM_ZERO             30

/** Reset entries (must be > 50Âµs at your timer frequency) */
#define WS2812_RESET_ENTRIES        1700

/** Bits per LED (24-bit RGB) */
#define WS2812_BITS_PER_LED         24

/** Buffer size: (LEDs Ã— 24 bits) + reset entries */
#define WS2812_BUFFER_SIZE          (WS2812_NUM_LEDS * WS2812_BITS_PER_LED + WS2812_RESET_ENTRIES)

/* ================================================================
   EXTERNAL HANDLES (declared in main.c)
   ================================================================ */

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef handle_GPDMA1_Channel1;

/* ================================================================
   PUBLIC FUNCTIONS
   ================================================================ */

/**
 * @brief Initialize the WS2812 driver
 * @note Must be called after TIM1 and DMA initialization
 */
void WS2812_Init(void);

/**
 * @brief Set color using 32-bit RGB value (0xRRGGBB)
 * @param color 32-bit color (e.g., 0xFF0000 = Red)
 */
void WS2812_SetColor(uint32_t color);

/**
 * @brief Turn all LEDs ON (white)
 */
void WS2812_TurnOn(void);

/**
 * @brief Turn all LEDs OFF
 */
void WS2812_TurnOff(void);

/**
 * @brief Set global brightness
 * @param brightness 0-100
 */
void WS2812_SetBrightness(uint8_t brightness);

/**
 * @brief Get current brightness
 * @return brightness value (0-100)
 */
uint8_t WS2812_GetBrightness(void);

/**
 * @brief Send data to LEDs (updates all LEDs)
 * @note Called automatically by Set functions
 */
void WS2812_Update(void);


#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
