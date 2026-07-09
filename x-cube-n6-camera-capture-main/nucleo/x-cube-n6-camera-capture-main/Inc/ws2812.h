/**
 * @file    ws2812.h
 * @brief   WS2812 RGB LED Driver - Minimal Version
 * @date    2026
 *
 * This is a minimal WS2812 driver for STM32 using TIM1_CH2 with DMA.
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
#define WS2812_NUM_LEDS             12

/** Default brightness (0-100%) */
#define WS2812_BRIGHTNESS_DEFAULT   10

/** Timer period (from your working configuration) */
#define WS2812_PERIOD               500

/** PWM values (from your working configuration) */
#define WS2812_PWM_ONE              80
#define WS2812_PWM_ZERO             30

/** Reset entries (must be > 50µs at your timer frequency) */
#define WS2812_RESET_ENTRIES        1700

/** Bits per LED (24-bit RGB) */
#define WS2812_BITS_PER_LED         24

/** Buffer size: (LEDs × 24 bits) + reset entries */
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

/**
 * @brief Blocking strobe flash — turn LEDs on, wait, then turn off.
 * @param color       32-bit RGB color (0xRRGGBB)
 * @param brightness  0-100% brightness
 * @param duration_ms Flash duration in milliseconds
 *
 * Example: WS2812_Flash(0xFFFFFF, 100, 2);  // 2ms white flash at 100%
 */
void WS2812_Flash(uint32_t color, uint8_t brightness, uint32_t duration_ms);

/**
 * @brief Start non-blocking strobe flash.
 *        Turns LEDs on immediately. Use WS2812_FlashStop() to turn off.
 * @param color      32-bit RGB color (0xRRGGBB)
 * @param brightness 0-100% brightness
 */
void WS2812_FlashStart(uint32_t color, uint8_t brightness);

/**
 * @brief Stop the current flash immediately.
 */
void WS2812_FlashStop(void);

/**
 * @brief Auto-stop flash if duration elapsed.
 * @param duration_ms Maximum flash duration
 * @return 0 if flash still on, 1 if flash was stopped
 */
uint8_t WS2812_FlashCheck(uint32_t duration_ms);

/**
 * @brief Get flash start timestamp (for external timing control).
 * @return HAL_GetTick() value when flash was started
 */
uint32_t WS2812_FlashGetStartTime(void);

/**
 * @brief Start illumination for camera capture (BLOCKING).
 *        This is the MAIN function for insect capture illumination.
 *        LEDs turn ON, stay on for duration_ms, then turn OFF.
 *
 * @param color       Illumination color (0xRRGGBB)
 * @param brightness  Brightness 0-100%
 * @param duration_ms How long to keep LEDs on (ms)
 *
 * CRITICAL TIMING: The duration MUST cover the full camera capture cycle:
 *   - Camera init: ~185ms
 *   - Warmup frames: 11 × 33ms = 363ms
 *   - Capture frame: 33ms
 *   - Total: ~581ms
 *
 * Use WS2812_ILLUMINATION_MS from app_config.h (set to 500ms).
 */
void WS2812_Illuminate(uint32_t color, uint8_t brightness, uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */