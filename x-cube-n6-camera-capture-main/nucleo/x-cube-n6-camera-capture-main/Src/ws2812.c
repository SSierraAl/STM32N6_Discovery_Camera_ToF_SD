/**
 * @file    ws2812.c
 * @brief   WS2812 RGB LED Driver - Minimal Implementation
 */

#include "ws2812.h"
#include "stm32n6xx_hal.h"
#include <stdio.h>

/* ================================================================
   PRIVATE VARIABLES
   ================================================================ */

/** LED buffer (GRB format) */
static uint32_t ws2812_led_buffer[WS2812_NUM_LEDS];

/** PWM data buffer (DMA ready) */
static uint16_t ws2812_pwm_buffer[WS2812_BUFFER_SIZE];

/** DMA transfer complete flag */
static volatile bool ws2812_dma_done = false;

/** Current brightness (0-100) */
static uint8_t ws2812_brightness = WS2812_BRIGHTNESS_DEFAULT;

/** Current color */
static uint32_t ws2812_current_color = 0;

/* ================================================================
   PRIVATE FUNCTION PROTOTYPES
   ================================================================ */

static uint32_t WS2812_ApplyBrightness(uint32_t color);
static void WS2812_BuildPWMData(void);

/* ================================================================
   DMA CALLBACK (must be linked in stm32n6xx_it.c)
   ================================================================ */

/**
 * @brief HAL TIM PWM Pulse Finished Callback
 * @note Must be called from stm32n6xx_it.c or main
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
        ws2812_dma_done = true;
    }
}

/* ================================================================
   PUBLIC FUNCTIONS
   ================================================================ */

/**
 * @brief Initialize the WS2812 driver
 */
void WS2812_Init(void)
{
    // Clear LED buffer
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        ws2812_led_buffer[i] = 0;
    }

    // Enable TIM1 main output
    //TIM1->BDTR |= TIM_BDTR_MOE;

    printf("[WS2812] Init complete!\n");

    // Turn off initially
    WS2812_TurnOff();
}

/**
 * @brief Set color using 32-bit RGB value
 */
void WS2812_SetColor(uint32_t color)
{
    ws2812_current_color = color;
    
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    
    // Store as GRB format (WS2812 expects GRB)
    uint32_t grb_color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        ws2812_led_buffer[i] = grb_color;
    }
    
    WS2812_Update();
}

/**
 * @brief Turn all LEDs ON (white)
 */
void WS2812_TurnOn(void)
{
    WS2812_SetColor(0xFFFFFF);
    printf("[WS2812] ON (white)\r\n");
}

/**
 * @brief Turn all LEDs OFF
 */
void WS2812_TurnOff(void)
{
    WS2812_SetColor(0x000000);
    printf("[WS2812] OFF\r\n");
}

/**
 * @brief Set global brightness
 */
void WS2812_SetBrightness(uint8_t brightness)
{
    if (brightness > 100) brightness = 100;
    ws2812_brightness = brightness;
    // Re-send current color with new brightness
    WS2812_SetColor(ws2812_current_color);
}

/**
 * @brief Get current brightness
 */
uint8_t WS2812_GetBrightness(void)
{
    return ws2812_brightness;
}

/**
 * @brief Send data to LEDs
 */
void WS2812_Update(void)
{
    ws2812_dma_done = false;
    
    // Build PWM data from LED buffer
    WS2812_BuildPWMData();
    
    // Clean cache for DMA (required for STM32N6)
    SCB_CleanDCache_by_Addr((uint32_t*)ws2812_pwm_buffer, 
                            WS2812_BUFFER_SIZE * sizeof(uint16_t));
    
    // Start DMA transfer
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
                          (uint32_t*)ws2812_pwm_buffer, 
                          WS2812_BUFFER_SIZE);
    
    // Wait for completion (with timeout)
    uint32_t timeout = HAL_GetTick();
    while (!ws2812_dma_done) {
        if (HAL_GetTick() - timeout > 1000) {
            printf("[WS2812] ERROR: DMA timeout!\n");
            break;
        }
    }
}

/* ================================================================
   PRIVATE FUNCTIONS
   ================================================================ */

/**
 * @brief Apply brightness to a color
 */
static uint32_t WS2812_ApplyBrightness(uint32_t color)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    
    // Apply brightness scaling
    r = (r * ws2812_brightness) / 100;
    g = (g * ws2812_brightness) / 100;
    b = (b * ws2812_brightness) / 100;
    
    // Return as GRB format (WS2812 expects GRB, not RGB)
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

/**
 * @brief Build PWM data from LED buffer
 */
static void WS2812_BuildPWMData(void)
{
    uint32_t idx = 0;
    
    // For each LED
    for (int led = 0; led < WS2812_NUM_LEDS; led++) {
        uint32_t color = WS2812_ApplyBrightness(ws2812_led_buffer[led]);
        
        // Send 24 bits (MSB first)
        for (int bit = 23; bit >= 0; bit--) {
            if (color & (1U << bit)) {
                ws2812_pwm_buffer[idx++] = WS2812_PWM_ONE;
            } else {
                ws2812_pwm_buffer[idx++] = WS2812_PWM_ZERO;
            }
        }
    }
    
    // Reset pulse
    for (int i = 0; i < WS2812_RESET_ENTRIES; i++) {
        ws2812_pwm_buffer[idx++] = 0;
    }
}
