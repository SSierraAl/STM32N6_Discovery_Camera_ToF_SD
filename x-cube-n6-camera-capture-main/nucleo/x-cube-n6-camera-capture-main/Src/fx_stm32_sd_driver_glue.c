/**
 ******************************************************************************
 * @file    fx_stm32_sd_driver_glue.c
 * @brief   SDMMC2 GPIO/MSP initialization and FileX glue layer.
 *
 *   This module provides two things:
 *   1. HAL_SD_MspInit() — configures SDMMC2 GPIO pins and enables VDDIO5
 *   2. fx_stm32_sd_*() — FileX driver glue functions (init, read, write)
 *
 *   In standalone mode, the FileX functions are NOT used (raw HAL_SD_*
 *   calls go directly to the SD card). However, the glue layer is kept
 *   for build compatibility in case FileX is re-enabled later.
 *
 *   Hardware: SDMMC2 on NUCLEO-N657X0
 *     - PC0  = CMD
 *     - PC2  = CLK
 *     - PC3  = D0
 *     - PC4  = D1
 *     - PC5  = D2
 *     - PE4  = D3
 *     - VDDIO5 must be enabled before SDMMC2 GPIO AF works
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_sd.h"
#include "stm32n6xx_hal_rcc_ex.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifndef UINT
#define UINT unsigned int
#endif
#ifndef INT
#define INT int
#endif

/** External SD handle declared in main.c */
extern SD_HandleTypeDef hsd1;

/** DMA TX completion semaphore (FileX glue) */
SemaphoreHandle_t sd_tx_semaphore;

/** DMA RX completion semaphore (FileX glue) */
SemaphoreHandle_t sd_rx_semaphore;

/** SD init status flag */
static volatile int sd_init_ok = 0;

/* ================================================================
 HAL_SD_MspInit — Called by HAL_SD_Init() automatically
 ================================================================ */

/**
 * @brief  Initialize SDMMC2 GPIO pins, clock, and interrupt.
 *
 *   This function is called automatically by HAL_SD_Init() before
 *   the SD card initialization sequence. It configures:
 *   - VDDIO5 power domain (required for SDMMC2 AF GPIO)
 *   - SDMMC2 peripheral clock (HCLK)
 *   - GPIO pins PC0,PC2-PC5 and PE4 as AF11_SDMMC2
 *   - SDMMC2 interrupt (priority 5)
 *
 * @param  sdHandle  SD handle (instance = SDMMC2)
 */
void HAL_SD_MspInit(SD_HandleTypeDef *sdHandle) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if (sdHandle->Instance == SDMMC2) {
		/* ---- Enable VDDIO5 power domain ----
		 SDMMC2 alternate function GPIOs are controlled by VDDIO5.
		 Must be enabled before configuring SDMMC2 GPIOs. */
		HAL_PWREx_EnableVddIO5();

		/* Wait for VDDIO5 to stabilize (~625us at 800MHz) */
		for (volatile uint32_t d = 0; d < 500000; d++)
			;
		printf("[SD] VDDIO5 enabled + settled\n");

		/* ---- Configure SDMMC2 peripheral clock source (HCLK) ---- */
		RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC2;
		PeriphClkInit.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_HCLK;
		HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
		printf("[SD] Peripheral clock source configured (HCLK)\n");

		/* ---- Enable clocks ---- */
		__HAL_RCC_SDMMC2_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();
		__HAL_RCC_SYSCFG_CLK_ENABLE();

		/* ---- Configure SDMMC2 GPIO pins (AF11, push-pull, pull-up) ---- */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;

		/* PC0 (CMD), PC2 (CLK), PC3 (D0), PC4 (D1), PC5 (D2) */
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
				| GPIO_PIN_5;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* PE4 (D3) */
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		/* ---- Enable SDMMC2 interrupt (priority 5) ---- */
		HAL_NVIC_SetPriority(SDMMC2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(SDMMC2_IRQn);
	}
}

/* ================================================================
 SDMMC2 IRQ Handler
 ================================================================ */

/**
 * @brief  SDMMC2 interrupt handler.
 *
 *   Called when SDMMC2 generates an interrupt (DMA transfer complete,
 *   error, etc.). Delegates to the HAL SD IRQ handler.
 */
void SDMMC2_IRQHandler(void) {
	HAL_SD_IRQHandler(&hsd1);
}

/* ================================================================
 FileX Glue Layer (init / deinit / read / write)
 Not used in standalone mode — retained for build compatibility.
 ================================================================ */

/**
 * @brief  FileX SD media initialization.
 *
 *   Resets SDMMC2, initializes in 1-bit mode, then switches to 4-bit.
 *   Creates DMA TX/RX semaphores for FileX.
 *
 * @param  instance  SD instance number (ignored, always instance 0)
 * @return 0 on success, 1 on failure
 */
INT fx_stm32_sd_init(UINT instance) {
	(void) instance;
	HAL_StatusTypeDef status;

	/* ---- Reset SDMMC2 peripheral ---- */
	__HAL_RCC_SDMMC2_FORCE_RESET();
	for (volatile uint32_t d = 0; d < 1000; d++)
		;
	__HAL_RCC_SDMMC2_RELEASE_RESET();
	for (volatile uint32_t d = 0; d < 1000; d++)
		;

	/* ---- Configure SD handle (1-bit mode for init) ---- */
	hsd1.Instance = SDMMC2;
	hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B; /* Start 1-bit */
	hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd1.Init.ClockDiv = 2; /* 200MHz/(2x8) = 12.5MHz */
	hsd1.State = HAL_SD_STATE_RESET;

	status = HAL_SD_Init(&hsd1);

	/* Debug: show clock register state */
	printf("[SD] CLKCR = 0x%08lX (CLKEN = %s)\n", (unsigned long) SDMMC2->CLKCR,
			(SDMMC2->CLKCR & 0x1) ? "SET" : "NOT SET");
	printf("[SD] POWER = 0x%08lX\n", (unsigned long) SDMMC2->POWER);

	if (status != HAL_OK) {
		printf("[SD] HAL_SD_Init failed: %lu\n", (unsigned long) status);
		return 1;
	}

	/* ---- Switch to 4-bit bus width for better performance ---- */
	printf("[SD] Switching to 4-bit bus width...\n");
	status = HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);
	if (status == HAL_OK) {
		hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
		printf("[SD] 4-bit bus width configured successfully\n");
	} else {
		printf("[SD] Wide bus config failed (%lu), staying in 1-bit\n",
				(unsigned long) status);
	}

	printf("[SD] OK | Blocks: %lu | Size: %lu\n",
			(unsigned long) hsd1.SdCard.BlockNbr,
			(unsigned long) hsd1.SdCard.BlockSize);

	/* ---- Create DMA semaphores (only once) ---- */
	if (sd_tx_semaphore == NULL)
		sd_tx_semaphore = xSemaphoreCreateBinary();
	if (sd_rx_semaphore == NULL)
		sd_rx_semaphore = xSemaphoreCreateBinary();

	if ((sd_tx_semaphore == NULL) || (sd_rx_semaphore == NULL))
		return 1;

	sd_init_ok = 1;
	return 0;
}

/**
 * @brief  FileX SD media deinitialization.
 *
 * @param  instance  SD instance number (ignored)
 * @return 0 (always succeeds)
 */
INT fx_stm32_sd_deinit(UINT instance) {
	(void) instance;
	sd_init_ok = 0;
	return 0;
}

/**
 * @brief  Check SD card status (ready for transfer?).
 *
 * @param  instance  SD instance number (ignored)
 * @return 0 if card is in TRANSFER state (ready), 1 otherwise
 */
INT fx_stm32_sd_get_status(UINT instance) {
	(void) instance;
	return (HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) ? 0 : 1;
}

/**
 * @brief  Read blocks from SD card using DMA.
 *
 * @param  instance      SD instance number (ignored)
 * @param  buffer        Output buffer
 * @param  start_block   First block to read (0-based)
 * @param  total_blocks  Number of blocks to read
 * @return 0 on success, 1 on failure
 */
INT fx_stm32_sd_read_blocks(UINT instance, UINT *buffer,
UINT start_block, UINT total_blocks) {
	(void) instance;
	HAL_StatusTypeDef result = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t*) buffer,
			start_block, total_blocks);
	if (result != HAL_OK) {
		printf("[SD] READ FAIL: block=%lu count=%lu err=%lu\n", start_block,
				total_blocks, (unsigned long) result);
	}
	return (result != HAL_OK) ? 1 : 0;
}

/**
 * @brief  Write blocks to SD card using DMA.
 *
 * @param  instance      SD instance number (ignored)
 * @param  buffer        Input buffer
 * @param  start_block   First block to write (0-based)
 * @param  total_blocks  Number of blocks to write
 * @return 0 on success, 1 on failure
 */
INT fx_stm32_sd_write_blocks(UINT instance, UINT *buffer,
UINT start_block, UINT total_blocks) {
	(void) instance;
	HAL_StatusTypeDef result = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t*) buffer,
			start_block, total_blocks);
	if (result != HAL_OK) {
		printf("[SD] WRITE FAIL: block=%lu count=%lu err=%lu\n", start_block,
				total_blocks, (unsigned long) result);
	}
	return (result != HAL_OK) ? 1 : 0;
}

/* ================================================================
 HAL SD DMA Completion Callbacks (FileX glue)
 ================================================================ */

/**
 * @brief  Called when SD DMA TX transfer completes.
 * @param  hsd  SD handle (unused)
 */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
	(void) hsd;
	if (sd_tx_semaphore != NULL)
		xSemaphoreGive(sd_tx_semaphore);
}

/**
 * @brief  Called when SD DMA RX transfer completes.
 * @param  hsd  SD handle (unused)
 */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
	(void) hsd;
	if (sd_rx_semaphore != NULL)
		xSemaphoreGive(sd_rx_semaphore);
}
