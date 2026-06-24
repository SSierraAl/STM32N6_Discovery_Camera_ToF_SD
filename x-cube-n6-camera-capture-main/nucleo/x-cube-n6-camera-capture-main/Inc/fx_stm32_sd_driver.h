/***************************************************************************
  * Copyright (c) 2024 Microsoft Corporation
  * Copyright (c) 2025 STMicroelectronics
  *
  * This program and the accompanying materials are made available under the
  * terms of the MIT License which is available at
  * https://opensource.org/licenses/MIT.
  *
  * SPDX-License-Identifier: MIT
  **************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FX_STM32_SD_DRIVER_H
#define FX_STM32_SD_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fx_api.h"
#include "stm32n6xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Default timeout used to wait for fx operations */
#define FX_STM32_SD_DEFAULT_TIMEOUT                           (10 * configTICK_RATE_HZ)

/* Let the filex low-level driver initialize the SD driver */
#define FX_STM32_SD_INIT                                 1

/* Disable cache maintenance (not needed for blocking mode) */
#define FX_STM32_SD_CACHE_MAINTENANCE                    0

/* Disable DMA API - use blocking mode instead */
#define FX_STM32_SD_DMA_API                              0

/* SDIO instance to be used by FileX */
#define FX_STM32_SD_INSTANCE                             0

/* Default sector size, used by the driver */
#define FX_STM32_SD_DEFAULT_SECTOR_SIZE                  512

/* Exported macro ------------------------------------------------------------*/
/* Define the CPU Cache maintenance macros */
#if (FX_STM32_SD_CACHE_MAINTENANCE == 1)
#define invalidate_cache_by_addr(__ptr__, __size__)                  SCB_InvalidateDCache_by_Addr((void *)__ptr__, (int32_t)__size__)
#define clean_cache_by_addr(__ptr__, __size__)                       SCB_CleanDCache_by_Addr((uint32_t *)__ptr__, (int32_t)__size__)
#endif
/* Define the macro to get the current time in ticks */
#define FX_STM32_SD_CURRENT_TIME()                            xTaskGetTickCount()

/* Macro called before initializing the SD driver */
#define FX_STM32_SD_PRE_INIT(_media_ptr)

/* Macro called after initializing the SD driver */
#define FX_STM32_SD_POST_INIT(_media_ptr)

/* Macro called after the SD deinit */
#define FX_STM32_SD_POST_DEINIT(_media_ptr)

/* Macro called after the abort request */
#define FX_STM32_SD_POST_ABORT(_media_ptr)

/* Macro called before performing read operation */
#define FX_STM32_SD_PRE_READ_TRANSFER(_media_ptr)

/* Macro called after performing read operation */
#define FX_STM32_SD_POST_READ_TRANSFER(_media_ptr)

/* Macro for read error handling */
#define FX_STM32_SD_READ_TRANSFER_ERROR(_status_)

/* Define how to notify about Read completion operation (blocking = no notify needed) */
#define FX_STM32_SD_READ_CPLT_NOTIFY()

/* Define how to notify about write completion operation (blocking = no notify needed) */
#define FX_STM32_SD_WRITE_CPLT_NOTIFY()

/* Macro called before performing write operation */
#define FX_STM32_SD_PRE_WRITE_TRANSFER(_media_ptr)

/* Macro called after performing write operation */
#define FX_STM32_SD_POST_WRITE_TRANSFER(_media_ptr)

/* Macro for write error handling */
#define FX_STM32_SD_WRITE_TRANSFER_ERROR(_status_)

/* Exported functions prototypes ---------------------------------------------*/


#define FX_STM32_SD_POST_INIT(_media_ptr)

/* USER CODE END FX_STM32_SD_POST_INIT */

/* Macro called after the SD deinit */
/* USER CODE BEGIN FX_STM32_SD_POST_DEINIT */

#define FX_STM32_SD_POST_DEINIT(_media_ptr)            do { \
                                                          vSemaphoreDelete(sd_rx_semaphore); \
                                                          vSemaphoreDelete(sd_tx_semaphore); \
                                                        } while(0)

/* USER CODE END FX_STM32_SD_POST_DEINIT */

/* Macro called after the abort request */
/* USER CODE BEGIN FX_STM32_SD_POST_ABORT */

#define FX_STM32_SD_POST_ABORT(_media_ptr)

/* USER CODE END FX_STM32_SD_POST_ABORT */

/* Macro called before performing read operation */
/* USER CODE BEGIN FX_STM32_SD_PRE_READ_TRANSFER */

#define FX_STM32_SD_PRE_READ_TRANSFER(_media_ptr)

/* USER CODE END FX_STM32_SD_PRE_READ_TRANSFER */

/* Macro called after performing read operation */
/* USER CODE BEGIN FX_STM32_SD_POST_READ_TRANSFER */

#define FX_STM32_SD_POST_READ_TRANSFER(_media_ptr)

/* USER CODE END FX_STM32_SD_POST_READ_TRANSFER */

/* Macro for read error handling */
/* USER CODE BEGIN FX_STM32_SD_READ_TRANSFER_ERROR */

#define FX_STM32_SD_READ_TRANSFER_ERROR(_status_)

/* USER CODE END FX_STM32_SD_READ_TRANSFER_ERROR */

/* Define how to notify about Read completion operation */

/* USER CODE BEGIN FX_STM32_SD_READ_CPLT_NOTIFY */

#define FX_STM32_SD_READ_CPLT_NOTIFY()                 do { \
                                                          if(xSemaphoreTake(sd_rx_semaphore, FX_STM32_SD_DEFAULT_TIMEOUT) != pdTRUE) \
                                                            { \
                                                              return FX_IO_ERROR; \
                                                            } \
                                                        } while(0)

/* USER CODE END FX_STM32_SD_READ_CPLT_NOTIFY */

/* Define how to notify about write completion operation */
/* USER CODE BEGIN FX_STM32_SD_WRITE_CPLT_NOTIFY */

#define FX_STM32_SD_WRITE_CPLT_NOTIFY()                do { \
                                                          if(xSemaphoreTake(sd_tx_semaphore, FX_STM32_SD_DEFAULT_TIMEOUT) != pdTRUE) \
                                                            { \
                                                              return FX_IO_ERROR; \
                                                            } \
                                                        } while(0)

/* USER CODE END FX_STM32_SD_WRITE_CPLT_NOTIFY */

/* Macro called before performing write operation */
/* USER CODE BEGIN FX_STM32_SD_PRE_WRITE_TRANSFER */

#define FX_STM32_SD_PRE_WRITE_TRANSFER(_media_ptr)

/* USER CODE END FX_STM32_SD_PRE_WRITE_TRANSFER */

/* Macro called after performing write operation */
/* USER CODE BEGIN FX_STM32_SD_POST_WRITE_TRANSFER */

#define FX_STM32_SD_POST_WRITE_TRANSFER(_media_ptr)

/* USER CODE END FX_STM32_SD_POST_WRITE_TRANSFER */

/* Macro for write error handling */
/* USER CODE BEGIN FX_STM32_SD_WRITE_TRANSFER_ERROR */

#define FX_STM32_SD_WRITE_TRANSFER_ERROR(_status_)

/* USER CODE END FX_STM32_SD_WRITE_TRANSFER_ERROR */

/* Exported functions prototypes ---------------------------------------------*/

INT fx_stm32_sd_init(UINT instance);
INT fx_stm32_sd_deinit(UINT instance);

INT fx_stm32_sd_get_status(UINT instance);

INT fx_stm32_sd_read_blocks(UINT instance, UINT *buffer, UINT start_block, UINT total_blocks);
INT fx_stm32_sd_write_blocks(UINT instance, UINT *buffer, UINT start_block, UINT total_blocks);

VOID fx_stm32_sd_driver(FX_MEDIA *media_ptr);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif

#endif /* FX_STM32_SD_DRIVER_H */
