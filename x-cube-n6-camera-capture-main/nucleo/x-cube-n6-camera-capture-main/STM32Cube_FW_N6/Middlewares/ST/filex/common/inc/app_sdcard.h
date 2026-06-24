/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_sdcard.h
  * @author  MCD Application Team
  * @brief   SD card storage helper API.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __APP_SDCARD_H__
#define __APP_SDCARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "fx_api.h"

UINT APP_SDCard_Init(FX_MEDIA *media_ptr);
UINT APP_SDCard_DeInit(void);
UINT APP_SDCard_CreateDirectory(const CHAR *directory_name);
UINT APP_SDCard_DeleteFile(const CHAR *file_name);
UINT APP_SDCard_WriteFile(const CHAR *file_name, const UCHAR *buffer, ULONG length);
UINT APP_SDCard_ReadFile(const CHAR *file_name, UCHAR *buffer, ULONG buffer_size, ULONG *actual_size);
UINT APP_SDCard_GetFreeSpace(ULONG *available_bytes);
UINT APP_SDCard_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_SDCARD_H__ */