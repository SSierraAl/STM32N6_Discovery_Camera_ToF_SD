/**
 ******************************************************************************
 * @file    i2c_arbiter.h
 * @brief   Mutual-exclusion arbiter for the shared I2C1 bus.
 *
 *   The IMX335 camera (via BSP_I2C1_*) and the VL53L5CX ToF sensor (via the
 *   raw `hi2c1` handle in platform.c) are TWO INDEPENDENT HAL I2C handle
 *   instances driving the SAME physical I2C1 peripheral. HAL_I2C transactions
 *   are multi-step register sequences (start/address/data/stop); if two
 *   RTOS tasks issue them concurrently on the same physical bus (e.g. the
 *   camera's ISP background process and the ToF ranging loop both running
 *   continuously in CAPTURE_MODE=1), they can interleave mid-transaction and
 *   corrupt the bus protocol — this is the root cause of the historical
 *   "I2C conflict — CMW_CAMERA_Init fails (ret=-7)" issue.
 *
 *   Every I2C1 entry point (camera side: cmw_io.h macros; ToF side:
 *   platform.c) takes this mutex before touching the bus and releases it
 *   right after, so the two subsystems can safely run at the same time.
 ******************************************************************************
 */
#ifndef I2C_ARBITER_H
#define I2C_ARBITER_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Global mutex protecting the physical I2C1 bus. Created once in
    main_freertos() BEFORE any task is started, so it is always valid by
    the time camera_task / sensor_task / btn_thread begin running. */
extern SemaphoreHandle_t g_i2c1_mutex;

/** Must be called once, before the FreeRTOS scheduler starts. */
void I2C_Arbiter_Init(void);

/** Locked wrappers around the camera's BSP I2C entry points (Discovery
    board: BSP_I2C1_WriteReg16 / BSP_I2C1_ReadReg16). cmw_io.h redirects
    CMW_I2C_WRITEREG16 / CMW_I2C_READREG16 to these so every camera/ISP
    register access (including the ones the closed-source ISP background
    process issues internally) is serialized against the ToF driver. */
int32_t I2C_Arbiter_CameraWriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t I2C_Arbiter_CameraReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif /* I2C_ARBITER_H */
