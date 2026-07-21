/**
 ******************************************************************************
 * @file    i2c_arbiter.c
 * @brief   Mutual-exclusion arbiter for the shared I2C1 bus. See i2c_arbiter.h.
 ******************************************************************************
 */
#include "i2c_arbiter.h"
#include "stm32n6570_discovery_bus.h"

SemaphoreHandle_t g_i2c1_mutex = NULL;

void I2C_Arbiter_Init(void)
{
    if (g_i2c1_mutex == NULL) {
        g_i2c1_mutex = xSemaphoreCreateMutex();
    }
}

int32_t I2C_Arbiter_CameraWriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
    int32_t ret;
    if (g_i2c1_mutex) xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
    ret = BSP_I2C1_WriteReg16(DevAddr, Reg, pData, Length);
    if (g_i2c1_mutex) xSemaphoreGive(g_i2c1_mutex);
    return ret;
}

int32_t I2C_Arbiter_CameraReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
    int32_t ret;
    if (g_i2c1_mutex) xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
    ret = BSP_I2C1_ReadReg16(DevAddr, Reg, pData, Length);
    if (g_i2c1_mutex) xSemaphoreGive(g_i2c1_mutex);
    return ret;
}
