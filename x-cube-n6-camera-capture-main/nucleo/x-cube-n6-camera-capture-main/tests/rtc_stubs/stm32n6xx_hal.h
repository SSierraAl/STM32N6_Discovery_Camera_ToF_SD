#ifndef RTC_TEST_HAL_H
#define RTC_TEST_HAL_H
#include <stdint.h>
typedef struct { int unused; } I2C_HandleTypeDef;
#define HAL_OK 0
#define HAL_I2C_ERROR_AF 4U
#define I2C_MEMADD_SIZE_8BIT 1
uint32_t HAL_GetTick(void);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *);
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *,unsigned,unsigned,unsigned,uint8_t *,unsigned,unsigned);
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *,unsigned,unsigned,unsigned,uint8_t *,unsigned,unsigned);
#endif
