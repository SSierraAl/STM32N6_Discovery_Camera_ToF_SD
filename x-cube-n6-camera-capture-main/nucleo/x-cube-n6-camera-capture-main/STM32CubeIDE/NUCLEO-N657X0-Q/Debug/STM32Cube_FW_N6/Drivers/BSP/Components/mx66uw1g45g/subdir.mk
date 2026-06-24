################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.c 

OBJS += \
./STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.o 

C_DEPS += \
./STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.d 


# Each subdirectory must supply rules for building sources it contributes
STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.o: D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.c STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DFX_STANDALONE_ENABLE -DSTM32N6570_DK_REV -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DCAMERA_SELFY=1 -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/src -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/inc -I../../../STM32Cube_FW_N6/Drivers/VL53L5CX_ULD_API/inc -I../../../STM32Cube_FW_N6/Drivers/VL53L5CX_ULD_API/src -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/core/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/ports/generic/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_device_classes/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../../../Lib/uvcl/Inc/usbx -I../../../Lib/uvcl/Src/usbx -I../../../Lib/uvcl/Inc -I../../../Lib/uvcl/Src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32Cube_FW_N6-2f-Drivers-2f-BSP-2f-Components-2f-mx66uw1g45g

clean-STM32Cube_FW_N6-2f-Drivers-2f-BSP-2f-Components-2f-mx66uw1g45g:
	-$(RM) ./STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.cyclo ./STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.d ./STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.o ./STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/mx66uw1g45g.su

.PHONY: clean-STM32Cube_FW_N6-2f-Drivers-2f-BSP-2f-Components-2f-mx66uw1g45g

