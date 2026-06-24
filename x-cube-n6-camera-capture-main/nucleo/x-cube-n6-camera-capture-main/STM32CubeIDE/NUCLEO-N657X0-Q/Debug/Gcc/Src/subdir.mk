################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Gcc/Src/console.c \
D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Gcc/Src/freertos_libc.c \
D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Gcc/Src/syscalls.c 

OBJS += \
./Gcc/Src/console.o \
./Gcc/Src/freertos_libc.o \
./Gcc/Src/syscalls.o 

C_DEPS += \
./Gcc/Src/console.d \
./Gcc/Src/freertos_libc.d \
./Gcc/Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Gcc/Src/console.o: D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Gcc/Src/console.c Gcc/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DFX_STANDALONE_ENABLE -DSTM32N6570_DK_REV -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DCAMERA_SELFY=1 -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../../Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/core/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/ports/generic/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_device_classes/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../../../Lib/uvcl/Inc/usbx -I../../../Lib/uvcl/Src/usbx -I../../../Lib/uvcl/Inc -I../../../Lib/uvcl/Src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Gcc/Src/freertos_libc.o: D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Gcc/Src/freertos_libc.c Gcc/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DFX_STANDALONE_ENABLE -DSTM32N6570_DK_REV -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DCAMERA_SELFY=1 -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../../Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/core/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/ports/generic/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_device_classes/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../../../Lib/uvcl/Inc/usbx -I../../../Lib/uvcl/Src/usbx -I../../../Lib/uvcl/Inc -I../../../Lib/uvcl/Src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Gcc/Src/syscalls.o: D:/STM_CODES/Discovery/x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/Gcc/Src/syscalls.c Gcc/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m55 -std=gnu11 -g3 -DSTM32N657xx -DFX_STANDALONE_ENABLE -DSTM32N6570_DK_REV -DUSE_FULL_ASSERT -DUSE_FULL_LL_DRIVER -DVECT_TAB_SRAM -DUSE_IMX335_SENSOR -DCAMERA_SELFY=1 -DTX_MAX_PARALLEL_NETWORKS=1 -DFEAT_FREERTOS -DUVC_LIB_USE_USBX -DUX_INCLUDE_USER_DEFINE_FILE -DUSBL_PACKET_PER_MICRO_FRAME=3 -DUX_STANDALONE -DUVCL_USBX_USE_FREERTOS -DUVC_LIB_USE_DMA -c -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Include -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc -I../../../Inc -I../../../STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Inc/Legacy -I../../../STM32Cube_FW_N6/Drivers/CMSIS/Include -I../../../STM32Cube_FW_N6/Drivers/CMSIS/DSP/Include -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/Common -I../../../STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK -I../../../STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx -I../../../Lib/Camera_Middleware -I../../../Lib/Camera_Middleware/sensors -I../../../Lib/Camera_Middleware/sensors/imx335 -I../../../Lib/Camera_Middleware/ISP_Library/isp/Inc -I../../../Lib/FreeRTOS/Source/include -I../../../Lib/FreeRTOS/Source/portable/GCC/ARM_CM55_NTZ/non_secure -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/core/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/ports/generic/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_device_classes/inc -I../../../STM32Cube_FW_N6/Middlewares/ST/usbx/common/usbx_stm32_device_controllers -I../../../Lib/uvcl/Inc/usbx -I../../../Lib/uvcl/Src/usbx -I../../../Lib/uvcl/Inc -I../../../Lib/uvcl/Src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -mcmse -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Gcc-2f-Src

clean-Gcc-2f-Src:
	-$(RM) ./Gcc/Src/console.cyclo ./Gcc/Src/console.d ./Gcc/Src/console.o ./Gcc/Src/console.su ./Gcc/Src/freertos_libc.cyclo ./Gcc/Src/freertos_libc.d ./Gcc/Src/freertos_libc.o ./Gcc/Src/freertos_libc.su ./Gcc/Src/syscalls.cyclo ./Gcc/Src/syscalls.d ./Gcc/Src/syscalls.o ./Gcc/Src/syscalls.su

.PHONY: clean-Gcc-2f-Src

