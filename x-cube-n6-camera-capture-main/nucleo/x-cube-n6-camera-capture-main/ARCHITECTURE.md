# STM32N6 Camera + ToF Project Architecture

## Entry Point
- **File:** `Src/main.c`
- **Function:** `main()` (line 241)
- **Flow:** `main()` → `main_freertos()` → `main_thread_fct()` and `btn_thread_fct()`

## Application Source Files (Src/)

### Core Application
- `Src/main.c` - Entry point, peripheral initialization, FreeRTOS tasks
- `Src/app.c` - Application framework, capture pipeline, list processing
- `Src/app_thread.c` - Thread management and synchronization
- `Src/app_cam.c` - Camera control (IMX335 via CMW)
- `Src/app_cvt.c` - Color space conversion
- `Src/app_dma2d.c` - DMA2D hardware accelerator
- `Src/app_sdcard.c` - SD card operations
- `Src/app_jpg.c` - JPEG encoding
- `Src/app_filex.c` - FileX filesystem integration
- `Src/app_fuseprogramming.c` - Fuse programming utilities

### Platform & Peripherals
- `Src/platform.c` - VL53L5CX I2C platform abstraction
- `Src/perf_debug.c` - Performance debugging utilities
- `Src/ws2812.c` - LED illumination control (WS2812 via GPDMA+TIM PWM)
- `Src/vl53l5cx_detection.c` - ToF sensor detection logic
- `Src/freertos_bsp.c` - FreeRTOS BSP integration
- `Src/fx_stm32_sd_driver_glue.c` - FileX SD driver glue layer

### HAL Overrides
- `Src/stm32n6xx_hal_sd.c` - SDMMC HAL driver
- `Src/stm32n6xx_hal_sd_ex.c` - SDMMC HAL extended
- `Src/stm32n6xx_ll_sdmmc.c` - Low-level SDMMC
- `Src/stm32n6xx_it.c` - Interrupt handlers

## Header Files (Inc/)

### Application Headers
- `Inc/main.h` - Main declarations
- `Inc/app.h` - Application framework
- `Inc/app_config.h` - Configuration parameters
- `Inc/app_cam.h` - Camera interface
- `Inc/app_thread.h` - Thread definitions
- `Inc/debug_color.h` - Debug color utilities

## Libraries (Lib/)

### Camera Middleware
- `Lib/Camera_Middleware/cmw_camera.c` - Camera middleware core
- `Lib/Camera_Middleware/sensors/imx335/` - IMX335 sensor driver
- `Lib/Camera_Middleware/ISP_Library/` - Image signal processor

### FreeRTOS
- `Lib/FreeRTOS/Source/` - RTOS kernel (tasks, queues, semaphores)

### UVC Library
- `Lib/uvcl/` - USB Video Class library

## Call Flow

```
main() [Src/main.c:241]
  └─ main_freertos() [Src/main.c:847]
       ├─ Setup_Mpu()
       ├─ Security_Config()
       ├─ SystemClock_Config()
       ├─ CONSOLE_Config()
       ├─ MX_TIM1_Init()
       ├─ MX_GPDMA1_Init()
       ├─ VL53L5CX_I2C_Init()
       └─ osKernelCreate() / vTaskStartScheduler()
            ├─ main_thread_fct() [Src/main.c:921]
            │    ├─ SD card initialization
            │    ├─ Camera initialization (app_cam)
            │    ├─ ISP configuration
            │    └─ STOP camera pipe (sleep mode)
            └─ btn_thread_fct() [Src/main.c:741]
                 ├─ Poll USER button (PC13)
                 ├─ START camera pipe
                 ├─ Capture 3 frames
                 ├─ SD_StoreRawImage()
                 └─ STOP camera pipe
```

## Hardware Components
- **MCU:** STM32N657 (NUCLEO-N657X0-Q / MB1940)
- **Camera:** CMW-IMX335 (5MP YUV422 via CSI-2)
- **ToF:** VL53L5CX (8x8 resolution, I2C)
- **Storage:** microSD (SDMMC2, 4-bit bus)
- **Button:** USER B1 = PC13
- **LEDs:** RED=PG10, GREEN=PG0
- **Illumination:** WS2812 (GPDMA1 + TIM1 PWM)

## Peripherals Used
- I2C1 (VL53L5CX)
- UART1 (Debug console)
- SDMMC2 (microSD)
- TIM1 (WS2812 PWM)
- GPDMA1 (WS2812 data)
- DCMIPP (Camera interface)
- DMA2D (Graphics accelerator)
- XSPI (PSRAM)