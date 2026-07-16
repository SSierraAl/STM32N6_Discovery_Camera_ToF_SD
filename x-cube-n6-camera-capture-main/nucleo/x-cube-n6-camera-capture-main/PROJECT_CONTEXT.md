# STM32N6 Camera + ToF Project - MCP Context

## Quick Reference for AI-Assisted Development

### Entry Point & Thread Architecture
```
main() [Src/main.c:241]
  └─ main_freertos() [Src/main.c:847] - FreeRTOS initialization
       ├─ Setup_Mpu() - Memory protection unit
       ├─ Security_Config() - Security peripheral
       ├─ SystemClock_Config() [Src/main.c:385] - HSE, PLL1-4, per clocks
       ├─ CONSOLE_Config() - UART1 debug console
       ├─ MX_TIM1_Init() - WS2812 PWM timer
       ├─ MX_GPDMA1_Init() - WS2812 data DMA
       ├─ VL53L5CX_I2C_Init() - ToF sensor I2C1
       └─ osKernelCreate() / vTaskStartScheduler()
            ├─ main_thread_fct() [Src/main.c:921] - INIT ONLY, then self-deletes
            │    ├─ SD card init (HAL_SD_Init)
            │    ├─ Camera init (CMW_CAMERA_Init)
            │    ├─ ISP config (ISP_Init, ISP_Start)
            │    ├─ ToF init (vl53l5cx_detection_init)
            │    ├─ WS2812 init (WS2812_Init)
            │    └─ STOP camera pipe → sleep mode
            └─ btn_thread_fct() [Src/main.c:741] - CAPTURE LOOP
                 ├─ Poll USER button (PC13)
                 ├─ START camera pipe
                 ├─ Capture 3 frames → SD_StoreRawImage()
                 └─ STOP camera pipe → sleep mode
```

### Key Source Files (Src/)
| File | Purpose | Key Functions |
|------|---------|---------------|
| `main.c` | Entry, init, threads | main(), main_thread_fct(), btn_thread_fct(), SystemClock_Config() |
| `app.c` | App framework, capture pipeline | app_run(), capture_init(), sr_init()/sr_lock()/sr_unlock() |
| `app_cam.c` | Camera control | app_cam_init(), app_cam_start(), app_cam_stop() |
| `app_thread.c` | Thread management | thread_create(), thread_sync() |
| `app_sdcard.c` | SD card ops | sd_read(), sd_write() |
| `app_jpg.c` | JPEG encoding | jpg_encode() |
| `ws2812.c` | LED illumination | WS2812_Init(), WS2812_Update(), WS2812_Flash() |
| `platform.c` | ToF I2C platform | VL53L5CX_RdByte(), VL53L5CX_WrByte() |
| `vl53l5cx_detection.c` | ToF detection | vl53l5cx_init(), vl53l5cx_poll() |
| `freertos_bsp.c` | FreeRTOS BSP | HAL_GetTick() |
| `stm32n6xx_it.c` | Interrupt handlers | HardFault, SysTick, etc. |
| `stm32n6xx_hal_sd.c` | SDMMC HAL overrides | HAL_SD_Init(), HAL_SD_ReadBlocks() |
| `perf_debug.c` | Performance debug | perf_start(), perf_stop() |

### Key Headers (Inc/)
| Header | Purpose |
|--------|---------|
| `main.h` | Main declarations |
| `app_config.h` | ALL configurable parameters (frame sizes, buffer counts, etc.) |
| `app_cam.h` | Camera interface |
| `app_thread.h` | Thread definitions |
| `ws2812.h` | LED control |
| `vl53l5cx_detection.h` | ToF detection API |
| `platform.h` | I2C platform |
| `debug_color.h` | Debug LED colors |
| `FreeRTOSConfig.h` | FreeRTOS configuration |
| `ulist.h` | Linked list utility |
| `utils.h` | General utilities |

### Libraries (Lib/)
| Library | What's Used | Purpose |
|---------|-------------|---------|
| `Camera_Middleware/sensors/imx335/` | cmw_imx335.c, imx335.c | IMX335 sensor driver |
| `Camera_Middleware/ISP_Library/` | isp_core.c, isp_services.c | Image signal processor |
| `FreeRTOS/Source/` | tasks.c, queue.c, semphr.c, list.c | RTOS kernel |
| `uvcl/` | uvcl.c, uvcl_desc.c | USB Video Class (if enabled) |

### Clocks (SystemClock_Config in main.c)
- **HSE:** External oscillator
- **PLL1:** CPU core clock
- **PLL2:** Peripheral clock (I2C, UART, etc.)
- **PLL3:** DSI/LTDC (if display enabled)
- **PLL4:** XSPI/PSRAM clock
- Key: modify PLL dividers for performance tuning

### Peripherals Used
| Peripheral | Purpose | Handler |
|------------|---------|---------|
| I2C1 | VL53L5CX ToF | hi2c1, platform.c |
| UART1 | Debug console | huart1 |
| SDMMC2 | microSD card | hsd1, stm32n6xx_hal_sd.c |
| TIM1 | WS2812 PWM | htim1 |
| GPDMA1 | WS2812 data | handle_GPDMA1_Channel1 |
| DCMIPP | Camera interface | app_cam.c |
| DMA2D | Graphics accel | app_dma2d.c |
| XSPI | PSRAM | stm32n6570_discovery_xspi.h |

### GPIO Mapping
| Pin | Function |
|-----|----------|
| PC13 | USER button (active HIGH, pull-down) |
| PG10 | RED LED |
| PG0 | GREEN LED |
| TIM1_CH | WS2812 PWM |

### SD Card Storage
- Raw YUV422 frames stored directly on SD
- Format: YUV422, resolution from app_config.h
- File naming: frame_XXXX.raw

### Camera (IMX335)
- 5MP sensor, YUV422 output via CSI-2
- Configured in app_config.h (binning, ROI, frame rate)
- ISP pipeline: raw → debayer → YUV422

### Common Modification Points
1. **Change frame size/resolution:** Edit `Inc/app_config.h`
2. **Modify capture logic:** Edit `Src/main.c` btn_thread_fct()
3. **Change LED patterns:** Edit `Src/ws2812.c` WS2812_Flash()
4. **Adjust clocks:** Edit `Src/main.c` SystemClock_Config()
5. **Add ToF detection zones:** Edit `Src/vl53l5cx_detection.c`
6. **Change SD write format:** Edit `Src/main.c` SD_StoreRawImage()
7. **Modify camera init:** Edit `Src/app_cam.c`

### Performance Debugging
- `Src/perf_debug.c` - timing measurements
- `Inc/debug_color.h` - LED indicators for states
- RED LED (PG10) = error
- GREEN LED (PG0) = ready