# Protocols, Abbreviations, and Terminology Glossary
## Everything You Need to Know to Understand the Code and Documentation

---

## Table of Contents

- [Protocols, Abbreviations, and Terminology Glossary](#protocols-abbreviations-and-terminology-glossary)
  - [Everything You Need to Know to Understand the Code and Documentation](#everything-you-need-to-know-to-understand-the-code-and-documentation)
  - [Table of Contents](#table-of-contents)
  - [1. Hardware Protocols](#1-hardware-protocols)
  - [2. STM32 Peripherals](#2-stm32-peripherals)
  - [3. Software Components](#3-software-components)
  - [4. FreeRTOS Terms](#4-freertos-terms)
  - [5. Image Format Terms](#5-image-format-terms)
  - [6. SD Card Terms](#6-sd-card-terms)
  - [7. Console Log Prefixes](#7-console-log-prefixes)
  - [8. Common Abbreviations in Code](#8-common-abbreviations-in-code)
  - [9. Quick Reference Card](#9-quick-reference-card)
    - [Time Units](#time-units)
    - [Memory Sizes](#memory-sizes)
    - [Key Registers](#key-registers)
    - [Capture Pipeline Steps (Simplified, Mode 4)](#capture-pipeline-steps-simplified-mode-4)
    - [Data Flow (Simplified)](#data-flow-simplified)

---

## 1. Hardware Protocols

| Term | Full Name | What It Is | Where Used |
|------|-----------|-----------|------------|
| **I2C** | Inter-Integrated Circuit | Two-wire serial bus (SCL + SDA) for short-distance chip-to-chip communication | Camera ↔ MCU, ToF ↔ MCU |
| **CSI-2** | Camera Serial Interface v2 | High-speed MIPI bus for camera sensor → MCU pixel data transfer | IMX335 → DCMIPP |
| **SDMMC** | SD/MMC Controller | Host controller for SD card communication (4-bit or 1-bit mode) | MCU ↔ SD card |
| **SPI** | Serial Peripheral Interface | Four-wire bus (SCK, MOSI, MISO, CS) for flash/memory | MCU ↔ PSRAM (Octal SPI) |
| **XSPI** | Extended SPI (Octal) | STM32's name for Quad/Octal SPI with memory-mapped mode | PSRAM access |
| **PWM** | Pulse Width Modulation | Generates variable-width pulses for LED brightness control | WS2812 timing |
| **DMA** | Direct Memory Access | Hardware engine that moves data between memory and peripherals WITHOUT CPU involvement | DCMIPP → PSRAM, SD writes, WS2812 data |
| **UART/USART** | Universal Async Receiver/Transmitter | Serial communication (TX + RX wires) for debug console | USB→Serial chip → PC |

---

## 2. STM32 Peripherals

| Acronym | Full Name | Purpose |
|---------|-----------|---------|
| **DCMIPP** | Dual Camera Memory-to-Pixel Preprocessor | Receives CSI-2 data, scales/crops/converts format, DMAs result to PSRAM |
| **NPU** | Neural Processing Unit | AI accelerator (not used in this project) |
| **GPDMA** | General Purpose DMA | Flexible DMA controller for WS2812 and other peripherals |
| **TIM1** | Advanced Timer 1 | Generates PWM signal for WS2812 LED data |
| **RCC** | Reset and Clock Control | Manages all clock sources, PLLs, peripheral clocks |
| **MPU** | Memory Protection Unit | Defines memory regions (cacheable, non-cacheable, access permissions) |
| **ICACHE/DCACHE** | Instruction/Data Cache | CPU cache for faster memory access (must be managed for DMA coherency) |
| **NVIC** | Nested Vectored Interrupt Controller | Hardware interrupt prioritization and dispatch |
| **SMPS** | Switching Mode Power Supply | Efficient voltage regulator (enabled for 400 MHz operation) |
| **RIF** | Resource Identification Firewall | Security module (configures which peripherals are secure/non-secure) |
| **IAC** | Illegal Access Controller | Detects and traps unauthorized memory/peripheral access |
| **PSRAM** | Pseudo-Static RAM | External 16MB DRAM accessed via XSPI (appears like SRAM, used for large buffers) |

---

## 3. Software Components

| Term | What It Is |
|------|-----------|
| **HAL** | Hardware Abstraction Layer. ST's library that provides functions like `HAL_SD_WriteBlocks()`, `HAL_I2C_Init()`, etc. |
| **BSP** | Board Support Package. ST's library for board-specific functions (LEDs, buttons, sensors) |
| **CMW** | Camera Middleware. ST's camera library (`CMW_CAMERA_Init()`, `CMW_CAMERA_Start()`, etc.) |
| **ISP** | Image Signal Processor. Software library that processes raw sensor data (auto-exposure, auto-white-balance) |
| **ULD** | Ultra-Low Drift. ST's VL53L5CX driver library (`VL53L5CX_Api_xxx` functions) |
| **FreeRTOS** | Real-Time Operating System. Manages tasks (threads), queues, semaphores, and scheduling |
| **CubeIDE** | STM32CubeIDE. ST's Eclipse-based IDE for STM32 development |
| **CubeMX** | STM32CubeMX. ST's tool for generating initialization code (pin config, clocks, etc.) |

---

## 4. FreeRTOS Terms

| Term | What It Is | Example in This Project |
|------|-----------|------------------------|
| **Task** | A thread of execution (like a function that runs forever in a loop) | `sensor_task`, `camera_task`, `storage_task` |
| **Queue** | A FIFO buffer for passing data between tasks | `camera_cmd_queue` (sensor → camera) |
| **Semaphore** | A signaling mechanism. One task "gives" it, another "takes" it | `camera_ready_sem` (camera → sensor: "capture done") |
| **Mutex** | A special semaphore for mutual exclusion. Only one task can "hold" it at a time | `i2c1_mutex` (prevents camera + ToF from using I2C simultaneously) |
| **Priority** | Higher number = runs first when multiple tasks are ready | sensor_task (IDLE+4) > camera_task (IDLE+3) > storage_task (IDLE+2) |
| **Stack** | Memory allocated for a task's local variables | Each task has a stack array in PSRAM |
| **vTaskDelay()** | Puts the current task to sleep for N milliseconds | `vTaskDelay(pdMS_TO_TICKS(15))` = sleep 15ms |
| **taskYIELD()** | Gives up the rest of the current time slice to another ready task | Used sparingly for scheduler handoff |
| **portMAX_DELAY** | Infinite timeout (wait forever) | `xQueueReceive(..., portMAX_DELAY)` = block until data available |

---

## 5. Image Format Terms

| Term | What It Is |
|------|-----------|
| **YUV422** | Color format: 1 brightness (Y) + 2/4 color (UV) = 2 bytes per pixel. Used by IMX335 output. |
| **Binning** | Combining adjacent sensor pixels into one. 2×2 binning = ¼ resolution, but faster readout. |
| **Full Resolution** | 2592×1944 = 5,038,920 pixels × 2 bytes = 9,677,376 bytes per frame |
| **Binned Resolution** | 1296×972 = 1,259,904 pixels × 2 bytes = 2,519,424 bytes per frame |
| **Rolling Shutter** | Sensor reads row-by-row (not all at once). Fast-moving objects appear skewed. |
| **Warmup Frames** | First N frames after power-on are unstable (wrong color/exposure) and must be discarded. |
| **Double Buffering** | Two memory buffers: while DCMIPP writes to buffer A, CPU reads from buffer B (or vice versa). |
| **Zero-Copy** | Data goes from hardware DMA directly to its final destination. No CPU memcpy needed. |

---

## 6. SD Card Terms

| Term | What It Is |
|------|-----------|
| **Block** | SD card's smallest writable unit = 512 bytes |
| **Batch** | Our group of blocks written in one HAL call = 64 blocks × 512 = 32 KB |
| **CMD13** | SD protocol command: "Send Status" — asks the card if it's ready for the next operation |
| **PROGRAMMING state** | SD card is busy internally (erasing/programming NAND flash). Cannot accept new commands. |
| **TRANSFER state** | SD card is ready to accept read/write commands. |
| **STA register** | SDMMC2 hardware status register. `0x5000` = Data Command Response Timeout (card rejected our write). |
| **HAL status** | Software status returned by HAL functions. `HAL_OK` = 0, `HAL_ERROR` = 1, etc. |
| **DMA** | Direct Memory Access. SD controller moves data between `sd_batch_buf` and SD card without CPU. |
| **SCB_CleanDCache** | CPU instruction: "flush modified cache lines to actual memory" — required before DMA reads from a buffer. |
| **SCB_InvalidateDCache** | CPU instruction: "discard stale cache lines, reload from memory" — required after DMA writes to a buffer. |

---

## 7. Console Log Prefixes

| Prefix | Source | What It Means |
|--------|--------|---------------|
| `[INIT]` | `main.c` | Board initialization step |
| `[SD]` | `main.c` or `app_thread.c` | SD card operation (init, write, error, recovery) |
| `[CAM]` | `app_cam.c` | Camera operation (init, capture, warmup, error) |
| `[SENSOR]` | `app_thread.c` | ToF sensor operation |
| `[BTN]` | `main.c` | Button press detected |
| `[IPC]` | `app_thread.c` | Inter-Process Communication (queues, semaphores) |
| `[I2C]` | `i2c_arbiter.c` or BSP | I2C bus operation |
| `[W]` / `[R]` | `main.c` SD benchmark | Write / Read speed test |
| `>>> INSECT DETECTED!` | `app_thread.c` | ToF detected insect, starting capture |

---

## 8. Common Abbreviations in Code

| Abbreviation | Full Form | Context |
|-------------|-----------|---------|
| `cfg` | Configuration | Function parameter name |
| `rc` | Return Code | Function result (0 = OK, -1 = error) |
| `st` | Status | HAL_StatusTypeDef result |
| `idx` | Index | Array or loop index |
| `blk` | Block | SD card block |
| `buf` | Buffer | Memory array |
| `hdr` | Header | SD image header structure |
| `snap` | Snapshot | Captured image |
| `frm` / `frame` | Frame | Single camera image |
| `ms` | Milliseconds | Time unit (1/1000 second) |
| `μs` | Microseconds | Time unit (1/1,000,000 second) |
| `t0` | Time Zero | Start timestamp for elapsed measurement |
| `elapsed` | Elapsed time | `HAL_GetTick() - t0` |
| `wait_ms` | Wait time in ms | Time spent polling for card ready |
| `write_ms` | Write time in ms | Time spent in DMA transfer |
| `gap_ms` | Gap time in ms | Time spent in vTaskDelay between batches |
| `WARMUP` | Warmup phase | Discarding unstable frames |
| `CAPTURE` | Capture phase | Saving actual frames |
| `STANDBY` | Standby mode | Camera/sensor powered but not streaming |
| `STREAMING` | Streaming mode | Camera sensor actively sending pixel data |
| `PERF` | Performance | Debug timing/analysis code |
| `DBG` | Debug | Diagnostic code |
| `FATAL` | Fatal error | Cannot recover, system halts |
| `WARN` | Warning | Non-critical issue |
| `OK` | Success | Operation completed without error |
| `FAIL` | Failure | Operation did not complete |
| `N/A` | Not Applicable | Field doesn't apply to this mode/parameter |

---

## 9. Quick Reference Card

### Time Units
```
1 second  = 1000 milliseconds (ms)
1 ms      = 1000 microseconds (μs)
Camera frame @ 30 FPS = 33.3 ms
Sensor warmup (11 frames) = 363 ms
SD write (1296×972 image) = ~2500 ms
SD write (2592×1944 image) = ~9500 ms
```

### Memory Sizes
```
1 Byte    = 8 bits (one pixel = 2 Bytes in YUV422)
1 KB      = 1024 Bytes
1 MB      = 1024 KB = 1,048,576 Bytes
SD block  = 512 Bytes
Batch     = 64 blocks = 32 KB
1296×972 image = 2,519,424 Bytes = 2.4 MB = 4921 blocks
2592×1944 image = 9,677,376 Bytes = 9.2 MB = 18908 blocks
PSRAM total = 16 MB
```

### Key Registers
```
IMX335 register 0x3000 = Mode Select
  0x00 = Streaming (camera active, sending pixels)
  0x01 = Standby (camera powered but idle, low power)

SDMMC2 STA register (status)
  0x0000 = All clear
  0x5000 = Data Command Response Timeout (card rejected write)
  Check with: printf("STA=0x%08lX", (unsigned long)SDMMC2->STA)
```

### Capture Pipeline Steps (Simplified, Mode 4)
```
1. ToF sees insect
2. Camera wakes from standby (I2C: write 0x00 to register 0x3000)
3. Wait 35ms for sensor to stabilize
4. Start DCMIPP pipe (hardware begins receiving pixels via CSI-2)
5. Set exposure/gain (I2C: write to camera registers)
6. Wait 11 warmup frames (discard, they're unstable)
7. Capture 4 frames (DMA writes directly to batch_buf, zero-copy)
8. Stop DCMIPP pipe
9. Camera back to standby (I2C: write 0x01 to register 0x3000)
10. Write 4 images to SD card (~2500ms each)
11. ToF resumes monitoring
```

### Data Flow (Simplified)
```
Insect → ToF sensor (I2C) → Detection algorithm
                                        ↓
                              sensor_task sends command
                                        ↓
                              camera_task receives command
                                        ↓
                              Camera wakes → captures frames → DCMIPP → DMA → PSRAM
                                        ↓
                              camera_task sends image pointers
                                        ↓
                              storage_task receives pointers
                                        ↓
                              CPU copies image to sd_batch_buf → DMA → SD card
```

---

*Glossary — Updated July 2026*