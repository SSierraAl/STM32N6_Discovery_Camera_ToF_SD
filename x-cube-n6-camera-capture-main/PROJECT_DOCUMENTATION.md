# STM32N6 Discovery Camera + ToF Insect Capture System
## Complete Project Documentation

---

## Table of Contents

1. [Overview](#1-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [Software Architecture](#3-software-architecture)
4. [File Structure](#4-file-structure)
5. [Boot Sequence](#5-boot-sequence)
6. [FreeRTOS Task Architecture](#6-freertos-task-architecture)
7. [Capture Pipeline](#7-capture-pipeline)
8. [SD Card Storage](#8-sd-card-storage)
9. [I2C Bus Arbitration](#9-i2c-bus-arbitration)
10. [Performance Debugging](#10-performance-debugging)
11. [Configuration Parameters](#11-configuration-parameters)
12. [Image Format](#12-image-format)
13. [Python SD Card Reader](#13-python-sd-card-reader)
14. [Troubleshooting](#14-troubleshooting)
15. [Quick Reference](#15-quick-reference)

---

## 1. Overview

### Purpose
This project implements an **autonomous insect detection and capture system** using the STM32N6570 Discovery board. It combines:
- **CMW-IMX335 camera** (5MP CSI-2 sensor) for high-speed photography
- **VL53L5CX ToF sensor** (8x8 or 4x4 resolution) for motion detection
- **WS2812 LED illumination** for synchronized lighting
- **microSD card storage** for raw YUV422 image capture

### Key Capabilities
- **Fully autonomous**: No PC connection needed after flashing
- **Low latency**: Sub-5ms insect detection → camera capture
- **Multiple capture modes**: On-demand, continuous, batch, callback-batch
- **Robust error recovery**: SD card auto-recovery, I2C conflict resolution, camera retry
- **Performance monitoring**: Built-in timing reports and bottleneck analysis

### Performance Summary
| Metric | Value |
|--------|-------|
| Camera sensor | IMX335 (5MP, 2592×1944 max) |
| Capture resolution | 1296×972 (binned) or 2592×1944 (full) |
| Frame format | YUV422 (2 bytes/pixel) |
| Camera frame rate | 30 FPS |
| ToF polling rate | ~200 Hz (5ms interval) |
| Detection → Capture start | <5ms |
| Single image capture | ~140ms |
| SD write (1296×972) | ~700-900ms |
| SD write (2592×1944) | ~2000ms |

---

## 2. Hardware Architecture

### Processor: STM32N6570
- **ARM Cortex-M55** with Helio Neural Network Accelerator
- **Clock**: 400 MHz (IC1), 200 MHz (AHB), 125 MHz (APB)
- **Memory**: 1 MB Flash, 320 KB SRAM, 16 MB PSRAM (XSPI)

### Peripherals Used
| Peripheral | Purpose | Pins |
|-----------|---------|------|
| **DCMIPP** | Camera image capture (hardware scaling, YUV conversion) | CSI-2 dedicated |
| **CSI-2** | Camera interface (receives pixel data) | Dedicated |
| **SDMMC2** | SD card controller (4-bit mode, 16.38 MB/s) | SD card socket |
| **I2C1** | ToF sensor communication (100 kHz) | PC1 (SCL), PH9 (SDA) |
| **TIM1** | WS2812 PWM generation (Channel 1) | PE9 |
| **GPDMA1** | WS2812 data transmission | Channel 1 |
| **USART1** | Debug console (115200 baud) | PE5 (TX), PE6 (RX) |
| **XSPI1/2** | PSRAM interface (Octal SPI, Dual Transfer Rate) | PSRAM chip |
| **GPIO** | LEDs, button | PG10 (RED), PG0 (GREEN), PC13 (BUTTON) |

### Power Architecture
- **VDD**: 3.3V main supply
- **VDDIO5**: SD card I/O voltage (enabled before SD init)
- **SMPS**: Switching mode power supply (overdrive enabled for 400 MHz)

---

## 3. Software Architecture

### Layer Diagram
```
┌─────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                         │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌──────────┐ │
│  │sensor_task│  │camera_task│  │storage_task│  │btn_thread│ │
│  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘  └────┬─────┘ │
│        │               │               │              │      │
├────────┼───────────────┼───────────────┼──────────────┼──────┤
│        │               │               │              │      │
│  ┌─────▼───────────────▼───────────────▼──────────────▼──────┐ │
│  │              IPC (FreeRTOS Queues + Semaphores)           │ │
│  │  camera_cmd_queue | storage_cmd_queue | sensor_event_queue│ │
│  │  camera_ready_sem | storage_done_sem                         │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                 │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐                   │
│  │ app_cam.c │  │app_thread.│  │  main.c   │                   │
│  │ (camera)  │  │  (tasks)  │  │ (init+SD) │                   │
│  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘                   │
│        │              │              │                          │
├────────┼──────────────┼──────────────┼──────────────────────────┤
│        │              │              │                          │
│  ┌─────▼──────────────▼──────────────▼─────────────────────────┐ │
│  │              MIDDLEWARE LAYER                                │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌────────────────────┐  │ │
│  │  │ CMW Camera  │  │ VL53L5CX    │  │   SD HAL (hsd1)    │  │ │
│  │  │ (ST lib)    │  │   (ST SDK)  │  │                    │  │ │
│  │  └─────────────┘  └─────────────┘  └────────────────────┘  │ │
│  └─────────────────────────────────────────────────────────────┘ │
│                                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐│
│  │I2C Arbiter│  │ WS2812   │  │ PerfDebug│  │ DCMIPP HAL       ││
│  │ (mutex)  │  │ (PWM+DMA)│  │ (timing) │  │ (camera pipe)    ││
│  └──────────┘  └──────────┘  └──────────┘  └──────────────────┘│
└──────────────────────────────────────────────────────────────────┘
```

---

## 4. File Structure

```
x-cube-n6-camera-capture-main/
│
├── nucleo/x-cube-n6-camera-capture-main/
│   │
│   ├── Src/                              # Source files
│   │   ├── main.c                        # Entry point, board init, btn_thread
│   │   ├── app_thread.c                  # RTOS tasks (sensor, camera, storage)
│   │   ├── app_cam.c                     # Camera abstraction (init, capture modes)
│   │   ├── i2c_arbiter.c                 # I2C1 mutex (camera ↔ ToF serialization)
│   │   ├── platform.c                    # VL53L5CX low-level I2C (wrapped by arbiter)
│   │   ├── vl53l5cx_detection.c          # Insect detection algorithm
│   │   ├── ws2812.c                      # LED driver (PWM + DMA)
│   │   └── perf_debug.c                  # Performance timing/reporting
│   │
│   ├── Inc/                              # Headers
│   │   ├── main.h                        # Global declarations (hi2c1, hsd1, etc.)
│   │   ├── app_config.h                  # ⭐ CENTRAL CONFIGURATION FILE
│   │   ├── app_thread.h                  # Task prototypes, IPC types
│   │   ├── app_cam.h                     # Camera API, capture mode functions
│   │   ├── i2c_arbiter.h                 # I2C mutex declaration
│   │   ├── vl53l5cx_detection.h          # Detection API, config macros
│   │   ├── ws2812.h                      # LED control API
│   │   ├── perf_debug.h                  # Timing markers, report functions
│   │   └── debug_color.h                 # RGB color constants
│   │
│   ├── Lib/                              # ST Middleware
│   │   ├── Camera_Middleware/            # CMW camera library
│   │   │   └── sensors/imx335/           # IMX335 sensor driver
│   │   └── VL53L5CX/                     # VL53L5CX ULD driver (closed source)
│   │
│   ├── Gcc/                              # Build system
│   │   ├── Makefile                      # Root makefile
│   │   └── LinkerScripts/                # Memory layout
│   │
│   └── STM32CubeIDE/                     # IDE project files
│       └── NUCLEO-N657X0-Q/
│           ├── .cproject                 # Eclipse/CubeIDE project
│           └── .project
│
├── read_sd_image.py                      # Python SD card reader + GUI viewer
├── SD_Image_Viewer.py                    # Enhanced image viewer
├── CALLBACK_BATCH_MODE_GUIDE.md          # Mode 4 detailed guide
├── PERFORMANCE_DEBUG_GUIDE.md            # Timing analysis guide
└── BATCH_MODE_TEST_PLAN.md               # Testing procedures
```

---

## 5. Boot Sequence

### Phase 1: Early Init (Before FreeRTOS)
```
main()
  │
  ├── Enable instruction cache (ICACHE)
  ├── Switch to HSI clock temporarily
  ├── HAL_Init() (system tick, NVIC defaults)
  ├── Setup_MPU() (non-cacheable region for DMA)
  ├── SCB_EnableICache() + SCB_EnableDCache()
  └── main_freertos()
        │
        ├── I2C_Arbiter_Init() ← Create I2C1 mutex
        ├── xTaskCreateStatic(main_thread, ...) ← Priority IDLE+1
        ├── xTaskCreateStatic(storage_task, ...) ← Priority IDLE+2
        ├── xTaskCreateStatic(camera_task, ...) ← Priority IDLE+3
        ├── xTaskCreateStatic(sensor_task, ...) ← Priority IDLE+4 (if CAPTURE_MODE != 0)
        ├── xTaskCreateStatic(btn_thread, ...) ← Priority IDLE+3 (if CAPTURE_MODE == 0)
        └── vTaskStartScheduler()
```

### Phase 2: main_thread (Board Initialization)
```
main_thread_fct()
  │
  ├── NVIC priority config (all interrupts same priority)
  ├── SystemClock_Config() ← PLL1=200MHz, PLL2=125MHz, PLL3=225MHz, PLL4=USB
  ├── vPortSetupTimerInterrupt() ← FreeRTOS tick source
  ├── CONSOLE_Config() ← USART1 @ 115200 baud
  ├── Fuse_Programming() ← Board-specific
  ├── VL53L5CX_I2C_Init() ← I2C1 on PC1/PH9 (100kHz)
  ├── WS2812_Init() ← TIM1 PWM + GPDMA1
  ├── BSP_XSPI_RAM_Init(0) ← PSRAM (Octal SPI, DTR)
  ├── BSP_XSPI_NOR_Init(0) ← NOR Flash
  ├── BSP_LED_Init(LED_GREEN, LED_RED)
  ├── BSP_PB_Init(BUTTON_USER1) ← PC13 (active HIGH)
  ├── Security_Config() ← RIF firewall rules
  ├── IAC_Config() ← Illegal access controller
  ├── LL_BUS_EnableClockLowPower(~0) ← Low power clock gates
  │
  ├── SD Card Init (with up to 5 retries)
  │   ├── HAL_PWREx_EnableVDDIO5()
  │   ├── SDMMC2 peripheral reset
  │   ├── HAL_SD_Init() ← 16.38 MB/s, 4-bit bus
  │   └── SD_Benchmarks() ← Write 32KB + Read 32KB, verify integrity
  │
  ├── IPC_Init() ← Create queues + semaphores
  │   ├── camera_cmd_queue (4 slots)
  │   ├── storage_cmd_queue (8 slots)
  │   ├── sensor_event_queue (8 slots)
  │   ├── camera_ready_sem (binary)
  │   └── storage_done_sem (counting, max 8)
  │
  ├── Camera Pre-Init (modes 1, 4 only, BEFORE system_ready=1)
  │   ├── Mode 4: CAM_CallbackInit() ← init + warmup + standby
  │   └── Mode 1: CAM_ContinuousStart() ← init + warmup + leave running
  │
  └── system_ready = 1 ← Signal tasks to start
      vTaskDelete(NULL) ← main_thread deletes itself
```

### Phase 3: Tasks Start Running
Once `system_ready = 1`:
- **sensor_task**: VL53L5CX_Init → VL53L5CX_StartRanging → VL53L5CX_LearnBaseline → wait 5s → monitor
- **camera_task**: ISP servicing loop (modes 1,2,4) or idle queue wait (mode 0)
- **storage_task**: Idle, waiting for storage commands
- **btn_thread**: Polling PC13 (mode 0 only)

---

## 6. FreeRTOS Task Architecture

### Task Priority Table
| Task | Priority | Stack Size | Purpose |
|------|----------|------------|---------|
| main_thread | IDLE+1 | configMINIMAL | Board init, then self-destructs |
| storage_task | IDLE+2 | STORAGE_TASK_STACK_SIZE (2048) | SD card writes (blocking OK) |
| camera_task | IDLE+3 | CAMERA_TASK_STACK_SIZE (1024) | Camera acquisition (fast response) |
| btn_thread | IDLE+3 | 4×configMINIMAL | Button polling (mode 0 only) |
| sensor_task | IDLE+4 | SENSOR_TASK_STACK_SIZE (1024) | ToF monitoring (highest, detection critical) |

### IPC Objects
| Object | Type | Size | Purpose |
|--------|------|------|---------|
| camera_cmd_queue | Queue | 4 × CameraCmd_t | sensor_task → camera_task (capture requests) |
| storage_cmd_queue | Queue | 8 × StorageCmd_t | camera_task → storage_task (image data pointers) |
| sensor_event_queue | Queue | 8 × CameraEvent_typed | sensor_task → camera_task (zone events) |
| camera_ready_sem | Binary Semaphore | - | camera_task → sensor_task (capture done signal) |
| storage_done_sem | Counting Semaphore | max 8 | storage_task → sensor_task (SD write done signal) |
| i2c1_mutex | Mutex | - | I2C1 bus serialization (camera ↔ ToF) |

---

## 7. Capture Pipeline

### Mode 4: Callback-Batch (Default, Recommended)

```
[BOOT]
  main_thread: CAM_CallbackInit()
    → CMW_CAMERA_Init() (I2C)
    → CMW_CAMERA_Start() continuous mode
    → Warmup: 11 frames (discarded)
    → HAL_DCMIPP_CSI_PIPE_Stop()
    → IMX335 → STANDBY (I2C register 0x3000 = 0x01)
    → g_callback_ready = 1

[TOF DETECTION]
  sensor_task: VL53L5CX_IsInsectDetected() == true
    → g_capture_busy = 1, g_sensor_state = PAUSED
    → Capture_RequestSnapshot(60000)
    → camera_cmd_queue: CAM_CMD_SNAP

[CAMERA CAPTURE]
  camera_task: receives CAM_CMD_SNAP
    → CAM_CallbackBatchSnap(batch_buf, frame_size)
      │
      ├── IMX335 → STREAMING (I2C register 0x3000 = 0x00)
      ├── HAL_Delay(35ms) ← Sensor wake time
      ├── CMW_CAMERA_DoubleBufferStart(capture_buf, save_buf)
      │   → DCMIPP alternates between two buffers
      ├── Re-apply exposure/gain (I2C)
      │
      ├── Loop k = 0 to (WARMUP + FRAMES - 1):
      │   ├── CAM_WaitNextFrameReady(120ms)
      │   │   → Polls g_frame_event_count (ISR-driven)
      │   │   → Also calls CAM_IspUpdate() (I2C)
      │   ├── If k >= WARMUP:
      │   │   → SCB_InvalidateDCache(batch_buf[out_idx])
      │   │   → captured++ (zero-copy!)
      │   └── If out_idx+2 exists:
      │       → HAL_DCMIPP_PIPE_SetMemoryAddress(batch_buf[out_idx+2])
      │         (reprogram 2 frames ahead)
      │
      ├── HAL_DCMIPP_CSI_PIPE_Stop()
      ├── IMX335 → STANDBY
      └── Return captured count (typically 4)
      │
    → xSemaphoreGive(camera_ready_sem)
    → For each frame: storage_cmd_queue: STORAGE_CMD_SAVE

[STORAGE]
  storage_task: receives STORAGE_CMD_SAVE × 4
    → SD_StoreRawImage(frame_data)
    → SD_StoreRawImage(frame_data)
    → SD_StoreRawImage(frame_data)
    → SD_StoreRawImage(frame_data)

[SYNC]
  sensor_task: waits for storage_done_sem × 4 (up to 60s total)
    → All 4 images confirmed on SD
    → g_capture_busy = 0, g_sensor_state = RUNNING
    → cooldown = 30 (skip next 30 ToF readings)
```

### Timing Breakdown (Mode 4, 1296×972)
| Phase | Duration | Notes |
|-------|----------|-------|
| ToF detection → camera_cmd enqueue | ~5ms | sensor_task priority 4 |
| camera_task wake + pipe start | ~35ms | IMX335 wake delay |
| Exposure/gain reconfig | ~3ms | I2C write |
| Warmup: 11 frames | ~363ms | 33ms per frame |
| Capture: 4 frames (zero-copy) | ~132ms | 33ms per frame |
| Pipe stop + standby | ~23ms | |
| **Camera total** | **~561ms** | |
| SD write × 4 images | ~2800ms | ~700ms per image |
| **Total capture cycle** | **~3361ms** | ToF paused during entire cycle |
| Cooldown | ~150ms | 30 ToF readings × 5ms |
| **Total** | **~3511ms** | |

---

## 8. SD Card Storage

### Write Strategy: Batched Multi-Block
The SD card is written in batches of **SD_BATCH_WRITE_BLOCKS = 64** blocks (32 KB) per `HAL_SD_WriteBlocks()` call.

```
For a 1296×972 YUV422 image (2,519,424 bytes + 64 byte header):
  Total blocks: ceil((2519424 + 64) / 512) = 4921 blocks
  Batches: ceil(4921 / 64) = 77 batches
  Per batch: 32 KB write → ~400μs actual DMA
```

### Write Loop Structure (per image)
```c
while (img_offset < img_size) {
    // 1. Fill batch buffer from image data
    memcpy(sd_batch_buf, img_buf + img_offset, blocks_in_batch * 512);
    
    // 2. Clean D-Cache (coherence between CPU and DMA)
    SCB_CleanDCache_by_Addr(sd_batch_buf, blocks_in_batch * 512);
    
    // 3. Wait for SD card ready (CMD13 poll)
    SD_WaitForReady();  // Up to 5000ms, 1ms resolution
    
    // 4. Inter-batch recovery gap
    vTaskDelay(pdMS_TO_TICKS(SD_BATCH_RECOVERY_GAP_MS));  // Default 15ms
    
    // 5. Write blocks
    HAL_SD_WriteBlocks(&hsd1, sd_batch_buf, current_block, blocks_in_batch, HAL_MAX_DELAY);
    
    // 6. Check for errors
    if (st != HAL_OK) → return -1 (image lost, caller retries)
    
    // 7. Advance
    img_offset += blocks_in_batch * 512;
    current_block += blocks_in_batch;
}
```

### Error Recovery
```
If SD_StoreRawImage() returns -1:
  │
  ├── storage_task: printf("[SD] FAIL — attempting recovery + retry...")
  ├── SD_Reinit():
  │   ├── HAL_SD_Abort(&hsd1)
  │   ├── HAL_SD_DeInit(&hsd1)
  │   ├── vTaskDelay(100ms)
  │   ├── __HAL_RCC_SDMMC2_FORCE_RESET()
  │   ├── __HAL_RCC_SDMMC2_RELEASE_RESET()
  │   ├── Reconfigure clock
  │   ├── HAL_SD_Init()
  │   └── HAL_SD_ConfigWideBusOperation(4-bit)
  │
  ├── If reinit OK: retry SD_StoreRawImage() with SAME block address
  └── If retry also fails: image LOST, SD_Reinit() again for next capture
```

### SD Card Timing Analysis
| Component | Duration (1296×972) | Percentage |
|-----------|---------------------|------------|
| CMD13 wait (card ready) | ~128ms total | 2.3% |
| DMA transfer | ~787ms total | 14.4% |
| Inter-batch gap (15ms × 76) | ~1140ms total | 20.8% |
| Overhead (memcpy, cache clean) | ~500ms | 9.1% |
| **Total per image** | **~2555ms** | |
| **4 images** | **~10220ms** | |

### Key Insight: The STA=0x5000 Problem
**Symptom**: `HAL_SD_WriteBlocks()` fails with `SDMMC2->STA = 0x5000` (Data Command Response Timeout)

**Root Cause**: The SD card's CMD13 status reports "TRANSFER ready" before the internal NAND flash erase/program cycles complete. Sending the next write command too early causes the card to reject it with a CRC timeout.

**Solution**: Two-part hybrid wait:
1. **Adaptive**: `SD_WaitForReady()` polls CMD13 until card reports TRANSFER (0-5000ms)
2. **Fixed**: `vTaskDelay(SD_BATCH_RECOVERY_GAP_MS)` adds 15ms minimum gap regardless

**Tuning**: Adjust `SD_BATCH_RECOVERY_GAP_MS` in `app_config.h`:
- `15ms`: Default, good balance
- `20ms`: More reliable for slow/failing cards
- `0ms`: Fastest, may cause CRC errors
- `30ms+`: Only if still seeing failures at 20ms

---

## 9. I2C Bus Arbitration

### The Conflict
Both camera and ToF sensor share **I2C1** (PC1=SCL, PH9=SDA):
- **VL53L5CX**: Direct `HAL_I2C_*()` calls in `platform.c` (s_hi2c → hi2c1)
- **Camera**: BSP functions (`BSP_I2C1_WriteReg16`) in `cmw_io.h` → `hbus_i2c1`

**Problem**: Two separate `I2C_HandleTypeDef` structures both manage the same physical I2C1 peripheral. Concurrent access (e.g., camera ISP polling + ToF ranging) corrupts the bus protocol.

**Symptom**: `CMW_CAMERA_Init()` fails with `ret = -7` during camera I2C communication.

### Solution: I2C Arbiter Mutex
```
i2c_arbiter.c
  │
  ├── xSemaphoreCreateMutex() → i2c1_mutex
  │   (created during I2C_Arbiter_Init() in main_freertos(), before tasks start)
  │
  ├── I2C_Take()  → xSemaphoreTake(i2c1_mutex, portMAX_DELAY)
  └── I2C_Give()  → xSemaphoreGive(i2c1_mutex)
```

**Wrapping strategy**:
- **ToF side** (`platform.c`): Every `HAL_I2C_*()` call wrapped with `I2C_Take()` / `I2C_Give()`
- **Camera side** (`cmw_io.h` macros): `CAM_I2C1_WriteReg16_Locked()` / `CAM_I2C1_ReadReg16_Locked()` take mutex before I2C, release after

**Result**: All I2C1 operations are serialized. No concurrent access. No bus corruption.

---

## 10. Performance Debugging

### PerfTimer Structure
```c
typedef struct {
    uint32_t phase_ms[PERF_PHASE_COUNT];     // Timestamp at each phase marker
    uint32_t sd_total_wait_ms;               // Cumulative CMD13 wait time
    uint32_t sd_total_write_ms;              // Cumulative DMA transfer time
    uint32_t sd_total_gap_ms;                // Cumulative inter-batch gap time
    uint32_t sd_batch_count;                 // Number of SD batches written
    uint32_t sd_max_batch_ms;                // Worst single batch time
    uint32_t sd_max_wait_ms;                 // Worst card-ready wait
} PerfTimer_t;
```

### Phase Markers
| Constant | Name | When |
|----------|------|------|
| 0 | START | Capture_RequestSnapshot called |
| 1 | CAM_INIT | Camera init complete |
| 2 | CAM_EXPO | Exposure/gain configured |
| 3 | CAM_WARMUP | Warmup frames done |
| 4 | CAM_SNAP | Final frame captured |
| 5 | CAM_STOP | Pipe stopped |
| 6 | CAM_DEINIT | Camera deinitialized |
| 7 | STORAGE | SD_StoreRawImage called |
| 8 | CACHE_CLEAN | D-Cache cleaned |
| 9 | SD_WRITE | HAL_SD_WriteBlocks called |
| 10 | DONE | All writes complete |

### Perf Debug Levels
```c
#define PERF_DEBUG_LEVEL 0  // Minimal (production): only total time
#define PERF_DEBUG_LEVEL 1  // Standard: phase breakdown
#define PERF_DEBUG_LEVEL 2  // Verbose: sub-phase + per-batch SD timing
#define PERF_DEBUG_LEVEL 3  // Debug: everything + per-block analysis
```

### Performance Report Example
```
╔══════════════════════════════════════════════════════════╗
║ PERFECT CAPTURE #1 — TIMING REPORT                       ║
╠══════════════════════════════════════════════════════════╣
║ PHASE BREAKDOWN                                          ║
╠──────────────────────────────────────────────────────────╣
║ Boot → Camera init       0ms    0.0%                     ║
║ Sensor + DCMIPP        42ms    0.5%                     ║
║ Exposure/Gain cfg       3ms    0.0%                     ║
║ Warmup frames         373ms    4.1%                     ║
║ Final frame grab      141ms    1.5%                     ║
║ Pipe stop              23ms    0.3%                     ║
║ Camera → Storage    6533ms   71.6%  ← BOTTLENECK       ║
╠──────────────────────────────────────────────────────────╣
║ SD CARD DETAIL                                           ║
║ Batches written: 308                                     ║
║ ┌───────────────────────────────────────────────────┐    ║
║ │ Wait (card ready): 128ms   ( 2.3%)               │    ║
║ │ DMA transfer:      787ms   (14.4%)               │    ║
║ │ Gap (inter-batch):4560ms   (83.3%) ← BOTTLENECK  │    ║
║ └───────────────────────────────────────────────────┘    ║
║ Peak single batch: 26ms                                  ║
║ DMA throughput: 3.05 MB/s                                ║
║ Effective throughput: 0.44 MB/s (incl wait+gap)           ║
╠──────────────────────────────────────────────────────────╣
║ BOTTLENECK ANALYSIS                                      ║
║ ⚡ Main bottleneck: SD INTER-BATCH GAP                   ║
║ Suggestions:                                             ║
║ • Reduce 20ms vTaskDelay between batches to 5ms          ║
║ • Use SD card write prefetching if supported             ║
║ • Pipeline: start next batch fill while card ready       ║
╚══════════════════════════════════════════════════════════╝
```

---

## 11. Configuration Parameters

### Complete Parameter Reference

See [TUNING_GUIDE.md](TUNING_GUIDE.md) for detailed explanation of each parameter and tuning recommendations.

**Quick summary of critical parameters:**

| Parameter | File | Default | Range | Impact |
|-----------|------|---------|-------|--------|
| `CAPTURE_MODE` | app_config.h | 4 | {0,1,2,4} | Operating mode |
| `CAM_BINNING` | app_config.h | auto | {0,1} | Resolution (0=full, 1=2x2) |
| `SNAP_WARMUP_FRAMES` | app_config.h | 11 | 5-20 | First frame quality |
| `CAM_EXPOSURE_VALUE` | app_config.h | 30 | 8-33266μs | Image brightness/shutter speed |
| `CAM_GAIN_VALUE` | app_config.h | 8 | 0-72000 | Image brightness (noise) |
| `CALLBACK_WARMUP_FRAMES` | app_config.h | 11 | 5-15 | Mode 4 warmup count |
| `CALLBACK_FRAMES` | app_config.h | 4 | 1-8 | Frames per detection (mode 4) |
| `SD_BATCH_WRITE_BLOCKS` | app_config.h | 64 | 16-128 | SD write batch size |
| `SD_BATCH_RECOVERY_GAP_MS` | app_config.h | 15 | 0-30 | SD inter-batch delay |
| `WS2812_MODE` | app_config.h | 1 | {0,1,2} | LED behavior |
| `WS2812_ILLUMINATION_BRIGHTNESS` | app_config.h | 20 | 0-100% | LED brightness |
| `VL53L5CX_DET_THRESHOLD_PCT` | vl53l5cx_detection.h | 6 (4×4) / 15 (8×8) | 1-100 | Signal drop threshold (%) |
| `VL53L5CX_DET_MOTION_THRESH` | vl53l5cx_detection.h | 60 (4×4) / 100 (8×8) | 0-255 | Motion indicator threshold |

---

## 12. Image Format

### SD Card Storage Format
Each image is stored as raw blocks (no filesystem):

```
Block 0:
  │ Offset │ Size │ Content │
  │ 0      │ 4    │ Magic: "EGDI" (0x49444745) │
  │ 4      │ 4    │ Width (uint32_t, little-endian) │
  │ 8      │ 4    │ Height (uint32_t) │
  │ 12     │ 4    │ Pixel format (0 = YUV422) │
  │ 16     │ 4    │ Data size (uint32_t, bytes) │
  │ 20     │ 4    │ Timestamp (uint32_t, HAL_GetTick()) │
  │ 24     │ 4    │ Checksum (XOR of all image bytes) │
  │ 28     │ 4    │ Snapshot ID (uint32_t, sequential) │
  │ 32     │ 32   │ Reserved (zeros) │
  │ 64     │ 448  │ Image data (first 448 bytes) │

Block 1..N:
  │ Full 512 bytes of image data per block │
  │ Last block may be zero-padded │
```

### Block Allocation
- **First image**: starts at block `SD_SNAP_BASE_BLOCK` (default 3072)
- **Subsequent images**: sequential, no gaps
- **Block advancement**: `g_sd_img_base_block` tracks next free block

### Python Reader
```python
# read_sd_image.py
# Usage: python read_sd_image.py /dev/mmcblk0p1  # Linux SD card reader
#        python read_sd_image.py D:               # Windows

# Features:
# - Scan SD card for images (fixed-stride + variable-stride)
# - Bulk read (single I/O for entire image = 100x faster)
# - GUI viewer with zoom, pan, fit-to-window
# - Save as BMP or PNG
# - Auto-detect mixed resolutions
```

---

## 13. Python SD Card Reader

### `read_sd_image.py` — SD Image Viewer
```
python read_sd_image.py <device_or_drive>

Examples:
  python read_sd_image.py /dev/mmcblk0    # Linux (raw device)
  python read_sd_image.py D:              # Windows (drive letter)
  python read_sd_image.py /dev/sdb        # Linux (USB reader)
```

### Features
| Feature | Description |
|---------|-------------|
| **Bulk read** | Reads entire image in ONE I/O call (~1s vs ~120s for block-by-block) |
| **Variable-stride scanner** | Handles mixed-resolution cards (some images 1296×972, some 2592×1944) |
| **YUV422 → RGB conversion** | Proper YCbCr to sRGB for display |
| **GUI viewer** | Tkinter-based, zoom, pan, fit-to-window |
| **Image saving** | Export as BMP (original) or PNG (scaled) |

### Architecture
```
SD card
  │
  ├── scan_snapshots()
  │   ├── Pass 1: Fixed-stride scan (SNAP_BLOCKS stride from header)
  │   ├── Pass 2: Variable-stride scan (compute actual blocks from each header)
  │   └── Return: list of (block_number, width, height, data_size)
  │
  ├── rbulk(fd, block, count)
  │   └── Single os.pread() for all blocks at once
  │
  └── SDImageViewer (Tkinter)
      ├── load_snapshot(index)
      ├── zoom in/out (mouse wheel)
      ├── fit to window
      └── save as BMP/PNG
```

---

## 14. Troubleshooting

### Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| `CMW_CAMERA_Init ret=-7` | I2C conflict (camera + ToF simultaneous) | I2C arbiter should handle; if not, check mutex |
| `CMW_CAMERA_Init ret=-7` | Camera I2C corrupted from hard reset | Added retry logic (3 attempts × 200ms delay) |
| SD write STA=0x5000 | Inter-batch gap too short | Increase `SD_BATCH_RECOVERY_GAP_MS` |
| SD write STA=0x5000 at same block | Bad SD card sector | Replace SD card or format with SDFormatter |
| First capture at boot (no insect) | ToF sensor not stabilized | 5-second warmup now built into sensor_task |
| Images green-tinted | Insufficient warmup frames | Increase `SNAP_WARMUP_FRAMES` or `CALLBACK_WARMUP_FRAMES` |
| Images too dark | Exposure too short or gain too low | Increase `CAM_EXPOSURE_VALUE` or `CAM_GAIN_VALUE` |
| Images blurry | Motion blur (exposure too long) | Decrease `CAM_EXPOSURE_VALUE`, increase LED brightness |
| `CAM_DoubleBufferStart` fails | `save_buf` not declared | Ensure `#if CAPTURE_MODE == 1 \|\| CAPTURE_MODE == 4` in main.c |
| Camera init hangs on boot | I2C conflict with ToF | Camera init happens BEFORE `system_ready=1` in main_thread |
| SD card not detected | Card power-up timing | Retries up to 5 times with 500ms delay; reformat card |
| Python reader very slow | Block-by-block reads | Use bulk read (`rbulk` function, already default) |
| Python "Number of blocks different" | Mixed resolutions on card | Variable-stride scanner handles this (Pass 2) |

---

## 15. Quick Reference

### Flashing
```bash
cd nucleo/x-cube-n6-camera-capture-main/
make -j$(nproc)           # Build
st-flash write Debug/app.bin 0x8000000  # Flash via ST-Link
```

### Serial Console
```bash
screen /dev/ttyUSB0 115200    # Linux
# Or use PuTTY on COM port     # Windows
```

### Reading SD Card
```bash
python read_sd_image.py /dev/mmcblk0    # Linux
python read_sd_image.py D:              # Windows
```

### Key Configuration Changes
| Want to... | Change | File |
|------------|--------|------|
| Switch resolution | `CAM_BINNING` | app_config.h |
| Change frames per detection | `CALLBACK_FRAMES` | app_config.h |
| Adjust image brightness | `CAM_EXPOSURE_VALUE`, `CAM_GAIN_VALUE` | app_config.h |
| Fix SD write errors | `SD_BATCH_RECOVERY_GAP_MS` | app_config.h |
| Make detection more sensitive | `VL53L5CX_DET_THRESHOLD_PCT`, `VL53L5CX_DET_MOTION_THRESH` | vl53l5cx_detection.h |
| Change LED color | `WS2812_ILLUMINATION_COLOR` | app_config.h |
| Change capture mode | `CAPTURE_MODE` | app_config.h |

---

*Document generated July 2026 | Project: STM32N6 Discovery Camera + ToF Insect Capture*