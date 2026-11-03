# AGENTS.MD — Project Guide for AI Coding Agents

## 1. Build System

### Build from STM32CubeIDE (mandatory)

The project **cannot** be built from the command line. The `Makefile` in `nucleo/x-cube-n6-camera-capture-main/` is present but `arm-none-eabi-gcc` + `make` are **not** on the shell PATH. All builds must go through **STM32CubeIDE**.

**How to build:**
1. Open STM32CubeIDE
2. File → Import → Existing Projects → select `nucleo/x-cube-n6-camera-capture-main/`
3. Right-click project → Build Project (or Ctrl+B)
4. Flash via ST-Link debugger

**Project structure:**
```
Discovery/                              ← repo root (this file lives here)
├── x-cube-n6-camera-capture-main/
│   └── nucleo/x-cube-n6-camera-capture-main/
│       ├── Src/                        ← .c source files
│       ├── Inc/                         ← .h headers (app_config.h, app_cam.h, image_filter.h)
│       ├── Lib/Camera_Middleware/       ← CMW middleware + IMX335 sensor driver
│       ├── STM32CubeIDE/               ← STM32CubeIDE project files
│       ├── Makefile                    ← present but unusable from shell
│       └── EWARM/                      ← IAR EWARM project files (not actively used)
├── docs/                               ← project documentation
├── AGENTS.MD                          ← this file
└── *_analysis.py, *_datalogger.py     ← Python tools for SD image decoding
```

### Key files

| File | Purpose |
|------|---------|
| `Inc/app_config.h` | **Single central configuration header.** All tuneable constants live here. |
| `Inc/app_cam.h` | Camera API declarations with full Doxygen-style docs. |
| `Src/app_cam.c` | Camera implementation: init, capture, AREA3/ROI, pipe config. |
| `Src/app_thread.c` | RTOS tasks: camera, sensor (ToF), storage. Multi-resolution pipeline. |
| `Inc/image_filter.h` | Detection algorithm parameters: downsample, thresholds, ROI. |
| `Src/vl53l5cx_detection.c` | VL53L5CX ToF sensor: init, ranging, baseline, zone detector. |

### Debug console

The firmware prints `[TAG]`-prefixed log lines over UART at 115200 baud. Connect via any serial terminal (PuTTY, screen, Arduino IDE Serial Monitor).

### Python companion tools

All at the repo root. Run from host PC (not the MCU):

- `read_sd_image.py`  — decode raw YUV422 frames from SD dump
- `vl53l5cx_zone_monitor.py` — real-time ToF zone visualisation
- `vl53l5cx_datalogger.py` — ToF datalog capture
- `SD_Image_Viewer.py` — GUI image viewer for captured frames

---

## 2. Code Style Guidelines

### 2.1 Configuration Header (`app_config.h`)

**Every tunable value** must live in `Inc/app_config.h`. Do **not** scatter `#define` constants in `.c` files. The file is organised in numbered sections:

```c
/* ================================================================
   SECTION N: SECTION TITLE
   ================================================================ */

/** Doxygen description of the parameter group. */
#ifndef SYMBOL_NAME
#define SYMBOL_NAME    default_value
#endif
```

- Every `#define` gets a **Doxygen `/** ... */` comment** explaining what it does and any interactions.
- Use `#ifndef` guards so build-time overrides from IDE preprocessor defines work.
- Use `#error` for invalid combinations (see `app_config.h` for examples).
- All resolution-dependent branches use `#if` chains, never runtime checks.

### 2.2 Include Style

Includes in `.c` files follow this order with blank-line separators:

```c
#include <stdio.h>        // 1. C standard library
#include <string.h>
#include "app_thread.h"   // 2. Project headers (alphabetical)
#include "app_config.h"
#include "app_cam.h"
#include "image_filter.h"
#include "stm32n6xx_hal.h"
```

Headers use the `#ifndef` / `#define` / `#endif` pattern (never `#pragma once`).

### 2.3 Formatting

| Rule | Standard |
|------|----------|
| Indentation | **2 spaces** (not tabs) |
| Braces | `{` on same line for functions, **next line** for control flow |
| Line length | Soft limit ~120 chars |
| Naming | `snake_case` for functions and variables, `UPPER_SNAKE` for defines |
| Types | `typedef struct` with `_t` suffix; `uint8_t`, `uint32_t`, etc. from `<stdint.h>` |
| Static helpers | `static int CAM_WriteSensorReg(...)` — private helpers are file-static |
| Extern globals | Declared in `.h` files, defined in one `.c`, referenced elsewhere via `extern` |

### 2.4 Comment Conventions

**Function declarations** in `.h` files use Doxygen:

```c
/**
 * @brief  One-line summary.
 *
 * Longer description of what the function does, when to call it,
 * and any side effects.
 *
 * @param  name   Description of parameter.
 * @param  buf    Buffer to fill (must be >= buf_size bytes).
 * @return 0 on success, -1 on failure.
 */
```

**Internal comments** in `.c` files use `/* sentence case */` for section headers
and `//` for inline notes:

```c
/* ================================================================
   SECTION HEADING
   ================================================================ */

void CAM_Function(void)
{
  HAL_Delay(20);  // brief inline note
```

**Algorithm sections** use boxed blocks:

```c
  /* ---------------------------------------------------------------
     Step N: Step title
     Explanation of what this step does and why.
     --------------------------------------------------------------- */
```

**Critical or non-obvious fixes** include a `/* === CRITICAL === */` or
`/* IMPORTANT: reason */` annotation so agents don't accidentally reorder.

### 2.5 Multi-Resolution Pipeline Convention

The capture pipeline has three stages (see `app_thread.c`, `CAM_CmdSnap` handler):

1. **Stage 1** — Downscale 1296×972 → 640×480 → `IF_Process()` detection
2. **Stage 2** — Save 2 batch frames (1296×972) to SD
3. **Stage 3** — `CAM_CaptureROI()`: DeInit→Init→PipeConfig→DoubleBufferStart→AREA3→warmup+capture→Stop

Stage 3 AREA3 must **always** program the sensor **after** `CMW_CAMERA_DoubleBufferStart()` because `Camera_Drv.Start()` resets the IMX335 to full 2592×1944.

### 2.6 Common Pitfalls

- **`Camera_Drv.Start()`** reinitialises the IMX335 AREA registers to full-frame. Program AREA3 **after** `DoubleBufferStart` (which calls Start).
- **`g_capture_busy`** is set by the ToF task before capture and cleared by the camera callback **after Stage 3** finishes. The ToF task must wait for this flag before restarting ranging.
- **PSRAM budget**: 1296×972 @ YUV422 = ~2.5 MB/frame. 2592×1944 = ~10 MB/frame. Full-res batch capture only works via AREA3 crop.
- **Storage semaphore**: `Capture_RequestSnapshot()` waits for Stage 2 SD tokens. Stage 3 SD saves happen after it returns — ToF ranging must not restart until `g_capture_busy == 0`.

### 2.7 Threading Model

| Task | Role | Stack (words) |
|------|------|---------------|
| `camera_task` | Camera control + detection + Stage 3 | 4096 |
| `sensor_task` | ToF ranging + trigger + SD Stage 2 | 4096 |
| `storage_task` | SD block write + meta | 2048 |

Tasks communicate via FreeRTOS queues (`camera_cmd_queue`, `storage_cmd_queue`)
and semaphores (`camera_ready_sem`, `storage_done_sem`). The `g_capture_busy`
volatile flag guards the critical capture window across tasks.

### 2.8 When Modifying Code

1. **Always test** with `CAPTURE_MODE=4`, `CAM_BINNING=1`, `CAM_EXPOSURE_MODE=1`.
2. **Keep `#ifndef` guards** around all `#define`s in `app_config.h`.
3. **Match the 2-space indent** and `{` on its own line for control flow.
4. **Document timing impact** in comments for any blocking operation.
5. **Never hardcode** sensor registers — define in `app_cam.c` with a section comment.
6. **Prefer fixed-width types** (`uint32_t`, not `int`) for hardware-facing code.
