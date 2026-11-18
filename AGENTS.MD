# AGENTS.MD — Project Guide for AI Coding Agents

## 1. Build and source navigation

Build and flash with STM32CubeIDE using the project at
`x-cube-n6-camera-capture-main/nucleo/x-cube-n6-camera-capture-main/STM32CubeIDE/NUCLEO-N657X0-Q/`.
The CubeIDE project uses linked sources, so import it in place. Use the repository
[README](README.md) for startup and the [firmware guide](docs/firmware-guide.md)
for source entry points, operating modes and verification. The
[camera pipeline guide](docs/MASSIF-Camera-Pipeline.md) describes buffer ownership,
the optical detector interface and crop selection.

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

### 2.5 Capture pipeline invariants

- The default mode 4 profile uses one full-field acquisition. Derive the
  overview and dynamic crop from the same exposure.
- Complete native reads before resizing the DMA source in place. Wait for
  storage to release a queued buffer before any code changes or reuses it.
- Use the [operating mode table](docs/firmware-guide.md#operating-modes) when
  modifying a build variant. The fixed DCMIPP crop and the older IMX335 AREA3
  path have different configuration and startup sequences.
- In the older AREA3 path, program the sensor ROI after
  `CMW_CAMERA_DoubleBufferStart()`. The driver start restores full-field readout.

### 2.6 When Modifying Code

1. **Always test** with `CAPTURE_MODE=4`, `CAM_BINNING=1`, `CAM_EXPOSURE_MODE=1`.
2. **Keep `#ifndef` guards** around all `#define`s in `app_config.h`.
3. **Match the 2-space indent** and `{` on its own line for control flow.
4. **Document timing impact** in comments for any blocking operation.
5. **Never hardcode** sensor registers — define in `app_cam.c` with a section comment.
6. **Prefer fixed-width types** (`uint32_t`, not `int`) for hardware-facing code.
