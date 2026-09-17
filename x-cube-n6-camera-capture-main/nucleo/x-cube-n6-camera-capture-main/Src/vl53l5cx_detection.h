#ifndef VL53L5CX_DETECTION_SRC_WRAPPER_H
#define VL53L5CX_DETECTION_SRC_WRAPPER_H

/* Tuning-only diagnostics added while characterizing the robust detector.
   Keep at 1 during validation. Set to 0 for the deployed production build:
   - compiles out the observation-only NOISEMETRIC calculations/print
   - suppresses FASTNOISE and TOFDEC UART prints from platform.c
   - does NOT disable the robust filtering/history/confirmation algorithm. */
#ifndef VL53L5CX_DET_TUNING_DIAGNOSTICS
#define VL53L5CX_DET_TUNING_DIAGNOSTICS  1
#endif

#ifndef VL53L5CX_DET_DEBUG_NOISE_METRICS
#define VL53L5CX_DET_DEBUG_NOISE_METRICS VL53L5CX_DET_TUNING_DIAGNOSTICS
#endif

#include "../Inc/vl53l5cx_detection.h"

/* platform.c currently contains only tuning printf() calls (FASTNOISE/TOFDEC).
   Suppress those at compile time in the production build without changing the
   detector computations that feed the decision layer. */
#if defined(_PLATFORM_H_) && !VL53L5CX_DET_TUNING_DIAGNOSTICS
#define printf(...) ((void)0)
#endif

#if defined(TOF_FAST_NOISE_FILTER_WIRING_REV)
int VL53L5CX_IsInsectDetectedFiltered(void);
#define VL53L5CX_IsInsectDetected VL53L5CX_IsInsectDetectedFiltered

#if !VL53L5CX_DUAL_SENSOR && !TEST_TOF_MODE
/* sensor_task never stops the single ToF during camera/SD capture.  Keep the
   task's legacy StartRanging() calls harmless unless a real StopRanging()
   occurred (the adaptive 2/2 baseline refresh). */
static uint8_t s_task_tof_ranging_active = 0U;

static inline void VL53L5CX_StartRangingTaskSafe(void)
{
    if (!s_task_tof_ranging_active) {
        VL53L5CX_StartRanging();
        s_task_tof_ranging_active = 1U;
    }
}

static inline void VL53L5CX_StopRangingTaskSafe(void)
{
    VL53L5CX_StopRanging();
    s_task_tof_ranging_active = 0U;
}

#define VL53L5CX_StartRanging VL53L5CX_StartRangingTaskSafe
#define VL53L5CX_StopRanging  VL53L5CX_StopRangingTaskSafe
#endif
#endif

#endif /* VL53L5CX_DETECTION_SRC_WRAPPER_H */
