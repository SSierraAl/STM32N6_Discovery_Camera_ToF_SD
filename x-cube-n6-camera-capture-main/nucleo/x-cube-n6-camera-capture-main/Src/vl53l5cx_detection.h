#ifndef VL53L5CX_DETECTION_SRC_WRAPPER_H
#define VL53L5CX_DETECTION_SRC_WRAPPER_H

#include "../Inc/vl53l5cx_detection.h"

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