#ifndef VL53L5CX_DETECTION_SRC_WRAPPER_H
#define VL53L5CX_DETECTION_SRC_WRAPPER_H

#include "../Inc/vl53l5cx_detection.h"

/* app_thread.c includes app_thread.h before this header. Redirect only that
   task-level query; the detector implementation itself keeps the raw API. */
#ifdef APP_THREAD_H
int VL53L5CX_IsInsectDetectedFiltered(void);
#define VL53L5CX_IsInsectDetected VL53L5CX_IsInsectDetectedFiltered
#endif

#endif /* VL53L5CX_DETECTION_SRC_WRAPPER_H */
