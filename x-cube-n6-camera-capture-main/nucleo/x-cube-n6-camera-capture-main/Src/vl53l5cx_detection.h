#ifndef VL53L5CX_DETECTION_SRC_WRAPPER_H
#define VL53L5CX_DETECTION_SRC_WRAPPER_H

#include "../Inc/vl53l5cx_detection.h"

#if defined(TOF_FAST_NOISE_FILTER_WIRING_REV)
int VL53L5CX_IsInsectDetectedFiltered(void);
#define VL53L5CX_IsInsectDetected VL53L5CX_IsInsectDetectedFiltered
#endif

#endif /* VL53L5CX_DETECTION_SRC_WRAPPER_H */