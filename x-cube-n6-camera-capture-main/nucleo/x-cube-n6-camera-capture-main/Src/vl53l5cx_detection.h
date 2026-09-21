#ifndef VL53L5CX_DETECTION_SRC_WRAPPER_H
#define VL53L5CX_DETECTION_SRC_WRAPPER_H

#include "../Inc/app_config.h"

/* Tuning-only diagnostics added while characterizing the robust detector.
   Previous value: 1 (printed NOISEMETRIC/FASTNOISE for every raw candidate).
   Disable the repeated NOISEMETRIC/FASTNOISE diagnostics when the per-zone
   detector is active, including camera mode. ZFRAME remains unchanged. */
#ifndef VL53L5CX_DET_TUNING_DIAGNOSTICS
#if (TEST_TOF_MODE && (VL53L5CX_DET_ZONE_SURVEY || VL53L5CX_DET_HIGH_SENS_TEST)) || \
    (!TEST_TOF_MODE && !VL53L5CX_DUAL_SENSOR && \
     (VL53L5CX_DET_RESOLUTION == 4) && VL53L5CX_DET_HIGH_SENS_CAMERA)
#define VL53L5CX_DET_TUNING_DIAGNOSTICS  0
#else
#define VL53L5CX_DET_TUNING_DIAGNOSTICS  1
#endif
#endif

#ifndef VL53L5CX_DET_DEBUG_NOISE_METRICS
#define VL53L5CX_DET_DEBUG_NOISE_METRICS VL53L5CX_DET_TUNING_DIAGNOSTICS
#endif

#include "../Inc/vl53l5cx_detection.h"

#if VL53L5CX_DET_TUNING_DIAGNOSTICS
#include <stdio.h>
#endif

/* Do not redefine printf here: this header is also included by main.c,
   app_thread.c, and vl53l5cx_detection.c. Individual tuning messages in
   platform.c are gated at their call sites. */

#if defined(TOF_FAST_NOISE_FILTER_WIRING_REV)
#include <string.h>
int VL53L5CX_IsInsectDetectedFiltered(void);

#if !VL53L5CX_DUAL_SENSOR && !TEST_TOF_MODE && (VL53L5CX_DET_RESOLUTION == 4)
/* Persistent-event guard for the task-level single-ToF path.
   The first SIGNAL event is always accepted. If the same single zone remains
   above the raw baseline threshold after capture, it is considered the same
   event unless the already-read ToF frames show a genuinely new local change.
   A clean raw frame clears the latch, so a later insect is accepted normally. */
#define TOF_TASK_LATCH_HISTORY_FRAMES      3U
#define TOF_TASK_LATCH_SIGNAL_EVID_PCT     2U
#define TOF_TASK_LATCH_DISTANCE_EVID_MM    3U

static uint16_t s_task_signal_latched_mask = 0U;
static uint32_t s_task_prev_signal[VL53L5CX_DET_NUM_ZONES] = {0};
static uint16_t s_task_prev_distance[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_task_prev_signal_valid[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_task_prev_distance_valid[VL53L5CX_DET_NUM_ZONES] = {0};
static uint16_t s_task_hist_signal[VL53L5CX_DET_NUM_ZONES][TOF_TASK_LATCH_HISTORY_FRAMES] = {{0}};
static uint16_t s_task_hist_distance[VL53L5CX_DET_NUM_ZONES][TOF_TASK_LATCH_HISTORY_FRAMES] = {{0}};
static uint8_t  s_task_hist_pos = 0U;
static uint32_t s_task_filter_generation = 0U;

static inline void VL53L5CX_ResetTaskFilterState(void)
{
    s_task_signal_latched_mask = 0U;
    memset(s_task_prev_signal, 0, sizeof(s_task_prev_signal));
    memset(s_task_prev_distance, 0, sizeof(s_task_prev_distance));
    memset(s_task_prev_signal_valid, 0, sizeof(s_task_prev_signal_valid));
    memset(s_task_prev_distance_valid, 0, sizeof(s_task_prev_distance_valid));
    memset(s_task_hist_signal, 0, sizeof(s_task_hist_signal));
    memset(s_task_hist_distance, 0, sizeof(s_task_hist_distance));
    s_task_hist_pos = 0U;
}

static inline uint32_t VL53L5CX_TaskAbsDiffU32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static inline uint32_t VL53L5CX_TaskHistoryMax(
    const uint16_t hist[VL53L5CX_DET_NUM_ZONES][TOF_TASK_LATCH_HISTORY_FRAMES],
    uint8_t zone)
{
    uint32_t max_value = 0U;
    if (zone >= VL53L5CX_DET_NUM_ZONES) return 0U;
    for (uint8_t i = 0U; i < TOF_TASK_LATCH_HISTORY_FRAMES; i++) {
        if (hist[zone][i] > max_value) max_value = hist[zone][i];
    }
    return max_value;
}

static inline int VL53L5CX_IsInsectDetectedTaskLatched(void)
{
    const uint32_t filter_generation = VL53L5CX_GetDetectionFilterGeneration();
    if (filter_generation != s_task_filter_generation) {
        VL53L5CX_ResetTaskFilterState();
        s_task_filter_generation = filter_generation;
    }

    const int accepted = VL53L5CX_IsInsectDetectedFiltered();
    VL53L5CX_DetectionResult_t res = VL53L5CX_GetResult();

    /* Maintain a tiny task-level temporal history from the frame already read
       by VL53L5CX_Update(); no additional sensor/I2C transaction is performed. */
    for (uint8_t z = 0U; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t signal = 0U;
        uint16_t distance = 0U;
        uint8_t status = 0U;
        VL53L5CX_GetZoneData(z, &signal, &distance, &status);

        const uint8_t fresh = (uint8_t)(VL53L5CX_IsZoneValid(z) &&
                                        VL53L5CX_STATUS_OK_FILT(status));
        const uint8_t signal_valid = (uint8_t)(fresh &&
                                               signal >= VL53L5CX_DET_MIN_SIGNAL);
        const uint8_t distance_valid = (uint8_t)(fresh && distance > 0U);

        s_task_hist_signal[z][s_task_hist_pos] = 0U;
        s_task_hist_distance[z][s_task_hist_pos] = 0U;

        if (signal_valid && s_task_prev_signal_valid[z] &&
            s_task_prev_signal[z] >= VL53L5CX_DET_MIN_SIGNAL) {
            uint32_t diff = VL53L5CX_TaskAbsDiffU32(signal, s_task_prev_signal[z]);
            uint32_t pct = (uint32_t)(((uint64_t)diff * 100ULL) /
                                      (uint64_t)s_task_prev_signal[z]);
            s_task_hist_signal[z][s_task_hist_pos] =
                (uint16_t)((pct > 65535U) ? 65535U : pct);
        }

        if (distance_valid && s_task_prev_distance_valid[z]) {
            uint32_t diff = VL53L5CX_TaskAbsDiffU32((uint32_t)distance,
                                                    (uint32_t)s_task_prev_distance[z]);
            s_task_hist_distance[z][s_task_hist_pos] =
                (uint16_t)((diff > 65535U) ? 65535U : diff);
        }

        s_task_prev_signal[z] = signal;
        s_task_prev_distance[z] = distance;
        s_task_prev_signal_valid[z] = signal_valid;
        s_task_prev_distance_valid[z] = distance_valid;
    }
    s_task_hist_pos = (uint8_t)((s_task_hist_pos + 1U) % TOF_TASK_LATCH_HISTORY_FRAMES);

    /* Clear stale latches as soon as the raw detector sees a genuinely clear
       frame. MOTION-only also proves that no signal zone is over threshold. */
    if (!res.insect_detected || res.trigger_source == VL53L5CX_TRIG_MOTION) {
        s_task_signal_latched_mask = 0U;
    } else if (res.trigger_source == VL53L5CX_TRIG_SIGNAL) {
        /* In SIGNAL-only frames every affected zone is a signal zone, so zones
           that disappeared from this mask have genuinely cleared. */
        uint16_t current_signal_mask = 0U;
        for (uint8_t i = 0U; i < res.affected_count; i++) {
            uint8_t z = res.affected_zones[i];
            if (z < 16U) current_signal_mask |= (uint16_t)(1U << z);
        }
        s_task_signal_latched_mask &= current_signal_mask;
    }

    if (!accepted) return 0;

    /* Only guard single-zone SIGNAL persistence. Multi-zone/ambiguous events
       remain fail-open, and BOTH always counts as new motion evidence. */
    if (res.affected_count == 1U &&
        (res.trigger_source == VL53L5CX_TRIG_SIGNAL ||
         res.trigger_source == VL53L5CX_TRIG_BOTH)) {
        const uint8_t z = res.affected_zones[0];
        if (z < 16U) {
            const uint16_t bit = (uint16_t)(1U << z);

            if (res.trigger_source == VL53L5CX_TRIG_SIGNAL &&
                (s_task_signal_latched_mask & bit) != 0U) {
                const uint32_t recent_signal = VL53L5CX_TaskHistoryMax(s_task_hist_signal, z);
                const uint32_t recent_distance = VL53L5CX_TaskHistoryMax(s_task_hist_distance, z);

                if (recent_signal < TOF_TASK_LATCH_SIGNAL_EVID_PCT &&
                    recent_distance < TOF_TASK_LATCH_DISTANCE_EVID_MM) {
#if VL53L5CX_DET_TUNING_DIAGNOSTICS
                    printf("TOFDEC,signal_hold,z=%u,histRfd=%lu,histRfs=%lu\r\n",
                           (unsigned)z,
                           (unsigned long)recent_distance,
                           (unsigned long)recent_signal);
#endif
                    return 0;
                }
            }

            /* First event, BOTH, or a same-zone event with new temporal
               evidence: accept and keep that zone latched until it clears. */
            s_task_signal_latched_mask |= bit;
        }
    }

    return 1;
}

#define VL53L5CX_IsInsectDetected VL53L5CX_IsInsectDetectedTaskLatched
#else
#define VL53L5CX_IsInsectDetected VL53L5CX_IsInsectDetectedFiltered
#endif

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
