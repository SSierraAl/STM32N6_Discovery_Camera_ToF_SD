#include <stdio.h>
#include <stdint.h>
#include "vl53l5cx_detection.h"

#if !VL53L5CX_DUAL_SENSOR && !TEST_TOF_MODE && (VL53L5CX_DET_RESOLUTION == 4)

#define TOF_REARM_FRAMES                 7U
#define TOF_FAST_MIN_VALID_ZONES        12U
#define TOF_FAST_STRONG_SHIFT_MM         2U
#define TOF_FAST_MAX_MAD_MM              1U
#define TOF_FAST_MIN_COHERENCE_PCT      90U

static uint16_t s_prev_distance[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_prev_valid[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_rearm_frames = 0;
static uint8_t  s_common_motion_streak = 0;

static uint32_t FastAbsI32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static int32_t FastMedianI32(const int32_t *values, uint8_t count)
{
    int32_t sorted[VL53L5CX_DET_NUM_ZONES];
    if (count == 0U) return 0;

    for (uint8_t i = 0; i < count; i++) sorted[i] = values[i];
    for (uint8_t i = 1; i < count; i++) {
        int32_t key = sorted[i];
        int j = (int)i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    if (count & 1U) return sorted[count / 2U];
    return (int32_t)(((int64_t)sorted[count / 2U - 1U] +
                      (int64_t)sorted[count / 2U]) / 2LL);
}

static uint32_t FastMedianU32(const uint32_t *values, uint8_t count)
{
    uint32_t sorted[VL53L5CX_DET_NUM_ZONES];
    if (count == 0U) return 0U;

    for (uint8_t i = 0; i < count; i++) sorted[i] = values[i];
    for (uint8_t i = 1; i < count; i++) {
        uint32_t key = sorted[i];
        int j = (int)i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    if (count & 1U) return sorted[count / 2U];
    return (uint32_t)(((uint64_t)sorted[count / 2U - 1U] +
                       (uint64_t)sorted[count / 2U]) / 2ULL);
}

int VL53L5CX_IsInsectDetectedFiltered(void)
{
    const int raw_detected = VL53L5CX_IsInsectDetected();
    VL53L5CX_DetectionResult_t res = {0};
    int32_t frame_delta[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t frame_valid[VL53L5CX_DET_NUM_ZONES] = {0};
    int32_t valid_delta[VL53L5CX_DET_NUM_ZONES];
    uint8_t valid_count = 0;

    if (raw_detected) res = VL53L5CX_GetResult();

    for (uint8_t z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t signal = 0;
        uint16_t distance = 0;
        uint8_t status = 0;
        VL53L5CX_GetZoneData(z, &signal, &distance, &status);

        uint8_t current_valid = (uint8_t)(VL53L5CX_IsZoneValid(z) &&
                                          VL53L5CX_STATUS_OK_FILT(status) &&
                                          distance > 0U);

        if (current_valid && s_prev_valid[z]) {
            frame_delta[z] = (int32_t)distance - (int32_t)s_prev_distance[z];
            frame_valid[z] = 1U;
            valid_delta[valid_count++] = frame_delta[z];
        }

        s_prev_distance[z] = distance;
        s_prev_valid[z] = current_valid;
    }

    int32_t global_delta = FastMedianI32(valid_delta, valid_count);
    uint32_t deviations[VL53L5CX_DET_NUM_ZONES];
    for (uint8_t i = 0; i < valid_count; i++)
        deviations[i] = FastAbsI32(valid_delta[i] - global_delta);

    uint32_t mad = FastMedianU32(deviations, valid_count);
    uint32_t coherence_tol = mad * 3U;
    if (coherence_tol < 1U) coherence_tol = 1U;

    uint8_t coherent = 0;
    for (uint8_t z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (frame_valid[z] && FastAbsI32(frame_delta[z] - global_delta) <= coherence_tol)
            coherent++;
    }

    uint32_t abs_global = FastAbsI32(global_delta);
    uint8_t common_motion = 0;
    if (valid_count >= TOF_FAST_MIN_VALID_ZONES) {
        if (abs_global >= TOF_FAST_STRONG_SHIFT_MM &&
            mad <= TOF_FAST_MAX_MAD_MM &&
            ((uint32_t)coherent * 100U) >=
                ((uint32_t)valid_count * TOF_FAST_MIN_COHERENCE_PCT)) {
            common_motion = 1U;
        } else if (abs_global == 1U && mad == 0U && coherent == valid_count) {
            common_motion = 1U;
        }
    }

    if (common_motion) {
        if (s_common_motion_streak < 255U) s_common_motion_streak++;
    } else {
        s_common_motion_streak = 0U;
    }

    if (s_rearm_frames > 0U) {
        s_rearm_frames--;
        if (s_rearm_frames > 0U) return 0;
    }

    if (!raw_detected) return 0;

    uint32_t local_residual = 0;
    uint8_t local_valid = 0;
    if (res.affected_count == 1U) {
        uint8_t z = res.affected_zones[0];
        if (z < VL53L5CX_DET_NUM_ZONES && frame_valid[z]) {
            local_residual = FastAbsI32(frame_delta[z] - global_delta);
            local_valid = 1U;
        }
    }

    uint32_t local_limit = (abs_global >= 3U) ? 1U : 0U;
    uint8_t veto = (uint8_t)(res.trigger_source == VL53L5CX_TRIG_SIGNAL &&
                             res.affected_count == 1U &&
                             common_motion &&
                             s_common_motion_streak >= 2U &&
                             local_valid &&
                             local_residual <= local_limit);

    printf("FASTNOISE,Gfd=%ld,MADfd=%lu,cohFd=%u/%u,Rfd=%lu,streak=%u,veto=%u\r\n",
           (long)global_delta,
           (unsigned long)mad,
           (unsigned)coherent,
           (unsigned)valid_count,
           (unsigned long)local_residual,
           (unsigned)s_common_motion_streak,
           (unsigned)veto);

    if (veto) return 0;

    s_rearm_frames = TOF_REARM_FRAMES;
    return 1;
}

#else

int VL53L5CX_IsInsectDetectedFiltered(void)
{
    return VL53L5CX_IsInsectDetected();
}

#endif
