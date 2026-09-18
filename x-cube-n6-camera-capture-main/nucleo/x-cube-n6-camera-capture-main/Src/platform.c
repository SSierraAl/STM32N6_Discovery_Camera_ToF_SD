/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "i2c_arbiter.h"
#include "vl53l5cx_detection.h"

extern I2C_HandleTypeDef 	hi2c1;

/* The physical I2C1 bus is ALSO driven by the camera (via a separate
   BSP_I2C1_* handle, see i2c_arbiter.h). Every transaction here takes
   g_i2c1_mutex so the ToF driver and the camera/ISP never interleave
   mid-transaction on the shared bus. */

uint8_t VL53L5CX_RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	uint8_t status = 0;
	uint8_t data_write[2];
	uint8_t data_read[1];

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	if (g_i2c1_mutex) xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
	status = HAL_I2C_Master_Transmit(&hi2c1, (p_platform->address << 1), data_write, 2, 100);
	status = HAL_I2C_Master_Receive(&hi2c1, (p_platform->address << 1), data_read, 1, 100);
	if (g_i2c1_mutex) xSemaphoreGive(g_i2c1_mutex);
	*p_value = data_read[0];
  
	return status;
}

uint8_t VL53L5CX_WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	uint8_t data_write[3];
	uint8_t status = 0;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = value & 0xFF;
	if (g_i2c1_mutex) xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
	status = HAL_I2C_Master_Transmit(&hi2c1,(p_platform->address << 1), data_write, 3, 100);
	if (g_i2c1_mutex) xSemaphoreGive(g_i2c1_mutex);

	return status;
}

uint8_t VL53L5CX_WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status;
	if (g_i2c1_mutex) xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
	status = HAL_I2C_Mem_Write(&hi2c1, (p_platform->address << 1), RegisterAdress,
									I2C_MEMADD_SIZE_16BIT, p_values, size, 65535);
	if (g_i2c1_mutex) xSemaphoreGive(g_i2c1_mutex);
	return status;
}

uint8_t VL53L5CX_RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t status;
	uint8_t data_write[2];
	data_write[0] = (RegisterAdress>>8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	if (g_i2c1_mutex) xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
	status = HAL_I2C_Master_Transmit(&hi2c1, (p_platform->address << 1), data_write, 2, 100);
	status += HAL_I2C_Master_Receive(&hi2c1, (p_platform->address << 1), p_values, size, 100);
	if (g_i2c1_mutex) xSemaphoreGive(g_i2c1_mutex);

	return status;
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform)
{
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO to LOW */
	VL53L5CX_WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	VL53L5CX_WaitMs(p_platform, 100);
  
	return 0;
}

void VL53L5CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t VL53L5CX_WaitMs(
		VL53L5CX_Platform *p_platform,
               uint32_t TimeMs)
{
	HAL_Delay(TimeMs);
	return 0;
}

#if !VL53L5CX_DUAL_SENSOR && !TEST_TOF_MODE && (VL53L5CX_DET_RESOLUTION == 4)

#define TOF_REARM_FRAMES                  5U
#define TOF_HISTORY_FRAMES                3U
#define TOF_FAST_MIN_VALID_ZONES         12U
#define TOF_FAST_STRONG_SHIFT_MM          2U
#define TOF_FAST_MAX_MAD_MM               1U
#define TOF_FAST_MIN_COHERENCE_PCT       90U
#define TOF_NEIGHBOR_SUPPORT_PCT          1U
#define TOF_LOCAL_EVID_SIGNAL_PCT         2U
#define TOF_LOCAL_EVID_DISTANCE_MM        3U
/* A strong single-zone baseline change remains fail-open even when it was
   already present in the first post-baseline frame. Weaker single-zone signal
   candidates must show a fresh local edge before they can trigger a capture. */
#define TOF_SIGNAL_STRONG_DROP_PCT        12U
/* Five consecutive weak/stable rejects match the field trace immediately
   preceding the false edge, while a real temporal edge is still accepted on
   its first frame. At 15 Hz this adapts after roughly one third of a second. */
#define TOF_STABLE_REFRESH_FRAMES         5U

#define TOF_DEC_SIGNAL_ACCEPT             1U
#define TOF_DEC_BOTH_ACCEPT               2U
#define TOF_DEC_MOTION_MULTI_ACCEPT       3U
#define TOF_DEC_MOTION_LOCAL_ACCEPT       4U
#define TOF_DEC_MOTION_PENDING            5U
#define TOF_DEC_MOTION_CONFIRMED          6U
#define TOF_DEC_VIBRATION_REJECT          7U
#define TOF_DEC_FAIL_OPEN_ACCEPT          8U
#define TOF_DEC_SIGNAL_STABLE_REJECT      9U

static uint16_t s_prev_distance[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_prev_valid[VL53L5CX_DET_NUM_ZONES] = {0};
static uint32_t s_prev_signal[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_prev_signal_valid[VL53L5CX_DET_NUM_ZONES] = {0};

static uint16_t s_hist_dist_residual[VL53L5CX_DET_NUM_ZONES][TOF_HISTORY_FRAMES] = {{0}};
static uint16_t s_hist_signal_residual[VL53L5CX_DET_NUM_ZONES][TOF_HISTORY_FRAMES] = {{0}};
static uint8_t  s_hist_dist_valid[VL53L5CX_DET_NUM_ZONES][TOF_HISTORY_FRAMES] = {{0}};
static uint8_t  s_hist_signal_valid[VL53L5CX_DET_NUM_ZONES][TOF_HISTORY_FRAMES] = {{0}};
static uint8_t  s_hist_pos = 0U;

static uint8_t  s_rearm_frames = 0U;
static uint8_t  s_common_motion_streak = 0U;
static uint8_t  s_motion_confirm_pending = 0U;
static uint8_t  s_motion_pending_zone = 0U;
static uint8_t  s_stable_reject_streak = 0U;
static uint8_t  s_baseline_refresh_requested = 0U;
/* One automatic refresh per uninterrupted raw-signal episode. A clear frame
   rearms it; a persistently bad sensor therefore cannot enter a refresh loop. */
static uint8_t  s_stable_refresh_armed = 1U;
static uint32_t s_filter_generation = 0U;

void VL53L5CX_ResetDetectionFilterState(void)
{
    memset(s_prev_distance, 0, sizeof(s_prev_distance));
    memset(s_prev_valid, 0, sizeof(s_prev_valid));
    memset(s_prev_signal, 0, sizeof(s_prev_signal));
    memset(s_prev_signal_valid, 0, sizeof(s_prev_signal_valid));
    memset(s_hist_dist_residual, 0, sizeof(s_hist_dist_residual));
    memset(s_hist_signal_residual, 0, sizeof(s_hist_signal_residual));
    memset(s_hist_dist_valid, 0, sizeof(s_hist_dist_valid));
    memset(s_hist_signal_valid, 0, sizeof(s_hist_signal_valid));
    s_hist_pos = 0U;
    s_rearm_frames = TOF_REARM_FRAMES;
    s_common_motion_streak = 0U;
    s_motion_confirm_pending = 0U;
    s_motion_pending_zone = 0U;
    s_stable_reject_streak = 0U;
    s_baseline_refresh_requested = 0U;
    s_filter_generation++;
}

uint32_t VL53L5CX_GetDetectionFilterGeneration(void)
{
    return s_filter_generation;
}

int VL53L5CX_TakeBaselineRefreshRequest(void)
{
    if (!s_baseline_refresh_requested) return 0;
    s_baseline_refresh_requested = 0U;
    return 1;
}

static uint32_t FastAbsI32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static uint16_t FastClampU16(uint32_t value)
{
    return (value > 65535U) ? 65535U : (uint16_t)value;
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

static uint32_t FastHistoryMax(const uint16_t hist[VL53L5CX_DET_NUM_ZONES][TOF_HISTORY_FRAMES],
                               const uint8_t valid[VL53L5CX_DET_NUM_ZONES][TOF_HISTORY_FRAMES],
                               uint8_t zone)
{
    uint32_t max_value = 0U;
    if (zone >= VL53L5CX_DET_NUM_ZONES) return 0U;

    for (uint8_t i = 0; i < TOF_HISTORY_FRAMES; i++) {
        if (valid[zone][i] && hist[zone][i] > max_value)
            max_value = hist[zone][i];
    }
    return max_value;
}

int VL53L5CX_IsInsectDetectedFiltered(void)
{
    const int raw_detected = VL53L5CX_IsInsectDetected();
    VL53L5CX_DetectionResult_t res = {0};

    int32_t frame_delta[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t frame_valid[VL53L5CX_DET_NUM_ZONES] = {0};
    int32_t valid_delta[VL53L5CX_DET_NUM_ZONES];
    uint8_t valid_count = 0U;

    int32_t signal_delta_pct[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t signal_frame_valid[VL53L5CX_DET_NUM_ZONES] = {0};
    int32_t valid_signal_delta[VL53L5CX_DET_NUM_ZONES];
    uint8_t signal_valid_count = 0U;
    uint32_t current_signal[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t current_signal_valid[VL53L5CX_DET_NUM_ZONES] = {0};

    if (raw_detected) res = VL53L5CX_GetResult();

    for (uint8_t z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        uint32_t signal = 0;
        uint16_t distance = 0;
        uint8_t status = 0;
        VL53L5CX_GetZoneData(z, &signal, &distance, &status);

        uint8_t zone_valid = (uint8_t)(VL53L5CX_IsZoneValid(z) &&
                                       VL53L5CX_STATUS_OK_FILT(status));
        uint8_t distance_valid = (uint8_t)(zone_valid && distance > 0U);
        uint8_t signal_valid = (uint8_t)(zone_valid &&
                                         signal >= VL53L5CX_DET_MIN_SIGNAL);

        current_signal[z] = signal;
        current_signal_valid[z] = signal_valid;

        if (distance_valid && s_prev_valid[z]) {
            frame_delta[z] = (int32_t)distance - (int32_t)s_prev_distance[z];
            frame_valid[z] = 1U;
            valid_delta[valid_count++] = frame_delta[z];
        }

        if (signal_valid && s_prev_signal_valid[z] &&
            s_prev_signal[z] >= VL53L5CX_DET_MIN_SIGNAL) {
            int64_t numerator = ((int64_t)signal - (int64_t)s_prev_signal[z]) * 100LL;
            signal_delta_pct[z] = (int32_t)(numerator / (int64_t)s_prev_signal[z]);
            signal_frame_valid[z] = 1U;
            valid_signal_delta[signal_valid_count++] = signal_delta_pct[z];
        }

        s_prev_distance[z] = distance;
        s_prev_valid[z] = distance_valid;
        s_prev_signal[z] = signal;
        s_prev_signal_valid[z] = signal_valid;
    }

    int32_t global_delta = FastMedianI32(valid_delta, valid_count);
    uint32_t deviations[VL53L5CX_DET_NUM_ZONES];
    for (uint8_t i = 0; i < valid_count; i++)
        deviations[i] = FastAbsI32(valid_delta[i] - global_delta);

    uint32_t mad = FastMedianU32(deviations, valid_count);
    uint32_t coherence_tol = mad * 3U;
    if (coherence_tol < 1U) coherence_tol = 1U;

    uint8_t coherent = 0U;
    for (uint8_t z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (frame_valid[z] && FastAbsI32(frame_delta[z] - global_delta) <= coherence_tol)
            coherent++;
    }

    int32_t global_signal_delta = FastMedianI32(valid_signal_delta, signal_valid_count);
    uint32_t signal_deviations[VL53L5CX_DET_NUM_ZONES];
    for (uint8_t i = 0; i < signal_valid_count; i++)
        signal_deviations[i] = FastAbsI32(valid_signal_delta[i] - global_signal_delta);
    uint32_t signal_mad = FastMedianU32(signal_deviations, signal_valid_count);

    for (uint8_t z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        s_hist_dist_valid[z][s_hist_pos] = 0U;
        s_hist_signal_valid[z][s_hist_pos] = 0U;

        if (frame_valid[z]) {
            s_hist_dist_residual[z][s_hist_pos] =
                FastClampU16(FastAbsI32(frame_delta[z] - global_delta));
            s_hist_dist_valid[z][s_hist_pos] = 1U;
        }
        if (signal_frame_valid[z]) {
            s_hist_signal_residual[z][s_hist_pos] =
                FastClampU16(FastAbsI32(signal_delta_pct[z] - global_signal_delta));
            s_hist_signal_valid[z][s_hist_pos] = 1U;
        }
    }
    s_hist_pos = (uint8_t)((s_hist_pos + 1U) % TOF_HISTORY_FRAMES);

    uint32_t abs_global = FastAbsI32(global_delta);
    uint8_t common_motion = 0U;
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
        s_common_motion_streak = 0U;
        s_motion_confirm_pending = 0U;
        s_stable_reject_streak = 0U;
        if (s_rearm_frames > 0U) return 0;
    }

    if (!raw_detected) {
        s_stable_reject_streak = 0U;
        s_stable_refresh_armed = 1U;
        if (s_motion_confirm_pending) {
            printf("TOFDEC,motion_drop,z=%u\r\n", (unsigned)s_motion_pending_zone);
            s_motion_confirm_pending = 0U;
        }
        return 0;
    }

    uint8_t best_z = 0U;
    uint32_t best_strength = 0U;
    if (res.affected_count > 0U) {
        best_z = res.affected_zones[0];
        best_strength = res.affected_drop[0];
        for (uint8_t i = 1U; i < res.affected_count; i++) {
            if (res.affected_drop[i] > best_strength) {
                best_strength = res.affected_drop[i];
                best_z = res.affected_zones[i];
            }
        }
    }

    uint32_t local_residual = 0U;
    uint8_t local_valid = 0U;
    int32_t candidate_signal_delta = 0;
    uint32_t local_signal_residual = 0U;
    uint8_t signal_local_valid = 0U;
    uint8_t neighbor_baseline_support = 0U;
    uint8_t neighbor_baseline_valid = 0U;
    uint8_t neighbor_fast_support = 0U;
    uint8_t neighbor_fast_valid = 0U;

    if (best_z < VL53L5CX_DET_NUM_ZONES && frame_valid[best_z]) {
        local_residual = FastAbsI32(frame_delta[best_z] - global_delta);
        local_valid = 1U;
    }
    if (best_z < VL53L5CX_DET_NUM_ZONES && signal_frame_valid[best_z]) {
        candidate_signal_delta = signal_delta_pct[best_z];
        local_signal_residual = FastAbsI32(signal_delta_pct[best_z] - global_signal_delta);
        signal_local_valid = 1U;
    }

    if (best_z < VL53L5CX_DET_NUM_ZONES) {
        int row = (int)best_z / 4;
        int col = (int)best_z % 4;
        for (int dr = -1; dr <= 1; dr++) {
            for (int dc = -1; dc <= 1; dc++) {
                if (dr == 0 && dc == 0) continue;
                int nr = row + dr;
                int nc = col + dc;
                if (nr < 0 || nr >= 4 || nc < 0 || nc >= 4) continue;

                uint8_t nz = (uint8_t)(nr * 4 + nc);
                if (current_signal_valid[nz]) {
                    uint32_t baseline_signal = 0U;
                    VL53L5CX_GetBaselineData(nz, &baseline_signal, NULL);
                    if (baseline_signal >= VL53L5CX_DET_MIN_SIGNAL) {
                        int64_t numerator = ((int64_t)current_signal[nz] -
                                             (int64_t)baseline_signal) * 100LL;
                        int32_t baseline_pct = (int32_t)(numerator /
                                                         (int64_t)baseline_signal);
                        neighbor_baseline_valid++;
                        if (FastAbsI32(baseline_pct) >= TOF_NEIGHBOR_SUPPORT_PCT)
                            neighbor_baseline_support++;
                    }
                }

                if (signal_frame_valid[nz]) {
                    neighbor_fast_valid++;
                    if (FastAbsI32(signal_delta_pct[nz] - global_signal_delta) >=
                        TOF_NEIGHBOR_SUPPORT_PCT)
                        neighbor_fast_support++;
                }
            }
        }
    }

    uint32_t max_recent_rfd = 0U;
    uint32_t max_recent_rfs = 0U;
    uint8_t affected_with_temporal_evidence = 0U;
    uint8_t affected_with_signal_edge = 0U;
    for (uint8_t i = 0U; i < res.affected_count; i++) {
        uint8_t z = res.affected_zones[i];
        uint32_t recent_rfd = FastHistoryMax(s_hist_dist_residual, s_hist_dist_valid, z);
        uint32_t recent_rfs = FastHistoryMax(s_hist_signal_residual, s_hist_signal_valid, z);
        if (recent_rfd > max_recent_rfd) max_recent_rfd = recent_rfd;
        if (recent_rfs > max_recent_rfs) max_recent_rfs = recent_rfs;
        if (recent_rfd >= TOF_LOCAL_EVID_DISTANCE_MM ||
            recent_rfs >= TOF_LOCAL_EVID_SIGNAL_PCT)
            affected_with_temporal_evidence++;
        if (recent_rfs >= TOF_LOCAL_EVID_SIGNAL_PCT)
            affected_with_signal_edge++;
    }

    uint32_t local_limit = (abs_global >= 3U) ? 1U : 0U;
    uint8_t vibration_veto = (uint8_t)(res.trigger_source == VL53L5CX_TRIG_SIGNAL &&
                                       res.affected_count == 1U &&
                                       common_motion &&
                                       s_common_motion_streak >= 2U &&
                                       local_valid &&
                                       local_residual <= local_limit &&
                                       max_recent_rfs <= 1U);

    uint8_t decision = TOF_DEC_FAIL_OPEN_ACCEPT;
    uint8_t accept = 1U;

    if (vibration_veto) {
        decision = TOF_DEC_VIBRATION_REJECT;
        accept = 0U;
        s_motion_confirm_pending = 0U;
    } else if (res.trigger_source == VL53L5CX_TRIG_BOTH) {
        /* One noisy motion zone combined with a persistent signal offset used
           to bypass every guard. Require a real local edge, or two consecutive
           BOTH frames in the same strongest zone, before photographing it. */
        if (affected_with_temporal_evidence > 0U) {
            decision = TOF_DEC_BOTH_ACCEPT;
            s_motion_confirm_pending = 0U;
        } else if (s_motion_confirm_pending && s_motion_pending_zone == best_z) {
            decision = TOF_DEC_MOTION_CONFIRMED;
            s_motion_confirm_pending = 0U;
        } else {
            decision = TOF_DEC_MOTION_PENDING;
            accept = 0U;
            s_motion_confirm_pending = 1U;
            s_motion_pending_zone = best_z;
        }
    } else if (res.trigger_source == VL53L5CX_TRIG_SIGNAL) {
        /* The captured idle logs show a persistent 7-11% baseline offset in
           one zone with no frame-to-frame distance/signal edge. Previously the
           SIGNAL branch accepted that first stable candidate unconditionally.
           Keep fast/strong events fail-open, but do not photograph a weak,
           stationary offset merely because it spans two adjacent zones. */
        if (affected_with_signal_edge == 0U &&
            best_strength < TOF_SIGNAL_STRONG_DROP_PCT) {
            decision = TOF_DEC_SIGNAL_STABLE_REJECT;
            accept = 0U;
        } else {
            decision = TOF_DEC_SIGNAL_ACCEPT;
        }
        s_motion_confirm_pending = 0U;
    } else if (res.trigger_source == VL53L5CX_TRIG_MOTION) {
        if (res.affected_count > 1U) {
            decision = TOF_DEC_MOTION_MULTI_ACCEPT;
            s_motion_confirm_pending = 0U;
        } else {
            uint8_t local_evidence = (uint8_t)(max_recent_rfd >= TOF_LOCAL_EVID_DISTANCE_MM ||
                                               max_recent_rfs >= TOF_LOCAL_EVID_SIGNAL_PCT);
            if (local_evidence) {
                decision = TOF_DEC_MOTION_LOCAL_ACCEPT;
                s_motion_confirm_pending = 0U;
            } else if (s_motion_confirm_pending && s_motion_pending_zone == best_z) {
                decision = TOF_DEC_MOTION_CONFIRMED;
                s_motion_confirm_pending = 0U;
            } else {
                decision = TOF_DEC_MOTION_PENDING;
                accept = 0U;
                s_motion_confirm_pending = 1U;
                s_motion_pending_zone = best_z;
            }
        }
    } else {
        s_motion_confirm_pending = 0U;
    }

    if (decision == TOF_DEC_SIGNAL_STABLE_REJECT) {
        if (s_stable_reject_streak < 255U) s_stable_reject_streak++;
        if (s_stable_reject_streak >= TOF_STABLE_REFRESH_FRAMES &&
            s_stable_refresh_armed && !s_baseline_refresh_requested) {
            s_baseline_refresh_requested = 1U;
            s_stable_refresh_armed = 0U;
            printf("TOFDEC,baseline_refresh_request,stable=%u\r\n",
                   (unsigned)s_stable_reject_streak);
        }
    } else {
        s_stable_reject_streak = 0U;
    }

    printf("FASTNOISE,Gfd=%ld,MADfd=%lu,cohFd=%u/%u,Rfd=%lu,Gfs=%ld,MADfs=%lu,Fsz=%ld,Rfs=%lu,validFs=%u,neighB=%u/%u,neighF=%u/%u,histRfd=%lu,histRfs=%lu,affEv=%u/%u,streak=%u,srej=%u,mPend=%u,dec=%u,veto=%u\r\n",
           (long)global_delta,
           (unsigned long)mad,
           (unsigned)coherent,
           (unsigned)valid_count,
           (unsigned long)local_residual,
           (long)global_signal_delta,
           (unsigned long)signal_mad,
           (long)candidate_signal_delta,
           (unsigned long)local_signal_residual,
           (unsigned)signal_valid_count,
           (unsigned)neighbor_baseline_support,
           (unsigned)neighbor_baseline_valid,
           (unsigned)neighbor_fast_support,
           (unsigned)neighbor_fast_valid,
           (unsigned long)max_recent_rfd,
           (unsigned long)max_recent_rfs,
           (unsigned)affected_with_temporal_evidence,
           (unsigned)res.affected_count,
           (unsigned)s_common_motion_streak,
           (unsigned)s_stable_reject_streak,
           (unsigned)s_motion_confirm_pending,
           (unsigned)decision,
           (unsigned)vibration_veto);

    (void)signal_local_valid;

    if (!accept) return 0;

    s_rearm_frames = TOF_REARM_FRAMES;
    return 1;
}

#else

void VL53L5CX_ResetDetectionFilterState(void)
{
}

uint32_t VL53L5CX_GetDetectionFilterGeneration(void)
{
    return 0U;
}

int VL53L5CX_TakeBaselineRefreshRequest(void)
{
    return 0;
}

int VL53L5CX_IsInsectDetectedFiltered(void)
{
    return VL53L5CX_IsInsectDetected();
}

#endif
