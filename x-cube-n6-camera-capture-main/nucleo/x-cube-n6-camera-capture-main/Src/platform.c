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

#define TOF_REARM_FRAMES                 10U
#define TOF_FAST_MIN_VALID_ZONES        12U
#define TOF_FAST_STRONG_SHIFT_MM         2U
#define TOF_FAST_MAX_MAD_MM              1U
#define TOF_FAST_MIN_COHERENCE_PCT      90U
#define TOF_NEIGHBOR_SUPPORT_PCT          1U

static uint16_t s_prev_distance[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_prev_valid[VL53L5CX_DET_NUM_ZONES] = {0};
static uint32_t s_prev_signal[VL53L5CX_DET_NUM_ZONES] = {0};
static uint8_t  s_prev_signal_valid[VL53L5CX_DET_NUM_ZONES] = {0};
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

    int32_t signal_delta_pct[VL53L5CX_DET_NUM_ZONES] = {0};
    uint8_t signal_frame_valid[VL53L5CX_DET_NUM_ZONES] = {0};
    int32_t valid_signal_delta[VL53L5CX_DET_NUM_ZONES];
    uint8_t signal_valid_count = 0;
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
        uint8_t current_valid = (uint8_t)(zone_valid && distance > 0U);
        uint8_t signal_valid = (uint8_t)(zone_valid &&
                                         signal >= VL53L5CX_DET_MIN_SIGNAL);

        current_signal[z] = signal;
        current_signal_valid[z] = signal_valid;

        if (current_valid && s_prev_valid[z]) {
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
        s_prev_valid[z] = current_valid;
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

    uint8_t coherent = 0;
    for (uint8_t z = 0; z < VL53L5CX_DET_NUM_ZONES; z++) {
        if (frame_valid[z] && FastAbsI32(frame_delta[z] - global_delta) <= coherence_tol)
            coherent++;
    }

    int32_t global_signal_delta = FastMedianI32(valid_signal_delta, signal_valid_count);
    uint32_t signal_deviations[VL53L5CX_DET_NUM_ZONES];
    for (uint8_t i = 0; i < signal_valid_count; i++)
        signal_deviations[i] = FastAbsI32(valid_signal_delta[i] - global_signal_delta);
    uint32_t signal_mad = FastMedianU32(signal_deviations, signal_valid_count);

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
        s_common_motion_streak = 0U;
        if (s_rearm_frames > 0U) return 0;
    }

    if (!raw_detected) return 0;

    uint32_t local_residual = 0;
    uint8_t local_valid = 0;
    int32_t candidate_signal_delta = 0;
    uint32_t local_signal_residual = 0;
    uint8_t signal_local_valid = 0;
    uint8_t neighbor_baseline_support = 0;
    uint8_t neighbor_baseline_valid = 0;
    uint8_t neighbor_fast_support = 0;
    uint8_t neighbor_fast_valid = 0;

    if (res.affected_count == 1U) {
        uint8_t z = res.affected_zones[0];
        if (z < VL53L5CX_DET_NUM_ZONES && frame_valid[z]) {
            local_residual = FastAbsI32(frame_delta[z] - global_delta);
            local_valid = 1U;
        }
        if (z < VL53L5CX_DET_NUM_ZONES && signal_frame_valid[z]) {
            candidate_signal_delta = signal_delta_pct[z];
            local_signal_residual = FastAbsI32(signal_delta_pct[z] - global_signal_delta);
            signal_local_valid = 1U;
        }

        if (z < VL53L5CX_DET_NUM_ZONES) {
            int row = (int)z / 4;
            int col = (int)z % 4;
            for (int dr = -1; dr <= 1; dr++) {
                for (int dc = -1; dc <= 1; dc++) {
                    if (dr == 0 && dc == 0) continue;
                    int nr = row + dr;
                    int nc = col + dc;
                    if (nr < 0 || nr >= 4 || nc < 0 || nc >= 4) continue;

                    uint8_t nz = (uint8_t)(nr * 4 + nc);
                    if (current_signal_valid[nz]) {
                        uint32_t baseline_signal = 0;
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
    }

    uint32_t local_limit = (abs_global >= 3U) ? 1U : 0U;
    uint8_t veto = (uint8_t)(res.trigger_source == VL53L5CX_TRIG_SIGNAL &&
                             res.affected_count == 1U &&
                             common_motion &&
                             s_common_motion_streak >= 2U &&
                             local_valid &&
                             local_residual <= local_limit);

    printf("FASTNOISE,Gfd=%ld,MADfd=%lu,cohFd=%u/%u,Rfd=%lu,Gfs=%ld,MADfs=%lu,Fsz=%ld,Rfs=%lu,validFs=%u,neighB=%u/%u,neighF=%u/%u,streak=%u,veto=%u\r\n",
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
           (unsigned)s_common_motion_streak,
           (unsigned)veto);

    (void)signal_local_valid;

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
