#include <assert.h>
#include <limits.h>
#include "perf_debug.h"

static uint32_t now;
uint32_t HAL_GetTick(void) { return now; }

int main(void)
{
    PerfTimer_t t;
    now = 0;
    Perf_Start(&t);
    Perf_Mark(&t, PERF_PHASE_CAM_INIT);
    now = 415; Perf_Mark(&t, PERF_PHASE_CAM_SNAP);
    now = 553; Perf_Mark(&t, PERF_PHASE_CAM_STOP);
    now = 580; Perf_Mark(&t, PERF_PHASE_CAM_DEINIT);
    Perf_Mark(&t, PERF_PHASE_CAM_END);
    now = 600;
    const uint32_t durations[] = {862, 951, 900, 903};
    for (unsigned f = 0; f < 4; ++f) {
        Perf_Mark(&t, PERF_PHASE_STORAGE);
        for (unsigned b = 0; b < 5; ++b)
            Perf_SD_RecordBatch(&t, 0, b == 4 ? (f < 2 ? 57 : 56) : 54,
                                0, b == 4 ? 825 : 1024);
        now += durations[f];
        Perf_StorageComplete(&t, durations[f], 2519424, 1);
    }
    now = 4839; Perf_Stop(&t);
    PerfTotals_t r = Perf_GetTotals(&t);
    assert(r.valid && r.total_ms == 4839 && r.camera_ms == 580);
    assert(r.storage_ms == 3616 && r.other_ms == 643);
#if PERF_TRACK_SD_WAIT_TIME
    assert(t.sd_total_write_ms == 1090 && r.storage_other_ms == 2526);
#else
    assert(t.sd_total_write_ms == 0 && r.storage_other_ms == 3616);
#endif
    assert(t.payload_bytes == 10077696 && t.sd_blocks_written == 19684);
    assert(t.storage_frames == 4 && t.sd_batch_count == 20);
    assert(t.phase_ticks[PERF_PHASE_STORAGE] == 600);
    assert(t.phase_count[PERF_PHASE_STORAGE] == 4);
    now += 2000; /* Later baseline refresh must not extend the report. */
    assert(Perf_TotalElapsed(&t) == 4839);
    Perf_PrintSummary(&t, 7);
    for (unsigned k = 0; k < PERF_STATS_WINDOW + 1U; k++) Perf_UpdateStats(&t);

    /* Latest board log: all groups and storage remainder must reconcile. */
    t.phase_ticks[PERF_PHASE_CAM_EXPO] = 41; t.phase_hit[PERF_PHASE_CAM_EXPO] = 1;
    t.phase_ticks[PERF_PHASE_CAM_WARMUP] = 41; t.phase_hit[PERF_PHASE_CAM_WARMUP] = 1;
    t.phase_ticks[PERF_PHASE_CAM_SNAP] = 414;
    t.phase_ticks[PERF_PHASE_CAM_STOP] = 552;
    t.phase_ticks[PERF_PHASE_CAM_DEINIT] = 579;
    t.phase_ticks[PERF_PHASE_CAM_END] = 634;
    t.phase_ticks[PERF_PHASE_STORAGE] = 646;
    t.phase_ticks[PERF_PHASE_DONE] = 4643;
    t.storage_wall_ms = 3997;
#if PERF_TRACK_SD_WAIT_TIME
    t.sd_total_write_ms = 893;
#endif
    r = Perf_GetTotals(&t);
    assert(r.valid && r.camera_ms == 634 && r.storage_ms == 3997 && r.other_ms == 12);
#if PERF_TRACK_SD_WAIT_TIME
    assert(r.storage_other_ms == 3104);
#endif
    Perf_PrintSummary(&t, 9);

    /* A retry contributes HAL traffic twice, but saved payload only once. */
    now = 0; Perf_Start(&t); Perf_Mark(&t, PERF_PHASE_CAM_INIT);
    now = 10; Perf_Mark(&t, PERF_PHASE_CAM_END); Perf_Mark(&t, PERF_PHASE_STORAGE);
    Perf_SD_RecordBatch(&t, 1, 5, 0, 1024);
    Perf_SD_RecordBatch(&t, 1, 5, 0, 0); /* failed HAL call */
    Perf_SD_RecordBatch(&t, 1, 5, 0, 1024); /* retry */
    Perf_StorageComplete(&t, 30, 512000, 1);
    Perf_StorageComplete(&t, 5, 512000, 0);
    now = 50; Perf_Stop(&t);
    r = Perf_GetTotals(&t);
    assert(r.valid && r.storage_ms == 35 && r.other_ms == 5);
    assert(t.payload_bytes == 512000 && t.sd_blocks_written == 2048);
    assert(t.storage_frames == 1 && t.storage_failures == 1);

    /* Wraparound: elapsed intervals remain small and positive. */
    now = UINT32_MAX - 9; Perf_Start(&t); Perf_Mark(&t, PERF_PHASE_CAM_INIT);
    now = 5; Perf_Mark(&t, PERF_PHASE_CAM_END); Perf_Mark(&t, PERF_PHASE_STORAGE);
    Perf_StorageComplete(&t, 10, 100, 1);
    now = 20; Perf_Stop(&t);
    r = Perf_GetTotals(&t);
    assert(r.valid && r.total_ms == 30 && r.camera_ms == 15 && r.other_ms == 5);

    /* Do not hide impossible accounting by clamping residual to zero. */
    t.storage_wall_ms = 100;
    assert(!Perf_GetTotals(&t).valid);
    t.storage_wall_ms = 10; t.sd_total_write_ms = 11;
    assert(!Perf_GetTotals(&t).valid);
    t.sd_total_write_ms = 0; t.phase_hit[PERF_PHASE_CAM_END] = 0;
    assert(!Perf_GetTotals(&t).valid);

    now = 0; Perf_Start(&t); Perf_Mark(&t, PERF_PHASE_CAM_INIT);
    Perf_Mark(&t, PERF_PHASE_CAM_END); Perf_Stop(&t);
    assert(Perf_GetTotals(&t).valid);
    Perf_PrintSummary(&t, 8); /* No nan/inf from empty or sub-ms measurements. */
    puts("PASS: timing accounting, batch payload, retries, rollover, invalid/zero intervals");
    return 0;
}
