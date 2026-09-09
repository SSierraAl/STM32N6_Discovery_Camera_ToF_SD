/**
 * ******************************************************************************
 * @file    perf_debug.c
 * @brief   Performance Debug Framework — Summary + Statistics Implementation
 *
 *   This file implements the structured output for capture timing analysis.
 *   The key idea: after each snapshot, print a clean report showing:
 *     1. Phase-by-phase timing breakdown with percentages
 *     2. SD card sub-analysis (wait vs write vs gap time)
 *     3. Bottleneck identification (what's consuming the most time)
 *     4. Running statistics (average over last N captures)
 *
 *   PERF_PRINT_SUMMARY gates reports; PERF_REPORT_DETAIL selects compact,
 *   phases or detailed tables, independently of application log verbosity.
 *   PERF_PRINT_STATS separately enables running statistics.
 * ******************************************************************************
 */

#include "perf_debug.h"
#include <string.h>
#include <math.h>



/* ================================================================
   Phase Name Strings
   ================================================================ */

const char* perf_phase_names[PERF_PHASE_COUNT] = {
    [PERF_PHASE_START]       = "START",
    [PERF_PHASE_CAM_INIT]    = "CAM_INIT",
    [PERF_PHASE_CAM_EXPO]    = "CAM_EXPOSURE",
    [PERF_PHASE_CAM_WARMUP]  = "CAM_WARMUP",
    [PERF_PHASE_CAM_SNAP]    = "CAM_SNAP",
    [PERF_PHASE_CAM_STOP]    = "CAM_STOP",
    [PERF_PHASE_CAM_DEINIT]  = "CAM_DEINIT",
    [PERF_PHASE_CAM_COPY]    = "CAM_COPY",
    [PERF_PHASE_CAM_RESTART] = "CAM_RESTART",
    [PERF_PHASE_CACHE_CLEAN] = "CACHE",
    [PERF_PHASE_SD_READY]    = "SD_READY",
    [PERF_PHASE_SD_WRITE]    = "SD_WRITE",
    [PERF_PHASE_SD_GAP]      = "SD_GAP",
    [PERF_PHASE_STORAGE]     = "STORAGE",
    [PERF_PHASE_DONE]        = "DONE",
    [PERF_PHASE_CAM_END]     = "CAM_END"
};

/* ================================================================
   Running Statistics Buffer
   ================================================================ */

#if PERF_STATS_WINDOW > 0 && PERF_PRINT_STATS && PERF_PRINT_SUMMARY
PerfStats_t g_perf_stats = {0};
#else
PerfStats_t g_perf_stats = {0}; /* dummy — stats are disabled */
#endif

/* ================================================================
   Helper: Compute camera total and SD total from phase ticks
   ================================================================ */

/* All intervals use uint32_t subtraction so a single tick rollover is safe.
   This report describes the serialized request -> camera -> storage path. */
PerfTotals_t Perf_GetTotals(const PerfTimer_t *t)
{
    PerfTotals_t r = {0};
    r.total_ms = Perf_TotalElapsed(t);
    r.valid = t->phase_hit[PERF_PHASE_START] && t->phase_hit[PERF_PHASE_DONE] &&
              t->phase_hit[PERF_PHASE_CAM_INIT] && t->phase_hit[PERF_PHASE_CAM_END];
    r.camera_ms = Perf_PhaseElapsed(t, PERF_PHASE_CAM_INIT, PERF_PHASE_CAM_END);
    r.storage_ms = t->storage_wall_ms;
    uint64_t accounted = (uint64_t)r.camera_ms + r.storage_ms;
    uint64_t detail = (uint64_t)t->sd_total_wait_ms + t->sd_total_write_ms + t->sd_total_gap_ms;
    if (accounted > r.total_ms || detail + t->sd_checksum_ms > r.storage_ms) r.valid = 0;
    if (accounted <= r.total_ms) r.other_ms = r.total_ms - (uint32_t)accounted;
    if (detail <= r.storage_ms) {
        r.sd_detail_ms = (uint32_t)detail;
        r.storage_other_ms = r.storage_ms - (uint32_t)detail;
    }
    uint32_t camera_start = t->phase_ticks[PERF_PHASE_CAM_INIT] - t->start_tick;
    uint32_t camera_end = t->phase_ticks[PERF_PHASE_CAM_END] - t->start_tick;
    if (camera_start > camera_end || camera_end > r.total_ms) r.valid = 0;
    if (t->phase_hit[PERF_PHASE_STORAGE]) {
        uint32_t storage_start = t->phase_ticks[PERF_PHASE_STORAGE] - t->start_tick;
        if (storage_start < camera_end || storage_start > r.total_ms ||
            r.storage_ms > r.total_ms - storage_start) r.valid = 0;
    } else if (r.storage_ms || t->storage_frames || t->storage_failures) r.valid = 0;
    return r;
}

#if PERF_STATS_WINDOW > 0 && PERF_PRINT_STATS && PERF_PRINT_SUMMARY
static void Perf_SplitTotals(const PerfTimer_t *t, uint32_t *cam, uint32_t *sd, uint32_t *other)
{
    PerfTotals_t r = Perf_GetTotals(t);
    *cam = r.camera_ms; *sd = r.storage_ms; *other = r.other_ms;
}

#endif

#if PERF_PRINT_SUMMARY && PERF_REPORT_DETAIL > 0
static const char *table_border = "+------------------------------------------+--------------------------+\n";
static void Perf_Row(const char *label, const char *value)
{
    printf("| %-40.40s | %24.24s |\n", label, value);
}
static void Perf_TimeRow(const char *label, uint32_t ms, uint32_t total)
{
    char value[48];
    snprintf(value, sizeof(value), "%lu ms / %.1f%%", (unsigned long)ms,
             total ? 100.0 * ms / total : 0.0);
    Perf_Row(label, value);
}
#if PERF_REPORT_DETAIL >= 2
static void Perf_CountRow(const char *label, uint32_t value)
{
    char text[32];
    snprintf(text, sizeof(text), "%lu", (unsigned long)value);
    Perf_Row(label, text);
}
#endif
static void Perf_RateRow(const char *label, uint64_t bytes, uint32_t ms)
{
    char value[48];
    if (ms) snprintf(value, sizeof(value), "%.2f MiB/s", (double)bytes * 1000.0 / 1048576.0 / ms);
    else snprintf(value, sizeof(value), "N/A (0 ms)");
    Perf_Row(label, value);
}
static void Perf_PhaseRow(const PerfTimer_t *t, PerfPhase_t from, PerfPhase_t to,
                          const char *label, uint32_t total)
{
    if (t->phase_hit[from] && t->phase_hit[to]) {
        uint32_t start = t->phase_ticks[from] - t->start_tick;
        uint32_t end = t->phase_ticks[to] - t->start_tick;
        if (end >= start) Perf_TimeRow(label, end - start, total);
    }
}
#endif

uint32_t Perf_PrintSummary(PerfTimer_t *t, uint32_t snap_id)
{
#if !PERF_PRINT_SUMMARY
    (void)snap_id;
    return Perf_TotalElapsed(t);
#else
    PerfTotals_t r = Perf_GetTotals(t);
#if PERF_REPORT_DETAIL == 0
    printf("[PERF] #%lu mode=%d saved=%lu failed=%lu total=%lu ms accounting=%s\n",
           (unsigned long)snap_id, CAPTURE_MODE, (unsigned long)t->storage_frames,
           (unsigned long)t->storage_failures, (unsigned long)r.total_ms, r.valid ? "OK" : "INVALID");
#else
    char value[48];
    printf("\n%s", table_border);
    snprintf(value, sizeof(value), "#%lu / mode %d", (unsigned long)snap_id, CAPTURE_MODE);
    Perf_Row("CAPTURE TIMING", value);
    snprintf(value, sizeof(value), "%lu saved / %lu failed", (unsigned long)t->storage_frames,
             (unsigned long)t->storage_failures);
    Perf_Row("Images", value);
    Perf_Row("Camera view", CAM_BINNING ? "FULL FIELD / HALF SIZE" : "FULL FIELD / FULL SIZE");
    printf("%s", table_border);
    Perf_Row("CAMERA PHASES (included in camera)", "ms / % of cycle");
    Perf_PhaseRow(t, PERF_PHASE_CAM_INIT, PERF_PHASE_CAM_EXPO, "Sensor init/wake + pipe start", r.total_ms);
    Perf_PhaseRow(t, PERF_PHASE_CAM_EXPO, PERF_PHASE_CAM_WARMUP, "Exposure/gain configuration", r.total_ms);
    Perf_PhaseRow(t, PERF_PHASE_CAM_WARMUP, PERF_PHASE_CAM_SNAP, "Warmup", r.total_ms);
    Perf_PhaseRow(t, PERF_PHASE_CAM_SNAP, PERF_PHASE_CAM_STOP, "Frame acquisition", r.total_ms);
    Perf_PhaseRow(t, PERF_PHASE_CAM_STOP, PERF_PHASE_CAM_DEINIT, "Pipe stop/standby", r.total_ms);
    Perf_PhaseRow(t, PERF_PHASE_CAM_DEINIT, PERF_PHASE_CAM_END, "Camera tail/diagnostics", r.total_ms);
    printf("%s", table_border);
    Perf_Row("GROUP TOTALS (add to cycle total)", "ms / % of cycle");
    Perf_TimeRow("Camera", r.camera_ms, r.total_ms);
    Perf_TimeRow("Storage wall (all images)", r.storage_ms, r.total_ms);
    Perf_TimeRow("Other / IPC", r.other_ms, r.total_ms);
    Perf_TimeRow("TOTAL CYCLE", r.total_ms, r.total_ms);
    Perf_Row("Accounting", r.valid ? "OK" : "INVALID");
    printf("%s", table_border);
#if PERF_REPORT_DETAIL >= 2
    Perf_Row("SD DETAIL (included in storage wall)", "ms / % of storage");
#if PERF_TRACK_SD_WAIT_TIME
    Perf_TimeRow("Recorded ready waits", t->sd_total_wait_ms, r.storage_ms);
    Perf_TimeRow("Blocking HAL writes (not DMA)", t->sd_total_write_ms, r.storage_ms);
    Perf_TimeRow("Inter-batch gaps", t->sd_total_gap_ms, r.storage_ms);
    Perf_TimeRow("Remaining storage work", r.storage_other_ms, r.storage_ms);
    Perf_TimeRow("  Of remaining: checksum", t->sd_checksum_ms, r.storage_ms);
#else
    Perf_Row("SD subphase tracking", "DISABLED");
#endif
    Perf_CountRow("HAL write calls", t->sd_batch_count);
    Perf_CountRow("Successfully written blocks", t->sd_blocks_written);
    snprintf(value, sizeof(value), "%lu blocks / %lu KiB", (unsigned long)SD_BATCH_WRITE_BLOCKS,
             (unsigned long)(((uint64_t)SD_BATCH_WRITE_BLOCKS * SD_BLOCK_SIZE) / 1024U));
    Perf_Row("Maximum per HAL call", value);
#if PERF_TRACK_SD_WAIT_TIME
    snprintf(value, sizeof(value), "%lu ms", (unsigned long)t->sd_max_batch_ms);
    Perf_Row("Longest HAL write", value);
    snprintf(value, sizeof(value), "%lu ms", (unsigned long)t->sd_max_wait_ms);
    Perf_Row("Longest recorded ready wait", value);
    if (r.valid) Perf_RateRow("HAL traffic rate (incl. retry blocks)",
                             (uint64_t)t->sd_blocks_written * SD_BLOCK_SIZE, t->sd_total_write_ms);
#endif
    printf("%s", table_border);
#endif
    snprintf(value, sizeof(value), "%.3f MiB", (double)t->payload_bytes / 1048576.0);
    Perf_Row("Saved image payload", value);
    if (r.valid) {
        Perf_RateRow("Storage payload rate", t->payload_bytes, r.storage_ms);
        Perf_RateRow("Cycle payload rate", t->payload_bytes, r.total_ms);
    }
    const char *largest = "N/A";
    if (!r.valid) largest = "UNKNOWN";
    else if (r.total_ms) {
        if (r.other_ms >= r.camera_ms && r.other_ms >= r.storage_ms) largest = "OTHER / IPC";
        else largest = r.storage_ms >= r.camera_ms ? "STORAGE WALL" : "CAMERA";
    }
    Perf_Row("Largest group", largest);
    printf("%s", table_border);
    printf("Scope: request -> all storage completions. Later ToF refresh and this table excluded.\n");
#if PERF_REPORT_DETAIL >= 2 && PERF_TRACK_SD_WAIT_TIME
    printf("Storage remainder: checksum/copy/cache/logging/recovery/untracked waits.\n");
#endif
    if (!r.valid) printf("[PERF] Missing/overlapping markers or counters outside cycle; do not use rates.\n");
#endif
    return r.total_ms;
#endif
}

void Perf_UpdateStats(PerfTimer_t *t)
{
    (void)t;
#if PERF_STATS_WINDOW > 0 && PERF_PRINT_STATS && PERF_PRINT_SUMMARY
    uint32_t total_ms = Perf_TotalElapsed(t);
    if (total_ms == 0 || !Perf_GetTotals(t).valid) return;

    uint32_t cam_ms = 0, sd_ms = 0, other_ms = 0;
    Perf_SplitTotals(t, &cam_ms, &sd_ms, &other_ms);

    uint32_t idx = g_perf_stats.index % PERF_STATS_WINDOW;
    g_perf_stats.total_times[idx] = total_ms;
    g_perf_stats.cam_times[idx]   = cam_ms;
    g_perf_stats.sd_times[idx]    = sd_ms;
    if (g_perf_stats.count < PERF_STATS_WINDOW)
        g_perf_stats.count++;
    g_perf_stats.index++;

    /* Print running average when window is full */
    if (g_perf_stats.count >= PERF_STATS_WINDOW) {
        uint32_t sum_total = 0, sum_cam = 0, sum_sd = 0;
        for (uint32_t i = 0; i < PERF_STATS_WINDOW; i++) {
            sum_total += g_perf_stats.total_times[i];
            sum_cam   += g_perf_stats.cam_times[i];
            sum_sd    += g_perf_stats.sd_times[i];
        }
        uint32_t avg_total = sum_total / PERF_STATS_WINDOW;
        uint32_t avg_cam   = sum_cam   / PERF_STATS_WINDOW;
        uint32_t avg_sd    = sum_sd    / PERF_STATS_WINDOW;

        /* Find min/max */
        uint32_t min_total = g_perf_stats.total_times[0];
        uint32_t max_total = g_perf_stats.total_times[0];
        for (uint32_t i = 1; i < PERF_STATS_WINDOW; i++) {
            if (g_perf_stats.total_times[i] < min_total)
                min_total = g_perf_stats.total_times[i];
            if (g_perf_stats.total_times[i] > max_total)
                max_total = g_perf_stats.total_times[i];
        }

        printf("\n[STATS] Last %d captures — Avg=%lums  Min=%lums  Max=%lums\n",
               PERF_STATS_WINDOW,
               (unsigned long)avg_total,
               (unsigned long)min_total,
               (unsigned long)max_total);
        printf("[STATS]   Camera: Avg=%lums (%.1f%%)  |  SD: Avg=%lums (%.1f%%)\n",
               (unsigned long)avg_cam,
               avg_total > 0 ? (100.0f * avg_cam) / avg_total : 0,
               (unsigned long)avg_sd,
               avg_total > 0 ? (100.0f * avg_sd) / avg_total : 0);
    }
#endif
}

/* ================================================================
   External reference to SD handle (for clock div display)
   ================================================================ */

/*
 * CubeIDE build bridge for the raw-SD append journal.
 *
 * This project links source files individually. Src/sd_storage_journal.c is
 * present in the repository but is not currently a linkedResource in the
 * CubeIDE .project, so its SDJ_* symbols would otherwise be missing at link
 * time. perf_debug.c is linked in every current build configuration and does
 * not include raw_image.h, making it a safe build-only host for the journal.
 *
 * Keep this include at the very end. TEST_TOF_MODE / ZFRAME behavior is not
 * changed; in test mode the sensor task still skips camera and SD at runtime.
 */
#include "sd_storage_journal.c"
