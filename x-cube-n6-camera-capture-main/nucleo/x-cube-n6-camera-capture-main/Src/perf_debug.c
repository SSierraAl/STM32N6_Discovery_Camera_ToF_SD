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
 *   All output is gated by PERF_DEBUG_LEVEL in app_config.h:
 *     Level 0: Only total time (1 line)
 *     Level 1: Phase breakdown table
 *     Level 2: + SD sub-analysis + bottleneck label
 *     Level 3: + raw counters + per-batch history
 * ******************************************************************************
 */

#include "perf_debug.h"
#include <string.h>
#include <math.h>
#include "main.h"

extern SD_HandleTypeDef hsd1;

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
    [PERF_PHASE_DONE]        = "DONE"
};

/* ================================================================
   Running Statistics Buffer
   ================================================================ */

#if PERF_STATS_WINDOW > 0
PerfStats_t g_perf_stats = {0};
#else
PerfStats_t g_perf_stats = {0}; /* dummy — stats are disabled */
#endif

/* ================================================================
   Helper: Compute camera total and SD total from phase ticks
   ================================================================ */

static void Perf_SplitTotals(const PerfTimer_t *t,
                              uint32_t *cam_ms, uint32_t *sd_ms, uint32_t *other_ms)
{
    *cam_ms = 0; *sd_ms = 0; *other_ms = 0;

    /* Camera phases: INIT -> DEINIT (or COPY/RESTART in continuous mode) */
    if (t->phase_hit[PERF_PHASE_CAM_INIT]) {
        uint32_t end_phase = PERF_PHASE_CAM_DEINIT;
        if (t->phase_hit[PERF_PHASE_CAM_RESTART])
            end_phase = PERF_PHASE_CAM_RESTART;
        else if (t->phase_hit[PERF_PHASE_CAM_SNAP])
            end_phase = PERF_PHASE_CAM_SNAP;

        if (t->phase_hit[end_phase])
            *cam_ms = t->phase_ticks[end_phase] - t->phase_ticks[PERF_PHASE_CAM_INIT];
    }

    /* SD phases: use accumulated counters if available */
#if PERF_TRACK_SD_WAIT_TIME
    if (t->sd_batch_count > 0) {
        *sd_ms = t->sd_total_wait_ms + t->sd_total_write_ms + t->sd_total_gap_ms;
    }
#endif

    /* Other = total - cam - sd */
    uint32_t total = Perf_TotalElapsed(t);
    if (total >= *cam_ms + *sd_ms)
        *other_ms = total - *cam_ms - *sd_ms;
}

/* ================================================================
   Helper: Identify the main bottleneck
   ================================================================ */

typedef enum {
    BOTTLENECK_CAMERA_INIT,
    BOTTLENECK_CAMERA_WARMUP,
    BOTTLENECK_SD_WAIT_READY,
    BOTTLENECK_SD_DMA_WRITE,
    BOTTLENECK_SD_GAP_DELAY,
    BOTTLENECK_BALANCED,
    BOTTLENECK_UNKNOWN
} Bottleneck_t;

static const char* bottleneck_names[] = {
    [BOTTLENECK_CAMERA_INIT]     = "CAMERA INIT/CONFIG",
    [BOTTLENECK_CAMERA_WARMUP]   = "CAMERA WARMUP FRAMES",
    [BOTTLENECK_SD_WAIT_READY]   = "SD CARD WAIT-READY",
    [BOTTLENECK_SD_DMA_WRITE]    = "SD DMA TRANSFER",
    [BOTTLENECK_SD_GAP_DELAY]    = "SD INTER-BATCH GAP",
    [BOTTLENECK_BALANCED]        = "BALANCED (no single bottleneck)",
    [BOTTLENECK_UNKNOWN]         = "UNKNOWN"
};

static Bottleneck_t Perf_FindBottleneck(const PerfTimer_t *t, uint32_t total_ms)
{
    if (total_ms == 0) return BOTTLENECK_UNKNOWN;

    uint32_t cam_init_ms  = Perf_PhaseElapsed(t, PERF_PHASE_CAM_INIT, PERF_PHASE_CAM_EXPO);
    uint32_t cam_warmup_ms = Perf_PhaseElapsed(t, PERF_PHASE_CAM_WARMUP, PERF_PHASE_CAM_SNAP);
    float pct;

    /* Check SD wait time (largest contributor) */
#if PERF_TRACK_SD_WAIT_TIME
    if (t->sd_total_wait_ms > 0) {
        pct = (100.0f * t->sd_total_wait_ms) / total_ms;
        if (pct > 40) return BOTTLENECK_SD_WAIT_READY;
    }

    if (t->sd_total_gap_ms > 0) {
        pct = (100.0f * t->sd_total_gap_ms) / total_ms;
        if (pct > 40) return BOTTLENECK_SD_GAP_DELAY;
    }

    if (t->sd_total_write_ms > 0) {
        pct = (100.0f * t->sd_total_write_ms) / total_ms;
        if (pct > 40) return BOTTLENECK_SD_DMA_WRITE;
    }
#endif

    /* Check camera warmup */
    pct = (100.0f * cam_warmup_ms) / total_ms;
    if (pct > 30) return BOTTLENECK_CAMERA_WARMUP;

    /* Check camera init */
    pct = (100.0f * cam_init_ms) / total_ms;
    if (pct > 30) return BOTTLENECK_CAMERA_INIT;

    return BOTTLENECK_BALANCED;
}

/* ================================================================
   Main Summary Printer
   ================================================================ */

uint32_t Perf_PrintSummary(PerfTimer_t *t, uint32_t snap_id)
{
    uint32_t total_ms = Perf_TotalElapsed(t);
    if (total_ms == 0) return 0;

#if PERF_DEBUG_LEVEL == 0
    /* Production: one line only */
    printf("[PERF] Snap #%lu: %lu ms total\n", (unsigned long)snap_id, (unsigned long)total_ms);
    return total_ms;
#endif

    uint32_t cam_ms, sd_ms, other_ms;
    Perf_SplitTotals(t, &cam_ms, &sd_ms, &other_ms);

    /* ---- Header ---- */
    printf("\n"
           "╔══════════════════════════════════════════════════════════╗\n");
    printf("║           PERFECT CAPTURE #%" PRIu32 " — TIMING REPORT             ║\n", snap_id);
    printf("╠══════════════════════════════════════════════════════════╣\n");

#if PERF_DEBUG_LEVEL >= 1
    /* ---- Phase Breakdown Table ---- */
    printf("║  PHASE BREAKDOWN                                         ║\n");
    printf("╠──────────────────────────────────────────────────────────╣\n");

    /* Build list of relevant phase transitions */
    typedef struct { PerfPhase_t from; PerfPhase_t to; const char* label; } PhaseDelta_t;

    const PhaseDelta_t deltas[] = {
        {PERF_PHASE_START,    PERF_PHASE_CAM_INIT,   "Boot → Camera init"},
        {PERF_PHASE_CAM_INIT, PERF_PHASE_CAM_EXPO,   "  Sensor + DCMIPP"},
        {PERF_PHASE_CAM_EXPO, PERF_PHASE_CAM_WARMUP, "  Exposure/Gain cfg"},
        {PERF_PHASE_CAM_WARMUP,PERF_PHASE_CAM_SNAP,  "  Warmup frames"},
        {PERF_PHASE_CAM_SNAP, PERF_PHASE_CAM_STOP,   "  Final frame grab"},
        {PERF_PHASE_CAM_STOP, PERF_PHASE_CAM_DEINIT, "  Pipe stop"},
        {PERF_PHASE_CAM_DEINIT,PERF_PHASE_SD_WRITE,  "  Camera → Storage"},
    };
    const int delta_count = sizeof(deltas) / sizeof(deltas[0]);

    for (int i = 0; i < delta_count; i++) {
        uint32_t elapsed = Perf_PhaseElapsed(t, deltas[i].from, deltas[i].to);
        if (elapsed == 0 && !(t->phase_hit[deltas[i].from] && t->phase_hit[deltas[i].to]))
            continue;
        float pct = total_ms > 0 ? (100.0f * elapsed) / total_ms : 0;
        printf("║  %-38s %4" PRIu32 "ms  %5.1f%%  ║\n",
               deltas[i].label, elapsed, pct);
    }

    /* Continuous-mode extra phases */
    if (t->phase_hit[PERF_PHASE_CAM_COPY]) {
        uint32_t copy_ms = Perf_PhaseElapsed(t, PERF_PHASE_CAM_STOP, PERF_PHASE_CAM_COPY);
        float pct = (100.0f * copy_ms) / total_ms;
        printf("║  %-38s %4" PRIu32 "ms  %5.1f%%  ║\n", "  memcpy (stop→copy)", copy_ms, pct);
    }
    if (t->phase_hit[PERF_PHASE_CAM_RESTART]) {
        uint32_t restart_ms = Perf_PhaseElapsed(t, PERF_PHASE_CAM_COPY, PERF_PHASE_CAM_RESTART);
        float pct = (100.0f * restart_ms) / total_ms;
        printf("║  %-38s %4" PRIu32 "ms  %5.1f%%  ║\n", "  Pipe restart", restart_ms, pct);
    }

    printf("╠──────────────────────────────────────────────────────────╣\n");

    /* ---- Group Totals ---- */
    float cam_pct  = total_ms > 0 ? (100.0f * cam_ms)  / total_ms : 0;
    float sd_pct   = total_ms > 0 ? (100.0f * sd_ms)   / total_ms : 0;
    float other_pct = total_ms > 0 ? (100.0f * other_ms) / total_ms : 0;

    printf("║  GROUP TOTALS                                            ║\n");
    printf("║  Camera (init+capture):    %6" PRIu32 "ms  %5.1f%%  ║\n", cam_ms, cam_pct);
    printf("║  Storage (SD card):        %6" PRIu32 "ms  %5.1f%%  ║\n", sd_ms, sd_pct);
    printf("║  Other (IPC, overhead):    %6" PRIu32 "ms  %5.1f%%  ║\n", other_ms, other_pct);
    printf("╠──────────────────────────────────────────────────────────╣\n");
#endif

#if PERF_DEBUG_LEVEL >= 2
    /* ---- SD Card Sub-Analysis ---- */
#if PERF_TRACK_SD_WAIT_TIME
    if (t->sd_batch_count > 0) {
        float throughput = (SNAP_FRAME_SIZE / 1048576.0f) / (t->sd_total_write_ms / 1000.0f);
        float effective_throughput = (SNAP_FRAME_SIZE / 1048576.0f) / (sd_ms / 1000.0f);

        printf("║  SD CARD DETAIL                                          ║\n");
        printf("║    Batches written:       %4" PRIu32 "                     ║\n", t->sd_batch_count);
        printf("║    Blocks/batch:          %4" PRIu32 "  (%" PRIu32 " KB)         ║\n",
               SD_BATCH_WRITE_BLOCKS,
               SD_BATCH_WRITE_BLOCKS * (SD_BLOCK_SIZE / 1024));
        printf("║    ┌───────────────────────────────────────────────────┐ ║\n");
        printf("║    │ Wait (card ready):  %6" PRIu32 "ms  (%5.1f%%)  │ ║\n",
               t->sd_total_wait_ms,
               sd_ms > 0 ? (100.0f * t->sd_total_wait_ms) / sd_ms : 0);
        printf("║    │ DMA transfer:       %6" PRIu32 "ms  (%5.1f%%)  │ ║\n",
               t->sd_total_write_ms,
               sd_ms > 0 ? (100.0f * t->sd_total_write_ms) / sd_ms : 0);
        printf("║    │ Gap (inter-batch):  %6" PRIu32 "ms  (%5.1f%%)  │ ║\n",
               t->sd_total_gap_ms,
               sd_ms > 0 ? (100.0f * t->sd_total_gap_ms) / sd_ms : 0);
        printf("║    └───────────────────────────────────────────────────┘ ║\n");
        printf("║    Peak single batch:    %4" PRIu32 "ms                    ║\n", t->sd_max_batch_ms);
        printf("║    Peak card-ready wait: %4" PRIu32 "ms                    ║\n", t->sd_max_wait_ms);
        printf("║    DMA throughput:       %.2f MB/s                       ║\n", throughput);
        printf("║    Effective throughput: %.2f MB/s (incl wait+gap)       ║\n", effective_throughput);
        printf("╠──────────────────────────────────────────────────────────╣\n");
    }
#endif

    /* ---- Bottleneck Identification ---- */
    Bottleneck_t bn = Perf_FindBottleneck(t, total_ms);
    const char* bn_name = bottleneck_names[bn];

    printf("║  BOTTLENECK ANALYSIS                                     ║\n");
    printf("║  ⚡ Main bottleneck: %-34s║\n", bn_name);

    /* Actionable suggestions */
    printf("║  Suggestions:                                            ║\n");
    switch (bn) {
        case BOTTLENECK_SD_WAIT_READY:
            printf("║    • Reduce SD_BATCH_WRITE_BLOCKS (32 instead of 64)    ║\n");
            printf("║    • Try higher SD clock divisor                        ║\n");
            printf("║    • Test with a faster SD card (U3/V30 rated)          ║\n");
            printf("║    • Add CMD23 (SET_BLOCK_COUNT) for write prefetching  ║\n");
            break;
        case BOTTLENECK_SD_GAP_DELAY:
            printf("║    • Reduce 20ms vTaskDelay between batches to 5ms      ║\n");
            printf("║    • Use SD card write prefetching if supported         ║\n");
            printf("║    • Pipeline: start next batch fill while card ready   ║\n");
            break;
        case BOTTLENECK_SD_DMA_WRITE:
            printf("║    • DMA throughput limited by SDMMC clock + card speed ║\n");
            printf("║    • Try larger batches (but risk CRC errors)           ║\n");
            printf("║    • Check SD clock: current div=%" PRIu32 "                 ║\n",
                   (uint32_t)hsd1.Init.ClockDiv);
            break;
        case BOTTLENECK_CAMERA_WARMUP:
            printf("║    • Reduce SNAP_WARMUP_FRAMES (current: %d)            ║\n", SNAP_WARMUP_FRAMES);
            printf("║    • Switch to CONTINUOUS mode (CAPTURE_MODE=1)         ║\n");
            printf("║    • Each frame = ~%dms at %d FPS                       ║\n",
                   1000 / SNAP_FPS, SNAP_FPS);
            break;
        case BOTTLENECK_CAMERA_INIT:
            printf("║    • Camera init is I2C-bound (sensor config)           ║\n");
            printf("║    • Switch to CONTINUOUS mode to init once at boot     ║\n");
            printf("║    • Cache sensor config to skip redundant I2C writes   ║\n");
            break;
        default:
            printf("║    • No single dominant bottleneck                      ║\n");
            printf("║    • Consider CONTINUOUS mode for faster total cycle    ║\n");
            break;
    }
    printf("╠──────────────────────────────────────────────────────────╣\n");
#endif

    /* ---- Footer ---- */
    float img_throughput = (SNAP_FRAME_SIZE / 1048576.0f) / (total_ms / 1000.0f);
    printf("║  TOTAL: %6" PRIu32 "ms | Image: %.3f MB | Throughput: %.2f MB/s ║\n",
           total_ms, SNAP_FRAME_SIZE / 1048576.0f, img_throughput);
    printf("╚══════════════════════════════════════════════════════════╝\n");

    return total_ms;
}

/* ================================================================
   Running Statistics
   ================================================================ */

void Perf_UpdateStats(PerfTimer_t *t)
{
#if PERF_STATS_WINDOW > 0
    uint32_t total_ms = Perf_TotalElapsed(t);
    if (total_ms == 0) return;

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

extern SD_HandleTypeDef hsd1;
