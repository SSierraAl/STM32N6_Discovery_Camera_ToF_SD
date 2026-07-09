/**
 * ******************************************************************************
 * @file    perf_debug.h
 * @brief   Performance Debug Framework — Granular Timing + Bottleneck Analysis
 *
 *   PURPOSE:
 *     Replace scattered printf() timing with a structured, configurable
 *     debug system that identifies where time is spent during capture:
 *       - Camera init (sensor detect, I2C config, DCMIPP setup)
 *       - Camera capture (exposure/gain, warmup frames, frame acquisition)
 *       - Camera deinit (pipe stop, deinit)
 *       - SD storage (card ready wait, batch writes, inter-batch gaps)
 *     Then print a clear summary with bottleneck identification.
 *
 *   USAGE:
 *     #include "perf_debug.h"
 *     #include "app_config.h"
 *
 *     PerfTimer_t t;
 *     PERF_START(t);
 *     PERF_MARK(t, PERF_CAM_INIT);
 *     ... camera init code ...
 *     PERF_MARK(t, PERF_CAM_EXPOSURE);
 *     ... exposure config ...
 *     PERF_MARK(t, PERF_CAM_WARMUP);
 *     ... warmup frames ...
 *     PERF_MARK(t, PERF_CAM_CAPTURE);
 *     ... frame captured ...
 *     PERF_MARK(t, PERF_CAM_DEINIT);
 *     ... deinit ...
 *     PERF_MARK(t, PERF_SD_WAIT);
 *     ... wait for card ...
 *     PERF_MARK(t, PERF_SD_WRITE);
 *     ... SD write ...
 *     PERF_MARK(t, PERF_DONE);
 *     PERF_STOP(t);
 *     PERF_PRINT_SUMMARY(t, snap_id);
 *
 *   DEBUG LEVELS (controlled by PERF_DEBUG_LEVEL in app_config.h):
 *     0 = Production: only final total time
 *     1 = Standard: phase breakdown percentages
 *     2 = Verbose: sub-phase timing + per-batch SD info (throttled)
 *     3 = Debug: everything + raw counters + wait-state details
 *
 * ******************************************************************************
 */

#ifndef PERF_DEBUG_H
#define PERF_DEBUG_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "stm32n6xx_hal.h"
#include "app_config.h"

/* ================================================================
   Phase Markers — ordered by typical execution flow
   ================================================================ */

typedef enum {
    PERF_PHASE_START      = 0,   /* Capture cycle begins */
    PERF_PHASE_CAM_INIT   = 1,   /* Camera sensor + DCMIPP init */
    PERF_PHASE_CAM_EXPO   = 2,   /* Exposure/gain configuration */
    PERF_PHASE_CAM_WARMUP = 3,   /* Warmup frame discard */
    PERF_PHASE_CAM_SNAP   = 4,   /* Final frame acquisition */
    PERF_PHASE_CAM_STOP   = 5,   /* DCMIPP pipe stop */
    PERF_PHASE_CAM_DEINIT = 6,   /* Camera deinit */
    PERF_PHASE_CAM_COPY   = 7,   /* memcpy to save buffer (continuous mode) */
    PERF_PHASE_CAM_RESTART= 8,   /* DCMIPP pipe restart (continuous mode) */
    PERF_PHASE_CACHE_CLEAN= 9,   /* D-Cache clean/invalidate ops */
    PERF_PHASE_SD_READY   = 10,  /* Wait for SD card ready */
    PERF_PHASE_SD_WRITE   = 11,  /* SD batch write (DMA transfer) */
    PERF_PHASE_SD_GAP     = 12,  /* Inter-batch gap (min 20ms delay) */
    PERF_PHASE_STORAGE    = 13,  /* Full storage task (camera_ready -> storage_done) */
    PERF_PHASE_DONE       = 14,  /* Capture cycle complete */
    PERF_PHASE_COUNT      = 15   /* Total number of phases */
} PerfPhase_t;

/* Human-readable phase names */
extern const char* perf_phase_names[PERF_PHASE_COUNT];

/* ================================================================
   Per-capture Timer Structure
   ================================================================ */

typedef struct {
    uint32_t start_tick;                          /* HAL_GetTick() at start */
    uint32_t phase_ticks[PERF_PHASE_COUNT];       /* tick at each phase mark */
    uint32_t phase_hit[PERF_PHASE_COUNT];         /* 1 if phase was marked */
    uint32_t phase_count[PERF_PHASE_COUNT];       /* for repeated phases (SD_WRITE) */

    /* SD-specific counters (when PERF_TRACK_SD_WAIT_TIME) */
    uint32_t sd_total_wait_ms;                    /* cumulative time waiting for card ready */
    uint32_t sd_total_write_ms;                   /* cumulative time in HAL_SD_WriteBlocks */
    uint32_t sd_total_gap_ms;                     /* cumulative inter-batch gap time */
    uint32_t sd_batch_count;                      /* number of SD batches written */
    uint32_t sd_max_batch_ms;                     /* slowest single batch write */
    uint32_t sd_max_wait_ms;                      /* longest single card-ready wait */

} PerfTimer_t;

/* ================================================================
   Running Statistics (when PERF_STATS_WINDOW > 0)
   ================================================================ */

typedef struct {
    uint32_t total_times[PERF_STATS_WINDOW];
    uint32_t cam_times[PERF_STATS_WINDOW];
    uint32_t sd_times[PERF_STATS_WINDOW];
    uint32_t count;
    uint32_t index;
} PerfStats_t;

extern PerfStats_t g_perf_stats;

/* ================================================================
   API — Timer Control
   ================================================================ */

/** Start a new capture timer (call at the beginning of a capture cycle) */
static inline void Perf_Start(PerfTimer_t *t)
{
    memset(t, 0, sizeof(*t));
    t->start_tick = HAL_GetTick();
    t->phase_ticks[PERF_PHASE_START] = t->start_tick;
    t->phase_hit[PERF_PHASE_START] = 1;
}

/** Mark a phase (call at key points in the capture pipeline) */
static inline void Perf_Mark(PerfTimer_t *t, PerfPhase_t phase)
{
    if (phase >= PERF_PHASE_COUNT) return;
    t->phase_ticks[phase] = HAL_GetTick();
    t->phase_hit[phase] = 1;
    t->phase_count[phase]++;
}

/** Stop the timer (call at the end of a capture cycle) */
static inline void Perf_Stop(PerfTimer_t *t)
{
    t->phase_ticks[PERF_PHASE_DONE] = HAL_GetTick();
    t->phase_hit[PERF_PHASE_DONE] = 1;
}

/** Get elapsed time between two phases */
static inline uint32_t Perf_PhaseElapsed(const PerfTimer_t *t, PerfPhase_t from, PerfPhase_t to)
{
    if (!t->phase_hit[from] || !t->phase_hit[to]) return 0;
    return t->phase_ticks[to] - t->phase_ticks[from];
}

/** Get total elapsed time */
static inline uint32_t Perf_TotalElapsed(const PerfTimer_t *t)
{
    if (!t->phase_hit[PERF_PHASE_DONE]) return HAL_GetTick() - t->start_tick;
    return t->phase_ticks[PERF_PHASE_DONE] - t->start_tick;
}

/* ================================================================
   API — SD Batch Tracking
   ================================================================ */

/** Record one SD batch write timing */
static inline void Perf_SD_RecordBatch(PerfTimer_t *t,
                                        uint32_t wait_ms,
                                        uint32_t write_ms,
                                        uint32_t gap_ms)
{
#if PERF_TRACK_SD_WAIT_TIME
    t->sd_total_wait_ms  += wait_ms;
    t->sd_total_write_ms += write_ms;
    t->sd_total_gap_ms   += gap_ms;
    t->sd_batch_count++;
    if (write_ms > t->sd_max_batch_ms) t->sd_max_batch_ms = write_ms;
    if (wait_ms  > t->sd_max_wait_ms)  t->sd_max_wait_ms  = wait_ms;
#endif
}

/* ================================================================
   API — Summary Printing
   ================================================================ */

/**
 * Print a structured performance summary.
 * @param t        Filled timer
 * @param snap_id  Snapshot number (for log correlation)
 * @return total_elapsed_ms
 */
uint32_t Perf_PrintSummary(PerfTimer_t *t, uint32_t snap_id);

/**
 * Update running statistics and print if window is full.
 * @param t  Filled timer
 */
void Perf_UpdateStats(PerfTimer_t *t);

/* ================================================================
   Convenience Macros
   ================================================================ */

#if PERF_DEBUG_LEVEL >= 1
    #define PERF_START(t)          Perf_Start(&(t))
    #define PERF_MARK(t, phase)    Perf_Mark(&(t), PERF_PHASE_##phase)
    #define PERF_STOP(t)           Perf_Stop(&(t))
    #define PERF_PRINT(t, id)      Perf_PrintSummary(&(t), id)
    #define PERF_STATS(t)          Perf_UpdateStats(&(t))
#else
    #define PERF_START(t)          Perf_Start(&(t))
    #define PERF_MARK(t, phase)    Perf_Mark(&(t), PERF_PHASE_##phase)
    #define PERF_STOP(t)           Perf_Stop(&(t))
    #define PERF_PRINT(t, id)      ((void)0)
    #define PERF_STATS(t)          ((void)0)
#endif

#endif /* PERF_DEBUG_H */
