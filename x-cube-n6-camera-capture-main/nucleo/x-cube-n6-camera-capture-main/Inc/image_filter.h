#ifndef IMAGE_FILTER_H
#define IMAGE_FILTER_H

#include <stdint.h>

/* ================================================================
   CONFIGURATION — tune these for your lighting / insect size
   ================================================================ */

#define IF_DETECT_W       640
#define IF_DETECT_H       480
#define IF_DETECT_FRAME_SZ  (IF_DETECT_W * IF_DETECT_H * 2)  /* YUV422 */

#define IF_DS             8      /* downsample block (8×8 original px) */
#define IF_GRID_W          (IF_DETECT_W / IF_DS)  /* 80 */
#define IF_GRID_H          (IF_DETECT_H / IF_DS)  /* 60 */

#define IF_HOT_DS          4      /* hot block = 4×4 downsampled cells */
#define IF_HOT_W            (IF_GRID_W / IF_HOT_DS)  /* 20 */
#define IF_HOT_H            (IF_GRID_H / IF_HOT_DS)  /* 15 */

#define IF_NHIST           16     /* residual histogram bins */
#define IF_T_PIXEL         16     /* |diff| >= this → "changed pixel" */
#define IF_T_BLOCK         7      /* block-mean |diff| ≥ this → "hot block" */
#define IF_T_DARK_BLOCK    1      /* dark-block threshold: 1 dark cell at d>=17 gives 17/16=1 */
#define IF_EMA_SHIFT       4      /* bg += (x - bg) >> IF_EMA_SHIFT */
#define IF_GLOBAL_MEAN     8      /* |mean shift| ≥ this → consider global */
#define IF_GLOBAL_MAX_HOT  6      /* ... AND hot blocks ≤ this → lighting */
#define IF_MAX_HOT_OBJECT  40     /* hot blocks ≤ this → localized object */
#define IF_MIN_OBJECT_BPS  2      /* object must change ≥ 0.02 % pixels (1 grid cell = 2 BPS) */
#define IF_CLEAN_MAX_BPS   25     /* CLEAN needs ≤ 0.25 % changed */
#define IF_CLEAN_MAX_DIFF  16     /* CLEAN needs max diff ≤ this */

/* ROI sizing (for Stage 3 fallback) */
#define IF_ROI_MARGIN      20     /* extra px around bbox (detect coords) */
#define IF_ROI_MAX_SZ_W    2000   /* max insect width in sensor px (AREA3 supports up to 2592) */
#define IF_ROI_MAX_SZ_H    1800   /* max insect height in sensor px (AREA3 supports up to 1944) */

/* ================================================================
   TYPES
   ================================================================ */

typedef struct {
    uint8_t  grid[IF_GRID_H][IF_GRID_W];       /* downsampled luma */
} IF_Grid_t;

/* Hot-block bitmap and bounding box (in detection-pixel coords) */
typedef struct {
    uint8_t  hot_map[IF_HOT_H][IF_HOT_W];      /* 1 = hot block */
    uint16_t bbox_x, bbox_y;                   /* top-left (detect px) */
    uint16_t bbox_w, bbox_h;                   /* size (detect px) */
    uint8_t  hot_count;                         /* number of hot blocks */
    uint8_t  hot_strong_count;                  /* blocks ≥ 2× threshold */
    uint8_t  has_insect;                        /* 0 or 1 */
    uint8_t  fallback;                          /* 1 = insect too large for ROI */
} IF_Result_t;

/* ================================================================
   API
   ================================================================ */

/* Call once at boot: zero background grid and state */
void IF_Init(void);

/* Downsample a 640×480 YUV422 frame into the grid (extract Y, 8×8 avg).
   frame must point to IF_DETECT_FRAME_SZ bytes of YUV422 data. */
void IF_Downscale(const uint8_t *frame, IF_Grid_t *grid);

/* EMA background update: bg[y][x] += (grid[y][x] - bg[y][x] + (1<<(EMA_SHIFT-1))) >> EMA_SHIFT
   When mask is non-NULL, cells where mask[y/4][x/4] != 0 are SKIPPED (masked update). */
void IF_UpdateBackground(const IF_Grid_t *grid, const uint8_t mask[IF_HOT_H][IF_HOT_W]);

/* Full detection pipeline on a 640×480 YUV422 frame.
   Internally calls Downscale, computes residual, histogram, hot blocks,
   bounding box. Sets result->has_insect and result->fallback. */
void IF_Process(const uint8_t *frame, IF_Result_t *result);

/* Quick check: map detection bbox to sensor ROI coords (2592×1944).
   Call after IF_Process if has_insect==1.
   If insect fits in ROI, writes roi_x/y/w/h and returns 0.
   If too large, returns -1 (caller should skip Stage 3). */
int  IF_CalcROI(const IF_Result_t *detection,
                uint16_t *roi_x, uint16_t *roi_y,
                uint16_t *roi_w, uint16_t *roi_h);

#endif /* IMAGE_FILTER_H */
