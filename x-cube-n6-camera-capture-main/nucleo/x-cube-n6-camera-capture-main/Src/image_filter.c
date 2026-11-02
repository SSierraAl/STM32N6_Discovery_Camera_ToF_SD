#include "image_filter.h"
#include <string.h>
#include <stdio.h>

/* ================================================================
   STATIC STATE — background grid persists across calls
   ================================================================ */

static uint8_t  bg_grid[IF_GRID_H][IF_GRID_W];   /* EMA background */
static uint8_t  bg_init_done;

/* ================================================================
   IF_Init
   ================================================================ */

void IF_Init(void)
{
    memset(bg_grid, 0, sizeof(bg_grid));
    bg_init_done = 0;
}

/* ================================================================
   IF_Downscale  —  640×480 YUV422 → 80×60 luma grid (8×8 avg)
   ================================================================
   YUYV byte layout: Y0 U0 Y1 V0  Y2 U2 Y3 V2  ...  640 Y per row.
   Row stride = 640 × 2 = 1280 bytes.
   Cell (gy, gx) = average of Y at block (gy*8..gy*8+7, gx*8..gx*8+7).
   ================================================================ */

void IF_Downscale(const uint8_t *frame, IF_Grid_t *grid)
{
    int gy, gx, dy, dx;
    unsigned sum;

    for (gy = 0; gy < IF_GRID_H; gy++) {
        for (gx = 0; gx < IF_GRID_W; gx++) {
            sum = 0;
            for (dy = 0; dy < IF_DS; dy++) {
                const uint8_t *row = frame + ((uint32_t)(gy * IF_DS + dy)) * IF_DETECT_W * 2;
                for (dx = 0; dx < IF_DS; dx++) {
                    sum += row[(gx * IF_DS + dx) * 2];   /* Y at even byte */
                }
            }
            grid->grid[gy][gx] = (uint8_t)(sum >> 6);   /* divide by 64 */
        }
    }
}

/* ================================================================
   IF_UpdateBackground  —  masked EMA update
   ================================================================
   If mask is non-NULL, cells in hot blocks (mask != 0) are SKIPPED.
   Formula:  bg += (grid - bg + round) >> IF_EMA_SHIFT
   ================================================================ */

void IF_UpdateBackground(const IF_Grid_t *grid, const uint8_t mask[IF_HOT_H][IF_HOT_W])
{
    int gy, gx;
    int round = 1 << (IF_EMA_SHIFT - 1);

    if (!bg_init_done) {
        /* First call: snap full grid as initial background */
        memcpy(bg_grid, grid->grid, sizeof(bg_grid));
        bg_init_done = 1;
        return;
    }

    for (gy = 0; gy < IF_GRID_H; gy++) {
        for (gx = 0; gx < IF_GRID_W; gx++) {
            if (mask && mask[gy / IF_HOT_DS][gx / IF_HOT_DS])
                continue;
            int diff = (int)grid->grid[gy][gx] - (int)bg_grid[gy][gx];
            bg_grid[gy][gx] = (uint8_t)(bg_grid[gy][gx] + ((diff + round) >> IF_EMA_SHIFT));
        }
    }
}

/* ================================================================
   IF_Process  —  full detection pipeline
   ================================================================ */

void IF_Process(const uint8_t *frame, IF_Result_t *result)
{
    uint8_t  grid[IF_GRID_H][IF_GRID_W];

    /* First call: snap frame as initial background, report no insect */
    if (!bg_init_done) {
        IF_Downscale(frame, (IF_Grid_t *)&grid);
        memcpy(bg_grid, grid, sizeof(bg_grid));
        bg_init_done = 1;
        memset(result, 0, sizeof(*result));
        return;
    }
    int      gy, gx, bx, by, dy, dx;
    int      sum, mean_g, mean_bg, global_shift;
    int      diff;
    uint8_t  ad[IF_GRID_H][IF_GRID_W];
    int      max_diff, changed, changed_bps;
    int      block_sum[IF_HOT_H][IF_HOT_W];
    int      block_mean;
    unsigned hot_sum;

    /* ---------- 1. Downscale ---------- */
    for (gy = 0; gy < IF_GRID_H; gy++) {
        for (gx = 0; gx < IF_GRID_W; gx++) {
            hot_sum = 0;
            for (dy = 0; dy < IF_DS; dy++) {
                const uint8_t *row = frame + ((uint32_t)(gy * IF_DS + dy)) * IF_DETECT_W * 2;
                for (dx = 0; dx < IF_DS; dx++) {
                    hot_sum += row[(gx * IF_DS + dx) * 2];
                }
            }
            grid[gy][gx] = (uint8_t)(hot_sum >> 6);
        }
    }

    /* ---------- 2. Means & global shift ---------- */
    sum = 0;
    for (gy = 0; gy < IF_GRID_H; gy++)
        for (gx = 0; gx < IF_GRID_W; gx++)
            sum += grid[gy][gx];
    mean_g = sum / (IF_GRID_H * IF_GRID_W);

    sum = 0;
    for (gy = 0; gy < IF_GRID_H; gy++)
        for (gx = 0; gx < IF_GRID_W; gx++)
            sum += bg_grid[gy][gx];
    mean_bg = sum / (IF_GRID_H * IF_GRID_W);

    global_shift = mean_g - mean_bg;

    /* ---------- 3. Residual, changed pixels, max diff ---------- */
    max_diff = 0;
    changed = 0;
    int dark_cells_vs_mean = 0;
    int bright_cells_vs_mean = 0;
    int dark_vs_bg_only = 0;
    for (gy = 0; gy < IF_GRID_H; gy++) {
        for (gx = 0; gx < IF_GRID_W; gx++) {
            diff = (int)grid[gy][gx] - (int)bg_grid[gy][gx] - global_shift;
            if (diff < 0) diff = -diff;
            int dark = (int)mean_g - (int)grid[gy][gx];
            if (dark > IF_T_PIXEL) {
                dark_cells_vs_mean++;
                if (dark > diff) dark_vs_bg_only++;
            }
            if ((int)grid[gy][gx] - (int)mean_g > IF_T_PIXEL) bright_cells_vs_mean++;
            if (dark > IF_T_PIXEL && dark > diff) {
                diff = dark;
            }
            if (diff > 255) diff = 255;
            ad[gy][gx] = (uint8_t)diff;
            if (diff > max_diff) max_diff = diff;
            if (diff > IF_T_PIXEL) changed++;
        }
    }
    changed_bps = changed * 10000 / (IF_GRID_H * IF_GRID_W);
    /* ---------- 4. Hot blocks (4×4 grid cells = 32×32 original px) ---------- */
    result->hot_count        = 0;
    result->hot_strong_count = 0;
    memset(result->hot_map, 0, sizeof(result->hot_map));

    for (by = 0; by < IF_HOT_H; by++) {
        for (bx = 0; bx < IF_HOT_W; bx++) {
            block_sum[by][bx] = 0;
            for (dy = 0; dy < IF_HOT_DS; dy++) {
                for (dx = 0; dx < IF_HOT_DS; dx++) {
                    block_sum[by][bx] += ad[by * IF_HOT_DS + dy][bx * IF_HOT_DS + dx];
                }
            }
            block_mean = block_sum[by][bx] / (IF_HOT_DS * IF_HOT_DS);
            if (block_mean >= IF_T_BLOCK) {
                result->hot_map[by][bx] = 1;
                result->hot_count++;
                if (block_mean >= 2 * IF_T_BLOCK)
                    result->hot_strong_count++;
            }
        }
    }

    /* ---------- 5. Bounding box (largest connected component) ---------- */
    {
        /* Find the largest 4-connected cluster in hot_map to get a tight bbox.
           hot_map is only 20x15 = 300 bytes, so a simple stack BFS is fine. */
        int best_x0 = IF_HOT_W, best_x1 = 0, best_y0 = IF_HOT_H, best_y1 = 0;
        int best_sz = 0;
        uint8_t visited[IF_HOT_H][IF_HOT_W];
        memset(visited, 0, sizeof(visited));
        int stack[IF_HOT_H * IF_HOT_W];
        int sp;

        for (by = 0; by < IF_HOT_H; by++) {
            for (bx = 0; bx < IF_HOT_W; bx++) {
                if (!result->hot_map[by][bx] || visited[by][bx])
                    continue;
                /* BFS this component */
                sp = 0;
                stack[sp++] = by * IF_HOT_W + bx;
                visited[by][bx] = 1;
                int c_x0 = bx, c_x1 = bx, c_y0 = by, c_y1 = by;
                int c_sz = 0;
                while (sp > 0) {
                    int cy = stack[--sp] / IF_HOT_W;
                    int cx = stack[sp] % IF_HOT_W;
                    c_sz++;
                    if (cx < c_x0) c_x0 = cx;
                    if (cx > c_x1) c_x1 = cx;
                    if (cy < c_y0) c_y0 = cy;
                    if (cy > c_y1) c_y1 = cy;
                    /* 4-connected neighbors */
                    if (cx > 0     && result->hot_map[cy][cx-1] && !visited[cy][cx-1]) { visited[cy][cx-1] = 1; stack[sp++] = cy * IF_HOT_W + (cx-1); }
                    if (cx < IF_HOT_W-1 && result->hot_map[cy][cx+1] && !visited[cy][cx+1]) { visited[cy][cx+1] = 1; stack[sp++] = cy * IF_HOT_W + (cx+1); }
                    if (cy > 0     && result->hot_map[cy-1][cx] && !visited[cy-1][cx]) { visited[cy-1][cx] = 1; stack[sp++] = (cy-1) * IF_HOT_W + cx; }
                    if (cy < IF_HOT_H-1 && result->hot_map[cy+1][cx] && !visited[cy+1][cx]) { visited[cy+1][cx] = 1; stack[sp++] = (cy+1) * IF_HOT_W + cx; }
                }
                if (c_sz > best_sz) {
                    best_sz = c_sz;
                    best_x0 = c_x0; best_x1 = c_x1;
                    best_y0 = c_y0; best_y1 = c_y1;
                }
            }
        }

        if (best_sz > 0) {
            result->hot_count = (uint8_t)best_sz;
            result->bbox_x = (uint16_t)(best_x0 * IF_HOT_DS * IF_DS);
            result->bbox_y = (uint16_t)(best_y0 * IF_HOT_DS * IF_DS);
            result->bbox_w = (uint16_t)((best_x1 - best_x0 + 1) * IF_HOT_DS * IF_DS);
            result->bbox_h = (uint16_t)((best_y1 - best_y0 + 1) * IF_HOT_DS * IF_DS);
        } else {
            result->bbox_x = 0;
            result->bbox_y = 0;
            result->bbox_w = 0;
            result->bbox_h = 0;
        }
    }

    /* ---------- 6. Dark-block analysis ----------
        Instead of using general hot_count (which also counts bright-side
        variation and can exceed IF_MAX_HOT_OBJECT even for a real insect),
        count blocks where the grid is significantly DARKER than the frame
        mean.  An insect is a dark object on a bright background.       */
    {
        int dc = 0, ds = 0;
        int dc_per_row[IF_HOT_H];
        for (by = 0; by < IF_HOT_H; by++) {
            int row_dc = 0;
            for (bx = 0; bx < IF_HOT_W; bx++) {
                int dsum = 0;
                for (dy = 0; dy < IF_HOT_DS; dy++)
                    for (dx = 0; dx < IF_HOT_DS; dx++) {
                        int d = mean_g - (int)grid[by * IF_HOT_DS + dy][bx * IF_HOT_DS + dx];
                        if (d > IF_T_PIXEL) dsum += d;
                    }
                int dm = dsum / (IF_HOT_DS * IF_HOT_DS);
                if (dm >= IF_T_DARK_BLOCK) {
                    dc++;
                    row_dc++;
                    if (dm >= 2 * IF_T_DARK_BLOCK) ds++;
                }
            }
            dc_per_row[by] = row_dc;
        }
        result->has_insect = (dc > 0 && dc <= IF_MAX_HOT_OBJECT) ? 1 : 0;
    }
    result->fallback = 0;

    /* ---------- 7. Background update (masked under insect) ---------- */
    {
        const uint8_t *mask_ptr = (const uint8_t *)result->hot_map;
        int round = 1 << (IF_EMA_SHIFT - 1);
        for (gy = 0; gy < IF_GRID_H; gy++) {
            for (gx = 0; gx < IF_GRID_W; gx++) {
                if (mask_ptr[(gy / IF_HOT_DS) * IF_HOT_W + (gx / IF_HOT_DS)])
                    continue;
                diff = (int)grid[gy][gx] - (int)bg_grid[gy][gx];
                bg_grid[gy][gx] = (uint8_t)(bg_grid[gy][gx] + ((diff + round) >> IF_EMA_SHIFT));
            }
        }
    }

    /* ---------- 8. Fallback check ---------- */
    if (result->has_insect) {
        uint16_t sensor_w = (uint16_t)(((uint32_t)result->bbox_w * 2592) / IF_DETECT_W);
        uint16_t sensor_h = (uint16_t)(((uint32_t)result->bbox_h * 1944) / IF_DETECT_H);
        if (sensor_w > IF_ROI_MAX_SZ_W || sensor_h > IF_ROI_MAX_SZ_H)
            result->fallback = 1;
    }
}

/* ================================================================
   IF_CalcROI  —  map detection bbox → sensor ROI (2592×1944)
   Returns 0 on success, -1 if insect too large for ROI window.
   ================================================================ */

int IF_CalcROI(const IF_Result_t *detection,
              uint16_t *roi_x, uint16_t *roi_y,
              uint16_t *roi_w, uint16_t *roi_h)
{
    uint32_t insect_cx, insect_cy;
    uint32_t insect_w, insect_h;
    uint32_t w, h;
    int32_t  x, y;

    if (!detection->has_insect)
        return -1;

    /* Map bounding box to sensor pixel coordinates */
    insect_w = (uint32_t)detection->bbox_w * 2592 / IF_DETECT_W;
    insect_h = (uint32_t)detection->bbox_h * 1944 / IF_DETECT_H;
    insect_cx = ((uint32_t)detection->bbox_x + (uint32_t)detection->bbox_w / 2) * 2592 / IF_DETECT_W;
    insect_cy = ((uint32_t)detection->bbox_y + (uint32_t)detection->bbox_h / 2) * 1944 / IF_DETECT_H;

    /* Fallback: insect too large for ROI? */
    if (insect_w > IF_ROI_MAX_SZ_W || insect_h > IF_ROI_MAX_SZ_H)
        return -1;

    /* Safety: reject edge false detections.  A real insect at the sensor
       edge with suspiciously few hot blocks is almost always a gamma/lighting
       artifact.  We let IF_ROI_MARGIN absorb 2 block-columns of drift before
       rejecting: only skip when the raw bbox touches the detection-frame edge
       AND hot_count is below a generous ceiling. */
    if ((detection->bbox_x < 16 || detection->bbox_y < 16) &&
        detection->hot_count <= IF_MAX_HOT_OBJECT / 2)
        return -1;

    /* ROI = insect size + margin */
    w = insect_w + (uint32_t)IF_ROI_MARGIN * 2592 / IF_DETECT_W * 2;
    h = insect_h + (uint32_t)IF_ROI_MARGIN * 1944 / IF_DETECT_H * 2;

    if (w > 2592) w = 2592;
    if (h > 1944) h = 1944;

    /* Center on insect */
    x = (int32_t)insect_cx - (int32_t)(w / 2);
    y = (int32_t)insect_cy - (int32_t)(h / 2);

    /* Clamp to sensor bounds */
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if ((uint32_t)x + w > 2592) x = (int32_t)(2592 - w);
    if ((uint32_t)y + h > 1944) y = (int32_t)(1944 - h);

    /* Even alignment (YUV422 requires 2-pixel alignment) */
    *roi_x = (uint16_t)((uint32_t)x & ~1U);
    *roi_y = (uint16_t)((uint32_t)y & ~1U);
    *roi_w = (uint16_t)(w & ~1U);
    *roi_h = (uint16_t)(h & ~1U);

    if (*roi_w < 64) *roi_w = 64;
    if (*roi_h < 64) *roi_h = 64;

    return 0;
}
