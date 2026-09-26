#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
insect_filter_explorer.py
=========================
Explores a SUPER-FAST "empty vs something-new" frame filter for the STM32N6
insect camera (CMW-IMX335, 1296x972 YUV422). Goal: run on the MCU in the
moment of capture, decide in a few ms whether the frame contains a tiny new
object (an insect), drop the "empty" frames, and keep the background
calibrated while illumination drifts.

TWO MODES (they share the same integer-only core, so the Python logic is a
1:1 template for the C port):

  batch : offline analysis of a whole folder.
          Builds a robust per-pixel MEDIAN reference background ("truth"),
          computes a battery of metrics per image, ranks them, writes a CSV
          and marked copies (red frame + label + hot-region box), amplified
          residual-difference images and tight crops of the suspected insect.

  live  : simulates the real on-MCU pipeline over the same folder.
          Frames processed in timestamp order with an integer-EMA background,
          per-frame classification with hysteresis, background updated ONLY
          on CLEAN frames (plus fast snap on GLOBAL illumination events and a
          periodic full re-calibration). Shows exactly which frames the MCU
          would have flagged as DIFFERENT and how long each takes.

CORE METRICS (all integer arithmetic, tiny memory):
   1. 8x8 downsample -> ~19.6K luma grid (~20 KB in RAM on the MCU)
   2. global-mean shift removed (illumination decoupled from objects)
   3. residual |diff| histogram, 16 bins            (shape of the change)
   4. changed-pixel count + % in bps                (localized vs global)
   5. hot 32x32 blocks count + bounding box         (WHERE the change is)
   6. max residual diff                             (contrast of the object)
   7. 24-bin RGB histogram distance to reference    (color / white-balance)
   8. luma mean/std                                 (scene level / texture)

CLASSIFICATION (pure integer comparisons -> direct C port):
   CLEAN      -> background EMA update (masked: skips nothing)
   DIFFERENT  -> localized object -> KEEP/STORE; background updates only
                 OUTSIDE the object mask (masked EMA)
   BROAD      -> scene-wide change; FIRST frame stored (event start), the
                 following ones trigger a fast background snap (ENV-CAL)
   STATIC-CAL -> same localized change >= ABSORB_STATIC frames at the same
                 place -> now part of the scene, absorbed into background
   + periodic full re-calibration every CAL_EVERY_CLEAN clean frames

Usage:
   python insect_filter_explorer.py --mode both
   python insect_filter_explorer.py --mode live --top 20
   python insect_filter_explorer.py --folder D:\\other --mode batch --no-mark
"""

import argparse
import csv
import re
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

# ==========================================================================
# TUNABLE PARAMETERS -- the ONLY knobs you will port into app_config.h
# ==========================================================================
DS              = 8     # downsample block in original px (average 8x8)
HOT_DS          = 4     # hot block = 4x4 downsampled cells = 32x32 original px
NHIST           = 16    # residual |diff| histogram bins
T_PIXEL         = 24    # residual |diff| >= this counts a "changed pixel"
T_BLOCK         = 10    # block-mean residual |diff| >= this counts a hot block
EMA_SHIFT       = 4     # CLEAN:  bg += (x - bg) >> 4   (alpha = 1/16)
SNAP_SHIFT      = 2     # GLOBAL: bg += (x - bg) >> 2   (alpha = 1/4, fast recalc)
GLOBAL_MEAN     = 8     # |mean shift| >= this to even consider GLOBAL
GLOBAL_MAX_HOT  = 6     # ... AND residual hot blocks <= this (pure lighting)
MAX_HOT_OBJECT  = 40    # residual hot blocks <= this -> localized object
MIN_OBJECT_BPS  = 5     # object must still change >= 0.05% of pixels
CLEAN_MAX_BPS   = 25    # CLEAN needs changed pixels <= 0.25% (bps = /10_000)
CLEAN_MAX_DIFF  = 16    # CLEAN needs max residual diff <= this
HYS_BPS         = 12    # once DIFFERENT, revert to CLEAN only below 0.12%
CAL_EVERY_CLEAN = 300   # periodic full re-calibration every N consecutive
                        # clean frames (guards very slow drift)
ABSORB_STATIC   = 4     # same local change >= N consecutive frames at the
                        # same place -> it is now part of the scene: absorb
                        # it into the background and stop storing
SESSION_GAP_S   = 60    # filename-timestamp gap (s) marking a new session
MARK_MIN_BPS    = 2     # batch mode: only mark frames with >= this much change


# ==========================================================================
# Metric container
# ==========================================================================
@dataclass
class FrameMetrics:
    name: str = ""
    ts: float = 0.0
    session: int = 0
    mean_y: int = 0
    std_y: int = 0
    global_shift: int = 0
    max_diff: int = 0
    changed: int = 0
    changed_bps: int = 0
    n_hot: int = 0
    n_hot_strong: int = 0
    hot_bbox: tuple = None          # original-px (x0, y0, x1, y1) or None
    hist: list = field(default_factory=list)
    hist_peak: int = 0
    hist_peak_bps: int = 0
    rgb_hist_dist: int = 0          # 0..100 (% normalized L1)
    color_cr: int = 0               # mean(R) - mean(Y)
    color_cb: int = 0               # mean(B) - mean(Y)
    mask: object = None             # downsampled-res bool mask of hot cells
    label: str = ""
    note: str = ""
    score: float = 0.0
    ms: float = 0.0


# ==========================================================================
# CORE math -- 1:1 portable to C (integer arithmetic only)
# ==========================================================================
def to_luma(arr: np.ndarray) -> np.ndarray:
    """RGB uint8 -> luma int32.  Y = (77*R + 150*G + 29*B) >> 8  (BT.601).
    On the MCU the Y plane of YUV422 is already available from DCMIPP."""
    a = arr.astype(np.int32)
    return (a[..., 0] * 77 + a[..., 1] * 150 + a[..., 2] * 29) >> 8


def downsample(y: np.ndarray, block: int = DS) -> np.ndarray:
    """Average-pool y into (H/block, W/block) int32 grid (MCU block filter).
    8x8 block: sum of 64 values -> shift 6. Keep values in 0..255 range."""
    h = y.shape[0] - y.shape[0] % block
    w = y.shape[1] - y.shape[1] % block
    y = y[:h, :w]
    shift = 2 * (block.bit_length() - 1)
    return y.reshape(h // block, block, w // block, block).sum(axis=(1, 3)) >> shift


def diff_histogram(ad: np.ndarray) -> np.ndarray:
    """16 bins of |diff|: bin0=[0..7], then 4-wide bins up to 255.
    == a 16x uint16 accumulator updated per pixel on the MCU."""
    h = np.zeros(NHIST, dtype=np.int64)
    h[0] = int((ad <= 7).sum())
    for i in range(1, NHIST):
        h[i] = int(((ad >= 4 * i) & (ad < 4 * i + 4)).sum())
    return h


def hot_blocks(ad: np.ndarray):
    """Mean |diff| over HOT_DS x HOT_DS downsampled cells (=32x32 original px).
    Returns (n_hot, n_hot_strong, bbox, max_block_mean, bmean_grid, mask_grid).
    strong = blocks with mean >= 2*T_BLOCK: the object core; weak blocks are
    usually the illumination halo and should not decide "localized" nor
    stretch the bounding box.
    bmean_grid: (H/16, W/16) block means; mask_grid: (H, W) bool cell mask,
    the dilated hot region used to freeze the background under the object."""
    h = ad.shape[0] - ad.shape[0] % HOT_DS
    w = ad.shape[1] - ad.shape[1] % HOT_DS
    b = ad[:h, :w].reshape(h // HOT_DS, HOT_DS, w // HOT_DS, HOT_DS)
    bmean = b.sum(axis=(1, 3)) >> (2 * (HOT_DS.bit_length() - 1))  # mean of 16
    hot = bmean >= T_BLOCK
    strong = bmean >= 2 * T_BLOCK
    n, ns = int(hot.sum()), int(strong.sum())
    bbox = None
    if ns:
        ys, xs = np.where(strong)
    elif n:
        ys, xs = np.where(hot)
    else:
        ys = xs = None
    if ys is not None:
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        bbox = (x0 * HOT_DS * DS, y0 * HOT_DS * DS,
                (x1 + 1) * HOT_DS * DS - 1, (y1 + 1) * HOT_DS * DS - 1)
    mask = np.zeros(ad.shape, dtype=bool)
    mask[:h, :w] = np.kron(hot, np.ones((HOT_DS, HOT_DS), dtype=bool))
    return n, ns, bbox, (int(bmean.max()) if bmean.size else 0), bmean, mask


def rgb_hist24(arr: np.ndarray) -> np.ndarray:
    """24-bin RGB histogram (8 per channel) == 3x8 streaming counters on MCU."""
    h = np.zeros(24, dtype=np.int64)
    for c in range(3):
        h[c * 8:(c + 1) * 8] = np.histogram(arr[..., c], bins=8, range=(0, 256))[0]
    return h



def compute_metrics(name: str, ts: float, g: np.ndarray, bg: np.ndarray,
                    rgb: np.ndarray = None, ref_hist: np.ndarray = None) -> FrameMetrics:
    """g, bg : downsampled int32 grids (same shape).
    The global-mean shift is removed FIRST, so illumination changes do not
    masquerade as objects; whatever is left is the real 'new thing'."""
    m = FrameMetrics(name=name, ts=ts)
    mean_g = int(round(float(g.mean())))
    gs = mean_g - int(round(float(bg.mean())))
    resid = g - bg - gs
    ad = np.abs(resid)

    m.mean_y, m.global_shift = mean_g, gs
    m.std_y = int(round(float(g.std())))
    m.max_diff = int(ad.max())
    m.changed = int((ad > T_PIXEL).sum())
    m.changed_bps = m.changed * 10000 // int(ad.size)
    m.n_hot, m.n_hot_strong, m.hot_bbox, _, _, m.mask = hot_blocks(ad)

    h = diff_histogram(ad)
    m.hist = [int(x) for x in h]
    tot = int(h.sum())
    m.hist_peak = int(h.argmax())
    m.hist_peak_bps = int(h[m.hist_peak]) * 10000 // max(1, tot)

    if rgb is not None:
        m.color_cr = int(round(float(rgb[..., 0].mean()) - mean_g))
        m.color_cb = int(round(float(rgb[..., 2].mean()) - mean_g))
        if ref_hist is not None:
            fh = rgb_hist24(rgb)
            l1 = int(np.abs(fh - ref_hist.astype(np.int64)).sum())
            m.rgb_hist_dist = l1 * 100 // max(1, 2 * int(fh.sum()))
    return m


def classify(m: FrameMetrics, hyst: bool = False):
    """Pure integer comparisons -> direct C port.
    Returns ("CLEAN"|"DIFFERENT"|"BROAD", note).
    hyst=True lowers the CLEAN bar while a DIFFERENT streak is running, so a
    slow-moving insect does not flicker DIFFERENT/CLEAN/DIFFERENT.
    BROAD = the change covers the whole scene: it is either illumination
    (LED/daylight) or a big object. The live state machine stores the FIRST
    broad frame of an event and re-calibrates on the following ones."""
    clean_bps = HYS_BPS if hyst else CLEAN_MAX_BPS
    if m.changed_bps <= clean_bps and m.max_diff <= CLEAN_MAX_DIFF:
        return "CLEAN", ""
    # noise guard: no hot block and almost no changed pixels -> sensor noise
    # or the dissolving ghost of an absorbed object
    if m.n_hot == 0 and m.changed_bps < MIN_OBJECT_BPS:
        return "CLEAN", "noise (maxd=%d)" % m.max_diff
    if m.n_hot_strong <= MAX_HOT_OBJECT and m.changed_bps >= MIN_OBJECT_BPS:
        return "DIFFERENT", "object (%d strong/%d weak blocks)" % (m.n_hot_strong, m.n_hot)
    note = "scene-wide change"
    if abs(m.global_shift) >= GLOBAL_MEAN:
        note += ", mean shift %+d" % m.global_shift
    return "BROAD", note


def object_score(m: FrameMetrics) -> float:
    """Composite 0..1 'how much of something new is here' (reporting only)."""
    s = 0.45 * min(1.0, m.changed_bps / 400.0)
    s += 0.35 * (m.max_diff / 255.0)
    s += 0.20 * min(1.0, m.n_hot_strong / 8.0)
    if m.label in ("GLOBAL", "ENV-CAL", "BROAD"):
        s *= 0.25
    return s


# ==========================================================================
# I/O helpers
# ==========================================================================
TS_RE = re.compile(r"IMG_(\d{8})_(\d{6})Z")


def parse_ts(path: Path) -> float:
    m = TS_RE.search(path.name)
    if m:
        return datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S").timestamp()
    return path.stat().st_mtime


def load_rgb(path: Path):
    arr = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
    return arr


def frame_grid(path: Path) -> np.ndarray:
    return downsample(to_luma(load_rgb(path)))


def fmt_ts(t: float) -> str:
    return datetime.fromtimestamp(t).strftime("%H:%M:%S") if t else "?"


def write_csv(results, csv_path: Path):
    cols = ["file", "session", "time", "label", "score", "note",
            "mean_y", "std_y", "global_shift", "max_diff",
            "changed", "changed_bps", "n_hot", "n_hot_strong",
            "bbox_x0", "bbox_y0", "bbox_x1", "bbox_y1",
            "hist_peak", "hist_peak_bps",
            "rgb_hist_dist", "color_cr", "color_cb", "ms"]
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for m in results:
            bb = m.hot_bbox or (0, 0, 0, 0)
            w.writerow([m.name, m.session, fmt_ts(m.ts), m.label,
                        "%.3f" % m.score, m.note, m.mean_y, m.std_y,
                        m.global_shift, m.max_diff, m.changed, m.changed_bps,
                        m.n_hot, m.n_hot_strong, bb[0], bb[1], bb[2], bb[3],
                        m.hist_peak, m.hist_peak_bps, m.rgb_hist_dist,
                        m.color_cr, m.color_cb, "%.1f" % m.ms])



# ==========================================================================
# Annotation: mark the "different" images (never touches the originals)
# ==========================================================================
def mark_image(src: Path, out_dir: Path, m: FrameMetrics,
               grid: np.ndarray = None, ref: np.ndarray = None):
    """Red frame + label + hot-region box on a COPY, plus an amplified
    residual-difference image and a tight crop of the suspected insect."""
    img = Image.open(src).convert("RGB")
    W, H = img.size
    d = ImageDraw.Draw(img)

    for k in range(10):
        d.rectangle([k, k, W - 1 - k, H - 1 - k], outline=(255, 0, 0))
    if m.hot_bbox:
        x0, y0, x1, y1 = m.hot_bbox
        d.rectangle([max(0, x0 - 14), max(0, y0 - 14),
                     min(W - 1, x1 + 14), min(H - 1, y1 + 14)],
                    outline=(255, 0, 0), width=4)
    txt = ("%s  score=%.2f  maxd=%d  chg=%dbps  hot=%d  gs=%+d"
           % (m.label, m.score, m.max_diff, m.changed_bps, m.n_hot, m.global_shift))
    d.text((16, H - 36), txt, fill=(255, 255, 0))
    img.save(out_dir / ("MARKED_" + src.name))

    if grid is not None and ref is not None:
        resid = np.abs((grid - ref).astype(np.int32))
        amp = np.clip(resid * 6, 0, 255).astype(np.uint8)
        Image.fromarray(amp, mode="L").resize((W, H), Image.NEAREST).save(
            out_dir / ("DIFF_" + src.name))
        if m.hot_bbox:
            x0, y0, x1, y1 = m.hot_bbox
            pad = 60
            c = img.crop((max(0, x0 - pad), max(0, y0 - pad),
                          min(W, x1 + pad), min(H, y1 + pad)))
            c.save(out_dir / ("CROP_" + src.name))


# ==========================================================================
# BATCH mode : robust per-SESSION median reference (each session = one
# illumination condition, just like the live EMA background converges to)
# ==========================================================================
def sessionize(files):
    """Group timestamp-ordered files into sessions (gap > SESSION_GAP_S)."""
    sessions, s, last = {}, 0, None
    for f in files:
        ts = parse_ts(f)
        if last is not None and ts - last > SESSION_GAP_S:
            s += 1
        sessions.setdefault(s, []).append(f)
        last = ts
    return sessions


def hot_ascii(g: np.ndarray, ref: np.ndarray) -> str:
    """ASCII heat-map of block-mean residual |diff| (each char = 32x32 px).
    legend: '.' <4   ' ' low   ':' <20   '*' <40   '#' >=40"""
    gs = int(round(float(g.mean()))) - int(round(float(ref.mean())))
    resid = np.abs(g - ref - gs)
    h = resid.shape[0] - resid.shape[0] % HOT_DS
    w = resid.shape[1] - resid.shape[1] % HOT_DS
    b = resid[:h, :w].reshape(h // HOT_DS, HOT_DS, w // HOT_DS, HOT_DS)
    b = b.sum(axis=(1, 3)) >> 4
    out = []
    for r, row in enumerate(b):
        line = "".join(" .:*#" [min(4, v // 4)] if v else " " for v in row)
        out.append("%3d|%s|" % (r * HOT_DS * DS, line))
    return "\n".join(out)


def run_batch(files, out_dir: Path, do_mark: bool, top: int, maps: int = 6):
    out_dir.mkdir(parents=True, exist_ok=True)
    n = len(files)
    print("\n=== BATCH MODE : per-session median reference (%d frames) ===" % n)
    t0 = time.perf_counter()
    grids, hists = [], []
    for f in files:
        grids.append(frame_grid(f))
        hists.append(rgb_hist24(load_rgb(f)))
    print("  loaded + downsampled in %.1f s" % (time.perf_counter() - t0))

    grid_all = np.stack(grids)
    hist_all = np.stack(hists)
    sessions = sessionize(files)
    print("  %d sessions detected" % len(sessions))

    results = []
    for s, sfiles in sessions.items():
        idxs = [files.index(f) for f in sfiles]
        ref = np.median(grid_all[idxs], axis=0).astype(np.int32)
        ref_hist = np.median(hist_all[idxs], axis=0)
        print("  session %d: %3d frames, ts %s .. %s"
              % (s, len(idxs), fmt_ts(parse_ts(sfiles[0])),
                 fmt_ts(parse_ts(sfiles[-1]))))
        for i in idxs:
            t1 = time.perf_counter()
            m = compute_metrics(files[i].name, parse_ts(files[i]), grid_all[i], ref,
                                rgb=load_rgb(files[i]), ref_hist=ref_hist)
            m.session = s
            label, note = classify(m)
            m.label, m.note = label, note
            m.score = object_score(m)
            m.ms = (time.perf_counter() - t1) * 1000
            results.append((m, i, ref))

    results.sort(key=lambda t: t[0].score, reverse=True)
    ranked = [m for m, _, _ in results]
    print_ranked(ranked)
    csv_path = out_dir / "insect_filter_batch.csv"
    write_csv(ranked, csv_path)

    for m, i, ref in results[:maps]:
        print("\n  ---- hot-block map (32x32 px per char) for %s  [%s] ----"
              % (m.name, m.note))
        print(hot_ascii(grid_all[i], ref))

    if do_mark:
        marked = [t for t in results
                  if t[0].label != "CLEAN" and t[0].changed_bps >= MARK_MIN_BPS]
        marked = marked[:top]
        for m, i, ref in marked:
            mark_image(files[i], out_dir, m, grid_all[i], ref)
        print("\n  marked %d images -> %s" % (len(marked), out_dir))
        print("  CSV      -> %s" % csv_path)
    return ranked



# ==========================================================================
# LIVE mode : simulate the on-MCU pipeline in timestamp order
# ==========================================================================
def same_location(a, b, tol=2 * HOT_DS * DS):
    """True if two bboxes have centers within tol px (64 px = 2 hot blocks)."""
    if not a or not b:
        return False
    ax, ay = (a[0] + a[2]) // 2, (a[1] + a[3]) // 2
    bx, by = (b[0] + b[2]) // 2, (b[1] + b[3]) // 2
    return abs(ax - bx) <= tol and abs(ay - by) <= tol


def run_live(files, out_dir: Path, do_mark: bool, top: int):
    out_dir.mkdir(parents=True, exist_ok=True)
    print("\n=== LIVE MODE : simulated MCU pipeline (integer EMA background) ===")
    bg = None
    bg_hist = None
    in_hyst = False
    broad_streak = 0
    clean_streak = 0
    static_streak = 0
    last_bbox = None
    session = 0
    last_ts = None
    calibrations = 0
    results = []
    keep = []                     # (file, metrics, grid, bg) for flagged frames

    for k, f in enumerate(files):
        t1 = time.perf_counter()
        arr = load_rgb(f)
        g = downsample(to_luma(arr))
        ts = parse_ts(f)

        if last_ts is not None and ts - last_ts > SESSION_GAP_S:
            session += 1
        session = max(1, session)
        last_ts = ts

        if bg is None:
            bg = g.copy()
            bg_hist = rgb_hist24(arr)
            m = FrameMetrics(name=f.name, ts=ts, session=session)
            m.label, m.note = "INIT", "background initialized"
            m.ms = (time.perf_counter() - t1) * 1000
            results.append(m)
            print("  [%3d] INIT       bg init  %s" % (k + 1, f.name))
            continue

        m = compute_metrics(f.name, ts, g, bg, rgb=arr, ref_hist=bg_hist)
        m.session = session
        label, note = classify(m, hyst=in_hyst)
        m.note = note

        if label == "CLEAN":
            m.label = "CLEAN"
            # calibration happens here: integer EMA, exact C equivalent:
            #   bg += (g - bg) >> EMA_SHIFT
            bg = bg + ((g - bg) >> EMA_SHIFT)
            bg_hist = bg_hist + ((rgb_hist24(arr) - bg_hist) >> EMA_SHIFT)
            in_hyst = False
            broad_streak = 0
            last_bbox = None
            static_streak = 0
            clean_streak += 1
            if clean_streak >= CAL_EVERY_CLEAN:
                bg, bg_hist = g.copy(), rgb_hist24(arr)
                calibrations += 1
                m.note += " | PERIODIC CALIBRATION (full snap)"
                clean_streak = 0
        elif label == "DIFFERENT":
            m.label = "DIFFERENT"
            # localized object:
            #  - background tracks the scene EVERYWHERE except under the
            #    object (masked EMA) -> a ghost dissolves in ~16 frames when
            #    the insect leaves
            bg = np.where(m.mask, bg, bg + ((g - bg) >> EMA_SHIFT))
            in_hyst = True
            clean_streak = 0
            broad_streak = 0
            if same_location(last_bbox, m.hot_bbox):
                static_streak += 1
            else:
                static_streak = 1
            if static_streak >= ABSORB_STATIC:
                # same change at the same place for several frames: it is a
                # STATIC scene element now -> absorb it, stop storing
                bg[m.mask] = g[m.mask]
                calibrations += 1
                m.label = "STATIC-CAL"
                m.note += " | static object absorbed into background"
                last_bbox = None
                static_streak = 0
            else:
                last_bbox = m.hot_bbox
                keep.append((f, m, g.copy(), bg.copy()))
        else:  # BROAD
            if broad_streak == 0:
                # first frame of a scene-wide event: store it (big object
                # entering, or the frame where the lighting changed)
                m.label = "DIFFERENT"
                m.note += " | event start (stored)"
                in_hyst = True
                clean_streak = 0
                broad_streak = 1
                last_bbox = None
                static_streak = 0
                keep.append((f, m, g.copy(), bg.copy()))
            else:
                # scene-wide change persists -> environment (illumination):
                # fast snap = 2x alpha-1/4 passes, frame discarded
                m.label = "ENV-CAL"
                m.note += " | RECALIBRATION"
                bg = bg + ((g - bg) >> SNAP_SHIFT)
                bg = bg + ((g - bg) >> SNAP_SHIFT)
                bg_hist = bg_hist + ((rgb_hist24(arr) - bg_hist) >> 1)
                in_hyst = False
                clean_streak = 0
                broad_streak += 1
                last_bbox = None
                static_streak = 0
                calibrations += 1
        m.score = object_score(m)

        m.ms = (time.perf_counter() - t1) * 1000
        results.append(m)
        print("  [%3d] %-9s gs=%+3d maxd=%3d chg=%5dbps hot=%3d rgbd=%2d%% %7.1f ms"
              % (k + 1, m.label, m.global_shift, m.max_diff, m.changed_bps,
                 m.n_hot, m.rgb_hist_dist, m.ms))

    write_csv(results, out_dir / "insect_filter_live.csv")

    flagged = [m for m in results if m.label == "DIFFERENT"]
    print("  ---- summary ----")
    print("  frames: %d | INIT: %d | CLEAN: %d | ENV-CAL: %d | STATIC-CAL: %d | DIFFERENT(stored): %d"
          % (len(results),
             sum(1 for m in results if m.label == "INIT"),
             sum(1 for m in results if m.label == "CLEAN"),
             sum(1 for m in results if m.label == "ENV-CAL"),
             sum(1 for m in results if m.label == "STATIC-CAL"),
             len(flagged)))
    print("  background recalibrations: %d" % calibrations)
    print("  pipeline time per frame (incl. PNG load): mean %.1f ms, max %.1f ms"
          % (np.mean([m.ms for m in results]), np.max([m.ms for m in results])))
    print("  frames the MCU would have STORED (DIFFERENT):")
    for m in flagged:
        print("     %s  %s  score=%.2f %s" % (m.name, fmt_ts(m.ts), m.score, m.note))

    if do_mark and keep:
        keep.sort(key=lambda t: t[1].score, reverse=True)
        for f, m, g, bg in keep[:top]:
            mark_image(f, out_dir, m, g, bg)
        print("  marked top %d flagged images -> %s" % (min(top, len(keep)), out_dir))
    return results


def print_ranked(results):
    print("  rank  file                                    label      score  maxd  chg(bps)  hot  gs   rgbd")
    for i, m in enumerate(results[:25], 1):
        print("  %3d   %-42s %-9s  %5.3f  %4d  %8d  %3d  %+3d  %3d%%"
              % (i, m.name[:42], m.label, m.score, m.max_diff,
                 m.changed_bps, m.n_hot, m.global_shift, m.rgb_hist_dist))
    n_diff = sum(1 for m in results if m.label == "DIFFERENT")
    n_broad = sum(1 for m in results if m.label == "BROAD")
    n_clean = sum(1 for m in results if m.label == "CLEAN")
    print("  ... totals: DIFFERENT=%d BROAD=%d CLEAN=%d (of %d)"
          % (n_diff, n_broad, n_clean, len(results)))


# ==========================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--folder", default=r"C:\Users\ssierra\Downloads\test_photos")
    ap.add_argument("--mode", choices=["batch", "live", "both"], default="both")
    ap.add_argument("--top", type=int, default=15,
                    help="max marked images per mode")
    ap.add_argument("--no-mark", action="store_true")
    args = ap.parse_args()

    folder = Path(args.folder)
    exts = (".png", ".jpg", ".jpeg")
    files = [p for p in folder.iterdir() if p.suffix.lower() in exts]
    files.sort(key=parse_ts)
    if not files:
        print("No images found in", folder)
        return
    print("Folder: %s (%d images, sorted by timestamp)" % (folder, len(files)))

    if args.mode in ("batch", "both"):
        run_batch(files, folder.parent / (folder.name + "_marked_batch"),
                  not args.no_mark, args.top)
    if args.mode in ("live", "both"):
        run_live(files, folder.parent / (folder.name + "_marked_live"),
                 not args.no_mark, args.top)

    print("\nNOTE: the live pipeline cost above includes PNG decoding; on the MCU")
    print("the whole core runs STREAMING over the YUV422 DMA buffer (~19.6K")
    print("downsampled integers): a few ms at most, no frame memory needed.")


if __name__ == "__main__":
    main()

