# Image_filter.py — Insect-Photo Filter: Full Algorithm Documentation

A fast **"empty photo vs. something new"** filter for the STM32N6 insect
camera (CMW-IMX335, 1296×972, YUV422). The Python file is a **1:1 template**
for the C port: the whole detection core uses **integer math only** (adds,
compares, bit shifts) — no floats, no external libraries, ~40 KB of RAM.

---

## 0. TL;DR (the 30-second version)

1. Shrink the picture 8× into a small grid of ~20K numbers.
2. Subtract the **background** (a calibrated copy of the empty scene) and the
   **average light level** — whatever is left is the *change*.
3. Measure the change: how many pixels changed, how big is the biggest
   difference, and **where** (a box around the changed area).
4. Decision:
   - change is tiny → **empty photo → discard, keep calibrating the background**
   - change is in a **small area** → **insect! → store the photo**
   - change is **everywhere** → **lighting changed → fast re-calibration**
5. The background re-calibrates itself continuously (EMA) and periodically
   (full snap every 300 clean frames) — that is the "periodic calibration".

---

## 1. What it does & why it exists

Your rig takes a photo every time the ToF sensor triggers. Most of those
triggers are empty: light flicker, a leaf, your own hand at the wrong moment,
a tiny insect that is only 1–2 % of the frame. Storing all of them wastes the
SD card and the ~700–900 ms that each SD write takes.

`Image_filter.py` answers, **per frame and in milliseconds**:

> *"Is there a NEW local thing in this picture compared to the scene I
> remember? If yes → STORE. If no → skip."*

Two run modes (they share the exact same core):

- **`batch`** — offline analysis of a whole folder. Builds a robust reference
  background (the *median* of each capture session), measures every image
  against it, ranks them, and writes the marked outputs.
- **`live`** — a faithful **simulation of the on-MCU pipeline**: images are
  processed in timestamp order, one integer-EMA background is maintained,
  every frame is classified with hysteresis, and the background re-calibrates
  itself. It tells you exactly which frames the MCU would have stored and how
  long each one takes.

---

## 2. Usage

```bat
:: default: both modes, mark the 20 top images
python Image_filter.py --mode both --top 20

:: only the MCU pipeline simulation
python Image_filter.py --mode live

:: only the offline folder analysis
python Image_filter.py --mode batch

:: another photo folder, no marked images (report only)
python Image_filter.py --folder D:\other_photos --no-mark
```

### Input / output folders

| Role | Default path | What it contains |
|---|---|---|
| **INPUT** | `C:\Users\ssierra\Downloads\test_photos` | 1296×972 PNGs (raw YUV422 converted to PNG). The timestamp is read from the filename: `IMG_YYYYMMDD_HHMMSSZ_seq_seq.png` |
| **OUTPUT (batch)** | `C:\Users\ssierra\Downloads\test_photos_marked_batch\` | see below |
| **OUTPUT (live)** | `C:\Users\ssierra\Downloads\test_photos_marked_live\` | see below |

The output folder is always a *sibling* of the input folder:
`<input_folder>_marked_batch` or `<input_folder>_marked_live`.
**Originals are never modified.**

### Output files

| Prefix / file | What it is |
|---|---|
| `MARKED_<orig>.png` | a copy of the photo with a **red border**, a yellow text line with all metrics, and a **red rectangle exactly over the object** (bounding box) |
| `DIFF_<orig>.png` | the **residual difference ×6 amplified** as a gray image: the object glows white on black — the fastest way to see *what changed* |
| `CROP_<orig>.png` | a tight crop (box ±60 px) of the object region — see the insect without opening the 2 MB file |
| `insect_filter_batch.csv` / `insect_filter_live.csv` | **every metric of every image** (columns explained in §8) |

---

## 3. The pipeline at a glance

```
  PHOTO 1296×972  (RGB PNG in tests / YUV422 on the MCU)
        │
        ▼
 ┌────────────────────────────────────────────────────┐
 │ STEP 0  LUMA                                        │
 │  Y = (77·R + 150·G + 29·B) >> 8        (§4.1)      │
 │  On the MCU the Y plane already exists in the       │
 │  YUV422 DMA buffer — this step is skipped there.    │
 └────────────────────────────────────────────────────┘
        │
        ▼
 ┌────────────────────────────────────────────────────┐
 │ STEP 1  DOWNSAMPLE 8×8                              │
 │  average every 8×8 pixel block → one number        │
 │  grid: 121×162 = 19 602 values in 0..255   (§4.2)  │
 └────────────────────────────────────────────────────┘
        │            frame grid  g        background grid  bg
        │                                    (calibrated memory of the scene)
        ▼
 ┌────────────────────────────────────────────────────┐
 │ STEP 2  REMOVE GLOBAL LIGHT SHIFT                   │
 │  gs    = avg(g) − avg(bg)                           │
 │  resid = g − bg − gs                (§4.3, §4.4)   │
 │  → a lighting change disappears from resid;         │
 │    only SPATIAL differences (objects) survive       │
 └────────────────────────────────────────────────────┘
        │
        ▼
 ┌────────────────────────────────────────────────────┐
 │ STEP 3  MEASURE |resid|                             │
 │  • 16-bin difference histogram      (§4.5)         │
 │  • changed-pixel count, in bps      (§4.6)         │
 │  • hot 32×32 blocks: weak + strong, (§4.7)         │
 │    bounding box of the strong blocks                │
 │  • max |resid|                                     │
 │  • (reporting) 24-bin RGB color histogram (§4.9)   │
 └────────────────────────────────────────────────────┘
        │
        ▼
 ┌────────────────────────────────────────────────────┐
 │ STEP 4  CLASSIFY:  CLEAN / DIFFERENT / BROAD  (§5) │
 └────────────────────────────────────────────────────┘
        │
        ▼
 ┌────────────────────────────────────────────────────┐
 │ STEP 5  UPDATE THE BACKGROUND  bg        (§6)      │
 │  EMA everywhere (clean) / EMA around the object    │
 │  (masked) / fast snap (lighting) / full snap       │
 │  (periodic)                                         │
 └────────────────────────────────────────────────────┘
```

Everything after STEP 1 works on **19 602 integers**, not on 1.26 million
pixels. That is the entire trick that makes it fast enough for an MCU.


---

## 4. The building blocks, with real numbers

### 4.1 Luma (Y) — "one brightness number per pixel"

A pixel is 3 numbers (R, G, B). The detector only needs **brightness**, so
each pixel is squashed into one:

```
Y = (77·R + 150·G + 29·B) >> 8
```

`>> 8` = divide by 256, and 77+150+29 = 256, so this is just a weighted
average where green counts most (humans see brightness mostly through green).
Result: one number 0..255 per pixel (0 = black, 255 = white).

> On the MCU this step **does not exist**: the camera pipeline (DCMIPP)
> already delivers YUV422, where the Y plane is exactly this brightness
> channel. The Python only converts it because the test photos are RGB PNGs.

### 4.2 Downsampling 8×8 — "1.26 million pixels → 20 000 numbers"

The photo is a grid of pixels. We cut it into **blocks of 8×8 = 64 pixels**,
and replace each whole block by **one number: the average of its 64 pixels**.

Why?

1. **Speed**: 1 257 408 pixels ÷ 64 = **19 602 numbers**. All the detection
   math (steps 2–4) runs on those 19 602 numbers, not on the full photo.
2. **Noise immunity**: sensor noise is random (−3, +2, −1, +4 …) and
   **cancels out** when you average 64 pixels. A real object is *not*
   random: it pushes the average of its blocks in one direction and stays
   there for several consecutive blocks.

**Worked example** — one 8×8 block over an empty part of the scene (all
pixels ≈ 180 gray):

```
 180 182 179 181 178 183 180 179      sum of the 64 values
 178 181 180 179 182 180 178 181      ─────────────────────  11 570
 181 179 180 182 178 180 181 179
 180 182 178 181 180 179 182 180      average = 11 570 / 64
 ...                              =  180.8  → stored as 180
 179 180 182 178 181 180 179 182

  → this block becomes ONE cell of value 180
```

**Worked example** — an 8×8 block where a dark insect covers ~16 pixels
(48 dark, rest 180):

```
 180 180  48  48 180 180 180 180
 180 180  48  48 180 180 180 180
 180 180  48  48 180 180 180 180      sum = 48·180 + 16·48
 ...                              =  8640 +  768  =  9 408
 180 180  48  48 180 180 180 180

  average = 9 408 / 64 = 147.0  → stored as 147
```

Empty block → 180. Block touched by the insect → 147. **The difference is
33 levels** — visible and stable, while pure noise would only wobble ±2.

In code (and in C) "divide by 64" is a bit shift: `sum >> 6`.

```
  1296×972 pixels                       121×162 cells
 ┌────────────────────────────┐         ┌──────────────────┐
 │ ▓▓▓▓▓▓▓▓ ─ 8×8 block ─ ▓▓▓ │         │ 180 181 147 182  │
 │ ▓▓▓▓▓▓▓▓ ─────── ▓▓▓▓▓▓▓▓ │  ──▶    │ 179 147 147 180  │
 │ ░░░░░░░░ ─ 8×8 block ─ ▓▓▓ │         │ 147 180 182 181  │
 └────────────────────────────┘         │  (each = one    │
    1296/8 = 162 cols                   │   8×8 average)  │
    972/8  = 121 rows (972 % 8 = 4      └──────────────────┘
    pixels on the bottom edge are
    ignored)
```

**Consequence to remember**: the detector's "pixel" is now an **8×8 block**.
The smallest thing it can resolve is a few blocks ≈ a 32×32 px area (§4.7).
A 20 px insect still works because it shifts the averages of 1–4 blocks
enough to exceed the thresholds — that is exactly what your Sep-24 test
series proved (see §9).


### 4.3 The background model (`bg`) — "the memory of the empty scene"

`bg` is **another copy of the same 121×162 grid**, filled with what the scene
looks like *right now with no insect in it*. It is the reference everything is
compared against:

```
        grid cell (i,j)          example
        ┌─────────────────────────────────────────────┐
        │ frame  g[i,j] = 147     (photo just taken)  │
        │ bg     bg[i,j] = 180    (my memory of empty)│
        └─────────────────────────────────────────────┘
        residual = 147 − 180 = −33   →  |resid| = 33  → "something is here"
```

`bg` is not fixed at boot. It **learns continuously** from every frame that
is classified as empty (that is the calibration, §6). One `bg` cell can hold
0..255, so it is a single byte → 19 602 bytes ≈ **20 KB**.

### 4.4 Removing the global light shift (`gs`)

Lighting (WS2812 LEDs, day/night, auto-exposure) changes the **mean** of the
*whole* image: every cell gets brighter or darker by roughly the same amount.
If we compared `g − bg` directly, a lighting change would look like "the whole
scene changed" and every photo would be flagged.

Fix: measure the mean shift and subtract it first:

```
gs    = avg(g) − avg(bg)              one single number for the frame
resid = g − bg − gs                   (done per cell, integer math)
```

Worked example: the LEDs dim the scene by 10 levels and an insect covers 2
cells.

```
                 no shift removal (g − bg)      with shift removal (resid)
   empty cell:    −10  (light!)                    −10 − (−10) =   0  ✓ gone
   insect cell:   −40  (light + insect)            −40 − (−10) = −30  ✓ only
                                                        the insect is left
```

After this step, `|resid|` is non-zero **only where space changed** — i.e.
only on the object. (This is why a pure lighting change ends up as `BROAD`
with a low residual, or as `CLEAN` after re-calibration.)

### 4.5 The 16-bin difference histogram — "the shape of the change"

Take **every cell** of `|resid|` (19 602 values, each 0..255) and drop it
into one of 16 bins:

```
 bin 0:        values 0..7      (basically "no change")
 bin 1:        values 8..11
 bin 2:        values 12..15
 bin 3:        values 16..19
 bin 4:        values 20..23
 bin 5:        values 24..27
 bin 6:        values 28..31
 bin 7:        values 32..35
 bin 8:        values 36..39
 bin 9:        values 40..43
 bin 10:       values 44..47
 bin 11:       values 48..51
 bin 12:       values 52..55
 bin 13:       values 56..59
 bin 14:       values 60..63
 bin 15:       values 64..255   (saturated: "huge" differences)
```

In code it is just 16 counters: for each cell, increment the counter of the
bin that contains its value. (On the MCU: one 16×`uint16` array updated per
cell — a few ns.)

Worked example, a frame with an empty scene + one small insect:

```
19 602 cells:
 19 596 cells have |resid| in 0..7   →  bin 0
     3 cells have |resid| 10..13     →  bin 2  (object edges)
     1 cell  has |resid| 30..33      →  bin 7  (object core)
     2 cells have |resid| 45..48     →  bin 10 (object core)

 histogram = [19596, 0, 3, 0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 0]
             └────────┬─────┘  └─┬─┘         └┬┘        └┬┘
              "no change"  small         medium     strong
                            diffs        diffs      diffs
```

Reading the **shape**:

| Shape | Meaning |
|---|---|
| almost everything in bin 0 | empty frame (CLEAN) |
| small bumps in low bins (0..4) | sensor noise / tiny ghost |
| bumps in mid/high bins + big bin 0 | a real object, contrast = which bins |
| the peak bin and its % of the total (`hist_peak`, `hist_peak_bps`) | summarized in the CSV for diagnostics |

> The histogram itself is **diagnostic/reporting only** — the decision uses
> the counts of §4.6–4.7, which are the histogram's "tail" (values above the
> thresholds) aggregated two different ways.


### 4.6 Changed-pixel count (`changed`, `changed_bps`)

One pass over the 19 602 cells:

```
for each cell:
    if |resid| > T_PIXEL (24):   changed += 1
```

`changed` is a raw count of cells that really differ from the background.
Because it grows with image size, the code normalizes it to **bps**
(basis points = parts per 10 000):

```
changed_bps = changed × 10 000 / 19 602
```

Examples:

```
changed =     0   →   0 bps   (nothing changed)
changed =   100   →  51 bps   (100 cells = ~0.5% = a small insect)
changed =  4 900   → 2500 bps (a quarter of the grid = scene-wide event)
changed = 19 602   → 10 000 bps (everything changed)
```

Why cells, not pixels? 19 602 is small, so `changed` fits in a 16-bit
counter and the whole pass is 19 602 comparisons — trivial for the MCU.

### 4.7 Hot blocks and the bounding box — "WHERE is it?"

Counting changed cells tells you *how much* changed. To know *where* (needed
for the red box in `MARKED_*`, the `CROP_*`, and for the "localized vs
scene-wide" decision), the cell grid is averaged a **second time**, in
4×4-cell blocks. 4 cells × 8 px = **32×32 original pixels per block**:

```
  cell grid 121×162                    block grid 30×40
  (each cell = 8×8 px)                 (each block = 4×4 cells = 32×32 px)
 ┌──────────────────────────┐         ┌─────────────────────┐
 │ c c c c│c c c c│c c c c c│         │   B₀₀  B₀₁  B₀₂ …  │
 │ c c c c│c c c c│c c c c c│  ──▶    │   B₁₀  B₁₁  B₁₂ …  │
 │ c c c c│c c c c│c c c c c│         │   B₂₀  B₂₁  B₂₂ …  │
 │ c c c c│c c c c│c c c c c│         │   …  (30 rows,      │
 └──────────────────────────┘         │        40 cols)     │
                                     └─────────────────────┘
  Bᵣ,𝒸 = mean of |resid| over its 16 cells  (= sum >> 4)
```

Each block gets a label from its mean:

```
mean < 10                →  not hot
10 ≤ mean < 20           →  WEAK hot   (T_BLOCK)
mean ≥ 20                →  STRONG hot (2·T_BLOCK)  = the object's core
```

Two counters are accumulated: `n_hot` (weak+strong) and `n_hot_strong`.

**The bounding box** is then simply the smallest rectangle that contains all
STRONG blocks (weak blocks are ignored on purpose — they are the soft edge /
lighting halo and would stretch the box):

```
  block grid (rows × cols), strong blocks marked:

              col 0  1  2  3  4  5
        row 0   .    .   *   *    .     * = strong block
        row 1   .    .   *   *    .
        row 2   .    .   .    .    .

  min row = 0, max row = 1   →   y0 = 0·32 = 0    y1 = (1+1)·32 − 1 = 63
  min col = 2, max col = 3   →   x0 = 2·32 = 64   x1 = (3+1)·32 − 1 = 127

  → bbox = (x=64, y=0) to (x=127, y=63)  in ORIGINAL pixels
           = a 64×64 px box tightly around the object core
```

The mapping is just multiplication/division by 32, because blocks are a
fixed 32×32 px. In C: `x0 = min_col << 5`, etc.

**Why this answers "localized vs scene-wide":**
a small insect touches a handful of blocks (`n_hot_strong` ≈ 1–10, out of
1200). A lighting change touches *hundreds* of blocks. So:

```
n_hot_strong ≤ MAX_HOT_OBJECT (40)   →  localized  →  DIFFERENT (store)
n_hot_strong > 40                    →  scene-wide →  BROAD
```


### 4.8 The object mask — "don't learn the insect as background"

When a frame is classified `DIFFERENT`, the background still has to keep
learning the *rest* of the scene (light drifts while the insect is there).
But the cells **under the insect** must NOT be learned, or the insect would
slowly "fade into" `bg` and disappear from detection.

The **mask** is a 0/1 map, one bit per *cell* (121×162), built from the hot
blocks: every cell that belongs to a hot block (weak or strong) gets a 1.

```
  block grid            mask (cell resolution)
      .   .   *   .        each hot block (4×4 cells) → 16 cells set to 1
      .   *   *   .
      .   *   .   .
            ↓
  cells:   0 0 0 0 0 0 0 0 0 0 0 0
           0 0 0 0 0 0 1 1 1 1 0 0
           0 0 0 0 0 0 1 1 1 1 0 0     ← 4×4 ones for block (0,2)
           0 0 0 0 0 0 1 1 1 1 0 0
           0 0 0 0 0 1 1 1 1 1 1 0     ← 4×4 ones for block (1,1) and (1,2)
           0 0 0 0 0 1 1 1 1 1 1 0
           0 0 0 0 0 0 1 1 1 1 0 0
           0 0 0 0 0 0 0 0 0 0 0 0

  background update (per cell):
       if mask == 0:   bg += (g − bg) >> EMA_SHIFT     ← normal EMA
       if mask == 1:   bg unchanged (frozen)           ← under the object
```

Memory: 19 602 bits ≈ **2.5 KB**. Two nice side effects:

- **While the insect stays**: its region is frozen, so the diff stays visible
  frame after frame (no "learning the object away").
- **When the insect leaves**: the cells are no longer hot → mask = 0 there →
  the EMA refills them with the real (empty) scene in ~16 frames. The
  leftover difference (a "ghost") decays on its own and falls below the
  thresholds. No special cleanup code needed.

### 4.9 The 24-bin RGB histogram — "did the COLOR change?"

The luma grid answers "what got brighter/darker". A second, independent check
answers "did the color balance change" (LED on vs daylight, white-balance
shift):

1. Build a histogram of the frame: **8 bins per channel** (R, G, B), each bin
   = 32 brightness levels → 24 counters.
   `bin 0 = 0..31, bin 1 = 32..63, …, bin 7 = 224..255`
2. Do the same for the background/reference (in `live` mode it is its own
   EMA, so the color reference tracks the scene too).
3. Distance = how different the two 24-bin shapes are:

```
L1      = Σ |frame_bin[k] − ref_bin[k]|        (k = 0..23)
distance% = L1 × 100 / (2 × total_pixels)

  0%  → identical color distribution
  100% → completely different color distribution
```

In your test set this metric is what separates "LED session" from "daylight
session" (values of 26–36 % between sessions, < 3 % inside a session). It is
**reporting/diagnostic only** — the store/discard decision never uses it, so
it is optional in the C port (but cheap: 24 counters updated while scanning).


---

## 5. Classification — the decision logic

One function, `classify()`, a chain of **integer comparisons only** (port to
C as-is). Inputs: the metrics of §4.6–4.7; output: one of three labels.

```
                         ┌──────────────────────────────────────────────┐
                         │ Q1: changed_bps ≤ 25  AND  max_diff ≤ 16 ?    │
                         │     (≤ 0.25% of cells changed,              │
                         │      and nothing brighter than 16 levels)    │
                         └──────────────────┬───────────────────────────┘
                               YES │              │ NO
                                   ▼              ▼
                              ┌─────────┐   ┌──────────────────────────────┐
                              │  CLEAN  │   │ Q2: zero hot blocks AND      │
                              │ (empty) │   │     changed_bps < 5 ?        │
                              └─────────┘   └──────────┬───────────────────┘
                                             YES (noise/ghost) │      │ NO
                                                               ▼      ▼
                                                          ┌────────┐  ┌───────────────────────────┐
                                                          │ CLEAN  │  │ Q3: n_hot_strong ≤ 40 ?    │
                                                          └────────┘  └──────┬──────────────┬─────┘
                                                   YES (localized object)  │              │ NO
                                                                           ▼              ▼
                                                                     ┌───────────┐  ┌────────┐
                                                                     │ DIFFERENT │  │ BROAD  │
                                                                     │  → STORE  │  │ scene- │
                                                                     └───────────┘  │ wide   │
                                                                                    └────────┘
```

| Label | Meaning | Action |
|---|---|---|
| **CLEAN** | background only, no detectable change (or sensor noise / dissolving ghost) | discard photo; background keeps EMA-learning |
| **DIFFERENT** | a **localized** change: ≤ 40 strong 32×32 blocks = something small appeared (insect) | **STORE the photo** |
| **BROAD** | the change covers the scene (> 40 strong blocks): lighting change, or a very large object | in `live` mode: store **only the FIRST frame** of the event ("event start"), every following broad frame triggers a fast background re-calibration (`ENV-CAL`) and is discarded |

The Q2 guard exists because of **noise and ghosts**: after the background
absorbs/learns a change, a few cells can still poke out 17–25 levels with no
block reaching the hot threshold. That is not an object — it is cleaned up,
not stored.

**Hysteresis (anti-flicker).** While a `DIFFERENT` streak is running, the Q1
bar drops from 25 to `HYS_BPS` (12) bps. A slow-moving insect then never
flickers `DIFFERENT → CLEAN → DIFFERENT` frame by frame; the streak ends only
when the change almost vanishes.

---

## 6. Background update & calibration — how `bg` stays "the empty scene"

| Situation | Update applied | Why |
|---|---|---|
| **CLEAN** | full EMA: `bg += (g − bg) >> 4` (α = 1/16) on **all** cells | the continuous "periodic calibration": `bg` chases slow light drift (one new step per clean frame) |
| **CLEAN × 300 in a row** | **full snap** `bg = g` (`CAL_EVERY_CLEAN`) | insurance against a very slow drift the EMA would only chase asymptotically |
| **DIFFERENT** | **masked EMA**: `bg += (g − bg) >> 4` only where mask = 0 (§4.8); frozen under the object | the scene around the insect keeps tracking; the insect is NOT learned into the background |
| **BROAD, first frame** | none (frame is stored as "event start") | keep evidence of the moment the scene changed |
| **BROAD, following frames** | **fast snap**: `bg += (g − bg) >> 2` (α = 1/4), applied **twice** per frame → error × 1/16 each frame | a lighting change is accepted quickly; convergence in ~3 frames (`ENV-CAL` label) |
| **STATIC-CAL** | `bg[mask] = g[mask]` (copy the object region into `bg`) | see below |

### What is STATIC-CAL and why it exists

If the *same* localized change appears in the *same* place for
`ABSORB_STATIC` (4) consecutive frames, it is no longer "something new" — it
is part of the scene now (a speck stuck on the glass, a plant, an insect that
landed and stopped). Storing 40 identical photos of it is exactly the waste
this filter exists to avoid. So:

- frames 1–3 at the same spot → **stored** (it *was* new),
- from frame 4 on → the region is **absorbed into `bg`**, not stored.

"If it moves or leaves, detect it again": a moved object changes location →
new bbox → stored again. A leaving object leaves a ghost that the masked EMA
dissolves in ~16 frames (§4.8). Same-spot test = bbox centers within 64 px
(`same_location()`).


---

## 7. Parameters — every knob, in plain words

All of them live in the `# TUNABLE PARAMETERS` block at the top of
`Image_filter.py`. In the firmware they become `#define`s in `app_config.h`.

### 7.1 Geometry — "at what scale do we look"

| Parameter | Value | What it is, in one sentence |
|---|---|---|
| `DS` | `8` | block size of the first averaging (§4.2); 8 → each grid cell = 8×8 px; changing it rescales everything |
| `HOT_DS` | `4` | block size of the second averaging (§4.7); hot block = `4·DS`×`4·DS` = 32×32 px = the detector's localization unit |
| `NHIST` | `16` | number of bins of the difference histogram (§4.5); purely diagnostic |

### 7.2 Detection thresholds — "what counts as an object"

| Parameter | Value | Plain meaning | Lower → | Higher → |
|---|---|---|---|---|
| `T_PIXEL` | `24` | a cell counts as "changed" only if it differs from the background by ≥ 24 brightness levels | more sensitive to faint things, but sensor noise starts to count | tiny/faint insects may stop counting |
| `T_BLOCK` | `10` | a 32×32 block is "hot" if its mean difference ≥ 10 (strong = ≥ 20) | the object's box grows (halo included) | a 32×32 blob with contrast < 10 never forms a hot block → the noise guard kills it |
| `CLEAN_MAX_BPS` | `25` | a frame is CLEAN if ≤ 0.25 % of its cells changed | almost everything gets stored | frames with a tiny insect can slip through as CLEAN |
| `CLEAN_MAX_DIFF` | `16` | … and nothing may differ by more than 16 levels | more sensitive | less sensitive |
| `MIN_OBJECT_BPS` | `5` | an object must change at least 0.05 % of cells (≈ 10 cells) to be believed; also the noise guard in Q2 | noise/false positives creep in | sub-cell faint objects may be ignored |
| `MAX_HOT_OBJECT` | `40` | max number of STRONG blocks for "it is a localized object" (more = BROAD) | big objects keep being stored as objects | a big insect (≈ 10 % of the frame) flips to BROAD (only first frame stored) |
| `HYS_BPS` | `12` | relaxed CLEAN bar during a running DIFFERENT streak (hysteresis, §5) | streaks last longer (safer) | flicker risk at the end of an event |

### 7.3 Calibration speed — "how fast `bg` learns"

| Parameter | Value | Plain meaning |
|---|---|---|
| `EMA_SHIFT` | `4` | learning rate on CLEAN frames: `>> 4` = α 1/16 (slow, robust). `>> 3` learns 2× faster but tracks noise more |
| `SNAP_SHIFT` | `2` | learning rate on BROAD/lighting frames: `>> 2` = α 1/4, applied twice per frame → converges in ~3 frames |
| `CAL_EVERY_CLEAN` | `300` | every 300 consecutive CLEAN frames → one full snap `bg = g` (periodic re-calibration against ultra-slow drift) |
| `ABSORB_STATIC` | `4` | consecutive frames with the same change at the same place before it is absorbed into `bg` (STATIC-CAL, §6). Raise to 8–10 if you want static objects to keep being stored longer; 1 = always store |
| `SESSION_GAP_S` | `60` | time gap (s) that splits a `batch` folder into sessions (each session gets its own median reference) |
| `GLOBAL_MEAN` | `8` | informational: how large a mean shift must be to mention "lighting" in the notes |
| `MARK_MIN_BPS` | `2` | `batch` mode: minimum change for an image to receive a marked copy |

### 7.4 Symptom → tuning cheat sheet

| Symptom | Likely cause | Fix |
|---|---|---|
| stores "empty" photos because of sensor noise | `T_PIXEL`/`T_BLOCK` too low, or `MIN_OBJECT_BPS` too low | raise `MIN_OBJECT_BPS` to 8–10 |
| misses the tiny insect | `T_PIXEL` or `T_BLOCK` too high | lower them (18 / 8), check `MIN_OBJECT_BPS` |
| a lighting change costs 1 extra stored frame | normal — that is the "event start" by design | accept it, or lower `MAX_HOT_OBJECT` so broad events never store |
| background "forgets" a quiet insect too fast | `ABSORB_STATIC` small or `EMA_SHIFT` large | raise `ABSORB_STATIC` to 8–10 |
| ghost frames after the insect left | masked EMA is slow (normal ~10–16 frames) | lower `EMA_SHIFT` to 3 if it bothers you |


---

## 8. Per-image metrics (the CSV columns)

Both CSVs (`insect_filter_batch.csv`, `insect_filter_live.csv`) contain one
row per image:

| Column | How it is computed | Used for |
|---|---|---|
| `file`, `session`, `time` | name; session index (batch) / live session; timestamp from the name | reporting |
| `label` | output of `classify()` + live-mode handling (`INIT`, `CLEAN`, `DIFFERENT`, `BROAD`/`ENV-CAL`, `STATIC-CAL`) | **the decision** |
| `score` | `0.45·min(1, bps/400) + 0.35·maxd/255 + 0.20·min(1, strong/8)` (×0.25 for BROAD) | ranking/reporting ONLY — never part of the decision |
| `note` | human-readable reason ("object (3 strong/9 weak blocks)", "RECALIBRATION", …) | debugging |
| `mean_y`, `std_y` | mean / std-dev of the frame luma grid | exposure & texture diagnostics |
| `global_shift` | `avg(frame) − avg(bg)` (§4.4) | residual computation; lighting notes |
| `max_diff` | max of `|resid|` over all cells | CLEAN bar; object contrast |
| `changed` | cells with `|resid| > T_PIXEL` (§4.6) | CLEAN bar; noise guard |
| `changed_bps` | `changed × 10 000 / cells` | CLEAN bar; object size |
| `n_hot` | 32×32 blocks with mean ≥ 10 (§4.7) | noise guard; diagnostics |
| `n_hot_strong` | 32×32 blocks with mean ≥ 20 | **localized vs BROAD**; box |
| `bbox_x0..bbox_y1` | rectangle around the strong blocks, original px (§4.7) | `MARKED_*` rectangle, `CROP_*` |
| `hist_peak`, `hist_peak_bps` | dominant histogram bin and its share % (§4.5) | "shape of the change" diagnostics |
| `rgb_hist_dist` | normalized L1 distance of the 24-bin RGB histograms (§4.9) | color/white-balance diagnostics |
| `color_cr`, `color_cb` | `avg(R)−avg(Y)`, `avg(B)−avg(Y)` | tint drift diagnostics |
| `ms` | per-frame wall time of the pipeline (Python, incl. PNG decode) | performance reference |

---

## 9. Results on the 233 test photos

Run on `C:\Users\ssierra\Downloads\test_photos` (22 capture sessions, several
days, several lighting conditions):

| Measure | batch mode | live mode (MCU simulation) |
|---|---|---|
| images flagged different | 29 objects + 17 scene-wide events | **41 stored out of 233** (≈ 82 % of photos discarded) |
| clean | 187 | 159 |
| background recalibrations | — | 32 (27 lighting BROAD + 5 STATIC-CAL absorptions) |
| missed real events | 0 | 0 |

**Event 1 — Sep 23, 13:59:46–14:02:08** (ids 0730–0792): a *large* object
appears center-frame and moves; its box travels from (288,544)–(671,831) to
(480,224)–(895,831). Strongest frames: 0790, 0791, 0792, 0745, 0746, 0742,
0754–0756. Live mode stored the event-start frames and re-calibrated between
its position changes.

**Event 2 — Sep 24, 17:13:41–17:15:24** (ids 2553–2592): the *tiny* insect,
tracked frame by frame:
enters top-left (box ~64×96 px) → moves to the lower-left area (2557–2560)
→ left border (2565–2568) → parks in the corner at **(0,128)–(31,159)**:
only **1–2 hot blocks, contrast 54–85 levels, 0.1–0.12 % of cells**
(2569–2580) → leaves. Detected in every phase.

All other ~209 photos were empty scene or illumination variation → filtered.
The `CROP_*` files in the marked folders show exactly what was detected.


---

## 10. Porting to the STM32N6 (Python → C map)

The whole core is portable as-is; the only platform-specific part is where
the pixels come from.

| Python (`Image_filter.py`) | C in the firmware | Notes |
|---|---|---|
| `to_luma()` | — | the MCU already has the Y plane in the DCMIPP YUV422 buffer (stride 2: `buf[2·i]`). This step exists only because the test PNGs are RGB |
| `downsample()` | `IF_Downsample8x8(const uint8_t *y_plane, uint8_t *grid)` | one pass over 1.26 M pixels: accumulate 64 values, store `sum >> 6`. ~5 ops/pixel |
| `compute_metrics()` (residual part) | `IF_Metrics(const uint8_t *g, const uint8_t *bg, FrameMetrics *m)` | one pass over 19 602 cells: sums → `gs`; then `resid = g − bg − gs` (int16 in registers): `changed` count, max, 16-bin hist |
| `hot_blocks()` | `IF_HotBlocks(const int16_t *ad, …)` | pass 1 over the grid → 30×40 block means (`sum >> 4`); pass 2: strong/weak counts, bbox (min/max of strong), mask (1 bit per cell) |
| `classify()` | `IF_Classify(const FrameMetrics *m, uint8_t hyst)` | the exact `if` chain of §5 |
| EMA / snap / mask / absorb (§6) | `IF_UpdateBg(uint8_t *bg, const uint8_t *g, const uint8_t *mask, uint8_t mode)` | `bg += (g − bg) >> SHIFT`, with/without mask; `mode` = CLEAN / MASKED / SNAP / ABSORB / FULL |
| `run_live()` loop | DCMIPP **frame-complete callback** (after the warm-up frames) | the decision happens **before** the SD write starts |

### MCU resource budget

| Resource | Size |
|---|---|
| frame grid `g` (uint8) | 19 602 B ≈ **20 KB** (PSRAM) |
| background `bg` (uint8) | ≈ **20 KB** (PSRAM) |
| block means (uint8, 30×40) | 1.2 KB |
| mask (1 bit per cell) | 2.5 KB |
| counters (hist16, bps, bbox, state) | < 1 KB (SRAM) |
| compute time | 1.26 M downsample ops + 19.6 K metric ops → **< 5 ms** at 300 MHz |

### The big win: decide before the SD write

Per `PROJECT_DOCUMENTATION.md`, writing one 1296×972 frame to SD takes
**700–900 ms**. The filter's verdict is known a few ms after frame end, so
empty frames can skip the write entirely — saving that 700–900 ms per
discarded photo and dramatically slowing the SD fill:

```
frame complete (DCMIPP)
   → IF_Downsample8x8 → IF_Metrics → IF_HotBlocks → IF_Classify
       ├─ CLEAN / ENV-CAL / STATIC-CAL → IF_UpdateBg(...) → NO SD WRITE
       └─ DIFFERENT → IF_UpdateBg(masked) → SD WRITE (existing flow)
```

---

## 11. Code layout & known limitations

| Block in `Image_filter.py` | Role | Portable? |
|---|---|---|
| `# TUNABLE PARAMETERS` | all knobs (§7) | yes → `app_config.h` |
| `FrameMetrics` | per-frame data struct | yes → `typedef struct` |
| `to_luma / downsample / diff_histogram / hot_blocks / rgb_hist24` | core arithmetic (§4) | **yes, 1:1** |
| `compute_metrics` | assembles the metrics | **yes, 1:1** |
| `classify` | the state machine (§5) | **yes, 1:1** |
| `object_score` | report ranking | no |
| `load_rgb / parse_ts / write_csv / mark_image / hot_ascii` | I/O, annotation, diagnostics | no (Python only) |
| `run_batch` | offline analysis | no (reference tool) |
| `run_live` loop | the MCU pipeline logic | **yes — this is the firmware state machine** |

Known limitations:

1. **Static objects ≥ 4 frames are absorbed** (STATIC-CAL, §6) — by design;
   raise `ABSORB_STATIC` (20–30) or set it to 1 to keep storing them.
2. **Objects > ~3 % of the frame** (> 40 strong blocks) are treated as
   BROAD: only the first frame is stored. Adjust `MAX_HOT_OBJECT` if a big
   insect should keep being stored.
3. **Two insects very close together** read as one blob (one box).
4. Minimum resolvable object ≈ a few 32×32 blocks; a sub-8-px speck averages
   away in the first downsample (it is below the camera's useful detail for
   this use case anyway).
5. `batch` mode assumes a static folder with timestamps in the filenames;
   `live` mode assumes chronological input (on the MCU that is given).
6. Timings reported in live mode include PNG decoding (~100 ms); the real
   MCU cost is the core only: a few ms per frame.

