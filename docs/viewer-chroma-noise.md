# Chroma noise reduction before color correction

This patch follows `massif-viewer-color-profile.patch`. It changes only the
desktop viewer. Firmware capture, exposure, gain, LED color/brightness, ToF
baseline and SD data are unchanged. No firmware build is needed.

## Why the grain becomes more visible

The experimental matrix computes G'=G+k(G-B), B'=B-k(G-B). Consequently,
G'-B'=(1+2k)(G-B). At k=0.40 it multiplies the difference, including its
noise, by 1.8 before rounding or clipping. It does not introduce a sharpening
kernel. The separate contrast curve can further increase visible grain.

On the supplied exports, in a flat background region x=15..30% of width,
y=30..50% of height, the standard deviation of a 3x3-Gaussian high-pass
residual in G-B was 3.24 with no experimental color, 5.82 with experimental
color and 5.82 with experimental color plus contrast. The corresponding
encoded-luminance residuals were 2.07, 2.78 and 3.28. These are local texture/
noise indicators, not calibrated sensor noise measurements.

## Processing and controls

The order is corrected YUYV decode, desktop white balance, optional chroma
noise reduction, optional color matrix, optional contrast, display/export.
The cached decoded image is never changed.

The initial UI choices are **Original contrast**, **Experimental color** and
**Mild chroma reduction**. The existing chroma coefficient in
`viewer_color_profile.py` is deliberately preserved: keep the preferred
`VIEWER_EXPERIMENTAL_CHROMA_STRENGTH = 0.40` there.

The new **Color noise** selector independently enables or disables smoothing.
Compare at native zoom, keeping color and contrast unchanged. Export names
include `_chroma_nr` when smoothing is enabled. Preview, single save and batch
extraction use the same selected pipeline. A batch freezes its settings on
start. Original-size export preserves the source dimensions.

## Tunable parameters

All new filter parameters are in `viewer_chroma_denoise.py`:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| VIEWER_CHROMA_DENOISE_AMOUNT | 0.65 | Filtered-chroma blend, 0..1; zero bypasses |
| VIEWER_CHROMA_DENOISE_DIAMETER | 5 | Odd spatial neighborhood diameter, pixels |
| VIEWER_CHROMA_DENOISE_SIGMA_COLOR | 18.0 | Positive bilateral range scale, encoded Y/Cb/Cr units |
| VIEWER_CHROMA_DENOISE_SIGMA_SPACE | 1.2 | Positive spatial scale, pixels |

The filter separates encoded RGB into BT.709-like Y/Cb/Cr coordinates and
performs a joint bilateral filter guided by all three components. Only its
chroma result is used. Original Y is retained to preserve luminance detail;
RGB rounding and gamut clipping can still change it slightly. A partial
chroma blend limits the effect. Strips carry a halo to avoid seams.

There is no spatial sharpening, detail synthesis or luminance smoothing.
This reduces color speckle; it does not remove all brightness grain or restore
blurred detail. Very small features defined only by color can be softened.
Strong color/brightness edges resist mixing but edge preservation is not
absolute. The unfiltered source remains available for inspecting insects.

## Measured check on the supplied image

Applying k=0.40 to the supplied no-contrast PNG gave a G-B residual deviation
of 5.858 in the background region. Filtering chroma first reduced it to
3.282, about 44% below the unfiltered corrected result. The corresponding
Y residual changed from 2.786 to 2.468. This is a local measurement, not a
44% improvement in overall image quality. Before the color matrix, median
absolute luminance change from the filter was 0.207 on the 0..255 scale.

The filter took about 100 ms for a synthetic 2592x1944 frame on the analysis
host. Desktop performance varies; no MCU processing or inference time is
added. Memory is bounded by a strip plus halo and the RGB output frame.

## Tests

```powershell
python -m unittest test_viewer_image_quality test_viewer_color_profile test_viewer_chroma_denoise -v
```

Twelve tests cover prior conversion/export behavior and the new synthetic
chroma noise reduction, luminance retention away from clipping, color-edge
behavior, thin luminance detail, strip boundaries, reversible UI toggles and
all eight color/contrast/noise batch combinations. No SD device is opened.
