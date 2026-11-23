# Visible grain reduction and loaded-parameter diagnostics

Apply after `massif-viewer-chroma-noise.patch`. This is a desktop-only patch:
no LED, exposure, gain, ISP, ToF, baseline or SD payload is changed. The
existing color strength (including the preferred 0.40) is retained.

## Why changing the old parameters could appear ineffective

The old filter intentionally retained luminance Y. It reduced colored speckle,
but could not smooth brightness grain. Its amount is a blend with a filtered
result, not a noise threshold. Increasing it cannot remove noise that the
underlying filter deliberately preserves.

The spatial sigma was 1.2 pixels. At distance r the bilateral spatial weight
is proportional to exp(-r*r/(2*sigma*sigma)); distant neighbors contribute
little even if the nominal diameter is increased to 21. A bigger neighborhood
mainly adds cost once it extends beyond a few sigma. A diameter of 5 is the
intended starting point. Avoid using diameter alone as a strength control.

Parameter files are imported by Python. Close the viewer fully and relaunch
after editing them. The GUI now shows the loaded amount, diameter, spatial
sigma and range sigma. If it displays old values, that process has not loaded
the edited file/settings. Chroma's amount is now read at call time instead
of being bound as a Python function default, supporting explicit runtime
overrides in diagnostics/tests; this does not automatically reload files.

## New mode

**Noise reduction** has three choices:

1. **Off** retains original grain.
2. **Mild chroma reduction** runs the previous color-only filter.
3. **Color + luminance grain reduction** also smooths small brightness changes.

The third choice is selected initially. Original contrast and the experimental
color selection remain the preferred comparison setup. Chroma smoothing runs
first, luminance smoothing second, the experimental matrix third and contrast
last. Cached decoded pixels remain unchanged. Every export path shares this
pipeline; the batch freezes its settings when starting.

New exports use `_color_luma_nr`, distinct from the previous `_chroma_nr`.
Original-resolution saves retain the source dimensions. Toggle with native
zoom for comparison; fit-to-window scaling may mask small changes.

## Luminance controls

The new settings are in `viewer_luma_denoise.py`:

| Setting | Default | Purpose |
| --- | ---: | --- |
| VIEWER_LUMA_DENOISE_AMOUNT | 0.55 | Blend, range 0..1; zero bypasses |
| VIEWER_LUMA_DENOISE_DIAMETER | 5 | Odd neighborhood diameter in pixels |
| VIEWER_LUMA_DENOISE_SIGMA_SPACE | 1.2 | Spatial scale in pixels |
| VIEWER_LUMA_DENOISE_SIGMA_COLOR | 8.0 | Brightness similarity scale, 8-bit units |
| VIEWER_LUMA_DENOISE_MAX_DELTA | 6.0 | Full-filter adjustment cap, 8-bit units |
| VIEWER_NOISE_DIAGNOSTICS | True | Print settings and measured sampled changes |

The luminance delta is added equally to R/G/B, preserving channel differences
except at clipping. At the defaults the maximum pre-rounding change is
0.55*6=3.3 levels per channel. Strong edges resist mixing, but fine weak
texture can soften. Retain Off for checking real insect detail. No sharpening
or synthesized detail is added; the filter does not restore focus or motion
detail. Strip halos match whole-image filtering without seams.

Terminal messages begin `[VIEWER][NR]`. They report loaded settings, sampled
mean absolute RGB change and percentage of changed sampled pixels BEFORE the
color matrix. A zero change is explicit; it can be normal for a flat noiseless
image or indicate that there is nothing this stage can change. The diagnostic
is a measured pixel difference, not a perceptual quality score.

## Check on supplied data

Using the previously supplied no-contrast PNG, k=0.40 and the same background
region x=15..30%, y=30..50%, the standard deviation of the encoded-luminance
3x3-Gaussian high-pass residual fell from 2.468 with chroma-only processing
to 1.692 with the new luminance stage, about 31%. This is a local texture/
grain indicator, not a sensor noise specification or an overall quality gain.
The two filters together took about 178 ms for the supplied 2048x1536 image
on the analysis host, with the original default chroma diameter of 5. Larger
user-configured diameters cost more. No MCU processing time is added.

## Verification

```powershell
python -m unittest test_viewer_image_quality test_viewer_color_profile test_viewer_chroma_denoise test_viewer_luma_denoise -v
```

Tests cover measured synthetic luminance-noise reduction, neutral colors,
bounded pixel changes, strong color edges, a thin dark line, exact strip/full
filter equivalence, loaded settings, terminal diagnostics, mode-2 export
combinations, reversible UI switching and 2592x1944 saving. No SD device opens.
