# Simplified snapshot viewer controls

The viewer applies the experimental color profile to previews, individual
PNG saves and batch extraction. Its parameters remain in
`viewer_color_profile.py`, including the pushed chroma strength of 0.40 and
white balance strength of 1.0. All other pushed processing parameters are
preserved, including the luminance denoise amount of 0.25.

The image controls contain only two selectors:

| Selector | Initial value | Available choices |
| --- | --- | --- |
| Appearance | Original contrast | Original contrast; Mild contrast |
| Noise reduction | Off | Off; Color + luminance grain reduction |

The color-profile selector and intermediate chroma-only noise choice are
removed. The underlying processing helpers still support explicit bypasses
for regression tests, but the GUI always uses the accepted experimental
profile. It remains a visually tuned profile rather than a colorimetric
calibration. Existing `_color_trial` export labels are retained for continuity.

The image-scale radio buttons and **Save at Current Scale** action are
removed. The viewer still fits a newly opened image to its window, supports
mouse-wheel zoom and supports dragging for inspection. Toggling contrast or
noise retains the current zoom and scroll position. There are no scale
controls or numeric filter settings in the panel; filter diagnostics remain
available in terminal logs.

**Save at Original Resolution** always preserves the source pixel dimensions,
regardless of the zoom on screen. It uses the currently selected contrast and
noise settings. Batch extraction uses the same fixed color profile and
captures the other selected settings when the batch starts. Exports retain
the existing category, time, contrast and noise filename conventions.

No firmware, LED color/brightness, exposure, gain, ToF algorithm, baseline or
SD payload is changed. This patch requires restarting the Python viewer only.

Regression verification checks removed-widget references in both cached and
fresh image paths, initial settings, reversible rendering, preview/export
consistency and native 2592x1944 export even with a different screen zoom.
