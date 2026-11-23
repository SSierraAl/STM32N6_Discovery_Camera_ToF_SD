# Desktop image contrast validation

The firmware and ToF pipeline are unchanged. This adjustment requires the
previous camera color decoding patch and the removal of automatic export
sharpening. Build or flash operations are not required.

## Controls and exports

The **Appearance** selector offers **Original contrast** and **Mild contrast**.
The latter is selected at startup. Both choices retain the existing corrected
color decoding and viewer white balance. Original contrast means no additional
tone curve, rather than unprocessed sensor RAW.

Preview, single-image save and batch extraction use the selected appearance.
The selection is fixed when a batch starts. Enhanced filenames include
`_contrast` before `.png`, keeping them separate from the normal exports.
**Save at Original Resolution** preserves the source dimensions; **Save at
Current Scale** deliberately resizes the output. Neither option writes back
to the SD image payload. Cached decoded images remain unchanged, so toggling
the selector does not accumulate processing.

For a comparison, select **100% (native)** and alternate the appearance.
The zoom and scroll position are retained. Compare the same object at the
same display size when comparing overview and full-resolution images.

## Curve and tuning

`VIEWER_CONTRAST_STRENGTH` in `viewer_image_quality.py` sets a fixed luminance
curve. The default is `0.5`; `0` disables it and values up to `1` are accepted.
For normalized luminance x, the curve is:

`f(x) = x + strength * x * (1-x) * (2*x-1)`.

The curve is smooth and monotonic over the supported range. It darkens values
below mid-gray, brightens those above it and preserves the black/white
endpoints. A luminance increment is added equally to RGB channels. Channel
differences are preserved except when an individual channel clips at 0 or
255; saturated subjects should therefore be checked before increasing the
strength. The curve does not normalize the histogram of each photo, preserving
a consistent mapping across insect images. It performs no spatial filtering,
sharpening, denoising or detail synthesis. Midtone noise may become slightly
more visible as contrast increases.

## Validation on the supplied original

The PNG inside `IMG_20261123_134029Z_000082_0082_full_5mp_multi.zip` is
2592 x 1944 pixels. On the text panel, using the native-image region
`x=1050:1700, y=100:1250`, the following values were measured with OpenCV
grayscale luminance. Dark-text samples are pixels below 100 in the original;
paper samples are original pixels in [155,180). The same masks were used
for both measurements. They describe this region, not a calibrated target.

| Measurement (8 bit) | Original contrast | Mild contrast |
| --- | ---: | ---: |
| Median dark-text value | 85 | 76 |
| Median paper value | 166 | 175 |
| Paper minus text | 81 | 99 |
| Pixels at grayscale 0 or 255, whole image | 0 | 0 |

The 22% increase in this luminance separation is a contrast measurement,
not evidence of increased optical resolution or restored fine detail.
The numeric transform took approximately 50 ms for 5 MP on the analysis host;
actual desktop display/export times depend on the computer. No MCU execution
time, buffer or ToF parameter is added.

## Sharpness limits and next acquisition check

The supplied picture still has soft letter edges. A single photograph cannot
separate focus error, object movement, printed-edge softness and ISP processing.
A global sharpness score would also respond to grain and cannot establish
whether this lens is correctly focused.

Before adding sharpening, place a stationary fine-print target at the actual
insect plane, keep illumination/exposure/gain fixed and compare small focus
adjustments at native scale. Inspect center and corners. Then repeat with
movement; if only the moving target becomes blurred, test a shorter exposure
and evaluate the resulting brightness/noise tradeoff. A tone curve improves
legibility but cannot recover missing optical or motion detail.

## Regression checks

Run `python -m unittest test_viewer_image_quality -v` in the repository root.
Checks use synthetic pixels and temporary PNG files. They cover monotonicity,
endpoint preservation, immutable source pixels, aligned QImage rows,
reversible GUI toggles, native-size saving and batch/preview consistency.
No physical SD device is accessed.
