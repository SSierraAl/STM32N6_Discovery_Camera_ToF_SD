# Desktop white balance and experimental color profile

This patch follows `massif-final-image-contrast.patch`. It changes the Python
viewer only. LED values, sensor exposure/gain, ISP firmware, ToF parameters,
baseline and SD image payloads are unchanged. No MCU build is required.

## White balance configuration

The active parameter is now `VIEWER_COLOR_WB_STRENGTH = 1.0` in
`viewer_color_profile.py`. This restores the purple compensation even when
the earlier diagnostic `VIEWER_WHITE_BALANCE_STRENGTH` in the main viewer was
left at zero. The old setting is retained for patch compatibility but is no
longer used by decoding. Future white balance changes belong in the new file.
The default does not change the existing white-balance algorithm.

## Optional color comparison

The **Color profile** selector starts at **Current color (white balance
restored)**. The second option applies an experimental global RGB matrix
after white balance and before the optional contrast curve:

| Output channel | Computation |
| --- | --- |
| R | R |
| G | G + k(G-B) |
| B | B - k(G-B) |

`VIEWER_EXPERIMENTAL_CHROMA_STRENGTH = 0.35` defines k. It accepts 0..0.5;
zero disables the matrix. Each matrix row sums to one, preserving exact
neutral grays. It does not identify or selectively recolor orange objects.

The candidate increases green-blue chroma separation, moving the observed
yellow cable toward the phone reference. It can also shift greens and blues,
deepen magenta reds and clip saturated channels. It is therefore opt-in and
must not be described as a calibrated camera profile. A reference color chart
under the actual ring is needed to fit and validate a general correction.
The phone image is a visual comparison; its RGB values are not an absolute
colorimetric reference. Existing photos establish the hue issue but do not
validate color fidelity across insect species or lighting conditions.

For illustration, an RGB sample [140,87,42] becomes [140,103,26] before the
optional tone curve. An exact neutral [176,176,176] remains unchanged. These
are arithmetic examples, not a promise that every yellow object is corrected.
Applying the candidate to the supplied already-rendered cable PNGs brought
approximately 0.112% (green/yellow) and 0.055% (black/red) of channel samples
to a new 0/255 boundary. These exploratory figures include prior contrast;
the real viewer applies the matrix before contrast and its results differ.

## Preview and export

At native zoom, alternate **Current color** and **Experimental color** while
checking yellow, green, blue, red, black and white. The cached original stays
unchanged, so repeated switches never accumulate corrections. Contrast is an
independent choice. Export and batch extraction use the selected combination.
Batch settings are captured when the worker starts.

Candidate exports contain `_color_trial` before `.png`; with contrast they
contain `_contrast_color_trial`. Existing corrected exports retain their
normal names. Original-size export keeps the source pixel dimensions.
No sharpening, denoising or synthesized detail is added.

All processing runs on the PC using short strips to limit temporary memory.
It adds no MCU detection, capture or storage latency.

## Regression verification

Run:

```powershell
python -m unittest test_viewer_image_quality test_viewer_color_profile -v
```

The tests cover neutral preservation, input immutability, numeric bounds,
restoration after a legacy zero-WB setting, default opt-in state, reversible
GUI selection, 5 MP native-size export and matching preview/batch processing
for every contrast/color combination. No SD device is opened.
