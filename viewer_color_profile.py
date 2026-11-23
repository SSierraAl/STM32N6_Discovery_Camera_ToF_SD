"""Desktop color controls; firmware, LED color and ToF are unchanged.

The experimental matrix is a visual tuning candidate, not a measured camera
calibration. A color chart under the actual ring is required for that claim.
"""

import numpy as np

# Centralized here so an earlier 0.0 diagnostic setting in the viewer cannot
# accidentally leave the normal preview purple. Range: 0..1.
VIEWER_COLOR_WB_STRENGTH = 1.0

# Dimensionless coefficient k in G'=G+k(G-B), B'=B-k(G-B), R'=R.
# This expands green-blue separation globally. Every row sums to one, so
# neutral grays stay neutral. It is NOT a selective orange-to-yellow recolor.
# k=0 disables the effect. Keep this opt-in until other colors are validated.
VIEWER_EXPERIMENTAL_CHROMA_STRENGTH = 0.40


def apply_experimental_color(rgb, strength=VIEWER_EXPERIMENTAL_CHROMA_STRENGTH):
    """Apply an optional global RGB matrix after desktop white balance.

    Args:
        rgb: uint8 array (height, width, 3), RGB order, never mutated.
        strength: Finite coefficient in [0, 0.5]; zero returns a copy.
    Returns:
        A new uint8 array with the same shape. Intermediate negative or
        above-range channels are clipped to 0/255. Neutral pixels remain
        identical. No spatial processing or detail synthesis occurs.
    Raises:
        ValueError: Unsupported input or coefficient.

    This matrix is a provisional way to evaluate the yellow displacement
    seen in the supplied cable photos. It can also shift green/cyan/blue,
    deepen magenta reds and clip saturated channels. Do not label its exports
    as color-calibrated or use them as quantitative insect color references.
    Strip processing bounds temporary memory; all work runs on the PC.
    """
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3
            or not np.isfinite(strength) or not 0 <= strength <= 0.5):
        raise ValueError("Expected uint8 RGB and strength in [0, 0.5]")
    result = np.empty_like(rgb)
    for start in range(0, rgb.shape[0], 128):
        strip = rgb[start:start + 128].astype(np.float32)
        delta = strength * (strip[:, :, 1] - strip[:, :, 2])
        strip[:, :, 1] += delta
        strip[:, :, 2] -= delta
        np.rint(strip, out=strip)
        np.clip(strip, 0, 255, out=strip)
        result[start:start + strip.shape[0]] = strip
    return result


def color_filename(filename, enabled):
    """Append _color_trial to a candidate export, preserving its extension."""
    if not enabled:
        return filename
    stem, separator, extension = filename.rpartition(".")
    return f"{stem}_color_trial.{extension}" if separator else f"{filename}_color_trial"
