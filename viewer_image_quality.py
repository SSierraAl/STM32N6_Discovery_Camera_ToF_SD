"""Reversible presentation contrast for the MASSIF desktop viewer.

This module never changes the sensor, firmware detector, or SD payload.
"""

import numpy as np


# 0 disables the curve; 1 is the strongest supported monotonic curve.
# 0.5 gives a modest midtone contrast increase without hard black/white cuts.
VIEWER_CONTRAST_STRENGTH = 0.5


def apply_mild_contrast(rgb, strength=VIEWER_CONTRAST_STRENGTH):
    """Apply a fixed, smooth S-curve to luminance without spatial filtering.

    Args:
        rgb: uint8 RGB array of shape (height, width, 3), left unchanged.
        strength: Curve strength in [0, 1]; zero returns an identical copy.

    Returns:
        A new uint8 RGB array with the same dimensions. Black and white
        endpoints remain fixed. Values below mid-gray become slightly darker
        and values above mid-gray slightly brighter.

    Raises:
        ValueError: Input layout, dtype or strength is invalid.

    A fixed curve keeps consecutive photos comparable. Histogram stretching
    per photo would make the appearance depend on the insects in the frame.
    No sharpening or denoising is applied: neither missing focus detail nor
    motion detail can be reconstructed here. Equal luminance increments are
    added to each RGB channel, preserving channel differences except where a
    channel reaches its 0/255 boundary. Temporary memory is bounded by strips;
    processing occurs on the PC and adds no firmware inference/capture time.
    """
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3
            or not np.isfinite(strength) or not 0 <= strength <= 1):
        raise ValueError("Expected uint8 RGB and a finite strength in [0, 1]")
    if strength == 0:
        return rgb.copy()

    x = np.arange(256, dtype=np.float32) / 255.0
    # Derivative is nonnegative for 0 <= strength <= 1. Unlike aggressive
    # levels/thresholds, this curve does not collapse a range of dark values.
    curve = x + strength * x * (1.0 - x) * (2.0 * x - 1.0)
    delta = np.rint(255.0 * curve).astype(np.int16) - np.arange(256, dtype=np.int16)
    result = np.empty_like(rgb)
    for start in range(0, rgb.shape[0], 128):
        strip = rgb[start:start + 128].astype(np.int16)
        # Integer approximation of BT.709 luminance; weights sum to 256.
        wide = strip.astype(np.int32)
        luma = ((54 * wide[:, :, 0] + 183 * wide[:, :, 1]
                 + 19 * wide[:, :, 2] + 128) >> 8).astype(np.uint8)
        adjusted = strip + delta[luma, None]
        result[start:start + strip.shape[0]] = np.clip(adjusted, 0, 255).astype(np.uint8)
    return result


def quality_filename(filename, enabled):
    """Mark enhanced PNG exports while preserving the existing image label.

    Args:
        filename: Existing export filename, including its extension.
        enabled: Whether presentation contrast was applied to this export.
    Returns:
        The unchanged filename or a filename ending in ``_contrast.png``.
    """
    if not enabled:
        return filename
    stem, separator, extension = filename.rpartition(".")
    return f"{stem}_contrast.{extension}" if separator else f"{filename}_contrast"
