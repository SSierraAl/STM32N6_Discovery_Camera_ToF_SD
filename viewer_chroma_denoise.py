"""Optional desktop chroma noise reduction before the experimental matrix."""

import cv2
import numpy as np

# Blend in [0,1]: 0 bypasses filtering, 1 uses all filtered chroma. A partial
# blend limits the loss of very small features defined by color alone.
VIEWER_CHROMA_DENOISE_AMOUNT = 0.65
# Neighborhood diameter in pixels, odd and at least 3; do not increase this
# to hide luminance grain because this stage only filters color differences.
VIEWER_CHROMA_DENOISE_DIAMETER = 5
# Bilateral range scale in encoded 8-bit Y/Cb/Cr units and spatial scale in
# pixels. Larger values smooth more color variation and may blur color edges.
VIEWER_CHROMA_DENOISE_SIGMA_COLOR = 18.0
VIEWER_CHROMA_DENOISE_SIGMA_SPACE = 1.2


def apply_chroma_denoise(rgb, amount=VIEWER_CHROMA_DENOISE_AMOUNT):
    """Reduce chroma speckle while retaining the original encoded luminance.

    Args:
        rgb: uint8 RGB array (height, width, 3), never modified.
        amount: Finite blend in [0,1]; zero returns an identical copy.
    Returns:
        A new uint8 RGB image with unchanged dimensions. Clipping/rounding
        can cause small luminance differences after reconstruction.
    Raises:
        ValueError: Invalid RGB layout, blend or filter configuration.

    The joint bilateral guide contains Y, Cb and Cr. Differences in brightness
    or color therefore reduce mixing across edges. Only filtered Cb/Cr are
    used; Y is retained from the input. This is applied BEFORE the global
    chroma matrix, preventing the matrix from amplifying unsmoothed speckle.
    It does not sharpen, infer detail, or remove luminance grain. Features
    distinguished only by color may still be softened; keep a bypass for
    insect inspection. Strips include a halo so their seams are equivalent
    to full-image processing. All processing occurs on the PC.
    """
    diameter = VIEWER_CHROMA_DENOISE_DIAMETER
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3
            or not np.isfinite(amount) or not 0 <= amount <= 1
            or diameter < 3 or diameter % 2 != 1
            or not np.isfinite(VIEWER_CHROMA_DENOISE_SIGMA_COLOR)
            or VIEWER_CHROMA_DENOISE_SIGMA_COLOR <= 0
            or not np.isfinite(VIEWER_CHROMA_DENOISE_SIGMA_SPACE)
            or VIEWER_CHROMA_DENOISE_SIGMA_SPACE <= 0):
        raise ValueError("Expected uint8 RGB, blend in [0,1], and a valid bilateral configuration")
    if amount == 0 or rgb.size == 0:
        return rgb.copy()

    result = np.empty_like(rgb)
    halo = diameter // 2
    for start in range(0, rgb.shape[0], 128):
        stop = min(start + 128, rgb.shape[0])
        lo, hi = max(0, start - halo), min(rgb.shape[0], stop + halo)
        source = rgb[lo:hi].astype(np.float32)
        y = (0.2126 * source[:, :, 0] + 0.7152 * source[:, :, 1]
             + 0.0722 * source[:, :, 2])
        cb = (source[:, :, 2] - y) / 1.8556
        cr = (source[:, :, 0] - y) / 1.5748
        guide = np.dstack((y, cb, cr))
        filtered = cv2.bilateralFilter(
            guide, diameter, VIEWER_CHROMA_DENOISE_SIGMA_COLOR,
            VIEWER_CHROMA_DENOISE_SIGMA_SPACE, borderType=cv2.BORDER_REFLECT_101)
        cb += amount * (filtered[:, :, 1] - cb)
        cr += amount * (filtered[:, :, 2] - cr)
        red, blue = y + 1.5748 * cr, y + 1.8556 * cb
        green = (y - 0.2126 * red - 0.0722 * blue) / 0.7152
        reconstructed = np.dstack((red, green, blue))
        np.rint(reconstructed, out=reconstructed)
        np.clip(reconstructed, 0, 255, out=reconstructed)
        result[start:stop] = reconstructed[start-lo:stop-lo].astype(np.uint8)
    return result


def denoise_filename(filename, enabled):
    """Mark noise-reduced exports independently of contrast/color choices."""
    if not enabled:
        return filename
    stem, separator, extension = filename.rpartition(".")
    return f"{stem}_chroma_nr.{extension}" if separator else f"{filename}_chroma_nr"
