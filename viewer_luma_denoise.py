"""Optional desktop luminance grain reduction and visible filter diagnostics."""

import cv2
import numpy as np
import viewer_chroma_denoise as chroma

# Mix in [0,1]; 0 bypasses luminance filtering. Start conservatively because
# low-contrast insect texture and grain can have similar spatial scales.
VIEWER_LUMA_DENOISE_AMOUNT = 0.5
# Odd diameter in pixels. Sigma-space, not diameter alone, controls how far
# neighbors contribute. Large diameters with small sigma-space mostly add cost.
VIEWER_LUMA_DENOISE_DIAMETER = 5
VIEWER_LUMA_DENOISE_SIGMA_SPACE = 1.2
# Range scale and maximum full-filter adjustment, in encoded 8-bit luma units.
# Edge differences much greater than sigma-color resist averaging. The cap
# limits the change to any pixel; neither mechanism guarantees no detail loss.
VIEWER_LUMA_DENOISE_SIGMA_COLOR = 8.0
VIEWER_LUMA_DENOISE_MAX_DELTA = 6.0
# Logs loaded configuration and measured pixel change for each render/export.
VIEWER_NOISE_DIAGNOSTICS = True


def apply_luma_denoise(rgb, amount=None):
    """Smooth small luminance fluctuations while keeping channel differences.

    Args:
        rgb: uint8 RGB array (height,width,3), never modified.
        amount: Blend in [0,1], or None to read the current module setting.
    Returns:
        New RGB image with the same dimensions. The same bounded luminance
        delta is added to R/G/B; channel differences remain unchanged except
        at 0/255 clipping. This does not synthesize or sharpen image detail.
    Raises:
        ValueError: Invalid input or filter configuration.

    A small bilateral neighborhood resists mixing across strong brightness
    edges. Weak, fine texture can still be smoothed: retain an Off mode for
    inspection. Strip halos prevent seams. All work occurs on the PC.
    """
    if amount is None:
        amount = VIEWER_LUMA_DENOISE_AMOUNT
    d = VIEWER_LUMA_DENOISE_DIAMETER
    if (rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3
            or not np.isfinite(amount) or not 0 <= amount <= 1
            or d < 3 or d % 2 != 1
            or any(not np.isfinite(v) or v <= 0 for v in (
                VIEWER_LUMA_DENOISE_SIGMA_SPACE, VIEWER_LUMA_DENOISE_SIGMA_COLOR,
                VIEWER_LUMA_DENOISE_MAX_DELTA))):
        raise ValueError("Invalid RGB layout, luminance amount or filter configuration")
    if amount == 0 or rgb.size == 0:
        return rgb.copy()
    result = np.empty_like(rgb)
    halo = d // 2
    for start in range(0, rgb.shape[0], 128):
        stop = min(start + 128, rgb.shape[0])
        lo, hi = max(0, start-halo), min(rgb.shape[0], stop+halo)
        src = rgb[lo:hi].astype(np.float32)
        y = .2126*src[:,:,0] + .7152*src[:,:,1] + .0722*src[:,:,2]
        filtered = cv2.bilateralFilter(y, d, VIEWER_LUMA_DENOISE_SIGMA_COLOR,
            VIEWER_LUMA_DENOISE_SIGMA_SPACE, borderType=cv2.BORDER_REFLECT_101)
        delta = amount * np.clip(filtered-y, -VIEWER_LUMA_DENOISE_MAX_DELTA,
                                 VIEWER_LUMA_DENOISE_MAX_DELTA)
        src += delta[:,:,None]
        np.rint(src, out=src)
        np.clip(src, 0, 255, out=src)
        result[start:stop] = src[start-lo:stop-lo].astype(np.uint8)
    return result


def noise_configuration(mode):
    """Describe effective settings loaded by this Python process.

    Args:
        mode: False/0=Off, True/1=color only, 2=color plus luminance.
    Returns:
        A compact string for the GUI and terminal. File edits require a
        restart to reload modules; the string makes stale settings visible.
    """
    if not mode:
        return "NR Off"
    text = (f"Chroma: amount={chroma.VIEWER_CHROMA_DENOISE_AMOUNT:g}, "
            f"diameter={chroma.VIEWER_CHROMA_DENOISE_DIAMETER}, "
            f"space={chroma.VIEWER_CHROMA_DENOISE_SIGMA_SPACE:g}, "
            f"range={chroma.VIEWER_CHROMA_DENOISE_SIGMA_COLOR:g}")
    if mode == 2:
        text += (f" | Luma: amount={VIEWER_LUMA_DENOISE_AMOUNT:g}, "
                 f"diameter={VIEWER_LUMA_DENOISE_DIAMETER}, "
                 f"space={VIEWER_LUMA_DENOISE_SIGMA_SPACE:g}, "
                 f"range={VIEWER_LUMA_DENOISE_SIGMA_COLOR:g}")
    return text


def log_noise_change(original, filtered, mode):
    """Log a sampled measured change; zero exposes a bypass/no-op explicitly."""
    if not VIEWER_NOISE_DIAGNOSTICS:
        return
    difference = np.abs(original[::8,::8].astype(np.int16)-filtered[::8,::8].astype(np.int16))
    mean = float(difference.mean()) if difference.size else 0.0
    fraction = float(np.any(difference > 0, axis=2).mean()) if difference.size else 0.0
    print(f"[VIEWER][NR] {noise_configuration(mode)} | "
          f"sample_mean_abs_change={mean:.3f}/255 changed_pixels={100*fraction:.1f}%", flush=True)


def noise_filename(filename, mode):
    """Keep previous color-only names; distinguish color+luminance exports."""
    if mode != 2:
        return chroma.denoise_filename(filename, mode)
    stem, separator, extension = filename.rpartition(".")
    return f"{stem}_color_luma_nr.{extension}" if separator else f"{filename}_color_luma_nr"
