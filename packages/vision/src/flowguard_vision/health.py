"""Image-health signals from grayscale frames (2-D uint8 arrays), numpy only.

The thresholds are calibration knobs: they depend on the camera and the
compression of the footage, so tune them on real clips before trusting them.
"""

import math

import numpy as np
from numpy.typing import NDArray

from flowguard_contracts import ImageHealth

Gray = NDArray[np.uint8]

FROZEN_MEAN_DIFF = 0.5  # grey levels; below this consecutive frames count as identical
SHARP_LAPLACIAN_VAR = 100.0  # Laplacian variance at or above this is "sharp" (blur = 0)


def is_frozen(prev: Gray, cur: Gray) -> bool:
    return float(np.abs(prev.astype(np.int16) - cur.astype(np.int16)).mean()) < FROZEN_MEAN_DIFF


def blur(gray: Gray) -> float:
    """0 = sharp, 1 = unusable, from the variance of the 4-neighbour Laplacian."""
    g = gray.astype(np.float64)
    lap = g[:-2, 1:-1] + g[2:, 1:-1] + g[1:-1, :-2] + g[1:-1, 2:] - 4 * g[1:-1, 1:-1]
    return float(1.0 - min(1.0, lap.var() / SHARP_LAPLACIAN_VAR))


def brightness(gray: Gray) -> float:
    return float(gray.mean() / 255.0)


def displacement_px(ref: Gray, cur: Gray) -> float:
    """Translation of `cur` against `ref` in pixels, by phase correlation.

    ponytail: translation only and whole-pixel; rotation or zoom of a knocked
    camera reads as a smaller shift. Use ORB + homography if that matters.
    """
    if ref.shape != cur.shape:
        raise ValueError(f"frame shape {cur.shape} differs from reference {ref.shape}")
    cross = np.fft.fft2(ref.astype(np.float64)) * np.conj(np.fft.fft2(cur.astype(np.float64)))
    cross /= np.maximum(np.abs(cross), 1e-12)
    peak = np.unravel_index(np.argmax(np.fft.ifft2(cross).real), ref.shape)
    dy, dx = (int(i) if i <= n // 2 else int(i) - n for i, n in zip(peak, ref.shape, strict=True))
    return math.hypot(dy, dx)


def image_health(prev: Gray | None, cur: Gray, ref: Gray) -> ImageHealth:
    return ImageHealth(
        frozen=prev is not None and is_frozen(prev, cur),
        blur=blur(cur),
        brightness=brightness(cur),
        displacement_px=displacement_px(ref, cur),
    )
