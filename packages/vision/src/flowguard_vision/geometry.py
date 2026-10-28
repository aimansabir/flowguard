"""Plane geometry for calibration: homography, polygons, line sides."""

import numpy as np
from numpy.typing import NDArray

Point = tuple[float, float]


def homography(pairs: list[tuple[Point, Point]]) -> NDArray[np.float64]:
    """3x3 matrix mapping the first point of each pair onto the second (DLT, 4+ pairs)."""
    rows: list[list[float]] = []
    for (x, y), (u, v) in pairs:
        rows.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        rows.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    h = np.linalg.svd(np.array(rows, dtype=np.float64))[2][-1].reshape(3, 3)
    return h / h[2, 2]


def apply(h: NDArray[np.float64], p: Point) -> Point:
    x, y, w = h @ np.array([p[0], p[1], 1.0])
    return float(x / w), float(y / w)


def in_polygon(p: Point, poly: list[Point]) -> bool:
    """Ray casting. Points exactly on an edge may fall either way."""
    x, y = p
    inside = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1], strict=True):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def side(line: list[Point], p: Point) -> float:
    """Signed area: positive on one side of the line a->b, negative on the other, 0 on it."""
    (ax, ay), (bx, by) = line
    return (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax)


def centroid(poly: list[Point]) -> Point:
    return sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly)
