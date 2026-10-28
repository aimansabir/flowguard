"""Frames -> Measurements -> JSONL."""

from collections.abc import Iterable, Iterator
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from flowguard_contracts import Measurement
from flowguard_vision.aggregate import Aggregator
from flowguard_vision.calibration import Calibration
from flowguard_vision.health import image_health
from flowguard_vision.tracking import Frame, Tracker


def measurements(
    frames: Iterable[Frame],
    tracker: Tracker,
    calib: Calibration,
    reference: NDArray[np.uint8],
) -> Iterator[Measurement]:
    """One Measurement per second of footage. Deterministic given a deterministic tracker."""
    agg = Aggregator(calib)
    prev: NDArray[np.uint8] | None = None
    for f in frames:
        yield from agg.update(f.t, tracker.update(f), image_health(prev, f.gray, reference))
        prev = f.gray


def write_jsonl(ms: Iterable[Measurement], path: Path) -> None:
    with path.open("w", encoding="utf-8") as out:
        for m in ms:
            out.write(m.model_dump_json() + "\n")
