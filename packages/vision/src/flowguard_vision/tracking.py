"""What the aggregation needs from a detector + tracker, and nothing more."""

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from flowguard_vision.geometry import Point


@dataclass(frozen=True)
class Frame:
    t: float  # seconds since the start of the clip
    gray: NDArray[np.uint8]


@dataclass(frozen=True)
class TrackObs:
    track_id: int
    cls: str  # one of flowguard_contracts.VEHICLE_CLASSES
    confidence: float
    foot: Point  # bottom-centre of the box, image px


class Tracker(Protocol):
    """Detector and tracker in one (ultralytics `track()` does both)."""

    def update(self, frame: Frame) -> list[TrackObs]: ...
