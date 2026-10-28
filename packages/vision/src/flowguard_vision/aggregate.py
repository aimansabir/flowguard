"""Tracks -> one Measurement per second (docs/specs/vision.md V3, V5).

Conventions, all of which the Guard sees as camera behaviour:
- A vehicle is an arrival the first time its track is seen inside the zone.
  `calibration.entry_line` is not used: a track the tracker starts late
  inside the zone still counts, one that never enters does not.
- A departure is a stop-line crossing from the zone side. Its movement is
  decided when the track reaches a movement region, so it is reported in
  that second, possibly a few seconds after the crossing. A track lost
  between the line and a region is counted in `unresolved_departures`
  and never reported.
- A track not seen for `ALIVE_S` is not counted in the zone. A track seen
  again under a new id near where another just vanished is a track break.
"""

import math
import statistics
from collections import Counter, deque
from dataclasses import dataclass, field

from flowguard_contracts import VEHICLE_CLASSES, ImageHealth, Measurement
from flowguard_vision.calibration import Calibration
from flowguard_vision.geometry import Point, apply, centroid, homography, in_polygon, side
from flowguard_vision.tracking import TrackObs

ALIVE_S = 1.0  # unseen for longer than this: not in the zone count
FORGET_S = 2.0  # unseen for longer than this: track dropped
STOP_HOLD_S = 1.0  # stopped = speed below the threshold for this long
BREAK_RADIUS_M = 5.0


@dataclass
class _Track:
    votes: Counter[str] = field(default_factory=Counter[str])
    hist: deque[tuple[float, Point]] = field(default_factory=deque[tuple[float, Point]])  # ground metres
    last_seen: float = 0.0
    last_px: Point = (0.0, 0.0)
    arrived: bool = False
    side: float | None = None  # last stop-line side, image px
    pending: bool = False  # crossed the stop line, movement not known yet
    departed: bool = False
    broke: bool = False

    @property
    def cls(self) -> str:
        return self.votes.most_common(1)[0][0]


def _zero() -> dict[str, int]:
    return dict.fromkeys(VEHICLE_CLASSES, 0)


class Aggregator:
    def __init__(self, calib: Calibration, interval_s: float = 1.0, stop_speed_mps: float = 0.1):
        # stop_speed_mps is a calibration knob: foot-point jitter far from the
        # camera can exceed 0.1 m/s on a vehicle that is really stopped.
        self._calib = calib
        self._interval = interval_s
        self._stop_speed = stop_speed_mps
        self._h = homography(calib.px_to_m)
        self._approach_sign = math.copysign(1.0, side(calib.stop_line, centroid(calib.zone_polygon)))
        self._tracks: dict[int, _Track] = {}
        self._next_boundary = interval_s
        self.unresolved_departures = 0
        self._reset_interval()

    def _reset_interval(self) -> None:
        self._arrivals = _zero()
        self._departures = {m: _zero() for m in ("left", "straight", "right")}
        self._confidences: list[float] = []
        self._breaks = 0
        self._health: ImageHealth | None = None

    def update(self, t: float, obs: list[TrackObs], health: ImageHealth | None = None) -> list[Measurement]:
        """Feed one frame; returns the Measurements of every second this frame completes."""
        seen = {o.track_id for o in obs}
        for o in obs:
            self._observe(t, o, seen)
        self._sweep(t)
        if health is not None:
            self._health = health
        out: list[Measurement] = []
        while t >= self._next_boundary:
            out.append(self._emit(self._next_boundary, t))
            self._next_boundary += self._interval
        return out

    def _observe(self, t: float, o: TrackObs, seen: set[int]) -> None:
        c = self._calib
        self._confidences.append(o.confidence)
        g = apply(self._h, o.foot)
        tr = self._tracks.get(o.track_id)
        if tr is None:
            self._count_break(t, g, seen)
            tr = self._tracks[o.track_id] = _Track()
        tr.votes[o.cls] += 1
        tr.last_seen, tr.last_px = t, o.foot
        tr.hist.append((t, g))
        while len(tr.hist) > 1 and tr.hist[1][0] <= t - STOP_HOLD_S:
            tr.hist.popleft()

        if not tr.arrived and in_polygon(o.foot, c.zone_polygon):
            tr.arrived = True
            self._arrivals[tr.cls] += 1

        s = side(c.stop_line, o.foot)
        if tr.arrived and not tr.departed and not tr.pending and tr.side is not None:
            from_zone_side = math.copysign(1.0, tr.side) == self._approach_sign
            if from_zone_side and s * self._approach_sign <= 0:
                tr.pending = True
        tr.side = s

        if tr.pending:
            for movement, region in c.movement_regions.items():
                if in_polygon(o.foot, region):
                    self._departures[movement][tr.cls] += 1
                    tr.pending, tr.departed = False, True
                    break

    def _count_break(self, t: float, g: Point, seen: set[int]) -> None:
        """A new track starting next to one that just went quiet is probably the same vehicle."""
        for tid, old in self._tracks.items():
            if tid in seen or old.broke or old.departed or old.pending:
                continue
            if t - old.last_seen <= FORGET_S and math.dist(old.hist[-1][1], g) <= BREAK_RADIUS_M:
                old.broke = True
                self._breaks += 1
                return

    def _sweep(self, t: float) -> None:
        for tid in [k for k, tr in self._tracks.items() if t - tr.last_seen > FORGET_S]:
            if self._tracks[tid].pending:
                self.unresolved_departures += 1
            del self._tracks[tid]

    def _emit(self, boundary: float, now: float) -> Measurement:
        zone, queue = _zero(), _zero()
        for tr in self._tracks.values():
            if now - tr.last_seen > ALIVE_S or tr.departed or tr.pending:
                continue
            if not in_polygon(tr.last_px, self._calib.zone_polygon):
                continue
            zone[tr.cls] += 1
            (t0, p0), (t1, p1) = tr.hist[0], tr.hist[-1]
            if t1 - t0 >= STOP_HOLD_S and math.dist(p0, p1) < self._stop_speed * (t1 - t0):
                queue[tr.cls] += 1

        quality: dict[str, object] = {"track_breaks_per_min": self._breaks * 60.0 / self._interval}
        if self._confidences:
            quality["detector_confidence"] = {
                "mean": statistics.fmean(self._confidences),
                "std": statistics.pstdev(self._confidences),
            }
        if self._health is not None:
            quality["image_health"] = self._health.model_dump()
        m = Measurement.model_validate({
            "time": boundary,
            "interval_s": self._interval,
            "junction_id": self._calib.junction_id,
            "approach_id": self._calib.approach_id,
            "source": "camera",
            "zone_length_m": self._calib.zone_length_m,
            "zone_counts": zone,
            "queue_counts": queue,
            "arrivals": self._arrivals,
            "departures": self._departures,
            "quality": quality,
        })
        self._reset_interval()
        return m
