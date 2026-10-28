"""Vision aggregation, geometry and image health on synthetic tracks (no footage needed).

Synthetic approach: image 700 x 100 px, ground = px / 10 (so 10 px = 1 m).
Vehicles drive along +x; the zone is x 0..600 (60 m), the stop line is x = 600,
and the movement regions sit beyond it split in three bands of y.
"""

import json
from collections.abc import Callable
from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from flowguard_contracts import ClassCounts, Corridor, Measurement, StepOutput
from flowguard_core import Core
from flowguard_sim.run import core_config
from flowguard_vision.aggregate import Aggregator
from flowguard_vision.calibration import Calibration
from flowguard_vision.geometry import apply, homography, in_polygon, side
from flowguard_vision.health import blur, brightness, displacement_px, image_health, is_frozen
from flowguard_vision.run import measurements, write_jsonl
from flowguard_vision.stub import empty_measurement
from flowguard_vision.tracking import Frame, TrackObs

FPS = 10


def calib(junction: str = "J", approach: str = "a") -> Calibration:
    zone = [(0.0, 0.0), (600.0, 0.0), (600.0, 100.0), (0.0, 100.0)]
    corners = [(0.0, 0.0), (700.0, 0.0), (700.0, 100.0), (0.0, 100.0)]
    return Calibration.model_validate({
        "junction_id": junction,
        "approach_id": approach,
        "zone_polygon": zone,
        "zone_length_m": 60,
        "entry_line": [(0, 0), (0, 100)],
        "stop_line": [(600, 0), (600, 100)],
        "movement_regions": {
            "left": [(600, 0), (700, 0), (700, 33), (600, 33)],
            "straight": [(600, 33), (700, 33), (700, 66), (600, 66)],
            "right": [(600, 66), (700, 66), (700, 100), (600, 100)],
        },
        "reference_frame": "ref.png",
        "px_to_m": [(p, (p[0] / 10, p[1] / 10)) for p in corners],
    })


def obs(tid: int, x: float, y: float = 50.0, cls: str = "car", conf: float = 0.9) -> TrackObs:
    return TrackObs(tid, cls, conf, (x, y))


def run(agg: Aggregator, seconds: float, per_frame: Callable[[float], list[TrackObs]]) -> list[Measurement]:
    out: list[Measurement] = []
    for k in range(int(seconds * FPS) + 1):
        t = k / FPS
        out += agg.update(t, per_frame(t))
    return out


def total(counts: ClassCounts) -> int:
    return sum(counts.model_dump().values())


def test_homography_maps_the_calibration_points() -> None:
    pairs = [
        ((0.0, 0.0), (1.0, 2.0)), ((10.0, 0.0), (6.0, 2.0)),
        ((10.0, 10.0), (6.0, 7.0)), ((0.0, 10.0), (1.0, 7.0)),
    ]
    h = homography(pairs)
    for src, dst in pairs:
        assert apply(h, src) == pytest.approx(dst)


def test_polygon_and_side() -> None:
    sq = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    assert in_polygon((5, 5), sq) and not in_polygon((15, 5), sq) and not in_polygon((5, -1), sq)
    line = [(0.0, 0.0), (0.0, 10.0)]
    assert side(line, (1, 5)) * side(line, (-1, 5)) < 0 and side(line, (0, 5)) == 0


def test_car_through_the_zone_balances_arrivals_and_departures() -> None:
    # 6 m/s = 60 px/s: enters at x=5, crosses the stop line between t=9.9 and 10.0
    ms = run(Aggregator(calib()), 14, lambda t: [obs(1, 5 + 60 * t)] if t <= 13 else [])
    assert [m.time for m in ms] == [float(s) for s in range(1, 15)]
    assert ms[0].arrivals.car == 1 and ms[0].zone_counts.car == 1
    assert all(m.zone_counts.car == 1 and total(m.arrivals) == 0 for m in ms[1:9])
    assert all(total(m.queue_counts) == 0 for m in ms)  # never stopped
    assert ms[9].departures.straight.car == 1 and ms[9].zone_counts.car == 0
    assert sum(total(m.arrivals) for m in ms) == sum(total(m.departures.straight) for m in ms) == 1
    assert all(m.source == "camera" and m.quality is not None for m in ms)


@pytest.mark.parametrize(("y", "movement"), [(10, "left"), (50, "straight"), (90, "right")])
def test_departure_movement_comes_from_the_region_reached(y: float, movement: str) -> None:
    ms = run(Aggregator(calib()), 12, lambda t: [obs(1, 5 + 60 * t, y)])
    got = {m: sum(getattr(mm.departures, m).car for mm in ms) for m in ("left", "straight", "right")}
    assert got == {m: int(m == movement) for m in got}


def test_departure_is_reported_when_the_movement_becomes_known() -> None:
    # crosses the line at t=10 outside every region, then is seen in the right region at t=11
    def frames(t: float) -> list[TrackObs]:
        if t <= 9.9:
            return [obs(1, 5 + 60 * t, 90)]
        if t == 10.0:
            return [obs(1, 650, 150)]  # beyond the line, outside every region
        return [obs(1, 650, 90)] if t >= 11 else []

    ms = run(Aggregator(calib()), 13, frames)
    assert ms[9].departures.right.car == 0 and total(ms[9].zone_counts) == 0  # crossed, not yet attributed
    assert ms[10].departures.right.car == 1


def test_track_lost_after_the_line_is_unresolved_not_reported() -> None:
    agg = Aggregator(calib())
    ms = run(agg, 15, lambda t: [obs(1, 5 + 60 * t, 50 if t < 10 else 150)] if t <= 10.0 else [])
    departed = [sum(total(getattr(m.departures, k)) for k in ("left", "straight", "right")) for m in ms]
    assert sum(departed) == 0
    assert agg.unresolved_departures == 1


def test_stopped_vehicle_is_queued_after_one_second_and_a_slow_one_is_not() -> None:
    ms = run(Aggregator(calib()), 5, lambda t: [obs(1, 300, 20), obs(2, 100 + 5 * t, 80, "truck")])
    assert ms[0].zone_counts.car == 1 and ms[0].zone_counts.truck == 1
    assert [m.queue_counts.car for m in ms[1:]] == [1, 1, 1, 1]
    assert all(m.queue_counts.truck == 0 for m in ms)  # 0.5 m/s


def test_track_break_is_counted_once_and_class_is_a_vote() -> None:
    def frames(t: float) -> list[TrackObs]:
        if t < 3:
            return [obs(1, 300, cls="car" if t < 2 else "truck")]
        return [obs(2, 302, cls="car")] if t >= 3.5 else []

    ms = run(Aggregator(calib()), 6, frames)
    assert [m.quality.track_breaks_per_min for m in ms if m.quality] == [0, 0, 0, 60, 0, 0]
    assert sum(total(m.arrivals) for m in ms) == 2  # a break double counts the arrival: camera behaviour
    assert ms[1].zone_counts.car == 1 and ms[1].zone_counts.truck == 0  # 20 car votes beat none


def test_detector_confidence_statistics() -> None:
    ms = run(Aggregator(calib()), 1, lambda t: [obs(1, 300, conf=0.5), obs(2, 200, conf=0.9)])
    assert ms[0].quality is not None
    dc = ms[0].quality.detector_confidence
    assert dc is not None
    assert dc.mean == pytest.approx(0.7) and dc.std == pytest.approx(0.2)


def test_a_gap_in_frames_still_emits_every_second() -> None:
    agg = Aggregator(calib())
    assert agg.update(0.0, [obs(1, 300)]) == []
    assert [m.time for m in agg.update(3.5, [])] == [1.0, 2.0, 3.0]


def test_vehicle_never_in_the_zone_is_never_counted() -> None:
    ms = run(Aggregator(calib()), 3, lambda t: [obs(1, 650, 50)])
    assert all(total(m.zone_counts) == total(m.arrivals) == 0 for m in ms)


def noise(seed: int = 0) -> NDArray[np.uint8]:
    return np.random.default_rng(seed).integers(0, 256, (64, 64), dtype=np.uint8)


def test_health_signals() -> None:
    a = noise()
    assert is_frozen(a, a.copy()) and not is_frozen(a, noise(1))
    flat = np.full((64, 64), 128, np.uint8)
    assert blur(flat) == 1.0 and blur(a) == 0.0
    assert brightness(np.zeros((8, 8), np.uint8)) == 0.0 and brightness(np.full((8, 8), 255, np.uint8)) == 1.0
    assert displacement_px(a, a) == 0.0
    assert displacement_px(a, np.roll(a, (3, 4), axis=(0, 1))) == pytest.approx(5.0)
    with pytest.raises(ValueError):
        displacement_px(a, a[:32])
    h = image_health(a, a, a)
    assert h.frozen and h.displacement_px == 0.0 and not image_health(None, a, a).frozen


class Script:
    """A Tracker that replays fixed observations."""

    def __init__(self, f: Callable[[float], list[TrackObs]]) -> None:
        self.f = f

    def update(self, frame: Frame) -> list[TrackObs]:
        return self.f(frame.t)


def test_pipeline_writes_validated_jsonl_deterministically(tmp_path: Path) -> None:
    ref = noise()
    frames = [Frame(k / FPS, ref) for k in range(41)]
    tracker = Script(lambda t: [obs(1, 300)])
    first = list(measurements(frames, tracker, calib(), ref))
    second = list(measurements(frames, tracker, calib(), ref))
    assert first == second and len(first) == 4
    assert all(m.quality.image_health.frozen for m in first)  # type: ignore  # identical frames
    path = tmp_path / "m.jsonl"
    write_jsonl(first, path)
    lines = path.read_text().splitlines()
    assert [Measurement.model_validate(json.loads(line)) for line in lines] == first


def test_camera_output_is_accepted_by_the_core(corridor: Corridor, corridor_path: Path) -> None:
    core = Core(corridor_path.read_text(), core_config("maxpressure").model_dump_json())
    target = corridor.approaches[0]
    agg = Aggregator(calib(target.junction, target.id))
    ms = run(agg, 5, lambda t: [obs(1, 300), obs(2, 320, cls="bus")])
    cam = ms[-1]
    assert total(cam.queue_counts) == 2
    others = [
        empty_measurement(cam.time, a.junction, a.id, 60.0) for a in corridor.approaches if a.id != target.id
    ]
    batch = "[" + ",".join(m.model_dump_json() for m in [cam, *others]) + "]"
    out = StepOutput.model_validate_json(core.step(cam.time, batch))
    state = next(s for s in out.states if s.approach_id == target.id)
    assert state.measured_queue_pcu > 0 and state.trust_state == "TRUSTED"
