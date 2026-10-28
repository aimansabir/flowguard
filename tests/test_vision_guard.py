"""Module 1 -> Module 2: Measurements built by the vision pipeline (frames ->
tracker -> aggregation -> image health) go into the core with Guard v1, as
the corridor's W_J1 camera. Synthetic frames and a scripted tracker stand in
for footage and YOLO/ByteTrack, which are not built yet."""

from collections.abc import Iterator

import numpy as np

from flowguard_contracts import Corridor, Measurement, StepOutput
from flowguard_core import Core
from flowguard_sim.paths import ROOT, load_json
from flowguard_sim.run import core_config
from flowguard_vision.calibration import Calibration
from flowguard_vision.run import measurements
from flowguard_vision.stub import empty_measurement
from flowguard_vision.tracking import Frame, TrackObs

FPS = 10
CALIB = Calibration.model_validate({
    "junction_id": "J1", "approach_id": "W_J1",
    "zone_polygon": [(0.0, 0.0), (600.0, 0.0), (600.0, 100.0), (0.0, 100.0)], "zone_length_m": 60,
    "entry_line": [(0, 0), (0, 100)], "stop_line": [(600, 0), (600, 100)],
    "movement_regions": {"left": [(600, 0), (700, 0), (700, 33), (600, 33)],
                         "straight": [(600, 33), (700, 33), (700, 66), (600, 66)],
                         "right": [(600, 66), (700, 66), (700, 100), (600, 100)]},
    "reference_frame": "ref.png",
    "px_to_m": [(p, (p[0] / 10, p[1] / 10))
                for p in [(0.0, 0.0), (700.0, 0.0), (700.0, 100.0), (0.0, 100.0)]],
})


class Scripted:
    """A car enters every 5 s and drives through at 10 m/s (100 px/s)."""

    def update(self, frame: Frame) -> list[TrackObs]:
        out: list[TrackObs] = []
        for k in range(int(frame.t // 5) + 1):
            x = (frame.t - 5 * k) * 100.0
            if x <= 690:
                out.append(TrackObs(k, "car", 0.9, (x, 50.0)))
        return out


def frames(seconds: int, frozen: tuple[int, int]) -> Iterator[Frame]:
    """A textured scene with fresh sensor noise each frame; inside `frozen` the
    picture stops changing (the feed repeats one frame)."""
    rng = np.random.default_rng(0)
    scene = rng.integers(60, 200, (100, 700)).astype(np.int16)
    last = scene.astype(np.uint8)
    for k in range(seconds * FPS + 1):
        t = k / FPS
        if not frozen[0] <= t < frozen[1]:
            last = np.clip(scene + rng.integers(-3, 4, scene.shape), 0, 255).astype(np.uint8)
        yield Frame(t, last)


def run_core(cams: list[Measurement], corridor: Corridor) -> list[StepOutput]:
    corridor_path = ROOT / load_json("config/sim.json")["corridor_file"]
    core = Core(corridor_path.read_text(), core_config("maxpressure_hyst_cap", "v1").model_dump_json())
    out: list[StepOutput] = []
    for m in cams:
        others = [empty_measurement(m.time, a.junction, a.id, 60.0)
                  for a in corridor.approaches if a.id != "W_J1"]
        batch = "[" + ",".join(x.model_dump_json() for x in [m, *others]) + "]"
        out.append(StepOutput.model_validate_json(core.step(m.time, batch)))
    return out


def test_vision_measurements_drive_the_guard(corridor: Corridor) -> None:
    ref = next(frames(0, (0, 0))).gray
    cams = list(measurements(frames(120, (60, 90)), Scripted(), CALIB, ref))
    assert len(cams) >= 120 and all(m.source == "camera" for m in cams)
    health = [m.quality.image_health if m.quality else None for m in cams]
    assert all(h is not None for h in health)
    assert any(h is not None and h.frozen for h in health)
    assert sum(sum(m.arrivals.model_dump().values()) for m in cams) >= 20  # the cars were counted

    states = [next(s for s in out.states if s.approach_id == "W_J1") for out in run_core(cams, corridor)]
    by_t = {s.time: s for s in states}
    # healthy image: trusted throughout the first minute
    assert all(by_t[t].trust_state == "TRUSTED" for t in by_t if 5 <= t < 60)
    # frozen image: the camera's own health report is weak evidence (E1):
    # suspect while it lasts, never faulty on that alone
    frozen = [by_t[t].trust_state for t in by_t if 70 <= t < 90]
    assert "SUSPECT" in frozen and "FAULTY" not in frozen
    # the camera is still used while suspect, with a wider margin
    s = next(by_t[t] for t in by_t if 70 <= t < 90 and by_t[t].trust_state == "SUSPECT")
    assert not s.override_flag and s.queue_estimate == s.measured_queue_pcu
