"""Live dashboard plumbing that does not need SUMO or a browser."""

import json
import threading
import time
from pathlib import Path

from flowguard_contracts import ClassCounts, Corridor, Measurement, StepOutput
from flowguard_core import Core
from flowguard_sim.live import Hub, downstream, frame, read_run
from flowguard_sim.run import StepEvent, core_config
from flowguard_vision.stub import empty_measurement


def queued(m: Measurement, cars: int) -> Measurement:
    counts = {**m.queue_counts.model_dump(), "car": cars}
    return m.model_copy(update={"queue_counts": ClassCounts.model_validate(counts)})


def test_downstream_follows_the_straight_movement(corridor: Corridor) -> None:
    down = downstream(corridor)
    assert down["W_J1"] == "J1_J2" and down["J1_J2"] == "J2_J3"
    assert down["N1_J1"] is None  # side street straight leaves the corridor
    assert set(down) == {a.id for a in corridor.approaches}


def test_frame_pressure_term_is_queue_minus_downstream_queue(corridor: Corridor, corridor_path: Path) -> None:
    core = Core(corridor_path.read_text(), core_config("maxpressure").model_dump_json())
    ms = [empty_measurement(0.0, a.junction, a.id, 60.0) for a in corridor.approaches]
    cars = {"W_J1": 6, "J1_J2": 2}
    ms = [queued(m, cars[m.approach_id]) if m.approach_id in cars else m for m in ms]
    batch = "[" + ",".join(m.model_dump_json() for m in ms) + "]"
    step = StepOutput.model_validate_json(core.step(0.0, batch))
    ev = StepEvent(0.0, ms, [], step, lambda c: core.pcu(c.model_dump_json()))
    f = frame(ev, downstream(corridor), None, None)
    assert f["app"]["W_J1"]["term"] == 4.0  # 6 PCU here - 2 PCU on the link ahead
    assert f["app"]["N1_J1"]["term"] == 0.0 and f["app"]["W_J1"]["tr"] == 0
    assert f["jn"]["J1"]["st"] == "green" and f["jn"]["J1"]["rec"] is None
    json.dumps(f)  # must be serialisable as sent


def test_read_run_groups_by_time(tmp_path: Path) -> None:
    m = [empty_measurement(t, "J1", a, 60.0) for t in (0.0, 1.0) for a in ("W_J1", "J2_J1")]
    lines = "".join(x.model_dump_json() + "\n" for x in m)
    (tmp_path / "measurements.jsonl").write_text(lines, encoding="utf-8")
    seen, truth, cmds = read_run(tmp_path)
    assert sorted(seen) == [0.0, 1.0] and all(len(v) == 2 for v in seen.values())
    assert not truth and not cmds


def test_hub_pace_holds_while_paused_and_follows_speed() -> None:
    hub = Hub()
    hub.speed = 20.0
    t0 = time.monotonic()
    hub.pace()
    assert 0.04 <= time.monotonic() - t0 < 0.5
    hub.paused = True
    done = threading.Event()
    threading.Thread(target=lambda: (hub.pace(), done.set()), daemon=True).start()
    assert not done.wait(0.3)
    hub.paused = False
    assert done.wait(2)
