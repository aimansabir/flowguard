"""Live dashboard plumbing that does not need SUMO or a browser."""

import json
import threading
import time
from pathlib import Path

from flowguard_contracts import ClassCounts, Corridor, Measurement, StepOutput
from flowguard_core import Core
from flowguard_sim.live import Hub, downstream, frame, layout, read_faults, read_run
from flowguard_sim.results_index import ResultsIndex
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


def test_frame_camera_counts_each_class(corridor: Corridor, corridor_path: Path) -> None:
    core = Core(corridor_path.read_text(), core_config("maxpressure").model_dump_json())
    ms = [empty_measurement(0.0, a.junction, a.id, 60.0) for a in corridor.approaches]
    zone = {**ms[0].zone_counts.model_dump(), "car": 3, "qingqi": 2}
    ms[0] = ms[0].model_copy(update={"zone_counts": ClassCounts.model_validate(zone)})
    batch = "[" + ",".join(m.model_dump_json() for m in ms) + "]"
    step = StepOutput.model_validate_json(core.step(0.0, batch))
    ev = StepEvent(0.0, ms, [], step, lambda c: core.pcu(c.model_dump_json()))
    cam = frame(ev, downstream(corridor), None, None)["app"][ms[0].approach_id]["cam"]
    assert cam["zc"] == [3, 0, 0, 2, 0, 0] and cam["z"] == 5  # car, motorcycle, rickshaw, qingqi, bus, truck


def test_layout_carries_seed_and_configs(corridor: Corridor) -> None:
    out = layout(corridor, "maxpressure_hyst_cap", "live", "x", 4.0, "v1", "smoke", 7, 300)
    assert out["seed"] == 7 and out["seconds"] == 300 and out["faults"] == "smoke"
    assert out["controller_cfg"]["capacity"]["spillback_occupancy"] > 0
    assert out["guard_cfg"]["kind"] == "v1" and out["guard_cfg"]["trust"]["recover_s"] > 0
    json.dumps(out)


def test_read_faults_groups_by_time(tmp_path: Path) -> None:
    rows = [{"time": 1.0, "approach_id": "W_J1", "fault": "freeze"},
            {"time": 1.0, "approach_id": "E_J3", "fault": None}]
    (tmp_path / "faults.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    assert read_faults(tmp_path) == {1.0: {"W_J1": "freeze", "E_J3": None}}
    assert read_faults(tmp_path / "missing") == {}


def test_results_index_reads_runs_without_writing(tmp_path: Path) -> None:
    done = tmp_path / "exp" / "runs" / "maxpressure_seed1"
    done.mkdir(parents=True)
    (done / "summary.json").write_text(json.dumps({"controller": "maxpressure", "guard": "v1", "seed": 1}))
    states = [{"time": float(t), "approach_id": "W_J1", "trust_state": "TRUSTED"} for t in range(3)]
    (done / "states.jsonl").write_text("".join(json.dumps(s) + "\n" for s in states))
    running = tmp_path / "live_run"
    running.mkdir()
    (running / "measurements.jsonl").write_text("")
    before = {p: p.stat().st_mtime_ns for p in tmp_path.rglob("*")}

    index = ResultsIndex(tmp_path)
    first = index.snapshot()
    assert [(r["run"], r["status"], r["group"]) for r in first["runs"]] == [
        ("maxpressure_seed1", "complete", "exp/runs"), ("live_run", "incomplete", "")]
    assert first["scoring"] == 1
    index.wait(10)
    row = index.snapshot()["runs"][0]
    assert row["eval"]["false_alarms"]["clean_rated_seconds"] == 3
    assert "episodes" not in row["eval"]
    assert {p: p.stat().st_mtime_ns for p in tmp_path.rglob("*")} == before  # nothing written


def test_results_index_of_a_missing_directory_is_empty(tmp_path: Path) -> None:
    root = tmp_path / "none"
    assert ResultsIndex(root).snapshot() == {"root": str(root), "runs": [], "scoring": 0}
