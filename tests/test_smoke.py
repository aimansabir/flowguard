"""Short closed-loop runs through SUMO, the observer and the Rust core."""

import json
from pathlib import Path
from typing import Any

import pytest
from flowguard_sim.run import run

LOGS = ["truth.jsonl", "measurements.jsonl", "states.jsonl", "commands.jsonl", "tripinfo.xml", "summary.json"]
CLASSES = ("car", "motorcycle", "rickshaw", "qingqi", "bus", "truck")


def total(counts: dict[str, int]) -> int:
    return sum(counts[c] for c in CLASSES)


def records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines()]


@pytest.mark.parametrize("controller", ["fixed", "maxpressure"])
def test_300s_run(controller: str, tmp_path: Path) -> None:
    summary = run(controller, 300, 1, results_dir=tmp_path)
    out = tmp_path / summary["run_id"]
    for name in LOGS:
        assert (out / name).stat().st_size > 0, name

    # 12 approaches and 3 junctions, one record each per second
    for name in ("truth.jsonl", "measurements.jsonl", "states.jsonl"):
        assert len(records(out / name)) == 300 * 12, name
    assert len(records(out / "commands.jsonl")) == 300 * 3
    assert summary["vehicles_departed"] > 0
    assert summary["camera_zone_m"] == 60


def test_observer_conserves_vehicles(tmp_path: Path) -> None:
    """In both views, vehicles in view now = in view last second + arrivals - departures."""
    summary = run("fixed", 300, 1, results_dir=tmp_path)
    out = tmp_path / summary["run_id"]
    for name in ("truth.jsonl", "measurements.jsonl"):
        previous: dict[str, int] = {}
        seen_departures = 0
        for m in records(out / name):
            a = m["approach_id"]
            departed = sum(total(m["departures"][mv]) for mv in ("left", "straight", "right"))
            seen_departures += departed
            expected = previous.get(a, 0) + total(m["arrivals"]) - departed
            assert total(m["zone_counts"]) == expected, (name, m["time"], a)
            assert total(m["queue_counts"]) <= total(m["zone_counts"])
            previous[a] = total(m["zone_counts"])
        assert seen_departures > 0, name


def test_camera_sees_less_than_truth(tmp_path: Path) -> None:
    summary = run("fixed", 300, 1, results_dir=tmp_path)
    out = tmp_path / summary["run_id"]
    truth, cam = records(out / "truth.jsonl"), records(out / "measurements.jsonl")
    assert all(total(c["zone_counts"]) <= total(t["zone_counts"]) for t, c in zip(truth, cam, strict=True))
    assert sum(total(c["zone_counts"]) for c in cam) < sum(total(t["zone_counts"]) for t in truth)
