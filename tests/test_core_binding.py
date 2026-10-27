"""The Rust core through its Python binding."""

import json
from pathlib import Path
from typing import Any

import pytest

from flowguard_contracts import VEHICLE_CLASSES, Corridor, StepOutput
from flowguard_core import Core
from flowguard_sim.run import core_config


def zero() -> dict[str, int]:
    return dict.fromkeys(VEHICLE_CLASSES, 0)


def measurement(approach: str, junction: str, queued_cars: int, straight_buses: int = 0) -> dict[str, Any]:
    q = {**zero(), "car": queued_cars}
    return {"time": 0, "interval_s": 1, "junction_id": junction, "approach_id": approach, "source": "sim",
            "zone_length_m": 60, "zone_counts": q, "queue_counts": q, "arrivals": zero(),
            "departures": {"left": zero(), "straight": {**zero(), "bus": straight_buses}, "right": zero()}}


def make(corridor_path: Path, controller: str = "maxpressure") -> Core:
    return Core(corridor_path.read_text(), core_config(controller).model_dump_json())


def test_step(corridor_path: Path) -> None:
    core = make(corridor_path)
    out = StepOutput.model_validate_json(core.step(0.0, json.dumps([measurement("W_J1", "J1", 5, 2)])))
    assert [s.approach_id for s in out.states] == ["W_J1"]
    state = out.states[0]
    assert state.queue_estimate == 5.0 and state.trust_state == "TRUSTED"
    assert state.departures_pcu == 6.0
    assert state.exits_to == {"J2": 2}
    assert [c.junction_id for c in out.commands] == ["J1", "J2", "J3"]
    assert core.controller_name == "maxpressure" and core.guard_name == "passthrough"


def test_pcu_uses_configured_weights(corridor_path: Path) -> None:
    core = make(corridor_path)
    assert core.pcu(json.dumps({**zero(), "car": 4, "motorcycle": 4, "bus": 1})) == 9.0


def test_errors_become_value_errors(corridor_path: Path, corridor: Corridor) -> None:
    core = make(corridor_path)
    with pytest.raises(ValueError, match="unknown approach"):
        core.step(0.0, json.dumps([measurement("nowhere", "J1", 0)]))
    with pytest.raises(ValueError, match="belongs to"):
        core.step(0.0, json.dumps([measurement("W_J1", "J2", 0)]))
    with pytest.raises(ValueError):
        core.step(0.0, "not json")
    with pytest.raises(ValueError, match="phases"):
        Core(corridor_path.read_text(), json.dumps({**json.loads(core_config("fixed").model_dump_json()),
                                                    "controller": {"kind": "fixed", "green_s": [40]}}))
    assert corridor.junctions  # fixture sanity


def test_fixed_plan_through_core(corridor_path: Path) -> None:
    core = make(corridor_path, "fixed")
    stages: list[tuple[int, str]] = []
    for t in range(80):
        out = StepOutput.model_validate_json(core.step(float(t), "[]"))
        stages.append((out.commands[0].phase_index, out.commands[0].stage))
    assert stages[39] == (0, "green") and stages[40] == (0, "yellow")
    assert stages[43] == (0, "all_red") and stages[45] == (1, "green")
    assert stages[65] == (1, "yellow") and stages[70] == (0, "green")


def test_hysteresis_config_through_core(corridor_path: Path) -> None:
    core = make(corridor_path, "maxpressure_hyst")
    assert core.controller_name == "maxpressure_hyst"
    # side street far ahead from t = 0: basic max-pressure asks at once, hysteresis waits for min green
    side = [measurement("N1_J1", "J1", 20)]
    first = StepOutput.model_validate_json(core.step(0.0, json.dumps(side)))
    assert first.requests["J1"] == 0
    basic = StepOutput.model_validate_json(make(corridor_path).step(0.0, json.dumps(side)))
    assert basic.requests["J1"] == 1
    bad = json.loads(core_config("maxpressure_hyst").model_dump_json())
    bad["controller"]["hysteresis"]["gain_pcu"] = -1
    with pytest.raises(ValueError, match="gain_pcu"):
        Core(corridor_path.read_text(), json.dumps(bad))
