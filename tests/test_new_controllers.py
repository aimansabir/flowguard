"""The D33-D35 controllers through the binding: they build from their
config files, start from the fair fixed plan, and run legal signals."""

import json
from pathlib import Path

import pytest
from test_core_binding import measurement

from flowguard_contracts import StepOutput
from flowguard_core import Core
from flowguard_sim import webster
from flowguard_sim.paths import load_json
from flowguard_sim.run import core_config

NEW = ("maxpressure_hyst_cap_lanes_moving", "actuated", "adaptive_split")


@pytest.mark.parametrize("controller", NEW)
def test_builds_and_steps(corridor_path: Path, controller: str) -> None:
    core = Core(corridor_path.read_text(), core_config(controller).model_dump_json())
    for t in range(120):
        ms = [measurement("W_J1", "J1", t % 7), measurement("N1_J1", "J1", 3)]
        out = StepOutput.model_validate_json(core.step(float(t), json.dumps(ms)))
        assert len(out.commands) == 3
    assert json.loads(core.interventions_json()).get("invalid_request", 0) == 0


def test_configs_come_from_the_fair_fixed_plan() -> None:
    plan = webster.plan(green_wave=True)
    act = core_config("actuated").controller.model_dump()
    factor = load_json("config/actuated.json")["max_green_factor"]
    assert act["max_green_s"] == [factor * g for g in plan["green_s"]]
    split = core_config("adaptive_split").controller.model_dump()
    assert split["green_s"] == plan["green_s"] and split["offsets_s"] == plan["offsets_s"]
    mp = core_config("maxpressure_hyst_cap_lanes_moving").controller.model_dump()
    assert mp["lane_weight"] and mp["moving_count"]


def test_existing_configs_unchanged() -> None:
    # the new options default off for every existing max-pressure variant
    for name in ("maxpressure", "maxpressure_hyst", "maxpressure_hyst_cap", "maxpressure_hyst_cap_serve"):
        ctl = core_config(name).controller.model_dump()
        assert not ctl.get("lane_weight") and not ctl.get("moving_count")
