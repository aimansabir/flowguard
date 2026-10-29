"""Webster fixed-time plan and green-wave offsets (flowguard_sim.webster)."""

from typing import Any

import pytest

from flowguard_contracts import Corridor
from flowguard_contracts.models import SignalTiming
from flowguard_sim import webster
from flowguard_sim.paths import load_json
from flowguard_sim.run import core_config

TIMING = SignalTiming(min_green_s=10, max_red_s=90, yellow_s=3, all_red_s=2)
CFG: dict[str, Any] = {"saturation_pcu_per_h_per_lane": 1800, "min_cycle_s": 40, "max_cycle_s": 120}


def flows(corridor: Corridor, main_per_lane: float, side_per_lane: float) -> dict[str, float]:
    """Same flow on every main (2-lane) and every side (1-lane) approach."""
    return {a.id: (main_per_lane if a.lanes == 2 else side_per_lane) * a.lanes for a in corridor.approaches}


def test_route_follows_the_corridor(corridor: Corridor) -> None:
    assert webster.route(corridor, "W_J1", "J3_E") == ["W_J1", "J1_J2", "J2_J3"]
    assert webster.route(corridor, "N2_J2", "J2_S2") == ["N2_J2"]
    with pytest.raises(ValueError, match="no route"):
        webster.route(corridor, "W_J1", "nowhere")


def test_approach_flows(corridor: Corridor) -> None:
    demand = load_json("config/demand.json")
    pcu = load_json("config/pcu.json")["weights"]
    mix = demand["class_mix"]
    pcu_per_veh = sum(s * pcu[c] for c, s in mix.items()) / sum(mix.values())
    peak = webster.approach_flows(corridor, demand, pcu, "peak")
    # every trip from W_J1 starts on it, nothing else joins it
    main_peak = demand["main_veh_per_h"] * (1 + demand["peak"]["extra"])
    assert peak["W_J1"] == pytest.approx(main_peak * pcu_per_veh)
    assert all(f > 0 for f in peak.values())
    mean = webster.approach_flows(corridor, demand, pcu, "mean")
    assert all(mean[a] < peak[a] for a in peak)
    with pytest.raises(ValueError, match="design_flow"):
        webster.approach_flows(corridor, demand, pcu, "busiest")


def test_webster_formula(corridor: Corridor) -> None:
    # one phase per approach (D32): y = 0.2 for each main-road direction, 0.1 for each side
    # street, Y = 0.6; L = 4 x 5 = 20; C = (30 + 5) / 0.4 = 87.5; greens share 67.5 s as
    # 2:2:1:1 -> 22.5, 22.5, 11.25, 11.25 -> rounded 22, 22, 11, 11
    greens = webster.webster_greens(corridor, flows(corridor, 360, 180), TIMING, CFG)
    assert greens == [22.0, 22.0, 11.0, 11.0]


def test_min_green_and_cycle_limits(corridor: Corridor) -> None:
    # light side street: its share is under min green
    assert webster.webster_greens(corridor, flows(corridor, 360, 18), TIMING, CFG)[2:] == [10.0, 10.0]
    # oversaturated (Y >= 1): the longest cycle whose reds all fit max red (90 s), not the
    # 120 s maximum: four equal phases of g s are each red 3g + 4 x 5 - 3 s, so g = 24
    assert webster.webster_greens(corridor, flows(corridor, 1800, 1800), TIMING, CFG) == [24.0] * 4
    # nearly empty: every phase at min green (four of them no longer fit the 40 s minimum cycle)
    greens = webster.webster_greens(corridor, flows(corridor, 10, 10), TIMING, CFG)
    assert greens == [10.0] * 4


def test_rejects_plans_over_max_red(corridor: Corridor) -> None:
    short_red = SignalTiming(min_green_s=10, max_red_s=30, yellow_s=3, all_red_s=2)
    with pytest.raises(ValueError, match="max red"):
        webster.webster_greens(corridor, flows(corridor, 1800, 1800), short_red, CFG)


def test_green_wave_offsets(corridor: Corridor) -> None:
    # internal main approaches are 285 m: 25.9 s per junction at 11 m/s
    offsets = webster.green_wave_offsets(corridor, "W_J1", 11.0, 45.0)
    assert offsets == {"J1": 0.0, "J2": pytest.approx(25.9), "J3": pytest.approx(570 / 11 - 45, abs=0.05)}
    # the other direction starts at J3
    assert webster.green_wave_offsets(corridor, "E_J3", 11.0, 45.0)["J3"] == 0.0
    with pytest.raises(ValueError, match="not an approach"):
        webster.green_wave_offsets(corridor, "J3_E", 11.0, 45.0)


@pytest.mark.parametrize("controller", ["fixed_webster", "fixed_webster_wave"])
def test_plan_is_a_valid_core_config(controller: str) -> None:
    ctl = core_config(controller).controller.model_dump()
    assert ctl["kind"] == "fixed" and len(ctl["green_s"]) == 4  # one phase per approach (D32)
    assert (ctl["offsets_s"] is not None) == (controller == "fixed_webster_wave")
