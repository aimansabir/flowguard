"""Fixed-time plan from the demand config (Webster 1958), optional green wave.

    uv run python -m flowguard_sim.webster          # print the plan

The core never sees demand, so the plan is worked out here and handed to the
core as an ordinary `fixed` controller config (green_s, offsets_s). D21.

1. Design flow per approach, PCU/h: each entry's rate from config/demand.json
   (peak or mean of the hour), split over its destinations, routed through
   the corridor map, times the mean PCU per vehicle of the class mix.
2. Flow ratio y of a phase = the highest flow per lane / saturation flow over
   its approaches, taken over every junction (one plan for all of them, so it
   is sized for the busiest).
3. Lost time L = one yellow + all-red per phase. Cycle
   C = (1.5 L + 5) / (1 - Y), clamped to [min_cycle_s, max_cycle_s].
4. Greens share C - L in proportion to y, rounded to whole seconds, at least
   min green. The cycle is then recomputed from the rounded greens.
5. Green wave (optional): each junction's plan starts later by the travel
   time from the first junction along the main road, at `speed_mps`.
"""

import math
from collections import deque
from typing import Any

from flowguard_contracts import Approach, Corridor
from flowguard_contracts.models import SignalTiming
from flowguard_sim.corridor import load_corridor
from flowguard_sim.demand import destinations, rate_per_s
from flowguard_sim.paths import ROOT, load_json, without_notes


def route(corridor: Corridor, entry: str, exit_edge: str) -> list[str]:
    """Approaches a trip from `entry` to `exit_edge` crosses, in order (fewest junctions)."""
    by_id = {a.id: a for a in corridor.approaches}
    paths: deque[list[str]] = deque([[entry]])
    seen = {entry}
    while paths:
        path = paths.popleft()
        m = by_id[path[-1]].movements
        for edge in (m.left, m.straight, m.right):
            if edge == exit_edge:
                return path
            if edge in by_id and edge not in seen:
                seen.add(edge)
                paths.append([*path, edge])
    raise ValueError(f"no route from {entry} to {exit_edge}")


def entry_rate_per_h(base_per_h: float, demand: dict[str, Any], design: str) -> float:
    if design == "peak":
        return base_per_h * (1 + demand["peak"]["extra"])
    if design == "mean":
        return sum(rate_per_s(base_per_h, t, demand["peak"]) for t in range(demand["duration_s"])) \
            * 3600 / demand["duration_s"]
    raise ValueError(f"design_flow must be 'peak' or 'mean', not {design!r}")


def approach_flows(corridor: Corridor, demand: dict[str, Any], pcu: dict[str, float],
                   design: str) -> dict[str, float]:
    """Design flow per approach, PCU/h."""
    mix: dict[str, float] = demand["class_mix"]
    pcu_per_veh = sum(share * pcu[c] for c, share in mix.items()) / sum(mix.values())
    flows = {a.id: 0.0 for a in corridor.approaches}
    for e in corridor.entries:
        base = demand["main_veh_per_h"] if e.kind == "main" else demand["side_veh_per_h"]
        rate = entry_rate_per_h(base, demand, design) * pcu_per_veh
        for exit_edge, p in destinations(e.edge, corridor, demand).items():
            for a in route(corridor, e.edge, exit_edge):
                flows[a] += rate * p
    return flows


def webster_greens(corridor: Corridor, flows: dict[str, float], timing: SignalTiming,
                   cfg: dict[str, Any]) -> list[float]:
    by_id = {a.id: a for a in corridor.approaches}
    n_phases = len(corridor.junctions[0].phases)
    if any(len(j.phases) != n_phases for j in corridor.junctions):
        raise ValueError("one fixed plan needs the same number of phases at every junction")
    sat: float = cfg["saturation_pcu_per_h_per_lane"]
    y = [max(flows[a] / (by_id[a].lanes * sat) for j in corridor.junctions for a in j.phases[k])
         for k in range(n_phases)]
    lost = n_phases * (timing.yellow_s + timing.all_red_s)
    total_y = sum(y)
    cycle = cfg["max_cycle_s"] if total_y >= 1 else (1.5 * lost + 5) / (1 - total_y)
    cycle = min(max(cycle, cfg["min_cycle_s"]), cfg["max_cycle_s"])
    greens = [float(max(math.ceil(timing.min_green_s), round((cycle - lost) * yk / total_y))) for yk in y]
    cycle = sum(greens) + lost
    for k, g in enumerate(greens):
        red = cycle - g - timing.yellow_s
        if red > timing.max_red_s:
            raise ValueError(f"phase {k} would be red {red:.0f} s, above max red {timing.max_red_s:.0f} s")
    return greens


def green_wave_offsets(corridor: Corridor, entry: str, speed_mps: float, cycle_s: float) -> dict[str, float]:
    """Offset per junction: travel time from the first junction along `entry`'s straight path."""
    by_id: dict[str, Approach] = {a.id: a for a in corridor.approaches}
    if entry not in by_id:
        raise ValueError(f"green wave entry {entry} is not an approach")
    a = by_id[entry]
    offsets = {a.junction: 0.0}
    t = 0.0
    while a.movements.straight in by_id:
        a = by_id[a.movements.straight]
        t += a.length_m / speed_mps
        offsets[a.junction] = round(t % cycle_s, 1)
    return offsets


def plan(green_wave: bool) -> dict[str, Any]:
    """`fixed` controller config from config/webster.json, demand, PCU and signal timing."""
    cfg = load_json("config/webster.json")
    corridor = load_corridor(ROOT / load_json("config/sim.json")["corridor_file"])
    timing = SignalTiming.model_validate(without_notes(load_json("config/signal.json")))
    flows = approach_flows(corridor, load_json("config/demand.json"), load_json("config/pcu.json")["weights"],
                           cfg["design_flow"])
    greens = webster_greens(corridor, flows, timing, cfg)
    ctl: dict[str, Any] = {"kind": "fixed", "green_s": greens}
    if green_wave:
        cycle = sum(greens) + len(greens) * (timing.yellow_s + timing.all_red_s)
        wave = cfg["green_wave"]
        ctl["offsets_s"] = green_wave_offsets(corridor, wave["entry"], wave["speed_mps"], cycle)
    return ctl


if __name__ == "__main__":
    cfg = load_json("config/webster.json")
    corridor = load_corridor(ROOT / load_json("config/sim.json")["corridor_file"])
    flows = approach_flows(corridor, load_json("config/demand.json"), load_json("config/pcu.json")["weights"],
                           cfg["design_flow"])
    print(f"design flow ({cfg['design_flow']}), PCU/h:")
    for a in corridor.approaches:
        print(f"  {a.id:<6} {flows[a.id]:7.0f}  ({a.lanes} lane{'s' if a.lanes > 1 else ''})")
    print("plan:", plan(green_wave=False))
    print("with green wave:", plan(green_wave=True))
