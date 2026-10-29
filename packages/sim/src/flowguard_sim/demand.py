"""Random one-hour demand for the corridor, saved as a .rou.xml file.

The file is made once per seed and then reused, so every controller we
compare drives exactly the same vehicles. Delete the file to regenerate it.

    uv run python -m flowguard_sim.demand --seed 1
    uv run python -m flowguard_sim.demand --seed 1 --scenario right_turn

A scenario (config/demand_scenarios/<name>.json) overrides keys of
config/demand.json and may add `entry_scale` ({entry edge: factor} on its
rate) and `turn_weights` ({"left"|"right": weight} for how the turning share
of each entry splits by the turn the trip makes first). Its files go to
data/demand/<name>/. Without a scenario the files are exactly as before.
"""

import argparse
import math
import random
from pathlib import Path
from typing import Any

from flowguard_contracts import Corridor
from flowguard_sim.corridor import load_corridor
from flowguard_sim.paths import ROOT, load_json

SCENARIO_DIR = ROOT / "config" / "demand_scenarios"


def demand_config(scenario: str | None, demand_dir: Path) -> tuple[dict[str, Any], Path, str]:
    """(config, directory for its files, label for the file header) of a demand scenario."""
    base = load_json("config/demand.json")
    if scenario in (None, "normal"):
        return base, Path(demand_dir), "config/demand.json"
    path = SCENARIO_DIR / f"{scenario}.json"
    if not path.is_file():
        known = sorted(f.stem for f in SCENARIO_DIR.glob("*.json"))
        raise ValueError(f"unknown demand scenario {scenario!r}; known: normal, {', '.join(known)}")
    rel = path.relative_to(ROOT).as_posix()
    over = {k: v for k, v in load_json(rel).items() if not k.startswith("_")}
    return {**base, **over}, Path(demand_dir) / scenario, rel


def first_turn(corridor: Corridor, entry: str, exit_edge: str) -> str:
    """The first turn ("left" or "right") a trip from `entry` to `exit_edge`
    makes, going straight until then; "straight" if it never turns."""
    by_id = {a.id: a for a in corridor.approaches}

    def straight_reach(edge: str | None) -> set[str]:
        seen: set[str] = set()
        while edge is not None and edge not in seen:
            seen.add(edge)
            edge = by_id[edge].movements.straight if edge in by_id else None
        return seen

    a = by_id.get(entry)
    while a is not None:
        for turn in ("left", "right"):
            if exit_edge in straight_reach(getattr(a.movements, turn)):
                return turn
        a = by_id.get(a.movements.straight or "")
    return "straight"


def demand_path(demand_dir: Path, seed: int) -> Path:
    return Path(demand_dir) / f"corridor_seed{seed}.rou.xml"


def ensure_demand(seed: int, corridor: Corridor, cfg: dict[str, Any], demand_dir: Path,
                  label: str = "config/demand.json") -> Path:
    path = demand_path(demand_dir, seed)
    if not path.exists():
        write_demand(path, seed, corridor, cfg, label)
    return path


def rate_per_s(base_per_h: float, t: float, peak: dict[str, float]) -> float:
    bump = peak["extra"] * math.exp(-0.5 * ((t - peak["centre_s"]) / peak["width_s"]) ** 2)
    return base_per_h * (1 + bump) / 3600


def destinations(entry: str, corridor: Corridor, cfg: dict[str, Any]) -> dict[str, float]:
    """Where traffic from one entry goes, as {exit_edge: probability}.

    Main road entries mostly go straight through and otherwise turn off into
    any side street. Side street entries go straight across or turn onto the
    main road (either direction).
    """
    info = next(e for e in corridor.entries if e.edge == entry)
    share: float = cfg["main_through_share"] if info.kind == "main" else cfg["side_through_share"]
    other_kind = "side" if info.kind == "main" else "main"
    others = [e.edge for e in corridor.exits if e.kind == other_kind]
    turn_w: dict[str, float] | None = cfg.get("turn_weights")
    if turn_w is None:
        weights = {e: (1 - share) / len(others) for e in others}
    else:
        raw = {e: float(turn_w.get(first_turn(corridor, entry, e), 1.0)) for e in others}
        weights = {e: (1 - share) * w / sum(raw.values()) for e, w in raw.items()}
    weights[info.through_exit] = share
    return weights


def entry_base_per_h(entry: str, kind: str, cfg: dict[str, Any]) -> float:
    base: float = cfg["main_veh_per_h"] if kind == "main" else cfg["side_veh_per_h"]
    scale: dict[str, float] | None = cfg.get("entry_scale")
    return base if scale is None else base * scale.get(entry, 1.0)


def write_demand(path: Path, seed: int, corridor: Corridor, cfg: dict[str, Any],
                 label: str = "config/demand.json") -> None:
    rng = random.Random(seed)
    classes = list(cfg["class_mix"])
    class_weights = list(cfg["class_mix"].values())
    kinds = {e.edge: e.kind for e in corridor.entries}
    entries = sorted(kinds)
    dests = {e: destinations(e, corridor, cfg) for e in entries}

    trips: list[tuple[int, str, str, str]] = []
    for t in range(cfg["duration_s"]):
        for entry in entries:
            base = entry_base_per_h(entry, kinds[entry], cfg)
            # one Bernoulli draw per second, fine while rates stay well under 1 veh/s
            if rng.random() < rate_per_s(base, t, cfg["peak"]):
                to = rng.choices(list(dests[entry]), weights=list(dests[entry].values()))[0]
                vtype = rng.choices(classes, weights=class_weights)[0]
                trips.append((t, vtype, entry, to))

    # the header text is part of the file hash; keep it stable
    lines = [f"<!-- generated by sim/demand.py, seed={seed}, {label} -->", "<routes>"]
    for i, (t, vtype, frm, to) in enumerate(trips):
        lines.append(f'    <trip id="{i}" depart="{t}.00" type="{vtype}" from="{frm}" to="{to}" '
                     f'departLane="best" departSpeed="max"/>')
    lines.append("</routes>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {len(trips)} trips to {path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--scenario", default="normal", help="demand scenario in config/demand_scenarios/")
    args = parser.parse_args()
    sim_cfg = load_json("config/sim.json")
    corridor = load_corridor(ROOT / sim_cfg["corridor_file"])
    cfg, folder, label = demand_config(args.scenario, ROOT / sim_cfg["demand_dir"])
    write_demand(demand_path(folder, args.seed), args.seed, corridor, cfg, label)
