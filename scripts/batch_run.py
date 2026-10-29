"""Run a grid of closed-loop simulations in parallel (docs/results.md, D33-D35).

    python scripts/batch_run.py --out results/d35 --seeds 1 2 3 \
        --controllers fixed_webster_wave actuated --demands normal heavy

Demand files are generated first, one process, so parallel runs never race to
write the same file. Each run then gets a fresh process (SUMO through traci is
one simulation per process) and writes results/<out>/<run_id>/ as
`flowguard_sim.run` does. Prints one line per finished run.
"""

import argparse
import itertools
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

from flowguard_sim.corridor import load_corridor
from flowguard_sim.demand import demand_config, ensure_demand
from flowguard_sim.paths import ROOT, load_json
from flowguard_sim.run import run


def one(controller: str, seed: int, demand: str, out: Path, seconds: int, guard: str,
        faults: str | None) -> str:
    s = run(controller, seconds, seed, results_dir=out, guard=guard, faults=faults,
            demand=None if demand == "normal" else demand)
    return (f"{controller:<34} seed {seed} {demand:<12} delay {s['avg_delay_s']:>7} s  "
            f"entry wait {s['avg_depart_delay_s']:>6} s  {s['throughput_veh_per_h']:>7} veh/h  "
            f"in network {s['vehicles_in_network_at_end']}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--controllers", nargs="+", required=True)
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    ap.add_argument("--demands", nargs="+", default=["normal"])
    ap.add_argument("--seconds", type=int, default=3600)
    ap.add_argument("--guard", default="passthrough")
    ap.add_argument("--faults", help="fault scenario in config/faults/ (default: clean cameras)")
    ap.add_argument("--jobs", type=int, default=8)
    a = ap.parse_args()

    sim_cfg = load_json("config/sim.json")
    corridor = load_corridor(ROOT / sim_cfg["corridor_file"])
    for demand, seed in itertools.product(a.demands, a.seeds):
        scenario = None if demand == "normal" else demand
        cfg, folder, label = demand_config(scenario, ROOT / sim_cfg["demand_dir"])
        ensure_demand(seed, corridor, cfg, folder, label)

    a.out.mkdir(parents=True, exist_ok=True)
    grid = list(itertools.product(a.controllers, a.seeds, a.demands))
    with ProcessPoolExecutor(a.jobs, max_tasks_per_child=1) as pool:
        futures = [pool.submit(one, c, s, d, a.out, a.seconds, a.guard, a.faults) for c, s, d in grid]
        for f in as_completed(futures):
            print(f.result(), flush=True)


if __name__ == "__main__":
    main()
