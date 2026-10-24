"""Run one closed-loop simulation.

    python -m experiments.run --controller fixed --seconds 3600 --seed 1

Each second: SUMO -> observer -> fault hook -> Guard -> controller -> envelope
-> SUMO. Everything is logged to results/<run_id>/.
"""
import argparse
import hashlib
import json
from datetime import datetime
from pathlib import Path

import traci

from control.envelope import SafetyEnvelope, SignalTiming
from control.fixed import FixedTimeController
from control.maxpressure import MaxPressureController
from experiments.metrics import RunMetrics
from guard.faults import FaultInjector
from guard.guard import Guard
from schemas.corridor import load_corridor
from sim.demand import ensure_demand
from sim.observer import SimObserver
from sim.signals import SignalApplier
from sim.sumo_env import sumo_binary

ROOT = Path(__file__).resolve().parent.parent
CONTROLLERS = ("fixed", "maxpressure")


def load_json(rel):
    return json.loads((ROOT / rel).read_text())


def make_controller(name, corridor, timing):
    if name == "fixed":
        return FixedTimeController(corridor, load_json("config/fixed.json")["green_s"], timing.clearance_s)
    if name == "maxpressure":
        return MaxPressureController(corridor)
    raise ValueError(f"unknown controller {name!r}")


class JsonlLog:
    def __init__(self, folder):
        self.files = {k: open(folder / f"{k}.jsonl", "w") for k in ("observations", "rated", "commands")}

    def write(self, kind, records):
        f = self.files[kind]
        for r in records:
            f.write(r.to_json() + "\n")

    def close(self):
        for f in self.files.values():
            f.close()


def run(controller_name, seconds, seed, results_dir=ROOT / "results"):
    sim_cfg = load_json("config/sim.json")
    pcu = load_json("config/pcu.json")["weights"]
    timing = SignalTiming.from_config(load_json("config/signal.json"))
    corridor = load_corridor(ROOT / sim_cfg["corridor_file"])
    demand = ensure_demand(seed, corridor, load_json("config/demand.json"), ROOT / sim_cfg["demand_dir"])

    run_id = f"{controller_name}_seed{seed}_{seconds}s_{datetime.now():%Y%m%d-%H%M%S}"
    out = Path(results_dir) / run_id
    out.mkdir(parents=True)

    controller = make_controller(controller_name, corridor, timing)
    envelope = SafetyEnvelope(corridor, timing, controller.name)
    faults, guard = FaultInjector(), Guard()
    metrics = RunMetrics(corridor)

    traci.start([
        sumo_binary("sumo"),
        "--net-file", str(ROOT / sim_cfg["net_file"]),
        "--additional-files", str(ROOT / sim_cfg["vtypes_file"]),
        "--route-files", str(demand),
        "--step-length", "1",
        "--lateral-resolution", str(sim_cfg["lateral_resolution_m"]),
        "--time-to-teleport", str(sim_cfg["time_to_teleport_s"]),
        "--seed", str(seed),
        "--end", str(seconds),
        "--tripinfo-output", str(out / "tripinfo.xml"),
        "--log", str(out / "sumo.log"),
        "--no-step-log", "true",
    ])
    observer = SimObserver(corridor, pcu)
    signals = SignalApplier(corridor)
    log = JsonlLog(out)
    last_commands = {}
    departed = teleports = collisions = 0

    try:
        for _ in range(seconds):
            t = traci.simulation.getTime()
            truth = observer.observe(t)
            metrics.add_observations(truth)
            seen = faults.apply(t, truth)
            rated = guard.rate(seen, last_commands)
            requests = controller.decide(t, rated, last_commands)
            commands = envelope.step(t, requests)
            signals.apply(commands)
            metrics.add_commands(commands)

            log.write("observations", seen)
            log.write("rated", rated)
            log.write("commands", commands)
            last_commands = {c.junction_id: c for c in commands}

            traci.simulationStep()
            departed += traci.simulation.getDepartedNumber()
            teleports += traci.simulation.getStartingTeleportNumber()
            collisions += traci.simulation.getCollidingVehiclesNumber()
        end_time = traci.simulation.getTime()
        in_network = traci.vehicle.getIDCount()
    finally:
        traci.close()
        log.close()

    summary = {
        "run_id": run_id,
        "controller": controller_name,
        "seed": seed,
        "seconds": seconds,
        "demand_file": demand.relative_to(ROOT).as_posix(),
        "demand_sha256": hashlib.sha256(demand.read_bytes()).hexdigest(),
        **metrics.summary(out / "tripinfo.xml", end_time, departed),
        "vehicles_in_network_at_end": in_network,
        "teleports": teleports,
        "colliding_vehicles": collisions,
        "envelope_interventions": {"total": sum(envelope.interventions.values()), **envelope.interventions},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def print_summary(s):
    print(f"\nrun {s['run_id']}  (demand {s['demand_file']}, sha256 {s['demand_sha256'][:12]})")
    print(f"  avg delay per vehicle   {s['avg_delay_s']} s  (avg insertion delay {s['avg_depart_delay_s']} s)")
    print(f"  throughput              {s['throughput_veh_per_h']} veh/h  "
          f"({s['vehicles_arrived']} arrived / {s['vehicles_departed']} departed, "
          f"{s['vehicles_in_network_at_end']} still in network)")
    print(f"  teleports {s['teleports']}, colliding vehicles {s['colliding_vehicles']}")
    print(f"  envelope interventions  {s['envelope_interventions']}")
    print(f"  {'approach':<8} {'max queue PCU':>14} {'longest red s':>14}")
    for a in s["max_queue_pcu"]:
        print(f"  {a:<8} {s['max_queue_pcu'][a]:>14} {s['longest_red_s'][a]:>14}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=CONTROLLERS, required=True)
    parser.add_argument("--seconds", type=int, default=3600)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()
    print_summary(run(args.controller, args.seconds, args.seed))
