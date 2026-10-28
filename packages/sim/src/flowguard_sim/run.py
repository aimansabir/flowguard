"""Run one closed-loop simulation.

    uv run python -m flowguard_sim.run --controller fixed --seconds 3600 --seed 1
    uv run python -m flowguard_sim.run --controller fixed --faults freeze   # config/faults/freeze.json
    uv run python -m flowguard_sim.run --controller maxpressure --guard v1 --faults mixed

Each second: SUMO -> observer -> fault hook -> core (Guard -> controller ->
envelope) -> SUMO. Logged to results/<run_id>/:
  truth.jsonl         full-edge ground truth Measurements (never shown to the core)
  measurements.jsonl  camera-zone Measurements as the core received them (after faults)
  states.jsonl        core ApproachStates (Guard output)
  commands.jsonl      envelope Commands
  faults.jsonl        with --faults only: active fault kind (or null) per approach-second
  tripinfo.xml, sumo.log, summary.json
"""
# pyright: basic
# traci ships without type information.

import argparse
import hashlib
import json
from collections.abc import Callable
from contextlib import ExitStack
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Literal, TextIO, cast

import traci
from pydantic import BaseModel

from flowguard_contracts import ClassCounts, CoreConfig, Measurement, StepOutput
from flowguard_core import Core
from flowguard_sim import webster
from flowguard_sim.conflicts import read_conflicts
from flowguard_sim.corridor import load_corridor
from flowguard_sim.demand import ensure_demand
from flowguard_sim.faults import FaultInjector, load_scenario
from flowguard_sim.metrics import RunMetrics
from flowguard_sim.observer import SimObserver
from flowguard_sim.paths import ROOT, load_json, without_notes
from flowguard_sim.signals import SignalApplier
from flowguard_sim.sumo_env import sumo_binary

CONTROLLERS = ("fixed", "fixed_webster", "fixed_webster_wave",
               "maxpressure", "maxpressure_hyst", "maxpressure_hyst_cap")
GUARDS = ("passthrough", "v1")
LOGS = ("truth", "measurements", "states", "commands")
FROM_CONFIG: Literal["config"] = "config"


def core_config(controller: str, guard: str = "passthrough") -> CoreConfig:
    """guard: "passthrough" (trust every camera) or "v1" (docs/decisions.md
    D26, every parameter at its default)."""
    if guard not in GUARDS:
        raise ValueError(f"unknown guard {guard!r}")
    mp = load_json("config/maxpressure.json")
    if controller == "fixed":
        ctl: dict[str, Any] = {"kind": "fixed", "green_s": load_json("config/fixed.json")["green_s"]}
    elif controller in ("fixed_webster", "fixed_webster_wave"):
        ctl = webster.plan(green_wave=controller == "fixed_webster_wave")
    elif controller == "maxpressure":
        ctl = {"kind": "maxpressure"}
    elif controller == "maxpressure_hyst":
        ctl = {"kind": "maxpressure", "hysteresis": mp["hysteresis"]}
    elif controller == "maxpressure_hyst_cap":
        ctl = {"kind": "maxpressure", "hysteresis": mp["hysteresis"], "capacity": mp["capacity"]}
    else:
        raise ValueError(f"unknown controller {controller!r}")
    return CoreConfig.model_validate({
        "pcu": load_json("config/pcu.json")["weights"],
        "timing": without_notes(load_json("config/signal.json")),
        "controller": ctl,
        "guard": {"kind": guard},
    })


def _jsonl(records: list[BaseModel]) -> str:
    return "".join(r.model_dump_json() + "\n" for r in records)


@dataclass(frozen=True)
class StepEvent:
    """One second of the closed loop, handed to `on_step` (dashboards, loggers)."""

    t: float
    seen: list[Measurement]  # what the core received
    truth: list[Measurement]  # full-edge ground truth
    step: StepOutput
    pcu: Callable[[ClassCounts], float]
    # active injected fault per approach (G0 ground truth): for display and
    # evaluation only, never given to the core
    faults: dict[str, str | None] = field(default_factory=lambda: {})


def run(controller: str, seconds: int, seed: int, results_dir: Path = ROOT / "results",
        camera_zone_m: float | None | Literal["config"] = FROM_CONFIG,
        faults: str | None = None, guard: str = "passthrough",
        on_step: Callable[[StepEvent], None] | None = None) -> dict[str, Any]:
    """camera_zone_m: metres the simulated camera sees; None = the whole edge;
    FROM_CONFIG = take it from config/sim.json. faults: scenario name in
    config/faults/, None = clean camera. guard: see core_config. on_step runs
    each second after the commands are applied and before SUMO advances, so
    traci state still matches t."""
    if seconds <= 0:
        raise ValueError("seconds must be > 0")
    sim_cfg = load_json("config/sim.json")
    zone: float | None = sim_cfg["camera_zone_m"] if camera_zone_m == FROM_CONFIG else camera_zone_m
    corridor_path = ROOT / sim_cfg["corridor_file"]
    corridor = load_corridor(corridor_path)
    demand = ensure_demand(seed, corridor, load_json("config/demand.json"), ROOT / sim_cfg["demand_dir"])
    injector = FaultInjector(load_scenario(faults, corridor, zone) if faults else None)

    core = Core(corridor_path.read_text(), core_config(controller, guard).model_dump_json())
    zone_tag = "full" if zone is None else f"{zone:g}m"
    fault_tag = f"_{faults}" if faults else ""
    guard_tag = "" if guard == "passthrough" else f"_guard{guard}"
    run_id = (f"{controller}{guard_tag}_seed{seed}_{seconds}s_{zone_tag}{fault_tag}"
              f"_{datetime.now():%Y%m%d-%H%M%S-%f}")
    out = Path(results_dir) / run_id
    out.mkdir(parents=True)

    def pcu(counts: ClassCounts) -> float:
        return core.pcu(counts.model_dump_json())

    metrics = RunMetrics(corridor, pcu)
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
    observer = SimObserver(corridor, zone, shifted=injector.shifts())
    conflicts = read_conflicts(ROOT / sim_cfg["net_file"], [j.id for j in corridor.junctions])
    signals = SignalApplier(corridor, conflicts)
    departed = teleports = collisions = 0

    with ExitStack() as stack:
        stack.callback(traci.close)
        logs: dict[str, TextIO] = {
            k: stack.enter_context(open(out / f"{k}.jsonl", "w", encoding="utf-8"))
            for k in LOGS + (("faults",) if faults else ())
        }
        for _ in range(seconds):
            t = cast(float, traci.simulation.getTime())
            view, truth = observer.observe(t)
            metrics.add_truth(truth)
            seen = injector.apply(t, view, observer.shifted_view)
            if faults:
                logs["faults"].write("".join(
                    json.dumps({"time": t, "approach_id": m.approach_id,
                                "fault": injector.active_kind(t, m.approach_id)}) + "\n"
                    for m in view))
            batch = "[" + ",".join(m.model_dump_json() for m in seen) + "]"
            step = StepOutput.model_validate_json(core.step(t, batch))
            signals.apply(step.commands)
            metrics.add_commands(step.commands)

            logs["truth"].write(_jsonl(list(truth)))
            logs["measurements"].write(_jsonl(list(seen)))
            logs["states"].write(_jsonl(list(step.states)))
            logs["commands"].write(_jsonl(list(step.commands)))
            if on_step is not None:
                labels = ({m.approach_id: injector.active_kind(t, m.approach_id) for m in view}
                          if faults else {})
                on_step(StepEvent(t, seen, truth, step, pcu, labels))

            traci.simulationStep()
            departed += cast(int, traci.simulation.getDepartedNumber())
            teleports += cast(int, traci.simulation.getStartingTeleportNumber())
            collisions += cast(int, traci.simulation.getCollidingVehiclesNumber())
        end_time = cast(float, traci.simulation.getTime())
        in_network = cast(int, traci.vehicle.getIDCount())

    interventions: dict[str, int] = json.loads(core.interventions_json())
    summary = {
        "run_id": run_id,
        "controller": controller,
        "guard": core.guard_name,
        "faults": faults,
        "seed": seed,
        "seconds": seconds,
        "camera_zone_m": zone,
        "demand_file": demand.relative_to(ROOT).as_posix(),
        # line endings normalised so Windows and Linux checkouts hash the same
        "demand_sha256": hashlib.sha256(demand.read_bytes().replace(b"\r\n", b"\n")).hexdigest(),
        **metrics.summary(out / "tripinfo.xml", end_time, departed),
        "vehicles_in_network_at_end": in_network,
        "teleports": teleports,
        "colliding_vehicles": collisions,
        "envelope_interventions": {"total": sum(interventions.values()), **interventions},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def print_summary(s: dict[str, Any]) -> None:
    print(f"\nrun {s['run_id']}  (demand {s['demand_file']}, sha256 {s['demand_sha256'][:12]})")
    print(f"  avg delay per vehicle   {s['avg_delay_s']} s  "
          f"(avg insertion delay {s['avg_depart_delay_s']} s)")
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
    parser.add_argument("--full-edge", action="store_true",
                        help="simulated camera sees the whole approach edge (v1 behaviour)")
    parser.add_argument("--faults", metavar="SCENARIO",
                        help="fault scenario name in config/faults/ (default: no faults)")
    parser.add_argument("--guard", choices=GUARDS, default="passthrough",
                        help="Guard rating the measurements (default: passthrough)")
    args = parser.parse_args()
    print_summary(run(args.controller, args.seconds, args.seed,
                      camera_zone_m=None if args.full_edge else FROM_CONFIG, faults=args.faults,
                      guard=args.guard))
