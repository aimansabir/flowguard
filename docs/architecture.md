# FlowGuard: architecture

## Pipeline

```
 producers                         decision core (Rust)                    actuation
 ---------                         --------------------                    ---------
 vision  (camera clips)  \
 sim     (SUMO observer)  >-- Measurement --> state --> Guard --> controller --> envelope --> Command --> signals
 replay  (historical)    /     (raw counts)   (PCU)   (trust)    (phase wish)   (legal)                (SUMO now,
                                                                                                          cabinet later)
```

One call per interval (1 s): `Core::step(t, measurements) -> StepOutput`.

| stage | input | output | where |
|---|---|---|---|
| producer | camera / SUMO / file | `Measurement` per approach | `packages/vision`, `packages/sim/observer.py` |
| state | `Measurement` | `ApproachState` at face value: PCU queue, arrivals, departures, exits per neighbour | `crates/core/src/state.rs` |
| Guard | states + measurements + last commands | trust state, queue estimate, margin | `crates/core/src/guard.rs` |
| controller | states + last commands | `{junction: wanted phase}` | `crates/core/src/control/` |
| envelope | wishes | `Command` per junction | `crates/core/src/safety.rs` |
| actuator | `Command` | SUMO light string (second conflict check) | `packages/sim/signals.py` |

## Boundaries (enforced by tests)

1. **Producers only describe what they see.** Raw per-class counts, no PCU,
   no trust judgement, no knowledge of neighbours.
2. **The core never reads simulator state.** It is a separate Rust crate
   whose only dependencies are serde and schemars (`tests/test_boundaries.py`).
   It sees Measurements and the static corridor map, nothing else.
3. **Vision depends on the contracts package only.** Not on the core, not on SUMO.
4. **Controllers never set signals.** They return a wish; the envelope owns
   every transition, and the actuator re-checks for conflicting greens.
5. **Ground truth stays in the harness.** `truth.jsonl` and metrics use the
   full-edge view; the core only ever gets the camera-zone view after faults.

## Repository layout

```
crates/core/        Rust decision core: contracts, corridor, state, guard, control, safety, engine
crates/py/          PyO3 bindings -> Python module flowguard_core
contracts/          flowguard.schema.json, generated from crates/core/src/contracts.rs
packages/contracts/ flowguard_contracts: pydantic models generated from the schema
packages/sim/       flowguard_sim: SUMO network, demand, observer, faults, signals, run, metrics
packages/vision/    flowguard_vision: camera -> Measurement (skeleton)
data/net/           SUMO network, corridor.json, vehicle types
data/demand/        saved demand per seed (committed, shared by every run)
config/             PCU weights, signal timing, fixed plan, demand, sim settings
scripts/            gen_contracts.py, parity.py
tests/              Python tests (contracts, binding, boundaries, signals, demand, smoke)
docs/               scope, architecture, contracts, decisions, results, specs/, team
```

## The contract is the source of truth

`crates/core/src/contracts.rs` defines every message. From it:

```
contracts.rs --(cargo run --bin gen_schema)--> contracts/flowguard.schema.json
             --(datamodel-codegen)----------> packages/contracts/.../models.py
```

`python scripts/gen_contracts.py` does both. A Rust test and a Python test
fail if either generated file is stale. Any language can read the JSON
Schema, so a component can be rewritten without touching the others.

Changing a contract: edit the Rust type, regenerate, update
`docs/contracts.md`, bump `SCHEMA_VERSION` major if an old reader would
misread the new message, and get a review from every module owner (see
`docs/team.md`).

## Why this stack

- **Rust core.** Typed, memory-safe, and the compiler checks every message
  shape. Pure logic, no I/O, so it is easy to unit-test and could later run
  standalone next to a cabinet.
- **JSON across the PyO3 boundary.** One serialisation path for the
  binding, the logs and any future network transport. At 3 junctions x 1 Hz
  the cost is irrelevant.
- **Python for sim, evaluation and vision.** SUMO TraCI, pandas, PyTorch,
  OpenCV and YOLO trackers are Python-first. The heavy vision compute runs
  in C++/CUDA underneath anyway.
- **uv workspace** for Python packages with separate dependency sets (vision
  will pull torch; the core and sim must not), **maturin** builds the
  extension, **pyright strict + ruff + clippy** keep both sides honest.

## Per-second data flow in a simulation run

```
t = SUMO time
view, truth = observer.observe(t)          # camera zone (60 m) and full edge
metrics.add_truth(truth)                   # max queue from truth
seen = faults.apply(t, view)               # fault injection hook
step = core.step(t, seen)                  # Guard -> controller -> envelope
signals.apply(step.commands)               # SUMO light strings, conflict check
log truth / measurements / states / commands
traci.simulationStep()
```

## Extending

- **New controller**: implement `Controller` in `crates/core/src/control/`,
  add a `ControllerConfig` variant, map it in `make_controller`, regenerate
  contracts, add a choice in `flowguard_sim/run.py:core_config`.
- **New Guard**: implement `Guard` in `crates/core/src/guard.rs`, add a
  `GuardConfig` variant. Keep `PassThrough` as the naive baseline.
- **New producer**: emit `flowguard_contracts.Measurement` objects. If they
  validate, the core accepts them.
- **New corridor**: write a `.net.xml` and a `corridor.json` (contract type
  `Corridor`), point `config/sim.json` at them. Nothing in the core changes.
