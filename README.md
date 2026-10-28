# FlowGuard

Trust-aware traffic signal control for a short corridor watched by cameras.
Cameras (or the simulator) report raw vehicle counts per approach; a **Guard**
rates each approach TRUSTED / SUSPECT / FAULTY and replaces bad readings with
an estimate; a **controller** picks green phases; a **safety envelope** makes
every signal change legal. Tested in SUMO on mixed traffic (cars,
motorcycles, rickshaws, qingqis, buses, trucks).

Research / final-year project. What we claim and what we don't:
[docs/scope.md](docs/scope.md).

## Documents

| | |
|---|---|
| [scope](docs/scope.md) | problem, research question, hypotheses, prior art, in/out of scope, metrics |
| [architecture](docs/architecture.md) | pipeline, boundaries, layout, stack, how to extend |
| [contracts](docs/contracts.md) | every message field and unit |
| [decisions](docs/decisions.md) | why things are the way they are |
| [results](docs/results.md) | every number we may quote, with command and commit |
| [team](docs/team.md) | who owns what, schedule, workflow |
| specs | [vision](docs/specs/vision.md), [guard](docs/specs/guard.md), [control](docs/specs/control.md), [platform](docs/specs/platform.md) |

## Layout

```
crates/core/        Rust decision core: state, Guard, controllers, safety envelope (no I/O)
crates/py/          PyO3 bindings -> Python module flowguard_core
contracts/          JSON Schema generated from the Rust contract types
packages/contracts/ flowguard_contracts: pydantic models generated from the schema
packages/sim/       flowguard_sim: SUMO network, demand, observer, faults, signals, runs, metrics
packages/vision/    flowguard_vision: camera -> Measurement (skeleton)
data/               SUMO network, corridor map, saved demand
config/             PCU weights, signal timing, fixed plan, demand, sim settings
scripts/            gen_contracts.py (regenerate contracts), parity.py
tests/              Python tests; Rust tests live next to the code
```

## Setup

Needs [uv](https://docs.astral.sh/uv/) and Rust (`rustup`). uv installs
Python 3.12 itself; SUMO comes from pip (`eclipse-sumo`), no separate install.

Windows without Visual Studio (what we use, about 1 GB):

```powershell
winget install Rustlang.Rustup        # or rustup-init.exe from rustup.rs
rustup set profile minimal
rustup toolchain install stable-x86_64-pc-windows-gnu
rustup default stable-x86_64-pc-windows-gnu
```

Linux / macOS: the default `rustup` install works.

Then, in the repo:

```
uv sync          # builds the Rust extension and installs every package
```

`uv sync` rebuilds the extension whenever Rust sources change.

## Running

```
uv run python -m flowguard_sim.run --controller fixed       --seconds 3600 --seed 1
uv run python -m flowguard_sim.run --controller maxpressure --seconds 3600 --seed 1
uv run python -m flowguard_sim.run --controller maxpressure_hyst --seconds 3600 --seed 1  # C1, config/maxpressure.json
uv run python -m flowguard_sim.run --controller maxpressure_hyst_cap --seconds 3600 --seed 1  # + C3 spillback gate
uv run python -m flowguard_sim.run --controller fixed_webster      --seconds 3600 --seed 1  # C5, config/webster.json
uv run python -m flowguard_sim.run --controller fixed_webster_wave --seconds 3600 --seed 1  # C5 + green wave
uv run python -m flowguard_sim.webster                                                   # print the Webster plan
uv run python -m flowguard_sim.run --controller maxpressure --full-edge     # camera sees the whole approach
uv run python -m flowguard_sim.compare results/<run_a>/summary.json results/<run_b>/summary.json
```

Each run writes `results/<run_id>/`: `truth.jsonl` (full-edge ground truth),
`measurements.jsonl` (what the core received), `states.jsonl` (Guard output),
`commands.jsonl` (signals), SUMO `tripinfo.xml` and `sumo.log`, and
`summary.json`. Metric definitions: docs/decisions.md D10.

Demand per seed is generated once into `data/demand/` and reused, so every
controller sees the same vehicles. Rebuild the network after editing
`data/net/*.nod.xml` / `*.edg.xml`: `uv run python -m flowguard_sim.build_net`.
The network drives on the left, as in Karachi (`driving_side` in
`config/sim.json`); which turning movements are protected or must give way
comes from SUMO's conflict matrix (docs/decisions.md D29).

## Camera faults

`--faults <name>` corrupts what the core receives using the scenario
`config/faults/<name>.json` (freeze, blackout, undercount, dropout,
displaced; format in docs/specs/guard.md G0, behaviour in docs/decisions.md D23):

```
uv run python -m flowguard_sim.run --controller maxpressure --seconds 300 --seed 1 --faults smoke
uv run python -m flowguard_sim.run --controller maxpressure --seconds 3600 --seed 1 --faults mixed
```

`smoke` has every kind inside the first 300 s; `mixed` is the spec example.
A fault run also writes `faults.jsonl`: the active fault kind (or null) per
approach per second, the ground truth for scoring the Guard. Without
`--faults` nothing changes.

`--guard v1` rates every camera TRUSTED / SUSPECT / FAULTY and replaces a
FAULTY camera's queue with a virtual estimate (docs/decisions.md D26); the
default `--guard passthrough` trusts every camera. Score the Guard of a fault
run against its `faults.jsonl` (detection, delay, false alarms, recovery,
confusion matrix; D27):

```
uv run python -m flowguard_sim.run --controller maxpressure --seconds 3600 --seed 1 --faults mixed --guard v1
uv run python -m flowguard_sim.guard_eval results/<run_id>
```

The evidence scores themselves (D24, D25) are not in the run logs; a Python
copy replays them over a run, per check and approach:

```
uv run python -m flowguard_sim.evidence_replay results/<run_id>
```

## Checks

```
cargo test                                  # Rust: envelope, controllers, corridor, contracts
cargo clippy --all-targets -- -D warnings
uv run pytest                               # Python: contracts, binding, boundaries, observer, smoke
uv run ruff check
uv run pyright                              # strict
```

CI runs all of these on every pull request.

## Changing a message

Edit `crates/core/src/contracts.rs`, run `python scripts/gen_contracts.py`
(regenerates the schema and the Python models; tests fail if you forget),
update docs/contracts.md, and get a review from every module owner.
