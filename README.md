# FlowGuard

Trust-aware signal control for a chain of three intersections. Cameras count
vehicles, a Guard rates each camera approach TRUSTED / SUSPECT / FAULTY, a
controller picks green phases, and a safety envelope makes every signal
change legal. Control is tested in SUMO.

This is the shared foundation (M0 + M1): message schemas, a placeholder
corridor in SUMO, the full closed loop with stub Guard and two simple
controllers, logging and metrics. No real perception or Guard logic yet.

## Layout

```
schemas/      message formats + corridor map loader (the contract between tracks)
sim/          SUMO network, vehicle types, demand, observer, signal output
perception/   stub, see perception/README.md
guard/        fault injection hook + Guard stub
control/      fixed-time and max-pressure controllers, safety envelope
experiments/  run script, metrics, compare
config/       PCU weights, envelope timing, fixed plan, demand, which network to use
results/      run outputs (gitignored)
tests/
docs/         schemas.md, decisions.md
```

## Setup (Windows, PowerShell)

Needs Python 3.12 (SUMO's pip wheels lag behind the newest Python).

```powershell
cd flowguard
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

`eclipse-sumo` from pip brings the SUMO binaries along, so you don't need a
separate SUMO install or `SUMO_HOME`. If you do have `SUMO_HOME` set, that
copy gets used instead.

On Linux/macOS it's the same apart from `source .venv/bin/activate`.

## Running

```powershell
python -m experiments.run --controller fixed --seconds 3600 --seed 1
python -m experiments.run --controller maxpressure --seconds 3600 --seed 1
python -m experiments.compare results\<run_a>\summary.json results\<run_b>\summary.json
```

Each run writes `results/<run_id>/` containing `observations.jsonl`,
`rated.jsonl`, `commands.jsonl`, SUMO's `tripinfo.xml` and `sumo.log`, and
`summary.json`. The metrics are defined in `docs/decisions.md` (D10).

Demand for a seed is generated once into `sim/demand/corridor_seed<N>.rou.xml`
and reused, so different controllers always get the same vehicles.

Rebuild the network after editing `sim/net/*.nod.xml` / `*.edg.xml`:

```powershell
python -m sim.build_net
```

## Tests

```powershell
python -m pytest
```

These cover schema JSON round-trips, the envelope rules (min green, yellow +
all-red before each change, max red, no conflicting greens), a check that
`guard/` and `control/` never import the simulator, and a 300 s smoke run of
both controllers.

## Plugging in

- **Perception:** produce `schemas.messages.Observation` objects with
  `source="camera"`. See `perception/README.md`.
- **Guard:** replace `guard/guard.py` (keep `rate(observations, last_commands)`)
  and add faults in `guard/faults.py`.
- **Control:** a controller is any object with `name` and
  `decide(t, rated_observations, last_commands) -> {junction_id: phase}`.
  Register it in `experiments/run.py:make_controller`.
