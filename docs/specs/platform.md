# Spec: Platform, simulation and evaluation (Module 4)

Owner: **Kabeer** (P1-P2, P5-P6), **Aimen** (P3-P4, with the vision data).
Code: `packages/sim`, `crates/core/src/safety.rs`, `scripts/`, CI.

## Goal

Make every experiment reproducible, comparable and cheap to run, keep the
safety envelope correct, and move from the placeholder corridor to a real one.

## Work items

- **P1 Experiment runner.** `uv run python -m flowguard_sim.experiment
  config/experiments/<name>.json`: a matrix of controllers x guards x fault
  scenarios x seeds, run in parallel processes (one SUMO per process),
  outputs `results/<name>/<run_id>/` plus `results/<name>/summary.csv` and an
  aggregate table (mean, standard deviation, 95 % interval per metric).
  Skips runs that already exist. Demand for each seed generated once.
- **P2 Continuous integration.** GitHub Actions on Linux: `cargo test`,
  `cargo clippy -- -D warnings`, `uv sync`, `uv run pytest`, `uv run ruff
  check`, `uv run pyright`. Required to merge into `main`.
- **P3 Real corridor.** Pick the Karachi corridor (3 signalised junctions),
  import with `netconvert --osm`, write `corridor.json` by hand (phases from
  site observation), check it loads in the core, point `config/sim.json`
  at it. Record choices in decisions.md.
- **P4 Demand from footage.** Turning counts per approach from the vision
  labels -> routes with SUMO `routeSampler`; a `replay` producer that feeds
  recorded Measurements to the core for offline runs.
- **P5 Sublane collisions.** SUMO reports a few collisions per hour with
  the sublane model; find out whether they are model artefacts, fix or
  document before final runs.
- **P6 Envelope for more than two phases.** The max-red feasibility check
  (D3) assumes two phases; generalise it, and test red accounting with 3-4
  phase junctions before the real corridor needs them.

## Requirements

- A run is fully determined by (corridor, demand seed, config files,
  fault scenario, code commit); the summary records all of them.
- Results never go into git; `docs/results.md` gets the tables.
- Any change to observer semantics needs the conservation test to pass and
  a decisions.md entry.

## Acceptance

- P1: one command reproduces every table in docs/results.md.
- P2: CI green on `main`; red CI blocks merges.
- P3/P4: real corridor runs end to end with both controllers.
