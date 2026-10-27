# Team, ownership and workflow

Proposed split, 2026-10-25. Owners decide inside their module and review
changes that touch it; anyone can work anywhere with the owner's review.

| person | owns | spec | main code |
|---|---|---|---|
| **Kabeer** | Control (Module 3); platform: experiment runner, CI, safety envelope (Module 4); integration of the whole loop | [control](specs/control.md), [platform](specs/platform.md) P1, P2, P5, P6 | `crates/core/src/control/`, `crates/core/src/safety.rs`, `packages/sim/` |
| **Khizer** | Guard (Module 2): fault injection, evidence, trust states, virtual sensor, Guard evaluation | [guard](specs/guard.md) | `crates/core/src/guard.rs` (+ new modules), `packages/sim/.../faults.py` |
| **Aimen** | Vision (Module 1) and field data: footage, calibration, labels, vision evaluation, real corridor and demand from counts | [vision](specs/vision.md), [platform](specs/platform.md) P3, P4 | `packages/vision/`, `data/` |

Shared: the contract (`crates/core/src/contracts.rs`), docs/scope.md,
docs/results.md, the final report.

## Why this split

- Each module has one owner and a stable interface (docs/contracts.md), so
  the three tracks can move in parallel and meet only at the contract.
- Vision needs footage and labelling time from day one; it has the longest
  lead time, so it starts in week 1 without waiting for anything.
- The Guard is the research contribution and needs the fault injector
  before anything else; it depends only on the simulator, which works today.
- Control and the experiment runner sit in the middle and are what turn the
  other two tracks into results, so the integration owner holds them.

## Proposed schedule (weeks from 2026-10-26; adjust to the real deadline)

| week | Kabeer | Khizer | Aimen |
|---|---|---|---|
| 1 | CI (P2); experiment runner (P1); C1 hysteresis | Fault injector + scenario suite (G0) | Calibration format + polygon tool; footage plan and permissions |
| 2 | C5 Webster baseline; C2 turning ratios | E1, E2, E5 evidence | Detection + tracking baseline, end-to-end Measurement on one clip |
| 3 | C3 capacity-aware | E3 conservation, E4 command response; state machine (G2) | Labelling; fine-tune six classes |
| 4 | C4 trust-aware control | Virtual sensor (G3); Guard eval (G4) | Image health + quality block; vision evaluation |
| 5 | Full matrix: H1-H4 over 10 seeds | Tune thresholds on the matrix; error levels from vision into the injector | Error table for the Guard; real corridor import (P3) |
| 6 | Real-corridor runs, results tables | Guard report section | Demand from counts (P4); vision report section |

## Workflow

- Branch per task: `<name>/<short-topic>` (e.g. `khizer/fault-injector`).
  Pull request into `main`, one review from another owner, CI green.
- `main` always builds and passes tests.
- **Contract changes** (`contracts.rs`): open the PR early, regenerate with
  `python scripts/gen_contracts.py`, update docs/contracts.md, tag all three
  owners. Major version bump if an old reader would misread the message.
- **Every decision that would surprise a teammate** goes in
  docs/decisions.md (what, why, what would change it).
- **Every number we may quote** goes in docs/results.md with commit, command
  and seeds.
- No results, videos or model weights in git. Record their location.

## Getting started (each person, once)

1. Install `uv` and Rust (`rustup`, GNU toolchain on Windows: see README).
2. `uv sync`, then `uv run pytest` and `cargo test`. All green = ready.
3. Read docs/scope.md, docs/architecture.md, docs/contracts.md and your spec.
