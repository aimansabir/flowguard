# Results log

Newest at the top. Each entry: date, commit, exact command, numbers, what it means.
One seed is a smoke check, not evidence; claims in the report need 10+ seeds.

## 2026-10-25: camera zone effect (seed 1, 3600 s, placeholder corridor)

Commit: `restructure` branch, after the Rust port. Commands:

```
uv run python -m flowguard_sim.run --controller fixed       --seconds 3600 --seed 1 [--full-edge]
uv run python -m flowguard_sim.run --controller maxpressure --seconds 3600 --seed 1 [--full-edge]
```

| | fixed, full edge | fixed, 60 m camera | max-pressure, full edge | max-pressure, 60 m camera |
|---|---|---|---|---|
| avg delay per vehicle (s) | 117.57 | 117.57 | 154.51 | **211.49** |
| avg insertion delay (s) | 0.01 | 0.01 | 0.01 | 7.74 |
| throughput (veh/h) | 3210 | 3210 | 3090 | 2936 |
| still in network at end | 89 | 89 | 209 | 363 |
| envelope interventions (min green) | 0 | 0 | 3006 | 3834 |
| teleports / colliding vehicles | 1 / 2 | 1 / 2 | 4 / 8 | 4 / 8 |

Readings:

1. **Basic max-pressure loses to the untuned fixed plan even with perfect
   full-edge data** (154.5 s vs 117.6 s). Likely causes, to test: no
   switching hysteresis (it asks to switch every second during min green,
   3006 interventions), straight-only downstream, and a fixed plan that
   happens to match this symmetric demand well. Fixing this is the first
   control task (docs/specs/control.md, C1).
2. **A camera that sees 60 m makes it much worse** (211.5 s, 363 vehicles
   stuck). Queues longer than the zone are invisible, so long main-road
   queues look the same as short ones. This is H4 in docs/scope.md and the
   first concrete motivation for the virtual sensor (docs/specs/guard.md, G3).
3. Fixed time is unaffected by the zone, as it should be.

## 2026-10-25: Rust port parity

Python baseline (commit 3cdb746) vs Rust core with `--full-edge`, seed 1,
3600 s: 10 800 Commands and 43 200 queue estimates compared per record,
0 differences for both controllers (`scripts/parity.py`). Summaries
identical: fixed 117.57 s / 3210 veh/h, max-pressure 154.51 s / 3090 veh/h /
3006 min-green interventions.
