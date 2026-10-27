# Results log

Newest at the top. Each entry: date, commit, exact command, numbers, what it means.
One seed is a smoke check, not evidence; claims in the report need 10+ seeds.

## 2026-10-27: C5 Webster fixed time and C3 spillback gate (seeds 1-3, 3600 s)

Commit: uncommitted on `module3/khizer`, base `7ef8217`. 3 seeds, not the
acceptance matrix. Mean over seeds (± sample standard deviation for delay).
Fixed-time plans don't read the camera, so their rows hold for any zone.

```
uv run python -m flowguard_sim.run --controller fixed_webster        --seconds 3600 --seed N
uv run python -m flowguard_sim.run --controller fixed_webster_wave   --seconds 3600 --seed N
uv run python -m flowguard_sim.run --controller maxpressure_hyst_cap --seconds 3600 --seed N [--full-edge]
uv run python -m flowguard_sim.webster     # plan: green_s [25, 10] (45 s cycle), offsets J1 0, J2 25.9, J3 6.8 s
```

| variant | view | avg delay (s) | throughput (veh/h) | still in network | interventions min green / max red |
|---|---|---|---|---|---|
| fixed 40/20 (D11) | any | 94.1 ± 20.3 | 3180 | 89 | 0 / 0 |
| **fixed Webster 25/10** | any | **58.5 ± 2.2** | 3185 | 83 | 0 / 0 |
| **fixed Webster + green wave** | any | **50.7 ± 4.4** | 3195 | 73 | 10 / 0 |
| max-pressure + hysteresis 8 | 60 m | 178.6 ± 17.6 | 2966 | 302 | 0 / 4 |
| ... + capacity gate 0.9 | 60 m | 178.6 ± 17.6 | 2966 | 302 | 0 / 4 |
| max-pressure + hysteresis 8 | full edge | 118.9 ± 28.0 | 3137 | 131 | 0 / 4 |
| ... + capacity gate 0.9 (also 0.6, 0.5) | full edge | 118.9 ± 28.0 | 3137 | 131 | 0 / 4 |

Readings:

1. **The fixed baseline was unfair, as the spec suspected.** Webster from
   the demand config cuts fixed-time delay by 38 % (94.1 -> 58.5 s) with
   less spread across seeds; a green wave for the eastbound main road takes
   it to 50.7 s. The 10 min-green interventions with the wave are all in the
   first 25 s: J2 and J3 start in phase 0 while their offset plan says
   phase 1, and the envelope holds min green once. Every claim that adaptive
   control beats fixed time must now be made against `fixed_webster_wave`.
2. **Max-pressure is now 3.5x worse than the fair baseline with the 60 m
   camera (178.6 vs 50.7 s) and 2.3x with full-edge data (118.9 s).** The
   camera costs 60 s on its own (H4, the case for the virtual sensor G3);
   the rest is the controller.
3. **The spillback gate changes nothing on this corridor.** Results are
   identical run for run, even at thresholds 0.5 and 0.6 with full-edge
   data. In the 0.5 run on seed 1 the gate condition held for 128 seconds,
   but in every one of them the upstream queue was already no longer than
   the downstream one, so the plain max-pressure term was already <= 0: the
   downstream subtraction already avoids pushing into fuller links here.
   Under max-pressure no internal link went above 68 % of storage. The gate
   needs a scenario with real spillback risk (higher demand or shorter
   links) to be tested; kept at 0.9, off by default.
4. **Jam spacing 7.5 m per PCU is too long for this traffic.** Under the
   old fixed plan J1_J2 and J2_J3 held up to 132-142 % of their computed
   storage: motorcycles and qingqis pack tighter than one PCU per 7.5 m.
   Needs measuring before the capacity numbers mean anything.

 (seeds 1-3, 3600 s, 60 m camera)

Commit: uncommitted on `module3/khizer`, base `7ef8217`. Tuning sweep, not
the acceptance matrix: 3 seeds, the same seeds the gain was chosen on.
Each cell is the mean over the seeds (± sample standard deviation for delay).

```
uv run python -m flowguard_sim.run --controller fixed            --seconds 3600 --seed N
uv run python -m flowguard_sim.run --controller maxpressure      --seconds 3600 --seed N
uv run python -m flowguard_sim.run --controller maxpressure_hyst --seconds 3600 --seed N   # gain_pcu in config/maxpressure.json
```

| variant | avg delay (s) | insertion delay (s) | throughput (veh/h) | still in network | interventions min green / max red |
|---|---|---|---|---|---|
| fixed | 94.1 ± 20.3 | 0.00 | 3180 | 89 | 0 / 0 |
| maxpressure (basic) | 198.7 ± 11.6 | 7.97 | 2928 | 331 | 3744 / 0 |
| hysteresis, gain 0 | 198.7 ± 11.6 | 7.97 | 2928 | 331 | 0 / 0 |
| hysteresis, gain 1 | 196.4 ± 9.5 | 4.62 | 2933 | 329 | 0 / 0 |
| hysteresis, gain 2 | 189.8 ± 8.4 | 7.13 | 2935 | 320 | 0 / 0 |
| hysteresis, gain 4 | 191.8 ± 14.2 | 4.71 | 2934 | 326 | 0 / 1 |
| **hysteresis, gain 8 (default)** | **178.6 ± 17.6** | 2.03 | 2966 | 302 | 0 / 4 |
| hysteresis, gain 12 | 160.8 ± 13.1 | 0.32 | 2989 | 279 | 0 / 9 |
| hysteresis, gain 16 | 123.6 ± 25.3 | 0.00 | 3114 | 154 | 0 / 20 |
| hysteresis, gain 24 | 144.7 ± 24.1 | 0.00 | 3112 | 156 | 0 / 55 |
| hysteresis, gain 32 | 225.9 ± 58.9 | 9.74 | 2917 | 302 | 0 / 99 |

Readings:

1. **The min-green interventions were harmless.** Gain 0 (hold min green,
   switch on any strict gain) gives the same traffic, run for run, as basic
   max-pressure: the envelope was already holding every green and switched
   at the same second. Interventions go from about 3700 per hour to 0, delay
   doesn't change. The earlier guess (2026-10-25 reading 1) that missing
   hysteresis causes the loss to fixed time is wrong.
2. **Small gains help a little.** Up to 8 PCU, delay drops about 20 s with
   max-red interventions near zero. Still about twice the fixed plan.
3. **Large gains turn into a main-road bias.** From about 12 PCU the side
   street rarely beats the main road by the gain, so it only gets green when
   the envelope forces it at max red (interventions climb with the gain).
   Gain 16 is closest to fixed (123.6 s) but works by starving side streets
   until max red, and the envelope is then doing the control. With a 60 m
   camera the main-road queue seen is capped at roughly 60 m x 2 lanes /
   7.5 m = 16 PCU, which may be why the effect peaks there; not yet tested.
4. Default set to 8: the largest gain that meets C1's goal (interventions
   near zero, delay not worse). The remaining gap to fixed time is for C3,
   C5 and the virtual sensor (G3), not for switching damping.

 (seed 1, 3600 s, placeholder corridor)

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
