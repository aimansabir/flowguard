# Results log

Newest at the top. Each entry: date, commit, exact command, numbers, what it means.
One seed is a smoke check, not evidence; claims in the report need 10+ seeds.

## 2026-10-29: adaptive controllers vs the fair fixed plan (seeds 1-3, 3600 s)

Commit: this commit on `control/adaptive-test`, base `ee21c95`. Built and run
in the Linux container (docker/Dockerfile, repo mounted). Guard passthrough,
clean cameras. 90 runs. Baseline `fixed_webster_wave`, tuned on normal
demand and not re-tuned per scenario. Nothing tuned (D36). Three seeds:
direction, not significance.

```
python scripts/batch_run.py --out results/d35 --seeds 1 2 3 --jobs 8     --controllers fixed_webster_wave maxpressure_hyst_cap maxpressure_hyst_cap_lanes_moving actuated adaptive_split     --demands normal heavy uneven right_turn main_heavy serviceable
python scripts/summarise_runs.py results/d35 --baseline fixed_webster_wave
```

Controllers: `maxpressure_hyst_cap_lanes_moving` = A (D33, lane weight +
moving count), `actuated` = B (D34), `adaptive_split` = C (D35).
`serviceable` is a modelling choice (0.8 x normal, Y = 0.85, D36).
Avg delay = SUMO timeLoss; entry wait (not in timeLoss) beside it.
Paired rows: mean difference to the baseline, (seeds better / seeds).

#### normal (seeds 1, 2, 3)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| fixed_webster_wave | 265.0 ± 44.7 | 3.8 ± 1.4 | 2890.3 ± 28.5 | 332.3 ± 49.1 | 90.0 ± 0.0 |
| maxpressure_hyst_cap | 357.0 ± 15.7 | 89.9 ± 6.9 | 2308.0 ± 20.1 | 315.3 ± 3.5 | 90.0 ± 0.0 |
| maxpressure_hyst_cap_lanes_moving | 297.7 ± 19.1 | 80.1 ± 5.5 | 2532.7 ± 25.7 | 323.3 ± 21.5 | 90.0 ± 0.0 |
| actuated | 442.6 ± 60.5 | 67.5 ± 29.9 | 2033.0 ± 106.5 | 543.3 ± 102.5 | 90.0 ± 0.0 |
| adaptive_split | 228.4 ± 5.4 | 5.8 ± 3.8 | 2878.0 ± 59.8 | 363.0 ± 70.6 | 90.0 ± 0.0 |

vs `fixed_webster_wave`, same seeds: mean difference (seeds better / seeds)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| maxpressure_hyst_cap | +92.0 (+35 %) (0/3) | +86.1 (0/3) | -582.3 (-20 %) (0/3) | -17.0 (-5 %) (1/3) | +0.0 (0/3) |
| maxpressure_hyst_cap_lanes_moving | +32.7 (+12 %) (0/3) | +76.3 (0/3) | -357.7 (-12 %) (0/3) | -9.0 (-3 %) (2/3) | +0.0 (0/3) |
| actuated | +177.7 (+67 %) (0/3) | +63.7 (0/3) | -857.3 (-30 %) (0/3) | +211.0 (+63 %) (0/3) | +0.0 (0/3) |
| adaptive_split | -36.6 (-14 %) (3/3) | +2.0 (1/3) | -12.3 (-0 %) (2/3) | +30.7 (+9 %) (1/3) | +0.0 (0/3) |

#### heavy (seeds 1, 2, 3)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| fixed_webster_wave | 403.2 ± 35.9 | 217.5 ± 12.4 | 3179.3 ± 115.4 | 583.3 ± 56.0 | 90.0 ± 0.0 |
| maxpressure_hyst_cap | 494.9 ± 15.2 | 196.1 ± 21.7 | 2396.3 ± 80.5 | 506.3 ± 42.0 | 90.0 ± 0.0 |
| maxpressure_hyst_cap_lanes_moving | 442.7 ± 9.7 | 268.2 ± 27.3 | 2647.3 ± 19.5 | 501.7 ± 20.1 | 90.0 ± 0.0 |
| actuated | 530.8 ± 24.3 | 166.7 ± 7.3 | 2333.7 ± 3.8 | 709.7 ± 66.6 | 90.0 ± 0.0 |
| adaptive_split | 384.8 ± 9.1 | 219.9 ± 17.2 | 3244.0 ± 22.9 | 559.3 ± 12.2 | 90.0 ± 0.0 |

vs `fixed_webster_wave`, same seeds: mean difference (seeds better / seeds)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| maxpressure_hyst_cap | +91.7 (+23 %) (0/3) | -21.4 (-10 %) (2/3) | -783.0 (-25 %) (0/3) | -77.0 (-13 %) (3/3) | +0.0 (0/3) |
| maxpressure_hyst_cap_lanes_moving | +39.5 (+10 %) (1/3) | +50.7 (+23 %) (0/3) | -532.0 (-17 %) (0/3) | -81.7 (-14 %) (3/3) | +0.0 (0/3) |
| actuated | +127.6 (+32 %) (0/3) | -50.8 (-23 %) (3/3) | -845.7 (-27 %) (0/3) | +126.3 (+22 %) (0/3) | +0.0 (0/3) |
| adaptive_split | -18.4 (-5 %) (2/3) | +2.4 (+1 %) (2/3) | +64.7 (+2 %) (2/3) | -24.0 (-4 %) (1/3) | +0.0 (0/3) |

#### main_heavy (seeds 1, 2, 3)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| fixed_webster_wave | 216.6 ± 5.0 | 105.9 ± 17.5 | 2702.7 ± 8.1 | 270.7 ± 10.6 | 90.0 ± 0.0 |
| maxpressure_hyst_cap | 366.1 ± 5.8 | 265.4 ± 28.1 | 1918.3 ± 38.2 | 281.0 ± 15.1 | 90.0 ± 0.0 |
| maxpressure_hyst_cap_lanes_moving | 313.6 ± 6.1 | 222.4 ± 4.7 | 2116.3 ± 36.5 | 270.0 ± 15.6 | 90.0 ± 0.0 |
| actuated | 416.0 ± 25.0 | 262.1 ± 44.4 | 1774.7 ± 24.6 | 326.3 ± 52.7 | 90.0 ± 0.0 |
| adaptive_split | 221.0 ± 22.9 | 89.9 ± 22.1 | 2698.7 ± 30.9 | 280.7 ± 31.9 | 90.0 ± 0.0 |

vs `fixed_webster_wave`, same seeds: mean difference (seeds better / seeds)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| maxpressure_hyst_cap | +149.5 (+69 %) (0/3) | +159.5 (+151 %) (0/3) | -784.3 (-29 %) (0/3) | +10.3 (+4 %) (1/3) | +0.0 (0/3) |
| maxpressure_hyst_cap_lanes_moving | +97.0 (+45 %) (0/3) | +116.5 (+110 %) (0/3) | -586.3 (-22 %) (0/3) | -0.7 (-0 %) (1/3) | +0.0 (0/3) |
| actuated | +199.4 (+92 %) (0/3) | +156.1 (+147 %) (0/3) | -928.0 (-34 %) (0/3) | +55.7 (+21 %) (0/3) | +0.0 (0/3) |
| adaptive_split | +4.4 (+2 %) (2/3) | -16.1 (-15 %) (3/3) | -4.0 (-0 %) (2/3) | +10.0 (+4 %) (1/3) | +0.0 (0/3) |

#### right_turn (seeds 1, 2, 3)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| fixed_webster_wave | 232.6 ± 0.4 | 3.7 ± 1.3 | 2915.7 ± 15.9 | 303.3 ± 6.7 | 90.0 ± 0.0 |
| maxpressure_hyst_cap | 316.5 ± 11.5 | 111.1 ± 7.8 | 2364.7 ± 26.6 | 304.7 ± 20.2 | 90.0 ± 0.0 |
| maxpressure_hyst_cap_lanes_moving | 294.0 ± 21.4 | 96.5 ± 23.2 | 2536.7 ± 81.4 | 330.7 ± 25.5 | 90.0 ± 0.0 |
| actuated | 348.3 ± 13.7 | 119.7 ± 17.8 | 2176.3 ± 42.3 | 382.0 ± 61.9 | 90.0 ± 0.0 |
| adaptive_split | 219.0 ± 15.8 | 11.6 ± 0.6 | 2922.0 ± 37.2 | 301.7 ± 36.3 | 90.0 ± 0.0 |

vs `fixed_webster_wave`, same seeds: mean difference (seeds better / seeds)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| maxpressure_hyst_cap | +83.8 (+36 %) (0/3) | +107.4 (0/3) | -551.0 (-19 %) (0/3) | +1.3 (+0 %) (2/3) | +0.0 (0/3) |
| maxpressure_hyst_cap_lanes_moving | +61.4 (+26 %) (0/3) | +92.8 (0/3) | -379.0 (-13 %) (0/3) | +27.3 (+9 %) (0/3) | +0.0 (0/3) |
| actuated | +115.7 (+50 %) (0/3) | +116.0 (0/3) | -739.3 (-25 %) (0/3) | +78.7 (+26 %) (0/3) | +0.0 (0/3) |
| adaptive_split | -13.6 (-6 %) (2/3) | +7.9 (0/3) | +6.3 (+0 %) (1/3) | -1.7 (-1 %) (2/3) | +0.0 (0/3) |

#### serviceable (seeds 1, 2, 3)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| fixed_webster_wave | 136.1 ± 19.5 | 0.0 ± 0.0 | 2471.0 ± 24.4 | 154.3 ± 22.0 | 90.0 ± 0.0 |
| maxpressure_hyst_cap | 289.7 ± 20.1 | 26.2 ± 1.5 | 2151.3 ± 69.6 | 289.7 ± 16.0 | 90.0 ± 0.0 |
| maxpressure_hyst_cap_lanes_moving | 209.3 ± 30.2 | 8.3 ± 10.9 | 2339.0 ± 11.5 | 271.0 ± 8.7 | 90.0 ± 0.0 |
| actuated | 356.6 ± 30.6 | 37.8 ± 7.4 | 1954.7 ± 25.5 | 384.0 ± 56.3 | 90.0 ± 0.0 |
| adaptive_split | 145.8 ± 22.9 | 0.4 ± 0.7 | 2498.0 ± 44.5 | 127.7 ± 28.9 | 90.0 ± 0.0 |

vs `fixed_webster_wave`, same seeds: mean difference (seeds better / seeds)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| maxpressure_hyst_cap | +153.6 (+113 %) (0/3) | +26.2 (0/3) | -319.7 (-13 %) (0/3) | +135.3 (+88 %) (0/3) | +0.0 (0/3) |
| maxpressure_hyst_cap_lanes_moving | +73.3 (+54 %) (0/3) | +8.2 (0/3) | -132.0 (-5 %) (0/3) | +116.7 (+76 %) (0/3) | +0.0 (0/3) |
| actuated | +220.6 (+162 %) (0/3) | +37.8 (0/3) | -516.3 (-21 %) (0/3) | +229.7 (+149 %) (0/3) | +0.0 (0/3) |
| adaptive_split | +9.7 (+7 %) (1/3) | +0.4 (1/3) | +27.0 (+1 %) (2/3) | -26.7 (-17 %) (2/3) | +0.0 (0/3) |

#### uneven (seeds 1, 2, 3)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| fixed_webster_wave | 299.9 ± 5.6 | 171.0 ± 11.4 | 2286.3 ± 79.6 | 294.7 ± 36.1 | 90.0 ± 0.0 |
| maxpressure_hyst_cap | 318.0 ± 5.3 | 107.6 ± 6.7 | 2157.7 ± 16.3 | 284.3 ± 5.1 | 90.0 ± 0.0 |
| maxpressure_hyst_cap_lanes_moving | 258.0 ± 25.5 | 135.3 ± 25.0 | 2388.3 ± 70.7 | 248.7 ± 4.7 | 90.0 ± 0.0 |
| actuated | 384.3 ± 17.1 | 129.8 ± 26.1 | 1991.3 ± 20.5 | 438.3 ± 31.8 | 90.0 ± 0.0 |
| adaptive_split | 254.3 ± 26.1 | 66.1 ± 29.8 | 2687.3 ± 93.5 | 345.3 ± 50.1 | 90.0 ± 0.0 |

vs `fixed_webster_wave`, same seeds: mean difference (seeds better / seeds)

| controller | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network | longest red (s) |
|---|---|---|---|---|---|
| maxpressure_hyst_cap | +18.1 (+6 %) (0/3) | -63.4 (-37 %) (3/3) | -128.7 (-6 %) (0/3) | -10.3 (-4 %) (2/3) | +0.0 (0/3) |
| maxpressure_hyst_cap_lanes_moving | -41.9 (-14 %) (3/3) | -35.7 (-21 %) (2/3) | +102.0 (+4 %) (2/3) | -46.0 (-16 %) (3/3) | +0.0 (0/3) |
| actuated | +84.4 (+28 %) (0/3) | -41.2 (-24 %) (3/3) | -295.0 (-13 %) (0/3) | +143.7 (+49 %) (0/3) | +0.0 (0/3) |
| adaptive_split | -45.7 (-15 %) (3/3) | -104.9 (-61 %) (3/3) | +401.0 (+18 %) (3/3) | +50.7 (+17 %) (1/3) | +0.0 (0/3) |

**Camera faults, adaptive_split, seed 1, normal demand, `--faults mixed`**
(one seed: a smoke check):

```
python scripts/batch_run.py --out results/d35_faults_pt --seeds 1 --controllers adaptive_split fixed_webster_wave --faults mixed --guard passthrough
python scripts/batch_run.py --out results/d35_faults_v1 --seeds 1 --controllers adaptive_split fixed_webster_wave --faults mixed --guard v1
python -m flowguard_sim.guard_eval results/d35_faults_v1/adaptive_split_*/
```

| run | avg delay (s) | entry wait (s) | throughput (veh/h) | left in network |
|---|---|---|---|---|
| adaptive_split, clean cameras | 223.5 | 7.5 | 2915 | 341 |
| adaptive_split, mixed faults, Guard passthrough | 263.4 | 4.8 | 2804 | 453 |
| adaptive_split, mixed faults, Guard v1 | 248.7 | 15.7 | 2749 | 380 |
| fixed_webster_wave (reads no camera: same with any Guard) | 316.6 | 4.0 | 2858 | 389 |

guard_eval, Guard v1 run: false FAULTY 1.41 % of clean approach-seconds
(SUSPECT or FAULTY 4.74 %); undercount detected in 41 s, displaced in
161 s; freeze flagged SUSPECT but never FAULTY; blackout (frozen-at-zero
counts on a quiet side street) not detected; dropout: 776 of 776 missing
seconds covered by the virtual sensor.

Readings:

1. **adaptive_split (C) is the only new controller that beats the fair fixed
   plan, and only modestly.** Delay: normal -37 s (-14 %, 3/3 seeds), uneven
   -46 s (-15 %, 3/3, with +18 % throughput), right-turn heavy -14 s (-6 %,
   2/3), heavy -18 s (-5 %, 2/3), busy main road +4 s (tie), serviceable
   +10 s (+7 %, worse on 2/3). Throughput is within 1-2 % of the fixed plan
   except uneven. It moves green toward the busier phases inside the same
   cycle and offsets; where demand matches the plan's design (serviceable,
   main_heavy) there is nothing to gain and its noise costs a little.
2. **Max-pressure with lane weight + moving count (A) closes 27-65 % of
   max-pressure's delay gap to the fixed plan but still loses to it
   everywhere except uneven** (-42 s, 3/3). Its corridor entries still back
   up (entry wait 80 s against 4 s on normal demand). Switch counts were not
   logged in these runs, so why it still loses is not measured here.
3. **The actuated controller (B) is the worst in every scenario** (delay
   +28 % to +162 %), even at serviceable demand. Likely causes, not measured
   in these runs: with 60 m of zone as the detector, a vehicle entering it
   every few seconds keeps extending a green (gap 3 s) until max green or
   max red while three approaches wait, and side-street greens are capped
   at 15 s. A 60 m zone is not a calibrated passage-time detector.
4. **Under camera faults C still beats the fixed plan on this seed** (249 s
   with Guard v1, 263 s without, against 317 s); faults cost it 25-40 s, and
   the Guard recovers about 15 s of that, at the price of more entry wait.
   One seed only.
5. Longest red is 90 s for every controller (the envelope's max red binds).

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
