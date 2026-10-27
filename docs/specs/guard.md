# Spec: Guard and fault injection (Module 2)

Owner: **Khizer**. Code: `crates/core/src/guard.rs` (Guard, Rust),
`packages/sim/src/flowguard_sim/faults.py` (injector, Python), Guard
evaluation script in `packages/sim`.

## Goal

Rate every approach TRUSTED / SUSPECT / FAULTY each second from evidence the
camera cannot fake on its own, and give controllers a usable queue estimate
when the camera is wrong. Measured against injected faults with known
ground truth.

## G0. Fault injector (do first)

Scenario files `config/faults/<name>.json`:

```json
{"seed": 7, "faults": [
  {"approach": "J1_J2", "kind": "freeze",     "start_s": 900,  "end_s": 1500},
  {"approach": "W_J1",  "kind": "undercount", "start_s": 600,  "end_s": 2400, "keep": 0.4},
  {"approach": "N2_J2", "kind": "blackout",   "start_s": 1200, "end_s": 1320},
  {"approach": "E_J3",  "kind": "dropout",    "start_s": 0,    "end_s": 3600, "p": 0.2},
  {"approach": "J2_J3", "kind": "displaced",  "start_s": 1800, "end_s": 3600, "shift_m": 40}
]}
```

| kind | what the core receives |
|---|---|
| freeze | the last pre-fault measurement repeated, `time` advancing, `image_health.frozen` left as the camera would report it (null in sim) |
| blackout | all counts zero |
| undercount | each count thinned binomially with probability `keep` (seeded) |
| dropout | measurement missing with probability `p` each second |
| displaced | counts taken from a zone shifted `shift_m` back from the stop line (observer support needed) |

Rules: never mutate inputs; deterministic per scenario seed; write
`faults.jsonl` (per approach-second: active fault kind or null) as ground
truth. `run(..., faults="name")` and a `--faults` CLI flag. Without a
scenario, behaviour is identical to today (tested).

## G1. Evidence (Rust, each unit-tested on its own)

- **E1 quality flags**: frozen, brightness near 0 or 1, blur high, detector
  confidence low, many track breaks. Weak evidence alone (decisions in
  scope.md: bad image is not proof of bad counts).
- **E2 frozen counts**: measurement identical (all count vectors) for N s
  while the approach shows green and the upstream sends vehicles.
- **E3 conservation across junctions**: vehicles the upstream approach sent
  toward this junction (`exits_to`, via the corridor movements) should
  arrive in this approach's zone within a travel-time window
  [L / v_free, L / v_free + slack]. Residual = expected - observed
  arrivals over a sliding window, normalised by expected.
- **E4 command response**: under green with a measured queue, departures
  should be above zero; a queue that never discharges under green is
  evidence. Under red, departures must be ~0.
- **E5 internal consistency**: `zone(t) - zone(t-1) - arrivals + departures`
  should be ~0; persistent violation is evidence.

Entry approaches (no upstream junction) only have E1, E2, E4, E5: state this
limitation in the report.

## G2. Trust state machine

Per approach, a score from the evidence, with hysteresis:
TRUSTED -> SUSPECT when score >= s1 for k1 s; SUSPECT -> FAULTY when
score >= s2 for k2 s; back to TRUSTED after r s clean. All thresholds in a
new `GuardConfig` variant (`{"kind": "v1", ...}`), not constants. SUSPECT
sets `margin`; FAULTY sets `override_flag` and uses G3.

## G3. Virtual sensor

`Q(t+1) = max(0, Q(t) + A(t) - D(t))` in PCU, where
- `A` = upstream `exits_to` toward this approach delayed by travel time
  (internal approaches) or the running mean arrival rate from trusted periods
  (entry approaches);
- `D` = saturation flow x lanes while the approach shows green (placeholder
  0.5 PCU/s per lane, i.e. 1800 PCU/h), capped at `Q`.
`margin` grows with time since the last trusted reading. The same
conservation estimate also extends a camera queue beyond `zone_length_m`
when the zone is full (results.md: the 60 m camera hides long queues).

## G4. Guard evaluation (Python, `flowguard_sim.guard_eval`)

From `states.jsonl` + `faults.jsonl`: detection rate, false-alarm rate
(approach-seconds flagged with no fault), detection delay, recovery time
after the fault ends, TRUSTED/SUSPECT/FAULTY vs fault/no-fault confusion
matrix. Per scenario and over seeds.

## Acceptance

- G0: scenario suite of at least 5 files covering every kind; injector tests.
- G1/G2: Rust unit tests per evidence and per state transition.
- H2 targets (scope.md) met or the miss explained: detection delay under
  60 s for freeze, blackout and undercount (keep <= 0.5); false alarms under
  1 % of approach-seconds on fault-free runs over 10 seeds.
- `PassThrough` remains available and unchanged.

## Interfaces to respect

- Guard input is `GuardInput` (measurements, corridor, last commands) and the
  states at face value. No simulator state, ever.
- New Guard parameters go through the contract (`GuardConfig`), regenerate
  with `python scripts/gen_contracts.py`, review with the other owners.
