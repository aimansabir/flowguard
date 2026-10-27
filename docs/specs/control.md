# Spec: Control (Module 3)

Owner: **Kabeer**. Code: `crates/core/src/control/`.

## Goal

Controllers that pick sensible phases from Guard output, degrade gracefully
when approaches are SUSPECT or FAULTY, and give a fair set of baselines for
the experiments. Controllers only return a wish; the envelope owns safety.

## Current state

- `FixedTime`: 40/20 s greens, 70 s cycle, no offsets (D11), untuned.
- `FixedTime` from Webster (C5, D21): 25/10 s, 45 s cycle, optional green
  wave offsets; CLI `fixed_webster`, `fixed_webster_wave`. 58.5 s and
  50.7 s delay vs 94.1 s for the untuned plan (3 seeds). **This is the
  fixed-time baseline from now on.**
- `MaxPressure`: basic straight-through pressure, re-decided every second (D12).
- `MaxPressure` + `hysteresis` (C1, D20): holds min green, switches only for a
  gain above `gain_pcu` (default 8, config/maxpressure.json); CLI
  `--controller maxpressure_hyst`.
- `MaxPressure` + `capacity` (C3, D22): spillback gate; CLI
  `maxpressure_hyst_cap`. Never changes a decision on the placeholder
  corridor.
- Result (docs/results.md): basic max-pressure loses to the untuned fixed
  plan even with perfect data, and loses badly with a 60 m camera.
  Hysteresis removes the min-green interventions but not the gap (178.6 s
  vs 94.1 s for fixed, 3 seeds): the loss is not caused by switching.
  Against the fair Webster + green wave baseline (50.7 s) the gap is 3.5x
  with the 60 m camera and 2.3x with full-edge data (118.9 s).

## Work items (in order)

- **C1 Stabilise max-pressure.** *Done (D20); 10-seed confirmation still to
  run with the experiment runner.* Only consider switching once the current
  green has run min green, and require a pressure gain above a threshold
  (hysteresis, PCU, configurable). Goal: interventions near zero, delay not
  worse. Report both.
- **C2 Movement-based pressure (Varaiya 2013 form).** Split each approach
  queue over its movements using turning ratios estimated online from
  `departures` (exponential moving average, configurable half-life);
  pressure of a movement = its share of the queue minus the queue of the
  approach it feeds. Needs the core to expose departures per movement in
  state (contract change, coordinate with Guard owner).
- **C3 Capacity-aware max-pressure (Gregoire et al. 2015).** *Implemented
  (D22); no effect on this corridor, needs a spillback scenario and a
  measured jam spacing to evaluate.* Downstream
  storage = length x lanes / jam spacing (7.5 m per PCU, placeholder; add
  to the corridor or derive in the core). Use occupancy (queue / storage)
  and stop releasing into a link above an occupancy threshold (spillback
  protection).
- **C4 Trust-aware use of state.** TRUSTED: estimate as is. SUSPECT:
  estimate with its margin, choose conservatively (documented rule, e.g.
  upper bound for upstream, lower bound for downstream). FAULTY: Guard
  virtual estimate. All approaches of a junction FAULTY: that junction
  falls back to its fixed plan.
- **C5 Fair fixed-time baseline.** *Done (D21); 10-seed confirmation still
  to run.* Webster cycle and split from the
  demand config; optional offsets for a green wave along the main road.
  An untuned fixed plan is not a fair baseline.

## Requirements

- Every controller variant is a `ControllerConfig` entry with explicit
  parameters, no hidden constants.
- Deterministic: same inputs, same output (no RNG in controllers).
- Rust unit tests for each rule (tie-breaking, hysteresis, capacity cut-off,
  trust handling, fallback).
- No controller reads `measured_queue_pcu` directly; use `queue_estimate`
  and `margin`, which is what the Guard intends.

## Acceptance

For each variant: 10 seeds x {no faults, the Guard spec scenario suite},
with and without Guard v1, in the experiment runner (docs/specs/platform.md).
Table in docs/results.md: delay, throughput, vehicles left, interventions,
max queue, with spread across seeds.
