# Spec: Control (Module 3)

Owner: **Kabeer**. Code: `crates/core/src/control/`.

## Goal

Controllers that pick sensible phases from Guard output, degrade gracefully
when approaches are SUSPECT or FAULTY, and give a fair set of baselines for
the experiments. Controllers only return a wish; the envelope owns safety.

## Current state

- `FixedTime`: 40/20 s greens, 70 s cycle, no offsets (D11), untuned.
- `MaxPressure`: basic straight-through pressure, re-decided every second (D12).
- Result (docs/results.md): basic max-pressure loses to the untuned fixed
  plan even with perfect data, and loses badly with a 60 m camera.

## Work items (in order)

- **C1 Stabilise max-pressure.** Only consider switching once the current
  green has run min green, and require a pressure gain above a threshold
  (hysteresis, PCU, configurable). Goal: interventions near zero, delay not
  worse. Report both.
- **C2 Movement-based pressure (Varaiya 2013 form).** Split each approach
  queue over its movements using turning ratios estimated online from
  `departures` (exponential moving average, configurable half-life);
  pressure of a movement = its share of the queue minus the queue of the
  approach it feeds. Needs the core to expose departures per movement in
  state (contract change, coordinate with Guard owner).
- **C3 Capacity-aware max-pressure (Gregoire et al. 2015).** Downstream
  storage = length x lanes / jam spacing (7.5 m per PCU, placeholder; add
  to the corridor or derive in the core). Use occupancy (queue / storage)
  and stop releasing into a link above an occupancy threshold (spillback
  protection).
- **C4 Trust-aware use of state.** TRUSTED: estimate as is. SUSPECT:
  estimate with its margin, choose conservatively (documented rule, e.g.
  upper bound for upstream, lower bound for downstream). FAULTY: Guard
  virtual estimate. All approaches of a junction FAULTY: that junction
  falls back to its fixed plan.
- **C5 Fair fixed-time baseline.** Webster cycle and split from the
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
