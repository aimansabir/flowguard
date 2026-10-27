# FlowGuard: scope

Status: research / final-year project prototype. Last revised 2026-10-25.

## Problem

Camera-based adaptive signal control trusts its cameras. A camera can keep
producing believable numbers while it is wrong: frozen feed, knocked out of
alignment, blur, darkness or glare, occlusion, a detector that undercounts.
The controller has no way to tell, so it times the signals from bad data.

FlowGuard puts a reliability layer (the **Guard**) between sensing and
control. It rates every approach TRUSTED / SUSPECT / FAULTY using evidence
the camera cannot fake on its own (neighbouring junctions, how queues
respond to green), and replaces bad readings with an estimate.

## Research question

Under realistic camera faults on a short corridor of mixed traffic, does
trust-aware adaptive control keep most of the benefit of adaptive control,
where naive adaptive control degrades below fixed-time?

### Hypotheses (each one is an experiment)

- **H1 (fault damage).** With injected camera faults and no Guard,
  max-pressure loses its advantage over a tuned fixed-time plan.
- **H2 (detection).** Guard v1 detects frozen feed, blackout and undercount
  within a bounded delay (target: under 60 s) with a false-alarm rate low
  enough not to matter in fault-free runs (target: under 1 % of approach-seconds).
- **H3 (recovery).** Trust-aware max-pressure (Guard v1 + virtual sensor)
  under faults recovers most of the gap between faulty naive max-pressure
  and fault-free max-pressure.
- **H4 (camera view).** A camera that sees only the last ~60 m of an approach
  hides long queues; conservation-based estimates recover part of that.
  Already observed: see `docs/results.md`.

## Prior art (what is not new, and what we build on)

Verify page numbers and editions before citing in the report.

- **Max-pressure control**: Varaiya, *Max pressure control of a network of
  signalized intersections*, Transportation Research Part C 36 (2013).
  Throughput-optimal under its assumptions; our baseline adaptive controller.
- **Capacity-aware max-pressure**: Gregoire, Qian, Frazzoli, de La Fortelle,
  Wongpiromsarn, *Capacity-aware backpressure traffic signal control*, IEEE
  Transactions on Control of Network Systems (2015). This is our planned
  spillback-aware extension; it is existing work, not our contribution.
- **Detector fault handling in practice**: standard signal controllers
  (NEMA TS 2 class) run detector diagnostics (no activity, maximum presence,
  erratic counts) and fall back to recall/fixed operation for a failed
  detector; see also the FHWA *Signal Timing Manual*, 2nd ed. (NCHRP Report
  812, 2015). So "detect a failure and fall back" is established practice.
- **Conservation checks on detector data**: Chen, Kwon, Rice, Skabardonis,
  Varaiya, *Detecting errors and imputing missing data for single-loop
  surveillance systems*, Transportation Research Record 1855 (2003).
  Freeway loops, offline; the idea of checking sensors against neighbours.
- **Simulator**: SUMO (Lopez et al., *Microscopic traffic simulation using
  SUMO*, IEEE ITSC 2018), sublane model for two- and three-wheelers.
- **Mixed-traffic PCU factors**: Indian Roads Congress IRC:106-1990 gives
  urban PCU factors for mixed traffic; our weights are placeholders until
  checked against it and against local counts.

### Our contribution, stated narrowly

1. A graded trust state (SUSPECT between working and failed) for camera
   approaches, driven by cross-junction conservation and command-response
   evidence, online, at 1 Hz.
2. A virtual-sensor fallback that keeps adaptive control running per
   approach instead of dropping the whole junction to fixed time.
3. Evaluation on heterogeneous traffic (motorcycles, rickshaws, qingqis)
   with a camera-sized observation zone, against fixed-time and naive
   adaptive baselines, with known injected faults as ground truth.

## In scope

- Placeholder 3-junction corridor in SUMO; later one real Karachi corridor
  imported from OpenStreetMap.
- Controllers: fixed-time (tuned), max-pressure, capacity-aware
  max-pressure, each with and without the Guard.
- Faults injected in simulation: freeze, blackout, undercount (scaled),
  displacement (wrong zone), intermittent dropout.
- Vision pipeline evaluated offline on recorded footage: counting accuracy,
  queue accuracy, movement classification, image-health flags.
- Safety envelope in software (min green, clearance, max red, no
  conflicting greens) as the stand-in for a real cabinet.

## Out of scope

- Driving real signals. A real deployment needs a traffic-authority
  partnership, NTCIP integration with the cabinet controller, and the
  cabinet hardware conflict monitor (which, not our envelope, is the real
  safety layer).
- Reinforcement-learning controllers (may appear later only as a comparison).
- Pedestrian phases, protected turns, emergency pre-emption, transit priority.
- Live video streaming infrastructure. Vision runs on recorded clips.

## Success metrics

Traffic (per run, docs/decisions.md D10): average delay per vehicle,
insertion delay, throughput, vehicles left in network, max queue, longest
red, envelope interventions, teleports and collisions.

Guard (per fault scenario): detection rate, false-alarm rate, detection
delay, recovery time after the fault clears, TRUSTED/SUSPECT/FAULTY
confusion matrix against injected ground truth.

Vision (offline): per-class count MAE per interval, queue MAE, departure
movement accuracy, image-health flag precision/recall.

Every claim is reported over at least 10 demand seeds with spread, not one run.

## Risks

- **Synthetic demand and placeholder PCU weights.** Results describe the
  model, not Karachi, until demand and weights are calibrated from footage.
- **Simulator camera is optimistic.** It never misses a vehicle inside its
  zone; the vision evaluation is what tells us real error levels, and the
  fault injector should use those levels.
- **Sublane collisions.** SUMO reports a few collisions per hour with the
  sublane model; they are logged, not hidden, and need a look before final runs.
