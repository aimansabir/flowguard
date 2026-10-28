# Decision log

Newest at the bottom. Say what was decided, why, and what would change it.
D1-D12 were written for the v1 Python implementation. Where a later entry
replaces one, the old entry says so; file paths in old entries are updated.

## D1. Plain dataclasses + JSON for the messages (2026-10-24)
**Superseded by D14.**
Standard library only, no pydantic. Each message has `to_json`/`from_json`
and a `schema_version`. Validation is minimal (vehicle classes, enum values).
Revisit if perception needs stricter validation of camera input.

## D2. Command has extra fields: `stage`, `target_phase`, `requested_phase`, `intervention`
The brief's Command only had the phase. Because the envelope owns yellow and
all-red, the Command has to say which stage is showing. Keeping the
controller's raw request and the envelope's reason in the same record means
you can see every override in the log without matching two files.

## D3. Controllers request a phase; the envelope owns every transition
A controller returns `{junction_id: wanted_phase}` and nothing else. The
envelope (`crates/core/src/safety.rs`, was `control/envelope.py`) applies, in this order:
1. finish any yellow â†’ all-red â†’ green already under way (requests are
   ignored until it's done, and that isn't counted as an intervention);
2. if any red phase would pass `max_red_s` once the clearance has run, start
   switching to the one that's been red longest;
3. otherwise switch to the requested phase if the current green has run for
   `min_green_s`, else hold it.
Min green is never cut short, even for max red. `SignalTiming.from_config`
refuses configs where `max_red_s < min_green_s + yellow_s + 2*all_red_s`,
because with two phases that's the condition under which a forced switch never
has to break min green.

Values (`config/signal.json`): min green 10 s, yellow 3 s, all-red 2 s,
max red 90 s. These are common textbook values, not Karachi traffic police
values. Check them against local practice.

## D4. Conflicting greens checked twice
By construction only one phase's approaches can be lit. As well as that,
`packages/sim/src/flowguard_sim/signals.py` checks every state string before sending it to SUMO and
raises if green links come from more than one phase. Left turns and u-turns
get permissive green (`g`, must yield) and everything else gets `G`.

## D5. Two phases per junction: main road, side street
This is the simplest legal plan for a 4-arm junction without protected left
turns. Phase lists come from `corridor.json`, so more phases need no code
change in the envelope or in max-pressure (the fixed plan needs one green per
phase).

## D6. Placeholder corridor written by hand, then netconvert
Hand-written nodes/edges (`data/net/*.nod.xml`, `*.edg.xml`) are easier to
read than netgenerate options. Three junctions 300 m apart, 300 m feeder
links at both ends, 200 m side streets. Main road: 2 lanes Ã— 3.2 m, 50 km/h.
Side streets: 1 lane Ã— 3.5 m, 40 km/h. `uv run python -m flowguard_sim.build_net` rebuilds the
`.net.xml` and derives `corridor.json` from it.

**To swap in the OSM corridor:** make a `.net.xml` with `netconvert --osm`,
then write `corridor.json` for it (the approaches and phase groups have to be
chosen by hand for real junctions, and `build_net.corridor_from_net` gives a
starting point). Then point `config/sim.json` at the new files. The observer,
Guard, controllers and envelope only use `corridor.json`, so none of them
change. Demand will also need replacing with counts from real footage.

## D7. Sublane model, vehicle types
`--lateral-resolution 0.8` turns on SUMO's sublane model (SL2015). Motorcycles,
rickshaws and qingqis use `latAlignment="arbitrary"` and a small `minGapLat`,
so they filter between cars. Sizes and speeds in `data/net/vtypes.add.xml` are
guesses. The type id is the class name, which is how the observer knows the
class.

## D8. Demand saved to a file, shared by every controller
`flowguard_sim/demand.py` makes `data/demand/corridor_seed<N>.rou.xml` once per seed and
the run script reuses it if it already exists. The file is committed. Each
summary stores the file's sha256 and `flowguard_sim.compare` warns if two runs
differ. So the controllers are compared on identical vehicles: same departure
times, types and origin/destination.
- Rates: 900 veh/h per main road direction, 150 veh/h per side street
  approach, multiplied by a Gaussian peak (+50 % at t = 1800 s, Ïƒ = 600 s).
- Main road traffic: 70 % straight through, the rest spread over all side
  exits. Side street traffic: 40 % straight across, the rest onto the main
  road in either direction.
- Mix: car 35 %, motorcycle 35 %, rickshaw 10 %, qingqi 8 %, bus 5 %, truck 7 %.
- One Bernoulli draw per second per entry (close to Poisson at these rates).
All guesses until we have counts from footage.

SUMO's own `--seed` (driver imperfection, etc.) is also set to the run seed,
so a run is fully repeatable.

## D9. Only the observer touches SUMO's state
**Observer details superseded by D16; the boundary itself now holds by construction (D13).**
`flowguard_sim/observer.py` is the only code that reads vehicle state. It produces
Observations, which go through the fault hook and then the Guard; the
controller only gets RatedObservations plus last second's Commands. Static
map data (`corridor.json`) is allowed. `tests/test_boundaries.py` fails if
anything in `guard/`, `control/` or `schemas/` imports traci, sumolib,
libsumo or `sim`.

Observer details:
- approach = the whole incoming edge (300 m main, 200 m side). A real camera
  sees less, and that will matter when we compare against camera data.
- queue = vehicles with speed < 0.1 m/s on the approach, PCU-weighted.
- `exits_to`: counted when a vehicle first appears on a link edge between two
  junctions, credited to the approach it was last seen on at the upstream
  junction.

`observations.jsonl` logs what the Guard received (after the fault hook).
Metrics use the observer's clean output. Once faults exist, the clean
observations should get a log file of their own.

## D10. Metric definitions
All from one run of `--seconds` simulated seconds (step 1 s).
- **Average delay per vehicle (s):** mean of SUMO's tripinfo `timeLoss` over
  vehicles that *finished their trip* before the end. `timeLoss` = actual
  travel time minus travel time at the vehicle's desired speed, so it
  includes time stopped at signals and in queues. Vehicles still driving at
  the end are not counted (we report how many there were).
- **Average insertion delay (s):** mean tripinfo `departDelay` for the same
  vehicles: how long a vehicle waited to enter the network after its
  scheduled time. Reported separately because `timeLoss` doesn't include it,
  and a controller that lets the entry queues back up would otherwise look
  better than it is.
- **Throughput (veh/h):** vehicles that finished their trip during the run Ã—
  3600 / run length.
- **Max queue per approach (PCU):** highest `queue_pcu` the observer reported
  for that approach in any second (ground truth, before faults).
- **Longest red per approach (s):** longest unbroken stretch of seconds where
  the approach was shown red (its own all-red included, yellow not), taken
  from the envelope's Commands.
- **Envelope interventions:** number of (junction, second) pairs where the
  envelope did something other than what the controller asked, split by rule
  (`min_green`, `max_red`, `invalid_request`). A controller that keeps asking
  to switch during a 10 s min green scores up to 10 for that one green.
- Also reported so nothing is hidden: departed / arrived / still in network,
  teleports (SUMO removes a vehicle stuck > 300 s), colliding vehicles.

## D11. Fixed-time plan
40 s main + 20 s side + 2 Ã— (3 s yellow + 2 s all-red) = 70 s cycle, same at
all three junctions with no offsets. That gives about 2:1 green, roughly in
line with the 900 vs 150 veh/h demand while keeping the side-street red well
under max red (47 s). Not optimised (no Webster); it's a baseline.

## D12. Max-pressure is the basic version
(Ported unchanged to `crates/core/src/control/maxpressure.rs`, see D15.)
`pressure(phase) = Î£ (queue on approach âˆ’ queue on the approach it feeds
downstream)`, using the Guard's `queue_estimate`, re-decided every second,
ties keep the current phase. Turning traffic isn't split out (only the
straight-through downstream counts), and there's no capacity weighting yet;
that's the next step for the control track.

## D13. Decision core in Rust, everything else in Python (2026-10-25)
State, Guard, controllers and the safety envelope live in `crates/core`, a
Rust library with no I/O and no simulator dependency. Python keeps the SUMO
harness, evaluation and vision, where the libraries are Python-first. Reasons:
the compiler checks every message shape, the core is pure and easy to test,
and it could later run on its own next to a cabinet. Cost: a second
language for the team and a build step (maturin). Speed was not a reason;
the core does a few decisions per second. Would change if the team cannot
work in Rust fast enough; the contract is language-neutral, so the core
could be rewritten without touching producers.

## D14. Contracts generated from the Rust types; JSON across PyO3
Replaces D1. `contracts.rs` is the single source of truth -> JSON Schema
(`schemars`) -> pydantic models (`datamodel-codegen`, pinned version).
Tests fail if either generated file is stale. The binding passes JSON text
both ways: one encoding for the binding, the logs and any future network
transport, at negligible cost (3 junctions x 1 Hz). Unknown fields are
rejected everywhere.

## D15. Port first, prove parity, then change behaviour
The Rust port reproduced the Python implementation exactly before anything
else changed: with the camera seeing the whole edge, seed 1, 3600 s, all
10 800 Commands and 43 200 queue values match for both controllers
(`scripts/parity.py`; baseline runs from commit 3cdb746). The envelope's
red-since bookkeeping keeps insertion order so ties break as in Python, and
max-pressure picks the first maximum like Python's `max`.

## D16. Measurement v2: raw counts from a camera-sized zone
Producers send raw per-class counts; PCU weighting and neighbour exits
moved into the core (weighting is calibration, not perception, and the
producer should not need the corridor graph). The simulated camera sees the
last `camera_zone_m` metres (60 m, placeholder for a pole camera) of each
approach. The observer also records arrivals into the zone and departures
over the stop line by movement (taken from the vehicle's route), which the
Guard needs for conservation checks and a virtual sensor. The full-edge view
is logged as `truth.jsonl` and used only for metrics. `--full-edge` restores
the v1 view. Tested: in both views, vehicles in view = previous + arrivals -
departures, every second.

## D17. Demand hash ignores line endings
`summary.demand_sha256` hashes the demand file with CRLF normalised to LF,
so Windows and Linux checkouts of the same file agree. Hashes in summaries
from before 2026-10-25 were taken over CRLF bytes and differ for that
reason only. `.gitattributes` now keeps text files LF in the repository.

## D18. Guard is a pluggable stage; pass-through stays as the baseline
`Guard` is a trait; `CoreConfig.guard` picks the implementation. The
current `PassThrough` trusts everything and is kept permanently as the
"naive adaptive" arm of every experiment.

## D19. Rust toolchain on Windows: GNU, no Visual Studio
Disk space on the dev machine is tight, so Rust uses the
`x86_64-pc-windows-gnu` toolchain (minimal profile). PyO3's
`generate-import-lib` feature links against CPython without MSVC. Dev builds
carry no debug info; release builds are stripped. Plain `cargo` commands
target only the core (`default-members`); the binding builds through
`uv sync`.

## D20. Max-pressure hysteresis is an option, not a new controller (2026-10-27)
`ControllerConfig::MaxPressure` gets an optional `hysteresis: {gain_pcu}`.
Absent (`{"kind": "maxpressure"}`, the old JSON): the basic version of D12,
unchanged, so the parity result and earlier runs still reproduce. Present:
the controller asks for the current phase while a change is under way or the
green has not run min green (read from the last Command, the envelope's own
timing), then asks for the best phase only if its pressure beats the current
phase's by more than `gain_pcu`; ties and gains at or below the threshold
keep the current phase. Controller name `maxpressure_hyst`, so logs say which
ran. The later control items (C2-C4) are meant to stack as further options on
the same controller instead of multiplying variants. `make_controller` now
gets the whole `SignalTiming`, because the controller needs `min_green_s`.
Schema version stays 2.0: the field is optional, and an old reader rejects it
(unknown field) rather than misreading it.
Default `gain_pcu` = 8 (config/maxpressure.json), from a 3-seed sweep
(docs/results.md, 2026-10-27): the largest gain with max-red interventions
near zero. Larger gains lower delay further but only by holding the main
road until the envelope forces max red. Would change: a 10-seed sweep on
seeds not used for tuning, or a different corridor/camera zone.

## D21. Webster plan is computed in Python and handed to the core as `fixed` (2026-10-27)
The core never sees demand (architecture boundary 2), so
`flowguard_sim/webster.py` works the plan out from config/demand.json,
config/pcu.json, config/signal.json and config/webster.json, and passes it as
an ordinary `fixed` config. Steps: design flow per approach by routing every
entry's destinations through the corridor map (fewest junctions); flow ratio
y per phase = highest flow per lane / saturation flow, over every junction,
because the contract has one plan for all junctions; lost time = one yellow +
all-red per phase; C = (1.5 L + 5) / (1 - Y) clamped to [40, 120] s; greens
split C - L by y, rounded, at least min green; plan refused if a phase would
be red longer than max red. Design flow is the peak of the hour by default
(`design_flow`), the usual choice for a fixed plan that must survive the peak.
Placeholders: saturation 1800 PCU/h/lane and progression speed 11 m/s.
The green wave is new contract field `offsets_s` on `fixed` (optional,
junction -> s): a junction asks at time t for what the plan says at
t - offset. Offsets follow one direction (eastbound from W_J1); a two-way
wave on a 45 s cycle isn't possible with these spacings. No offsets = the
old behaviour, byte for byte (same results in docs/results.md).
Would change: per-junction plans (contract change), measured saturation
flow, or a real corridor with different spacings.

## D22. Capacity-aware max-pressure is a spillback gate, in PCU (2026-10-27)
`maxpressure` gets an optional `capacity: {jam_spacing_m,
spillback_occupancy}`. Storage of an approach = length_m x lanes /
jam_spacing_m, derived in the core from the corridor map. When the approach
an approach feeds (straight through) holds at least spillback_occupancy of
its storage, that approach's pressure term becomes min(queue - downstream,
0): no positive pressure into a nearly full link, but a negative term is
kept. Not the full normalised form of Gregoire et al. 2015: pressure stays in
PCU so the hysteresis gain (D20) means the same with or without the gate,
and the options stack (`maxpressure_hyst_cap`). Name suffixes `_hyst`,
`_cap` show which options ran. On the placeholder corridor the gate never
changes a decision (docs/results.md, 2026-10-27), so its value is still
unshown. Would change: a spillback scenario where it does fire, or turning
movements (C2), where the downstream of a turn is a different link.

## D23. Fault injector semantics (G0, 2026-10-27)
`flowguard_sim/faults.py` implements the five faults of docs/specs/guard.md G0
between the observer and the core. The core only ever receives ordinary
Measurements, so the contract and the Rust code are unchanged.

- **Windows.** A fault is active for `start_s <= t < end_s`, `t` being the
  SUMO time of the observation. Two faults on the same approach may not
  overlap (back-to-back is fine), so each approach-second has at most one
  fault kind.
- **freeze.** The camera repeats the last measurement it produced before
  the fault (the uncorrupted one), whole: zone and queue counts, arrivals,
  departures and `quality`; only `time` advances. If the run starts frozen,
  the first measurement is repeated. `image_health` stays as the camera
  reports it (null in the simulator), so the Guard gets no frozen flag for
  free. Repeated arrivals and departures with a constant zone break the
  conservation invariant (D16); a real frozen pipeline might report zero
  flow instead. Revisit if the vision pipeline shows how its tracker behaves
  on a frozen frame.
- **blackout.** The measurement still arrives with every count zero (zone,
  queue, arrivals, all departures); `quality` is unchanged.
- **undercount.** Every class count in zone, queue, arrivals and each
  departure movement is thinned independently, Binomial(n, `keep`). So
  `queue` can exceed `zone` and conservation breaks, which a detector that
  misses whole vehicles consistently would not do. Revisit with the error
  levels the vision evaluation measures (scope.md risks).
- **dropout.** Each second in the window the measurement is missing with
  probability `p`; the core then has no state for that approach that
  second. Seconds that did get through are unchanged.
- **displaced.** The observer counts a second zone of the same length as
  the camera zone, moved `shift_m` back from the stop line
  (`SimObserver(shifted=...)`, only for approaches with a displaced fault);
  during the window that view replaces the camera view. Zone and queue counts
  and arrivals come from the shifted zone; `zone_length_m` stays the camera's.
  Departures stay the real stop-line departures, because docs/contracts.md
  defines `departures` as vehicles that crossed the stop line, and one field
  meaning differently under one fault would make every consumer check the
  fault. Consequence: the shifted zone's change no longer equals arrivals
  minus departures, which is evidence a Guard may use (E5), and a real
  displaced camera could not see the stop line at all, so this is optimistic.
  `shift_m + camera zone` must fit on the approach; a full-edge camera
  cannot be displaced.
- **Determinism.** Each fault has its own `random.Random` seeded with
  `"<scenario seed>:<position in the file>"`, so a scenario corrupts the
  same run the same way every time, and adding a fault does not change the
  draws of the others (reordering the file does). The run seed is not part
  of it: across demand seeds the same scenario draws the same numbers.
  Revisit if multi-seed evaluation needs independent fault draws per seed.
- **Ground truth.** With `--faults`, `faults.jsonl` has one row per approach
  per second, `{"time", "approach_id", "fault"}`, with the active kind or
  null; dropout is logged for its whole window, including seconds that got
  through (missing seconds are the absent rows of measurements.jsonl). The
  clean camera view is not logged separately. `summary.json` records
  `faults` (scenario name or null) and the run id includes the name.
- **Validation.** Scenario files reject unknown fields and kinds, a missing
  or stray parameter (`keep` for undercount, `p` for dropout, `shift_m` for
  displaced, nothing for the others), `keep`/`p` outside 0-1, `shift_m <= 0`,
  `end_s <= start_s`, unknown approaches, overlapping faults, a shift that does
  not fit, and two different shifts for one approach.
- **No scenario** means no change: the injector returns its input as is,
  the observer computes no shifted view and no `faults.jsonl` is written.
  Tested byte-for-byte against a run whose faults never start.

## D24. Guard evidence E1, E2, E5 (G1, 2026-10-27)
`crates/core/src/guard/evidence.rs` computes three of the five checks of
docs/specs/guard.md G1 for every approach each second. Each gives a score
in 0-1 (0 = nothing suspicious) or null when it has nothing to judge. No
Guard uses them yet: `PassThrough` is unchanged and combining the scores is
G2, so runs and contracts are unchanged. Parameters live in
`EvidenceParams` until G2 moves them into `GuardConfig`. Every threshold
is a placeholder, chosen on the seed-1 runs below; revisit with the error
levels from the vision evaluation.

- **E1 quality.** Any bad flag the producer reported (image frozen,
  brightness <= 0.05 or >= 0.95, blur > 0.7, displacement > 20 px, mean
  detector confidence < 0.3, track breaks > 30/min) scores 0.5, else 0;
  null when nothing was reported. Capped at 0.5 because a bad image is not
  proof of bad counts (scope.md), so it can never be full evidence alone.
  Displacement is not in the spec's list; added because vision reports it
  (V4) and it is the camera-side sign of a displaced camera. The simulator
  reports no quality, so E1 is always null in SUMO runs (D23).
- **E2 frozen counts.** Counts the intervals in which every count vector
  (zone, queue, arrivals, departures by movement) equals the last
  measurement received, the approach had green in last second's command, and
  the measurement should have changed: vehicles in the zone, or an upstream
  approach sent vehicles toward it in the last 60 s. Score = count / 20 s,
  capped at 1. A change resets the count; red, yellow, an empty zone with no
  supply, and a missing measurement hold it, so a camera frozen across
  several cycles keeps building evidence over its green seconds. A standing
  queue (all stopped, nothing in or out) caps the score at 0.5: a blocked
  exit gives exactly that under green. In the clean fixed-time run a
  spillback held S2_J2 at 11 stopped vehicles for 45 s of green, which
  scored 1 before the cap. Entry approaches get no supply, so an all-zero
  blackout there is indistinguishable from an empty road.
- **E5 internal consistency.** Residual per interval
  `zone(t) - zone(t-1) - arrivals(t) + departures(t)` (vehicles, 0 for the
  simulator by D16), only between consecutive measurements; a gap clears the
  history. Residuals are summed over sliding 5-interval blocks, so a vehicle
  reported up to a few seconds late cancels out (vision V3 allows late
  departures). Score = (mean |block sum| over the last 60 intervals - 1) / 1,
  clamped to 0-1; null until 30 consecutive intervals. Undercount and
  displaced break it; freeze only when its repeated arrivals and departures
  differ; blackout only at onset. Evidence persists for up to the window
  after a fault ends.
- **Calibration** (Python replica of the same logic over logged runs:
  maxpressure and fixed clean, maxpressure `mixed`, fixed `displaced`, and a
  scratch scenario with freeze on two entry approaches, blackout on two
  internal approaches, undercount keep 0.7 and displaced 20 m, under both
  controllers). Seconds with score 1: clean 0 for E2 and E5, also with
  synthetic noise (departures 0-3 s late, 5 % one-second zone flicker), apart
  from E5's tail after a fault ends. E2 freeze 93-97 %, internal blackout
  81-92 %. E5 undercount keep 0.4 88 %, keep 0.7 16-36 %; displaced 40 m
  19-64 %, 20 m 0-13 %; at score >= 0.5 these rise to 59-99 % and 8-87 %.

## D25. Guard evidence E3, E4 (G1, 2026-10-28)
`crates/core/src/guard/evidence/link.rs` (E3) and `response.rs` (E4)
complete G1. As in D24 the scores are 0-1 or null, no Guard uses them yet,
and `PassThrough`, runs and contracts are unchanged. `EvidenceTracker::new`
now takes the core's PCU weights.

- **Inputs the map lacks.** E3 needs a free-flow speed and how densely
  vehicles pack; the corridor map has neither, only SUMO's network files
  do. Decided by the module owner: internal provisional parameters, no
  contract change. If a real site needs them per link, add them to the
  corridor format together with the G2 `GuardConfig` change.
- **E3, conservation across junctions.** The feeders of an approach are
  every upstream approach and movement whose target is it (through traffic
  and side-street turns); approaches without feeders (entries) have no E3.
  Per link a running balance of PCU between the upstream stop line and the
  camera zone: + sent (feeder departures) - arrived (zone arrivals). Each
  second it is held between two bounds: at least what was sent too recently
  to have arrived (`stretch / (v * fast_factor)`, 13.5 s on our links, with
  stretch = link length - zone length), at most what was sent within the
  slowest trip (`stretch / v + slack_s`, 46.2 s) plus an allowance for a
  queue held on the link, and never more than the link stores
  (`stretch * lanes / jam_spacing_m`, 128.6 PCU). The allowance is the full
  storage while the downstream zone is congested and drains at saturation
  flow after. How far the balance had to be pushed back is the evidence:
  share = corrections / max(sent, `min_volume_pcu`) over `link_window_s`,
  score = (share - 0.2) / 0.2 clamped to 0-1. Both directions count
  (vehicles vanishing: undercount, blackout; appearing: double counting).
  While the allowance is above zero the balance is only known to lie
  between the bounds, so corrections then are a silent re-sync and only
  more than the link stores counts. Found in the seed-1 calibration: without
  this, an error stored during congestion surfaced as evidence up to 900 s
  after the fault, when the queue cleared. The bounds also stop small
  miscounts from building up. A fresh balance (start, or any missing
  measurement on the link or a feeder) assumes the link may be full, and is
  judged only once the allowance has run out and a slowest trip has passed
  (175 s on our links). A camera that sees the whole link leaves nothing to
  balance: no E3.
- **E4, command response.** Uses the envelope's Command, as `Signal`: green
  with its elapsed time, clearing (yellow or all-red of the approach's own
  phase), or red (another phase holds the junction). E2's `is_green` now
  reads the same function. Stalled green counts seconds of green past
  `startup_s` with a queue at the start of the interval, no departure, and
  every link the approach feeds known not to be full (E3 fill below
  `congested_fraction`). A departure under green resets it; red, clearing,
  an unknown or full exit and missing data hold it. It asks for departures,
  not a shrinking queue, so arrivals during green do not matter. Score =
  min(1, stalled / `stall_s`) * `stall_weight`: a queue that never
  discharges has innocent causes too. In the clean fixed-time run J2_J3's
  head vehicle was stuck inside junction J3 for about 300 s of green
  (gridlock; J2_J3's exits leave the corridor, so the spillback check cannot
  help), which scored as a full fault before the cap. Red departures: per
  movement, departures under red per minute of red over `red_window_s` (red
  time floored at `red_floor_s`); score = max over movements of
  (rate - tolerance) / tolerance clamped. The corridor map lights every
  movement of an approach with its phase (`signals.py`), so nothing may go
  on red here, and the map cannot express a free turn. Real video will need
  the tolerances raised, especially for the free nearside turn: left in
  Karachi's left-hand traffic (the SUMO network drives on the right, so its
  "left" is the far-side turn).
- **Parameters** (`EvidenceParams`, all provisional research values):
  `free_flow_mps` 13.89 (50 km/h urban limit, equal to SUMO's speed on the
  four main-road links E3 covers); `fast_factor` 1.2 (SUMO's fastest car,
  16.7 m/s); `slack_s` 30 (start from a stop, rickshaws and qingqis at
  11.1 m/s, crossing the junction, yielding left turns); `jam_spacing_m` 3.5
  (densest seen: clean seed-1 runs held 31 PCU in a 60 m two-lane zone,
  3.9 m per PCU-lane; set denser so storage is an upper bound; car-only
  queues are about 7 m); `congested_fraction` 0.5 (a queue reaching the back
  of the zone at about 7 m per PCU); `saturation_flow` 0.5 PCU/s per lane
  (1800 PCU/h, the usual base value); `link_window_s` 120 (longer than a
  signal cycle: fixed plan 70 s, max red 90 s); `min_volume_pcu` 10 (about
  300 PCU/h); `link_tolerance` 0.2; `startup_s` 4 (start-up lost time about
  2 s plus one saturation headway); `stall_s` 15; `stall_weight` 0.5;
  `red_window_s` 120, `red_floor_s` 60, `red_tolerance_per_min` 2 per
  movement.
- **Python copy.** `flowguard_sim.evidence_replay` is a line-by-line copy of
  E1-E5 for offline calibration on logged runs (the core exposes no
  evidence). `tests/fixtures/evidence_replay.json` (a 240 s maxpressure run
  of `config/faults/evidence_fixture.json`, with quality reports and red
  runners added and shorter windows) holds the inputs and the scores; the
  Rust test `matches_the_python_replay_fixture` and
  `tests/test_evidence_replay.py` both require them, so the two cannot
  drift. Rebuild with `python -m flowguard_sim.evidence_replay --fixture`.
- **Calibration, exploratory, not results to quote.** 3600 s runs, fixed and
  maxpressure, seeds 1 and 2 (seed-2 demand generated locally, not
  committed), each clean and with three scenarios: `mixed`; faults on the
  links between junctions (undercount and dropout upstream, freeze,
  blackout, undercount 0.5, displaced 40 m); freeze on entries, blackout on
  two links, undercount 0.7, displaced 20 m. Thresholds were set before the
  runs; seed 1 showed the two structural problems above (stall cap, silent
  re-sync); seed 2 was held out. Seconds with score 1, seed 1 / seed 2.
  Clean runs: E3 0 / 0 of 27,400 scored; E4 red 0 / 0; E4 stall never
  above 0.5 by design (300 / 51 s at 0.5, gridlock); E2 0 / 6 (seed 2,
  t = 90-95 s: first vehicles sent toward an empty link not arrived yet, a
  start-up transient of D24's 60 s supply window). Fault runs away from
  faults: E3 0 / 47, all 121-163 s after a fault on that link, whose
  corrections the window still holds. Share of fault seconds with score
  >= 0.5, seed 1 / seed 2: E3 blackout 67 / 75 %, undercount 57 / 45 %
  (keep 0.5 far higher than 0.7), freeze 63 / 94 %, displaced 5 / 15 %; E3
  on the approach downstream of a faulty feeder, undercount 30 / 58 %,
  freeze 23 / 55 %; E4 red departures, a freeze repeating its departures
  through red, 0 / 24 %; E4 stall (at 0.5), freeze 68 / 45 %.
- **Limits.** E3 compares two cameras and cannot say which is wrong: its
  evidence lands on the downstream approach, so G2 must weigh the feeders'
  own evidence. It is silent while a queue holds the link (except overfill)
  and unavailable for about 3 minutes after any missing measurement on the
  link or a feeder, so a feeder with frequent dropouts disables it (no
  balance, no evidence). It sees counts, not timing: a steady stream
  arriving late looks like one on time. Driveways, parking and other
  mid-link sources and sinks are not modelled; the simulator has none. E4
  cannot tell gridlock or an incident from a camera that misses departures,
  hence the cap.
