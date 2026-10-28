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
  (`stretch * lanes / min_spacing_m`, 128.6 PCU). The allowance is the full
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
  Karachi's left-hand traffic (the SUMO network drove on the right when this
  was calibrated; it drives on the left since D29).
- **Parameters** (`EvidenceParams`, all provisional research values):
  `free_flow_mps` 13.89 (50 km/h urban limit, equal to SUMO's speed on the
  four main-road links E3 covers); `fast_factor` 1.2 (SUMO's fastest car,
  16.7 m/s); `slack_s` 30 (start from a stop, rickshaws and qingqis at
  11.1 m/s, crossing the junction, yielding left turns); `min_spacing_m` 3.5
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

## D26. Guard v1: trust state (G2), virtual sensor (G3), `GuardConfig` v1 (2026-10-28)
`{"kind": "v1"}` in `CoreConfig.guard` runs `crates/core/src/guard/v1.rs`;
`{"kind": "passthrough"}` stays the default and unchanged. Additive contract
change: every block and field of v1 has a default, so `{"kind": "v1"}` is
the Guard documented here. `run.py --guard v1` selects it.

- **Config.** `evidence` (the D24/D25 parameters, one block per check; E3's
  spacing renamed `min_spacing_m` so it cannot be mistaken for max-pressure's
  `capacity.jam_spacing_m`, which means a typical queue, D22), `combine`
  (`weights` per check, 0-1, default 1), `trust` and `virtual_sensor` below.
  `EvidenceParams::default()` is derived from the contract defaults, one
  source.
- **Combination.** The spec gives no rule. Noisy-OR over the checks that
  could judge this second: `score = 1 - product(1 - weight x check)`. A
  check that cannot judge (null) is left out, not taken as clean; no check
  at all gives no score. Chosen because it is the standard way to combine
  independent signs, never inflates a single check, stays in 0-1 and lets
  weak signs add up. Not tuned on any run. The weak checks keep their caps
  (E1, E2 standing queue, E4 stall: 0.5 each), so all three together reach
  0.875, below `faulty_score` 0.9: weak evidence alone can make an
  approach SUSPECT, never FAULTY (unit-tested).
- **Trust state machine** (spec G2), per approach, timers in seconds of
  measurements: TRUSTED -> SUSPECT once score >= `suspect_score` (0.5) for
  `suspect_s` (5) consecutive seconds; SUSPECT -> FAULTY once score >=
  `faulty_score` (0.9) for `faulty_s` (10) consecutive seconds (counted from
  when the strong evidence began, also while still TRUSTED; FAULTY is only
  reached through SUSPECT); SUSPECT or FAULTY -> TRUSTED after `recover_s`
  (30) clean seconds, a score at or above `suspect_score` resetting that
  timer. A second without a measurement, or with no score, holds the state
  and every timer. A clean second only counts toward recovery when E5 can
  judge (the camera's own consistency; not E3, which depends on its
  neighbours), so the seconds right after a gap hold the timer instead of
  passing as clean. Defaults are provisional: chosen before any run, so that
  a strong sign must last 10 s and a recovery 30 s; detection delay is then
  the evidence's own build-up plus 10 s.
- **Virtual sensor** (spec G3), every approach, every second, whether its
  camera is trusted or not: `Q(t+1) = max(0, Q(t) + A(t) - D(t))` in PCU,
  capped at what the approach can hold (length x lanes / `min_spacing_m`).
  A internal approach: PCU its feeders (every upstream approach and
  movement whose target it is, the E3 topology and the same
  `sent_toward` function) sent toward it, arriving `length / free_flow_mps`
  later (20.5 s on our main road). If a feeder sent nothing (missing) or is
  FAULTY, that second's amount is unknown and arrives at the trusted
  arrival rate instead. A entry approach: the trusted arrival rate. The
  trusted arrival rate is the mean of the measured zone arrivals over the
  last `arrival_window_s` (300) trusted seconds, however long ago; before
  any trusted second it is 0 (nothing to go on: the estimate can only
  drain). D: `saturation_flow` x lanes (0.5 PCU/s per lane, the spec's
  placeholder) while the envelope's Command shows the approach green (not
  yellow, all-red or red, and never the controller's request), never more
  than there is. Anchor: on every second the approach is TRUSTED with a
  score below `suspect_score`, Q is set to the measured queue, the arrivals
  join the trusted rate and the margin resets; a zone at least
  `congested_fraction` full is only a lower bound (Q = max(Q, measured)),
  because the queue may continue upstream of what the camera sees, so Q can
  exceed the camera's queue (spec: extend a camera queue beyond the zone).
  A suspect or faulty camera never anchors; the first trusted second after
  recovery re-anchors, dropping whatever error the estimate built up.
- **Output.**

  | trust | `queue_estimate` | `margin` | `override_flag` |
  |---|---|---|---|
  | TRUSTED | measured | 0 | false |
  | SUSPECT | measured | \|measured - Q\| + virtual margin | false |
  | FAULTY | Q | virtual margin | true |

  `margin` is a +/- range in PCU around `queue_estimate` for a conservative
  controller (control spec C4). Virtual margin =
  `margin_growth_pcu_per_s` (0.05) x seconds since the last anchor: 0 at a
  trusted reading, growing linearly while untrusted (spec: grows with time
  since the last trusted reading), never negative. A SUSPECT camera is kept,
  and its range covers both it and the virtual estimate. `override_flag`
  means exactly what docs/contracts.md says: the estimate is not the
  measured queue. `extend_queue_beyond_zone` (default false) would also
  report Q for a TRUSTED approach whose full zone hides a longer queue, with
  `override_flag` set; off by default because it changes what controllers
  see in fault-free runs (H1, H3), to switch on as its own experiment.
  Controllers are unchanged: they read `queue_estimate` as before (C4,
  using `margin`, is Module 3's).
- **Limits.** The point queue ignores travel inside the zone and start-up
  lost time; vehicles that turn off before the stop line, driveways and
  parking are not modelled. Entry estimates are only as good as the trusted
  arrival rate (no time-of-day profile). A FAULTY feeder is replaced by the
  downstream approach's own trusted rate, not by the feeder's virtual
  discharge.

## D27. Guard evaluation semantics (G4, 2026-10-28)
`flowguard_sim.guard_eval` scores a run's `states.jsonl` against its
`faults.jsonl` and writes `guard_eval.json` next to them.

- **Samples.** Every approach-second in `faults.jsonl`; truth = the fault
  kind active on that approach, or clean; decision = the logged
  `trust_state`, or MISSING when no state was logged (dropout). A run
  without `faults.jsonl` is all clean, over the approach-seconds with a
  state. FAULTY is the positive decision; SUSPECT is counted on its own,
  never folded into either side (the confusion matrix has TRUSTED, SUSPECT,
  FAULTY and MISSING columns per truth row).
- **Episodes.** A maximal run of consecutive seconds of one fault kind on
  one approach; it ends at the first second without it. One that lasts to
  the end of the log is censored (no end, no recovery). Repeated windows on
  one approach are separate episodes.
- **Detection.** Detected = FAULTY at least once inside the episode;
  detection delay = first FAULTY - start; flag delay = first SUSPECT or
  FAULTY - start. A verdict after the fault has ended does not count. A
  fault active from t = 0 starts at the first logged second.
- **Recovery.** First TRUSTED at or after the end - end (0 if already
  TRUSTED), skipping seconds without a state; none if never TRUSTED again
  before the log ends.
- **False alarms.** Clean approach-seconds with a state rated FAULTY (and
  SUSPECT or FAULTY) / clean approach-seconds with a state. Also reported
  outside recovery, i.e. without the seconds between the end of an episode
  on that approach and its return to TRUSTED, which recovery time already
  measures. A clean approach downstream of a faulty one is counted as clean
  (its E3 evidence may still fire: D25).
- **Dropout** episodes are scored like the others but the Guard cannot rate
  a second it never received; its MISSING column is the dropout itself.
- **Exploratory end-to-end runs (2026-10-28; not the formal experiment).**
  Guard v1 with every default, 3600 s, `maxpressure` and Module 3's
  `maxpressure_hyst_cap`, seed 1 with each single-kind scenario and `mixed`,
  and clean runs on seeds 1 and 2 (16 runs, Linux container, same code as
  CI). Clean runs: FAULTY on 0.00 % of approach-seconds on both seeds and
  both controllers, SUSPECT or FAULTY 0.00-0.73 % (weak E2/E4 evidence on
  gridlocked approaches). Detection delay: freeze 25-57 s, displaced
  41-75 s, undercount 32-38 s under `maxpressure` and 213-296 s under
  `maxpressure_hyst_cap`; recovery 31-109 s after the fault ends. The spec's
  blackout on entry road N2_J2 was never detected: an all-zero entry camera
  looks like an empty road (D24 limit). Dropout seconds are MISSING by
  construction. In fault runs, 0.67-1.55 % of clean approach-seconds were
  FAULTY, mostly approaches downstream of a faulty camera (E3 cannot say
  which of two cameras is wrong, D25) and recovery tails. One worked
  example (`maxpressure_hyst_cap`, `mixed`, J1_J2 frozen 900-1500 s):
  SUSPECT at 915 s, FAULTY at 929 s with the virtual queue (4.9 PCU)
  replacing the frozen 18.5 PCU and the margin growing to 29.5 PCU by
  1499 s, TRUSTED again at 1531 s. The H2 targets (scope.md) need 10 seeds
  and remain the formal Guard evaluation.

## D28. Why blackout and dropout were missed; E6 improbable silence (2026-10-28)
The exploratory runs (D27) detected freeze, undercount and displaced, never
the spec's blackout (N2_J2, 1200-1320 s) or dropout (E_J3). Two different
causes, neither a defect in E1-E5:

- **Blackout: not enough evidence in that scenario.** An all-zero camera
  is exactly what an empty road reports. On an entry approach no check
  could tell them apart: E2 counts a repeat only with vehicles in the zone
  or sent from upstream (none upstream of an entry), E3 needs an upstream
  camera, E4 sees no queue and no red-light departure, E5 is satisfied by
  zeros, and the simulator reports no image quality (E1). The one signal
  left is the camera's own history: on an approach that has been carrying
  r vehicles a second, T seconds with none at all have probability
  exp(-r T). Measured on the exploratory runs, natural silences on clean
  approaches never got below p of about 10^-2.1; the spec blackout, on a
  side street at about 0.06 PCU/s, only reached about 10^-3.2 in its last
  second: about six vehicles were expected in the whole two minutes. No
  sound test calls that FAULTY without flagging ordinary quiet spells. The
  same blackout on a main-road entry (about 0.2 PCU/s) is another matter:
  E1-E5 missed it too, and E3 then blamed the downstream camera for the
  vehicles that appeared from nowhere.
- **E6 improbable silence** (`guard/silence.rs`, `evidence.silence`,
  `combine.weights.silence`; additive contract change). Per approach, the
  run of consecutive all-zero measurements (zone, queue, arrivals,
  departures) of T seconds is scored from -log10 p, p = exp(-r T), with r
  the approach's trusted arrival rate (G3: mean arrivals over the last
  `arrival_window_s` trusted seconds) frozen when the silence starts, so
  the silence cannot lower its own rate. Score 0 down to `p_start` 10^-3,
  1 at `p_full` 10^-5, linear in -log10 p between: fixed significance
  levels, not fitted to a run (10^-3 is ten times rarer than the rarest
  natural silence seen). r is in PCU, which a motorcycle counts as half:
  fewer PCU than vehicles, so p comes out larger, i.e. conservative. No
  score (null) until a full trusted window backs r: a rate from the first
  few seconds is not a rate (found by a unit test: one car at t = 0 made r
  1 PCU/s and normal gaps FAULTY). Minimum detectable blackout: 4.8 x ln 10
  / r seconds to reach 0.9, i.e. 44 s at 0.25 PCU/s, 4.6 min at 0.04 PCU/s.
- **Dropout: an architecture gap, left open.** A missing measurement is
  not hidden from the core: the Guard holds trust and the virtual sensor
  keeps running. But the core emits a state only for approaches that sent
  a measurement, so there is no verdict to score and the controller's
  `queue()` reads a missing approach as 0. In the exploratory dropout run
  E_J3 was missing 719 of 3600 s while its real queue averaged 18.9 PCU
  (574 s above 5 PCU). Closing this means the core reporting a state for
  an approach without a measurement (the virtual estimate, flagged as not
  measured), which changes what `StepOutput.states` and the controllers'
  input mean: a decision for the team (Module 3 reads it), not made here.
- **Before and after E6, exploratory** (right-hand network as in D27,
  seed 1 fault runs and clean seeds 1-3, `maxpressure` and
  `maxpressure_hyst_cap`; only E6 differs). A 300 s blackout on the main
  entry W_J1: missed -> detected in 31 s under `maxpressure`, 13 s both
  times under `maxpressure_hyst_cap`. A 120 s blackout on internal link
  J3_J2: 52 -> 43 s and 67 -> 41 s. The spec blackout (side street N2_J2)
  and dropout: still missed, as expected above. Freeze, undercount,
  displaced and `mixed`: identical results. Clean runs: FAULTY 0.00-0.07 %
  of approach-seconds, unchanged; the main-entry blackout runs 0.00 ->
  0.07 % and 0.00 -> 0.20 %.

## D29. Karachi drives on the left: network and signal permissions (2026-10-28)
The placeholder corridor was built by netconvert with its default,
right-hand traffic; Karachi drives on the left. `config/sim.json`
`driving_side: "left"` now makes `build_net` pass `--lefthand`.

- **Audit of the right-hand model** (before the change). Every junction
  has two phases (main road, side street), each lighting all movements of
  both opposite approaches. Using SUMO's own conflict matrix
  (`<request foes response>` in the network): no two protected (`G`)
  movements of a phase cross or merge, every conflict inside a phase is
  between a protected movement and a permissive (`g`) one that gives way,
  so in its own right-hand world the model was consistent and SUMO enforced
  the yielding. Of 41 teleports in 16 calibration runs, 40 were rear-end or
  lane-change collisions on the links between junctions (sublane car
  following, scope.md) and one was two motorcycles going straight through
  J1 on the same internal lane; none came from a turning conflict, and no
  run had a jam or deadlock teleport. The long stalls seen earlier (D25:
  J2_J3 discharging nothing for about 300 s of green) are a permissive
  turn under heavy opposing flow: the left-turner waits inside the
  junction for a gap and blocks the shared straight-and-left lane behind
  it. Realistic behaviour of a permissive turn, a capacity effect, not a
  collision.
- **The real defect.** `signals.py` decided which movements yield by
  direction code: left turns (`l`, `L`, `t`) permissive, everything else
  protected. That is only right for right-hand traffic. On the left-hand
  network the turn across oncoming traffic is the right turn, and the old
  rule would have given it protected green against the opposing straight:
  18 conflicting protected pairs over the three junctions (tested). The
  envelope (Rust) only guarantees one phase at a time and the clearance
  times; it does not know movements, so nothing caught this.
- **Change.** `conflicts.py` reads, for each signalised junction, which
  links (lane-to-lane movements) are foes and which gives way to which,
  from the network itself (sumolib, keyed by signal link index). A link of
  the lit phase is protected unless it must give way to another link green
  at the same time, then permissive. On the right-hand network this
  reproduces the old states exactly (all three junctions, both phases); on
  the left-hand one the right turns are permissive, giving way to the
  opposing straight and left, and the left turns, the nearside turn, are
  protected. `SignalApplier` checks every phase at start-up and every state
  it sends: no approaches of two phases together (D4) and no two protected
  movements in conflict. Yellow and all-red are unchanged (envelope).
- **Two phases kept.** The audit found the two-phase permissive design
  movement-safe once permissions come from the conflict matrix. Protected
  turn phases would need phases of movements rather than approaches: a
  corridor contract change and controller changes (Module 3), justified by
  capacity at most, not by safety. Approach-level split phasing (one
  approach per phase) is already expressible in `corridor.json` if the
  team wants it; the controllers and the envelope handle any number of
  phases, and the start-up check validates whatever phases are given.
- **Compatibility.** Approaches, lengths, lanes, turn targets, entries,
  exits and phases in `corridor.json` are identical (movement names are
  geometric); only the signal link order changed, which only `signals.py`
  reads. No Rust, contract or controller change. The evidence fixture
  (D25) and the E1-E5 calibration were made on the right-hand network.
- **Comparison, exploratory** (same demand seeds and controllers, 3600 s,
  Guard passthrough, Linux container; right-hand -> left-hand). Normal
  demand, `fixed`, `fixed_webster`, `maxpressure`, `maxpressure_hyst_cap`
  on seeds 1 and 2 (16 runs): no junction collisions and no jam or deadlock
  teleports on either model; link collisions similar (10 -> 11 teleports,
  20 -> 22 colliding vehicles, the sublane rear-ends); fixed-time signal
  behaviour identical (309 and 480 greens); average delay within seed
  noise except `fixed` seed 1, 118 -> 78 s, whose right-hand run had the
  permissive-turn stall at J3 (max queue 108 -> 58.5 PCU); the max-pressure
  variants 3-13 % lower on both seeds. 1.5x demand, seed 1:
  `maxpressure_hyst_cap` 356 -> 303 s delay, 3195 -> 3362 veh/h, with 2
  junction collisions and 4 jam teleports on the right-hand network and
  none on the left-hand one; `fixed` 281 -> 274 s, no junction collisions
  on either. Two seeds and one high-demand seed show the left-hand model is
  not worse; they do not show it is better.
- **Limits.** Two phases with permissive crossing turns lose capacity when
  opposing flow is heavy (the stall above), which real Karachi junctions
  often answer with split phasing or a protected turn; this model does not
  test those. Turning lanes come from netconvert defaults for the
  hand-written network, not surveyed lane markings. The E1-E5 thresholds
  (D24, D25) and the evidence fixture were calibrated on the right-hand
  network; recalibrate when the corridor is final.

## D30. Guard v1 covers missing measurements and stops blaming the wrong camera (2026-10-28)
Two limits of D25/D28, fixed in `guard/v1.rs` without a contract change.

- **Missing measurements (dropout).** The core used to emit a state only for
  approaches that sent a measurement, so a controller read a camera whose
  link was down as an empty road (`DecisionInput::queue` gives 0): in the
  exploratory dropout run E_J3 was missing 719 of 3600 s with a real queue
  of 18.9 PCU on average. Guard v1 now gives such an approach a state from
  its virtual sensor: `queue_estimate` and `margin` from G3 (the margin keeps
  growing with time since the last trusted reading), `override_flag` true,
  measured fields 0, trust state held (an absent camera has not shown a
  fault in its counts). Passthrough is unchanged (no state, as before), and
  so are the controllers: they read `queue_estimate` as before and now get
  the estimate instead of 0. Missing seconds are still visible to every
  reader: the approach has no line in `measurements.jsonl`, and G4 scores
  them as MISSING and counts them as covered by the virtual sensor.
- **E3 blaming the downstream camera.** E3 compares two cameras and cannot
  say which one is wrong; in the exploratory runs an undercounting camera
  made the camera downstream of it FAULTY (J1_J2 at 699-864 s in `mixed`).
  Before combining, E3 on an approach is now scaled by 1 - the strongest
  suspicion among its feeders, where a feeder's suspicion is its own
  combined evidence without E3 (E1, E2, E4, E5, E6), or 0.5 if it is already
  SUSPECT, 1 if FAULTY. A disagreement that a faulty feeder explains no
  longer lands on its healthy neighbour; one that no feeder explains still
  does. Uses only the Guard's own evidence, never fault labels. Unit test:
  an undercounting feeder whose own E5 fires leaves the downstream camera
  TRUSTED, and the same test fails with the scaling switched off (checked).
  Limit: a downstream fault during a period its feeder is also suspect is
  only caught by the downstream camera's own checks.

## D31. Dashboard: five views on one stream, research results read-only (2026-10-28)
The live dashboard (`flowguard_sim/live.py` + `live.html`) is one page with
five views, Overview, Vision, Guard, Controller and Research, switched by the
URL fragment. All views read the same server-sent event stream and the same
frame history, so switching views never restarts or reconnects the
simulation; selections (camera, junction) persist between views.

What the backend sends was extended only to expose data that already
existed: per-class zone counts in each camera summary, the seed and run
length, the controller and Guard configurations the core was built with,
and, in replay, the run's injected faults (`faults.jsonl`) and the E1-E5
parity view. Nothing in the core, the controllers, SUMO or fault injection
changed.

Rules the page follows:
- The trust state, override and estimate shown are the core's outputs. The
  E1-E5 bars are the Python parity copy (D25) and are labelled as such; E6
  and the combined score are not exposed by the core and are not drawn.
- Injected faults are experiment ground truth, drawn in their own colour and
  lane, never mixed into the Guard's verdict.
- A missing measurement (`NO DATA`) is shown separately from the trust state.
- The Controller view recomputes max-pressure from the same estimates with
  the formula in `control/maxpressure.rs` (including the spillback gate) to
  explain a decision. When the exposed outputs cannot explain one, it says
  so instead of guessing.
- Signal colours on the map come from SUMO's own link states when the run
  provides them; a check over a 900 s run found them equal to the core's
  commands in every approach-second.
- The Vision view shows a camera image only if the video pipeline supplies
  one. With the simulated camera it renders a camera-style perspective of
  the approach from SUMO's network and vehicle positions, labelled as such:
  its boxes are the vehicles SUMO places in the 60 m zone (ground truth),
  never presented as YOLO detections, and the panel beside it puts the
  camera's reported counts next to them, so an injected fault is visible.

Research view: `GET /api/results` lists the run folders under `--results`
(default `results/`), reading `summary.json`, and scores runs that have a
Guard log with `guard_eval.evaluate` in a separate process, so a live run is
not slowed. It writes nothing into the run folders; a `guard_eval.json`
already there is used as is. Findings are labelled preliminary.
