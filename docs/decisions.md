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
