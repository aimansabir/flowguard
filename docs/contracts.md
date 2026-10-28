# Contracts (schema v2.0)

Source of truth: `crates/core/src/contracts.rs`. Generated:
`contracts/flowguard.schema.json` and `packages/contracts/src/flowguard_contracts/models.py`
(run `python scripts/gen_contracts.py`). All messages are JSON objects,
unknown fields are rejected, and every message carries `schema_version`. A
different major version is refused.

Times are seconds since the start of the run. Counts are vehicles. PCU
(passenger car units) values are computed **only in the core**, with the
weights in `config/pcu.json` (placeholders until calibrated).

Vehicle classes, always all six keys: `car, motorcycle, rickshaw, qingqi, bus, truck`.
Movements: `left, straight, right` (no U-turns).

## Measurement (producer -> core)

One per approach per interval.

| field | type | unit | meaning |
|---|---|---|---|
| `schema_version` | str | | `"2.0"` (defaults to current if omitted) |
| `time` | float | s | end of the interval |
| `interval_s` | float | s | interval length, 1.0 in the simulator, must be > 0 |
| `junction_id` | str | | must match the corridor map |
| `approach_id` | str | | incoming approach; SUMO edge id in the simulator |
| `source` | `sim` / `camera` / `replay` | | producer kind |
| `zone_length_m` | float | m | how far back from the stop line the producer sees, > 0 |
| `zone_counts` | ClassCounts | veh | vehicles in the zone at `time` |
| `queue_counts` | ClassCounts | veh | of those, stopped (speed < 0.1 m/s) |
| `arrivals` | ClassCounts | veh | entered the zone during the interval |
| `departures` | {left, straight, right: ClassCounts} | veh | crossed the stop line during the interval, by movement taken |
| `quality.detector_confidence` | {mean, std} or null | 0-1 | detector scores over the interval |
| `quality.track_breaks_per_min` | float or null | 1/min | tracks lost and restarted |
| `quality.image_health` | {frozen, blur, brightness, displacement_px} or null | | blur and brightness 0-1, displacement in pixels |
| `quality.count_error` | float or null | veh | producer's own error estimate on `zone_counts` |

Invariant a producer must keep, per approach:
`zone_counts(t) = zone_counts(t-1) + arrivals(t) - departures_total(t)`
(the simulator observer is tested for it; a camera will only approximate it,
and how badly it fails is evidence for the Guard).

## ApproachState (core: Guard output, controller input)

| field | type | unit | meaning |
|---|---|---|---|
| `time`, `junction_id`, `approach_id` | | | as the measurement |
| `measured_queue_pcu` | float | PCU | `queue_counts` weighted |
| `queue_estimate` | float | PCU | what controllers use; equals measured when trusted |
| `margin` | float | PCU | +/- uncertainty on the estimate |
| `trust_state` | `TRUSTED` / `SUSPECT` / `FAULTY` | | Guard verdict |
| `override_flag` | bool | | estimate is not the measured value |
| `arrivals_pcu`, `departures_pcu` | float | PCU | weighted flows this interval |
| `exits_to` | {junction: int} | veh | departures heading to each neighbour; every neighbour key present |

## Command (envelope -> signals)

| field | type | meaning |
|---|---|---|
| `time` | float | second this applies to |
| `junction_id` | str | |
| `phase_index` | int | phase lit: the green one, or the one being cleared |
| `phase_started_at` | float | when the current stage began |
| `controller_name` | str | `fixed`, `maxpressure`, ... |
| `stage` | `green` / `yellow` / `all_red` | |
| `target_phase` | int | phase being switched to; equals `phase_index` when settled |
| `requested_phase` | int or null | controller wish this second |
| `intervention` | `min_green` / `max_red` / `invalid_request` or null | set when the envelope overrode the wish |

Light for an approach: green if `stage == green` and the approach is in
`phase_index`'s phase, yellow if `stage == yellow` and it is, red otherwise.

## Corridor (static map, `data/net/corridor.json`)

| field | meaning |
|---|---|
| `junctions[]` | `id`, `phases` (phase -> approaches that get green), `links` (SUMO signal link order: approach + direction code) |
| `approaches[]` | `id`, `junction`, `length_m`, `lanes`, `movements` {left, straight, right: edge or null} |
| `entries[]` | edges where traffic enters: `edge`, `kind` (main/side), `through_exit` |
| `exits[]` | edges where traffic leaves: `edge`, `kind` |

Derived in the core, never stored: which approach a movement feeds, the
downstream approach for max-pressure, neighbouring junctions.

## CoreConfig

`pcu` (weights per class), `timing` (`min_green_s`, `max_red_s`, `yellow_s`,
`all_red_s`), `controller`, `guard`: `{"kind": "passthrough"}` (default: every
camera trusted as it is) or `{"kind": "v1", ...}` (Guard v1, docs/decisions.md
D26). v1's blocks, each optional with every field defaulted:

| block | fields | meaning |
|---|---|---|
| `evidence` | `quality`, `frozen`, `conservation`, `link`, `response`, `silence` | parameters of checks E1, E2, E5, E3, E4 (D24, D25) and E6 (`p_start`, `p_full`, D28) |
| `combine` | `weights` {`quality`, `frozen`, `conservation`, `cross_junction`, `stalled_green`, `red_departures`, `silence`} | 0-1 per check in the noisy-OR score, default 1 |
| `trust` | `suspect_score`, `suspect_s`, `faulty_score`, `faulty_s`, `recover_s` | G2 thresholds (score 0-1) and durations (s) |
| `virtual_sensor` | `arrival_window_s`, `margin_growth_pcu_per_s`, `extend_queue_beyond_zone` | G3 trusted-rate window (s), margin growth (PCU/s), report the virtual queue past a full zone while trusted |

Under v1, `queue_estimate` is the measured queue when TRUSTED or SUSPECT and
the virtual queue when FAULTY; `margin` is 0 when TRUSTED, |measured -
virtual| + the virtual margin when SUSPECT, the virtual margin when FAULTY.

Controllers (all optional fields may be left out or null):

| kind | fields | meaning |
|---|---|---|
| `fixed` | `green_s` [s per phase] | same plan at every junction |
| | `offsets_s` {junction: s} | start that junction's plan later (green wave, D21); missing = 0 |
| `maxpressure` | (none) | basic max-pressure (D12) |
| | `hysteresis` {`gain_pcu`} | hold min green, switch only for a pressure gain above `gain_pcu` PCU (D20) |
| | `capacity` {`jam_spacing_m`, `spillback_occupancy`} | storage = length x lanes / jam spacing; an approach whose downstream holds at least `spillback_occupancy` of its storage adds no positive pressure (D22) |
The core refuses `max_red_s < min_green_s + yellow_s + 2 * all_red_s`, a fixed
plan whose length differs from a junction's phase count, an offset for an
unknown junction, non-positive weights, a negative `gain_pcu`, and capacity
settings outside `jam_spacing_m > 0`, `0 < spillback_occupancy <= 1`.

## StepOutput

`states` (ApproachState per measured approach, corridor order), `requests`
(controller wish per junction, before the envelope), `commands` (one per junction).

## Changes from v1

- `Observation` -> `Measurement`; `queue_pcu` -> raw `queue_counts`; PCU moves to the core.
- `exits_to` removed from the producer; derived in the core from `departures`.
- New: `zone_length_m`, `zone_counts`, `arrivals`, `departures`, `interval_s`, `quality` block.
- `RatedObservation` -> `ApproachState` (flat, no embedded observation).
- `image_health.displacement` -> `displacement_px`; `error_margin` -> `quality.count_error`.
- Corridor map: lists instead of maps, approaches carry length, lanes and movements.
