# Message schemas (v1.0)

These three messages are the contract between perception, the Guard, control
and the simulator. Code: `schemas/messages.py`. All are JSON objects and all
carry `schema_version`. A change in the major number means old readers must
not accept the message (`from_dict` raises).

Times are simulation/wall seconds since the start of the run. Queues are in
PCU (passenger car units), using the weights in `config/pcu.json`, which are
**placeholders** until calibrated.

## Observation

Perception or simulator → Guard. One per camera approach per second.

| field | type | unit | meaning |
|---|---|---|---|
| `schema_version` | str | – | `"1.0"` |
| `time` | float | s | when the observation was taken |
| `junction_id` | str | – | e.g. `"J1"` |
| `approach_id` | str | – | the approach the camera watches. In SUMO this is the incoming edge id, e.g. `"W_J1"` |
| `counts_by_class` | object | vehicles | vehicles currently on the approach, keys exactly `car, motorcycle, rickshaw, qingqi, bus, truck` |
| `queue_pcu` | float | PCU | stopped vehicles on the approach (speed < 0.1 m/s), each weighted by its PCU |
| `exits_to` | object | vehicles | `{neighbour_junction_id: n}`: vehicles that came from this approach and entered the link toward that neighbour during the last second. Has a key for every neighbouring junction, 0 if none |
| `detector_confidence` | object or null | 0–1 | `{mean, std}` of detector scores over the second. Null from the simulator |
| `track_breaks_per_min` | float or null | 1/min | how often tracks are lost and restarted. Null from the simulator |
| `image_health` | object or null | – | `{frozen: bool, blur: 0–1, brightness: 0–1, displacement: px}`. Null from the simulator |
| `error_margin` | float or null | PCU | perception's own estimate of how wrong `queue_pcu` might be. Optional |
| `source` | str | – | `"sim"` or `"camera"` |

## RatedObservation

Guard → controller. The controller only ever sees these.

| field | type | unit | meaning |
|---|---|---|---|
| `schema_version` | str | – | `"1.0"` |
| `observation` | Observation | – | the observation as the Guard received it |
| `queue_estimate` | float | PCU | the queue the controller should use. Equals `queue_pcu` when trusted; the Guard may replace it (e.g. from neighbours) when it isn't |
| `margin` | float | PCU | ± uncertainty on `queue_estimate` |
| `trust_state` | str | – | `TRUSTED`, `SUSPECT` or `FAULTY` |
| `override_flag` | bool | – | true when `queue_estimate` is not the camera's own number |

## Command

Safety envelope → simulator, and fed back to the Guard and controllers the
next second. Controllers do **not** produce Commands: they only request a
green phase, and the envelope turns that into a Command.

| field | type | unit | meaning |
|---|---|---|---|
| `schema_version` | str | – | `"1.0"` |
| `time` | float | s | when this Command applies |
| `junction_id` | str | – | e.g. `"J1"` |
| `phase_index` | int | – | phase whose approaches are lit: the green phase, or the phase being cleared during yellow/all-red. Phase 0 = main road, 1 = side street on the placeholder corridor |
| `phase_started_at` | float | s | when the current `stage` began |
| `controller_name` | str | – | which controller was in charge, e.g. `"fixed"` |
| `stage` | str | – | `green`, `yellow` or `all_red` |
| `target_phase` | int | – | phase being switched to (equals `phase_index` when settled) |
| `requested_phase` | int or null | – | what the controller asked for this second, null if nothing |
| `intervention` | str or null | – | set when the envelope didn't do what was asked: `min_green`, `max_red` or `invalid_request` |

Signal for an approach under a Command: green if `stage == green` and the
approach is in `phase_index`'s phase, yellow if `stage == yellow` and it is,
red otherwise (`Junction.light` in `schemas/corridor.py`).

## Corridor map (`sim/net/corridor.json`)

Not a message but shared by everyone: junctions, their approaches, which
approaches each phase serves, which approach feeds which (`downstream`), and
the signal link order SUMO uses. It's static map knowledge, so the Guard and
controllers may read it. Loaded by `schemas/corridor.py`.
