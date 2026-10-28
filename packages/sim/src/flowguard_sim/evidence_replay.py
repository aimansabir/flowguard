"""Python copy of the Guard evidence checks in crates/core/src/guard/evidence*.rs.

The core runs the Rust code and does not expose evidence. This copy replays
logged runs offline, to choose and check thresholds (docs/decisions.md D24,
D25). tests/fixtures/evidence_replay.json holds inputs from a real fault run
and the scores both implementations must give, so the two cannot drift apart.

    uv run python -m flowguard_sim.evidence_replay results/<run> [results/<run> ...]

prints, per check and approach, the seconds with high evidence: during each
fault kind on that approach, during a fault on an approach feeding it, and
clean (no fault on it or its feeders). Exploratory, not a result to quote.
"""

import argparse
import json
from collections import defaultdict, deque
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from flowguard_sim.paths import ROOT, load_json

Json = dict[str, Any]
CLASSES = ("car", "motorcycle", "rickshaw", "qingqi", "bus", "truck")
MOVEMENTS = ("left", "straight", "right")
FIELDS = ("quality", "frozen", "conservation", "cross_junction", "stalled_green", "red_departures")
Evidence = tuple[float | None, float | None, float | None, float | None, float | None, float | None]
NONE: Evidence = (None, None, None, None, None, None)
AFTER_S = 120.0  # summaries label this long after a fault "after <kind>": windows still hold it
Vec = tuple[int, int, int, int, int, int]


@dataclass(frozen=True)
class Params:
    """Same fields, order and defaults as EvidenceParams in evidence.rs."""

    brightness_edge: float = 0.05
    blur_max: float = 0.7
    confidence_min: float = 0.3
    track_breaks_max: float = 30.0
    displacement_max_px: float = 20.0
    quality_weight: float = 0.5
    frozen_s: float = 20.0
    supply_s: float = 60.0
    standing_weight: float = 0.5
    window: int = 60
    block: int = 5
    tolerance: float = 1.0
    free_flow_mps: float = 13.89
    fast_factor: float = 1.2
    slack_s: float = 30.0
    jam_spacing_m: float = 3.5
    congested_fraction: float = 0.5
    saturation_flow: float = 0.5
    link_window_s: float = 120.0
    min_volume_pcu: float = 10.0
    link_tolerance: float = 0.2
    startup_s: float = 4.0
    stall_s: float = 15.0
    stall_weight: float = 0.5
    red_window_s: float = 120.0
    red_floor_s: float = 60.0
    red_tolerance_per_min: tuple[float, float, float] = (2.0, 2.0, 2.0)

    def to_json(self) -> Json:
        d = asdict(self)
        d["red_tolerance_per_min"] = list(self.red_tolerance_per_min)
        return d

    @staticmethod
    def from_json(d: Json) -> "Params":
        fields: dict[str, Any] = dict(d)
        fields["red_tolerance_per_min"] = tuple(d["red_tolerance_per_min"])
        return Params(**fields)


def vec(c: Json) -> Vec:
    return (int(c["car"]), int(c["motorcycle"]), int(c["rickshaw"]), int(c["qingqi"]),
            int(c["bus"]), int(c["truck"]))


def pcu(c: Vec, w: Vec | tuple[float, ...]) -> float:
    """ClassCounts::pcu, summed in the same order."""
    return (c[0] * w[0] + c[1] * w[1] + c[2] * w[2] + c[3] * w[3] + c[4] * w[4] + c[5] * w[5])


@dataclass(frozen=True)
class Counts:
    time: float
    zone: Vec
    queue: Vec
    arrivals: Vec
    departures: tuple[Vec, Vec, Vec]

    @staticmethod
    def of(m: Json) -> "Counts":
        d = m["departures"]
        return Counts(float(m["time"]), vec(m["zone_counts"]), vec(m["queue_counts"]), vec(m["arrivals"]),
                      (vec(d["left"]), vec(d["straight"]), vec(d["right"])))

    def same_as(self, o: "Counts") -> bool:
        return (self.zone, self.queue, self.arrivals, self.departures) == (
            o.zone, o.queue, o.arrivals, o.departures)

    def departed(self) -> int:
        return sum(sum(d) for d in self.departures)

    def standing(self) -> bool:
        z = sum(self.zone)
        return z > 0 and sum(self.queue) == z and sum(self.arrivals) == 0 and self.departed() == 0


def follows(prev: Counts, now: Counts, interval: float) -> bool:
    return abs(now.time - prev.time - interval) < 1e-6


# ---------------------------------------------------------------- E1


def quality(q: Json | None, p: Params) -> float | None:
    flags: list[bool] = []
    q = q or {}
    h = q.get("image_health")
    if h is not None:
        flags += [bool(h["frozen"]),
                  h["brightness"] <= p.brightness_edge or h["brightness"] >= 1.0 - p.brightness_edge,
                  h["blur"] > p.blur_max, h["displacement_px"] > p.displacement_max_px]
    c = q.get("detector_confidence")
    if c is not None:
        flags.append(c["mean"] < p.confidence_min)
    b = q.get("track_breaks_per_min")
    if b is not None:
        flags.append(b > p.track_breaks_max)
    if not flags:
        return None
    return p.quality_weight if any(flags) else 0.0


# ---------------------------------------------------------------- signals


@dataclass(frozen=True)
class Signal:
    kind: str  # "green", "clearing" or "red"
    elapsed: float = 0.0


def signal(phases: list[list[int]], approach: int, command: Json | None, t: float) -> Signal | None:
    if command is None:
        return None
    i = command["phase_index"]
    lit = i < len(phases) and approach in phases[i]
    if lit and command["stage"] == "green":
        return Signal("green", t - command["phase_started_at"])
    return Signal("clearing") if lit else Signal("red")


# ---------------------------------------------------------------- E3


@dataclass(frozen=True)
class LinkInput:
    sent: float
    arrived: float
    queue: float
    zone_m: float
    interval: float


def transit(sent: deque[tuple[float, float]], t: float, within: float) -> float:
    total = 0.0
    for s, v in sent:
        if t - s < within:
            total += v
    return total


@dataclass
class LinkBalance:
    length_m: float
    lanes: float
    last_t: float | None = None
    balance: float = 0.0
    allowance: float = 0.0
    settled_at: float | None = None
    sent: deque[tuple[float, float]] = field(default_factory=lambda: deque[tuple[float, float]]())
    window: deque[tuple[float, float, float]] = field(
        default_factory=lambda: deque[tuple[float, float, float]]())

    def reset(self) -> None:
        self.last_t = None
        self.settled_at = None
        self.sent.clear()
        self.window.clear()

    def update(self, t: float, x: LinkInput | None, p: Params) -> tuple[float | None, float | None]:
        """(score, fill), mirroring LinkBalance::update."""
        if x is None:
            self.reset()
            return None, None
        if self.last_t is not None and abs(t - self.last_t - x.interval) >= 1e-6:
            self.reset()
        stretch = self.length_m - x.zone_m
        if stretch <= 0.0:
            self.reset()
            return None, None
        storage = stretch * self.lanes / p.jam_spacing_m
        if self.last_t is None:
            self.balance = 0.0
            self.allowance = storage
        self.last_t = t

        earliest = stretch / (p.free_flow_mps * p.fast_factor)
        latest = stretch / p.free_flow_mps + p.slack_s
        self.sent.append((t, x.sent))
        while self.sent and t - self.sent[0][0] >= latest:
            self.sent.popleft()
        self.balance += x.sent - x.arrived

        zone_storage = x.zone_m * self.lanes / p.jam_spacing_m
        if x.queue >= p.congested_fraction * zone_storage:
            self.allowance = storage
        else:
            self.allowance = max(self.allowance - self.lanes * p.saturation_flow * x.interval, 0.0)
        if self.allowance <= 0.0 and self.settled_at is None:
            self.settled_at = t

        lower = transit(self.sent, t, earliest)
        upper = max(min(transit(self.sent, t, latest) + self.allowance, storage), lower)
        if self.allowance > 0.0:
            correction = max(self.balance - storage, 0.0)
        elif self.balance < lower:
            correction = lower - self.balance
        elif self.balance > upper:
            correction = self.balance - upper
        else:
            correction = 0.0
        self.balance = min(max(self.balance, lower), upper)

        if self.settled_at is None or t - self.settled_at < latest:
            return None, None
        self.window.append((t, x.sent, correction))
        while self.window and t - self.window[0][0] >= p.link_window_s:
            self.window.popleft()
        sent = wrong = 0.0
        for _, s, c in self.window:
            sent += s
            wrong += c
        share = wrong / max(sent, p.min_volume_pcu)
        return clamp((share - p.link_tolerance) / p.link_tolerance), self.balance / storage


def clamp(x: float) -> float:
    return min(max(x, 0.0), 1.0)


# ---------------------------------------------------------------- E4


@dataclass(frozen=True)
class ResponseInput:
    signal: Signal
    interval: float
    queue_before: int | None
    departures: tuple[int, int, int]
    blocked: bool | None


@dataclass
class Response:
    stalled_s: float = 0.0
    history: deque[tuple[float, float, bool, tuple[int, int, int]]] = field(
        default_factory=lambda: deque[tuple[float, float, bool, tuple[int, int, int]]]())

    def update(self, t: float, x: ResponseInput, p: Params) -> tuple[float, float]:
        if x.signal.kind == "green":
            if sum(x.departures) > 0:
                self.stalled_s = 0.0
            elif (x.signal.elapsed > p.startup_s and x.queue_before is not None and x.queue_before > 0
                  and x.blocked is False):
                self.stalled_s += x.interval
        stalled = min(self.stalled_s / p.stall_s, 1.0) * p.stall_weight

        self.history.append((t, x.interval, x.signal.kind == "red", x.departures))
        while self.history and t - self.history[0][0] >= p.red_window_s:
            self.history.popleft()
        red_s = 0.0
        dep = [0, 0, 0]
        for _, dt, red, d in self.history:
            if red:
                red_s += dt
                for i in range(3):
                    dep[i] += d[i]
        score = 0.0
        for n, tol in zip(dep, p.red_tolerance_per_min, strict=True):
            per_min = n * 60.0 / max(red_s, p.red_floor_s)
            score = max(score, clamp((per_min - tol) / tol))
        return stalled, score


# ---------------------------------------------------------------- tracker


@dataclass
class Track:
    last: Counts | None = None
    frozen_s: float = 0.0
    supplied_at: float | None = None
    residuals: deque[int] = field(default_factory=lambda: deque[int]())
    response: Response = field(default_factory=Response)


def conservation(residuals: deque[int], p: Params) -> float | None:
    if len(residuals) < max(p.window // 2, p.block):
        return None
    r = list(residuals)
    blocks = [float(abs(sum(r[i:i + p.block]))) for i in range(len(r) - p.block + 1)]
    total = 0.0
    for b in blocks:
        total += b
    return clamp((total / len(blocks) - p.tolerance) / p.tolerance)


class Tracker:
    """EvidenceTracker. Measurements as logged (JSON dicts), indexed by approach."""

    def __init__(self, corridor: Json, weights: Json, params: Params | None = None):
        self.p = params or Params()
        self.w = tuple(float(weights[k]) for k in CLASSES)
        approaches: list[Json] = corridor["approaches"]
        index = {a["id"]: i for i, a in enumerate(approaches)}
        jindex = {j["id"]: i for i, j in enumerate(corridor["junctions"])}
        self.ids = [a["id"] for a in approaches]
        self.junction = [jindex[a["junction"]] for a in approaches]
        self.phases: list[list[list[int]]] = [[[index[a] for a in ph] for ph in j["phases"]]
                                              for j in corridor["junctions"]]
        # approach -> (movement index, approach that movement feeds)
        self.feeds = [[(k, index[a["movements"][mv]]) for k, mv in enumerate(MOVEMENTS)
                       if a["movements"].get(mv) in index] for a in approaches]
        self.feeders: list[list[tuple[int, int]]] = [[] for _ in approaches]
        for u, fs in enumerate(self.feeds):
            for k, d in fs:
                self.feeders[d].append((u, k))
        self.links = [LinkBalance(float(a["length_m"]), float(a["lanes"])) if self.feeders[i] else None
                      for i, a in enumerate(approaches)]
        self.tracks = [Track() for _ in approaches]

    def update(self, t: float, ms: list[Json | None], commands: list[Json | None]) -> list[Evidence]:
        p = self.p
        for u, m in enumerate(ms):
            if m is None:
                continue
            for k, d in self.feeds[u]:
                if sum(vec(m["departures"][MOVEMENTS[k]])) > 0:
                    self.tracks[d].supplied_at = t
        cross: list[tuple[float | None, float | None]] = []
        for d, link in enumerate(self.links):
            cross.append(link.update(t, self.link_input(ms, d), p) if link is not None else (None, None))
        return [self.check(t, a, ms[a], commands[self.junction[a]], cross) for a in range(len(ms))]

    def link_input(self, ms: list[Json | None], d: int) -> LinkInput | None:
        down = ms[d]
        if down is None:
            return None
        sent = 0.0
        for u, k in self.feeders[d]:
            m = ms[u]
            if m is None:
                return None
            sent += pcu(vec(m["departures"][MOVEMENTS[k]]), self.w)
        return LinkInput(sent, pcu(vec(down["arrivals"]), self.w), pcu(vec(down["queue_counts"]), self.w),
                         float(down["zone_length_m"]), float(down["interval_s"]))

    def blocked(self, a: int, cross: list[tuple[float | None, float | None]]) -> bool | None:
        fills = [cross[d][1] for _, d in self.feeds[a]]
        if any(f is not None and f >= self.p.congested_fraction for f in fills):
            return True
        if any(f is None for f in fills):
            return None
        return False

    def check(self, t: float, a: int, m: Json | None, command: Json | None,
              cross: list[tuple[float | None, float | None]]) -> Evidence:
        p, track = self.p, self.tracks[a]
        if m is None:
            track.residuals.clear()
            return NONE
        now = Counts.of(m)
        interval = float(m["interval_s"])
        last = track.last

        if last is not None and last.same_as(now):
            supplied = track.supplied_at is not None and t - track.supplied_at <= p.supply_s
            sig = signal(self.phases[self.junction[a]], a, command, t)
            if sig is not None and sig.kind == "green" and (sum(now.zone) > 0 or supplied):
                track.frozen_s += interval
        else:
            track.frozen_s = 0.0
        cap = p.standing_weight if now.standing() else 1.0
        frozen = min(track.frozen_s / p.frozen_s, cap)

        stalled = red = None
        sig = signal(self.phases[self.junction[a]], a, command, t)
        if sig is not None:
            before = sum(last.queue) if last is not None and follows(last, now, interval) else None
            x = ResponseInput(sig, interval, before, (sum(now.departures[0]), sum(now.departures[1]),
                                                      sum(now.departures[2])), self.blocked(a, cross))
            stalled, red = track.response.update(t, x, p)

        if last is not None and follows(last, now, interval):
            track.residuals.append(sum(now.zone) - sum(last.zone) - sum(now.arrivals) + now.departed())
            if len(track.residuals) > p.window:
                track.residuals.popleft()
        else:
            track.residuals.clear()
        track.last = now

        return (quality(m.get("quality"), p), frozen, conservation(track.residuals, p), cross[a][0],
                stalled, red)


# ---------------------------------------------------------------- logged runs


def load_corridor() -> Json:
    return load_json(load_json("config/sim.json")["corridor_file"])


def read_run(run: Path, ids: list[str], junctions: list[str]) -> tuple[
        list[float], dict[float, list[Json | None]], dict[float, list[Json | None]], dict[float, Json]]:
    """Times, measurements and commands per time (indexed like the corridor), faults per time."""
    ms: dict[float, list[Json | None]] = defaultdict(lambda: [None] * len(ids))
    pos = {a: i for i, a in enumerate(ids)}
    for line in (run / "measurements.jsonl").read_text().splitlines():
        m = json.loads(line)
        ms[float(m["time"])][pos[m["approach_id"]]] = m
    cs: dict[float, list[Json | None]] = defaultdict(lambda: [None] * len(junctions))
    jpos = {j: i for i, j in enumerate(junctions)}
    for line in (run / "commands.jsonl").read_text().splitlines():
        c = json.loads(line)
        cs[float(c["time"])][jpos[c["junction_id"]]] = c
    faults: dict[float, Json] = defaultdict(dict)
    if (run / "faults.jsonl").exists():
        for line in (run / "faults.jsonl").read_text().splitlines():
            f = json.loads(line)
            faults[float(f["time"])][f["approach_id"]] = f["fault"]
    times = sorted(cs)
    return times, ms, cs, faults


def replay(run: Path, params: Params | None = None) -> tuple[Tracker, dict[float, list[Evidence]],
                                                             dict[float, Json]]:
    corridor = load_corridor()
    tracker = Tracker(corridor, load_json("config/pcu.json")["weights"], params)
    times, ms, cs, faults = read_run(run, tracker.ids, [j["id"] for j in corridor["junctions"]])
    out: dict[float, list[Evidence]] = {}
    for t in times:
        # the core decides at t on the commands it issued at t - 1
        out[t] = tracker.update(t, ms[t], cs.get(t - 1, [None] * len(corridor["junctions"])))
    return tracker, out, faults


def summarize(runs: list[Path], params: Params | None = None) -> None:
    for run in runs:
        tracker, out, faults = replay(run, params)
        print(run.name)
        # last time a fault was active on each approach or one feeding it, for "after" labels
        recent: dict[tuple[float, int], str] = {}
        last: dict[int, tuple[float, str]] = {}
        for t in sorted(out):
            for a in range(len(tracker.ids)):
                for u in [a] + [u for u, _ in tracker.feeders[a]]:
                    kind = faults.get(t, {}).get(tracker.ids[u])
                    if kind:
                        last[a] = (t, kind)
                if a in last and t - last[a][0] <= AFTER_S:
                    recent[(t, a)] = last[a][1]
        for f, name in enumerate(FIELDS):
            # (approach, label) -> [seconds >= 1, seconds >= 0.5, seconds scored]
            tally: dict[tuple[str, str], list[int]] = defaultdict(lambda: [0, 0, 0])
            for t, ev in out.items():
                for a, e in enumerate(ev):
                    own = faults.get(t, {}).get(tracker.ids[a])
                    fed: set[str] = {k for u, _ in tracker.feeders[a]
                                     if (k := faults.get(t, {}).get(tracker.ids[u]))}
                    label = own or (f"feeder {'/'.join(sorted(fed))}" if fed else
                                    f"after {recent[(t, a)]}" if (t, a) in recent else "clean")
                    v = e[f]
                    cell = tally[(tracker.ids[a], label)]
                    cell[2] += v is not None
                    cell[1] += v is not None and v >= 0.5
                    cell[0] += v is not None and v >= 1.0
            rows = [f"{a}:{lab} {c[0]}/{c[1]}/{c[2]}" for (a, lab), c in sorted(tally.items())
                    if c[1] or lab != "clean"]
            print(f"  {name} (>=1 / >=0.5 / scored): " + ("; ".join(rows) or "nothing"))


# ---------------------------------------------------------------- fixture


def encode(m: Json, a: int) -> list[Any]:
    """One measurement as a fixture row; an all-zero count vector is written as 0
    and a quality report with nothing in it as null."""
    def counts(c: Json) -> list[int] | int:
        v = list(vec(c))
        return v if any(v) else 0
    d = m["departures"]
    q: Json = m.get("quality") or {}
    return [a, counts(m["zone_counts"]), counts(m["queue_counts"]), counts(m["arrivals"]),
            counts(d["left"]), counts(d["straight"]), counts(d["right"]),
            m["zone_length_m"], m["interval_s"], q if any(v is not None for v in q.values()) else None]


def decode(row: list[Any], t: float) -> Json:
    def counts(v: list[int] | int) -> Json:
        return dict(zip(CLASSES, v if isinstance(v, list) else [0] * 6, strict=True))
    return {"time": t, "interval_s": row[8], "zone_length_m": row[7], "zone_counts": counts(row[1]),
            "queue_counts": counts(row[2]), "arrivals": counts(row[3]),
            "departures": {"left": counts(row[4]), "straight": counts(row[5]), "right": counts(row[6])},
            "quality": row[9]}


def check_fixture(fixture: Json) -> list[str]:
    """Replays the fixture inputs; returns every score that differs from the stored one."""
    corridor = load_corridor()
    tracker = Tracker(corridor, fixture["pcu"], Params.from_json(fixture["params"]))
    n = len(corridor["approaches"])
    bad: list[str] = []
    for step, expected in zip(fixture["steps"], fixture["expected"], strict=True):
        t = float(step["t"])
        ms: list[Json | None] = [None] * n
        for row in step["m"]:
            ms[row[0]] = decode(row, t)
        commands: list[Json | None] = [None if c is None else
                                       {"phase_index": c[0], "stage": c[1], "phase_started_at": c[2]}
                                       for c in step["c"]]
        for a, (got, want) in enumerate(zip(tracker.update(t, ms, commands), expected, strict=True)):
            for name, g, w in zip(FIELDS, got, want, strict=True):
                if (g is None) != (w is None) or (g is not None and abs(g - w) > 1e-9):
                    bad.append(f"t={t} {tracker.ids[a]} {name}: got {g}, fixture {w}")
    return bad


def build_fixture(run: Path, seconds: int, params: Params, edit: Callable[[float, str, Json], None]) -> Json:
    """Fixture from the first `seconds` of a logged run. `edit(t, approach, measurement)` may change
    a measurement first, to add what the simulator never produces (quality reports, red runners)."""
    corridor = load_corridor()
    weights = load_json("config/pcu.json")["weights"]
    tracker = Tracker(corridor, weights, params)
    jids = [j["id"] for j in corridor["junctions"]]
    times, ms, cs, _ = read_run(run, tracker.ids, jids)
    steps: list[Json] = []
    expected: list[list[tuple[float | None, ...]]] = []
    for t in times[:seconds]:
        row = ms[t]
        for a, m in enumerate(row):
            if m is not None:
                edit(t, tracker.ids[a], m)
        commands = cs.get(t - 1, [None] * len(jids))
        steps.append({"t": t, "m": [encode(m, a) for a, m in enumerate(row) if m is not None],
                      "c": [None if c is None else [c["phase_index"], c["stage"], c["phase_started_at"]]
                            for c in commands]})
        scores = tracker.update(t, [None if m is None else decode(encode(m, 0), t) for m in row],
                                       [None if c is None else {"phase_index": c["phase_index"],
                                                                "stage": c["stage"],
                                                                "phase_started_at": c["phase_started_at"]}
                                        for c in commands])
        expected.append([tuple(None if v is None else round(v, 10) for v in e) for e in scores])
    return {"_note": f"Inputs and expected evidence scores from {run.name}, first {seconds} s. "
                     "Shared by crates/core/src/guard/evidence.rs and tests/test_evidence_replay.py.",
            "pcu": weights, "default_params": Params().to_json(), "params": params.to_json(),
            "steps": steps, "expected": expected}


FIXTURE = ROOT / "tests" / "fixtures" / "evidence_replay.json"
# shorter windows than the defaults, so E3 and E4 are scored within the 240 s
FIXTURE_PARAMS = Params(window=30, frozen_s=10.0, slack_s=20.0, saturation_flow=2.0, link_window_s=60.0,
                        stall_s=10.0, red_window_s=60.0, red_floor_s=30.0)
_HEALTHY: Json = {"detector_confidence": {"mean": 0.8, "std": 0.1}, "track_breaks_per_min": 2.0,
                  "image_health": {"frozen": False, "blur": 0.1, "brightness": 0.5, "displacement_px": 1.0}}
_BLURRY: Json = {**_HEALTHY, "image_health": {**_HEALTHY["image_health"], "blur": 0.9}}


def _fixture_edit(t: float, approach: str, m: Json) -> None:
    if approach == "N1_J1":
        m["quality"] = _BLURRY if t >= 100 else _HEALTHY
    elif approach == "S1_J1":
        m["quality"] = {"detector_confidence": {"mean": 0.2, "std": 0.05}}
    elif approach == "S3_J3" and t >= 60 and int(t) % 7 == 0:
        m["departures"]["straight"]["car"] += 1  # a red runner whenever S3_J3 is red


def make_fixture(results_dir: Path) -> None:
    """Rewrites FIXTURE from a 240 s maxpressure seed-1 run of config/faults/evidence_fixture.json."""
    from flowguard_sim.run import run  # SUMO only for this

    summary = run("maxpressure", 240, 1, results_dir=results_dir, faults="evidence_fixture")
    fixture = build_fixture(results_dir / summary["run_id"], 240, FIXTURE_PARAMS, _fixture_edit)
    fixture["_note"] += (" Added: quality reports on N1_J1 and S1_J1 (the simulator reports none) and one"
                         " extra straight departure on S3_J3 every 7 s from 60 s, red or not.")
    FIXTURE.parent.mkdir(exist_ok=True)
    with open(FIXTURE, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(fixture, separators=(",", ":")) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="*", type=Path, help="run directories to summarise")
    parser.add_argument("--fixture", action="store_true", help=f"rewrite {FIXTURE.relative_to(ROOT)}")
    args = parser.parse_args()
    if args.fixture:
        make_fixture(ROOT / "results")
    summarize(args.runs)
