"""Camera fault injection between the observer and the core (docs/specs/guard.md, G0).

A scenario is a file `config/faults/<name>.json`:

    {"seed": 7, "faults": [
      {"approach": "J1_J2", "kind": "freeze",     "start_s": 900,  "end_s": 1500},
      {"approach": "W_J1",  "kind": "undercount", "start_s": 600,  "end_s": 2400, "keep": 0.4}]}

A fault is active for start_s <= t < end_s. Faults on the same approach may
not overlap, so every approach-second has at most one active kind (the
ground truth logged to faults.jsonl). Each fault draws from its own random
stream seeded from the scenario seed and its position in the file, so a
scenario always corrupts the same run the same way.

The injector only sees Measurements: the camera views the core would get,
plus, for displaced faults, the observer's view of the shifted zone (it never
reads SUMO itself). It never modifies them in place: corrupted measurements
are new objects.
"""

import random
from pathlib import Path
from typing import Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

from flowguard_contracts import MOVEMENTS, VEHICLE_CLASSES, ClassCounts, Corridor, Measurement, MovementCounts
from flowguard_sim.paths import ROOT, load_json, without_notes

FaultKind = Literal["freeze", "blackout", "undercount", "dropout", "displaced"]
FAULTS_DIR = ROOT / "config" / "faults"
# the one parameter each kind needs; kinds not listed take none
PARAMS: dict[str, str] = {"undercount": "keep", "dropout": "p", "displaced": "shift_m"}


class Fault(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    approach: str
    kind: FaultKind
    start_s: float = Field(ge=0)
    end_s: float
    keep: float | None = Field(default=None, ge=0, le=1)
    """undercount: probability each vehicle is still counted."""
    p: float | None = Field(default=None, ge=0, le=1)
    """dropout: probability the measurement is missing in a given second."""
    shift_m: float | None = Field(default=None, gt=0)
    """displaced: how far the camera zone moved back from the stop line."""

    @model_validator(mode="after")
    def _check(self) -> Self:
        if self.end_s <= self.start_s:
            raise ValueError(f"{self.approach} {self.kind}: end_s must be after start_s")
        for name in ("keep", "p", "shift_m"):
            needed = PARAMS.get(self.kind) == name
            if needed != (getattr(self, name) is not None):
                raise ValueError(f"{self.kind} {'needs' if needed else 'does not take'} {name!r}")
        return self

    def active(self, t: float) -> bool:
        return self.start_s <= t < self.end_s


class Scenario(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    seed: int
    faults: list[Fault]

    def check(self, corridor: Corridor, camera_zone_m: float | None) -> Self:
        """Every approach exists, no approach has two faults at once, and a
        displaced zone still fits on its approach (camera_zone_m as in
        SimObserver: None = whole edge)."""
        length = {a.id: a.length_m for a in corridor.approaches}
        for f in self.faults:
            if f.approach not in length:
                raise ValueError(f"unknown approach {f.approach!r}")
        for i, f in enumerate(self.faults):
            for g in self.faults[i + 1:]:
                if f.approach == g.approach and f.start_s < g.end_s and g.start_s < f.end_s:
                    raise ValueError(f"overlapping faults on {f.approach}: {f.kind} and {g.kind}")
        for f in self.faults:
            if f.shift_m is None:
                continue
            zone = min(camera_zone_m, length[f.approach]) if camera_zone_m else length[f.approach]
            if f.shift_m + zone > length[f.approach]:
                raise ValueError(f"displaced {f.approach}: shift_m {f.shift_m:g} + camera zone {zone:g} m "
                                 f"is longer than the approach ({length[f.approach]:g} m)")
        shifts = [(f.approach, f.shift_m) for f in self.faults if f.shift_m is not None]
        if len(dict(shifts)) < len(set(shifts)):
            raise ValueError("an approach can only be displaced by one shift_m per scenario")
        return self


def load_scenario(name: str, corridor: Corridor, camera_zone_m: float | None,
                  folder: Path | None = None) -> Scenario:
    path = (folder or FAULTS_DIR) / f"{name}.json"
    return Scenario.model_validate(without_notes(load_json(path))).check(corridor, camera_zone_m)


def _zero() -> ClassCounts:
    return ClassCounts.model_validate(dict.fromkeys(VEHICLE_CLASSES, 0))


def _thin(counts: ClassCounts, keep: float, rng: random.Random) -> ClassCounts:
    return ClassCounts.model_validate(
        {c: rng.binomialvariate(getattr(counts, c), keep) for c in VEHICLE_CLASSES})


class FaultInjector:
    """Sits between the observer and the core. Without a scenario it returns
    its input unchanged."""

    def __init__(self, scenario: Scenario | None = None):
        self.scenario = scenario or Scenario(seed=0, faults=[])
        self._rng = [random.Random(f"{self.scenario.seed}:{i}") for i in range(len(self.scenario.faults))]
        self._last_seen: dict[str, Measurement] = {}  # approach -> previous uncorrupted measurement
        self._frozen: dict[int, Measurement] = {}  # fault index -> measurement it keeps repeating

    def active(self, t: float, approach: str) -> int | None:
        """Index of the fault active on `approach` at `t`, if any."""
        for i, f in enumerate(self.scenario.faults):
            if f.approach == approach and f.active(t):
                return i
        return None

    def active_kind(self, t: float, approach: str) -> FaultKind | None:
        i = self.active(t, approach)
        return None if i is None else self.scenario.faults[i].kind

    def shifts(self) -> dict[str, float]:
        """Approach -> shift_m for every displaced fault: the shifted views
        the observer has to produce (SimObserver(shifted=...))."""
        return {f.approach: f.shift_m for f in self.scenario.faults if f.shift_m is not None}

    def apply(self, t: float, measurements: list[Measurement],
              shifted: dict[str, Measurement] | None = None) -> list[Measurement]:
        """shifted: the observer's displaced-camera views, needed while a
        displaced fault is active."""
        if not self.scenario.faults:
            return measurements
        out: list[Measurement] = []
        for m in measurements:
            i = self.active(t, m.approach_id)
            seen = m if i is None else self._corrupt(i, m, shifted or {})
            self._last_seen[m.approach_id] = m
            if seen is not None:
                out.append(seen)
        return out

    def _corrupt(self, i: int, m: Measurement, shifted: dict[str, Measurement]) -> Measurement | None:
        f, rng = self.scenario.faults[i], self._rng[i]
        if f.kind == "freeze":
            # the last pre-fault measurement; if the run starts frozen, the first one
            frozen = self._frozen.setdefault(i, self._last_seen.get(m.approach_id, m))
            return frozen.model_copy(update={"time": m.time}, deep=True)
        if f.kind == "blackout":
            return m.model_copy(deep=True, update={
                "zone_counts": _zero(), "queue_counts": _zero(), "arrivals": _zero(),
                "departures": MovementCounts.model_validate({mv: _zero() for mv in MOVEMENTS})})
        if f.kind == "undercount":
            assert f.keep is not None
            keep = f.keep
            return m.model_copy(deep=True, update={
                "zone_counts": _thin(m.zone_counts, keep, rng),
                "queue_counts": _thin(m.queue_counts, keep, rng),
                "arrivals": _thin(m.arrivals, keep, rng),
                "departures": MovementCounts.model_validate(
                    {mv: _thin(getattr(m.departures, mv), keep, rng) for mv in MOVEMENTS})})
        if f.kind == "dropout":
            assert f.p is not None
            return None if rng.random() < f.p else m
        if f.kind == "displaced":
            view = shifted.get(m.approach_id)
            if view is None or view.time != m.time:
                raise ValueError(f"no shifted view of {m.approach_id} at t={m.time:g}: "
                                 f"create the observer with shifted=injector.shifts()")
            return view.model_copy(deep=True)
        raise AssertionError(f"unhandled fault kind {f.kind}")
