"""Fault injector (docs/specs/guard.md, G0): each kind, scenario validation,
determinism, inputs never modified, and runs without a scenario unchanged."""

import json
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from flowguard_contracts import MOVEMENTS, VEHICLE_CLASSES, Corridor, Measurement
from flowguard_sim import faults as faults_module
from flowguard_sim.faults import FAULTS_DIR, FaultInjector, Scenario, load_scenario
from flowguard_sim.observer import SimObserver
from flowguard_sim.run import run

APPROACHES = ("W_J1", "J1_J2")


def counts(base: int) -> dict[str, int]:
    return {c: base + i for i, c in enumerate(VEHICLE_CLASSES)}


def measurement(t: float, approach: str = "W_J1") -> Measurement:
    """Every count non-zero and different each second, so changes are visible."""
    k = int(t) % 7 + 1
    return Measurement.model_validate({
        "time": t, "interval_s": 1.0, "junction_id": "J1" if approach == "W_J1" else "J2",
        "approach_id": approach, "source": "sim", "zone_length_m": 60.0,
        "zone_counts": counts(10 + k), "queue_counts": counts(5 + k), "arrivals": counts(k),
        "departures": {mv: counts(k + j) for j, mv in enumerate(MOVEMENTS)},
    })


def shifted(t: float, approach: str = "W_J1") -> Measurement:
    """Stand-in for the observer's displaced-camera view: other counts, same second."""
    return measurement(t + 3, approach).model_copy(update={"time": t})


def feed(injector: FaultInjector, seconds: int) -> list[list[Measurement]]:
    return [injector.apply(t, [measurement(t, a) for a in APPROACHES], {a: shifted(t, a) for a in APPROACHES})
            for t in range(seconds)]


def injector(*faults: dict[str, Any], seed: int = 7) -> FaultInjector:
    return FaultInjector(Scenario.model_validate({"seed": seed, "faults": list(faults)}))


def by_approach(out: list[Measurement], approach: str) -> Measurement | None:
    return next((m for m in out if m.approach_id == approach), None)


def vectors(m: Measurement) -> list[dict[str, int]]:
    d = m.model_dump()
    return [d["zone_counts"], d["queue_counts"], d["arrivals"], *(d["departures"][mv] for mv in MOVEMENTS)]


def test_no_scenario_returns_input_unchanged() -> None:
    batch = [measurement(3, a) for a in APPROACHES]
    assert FaultInjector().apply(3, batch) is batch


def test_untouched_approaches_pass_through() -> None:
    inj = injector({"approach": "J1_J2", "kind": "blackout", "start_s": 0, "end_s": 10})
    batch = [measurement(1, a) for a in APPROACHES]
    assert inj.apply(1, batch)[0] is batch[0]


def test_freeze_repeats_last_pre_fault_measurement() -> None:
    out = feed(injector({"approach": "W_J1", "kind": "freeze", "start_s": 5, "end_s": 9}), 12)
    before = by_approach(out[4], "W_J1")
    assert before is not None and before == measurement(4)
    for t in range(5, 9):
        frozen = by_approach(out[t], "W_J1")
        assert frozen is not None
        assert frozen.time == t  # time keeps advancing
        assert frozen.model_dump(exclude={"time"}) == before.model_dump(exclude={"time"})
        assert frozen.quality is not None and frozen.quality.image_health is None  # sim camera: no flag
    assert by_approach(out[9], "W_J1") == measurement(9)  # live again after end_s
    assert by_approach(out[6], "J1_J2") == measurement(6, "J1_J2")


def test_freeze_from_the_first_second_repeats_the_first_measurement() -> None:
    out = feed(injector({"approach": "W_J1", "kind": "freeze", "start_s": 0, "end_s": 5}), 5)
    first = measurement(0).model_dump(exclude={"time"})
    frozen = [by_approach(o, "W_J1") for o in out]
    assert all(m is not None and m.model_dump(exclude={"time"}) == first for m in frozen)


def test_blackout_zeroes_every_count() -> None:
    out = feed(injector({"approach": "W_J1", "kind": "blackout", "start_s": 2, "end_s": 4}), 6)
    for t in (2, 3):
        m = by_approach(out[t], "W_J1")
        assert m is not None
        assert all(v == 0 for vec in vectors(m) for v in vec.values())
        clean = measurement(t)
        assert (m.time, m.zone_length_m, m.source, m.quality) == (
            clean.time, clean.zone_length_m, clean.source, clean.quality)
    assert by_approach(out[1], "W_J1") == measurement(1)
    assert by_approach(out[4], "W_J1") == measurement(4)


def test_undercount_thins_every_count_binomially() -> None:
    out = feed(injector({"approach": "W_J1", "kind": "undercount", "start_s": 0, "end_s": 2000, "keep": 0.4}),
               2000)
    kept = total = 0
    for t, o in enumerate(out):
        m = by_approach(o, "W_J1")
        assert m is not None
        for got, had in zip(vectors(m), vectors(measurement(t)), strict=True):
            assert all(0 <= got[c] <= had[c] for c in VEHICLE_CLASSES)
            kept += sum(got.values())
            total += sum(had.values())
    assert 0.38 < kept / total < 0.42


@pytest.mark.parametrize(("keep", "expect"), [(1.0, "same"), (0.0, "zero")])
def test_undercount_extremes(keep: float, expect: str) -> None:
    fault = {"approach": "W_J1", "kind": "undercount", "start_s": 0, "end_s": 5, "keep": keep}
    out = feed(injector(fault), 5)
    for t, o in enumerate(out):
        m = by_approach(o, "W_J1")
        assert m is not None
        if expect == "same":
            assert m == measurement(t)
        else:
            assert all(v == 0 for vec in vectors(m) for v in vec.values())


def test_dropout_removes_measurements_at_rate_p() -> None:
    fault = {"approach": "W_J1", "kind": "dropout", "start_s": 100, "end_s": 2100, "p": 0.2}
    out = feed(injector(fault), 2200)
    missing = [t for t, o in enumerate(out) if by_approach(o, "W_J1") is None]
    assert all(100 <= t < 2100 for t in missing)
    assert 0.17 < len(missing) / 2000 < 0.23
    kept = [by_approach(o, "W_J1") for t, o in enumerate(out) if t not in missing]
    assert all(m == measurement(int(m.time)) for m in kept if m is not None)  # survivors unchanged
    assert all(by_approach(o, "J1_J2") is not None for o in out)


@pytest.mark.parametrize("p", [0.0, 1.0])
def test_dropout_extremes(p: float) -> None:
    out = feed(injector({"approach": "W_J1", "kind": "dropout", "start_s": 0, "end_s": 20, "p": p}), 20)
    present = sum(by_approach(o, "W_J1") is not None for o in out)
    assert present == (20 if p == 0 else 0)


ALL_KINDS: list[dict[str, Any]] = [
    {"approach": "W_J1", "kind": "freeze", "start_s": 3, "end_s": 40},
    {"approach": "W_J1", "kind": "blackout", "start_s": 40, "end_s": 60},
    {"approach": "J1_J2", "kind": "undercount", "start_s": 0, "end_s": 60, "keep": 0.5},
    {"approach": "W_J1", "kind": "dropout", "start_s": 60, "end_s": 100, "p": 0.5},
    {"approach": "J1_J2", "kind": "displaced", "start_s": 60, "end_s": 100, "shift_m": 40},
]


def test_inputs_never_modified() -> None:
    inj = injector(*ALL_KINDS)
    for t in range(100):
        batch = [measurement(t, a) for a in APPROACHES]
        views = {a: shifted(t, a) for a in APPROACHES}
        snapshot = [m.model_dump() for m in [*batch, *views.values()]]
        out = inj.apply(t, batch, views)
        assert [m.model_dump() for m in [*batch, *views.values()]] == snapshot, t
        for m in out:  # corrupted outputs never share nested objects with the inputs
            src = next(b for b in batch if b.approach_id == m.approach_id)
            assert m is src or (m.zone_counts is not src.zone_counts and m.departures is not src.departures)
            view = views[m.approach_id]
            assert m.zone_counts is not view.zone_counts and m.departures is not view.departures


def dumps(out: list[list[Measurement]]) -> list[list[str]]:
    return [[m.model_dump_json() for m in o] for o in out]


def test_deterministic_per_seed() -> None:
    a, b = dumps(feed(injector(*ALL_KINDS), 100)), dumps(feed(injector(*ALL_KINDS), 100))
    assert a == b
    assert dumps(feed(injector(*ALL_KINDS, seed=8), 100)) != a


def test_each_fault_has_its_own_random_stream() -> None:
    """Adding a fault elsewhere in the file does not change another fault's draws."""
    undercount = {"approach": "J1_J2", "kind": "undercount", "start_s": 0, "end_s": 50, "keep": 0.5}
    alone = feed(injector({"approach": "W_J1", "kind": "blackout", "start_s": 0, "end_s": 1}, undercount), 50)
    extra = {"approach": "W_J1", "kind": "dropout", "start_s": 0, "end_s": 50, "p": 0.5}
    with_more = feed(injector({"approach": "W_J1", "kind": "blackout", "start_s": 0, "end_s": 1}, undercount,
                              extra), 50)
    assert [by_approach(o, "J1_J2") for o in alone] == [by_approach(o, "J1_J2") for o in with_more]


def test_active_kind() -> None:
    inj = injector(*ALL_KINDS)
    assert inj.active_kind(2, "W_J1") is None
    assert inj.active_kind(3, "W_J1") == "freeze"
    assert inj.active_kind(40, "W_J1") == "blackout"  # end_s is exclusive
    assert inj.active_kind(99, "W_J1") == "dropout"
    assert inj.active_kind(100, "W_J1") is None
    assert inj.active_kind(10, "J1_J2") == "undercount"
    assert inj.active_kind(60, "J1_J2") == "displaced"


def test_displaced_substitutes_the_shifted_view() -> None:
    out = feed(injector(displaced_on("W_J1", 40, 5, 9)), 12)
    for t in range(12):
        m = by_approach(out[t], "W_J1")
        assert m == (shifted(t) if 5 <= t < 9 else measurement(t)), t
        assert by_approach(out[t], "J1_J2") == measurement(t, "J1_J2")
    assert shifted(6) != measurement(6)  # so the check above can tell them apart


def test_displaced_needs_the_shifted_view_of_that_second() -> None:
    inj = injector({"approach": "W_J1", "kind": "displaced", "start_s": 0, "end_s": 9, "shift_m": 40})
    assert inj.shifts() == {"W_J1": 40}
    with pytest.raises(ValueError, match="shifted view"):
        inj.apply(1, [measurement(1)])
    with pytest.raises(ValueError, match="shifted view"):
        inj.apply(2, [measurement(2)], {"W_J1": shifted(1)})  # stale


@pytest.mark.parametrize("fault", [
    {"approach": "W_J1", "kind": "undercount", "start_s": 0, "end_s": 10},                 # keep missing
    {"approach": "W_J1", "kind": "freeze", "start_s": 0, "end_s": 10, "p": 0.1},           # stray parameter
    {"approach": "W_J1", "kind": "dropout", "start_s": 0, "end_s": 10, "p": 1.5},          # p out of range
    {"approach": "W_J1", "kind": "blackout", "start_s": 10, "end_s": 10},                  # empty window
    {"approach": "W_J1", "kind": "smoke", "start_s": 0, "end_s": 10},                      # unknown kind
    {"approach": "W_J1", "kind": "blackout", "start_s": 0, "end_s": 10, "extra": 1},       # unknown field
    {"approach": "W_J1", "kind": "displaced", "start_s": 0, "end_s": 10},                  # shift_m missing
    {"approach": "W_J1", "kind": "displaced", "start_s": 0, "end_s": 10, "shift_m": 0},    # no shift
])
def test_rejects_invalid_faults(fault: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        Scenario.model_validate({"seed": 1, "faults": [fault]})


def test_rejects_unknown_approach_and_overlap(corridor: Corridor) -> None:
    unknown = Scenario.model_validate(
        {"seed": 1, "faults": [{"approach": "NOPE", "kind": "blackout", "start_s": 0, "end_s": 5}]})
    with pytest.raises(ValueError, match="unknown approach"):
        unknown.check(corridor, 60)
    overlap = Scenario.model_validate({"seed": 1, "faults": [
        {"approach": "W_J1", "kind": "blackout", "start_s": 0, "end_s": 5},
        {"approach": "W_J1", "kind": "freeze", "start_s": 4, "end_s": 8}]})
    with pytest.raises(ValueError, match="overlapping"):
        overlap.check(corridor, 60)
    back_to_back = Scenario.model_validate({"seed": 1, "faults": [
        {"approach": "W_J1", "kind": "blackout", "start_s": 0, "end_s": 5},
        {"approach": "W_J1", "kind": "freeze", "start_s": 5, "end_s": 8}]})
    back_to_back.check(corridor, 60)


def displaced_on(approach: str, shift_m: float, start_s: float = 0, end_s: float = 10) -> dict[str, Any]:
    return {"approach": approach, "kind": "displaced", "start_s": start_s, "end_s": end_s, "shift_m": shift_m}


def test_displaced_shift_must_fit_on_the_approach(corridor: Corridor) -> None:
    """Side road N2_J2 is 189.6 m: a 60 m zone can move back at most 129.6 m."""
    def check(*faults: dict[str, Any], zone: float | None = 60) -> None:
        Scenario.model_validate({"seed": 1, "faults": list(faults)}).check(corridor, zone)

    check(displaced_on("N2_J2", 129.6))
    with pytest.raises(ValueError, match="longer than the approach"):
        check(displaced_on("N2_J2", 129.7))
    with pytest.raises(ValueError, match="longer than the approach"):
        check(displaced_on("J2_J3", 1), zone=None)  # camera already sees the whole edge
    with pytest.raises(ValueError, match="one shift_m"):
        check(displaced_on("J2_J3", 40, 0, 10), displaced_on("J2_J3", 50, 20, 30))
    check(displaced_on("J2_J3", 40, 0, 10), displaced_on("J2_J3", 40, 20, 30))


def test_observer_rejects_a_shift_that_does_not_fit(corridor: Corridor) -> None:
    SimObserver(corridor, 60, shifted={"N2_J2": 129.6})
    for shift, zone in [({"N2_J2": 129.7}, 60), ({"N2_J2": 0}, 60), ({"J2_J3": 1}, None), ({"NOPE": 10}, 60)]:
        with pytest.raises(ValueError, match="cannot shift"):
            SimObserver(corridor, zone, shifted=shift)


def test_observer_without_shift_has_no_shifted_view(corridor: Corridor) -> None:
    assert SimObserver(corridor, 60).shift == {}
    assert SimObserver(corridor, 60).shifted_view == {}


def test_scenario_suite_loads(corridor: Corridor) -> None:
    names = sorted(p.stem for p in FAULTS_DIR.glob("*.json"))
    assert len(names) >= 5
    kinds = set[str]()
    for name in names:
        scenario = load_scenario(name, corridor, 60)
        FaultInjector(scenario)
        kinds |= {f.kind for f in scenario.faults}
    assert kinds == {"freeze", "blackout", "undercount", "dropout", "displaced"}
    for kind in kinds:  # each kind also has a scenario of its own
        assert {f.kind for f in load_scenario(kind, corridor, 60).faults} == {kind}


# --- closed loop through SUMO ---

def records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def keyed(path: Path) -> dict[tuple[float, str], dict[str, Any]]:
    return {(m["time"], m["approach_id"]): m for m in records(path)}


def test_run_with_faults_logs_ground_truth(tmp_path: Path, corridor: Corridor) -> None:
    """smoke.json under the fixed controller. Fixed time ignores measurements,
    so traffic is the same as a clean run and the clean run's camera view is
    what each corrupted record should be compared with."""
    summary = run("fixed", 300, 1, results_dir=tmp_path, faults="smoke")
    out = tmp_path / summary["run_id"]
    clean_run = run("fixed", 300, 1, results_dir=tmp_path / "clean")
    clean = keyed(tmp_path / "clean" / clean_run["run_id"] / "measurements.jsonl")
    assert summary["faults"] == "smoke"
    scenario = load_scenario("smoke", corridor, 60)

    truth_log = records(out / "faults.jsonl")
    assert len(truth_log) == 300 * 12
    for r in truth_log:
        active = [f.kind for f in scenario.faults if f.approach == r["approach_id"] and f.active(r["time"])]
        assert r["fault"] == (active[0] if active else None)

    seen = keyed(out / "measurements.jsonl")
    states = records(out / "states.jsonl")
    assert len(states) == len(seen) < 300 * 12  # dropout removed some
    dropped = 300 - sum(a == "E_J3" for _, a in seen)
    assert 30 < dropped < 90  # p = 0.2 over 300 s
    no_fault = [(r["time"], r["approach_id"]) for r in truth_log if r["fault"] is None]
    assert all(seen[k] == clean[k] for k in no_fault)

    blackout = [seen[(float(t), "N2_J2")] for t in range(60, 120)]
    assert all(sum(m["zone_counts"].values()) == 0 for m in blackout)

    frozen = [seen[(float(t), "J1_J2")] for t in range(59, 180)]
    assert all({**m, "time": 0} == {**frozen[0], "time": 0} for m in frozen)

    # displaced J2_J3, [60, 240), 40 m back: different zone, same stop-line departures
    window = [(float(t), "J2_J3") for t in range(60, 240)]
    assert all(seen[k]["departures"] == clean[k]["departures"] for k in window)
    assert all(seen[k]["zone_length_m"] == clean[k]["zone_length_m"] == 60 for k in window)
    differ = sum(seen[k]["zone_counts"] != clean[k]["zone_counts"] for k in window)
    assert differ > 30, differ
    truth = keyed(out / "truth.jsonl")  # the shifted zone lies on the edge, so never sees more than all of it
    assert all(seen[k]["zone_counts"][c] <= n for k in window for c, n in truth[k]["zone_counts"].items())
    outside = [(float(t), "J2_J3") for t in [*range(60), *range(240, 300)]]
    assert all(seen[k] == clean[k] for k in outside)


def test_no_scenario_runs_are_unchanged(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A scenario whose faults never start drives exactly the same run as no
    scenario, even with the observer computing a shifted view, and a run
    without a scenario writes no faults.jsonl."""
    clean = run("maxpressure", 300, 1, results_dir=tmp_path / "clean")
    clean_out = tmp_path / "clean" / clean["run_id"]
    assert not (clean_out / "faults.jsonl").exists()
    assert clean["faults"] is None

    folder = tmp_path / "scenarios"
    folder.mkdir()
    (folder / "late.json").write_text(json.dumps({"seed": 1, "faults": [
        {"approach": "J1_J2", "kind": "freeze", "start_s": 1000, "end_s": 2000},
        {"approach": "J2_J3", "kind": "displaced", "start_s": 1000, "end_s": 2000, "shift_m": 40}]}))
    monkeypatch.setattr(faults_module, "FAULTS_DIR", folder)
    late = run("maxpressure", 300, 1, results_dir=tmp_path / "late", faults="late")
    late_out = tmp_path / "late" / late["run_id"]

    for name in ("truth.jsonl", "measurements.jsonl", "states.jsonl", "commands.jsonl"):
        assert (clean_out / name).read_text() == (late_out / name).read_text(), name
    assert all(r["fault"] is None for r in records(late_out / "faults.jsonl"))


@pytest.mark.parametrize("controller", ["maxpressure_hyst_cap", "fixed_webster_wave"])
def test_faults_work_under_every_controller_family(controller: str, tmp_path: Path) -> None:
    """The injector sits before the core, so the Module 3 controllers (D20-D22)
    get the same corrupted measurements: every kind applied and logged."""
    summary = run(controller, 300, 1, results_dir=tmp_path, faults="smoke")
    out = tmp_path / summary["run_id"]
    assert (summary["controller"], summary["faults"]) == (controller, "smoke")
    assert {r["fault"] for r in records(out / "faults.jsonl")} - {None} == {
        "freeze", "blackout", "undercount", "dropout", "displaced"}
    seen = keyed(out / "measurements.jsonl")
    assert sum(a == "E_J3" for _, a in seen) < 300  # dropout
    assert all(sum(seen[(float(t), "N2_J2")]["zone_counts"].values()) == 0 for t in range(60, 120))
    frozen = [seen[(float(t), "J1_J2")] for t in range(59, 180)]
    assert all({**m, "time": 0} == {**frozen[0], "time": 0} for m in frozen)
    assert len(records(out / "commands.jsonl")) == 300 * 3


def test_displaced_runs_are_deterministic(tmp_path: Path) -> None:
    a = run("maxpressure", 120, 1, results_dir=tmp_path / "a", faults="smoke")
    b = run("maxpressure", 120, 1, results_dir=tmp_path / "b", faults="smoke")
    for name in ("faults.jsonl", "measurements.jsonl", "states.jsonl", "commands.jsonl"):
        first, second = tmp_path / "a" / a["run_id"] / name, tmp_path / "b" / b["run_id"] / name
        assert first.read_text() == second.read_text(), name
