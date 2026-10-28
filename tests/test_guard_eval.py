"""Guard evaluation (flowguard_sim.guard_eval) on small hand-made logs whose
metrics can be worked out by hand."""

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from flowguard_sim.guard_eval import episodes, evaluate, load, main

T, S, F = "TRUSTED", "SUSPECT", "FAULTY"


def write_run(tmp_path: Path, faults: Mapping[str, Sequence[str | None]] | None,
              states: Mapping[str, Sequence[str | None]]) -> Path:
    """faults: approach -> kind or None per second from t = 0; states:
    approach -> trust state or None (no state logged) per second."""
    run = tmp_path / "run"
    run.mkdir()
    rows = [{"time": float(t), "approach_id": a, "trust_state": s}
            for a, seq in states.items() for t, s in enumerate(seq) if s is not None]
    (run / "states.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    if faults is not None:
        rows = [{"time": float(t), "approach_id": a, "fault": k}
                for a, seq in faults.items() for t, k in enumerate(seq)]
        (run / "faults.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    return run


def ep(run: Path) -> list[dict[str, Any]]:
    episodes_: list[dict[str, Any]] = evaluate(run)["episodes"]
    return episodes_


def test_detected_fault_delay_and_recovery(tmp_path: Path) -> None:
    # freeze on t = 10..19; SUSPECT at 14, FAULTY 15..22, TRUSTED again at 23
    faults: list[str | None] = [None] * 10 + ["freeze"] * 10 + [None] * 10
    states: Sequence[str | None] = [T] * 14 + [S] + [F] * 8 + [T] * 7
    run = write_run(tmp_path, {"A": faults}, {"A": states})
    (e,) = ep(run)
    assert (e["kind"], e["start_s"], e["end_s"], e["censored"]) == ("freeze", 10.0, 20.0, False)
    assert (e["detected"], e["detection_delay_s"], e["flag_delay_s"]) == (True, 5.0, 4.0)
    assert e["recovery_s"] == 3.0
    r = evaluate(run)
    assert r["by_kind"]["freeze"]["detection_rate"] == 1.0
    assert r["confusion"]["freeze"] == {"TRUSTED": 4, "SUSPECT": 1, "FAULTY": 5, "MISSING": 0}
    assert r["confusion"]["clean"] == {"TRUSTED": 17, "SUSPECT": 0, "FAULTY": 3, "MISSING": 0}
    # the 3 FAULTY seconds after the fault are false alarms, all within recovery
    fa = r["false_alarms"]
    assert (fa["clean_rated_seconds"], fa["faulty_rate"]) == (20, 3 / 20)
    assert fa["faulty_rate_outside_recovery"] == 0.0


def test_suspect_without_faulty_is_not_a_detection(tmp_path: Path) -> None:
    run = write_run(tmp_path, {"A": [None] * 5 + ["undercount"] * 10 + [None] * 5},
                    {"A": [T] * 8 + [S] * 7 + [T] * 5})
    (e,) = ep(run)
    assert (e["detected"], e["detection_delay_s"]) == (False, None)
    assert (e["flag_delay_s"], e["recovery_s"]) == (3.0, 0.0)
    assert evaluate(run)["by_kind"]["undercount"]["detection_rate"] == 0.0


def test_never_flagged(tmp_path: Path) -> None:
    run = write_run(tmp_path, {"A": [None] * 5 + ["blackout"] * 5 + [None] * 5}, {"A": [T] * 15})
    (e,) = ep(run)
    assert (e["detected"], e["flag_delay_s"], e["recovery_s"]) == (False, None, 0.0)


def test_fault_from_start_to_end_is_censored(tmp_path: Path) -> None:
    run = write_run(tmp_path, {"A": ["freeze"] * 20}, {"A": [T] * 3 + [S] * 2 + [F] * 15})
    (e,) = ep(run)
    assert (e["start_s"], e["end_s"], e["censored"]) == (0.0, None, True)
    assert (e["detection_delay_s"], e["recovery_s"]) == (5.0, None)
    k = evaluate(run)["by_kind"]["freeze"]
    assert (k["ended"], k["recovered"], k["mean_recovery_s"]) == (0, 0, None)


def test_verdict_after_the_fault_does_not_count(tmp_path: Path) -> None:
    run = write_run(tmp_path, {"A": [None] * 5 + ["freeze"] * 5 + [None] * 10},
                    {"A": [T] * 10 + [F] * 4 + [T] * 6})
    (e,) = ep(run)
    assert (e["detected"], e["recovery_s"]) == (False, 4.0)


def test_missing_states_from_dropout(tmp_path: Path) -> None:
    # dropout t = 0..9 with every other state missing; after a freeze the
    # first state is missing, so recovery skips it
    faults: list[str | None] = ["dropout"] * 10 + ["freeze"] * 5 + [None] * 5
    states: list[str | None] = [T, None] * 5 + [S, F, F, F, F] + [None, None, T, T, T]
    run = write_run(tmp_path, {"A": faults}, {"A": states})
    r = evaluate(run)
    assert r["confusion"]["dropout"] == {"TRUSTED": 5, "SUSPECT": 0, "FAULTY": 0, "MISSING": 5}
    drop, freeze = r["episodes"]
    assert (drop["kind"], drop["end_s"], drop["detected"]) == ("dropout", 10.0, False)
    assert (freeze["start_s"], freeze["detection_delay_s"], freeze["recovery_s"]) == (10.0, 1.0, 2.0)
    assert r["confusion"]["clean"]["MISSING"] == 2
    assert r["false_alarms"]["clean_rated_seconds"] == 3


def test_repeated_windows_and_several_approaches(tmp_path: Path) -> None:
    a: list[str | None] = ["freeze"] * 5 + [None] * 5 + ["freeze"] * 5 + [None] * 5
    b: list[str | None] = [None] * 10 + ["blackout"] * 10
    run = write_run(tmp_path, {"A": a, "B": b, "C": [None] * 20},
                    {"A": [F] * 5 + [T] * 5 + [T] * 2 + [F] * 3 + [T] * 5,
                     "B": [T] * 12 + [S] * 2 + [F] * 6,
                     "C": [T] * 19 + [F]})
    eps = ep(run)
    assert [(e["approach"], e["kind"], e["start_s"]) for e in eps] == [
        ("A", "freeze", 0.0), ("A", "freeze", 10.0), ("B", "blackout", 10.0)]
    assert [e["detection_delay_s"] for e in eps] == [0.0, 2.0, 4.0]
    assert [e["recovery_s"] for e in eps] == [0.0, 0.0, None]
    r = evaluate(run)
    assert r["by_kind"]["freeze"]["episodes"] == 2
    assert r["by_kind"]["freeze"]["mean_detection_delay_s"] == 1.0
    # C has no fault: its one FAULTY second is a false alarm outside any recovery
    fa = r["false_alarms"]
    assert fa["clean_rated_seconds"] == 10 + 10 + 20
    assert fa["faulty_rate"] == 1 / 40
    assert fa["faulty_rate_outside_recovery"] == 1 / 40


def test_clean_run_without_faults_file(tmp_path: Path) -> None:
    run = write_run(tmp_path, None, {"A": [T] * 98 + [S, F], "B": [T] * 100})
    r = evaluate(run)
    assert r["episodes"] == []
    assert r["false_alarms"]["faulty_rate"] == 1 / 200
    assert r["false_alarms"]["suspect_or_faulty_rate"] == 2 / 200


def test_load_and_episodes_agree(tmp_path: Path) -> None:
    run = write_run(tmp_path, {"A": [None, "freeze", "freeze", None]}, {"A": [T, T, F, T]})
    truth, decision = load(run)
    assert truth["A"] == {0.0: "clean", 1.0: "freeze", 2.0: "freeze", 3.0: "clean"}
    assert decision["A"][2.0] == F
    assert len(episodes(truth, decision)) == 1


def test_cli_writes_the_report(tmp_path: Path, capsys: Any) -> None:
    run = write_run(tmp_path, {"A": [None] * 5 + ["freeze"] * 5}, {"A": [T] * 7 + [F] * 3})
    main([run])
    saved = json.loads((run / "guard_eval.json").read_text())
    assert saved["by_kind"]["freeze"]["detected"] == 1
    assert "freeze" in capsys.readouterr().out
