"""Guard evaluation (docs/specs/guard.md G4) of one run against its injected faults.

    uv run python -m flowguard_sim.guard_eval results/<run_id> [results/<run_id> ...]

Reads `states.jsonl` (the Guard's verdict per approach-second it had a
measurement) and `faults.jsonl` (G0 ground truth: the active fault kind or
null per approach-second), prints a summary and writes `guard_eval.json`
into each run directory. Semantics (docs/decisions.md D27):

- Every approach-second in faults.jsonl is one sample. Its truth is the fault
  kind active on that approach, or "clean". Its decision is the logged
  trust_state, or MISSING when no state was logged (the measurement never
  arrived, e.g. dropout). A run without faults.jsonl is all clean, over the
  approach-seconds that have a state.
- FAULTY is the positive decision. SUSPECT is reported on its own, never
  folded into either side.
- An episode is a maximal run of consecutive seconds with the same fault kind
  on one approach; it ends at the first second without it (`end_s`). If it
  lasts to the end of the log it is censored: no end, no recovery.
- Detected: FAULTY at least once within [start_s, end_s). Detection delay =
  first FAULTY - start_s; flag delay = first SUSPECT or FAULTY - start_s.
  A verdict after the fault has ended does not count as detecting it.
- Recovery time = first TRUSTED at or after end_s - end_s (0 if TRUSTED at
  end_s); seconds without a state are skipped. None if never TRUSTED again
  before the log ends.
- False-alarm rate = clean approach-seconds rated FAULTY / clean
  approach-seconds with a state (also with SUSPECT or FAULTY). "Recovering"
  seconds, from the end of an episode on that approach until it is TRUSTED
  again, are clean but reported separately as well, since recovery time
  already measures them.
"""

import argparse
import json
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean
from typing import Any

STATES = ("TRUSTED", "SUSPECT", "FAULTY", "MISSING")
CLEAN = "clean"


@dataclass(frozen=True)
class Episode:
    approach: str
    kind: str
    start_s: float
    end_s: float | None  # None: still active when the log ends
    detected: bool
    detection_delay_s: float | None
    flag_delay_s: float | None
    recovery_s: float | None

    @property
    def censored(self) -> bool:
        return self.end_s is None


def _records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def load(run: Path) -> tuple[dict[str, dict[float, str]], dict[str, dict[float, str]]]:
    """(truth, decision): approach -> time -> fault kind or "clean" / trust state."""
    decision: dict[str, dict[float, str]] = defaultdict(dict)
    for s in _records(run / "states.jsonl"):
        decision[s["approach_id"]][float(s["time"])] = s["trust_state"]
    truth: dict[str, dict[float, str]] = defaultdict(dict)
    if (run / "faults.jsonl").exists():
        for f in _records(run / "faults.jsonl"):
            truth[f["approach_id"]][float(f["time"])] = f["fault"] or CLEAN
    else:
        for a, times in decision.items():
            truth[a] = dict.fromkeys(times, CLEAN)
    return truth, decision


def episodes(truth: dict[str, dict[float, str]], decision: dict[str, dict[float, str]]) -> list[Episode]:
    out: list[Episode] = []
    for a, by_time in sorted(truth.items()):
        times = sorted(by_time)
        rated = decision.get(a, {})
        i = 0
        while i < len(times):
            kind = by_time[times[i]]
            if kind == CLEAN:
                i += 1
                continue
            j = i
            while j + 1 < len(times) and by_time[times[j + 1]] == kind:
                j += 1
            start = times[i]
            end = times[j + 1] if j + 1 < len(times) else None
            during = [(t, rated[t]) for t in times[i:j + 1] if t in rated]
            faulty = next((t for t, s in during if s == "FAULTY"), None)
            flagged = next((t for t, s in during if s in ("SUSPECT", "FAULTY")), None)
            recovery = None
            if end is not None:
                back = next((t for t in times[j + 1:] if rated.get(t) == "TRUSTED"), None)
                recovery = None if back is None else back - end
            out.append(Episode(a, kind, start, end, faulty is not None,
                               None if faulty is None else faulty - start,
                               None if flagged is None else flagged - start, recovery))
            i = j + 1
    return out


def _recovering(truth: dict[str, dict[float, str]], eps: list[Episode]) -> set[tuple[str, float]]:
    """Clean approach-seconds between the end of an episode and the return to TRUSTED."""
    out: set[tuple[str, float]] = set()
    for e in eps:
        if e.end_s is None:
            continue
        until = e.end_s + e.recovery_s if e.recovery_s is not None else float("inf")
        out |= {(e.approach, t) for t, k in truth[e.approach].items()
                if k == CLEAN and e.end_s <= t < until}
    return out


def _rate(n: int, d: int) -> float | None:
    return n / d if d else None


def evaluate(run: Path) -> dict[str, Any]:
    truth, decision = load(run)
    eps = episodes(truth, decision)
    recovering = _recovering(truth, eps)

    confusion: dict[str, dict[str, int]] = defaultdict(lambda: dict.fromkeys(STATES, 0))
    clean = {"rated": 0, "faulty": 0, "flagged": 0, "rated_outside_recovery": 0, "faulty_outside_recovery": 0,
             "flagged_outside_recovery": 0}
    for a, by_time in truth.items():
        for t, kind in by_time.items():
            state = decision.get(a, {}).get(t, "MISSING")
            confusion[kind][state] += 1
            if kind != CLEAN or state == "MISSING":
                continue
            outside = (a, t) not in recovering
            for suffix, include in (("", True), ("_outside_recovery", outside)):
                if include:
                    clean["rated" + suffix] += 1
                    clean["faulty" + suffix] += state == "FAULTY"
                    clean["flagged" + suffix] += state in ("SUSPECT", "FAULTY")

    by_kind: dict[str, dict[str, Any]] = {}
    for kind in sorted({e.kind for e in eps}):
        mine = [e for e in eps if e.kind == kind]
        delays = [e.detection_delay_s for e in mine if e.detection_delay_s is not None]
        recoveries = [e.recovery_s for e in mine if e.recovery_s is not None]
        row = confusion[kind]
        by_kind[kind] = {
            "episodes": len(mine),
            "detected": sum(e.detected for e in mine),
            "detection_rate": _rate(sum(e.detected for e in mine), len(mine)),
            "flagged": sum(e.flag_delay_s is not None for e in mine),
            "mean_detection_delay_s": mean(delays) if delays else None,
            "max_detection_delay_s": max(delays) if delays else None,
            "recovered": len(recoveries),
            "ended": sum(not e.censored for e in mine),
            "mean_recovery_s": mean(recoveries) if recoveries else None,
            "faulty_share_of_rated_seconds": _rate(row["FAULTY"],
                                                   row["TRUSTED"] + row["SUSPECT"] + row["FAULTY"]),
        }

    return {
        "run": run.name,
        "approach_seconds": sum(len(v) for v in truth.values()),
        "confusion": {k: confusion[k] for k in sorted(confusion)},
        "false_alarms": {
            "clean_rated_seconds": clean["rated"],
            "faulty_rate": _rate(clean["faulty"], clean["rated"]),
            "suspect_or_faulty_rate": _rate(clean["flagged"], clean["rated"]),
            "faulty_rate_outside_recovery": _rate(clean["faulty_outside_recovery"],
                                                  clean["rated_outside_recovery"]),
            "suspect_or_faulty_rate_outside_recovery": _rate(clean["flagged_outside_recovery"],
                                                             clean["rated_outside_recovery"]),
        },
        "by_kind": by_kind,
        "episodes": [asdict(e) | {"censored": e.censored} for e in eps],
    }


def _fmt(x: float | None, pct: bool = False) -> str:
    if x is None:
        return "-"
    return f"{100 * x:.2f} %" if pct else f"{x:g}"


def print_report(r: dict[str, Any]) -> None:
    fa = r["false_alarms"]
    print(f"\n{r['run']}  ({r['approach_seconds']} approach-seconds)")
    print(f"  false alarms on {fa['clean_rated_seconds']} clean rated approach-seconds: "
          f"FAULTY {_fmt(fa['faulty_rate'], True)}, "
          f"SUSPECT or FAULTY {_fmt(fa['suspect_or_faulty_rate'], True)}"
          f" (outside recovery: {_fmt(fa['faulty_rate_outside_recovery'], True)}, "
          f"{_fmt(fa['suspect_or_faulty_rate_outside_recovery'], True)})")
    print(f"  {'kind':<11} {'episodes':>8} {'detected':>8} {'flagged':>7} {'mean delay':>10} "
          f"{'max delay':>9} {'recovered':>9} {'mean recov':>10}")
    for kind, k in r["by_kind"].items():
        print(f"  {kind:<11} {k['episodes']:>8} {k['detected']:>8} {k['flagged']:>7} "
              f"{_fmt(k['mean_detection_delay_s']):>10} {_fmt(k['max_detection_delay_s']):>9} "
              f"{k['recovered']:>4}/{k['ended']:<4} {_fmt(k['mean_recovery_s']):>10}")
    print(f"  {'truth':<11} " + " ".join(f"{s:>8}" for s in STATES))
    for kind, row in r["confusion"].items():
        print(f"  {kind:<11} " + " ".join(f"{row[s]:>8}" for s in STATES))


def main(runs: list[Path]) -> None:
    for run in runs:
        report = evaluate(run)
        (run / "guard_eval.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print_report(report)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", type=Path)
    main(parser.parse_args().runs)
