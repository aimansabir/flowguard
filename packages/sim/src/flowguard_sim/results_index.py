"""Index of the runs under a results directory, for the dashboard's Research view.

Read-only: a run directory is never written to. A run is any directory holding
`summary.json` (written when a run finishes) or `measurements.jsonl` (a run
still going, or one that stopped early). Runs with a Guard log are scored with
`guard_eval.evaluate` in a separate process (a one-hour run takes a few
seconds of parsing, which must not hold up a live simulation in this one); a
`guard_eval.json` already in the directory (written by the
guard_eval CLI) is used as is.
"""

import json
import multiprocessing
import threading
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

from flowguard_sim.guard_eval import evaluate

# guard_eval output kept per run; the episode list is left out (it can be long)
EVAL_KEYS = ("confusion", "missing_covered_by_virtual_sensor", "false_alarms", "by_kind")


def find_runs(root: Path, depth: int = 4) -> list[Path]:
    """Run directories under `root`, at most `depth` levels down."""
    if not root.is_dir():
        return []
    out: list[Path] = []
    stack = [(root, 0)]
    while stack:
        d, level = stack.pop()
        if (d / "summary.json").is_file() or (d / "measurements.jsonl").is_file():
            out.append(d)
            continue
        if level < depth:
            stack.extend((c, level + 1) for c in d.iterdir() if c.is_dir())
    return sorted(out)


def describe(run: Path, root: Path) -> dict[str, Any]:
    summary_file = run / "summary.json"
    summary = json.loads(summary_file.read_text(encoding="utf-8")) if summary_file.is_file() else None
    return {
        "run": run.name,
        "group": str(run.parent.relative_to(root)).replace("\\", "/") if run.parent != root else "",
        "status": "complete" if summary else "incomplete",
        "summary": summary,
        "files": sorted(p.name for p in run.iterdir() if p.is_file()),
        "eval": None,
    }


def scored(run: Path) -> dict[str, Any] | None:
    """guard_eval for a run with a Guard log, None for one without."""
    if not (run / "states.jsonl").is_file():
        return None
    cached = run / "guard_eval.json"
    r = json.loads(cached.read_text(encoding="utf-8")) if cached.is_file() else evaluate(run)
    return {k: r[k] for k in EVAL_KEYS if k in r}


class ResultsIndex:
    """Scans on request; scores complete runs once, in the background."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self._lock = threading.Lock()
        self._evals: dict[Path, dict[str, Any] | None] = {}
        self._errors: dict[Path, str] = {}
        self._worker: threading.Thread | None = None

    def snapshot(self) -> dict[str, Any]:
        runs = find_runs(self.root)
        rows: list[dict[str, Any]] = []
        todo: list[Path] = []
        for r in runs:
            row = describe(r, self.root)
            with self._lock:
                if r in self._evals:
                    row["eval"] = self._evals[r]
                elif r in self._errors:
                    row["eval_error"] = self._errors[r]
                elif row["status"] == "complete":
                    todo.append(r)
            rows.append(row)
        self._score(todo)
        return {"root": str(self.root), "runs": rows, "scoring": len(todo)}

    def wait(self, timeout: float | None = None) -> None:
        """Block until the runs being scored are done (tests, scripts)."""
        if self._worker:
            self._worker.join(timeout)

    def _score(self, todo: list[Path]) -> None:
        if not todo or (self._worker and self._worker.is_alive()):
            return

        def work() -> None:
            # spawn, not fork: this process runs the HTTP and simulation threads
            with ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context("spawn")) as pool:
                for r in todo:
                    try:
                        result = pool.submit(scored, r).result()
                    except (OSError, ValueError, KeyError) as e:
                        with self._lock:
                            self._errors[r] = str(e)
                        continue
                    with self._lock:
                        self._evals[r] = result

        self._worker = threading.Thread(target=work, daemon=True)
        self._worker.start()
