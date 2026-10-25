"""Compare commands and queues of a v1 (Python) run against a v2 (Rust core) run.

    uv run python scripts/parity.py <v1_run_dir> <v2_run_dir>

Used once to prove the Rust port reproduces the Python implementation: with
the camera seeing the whole edge, every Command and every queue estimate must
match. schema_version differs by design and is ignored.
"""

import json
import sys
from pathlib import Path
from typing import Any

FIELDS = ["time", "junction_id", "phase_index", "phase_started_at", "controller_name", "stage",
          "target_phase", "requested_phase", "intervention"]


def load(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines()]


def main(v1: Path, v2: Path) -> int:
    old, new = load(v1 / "commands.jsonl"), load(v2 / "commands.jsonl")
    assert len(old) == len(new), f"{len(old)} vs {len(new)} commands"
    bad = [i for i, (a, b) in enumerate(zip(old, new, strict=True)) if any(a[f] != b[f] for f in FIELDS)]
    rated, states = load(v1 / "rated.jsonl"), load(v2 / "states.jsonl")
    assert len(rated) == len(states), f"{len(rated)} vs {len(states)} states"
    qbad = [i for i, (a, b) in enumerate(zip(rated, states, strict=True))
            if (a["observation"]["approach_id"], a["queue_estimate"])
            != (b["approach_id"], b["queue_estimate"])]
    print(f"commands: {len(old)} compared, {len(bad)} differ; "
          f"queues: {len(rated)} compared, {len(qbad)} differ")
    for i in (bad + qbad)[:5]:
        print("first difference at record", i)
    return 1 if bad or qbad else 0


if __name__ == "__main__":
    sys.exit(main(Path(sys.argv[1]), Path(sys.argv[2])))
