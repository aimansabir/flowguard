"""Print two or more run summaries side by side.

    uv run python -m flowguard_sim.compare results/<run_a>/summary.json results/<run_b>/summary.json
"""

import json
import sys
from pathlib import Path
from typing import Any

SCALARS = [
    ("avg delay per vehicle (s)", "avg_delay_s"),
    ("avg insertion delay (s)", "avg_depart_delay_s"),
    ("throughput (veh/h)", "throughput_veh_per_h"),
    ("vehicles departed", "vehicles_departed"),
    ("vehicles arrived", "vehicles_arrived"),
    ("still in network at end", "vehicles_in_network_at_end"),
    ("teleports", "teleports"),
    ("colliding vehicles", "colliding_vehicles"),
]


def row(label: str, values: list[Any]) -> None:
    print(f"{label:<32}" + "".join(f"{str(v):>16}" for v in values))


def main(paths: list[str]) -> None:
    runs: list[dict[str, Any]] = [json.loads(Path(p).read_text()) for p in paths]
    if len({r["demand_sha256"] for r in runs}) > 1:
        print("WARNING: these runs used different demand files\n")

    row("", [r["controller"] for r in runs])
    row("camera zone (m)", [r.get("camera_zone_m") or "full" for r in runs])
    for label, key in SCALARS:
        row(label, [r[key] for r in runs])
    row("envelope interventions", [r["envelope_interventions"]["total"] for r in runs])
    for key in ("min_green", "max_red", "invalid_request"):
        row(f"  {key}", [r["envelope_interventions"].get(key, 0) for r in runs])
    print("\nmax queue (PCU) / longest red (s) per approach")
    for a in runs[0]["max_queue_pcu"]:
        row(f"  {a}", [f"{r['max_queue_pcu'][a]} / {r['longest_red_s'][a]:g}" for r in runs])


if __name__ == "__main__":
    main(sys.argv[1:])
