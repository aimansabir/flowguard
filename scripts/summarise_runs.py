"""Results table from the summary.json files under a folder (docs/results.md).

    python scripts/summarise_runs.py results/d35 --baseline fixed_webster_wave

Per demand and controller: mean ± sample standard deviation over seeds of
average delay (SUMO timeLoss, per arrived vehicle), the wait to enter the
network (not in timeLoss), throughput, vehicles still in the network at the
end and the longest red any approach saw. Then each controller against the
baseline on the same seed and demand (the same vehicles): mean difference
and the number of seeds where it was better. Markdown on stdout.
"""

import argparse
import json
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

# key, column label, +1 if higher is better, -1 if lower is better
METRICS: list[tuple[str, str, int]] = [
    ("avg_delay_s", "avg delay (s)", -1),
    ("avg_depart_delay_s", "entry wait (s)", -1),
    ("throughput_veh_per_h", "throughput (veh/h)", 1),
    ("vehicles_in_network_at_end", "left in network", -1),
    ("longest_red", "longest red (s)", -1),
]


def load(root: Path) -> dict[tuple[str, str], dict[int, dict[str, Any]]]:
    by: dict[tuple[str, str], dict[int, dict[str, Any]]] = defaultdict(dict)
    for f in sorted(root.rglob("summary.json")):
        s = json.loads(f.read_text(encoding="utf-8"))
        s["longest_red"] = max(s["longest_red_s"].values())
        by[(s["demand"], s["controller"])][s["seed"]] = s
    return by


def cell(xs: list[float]) -> str:
    return f"{st.mean(xs):.1f} ± {st.stdev(xs):.1f}" if len(xs) > 1 else f"{xs[0]:.1f}"


def main(root: Path, baseline: str, order: list[str]) -> None:
    by = load(root)
    demands = sorted({d for d, _ in by}, key=lambda d: (d != "normal", d))
    for demand in demands:
        others = sorted(c for d, c in by if d == demand and c not in order)
        ctls = [c for c in order if (demand, c) in by] + others
        seeds = sorted({s for c in ctls for s in by[(demand, c)]})
        print(f"\n#### {demand} (seeds {', '.join(map(str, seeds))})\n")
        print("| controller | " + " | ".join(m[1] for m in METRICS) + " |")
        print("|---|" + "---|" * len(METRICS))
        for c in ctls:
            runs = by[(demand, c)]
            cells = [cell([runs[s][k] for s in sorted(runs)]) for k, _, _ in METRICS]
            print(f"| {c} | " + " | ".join(cells) + " |")
        base = by.get((demand, baseline), {})
        if not base:
            continue
        print(f"\nvs `{baseline}`, same seeds: mean difference (seeds better / seeds)\n")
        print("| controller | " + " | ".join(m[1] for m in METRICS) + " |")
        print("|---|" + "---|" * len(METRICS))
        for c in ctls:
            if c == baseline:
                continue
            runs = by[(demand, c)]
            common = sorted(set(runs) & set(base))
            cells: list[str] = []
            for k, _, better in METRICS:
                diffs = [runs[s][k] - base[s][k] for s in common]
                ref = st.mean(base[s][k] for s in common)
                pct = f" ({100 * st.mean(diffs) / ref:+.0f} %)" if abs(ref) >= 100 else ""
                wins = sum(d * better > 0 for d in diffs)
                cells.append(f"{st.mean(diffs):+.1f}{pct} ({wins}/{len(diffs)})")
            print(f"| {c} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--baseline", default="fixed_webster_wave")
    ap.add_argument("--order", nargs="*", default=["fixed_webster_wave", "maxpressure_hyst_cap",
                                                   "maxpressure_hyst_cap_lanes_moving", "actuated",
                                                   "adaptive_split"])
    a = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[union-attr]  # Windows consoles
    main(a.root, a.baseline, a.order)
