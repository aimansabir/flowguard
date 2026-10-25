"""Run metrics. Definitions are written out in docs/decisions.md (D10)."""

import xml.etree.ElementTree as ET
from collections.abc import Callable
from pathlib import Path
from typing import Any

from flowguard_contracts import ClassCounts, Command, Corridor, Measurement

from flowguard_sim.corridor import approaches_of, junctions_by_id, light


class RunMetrics:
    def __init__(self, corridor: Corridor, pcu: Callable[[ClassCounts], float]):
        """`pcu` turns class counts into PCU with the core's weights, so the
        weighting lives in one place."""
        self.junctions = junctions_by_id(corridor)
        self.approaches = {j.id: [a.id for a in approaches_of(corridor, j.id)] for j in corridor.junctions}
        self.pcu = pcu
        approaches = [a.id for a in corridor.approaches]
        self.max_queue = dict.fromkeys(approaches, 0.0)
        self.longest_red = dict.fromkeys(approaches, 0.0)
        self.red_since: dict[str, float] = {}

    def add_truth(self, truth: list[Measurement]) -> None:
        for m in truth:
            self.max_queue[m.approach_id] = max(self.max_queue[m.approach_id], self.pcu(m.queue_counts))

    def add_commands(self, commands: list[Command]) -> None:
        for c in commands:
            junction = self.junctions[c.junction_id]
            for a in self.approaches[c.junction_id]:
                if light(junction, a, c) == "r":
                    self.red_since.setdefault(a, c.time)
                elif a in self.red_since:
                    self._close_red(a, c.time)

    def _close_red(self, approach: str, t: float) -> None:
        self.longest_red[approach] = max(self.longest_red[approach], t - self.red_since.pop(approach))

    def summary(self, tripinfo_file: Path, end_time: float, departed: int) -> dict[str, Any]:
        for a in list(self.red_since):
            self._close_red(a, end_time)

        time_loss: list[float] = []
        depart_delay: list[float] = []
        for _, el in ET.iterparse(tripinfo_file):
            if el.tag == "tripinfo":
                time_loss.append(float(el.get("timeLoss", "nan")))
                depart_delay.append(float(el.get("departDelay", "nan")))
                el.clear()
        arrived = len(time_loss)

        return {
            "vehicles_departed": departed,
            "vehicles_arrived": arrived,
            "avg_delay_s": round(sum(time_loss) / arrived, 2) if arrived else None,
            "avg_depart_delay_s": round(sum(depart_delay) / arrived, 2) if arrived else None,
            "throughput_veh_per_h": round(arrived * 3600 / end_time, 1) if end_time > 0 else None,
            "max_queue_pcu": {a: round(q, 1) for a, q in self.max_queue.items()},
            "longest_red_s": self.longest_red,
        }
