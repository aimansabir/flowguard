"""Run metrics. Definitions are written out in docs/decisions.md (D10)."""
import xml.etree.ElementTree as ET


class RunMetrics:
    def __init__(self, corridor):
        self.corridor = corridor
        approaches = [a for j in corridor.junctions.values() for a in j.approaches]
        self.max_queue = dict.fromkeys(approaches, 0.0)
        self.longest_red = dict.fromkeys(approaches, 0.0)
        self.red_since = {}

    def add_observations(self, observations):
        for o in observations:
            self.max_queue[o.approach_id] = max(self.max_queue[o.approach_id], o.queue_pcu)

    def add_commands(self, commands):
        for c in commands:
            junction = self.corridor.junctions[c.junction_id]
            for a in junction.approaches:
                if junction.light(a, c) == "r":
                    self.red_since.setdefault(a, c.time)
                elif a in self.red_since:
                    self._close_red(a, c.time)

    def _close_red(self, approach, t):
        self.longest_red[approach] = max(self.longest_red[approach], t - self.red_since.pop(approach))

    def summary(self, tripinfo_file, end_time, departed):
        for a in list(self.red_since):
            self._close_red(a, end_time)

        time_loss, depart_delay = [], []
        for _, el in ET.iterparse(tripinfo_file):
            if el.tag == "tripinfo":
                time_loss.append(float(el.get("timeLoss")))
                depart_delay.append(float(el.get("departDelay")))
                el.clear()
        arrived = len(time_loss)

        return {
            "vehicles_departed": departed,
            "vehicles_arrived": arrived,
            "avg_delay_s": round(sum(time_loss) / arrived, 2) if arrived else None,
            "avg_depart_delay_s": round(sum(depart_delay) / arrived, 2) if arrived else None,
            "throughput_veh_per_h": round(arrived * 3600 / end_time, 1),
            "max_queue_pcu": {a: round(q, 1) for a, q in self.max_queue.items()},
            "longest_red_s": self.longest_red,
        }
