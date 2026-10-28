"""Builds Measurements from SUMO true state. The only code that reads it.

Two views per approach every second:
- camera view: only the last `camera_zone_m` metres before the stop line,
  what a pole-mounted camera can actually see. This goes to the core.
- truth: the whole approach edge. Logged as ground truth and used for metrics,
  never shown to the core.

Counts are raw per vehicle class; PCU weighting is the core's job.
Arrivals = vehicles that entered the view this second (including vehicles
inserted inside it). Departures = vehicles that left the approach edge this
second, split by the movement their route takes next.
"""
# pyright: basic
# traci ships without type information.

from dataclasses import dataclass, field

import traci
from traci import constants as tc

from flowguard_contracts import Corridor, Measurement
from flowguard_sim.corridor import approaches_of

HALT_SPEED = 0.1  # m/s, same threshold SUMO uses for "halting"
CLASSES = ("car", "motorcycle", "rickshaw", "qingqi", "bus", "truck")
SUBSCRIBED = [tc.VAR_TYPE, tc.VAR_ROAD_ID, tc.VAR_SPEED, tc.VAR_LANEPOSITION, tc.VAR_POSITION, tc.VAR_ANGLE]


def _zeros() -> dict[str, int]:
    return dict.fromkeys(CLASSES, 0)


@dataclass
class _Tally:
    zone: dict[str, int] = field(default_factory=_zeros)
    queue: dict[str, int] = field(default_factory=_zeros)
    arrivals: dict[str, int] = field(default_factory=_zeros)


class SimObserver:
    def __init__(self, corridor: Corridor, camera_zone_m: float | None):
        """camera_zone_m=None: the camera sees the whole edge (same as truth)."""
        self.approach_ids = [a.id for j in corridor.junctions for a in approaches_of(corridor, j.id)]
        self.junction_of = {a.id: a.junction for a in corridor.approaches}
        self.length = {a.id: a.length_m for a in corridor.approaches}
        self.zone = {a.id: min(camera_zone_m, a.length_m) if camera_zone_m else a.length_m
                     for a in corridor.approaches}
        # approach -> {edge the vehicle turns onto: movement}
        self.movement_of = {
            a.id: {edge: mv for mv, edge in a.movements.model_dump().items() if edge}
            for a in corridor.approaches
        }
        self.routes: dict[str, list[str]] = {}
        self.prev: dict[str, tuple[str, bool]] = {}  # vehicle -> (approach, in camera zone) last second

    def observe(self, t: float) -> tuple[list[Measurement], list[Measurement]]:
        """(camera view, truth), one Measurement per approach each."""
        for vid in traci.simulation.getDepartedIDList():
            traci.vehicle.subscribe(vid, SUBSCRIBED)
            self.routes[vid] = list(traci.vehicle.getRoute(vid))
        for vid in traci.simulation.getArrivedIDList():
            self.prev.pop(vid, None)
            self.routes.pop(vid, None)

        cam = {a: _Tally() for a in self.approach_ids}
        truth = {a: _Tally() for a in self.approach_ids}
        departures = {a: {m: _zeros() for m in ("left", "straight", "right")} for a in self.approach_ids}

        now: dict[str, tuple[str, bool]] = {}
        for vid, v in traci.vehicle.getAllSubscriptionResults().items():
            road, vtype = v[tc.VAR_ROAD_ID], v[tc.VAR_TYPE]
            was = self.prev.get(vid)

            if was is not None and was[0] != road:
                route = self.routes[vid]
                next_edge = route[route.index(was[0]) + 1]
                departures[was[0]][self.movement_of[was[0]][next_edge]][vtype] += 1

            if road not in cam:
                continue
            stopped = v[tc.VAR_SPEED] < HALT_SPEED
            in_zone = v[tc.VAR_LANEPOSITION] >= self.length[road] - self.zone[road]
            now[vid] = (road, in_zone)

            views = [(truth[road], True, was is not None and was[0] == road),
                     (cam[road], in_zone, was is not None and was[0] == road and was[1])]
            for tally, inside, was_inside in views:
                if not inside:
                    continue
                tally.zone[vtype] += 1
                if stopped:
                    tally.queue[vtype] += 1
                if not was_inside:
                    tally.arrivals[vtype] += 1
        self.prev = now

        def build(a: str, tally: _Tally, zone_m: float) -> Measurement:
            return Measurement.model_validate({
                "time": t, "interval_s": 1.0, "junction_id": self.junction_of[a], "approach_id": a,
                "source": "sim", "zone_length_m": zone_m,
                "zone_counts": tally.zone, "queue_counts": tally.queue, "arrivals": tally.arrivals,
                "departures": departures[a],
            })

        return ([build(a, cam[a], self.zone[a]) for a in self.approach_ids],
                [build(a, truth[a], self.length[a]) for a in self.approach_ids])
