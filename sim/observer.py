"""Builds Observations from SUMO's true state. The only place that reads it.

Downstream code (faults, Guard, controllers) just gets Observations, the same
thing a camera will send later.
"""
import traci
from traci import constants as tc

from schemas.messages import VEHICLE_CLASSES, Observation

HALT_SPEED = 0.1  # m/s, same threshold SUMO uses for "halting"


class SimObserver:
    def __init__(self, corridor, pcu):
        self.corridor = corridor
        self.pcu = pcu
        self.junction_of = {a: j.id for j in corridor.junctions.values() for a in j.approaches}
        self.neighbours = {jid: sorted({to for frm, to in corridor.exit_links.values() if frm == jid})
                           for jid in corridor.junctions}
        self.last_approach = {}  # vehicle id -> last approach edge it was seen on

    def observe(self, t):
        for vid in traci.simulation.getDepartedIDList():
            traci.vehicle.subscribe(vid, [tc.VAR_TYPE, tc.VAR_ROAD_ID, tc.VAR_SPEED])
        for vid in traci.simulation.getArrivedIDList():
            self.last_approach.pop(vid, None)

        counts = {a: dict.fromkeys(VEHICLE_CLASSES, 0) for a in self.junction_of}
        queue = dict.fromkeys(self.junction_of, 0.0)
        exits = {a: dict.fromkeys(self.neighbours[j], 0) for a, j in self.junction_of.items()}

        for vid, v in traci.vehicle.getAllSubscriptionResults().items():
            road, vtype = v[tc.VAR_ROAD_ID], v[tc.VAR_TYPE]
            prev = self.last_approach.get(vid)

            # just moved onto the link toward the next junction?
            if road in self.corridor.exit_links and prev is not None and prev != road:
                frm, to = self.corridor.exit_links[road]
                if self.junction_of[prev] == frm:
                    exits[prev][to] += 1

            if road in counts:
                counts[road][vtype] += 1
                if v[tc.VAR_SPEED] < HALT_SPEED:
                    queue[road] += self.pcu[vtype]
                self.last_approach[vid] = road

        return [
            Observation(time=t, junction_id=self.junction_of[a], approach_id=a,
                        counts_by_class=counts[a], queue_pcu=queue[a], exits_to=exits[a], source="sim")
            for a in self.junction_of
        ]
