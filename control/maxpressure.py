class MaxPressureController:
    """Basic max-pressure, re-decided every second at every junction.

    pressure(phase) = sum over its approaches of
        queue on the approach - queue on the approach it feeds at the next junction
    Approaches whose through traffic leaves the corridor count downstream as 0.
    Queues are the Guard's queue_estimate (PCU). Ties keep the current phase.
    Not capacity-aware yet.
    """

    name = "maxpressure"

    def __init__(self, corridor):
        self.corridor = corridor

    def decide(self, t, rated, last_commands):
        queue = {r.approach_id: r.queue_estimate for r in rated}
        requests = {}
        for jid, junction in self.corridor.junctions.items():
            pressures = [sum(self._pressure(a, queue) for a in phase) for phase in junction.phases]
            current = last_commands[jid].target_phase if jid in last_commands else 0
            requests[jid] = max(range(len(pressures)), key=lambda i: (pressures[i], i == current))
        return requests

    def _pressure(self, approach, queue):
        down = self.corridor.downstream.get(approach)
        return queue.get(approach, 0.0) - (queue.get(down, 0.0) if down else 0.0)
