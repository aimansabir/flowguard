class FixedTimeController:
    """Plain fixed-time plan, same at every junction, no offsets between them.

    Asks for phase k until its planned green is used up, then asks for the
    next one. The envelope adds yellow + all-red in between, so the cycle is
    sum(greens) + one clearance per phase.
    """

    name = "fixed"

    def __init__(self, corridor, greens, clearance_s):
        for j in corridor.junctions.values():
            if len(j.phases) != len(greens):
                raise ValueError(f"{j.id} has {len(j.phases)} phases, plan has {len(greens)}")
        self.junction_ids = list(corridor.junctions)
        self.green_ends = []
        t = 0
        for g in greens:
            t += g
            self.green_ends.append(t)
            t += clearance_s
        self.cycle = t

    def decide(self, t, rated, last_commands):
        pos = t % self.cycle
        phase = next((i for i, end in enumerate(self.green_ends) if pos < end), 0)
        return {jid: phase for jid in self.junction_ids}
