"""Turns envelope Commands into SUMO signal state strings and applies them."""


def state_string(junction, cmd):
    lights = []
    for approach, direction in junction.links:
        light = junction.light(approach, cmd)
        # left turns and u-turns cross oncoming traffic, so they get a
        # permissive green and have to yield
        if light == "G" and direction in ("l", "L", "t"):
            light = "g"
        lights.append(light)
    return "".join(lights)


def has_conflicting_greens(junction, state):
    """True if links from approaches in different phases are green together."""
    green = {junction.links[i][0] for i, ch in enumerate(state) if ch in "Gg"}
    return not any(green <= set(phase) for phase in junction.phases)


class SignalApplier:
    def __init__(self, corridor):
        self.corridor = corridor
        self.shown = {}

    def apply(self, commands):
        import traci

        for cmd in commands:
            junction = self.corridor.junctions[cmd.junction_id]
            state = state_string(junction, cmd)
            if has_conflicting_greens(junction, state):
                raise RuntimeError(f"conflicting greens at {cmd.junction_id}: {state}")
            if self.shown.get(cmd.junction_id) != state:
                traci.trafficlight.setRedYellowGreenState(cmd.junction_id, state)
                self.shown[cmd.junction_id] = state
