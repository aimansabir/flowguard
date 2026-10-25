"""Turns envelope Commands into SUMO signal state strings and applies them.

The conflict check here is the second line of defence (decisions.md D4): the
core already lights one phase at a time, and this refuses to send SUMO any
state where green links come from more than one phase.
"""

from flowguard_contracts import Command, Corridor, Junction

from flowguard_sim.corridor import junctions_by_id, light


def state_string(junction: Junction, cmd: Command) -> str:
    lights: list[str] = []
    for link in junction.links:
        lit = light(junction, link.approach, cmd)
        # left turns and u-turns cross oncoming traffic, so they get a
        # permissive green and have to yield
        if lit == "G" and link.direction in ("l", "L", "t"):
            lit = "g"
        lights.append(lit)
    return "".join(lights)


def has_conflicting_greens(junction: Junction, state: str) -> bool:
    """True if links from approaches in different phases are green together."""
    green = {junction.links[i].approach for i, ch in enumerate(state) if ch in "Gg"}
    return not any(green <= set(phase) for phase in junction.phases)


class SignalApplier:
    def __init__(self, corridor: Corridor):
        self.junctions = junctions_by_id(corridor)
        self.shown: dict[str, str] = {}

    def apply(self, commands: list[Command]) -> None:
        import traci

        for cmd in commands:
            junction = self.junctions[cmd.junction_id]
            state = state_string(junction, cmd)
            if has_conflicting_greens(junction, state):
                raise RuntimeError(f"conflicting greens at {cmd.junction_id}: {state}")
            if self.shown.get(cmd.junction_id) != state:
                traci.trafficlight.setRedYellowGreenState(cmd.junction_id, state)  # pyright: ignore
                self.shown[cmd.junction_id] = state
