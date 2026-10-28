"""Turns envelope Commands into SUMO signal state strings and applies them.

Each link (lane-to-lane movement) of an approach in the lit phase gets a
green. It is protected ('G') unless it must give way to another movement
green at the same time, in which case it is permissive ('g') and yields as
SUMO's junction model prescribes: the turn across oncoming traffic (the
right turn in left-hand traffic, the left turn in right-hand traffic). Which
movements conflict and which one yields comes from the network itself
(conflicts.py), not from direction codes, so it holds for either driving
side (decisions.md D29).

Two checks guard what reaches SUMO (decisions.md D4, D29): no state may
light approaches of more than one phase, and no two protected greens may
conflict. Every phase is checked when the applier starts.
"""

from flowguard_contracts import Command, Corridor, Junction
from flowguard_sim.conflicts import Conflicts
from flowguard_sim.corridor import junctions_by_id, light


def state_string(junction: Junction, cmd: Command, conflicts: Conflicts) -> str:
    lit = [light(junction, link.approach, cmd) for link in junction.links]
    out: list[str] = []
    for i, ch in enumerate(lit):
        if ch == "G" and any(lit[j] == "G" and conflicts.yields[i][j] for j in range(len(lit))):
            ch = "g"
        out.append(ch)
    return "".join(out)


def has_conflicting_greens(junction: Junction, state: str) -> bool:
    """True if links from approaches in different phases are green together."""
    green = {junction.links[i].approach for i, ch in enumerate(state) if ch in "Gg"}
    return not any(green <= set(phase) for phase in junction.phases)


def protected_conflicts(state: str, conflicts: Conflicts) -> list[tuple[int, int]]:
    """Pairs of links that are both protected green yet cross or merge."""
    green = [i for i, ch in enumerate(state) if ch == "G"]
    return [(i, j) for i in green for j in green if i < j and conflicts.foes[i][j]]


def check_phases(junction: Junction, conflicts: Conflicts) -> None:
    """Raises if any phase of the junction would light conflicting protected movements."""
    if len(conflicts.foes) != len(junction.links):
        raise ValueError(f"{junction.id}: {len(junction.links)} links in the corridor map, "
                         f"{len(conflicts.foes)} in the network")
    for phase in range(len(junction.phases)):
        state = state_string(junction, _green(junction.id, phase), conflicts)
        if bad := protected_conflicts(state, conflicts):
            raise ValueError(f"{junction.id} phase {phase} ({state}) gives conflicting movements "
                             f"protected green: links {bad}")


def _green(junction_id: str, phase: int) -> Command:
    return Command.model_validate({"time": 0, "junction_id": junction_id, "phase_index": phase,
                                   "phase_started_at": 0, "controller_name": "check", "stage": "green",
                                   "target_phase": phase, "requested_phase": None, "intervention": None})


class SignalApplier:
    def __init__(self, corridor: Corridor, conflicts: dict[str, Conflicts]):
        self.junctions = junctions_by_id(corridor)
        self.conflicts = conflicts
        for jid, junction in self.junctions.items():
            check_phases(junction, conflicts[jid])
        self.shown: dict[str, str] = {}

    def apply(self, commands: list[Command]) -> None:
        import traci  # pyright: ignore[reportMissingTypeStubs]

        for cmd in commands:
            junction = self.junctions[cmd.junction_id]
            conflicts = self.conflicts[cmd.junction_id]
            state = state_string(junction, cmd, conflicts)
            if has_conflicting_greens(junction, state) or protected_conflicts(state, conflicts):
                raise RuntimeError(f"conflicting greens at {cmd.junction_id}: {state}")
            if self.shown.get(cmd.junction_id) != state:
                traci.trafficlight.setRedYellowGreenState(cmd.junction_id, state)  # pyright: ignore
                self.shown[cmd.junction_id] = state
