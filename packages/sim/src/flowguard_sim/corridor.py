"""Loading the corridor map plus the few lookups the harness needs."""

from pathlib import Path

from flowguard_contracts import Approach, Command, Corridor, Junction


def load_corridor(path: Path) -> Corridor:
    return Corridor.model_validate_json(path.read_text())


def junctions_by_id(corridor: Corridor) -> dict[str, Junction]:
    return {j.id: j for j in corridor.junctions}


def approaches_of(corridor: Corridor, junction_id: str) -> list[Approach]:
    return [a for a in corridor.approaches if a.junction == junction_id]


def light(junction: Junction, approach: str, cmd: Command) -> str:
    """'G', 'y' or 'r' for one approach under a Command."""
    if cmd.stage == "all_red" or approach not in junction.phases[cmd.phase_index]:
        return "r"
    return "G" if cmd.stage == "green" else "y"
