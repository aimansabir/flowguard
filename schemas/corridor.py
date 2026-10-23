"""Static map of the corridor, loaded from sim/net/corridor.json.

This is map knowledge (which approaches exist, which phase serves which
approach, what feeds what), not live simulator state, so the Guard and the
controllers are allowed to use it. Nothing here touches SUMO.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class Junction:
    id: str
    approaches: list[str]
    phases: list[list[str]]  # phase index -> approaches that get green
    links: list[list[str]] = field(default_factory=list)  # signal link index -> [approach, direction]

    def light(self, approach, cmd):
        """'G', 'y' or 'r' for one approach under a Command."""
        if cmd.stage == "all_red" or approach not in self.phases[cmd.phase_index]:
            return "r"
        return "G" if cmd.stage == "green" else "y"


@dataclass
class Corridor:
    junctions: dict[str, Junction]
    downstream: dict[str, str | None]  # approach -> approach its through traffic joins next (None = leaves)
    exit_links: dict[str, list[str]]  # edge between two junctions -> [from_junction, to_junction]
    entries: dict[str, dict]  # edge where traffic enters -> {"kind": main|side, "through_exit": edge}
    exits: dict[str, str]  # edge where traffic leaves -> main|side

    def junction_of(self, approach):
        for j in self.junctions.values():
            if approach in j.approaches:
                return j.id
        raise KeyError(approach)


def load_corridor(path):
    raw = json.loads(Path(path).read_text())
    junctions = {jid: Junction(id=jid, **j) for jid, j in raw["junctions"].items()}
    return Corridor(junctions, raw["downstream"], raw["exit_links"], raw["entries"], raw["exits"])
