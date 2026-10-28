"""Which signal links of a junction conflict, read from SUMO's own network.

netconvert computes, for every pair of links (lane-to-lane connections)
through a junction, whether they cross or merge (foes) and which one has
right of way. That depends on the geometry and on the driving side, so it is
the right source for which movements may share a green and which turning
movement must give way (docs/decisions.md D29).
"""
# pyright: basic
# sumolib ships without type information.

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Conflicts:
    """By signal link index: `foes[i][j]` when links i and j cross or merge,
    `yields[i][j]` when link i must give way to link j if both have green."""

    foes: tuple[tuple[bool, ...], ...]
    yields: tuple[tuple[bool, ...], ...]

    @staticmethod
    def build(n: int, foes: set[tuple[int, int]], yields: set[tuple[int, int]]) -> "Conflicts":
        """From pairs; `foes` is made symmetric."""
        both = foes | {(j, i) for i, j in foes}
        return Conflicts(tuple(tuple((i, j) in both for j in range(n)) for i in range(n)),
                         tuple(tuple((i, j) in yields for j in range(n)) for i in range(n)))


def read_conflicts(net_file: Path, junction_ids: list[str]) -> dict[str, Conflicts]:
    import sumolib

    net = sumolib.net.readNet(str(net_file), withPrograms=True)
    out: dict[str, Conflicts] = {}
    for jid in junction_ids:
        node = net.getNode(jid)
        links = {index: next(c for c in in_lane.getOutgoing() if c.getToLane() == out_lane)
                 for in_lane, out_lane, index in net.getTLS(jid).getConnections()}
        n = max(links) + 1
        if sorted(links) != list(range(n)):
            raise ValueError(f"{jid}: signal link indices are not 0..{n - 1}")
        # areFoes takes the junction's own link indices
        index = {i: node.getLinkIndex(c) for i, c in links.items()}
        foes = {(i, j) for i in links for j in links if i != j and node.areFoes(index[i], index[j])}
        # forbids(a, b): a has priority over b, so b gives way to a
        yields = {(i, j) for i in links for j in links if i != j and node.forbids(links[j], links[i])}
        out[jid] = Conflicts.build(n, foes, yields)
    return out
