"""Build the placeholder corridor and its corridor.json.

    uv run python -m flowguard_sim.build_net

Runs netconvert on the hand-written node/edge files, then reads the network
back with sumolib and writes corridor.json (contract type `Corridor`): the
junctions with their phases and signal link order, every approach with its
length, lanes and where each turn leads, and where traffic enters and leaves.
Phases are exclusive by approach (decisions.md D32): one phase per incoming
approach, so only one approach of a junction is ever green, in the same
geographic order at every junction (from the west, east, north, south) so a
fixed plan and its green wave mean the same thing everywhere. For the real
OSM corridor the phases have to be chosen by hand; this gives a starting
point (decisions.md D6). The driving side comes from
config/sim.json (`driving_side`, D29): Karachi drives on the left.
"""
# pyright: basic
# sumolib ships without type information.

import subprocess
from typing import Any

from flowguard_contracts import Corridor
from flowguard_sim.paths import ROOT, load_json
from flowguard_sim.sumo_env import sumo_binary

NET_DIR = ROOT / "data" / "net"
JUNCTIONS = ["J1", "J2", "J3"]
MAIN_ROAD_NODES = set(JUNCTIONS) | {"W", "E"}
# SUMO direction codes -> movement; "t" (u-turn) is not modelled
DIRECTION_TO_MOVEMENT = {"s": "straight", "l": "left", "L": "left", "r": "right", "R": "right"}


def is_main(edge: Any) -> bool:
    return {edge.getFromNode().getID(), edge.getToNode().getID()} <= MAIN_ROAD_NODES


def straight_on(edge: Any) -> Any:
    for to_edge, conns in edge.getOutgoing().items():
        if any(c.getDirection() == "s" for c in conns):
            return to_edge
    return None


def movement_targets(edge: Any) -> dict[str, str | None]:
    out: dict[str, str | None] = {"left": None, "straight": None, "right": None}
    for to_edge, conns in edge.getOutgoing().items():
        for c in conns:
            m = DIRECTION_TO_MOVEMENT.get(c.getDirection())
            if m:
                out[m] = to_edge.getID()
    return out


def phase_order(junction: Any, main: list[Any], side: list[Any]) -> list[Any]:
    """Main-road approaches from the west then the east, side streets from the
    north then the south (by where each approach starts relative to the junction)."""
    x0, y0 = junction.getCoord()
    by_x = sorted(main, key=lambda e: e.getFromNode().getCoord()[0] - x0)
    by_y = sorted(side, key=lambda e: y0 - e.getFromNode().getCoord()[1])
    return by_x + by_y


def corridor_from_net(net: Any) -> Corridor:
    junctions: list[dict[str, Any]] = []
    approaches: list[dict[str, Any]] = []
    for jid in JUNCTIONS:
        incoming = list(net.getNode(jid).getIncoming())
        main = [e for e in incoming if is_main(e)]
        side = [e for e in incoming if not is_main(e)]

        links: dict[int, dict[str, str]] = {}
        for in_lane, out_lane, index in net.getTLS(jid).getConnections():
            direction = next(c.getDirection() for c in in_lane.getOutgoing() if c.getToLane() == out_lane)
            links[index] = {"approach": in_lane.getEdge().getID(), "direction": direction}

        junctions.append({
            "id": jid,
            "phases": [[e.getID()] for e in phase_order(net.getNode(jid), main, side)],
            "links": [links[i] for i in range(len(links))],
        })
        for e in main + side:
            approaches.append({
                "id": e.getID(),
                "junction": jid,
                "length_m": round(e.getLength(), 2),
                "lanes": e.getLaneNumber(),
                "movements": movement_targets(e),
            })

    entries: list[dict[str, str]] = []
    exits: list[dict[str, str]] = []
    for edge in net.getEdges():
        frm, to = edge.getFromNode().getID(), edge.getToNode().getID()
        kind = "main" if is_main(edge) else "side"
        if frm not in JUNCTIONS:
            through = edge
            while through.getToNode().getID() in JUNCTIONS:
                through = straight_on(through)
            entries.append({"edge": edge.getID(), "kind": kind, "through_exit": through.getID()})
        if to not in JUNCTIONS:
            exits.append({"edge": edge.getID(), "kind": kind})

    return Corridor.model_validate(
        {"junctions": junctions, "approaches": approaches, "entries": entries, "exits": exits}
    )


def build() -> None:
    import sumolib

    net_file = NET_DIR / "corridor.net.xml"
    side = load_json("config/sim.json")["driving_side"]
    if side not in ("left", "right"):
        raise ValueError(f"driving_side must be left or right, not {side!r}")
    subprocess.run([
        sumo_binary("netconvert"),
        "--node-files", str(NET_DIR / "corridor.nod.xml"),
        "--edge-files", str(NET_DIR / "corridor.edg.xml"),
        "--output-file", str(net_file),
        "--no-turnarounds", "true",
        "--offset.disable-normalization", "true",
        "--lefthand", "true" if side == "left" else "false",
    ], check=True)

    corridor = corridor_from_net(sumolib.net.readNet(str(net_file)))
    (NET_DIR / "corridor.json").write_text(corridor.model_dump_json(indent=2) + "\n", encoding="utf-8")
    print(f"wrote {net_file} and corridor.json")


if __name__ == "__main__":
    build()
