"""Build the placeholder corridor and its corridor.json.

    uv run python -m flowguard_sim.build_net

Runs netconvert on the hand-written node/edge files, then reads the network
back with sumolib and writes corridor.json (contract type `Corridor`): the
junctions with their two phases and signal link order, every approach with
its length, lanes and where each turn leads, and where traffic enters and
leaves. For the real OSM corridor the phases have to be chosen by hand; this
gives a starting point (decisions.md D6).
"""
# pyright: basic
# sumolib ships without type information.

import subprocess
from typing import Any

from flowguard_contracts import Corridor
from flowguard_sim.paths import ROOT
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
            "phases": [[e.getID() for e in main], [e.getID() for e in side]],
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
    subprocess.run([
        sumo_binary("netconvert"),
        "--node-files", str(NET_DIR / "corridor.nod.xml"),
        "--edge-files", str(NET_DIR / "corridor.edg.xml"),
        "--output-file", str(net_file),
        "--no-turnarounds", "true",
        "--offset.disable-normalization", "true",
    ], check=True)

    corridor = corridor_from_net(sumolib.net.readNet(str(net_file)))
    (NET_DIR / "corridor.json").write_text(corridor.model_dump_json(indent=2) + "\n", encoding="utf-8")
    print(f"wrote {net_file} and corridor.json")


if __name__ == "__main__":
    build()
