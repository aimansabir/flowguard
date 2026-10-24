"""Build the placeholder corridor and its corridor.json.

    python -m sim.build_net

Runs netconvert on the hand-written node/edge files, then reads the network
back with sumolib and writes corridor.json: the approaches at each junction,
the two signal phases, the link order of each traffic light, what feeds what,
and where traffic enters and leaves. For the real OSM corridor we need a
.net.xml plus the same corridor.json, see docs/decisions.md.
"""
import json
import subprocess
from pathlib import Path

from sim.sumo_env import sumo_binary

NET_DIR = Path(__file__).parent / "net"
JUNCTIONS = ["J1", "J2", "J3"]
MAIN_ROAD_NODES = set(JUNCTIONS) | {"W", "E"}


def is_main(edge):
    return {edge.getFromNode().getID(), edge.getToNode().getID()} <= MAIN_ROAD_NODES


def straight_on(edge):
    for to_edge, conns in edge.getOutgoing().items():
        if any(c.getDirection() == "s" for c in conns):
            return to_edge
    return None


def corridor_from_net(net):
    out = {"junctions": {}, "downstream": {}, "exit_links": {}, "entries": {}, "exits": {}}

    for jid in JUNCTIONS:
        incoming = [e for e in net.getNode(jid).getIncoming()]
        main = [e.getID() for e in incoming if is_main(e)]
        side = [e.getID() for e in incoming if not is_main(e)]

        links = {}
        for in_lane, out_lane, index in net.getTLS(jid).getConnections():
            direction = next(c.getDirection() for c in in_lane.getOutgoing() if c.getToLane() == out_lane)
            links[index] = [in_lane.getEdge().getID(), direction]

        out["junctions"][jid] = {
            "approaches": main + side,
            "phases": [main, side],
            "links": [links[i] for i in range(len(links))],
        }

    for edge in net.getEdges():
        eid = edge.getID()
        frm, to = edge.getFromNode().getID(), edge.getToNode().getID()
        kind = "main" if is_main(edge) else "side"

        if to in JUNCTIONS:
            nxt = straight_on(edge)
            fed = nxt is not None and nxt.getToNode().getID() in JUNCTIONS
            out["downstream"][eid] = nxt.getID() if fed else None
        if frm in JUNCTIONS and to in JUNCTIONS:
            out["exit_links"][eid] = [frm, to]
        if frm not in JUNCTIONS:
            through = edge
            while through.getToNode().getID() in JUNCTIONS:
                through = straight_on(through)
            out["entries"][eid] = {"kind": kind, "through_exit": through.getID()}
        if to not in JUNCTIONS:
            out["exits"][eid] = kind

    return out


def build():
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
    (NET_DIR / "corridor.json").write_text(json.dumps(corridor, indent=2))
    print(f"wrote {net_file} and corridor.json")


if __name__ == "__main__":
    build()
