"""Live dashboard: watch the closed loop decide, from SUMO or from recorded data.

    uv run python -m flowguard_sim.live --controller maxpressure_hyst --seconds 1800 --open
    uv run python -m flowguard_sim.live --replay results/<run_dir> --controller maxpressure --open

Sources:
  live    SUMO steps, the observer builds Measurements, the core decides, signals go back.
  replay  Measurements recorded in an earlier run (measurements.jsonl) are fed to the core
          under whatever controller you pick. Open loop: the recorded traffic does not react
          to the new signals. If commands.jsonl sits next to it, the recorded decisions are
          shown beside the new ones.

The server pushes one frame per simulated second over server-sent events (stdlib only).
The page keeps every frame, so you can scrub back while the run continues.
"""
# pyright: basic
# traci ships without type information.

import argparse
import json
import threading
import time
import webbrowser
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from flowguard_contracts import Command, Corridor, Measurement
from flowguard_core import Core
from flowguard_sim.corridor import load_corridor
from flowguard_sim.paths import ROOT, load_json, without_notes
from flowguard_sim.run import CONTROLLERS, StepEvent, core_config, run

CLASSES = ("car", "motorcycle", "rickshaw", "qingqi", "bus", "truck")
TRUST = ("TRUSTED", "SUSPECT", "FAULTY")
PAGE = Path(__file__).with_name("live.html")


class Hub:
    """Everything the HTTP threads share with the simulation thread."""

    def __init__(self) -> None:
        self.msgs: list[tuple[str, str]] = []  # (event, json); index 0 is "init"
        self.cond = threading.Condition()
        self.paused = False
        self.speed = 4.0
        self.client_seen = threading.Event()
        self.done = False

    def publish(self, event: str, payload: Any) -> None:
        with self.cond:
            self.msgs.append((event, json.dumps(payload, separators=(",", ":"))))
            self.done = event == "done"
            self.cond.notify_all()

    def pace(self) -> None:
        """Sleep so one simulated second takes 1/speed wall seconds; hold while paused."""
        start = time.monotonic()
        while True:
            if self.paused:
                time.sleep(0.05)
                start = time.monotonic()
            elif time.monotonic() - start >= 1.0 / self.speed:
                return
            else:
                time.sleep(0.005)


def make_handler(hub: Hub) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_: Any) -> None:
            pass

        def do_GET(self) -> None:
            if self.path == "/events":
                self._events()
            else:
                body = PAGE.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        def do_POST(self) -> None:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if "paused" in body:
                hub.paused = bool(body["paused"])
            if "speed" in body:
                hub.speed = max(0.1, min(1000.0, float(body["speed"])))
            self.send_response(204)
            self.end_headers()

        def _events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            hub.client_seen.set()
            i = 0
            try:
                while True:
                    with hub.cond:
                        hub.cond.wait_for(lambda n=i: len(hub.msgs) > n, timeout=10)
                        batch, i = hub.msgs[i:], len(hub.msgs)
                    out = "".join(f"event: {e}\ndata: {d}\n\n" for e, d in batch) or ": keepalive\n\n"
                    self.wfile.write(out.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionError, OSError):
                return

    return Handler


# ---------- static layout sent once ----------

def network_geometry() -> dict[str, Any]:
    """Lane polylines and bounds from the SUMO net file (net coordinates, y up)."""
    sim_cfg = load_json("config/sim.json")
    root = ET.parse(ROOT / sim_cfg["net_file"]).getroot()
    loc = root.find("location")
    assert loc is not None
    edges = []
    for e in root.iter("edge"):
        lanes = [[[float(v) for v in p.split(",")] for p in lane.attrib["shape"].split()]
                 for lane in e.iter("lane")]
        edges.append({"id": e.attrib["id"], "internal": e.attrib.get("function") == "internal",
                      "lanes": lanes, "width": float(next(e.iter("lane")).attrib.get("width", 3.2))})
    return {"bounds": [float(v) for v in loc.attrib["convBoundary"].split(",")], "edges": edges}


def downstream(corridor: Corridor) -> dict[str, str | None]:
    """Approach -> the approach its straight-through traffic joins (None = leaves the corridor).
    Same definition as the core's max-pressure (crates/core/src/corridor.rs)."""
    ids = {a.id for a in corridor.approaches}
    return {a.id: a.movements.straight if a.movements.straight in ids else None for a in corridor.approaches}


def layout(corridor: Corridor, controller: str, mode: str, label: str, speed: float) -> dict[str, Any]:
    cap = load_json("config/maxpressure.json")["capacity"]
    down = downstream(corridor)
    approaches = [{
        "id": a.id, "junction": a.junction, "length": a.length_m, "lanes": a.lanes, "down": down[a.id],
        "storage": a.length_m * a.lanes / cap["jam_spacing_m"],
    } for a in corridor.approaches]
    cfg = core_config(controller).model_dump()
    return {
        "mode": mode, "source": label, "speed": speed, "controller": controller,
        "guard": cfg["guard"]["kind"],
        "timing": without_notes(load_json("config/signal.json")), "spillback": cap["spillback_occupancy"],
        "classes": CLASSES, "net": network_geometry(),
        "junctions": [{"id": j.id, "phases": j.phases} for j in corridor.junctions],
        "approaches": approaches,
    }


# ---------- per-second frame ----------

def frame(ev: StepEvent, down: dict[str, str | None], vehicles: list[list[float]] | None,
          recorded: dict[str, Command] | None) -> dict[str, Any]:
    truth = {m.approach_id: m for m in ev.truth}
    states = {s.approach_id: s for s in ev.step.states}
    q = {a: s.queue_estimate for a, s in states.items()}

    def term(a: str) -> float:
        return q[a] - q.get(down[a] or "", 0.0)

    app = {}
    for a, s in states.items():
        app[a] = {
            "q": round(s.measured_queue_pcu, 2), "est": round(s.queue_estimate, 2), "m": round(s.margin, 2),
            "tr": TRUST.index(s.trust_state), "ov": s.override_flag, "arr": round(s.arrivals_pcu, 2),
            "dep": round(s.departures_pcu, 2), "term": round(term(a), 2),
            "tq": round(ev.pcu(truth[a].queue_counts), 2) if a in truth else None,
            "ex": {j: n for j, n in s.exits_to.items() if n},
        }
    jn = {}
    for c in ev.step.commands:
        jn[c.junction_id] = {
            "ph": c.phase_index, "tg": c.target_phase, "st": c.stage, "since": c.phase_started_at,
            "rq": c.requested_phase if c.requested_phase is not None else ev.step.requests.get(c.junction_id),
            "iv": c.intervention,
            "rec": None if recorded is None or c.junction_id not in recorded else
            [recorded[c.junction_id].phase_index, recorded[c.junction_id].stage],
        }
    return {"t": ev.t, "app": app, "jn": jn, "veh": vehicles}


def vehicle_snapshot() -> list[list[float]]:
    import traci
    from traci import constants as tc

    out = []
    for v in traci.vehicle.getAllSubscriptionResults().values():
        x, y = v[tc.VAR_POSITION]
        out.append([round(x, 1), round(y, 1), round(v[tc.VAR_ANGLE]), CLASSES.index(v[tc.VAR_TYPE]),
                    round(v[tc.VAR_SPEED], 1)])
    return out


# ---------- sources ----------

def serve_live(hub: Hub, corridor: Corridor, controller: str, seconds: int, seed: int) -> None:
    down = downstream(corridor)
    hub.publish("init", layout(corridor, controller, "live", f"SUMO, seed {seed}", hub.speed))
    hub.client_seen.wait(timeout=120)

    def on_step(ev: StepEvent) -> None:
        hub.publish("frame", frame(ev, down, vehicle_snapshot(), None))
        hub.pace()

    summary = run(controller, seconds, seed, on_step=on_step)
    hub.publish("done", summary)


def read_run(path: Path) -> tuple[dict[float, list[Measurement]], dict[float, list[Measurement]],
                                  dict[float, dict[str, Command]]]:
    """Group a past run's logs by time. truth and commands are optional (empty if absent)."""
    def group(file: Path, model: Any) -> dict[float, list[Any]]:
        out: dict[float, list[Any]] = {}
        if file.is_file():
            for line in file.read_text(encoding="utf-8").splitlines():
                rec = model.model_validate_json(line)
                out.setdefault(rec.time, []).append(rec)
        return out

    cmds = {t: {c.junction_id: c for c in cs} for t, cs in group(path / "commands.jsonl", Command).items()}
    return group(path / "measurements.jsonl", Measurement), group(path / "truth.jsonl", Measurement), cmds


def serve_replay(hub: Hub, corridor: Corridor, controller: str, path: Path) -> None:
    seen, truth, cmds = read_run(path)
    if not seen:
        raise SystemExit(f"no measurements.jsonl in {path}")
    down = downstream(corridor)
    core = Core((ROOT / load_json("config/sim.json")["corridor_file"]).read_text(),
                core_config(controller).model_dump_json())
    hub.publish("init", layout(corridor, controller, "replay", f"recorded run {path.name}", hub.speed))
    hub.client_seen.wait(timeout=120)

    from flowguard_contracts import ClassCounts, StepOutput

    def pcu(c: ClassCounts) -> float:
        return core.pcu(c.model_dump_json())

    for t in sorted(seen):
        batch = "[" + ",".join(m.model_dump_json() for m in seen[t]) + "]"
        step = StepOutput.model_validate_json(core.step(t, batch))
        event = StepEvent(t, seen[t], truth.get(t, []), step, pcu)
        hub.publish("frame", frame(event, down, None, cmds.get(t)))
        hub.pace()
    hub.publish("done", {"controller": controller, "seconds": len(seen),
                         "interventions": json.loads(core.interventions_json())})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controller", choices=CONTROLLERS, default="maxpressure_hyst")
    ap.add_argument("--seconds", type=int, default=1800)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--speed", type=float, default=4.0, help="simulated seconds per wall second")
    ap.add_argument("--replay", type=Path, help="results/<run> directory to use as the data source")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--open", action="store_true")
    args = ap.parse_args()

    corridor = load_corridor(ROOT / load_json("config/sim.json")["corridor_file"])
    hub = Hub()
    hub.speed = args.speed
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(hub))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{args.port}/"
    print(f"dashboard at {url}  (Ctrl-C to stop)")
    if args.open:
        webbrowser.open(url)
    try:
        if args.replay:
            serve_replay(hub, corridor, args.controller, args.replay)
        else:
            serve_live(hub, corridor, args.controller, args.seconds, args.seed)
        print("run finished; the page keeps the history. Ctrl-C to stop the server.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
