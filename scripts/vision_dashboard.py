"""Vision demo: synthetic traffic -> real Aggregator -> one self-contained HTML dashboard.

    uv run python scripts/vision_dashboard.py [--open]

There is no footage yet, so a seeded toy simulation plays the road and the camera
(with detector noise, track breaks and four camera faults). Everything the dashboard
shows as "camera" comes out of flowguard_vision.Aggregator and health.image_health,
the same code real footage will go through. The grey "truth" is what the toy sim knows.
Output: results/vision_dashboard.html (gitignored).
"""

import argparse
import json
import os
import random
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from flowguard_contracts import VEHICLE_CLASSES
from flowguard_vision.aggregate import Aggregator
from flowguard_vision.calibration import Calibration
from flowguard_vision.health import image_health
from flowguard_vision.tracking import TrackObs

FPS, DT, DUR = 10, 0.1, 200
STOP_X = 60.0  # metres; zone is 0..60, ground = px / 10
LANES = {"left": 15.0, "straight": 50.0, "right": 85.0}  # y in px
LENGTH = {"car": 4.5, "motorcycle": 2.0, "rickshaw": 3.0, "qingqi": 3.0, "bus": 10.0, "truck": 8.0}
VFREE = {"car": 9.0, "motorcycle": 10.0, "rickshaw": 6.0, "qingqi": 6.5, "bus": 7.0, "truck": 7.0}
MIX = [0.55, 0.20, 0.10, 0.05, 0.04, 0.06]  # in VEHICLE_CLASSES order
RED_S, GREEN_S = 25.0, 15.0
EPISODES = [  # camera faults, to show what each health signal does
    (120.0, 135.0, "frozen", "Camera frozen: frames and tracks repeat"),
    (150.0, 165.0, "dark", "Night / dark: brightness and detector confidence fall"),
    (165.0, 180.0, "blur", "Blur: sharpness lost, confidence falls"),
    (180.0, 195.0, "shifted", "Camera knocked: scene shifts against the reference"),
]


def episode(t: float) -> str | None:
    return next((k for a, b, k, _ in EPISODES if a <= t < b), None)


def is_green(t: float) -> bool:
    return t % (RED_S + GREEN_S) >= RED_S


def calibration() -> Calibration:
    corners = [(0.0, 0.0), (700.0, 0.0), (700.0, 100.0), (0.0, 100.0)]
    return Calibration.model_validate({
        "junction_id": "J1", "approach_id": "W_J1",
        "zone_polygon": [(0, 0), (600, 0), (600, 100), (0, 100)], "zone_length_m": 60,
        "entry_line": [(0, 0), (0, 100)], "stop_line": [(600, 0), (600, 100)],
        "movement_regions": {
            "left": [(600, 0), (700, 0), (700, 33), (600, 33)],
            "straight": [(600, 33), (700, 33), (700, 66), (600, 66)],
            "right": [(600, 66), (700, 66), (700, 100), (600, 100)],
        },
        "reference_frame": "ref.png",
        "px_to_m": [(p, (p[0] / 10, p[1] / 10)) for p in corners],
    })


class Veh:
    def __init__(self, vid: int, cls: str, lane: str):
        self.vid, self.cls, self.lane = vid, cls, lane
        self.x, self.v = 0.6, VFREE[cls] * 0.8
        self.track = vid
        self.dropout_until = 0.0
        self.crossed = False


def step_traffic(vehs: list[Veh], t: float) -> None:
    for lane in LANES:
        line = sorted((v for v in vehs if v.lane == lane), key=lambda v: -v.x)
        leader: Veh | None = None
        for v in line:
            gap = 1e9 if leader is None else leader.x - LENGTH[leader.cls] - v.x
            if not is_green(t) and v.x < STOP_X - 0.5:
                gap = min(gap, STOP_X - 1.0 - v.x)
            vdes = min(VFREE[v.cls], max(0.0, (gap - 2.0) / 1.2))
            v.v = max(0.0, v.v + max(-6.0, min(2.5, (vdes - v.v) / 0.5)) * DT)
            v.x += v.v * DT
            leader = v


def gray_frame(base: NDArray[np.uint8], rng: np.random.Generator, kind: str | None) -> NDArray[np.uint8]:
    img: NDArray[np.float64] = base.astype(np.float64)
    if kind == "blur":
        img = np.mean([np.roll(img, (dy, dx), (0, 1)) for dy in range(-3, 4) for dx in range(-3, 4)], axis=0)
    if kind == "shifted":
        img = np.roll(img, (5, 8), (0, 1))
    img = img + rng.normal(0, 2, base.shape)
    if kind == "dark":
        img = img * 0.3
    return np.clip(img, 0, 255).astype(np.uint8)


def simulate(seed: int = 7) -> dict[str, object]:
    rnd, nprng = random.Random(seed), np.random.default_rng(seed)
    base = nprng.integers(0, 256, (48, 84), dtype=np.uint8)
    ref = gray_frame(base, nprng, None)
    agg = Aggregator(calibration())
    vehs: list[Veh] = []
    nxt_id, nxt_track = 1, 1000
    frames: list[list[list[float]]] = []
    truth: list[list[list[float]]] = []
    ms: list[dict[str, object]] = []
    truth_sec: list[dict[str, int]] = []
    sec = {"arr": 0, "dep": 0}
    prev: NDArray[np.uint8] | None = None
    frozen_obs: list[TrackObs] = []
    frozen_gray: NDArray[np.uint8] | None = None

    for k in range(DUR * FPS + 1):
        t = round(k * DT, 1)
        if k:
            step_traffic(vehs, t)
            if rnd.random() < 0.40 * DT:
                lane = rnd.choices(list(LANES), [0.25, 0.5, 0.25])[0]
                cls = rnd.choices(VEHICLE_CLASSES, MIX)[0]
                tail = min((v.x - LENGTH[v.cls] for v in vehs if v.lane == lane), default=99.0)
                if tail > 3.0:
                    vehs.append(Veh(nxt_id, cls, lane))
                    nxt_id += 1
                    sec["arr"] += 1
        for v in vehs:
            if v.x >= STOP_X and not v.crossed:
                v.crossed = True
                sec["dep"] += 1
        vehs = [v for v in vehs if v.x < 72]

        kind = episode(t)
        obs: list[TrackObs] = []
        for v in vehs:
            if t < v.dropout_until:
                continue
            if rnd.random() < (0.02 if kind == "dark" else 0.0006):
                v.dropout_until = t + rnd.uniform(0.3, 0.9)
                v.track = nxt_track  # tracker restarts under a new id afterwards
                nxt_track += 1
                continue
            base_conf = {"dark": 0.55, "blur": 0.6}.get(kind or "", 0.87)
            cls = v.cls if rnd.random() > 0.03 else rnd.choice(VEHICLE_CLASSES)
            dx, dy = (8, 5) if kind == "shifted" else (0, 0)
            obs.append(TrackObs(
                v.track, cls, min(0.99, max(0.05, rnd.gauss(base_conf, 0.07))),
                (v.x * 10 + dx + rnd.gauss(0, 0.3), LANES[v.lane] + dy + rnd.gauss(0, 0.3)),
            ))
        gray = gray_frame(base, nprng, kind)
        if kind == "frozen":
            if frozen_gray is None:
                frozen_obs, frozen_gray = obs, gray
            obs, gray = frozen_obs, frozen_gray
        else:
            frozen_gray = None
        health = image_health(prev, gray, ref)
        prev = gray

        frames.append([[o.track_id, VEHICLE_CLASSES.index(o.cls), round(o.foot[0], 1), round(o.foot[1], 1),
                        round(o.confidence, 2)] for o in obs])
        truth.append([[round(v.x * 10, 1), LANES[v.lane], VEHICLE_CLASSES.index(v.cls), LENGTH[v.cls]]
                      for v in vehs])
        for m in agg.update(t, obs, health):
            inzone = [v for v in vehs if 0 <= v.x < STOP_X]
            truth_sec.append({"zone": len(inzone), "queue": sum(v.v < 0.1 for v in inzone),
                              "arr": sec["arr"], "dep": sec["dep"]})
            sec = {"arr": 0, "dep": 0}
            ms.append(json.loads(m.model_dump_json()))
    return {
        "fps": FPS, "dur": DUR, "classes": list(VEHICLE_CLASSES), "lanes": list(LANES.values()),
        "cycle": [RED_S, GREEN_S],
        "episodes": [{"t0": a, "t1": b, "kind": k, "label": s} for a, b, k, s in EPISODES],
        "frames": frames, "truth": truth, "ms": ms, "truth_sec": truth_sec,
        "unresolved": agg.unresolved_departures,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--open", action="store_true", help="open the page in the default browser")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    root = Path(__file__).resolve().parent.parent
    data = json.dumps(simulate(args.seed), separators=(",", ":"))
    html = (Path(__file__).with_name("vision_dashboard.html")).read_text(encoding="utf-8")
    out = root / "results" / "vision_dashboard.html"
    out.parent.mkdir(exist_ok=True)
    out.write_text(html.replace("/*DATA*/null", data), encoding="utf-8")
    print(out)
    if args.open:
        os.startfile(out)  # type: ignore[attr-defined]  # Windows


if __name__ == "__main__":
    main()
