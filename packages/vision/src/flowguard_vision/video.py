"""Recorded video -> one Measurement per second -> JSONL (docs/specs/vision.md).

    uv pip install ultralytics opencv-python-headless
    uv run python -m flowguard_vision.video --video clip.mp4 --calib calib/W_J1.json --out W_J1.jsonl

The calibration file names the approach, its zone and lines and the reference
frame (relative to the calibration file). Detection and tracking are
`yolo.YoloByteTrack`; everything after it (aggregation, image health,
Measurement) is the tested pipeline in run.py. Not validated on real footage.
"""
# pyright: basic
# opencv ships without complete type information.

import argparse
from collections.abc import Iterator
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from flowguard_vision.calibration import load_calibration
from flowguard_vision.run import measurements, write_jsonl
from flowguard_vision.tracking import Frame


def frames_from_video(path: Path, every_s: float = 0.0) -> Iterator[Frame]:
    """Grayscale frames with their time in the clip; `every_s` > 0 skips frames
    so they are at least that far apart (the tracker needs a few per second)."""
    import cv2  # pyright: ignore[reportMissingImports]  # optional

    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    index, last = 0, -1e9
    try:
        while True:
            ok, bgr = cap.read()
            if not ok:
                return
            t = index / fps
            index += 1
            if t - last >= every_s:
                last = t
                yield Frame(t, cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    finally:
        cap.release()


def reference_frame(calib_path: Path, name: str) -> NDArray[np.uint8]:
    import cv2  # pyright: ignore[reportMissingImports]  # optional

    image = cv2.imread(str(calib_path.parent / name), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"reference frame {name} not found next to {calib_path}")
    return image


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--calib", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--model", default="yolov8n.pt", help="Ultralytics weights")
    ap.add_argument("--every", type=float, default=0.1, help="seconds between processed frames")
    args = ap.parse_args()

    from flowguard_vision.yolo import YoloByteTrack

    calib = load_calibration(args.calib)
    ref = reference_frame(args.calib, calib.reference_frame)
    frames = frames_from_video(args.video, args.every)
    write_jsonl(measurements(frames, YoloByteTrack(args.model), calib, ref), args.out)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
