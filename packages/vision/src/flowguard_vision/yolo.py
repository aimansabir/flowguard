"""Detector + tracker behind the `Tracker` protocol: Ultralytics YOLO with ByteTrack.

Optional, not a dependency of this package (the rest of FlowGuard never needs it):

    uv pip install ultralytics opencv-python-headless

COCO, which the stock YOLO weights are trained on, has car, motorcycle, bus and
truck but no rickshaw or qingqi (docs/specs/vision.md V1): with stock weights
those are either missed or counted as whatever COCO class they resemble most,
and `classes` should map a fine-tuned model's labels instead. Accuracy on
Karachi footage is not validated.
"""
# pyright: basic
# ultralytics ships without type information.

from collections.abc import Mapping
from typing import Any

import numpy as np

from flowguard_vision.tracking import Frame, TrackObs

COCO = {"car": "car", "motorcycle": "motorcycle", "bus": "bus", "truck": "truck"}


def observations(boxes: Any, names: Mapping[int, str], classes: Mapping[str, str]) -> list[TrackObs]:
    """One tracked box per TrackObs, foot point = bottom-centre of the box.
    Boxes without a track id (ByteTrack not yet confirmed) and labels not in
    `classes` are dropped."""
    if boxes is None or boxes.id is None:
        return []
    out: list[TrackObs] = []
    rows = zip(boxes.xyxy.tolist(), boxes.id.tolist(), boxes.cls.tolist(), boxes.conf.tolist(), strict=True)
    for (x1, _, x2, y2), tid, label, conf in rows:
        cls = classes.get(names[int(label)])
        if cls is not None:
            out.append(TrackObs(int(tid), cls, float(conf), ((x1 + x2) / 2.0, y2)))
    return out


class YoloByteTrack:
    """`Tracker` over Ultralytics' `track()` (YOLO detection + ByteTrack ids)."""

    def __init__(self, model: str = "yolov8n.pt", confidence: float = 0.25,
                 classes: Mapping[str, str] = COCO) -> None:
        from ultralytics import YOLO  # pyright: ignore[reportMissingImports]  # optional

        self._model = YOLO(model)
        self._confidence = confidence
        self._classes = classes

    def update(self, frame: Frame) -> list[TrackObs]:
        # health signals use grayscale; the detector does better on the colour frame (about 15 % more
        # detections on a street clip), so use it when the source has one
        image = frame.bgr if frame.bgr is not None else np.repeat(frame.gray[:, :, None], 3, axis=2)
        result = self._model.track(image, persist=True, tracker="bytetrack.yaml", conf=self._confidence,
                                   verbose=False)[0]
        return observations(result.boxes, result.names, self._classes)
