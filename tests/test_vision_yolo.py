"""The YOLO + ByteTrack adapter without the model: tracked boxes -> TrackObs.
Real detection needs ultralytics and footage, neither of which the tests use."""

from typing import Any

from flowguard_vision.yolo import COCO, observations


class Tensor:
    def __init__(self, rows: list[Any]) -> None:
        self.rows = rows

    def tolist(self) -> list[Any]:
        return self.rows


class Boxes:
    def __init__(self, xyxy: list[list[float]], ids: list[int] | None, cls: list[int],
                 conf: list[float]) -> None:
        self.xyxy, self.cls, self.conf = Tensor(xyxy), Tensor(cls), Tensor(conf)
        self.id = None if ids is None else Tensor(ids)


NAMES = {0: "person", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


def test_tracked_boxes_become_foot_points() -> None:
    boxes = Boxes([[10, 20, 30, 60], [100, 0, 140, 50], [0, 0, 5, 5]], [7, 8, 9], [2, 3, 0], [0.9, 0.6, 0.8])
    out = observations(boxes, NAMES, COCO)
    assert [(o.track_id, o.cls, o.foot) for o in out] == [(7, "car", (20.0, 60)),
                                                          (8, "motorcycle", (120.0, 50))]
    assert out[0].confidence == 0.9  # the person (class 0) is not a vehicle class


def test_untracked_boxes_are_dropped() -> None:
    assert observations(Boxes([[0, 0, 1, 1]], None, [2], [0.9]), NAMES, COCO) == []
    assert observations(None, NAMES, COCO) == []


def test_fine_tuned_labels_can_map_to_karachi_classes() -> None:
    names = {0: "auto_rickshaw", 1: "chingchi"}
    boxes = Boxes([[0, 0, 2, 4], [10, 0, 12, 4]], [1, 2], [0, 1], [0.7, 0.7])
    out = observations(boxes, names, {"auto_rickshaw": "rickshaw", "chingchi": "qingqi"})
    assert [o.cls for o in out] == ["rickshaw", "qingqi"]


def test_modules_import_without_the_ml_dependencies() -> None:
    import importlib

    for name in ("flowguard_vision.video", "flowguard_vision.yolo"):
        assert importlib.import_module(name)
