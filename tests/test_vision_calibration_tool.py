"""The calibration tool's builder (packages/vision/tools/calibrate.js) writes files the pipeline accepts."""

import json
import shutil
import subprocess
from pathlib import Path
from typing import Any

import pytest

from flowguard_vision.calibration import Calibration

TOOL = Path(__file__).parents[1] / "packages" / "vision" / "tools" / "calibrate.js"
pytestmark = pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")

SQUARE = [[100.04, 200], [300, 200], [300, 400], [100, 400]]


def state(**over: Any) -> dict[str, Any]:
    s: dict[str, Any] = {
        "junction_id": "J1", "approach_id": "W_J1", "zone_length_m": 60, "reference_frame": "ref.png",
        "zone": SQUARE, "entry": [[100, 200], [300, 200]], "stop": [[100, 400], [300, 400]],
        "movements": {"left": [], "straight": SQUARE, "right": []},
        "pairs": [{"px": p, "m": [i * 3.5, i * 10.0]} for i, p in enumerate(SQUARE)],
    }
    return {**s, **over}


def build(s: dict[str, Any]) -> dict[str, Any]:
    js = (f"const t=require({json.dumps(TOOL.as_posix())});"
          f"console.log(JSON.stringify(t.buildCalibration({json.dumps(s)})))")
    out = subprocess.run(["node", "-e", js], capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def test_built_file_is_accepted_by_the_pipeline_model() -> None:
    res = build(state())
    assert res["errors"] == []
    calib = Calibration.model_validate(res["calibration"])
    assert calib.zone_polygon[0] == (100.0, 200.0)  # pixels rounded to 0.1
    assert set(calib.movement_regions) == {"straight"}  # empty regions are left out


@pytest.mark.parametrize(
    "over,word",
    [
        ({"zone": SQUARE[:2]}, "zone"),
        ({"entry": [[0, 0]]}, "entry"),
        ({"stop": []}, "stop"),
        ({"zone_length_m": 0}, "length"),
        ({"approach_id": " "}, "Approach"),
        ({"movements": {"left": SQUARE[:2]}}, "left"),
        ({"pairs": []}, "4 point pairs"),
        ({"pairs": [{"px": p, "m": [1, 1]} for p in SQUARE]}, "differ"),
        ({"pairs": [{"px": p, "m": [None, 1]} for p in SQUARE]}, "numeric"),
    ],
)
def test_incomplete_drawings_are_reported_not_built(over: dict[str, Any], word: str) -> None:
    res = build(state(**over))
    assert res["calibration"] is None
    assert any(word.lower() in e.lower() for e in res["errors"]), res["errors"]
