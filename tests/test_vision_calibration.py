from typing import Any

import pytest
from pydantic import ValidationError

from flowguard_vision.calibration import Calibration

SQUARE = [(0, 0), (10, 0), (10, 10), (0, 10)]


def good() -> dict[str, Any]:
    return {
        "junction_id": "A",
        "approach_id": "a_w",
        "zone_polygon": SQUARE,
        "zone_length_m": 60,
        "entry_line": [(0, 0), (10, 0)],
        "stop_line": [(0, 10), (10, 10)],
        "movement_regions": {"left": SQUARE, "straight": SQUARE},
        "reference_frame": "ref.png",
        "px_to_m": [(p, (p[0] * 0.1, p[1] * 0.1)) for p in SQUARE],
    }


def test_valid_calibration_round_trips() -> None:
    c = Calibration.model_validate(good())
    assert Calibration.model_validate_json(c.model_dump_json()) == c


@pytest.mark.parametrize(
    "key,value",
    [
        ("zone_polygon", [(0, 0), (1, 1)]),
        ("zone_length_m", 0),
        ("stop_line", [(0, 0)]),
        ("px_to_m", [((0, 0), (0, 0))] * 3),
        ("movement_regions", {"uturn": SQUARE}),
        ("extra_field", 1),
    ],
)
def test_bad_calibration_rejected(key: str, value: Any) -> None:
    with pytest.raises(ValidationError):
        Calibration.model_validate({**good(), key: value})
