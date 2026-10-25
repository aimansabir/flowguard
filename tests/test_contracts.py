"""Python side of the contract: generated models match the schema, and
validation rejects what the Rust core would reject."""

import json
from typing import Any

import pytest
from pydantic import BaseModel, ValidationError

import flowguard_core
from flowguard_contracts import SCHEMA_VERSION, VEHICLE_CLASSES, Command, Measurement, models
from flowguard_sim.paths import ROOT

SCHEMA: dict[str, Any] = json.loads((ROOT / "contracts" / "flowguard.schema.json").read_text())


def measurement(**over: Any) -> dict[str, Any]:
    zero = dict.fromkeys(VEHICLE_CLASSES, 0)
    m: dict[str, Any] = {
        "time": 1.0, "interval_s": 1.0, "junction_id": "J1", "approach_id": "W_J1", "source": "camera",
        "zone_length_m": 60.0, "zone_counts": {**zero, "car": 3}, "queue_counts": zero, "arrivals": zero,
        "departures": {"left": zero, "straight": zero, "right": zero},
        "quality": {"detector_confidence": {"mean": 0.8, "std": 0.1}, "track_breaks_per_min": 1.0,
                    "image_health": {"frozen": False, "blur": 0.1, "brightness": 0.5, "displacement_px": 0.0},
                    "count_error": 1.0},
    }
    m.update(over)
    return m


def test_checked_in_schema_matches_compiled_core() -> None:
    assert json.loads(flowguard_core.CONTRACT_SCHEMA) == SCHEMA, "run python scripts/gen_contracts.py"
    assert flowguard_core.SCHEMA_VERSION == SCHEMA_VERSION


@pytest.mark.parametrize("name", sorted(SCHEMA["$defs"]))
def test_models_match_schema(name: str) -> None:
    definition = SCHEMA["$defs"][name]
    if "properties" not in definition:
        return  # enums and unions
    model = getattr(models, name)
    assert issubclass(model, BaseModel)
    assert set(model.model_fields) == set(definition["properties"]), name


def test_round_trip() -> None:
    m = Measurement.model_validate(measurement())
    assert Measurement.model_validate_json(m.model_dump_json()) == m
    assert json.loads(m.model_dump_json())["schema_version"] == SCHEMA_VERSION


@pytest.mark.parametrize("bad", [
    {"zone_counts": {"car": 1}},                                            # missing classes
    {"zone_counts": {**dict.fromkeys(VEHICLE_CLASSES, 0), "tank": 1}},      # unknown class
    {"source": "radar"},
    {"arrivals": {**dict.fromkeys(VEHICLE_CLASSES, 0), "car": -1}},          # negative count
    {"extra_field": 1},
])
def test_rejects_invalid(bad: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        Measurement.model_validate(measurement(**bad))


def test_command_literals() -> None:
    with pytest.raises(ValidationError):
        Command.model_validate({"time": 0, "junction_id": "J", "phase_index": 0, "phase_started_at": 0,
                                "controller_name": "x", "stage": "blue", "target_phase": 0,
                                "requested_phase": None, "intervention": None})
