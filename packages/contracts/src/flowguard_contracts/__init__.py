"""FlowGuard message contracts.

`models.py` is generated from contracts/flowguard.schema.json, which is
generated from the Rust types in crates/core/src/contracts.rs. Edit the Rust
types, then run `python scripts/gen_contracts.py`. Field meanings: docs/contracts.md.
"""

from flowguard_contracts.models import (
    Approach,
    ApproachState,
    ClassCounts,
    Command,
    CoreConfig,
    Corridor,
    DetectorConfidence,
    ImageHealth,
    Junction,
    Measurement,
    MovementCounts,
    Quality,
    StepOutput,
)

SCHEMA_VERSION = "2.0"
VEHICLE_CLASSES = ("car", "motorcycle", "rickshaw", "qingqi", "bus", "truck")
MOVEMENTS = ("left", "straight", "right")

__all__ = [
    "MOVEMENTS",
    "SCHEMA_VERSION",
    "VEHICLE_CLASSES",
    "Approach",
    "ApproachState",
    "ClassCounts",
    "Command",
    "CoreConfig",
    "Corridor",
    "DetectorConfidence",
    "ImageHealth",
    "Junction",
    "Measurement",
    "MovementCounts",
    "Quality",
    "StepOutput",
]
