"""Messages passed between perception/sim, the Guard, control and the simulator.

Field meanings and units are in docs/schemas.md. If you change what a field
means, or remove one, bump SCHEMA_VERSION and update the doc.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Optional

SCHEMA_VERSION = "1.0"

VEHICLE_CLASSES = ("car", "motorcycle", "rickshaw", "qingqi", "bus", "truck")
SOURCES = ("sim", "camera")
TRUST_STATES = ("TRUSTED", "SUSPECT", "FAULTY")
STAGES = ("green", "yellow", "all_red")


def _check_version(d):
    version = d.get("schema_version", SCHEMA_VERSION)
    if version.split(".")[0] != SCHEMA_VERSION.split(".")[0]:
        raise ValueError(f"schema version {version} not compatible with {SCHEMA_VERSION}")


class _Message:
    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return json.dumps(self.to_dict())

    @classmethod
    def from_json(cls, text):
        return cls.from_dict(json.loads(text))

    @classmethod
    def from_dict(cls, d):
        _check_version(d)
        return cls(**d)


@dataclass
class DetectorConfidence:
    mean: float
    std: float


@dataclass
class ImageHealth:
    frozen: bool
    blur: float
    brightness: float
    displacement: float


@dataclass
class Observation(_Message):
    time: float
    junction_id: str
    approach_id: str
    counts_by_class: dict[str, int]
    queue_pcu: float
    exits_to: dict[str, int]
    source: str
    detector_confidence: Optional[DetectorConfidence] = None
    track_breaks_per_min: Optional[float] = None
    image_health: Optional[ImageHealth] = None
    error_margin: Optional[float] = None
    schema_version: str = SCHEMA_VERSION

    def __post_init__(self):
        if set(self.counts_by_class) != set(VEHICLE_CLASSES):
            raise ValueError(f"counts_by_class must have exactly {VEHICLE_CLASSES}")
        if self.source not in SOURCES:
            raise ValueError(f"bad source {self.source!r}")

    @classmethod
    def from_dict(cls, d):
        _check_version(d)
        d = dict(d)
        if d.get("detector_confidence") is not None:
            d["detector_confidence"] = DetectorConfidence(**d["detector_confidence"])
        if d.get("image_health") is not None:
            d["image_health"] = ImageHealth(**d["image_health"])
        return cls(**d)


@dataclass
class RatedObservation(_Message):
    observation: Observation
    queue_estimate: float
    margin: float
    trust_state: str
    override_flag: bool
    schema_version: str = SCHEMA_VERSION

    def __post_init__(self):
        if self.trust_state not in TRUST_STATES:
            raise ValueError(f"bad trust_state {self.trust_state!r}")

    @property
    def approach_id(self):
        return self.observation.approach_id

    @classmethod
    def from_dict(cls, d):
        _check_version(d)
        d = dict(d)
        d["observation"] = Observation.from_dict(d["observation"])
        return cls(**d)


@dataclass
class Command(_Message):
    time: float
    junction_id: str
    phase_index: int
    phase_started_at: float
    controller_name: str
    stage: str = "green"
    target_phase: Optional[int] = None
    requested_phase: Optional[int] = None
    intervention: Optional[str] = None
    schema_version: str = SCHEMA_VERSION

    def __post_init__(self):
        if self.stage not in STAGES:
            raise ValueError(f"bad stage {self.stage!r}")
        if self.target_phase is None:
            self.target_phase = self.phase_index
