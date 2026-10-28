"""Per-camera calibration file, `calib/<approach_id>.json` (docs/specs/vision.md).

Image coordinates are pixels, ground coordinates are metres.
"""

from pathlib import Path
from typing import Annotated, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator

Point = tuple[float, float]
Polygon = Annotated[list[Point], Field(min_length=3)]
Line = Annotated[list[Point], Field(min_length=2, max_length=2)]


class Calibration(BaseModel):
    model_config = ConfigDict(extra="forbid")

    junction_id: str
    approach_id: str
    zone_polygon: Polygon
    zone_length_m: float = Field(gt=0)
    entry_line: Line
    stop_line: Line
    # polygons a departing vehicle ends up in, keyed by movement
    movement_regions: dict[str, Polygon]
    reference_frame: str
    # four (image px, ground m) pairs, enough to fix a homography
    px_to_m: Annotated[list[tuple[Point, Point]], Field(min_length=4, max_length=4)]

    @model_validator(mode="after")
    def _movements_are_known(self) -> Self:
        unknown = set(self.movement_regions) - {"left", "straight", "right"}
        if unknown:
            raise ValueError(f"unknown movements {sorted(unknown)}")
        return self


def load_calibration(path: Path) -> Calibration:
    return Calibration.model_validate_json(path.read_text(encoding="utf-8"))
