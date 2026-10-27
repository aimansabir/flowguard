"""Placeholder producer: a valid camera Measurement with nothing seen.

Replace with the real pipeline (docs/specs/vision.md). Kept small on purpose:
its job is to pin down the output format the pipeline must produce.
"""

from flowguard_contracts import VEHICLE_CLASSES, Measurement


def empty_measurement(time: float, junction_id: str, approach_id: str, zone_length_m: float) -> Measurement:
    zero = dict.fromkeys(VEHICLE_CLASSES, 0)
    return Measurement.model_validate({
        "time": time,
        "interval_s": 1.0,
        "junction_id": junction_id,
        "approach_id": approach_id,
        "source": "camera",
        "zone_length_m": zone_length_m,
        "zone_counts": zero,
        "queue_counts": zero,
        "arrivals": zero,
        "departures": {"left": zero, "straight": zero, "right": zero},
    })
