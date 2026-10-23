import json

import pytest

from schemas.messages import (SCHEMA_VERSION, Command, DetectorConfidence, ImageHealth, Observation,
                              RatedObservation)

COUNTS = {"car": 3, "motorcycle": 5, "rickshaw": 1, "qingqi": 0, "bus": 1, "truck": 0}


def sim_obs():
    return Observation(time=12.0, junction_id="J1", approach_id="W_J1", counts_by_class=COUNTS,
                       queue_pcu=6.5, exits_to={"J2": 2}, source="sim")


def camera_obs():
    return Observation(time=12.0, junction_id="J1", approach_id="W_J1", counts_by_class=COUNTS,
                       queue_pcu=6.5, exits_to={"J2": 2}, source="camera",
                       detector_confidence=DetectorConfidence(mean=0.81, std=0.07),
                       track_breaks_per_min=1.5,
                       image_health=ImageHealth(frozen=False, blur=0.1, brightness=0.6, displacement=0.0),
                       error_margin=1.2)


@pytest.mark.parametrize("msg", [
    sim_obs(),
    camera_obs(),
    RatedObservation(observation=camera_obs(), queue_estimate=6.0, margin=1.0,
                     trust_state="SUSPECT", override_flag=True),
    Command(time=12.0, junction_id="J1", phase_index=1, phase_started_at=5.0, controller_name="fixed",
            stage="yellow", target_phase=0, requested_phase=0),
])
def test_round_trip(msg):
    text = msg.to_json()
    assert json.loads(text)["schema_version"] == SCHEMA_VERSION
    assert type(msg).from_json(text) == msg


def test_sim_observation_leaves_camera_fields_null():
    d = json.loads(sim_obs().to_json())
    assert d["detector_confidence"] is None
    assert d["track_breaks_per_min"] is None
    assert d["image_health"] is None


def test_rejects_missing_vehicle_class():
    with pytest.raises(ValueError):
        Observation(time=0, junction_id="J1", approach_id="W_J1", counts_by_class={"car": 1},
                    queue_pcu=0, exits_to={}, source="sim")


def test_rejects_other_major_version():
    d = sim_obs().to_dict()
    d["schema_version"] = "2.0"
    with pytest.raises(ValueError):
        Observation.from_dict(d)
