"""Whatever vision produces must be accepted by the core unchanged."""

import json
from pathlib import Path

from flowguard_contracts import Corridor, StepOutput
from flowguard_core import Core
from flowguard_sim.run import core_config
from flowguard_vision.stub import empty_measurement


def test_stub_measurements_feed_the_core(corridor: Corridor, corridor_path: Path) -> None:
    core = Core(corridor_path.read_text(), core_config("maxpressure").model_dump_json())
    ms = [empty_measurement(0.0, a.junction, a.id, 60.0) for a in corridor.approaches]
    batch = "[" + ",".join(m.model_dump_json() for m in ms) + "]"
    out = StepOutput.model_validate_json(core.step(0.0, batch))
    assert len(out.states) == len(corridor.approaches)
    assert all(s.queue_estimate == 0.0 and s.trust_state == "TRUSTED" for s in out.states)
    assert json.loads(ms[0].model_dump_json())["source"] == "camera"
