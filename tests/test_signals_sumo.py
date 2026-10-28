"""Signal permissions as SUMO itself sees them, through a real run
(decisions.md D29): the link order SUMO controls is the corridor map's, and
every state the applier sets, permissive crossing turns included, is the
state SUMO then shows."""
# pyright: basic
# traci ships without type information.

from pathlib import Path
from typing import Any

import pytest
import traci

from flowguard_sim import signals as signals_module
from flowguard_sim.run import run


def test_sumo_applies_the_movement_states(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    applied: list[str] = []
    original = signals_module.SignalApplier.apply

    def apply(self: Any, commands: Any) -> None:
        original(self, commands)
        for jid, state in self.shown.items():
            assert traci.trafficlight.getRedYellowGreenState(jid) == state
            links = traci.trafficlight.getControlledLinks(jid)
            controlled = [link[0][0].rsplit("_", 1)[0] for link in links]
            assert controlled == [link.approach for link in self.junctions[jid].links]
            applied.append(state)

    monkeypatch.setattr(signals_module.SignalApplier, "apply", apply)
    run("fixed", 120, 1, results_dir=tmp_path)
    assert any("g" in s for s in applied)  # permissive crossing turns reached SUMO
    assert any("y" in s for s in applied) and any(set(s) == {"r"} for s in applied)  # yellow, all-red
