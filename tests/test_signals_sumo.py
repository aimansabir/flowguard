"""Signal permissions as SUMO itself sees them, through a real run
(decisions.md D29): the link order SUMO controls is the corridor map's, and
every state the applier sets is the state SUMO then shows, and with
exclusive approach phases (D32) SUMO never shows two approaches of a
junction green."""
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
    assert any("y" in s for s in applied) and any(set(s) == {"r"} for s in applied)  # yellow, all-red


def test_sumo_never_shows_two_approaches_of_a_junction_green(tmp_path: Path,
                                                             monkeypatch: pytest.MonkeyPatch) -> None:
    """Exclusive approach phases (decisions.md D32), checked on what SUMO itself reports
    every second of a max-pressure run: at most one incoming approach of a junction is lit
    (green or yellow), and no green is permissive."""
    seen: dict[str, set[str]] = {}
    original = signals_module.SignalApplier.apply

    def apply(self: Any, commands: Any) -> None:
        original(self, commands)
        for jid in self.junctions:
            state = traci.trafficlight.getRedYellowGreenState(jid)
            links = traci.trafficlight.getControlledLinks(jid)
            lit = {links[i][0][0].rsplit("_", 1)[0] for i, ch in enumerate(state) if ch in "GgYy"}
            assert len(lit) <= 1, (jid, state, lit)
            assert "g" not in state, (jid, state)
            seen.setdefault(jid, set()).update(lit)

    monkeypatch.setattr(signals_module.SignalApplier, "apply", apply)
    run("maxpressure_hyst_cap", 600, 1, results_dir=tmp_path)
    # over ten minutes every approach of every junction was served
    assert all(len(approaches) == 4 for approaches in seen.values()), seen
