"""The Python copy of the Guard evidence checks (flowguard_sim.evidence_replay)
must score the shared fixture exactly as stored; the Rust test
`matches_the_python_replay_fixture` holds the core to the same file."""

import json
from typing import Any

from flowguard_sim.evidence_replay import FIXTURE, Params, check_fixture


def fixture() -> dict[str, Any]:
    data: dict[str, Any] = json.loads(FIXTURE.read_text())
    return data


def test_replay_matches_fixture() -> None:
    bad = check_fixture(fixture())
    assert not bad, f"{len(bad)} mismatches, first: {bad[:10]}"


def test_default_params_match_fixture() -> None:
    assert Params().to_json() == fixture()["default_params"]


def test_fixture_exercises_every_check() -> None:
    expected = fixture()["expected"]
    for k in range(6):
        assert any(e[k] for step in expected for e in step), f"score {k} is never above zero"
