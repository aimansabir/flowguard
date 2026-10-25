from flowguard_contracts import Command, Junction
from flowguard_sim.signals import has_conflicting_greens, state_string

JUNCTION = Junction.model_validate({
    "id": "J",
    "phases": [["e", "w"], ["n", "s"]],
    "links": [{"approach": a, "direction": d} for a, d in
              [("e", "s"), ("e", "l"), ("w", "s"), ("w", "r"), ("n", "s"), ("n", "l"), ("s", "s")]],
})


def cmd(phase: int, stage: str) -> Command:
    return Command.model_validate({"time": 0, "junction_id": "J", "phase_index": phase, "phase_started_at": 0,
                                   "controller_name": "t", "stage": stage, "target_phase": phase,
                                   "requested_phase": None, "intervention": None})


def test_state_strings() -> None:
    assert state_string(JUNCTION, cmd(0, "green")) == "GgGGrrr"  # left turn is permissive
    assert state_string(JUNCTION, cmd(1, "green")) == "rrrrGgG"
    assert state_string(JUNCTION, cmd(0, "yellow")) == "yyyyrrr"
    assert state_string(JUNCTION, cmd(1, "all_red")) == "rrrrrrr"


def test_no_command_produces_conflicting_greens() -> None:
    for phase in (0, 1):
        for stage in ("green", "yellow", "all_red"):
            assert not has_conflicting_greens(JUNCTION, state_string(JUNCTION, cmd(phase, stage)))


def test_conflict_check_catches_bad_state() -> None:
    assert has_conflicting_greens(JUNCTION, "GGGGGGG")
    assert not has_conflicting_greens(JUNCTION, "GgGGrrr")
