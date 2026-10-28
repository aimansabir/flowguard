"""Signal states from movement conflicts (decisions.md D29): which movements
of a lit phase are protected or permissive, and the checks that keep
conflicting movements from both getting protected green."""

import xml.etree.ElementTree as ET

import pytest

from flowguard_contracts import Command, Junction
from flowguard_sim.conflicts import Conflicts, read_conflicts
from flowguard_sim.corridor import light, load_corridor
from flowguard_sim.paths import ROOT, load_json
from flowguard_sim.signals import check_phases, has_conflicting_greens, protected_conflicts, state_string

# A four-arm junction in left-hand traffic. Main road e/w (phase 0), side
# street n/s (phase 1). Links: 0 e straight, 1 e left, 2 e right,
# 3 w straight, 4 w left, 5 w right, 6 n straight, 7 n right, 8 s straight.
LINKS = [("e", "s"), ("e", "l"), ("e", "r"), ("w", "s"), ("w", "l"), ("w", "r"),
         ("n", "s"), ("n", "r"), ("s", "s")]
JUNCTION = Junction.model_validate({
    "id": "J", "phases": [["e", "w"], ["n", "s"]],
    "links": [{"approach": a, "direction": d} for a, d in LINKS],
})
# A right turn crosses the opposing straight and left turn and gives way to
# them; the two right turns pass each other; side-street straights cross
# main-road movements (never green together); n right crosses s straight.
CONFLICTS = Conflicts.build(
    9,
    foes={(2, 3), (2, 4), (5, 0), (5, 1), (7, 8), *((m, s) for m in range(6) for s in (6, 7, 8))},
    yields={(2, 3), (2, 4), (5, 0), (5, 1), (7, 8)},
)


def cmd(phase: int, stage: str, junction: str = "J") -> Command:
    return Command.model_validate({"time": 0, "junction_id": junction, "phase_index": phase,
                                   "phase_started_at": 0, "controller_name": "t", "stage": stage,
                                   "target_phase": phase, "requested_phase": None, "intervention": None})


def green(phase: int) -> str:
    return state_string(JUNCTION, cmd(phase, "green"), CONFLICTS)


def test_opposite_straights_share_protected_green() -> None:
    s = green(0)
    assert (s[0], s[3]) == ("G", "G")
    assert not CONFLICTS.foes[0][3]


def test_right_turn_yields_to_opposing_straight() -> None:
    s = green(0)
    assert (s[2], s[5]) == ("g", "g")


def test_simultaneous_right_turns_do_not_conflict() -> None:
    assert not CONFLICTS.foes[2][5]
    assert protected_conflicts(green(0), CONFLICTS) == []


def test_left_turns_are_protected() -> None:
    s = green(0)
    assert (s[1], s[4]) == ("G", "G")


def test_mixed_movements_per_phase() -> None:
    assert green(0) == "GGgGGgrrr"
    assert green(1) == "rrrrrrGgG"  # n right gives way to s straight


def test_yellow_and_all_red() -> None:
    assert state_string(JUNCTION, cmd(0, "yellow"), CONFLICTS) == "yyyyyyrrr"
    assert state_string(JUNCTION, cmd(1, "yellow"), CONFLICTS) == "rrrrrryyy"
    assert state_string(JUNCTION, cmd(0, "all_red"), CONFLICTS) == "r" * 9


def test_no_command_lights_conflicting_movements() -> None:
    check_phases(JUNCTION, CONFLICTS)
    for phase in (0, 1):
        for stage in ("green", "yellow", "all_red"):
            s = state_string(JUNCTION, cmd(phase, stage), CONFLICTS)
            assert not has_conflicting_greens(JUNCTION, s)
            assert protected_conflicts(s, CONFLICTS) == []


def test_invalid_phase_is_rejected() -> None:
    # crossing straights in one phase: neither gives way, so both would be protected
    bad = Junction.model_validate({**JUNCTION.model_dump(), "phases": [["e", "n"], ["w", "s"]]})
    with pytest.raises(ValueError, match="conflicting movements"):
        check_phases(bad, CONFLICTS)


def test_checks_catch_bad_states() -> None:
    assert has_conflicting_greens(JUNCTION, "G" * 9)
    assert protected_conflicts("GGGGGGGrr", CONFLICTS) != []
    assert not has_conflicting_greens(JUNCTION, "GGgGGgrrr")


def test_link_count_must_match_the_network() -> None:
    with pytest.raises(ValueError, match="links"):
        check_phases(JUNCTION, Conflicts.build(5, set(), set()))


# --- the corridor network itself ---

SIM = load_json("config/sim.json")
CORRIDOR = load_corridor(ROOT / SIM["corridor_file"])
NET = ROOT / SIM["net_file"]
LEFT_HAND = SIM["driving_side"] == "left"


def test_network_drives_on_the_configured_side() -> None:
    assert (ET.parse(NET).getroot().get("lefthand") == "true") == LEFT_HAND


@pytest.mark.parametrize("junction", CORRIDOR.junctions, ids=lambda j: j.id)
def test_every_phase_is_movement_safe(junction: Junction) -> None:
    conflicts = read_conflicts(NET, [junction.id])[junction.id]
    check_phases(junction, conflicts)
    crossing = "r" if LEFT_HAND else "l"
    for phase, approaches in enumerate(junction.phases):
        s = state_string(junction, cmd(phase, "green", junction.id), conflicts)
        for i, link in enumerate(junction.links):
            if link.approach in approaches:
                # the turn across oncoming traffic gives way; straight on and
                # the nearside turn cross nothing that is green with them
                assert s[i] == ("g" if link.direction == crossing else "G"), (phase, link)


def test_direction_rule_would_conflict_in_left_hand_traffic() -> None:
    """The rule this replaces ("left turns yield") only held for right-hand
    traffic: in left-hand traffic it protects right turns that cross the
    opposing straight."""
    if not LEFT_HAND:
        pytest.skip("right-hand network")
    for junction in CORRIDOR.junctions:
        conflicts = read_conflicts(NET, [junction.id])[junction.id]
        c = cmd(0, "green", junction.id)
        lit = [light(junction, link.approach, c) for link in junction.links]
        old = "".join("g" if x == "G" and link.direction in ("l", "L", "t") else x
                      for x, link in zip(lit, junction.links, strict=True))
        assert protected_conflicts(old, conflicts) != []
