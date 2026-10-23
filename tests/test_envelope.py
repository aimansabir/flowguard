import random

from control.envelope import SafetyEnvelope, SignalTiming
from schemas.corridor import Corridor, Junction
from sim.signals import has_conflicting_greens, state_string

JUNCTION = Junction(
    id="J", approaches=["e", "w", "n", "s"], phases=[["e", "w"], ["n", "s"]],
    links=[["e", "s"], ["e", "l"], ["w", "s"], ["w", "r"], ["n", "s"], ["n", "l"], ["s", "s"]],
)
CORRIDOR = Corridor(junctions={"J": JUNCTION}, downstream={}, exit_links={}, entries={}, exits={})
TIMING = SignalTiming(min_green_s=10, max_red_s=60, yellow_s=3, all_red_s=2)


def drive(request_at, seconds):
    env = SafetyEnvelope(CORRIDOR, TIMING, "test")
    return env, [env.step(t, {"J": request_at(t)})[0] for t in range(seconds)]


def random_requests(seed):
    rng = random.Random(seed)
    return lambda t: rng.choice([0, 1, None])


def test_rejects_green_shorter_than_min():
    env, cmds = drive(lambda t: 1, 30)
    assert all(c.stage == "green" and c.phase_index == 0 for c in cmds[:TIMING.min_green_s])
    assert cmds[TIMING.min_green_s].stage == "yellow"
    assert env.interventions["min_green"] == TIMING.min_green_s


def test_every_green_lasts_at_least_min_green():
    _, cmds = drive(random_requests(1), 3000)
    lengths, run = [], 0
    for prev, cur in zip(cmds, cmds[1:]):
        run += prev.stage == "green"
        if prev.stage == "green" and cur.stage != "green":
            lengths.append(run)
            run = 0
    assert lengths and min(lengths) >= TIMING.min_green_s


def test_yellow_and_all_red_before_every_change():
    _, cmds = drive(random_requests(2), 3000)
    clearance = ["yellow"] * TIMING.yellow_s + ["all_red"] * TIMING.all_red_s
    changes = 0
    for i in range(1, len(cmds)):
        if cmds[i].stage == "green" and cmds[i - 1].stage != "green":
            changes += 1
            assert [c.stage for c in cmds[i - len(clearance):i]] == clearance
            assert cmds[i].phase_index != cmds[i - len(clearance) - 1].phase_index
    assert changes > 10


def test_never_conflicting_greens():
    _, cmds = drive(random_requests(3), 3000)
    for c in cmds:
        assert not has_conflicting_greens(JUNCTION, state_string(JUNCTION, c))


def test_conflict_check_catches_bad_state():
    assert has_conflicting_greens(JUNCTION, "GGGGGGG")
    assert not has_conflicting_greens(JUNCTION, "GgGGrrr")


def test_max_red_forces_a_switch():
    env, cmds = drive(lambda t: 0, 600)  # controller never lets the side street go
    assert env.interventions["max_red"] > 0
    red, longest = 0, 0
    for c in cmds:
        red = red + 1 if JUNCTION.light("n", c) == "r" else 0
        longest = max(longest, red)
    assert longest <= TIMING.max_red_s


def test_bad_phase_index_is_ignored():
    env, cmds = drive(lambda t: 7, 20)
    assert all(c.phase_index == 0 and c.stage == "green" for c in cmds)
    assert env.interventions["invalid_request"] == 20
