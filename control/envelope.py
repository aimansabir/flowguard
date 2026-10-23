"""Safety envelope: the only thing that decides what the signals actually show.

Controllers just say which phase they'd like green. The envelope turns that
into a legal sequence: minimum green, yellow then all-red on every change, no
phase red for longer than max_red, and only one phase green at a time. A
controller can't skip any of it.
"""
from collections import Counter
from dataclasses import dataclass

from schemas.messages import Command


@dataclass
class SignalTiming:
    min_green_s: int
    max_red_s: int
    yellow_s: int
    all_red_s: int

    @classmethod
    def from_config(cls, cfg):
        timing = cls(cfg["min_green_s"], cfg["max_red_s"], cfg["yellow_s"], cfg["all_red_s"])
        # with two phases, a forced switch has to be possible without cutting a green short
        if timing.max_red_s < timing.min_green_s + timing.yellow_s + 2 * timing.all_red_s:
            raise ValueError("max_red_s is too short for this min green and clearance")
        return timing

    @property
    def clearance_s(self):
        return self.yellow_s + self.all_red_s


@dataclass
class _Signal:
    phase: int  # phase that is green, or being cleared during yellow/all-red
    stage: str
    stage_started: float
    target: int  # phase we're changing to (== phase when settled)
    red_since: dict  # phase -> time it went red, for phases not currently green


class SafetyEnvelope:
    def __init__(self, corridor, timing, controller_name):
        self.timing = timing
        self.controller_name = controller_name
        self.interventions = Counter()
        self.n_phases = {jid: len(j.phases) for jid, j in corridor.junctions.items()}
        self.signals = {
            jid: _Signal(phase=0, stage="green", stage_started=0.0, target=0,
                         red_since={p: 0.0 for p in range(1, n)})
            for jid, n in self.n_phases.items()
        }

    def step(self, t, requests):
        """requests: {junction_id: wanted phase}. Missing = no preference."""
        return [self._step_one(jid, t, requests.get(jid)) for jid in self.signals]

    def _step_one(self, jid, t, requested):
        s = self.signals[jid]
        timing = self.timing
        reason = None
        wanted = requested
        if wanted is not None and not 0 <= wanted < self.n_phases[jid]:
            reason, wanted = "invalid_request", None

        # finish a change that's already under way; requests are ignored until it's done
        if s.stage == "yellow" and t - s.stage_started >= timing.yellow_s:
            s.stage, s.stage_started = "all_red", t
            s.red_since[s.phase] = t
        if s.stage == "all_red" and t - s.stage_started >= timing.all_red_s:
            del s.red_since[s.target]
            s.phase, s.stage, s.stage_started = s.target, "green", t

        if s.stage == "green":
            green_for = t - s.stage_started
            # the clearance still has to run after we start switching, so a
            # starving phase is caught clearance_s before it hits max_red
            starving = [p for p, since in s.red_since.items()
                        if t - since + timing.clearance_s >= timing.max_red_s]
            if starving:
                target = min(starving, key=s.red_since.get)  # red the longest
            elif wanted is not None and wanted != s.phase:
                target = wanted
            else:
                target = None

            if target is not None and green_for < timing.min_green_s:
                if wanted is not None and wanted != s.phase:
                    reason = "min_green"
            elif target is not None:
                s.stage, s.stage_started, s.target = "yellow", t, target
                if wanted != target:
                    reason = "max_red"

        if reason:
            self.interventions[reason] += 1
        return Command(time=t, junction_id=jid, phase_index=s.phase, phase_started_at=s.stage_started,
                       controller_name=self.controller_name, stage=s.stage, target_phase=s.target,
                       requested_phase=requested, intervention=reason)
