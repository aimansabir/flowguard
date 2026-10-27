from flowguard_contracts import Measurement


class FaultInjector:
    """Sits between the observer and the core. For now it changes nothing.

    HOOK (spec: docs/specs/guard.md, fault injection): take the clean camera
    measurements and return corrupted copies: frozen feed = repeat an old
    measurement, blackout = zeros or missing, undercount = scale counts down,
    displacement = counts from the wrong zone. Never modify the inputs in
    place; the run loop logs them as ground truth for scoring the Guard.
    """

    def apply(self, t: float, measurements: list[Measurement]) -> list[Measurement]:
        return measurements
