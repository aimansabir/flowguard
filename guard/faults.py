class FaultInjector:
    """Sits between the observer and the Guard. For now it changes nothing.

    HOOK: faults go in apply(). Take the clean observations and return
    corrupted copies (frozen feed = repeat an old observation, blackout = all
    zeros or missing, undercount = scale counts down...). Don't modify the
    originals in place, the run loop keeps them as ground truth for scoring
    the Guard.
    """

    def apply(self, t, observations):
        return observations
