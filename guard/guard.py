from schemas.messages import RatedObservation


class Guard:
    """Stub: trusts every camera and passes its queue through unchanged.

    The real Guard will compare each approach against its neighbours
    (exits_to upstream vs arrivals here), against the signal Commands it is
    fed back (a queue shouldn't grow on green), and against image health.
    """

    def __init__(self):
        self.last_commands = {}

    def rate(self, observations, last_commands):
        self.last_commands = last_commands
        return [
            RatedObservation(observation=obs, queue_estimate=obs.queue_pcu, margin=0.0,
                             trust_state="TRUSTED", override_flag=False)
            for obs in observations
        ]
