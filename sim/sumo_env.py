import os


def sumo_binary(name="sumo"):
    """Path to a SUMO tool (sumo, sumo-gui, netconvert...).

    Uses SUMO_HOME if set, otherwise the copy pip installed with eclipse-sumo.
    """
    if "SUMO_HOME" not in os.environ:
        import sumo
        os.environ["SUMO_HOME"] = sumo.SUMO_HOME
    import sumolib
    return sumolib.checkBinary(name)
