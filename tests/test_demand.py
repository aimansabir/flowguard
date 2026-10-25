from pathlib import Path

from flowguard_contracts import Corridor
from flowguard_sim.demand import write_demand
from flowguard_sim.paths import ROOT, load_json


def test_committed_demand_regenerates_identically(corridor: Corridor, tmp_path: Path) -> None:
    """Same seed + config + corridor must give the committed file, byte for byte
    (ignoring line endings), or comparisons between runs stop being fair."""
    fresh = tmp_path / "seed1.rou.xml"
    write_demand(fresh, 1, corridor, load_json("config/demand.json"))
    committed = (ROOT / "data" / "demand" / "corridor_seed1.rou.xml").read_bytes()
    assert fresh.read_bytes().replace(b"\r\n", b"\n") == committed.replace(b"\r\n", b"\n")
