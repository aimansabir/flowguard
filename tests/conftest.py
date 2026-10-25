from pathlib import Path

import pytest

from flowguard_contracts import Corridor
from flowguard_sim.corridor import load_corridor
from flowguard_sim.paths import ROOT


@pytest.fixture(scope="session")
def corridor() -> Corridor:
    return load_corridor(ROOT / "data" / "net" / "corridor.json")


@pytest.fixture(scope="session")
def corridor_path() -> Path:
    return ROOT / "data" / "net" / "corridor.json"
