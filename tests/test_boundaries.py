"""The Guard and controllers must never read SUMO directly, only Observations.

Checked by looking at what their modules import.
"""
import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
FORBIDDEN = {"traci", "sumolib", "libsumo", "sumo", "sim"}


@pytest.mark.parametrize("path", sorted(
    p for pkg in ("guard", "control", "schemas") for p in (ROOT / pkg).glob("*.py")
), ids=lambda p: f"{p.parent.name}/{p.name}")
def test_no_simulator_imports(path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        else:
            continue
        for name in names:
            assert name.split(".")[0] not in FORBIDDEN, f"{path.name} imports {name}"
