"""Package boundaries, checked from what each module imports.

- vision produces Measurements and may only depend on the contracts;
- contracts depend on nothing of ours;
- the decision core is Rust and cannot import Python at all, and its crate
  has no simulator dependency (checked on Cargo.toml).
"""

import ast
import tomllib
from pathlib import Path

import pytest

from flowguard_sim.paths import ROOT

SIM = {"traci", "sumolib", "libsumo", "sumo", "flowguard_sim"}
RULES = {
    "vision": SIM | {"flowguard_core"},
    "contracts": SIM | {"flowguard_core", "flowguard_vision"},
}


def imports(path: Path) -> list[str]:
    names: list[str] = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            names += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names.append(node.module or "")
    return names


@pytest.mark.parametrize("package", sorted(RULES))
def test_package_imports(package: str) -> None:
    files = sorted((ROOT / "packages" / package / "src").rglob("*.py"))
    assert files
    for path in files:
        for name in imports(path):
            assert name.split(".")[0] not in RULES[package], f"{path.name} imports {name}"


def test_core_crate_has_no_simulator_dependency() -> None:
    manifest = tomllib.loads((ROOT / "crates" / "core" / "Cargo.toml").read_text())
    assert set(manifest["dependencies"]) <= {"serde", "serde_json", "schemars"}
