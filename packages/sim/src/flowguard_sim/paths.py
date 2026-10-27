"""Repo-relative locations. Data and config live at the repo root, not in the package."""

import json
from pathlib import Path
from typing import Any


def _find_root() -> Path:
    for p in Path(__file__).resolve().parents:
        if (p / "config" / "sim.json").is_file():
            return p
    raise RuntimeError("flowguard repo root not found (no config/sim.json above this package)")


ROOT = _find_root()


def load_json(rel: str | Path) -> dict[str, Any]:
    data: dict[str, Any] = json.loads((ROOT / rel).read_text())
    return data


def without_notes(cfg: dict[str, Any]) -> dict[str, Any]:
    """Config files carry a human `_note`; the core rejects unknown keys."""
    return {k: v for k, v in cfg.items() if not k.startswith("_")}
