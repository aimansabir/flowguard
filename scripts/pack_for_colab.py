"""Zip the two packages the vision pipeline needs, to upload to Google Colab.

    uv run python scripts/pack_for_colab.py        # writes packages/vision/colab/flowguard_vision_bundle.zip

Colab only needs the Python sources (contracts for the Measurement model, vision for the
pipeline); torch, ultralytics and opencv come from Colab itself.
"""

import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "packages" / "vision" / "colab" / "flowguard_vision_bundle.zip"
SOURCES = [ROOT / "packages" / "contracts" / "src" / "flowguard_contracts",
           ROOT / "packages" / "vision" / "src" / "flowguard_vision"]


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        for pkg in SOURCES:
            for f in sorted(pkg.rglob("*")):
                if f.is_file() and "__pycache__" not in f.parts:
                    z.write(f, f.relative_to(pkg.parent).as_posix())
    print(f"wrote {OUT} ({OUT.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
