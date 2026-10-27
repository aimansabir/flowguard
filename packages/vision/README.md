# flowguard_vision

Camera footage -> one `flowguard_contracts.Measurement` per approach per
second, `source = "camera"`. Full spec: `docs/specs/vision.md`.

Status: skeleton. `stub.py` builds a valid empty Measurement so the rest of
the pipeline (and its tests) can be wired before detection exists.

Rules:
- Depend on `flowguard_contracts` only (enforced by `tests/test_boundaries.py`).
- Raw per-class counts; never PCU, never trust judgements. That is the core's job.
- ML dependencies (ultralytics, opencv, torch) go into an optional extra
  of this package, so nobody else has to install them.
