# flowguard_vision

Camera footage -> one `flowguard_contracts.Measurement` per approach per
second, `source = "camera"`. Full spec: `docs/specs/vision.md`.

Status: everything after the detector is built and tested on synthetic tracks:
calibration + homography (`calibration.py`, `geometry.py`), per-second aggregation
(`aggregate.py`), image-health signals (`health.py`), and `run.py` (frames ->
Measurements -> JSONL). The detector and tracker plug in through the `Tracker`
protocol in `tracking.py`. `yolo.py` implements it with Ultralytics YOLO + ByteTrack
and `video.py` is the CLI for recorded clips (`python -m flowguard_vision.video --video
clip.mp4 --calib calib/W_J1.json --out W_J1.jsonl`); both need the optional `ultralytics`
and `opencv-python-headless` installs and are tested only on fake detector output. They
have not been run on real footage, and stock COCO weights have no rickshaw or qingqi
class: accuracy on Karachi video is not validated. `stub.py` builds a valid empty Measurement. `scripts/vision_dashboard.py`
shows the pipeline on a toy scene with injected camera faults.

Tools for a real clip:
- `tools/calibrate.html` (open in a browser, no install): draw the zone, entry and stop lines, turn
  regions and four ground points on one frame, then download `<approach_id>.json`. Keep the frame
  image next to it.
- `colab/flowguard_vision.ipynb`: runs YOLO + ByteTrack on a Colab GPU. Build the upload with
  `uv run python scripts/pack_for_colab.py`; the notebook lists the files it needs.

Rules:
- Depend on `flowguard_contracts` only (enforced by `tests/test_boundaries.py`).
- Raw per-class counts; never PCU, never trust judgements. That is the core's job.
- ML dependencies (ultralytics, opencv, torch) go into an optional extra
  of this package, so nobody else has to install them.
