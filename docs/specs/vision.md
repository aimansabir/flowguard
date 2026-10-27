# Spec: Vision (Module 1)

Owner: **Aimen**. Package: `packages/vision` (`flowguard_vision`).
Depends on: `flowguard_contracts` only. Never on the core or SUMO.

## Goal

Turn recorded footage of one approach into one `Measurement` per second
(`source = "camera"`) that the core accepts unchanged, and measure how wrong
those measurements are. The error levels feed the fault injector so the
simulator camera stops being optimistic.

## Interface

Input:
- a video clip of one approach (fixed camera);
- a calibration file per camera, `calib/<approach_id>.json`:
  `junction_id`, `approach_id`, `zone_polygon` (image px), `zone_length_m`,
  `entry_line`, `stop_line`, `movement_regions` {left, straight, right}
  (polygons a departing vehicle ends up in), `reference_frame` path,
  `px_to_m` homography (4 point pairs image -> ground).

Output: JSONL of `Measurement`, one per second, validated by
`flowguard_contracts.Measurement` before writing.

```
uv run python -m flowguard_vision.run clip.mp4 --calib calib/W_J1.json --out measurements.jsonl
```

## Requirements

- **V1 Detection.** Six classes exactly: car, motorcycle, rickshaw, qingqi,
  bus, truck. COCO has no rickshaw or qingqi, so a fine-tuned YOLO model is
  needed; start from a COCO-pretrained checkpoint.
- **V2 Tracking.** ByteTrack (or equivalent) so each vehicle is counted once.
  Track speed in m/s via the homography; stopped = speed < 0.1 m/s held 1 s.
- **V3 Aggregation per second**, matching docs/contracts.md exactly:
  `zone_counts` (tracks inside the zone polygon at the end of the second),
  `queue_counts` (of those, stopped), `arrivals` (tracks that entered the
  zone this second), `departures` by movement (crossed the stop line this
  second; movement decided when the track reaches a movement region, so a
  departure may be emitted a few seconds late: document the lag).
- **V4 Image health** every second: `frozen` (consecutive frame difference
  below threshold), `blur` (variance of Laplacian mapped to 0-1),
  `brightness` (mean luminance 0-1), `displacement_px` (ORB features +
  homography against `reference_frame`).
- **V5 Detector quality**: `detector_confidence` mean/std over the second,
  `track_breaks_per_min` (tracks lost then restarted nearby).
- **V6** `count_error`: leave null until V8 gives a calibrated error model.
- **V7 Determinism**: same clip + model + calib = same output.
- **V8 Evaluation** (below) with a written report in `docs/results.md`.

## Evaluation

Dataset: at least 3 approaches x 3 conditions (day, night, rain or glare if
available) x 10 minutes, hand-labelled. Labelling unit: per 10-second
window, per class: vehicles in zone at window end, arrivals, departures by
movement. Keep labels in `data/vision_labels/` (CSV, committed) and clips
outside git (too large; record their location and checksums).

Metrics:
- per-class count MAE and bias for zone counts, arrivals and departures, at
  10 s windows (and 60 s);
- queue MAE (vehicles and PCU using `config/pcu.json`);
- departure movement accuracy (confusion matrix left/straight/right);
- image-health flags: precision/recall on synthetic corruptions of clean
  clips (Gaussian blur, brightness scale, repeated frames, shifted crop),
  where ground truth is known by construction;
- throughput: frames per second on the dev laptop.

Deliverable for the Guard: a table of error levels (bias and spread per
class and condition). The fault injector's undercount and noise scenarios
use these numbers.

## Work items (in order)

1. Calibration file format + a small tool to draw polygons on a frame.
2. Footage collection plan (which approaches, camera positions, permissions).
3. Detection + tracking baseline on COCO classes; aggregation to Measurement;
   end-to-end run accepted by `flowguard_core.Core.step`.
4. Labelling and fine-tuning for the six classes.
5. Image-health signals and quality block.
6. Evaluation report and error table.

## Acceptance

- Output of `flowguard_vision.run` validates and is accepted by the core
  (an integration test feeds a short sample clip's output into `Core.step`).
- Evaluation report with all metrics above, per condition.
- `tests/test_boundaries.py` still passes (no imports of core or SUMO).

## Notes

- ML dependencies (ultralytics, opencv, torch) go into an optional extra of
  `flowguard-vision`, so the rest of the team never installs them.
- Lane discipline is weak on the target roads; count per approach, not per lane.
