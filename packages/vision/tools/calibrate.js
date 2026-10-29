// Calibration file builder for flowguard_vision (format: calibration.py, docs/specs/vision.md).
// Shared by calibrate.html and the Python test, which runs it under node.
"use strict";

const MOVEMENTS = ["left", "straight", "right"];
const round1 = (n) => Math.round(n * 10) / 10;
const pt = (p) => [round1(p[0]), round1(p[1])];
const finite = (p) => Array.isArray(p) && p.length === 2 && p.every((v) => Number.isFinite(v));

// state: {junction_id, approach_id, zone_length_m, reference_frame, zone, entry, stop,
//         movements:{left,straight,right}, pairs:[{px:[x,y], m:[X,Y]}]}   (px = image pixels, m = metres)
// returns {errors:[string], calibration:object|null}
function buildCalibration(s) {
  const errors = [];
  if (!String(s.junction_id || "").trim()) errors.push("Junction id is empty.");
  if (!String(s.approach_id || "").trim()) errors.push("Approach id is empty.");
  if (!(Number(s.zone_length_m) > 0)) errors.push("Zone length must be more than 0 metres.");
  if (!String(s.reference_frame || "").trim()) errors.push("Reference frame file name is empty.");
  if ((s.zone || []).length < 3) errors.push("The zone needs at least 3 points.");
  if ((s.entry || []).length !== 2) errors.push("The entry line needs exactly 2 points.");
  if ((s.stop || []).length !== 2) errors.push("The stop line needs exactly 2 points.");
  const regions = {};
  for (const m of MOVEMENTS) {
    const poly = (s.movements || {})[m] || [];
    if (poly.length === 0) continue;
    if (poly.length < 3) errors.push(`The ${m} region needs at least 3 points (or none).`);
    else regions[m] = poly.map(pt);
  }
  const pairs = s.pairs || [];
  if (pairs.length !== 4) errors.push("Exactly 4 point pairs (pixel and metres) are needed for the homography.");
  else if (!pairs.every((p) => finite(p.px) && finite(p.m))) errors.push("Every point pair needs a pixel point and numeric metres.");
  else if (new Set(pairs.map((p) => p.m.join(","))).size < 4) errors.push("The 4 ground points must all differ.");
  if (errors.length) return { errors, calibration: null };
  return {
    errors,
    calibration: {
      junction_id: String(s.junction_id).trim(),
      approach_id: String(s.approach_id).trim(),
      zone_polygon: s.zone.map(pt),
      zone_length_m: Number(s.zone_length_m),
      entry_line: s.entry.map(pt),
      stop_line: s.stop.map(pt),
      movement_regions: regions,
      reference_frame: String(s.reference_frame).trim(),
      px_to_m: pairs.map((p) => [pt(p.px), [Number(p.m[0]), Number(p.m[1])]]),
    },
  };
}

if (typeof module !== "undefined") module.exports = { buildCalibration, MOVEMENTS };
