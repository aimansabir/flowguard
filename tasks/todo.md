# Restructure: Rust core + typed Python (2026-10-25)

Scope: research/FYP prototype. Rust core (state, Guard, control, safety) behind
PyO3, typed Python for sim/eval/vision, raw-count Measurement contract v2,
max-pressure family. Team split in docs/team.md.

## Done
- [x] Toolchain: Rust GNU (no Visual Studio), uv-managed Python 3.12
- [x] Baseline runs from the old Python code, determinism confirmed
- [x] cargo + uv workspace; data/ for network and demand
- [x] crates/core: contracts, corridor index, state, Guard trait + PassThrough, fixed, max-pressure, envelope, engine
- [x] crates/py: PyO3 binding, JSON in/out
- [x] contracts/ schema and pydantic models generated from Rust; staleness tests on both sides
- [x] corridor v2 from build_net; demand regenerates byte-identical
- [x] observer: 60 m camera zone + full-edge truth, arrivals, departures by movement
- [x] parity gate: 0 differences in 10 800 commands and 43 200 queues, both controllers
- [x] old Python control/guard/schemas removed
- [x] vision skeleton + core acceptance test
- [x] docs: scope, architecture, contracts, decisions D13-D19, results, specs x4, team
- [x] CI workflow; README

## Next (owners in docs/team.md)
- [ ] Merge `restructure` into `main` after review
- [ ] Week 1 items per docs/team.md

## Review
- Verification: `cargo test` 24 passed, clippy -D warnings clean; `uv run pytest` 50 passed;
  ruff clean; pyright strict 0 errors; parity script 0 differences.
- Finding: basic max-pressure is worse than the untuned fixed plan (154.5 s vs 117.6 s)
  and much worse with a 60 m camera (211.5 s). See docs/results.md.
- Not done / known gaps: CI not yet run on GitHub; PyO3 crate not covered by clippy
  (built by maturin only); sublane collisions unexplained (P5); envelope max-red check
  assumes two phases (P6).
