import json

import pytest

from experiments.run import run

FILES = ["observations.jsonl", "rated.jsonl", "commands.jsonl", "tripinfo.xml", "summary.json"]


@pytest.mark.parametrize("controller", ["fixed", "maxpressure"])
def test_300s_run(controller, tmp_path):
    summary = run(controller, 300, 1, results_dir=tmp_path)
    out = tmp_path / summary["run_id"]
    for name in FILES:
        assert (out / name).stat().st_size > 0, name

    # 12 approaches and 3 junctions, one record each per second
    with open(out / "observations.jsonl") as f:
        assert sum(1 for _ in f) == 300 * 12
    with open(out / "commands.jsonl") as f:
        assert sum(1 for _ in f) == 300 * 3
    assert json.loads((out / "summary.json").read_text())["vehicles_departed"] > 0
