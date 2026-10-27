"""Validate saved phase-benchmark evidence and aggregate matched A/B runs."""
import hashlib
import json
import statistics
import sys
from pathlib import Path

tag = sys.argv[1] if len(sys.argv) > 1 else "2026-10-22-taepyeongyang-cpguard-phases"
root = Path(__file__).resolve().parent / "results"
expected_hashes = {
    "before": "B39DBE1B80CB7A8E13B3EEB63F536126BF6B4C5514C842F280C183D3A1E18C64",
    "after": "AA0BE3DC49052F784E08C07E6EAC0F0E91F6D4546E291FE544F398F7D1F10D39",
}
rows = []
configs = []
init_hashes = set()
for index, arm in enumerate(["before", "after", "after", "before"], 1):
    prefix = root / f"{tag}-r{index}-{arm}"
    result = json.loads(Path(str(prefix) + ".summary.json").read_text(encoding="utf-8"))
    assert result["passed"] and not result["errors"], result["errors"]
    assert result["dll_sha256"] == expected_hashes[arm]
    assert result["input_sha256"] == "2E35C991275F8C4E991440A63B2D8058B74C29D974A71C96E78D257F00D191BC"
    configs.append(result["config"])
    for phase in ["Init", "Explore", "Compress"]:
        evidence = result["checkpoints"][phase]
        assert evidence["placements"] == 4000 and evidence["native_collision_count"] == 0
        raw = Path(str(prefix) + f".{phase}.json").read_bytes()
        assert hashlib.sha256(raw).hexdigest().upper() == evidence["solution_sha256"]
        status = json.loads(Path(str(prefix) + f".{phase}.collisions.json").read_text(encoding="utf-8"))
        assert len(status) == 4000 and not any(x["collision"] for x in status)
    init_hashes.add(result["checkpoints"]["Init"]["solution_sha256"])
    row = {"run": index, "arm": arm}
    for phase in ["init", "explore", "compress"]:
        row[f"{phase}_wall_s"] = result["seconds"][phase]
        row[f"{phase}_cpu_s"] = result["cpu_seconds"][phase]
        row[f"{phase}_width_mm"] = result["width"][phase]
        row[f"{phase}_efficiency_percent"] = result["efficiency"][phase] * 100
    row["final_native_density_percent"] = result["checkpoints"]["Compress"]["native_density"] * 100
    row["final_solution_sha256"] = result["checkpoints"]["Compress"]["solution_sha256"]
    row["callback_count"] = result["callback_count"]
    rows.append(row)
assert all(c == configs[0] for c in configs), "Config mismatch"
assert len(init_hashes) == 1, "Initial layouts differ"
metrics = [k for k in rows[0] if k.endswith(("_s", "_mm", "_percent"))]
aggregates = {}
for arm in ["before", "after"]:
    samples = [r for r in rows if r["arm"] == arm]
    aggregates[arm] = {
        metric: {"mean": statistics.mean(r[metric] for r in samples),
                 "min": min(r[metric] for r in samples),
                 "max": max(r[metric] for r in samples)}
        for metric in metrics
    }
deltas = {}
for metric in metrics:
    before, after = (aggregates[a][metric]["mean"] for a in ["before", "after"])
    deltas[metric] = {"absolute": after - before, "relative_percent": (after / before - 1) * 100 if before else None}
summary = {"validated": True, "runs": rows, "aggregates": aggregates, "deltas": deltas,
           "same_config": True, "same_init_layout_sha256": next(iter(init_hashes)),
           "config": configs[0], "dll_sha256": expected_hashes}
(root / f"{tag}-summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
print(json.dumps({k: summary[k] for k in ["validated", "runs", "deltas", "same_init_layout_sha256"]}, indent=2))
