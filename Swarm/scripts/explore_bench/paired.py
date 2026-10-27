"""Sequential, alternating A/B Explore-only measurements; Python standard library only."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def tolerance(width):
    return max(0.00001, abs(width) * 0.0000002)


def observations(summary):
    points = [(0.0, summary["initial"]["width"])]
    points += [(x["seconds"], x["width"]) for x in summary["trace"] if x["certified"]]
    points.append((summary["explore_seconds"], summary["final"]["width"]))
    return sorted(points)


def first_observed(summary, target):
    return next((t for t, width in observations(summary) if width <= target + tolerance(target)), None)


def compare(a, b, marker, seed, duration, repeat, order):
    if not a["passed"] or not b["passed"]:
        raise RuntimeError("Harness validation failed")
    for key in ("input_sha256", "seed", "requested_explore_seconds", "config", "abi"):
        if a[key] != b[key]:
            raise RuntimeError(f"A/B {key} mismatch for {marker}, seed {seed}")
    same_initial = a["initial"]["width"] == b["initial"]["width"] and a["initial"]["pose_sha256"] == b["initial"]["pose_sha256"]
    if not same_initial:
        raise RuntimeError(f"A/B Init layout mismatch for {marker}, seed {seed}; cannot attribute result to Explore")
    aw, bw = a["final"]["width"], b["final"]["width"]
    targets = []
    best = math.inf
    for time, width in observations(a):
        if width >= best:
            continue
        best = width
        targets.append({"baseline_width": width, "baseline_first_observed_seconds": time,
                        "candidate_first_observed_seconds": first_observed(b, width)})
    return {"marker": marker, "seed": seed, "seconds": duration, "repeat": repeat, "order": order,
            "initial_width": a["initial"]["width"], "identical_initial_poses": same_initial,
            "baseline_width": aw, "candidate_width": bw, "delta_mm": bw - aw,
            "no_width_regression": bw <= aw + tolerance(aw),
            "baseline_init_seconds": a["init_seconds"], "candidate_init_seconds": b["init_seconds"],
            "baseline_explore_seconds": a["explore_seconds"], "candidate_explore_seconds": b["explore_seconds"],
            "baseline_final_target_seconds": first_observed(a, aw),
            "candidate_baseline_final_target_seconds": first_observed(b, aw),
            "baseline_targets": targets}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline", type=Path, required=True)
    p.add_argument("--candidate", type=Path, required=True)
    p.add_argument("--harness", type=Path, default=Path(__file__).parent / "bin/Release/net10.0/ExploreBench.dll")
    p.add_argument("--seeds", type=int, nargs="+", default=[17, 29, 43])
    p.add_argument("--seconds", type=float, nargs="+", default=[20, 60])
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--marker", action="append", help="name=absolute/input.json; default WSjk and ops2")
    p.add_argument("--out", type=Path, help="New output directory; default current experiment/paired-TIMESTAMP")
    p.add_argument("--timeout", type=float, default=900, help="Seconds per process, including Init")
    args = p.parse_args()
    if any(x <= 0 or x >= 2**64 for x in args.seeds) or any(not math.isfinite(x) or x <= 0 for x in args.seconds) or args.repeats < 1:
        p.error("Seeds, seconds, and repeats must be positive; seeds must fit u64")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        p.error("Timeout must be finite and positive")
    markers = {"wsjk": Path("C:/Caplanet/AiWorks/InputData/여성WSjk_비교용-G_New_SwarmV1.json"),
               "ops2": ROOT / "target/phase-improvements-bench/inputs/ops2.json"}
    if args.marker:
        markers = {}
        for spec in args.marker:
            name, path = spec.split("=", 1)
            if not name or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in name):
                p.error("Marker names use letters, digits, underscore or dash")
            if name in markers:
                p.error("Duplicate marker name")
            markers[name] = Path(path)
    for path in [args.baseline, args.candidate, args.harness, *markers.values()]:
        if not path.is_file():
            p.error(f"File missing: {path}")
    experiment = Path((ROOT / "target/explore-priorities-current.txt").read_text(encoding="utf-8-sig").strip())
    out = (args.out or experiment / ("paired-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))).resolve()
    out.mkdir(parents=True, exist_ok=False)
    dlls = {"A": args.baseline.resolve(), "B": args.candidate.resolve()}
    hashes = {key: sha256(path) for key, path in dlls.items()}
    results = []
    manifest = {"started_utc": datetime.now(timezone.utc).isoformat(), "dlls": {k: str(v) for k, v in dlls.items()},
                "dll_sha256": hashes, "seeds": args.seeds, "seconds": args.seconds, "repeats": args.repeats,
                "harness_sha256": sha256(args.harness), "markers": {k: {"path": str(v.resolve()), "sha256": sha256(v)} for k, v in markers.items()},
                "note": "Sequential processes with alternating AB/BA. Same per-pair seed for Init and Explore; no reseed API. Native diagnostic callback overhead included. Small samples do not establish statistical superiority."}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    pair = 0
    # One lock across output directories prevents accidental concurrent driver runs.
    lock = ROOT / "scripts/explore_bench/.paired-running.lock"
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError:
        raise RuntimeError(f"Benchmark lock exists: {lock}; check prior process before removing stale lock")
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(str(os.getpid()))
        for repeat in range(args.repeats):
            for marker, input_path in markers.items():
                for seed in args.seeds:
                    for duration in args.seconds:
                        order = "AB" if pair % 2 == 0 else "BA"
                        stem = f"{pair:03d}-{marker}-seed{seed}-{duration:g}s-r{repeat}"
                        summaries = {}
                        for label in order:
                            if sha256(dlls[label]) != hashes[label]:
                                raise RuntimeError(f"{label} DLL changed during paired run")
                            prefix = out / (stem + "-" + label)
                            cmd = ["dotnet", str(args.harness.resolve()), str(dlls[label]), str(input_path.resolve()),
                                   str(seed), str(duration), str(prefix)]
                            print("Running", stem, label, flush=True)
                            with Path(str(prefix) + ".log").open("w", encoding="utf-8") as log:
                                subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout, check=True)
                            summary = json.loads(Path(str(prefix) + ".summary.json").read_text(encoding="utf-8"))
                            if summary["dll_sha256"] != hashes[label]:
                                raise RuntimeError("Harness loaded DLL hash mismatch")
                            summaries[label] = summary
                        row = compare(summaries["A"], summaries["B"], marker, seed, duration, repeat, order)
                        results.append(row)
                        report = {"pairs": results, "all_pairs_no_width_regression": all(x["no_width_regression"] for x in results),
                                  "note": manifest["note"], "target_timing_note": "First observed native-certified trace time; callback throttling makes it an upper bound, null means not observed within budget."}
                        (out / "comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
                        fields = [x for x in row if x != "baseline_targets"]
                        with (out / "comparison.csv").open("w", newline="", encoding="utf-8-sig") as stream:
                            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
                            writer.writeheader()
                            writer.writerows(results)
                        print(f"{stem}: {row['baseline_width']:.6f} -> {row['candidate_width']:.6f}; delta={row['delta_mm']:+.6f} mm", flush=True)
                        pair += 1
    finally:
        lock.unlink()
    print(out)
    return 0 if all(x["no_width_regression"] for x in results) else 2


if __name__ == "__main__":
    sys.exit(main())
