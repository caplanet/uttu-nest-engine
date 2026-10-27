"""Fast constructor comparison: Init only (Explore/Compress/Polish clipped to a token budget).

Init is ~5s, so this ranks initial-placement engines in about a minute per arm instead of
the 10 minutes a full pipeline run costs. Seed *width* is known not to transfer directly to
final density on this marker, so treat the ranking as a map of the construction landscape,
not as a prediction of the final result.

  python seed_sweep.py --out <dir> --seeds 1 2
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ARMS = HERE / "arms"
RUN_PS1 = HERE / "run.ps1"
DLL = HERE.parents[1] / "target" / "release" / "cpswcore.dll"
MARKER = Path(r"C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json")


def run(seed, prefix, overrides):
    cmd = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(RUN_PS1),
           "-Dll", str(DLL), "-Marker", str(MARKER), "-Seed", str(seed), "-OutPrefix", str(prefix),
           "-ExploreS", "1", "-CompressS", "0", "-PolishS", "0", "-OverlapMm", "0.8", "-AffinityMask", "224"]
    if overrides:
        cmd += ["-Overrides", str(overrides)]
    subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    p = Path(str(prefix) + ".summary.json")
    if not p.exists():
        return None
    d = json.loads(p.read_text(encoding="utf-8"))
    return d if d.get("passed") else {"failed": True, "errors": d.get("errors", [])[:1]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--arms", nargs="*", default=None)
    args = ap.parse_args()

    names = args.arms or ["baseline"] + sorted(p.stem for p in ARMS.glob("seed_*.json"))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name in names:
        ov = "" if name == "baseline" else str(ARMS / f"{name}.json")
        widths, times, fails = [], [], []
        for seed in args.seeds:
            t0 = time.time()
            d = run(seed, out / f"{name}_s{seed}", ov)
            if d is None or d.get("failed"):
                fails.append(str(d))
                continue
            widths.append(d["width"]["init"])
            times.append(d["seconds"]["init"])
        if not widths:
            print(f"{name:<22} FAILED {fails[:1]}", flush=True)
            rows.append((1e9, name, None, None))
            continue
        mw = sum(widths) / len(widths)
        mt = sum(times) / len(times)
        # Efficiency uses the same definition as the full pipeline harness.
        d0 = json.loads((out / f"{name}_s{args.seeds[0]}.summary.json").read_text(encoding="utf-8"))
        eff = d0["input_area_mm2"] / (mw * d0["strip_height"])
        rows.append((mw, name, eff, mt))
        print(f"{name:<22} init width {mw:>8.1f}  eff {eff * 100:>7.3f}%  init {mt:>5.1f}s  "
              + " ".join(f"{w:.0f}" for w in widths), flush=True)
    print("\n-- ranked by init width --")
    for mw, name, eff, mt in sorted(rows):
        if eff:
            print(f"  {name:<22} {mw:>8.1f} mm  {eff * 100:>7.3f}%  ({mt:.1f}s)")


if __name__ == "__main__":
    main()
