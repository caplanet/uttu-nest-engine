"""Three independent one-core searches at once, instead of one three-core search.

`separator.n_workers` is best-of-k duplicate search, not work splitting, so three
cores inside one run buy diversity at one point in the search rather than wall-clock.
Spending the same three cores on three whole runs with different seeds buys diversity
across the entire trajectory instead, and the harness keeps the best.

Same total core budget as sweep.py (three cores, one thread and one core per run),
so the two are directly comparable. Wall clock is one run, not three.

  python parallel_seeds.py --arm par_baseline --seeds 1 2 3 --out <dir>
  python parallel_seeds.py --arm par_band --overrides arms/band_full.json --seeds 1 2 3 --out <dir>
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN_PS1 = HERE / "run.ps1"
DEFAULT_DLL = HERE.parents[1] / "target" / "release" / "cpswcore.dll"
DEFAULT_MARKER = Path(r"C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json")
# Cores 5, 6 and 7 — the same three sweep.py pins to, one per concurrent run.
DEFAULT_MASKS = [32, 64, 128]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True)
    ap.add_argument("--seeds", type=int, nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--overrides", default="")
    ap.add_argument("--dll", default=str(DEFAULT_DLL))
    ap.add_argument("--marker", default=str(DEFAULT_MARKER))
    ap.add_argument("--explore", type=float, default=480)
    ap.add_argument("--compress", type=float, default=60)
    ap.add_argument("--polish", type=float, default=60)
    ap.add_argument("--overlap", type=float, default=0.8)
    ap.add_argument("--masks", type=int, nargs="*", default=DEFAULT_MASKS)
    args = ap.parse_args()

    if len(args.seeds) > len(args.masks):
        raise SystemExit(f"{len(args.seeds)} seeds but only {len(args.masks)} cores; "
                         "running more would exceed the three-core budget")

    outdir = Path(args.out) / args.arm
    outdir.mkdir(parents=True, exist_ok=True)

    procs = []
    t0 = time.time()
    for seed, mask in zip(args.seeds, args.masks):
        prefix = outdir / f"seed{seed}"
        cmd = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(RUN_PS1),
               "-Dll", args.dll, "-Marker", args.marker, "-Seed", str(seed), "-OutPrefix", str(prefix),
               "-ExploreS", str(args.explore), "-CompressS", str(args.compress),
               "-PolishS", str(args.polish), "-OverlapMm", str(args.overlap),
               "-AffinityMask", str(mask), "-RayonThreads", "1"]
        if args.overrides:
            cmd += ["-Overrides", args.overrides]
        procs.append((seed, prefix, subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)))
        print(f"[{args.arm}] seed {seed} started on affinity mask {mask}", flush=True)

    results = []
    for seed, prefix, proc in procs:
        proc.wait()
        summary = Path(str(prefix) + ".summary.json")
        if not summary.exists():
            print(f"[{args.arm}] seed {seed} produced no summary", flush=True)
            results.append({"seed": seed, "passed": False})
            continue
        d = json.loads(summary.read_text(encoding="utf-8"))
        results.append({
            "seed": seed, "passed": d["passed"], "errors": d.get("errors", []),
            "width_init": d["width"]["init"], "width_explore": d["width"]["explore"],
            "width_compress": d["width"]["compress"], "width_final": d["width"]["final"],
            "eff_init": d["efficiency"]["init"], "eff_explore": d["efficiency"]["explore"],
            "eff_compress": d["efficiency"]["compress"], "eff_final": d["efficiency"]["final"],
            "seconds": d["seconds"],
        })
        print(f"[{args.arm}] seed {seed} final={d['width']['final']:.2f} "
              f"eff={d['efficiency']['final'] * 100:.3f}%", flush=True)
    wall = time.time() - t0

    ok = [r for r in results if r.get("passed")]
    best = max(ok, key=lambda r: r["eff_final"]) if ok else None
    agg = {
        "arm": args.arm, "mode": "parallel_one_core_per_seed", "overrides": args.overrides,
        "explore_s": args.explore, "compress_s": args.compress, "polish_s": args.polish,
        "overlap_mm": args.overlap, "masks": args.masks, "wall_s": wall,
        "n_ok": len(ok), "n_total": len(results),
        "best_eff": best["eff_final"] if best else None,
        "best_seed": best["seed"] if best else None,
        "best_width": best["width_final"] if best else None,
        "mean_eff": sum(r["eff_final"] for r in ok) / len(ok) if ok else None,
        "runs": results,
    }
    (outdir / "sweep.json").write_text(json.dumps(agg, indent=1, ensure_ascii=False), encoding="utf-8")
    if best:
        print(f"[{args.arm}] best-of-{len(ok)} = seed {best['seed']} "
              f"{best['width_final']:.2f}mm {best['eff_final'] * 100:.3f}%  (wall {wall:.0f}s)", flush=True)
    else:
        print(f"[{args.arm}] no passing runs", flush=True)


if __name__ == "__main__":
    main()
