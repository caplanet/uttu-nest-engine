"""Sequential seed sweep for one arm. One run at a time, pinned to 3 cores by run.ps1.

  python sweep.py --arm baseline --seeds 1 2 3 --out <dir>
  python sweep.py --arm relief --overrides arms/relief.json --seeds 1 2 3 --out <dir>
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

# Windows 콘솔(cp949)에서 네이티브 오류 문자열의 대체 문자(U+FFFD)를 찍다가 죽지 않게 한다.
sys.stdout.reconfigure(errors="replace")

HERE = Path(__file__).resolve().parent
RUN_PS1 = HERE / "run.ps1"
DEFAULT_DLL = HERE.parents[1] / "target" / "release" / "cpswcore.dll"
DEFAULT_MARKER = Path(r"C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json")


def run_one(dll, marker, seed, prefix, explore, compress, polish, overlap, overrides, affinity, capture_log=False):
    cmd = [
        "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(RUN_PS1),
        "-Dll", str(dll), "-Marker", str(marker), "-Seed", str(seed), "-OutPrefix", str(prefix),
        "-ExploreS", str(explore), "-CompressS", str(compress), "-PolishS", str(polish),
        "-OverlapMm", str(overlap), "-AffinityMask", str(affinity),
    ]
    if overrides:
        cmd += ["-Overrides", str(overrides)]
    if capture_log:
        cmd += ["-CaptureLog"]
    t0 = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    wall = time.time() - t0
    summary = Path(str(prefix) + ".summary.json")
    if not summary.exists():
        return {"seed": seed, "passed": False, "error": (proc.stdout or "") + (proc.stderr or "")[:2000], "wall_s": wall}
    d = json.loads(summary.read_text(encoding="utf-8"))
    return {
        "seed": seed,
        "passed": d["passed"],
        "errors": d.get("errors", []),
        "width_init": d["width"]["init"],
        "width_explore": d["width"]["explore"],
        "width_compress": d["width"]["compress"],
        "width_final": d["width"]["final"],
        "eff_init": d["efficiency"]["init"],
        "eff_explore": d["efficiency"]["explore"],
        "eff_compress": d["efficiency"]["compress"],
        "eff_final": d["efficiency"]["final"],
        "seconds": d["seconds"],
        "wall_s": wall,
    }


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
    ap.add_argument("--affinity", type=int, default=224)
    ap.add_argument("--capture-log", action="store_true", dest="capture_log")
    args = ap.parse_args()

    outdir = Path(args.out) / args.arm
    outdir.mkdir(parents=True, exist_ok=True)
    results = []
    for seed in args.seeds:
        prefix = outdir / f"seed{seed}"
        r = run_one(args.dll, args.marker, seed, prefix, args.explore, args.compress,
                    args.polish, args.overlap, args.overrides, args.affinity, args.capture_log)
        results.append(r)
        eff = r.get("eff_final")
        line = (f"[{args.arm}] seed={seed} passed={r['passed']} "
                f"final={r.get('width_final')} eff={eff * 100:.3f}%" if eff else
                f"[{args.arm}] seed={seed} FAILED {str(r.get('error'))[:300]}")
        print(line, flush=True)
        # Persist after every seed so a long sweep is never lost.
        agg = summarize(args, results)
        (outdir / "sweep.json").write_text(json.dumps(agg, indent=1, ensure_ascii=False), encoding="utf-8")
    agg = summarize(args, results)
    ok = [r for r in results if r.get("passed")]
    if ok:
        effs = sorted(r["eff_final"] for r in ok)
        print(f"[{args.arm}] n={len(ok)} mean={agg['mean_eff'] * 100:.3f}% "
              f"median={agg['median_eff'] * 100:.3f}% best={effs[-1] * 100:.3f}% worst={effs[0] * 100:.3f}% "
              f"mean_width={agg['mean_width']:.2f}", flush=True)
    else:
        print(f"[{args.arm}] no passing runs", flush=True)


def summarize(args, results):
    ok = [r for r in results if r.get("passed")]
    effs = [r["eff_final"] for r in ok]
    widths = [r["width_final"] for r in ok]
    return {
        "arm": args.arm,
        "overrides": args.overrides,
        "explore_s": args.explore, "compress_s": args.compress, "polish_s": args.polish,
        "overlap_mm": args.overlap, "affinity_mask": args.affinity,
        "dll": args.dll, "marker": args.marker,
        "n_ok": len(ok), "n_total": len(results),
        "mean_eff": sum(effs) / len(effs) if effs else None,
        "median_eff": sorted(effs)[len(effs) // 2] if effs else None,
        "best_eff": max(effs) if effs else None,
        "worst_eff": min(effs) if effs else None,
        "mean_width": sum(widths) / len(widths) if widths else None,
        "best_width": min(widths) if widths else None,
        "runs": results,
    }


if __name__ == "__main__":
    sys.exit(main())
