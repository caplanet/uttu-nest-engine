"""Per-seed paired comparison of arms against a baseline sweep.

  python compare.py --runs <dir> --baseline baseline [--arms a b c]

Seed noise on this marker is ~1.7%p, far larger than most arm effects, so the
per-seed delta is the signal; the mean is only a summary of it.
"""
import argparse
import json
from pathlib import Path

TARGET_EFF = 0.82


def load(runs, arm):
    p = Path(runs) / arm / "sweep.json"
    if not p.exists():
        return None
    return json.loads(p.read_text(encoding="utf-8"))


def by_seed(d):
    return {r["seed"]: r for r in d["runs"] if r.get("passed")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", required=True)
    ap.add_argument("--baseline", default="baseline")
    ap.add_argument("--arms", nargs="*", default=None)
    args = ap.parse_args()

    runs = Path(args.runs)
    base = load(runs, args.baseline)
    if base is None:
        raise SystemExit(f"no baseline sweep at {runs / args.baseline}")
    bseed = by_seed(base)
    arms = args.arms or sorted(p.name for p in runs.iterdir()
                               if p.is_dir() and p.name != args.baseline and (p / "sweep.json").exists())

    target_w = None
    for r in base["runs"]:
        if r.get("passed"):
            target_w = r["width_final"] * r["eff_final"] / TARGET_EFF
            break

    print(f"target: eff {TARGET_EFF * 100:.1f}%  width {target_w:.1f} mm" if target_w else "")
    print(f"\n{'arm':<24} {'n':>2} {'mean eff':>9} {'vs base':>8} {'mean width':>11} {'per-seed delta mm (- = better)'}")
    print("-" * 100)
    bw = [r["width_final"] for r in base["runs"] if r.get("passed")]
    be = [r["eff_final"] for r in base["runs"] if r.get("passed")]
    print(f"{args.baseline:<24} {len(be):>2} {sum(be) / len(be) * 100:>8.3f}% {'':>8} "
          f"{sum(bw) / len(bw):>10.2f}  " + " ".join(f"s{r['seed']}={r['width_final']:.1f}" for r in base["runs"] if r.get("passed")))

    rows = []
    for arm in arms:
        d = load(runs, arm)
        if not d:
            continue
        aseed = by_seed(d)
        common = sorted(set(aseed) & set(bseed))
        if not common:
            print(f"{arm:<24} (no common passing seeds)")
            continue
        deltas = [aseed[s]["width_final"] - bseed[s]["width_final"] for s in common]
        effs = [aseed[s]["eff_final"] for s in common]
        beffs = [bseed[s]["eff_final"] for s in common]
        mean_eff = sum(effs) / len(effs)
        d_eff = (mean_eff - sum(beffs) / len(beffs)) * 100
        wins = sum(1 for x in deltas if x < -0.01)
        rows.append((d_eff, arm, len(common), mean_eff, sum(aseed[s]["width_final"] for s in common) / len(common),
                     " ".join(f"s{s}={dd:+.1f}" for s, dd in zip(common, deltas)), wins, len(common)))
    for d_eff, arm, n, mean_eff, mw, per, wins, tot in sorted(rows, reverse=True):
        print(f"{arm:<24} {n:>2} {mean_eff * 100:>8.3f}% {d_eff:>+7.3f}p {mw:>10.2f}  {per}  ({wins}/{tot} win)")


if __name__ == "__main__":
    main()
