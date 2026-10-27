"""Per-seed markdown table of Init / Explore / Final efficiency for one or more arms.

usage: python seed_table.py name=<dir> [name=<dir> ...] [--seeds 1 2 ...]

<dir> holds FullBench summaries named seed<N>.summary.json (sweep.py / run.ps1 layout).
Missing seeds are shown as '-'. Means are over the seeds each arm actually has.
"""
import argparse
import json
from pathlib import Path


def load(d):
    out = {}
    for p in Path(d).glob("seed*.summary.json"):
        try:
            s = int(p.name[len("seed"):].split(".")[0])
            j = json.loads(p.read_text(encoding="utf-8"))
        except (ValueError, json.JSONDecodeError):
            continue
        if not j.get("passed"):
            continue
        e = j.get("efficiency", {})
        out[s] = (e.get("init"), e.get("explore"), e.get("final"))
    return out


def pct(v):
    return "-" if v is None else f"{v * 100:.3f}%"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("arms", nargs="+", help="name=dir")
    ap.add_argument("--seeds", type=int, nargs="*")
    a = ap.parse_args()
    arms = [(x.split("=", 1)[0], load(x.split("=", 1)[1])) for x in a.arms]
    seeds = a.seeds or sorted({s for _, d in arms for s in d})
    head = "| 시드 | " + " | ".join(f"{n} 초기 | {n} Explore | {n} 최종" for n, _ in arms) + " |"
    print(head)
    print("|---:|" + "---:|---:|---:|" * len(arms))
    for s in seeds:
        cells = []
        for _, d in arms:
            i, x, f = d.get(s, (None, None, None))
            cells += [pct(i), pct(x), pct(f)]
        print(f"| {s} | " + " | ".join(cells) + " |")
    cells = []
    for _, d in arms:
        vals = [d[s] for s in seeds if s in d]
        for k in range(3):
            xs = [v[k] for v in vals if v[k] is not None]
            cells.append(f"**{pct(sum(xs) / len(xs))}**" if xs else "-")
    print("| 평균 | " + " | ".join(cells) + " |")


main()
