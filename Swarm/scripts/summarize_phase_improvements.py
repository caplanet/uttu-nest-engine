"""Summarize complete paired runs without silently discarding failed/missing seeds."""
import hashlib
import json
import statistics as st
from pathlib import Path

root = Path(__file__).resolve().parents[1]
bench = root / "target/phase-improvements-bench"
summary = {}
rows = []
for name in ("wsjk", "ops2"):
    paired = []
    for seed in range(11, 21):
        a = json.loads((bench / f"paired-baseline/{name}-{seed}.json").read_text())
        b = json.loads((bench / f"paired-candidate/{name}-{seed}.json").read_text())
        assert a["seed"] == b["seed"] == seed
        assert a["initial_width"] == b["initial_width"], (name, seed, "different initial layout")
        aw, bw = a["rows"][-1]["width"], b["rows"][-1]["width"]
        paired.append((a,b))
        rows.append({"input":name,"seed":seed,"before_mm":aw,"after_mm":bw,"saved_mm":aw-bw,
            "improvement_percent":(aw-bw)/aw*100.})
    selected = [r for r in rows if r["input"] == name]
    result = {
        "pairs":len(paired),
        "input_sha256":hashlib.sha256((bench/f"inputs/{name}.json").read_bytes()).hexdigest(),
        "initial_width_equal":True,
        "before_median_mm":st.median(r["before_mm"] for r in selected),
        "after_median_mm":st.median(r["after_mm"] for r in selected),
        "paired_median_saved_mm":st.median(r["saved_mm"] for r in selected),
        "paired_mean_saved_mm":st.mean(r["saved_mm"] for r in selected),
        "paired_median_improvement_percent":st.median(r["improvement_percent"] for r in selected),
        "wins":sum(r["saved_mm"] > .001 for r in selected),
        "ties":sum(abs(r["saved_mm"]) <= .001 for r in selected),
        "losses":sum(r["saved_mm"] < -.001 for r in selected),
        "worst_saved_mm":min(r["saved_mm"] for r in selected),
        "best_saved_mm":max(r["saved_mm"] for r in selected),
        "validation_before_median_ms":st.median(a["validation_ms_per_call"] for a,b in paired),
        "validation_after_median_ms":st.median(b["validation_ms_per_call"] for a,b in paired),
        "phase_seconds":{p:{variant:st.median(next(r["seconds"] for r in (a if variant=="before" else b)["rows"] if r["phase"]==p) for a,b in paired)
            for variant in ("before","after")} for p in ("Explore","Compress","Polish")},
        "candidate_metrics_median":{k:st.median(b["metrics"][k] for a,b in paired) for k in paired[0][1]["metrics"]},
    }
    summary[name] = result
(bench/"summary.json").write_text(json.dumps({"summary":summary,"pairs":rows},indent=2),encoding="utf-8")
lines = ["| 입력 | 쌍 수 | 전 길이 중앙값(mm) | 후 길이 중앙값(mm) | paired 절감 중앙값(mm) | 개선/동일/악화 |",
         "|---|---:|---:|---:|---:|---|"]
for name,s in summary.items():
    lines.append(f'| {name} | {s["pairs"]} | {s["before_median_mm"]:.3f} | {s["after_median_mm"]:.3f} | {s["paired_median_saved_mm"]:.3f} | {s["wins"]}/{s["ties"]}/{s["losses"]} |')
lines.extend(["", "| 입력 | seed | 전(mm) | 후(mm) | 절감(mm, 양수=개선) |", "|---|---:|---:|---:|---:|"])
for r in rows:
    lines.append(f'| {r["input"]} | {r["seed"]} | {r["before_mm"]:.3f} | {r["after_mm"]:.3f} | {r["saved_mm"]:.3f} |')
(bench/"summary.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
print(json.dumps(summary,ensure_ascii=False,indent=2))
