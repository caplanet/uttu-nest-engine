"""Merge a marker (instance JSON) with a layout into one warm-start input for FullBench.

swarm_init_from_json accepts jagua-rs ExtSPOutput: the instance fields plus a "solution"
object. Given that, Swarm restores every item and skips its own LBF placement
(tests/warm_start.rs: init_with_solution_skips_lbf_placement). FullBench passes the marker
file text straight to swarm_init_from_json and computes efficiency from its "items", so the
merged file can be used as -Marker unchanged.

usage: python make_warm.py <marker.json> <layout.json> <out.json>

<layout.json> is the nesting-engine output ({"strip_width", "layout": {...}, "density"}).
jagua-rs requires solution.run_time_sec, which nesting-engine does not write; it is added as 0.
"""
import json
import sys


def main():
    marker, layout, out = sys.argv[1:4]
    inst = json.load(open(marker, encoding="utf-8"))
    sol = json.load(open(layout, encoding="utf-8"))
    lay = sol["layout"]
    placed = lay["placed_items"]
    demand = {it["id"]: it.get("demand", 1) for it in inst["items"]}
    got = {}
    for p in placed:
        got[p["item_id"]] = got.get(p["item_id"], 0) + 1
    missing = {k: v - got.get(k, 0) for k, v in demand.items() if v - got.get(k, 0) > 0}
    if missing:
        raise SystemExit(f"layout does not place every item: missing {missing}")
    inst["solution"] = {
        "strip_width": float(sol["strip_width"]),
        "layout": {
            "container_id": int(lay.get("container_id", 0)),
            "placed_items": [
                {
                    "item_id": int(p["item_id"]),
                    "transformation": {
                        "rotation": float(p["transformation"]["rotation"]),
                        "translation": [float(v) for v in p["transformation"]["translation"]],
                    },
                }
                for p in placed
            ],
            "density": float(lay.get("density", sol.get("density", 0.0))),
        },
        "density": float(sol.get("density", 0.0)),
        "run_time_sec": 0,
    }
    json.dump(inst, open(out, "w", encoding="utf-8"), ensure_ascii=False)
    print(f"{out}: {len(placed)} items, strip_width {sol['strip_width']:.2f}")


main()
