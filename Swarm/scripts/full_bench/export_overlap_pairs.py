"""Export the deepest overlapping piece pairs so they can be checked in CAD.

Swarm's guard bounds penetration between the ORIGINAL input outlines, so if a downstream
viewer reports no overlap for the same layout, the two are not looking at the same
geometry. This writes the exact world coordinates Swarm's own numbers refer to, plus an
SVG of each pair, so the discrepancy can be settled on one concrete pair instead of in
the abstract.

  python export_overlap_pairs.py --marker <input.json> --solution <Polish.json> --out <dir>
"""
import argparse
import json
import math
import os

from shapely.geometry import Polygon
from shapely.strtree import STRtree


def placed(marker_path, solution_path):
    marker = json.loads(open(marker_path, encoding="utf-8").read())
    shapes = {}
    for item in marker["items"]:
        data = item.get("shape", {}).get("data")
        if data:
            shapes[item["id"]] = [(p[0], p[1]) for p in data]
    solution = json.loads(open(solution_path, encoding="utf-8").read())
    out = []
    for pi in solution["layout"]["placed_items"]:
        tr = pi["transformation"]
        rot = math.radians(tr["rotation"])
        cos_r, sin_r = math.cos(rot), math.sin(rot)
        tx, ty = tr["translation"]
        pts = [(x * cos_r - y * sin_r + tx, x * sin_r + y * cos_r + ty)
               for x, y in shapes[pi["item_id"]]]
        poly = Polygon(pts)
        if not poly.is_valid:
            poly = poly.buffer(0)
        out.append({"item_id": pi["item_id"], "rotation_deg": tr["rotation"],
                    "translation": tr["translation"], "points": pts, "poly": poly})
    return out, float(solution["strip_width"]), float(marker["strip_height"])


def depth_of(geom, hi):
    if geom.is_empty or geom.area <= 0.0:
        return 0.0
    lo = 0.0
    if not geom.buffer(-hi, join_style=3, mitre_limit=2.0).is_empty:
        return 2.0 * hi
    for _ in range(24):
        mid = (lo + hi) / 2.0
        if geom.buffer(-mid, join_style=3, mitre_limit=2.0).is_empty:
            hi = mid
        else:
            lo = mid
    return 2.0 * lo


def svg(path, a, b, common, pad=8.0):
    xs = [p[0] for p in a["points"]] + [p[0] for p in b["points"]]
    ys = [p[1] for p in a["points"]] + [p[1] for p in b["points"]]
    x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
    w, h = x1 - x0, y1 - y0

    def ring(points):
        return " ".join(f"{x - x0:.4f},{y1 - y:.4f}" for x, y in points)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.3f} {h:.3f}" width="900">',
        f'<rect width="{w:.3f}" height="{h:.3f}" fill="#fbfbfa"/>',
        f'<polygon points="{ring(a["points"])}" fill="#3b82f6" fill-opacity="0.18" stroke="#1d4ed8" stroke-width="{w / 900:.4f}"/>',
        f'<polygon points="{ring(b["points"])}" fill="#f97316" fill-opacity="0.18" stroke="#c2410c" stroke-width="{w / 900:.4f}"/>',
    ]
    geoms = [common] if common.geom_type == "Polygon" else list(getattr(common, "geoms", []))
    for g in geoms:
        if g.is_empty or g.geom_type != "Polygon":
            continue
        parts.append(f'<polygon points="{ring(list(g.exterior.coords))}" fill="#dc2626" fill-opacity="0.85" stroke="none"/>')
    parts.append("</svg>")
    open(path, "w", encoding="utf-8").write("\n".join(parts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker", required=True)
    ap.add_argument("--solution", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--overlap", type=float, default=0.8)
    ap.add_argument("--top", type=int, default=3)
    a = ap.parse_args()

    items, width, height = placed(a.marker, a.solution)
    geoms = [i["poly"] for i in items]
    tree = STRtree(geoms)
    pairs = []
    for i, it in enumerate(items):
        for j in tree.query(it["poly"]):
            if j <= i:
                continue
            common = it["poly"].intersection(geoms[j])
            if common.is_empty or common.area <= 1.0e-9:
                continue
            pairs.append((depth_of(common, a.overlap), common.area, i, int(j), common))
    pairs.sort(key=lambda t: (-t[1], -t[0]))

    os.makedirs(a.out, exist_ok=True)
    report = {"marker": a.marker, "solution": a.solution, "strip_width": width,
              "strip_height": height, "overlap_setting_mm": a.overlap, "pairs": []}
    for rank, (depth, area, i, j, common) in enumerate(pairs[:a.top], start=1):
        x, y = items[i], items[j]
        name = f"pair{rank}_items_{x['item_id']}_{y['item_id']}"
        svg(os.path.join(a.out, name + ".svg"), x, y, common)
        report["pairs"].append({
            "rank": rank, "penetration_mm": round(depth, 5), "intersection_area_mm2": round(area, 4),
            "svg": name + ".svg",
            "a": {"item_id": x["item_id"], "rotation_deg": x["rotation_deg"],
                  "translation": x["translation"], "points": [[round(p[0], 5), round(p[1], 5)] for p in x["points"]]},
            "b": {"item_id": y["item_id"], "rotation_deg": y["rotation_deg"],
                  "translation": y["translation"], "points": [[round(p[0], 5), round(p[1], 5)] for p in y["points"]]},
        })
        print(f"#{rank} items {x['item_id']}/{y['item_id']}: 침투 {depth:.4f}mm, 교집합 {area:.3f} mm² -> {name}.svg")
    path = os.path.join(a.out, "overlap_pairs.json")
    open(path, "w", encoding="utf-8").write(json.dumps(report, ensure_ascii=False, indent=1))
    print(f"좌표 원본: {path}")


if __name__ == "__main__":
    raise SystemExit(main())
