"""How much of the overlap allowance does a produced layout actually consume?

`piece_overlap_mm = d` deflates every piece by d/2, so the engine is free to let two
original outlines interpenetrate by up to d. Whether it *does* is a different question,
and it decides whether raising d can buy anything: if the search only ever uses a
fraction of the budget, a larger budget changes nothing.

For each overlapping pair this reports the true penetration depth, defined the same way
swarm-core/src/original_overlap.rs defines acceptance — the intersection survives
erosion by h exactly when its largest inscribed circle has radius > h, so the depth the
guard bounds is 2·r_max. r_max is found by bisection on the eroded area.

Also reports how far original outlines cross the strip rectangle, which the guard does
not check at all.

  python penetration_depth.py --marker <input.json> --solution <Polish.json> [--overlap 0.8]
"""
import argparse
import json
import math

from shapely.geometry import Polygon
from shapely.strtree import STRtree

GRID_GUARD_MM = 0.0005


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
        poly = Polygon([(x * cos_r - y * sin_r + tx, x * sin_r + y * cos_r + ty)
                        for x, y in shapes[pi["item_id"]]])
        if not poly.is_valid:
            poly = poly.buffer(0)
        out.append((pi["item_id"], poly))
    return out, float(solution["strip_width"]), float(marker["strip_height"])


def max_inscribed_radius(geom, hi):
    """Largest r with geom.buffer(-r) non-empty, by bisection."""
    if geom.is_empty or geom.area <= 0.0:
        return 0.0
    lo = 0.0
    if not geom.buffer(-hi, join_style=3, mitre_limit=2.0).is_empty:
        return hi
    for _ in range(24):
        mid = (lo + hi) / 2.0
        if geom.buffer(-mid, join_style=3, mitre_limit=2.0).is_empty:
            hi = mid
        else:
            lo = mid
    return lo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker", required=True)
    ap.add_argument("--solution", required=True)
    ap.add_argument("--overlap", type=float, default=0.8)
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    polys, width, height = placed(a.marker, a.solution)
    geoms = [p for _, p in polys]
    tree = STRtree(geoms)
    # Search a little past the nominal budget so a violation is visible, not clipped.
    ceiling = a.overlap

    depths = []
    for i, (_, poly) in enumerate(polys):
        for j in tree.query(poly):
            if j <= i:
                continue
            common = poly.intersection(geoms[j])
            if common.is_empty or common.area <= 1.0e-9:
                continue
            r = max_inscribed_radius(common, ceiling)
            depths.append((2.0 * r, common.area, polys[i][0], polys[j][0]))

    depths.sort(reverse=True)
    budget = a.overlap
    used = depths[0][0] if depths else 0.0
    label = a.label or a.solution
    print(f"\n== {label}")
    print(f"   조각 {len(polys)}개, 스트립 {width:.2f} x {height:.2f}, 허용 겹침 {budget:.2f}mm")
    print(f"   접촉 쌍 {len(depths)}개, 최대 침투 {used:.4f}mm (허용치의 {used / budget * 100:.1f}%)")
    for k in (0.75, 0.5, 0.25):
        n = sum(1 for d, *_ in depths if d >= budget * k)
        print(f"   허용치의 {k * 100:.0f}% 이상 쓰는 쌍: {n}개")
    for d, area, x, y in depths[:5]:
        print(f"      items {x}/{y}: 침투 {d:.4f}mm, 교집합 {area:.3f} mm²")

    mnx = min(p.bounds[0] for _, p in polys)
    mny = min(p.bounds[1] for _, p in polys)
    mxx = max(p.bounds[2] for _, p in polys)
    mxy = max(p.bounds[3] for _, p in polys)
    print(f"   원단 이탈 — 좌 {max(0.0, -mnx):.4f}  우 {max(0.0, mxx - width):.4f}  "
          f"하 {max(0.0, -mny):.4f}  상 {max(0.0, mxy - height):.4f} (mm)")


if __name__ == "__main__":
    raise SystemExit(main())
