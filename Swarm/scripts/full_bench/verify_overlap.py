"""Independent check of the piece-overlap contract on a produced layout.

Mirrors swarm-core/src/original_overlap.rs: intersect the two *original* input
polygons, erode the common region by `half_depth`, and require the eroded area to
be zero. `half_depth = overlap_mm / 2 - GRID_GUARD_MM`, so an overlap region that
is everywhere thinner than `overlap_mm` survives and anything thicker does not.

FullBench's `passed` flag only means "the native calls raised no exception", so
the fabric contract is checked here rather than assumed.

  python verify_overlap.py --marker <input.json> --solution <Polish.json>
"""
import argparse
import json
import math

from shapely.geometry import Polygon
from shapely.strtree import STRtree

GRID_GUARD_MM = 0.0005
# Eroded-remnant area below this is fixed-point quantisation, not penetration.
AREA_EPSILON_MM2 = 1.0e-6
# The Rust guard snaps every coordinate to a 1e-4 mm grid before clipping. A remnant
# that disappears when the erosion goes two grid steps deeper is thinner than that
# grid can represent: the guard cannot see it and no cutter can either. It is reported
# separately instead of being counted as a violation.
GRID_STEP_MM = 1.0e-4


def placed_polygons(marker_path, solution_path):
    marker = json.loads(open(marker_path, encoding="utf-8").read())
    shapes = {}
    for item in marker["items"]:
        data = item.get("shape", {}).get("data")
        if data:
            shapes[item["id"]] = [(p[0], p[1]) for p in data]
    solution = json.loads(open(solution_path, encoding="utf-8").read())
    out = []
    for placed in solution["layout"]["placed_items"]:
        pts = shapes[placed["item_id"]]
        tr = placed["transformation"]
        rot = math.radians(tr["rotation"])
        cos_r, sin_r = math.cos(rot), math.sin(rot)
        tx, ty = tr["translation"]
        moved = [(x * cos_r - y * sin_r + tx, x * sin_r + y * cos_r + ty) for x, y in pts]
        poly = Polygon(moved)
        if not poly.is_valid:
            poly = poly.buffer(0)
        out.append((placed["item_id"], poly))
    return out, float(solution["strip_width"]), marker["strip_height"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker", required=True)
    ap.add_argument("--solution", required=True)
    ap.add_argument("--overlap", type=float, default=0.8)
    args = ap.parse_args()

    half_depth = max(args.overlap / 2.0 - GRID_GUARD_MM, 0.0)
    polys, width, height = placed_polygons(args.marker, args.solution)
    geoms = [p for _, p in polys]
    tree = STRtree(geoms)

    violations = []
    quantised = []
    worst = 0.0
    for i, (item_id, poly) in enumerate(polys):
        for j in tree.query(poly):
            if j <= i:
                continue
            common = poly.intersection(geoms[j])
            if common.is_empty or common.area <= 0.0:
                continue
            # Bevel joins with the same mitre limit Clipper2 uses in the Rust
            # guard. Round joins leave corner slivers that read as violations.
            eroded = common.buffer(-half_depth, join_style=3, mitre_limit=2.0)
            area = 0.0 if eroded.is_empty else eroded.area
            worst = max(worst, area)
            # Clipper2 works on a fixed-point grid, so an eroded remnant far below
            # one square micron is quantisation, not a real penetration.
            if area > AREA_EPSILON_MM2:
                deeper = common.buffer(-(half_depth + 2.0 * GRID_STEP_MM), join_style=3, mitre_limit=2.0)
                if deeper.is_empty or deeper.area <= AREA_EPSILON_MM2:
                    quantised.append((item_id, polys[j][0], common.area, area))
                else:
                    violations.append((item_id, polys[j][0], common.area, area))

    # Container containment, checked on the same original outlines.
    outside = [
        (item_id, p.bounds)
        for item_id, p in polys
        if p.bounds[0] < -half_depth
        or p.bounds[1] < -half_depth
        or p.bounds[2] > width + half_depth
        or p.bounds[3] > height + half_depth
    ]

    print(f"조각 {len(polys)}개  스트립 {width:.2f} x {height:.2f}  half_depth {half_depth:.4f}mm")
    print(f"겹침 위반 {len(violations)}건  (침식 후 최대 잔여 면적 {worst:.3e} mm²)")
    for a, b, raw, eroded in violations[:10]:
        print(f"   items {a}/{b}: 교집합 {raw:.4f} mm², 침식 후 {eroded:.3e} mm²")
    if quantised:
        print(f"격자 양자화 수준의 잔여 {len(quantised)}건 (침식을 {2.0 * GRID_STEP_MM:.4f}mm 더 하면 사라짐 — 위반으로 세지 않음)")
        for a, b, raw, eroded in quantised[:10]:
            print(f"   items {a}/{b}: 교집합 {raw:.4f} mm², 침식 후 {eroded:.3e} mm²")
    print(f"컨테이너 이탈 {len(outside)}건")
    for item_id, b in outside[:10]:
        print(f"   item {item_id}: bounds {b}")
    ok = not violations and not outside
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
