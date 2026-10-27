"""How far can piece_overlap_mm be raised before the marker's own shapes break?

An overlap allowance of `d` is implemented by deflating every piece by `d/2`
(swarm-core/src/util/io.rs passes -d as jagua-rs `min_item_separation`). A polygon
offset inward by `d/2` loses any feature thinner than `d`, and a whole piece narrower
than `d` disappears. Once a feature is gone from the collision shape, two pieces can
overlap there by far more than `d` and nothing in the pipeline will object.

So the ceiling is set by the marker, not by a policy number. This reports, per piece:
the deflation at which it first loses area faster than the perimeter predicts, at which
it splits into pieces, and at which it vanishes.

  python feature_width.py --marker <input.json> [--levels 0.8 1.2 1.6 2.0 3.0]
"""
import argparse
import json

from shapely.geometry import Polygon
from shapely.geometry.base import BaseMultipartGeometry


def pieces(marker_path):
    marker = json.loads(open(marker_path, encoding="utf-8").read())
    out = []
    for item in marker["items"]:
        data = item.get("shape", {}).get("data")
        if not data:
            continue
        poly = Polygon([(p[0], p[1]) for p in data])
        if not poly.is_valid:
            poly = poly.buffer(0)
        demand = int(item.get("demand", 1) or 1)
        out.append((item["id"], demand, poly))
    return out, float(marker["strip_height"])


def deflate(poly, d):
    # Bevel joins with the same mitre limit the Rust guard uses, so the result
    # matches what the engine's collision shape would become.
    return poly.buffer(-d, join_style=3, mitre_limit=2.0)


def parts(geom):
    if geom.is_empty:
        return 0
    if isinstance(geom, BaseMultipartGeometry):
        return len(geom.geoms)
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker", required=True)
    ap.add_argument("--levels", type=float, nargs="+", default=[0.8, 1.0, 1.2, 1.6, 2.0, 3.0, 4.0])
    a = ap.parse_args()

    items, height = pieces(a.marker)
    total_demand = sum(d for _, d, _ in items)
    total_area = sum(d * p.area for _, d, p in items)
    print(f"조각 종류 {len(items)}개, 배치 수 {total_demand}개, 원단 폭(높이) {height:.1f}mm")
    print(f"입력 면적 합계 {total_area:,.0f} mm²\n")

    print(f"{'겹침 d':>8}{'깎임 d/2':>10}{'소멸':>7}{'분할':>7}{'면적손실':>11}{'최소 폭 조각':>14}")
    for d in a.levels:
        half = d / 2.0
        gone = split = 0
        lost = 0.0
        worst = None
        for item_id, demand, poly in items:
            small = deflate(poly, half)
            n = parts(small)
            if n == 0:
                gone += demand
            elif n > 1:
                split += demand
            lost += demand * (poly.area - (0.0 if small.is_empty else small.area))
            # A piece whose inradius is below half vanishes; track the tightest margin.
            margin = 0.0 if small.is_empty else small.buffer(0).area / max(poly.area, 1e-9)
            if worst is None or margin < worst[1]:
                worst = (item_id, margin)
        print(f"{d:>8.2f}{half:>10.2f}{gone:>7}{split:>7}{lost / total_area * 100:>10.2f}%"
              f"{str(worst[0]) + ' (' + format(worst[1] * 100, '.1f') + '%)':>14}")

    # The binding number: the largest deflation every piece survives intact.
    lo, hi = 0.0, 20.0
    for _ in range(40):
        mid = (lo + hi) / 2.0
        ok = all(parts(deflate(p, mid)) == 1 for _, _, p in items)
        lo, hi = (mid, hi) if ok else (lo, mid)
    print(f"\n모든 조각이 하나의 온전한 링으로 살아남는 최대 깎임: {lo:.3f}mm")
    print(f"→ 그에 해당하는 겹침 허용 상한: {lo * 2:.3f}mm")


if __name__ == "__main__":
    raise SystemExit(main())
