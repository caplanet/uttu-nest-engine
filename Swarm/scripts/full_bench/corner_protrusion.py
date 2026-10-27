"""How far past the fabric edge can an original outline reach, given the marker's corners?

Swarm insets the container by overlap/2 (swarm-core/src/util/io.rs apply_overlap_strip_offset),
so the *eroded* collision shape stays inside. The original outline does not: eroding a polygon
by r pulls a convex vertex inward by r/sin(theta/2), not r, so when the eroded shape touches the
inset boundary the original vertex protrudes by

    r * (1/sin(theta/2) - 1),  r = overlap/2

which grows without bound as the corner sharpens. Nothing in the codebase checks original
outlines against the container, so this is the quantity that decides whether a given overlap
setting can honour an "outside the fabric" budget.

This reports, for the marker's actual geometry, the sharpest convex corner and the resulting
worst-case protrusion at several overlap settings — and the largest overlap that fits a budget.

  python corner_protrusion.py --marker <input.json> [--budget 0.1]
"""
import argparse
import json
import math


def corners(marker_path):
    """Interior angle at every convex vertex of every piece, in degrees."""
    marker = json.loads(open(marker_path, encoding="utf-8").read())
    out = []
    for item in marker["items"]:
        data = item.get("shape", {}).get("data")
        if not data:
            continue
        pts = [(p[0], p[1]) for p in data]
        if len(pts) > 1 and pts[0] == pts[-1]:
            pts = pts[:-1]
        n = len(pts)
        if n < 3:
            continue
        # Positive shoelace means counter-clockwise; convexity test needs the winding.
        area2 = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1] for i in range(n))
        ccw = area2 > 0
        for i in range(n):
            ax, ay = pts[(i - 1) % n]
            bx, by = pts[i]
            cx, cy = pts[(i + 1) % n]
            ux, uy = ax - bx, ay - by
            vx, vy = cx - bx, cy - by
            lu = math.hypot(ux, uy)
            lv = math.hypot(vx, vy)
            if lu < 1e-9 or lv < 1e-9:
                continue
            cosang = max(-1.0, min(1.0, (ux * vx + uy * vy) / (lu * lv)))
            interior = math.degrees(math.acos(cosang))
            cross = ux * vy - uy * vx
            # Interior angle < 180 (convex) when the turn matches the winding.
            convex = (cross < 0) if ccw else (cross > 0)
            if convex:
                out.append((interior, item["id"], i, (bx, by)))
    return out


def protrusion(theta_deg, r):
    s = math.sin(math.radians(theta_deg) / 2.0)
    return r * (1.0 / s - 1.0) if s > 1e-9 else float("inf")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker", required=True)
    ap.add_argument("--budget", type=float, default=0.1, help="allowed protrusion past the fabric edge, mm")
    ap.add_argument("--levels", type=float, nargs="+", default=[0.8, 1.0, 1.2, 1.6, 2.0])
    a = ap.parse_args()

    cs = corners(a.marker)
    cs.sort()
    print(f"볼록 꼭짓점 {len(cs):,}개")
    print("가장 날카로운 10개:")
    for ang, item_id, idx, (x, y) in cs[:10]:
        print(f"   {ang:7.3f}°  item {item_id} 정점 {idx}  ({x:.2f}, {y:.2f})")

    sharpest = cs[0][0]
    print(f"\n최소 꼭짓각 {sharpest:.3f}° (item {cs[0][1]})")
    print(f"\n{'겹침 d':>8}{'r=d/2':>8}{'최악 초과량':>13}{'예산 ' + format(a.budget, '.2f') + 'mm':>14}")
    for d in a.levels:
        p = protrusion(sharpest, d / 2.0)
        print(f"{d:>8.2f}{d / 2.0:>8.2f}{p:>12.3f}mm{'통과' if p <= a.budget else '초과':>14}")

    # Largest overlap whose worst-case protrusion fits the budget.
    k = 1.0 / math.sin(math.radians(sharpest) / 2.0) - 1.0
    if k > 1e-9:
        d_max = 2.0 * a.budget / k
        print(f"\n예산 {a.budget:.2f}mm 를 지키는 최대 겹침 허용: {d_max:.4f}mm")
        print(f"현재 0.8mm 를 지키려면 컨테이너를 추가로 {protrusion(sharpest, 0.4) - a.budget:.3f}mm 더 줄여야 한다")

    # How many corners actually matter: only those that can reach a wall.
    for d in a.levels:
        r = d / 2.0
        n_over = sum(1 for ang, *_ in cs if protrusion(ang, r) > a.budget)
        print(f"   겹침 {d:.1f}mm 에서 예산을 넘길 수 있는 꼭짓점: {n_over:,}개 ({n_over / len(cs) * 100:.1f}%)")


if __name__ == "__main__":
    raise SystemExit(main())
