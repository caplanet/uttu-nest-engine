"""Where does the wasted area sit? Rasterised free-space comparison of two layouts.

The four structural metrics in competitor_structure.rs (order signature, contact length,
same-shape adjacency, column runs) do not explain the width gap, so this looks at the
waste itself: how much, where along x, and in how large a piece.

  python waste_profile.py --marker <input.json> --ours <Polish.json> --competitor <ref.json>
"""
import argparse
import json
import math
import numpy as np


def polys_from_marker_solution(marker_path, solution_path):
    """Place each item's *input* polygon by the solution's rotation+translation."""
    m = json.loads(open(marker_path, encoding="utf-8").read())
    shapes = {}
    for it in m["items"]:
        d = it.get("shape", {}).get("data")
        if d:
            shapes[it["id"]] = [(p[0], p[1]) for p in d]
    sol = json.loads(open(solution_path, encoding="utf-8").read())
    out = []
    for pi in sol["layout"]["placed_items"]:
        pts = shapes[pi["item_id"]]
        tr = pi["transformation"]
        r = math.radians(tr["rotation"])
        c, s = math.cos(r), math.sin(r)
        tx, ty = tr["translation"]
        out.append([(x * c - y * s + tx, x * s + y * c + ty) for x, y in pts])
    return out, float(sol["strip_width"]), m["strip_height"]


def polys_from_competitor(path, strip_height):
    d = json.loads(open(path, encoding="utf-8").read())
    raw = [[(p[0], p[1]) for p in poly] for poly in d["polygons"]]
    xs = [p[0] for poly in raw for p in poly]
    ys = [p[1] for poly in raw for p in poly]
    min_x, min_y, max_x = min(xs), min(ys), max(xs)
    length = max_x - min_x
    # File notes record a global mirror; competitor_structure.rs picks "both" for this marker.
    return [[(length - (x - min_x), strip_height - (y - min_y)) for x, y in poly] for poly in raw], length


def rasterise(polys, width, height, cell):
    nx, ny = int(width / cell) + 1, int(height / cell) + 1
    grid = np.zeros((ny, nx), dtype=bool)
    ys = (np.arange(ny) + 0.5) * cell
    for poly in polys:
        n = len(poly)
        px = np.array([p[0] for p in poly])
        py = np.array([p[1] for p in poly])
        lo = max(0, int(py.min() / cell) - 1)
        hi = min(ny, int(py.max() / cell) + 2)
        for iy in range(lo, hi):
            yc = ys[iy]
            xs_hit = []
            for i in range(n):
                x0, y0 = px[i], py[i]
                x1, y1 = px[(i + 1) % n], py[(i + 1) % n]
                if (y0 <= yc) != (y1 <= yc):
                    xs_hit.append(x0 + (yc - y0) * (x1 - x0) / (y1 - y0))
            if not xs_hit:
                continue
            xs_hit.sort()
            for a, b in zip(xs_hit[0::2], xs_hit[1::2]):
                ia, ib = int(a / cell) + 1, int(b / cell)
                if ib >= ia:
                    grid[iy, max(0, ia):min(nx, ib + 1)] = True
    return grid


def components(free):
    """Connected free-space components (4-neighbour), returned as cell counts."""
    ny, nx = free.shape
    seen = np.zeros_like(free)
    sizes = []
    for sy in range(ny):
        for sx in range(nx):
            if not free[sy, sx] or seen[sy, sx]:
                continue
            stack = [(sy, sx)]
            seen[sy, sx] = True
            n = 0
            while stack:
                y, x = stack.pop()
                n += 1
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny2, nx2 = y + dy, x + dx
                    if 0 <= ny2 < ny and 0 <= nx2 < nx and free[ny2, nx2] and not seen[ny2, nx2]:
                        seen[ny2, nx2] = True
                        stack.append((ny2, nx2))
            sizes.append(n)
    return sorted(sizes, reverse=True)


def report(name, polys, width, height, cell):
    grid = rasterise(polys, width, height, cell)
    area_cell = cell * cell
    occupied = grid.sum() * area_cell
    total = width * height
    free = ~grid
    # Column occupancy along x.
    col = grid.mean(axis=0)
    nx = grid.shape[1]
    bands = 10
    per = [col[int(i * nx / bands):int((i + 1) * nx / bands)].mean() * 100 for i in range(bands)]
    sizes = components(free)
    big = [s * area_cell for s in sizes[:5]]
    free_area = total - occupied
    print(f"\n== {name}  폭 {width:.1f}mm  격자 {cell}mm")
    print(f"   점유 {occupied / total * 100:.2f}%  낭비 {free_area:,.0f} mm²")
    print("   x 10구간 점유율: " + " ".join(f"{v:.0f}" for v in per))
    print(f"   자유공간 덩어리 {len(sizes)}개, 상위5 면적: " + " ".join(f"{a:,.0f}" for a in big))
    print(f"   상위5가 전체 낭비의 {sum(big) / max(free_area, 1) * 100:.1f}%")
    return per, sizes, free_area


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker", required=True)
    ap.add_argument("--ours", required=True)
    ap.add_argument("--competitor", required=True)
    ap.add_argument("--cell", type=float, default=6.0)
    a = ap.parse_args()

    ours, our_w, h = polys_from_marker_solution(a.marker, a.ours)
    comp, comp_w = polys_from_competitor(a.competitor, h)
    report("상용 최종", comp, comp_w, h, a.cell)
    report("우리 최종", ours, our_w, h, a.cell)


if __name__ == "__main__":
    main()
