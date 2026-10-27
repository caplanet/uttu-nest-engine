"""Draw a produced layout as an SVG, so it can be compared against a reference nester's screenshots.

The numbers say how dense a layout is; they do not say what it looks like. Structural differences —
whether pieces hug both fabric edges, whether small pieces sit inside large pieces' concavities,
whether the frontier advances as a band — are visible in a picture and invisible in a width.

  python render_layout.py --marker <input.json> --solution <Init.json> --out <file.svg>
  python render_layout.py ... --first 50     # only the N pieces nearest the left edge
"""
import argparse
import json
import math

PALETTE = ["#7fd8e8", "#8fb4e8", "#f0b49c", "#e8917a", "#a8d8b0", "#c9a8e0", "#e8d08f", "#9fc9d8"]


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
        out.append((pi["item_id"], pts))
    return out, float(solution["strip_width"]), float(marker["strip_height"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker", required=True)
    ap.add_argument("--solution", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--first", type=int, default=0, help="draw only the N leftmost pieces")
    ap.add_argument("--width-px", type=int, default=1600)
    # The reference nester's screenshots are mirrored across the Y axis, so its
    # left edge is this data's right edge. Top and bottom are unchanged.
    ap.add_argument("--mirror-x", action="store_true")
    a = ap.parse_args()

    pieces, W, H = placed(a.marker, a.solution)
    if a.first > 0:
        pieces.sort(key=lambda p: min(x for x, _ in p[1]))
        pieces = pieces[:a.first]
        W = max(max(x for x, _ in pts) for _, pts in pieces) * 1.05

    scale = a.width_px / W
    h_px = H * scale

    def ring(pts):
        # SVG y grows downward, so (H - y) draws y=0 at the bottom, as the data means it.
        return " ".join(
            f"{(W - x if a.mirror_x else x) * scale:.2f},{(H - y) * scale:.2f}" for x, y in pts
        )

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {a.width_px} {h_px:.1f}" width="{a.width_px}">',
        f'<rect width="{a.width_px}" height="{h_px:.1f}" fill="#ffffff"/>',
        f'<rect x="0" y="0" width="{a.width_px}" height="{h_px:.1f}" fill="none" stroke="#1a237e" stroke-width="2"/>',
    ]
    for idx, (item_id, pts) in enumerate(pieces):
        fill = PALETTE[item_id % len(PALETTE)]
        parts.append(f'<polygon points="{ring(pts)}" fill="{fill}" fill-opacity="0.85" '
                     f'stroke="#123" stroke-width="0.7"/>')
    parts.append("</svg>")
    open(a.out, "w", encoding="utf-8").write("\n".join(parts))

    area = 0.0
    for _, pts in pieces:
        n = len(pts)
        area += abs(sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
                        for i in range(n))) / 2.0
    print(f"{len(pieces)}개 조각, 스트립 {W:.1f} x {H:.1f}, 점유 {area / (W * H) * 100:.2f}% -> {a.out}")


if __name__ == "__main__":
    raise SystemExit(main())
