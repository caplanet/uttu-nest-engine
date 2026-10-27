#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""swarm_output_to_plt.py -- Swarm/jagua-rs 배치 결과(JSON)를 HPGL(.plt) 마커로 되돌린다.

입력
    * 배치 결과 : {"strip_width", "layout": {"placed_items":[{item_id, transformation:{rotation, translation}}]}}
    * 인스턴스  : 그 결과가 참조하는 items[*].shape (bbox 최소가 (0,0) 으로 정규화된 도형)
    * 원본 PLT  : (선택) 조각 안쪽 주석 -- 결방향선/너치/라벨 텍스트 -- 를 그대로 옮겨오기 위한 출처

좌표 규약(실측으로 확정, plt_to_swarm.py 와 같은 근거)
    * 40 PLU = 1 mm 정수 스케일.
    * 인스턴스 도형은 플롯된 마커를 **180도 회전**한 프레임이다. 그래서 마커 좌표로
      되돌릴 때 (L - tx - u, H - ty - v) 로 뒤집는다. 회전이므로 감김/좌우 손잡이가
      보존된다. 거울은 절대 쓰지 않는다.
    * 결과가 rotation != 0 인 아이템을 담고 있으면 회전도 함께 적용한다.

출력 바이트 규약은 원본 PLT 와 같다: ASCII, LF 개행, 파일 끝 개행 있음,
`PU;` + `SP1;` 이 그리기 그룹 경계.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plt_to_swarm as P   # noqa: E402  (PLT 파서/조각 추출 재사용)

PLU_PER_MM = 40
PLOT_MARGIN_PLU = (723, 842)   # 원본 마커의 테두리 최소 모서리와 같은 플롯 여백


def log(msg):
    sys.stderr.write("[swarm2plt] " + msg + "\n")
    sys.stderr.flush()


# ---------------------------------------------------------------------------
# 원본 PLT 에서 조각별 주석 스트로크를 뽑아온다
# ---------------------------------------------------------------------------

def harvest_annotations(plt_path):
    """조각 폴리라인의 로컬 좌표와, 그 안에 들어있는 주석 스트로크를 돌려준다.

    반환: rings[plu_index] = 조각 링(PLU),
          anns[plu_index]  = [[(x,y),...], ...]  조각 bbox 최소 모서리 기준 로컬 PLU

    귀속 규칙: 주석 그룹의 스트로크 길이 중 그 조각 폴리곤 안에 들어가는 비율이
    가장 큰 조각에 붙이되, 비율이 0.5 미만이면 버린다(마커 전체 안내문 등).
    """
    from shapely.geometry import Polygon, LineString, MultiLineString
    from shapely.strtree import STRtree

    polylines, groups = P.parse_plt(plt_path)
    border, pieces = P.extract_pieces(polylines, groups)

    polys, keys = [], []
    for idx, ring, _a in pieces:
        g = Polygon(ring)
        polys.append(g if g.is_valid else g.buffer(0))
        keys.append(idx)
    tree = STRtree(polys)

    piece_pl = set(i for i, _, _ in pieces)
    gi_of = {}
    for gi, g in enumerate(groups):
        for pi in g:
            gi_of[pi] = gi
    border_gi = gi_of[border[0]]

    anns = {k: [] for k in keys}
    dropped = 0
    for gi, g in enumerate(groups):
        if gi == border_gi:
            continue
        if len(g) == 1 and g[0] in piece_pl:
            continue                      # 조각 자기 자신
        strokes = [polylines[pi] for pi in g if len(polylines[pi]) >= 2]
        if not strokes:
            continue
        geom = MultiLineString([LineString(s) for s in strokes])
        total = geom.length
        if total <= 0:
            continue
        best_k, best_f = None, 0.0
        for k in tree.query(geom):
            f = geom.intersection(polys[k]).length / total
            if f > best_f:
                best_k, best_f = k, f
        if best_k is None or best_f < 0.5:
            dropped += 1
            continue
        anns[keys[best_k]].extend(strokes)

    rings = dict((idx, ring) for idx, ring, _a in pieces)
    n = sum(len(v) for v in anns.values())
    log("annotations: %d strokes attached to pieces, %d groups dropped (marker-level text)"
        % (n, dropped))
    return rings, anns, pieces


def match_pieces_to_items(pieces, inst_items):
    """PLT 조각 <-> 인스턴스 item id 대응. plt_to_swarm 과 같은 도형 매칭을 쓴다."""
    shapes = []
    for idx, ring, _a in pieces:
        pts = P.transform_ring(ring)
        mnx, mny, mxx, mxy = P.bbox(pts)
        shapes.append({"pts": pts, "segs": P._ring_segments(pts),
                       "area": abs(P.signed_area_f(pts)), "w": mxx - mnx, "h": mxy - mny,
                       "plu_index": idx})
    shapes.sort(key=lambda s: (-s["area"], s["plu_index"]))
    cands = P.build_candidates(shapes, inst_items)
    match_ref, matched = P.perfect_matching(cands, len(inst_items))
    log("piece<->item matching: %d/%d" % (matched, len(shapes)))
    if matched != len(shapes):
        log("WARN: not a perfect matching; annotations may be attached to the wrong item")
    out = {}
    for ri, pi in enumerate(match_ref):
        if pi >= 0:
            out[inst_items[ri]["id"]] = shapes[pi]["plu_index"]
    return out


# ---------------------------------------------------------------------------
# HPGL 출력
# ---------------------------------------------------------------------------

def emit(path, groups_plu, extent_mm):
    """groups_plu: [[(x,y),...], ...] 를 그리기 그룹 단위로 받는다(각 그룹 = 폴리라인 리스트)."""
    out = []
    out.append("NE%d,%d;" % extent_mm)
    out.append("IN;")
    for i in range(1, 9):
        out.append("VS32,%d;" % i)
    out.append("WU0;")
    for i in range(1, 9):
        out.append("PW0.350,%d;" % i)
    for grp in groups_plu:
        out.append("PU;")
        out.append("SP1;")
        for stroke in grp:
            if len(stroke) < 2:
                continue
            out.append("PU%d,%d;" % stroke[0])
            for pt in stroke[1:]:
                out.append("PD%d,%d;" % pt)
    text = "\n".join(out) + "\n"
    with open(path, "wb") as f:
        f.write(text.encode("ascii"))
    return len(text)


def main():
    ap = argparse.ArgumentParser(description="Swarm layout JSON -> HPGL .plt marker")
    ap.add_argument("--result", required=True)
    ap.add_argument("--instance", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--source-plt", default=None,
                    help="주석(라벨/결방향/너치)을 옮겨올 원본 마커 PLT. 주지 않으면 "
                         "조각 윤곽 + 테두리만 나온다(배치 결과에 대한 순수 변환).")
    ap.add_argument("--border", choices=["strip", "content"], default="strip",
                    help="테두리를 솔버가 보고한 strip_width 로 그릴지(strip), "
                         "실제 배치 범위로 그릴지(content)")
    args = ap.parse_args()

    res = json.load(open(args.result, encoding="utf-8"))
    inst = json.load(open(args.instance, encoding="utf-8"))
    H = float(inst["strip_height"])
    placed = res["layout"]["placed_items"]
    shp = {it["id"]: [(float(x), float(y)) for x, y in it["shape"]["data"]]
           for it in inst["items"]}
    log("result: %d placed items, strip_width=%.4f, density=%.6f"
        % (len(placed), res["strip_width"], res.get("density", float("nan"))))

    # --- 각 아이템의 마커 좌표(인스턴스 프레임) 도형 ---
    world = {}
    rots = set()
    for p in placed:
        i = p["item_id"]
        tr = p["transformation"]
        rot = float(tr.get("rotation", 0.0))
        rots.add(rot)
        tx, ty = (float(v) for v in tr["translation"])
        pts = shp[i]
        if rot:
            c, s = math.cos(rot), math.sin(rot)
            pts = [(x * c - y * s, x * s + y * c) for x, y in pts]
            mnx = min(q[0] for q in pts)
            mny = min(q[1] for q in pts)
            pts = [(x - mnx, y - mny) for x, y in pts]
        world[i] = ([(x + tx, y + ty) for x, y in pts], tx, ty,
                    max(q[0] for q in pts), max(q[1] for q in pts))
    log("rotations present: %s" % sorted(rots))

    cx1 = max(max(q[0] for q in w[0]) for w in world.values())
    cx0 = min(min(q[0] for q in w[0]) for w in world.values())
    cy1 = max(max(q[1] for q in w[0]) for w in world.values())
    cy0 = min(min(q[1] for q in w[0]) for w in world.values())
    log("content bbox: x=[%.4f, %.4f] y=[%.4f, %.4f]" % (cx0, cx1, cy0, cy1))
    L = float(res["strip_width"]) if args.border == "strip" else cx1
    log("border length L = %.4f mm (mode=%s), height = %.3f mm" % (L, args.border, H))

    # --- 주석 수확 + 조각 대응 ---
    anns_local = {}
    if args.source_plt:
        rings, anns, pieces = harvest_annotations(args.source_plt)
        id2plu = match_pieces_to_items(pieces, inst["items"])
        for item_id, plu_idx in id2plu.items():
            ring = rings[plu_idx]
            bx = min(q[0] for q in ring)
            by = min(q[1] for q in ring)
            anns_local[item_id] = [[(x - bx, y - by) for x, y in s] for s in anns[plu_idx]]

    # --- PLT 좌표로 변환 ---
    OX, OY = PLOT_MARGIN_PLU
    groups = []
    dev_max = 0.0
    for i in sorted(world):
        pts, tx, ty, w, h = world[i]
        # 인스턴스 프레임 -> 마커(PLT) 프레임: 마커 전체를 180도 회전
        ring = [(int(round((L - x) * PLU_PER_MM)) + OX,
                 int(round((H - y) * PLU_PER_MM)) + OY) for x, y in pts]
        ring.append(ring[0])                      # 닫는다(원본 PLT 규약)
        groups.append([ring])
        # 주석: 조각 bbox 최소 모서리에 맞춰 얹는다. 조각당 한 그룹으로 묶어
        # 원본과 비슷한 그룹 구조를 유지한다.
        if anns_local.get(i):
            ax0 = int(round((L - tx - w) * PLU_PER_MM)) + OX
            ay0 = int(round((H - ty - h) * PLU_PER_MM)) + OY
            groups.append([[(ax0 + x, ay0 + y) for x, y in s] for s in anns_local[i]])

    # 테두리(마커 사각형) -- 원본과 같이 마지막 그룹
    bx1 = int(round(L * PLU_PER_MM)) + OX
    by1 = int(round(H * PLU_PER_MM)) + OY
    groups.append([[(OX, OY), (bx1, OY), (bx1, by1), (OX, by1), (OX, OY)]])

    allx = [p[0] for g in groups for s in g for p in s]
    ally = [p[1] for g in groups for s in g for p in s]
    extent = (int(math.ceil(max(allx) / float(PLU_PER_MM))),
              int(math.ceil(max(ally) / float(PLU_PER_MM))))
    if min(allx) < 0 or min(ally) < 0:
        log("WARN: negative plotter coordinates (min x=%d, y=%d) -- content overruns the border"
            % (min(allx), min(ally)))

    size = emit(args.output, groups, extent)
    log("wrote %s (%d bytes, %d groups, %d strokes, NE%d,%d)"
        % (os.path.basename(args.output), size, len(groups),
           sum(len(g) for g in groups), extent[0], extent[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
