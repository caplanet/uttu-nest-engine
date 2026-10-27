# -*- coding: utf-8 -*-
"""PLT 단순화 오차 -> 겹침 판정용 보정량(mm) 산출.

배치는 PLT 유래 '단순화 폴리곤' 으로 하지만 실제로 재단되는 것은 원본(CAD) 윤곽이다.
PLT 폴리라인은 CAD 곡선을 정점 1/4 수준으로 간략화한 것이라 원본 윤곽이 폴리곤
바깥으로 삐져나오는 구간이 생긴다. 그 깊이가 곧 '원본끼리 안 겹치게 하려면 조각마다
몇 mm 를 부풀려야 하는가' 다.

  delta_out(i) = max_{q in CAD_i 경계} dist(q, PLT_i 영역)      # 영역 안이면 0

정렬 규약: 같은 조각의 CAD/PLT 윤곽을 bbox 최소 모서리로 맞춘다. placed_hint 의
tr_x/tr_y 가 bbox 최소 모서리 기준이므로 파이프라인이 실제로 쓰는 정렬과 같다.
(참고로 --best-align 을 주면 조각마다 최적 평행이동을 찾아 '순수 단순화 성분'만
남긴 값을 함께 보여준다. 정렬 편향이 얼마인지 보는 용도이지, 운영 수치는 bbox 쪽이다.)

배치기 쪽 대응:
  * jagua 의 poly_simpl_tolerance 는 아이템을 '부풀리는' 방향으로만 단순화하므로
    (config.rs: "maximum allowable inflation of items") 겹침을 새로 만들지 않는다.
  * min_item_separation 은 importer 가 절반씩 각 도형 오프셋으로 적용한다
    (offset = separation / 2). 따라서 필요한 조각별 오프셋 t 에 대해
    min_item_separation = 2 * t 로 주면 된다.

사용법:
    python simplification_margin.py [--best-align] [--overlaps]
"""
import argparse
import json
import math

import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.strtree import STRtree

CAD = r"C:\Caplanet\AiLabs\Test Data Files\여성WSjk_비교용-G_New_Swarm_신규2.json"
MARKERS = {
    "G  (기존)": r"C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_Swarm_PLT변환.json",
    "G2 (신규)": r"C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G2_Swarm_PLT변환.json",
}
STEP = 0.02          # 경계 조밀화 간격(mm). 0.025 격자보다 촘촘해야 최대값을 놓치지 않는다.
GRID_MM = 0.025      # PLT 좌표 격자 (40 PLU = 1 mm)


def fix(g):
    return g if g.is_valid else g.buffer(0)


def load_cad():
    return {it["id"]: [(float(x), float(y)) for x, y in it["shape"]["data"]]
            for it in json.load(open(CAD, encoding="utf-8"))["items"]}


def load_marker(path):
    """출력 JSON 의 shape 는 'plt' 방향이다. (w-x, h-y) 로 되돌리면 CAD 와 같은
    ref180 프레임이 된다. 양쪽 다 bbox 최소가 (0,0) 이라 그대로 겹쳐 볼 수 있다."""
    doc = json.load(open(path, encoding="utf-8"))
    rings, hints = {}, {}
    for it in doc["items"]:
        p = [(float(x), float(y)) for x, y in it["shape"]["data"]]
        w = max(q[0] for q in p)
        h = max(q[1] for q in p)
        rings[it["id"]] = [(w - x, h - y) for x, y in p]
        hints[it["id"]] = it["placed_hint"]
    return doc, rings, hints


def delta_out(cad_pts, plt_poly, dx=0.0, dy=0.0, _cache={}):
    key = id(cad_pts)
    if key not in _cache:
        _cache[key] = np.asarray(shapely.segmentize(Polygon(cad_pts).exterior, STEP).coords)
    c = _cache[key] + np.array([dx, dy])
    return float(shapely.distance(shapely.points(c), plt_poly).max())


def best_align(cad_pts, plt_poly, step0=0.12, rounds=8):
    bx = by = 0.0
    best = delta_out(cad_pts, plt_poly)
    step = step0
    for _ in range(rounds):
        moved = True
        while moved:
            moved = False
            for ddx, ddy in ((step, 0), (-step, 0), (0, step), (0, -step),
                             (step, step), (-step, -step), (step, -step), (-step, step)):
                v = delta_out(cad_pts, plt_poly, bx + ddx, by + ddy)
                if v < best - 1e-6:
                    best, bx, by = v, bx + ddx, by + ddy
                    moved = True
        step /= 2
    return best, bx, by


def placed(rings, hints, cad):
    """마커 좌표계에 놓인 (PLT 폴리곤, CAD 폴리곤)."""
    ids = sorted(rings)
    plt_p, cad_p = [], []
    for i in ids:
        h = hints[i]
        w = max(q[0] for q in rings[i])
        hh = max(q[1] for q in rings[i])
        plt_p.append(fix(Polygon([(w - x + h["tr_x"], hh - y + h["tr_y"]) for x, y in rings[i]])))
        c = cad[i]
        cw = max(q[0] for q in c)
        ch = max(q[1] for q in c)
        cad_p.append(fix(Polygon([(cw - x + h["tr_x"], ch - y + h["tr_y"]) for x, y in c])))
    return ids, plt_p, cad_p


def overlaps(ids, polys):
    tree = STRtree(polys)
    pairs, total = [], 0.0
    for i, g in enumerate(polys):
        for j in tree.query(g):
            if j <= i or not g.intersects(polys[j]):
                continue
            inter = g.intersection(polys[j])
            if inter.area <= 1e-12:
                continue
            total += inter.area
            lo, hi = 0.0, max(inter.bounds[2] - inter.bounds[0],
                              inter.bounds[3] - inter.bounds[1]) / 2 + 1e-6
            for _ in range(40):
                mid = (lo + hi) / 2
                if inter.buffer(-mid).is_empty:
                    hi = mid
                else:
                    lo = mid
            pairs.append((ids[i], ids[j], inter.area, hi * 2))
    pairs.sort(key=lambda t: -t[3])
    return pairs, total


def required_erosion(polys):
    def clashes(t):
        gs = [g for g in (p.buffer(-t) if t > 0 else p for p in polys) if not g.is_empty]
        tree = STRtree(gs)
        for i, g in enumerate(gs):
            for j in tree.query(g):
                if j > i and g.intersection(gs[j]).area > 1e-9:
                    return True
        return False

    if not clashes(0.0):
        return 0.0
    lo, hi = 0.0, 1.0
    while clashes(hi):
        hi *= 2
        if hi > 20:
            return float("inf")
    for _ in range(24):
        mid = (lo + hi) / 2
        if clashes(mid):
            lo = mid
        else:
            hi = mid
    return hi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--best-align", action="store_true",
                    help="조각마다 최적 평행이동을 찾아 정렬 편향을 분리한다(느림)")
    ap.add_argument("--overlaps", action="store_true", default=True,
                    help="마커에 실제로 놓인 상태의 겹침도 잰다")
    args = ap.parse_args()

    cad = load_cad()
    for tag, path in MARKERS.items():
        doc, rings, hints = load_marker(path)
        print("\n" + "=" * 74)
        print(tag)
        print("=" * 74)

        ds, best = [], []
        for i in sorted(cad):
            P = fix(Polygon(rings[i]))
            ds.append(delta_out(cad[i], P))
            if args.best_align:
                best.append(best_align(cad[i], P)[0])
        ds = np.array(ds)
        print("[1] 단순화 오차 delta_out (원본이 PLT 폴리곤 밖으로 나간 깊이, mm)")
        print("    max=%.4f  p99=%.4f  p95=%.4f  p90=%.4f  median=%.4f  mean=%.4f"
              % (ds.max(), *np.percentile(ds, [99, 95, 90, 50]), ds.mean()))
        print("    0.15mm 초과 조각 %d개 / 156" % int((ds > 0.15).sum()))
        if best:
            b = np.array(best)
            print("    최적 정렬 시 max=%.4f  (차이 %.4f 는 bbox 정렬 편향)"
                  % (b.max(), ds.max() - b.max()))
        print("    참고: PLT 좌표 격자 %.3fmm -> 양자화 상한 %.4fmm"
              % (GRID_MM, GRID_MM * math.sqrt(2) / 2))

        ids, plt_p, cad_p = placed(rings, hints, cad)
        if args.overlaps:
            pp, pt = overlaps(ids, plt_p)
            cp, ct = overlaps(ids, cad_p)
            print("[2] 이 마커에 실제로 놓인 상태의 겹침")
            print("    PLT 폴리곤끼리 : %d쌍, %.3f mm2, 최대 침투 %.4f mm"
                  % (len(pp), pt, pp[0][3] if pp else 0))
            print("    원본 CAD끼리   : %d쌍, %.3f mm2, 최대 침투 %.4f mm  %s"
                  % (len(cp), ct, cp[0][3] if cp else 0,
                     ("(최악 쌍 id %d x %d)" % (cp[0][0], cp[0][1])) if cp else ""))
            t = required_erosion(cad_p)
            print("    이 배치의 원본 겹침을 없애는 조각별 보정 t = %.4f mm"
                  " -> min_item_separation %.3f mm" % (t, 2 * t))

        per = sum(Polygon(r).length for r in rings.values())
        area = sum(Polygon(r).area for r in rings.values())
        L = max(max(q[0] for q in it["shape"]["data"]) + it["placed_hint"]["tr_x"]
                for it in doc["items"])
        H = doc["strip_height"]
        util = area / (L * H)
        print("[3] 보정량별 커버리지와 원단 비용(추정: 오프셋 면적을 같은 밀도로 흡수)")
        print("    %-9s %-18s %-14s %s" % ("t(mm)", "min_item_sep(mm)", "커버 조각", "마커 길이 증가"))
        for t in (0.05, 0.10, 0.1277, 0.15, 0.20, 0.2558, float(ds.max()), 0.30):
            cov = int((ds <= t + 1e-9).sum())
            add = per * t + len(rings) * math.pi * t * t
            dL = add / (H * util)
            print("    %-9.4f %-18.3f %3d/156 (%5.1f%%)  %+7.2f mm (%+.3f%%)"
                  % (t, 2 * t, cov, 100.0 * cov / 156, dL, 100 * dL / L))


if __name__ == "__main__":
    main()
