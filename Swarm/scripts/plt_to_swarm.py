#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plt_to_swarm.py -- HPGL(.plt) 마커 파일을 Swarm/jagua-rs 네스팅 인스턴스 JSON 으로 변환한다.

사용법:
    python plt_to_swarm.py <입력.plt> <출력.json> [--reference <참조.json>]
                           [--name <name 필드>] [--report <애매성리포트.json>]
                           [--no-hints] [--selftest-reference]

설계 근거 요약(모두 실측으로 확정된 사실이며, 각 단계 주석에 근거를 다시 적었다):
  * 40 HPGL 단위(PLU) = 1 mm 는 정확한 정수 스케일이다. 마커 테두리 높이
    60960 PLU / 40 = 1524.000 mm 로 딱 떨어진다.
  * 재단 조각은 '닫힌 폴리라인 중 |면적| >= 10000 mm2' 로 정확히 157개가 잡히고,
    그중 부호면적이 양수인 유일한 하나가 마커 테두리다 -> 나머지 156개가 조각.
  * 참조 JSON 의 도형은 PLT 도형을 180도 회전한 뒤 자기 bbox 최소 모서리를
    (0,0) 으로 옮긴 것이다. 156/156 전량에서 성립하며 항등/거울 변환은 기각됐다.
  * 도형 정규화로 버려지는 '조각이 마커 안에서 놓여 있던 자리'를 items[*].placed_hint
    로 되살린다. 정규화가 뺀 병진을 그대로 되돌려 주는 값이며, 도형 + 힌트를 합치면
    원본 마커 배치가 정수 오차 0 으로 재구성된다(--no-hints 로 끌 수 있다).
  * 출력 바이트 규약은 .NET(System.Text.Json) 계열이다: CRLF, 끝 개행 없음,
    들여쓰기 2칸, 정수값 float 은 '0' 처럼 소수점 없이, 지수는 대문자 'E'.
"""

import argparse
import json
import math
import os
import sys
from collections import OrderedDict

# ---------------------------------------------------------------------------
# 상수
# ---------------------------------------------------------------------------

# 40 PLU = 1 mm. 반올림으로 추정한 값이 아니라 정확한 정수 스케일이다.
# 근거: 좌표 최대값 259691/6492.275 = 40.0 이고, 테두리 높이 60960/40 = 1524.000mm.
PLU_PER_MM = 40

# 닫힌 폴리곤을 '재단 조각'으로 볼 면적 하한(mm2).
# 근거: 실측 조각 최소 면적 15792.67 mm2, 최대 라벨 상자 1408.00 mm2 로 11.2배 간극이
# 있고, 열린 폴리라인의 최대 함축 면적도 7491.8 mm2 로 이 값 아래다.
# 즉 1408 초과 ~ 15792 이하 어디를 잡아도 같은 156개가 나오는 매우 넓은 안전 구간이다.
PIECE_AREA_MIN_MM2 = 10000.0

# 기대되는 조각 수(이 마커: 39조각 x S/M/L/XL 4벌).
EXPECTED_PIECE_COUNT = 156

# --- 참조 JSON 과의 도형 매칭 파라미터 ---
# 사전 필터: 면적 상대차와 bbox 폭/높이 차.
# 근거: 정렬 면적 쌍별 상대차 최대 6.66e-4, bbox 폭 최대차 0.131mm / 높이 0.098mm.
MATCH_AREA_REL_TOL = 1.5e-3
MATCH_BBOX_TOL_MM = 0.6
# 양방향 정점-선분 Hausdorff 허용 오차(mm).
# 근거: 참 매칭의 실측 최대 편차가 0.2559mm(= PLT 0.025mm 양자화 + 곡선 간략화 sagitta).
# tau 를 0.26~1.00 으로 흔들어도 매칭 결과(유일 87개)가 불변임을 확인했다.
MATCH_TAU_MM = 0.30

# 검증 기대치(이 PLT 파일 고유값). 다른 파일에서는 경고만 내고 진행한다.
EXPECT_AREA_SUM_MM2 = 7752563.72
EXPECT_X_SPAN_MM = 6456.125
EXPECT_Y_SPAN_MM = 1524.000


def log(msg):
    """진행 로그. 콘솔이 cp949 라 ASCII 만 쓴다(한글 출력 금지)."""
    sys.stderr.write("[plt2swarm] " + msg + "\n")
    sys.stderr.flush()


def warn(msg):
    sys.stderr.write("[plt2swarm][WARN] " + msg + "\n")
    sys.stderr.flush()


# ---------------------------------------------------------------------------
# 1. HPGL 파싱
# ---------------------------------------------------------------------------

def parse_plt(path):
    """PLT 를 폴리라인 리스트로 파싱한다.

    반환: (polylines, groups)
      polylines: [[(x_plu, y_plu), ...], ...]  좌표는 전부 정수 PLU
      groups:    [[polyline_index, ...], ...]  'SP' 명령으로 구분되는 그리기 그룹

    파싱 규약(실측으로 확정):
      - 파일은 latin-1 로 읽는다. HPGL 은 ASCII 이므로 손실이 없고, cp949/utf-8
        디코딩 실패 위험을 없앤다.
      - 명령은 ';' 로 구분된다.
      - 'PU x,y'  (인자 있음) = 새 폴리라인 시작(첫 점이 그 좌표)
      - 'PD x,y'  (인자 있음) = 현재 폴리라인에 점 추가
      - 인자 없는 'PU;' / 'SP1;' / 그 외 모든 명령 = 현재 폴리라인 종료
      실측: PD 고아 0개, 1점 폴리라인 0개, 'PU x,y;PU x,y;' 연속 쌍 0개.
      종결자를 어떻게 잡든(SP 포함/제외, 인자없는 PU 포함/제외) 결과가 완전히 동일했다.
    """
    with open(path, "r", encoding="latin-1") as f:
        raw = f.read()

    polylines = []
    groups = []
    cur = None           # 현재 폴리라인
    cur_group = []       # 현재 그리기 그룹(폴리라인 인덱스 목록)
    op_counts = {}

    def flush_poly():
        nonlocal cur
        if cur is not None:
            polylines.append(cur)
            cur_group.append(len(polylines) - 1)
            cur = None

    def flush_group():
        nonlocal cur_group
        if cur_group:
            groups.append(cur_group)
        cur_group = []

    for token in raw.split(";"):
        token = token.strip()
        if not token:
            continue
        op = token[:2].upper()
        arg = token[2:].strip()
        op_counts[op] = op_counts.get(op, 0) + 1

        if op == "PU" and arg:
            # 새 폴리라인 시작. 이전 것을 확정한다.
            flush_poly()
            pts = _parse_coords(arg)
            if not pts:
                warn("PU with unparsable args: %r" % token[:40])
                continue
            cur = list(pts)
        elif op == "PD" and arg:
            pts = _parse_coords(arg)
            if cur is None:
                # 실측상 0건이지만 방어적으로 처리한다.
                warn("orphan PD (no preceding PU): %r" % token[:40])
                cur = list(pts)
            else:
                cur.extend(pts)
        else:
            # 인자 없는 PU;, SP, VS, PW, WU, IN, NE 등. 폴리라인을 끊는다.
            flush_poly()
            if op == "SP":
                # 'PU;SP1;' 쌍이 그리기 그룹 경계다(실측 2049회 반복, 완전 일관).
                flush_group()

    flush_poly()
    flush_group()

    log("parsed: %d polylines, %d groups, ops=%s"
        % (len(polylines), len(groups),
           ",".join("%s:%d" % kv for kv in sorted(op_counts.items()))))
    return polylines, groups


def _parse_coords(arg):
    """'x,y' 또는 'x,y,x,y,...' 를 정수 좌표쌍 리스트로. 좌표는 전부 정수 PLU 다."""
    parts = [p for p in arg.replace(" ", "").split(",") if p != ""]
    if len(parts) % 2 != 0:
        return []
    out = []
    for i in range(0, len(parts), 2):
        try:
            out.append((int(parts[i]), int(parts[i + 1])))
        except ValueError:
            # 소수 좌표는 이 파일에 존재하지 않지만(전부 정수) 방어적으로 허용한다.
            try:
                out.append((int(round(float(parts[i]))),
                            int(round(float(parts[i + 1])))))
            except ValueError:
                return []
    return out


# ---------------------------------------------------------------------------
# 2. 기하 유틸 (부호면적은 정수 PLU 로 정확히 계산한다)
# ---------------------------------------------------------------------------

def signed_area_plu2(ring):
    """정수 PLU 좌표 링의 부호면적 x2 (정수, 오차 없음). 반시계=양수."""
    s = 0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s


def signed_area_mm2(ring_plu):
    """정수 PLU 링의 부호면적(mm2). 2*PLU_PER_MM^2 = 3200 으로 나눈다."""
    return signed_area_plu2(ring_plu) / (2.0 * PLU_PER_MM * PLU_PER_MM)


def signed_area_f(pts):
    """실수 좌표 링의 부호면적."""
    s = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s * 0.5


def bbox(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def _orient(ax, ay, bx, by, cx, cy):
    v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    if v > 0:
        return 1
    if v < 0:
        return -1
    return 0


def _on_seg(ax, ay, bx, by, px, py):
    return (min(ax, bx) <= px <= max(ax, bx)
            and min(ay, by) <= py <= max(ay, by))


def segments_properly_intersect(a, b, c, d):
    """두 선분이 (끝점 공유가 아닌) 실제 교차를 하는지."""
    o1 = _orient(a[0], a[1], b[0], b[1], c[0], c[1])
    o2 = _orient(a[0], a[1], b[0], b[1], d[0], d[1])
    o3 = _orient(c[0], c[1], d[0], d[1], a[0], a[1])
    o4 = _orient(c[0], c[1], d[0], d[1], b[0], b[1])
    if o1 != o2 and o3 != o4:
        return True
    # 공선 겹침
    if o1 == 0 and _on_seg(a[0], a[1], b[0], b[1], c[0], c[1]):
        return True
    if o2 == 0 and _on_seg(a[0], a[1], b[0], b[1], d[0], d[1]):
        return True
    if o3 == 0 and _on_seg(c[0], c[1], d[0], d[1], a[0], a[1]):
        return True
    if o4 == 0 and _on_seg(c[0], c[1], d[0], d[1], b[0], b[1]):
        return True
    return False


def count_self_intersections(ring):
    """닫힌 링의 자기교차 쌍 수. 인접 선분(끝점 공유)은 제외한다."""
    n = len(ring)
    cnt = 0
    for i in range(n):
        a = ring[i]
        b = ring[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i:
                continue
            # 인접(끝점 공유) 선분은 건너뛴다
            if j == (i + 1) % n or (j + 1) % n == i:
                continue
            c = ring[j]
            d = ring[(j + 1) % n]
            if segments_properly_intersect(a, b, c, d):
                cnt += 1
    return cnt


# ---------------------------------------------------------------------------
# 3. 조각 추출
# ---------------------------------------------------------------------------

def extract_pieces(polylines, groups):
    """닫힌 폴리라인에서 마커 테두리와 156개 재단 조각을 뽑는다.

    1차 규칙: 닫힘(첫 점 == 끝 점, 정수 완전 일치 / 허용오차 없음) 이고
              |면적| >= 10000 mm2 인 폴리곤 -> 정확히 157개.
              그중 부호면적이 양수인 유일한 하나가 마커 테두리.
              (실측: 조각 156개 전부 음수(CW), 테두리만 양수(CCW).)
    2차 규칙(교차 검증, 반드시 함께 실행): 'PU;SP1;' 로 구분되는 그리기 그룹 중
              폴리라인이 정확히 1개이고 그것이 닫힌 조각인 그룹 -> 156개.
              두 규칙의 결과 집합이 다르면 실패로 본다.

    닫힘 판정에 허용오차를 두지 않는 이유: 끝점 간격이 0.25mm 이내인 '거의 닫힌'
    열린 폴리라인이 0개다. 허용오차는 이득 없이 노이즈만 들인다.

    금지 사항:
      - 열린 폴리라인을 끝점 매칭으로 이어붙이지 않는다. 그렇게 하면 주석 그룹
        8곳에서 약 20620 mm2 짜리 가짜 조각이 생겨 156개가 164개가 된다.
      - 점 개수로 거르지 않는다. 조각의 정점 수는 10~158 이고, 제외 대상인
        닫힌 주석 도형 중 1233개가 10점 이상이다. 어떤 임계값도 오분류를 낸다.
    """
    closed = []   # (polyline_index, ring_plu)
    for idx, pl in enumerate(polylines):
        if len(pl) >= 4 and pl[0] == pl[-1]:
            closed.append((idx, pl[:-1]))   # 마지막 중복점 제거 -> 열린 링

    log("closed polylines: %d" % len(closed))

    big = []
    for idx, ring in closed:
        a2 = signed_area_plu2(ring)
        a_mm2 = a2 / (2.0 * PLU_PER_MM * PLU_PER_MM)
        if abs(a_mm2) >= PIECE_AREA_MIN_MM2:
            big.append((idx, ring, a_mm2))

    log("closed polygons with |area| >= %.0f mm2: %d"
        % (PIECE_AREA_MIN_MM2, len(big)))
    if len(big) != EXPECTED_PIECE_COUNT + 1:
        warn("expected %d (border + %d pieces) but got %d"
             % (EXPECTED_PIECE_COUNT + 1, EXPECTED_PIECE_COUNT, len(big)))

    positives = [t for t in big if t[2] > 0]
    if len(positives) != 1:
        warn("expected exactly 1 CCW (positive-area) polygon = marker border, "
             "got %d; falling back to largest-|area| rule" % len(positives))
        border = max(big, key=lambda t: abs(t[2]))
    else:
        border = positives[0]

    pieces = [t for t in big if t[0] != border[0]]
    # 결정론적 순서: |면적| 내림차순, 동률이면 폴리라인 인덱스.
    pieces.sort(key=lambda t: (-abs(t[2]), t[0]))

    # --- 2차 규칙 교차 검증 ---
    piece_idx_set = set(t[0] for t in pieces)
    group_singletons = set()
    closed_idx = dict((i, r) for i, r in closed)
    for g in groups:
        if len(g) != 1:
            continue
        i = g[0]
        r = closed_idx.get(i)
        if r is None:
            continue
        a_mm2 = signed_area_plu2(r) / (2.0 * PLU_PER_MM * PLU_PER_MM)
        if abs(a_mm2) >= PIECE_AREA_MIN_MM2 and i != border[0]:
            group_singletons.add(i)

    if group_singletons != piece_idx_set:
        warn("CROSS-CHECK MISMATCH: area rule gave %d pieces, group rule gave %d; "
             "symmetric difference = %d"
             % (len(piece_idx_set), len(group_singletons),
                len(piece_idx_set ^ group_singletons)))
    else:
        log("cross-check OK: area rule and PU;SP1; group rule agree on %d pieces"
            % len(piece_idx_set))

    return border, pieces


# ---------------------------------------------------------------------------
# 4. 좌표 변환 (180도 회전 + bbox 최소 모서리를 (0,0) 으로 정규화)
# ---------------------------------------------------------------------------

def transform_ring(ring_plu):
    """정수 PLU 링 -> 참조 규약에 맞춘 mm 링.

    변환: (x, y) -> ((x_max_p - x)/40, (y_max_p - y)/40)
      * 두 좌표의 부호를 뒤집는 것이 180도 회전이고, 자기 bbox 최대값을 빼는 것이
        곧 '회전 후 bbox 최소 모서리를 (0,0) 으로 이동' 과 동일하다.
      * 전역 마커 최대값(259691/62644)을 쓰면 안 된다 -- 정규화 전 오프셋이 달라진다.
        (조각별 bbox 를 쓰면 정규화가 병진을 흡수하므로 전역 상수는 필요 없다.)
      * 거울 변환(x 또는 y 단독 뒤집기)은 절대 쓰지 않는다. 실측에서 mirrorX 는
        0/156, mirrorY 는 140/156 으로 기각됐고, 거울은 감김 방향까지 뒤집는다.
      * 정수 PLU 단계에서 뺄셈을 먼저 하고 40 으로 나누면 결과가 0.025 배수 격자에
        정확히 올라간다(부동소수 누적 오차 없음).

    감김 방향은 뒤집지 않는다: PLT 조각은 156/156 부호면적 음수(CW)이고
    180도 회전은 부호를 보존하므로 결과도 음수 -- 참조 JSON 156개와 동일하다.
    """
    xmax = max(p[0] for p in ring_plu)
    ymax = max(p[1] for p in ring_plu)
    out = []
    for (x, y) in ring_plu:
        # round(v, 3): PLT 유래 값은 전부 0.025 격자 위에 있어 3자리로 무손실이며,
        # 161.57500000000002 같은 이진 부동소수 꼬리를 없애준다.
        vx = round((xmax - x) / float(PLU_PER_MM), 3)
        vy = round((ymax - y) / float(PLU_PER_MM), 3)
        out.append((vx, vy))
    return out


# 방향 규약 ------------------------------------------------------------------
#
# transform_ring 은 조각을 **180도 회전**해서 정규화한다. 이것은 참조 파일
# (여성WSjk_비교용-G_New_Swarm_신규2.json) 의 규약이며, 그 파일과 156/156 정점
# 순서까지 일치함이 실측으로 확인됐다.
#
# 그러나 그 방향은 원본 PLT 플롯을 180도 돌려 놓은 것이라, 마커를 눈으로 비교하면
# 뒤집혀 보인다. 그래서 기본 출력 방향은 'plt' -- 원본 플롯 그대로 -- 로 둔다.
#
#   plt    : 도형 = 조각을 회전 없이 정규화. 힌트를 적용하면 **원본 마커가 그대로**
#            재구성된다 (테두리 최소 모서리가 원점).
#   ref180 : 도형 = 조각을 180도 회전 후 정규화. 참조 파일과 같은 방향.
#            힌트를 적용하면 원본 마커를 180도 돌린 배치가 된다.
#
# 두 방향 모두 **회전**이므로 감김 방향(CW)과 조각의 좌우 손잡이(handedness)가
# 보존된다. 거울 변환은 어느 쪽에도 쓰지 않는다 -- 거울은 좌/우 짝 조각을 뒤바꿔
# 물리적으로 다른 조각을 만든다.

ORIENT_PLT = "plt"
ORIENT_REF180 = "ref180"


def emit_ring(shape, orientation):
    """내보낼 도형 좌표. shape["pts"] 는 항상 ref180 방향(매칭용)이다.

    plt 방향은 그 bbox 안에서 다시 180도 돌리면 된다:
    ``(x, y) -> (W - x, H - y)``. 정규화된 도형이라 bbox 최소가 (0,0) 이므로
    W/H 는 곧 bbox 최대값이다. 두 값 모두 0.025 배수라 뺄셈이 무손실이고,
    정점 순서는 그대로다(회전은 순서를 뒤집지 않는다).
    """
    pts = shape["pts"]
    if orientation == ORIENT_REF180:
        return [[x, y] for (x, y) in pts]
    w = max(p[0] for p in pts)
    h = max(p[1] for p in pts)
    return [[round(w - x, 3), round(h - y, 3)] for (x, y) in pts]


def placed_hint(item_id, shape, border, orientation):
    """조각이 원본 마커에서 놓여 있던 자리를 placed_hint 로 만든다.

    ## 좌표 유도

    ``border`` 는 마커 테두리 폴리곤의 PLU bbox ``(BX0, BY0, BX1, BY1)`` 다.
    테두리 최소 모서리를 원점으로 삼는다.

    * ``plt`` 방향: 도형이 ``(x - xmin_p)/40`` 로 정규화돼 있으므로
      ``tr = ((xmin_p - BX0)/40, (ymin_p - BY0)/40)`` 를 더하면
      ``(x - BX0)/40`` 이 되어 **원본 마커가 그대로** 재구성된다.
    * ``ref180`` 방향: 도형이 ``(xmax_p - x)/40`` 이므로
      ``tr = ((BX1 - xmax_p)/40, (BY1 - ymax_p)/40)`` 를 더하면
      ``(BX1 - x)/40`` 이 되어 마커를 180도 돌린 배치가 나온다.

    BX0/BY0/BX1/BY1 을 상수로 박지 않고 테두리 링에서 유도하므로 다른 마커
    파일에도 같은 코드가 그대로 동작한다.

    ## 필드 규약 (src/grouped_input.rs 의 PlacedHintInput 이 정본)

    ``deny_unknown_fields`` 라 정의된 8개 밖의 필드를 넣으면 파싱이 실패한다.
    여기서는 필수 5개와 stripe 2개만 쓰고 ``ver`` 는 생략한다(serde default).

      * ``item_id`` 는 바깥 item.id 와 반드시 같아야 한다. 다르면 InvalidHint.
      * ``is_freeze`` 는 **정수** 0 또는 1 만 허용된다. Rust 는 i32 로 읽으므로
        JSON ``true`` 를 쓰면 깨진다. C# 은 bool 도 받으므로 정수가 양쪽 모두
        통과하는 유일한 표기다.
      * ``is_freeze = 0`` (warm start) 을 쓴다. 0 이면 WPF 의 "Use Hints" 토글로
        사용자가 켜고 끌 수 있다. 1 로 고정하면 끌 수 없고, 이 마커는 양자화
        슬리버 겹침이 있어 frozen 검증에서 레인 구성 자체가 실패한다.
      * ``rot_angle_in_rad = 0.0``. 방향은 이미 도형 데이터에 구워져 있고 tr 도
        같은 방향 프레임에서 계산했다. 각을 더 주면 이중 적용이고,
        allowed_orientations 가 [0] 이라 0 이외의 각은 거부된다.
    """
    bx0, by0, bx1, by1 = border
    if orientation == ORIENT_REF180:
        tr_x = (bx1 - shape["plu_xmax"]) / float(PLU_PER_MM)
        tr_y = (by1 - shape["plu_ymax"]) / float(PLU_PER_MM)
    else:
        tr_x = (shape["plu_xmin"] - bx0) / float(PLU_PER_MM)
        tr_y = (shape["plu_ymin"] - by0) / float(PLU_PER_MM)
    return OrderedDict([
        ("item_id", item_id),
        ("is_freeze", 0),
        ("tr_x", round(tr_x, 3)),
        ("tr_y", round(tr_y, 3)),
        ("rot_angle_in_rad", 0.0),
        ("stripe_interval_y", 0.0),
        ("stripe_interval_x", 0.0),
    ])


def verify_hints(items, pieces_by_index, shapes, order, border, orientation):
    """힌트가 원본 마커를 정확히 재구성하는지 정수 산술로 전수 검증한다.

    도형 좌표와 tr 이 모두 0.025 배수라 부동소수 오차 없이 정수로 환산할 수 있다.
    조각 하나라도 어긋나면 힌트가 엉뚱한 아이템에 붙은 것이므로 즉시 경고한다.
    기대 좌표는 방향에 따라 다르다 -- plt 는 (x - BX0), ref180 은 (BX1 - x).
    """
    bx0, by0, bx1, by1 = border
    bad = 0
    checked = 0
    for k, pi in enumerate(order):
        item = items[k]
        hint = item.get("placed_hint")
        if hint is None:
            continue
        ring = pieces_by_index[shapes[pi]["plu_index"]]
        tx = int(round(hint["tr_x"] * PLU_PER_MM))
        ty = int(round(hint["tr_y"] * PLU_PER_MM))
        pts = item["shape"]["data"]
        if len(pts) != len(ring):
            warn("hint check: item %s vertex count %d != piece %d"
                 % (item["id"], len(pts), len(ring)))
            bad += 1
            continue
        for (vx, vy), (px, py) in zip(pts, ring):
            gx = int(round(vx * PLU_PER_MM)) + tx
            gy = int(round(vy * PLU_PER_MM)) + ty
            if orientation == ORIENT_REF180:
                ex, ey = bx1 - px, by1 - py
            else:
                ex, ey = px - bx0, py - by0
            checked += 1
            if gx != ex or gy != ey:
                warn("hint check: item %s vertex mismatch (%d,%d) != (%d,%d)"
                     % (item["id"], gx, gy, ex, ey))
                bad += 1
                break
    if bad:
        warn("HINT RECONSTRUCTION FAILED for %d item(s)" % bad)
    else:
        log("hint self-check OK: %d vertices reconstruct the marker exactly "
            "(orientation=%s)" % (checked, orientation))
    return bad == 0


# ---------------------------------------------------------------------------
# 5. 참조 JSON 과의 도형 매칭 (domain 메타데이터 이전용)
# ---------------------------------------------------------------------------

def _ring_segments(pts):
    """닫힌 링의 선분 리스트 (ax, ay, dx, dy, len2, minx, miny, maxx, maxy)."""
    segs = []
    n = len(pts)
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        dx = bx - ax
        dy = by - ay
        l2 = dx * dx + dy * dy
        segs.append((ax, ay, dx, dy, l2,
                     min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)))
    return segs


def _max_pt_to_ring_dist(pts, segs, limit):
    """pts 각 점에서 링 경계까지 거리의 최대값. limit 를 넘으면 즉시 중단하고
    limit + 1.0 을 반환한다(비매칭 후보를 빠르게 버리기 위한 조기 종료)."""
    lim2 = limit * limit
    worst2 = 0.0
    for (px, py) in pts:
        best2 = float("inf")
        for (ax, ay, dx, dy, l2, mnx, mny, mxx, mxy) in segs:
            # bbox 프루닝: 이 선분이 현재 best 보다 확실히 멀면 건너뛴다.
            if px < mnx:
                gx = mnx - px
            elif px > mxx:
                gx = px - mxx
            else:
                gx = 0.0
            if gx * gx >= best2:
                continue
            if py < mny:
                gy = mny - py
            elif py > mxy:
                gy = py - mxy
            else:
                gy = 0.0
            if gx * gx + gy * gy >= best2:
                continue
            if l2 <= 0.0:
                t = 0.0
            else:
                t = ((px - ax) * dx + (py - ay) * dy) / l2
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0
            ex = px - (ax + t * dx)
            ey = py - (ay + t * dy)
            d2 = ex * ex + ey * ey
            if d2 < best2:
                best2 = d2
                if best2 == 0.0:
                    break
        if best2 > worst2:
            worst2 = best2
            if worst2 > lim2:
                return limit + 1.0
    return math.sqrt(worst2)


def hausdorff(pts_a, segs_a, pts_b, segs_b, limit):
    """양방향 정점-선분 Hausdorff. 한쪽이라도 limit 를 넘으면 limit+1 반환.

    '1등만 취하기'가 아니라 이 거리로 tau 이내 후보 '전체'를 모으는 데 쓴다.
    2등과의 margin 이 0.0001mm 인 경우가 있어 1등 선택은 의미가 없기 때문이다.
    """
    d1 = _max_pt_to_ring_dist(pts_a, segs_b, limit)
    if d1 > limit:
        return limit + 1.0
    d2 = _max_pt_to_ring_dist(pts_b, segs_a, limit)
    if d2 > limit:
        return limit + 1.0
    return max(d1, d2)


def build_candidates(piece_shapes, ref_items):
    """각 PLT 조각에 대해 tau 이내의 참조 아이템 후보 집합을 만든다.

    사전 필터(면적 상대차 / bbox 폭·높이 차)로 비용이 큰 Hausdorff 계산을 줄인다.
    도형은 양쪽 모두 이미 '180도 회전 + bbox 최소 (0,0) 정규화' 된 같은 좌표계다.
    """
    ref_meta = []
    for it in ref_items:
        pts = [(float(p[0]), float(p[1])) for p in it["shape"]["data"]]
        _, _, w, h = bbox(pts)   # 정규화돼 있으므로 min 은 (0,0)
        ref_meta.append({
            "pts": pts,
            "segs": _ring_segments(pts),
            "area": abs(signed_area_f(pts)),
            "w": w, "h": h,
        })

    cands = []
    n_pairs = 0
    for pi, sh in enumerate(piece_shapes):
        pts = sh["pts"]
        segs = sh["segs"]
        area = sh["area"]
        w, h = sh["w"], sh["h"]
        hits = []
        for ri, rm in enumerate(ref_meta):
            if abs(area - rm["area"]) > MATCH_AREA_REL_TOL * max(area, rm["area"]):
                continue
            if abs(w - rm["w"]) > MATCH_BBOX_TOL_MM:
                continue
            if abs(h - rm["h"]) > MATCH_BBOX_TOL_MM:
                continue
            n_pairs += 1
            d = hausdorff(pts, segs, rm["pts"], rm["segs"], MATCH_TAU_MM)
            if d <= MATCH_TAU_MM:
                hits.append((ri, d))
        hits.sort(key=lambda t: (t[0]))   # 참조 id 오름차순(결정론적 타이브레이크)
        cands.append(hits)
        if (pi + 1) % 40 == 0:
            log("  matching progress: %d/%d pieces" % (pi + 1, len(piece_shapes)))
    log("candidate pairs evaluated: %d" % n_pairs)
    return cands


def perfect_matching(cands, n_ref):
    """Kuhn 이분 최대매칭. 완전매칭(156/156)이 존재하는지도 이걸로 확인한다.

    조각을 |면적| 내림차순으로 처리하고 각 조각의 후보를 참조 id 오름차순으로
    시도한다. 기하학적으로 완전 교환 가능한 클러스터 안에서는 이 순서가
    '|면적| 내림차순 <-> id 오름차순' 고정 규칙과 같은 결과를 주므로 실행마다
    동일한 배정이 나온다.

    정정(실측): 정규화 후 좌표가 바이트 단위로 완전히 같은 도형 클래스는 2개뿐이고
    각각 2조각(총 4조각: item 57/58, 73/74)이다. 예전 주석의 '17개 패밀리가 바이트
    동일' 은 과장이었다. 나머지 152조각은 도형이 유일하므로 id 대응이 결정적이다.
    그 4조각은 도형이 같아 힌트를 서로 바꿔도 마커 기하 재구성은 불변이고,
    어긋날 수 있는 것은 size 라벨(S vs L)뿐이다.
    """
    match_ref = [-1] * n_ref   # ref index -> piece index

    def try_assign(pi, visited):
        for (ri, _d) in cands[pi]:
            if visited[ri]:
                continue
            visited[ri] = True
            if match_ref[ri] == -1 or try_assign(match_ref[ri], visited):
                match_ref[ri] = pi
                return True
        return False

    sys.setrecursionlimit(10000)
    matched = 0
    # cands 는 이미 |면적| 내림차순으로 정렬된 조각 순서다.
    for pi in range(len(cands)):
        if try_assign(pi, [False] * n_ref):
            matched += 1
    return match_ref, matched


# ---------------------------------------------------------------------------
# 6. .NET 호환 JSON 직렬화
# ---------------------------------------------------------------------------

def fmt_number(v):
    """참조 파일(.NET System.Text.Json)과 같은 숫자 표기를 만든다.

    (a) bool -> true/false
    (b) int  -> str(int)
    (c) float 이 정수값이고 |x| < 1e15 -> 소수점 없이 (0.0 은 '0', '0.0' 아님)
    (d) 그 외 float -> 최단 왕복(repr)
    (e) 지수 표기는 대문자 'E' + 부호 + 2자리 패딩 (1.1852535e-13 -> 1.1852535E-13)

    (c)/(e) 를 빼먹으면 참조와 바이트가 어긋난다. 우리 좌표는 전부 0.025 배수라
    지수 표기가 나올 일은 없지만, 정규화로 생기는 정확한 0 은 반드시 '0' 이어야 한다.
    """
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if v != v or v in (float("inf"), float("-inf")):
            raise ValueError("non-finite number in output")
        if v == int(v) and abs(v) < 1e15:
            return str(int(v))
        s = repr(v)
        if "e" in s or "E" in s:
            mant, _, exp = s.replace("E", "e").partition("e")
            sign = "+"
            if exp[0] in "+-":
                sign = exp[0]
                exp = exp[1:]
            exp = exp.lstrip("0") or "0"
            if len(exp) < 2:
                exp = "0" + exp
            s = mant + "E" + sign + exp
        return s
    raise TypeError("unsupported number type: %r" % type(v))


_ESCAPE = {
    '"': '\\"', "\\": "\\\\", "\n": "\\n", "\r": "\\r", "\t": "\\t",
    "\b": "\\b", "\f": "\\f",
}


def json_string(s):
    """UTF-8 원문 유지(ensure_ascii=False). 한글을 \\uXXXX 로 escape 하지 않는다."""
    out = ['"']
    for ch in s:
        e = _ESCAPE.get(ch)
        if e is not None:
            out.append(e)
        elif ord(ch) < 0x20:
            out.append("\\u%04x" % ord(ch))
        else:
            out.append(ch)
    out.append('"')
    return "".join(out)


def dumps_dotnet(obj, indent=2):
    """참조 파일과 동일한 레이아웃(2칸 들여쓰기, ': ' 구분, 배열 원소 한 줄씩)으로
    직렬화한다. 줄바꿈은 '\\n' 으로 만들고 호출부에서 CRLF 로 바꾼다."""
    buf = []

    def emit(o, lvl):
        pad = " " * (indent * lvl)
        pad2 = " " * (indent * (lvl + 1))
        if isinstance(o, dict):
            if not o:
                buf.append("{}")
                return
            buf.append("{\n")
            items = list(o.items())
            for i, (k, v) in enumerate(items):
                buf.append(pad2)
                buf.append(json_string(k))
                buf.append(": ")
                emit(v, lvl + 1)
                buf.append(",\n" if i < len(items) - 1 else "\n")
            buf.append(pad)
            buf.append("}")
        elif isinstance(o, (list, tuple)):
            if not o:
                buf.append("[]")
                return
            buf.append("[\n")
            for i, v in enumerate(o):
                buf.append(pad2)
                emit(v, lvl + 1)
                buf.append(",\n" if i < len(o) - 1 else "\n")
            buf.append(pad)
            buf.append("]")
        elif o is None:
            buf.append("null")
        elif isinstance(o, str):
            buf.append(json_string(o))
        else:
            buf.append(fmt_number(o))

    emit(obj, 0)
    return "".join(buf)


def write_dotnet_json(path, obj):
    """UTF-8(BOM 없음) + 전부 CRLF + 파일 끝 개행 없음."""
    text = dumps_dotnet(obj).replace("\n", "\r\n")
    with open(path, "wb") as f:
        f.write(text.encode("utf-8"))
    return len(text.encode("utf-8"))


# ---------------------------------------------------------------------------
# 7. 자체 검증
# ---------------------------------------------------------------------------

def validate(items, strip_height, border_ring_plu):
    ok = True

    if len(items) != EXPECTED_PIECE_COUNT:
        warn("item count = %d (expected %d)" % (len(items), EXPECTED_PIECE_COUNT))
        ok = False

    ids = [it["id"] for it in items]
    if ids != list(range(len(items))):
        warn("ids are not contiguous 0..%d -- jagua-rs import_instance will bail"
             % (len(items) - 1))
        ok = False

    total_area = 0.0
    bad_norm = bad_wind = bad_open = bad_dup = bad_selfint = 0
    vmin = 10 ** 9
    vmax = 0
    for it in items:
        pts = [(p[0], p[1]) for p in it["shape"]["data"]]
        vmin = min(vmin, len(pts))
        vmax = max(vmax, len(pts))
        # 링은 닫지 않는다(첫 점 != 끝 점). 참조 156개도 전부 열려 있다.
        if pts[0] == pts[-1]:
            bad_open += 1
        mnx, mny, _, _ = bbox(pts)
        # 정확히 0.0 이어야 한다(허용오차 없음).
        if mnx != 0.0 or mny != 0.0:
            bad_norm += 1
        a = signed_area_f(pts)
        # 감김: PLT 도 참조도 156/156 음수(CW). 180도 회전은 부호를 보존한다.
        if a >= 0:
            bad_wind += 1
        total_area += abs(a)
        # 비연속 중복점은 jagua-rs SPolygon::new 가 bail 한다.
        if len(set(pts)) != len(pts):
            bad_dup += 1
        if count_self_intersections(pts) > 0:
            bad_selfint += 1

    for label, n in (("ring not open", bad_open),
                     ("bbox min != (0,0)", bad_norm),
                     ("winding not CW", bad_wind),
                     ("duplicate vertices", bad_dup),
                     ("self-intersecting", bad_selfint)):
        if n:
            warn("%d items: %s" % (n, label))
            ok = False

    bmnx, bmny, bmxx, bmxy = bbox(border_ring_plu)
    x_span = (bmxx - bmnx) / float(PLU_PER_MM)
    y_span = (bmxy - bmny) / float(PLU_PER_MM)

    log("vertex count range: %d..%d" % (vmin, vmax))
    log("total piece area  : %.2f mm2 (expected ~%.2f, diff %+.2f)"
        % (total_area, EXPECT_AREA_SUM_MM2, total_area - EXPECT_AREA_SUM_MM2))
    log("marker border     : %.3f x %.3f mm  (strip_height=%s)"
        % (x_span, y_span, strip_height))
    if y_span > 0 and x_span > 0:
        log("commercial density: %.4f %%" % (100.0 * total_area / (x_span * y_span)))

    if abs(total_area - EXPECT_AREA_SUM_MM2) > EXPECT_AREA_SUM_MM2 * 1e-4:
        warn("total area deviates from expected by more than 0.01%")
    if abs(x_span - EXPECT_X_SPAN_MM) > 0.01:
        warn("border x span %.3f != expected %.3f" % (x_span, EXPECT_X_SPAN_MM))
    if abs(y_span - EXPECT_Y_SPAN_MM) > 0.01:
        warn("border y span %.3f != expected %.3f" % (y_span, EXPECT_Y_SPAN_MM))

    return ok, total_area, x_span, y_span


# ---------------------------------------------------------------------------
# 8. 메인
# ---------------------------------------------------------------------------

def default_name_from_output(out_path):
    """참조 규약: 출력 파일명에서 '_Swarm_<태그>.json' 을 뗀 것이 name 이다.
    (참조: '여성WSjk_비교용-G_New_Swarm_신규2.json' -> '여성WSjk_비교용-G_New')"""
    base = os.path.basename(out_path)
    if base.lower().endswith(".json"):
        base = base[:-5]
    i = base.find("_Swarm_")
    if i >= 0:
        base = base[:i]
    return base


def main():
    ap = argparse.ArgumentParser(
        description="Convert an HPGL .plt marker into a Swarm/jagua-rs strip-packing instance JSON.")
    ap.add_argument("input", help="input .plt path")
    ap.add_argument("output", help="output .json path")
    ap.add_argument("--reference", default=None,
                    help="optional reference instance JSON used to transfer item order "
                         "and domain metadata (piece_name/size/... cannot be recovered "
                         "from the PLT: it contains zero LB text commands)")
    ap.add_argument("--name", default=None, help="override the 'name' field")
    ap.add_argument("--orientation", choices=["plt", "ref180"], default="plt",
                    help="piece/layout orientation. 'plt' (default) keeps the original "
                         "plotted marker orientation; 'ref180' matches the reference "
                         "instance file, which is the marker rotated 180 degrees")
    ap.add_argument("--no-hints", action="store_true",
                    help="do not emit items[*].placed_hint (positions from the marker)")
    ap.add_argument("--report", default=None,
                    help="optional path for a match-ambiguity report JSON")
    ap.add_argument("--selftest-reference", action="store_true",
                    help="re-serialize --reference with our encoder and check byte equality")
    args = ap.parse_args()

    # --- 직렬화 자체검증: 참조 파일을 우리 인코더로 재직렬화해 바이트 일치를 본다 ---
    if args.selftest_reference:
        if not args.reference:
            log("selftest needs --reference")
            return 2
        with open(args.reference, "rb") as f:
            orig = f.read()
        ref = json.load(open(args.reference, "r", encoding="utf-8"),
                        object_pairs_hook=OrderedDict)
        re_ser = dumps_dotnet(ref).replace("\n", "\r\n").encode("utf-8")
        log("selftest: original=%d bytes, reserialized=%d bytes, identical=%s"
            % (len(orig), len(re_ser), orig == re_ser))
        if orig != re_ser:
            for i in range(min(len(orig), len(re_ser))):
                if orig[i] != re_ser[i]:
                    log("first diff at byte %d: orig=%r new=%r"
                        % (i, orig[i - 40:i + 40], re_ser[i - 40:i + 40]))
                    break
            return 1
        return 0

    log("input : %s" % os.path.basename(args.input))
    log("output: %s" % os.path.basename(args.output))

    polylines, groups = parse_plt(args.input)
    border, pieces = extract_pieces(polylines, groups)

    # strip_height 는 마커 테두리 사각형의 bbox 높이에서 유도한다.
    # 헤더 NE 의 두 번째 값(1566)이나 전체 좌표 y 범위(1566.1mm)를 쓰면 안 된다 --
    # 그것은 플로터 캔버스이지 유효 재단 폭이 아니다.
    b = border[1]
    bmny = min(p[1] for p in b)
    bmxy = max(p[1] for p in b)
    sh = (bmxy - bmny) / float(PLU_PER_MM)
    strip_height = int(sh) if sh == int(sh) else sh

    # --- 도형 변환 ---
    shapes = []
    for (idx, ring, a_mm2) in pieces:
        pts = transform_ring(ring)
        mnx, mny, mxx, mxy = bbox(pts)
        shapes.append({
            "pts": pts,
            "segs": _ring_segments(pts),
            "area": abs(signed_area_f(pts)),
            "w": mxx - mnx, "h": mxy - mny,
            "plu_index": idx,
            # 힌트 계산용: 이 조각 링의 PLU bbox 최대 모서리.
            # transform_ring 이 이 값을 빼서 정규화했으므로, 전역 테두리 최대값에서
            # 이 값을 빼면 정규화로 잃어버린 병진이 정확히 복원된다.
            "plu_xmax": max(p[0] for p in ring),
            "plu_ymax": max(p[1] for p in ring),
            "plu_xmin": min(p[0] for p in ring),
            "plu_ymin": min(p[1] for p in ring),
        })
    log("transformed %d piece rings (rot180 + bbox-min normalization)" % len(shapes))

    # --- domain 메타데이터 이전 ---
    ref_items = None
    if args.reference and os.path.exists(args.reference):
        ref = json.load(open(args.reference, "r", encoding="utf-8"),
                        object_pairs_hook=OrderedDict)
        ref_items = ref["items"]
        log("reference loaded: %d items" % len(ref_items))
    elif args.reference:
        warn("reference not found: %s" % args.reference)

    assignment = None       # piece index -> reference index
    ambiguity = None
    if ref_items:
        log("matching PLT pieces to reference items (tau=%.2f mm, bidirectional "
            "vertex-segment Hausdorff)..." % MATCH_TAU_MM)
        cands = build_candidates(shapes, ref_items)
        sizes = [len(c) for c in cands]
        log("candidate set sizes: min=%d max=%d, unique(=1): %d, empty: %d"
            % (min(sizes), max(sizes), sizes.count(1), sizes.count(0)))
        match_ref, matched = perfect_matching(cands, len(ref_items))
        log("bipartite matching: %d/%d matched" % (matched, len(shapes)))
        if matched != len(shapes):
            warn("NOT a perfect matching -- metadata transfer is unreliable")
        assignment = [-1] * len(shapes)
        for ri, pi in enumerate(match_ref):
            if pi >= 0:
                assignment[pi] = ri

        # 애매성 분류.
        # 기하학적으로 교환 가능한 클러스터 안에서는 size/piece_name 배정이 임의다.
        # 클러스터 전체의 multiset 은 보존되므로 마커 단위 통계는 정확하지만,
        # 개별 조각의 라벨은 신뢰하면 안 된다. 그래서 리포트로 남긴다.
        ambiguity = {"exact": [], "size_ambiguous": [], "name_ambiguous": []}
        for pi, hits in enumerate(cands):
            ri = assignment[pi]
            names = set(ref_items[r]["domain"]["piece_name"] for r, _ in hits)
            szs = set(ref_items[r]["domain"]["size"] for r, _ in hits)
            entry = {
                "assigned_item_id": (ref_items[ri]["id"] if ri >= 0 else None),
                "n_candidates": len(hits),
                "best_dist_mm": (min(d for _, d in hits) if hits else None),
                "piece_name_candidates": sorted(names),
                "size_candidates": sorted(szs),
            }
            if len(hits) == 1:
                ambiguity["exact"].append(entry)
            else:
                if len(names) > 1:
                    ambiguity["name_ambiguous"].append(entry)
                if len(szs) > 1:
                    ambiguity["size_ambiguous"].append(entry)
        log("metadata confidence: exact=%d, size-ambiguous=%d, name-ambiguous=%d"
            % (len(ambiguity["exact"]), len(ambiguity["size_ambiguous"]),
               len(ambiguity["name_ambiguous"])))

    # --- items 조립 ---
    # 힌트 좌표계의 원점이 되는 테두리 최대 모서리 (PLU).
    border_plu = (min(p[0] for p in b), min(p[1] for p in b),
                  max(p[0] for p in b), max(p[1] for p in b))
    orientation = args.orientation
    emit_hints = not args.no_hints
    log("orientation: %s (%s)" % (orientation,
        "original plotted marker" if orientation == ORIENT_PLT
        else "reference convention = marker rotated 180 deg"))
    items = []
    if assignment and all(a >= 0 for a in assignment):
        # 참조와 같은 배열 순서(39 패밀리 x M,S,L,XL)로 정렬한다.
        order = sorted(range(len(shapes)), key=lambda pi: ref_items[assignment[pi]]["id"])
        for pi in order:
            rit = ref_items[assignment[pi]]
            items.append(OrderedDict([
                ("id", rit["id"]),
                ("demand", 1),
                ("allowed_orientations", [0]),
                ("shape", OrderedDict([
                    ("type", "simple_polygon"),
                    ("data", emit_ring(shapes[pi], orientation)),
                ])),
            ]))
            if emit_hints:
                items[-1]["placed_hint"] = placed_hint(
                    rit["id"], shapes[pi], border_plu, orientation)
            items[-1]["domain"] = OrderedDict(rit["domain"])
    else:
        # 참조 없이 자립 변환. domain 은 전역 상수만 채운다.
        # PLT 에는 LB(텍스트) 명령이 0개라 piece_name/size 는 복원 불가다.
        if ref_items:
            warn("incomplete matching; falling back to synthetic domain metadata")
        for pi in range(len(shapes)):
            items.append(OrderedDict([
                ("id", pi),
                ("demand", 1),
                ("allowed_orientations", [0]),
                ("shape", OrderedDict([
                    ("type", "simple_polygon"),
                    ("data", emit_ring(shapes[pi], orientation)),
                ])),
                ("placed_hint", placed_hint(pi, shapes[pi], border_plu, orientation)
                 if emit_hints else None),
                ("domain", OrderedDict([
                    ("family_id", None),
                    ("size_group_id", None),
                    ("piece_name", None),
                    ("size", None),
                    ("direction", "normal"),
                    ("material", "G"),
                    ("flip", False),
                    ("grain_axis_deg", 0),
                    ("grain_directional", True),
                    ("fold_mode", "none"),
                ])),
            ]))
            if not emit_hints:
                del items[-1]["placed_hint"]

    name = args.name or default_name_from_output(args.output)

    # 최상위 키 순서는 참조와 정확히 같게 고정한다.
    # 참고: domain_schema_version / fabric_domain / item.domain 은 jagua-rs 의
    # ExtSPInstance/ExtItem 이 파싱하지 않는(serde 가 조용히 무시하는) 순수
    # 메타데이터다. 그래도 상용 툴체인 호환을 위해 참조값을 그대로 승계한다.
    doc = OrderedDict([
        ("domain_schema_version", 2),
        ("fabric_domain", OrderedDict([
            ("material", "G"),      # PLT 파일명 끝 토큰 ' G' = 원단 코드
            # 조각 간 추가 여유 없음. 정정(실측): 원본 마커 자체에 양자화 슬리버
            # 겹침이 49쌍 / 총 4.86 mm2 있고(최대 침투 깊이 0.0834mm), 3.1175 mm2 는
            # 총합이 아니라 최악 단일 쌍(item 38 x 122) 값이다.
            ("margin", 0),
            ("max_length", 10000),  # 솔버 상한(mm). 마커 실제 길이로 바꾸지 않는다.
        ])),
        ("name", name),
        ("items", items),
        ("strip_height", strip_height),
    ])

    ok, total_area, x_span, y_span = validate(items, strip_height, b)

    if emit_hints:
        # 힌트 재구성 자체검증. 조각 링을 인덱스로 다시 찾아 원본과 대조한다.
        pieces_by_index = dict((idx, ring) for (idx, ring, _a) in pieces)
        order_used = order if (assignment and all(a >= 0 for a in assignment)) \
            else list(range(len(shapes)))
        ok = verify_hints(items, pieces_by_index, shapes, order_used,
                          border_plu, orientation) and ok
        log("placed_hint: %d emitted (is_freeze=0, warm start; "
            "frame origin = marker border corner)"
            % sum(1 for it in items if "placed_hint" in it))

    size = write_dotnet_json(args.output, doc)
    log("wrote %s (%d bytes)" % (os.path.basename(args.output), size))

    if args.report and ambiguity is not None:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(ambiguity, f, ensure_ascii=False, indent=2)
        log("wrote ambiguity report: %s" % os.path.basename(args.report))

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
