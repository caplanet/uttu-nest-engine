//! 입력·출력 (§1.6).
//!
//! 입력은 기존 입력 계약(`id`, `demand`, `allowed_orientations`, `shape`, `strip_height`)을
//! 그대로 받는다. §13.1 "기존 입력 계약이 본 문서 설명보다 우선한다" 에 따라
//! `allowed_orientations` 는 **도(°)** 로 읽는다(지시서 본문은 24방향 인덱스라고 적는다).
//!
//! 출력은 기존 검증 도구(`scripts/full_bench/verify_overlap.py`, `penetration_depth.py`,
//! `render_layout.py`)가 읽는 형식이다 — 원본 좌표를 회전한 뒤 `translation` 을 더한다.

use crate::geom::{BBox, GRID_PER_MM, Pt, area2, cleanup};
use anyhow::{Context, Result, bail};
use clipper2::{Centi, EndType, JoinType, Paths, inflate, simplify};
use serde::Deserialize;
use std::collections::HashMap;

#[derive(Deserialize)]
struct ExtShape {
    #[serde(rename = "type")]
    kind: String,
    data: Vec<[f64; 2]>,
}

#[derive(Deserialize)]
struct ExtItem {
    id: i64,
    #[serde(default = "one")]
    demand: usize,
    #[serde(default)]
    allowed_orientations: Vec<f64>,
    shape: ExtShape,
}

fn one() -> usize {
    1
}

#[derive(Deserialize)]
struct ExtInstance {
    #[serde(default)]
    name: String,
    items: Vec<ExtItem>,
    strip_height: f64,
}

/// 조각 하나의 회전 변형 (§1.2 `RotRecord` 의 의미만 옮긴다).
#[derive(Clone, Debug)]
pub struct Shape {
    pub item_id: i64,
    pub rotation_deg: f64,
    /// 회전한 원본 외곽선을 bbox 최소점이 원점에 오도록 옮긴 것 (격자, 반시계).
    pub orig: Vec<Pt>,
    /// 같은 좌표계의 충돌 외곽선. 겹침 허용 `d` 면 `d/2` 만큼 수축한다.
    pub coll: Vec<Pt>,
    /// 원본 외곽선의 bbox — 언제나 `[0, w] × [0, h]`.
    pub bbox: BBox,
    /// 원본 면적 (mm²). 밀도 분자는 입력 도형의 실제 면적이다.
    pub area_mm2: f64,
    /// 출력 변환 복원용: 회전한 원본의 bbox 최소점 (mm).
    pub min_mm: (f64, f64),
    /// NFP 캐시 키 — 충돌 외곽선이 같은 형상끼리 공유한다.
    pub geom_key: usize,
    /// 회전한 원본 외곽선 (mm, 정규화 전). 원본 인증용.
    pub raw_mm: Vec<(f64, f64)>,
}

/// 배치할 조각 하나 (수량만큼 복제된다. 복제본은 원래 `id` 를 유지한다).
#[derive(Clone, Debug)]
pub struct Piece {
    pub item_id: i64,
    /// 허용 회전별 `Instance::shapes` 인덱스.
    pub shapes: Vec<usize>,
}

pub struct Instance {
    pub name: String,
    /// 스트립 높이 (격자).
    pub strip_height: i64,
    pub strip_height_mm: f64,
    pub shapes: Vec<Shape>,
    pub pieces: Vec<Piece>,
    pub overlap_mm: f64,
    pub total_area_mm2: f64,
}

/// 확정된 배치 하나. `pos` 는 `Shape` 좌표계 원점이 놓인 격자 위치다.
#[derive(Clone, Copy, Debug)]
pub struct Placement {
    pub piece: usize,
    pub shape: usize,
    pub pos: Pt,
}

/// 격자 반올림이 외곽선을 안쪽으로 최대 `0.5·√2` 칸 밀 수 있고, 단순화가 1칸을 더한다.
/// 수축량에서 이만큼을 **빼서** 충돌 외곽선이 이상적 수축본(`원본 ⊖ d/2`)을 항상 덮게
/// 한다 — 그래야 충돌 외곽선 기준 무겹침이 원본 기준 겹침 `d` 이하를 보장한다.
/// (더하면 충돌 외곽선이 작아져 원본 겹침이 `d + 0.04mm` 까지 커진다.)
const DEFLATE_GUARD_MM: f64 = 0.02;

fn to_grid(v: f64) -> i64 {
    (v * GRID_PER_MM).round() as i64
}

fn poly_area_mm2(pts: &[(f64, f64)]) -> f64 {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let (x1, y1) = pts[i];
            let (x2, y2) = pts[(i + 1) % n];
            x1 * y2 - x2 * y1
        })
        .sum::<f64>()
        .abs()
        / 2.0
}

fn deflate(pts_mm: &[(f64, f64)], by_mm: f64) -> Result<Vec<(f64, f64)>> {
    let paths: Paths<Centi> = vec![pts_mm.to_vec()].into();
    let shrunk = inflate(paths, -by_mm, JoinType::Round, EndType::Polygon, 2.0);
    let shrunk = simplify(shrunk, 0.01, false);
    let mut best: Option<Vec<(f64, f64)>> = None;
    let mut best_area = 0.0;
    let mut n_parts = 0;
    for path in shrunk.iter() {
        let v: Vec<(f64, f64)> = path.iter().map(|p| (p.x(), p.y())).collect();
        let a = poly_area_mm2(&v);
        n_parts += 1;
        if a > best_area {
            best_area = a;
            best = Some(v);
        }
    }
    if n_parts > 1 {
        eprintln!("[io] 수축 결과가 {n_parts} 조각으로 나뉘어 가장 큰 것만 쓴다");
    }
    best.context("수축 결과가 비었다 — 겹침 허용이 조각보다 크다")
}

pub fn load(path: &std::path::Path, overlap_mm: f64) -> Result<Instance> {
    load_with(path, overlap_mm, 0.0)
}

/// `edge_inset_mm` > 0 이면 모든 조각의 원본을 원단 벽에서 그만큼 떨어뜨려 놓게 한다(좌·하·상).
///
/// 지시서 기본은 0 이다. Swarm 에 warm start 로 넘길 때만 쓴다 — Swarm 은 가져온 배치의 원단 포함을
/// 자기 충돌 형상(겹침 허용치 절반만큼 줄이고 외곽을 부풀리는 단순화를 거친 도형)의 bbox 로
/// 정확히 검사해서, 원본이 벽에 딱 붙은 배치는 "outside the fixed strip bounds" 로 통째로 버린다.
pub fn load_with(path: &std::path::Path, overlap_mm: f64, edge_inset_mm: f64) -> Result<Instance> {
    let g = edge_inset_mm.max(0.0);
    let text = std::fs::read_to_string(path).with_context(|| format!("{}", path.display()))?;
    let ext: ExtInstance = serde_json::from_str(&text).context("입력 JSON")?;
    let mut shapes: Vec<Shape> = Vec::new();
    let mut pieces: Vec<Piece> = Vec::new();
    let mut geom_keys: HashMap<Vec<Pt>, usize> = HashMap::new();
    let mut total_area = 0.0;

    for item in &ext.items {
        if !matches!(item.shape.kind.as_str(), "simple_polygon" | "polygon") {
            // §1.6.6: 다중 윤곽(구멍 있는 형상)은 V1 에서 명시적 오류로 거부한다.
            bail!("item {}: 지원하지 않는 형상 종류 '{}'", item.id, item.shape.kind);
        }
        let orientations = if item.allowed_orientations.is_empty() {
            vec![0.0]
        } else {
            item.allowed_orientations.clone()
        };
        let area = poly_area_mm2(&item.shape.data.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>());
        let mut shape_ids = Vec::new();
        for &deg in &orientations {
            let (s, c) = deg.to_radians().sin_cos();
            let raw: Vec<(f64, f64)> =
                item.shape.data.iter().map(|p| (p[0] * c - p[1] * s, p[0] * s + p[1] * c)).collect();
            let min_x = raw.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
            let min_y = raw.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
            let norm: Vec<(f64, f64)> = raw.iter().map(|p| (p.0 - min_x + g, p.1 - min_y + g)).collect();
            // bbox 는 반올림한 격자 외곽선이 아니라 원본 실수 좌표를 **올림**해서 잡는다. 반올림
            // bbox 로 원단 경계(IFP)를 잡으면 원본이 원단 밖으로 최대 0.005mm 나가고, Swarm 은
            // warm start 로 받은 배치의 원단 포함을 정확히 검사해서 배치 전체를 버린다
            // (`src/ffi_step.rs` legacy_seed_problem, 실측: "seeded item 7 lies outside the fixed
            // strip bounds"). 최소점은 정규화로 정확히 0 이다.
            let max_x = norm.iter().map(|p| p.0).fold(0.0, f64::max);
            let max_y = norm.iter().map(|p| p.1).fold(0.0, f64::max);
            let orig = cleanup(&norm.iter().map(|p| Pt::new(to_grid(p.0), to_grid(p.1))).collect::<Vec<_>>());
            let coll_mm = if overlap_mm > 0.0 {
                deflate(&norm, overlap_mm / 2.0 - DEFLATE_GUARD_MM)?
            } else {
                norm.clone()
            };
            let coll = cleanup(&coll_mm.iter().map(|p| Pt::new(to_grid(p.0), to_grid(p.1))).collect::<Vec<_>>());
            if orig.len() < 3 || coll.len() < 3 || area2(&coll) <= 0 {
                bail!("item {} ({deg}°): 퇴화 형상", item.id);
            }
            let next = geom_keys.len();
            let geom_key = *geom_keys.entry(coll.clone()).or_insert(next);
            shape_ids.push(shapes.len());
            shapes.push(Shape {
                item_id: item.id,
                rotation_deg: deg,
                bbox: {
                    let b = BBox::of(&orig);
                    BBox {
                        x0: 0,
                        y0: 0,
                        x1: b.x1.max(((max_x + g) * GRID_PER_MM).ceil() as i64),
                        y1: b.y1.max(((max_y + g) * GRID_PER_MM).ceil() as i64),
                    }
                },
                orig,
                coll,
                area_mm2: area,
                min_mm: (min_x - g, min_y - g),
                geom_key,
                raw_mm: raw,
            });
        }
        for _ in 0..item.demand {
            pieces.push(Piece { item_id: item.id, shapes: shape_ids.clone() });
            total_area += area;
        }
    }

    Ok(Instance {
        name: ext.name,
        strip_height: to_grid(ext.strip_height),
        strip_height_mm: ext.strip_height,
        shapes,
        pieces,
        overlap_mm,
        total_area_mm2: total_area,
    })
}

impl Instance {
    /// 배치의 출력 변환 `(회전°, translation mm)`. 원본 좌표를 회전한 뒤 더하는 값이다.
    pub fn transformation(&self, p: &Placement) -> (f64, [f64; 2]) {
        self.transformation_mirrored(p, None)
    }

    /// `mirror_x` 가 `Some(W)` 이면 x 를 `W` 기준으로 뒤집는다 (§12.1.27 후속).
    ///
    /// **한계**: 저장 형식이 `회전 + 이동`(반사 없음)이라 진짜 반사는 표현할 수 없다.
    /// 대신 조각의 **bbox 구간만** `W` 기준으로 접는다 — 조각 `[lo, hi]` 가
    /// `[W − hi, W − lo]` 로 간다. 외곽선 자체는 반사되지 않고 평행이동만 하므로,
    /// 결과가 원본의 거울상과 같고 무겹침이 보존되는 것은 **x 축 대칭 외곽선**(직사각형
    /// 등)에서만이다. 비대칭 외곽선은 접은 자리에서 이웃과 겹칠 수 있다 — 대조용이다.
    /// 왼쪽 벽이든 오른쪽 벽이든 같은 문제라 **폭은 줄지 않는다**.
    ///
    /// 이동값 유도: 회전한 원본(`raw_mm`)의 최소 x 가 `min_mm.0` 이고, 평행이동 뒤 조각의
    /// 좌측 끝이 `W − shape_w − pos.x/G` 여야 하므로
    /// `tx' = W − shape_w − pos.x/G − min_mm.0 = W − shape_w − tx − 2·min_mm.0` 이다.
    /// (`min_mm.0` 이 0 인 입력에서는 종전 식 `W − shape_w − tx` 와 같다.)
    pub fn transformation_mirrored(&self, p: &Placement, mirror_x: Option<f64>) -> (f64, [f64; 2]) {
        let s = &self.shapes[p.shape];
        let tx = p.pos.x as f64 / GRID_PER_MM - s.min_mm.0;
        let ty = p.pos.y as f64 / GRID_PER_MM - s.min_mm.1;
        match mirror_x {
            Some(w) => {
                let shape_w = s.bbox.x1 as f64 / GRID_PER_MM;
                (s.rotation_deg, [w - shape_w - tx - 2.0 * s.min_mm.0, ty])
            }
            None => (s.rotation_deg, [tx, ty]),
        }
    }

    /// 배치된 원본 외곽선 (mm). 인증은 반올림 전 실수 좌표로 한다.
    pub fn placed_original_mm(&self, p: &Placement) -> Vec<(f64, f64)> {
        self.placed_original_mm_mirrored(p, None)
    }

    /// `mirror_x` 적용판.
    ///
    /// `raw_mm` 은 **이미 회전한** 원본 외곽선이다(`load_with` 가 `rotation_deg` 로 돌려
    /// 저장한다). 따라서 여기서는 `transformation_mirrored` 의 이동값만 더한다 — 종전에는
    /// 회전을 한 번 더 적용해 회전 허용 인스턴스에서 외곽선이 두 번 돌아갔다.
    pub fn placed_original_mm_mirrored(&self, p: &Placement, mirror_x: Option<f64>) -> Vec<(f64, f64)> {
        let s = &self.shapes[p.shape];
        let (_rot, [tx, ty]) = self.transformation_mirrored(p, mirror_x);
        s.raw_mm.iter().map(|&(x, y)| (x + tx, y + ty)).collect()
    }

    /// 사용 폭 (mm) — 배치된 원본 외곽선의 최대 x.
    pub fn used_width_mm(&self, placements: &[Placement]) -> f64 {
        placements
            .iter()
            .flat_map(|p| self.placed_original_mm(p))
            .map(|(x, _)| x)
            .fold(0.0, f64::max)
    }
}

/// 기존 검증 도구가 읽는 형식으로 저장한다.
pub fn write_solution(path: &std::path::Path, inst: &Instance, placements: &[Placement]) -> Result<f64> {
    write_solution_mirrored(path, inst, placements, None)
}

/// `mirror_x` 가 `Some(W)` 이면 저장 좌표계를 x 반전한다. 폭·밀도는 불변이다.
pub fn write_solution_mirrored(
    path: &std::path::Path,
    inst: &Instance,
    placements: &[Placement],
    mirror_x: Option<f64>,
) -> Result<f64> {
    let width = inst.used_width_mm(placements);
    let placed_area: f64 = placements.iter().map(|p| inst.shapes[p.shape].area_mm2).sum();
    let density = if width > 0.0 { placed_area / (width * inst.strip_height_mm) } else { 0.0 };
    let items: Vec<serde_json::Value> = placements
        .iter()
        .map(|p| {
            let (rot, t) = inst.transformation_mirrored(p, mirror_x);
            serde_json::json!({
                "item_id": inst.pieces[p.piece].item_id,
                "transformation": { "rotation": rot, "translation": t }
            })
        })
        .collect();
    let doc = serde_json::json!({
        "strip_width": width,
        "layout": { "container_id": 0, "placed_items": items, "density": density },
        "density": density,
    });
    std::fs::write(path, serde_json::to_string_pretty(&doc)?)?;
    Ok(width)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §12.1.27 후속 — `--mirror` 는 배치를 `W` 기준으로 접는다: 모든 조각의 x 구간이
    /// `[lo, hi] → [W−hi, W−lo]` 로 가고, 폭·밀도는 그대로다.
    ///
    /// item 0 은 원본 좌표가 원점에서 떨어져 있다(`min_mm ≠ 0`) — 종전 식
    /// `W − shape_w − tx` 는 이 경우 `2·min_mm.0` 만큼 어긋났다.
    #[test]
    fn mirror_maps_every_piece_to_w_minus_x() {
        let path = std::env::temp_dir().join(format!("nesting_engine_mirror_{}.json", std::process::id()));
        std::fs::write(
            &path,
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[1,1],[4,1],[4,3],[1,3]]}},
                {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[2,0],[2,5],[0,5]]}}
            ]}"#,
        )
        .unwrap();
        let inst = load(&path, 0.0).unwrap();
        let _ = std::fs::remove_file(&path);

        let placed = vec![
            Placement { piece: 0, shape: inst.pieces[0].shapes[0], pos: Pt::new(0, 0) },
            Placement { piece: 1, shape: inst.pieces[1].shapes[0], pos: Pt::new(300, 0) },
        ];
        let w = inst.used_width_mm(&placed);
        assert!(w > 0.0);

        let plain = std::env::temp_dir().join(format!("nesting_engine_mirror_a_{}.json", std::process::id()));
        let mir = std::env::temp_dir().join(format!("nesting_engine_mirror_b_{}.json", std::process::id()));
        write_solution_mirrored(&plain, &inst, &placed, None).unwrap();
        write_solution_mirrored(&mir, &inst, &placed, Some(w)).unwrap();

        let span = |file: &std::path::Path, item_id: i64| -> (f64, f64) {
            let doc: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(file).unwrap()).unwrap();
            let it = doc["layout"]["placed_items"].as_array().unwrap().iter()
                .find(|p| p["item_id"].as_i64() == Some(item_id)).unwrap();
            let t = &it["transformation"]["translation"];
            let rot: f64 = it["transformation"]["rotation"].as_f64().unwrap();
            let (tx, ty) = (t[0].as_f64().unwrap(), t[1].as_f64().unwrap());
            let (c, s) = (rot.to_radians().cos(), rot.to_radians().sin());
            let shape = &inst.shapes[inst.pieces.iter().find(|p| p.item_id == item_id).map(|_| 0usize)
                .map(|_| if item_id == 0 { 0 } else { 1 }).unwrap()];
            let xs: Vec<f64> = shape.raw_mm.iter().map(|&(x, y)| x * c - y * s + tx).collect();
            (xs.iter().cloned().fold(f64::MAX, f64::min), xs.iter().cloned().fold(f64::MIN, f64::max))
        };

        for item in [0i64, 1] {
            let (lo, hi) = span(&plain, item);
            let (mlo, mhi) = span(&mir, item);
            assert!((mlo - (w - hi)).abs() < 0.02, "item {item}: {mlo} != {}", w - hi);
            assert!((mhi - (w - lo)).abs() < 0.02, "item {item}: {mhi} != {}", w - lo);
        }
        let _ = std::fs::remove_file(&plain);
        let _ = std::fs::remove_file(&mir);
    }

    /// `raw_mm` 은 이미 회전돼 있으므로 `placed_original_mm` 은 이동만 더해야 한다.
    /// 3×1 막대를 90° 돌려 `pos = (200, 0)` 에 놓으면 원본은 x ∈ [2, 3], y ∈ [0, 3] 을
    /// 차지한다. 회전을 한 번 더 적용하면 x ∈ [0, 3], y ∈ [−3, 0] 이 되어 틀린다.
    #[test]
    fn placed_original_applies_rotation_exactly_once() {
        let path = std::env::temp_dir().join(format!("nesting_engine_rot_{}.json", std::process::id()));
        std::fs::write(
            &path,
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":1,"allowed_orientations":[90],"shape":{"type":"simple_polygon","data":[[0,0],[3,0],[3,1],[0,1]]}}
            ]}"#,
        )
        .unwrap();
        let inst = load(&path, 0.0).unwrap();
        let _ = std::fs::remove_file(&path);
        let p = Placement { piece: 0, shape: inst.pieces[0].shapes[0], pos: Pt::new(200, 0) };
        let pts = inst.placed_original_mm(&p);
        let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for &(x, y) in &pts {
            x0 = x0.min(x);
            x1 = x1.max(x);
            y0 = y0.min(y);
            y1 = y1.max(y);
        }
        assert!((x0 - 2.0).abs() < 1e-9 && (x1 - 3.0).abs() < 1e-9, "x ∈ [{x0}, {x1}]");
        assert!(y0.abs() < 1e-9 && (y1 - 3.0).abs() < 1e-9, "y ∈ [{y0}, {y1}]");
        assert!((inst.used_width_mm(&[p]) - 3.0).abs() < 1e-9);
    }
}
