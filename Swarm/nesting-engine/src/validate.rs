//! 원본 기하 인증 (§8, §13.1).
//!
//! "인증은 항상 원본 기하 기준이다. 단순화 다각형 기준 무겹침은 인증이 아니다." 여기서는
//! 충돌 외곽선(수축본)이 아니라 **입력 도형을 회전·이동한 실수 좌표**로 검사한다.
//!
//! 겹침 판정은 저장소의 기존 가드(`swarm-core/src/original_overlap.rs`)와 같다 — 두
//! 원본 도형의 교집합을 `half_depth = d/2 − 0.0005mm` 만큼 침식해서 남는 넓이가 있으면
//! 허용치 `d` 를 넘은 겹침이다. 원단 경계는 원본 외곽선이 스트립 사각형을 넘는 거리로 잰다.

use crate::io::{Instance, Placement};
use clipper2::{Centi, EndType, FillRule, JoinType, Paths, inflate, intersect};

/// Clipper2 고정소수 격자 오차를 넘는 실제 겹침만 센다 (mm²).
const AREA_EPS_MM2: f64 = 1.0e-6;
const GRID_GUARD_MM: f64 = 0.0005;

#[derive(Clone, Debug)]
pub struct Report {
    pub placed: usize,
    pub total: usize,
    pub pairs_checked: usize,
    /// (item_id, item_id, 침식 후 남은 넓이 mm²)
    pub overlaps: Vec<(i64, i64, f64)>,
    /// 원단 밖으로 나간 최대 거리 (mm): 좌, 우(사용 폭 기준), 하, 상.
    pub overrun: [f64; 4],
    pub used_width_mm: f64,
    pub density: f64,
}

impl Report {
    /// 겹침 0, 원단 밖 허용치 이내, 전부 배치.
    pub fn passes(&self, fabric_budget_mm: f64) -> bool {
        self.overlaps.is_empty()
            && self.overrun.iter().all(|&o| o <= fabric_budget_mm)
            && self.placed == self.total
    }
}

fn bbox(p: &[(f64, f64)]) -> [f64; 4] {
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for &(x, y) in p {
        b[0] = b[0].min(x);
        b[1] = b[1].min(y);
        b[2] = b[2].max(x);
        b[3] = b[3].max(y);
    }
    b
}

pub fn certify(inst: &Instance, placements: &[Placement]) -> Report {
    let polys: Vec<Vec<(f64, f64)>> = placements.iter().map(|p| inst.placed_original_mm(p)).collect();
    let boxes: Vec<[f64; 4]> = polys.iter().map(|p| bbox(p)).collect();
    let width = inst.used_width_mm(placements);
    let height = inst.strip_height_mm;
    let half_depth = (inst.overlap_mm / 2.0 - GRID_GUARD_MM).max(0.0);

    let mut overrun = [0.0f64; 4];
    for b in &boxes {
        overrun[0] = overrun[0].max(-b[0]);
        overrun[1] = overrun[1].max(b[2] - width);
        overrun[2] = overrun[2].max(-b[1]);
        overrun[3] = overrun[3].max(b[3] - height);
    }

    let mut overlaps = Vec::new();
    let mut pairs = 0;
    for i in 0..polys.len() {
        for j in (i + 1)..polys.len() {
            let (a, b) = (boxes[i], boxes[j]);
            if a[2] <= b[0] || b[2] <= a[0] || a[3] <= b[1] || b[3] <= a[1] {
                continue;
            }
            pairs += 1;
            let pa: Paths<Centi> = vec![polys[i].clone()].into();
            let pb: Paths<Centi> = vec![polys[j].clone()].into();
            let Ok(common) = intersect(pa, pb, FillRule::NonZero) else {
                overlaps.push((inst.pieces[placements[i].piece].item_id, inst.pieces[placements[j].piece].item_id, f64::NAN));
                continue;
            };
            if common.is_empty() {
                continue;
            }
            let rest = if half_depth > 0.0 {
                inflate(common, -half_depth, JoinType::Bevel, EndType::Polygon, 2.0)
            } else {
                common
            };
            let area = rest.signed_area().abs();
            if area > AREA_EPS_MM2 {
                overlaps.push((
                    inst.pieces[placements[i].piece].item_id,
                    inst.pieces[placements[j].piece].item_id,
                    area,
                ));
            }
        }
    }

    let placed_area: f64 = placements.iter().map(|p| inst.shapes[p.shape].area_mm2).sum();
    Report {
        placed: placements.len(),
        total: inst.pieces.len(),
        pairs_checked: pairs,
        overlaps,
        overrun,
        used_width_mm: width,
        density: if width > 0.0 { placed_area / (width * height) } else { 0.0 },
    }
}
