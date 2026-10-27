//! **점유 래스터 맞춤 평가** — 후보 자리의 "실제 빈 면적" 을 센다.
//!
//! bbox 증가량 기준 낭비는 큰 조각 사이의 오목한 틈(bbox 안쪽 빈 곳)을 채워도 이득이
//! 0 이라 "틈 채우기" 가 성립하지 않는다(HANDOFF §9-G). 여기서는 스트립을 격자로 덮고
//! 배치된 도형을 채워 두었다가, 후보 자리에 대해 세 가지를 센다.
//!
//! * `bbox_empty` — 후보 bbox 안에서 조각도 이웃도 덮지 않는 칸 (bbox 안쪽 틈)
//! * `shadow_left` — 조각 각 행의 가장 왼쪽 칸에서 왼쪽으로, 점유 칸을 만날 때까지의 빈 칸
//! * `shadow_bottom` — 조각 각 열의 가장 아래 칸에서 아래로, 같은 방식의 빈 칸
//!
//! 그림자는 "이 조각을 여기 놓으면 뒤에 갇히는 빈 공간" 의 근사다. 상한(`cap`)을 두어
//! 열린 공간(아직 아무것도 없는 곳)을 무한 벌점으로 세지 않는다.
use jagua_rs::geometry::primitives::{Point, SPolygon};

#[derive(Clone, Debug)]
pub struct OccRaster {
    cell: f32,
    w: usize,
    h: usize,
    occ: Vec<bool>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Fit {
    pub piece_cells: usize,
    pub bbox_empty: usize,
    pub shadow_left: usize,
    pub shadow_bottom: usize,
}

impl Fit {
    /// 낭비 칸 수 합.
    pub fn waste_cells(&self) -> usize {
        self.bbox_empty + self.shadow_left + self.shadow_bottom
    }
}

/// 한 행에서 조각이 덮는 칸 구간들 (x0..=x1 칸 인덱스). 칸 중심이 구간 안이면 덮인 것.
fn row_spans(
    verts: &[Point],
    tx: f32,
    ty: f32,
    yc: f32,
    cell: f32,
    w: usize,
    out: &mut Vec<(usize, usize)>,
    xs: &mut Vec<f32>,
) {
    out.clear();
    let n = verts.len();
    xs.clear();
    for i in 0..n {
        let (x0, y0) = (verts[i].0 + tx, verts[i].1 + ty);
        let (x1, y1) = (verts[(i + 1) % n].0 + tx, verts[(i + 1) % n].1 + ty);
        if (y0 <= yc) != (y1 <= yc) {
            xs.push(x0 + (yc - y0) * (x1 - x0) / (y1 - y0));
        }
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    for pair in xs.chunks(2) {
        if pair.len() < 2 {
            break;
        }
        let c0 = ((pair[0] / cell) - 0.5).ceil().max(0.0) as usize;
        let c1 = ((pair[1] / cell) - 0.5).floor();
        if c1 < 0.0 {
            continue;
        }
        let c1 = (c1 as usize).min(w.saturating_sub(1));
        if c0 <= c1 {
            out.push((c0, c1));
        }
    }
}

impl OccRaster {
    pub fn new(strip_w: f32, strip_h: f32, cell: f32) -> Self {
        assert!(
            strip_w.is_finite()
                && strip_w >= 0.0
                && strip_h.is_finite()
                && strip_h >= 0.0
                && cell.is_finite()
                && cell > 0.0,
            "raster dimensions must be finite and nonnegative, with positive cell size"
        );
        // This is an approximate ranking grid; exact CDE still decides placement.
        // Bound its memory even for extreme aspect ratios or tiny requested cells.
        const MAX_CELLS: usize = 1_048_576;
        let dimensions = |cell: f32| {
            (
                (strip_w as f64 / cell as f64).ceil().max(1.0) as usize,
                (strip_h as f64 / cell as f64).ceil().max(1.0) as usize,
            )
        };
        let mut cell = cell;
        let (mut w, mut h) = dimensions(cell);
        if w.checked_mul(h).is_none_or(|n| n > MAX_CELLS) {
            let budget = MAX_CELLS as f64;
            cell = (cell as f64)
                .max((strip_w as f64 * strip_h as f64 / budget).sqrt())
                .max(strip_w as f64 / budget)
                .max(strip_h as f64 / budget) as f32;
            (w, h) = dimensions(cell);
            while w.checked_mul(h).is_none_or(|n| n > MAX_CELLS) {
                cell = (cell * 1.01).min(f32::MAX);
                (w, h) = dimensions(cell);
            }
        }
        OccRaster {
            cell,
            w,
            h,
            occ: vec![false; w * h],
        }
    }

    pub fn cell(&self) -> f32 {
        self.cell
    }

    #[inline(always)]
    fn at(&self, x: usize, y: usize) -> bool {
        x < self.w && y < self.h && self.occ[y * self.w + x]
    }

    fn row_range(&self, poly: &SPolygon, ty: f32) -> (usize, usize) {
        let r0 = (((poly.bbox().y_min + ty) / self.cell) - 0.5).ceil().max(0.0) as usize;
        let r1 = (((poly.bbox().y_max + ty) / self.cell) - 0.5)
            .floor()
            .max(0.0) as usize;
        let last = self.h.saturating_sub(1);
        (r0.min(last), r1.min(last))
    }

    /// 배치된 도형을 점유로 표시한다.
    pub fn mark(&mut self, poly: &SPolygon, tx: f32, ty: f32) {
        let (r0, r1) = self.row_range(poly, ty);
        let mut spans = Vec::with_capacity(4);
        let mut xs = Vec::with_capacity(8);
        for r in r0..=r1 {
            let yc = (r as f32 + 0.5) * self.cell;
            row_spans(
                &poly.vertices(),
                tx,
                ty,
                yc,
                self.cell,
                self.w,
                &mut spans,
                &mut xs,
            );
            for &(a, b) in &spans {
                self.occ[r * self.w + a..=r * self.w + b].fill(true);
            }
        }
    }

    /// 후보 자리의 맞춤을 센다 (표시하지 않는다).
    pub fn fit(&self, poly: &SPolygon, tx: f32, ty: f32, cap: usize) -> Fit {
        let (r0, r1) = self.row_range(poly, ty);
        let c0 = (((poly.bbox().x_min + tx) / self.cell) - 0.5).ceil().max(0.0) as usize;
        let c1 = ((((poly.bbox().x_max + tx) / self.cell) - 0.5)
            .floor()
            .max(0.0) as usize)
            .min(self.w.saturating_sub(1));
        let ncols = c1.saturating_sub(c0) + 1;
        let mut col_bottom: Vec<Option<usize>> = vec![None; ncols];
        let mut fit = Fit::default();
        let mut spans = Vec::with_capacity(4);
        let mut xs = Vec::with_capacity(8);
        let mut covered_row: Vec<bool> = vec![false; ncols];
        for r in r0..=r1 {
            let yc = (r as f32 + 0.5) * self.cell;
            row_spans(
                &poly.vertices(),
                tx,
                ty,
                yc,
                self.cell,
                self.w,
                &mut spans,
                &mut xs,
            );
            covered_row.iter_mut().for_each(|c| *c = false);
            let mut leftmost: Option<usize> = None;
            for &(a, b) in &spans {
                for x in a..=b {
                    if x >= c0 && x <= c1 {
                        covered_row[x - c0] = true;
                    }
                }
                fit.piece_cells += b - a + 1;
                leftmost = Some(leftmost.map_or(a, |l: usize| l.min(a)));
            }
            for x in c0..=c1 {
                let k = x - c0;
                if covered_row[k] {
                    if col_bottom[k].is_none() {
                        col_bottom[k] = Some(r);
                    }
                } else if !self.at(x, r) {
                    fit.bbox_empty += 1;
                }
            }
            if let Some(l) = leftmost {
                let mut x = l;
                let mut n = 0usize;
                while x > 0 && n < cap {
                    x -= 1;
                    if self.at(x, r) {
                        break;
                    }
                    n += 1;
                }
                fit.shadow_left += n;
            }
        }
        for (k, cb) in col_bottom.iter().enumerate() {
            let Some(rb) = *cb else { continue };
            let x = c0 + k;
            let mut r = rb;
            let mut n = 0usize;
            while r > 0 && n < cap {
                r -= 1;
                if self.at(x, r) {
                    break;
                }
                n += 1;
            }
            fit.shadow_bottom += n;
        }
        fit
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(w: f32, h: f32) -> SPolygon {
        SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(w, 0.0),
            Point(w, h),
            Point(0.0, h),
        ])
        .unwrap()
    }

    #[test]
    fn extreme_dimensions_have_bounded_memory() {
        for (w, h, cell) in [
            (f32::MAX, f32::MAX, f32::MIN_POSITIVE),
            (f32::MAX, 1.0, 0.01),
            (1.0, f32::MAX, 0.01),
            (100.0, 100.0, 1.0e-20),
        ] {
            let r = OccRaster::new(w, h, cell);
            assert!(r.occ.len() <= 1_048_576);
            assert!(r.cell.is_finite() && r.cell >= cell);
            assert!(r.w > 0 && r.h > 0);
        }
        let r = OccRaster::new(100.0, 200.0, 0.5);
        assert_eq!((r.cell, r.w, r.h), (0.5, 200, 400));
    }

    #[test]
    fn scanlines_match_independent_cell_center_oracle() {
        let polygon = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(8.0, 0.0),
            Point(8.0, 3.0),
            Point(3.0, 3.0),
            Point(3.0, 8.0),
            Point(0.0, 8.0),
        ])
        .unwrap();
        for step in 0..32 {
            let tx = step as f32 * 0.13 - 2.0;
            let ty = step as f32 * 0.17 - 2.0;
            let mut r = OccRaster::new(20.0, 20.0, 0.5);
            r.mark(&polygon, tx, ty);
            for y in 0..r.h {
                for x in 0..r.w {
                    let px = (x as f32 + 0.5) * r.cell - tx;
                    let py = (y as f32 + 0.5) * r.cell - ty;
                    let inside =
                        px >= 0.0 && px <= 8.0 && py >= 0.0 && py < 8.0 && (px <= 3.0 || py < 3.0);
                    assert_eq!(r.at(x, y), inside, "step={step}, cell=({x},{y})");
                }
            }
        }
    }

    #[test]
    fn a_square_in_the_corner_has_no_waste() {
        let r = OccRaster::new(100.0, 100.0, 1.0);
        let f = r.fit(&rect(10.0, 10.0), 0.0, 0.0, 64);
        assert_eq!(f.piece_cells, 100);
        assert_eq!(f.bbox_empty, 0);
        assert_eq!(f.shadow_left, 0);
        assert_eq!(f.shadow_bottom, 0);
    }

    #[test]
    fn a_gap_behind_the_piece_is_counted_as_shadow() {
        let mut r = OccRaster::new(100.0, 100.0, 1.0);
        r.mark(&rect(10.0, 10.0), 0.0, 0.0);
        // 왼쪽 10칸 점유 뒤에 5칸 띄우고 놓는다 → 행마다 5칸 그림자
        let f = r.fit(&rect(10.0, 10.0), 15.0, 0.0, 64);
        assert_eq!(f.shadow_left, 5 * 10);
        assert_eq!(f.shadow_bottom, 0);
        // 딱 붙이면 0
        let f = r.fit(&rect(10.0, 10.0), 10.0, 0.0, 64);
        assert_eq!(f.shadow_left, 0);
    }

    #[test]
    fn a_notch_filled_by_a_neighbour_is_not_bbox_waste() {
        // L 자 조각의 bbox 오목부에 이웃이 이미 있으면 bbox_empty 가 0 이다.
        let l = SPolygon::new(vec![
            Point(0.0, 0.0),
            Point(20.0, 0.0),
            Point(20.0, 10.0),
            Point(10.0, 10.0),
            Point(10.0, 20.0),
            Point(0.0, 20.0),
        ])
        .unwrap();
        let mut r = OccRaster::new(100.0, 100.0, 1.0);
        let empty = r.fit(&l, 0.0, 0.0, 64);
        assert_eq!(empty.bbox_empty, 100);
        r.mark(&rect(10.0, 10.0), 10.0, 10.0);
        let filled = r.fit(&l, 0.0, 0.0, 64);
        assert_eq!(filled.bbox_empty, 0);
    }
}
