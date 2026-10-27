//! **상용 최종 배치의 구조를 읽는다** — 3초에 77~78% 를 만드는 구성 규칙의 흔적을 찾는다.
//!
//! # 배경
//!
//! 상용은 실행 후 약 3초, 구성 단계만으로 77~78% 에 닿고(목격치) 그 구조가 82.28% 까지
//! 이어진다. 우리 구성기는 같은 시간에 70~73% 이고, 폭만 높인 시드는 최종에 전달되지
//! 않는다. 공차(겹침 허용)는 답이 아니었다(사용자 실측). 남는 후보는 **평가 기준**(접촉
//! 길이·정확 맞춤)과 **배치 순서/묶음 구조**다. 초기 배치 좌표는 없지만 최종 좌표는
//! 있고, 구성 규칙은 최종 배치에 흔적을 남긴다.
//!
//! # 무엇을 재나 (상용 vs 우리 LBF 시드 vs 우리 오목도 시드, 같은 축)
//!
//! 1. **순서 서명** — x 순위와 (면적 / 높이 / 오목도 점수 / 왼쪽·아래 오목도) 의 순위 상관.
//!    큰 것부터면 면적 상관이 강하게 음수, 오목도부터면 오목도 상관이 음수.
//! 2. **접촉 길이** — 인접 조각 경계가 서로 0.5mm 안에 있는 길이의 합 / 전체 둘레.
//!    접촉 *길이* 를 극대화하는 규칙이면 이 값이 우리보다 뚜렷이 높다.
//! 3. **동일 형상 묶음** — 면적·bbox 가 같은 조각 그룹이 서로 이웃(간격 < 1mm)인 비율.
//! 4. **열/행 구조** — x 순으로 훑을 때 y 가 단조로 오르내리는 런 길이(열 채움 서명).
//!
//! ```powershell
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test competitor_structure -- --ignored --nocapture --test-threads=1
//! ```
#![cfg(feature = "adaptive")]

use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::io::export::int_to_ext_transformation;
use jagua_rs::io::import::Importer;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::{DEFAULT_SWARM_CONFIG, RasterNfpConfig};
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::concavity::analyze_spolygon;
use swarm_core::optimizer::lbf::{LBFBuilder, LbfOrderMode};
use swarm_core::util::io::read_spp_input;
use std::path::Path;

type Poly = Vec<(f64, f64)>;

/// 마커 경로. `SWARM_INPUT` 으로 바꾼다(기본 `data/input/wsjk156.json`).
fn input_path() -> std::path::PathBuf {
    match std::env::var("SWARM_INPUT") {
        Ok(p) => std::path::PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input").join("wsjk156.json"),
    }
}

fn competitor_path() -> std::path::PathBuf {
    match std::env::var("COMPETITOR_JSON") {
        Ok(p) => std::path::PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/reference").join("competitor_wsjk156.json"),
    }
}

/// 겹침 허용(mm). `CS_OVERLAP_MM=0.8` 이면 조각을 변당 0.4mm 줄인 기하로 읽는다.
/// competitor_warm_start.rs 와 같은 관례(음수 separation)를 쓴다.
fn overlap_mm() -> f32 {
    std::env::var("CS_OVERLAP_MM").ok().and_then(|v| v.trim().parse().ok()).unwrap_or(0.0)
}

fn load() -> SPInstance {
    let path = input_path();
    let (ext, _) = read_spp_input(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let c = DEFAULT_SWARM_CONFIG;
    let overlap = overlap_mm();
    let separation = if overlap > 0.0 { Some(-overlap) } else { c.min_item_separation };
    let _ = Importer::new;
    swarm_core::util::io::import_spp_instance(
        &ext,
        c.cde_config,
        c.poly_simpl_tolerance,
        separation,
        c.narrow_concavity_cutoff_ratio,
    )
    .expect("import")
}

// ───────── 형상 특징과 대응 (competitor_warm_start.rs 와 같은 방식) ─────────

#[derive(Clone, Copy)]
struct Sig { area: f64, w: f64, h: f64, cx: f64, cy: f64, n20: f64, n02: f64, n11: f64, dx: f64, dy: f64 }

fn signature(pts: &[(f64, f64)]) -> Sig {
    let n = pts.len();
    let (mut a2, mut cx, mut cy, mut m20, mut m02, mut m11) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for i in 0..n {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % n];
        let cr = x0 * y1 - x1 * y0;
        a2 += cr; cx += (x0 + x1) * cr; cy += (y0 + y1) * cr;
        m20 += (x0 * x0 + x0 * x1 + x1 * x1) * cr;
        m02 += (y0 * y0 + y0 * y1 + y1 * y1) * cr;
        m11 += (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) * cr;
    }
    let s = if a2 < 0.0 { -1.0 } else { 1.0 };
    let area = (a2 / 2.0).abs().max(1e-9);
    let (cx, cy) = (s * cx / (6.0 * area), s * cy / (6.0 * area));
    let (m20, m02, m11) = (s * m20 / 12.0, s * m02 / 12.0, s * m11 / 24.0);
    let (mu20, mu02, mu11) = (m20 - area * cx * cx, m02 - area * cy * cy, m11 - area * cx * cy);
    let (x0, x1) = pts.iter().fold((f64::MAX, f64::MIN), |(l, h), p| (l.min(p.0), h.max(p.0)));
    let (y0, y1) = pts.iter().fold((f64::MAX, f64::MIN), |(l, h), p| (l.min(p.1), h.max(p.1)));
    let (w, h) = ((x1 - x0).max(1e-9), (y1 - y0).max(1e-9));
    Sig { area, w, h, cx, cy, n20: mu20 / (area * area), n02: mu02 / (area * area), n11: mu11 / (area * area),
          dx: (cx - (x0 + x1) / 2.0) / w, dy: (cy - (y0 + y1) / 2.0) / h }
}

fn dist(a: &Sig, b: &Sig) -> f64 {
    let rel = |p: f64, q: f64| ((p - q) / q.abs().max(1e-9)).abs();
    rel(a.area, b.area) * 20.0 + rel(a.w, b.w) * 10.0 + rel(a.h, b.h) * 10.0
        + ((a.n20 - b.n20).powi(2) + (a.n02 - b.n02).powi(2) + (a.n11 - b.n11).powi(2)).sqrt() * 50.0
        + ((a.dx - b.dx).powi(2) + (a.dy - b.dy).powi(2)).sqrt() * 5.0
}

fn assign(items: &[Sig], polys: &[Sig]) -> (Vec<usize>, f64) {
    let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
    for (pi, p) in polys.iter().enumerate() { for (ii, it) in items.iter().enumerate() { pairs.push((dist(p, it), pi, ii)); } }
    pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let mut map = vec![usize::MAX; polys.len()];
    let mut used = vec![false; items.len()];
    let mut total = 0.0;
    for (d, pi, ii) in pairs {
        if map[pi] != usize::MAX || used[ii] { continue; }
        map[pi] = ii; used[ii] = true; total += d;
    }
    (map, total)
}

/// 상용 파일 → (조각별 다각형, item_id). 전역 미러 네 가설 중 최선을 고른다.
fn competitor_layout(inst: &SPInstance, item_sigs: &[Sig]) -> Vec<(usize, Poly)> {
    let path = competitor_path();
    let doc: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))).unwrap();
    let raw: Vec<Poly> = doc["polygons"].as_array().unwrap().iter()
        .map(|p| p.as_array().unwrap().iter().map(|v| (v[0].as_f64().unwrap(), v[1].as_f64().unwrap())).collect()).collect();
    let height = inst.base_strip.fixed_height as f64;
    let (min_x, min_y) = raw.iter().flatten().fold((f64::MAX, f64::MAX), |(a, b), p| (a.min(p.0), b.min(p.1)));
    let max_x = raw.iter().flatten().fold(f64::MIN, |a, p| a.max(p.0));
    let len = max_x - min_x;
    let norm: Vec<Poly> = raw.iter().map(|p| p.iter().map(|&(x, y)| (x - min_x, y - min_y)).collect()).collect();
    let flips: [(&str, fn((f64, f64), f64, f64) -> (f64, f64)); 4] = [
        ("none", |p, _, _| p), ("flipY", |p, _, h| (p.0, h - p.1)), ("flipX", |p, l, _| (l - p.0, p.1)), ("both", |p, l, h| (l - p.0, h - p.1)),
    ];
    let mut best: Option<(f64, Vec<(usize, Poly)>)> = None;
    for (name, f) in flips {
        let polys: Vec<Poly> = norm.iter().map(|p| p.iter().map(|&q| f(q, len, height)).collect()).collect();
        let sigs: Vec<Sig> = polys.iter().map(|p| signature(p)).collect();
        let (map, total) = assign(item_sigs, &sigs);
        if best.as_ref().map_or(true, |b| total < b.0) {
            best = Some((total, polys.into_iter().enumerate().map(|(pi, p)| (map[pi], p)).collect()));
            println!("  대응 가설 {name}: 총 불일치 {total:.3}");
        }
    }
    best.unwrap().1
}

/// FullBench 가 저장한 solution JSON(ExtSPSolution) → (item_id, 실제 도형 다각형).
/// 내보내기 좌표는 원본 도형(`shape_orig`) 기준이므로 원본 정점에 회전 후 평행이동을 적용한다.
fn our_solution_from_json(inst: &SPInstance, path: &Path) -> Vec<(usize, Poly)> {
    let doc: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))).unwrap();
    let items = doc["layout"]["placed_items"].as_array().expect("placed_items");
    items.iter().map(|pi| {
        let id = pi["item_id"].as_u64().expect("item_id") as usize;
        let tr = &pi["transformation"];
        let rot = tr["rotation"].as_f64().expect("rotation").to_radians();
        let t = tr["translation"].as_array().expect("translation");
        let (tx, ty) = (t[0].as_f64().unwrap(), t[1].as_f64().unwrap());
        let (sin, cos) = (rot.sin(), rot.cos());
        let poly = inst.item(id).shape_orig().shape.vertices().iter()
            .map(|p| {
                let (x, y) = (p.0 as f64, p.1 as f64);
                (x * cos - y * sin + tx, x * sin + y * cos + ty)
            })
            .collect();
        (id, poly)
    }).collect()
}

/// 우리 배치 → (item_id, 실제 도형 다각형).
fn our_layout(inst: &SPInstance, sol: &SPSolution) -> Vec<(usize, Poly)> {
    sol.layout_snapshot().placed_items().values().map(|pi| {
        let item = inst.item(pi.item().idx());
        let ext = int_to_ext_transformation(&pi.d_transf(), &item.shape_orig().pre_transform);
        let placed = item.shape_orig().shape.transform_clone(&ext.compose());
        (pi.item().idx(), placed.vertices().iter().map(|p| (p.0 as f64, p.1 as f64)).collect())
    }).collect()
}

// ───────── 지표 ─────────

fn bbox(p: &Poly) -> (f64, f64, f64, f64) {
    p.iter().fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |(a, b, c, d), q| (a.min(q.0), b.min(q.1), c.max(q.0), d.max(q.1)))
}

fn seg_dist(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (vx, vy) = (b.0 - a.0, b.1 - a.1);
    let l2 = vx * vx + vy * vy;
    let t = if l2 > 0.0 { (((p.0 - a.0) * vx + (p.1 - a.1) * vy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    ((p.0 - (a.0 + t * vx)).powi(2) + (p.1 - (a.1 + t * vy)).powi(2)).sqrt()
}

fn perimeter(p: &Poly) -> f64 {
    (0..p.len()).map(|i| { let a = p[i]; let b = p[(i + 1) % p.len()]; ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt() }).sum()
}

/// 조각 A 의 둘레 중, 이웃 B 들의 경계에서 `delta` 안에 있는 길이. 2mm 간격 샘플.
fn contact_length(a: &Poly, neighbours: &[&Poly], delta: f64) -> f64 {
    let step = 2.0;
    let mut touched = 0.0;
    for i in 0..a.len() {
        let (p, q) = (a[i], a[(i + 1) % a.len()]);
        let l = ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2)).sqrt();
        let n = (l / step).ceil().max(1.0) as usize;
        for k in 0..n {
            let t = (k as f64 + 0.5) / n as f64;
            let s = (p.0 + t * (q.0 - p.0), p.1 + t * (q.1 - p.1));
            let near = neighbours.iter().any(|b| (0..b.len()).any(|j| seg_dist(s, b[j], b[(j + 1) % b.len()]) <= delta));
            if near { touched += l / n as f64; }
        }
    }
    touched
}

fn rank(v: &[f64]) -> Vec<f64> {
    let mut idx: Vec<usize> = (0..v.len()).collect();
    idx.sort_by(|&a, &b| v[a].partial_cmp(&v[b]).unwrap());
    let mut r = vec![0.0; v.len()];
    for (k, &i) in idx.iter().enumerate() { r[i] = k as f64; }
    r
}

fn spearman(a: &[f64], b: &[f64]) -> f64 {
    let (ra, rb) = (rank(a), rank(b));
    let n = a.len() as f64;
    let (ma, mb) = (ra.iter().sum::<f64>() / n, rb.iter().sum::<f64>() / n);
    let cov: f64 = ra.iter().zip(&rb).map(|(x, y)| (x - ma) * (y - mb)).sum();
    let (va, vb): (f64, f64) = (ra.iter().map(|x| (x - ma).powi(2)).sum(), rb.iter().map(|y| (y - mb).powi(2)).sum());
    cov / (va * vb).sqrt().max(1e-12)
}

struct Report { name: String, corr_area: f64, corr_h: f64, corr_conc: f64, corr_left: f64, corr_bottom: f64, contact: f64, twin_adjacent: f64, twin_groups: usize, run_len: f64, width: f64 }

fn analyze(name: &str, inst: &SPInstance, layout: &[(usize, Poly)]) -> Report {
    let n = layout.len();
    let boxes: Vec<_> = layout.iter().map(|(_, p)| bbox(p)).collect();
    let xs: Vec<f64> = boxes.iter().map(|b| (b.0 + b.2) / 2.0).collect();
    let width = boxes.iter().map(|b| b.2).fold(f64::MIN, f64::max);
    let areas: Vec<f64> = layout.iter().map(|(id, _)| inst.item(*id).shape_orig().area() as f64).collect();
    let hs: Vec<f64> = boxes.iter().map(|b| b.3 - b.1).collect();
    let concs: Vec<_> = layout.iter().map(|(id, _)| analyze_spolygon(&inst.item(*id).shape_orig().shape)).collect();
    let conc: Vec<f64> = concs.iter().map(|s| s.final_score).collect();
    let left: Vec<f64> = concs.iter().map(|s| s.left_score).collect();
    let bottom: Vec<f64> = concs.iter().map(|s| s.bottom_score).collect();

    // 접촉 길이 / 둘레
    let (mut touched, mut perim) = (0.0, 0.0);
    for i in 0..n {
        let bi = boxes[i];
        let nb: Vec<&Poly> = (0..n).filter(|&j| j != i).filter(|&j| {
            let bj = boxes[j];
            bj.0 <= bi.2 + 1.0 && bj.2 >= bi.0 - 1.0 && bj.1 <= bi.3 + 1.0 && bj.3 >= bi.1 - 1.0
        }).map(|j| &layout[j].1).collect();
        touched += contact_length(&layout[i].1, &nb, 0.5);
        perim += perimeter(&layout[i].1);
    }

    // 동일 형상 묶음: 면적·bbox(1mm) 동일 그룹에서 이웃(간격<1mm) 인 쌍의 비율
    let mut groups: std::collections::HashMap<(i64, i64, i64), Vec<usize>> = Default::default();
    for i in 0..n {
        let it = inst.item(layout[i].0);
        let b = it.shape_orig().shape.bbox();
        groups.entry(((it.shape_orig().area() * 10.0) as i64, ((b.x_max - b.x_min)) as i64, ((b.y_max - b.y_min)) as i64)).or_default().push(i);
    }
    let (mut twin_pairs, mut twin_adj) = (0usize, 0usize);
    for g in groups.values().filter(|g| g.len() >= 2) {
        for a in 0..g.len() { for b in a + 1..g.len() {
            twin_pairs += 1;
            let (bi, bj) = (boxes[g[a]], boxes[g[b]]);
            let gap = (bj.0 - bi.2).max(bi.0 - bj.2).max(bj.1 - bi.3).max(bi.1 - bj.3);
            if gap < 1.0 { twin_adj += 1; }
        }}
    }

    // 열 채움 서명: x 순으로 훑을 때 y 중심의 단조 런 평균 길이
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| xs[a].partial_cmp(&xs[b]).unwrap());
    let ys: Vec<f64> = order.iter().map(|&i| (boxes[i].1 + boxes[i].3) / 2.0).collect();
    let (mut runs, mut cur, mut dir) = (1usize, 1usize, 0i8);
    for k in 1..ys.len() {
        let d = if ys[k] > ys[k - 1] { 1 } else { -1 };
        if dir == 0 || d == dir { cur += 1; } else { runs += 1; cur = 1; }
        dir = d;
    }
    let _ = cur;

    Report {
        name: name.to_string(),
        corr_area: spearman(&xs, &areas), corr_h: spearman(&xs, &hs), corr_conc: spearman(&xs, &conc),
        corr_left: spearman(&xs, &left), corr_bottom: spearman(&xs, &bottom),
        contact: touched / perim.max(1e-9),
        twin_adjacent: if twin_pairs > 0 { twin_adj as f64 / twin_pairs as f64 } else { 0.0 },
        twin_groups: groups.values().filter(|g| g.len() >= 2).count(),
        run_len: n as f64 / runs as f64,
        width,
    }
}

#[test]
#[ignore = "진단 — 탐색 없음, 수 초"]
fn what_structure_does_the_competitor_layout_have() {
    let inst = load();
    let c = DEFAULT_SWARM_CONFIG;
    let item_sigs: Vec<Sig> = inst.items.iter()
        .map(|(it, _)| signature(&it.shape_orig().shape.vertices().iter().map(|p| (p.0 as f64, p.1 as f64)).collect::<Vec<_>>())).collect();

    println!("\n══════ 상용 최종 배치의 구조 vs 우리 시드 ══════");
    let comp = competitor_layout(&inst, &item_sigs);
    let mut reports = vec![analyze("상용 최종 (82.28%)", &inst, &comp)];
    // 세 번째 행은 2026-09-11 실험에서 시드 폭이 가장 좋았던 구성이다(격자 0.02 / 4M 셀 / 난수 4000).
    // 셀 크기 = max(최소변 x cell_ratio, sqrt(면적 / max_cells)) 라 두 항을 같이 낮춰야 실제로 미세해진다.
    let mut fine = c.lbf_raster;
    fine.cell_ratio = 0.02;
    fine.max_cells = 4_000_000;
    fine.n_random_samples = 4_000;
    let arms: [(&str, LbfOrderMode, RasterNfpConfig); 8] = [
        ("LBF 큰것부터(기본)", LbfOrderMode::LargestFirst, c.lbf_raster),
        ("LBF 작은것부터", LbfOrderMode::SmallestFirst, c.lbf_raster),
        ("LBF 중간부터(큰쪽)", LbfOrderMode::MiddleOutLargeFirst, c.lbf_raster),
        ("LBF 중간부터(작은쪽)", LbfOrderMode::MiddleOutSmallFirst, c.lbf_raster),
        ("LBF 오목도부터", LbfOrderMode::ConcavityFirst, c.lbf_raster),
        ("LBF 크기x오목도", LbfOrderMode::SizeTimesConcavity, c.lbf_raster),
        ("LBF 큰것부터+미세격자", LbfOrderMode::LargestFirst, fine),
        ("LBF 작은것부터+미세격자", LbfOrderMode::SmallestFirst, fine),
    ];
    for (label, mode, raster) in arms {
        let built = LBFBuilder::new(inst.clone(), Xoshiro256PlusPlus::seed_from_u64(0), LBF_SAMPLE_CONFIG, c.lbf_x_multiplier, c.lbf_y_multiplier)
            .with_order_mode(mode).with_raster_config(raster).construct();
        let sol = built.prob.save();
        reports.push(analyze(label, &inst, &our_layout(&inst, &sol)));
    }

    // 상용 행은 최종 배치라 구성 직후인 위 행들과 단계가 다르다. 같은 단계로 비교하려면
    // 우리 탐색이 끝난 배치도 올려야 한다. CS_OUR_SOLUTION 에 FullBench 가 저장한 경로를 준다.
    if let Ok(sol_path) = std::env::var("CS_OUR_SOLUTION") {
        let p = std::path::PathBuf::from(sol_path);
        reports.push(analyze("우리 최종(탐색 후)", &inst, &our_solution_from_json(&inst, &p)));
    }

    println!("\n  {:22} {:>7} {:>7} {:>7} {:>7} {:>7} {:>8} {:>10} {:>7} {:>8}", "배치", "x~면적", "x~높이", "x~오목", "x~왼쪽", "x~아래", "접촉비", "동형이웃", "런길이", "폭");
    for r in &reports {
        println!("  {:22} {:+7.2} {:+7.2} {:+7.2} {:+7.2} {:+7.2} {:7.1}% {:8.0}%/{:<2} {:7.1} {:8.0}",
            r.name, r.corr_area, r.corr_h, r.corr_conc, r.corr_left, r.corr_bottom, r.contact * 100.0, r.twin_adjacent * 100.0, r.twin_groups, r.run_len, r.width);
    }
    println!("\n  읽는 법 ───────────────────────────────────────────────────────────");
    println!("   x~면적  : x 위치와 면적의 순위 상관. 큰 것부터 왼쪽에 놓으면 강한 음수.");
    println!("   x~오목/왼쪽/아래 : 오목도 점수와의 상관. 오목한 것을 왼쪽에 먼저 놓으면 음수.");
    println!("   접촉비  : 이웃 경계에서 0.5mm 안에 있는 둘레 길이 / 전체 둘레. 접촉 길이를 극대화하는 규칙이면 높다.");
    println!("   동형이웃: 동일 형상 그룹 안의 쌍 중 서로 이웃(간격<1mm)인 비율 / 그룹 수. 묶어 놓으면 높다.");
    println!("   런길이  : x 순으로 훑을 때 y 가 단조로 이어지는 평균 길이. 열 단위 채움이면 길다.\n");
}
