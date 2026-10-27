//! **상용 배치를 우리 탐색의 시드로** — B 팔. 우리 탐색이 그 품질의 해를 유지하는가,
//! 나빠뜨리는가, 더 좁히는가.
//!
//! # 왜 결정적인가
//!
//! `data/reference/competitor_wsjk156.json` 은 상용 마커의 실제 다각형 좌표다
//! (HPGL 변환, 우리 밀도 정의로 **82.28%**, 회전 0·겹침 0). 우리 최고는 10분 80.92% /
//! 30분 81.10% 다. 남은 1.2~1.4%p 가 **탐색의 한계**인지 **출발 구조의 차이**인지는
//! 이 배치에서 우리 탐색을 돌려 봐야만 갈린다.
//!
//! * 나빠진다 → 우리 탐색은 그 품질의 해를 다룰 수 없다(분리기·평가기 문제).
//! * 유지만 한다 → 82.3% 근방이 이 분리기의 고정점이다.
//! * 더 좁아진다 → 격차는 탐색이 아니라 **출발 구조**다(시드/초기 궤적 문제).
//!
//! # 조각 대응 — 파일에는 `item_id` 가 없다
//!
//! 다각형뿐이라 우리 인스턴스의 조각과 대응을 만들어야 한다. 플로터 출력이라 정점이
//! 단순화돼 있어(median 51 vs 원본 179) 정확한 기하 해시는 안 맞는다. 대신
//! **이동에 불변인 특징** — 면적, bbox 폭·높이, 중심 2차 모멘트(`mu20/mu02/mu11`),
//! bbox 중심 대비 무게중심 편이 — 로 맞춘다. 면적이 같은 조각들(좌우 쌍 등 61개)은
//! 동일 형상이라 어느 쪽에 대응시켜도 무방하다.
//!
//! 파일 `notes` 가 "전체가 x축 기준 전역 미러" 라고 적었으므로 네 가설
//! (그대로 / y 반전 / x 반전 / 둘 다)을 전부 대응시켜 **총 불일치가 가장 작은 것**을
//! 고른다. 미러 판정에는 `mu11` 부호와 무게중심 편이 부호가 쓰인다(회전 180°는
//! 2차 모멘트로 구분되지 않으므로 편이가 필요하다).
//!
//! # 우리 충돌 기하는 보수적이다
//!
//! 우리 충돌 도형은 단순화로 x1.00098 부풀어 있고, 상용 배치의 인접 간격 median 은
//! 0.187mm 다. 그러므로 상용 배치가 **우리 CDE 에서는 겹침으로 보일 수 있다.** 그건
//! 결함이 아니라 기하 정의 차이이고, 탐색의 첫 일이 그 해소가 된다. 진입 손실을 찍는다.
//!
//! ```powershell
//! $env:WS_SECS='480'; $env:WS_CMPR_SECS='120'; $env:WS_SEED='0'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test competitor_warm_start -- --ignored --nocapture --test-threads=1
//! ```
#![cfg(feature = "adaptive")]

use jagua_rs::geometry::DTransformation;
use jagua_rs::io::import::{Importer, ext_to_int_transformation};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

fn env_u64(key: &str, default: u64) -> u64 {
    std::env::var(key).ok().and_then(|v| v.trim().parse().ok()).unwrap_or(default)
}

fn input_path() -> PathBuf {
    match std::env::var("SWARM_INPUT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input").join("wsjk156.json"),
    }
}

fn competitor_path() -> PathBuf {
    match std::env::var("COMPETITOR_JSON") {
        Ok(p) => PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/reference")
            .join("competitor_wsjk156.json"),
    }
}

/// 기하 선택. `WS_GEOMETRY=exact` 면 단순화·좁은 오목부 메움을 끈다(부풀림 x1.0000,
/// 정점 6배). `WS_OVERLAP_MM=2` 면 겹침 허용을 켠다(원단 경계는 `import_spp_instance`
/// 가 닫는다). 프로덕션과 같은 import 경로를 쓴다.
fn geometry_label() -> String {
    let exact = std::env::var("WS_GEOMETRY").map(|v| v.trim() == "exact").unwrap_or(false);
    let overlap = std::env::var("WS_OVERLAP_MM").unwrap_or_else(|_| "0".into());
    format!("{} · 겹침 {overlap}mm", if exact { "정확(단순화 없음)" } else { "기본(단순화 x1.00098)" })
}

fn load(path: &Path) -> SPInstance {
    let (ext, _) = read_spp_input(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let c = DEFAULT_SWARM_CONFIG;
    let exact = std::env::var("WS_GEOMETRY").map(|v| v.trim() == "exact").unwrap_or(false);
    let overlap: f32 = std::env::var("WS_OVERLAP_MM").ok().and_then(|v| v.trim().parse().ok()).unwrap_or(0.0);
    let separation = if overlap > 0.0 { Some(-overlap) } else { c.min_item_separation };
    let _ = Importer::new;
    swarm_core::util::io::import_spp_instance(
        &ext,
        c.cde_config,
        if exact { None } else { c.poly_simpl_tolerance },
        separation,
        if exact { None } else { c.narrow_concavity_cutoff_ratio },
    )
    .expect("import")
}

/// 이동 불변 형상 특징. 전부 스케일 정규화해 한 거리로 비교한다.
#[derive(Clone, Copy, Debug)]
struct Sig {
    area: f64,
    w: f64,
    h: f64,
    cx: f64,
    cy: f64,
    /// 중심 2차 모멘트를 면적²으로 정규화.
    n20: f64,
    n02: f64,
    n11: f64,
    /// 무게중심 − bbox 중심, bbox 크기로 정규화. 미러/180° 를 가른다.
    dx: f64,
    dy: f64,
}

fn signature(pts: &[(f64, f64)]) -> Sig {
    let n = pts.len();
    let (mut a2, mut cx, mut cy, mut m20, mut m02, mut m11) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for i in 0..n {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % n];
        let cr = x0 * y1 - x1 * y0;
        a2 += cr;
        cx += (x0 + x1) * cr;
        cy += (y0 + y1) * cr;
        m20 += (x0 * x0 + x0 * x1 + x1 * x1) * cr;
        m02 += (y0 * y0 + y0 * y1 + y1 * y1) * cr;
        m11 += (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) * cr;
    }
    let area = a2 / 2.0; // 부호는 감기 방향 — 미러 가설에서 뒤집히므로 abs 로 정규화
    let s = if area < 0.0 { -1.0 } else { 1.0 };
    let area = area.abs().max(1e-9);
    let cx = s * cx / (6.0 * area);
    let cy = s * cy / (6.0 * area);
    let m20 = s * m20 / 12.0;
    let m02 = s * m02 / 12.0;
    let m11 = s * m11 / 24.0;
    let (mu20, mu02, mu11) = (m20 - area * cx * cx, m02 - area * cy * cy, m11 - area * cx * cy);
    let (x0, x1) = pts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.0), hi.max(p.0)));
    let (y0, y1) = pts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(p.1), hi.max(p.1)));
    let (w, h) = ((x1 - x0).max(1e-9), (y1 - y0).max(1e-9));
    Sig {
        area,
        w,
        h,
        cx,
        cy,
        n20: mu20 / (area * area),
        n02: mu02 / (area * area),
        n11: mu11 / (area * area),
        dx: (cx - (x0 + x1) / 2.0) / w,
        dy: (cy - (y0 + y1) / 2.0) / h,
    }
}

/// 두 특징의 거리. 면적·bbox 는 상대 오차, 모멘트·편이는 절대 차.
fn dist(a: &Sig, b: &Sig) -> f64 {
    let rel = |p: f64, q: f64| ((p - q) / q.abs().max(1e-9)).abs();
    let d_area = rel(a.area, b.area) * 20.0;
    let d_w = rel(a.w, b.w) * 10.0;
    let d_h = rel(a.h, b.h) * 10.0;
    let d_m = ((a.n20 - b.n20).powi(2) + (a.n02 - b.n02).powi(2) + (a.n11 - b.n11).powi(2)).sqrt() * 50.0;
    let d_c = ((a.dx - b.dx).powi(2) + (a.dy - b.dy).powi(2)).sqrt() * 5.0;
    d_area + d_w + d_h + d_m + d_c
}

#[derive(Clone, Copy, Debug)]
enum Mirror {
    None,
    FlipY,
    FlipX,
    Both,
}

fn apply(m: Mirror, pts: &[(f64, f64)], len: f64, height: f64) -> Vec<(f64, f64)> {
    pts.iter()
        .map(|&(x, y)| match m {
            Mirror::None => (x, y),
            Mirror::FlipY => (x, height - y),
            Mirror::FlipX => (len - x, y),
            Mirror::Both => (len - x, height - y),
        })
        .collect()
}

/// 탐욕 일대일 대응: 불일치가 작은 쌍부터 확정한다. `(대응, 총 불일치, 최악 불일치)`.
fn assign(items: &[Sig], polys: &[Sig]) -> (Vec<usize>, f64, f64) {
    let mut pairs: Vec<(f64, usize, usize)> = Vec::with_capacity(items.len() * polys.len());
    for (pi, p) in polys.iter().enumerate() {
        for (ii, it) in items.iter().enumerate() {
            pairs.push((dist(p, it), pi, ii));
        }
    }
    pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let mut poly_to_item = vec![usize::MAX; polys.len()];
    let mut used = vec![false; items.len()];
    let (mut total, mut worst) = (0.0, 0.0f64);
    for (d, pi, ii) in pairs {
        if poly_to_item[pi] != usize::MAX || used[ii] {
            continue;
        }
        poly_to_item[pi] = ii;
        used[ii] = true;
        total += d;
        worst = worst.max(d);
    }
    (poly_to_item, total, worst)
}

#[test]
#[ignore = "실측 하네스 — 탐색 + 압축 예산만큼 돈다"]
fn our_search_seeded_with_the_competitor_layout() {
    let inst = load(&input_path());
    let c = DEFAULT_SWARM_CONFIG;
    let secs = env_u64("WS_SECS", 480);
    let cmpr_secs = env_u64("WS_CMPR_SECS", 120);
    let seed = env_u64("WS_SEED", 0);

    // ── 상용 배치 읽기 ──
    let raw = std::fs::read_to_string(competitor_path()).expect("competitor json");
    let doc: serde_json::Value = serde_json::from_str(&raw).expect("parse");
    let polys_raw: Vec<Vec<(f64, f64)>> = doc["polygons"]
        .as_array()
        .expect("polygons")
        .iter()
        .map(|p| p.as_array().unwrap().iter().map(|v| (v[0].as_f64().unwrap(), v[1].as_f64().unwrap())).collect())
        .collect();
    let height = inst.base_strip.fixed_height as f64;
    let (min_x, min_y) = polys_raw.iter().flatten().fold((f64::MAX, f64::MAX), |(a, b), p| (a.min(p.0), b.min(p.1)));
    let max_x = polys_raw.iter().flatten().fold(f64::MIN, |a, p| a.max(p.0));
    let len = max_x - min_x;
    // 원점을 (0,0) 으로 — 파일은 x 18.1 / y 21.05 오프셋을 갖는다.
    let polys_norm: Vec<Vec<(f64, f64)>> = polys_raw.iter().map(|p| p.iter().map(|&(x, y)| (x - min_x, y - min_y)).collect()).collect();
    let comp_area: f64 = polys_norm.iter().map(|p| signature(p).area).sum();
    println!("\n══════ 상용 배치 → 우리 탐색 (B 팔) ══════");
    println!("  상용    조각 {} · 길이 {len:.2} · 밀도(우리 정의) {:.2}%", polys_norm.len(), comp_area / (len * height) * 100.0);
    println!("  기하    {}", geometry_label());

    // ── 조각 대응 ──
    let item_sigs: Vec<Sig> = inst
        .items
        .iter()
        .map(|(it, _)| signature(&it.shape_orig().shape.vertices().iter().map(|p| (p.0 as f64, p.1 as f64)).collect::<Vec<_>>()))
        .collect();
    let mut best: Option<(Mirror, Vec<usize>, f64, f64)> = None;
    for m in [Mirror::None, Mirror::FlipY, Mirror::FlipX, Mirror::Both] {
        let sigs: Vec<Sig> = polys_norm.iter().map(|p| signature(&apply(m, p, len, height))).collect();
        let (map, total, worst) = assign(&item_sigs, &sigs);
        println!("  가설 {m:?}: 총 불일치 {total:9.3}  최악 {worst:.3}");
        if best.as_ref().map_or(true, |b| total < b.2) {
            best = Some((m, map, total, worst));
        }
    }
    let (mirror, map, total, worst) = best.unwrap();
    println!("  채택   {mirror:?} (총 {total:.3}, 최악 {worst:.3}) — 최악이 1.0 을 넘으면 대응을 의심하라");
    let polys: Vec<Vec<(f64, f64)>> = polys_norm.iter().map(|p| apply(mirror, p, len, height)).collect();

    // ── 배치: 무게중심 정렬로 평행이동을 구한다 (회전 0) ──
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.change_strip_width(len as f32 * 1.02).expect("strip width must be positive");
    for (pi, poly) in polys.iter().enumerate() {
        let item_id = map[pi];
        let item = inst.item(item_id);
        let s_p = signature(poly);
        let s_i = &item_sigs[item_id];
        let ext = DTransformation::new(0.0, ((s_p.cx - s_i.cx) as f32, (s_p.cy - s_i.cy) as f32));
        let d_transf = ext_to_int_transformation(&ext, &item.shape_orig().pre_transform);
        prob.place_item(SPPlacement { item_idx: item_id, d_transf });
    }
    prob.fit_strip().expect("strip width must be positive");
    let placed = prob.n_placed_items();
    let seed_sol = prob.save();
    let sep_cfg = c.expl_cfg.separator_config;
    let mut sep = Separator::new(inst.clone(), prob, Xoshiro256PlusPlus::seed_from_u64(seed + 1), sep_cfg);
    let entry_loss = sep.ct.get_total_loss();
    println!(
        "  우리 CDE 로 본 상용 배치   배치 {placed}/{}  폭 {:.2}  밀도 {:.2}%  진입 손실 {entry_loss:.1}",
        inst.total_item_qty(),
        seed_sol.strip_width(),
        seed_sol.density() * 100.0
    );
    if entry_loss > 0.0 {
        println!("  (손실 > 0 = 우리 보수적 충돌 기하(x1.00098 부풀림)에서는 겹침으로 보인다. 탐색의 첫 일이 이 해소다)");
    }

    // ── 프로브 모드: 탐색 대신 분리 단독으로 "이 폭에서 풀리는가" 만 잰다 ──
    //
    // 탐색이 초기 폭에서 한 걸음도 못 나갈 때 두 해석이 갈린다 — 초기 분리가 풀렸는데
    // 좁히기가 전부 실패한 것인지, 초기 분리조차 안 풀린 것인지. 폭을 조금씩 넓혀 가며
    // 잔여 손실을 찍으면 우리 CDE 기준 feasible 경계가 어디인지 바로 보인다.
    if let Ok(v) = std::env::var("WS_PROBE_SECS") {
        let probe = Duration::from_secs(v.trim().parse().unwrap_or(120));
        let w0 = seed_sol.strip_width();
        let widths: Vec<f32> = std::env::var("WS_PROBE_WIDTHS")
            .unwrap_or_else(|_| "1.0,1.002,1.005,1.01".into())
            .split(',')
            .filter_map(|x| x.trim().parse().ok())
            .collect();
        for f in widths {
            let mut sep = Separator::new(inst.clone(), { let mut p = SPProblem::new(inst.clone()).expect("strip forms a valid container"); p.restore(&seed_sol); p }, Xoshiro256PlusPlus::seed_from_u64(seed + 1), sep_cfg);
            sep.change_strip_width(w0 * f, None);
            let entry = sep.ct.get_total_loss();
            let t = Instant::now();
            let mut term = BasicTerminator::new();
            term.new_timeout(probe);
            let (_, snap) = sep.separate(&term, &mut None);
            let res = snap.get_total_loss();
            println!(
                "  분리 단독  폭 x{f:.3} = {:8.2} ({:.2}%)  진입 {entry:9.1} → 잔여 {res:9.1} ({:.1}% 해소)  {:.0}s{}",
                w0 * f,
                inst.items.iter().map(|(it, q)| it.shape_orig().area() * *q as f32).sum::<f32>() / (w0 * f * height as f32) * 100.0,
                if entry > 0.0 { (1.0 - res / entry) * 100.0 } else { 100.0 },
                t.elapsed().as_secs_f32(),
                if res == 0.0 { "   ◀ feasible" } else { "" }
            );
        }
        return;
    }

    // ── 우리 탐색 + 압축 ──
    let mut expl = c.expl_cfg;
    expl.time_limit = Duration::from_secs(secs);
    let t = Instant::now();
    let mut term = BasicTerminator::new();
    term.new_timeout(expl.time_limit);
    let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(&inst, &mut sep, &mut None, &term, &expl);
    let first = sols.first().map(|s| s.strip_width());
    let expl_sol = sols.last().cloned().unwrap_or_else(|| seed_sol.clone());
    println!(
        "  탐색 {secs}s   첫 feasible 폭 {}  최종 {:.2} ({:.2}%)  {}개 개선  {:.0}s",
        first.map_or("없음".to_string(), |w| format!("{w:.2}")),
        expl_sol.strip_width(),
        expl_sol.density() * 100.0,
        sols.len(),
        t.elapsed().as_secs_f32()
    );
    let mut cmpr = c.cmpr_cfg;
    cmpr.time_limit = Duration::from_secs(cmpr_secs);
    sep.change_strip_width(expl_sol.strip_width(), None);
    sep.rollback(&expl_sol, None);
    let mut term = BasicTerminator::new();
    term.new_timeout(cmpr.time_limit);
    let fin = swarm_core::optimizer::compress::compression_phase(&inst, &mut sep, &expl_sol, &mut None, &term, &cmpr);
    println!("  압축 {cmpr_secs}s   최종 폭 {:.2}  밀도 {:.2}%", fin.strip_width(), fin.density() * 100.0);

    println!("\n  읽는 법 ─ 상용 82.28% / 우리 LBF 출발 같은 예산 seed 0: 78.14%");
    println!("   · 최종 < 상용   → 우리 탐색은 그 품질의 해를 유지하지 못한다");
    println!("   · 최종 ≈ 상용   → 82.3% 근방이 이 분리기의 고정점이다");
    println!("   · 최종 > 상용   → 격차는 탐색이 아니라 출발 구조다\n");
}
