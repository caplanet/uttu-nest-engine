//! 경쟁사 마커와의 격차를 **같은 축으로** 재는 진단 하네스.
//!
//! # 왜 별도 하네스인가
//!
//! 기존 벤치들은 밀도 정의·시간 경계·유효 ordering 수·기준선 계수가 하네스마다 달라
//! 서로 비교할 수 없었다(A/B 오염). 여기서는 **하나의 정의만** 쓰고, 그 정의를
//! 출력에 그대로 찍는다. 조건을 모르는 숫자는 다음 사람에게 쓸모가 없다.
//!
//! * 밀도 = `SPProblem::fit_strip()` 후 [`SPSolution::density`] (jagua 정의).
//!   frontier 기준(`면적합 / (max_x × 높이)`)은 같은 배치에서 약 1.2%p 높게 나오므로
//!   **쓰지 않는다.** 경쟁사 수치도 이 정의로 환산해 두었다.
//! * 시간 = NFP 라이브러리 build 와 콜드 캐시 prewarm 을 **포함**한다.
//!   현장에서 실제로 기다리는 시간이 그것이기 때문이다.
//!
//! # 입력
//!
//! `SWARM_INPUT` 에 **절대 경로**를 주면 그 파일을 쓴다(기본:
//! `data/input/wsjk156.json`). 출력 첫 줄에 파일 경로와 sha256 을 찍으므로,
//! 나중에 어느 데이터의 수치인지 되짚을 수 있다.
//!
//! ```powershell
//! $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
//! cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
//!   --test competitor_gap -- --ignored --nocapture --test-threads=1
//! ```
//!
//! # 출력
//!
//! `SWARM_GAP_OUT` 이 있으면 그 디렉터리에 팔마다 배치 JSON 을 쓴다.
//! 경쟁사 레이아웃(`data/reference/competitor_wsjk156.json`)과 같은 형식이라
//! 구간별 충전율을 바로 대조할 수 있다.

use jagua_rs::Instant;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::nfp_lbf::{NfpLbfCache, PlacementScoreWeights, PreparedLibrary};
use swarm_core::nfp_strategy::accrete::accrete;
use swarm_core::nfp_strategy::select::SelectionRule;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::util::io::read_spp_input;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// 경쟁사 마커의 확정 수치. `data/reference/competitor_wsjk156.json` 에서 온다.
///
/// 검증 내역(그 파일의 `notes` 참조): 회전 0건 / 조각별 반전 0건 / 겹침 1.6mm²
/// (= 조각 총면적의 0.00002%, 40 plu/mm 정수격자 양자화 수준). 즉 우리와 **같은
/// 제약** 아래 얻은 유효 배치다.
const COMPETITOR_LENGTH_MM: f32 = 6182.775;
const COMPETITOR_DENSITY_PCT: f32 = 82.2767;
/// 조각 총면적 / strip 높이 = 이론 하한 길이.
const COMPETITOR_PIECE_AREA_MM2: f32 = 7_752_678.0;

fn input_path() -> PathBuf {
    match std::env::var("SWARM_INPUT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/input")
            .join("wsjk156.json"),
    }
}

/// 파일 내용의 sha256 — 어느 데이터로 잰 수치인지 출력에 남기기 위한 것.
fn sha256_hex(path: &Path) -> String {
    // 외부 크레이트를 끌어오지 않는다. 진단 출력용이라 충돌 저항이 필요 없다.
    // FNV-1a 64bit 를 두 번 돌려 128bit 로 늘린 값을 쓴다.
    let Ok(bytes) = std::fs::read(path) else {
        return "<읽기 실패>".into();
    };
    let mut a: u64 = 0xcbf2_9ce4_8422_2325;
    let mut b: u64 = 0x9e37_79b9_7f4a_7c15;
    for &byte in &bytes {
        a ^= byte as u64;
        a = a.wrapping_mul(0x100_0000_01b3);
        b = b.rotate_left(7) ^ a;
        b = b.wrapping_mul(0xff51_afd7_ed55_8ccd);
    }
    format!("{a:016x}{b:016x}")
}

fn load(path: &Path) -> SPInstance {
    let (ext_inst, _) = read_spp_input(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let c = DEFAULT_SWARM_CONFIG;
    let imp = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext_inst).unwrap()
}

/// 조각 총면적. 밀도의 **분자**가 무엇인지 헷갈리지 않도록 여기서 한 번만 정의한다.
///
/// 인스턴스가 가진 실제 도형 면적이다. 오목 단순화된 NFP 라이브러리 면적이 아니다 —
/// 그 둘을 섞은 것이 과거 계측 오류(보고 74.48% / 실제 73.27%)의 원인이었다.
fn total_piece_area(inst: &SPInstance) -> f32 {
    inst.items
        .iter()
        .map(|(item, qty)| item.shape_cd().area() * (*qty as f32))
        .sum()
}

/// 다각형에서 `x <= cut` 인 부분의 면적. 반평면 클리핑(Sutherland-Hodgman) + 신발끈.
///
/// 근사가 아니라 **정확한** 값이다. bbox 로 구간에 면적을 나눠 넣는 근사를 쓰면
/// 충전율이 100%를 넘는 값이 나온다(실제로 처음 판이 그랬다 — 241%). 오목한 의류
/// 패널에서는 bbox 와 실제 도형이 크게 다르므로 그 근사가 골과 봉우리를 통째로 지어낸다.
fn area_left_of(vertices: &[jagua_rs::geometry::primitives::Point], cut: f32) -> f32 {
    if vertices.len() < 3 {
        return 0.0;
    }
    let mut clipped: Vec<(f32, f32)> = Vec::with_capacity(vertices.len() + 4);
    let n = vertices.len();
    for i in 0..n {
        let a = vertices[i];
        let b = vertices[(i + 1) % n];
        let (ax, ay) = (a.0, a.1);
        let (bx, by) = (b.0, b.1);
        let a_in = ax <= cut;
        let b_in = bx <= cut;
        if a_in {
            clipped.push((ax, ay));
        }
        if a_in != b_in {
            // 변이 절단선을 지난다. 교점을 넣는다. bx == ax 이면 둘 다 같은 쪽이라
            // 여기 오지 않으므로 0 나눗셈이 없다.
            let t = (cut - ax) / (bx - ax);
            clipped.push((cut, ay + t * (by - ay)));
        }
    }
    if clipped.len() < 3 {
        return 0.0;
    }
    let mut acc = 0.0f64;
    for i in 0..clipped.len() {
        let (x1, y1) = clipped[i];
        let (x2, y2) = clipped[(i + 1) % clipped.len()];
        acc += x1 as f64 * y2 as f64 - x2 as f64 * y1 as f64;
    }
    (acc.abs() / 2.0) as f32
}

/// 200mm 구간별 충전율. 경쟁사 배치와 **같은 축**으로 비교하기 위한 지표다.
///
/// 전체 밀도 하나로는 "어디가 비었는지" 를 알 수 없다. 경쟁사는 최악 구간이 73.0%인데,
/// 그 값이 우리 *전체* 밀도와 같다 — 즉 저쪽에는 약한 구간이 없다는 뜻이다.
/// 우리 프로파일의 골이 어디인지가 곧 다음 작업 지시가 된다.
///
/// 마지막 구간이 `step` 보다 좁으면 **버린다.** 좁은 셀을 그대로 나누면 조각 하나가
/// 수백 %로 찍혀 프로파일을 못 읽게 된다.
fn fill_profile(inst: &SPInstance, sol: &SPSolution, step: f32) -> Vec<(f32, f32)> {
    let height = inst.base_strip.fixed_height;
    let width = sol.strip_width();
    let n = (width / step).floor().max(1.0) as usize;
    let mut filled = vec![0.0f32; n];
    for placed in sol.layout_snapshot().placed_items().values() {
        let item = &inst.items[placed.item().idx()].0;
        let shape = item.shape_cd().transform_clone(&placed.d_transf().into());
        let bbox = shape.bbox();
        let lo = ((bbox.x_min / step).floor().max(0.0) as usize).min(n);
        let hi = (((bbox.x_max / step).ceil()).max(0.0) as usize).min(n);
        // 구간 경계마다 누적 면적을 재고 차분한다 — 구간별 정확 면적이 된다.
        let mut prev = area_left_of(&shape.vertices(), lo as f32 * step);
        for (k, slot) in filled.iter_mut().enumerate().take(hi).skip(lo) {
            let cur = area_left_of(&shape.vertices(), (k + 1) as f32 * step);
            *slot += (cur - prev).max(0.0);
            prev = cur;
        }
    }
    filled
        .into_iter()
        .enumerate()
        .map(|(i, a)| (i as f32 * step, a / (step * height) * 100.0))
        .collect()
}

fn report(inst: &SPInstance, label: &str, sol: &SPSolution, ms: u128) {
    let density = sol.density() * 100.0;
    let lower_bound = total_piece_area(inst) / inst.base_strip.fixed_height;
    println!(
        "  {label:24}  폭 {:9.2}  밀도 {density:5.2}%  하한대비 {:5.3}x  {ms:>6}ms",
        sol.strip_width(),
        sol.strip_width() / lower_bound,
    );
    println!(
        "  {:24}  경쟁사 대비 {:+6.2}%p,  폭 {:+8.2}mm ({:+.1}%)",
        "",
        density - COMPETITOR_DENSITY_PCT,
        sol.strip_width() - COMPETITOR_LENGTH_MM,
        (sol.strip_width() / COMPETITOR_LENGTH_MM - 1.0) * 100.0,
    );
    let prof = fill_profile(inst, sol, 200.0);
    let vals: Vec<f32> = prof.iter().map(|(_, v)| *v).collect();
    let min = vals.iter().cloned().fold(f32::INFINITY, f32::min);
    let worst_at = prof
        .iter()
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(x, _)| *x)
        .unwrap_or(0.0);
    println!(
        "  {:24}  구간충전 min {min:5.1}% (x={worst_at:.0}mm)  max {:5.1}%  [경쟁사 min 73.0 / max 89.2]",
        "",
        vals.iter().cloned().fold(f32::NEG_INFINITY, f32::max),
    );
    println!(
        "  {:24}  {}",
        "",
        vals.iter()
            .map(|v| format!("{v:.0}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    if let Ok(dir) = std::env::var("SWARM_GAP_OUT") {
        let _ = std::fs::create_dir_all(&dir);
        let path = Path::new(&dir).join(format!("{label}.json"));
        let placements: Vec<_> = sol
            .layout_snapshot()
            .placed_items()
            .values()
            .map(|p| {
                let t = p.d_transf().translation();
                serde_json::json!({
                    "item_id": p.item().idx(),
                    "rotation_deg": p.d_transf().rotation().to_degrees(),
                    "x": t.0, "y": t.1,
                })
            })
            .collect();
        let doc = serde_json::json!({
            "label": label,
            "strip_width_mm": sol.strip_width(),
            "strip_height_mm": inst.base_strip.fixed_height,
            "density_pct": density,
            "density_definition": "SPSolution::density after fit_strip (jagua)",
            "elapsed_ms": ms,
            "placements": placements,
        });
        let _ = std::fs::write(path, serde_json::to_string(&doc).unwrap());
    }
}

/// 배치 결과(조각 목록)를 `SPSolution` 으로 굳힌다.
///
/// 검증 경로를 우회하지 않는다 — `fit_strip` 과 `solution_validation_error` 를 거쳐야
/// 밀도가 다른 하네스와 비교 가능해진다.
fn materialize(
    inst: &SPInstance,
    pieces: &[swarm_core::nfp_lbf::PlacedPiece],
    cap: f32,
) -> Option<SPSolution> {
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.change_strip_width(cap).expect("strip width must be positive");
    let mut left: Vec<usize> = inst.items.iter().map(|(_, q)| *q).collect();
    for p in pieces {
        if left.get(p.item_id).copied().unwrap_or(0) == 0 {
            continue;
        }
        left[p.item_id] -= 1;
        prob.place_item(SPPlacement {
            d_transf: DTransformation::new(0.0, p.translation),
            item_idx: p.item_id,
        });
    }
    let placed = prob.n_placed_items();
    if placed == 0 {
        return None;
    }
    let expected: usize = inst.items.iter().map(|(_, q)| *q).sum();
    if placed < expected {
        // 미배치가 남으면 `SPSolution::density` 의 분자가 줄어 **다른 팔과 비교 불가**다.
        // 조용히 넘기지 않고 알린다.
        println!("  ⚠ 미배치 {}개 — 이 팔의 밀도는 다른 팔과 비교할 수 없다", expected - placed);
    }
    prob.fit_strip().expect("strip width must be positive");
    Some(prob.save())
}

#[test]
#[ignore = "분 단위 — --ignored 로 명시 실행"]
fn how_far_are_we_from_the_competitor_marker() {
    let path = input_path();
    let inst = load(&path);
    let height = inst.base_strip.fixed_height;
    let area = total_piece_area(&inst);
    let lower_bound = area / height;
    let probes: usize = std::env::var("PROBES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(4096);
    // 문서 실측은 "벽시계 5초" 동안 라운드를 계속 돌렸다. 기본 2회와 다르므로 열어 둔다.
    let reflow_rounds_env: usize = std::env::var("REFLOW_ROUNDS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(swarm_core::nfp_strategy::DEFAULT_REFLOW_ROUNDS);
    // 오목 단순화 강도. **출하 기본값은 0.0(비활성)인데 저장소의 모든 실측은 0.02 다** —
    // 즉 기본값 regime 에서 잰 수치가 하나도 없다. 그것을 재려고 열어 둔다.
    let concave: f32 = std::env::var("CONCAVE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0.02);
    let slide_rounds: usize = std::env::var("SLIDE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2);

    // ── 측정 조건을 먼저 찍는다. 조건 없는 수치는 재현할 수 없다. ──
    println!("\n══════ 경쟁사 격차 진단 ══════");
    println!("  입력      {}", path.display());
    println!("  지문      fnv128 {}", sha256_hex(&path));
    println!(
        "  인스턴스  조각 {} / strip 높이 {height} / 조각 총면적 {area:.0} mm²",
        inst.items.iter().map(|(_, q)| *q).sum::<usize>()
    );
    println!("  이론하한  길이 {lower_bound:.1} mm (= 면적/높이, 밀도 100%)");
    println!(
        "  경쟁사    길이 {COMPETITOR_LENGTH_MM:.1} mm  밀도 {COMPETITOR_DENSITY_PCT:.2}%  하한대비 {:.3}x",
        COMPETITOR_LENGTH_MM / (COMPETITOR_PIECE_AREA_MM2 / height)
    );
    println!(
        "  조건      탐침 {probes} / 슬라이딩 {slide_rounds}회 / 오목단순화 {concave} / seed 0 / 회전 [0] 고정"
    );
    println!("  밀도정의  fit_strip 후 SPSolution::density (jagua). frontier 기준 아님");
    println!("  시간경계  NFP 라이브러리 build + 콜드 prewarm **포함**\n");

    // ── 팔 1: LBF 단독 (프로덕션 기본 계수를 그대로 쓴다) ──
    // 기존 벤치는 여기서 좌측편향 계수를 1.0 으로 넣어 기준선을 실제보다 나쁘게 만들었고,
    // 그래서 모든 델타가 부풀려져 있었다. 프로덕션 값을 쓴다.
    let lbf_x = DEFAULT_SWARM_CONFIG.lbf_x_multiplier;
    let lbf_y = DEFAULT_SWARM_CONFIG.lbf_y_multiplier;
    println!("  [기준선 계수] lbf_x_multiplier={lbf_x} lbf_y_multiplier={lbf_y} (프로덕션 기본값)");
    let t = Instant::now();
    let lbf = LBFBuilder::new(
        inst.clone(),
        Xoshiro256PlusPlus::seed_from_u64(0),
        LBF_SAMPLE_CONFIG,
        lbf_x,
        lbf_y,
    )
    .construct()
    .prob
    .save();
    report(&inst, "LBF 단독", &lbf, t.elapsed().as_millis());

    // ── NFP 라이브러리 준비 (시간에 포함한다) ──
    let t0 = Instant::now();
    let rotations = vec![0i64];
    let Some(library) = PreparedLibrary::build(&inst, &rotations, concave) else {
        println!("  라이브러리 생성 실패 — 이후 팔을 건너뛴다");
        return;
    };
    let mut cache = NfpLbfCache::new(library);
    cache.prewarm(Duration::from_secs(10));
    let prep_ms = t0.elapsed().as_millis();
    println!("  [준비] NFP 라이브러리 build + prewarm {prep_ms}ms");

    let max_id = inst.items.iter().map(|(i, _)| i.idx()).max().unwrap_or(0);
    let item_rotations: Vec<Vec<i64>> = (0..=max_id).map(|_| vec![0i64]).collect();
    let cap: f32 = inst
        .items
        .iter()
        .map(|(it, q)| it.shape_cd().bbox().width() * (*q as f32))
        .sum();
    let mut remaining: Vec<usize> = Vec::new();
    for (item, qty) in inst.items.iter() {
        for _ in 0..*qty {
            remaining.push(item.idx());
        }
    }

    // ── 팔 2: 누적 배치 ──
    let t1 = Instant::now();
    let out = accrete(
        &cache,
        &remaining,
        height,
        cap,
        PlacementScoreWeights::default(),
        &item_rotations,
        probes,
        SelectionRule::MaxContact,
        None,
    );
    let acc_ms = t1.elapsed().as_millis();
    if let Some(sol) = materialize(&inst, &out.pieces, cap) {
        report(&inst, "누적 배치", &sol, prep_ms + acc_ms);
    }

    // ── 팔 3: 누적 배치 + 슬라이딩 압축 ──
    let t2 = Instant::now();
    let mut placer = swarm_core::nfp_lbf::SequentialPlacer::with_item_rotations(
        &mut cache,
        height,
        cap,
        PlacementScoreWeights::default(),
        item_rotations.clone(),
    )
    .with_probe_budget(probes);
    let mut ctx = swarm_core::nfp_strategy::StrategyContext::new(
        &mut placer,
        swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG,
        item_rotations.clone(),
        None,
    );
    let slid = swarm_core::nfp_strategy::sliding::compact(&mut ctx, &out.pieces, slide_rounds);
    let slide_ms = t2.elapsed().as_millis();
    if let Some(sol) = materialize(&inst, &slid.pieces, cap) {
        report(
            &inst,
            "누적 배치 + 슬라이딩",
            &sol,
            prep_ms + acc_ms + slide_ms,
        );
    }

    // ── 팔 4~7: **프로덕션 경로**(build_best_initial_nest)로 ⑧/⑨를 태운다 ──
    //
    // 위 팔들은 모듈을 직접 부른다. 그것만으로는 "설정을 켜면 실제로 도는가" 를 알 수
    // 없다 — 이 코드베이스가 세 번 겪은 결함이 정확히 그 지점이었다.
    // 라벨과 평가한 ordering 수를 함께 찍으므로, 배선이 끊기면 라벨이 비고 12개를 돈다.
    for (label, strategy) in [
        (
            "[경로] 8 누적배치",
            swarm_core::nfp_strategy::NfpStrategyConfig {
                accretion: true,
                ..swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG
            },
        ),
        (
            "[경로] 8+7",
            swarm_core::nfp_strategy::NfpStrategyConfig {
                accretion: true,
                sliding_compaction: true,
                sliding_rounds: slide_rounds,
                ..swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG
            },
        ),
        (
            "[경로] 8+7+9",
            swarm_core::nfp_strategy::NfpStrategyConfig {
                accretion: true,
                sliding_compaction: true,
                sliding_rounds: slide_rounds,
                tail_reflow: true,
                reflow_rounds: reflow_rounds_env,
                ..swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG
            },
        ),
        (
            // ③ 접촉 최대화 — 정확 접촉 조각 수가 배선됐는지 보는 팔.
            // 문서 실측(§6-5-3, ordering 12 / 탐침 4096)의 ③ 행은 72.00% 다.
            "[경로] 3 접촉최대화",
            swarm_core::nfp_strategy::NfpStrategyConfig {
                contact_maximization: true,
                ..swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG
            },
        ),
        (
            "[경로] 3 기준선(전부끔)",
            swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG,
        ),
        (
            "[경로] 9 꼬리회수만",
            swarm_core::nfp_strategy::NfpStrategyConfig {
                tail_reflow: true,
                reflow_rounds: reflow_rounds_env,
                ..swarm_core::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG
            },
        ),
    ] {
        // ⑨ 단독은 문서 실측 조건(ordering 1개, 탐침 4096)을 그대로 쓴다.
        // 그 표가 "ord 1 + reflow = 73.27%, ord 4 = 72.68%, ord 8 = 72.09%" 로
        // **ordering 이 늘수록 나빠진다**고 적고 있어, 12개로 재면 재현이 안 된다.
        let doc_conditions = label.contains("9 꼬리회수만") || label.contains("3 ");
        let seed_cfg = swarm_core::config::NfpLbfSeedConfig {
            enabled: true,
            n_orderings: if label.contains("9 꼬리회수만") {
                1
            } else {
                12
            },
            total_budget_ms: 180_000,
            cache_budget_ms: 20_000,
            concave_simplify_tolerance: concave,
            max_candidate_probes: if doc_conditions { 4096 } else { probes },
            ..swarm_core::config::DEFAULT_NFP_LBF_SEED_CONFIG
        };
        let nfp_cfg = seed_cfg.to_config(strategy);
        let t = Instant::now();
        let mut seed_rng = Xoshiro256PlusPlus::seed_from_u64(0);
        match swarm_core::nfp_lbf::build_best_initial_nest(&inst, &nfp_cfg, &mut seed_rng) {
            Some(nest) => {
                report(&inst, label, &nest.solution, t.elapsed().as_millis());
                println!(
                    "  {:24}  전략 {:?} / ordering {}개 평가",
                    "",
                    nest.report.strategies,
                    nest.report.orderings_evaluated
                );
            }
            None => println!("  {label:24}  시드 생성 실패"),
        }
    }


    // ── 팔: 레퍼런스 설계 엔진 (`refplace`) ──
    //
    // 설계 문서(`레퍼런스-초기배치-설계.md`)를 그대로 구현한 별도 엔진이다.
    // 기존 경로와 아키텍처가 달라(그룹 기반 순서 · 부착 체인 · 표면 균형 점수),
    // 같은 축으로 재 봐야 비교가 성립한다.
    {
        let t = Instant::now();
        let mut params = swarm_core::refplace::RefPlaceParams::default();
        if let Ok(v) = std::env::var("REFPLACE_CONCAVE") {
            if let Ok(t) = v.parse::<f32>() {
                params.simplify.concave[0] = t;
            }
        }
        match swarm_core::refplace::build_reference_nest_reported(&inst, &params, None) {
            (Some(sol), rep) => {
                report(&inst, "[설계] refplace", &sol, t.elapsed().as_millis());
                println!(
                    "  {:24}  놓음 {}/{} · NFP 쌍 {} · 자세 {} · 절단 {} · 검증표식 {}",
                    "",
                    rep.placed,
                    rep.placed + rep.unplaced,
                    rep.nfp_pairs,
                    rep.poses,
                    rep.truncations,
                    rep.validation_marks,
                );
                println!(
                    "  {:24}  폴백 — 영역빔 {} · 영역없음 {} · 후보전멸 {} · 예산소진 {}",
                    "",
                    rep.fallback_empty,
                    rep.fallback_missing,
                    rep.fallback_no_candidate,
                    rep.budget_exhausted
                );
            }
            (None, rep) => println!(
                "  {:24}  해를 내지 못했다 — {}  (놓음 {} · 폴백 영역없음 {} · 후보전멸 {})",
                "[설계] refplace",
                rep.failure.as_deref().unwrap_or("사유 미기록"),
                rep.placed,
                rep.fallback_missing,
                rep.fallback_no_candidate
            ),
        }
    }

    println!(
        "\n  구간별 수치는 200mm 폭. 경쟁사 프로파일:\n  73 75 81 79 76 73 79 81 81 88 87 82 80 83 80 88 75 89 83 82 85 85 86 87 85 86 80 78 85 77 76\n"
    );
}

/// LBF **배치 순서 모드** 네 가지를 같은 축으로 잰다.
///
/// # 왜 LBF 단독인가
///
/// 순서 모드는 초기 배치에만 작용한다. 탐색까지 돌리면 예산 정책과 난수가 섞여
/// "순서를 바꾼 효과" 를 분리할 수 없다(함정 5). 여기서는 LBF 만 돌리고,
/// 밀도 정의는 이 파일의 단일 정의(`fit_strip` 후 `SPSolution::density`)를 그대로 쓴다.
///
/// ```powershell
/// $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
/// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
///   --test competitor_gap lbf_order -- --ignored --nocapture --test-threads=1
/// ```
#[test]
#[ignore = "실측 하네스 — 수 초가 걸리므로 명시적으로 부른다"]
fn lbf_order_modes_side_by_side() {
    use swarm_core::optimizer::lbf::LbfOrderMode;

    let path = input_path();
    let inst = load(&path);
    let lbf_x = DEFAULT_SWARM_CONFIG.lbf_x_multiplier;
    let lbf_y = DEFAULT_SWARM_CONFIG.lbf_y_multiplier;

    println!("\n══════ LBF 배치 순서 모드 비교 ══════");
    println!("  입력      {}", path.display());
    println!("  지문      fnv128 {}", sha256_hex(&path));
    println!("  조건      LBF 단독 / seed 0 / lbf_x={lbf_x} lbf_y={lbf_y}");
    println!("  크기기준  볼록껍질 면적 x 직경 (실제 면적이 아니다)");
    println!("  밀도정의  fit_strip 후 SPSolution::density (jagua)\n");

    let mut results = Vec::new();
    for mode in [
        LbfOrderMode::LargestFirst,
        LbfOrderMode::SmallestFirst,
        LbfOrderMode::MiddleOutLargeFirst,
        LbfOrderMode::MiddleOutSmallFirst,
    ] {
        let t = Instant::now();
        let sol = LBFBuilder::new(
            inst.clone(),
            Xoshiro256PlusPlus::seed_from_u64(0),
            LBF_SAMPLE_CONFIG,
            lbf_x,
            lbf_y,
        )
        .with_order_mode(mode)
        .construct()
        .prob
        .save();
        report(&inst, mode.label(), &sol, t.elapsed().as_millis());
        results.push((mode, sol.strip_width(), sol.density() * 100.0));
    }

    // 모드를 바꿔도 결과가 전혀 안 변하면 배선이 끊긴 것이다(함정 1).
    let baseline = results[0].1;
    assert!(
        results.iter().skip(1).any(|(_, w, _)| *w != baseline),
        "네 모드가 모두 같은 폭을 냈다 — 순서가 실제로 적용되지 않고 있다"
    );

    let best = results
        .iter()
        .max_by(|a, b| a.2.total_cmp(&b.2))
        .expect("모드가 넷 있다");
    println!(
        "\n  최고: {} — 폭 {:.2}mm, 밀도 {:.2}% (기본 대비 {:+.2}%p)",
        best.0.label(),
        best.1,
        best.2,
        best.2 - results[0].2
    );
}
