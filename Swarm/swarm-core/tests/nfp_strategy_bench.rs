//! NFP 초기 배치 전략 7종 벤치 — 여성WSjk 156-패널이 기준 데이터다.
//!
//! `data/input/wsjk156.json` 은 사용자가 지정한
//! `여성WSjk_비교용-G_New_Swarm_신규2.json` 과 **바이트 단위로 동일**하다(sha256 확인).
//!
//! 기본으로는 `#[ignore]` 다. 한 번 도는 데 분 단위가 걸려 CI 에 넣을 수 없다.
//!
//! ```text
//! cargo test -p swarm-core --release \
//!     --features "simd,capmods,adaptive,structure-first" \
//!     --test nfp_strategy_bench -- --ignored --nocapture --test-threads=1
//! ```
//!
//! 환경변수
//!
//! ```text
//! BENCH_INSTANCES   쉼표 구분 인스턴스 파일명 (기본: wsjk156.json)
//! BENCH_EXPLORE_S   시드 이후 탐색을 몇 초 돌릴지 (기본 0 = 초기 배치만 본다)
//! BENCH_PROBES      탐침 예산 (기본 4096)
//! BENCH_ORDERINGS   ordering 수 (기본 50)
//! BENCH_ARMS        쉼표 구분 부분문자열. 라벨이 하나라도 포함하는 팔만 돈다
//!                   (기본: 전부). 예: BENCH_ARMS='②' 는 ②가 들어간 팔만.
//! ```
#![cfg(feature = "adaptive")]

use jagua_rs::probs::spp::entities::{SPInstance, SPProblem, SPSolution};
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::nfp_lbf::{NfpLbfConfig, build_best_initial_nest};
use swarm_core::nfp_strategy::pipeline::search_pairs::{self, SearchPairs};
use swarm_core::nfp_strategy::{DEFAULT_NFP_STRATEGY_CONFIG, NfpStrategyConfig};
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use swarm_core::util::terminator::{BasicTerminator, Terminator};
use std::path::Path;
use std::time::{Duration, Instant};

fn env_usize(key: &str, default: usize) -> usize {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn load(name: &str) -> SPInstance {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../data/input")
        .join(name);
    let (ext_inst, _) = read_spp_input(&path).unwrap_or_else(|e| panic!("{name}: {e}"));
    let c = DEFAULT_SWARM_CONFIG;
    let imp = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&imp, &ext_inst)
        .unwrap_or_else(|e| panic!("{name}: {e:?}"))
}

/// 기준선 — 기존 LBF 생성기.
///
/// # 좌측편향 계수는 **프로덕션 값**을 쓴다
///
/// 예전에는 `(1.0, 1.0)` 을 넘겼다. 프로덕션은 `(10.0, 1.0)` 이다
/// (`DEFAULT_SWARM_CONFIG::lbf_x_multiplier`). 그러면 기준선이 실제보다 **나쁘게**
/// 나오고, 모든 전략의 `(+x.xx%p)` 델타가 그만큼 부풀려진다.
///
/// 실측 차이(wsjk156, `SPSolution::density`): `(1.0, 1.0)` 69.45% vs
/// **`(10.0, 1.0)` 70.61%** — 약 1.16%p 다. 이 저장소의 문서에 적힌 "LBF 대비 +2.30%p"
/// 같은 수치는 그만큼 과대평가돼 있다.
///
/// A/B 에서 기준선은 **실제로 배포되는 것**이어야 한다. 그러지 않으면 재는 것이
/// "전략이 얼마나 좋은가" 가 아니라 "기준선을 얼마나 나쁘게 만들었나" 가 된다.
fn lbf_baseline(inst: &SPInstance) -> (f32, f32, u128) {
    let t = Instant::now();
    let rng = Xoshiro256PlusPlus::seed_from_u64(0);
    let sol = LBFBuilder::new(
        inst.clone(),
        rng,
        LBF_SAMPLE_CONFIG,
        DEFAULT_SWARM_CONFIG.lbf_x_multiplier,
        DEFAULT_SWARM_CONFIG.lbf_y_multiplier,
    )
        .construct()
        .prob
        .save();
    (
        sol.strip_width(),
        sol.density() * 100.0,
        t.elapsed().as_millis(),
    )
}

fn seed_config(strategy: NfpStrategyConfig) -> NfpLbfConfig {
    // 유효 ordering 수를 팔마다 **같게** 맞춘다.
    //
    // 프로덕션에서는 `scale_orderings` 가 전략 비용만큼 ordering 수를 줄여 전체 예산을
    // 지킨다. 그런데 A/B 에 그걸 그대로 적용하면 "전략이 나쁜 것"과 "ordering 을 덜
    // 돈 것"이 섞여 버린다 - 실제로 첫 측정에서 팔마다 12/8/6/3/1 개로 갈렸다.
    //
    // 그래서 여기서는 비용 배수를 미리 곱해 **평가되는 ordering 수가 같아지게** 한다.
    // 시간은 당연히 전략마다 다르고, 그 시간이 곧 그 전략의 비용이다.
    let target = env_usize("BENCH_ORDERINGS", 50);
    let factor = strategy.estimated_cost_factor().max(1.0);
    let requested = ((target as f32) * factor).ceil() as usize;

    NfpLbfConfig {
        n_orderings: requested,
        cache_budget: Duration::from_secs(10),
        // 전략 경로는 훨씬 비싸다. 예산 고갈이 결과를 흐리지 않도록 넉넉히 준다.
        total_budget: Duration::from_secs(600),
        concave_simplify_tolerance: 0.02,
        max_candidate_probes: env_usize("BENCH_PROBES", 4096),
        strategy,
        ..NfpLbfConfig::default()
    }
}

/// 시드에서 탐색을 `secs` 초 돌린 뒤의 밀도.
fn explore_from(inst: &SPInstance, seed: &SPSolution, secs: u64) -> (f32, f32) {
    let mut prob = SPProblem::new(inst.clone()).expect("strip forms a valid container");
    prob.restore(seed);
    let expl = DEFAULT_SWARM_CONFIG.expl_cfg;
    let mut sep = Separator::new(
        inst.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(0),
        expl.separator_config,
    );
    let mut term = BasicTerminator::new();
    term.new_timeout(Duration::from_secs(secs));
    let sols = swarm_core::optimizer::explore_mods::exploration_phase_adaptive(
        inst, &mut sep, &mut None, &term, &expl,
    );
    match sols.last() {
        Some(s) => (s.strip_width(), s.density() * 100.0),
        None => (seed.strip_width(), seed.density() * 100.0),
    }
}

/// 벤치 팔 하나 — 라벨, 켤 전략, 그리고 ⑤/⑥ 이 ② 쌍을 다루는 방식.
///
/// `pairs` 는 설정(`NfpStrategyConfig`)이 아니라 계측 노브라 여기 따로 붙는다.
/// 기본은 `Off` — 왜 그게 기본인지는 `nfp_strategy::pipeline::search_pairs` 문서의
/// 실측표에 있다.
struct Arm {
    label: &'static str,
    strategy: NfpStrategyConfig,
    pairs: SearchPairs,
}

fn arm(label: &'static str, strategy: NfpStrategyConfig) -> Arm {
    Arm {
        label,
        strategy,
        pairs: SearchPairs::Off,
    }
}

/// 켤 전략 하나(혹은 조합)와 그 라벨.
fn arms() -> Vec<Arm> {
    let off = DEFAULT_NFP_STRATEGY_CONFIG;
    let raw: Vec<(&'static str, NfpStrategyConfig)> = vec![
        ("기준 (전부 끔)", off),
        (
            "① 인스턴스 회전",
            NfpStrategyConfig {
                instance_rotations: true,
                ..off
            },
        ),
        (
            "② 쌍별 클러스터링",
            NfpStrategyConfig {
                pairwise_clustering: true,
                ..off
            },
        ),
        (
            "③ 접촉 최대화",
            NfpStrategyConfig {
                contact_maximization: true,
                ..off
            },
        ),
        (
            "④ 포켓 우선",
            NfpStrategyConfig {
                pocket_first: true,
                ..off
            },
        ),
        (
            "⑤ 빔 서치 (폭 2, 기본)",
            NfpStrategyConfig {
                beam_search: true,
                beam_width: 2,
                ..off
            },
        ),
        (
            "⑤ 빔 서치 (폭 4)",
            NfpStrategyConfig {
                beam_search: true,
                beam_width: 4,
                ..off
            },
        ),
        (
            "⑥ best-fit (12)",
            NfpStrategyConfig {
                best_fit_selection: true,
                best_fit_lookahead: 12,
                ..off
            },
        ),
        (
            "⑦ 슬라이딩 (2회)",
            NfpStrategyConfig {
                sliding_compaction: true,
                sliding_rounds: 2,
                ..off
            },
        ),
        // 조합 — 비용이 곱해지지 않는 것들만 먼저 본다.
        (
            "①+③+⑦",
            NfpStrategyConfig {
                instance_rotations: true,
                contact_maximization: true,
                sliding_compaction: true,
                ..off
            },
        ),
        (
            "①+②+③+④+⑦",
            NfpStrategyConfig {
                instance_rotations: true,
                pairwise_clustering: true,
                contact_maximization: true,
                pocket_first: true,
                sliding_compaction: true,
                ..off
            },
        ),
        (
            "③+⑦",
            NfpStrategyConfig {
                contact_maximization: true,
                sliding_compaction: true,
                ..off
            },
        ),
        // ②를 뺀 대조군. "최고 조합" 의 공을 ②가 실제로 나눠 갖는지 보려면
        // 같은 조합에서 ②만 빠진 값이 있어야 한다.
        (
            "③+④+⑦ (② 없음)",
            NfpStrategyConfig {
                contact_maximization: true,
                pocket_first: true,
                sliding_compaction: true,
                ..off
            },
        ),
        (
            "②+③+④+⑦",
            NfpStrategyConfig {
                pairwise_clustering: true,
                contact_maximization: true,
                pocket_first: true,
                sliding_compaction: true,
                ..off
            },
        ),
        (
            "②+③+④+⑤+⑦",
            NfpStrategyConfig {
                pairwise_clustering: true,
                contact_maximization: true,
                pocket_first: true,
                beam_search: true,
                sliding_compaction: true,
                ..off
            },
        ),
        (
            "전부 켬",
            NfpStrategyConfig {
                instance_rotations: true,
                pairwise_clustering: true,
                contact_maximization: true,
                pocket_first: true,
                beam_search: true,
                best_fit_selection: true,
                sliding_compaction: true,
                ..off
            },
        ),
    ];

    let mut out: Vec<Arm> = raw.into_iter().map(|(l, s)| arm(l, s)).collect();

    // ── ②를 ⑤/⑥ 에 넘기는 팔 ────────────────────────────────────────────
    //
    // 같은 전략 조합을 노브만 바꿔 여러 번 돈다. 리포트의 "쌍" 이 0이면 배선이 안 된
    // 것이므로 `Off` 팔과 나란히 봐야 한다.
    let beam2 = NfpStrategyConfig {
        pairwise_clustering: true,
        beam_search: true,
        beam_width: 2,
        ..DEFAULT_NFP_STRATEGY_CONFIG
    };
    let bestfit = NfpStrategyConfig {
        pairwise_clustering: true,
        best_fit_selection: true,
        best_fit_lookahead: 12,
        ..DEFAULT_NFP_STRATEGY_CONFIG
    };
    let combo = NfpStrategyConfig {
        pairwise_clustering: true,
        contact_maximization: true,
        pocket_first: true,
        beam_search: true,
        sliding_compaction: true,
        ..DEFAULT_NFP_STRATEGY_CONFIG
    };
    for (label, strategy, pairs) in [
        ("②+⑤ 폭2 (쌍 안 넘김)", beam2, SearchPairs::Off),
        ("②+⑤ 폭2 (쌍 넘김)", beam2, SearchPairs::Replace),
        ("②+⑤ 폭2 (둘 다 살림)", beam2, SearchPairs::KeepBoth),
        ("②+⑥ 12 (쌍 안 넘김)", bestfit, SearchPairs::Off),
        ("②+⑥ 12 (쌍 넘김)", bestfit, SearchPairs::Replace),
        ("②+③+④+⑤+⑦ (쌍 넘김)", combo, SearchPairs::Replace),
        ("②+③+④+⑤+⑦ (둘 다)", combo, SearchPairs::KeepBoth),
    ] {
        out.push(Arm {
            label,
            strategy,
            pairs,
        });
    }
    out
}

#[test]
#[ignore = "분 단위로 걸린다 — --ignored 로 명시적으로 돌릴 것"]
fn strategy_ab_on_the_garment_marker() {
    let names: Vec<String> = std::env::var("BENCH_INSTANCES")
        .unwrap_or_else(|_| "wsjk156.json".into())
        .split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect();
    let explore_secs = env_usize("BENCH_EXPLORE_S", 0) as u64;
    let probes = env_usize("BENCH_PROBES", 4096);
    let orderings = env_usize("BENCH_ORDERINGS", 50);

    for name in names {
        let inst = load(&name);
        let n_items: usize = inst.items.iter().map(|(_, q)| *q).sum();
        let (lbf_w, lbf_d, lbf_ms) = lbf_baseline(&inst);

        println!("\n╔══ {name} — {n_items}조각, 탐침 {probes}, ordering {orderings} ══");
        println!("║ 기준선 LBF 단독            폭 {lbf_w:9.2}  밀도 {lbf_d:5.2}%  {lbf_ms:>6}ms");
        println!("╟─────────────────────────────────────────────────────────────────────");

        let mut rows: Vec<(String, f32, f32, u128, String)> = Vec::new();
        // 팔 하나가 분 단위이므로, 재는 팔만 고를 수 있어야 반복이 가능하다.
        let only: Vec<String> = std::env::var("BENCH_ARMS")
            .unwrap_or_default()
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        for Arm {
            label,
            strategy,
            pairs,
        } in arms()
        {
            if !only.is_empty() && !only.iter().any(|p| label.contains(p.as_str())) {
                continue;
            }
            // 노브는 전역이다. 팔마다 반드시 다시 세팅해야 앞 팔의 값이 새지 않는다.
            search_pairs::set(pairs);
            let cfg = seed_config(strategy);
            let mut rng = Xoshiro256PlusPlus::seed_from_u64(0);
            let t = Instant::now();
            match build_best_initial_nest(&inst, &cfg, &mut rng) {
                Some(nest) => {
                    let w = nest.solution.strip_width();
                    let d = nest.solution.density() * 100.0;
                    let ms = t.elapsed().as_millis();
                    let note = format!(
                        "ord {} / 쌍 {} / 이동 {}",
                        nest.report.orderings_evaluated,
                        nest.report.clusters_applied,
                        nest.report.pieces_slid,
                    );
                    println!(
                        "║ {label:<24} 폭 {w:9.2}  밀도 {d:5.2}%  {ms:>6}ms  ({:+.2}%p)  {note}",
                        d - lbf_d
                    );
                    rows.push((label.to_string(), w, d, ms, note));
                }
                None => println!("║ {label:<24} -> None (시드 생성 실패)"),
            }
        }

        // 탐색까지 이어서 — 시드 우위가 살아남는지.
        if explore_secs > 0 {
            println!("╟── 탐색 {explore_secs}초 후 ───────────────────────────────────────────");
            search_pairs::set(SearchPairs::Off);
            let mut rng = Xoshiro256PlusPlus::seed_from_u64(0);
            let base =
                build_best_initial_nest(&inst, &seed_config(DEFAULT_NFP_STRATEGY_CONFIG), &mut rng);
            if let Some(base) = base {
                let (bw, bd) = explore_from(&inst, &base.solution, explore_secs);
                println!("║ 기준 시드 -> 폭 {bw:9.2}  밀도 {bd:5.2}%");
            }
            // 초기 배치에서 가장 좋았던 팔 하나만 이어서 본다 (전부 돌리면 시간이 폭발한다).
            if let Some(best) = rows
                .iter()
                // **기준 팔을 먼저 제외하고** 최선을 고른다.
                //
                // 예전에는 `max_by(...).filter(...)` 였다. 그러면 기준 팔이 1등일 때
                // `filter` 가 `None` 을 만들어 **전략 팔 탐색이 통째로 생략된다** —
                // 차선 팔로 넘어가지 않는다. "전략이 탐색 후에도 이기는가" 라는 질문에
                // 답해야 할 구간이, 하필 기준선이 셀 때 사라지는 것이다.
                .filter(|r| r.0 != "기준 (전부 끔)")
                .max_by(|a, b| a.2.total_cmp(&b.2))
            {
                if let Some(a) = arms().into_iter().find(|a| a.label == best.0) {
                    search_pairs::set(a.pairs);
                    let mut rng = Xoshiro256PlusPlus::seed_from_u64(0);
                    if let Some(nest) =
                        build_best_initial_nest(&inst, &seed_config(a.strategy), &mut rng)
                    {
                        let (w, d) = explore_from(&inst, &nest.solution, explore_secs);
                        println!("║ {:<10} -> 폭 {w:9.2}  밀도 {d:5.2}%", best.0);
                    }
                }
            }
        }
        println!("╚══════════════════════════════════════════════════════════════════════");
    }
}
