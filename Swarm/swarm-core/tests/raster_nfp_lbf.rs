//! capmods 래스터 NFP(LBF 초기 배치) 검증.
//!
//! 실행:  cargo test --release -p swarm-core --features capmods --test raster_nfp_lbf
//!
//! 핵심 관심사는 두 가지다.
//!   1. **정확성** — 래스터는 근사이므로, 그 후보가 CDE 검증을 거쳐 실제로 겹침 없는
//!      배치만 만들어 내는지. (래스터 해상도와 무관하게 항상 성립해야 한다.)
//!   2. **효과** — 좁은 포켓을 난수로 놓치는 문제가 줄어 초기 폭이 실제로 좁아지는지.
#![cfg(feature = "capmods")]

use jagua_rs::probs::spp::entities::SPInstance;
use rand::SeedableRng;
use rand::rngs::Xoshiro256PlusPlus;
use swarm_core::config::{DEFAULT_RASTER_NFP_CONFIG, DEFAULT_SWARM_CONFIG, RasterNfpConfig};
use swarm_core::consts::LBF_SAMPLE_CONFIG;
use swarm_core::optimizer::lbf::LBFBuilder;
use swarm_core::optimizer::separator::Separator;
use swarm_core::util::io::read_spp_input;
use std::path::Path;

/// `data/input`의 인스턴스 이름, 또는 `.json`으로 끝나면 그대로 경로로 취급한다.
/// (환경변수 `SWARM_EXTRA_INSTANCE`로 저장소 밖 실물 마커를 끼워 넣을 때 쓴다.)
fn load(name: &str) -> SPInstance {
    let path = if name.ends_with(".json") {
        std::path::PathBuf::from(name)
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../data/input/{name}.json"))
    };
    let (ext_inst, _) =
        read_spp_input(&path).unwrap_or_else(|e| panic!("failed to read {}: {e}", path.display()));
    let cfg = DEFAULT_SWARM_CONFIG;
    let importer = swarm_core::util::io::new_importer(cfg.cde_config, cfg.poly_simpl_tolerance, cfg.min_item_separation, cfg.narrow_concavity_cutoff_ratio);
    jagua_rs::probs::spp::io::import_instance(&importer, &ext_inst)
        .expect("failed to import instance")
}

/// LBF로 초기해를 만들고 (폭, 배치된 아이템 수, 총 충돌 loss)를 돌려준다.
fn build_lbf(instance: &SPInstance, seed: u64, raster: RasterNfpConfig) -> (f32, usize, f32) {
    let builder = LBFBuilder::new(
        instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(seed),
        LBF_SAMPLE_CONFIG,
        10.0,
        1.0,
    )
    .with_raster_config(raster)
    .construct();

    let width = builder.prob.strip_width();
    let n_placed = builder.prob.layout().placed_items().len();

    // CDE 기반 충돌 트래커로 실제 겹침이 있는지 독립 검증한다.
    let sep = Separator::new(
        instance.clone(),
        builder.prob,
        Xoshiro256PlusPlus::seed_from_u64(seed + 1),
        DEFAULT_SWARM_CONFIG.expl_cfg.separator_config,
    );
    (width, n_placed, sep.ct.get_total_loss())
}

/// 래스터 후보로 만든 초기해는 항상 겹침이 없어야 한다 (해상도가 거칠어도).
#[test]
fn raster_lbf_produces_feasible_layouts() {
    for name in ["fu", "blaz1", "shirts"] {
        let instance = load(name);
        let n_items = instance.total_item_qty();

        // 일부러 거친 격자(셀이 아이템만큼 큼)까지 포함해 근사 오차를 키운 상태로 검증한다.
        for cell_ratio in [0.2, 0.35, 1.0] {
            let cfg = RasterNfpConfig {
                enabled: true,
                cell_ratio,
                ..DEFAULT_RASTER_NFP_CONFIG
            };
            let (width, n_placed, loss) = build_lbf(&instance, 42, cfg);

            assert_eq!(
                n_placed, n_items,
                "{name} (cell {cell_ratio}): all items must be placed"
            );
            assert_eq!(
                loss, 0.0,
                "{name} (cell {cell_ratio}): LBF layout must be collision-free, got loss {loss}"
            );
            assert!(
                width.is_finite() && width > 0.0,
                "{name}: invalid strip width {width}"
            );
        }
    }
}

/// 래스터 설정별 초기 폭을 원본(난수) 경로와 비교한다.
/// 확률적이라 단정하지 않고 표로 출력한다 — 채택 여부는 이 값과 벤치마크로 판단한다.
#[test]
fn raster_lbf_width_comparison() {
    const N_SEEDS: u64 = 5;
    let off = RasterNfpConfig {
        enabled: false,
        ..DEFAULT_RASTER_NFP_CONFIG
    };

    // (라벨, 셀 비율, 후보 수, 병행할 난수 샘플 수)
    let variants: [(&str, f32, usize, usize); 5] = [
        ("cell.12/064", 0.12, 64, 1000),
        ("cell.08/064", 0.08, 64, 1000),
        ("cell.08/128", 0.08, 128, 1000),
        ("cell.05/128", 0.05, 128, 1000),
        ("cell.12/128", 0.12, 128, 1000),
    ];

    let mut instances: Vec<String> = ["fu", "blaz1", "dagli", "shirts"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    if let Ok(extra) = std::env::var("SWARM_EXTRA_INSTANCE")
        && !extra.is_empty()
    {
        instances.push(extra);
    }

    println!(
        "{:10} {:>10}   {}",
        "instance",
        "random",
        variants
            .iter()
            .map(|v| format!("{:>18}", v.0))
            .collect::<String>()
    );
    for name in &instances {
        let instance = load(name);
        let name = Path::new(name)
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| name.clone());

        let mut w_off = 0.0;
        for seed in 0..N_SEEDS {
            let (w, _, loss) = build_lbf(&instance, seed, off);
            assert_eq!(loss, 0.0, "{name}: baseline LBF must be feasible");
            w_off += w;
        }
        w_off /= N_SEEDS as f32;

        let mut row = String::new();
        for &(_, cell_ratio, n_candidates, n_random_samples) in &variants {
            let cfg = RasterNfpConfig {
                enabled: true,
                cell_ratio,
                n_candidates,
                n_random_samples,
                ..DEFAULT_RASTER_NFP_CONFIG
            };
            let mut w_on = 0.0;
            let t0 = std::time::Instant::now();
            for seed in 0..N_SEEDS {
                let (w, _, loss) = build_lbf(&instance, seed, cfg);
                assert_eq!(loss, 0.0, "{name}: raster LBF must be feasible");
                w_on += w;
            }
            let ms = t0.elapsed().as_secs_f32() * 1000.0 / N_SEEDS as f32;
            w_on /= N_SEEDS as f32;
            row += &format!(
                "{:>18}",
                format!("{:+.1}% {:.0}ms", (w_on / w_off - 1.0) * 100.0, ms)
            );
        }
        println!("{name:10} {w_off:10.3}   {row}");
    }
}

/// **홀 우선 부착** 실측: 포켓 후보를 시드 순서 앞쪽에 넣으면 초기 배치가 좋아지는가.
///
/// # 무엇을 비교하나
///
/// 같은 인스턴스·같은 seed 로 `hole_first` 만 바꿔 초기 배치를 만들고 폭·밀도·시간을
/// 나란히 찍는다. 다른 노브는 전부 기본값이라, 차이는 후보 순서 하나에서만 온다.
///
/// `hole_share` 도 훑는다 — 포켓 후보에 앞자리를 얼마나 내줄지가 유일한 자유도다.
/// 전부 내주면(1.0) 포켓 후보가 CDE 에서 전멸했을 때 그 아이템이 시드를 통째로 날린다.
///
/// ```powershell
/// $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
/// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
///   --test raster_nfp_lbf hole_first -- --ignored --nocapture --test-threads=1
/// ```
#[test]
#[ignore = "실측 하네스 — 격자 전수 스캔이라 수십 초가 걸린다"]
fn hole_first_seed_ordering_measured() {
    use std::time::Instant;

    let path = std::env::var("SWARM_INPUT").ok().map(std::path::PathBuf::from);
    let (label, instance) = match &path {
        Some(p) => {
            let (ext, _) = read_spp_input(p).unwrap_or_else(|e| panic!("{}: {e}", p.display()));
            let c = DEFAULT_SWARM_CONFIG;
            let imp = swarm_core::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
            (
                p.display().to_string(),
                jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect("import"),
            )
        }
        None => ("shirts".to_string(), load("shirts")),
    };

    let n_items: usize = instance.total_item_qty();
    println!("\n══════ 홀 우선 부착 (초기 배치) ══════");
    println!("  입력    {label}");
    println!("  조각    {n_items}개 / seed 0 / lbf_x=10 lbf_y=1");
    println!("  포켓    strip 우측에 닿지 않는 4-연결 빈 영역 (폴리시와 같은 정의)\n");

    let mut rows = vec![];
    for (name, cfg) in [
        (
            "끔 (기존)".to_string(),
            RasterNfpConfig {
                hole_first: false,
                ..DEFAULT_RASTER_NFP_CONFIG
            },
        ),
        (
            "켬 share 0.25".to_string(),
            RasterNfpConfig {
                hole_first: true,
                hole_share: 0.25,
                ..DEFAULT_RASTER_NFP_CONFIG
            },
        ),
        (
            "켬 share 0.50".to_string(),
            RasterNfpConfig {
                hole_first: true,
                hole_share: 0.5,
                ..DEFAULT_RASTER_NFP_CONFIG
            },
        ),
        (
            "켬 share 1.00".to_string(),
            RasterNfpConfig {
                hole_first: true,
                hole_share: 1.0,
                ..DEFAULT_RASTER_NFP_CONFIG
            },
        ),
    ] {
        swarm_core::sample::raster_nfp::hole_stats::reset();
        let t = Instant::now();
        let (width, placed, loss) = build_lbf(&instance, 0, cfg);
        let ms = t.elapsed().as_millis();
        let (calls, found, emitted, accepted) = swarm_core::sample::raster_nfp::hole_stats::snapshot();
        let area: f32 = instance
            .items
            .iter()
            .map(|(it, q)| it.shape_cd().area() * *q as f32)
            .sum();
        let density = area / (width * instance.base_strip.fixed_height) * 100.0;
        println!(
            "  {name:16}  폭 {width:9.2}  밀도 {density:5.2}%  배치 {placed}/{n_items}  겹침손실 {loss:.4}  {ms:>6}ms"
        );
        // 켰는데 아무것도 안 바뀌었을 때, "포켓이 없었다" 와 "배선이 끊겼다" 를 가른다.
        println!(
            "  {:16}  홀 진단: 호출 {calls} / 포켓발견 {found} / 앞자리 {emitted} / **배치채택 {accepted}**",
            ""
        );
        assert_eq!(placed, n_items, "{name}: 모든 조각이 배치돼야 한다");
        assert_eq!(loss, 0.0, "{name}: 초기 배치에 겹침이 있으면 안 된다");
        rows.push((name, width, density));
    }

    let base = rows[0].1;
    println!();
    for (name, width, density) in rows.iter().skip(1) {
        println!(
            "  {name:16}  기존 대비 폭 {:+8.2}mm ({:+.2}%),  밀도 {:+.2}%p",
            width - base,
            (width / base - 1.0) * 100.0,
            density - rows[0].2
        );
    }
    println!();
}

/// **홀 우선 부착 배선 회귀.** 켜면 포켓에 실제로 조각이 들어가야 한다.
///
/// # 왜 카운터까지 보나
///
/// 이 기능은 "켰는데 결과가 안 바뀌는" 상태가 정상일 수도 있다 — 포켓이 안 생기는
/// 인스턴스가 있기 때문이다. 그래서 폭만 보면 배선이 끊겨도 알 수 없다.
/// 진단 카운터로 **포켓을 찾았고, 앞자리에 실었고, 실제 배치로 채택했다**를
/// 각각 확인한다.
///
/// 처음 구현은 후보 순서만 바꿨고, 이 테스트의 `배치 채택 > 0` 단정이 그것을
/// 잡아냈다 — 순서를 바꿔도 호출부가 손실 최소 하나만 고르므로 포켓은 언제나 졌다.
#[test]
fn hole_first_actually_places_items_into_pockets() {
    let instance = load("shirts");
    let stats = swarm_core::sample::raster_nfp::hole_stats::snapshot;

    // 꺼짐: 홀 경로가 한 번도 안 돌아야 한다.
    swarm_core::sample::raster_nfp::hole_stats::reset();
    let (width_off, placed_off, loss_off) = build_lbf(
        &instance,
        0,
        RasterNfpConfig {
            hole_first: false,
            ..DEFAULT_RASTER_NFP_CONFIG
        },
    );
    let (calls_off, found_off, _, accepted_off) = stats();
    assert_eq!(calls_off, 0, "꺼져 있는데 홀 경로가 돌았다");
    assert_eq!(found_off, 0);
    assert_eq!(accepted_off, 0);

    // 켜짐: 포켓을 찾고, 앞자리에 싣고, 실제로 배치해야 한다.
    swarm_core::sample::raster_nfp::hole_stats::reset();
    let (width_on, placed_on, loss_on) = build_lbf(
        &instance,
        0,
        RasterNfpConfig {
            hole_first: true,
            hole_share: 0.5,
            ..DEFAULT_RASTER_NFP_CONFIG
        },
    );
    let (calls_on, found_on, emitted_on, accepted_on) = stats();
    assert!(calls_on > 0, "홀 경로가 한 번도 안 돌았다 — 설정이 도달하지 않는다");
    assert!(found_on > 0, "포켓 후보를 하나도 못 찾았다");
    assert!(emitted_on > 0, "포켓 후보가 앞자리에 하나도 안 실렸다");
    assert!(
        accepted_on > 0,
        "포켓 후보가 실제 배치로 채택된 적이 없다 — 순서만 바꾸고 선택은 그대로면 이렇게 된다"
    );

    // 어느 쪽이든 유효한 배치여야 한다. 홀에 넣다가 겹치면 그게 진짜 결함이다.
    assert_eq!(placed_off, instance.total_item_qty());
    assert_eq!(placed_on, instance.total_item_qty());
    assert_eq!(loss_off, 0.0, "홀 우선 끔: 겹침이 있으면 안 된다");
    assert_eq!(loss_on, 0.0, "홀 우선 켬: 겹침이 있으면 안 된다");

    // 폭이 달라져야 한다 — 같으면 채택이 결과에 아무 영향도 못 준 것이다.
    assert_ne!(
        width_off, width_on,
        "포켓에 {accepted_on}개를 넣었는데 폭이 그대로다"
    );
}
