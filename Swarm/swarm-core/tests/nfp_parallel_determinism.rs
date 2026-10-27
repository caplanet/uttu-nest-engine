//! 병렬화한 로직이 **실행마다 같은 결과**를 내는지 지킨다.
//!
//! 이 코드베이스는 회귀를 비트 단위 비교로 잡는다. 병렬 축약이 순서에 의존하면
//! 결과가 실행마다 흔들리고, 그 순간 모든 실측치가 무의미해진다.
//!
//! 특히 위험한 것 둘.
//!
//! * **부동소수 epsilon 비교는 추이성이 없다.** `|a-b| <= eps` 로 동률을 판정하면
//!   `a~b`, `b~c` 인데 `a<c` 인 삼각형이 생긴다. 그런 비교로 최댓값을 고르면
//!   **훑는 순서에 따라 답이 달라진다** — 순차에서는 늘 같은 순서라 안 드러나지만,
//!   병렬로 청크를 나눠 축약하면 즉시 드러난다.
//! * **rayon 의 청크 분할**은 스레드 수에 따라 달라질 수 있다. 청크 경계가 바뀌면
//!   축약 순서가 바뀐다.
//!
//! 그래서 "돌려 보니 같더라" 로는 부족하고, **여러 번 + 스레드 수를 바꿔 가며**
//! 확인해야 한다.

#![cfg(feature = "adaptive")]

use jagua_rs::probs::spp::entities::SPInstance;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::nfp_lbf::{NfpLbfCache, PlacementScoreWeights, PreparedLibrary};
use swarm_core::nfp_strategy::accrete::{AccretionOutcome, accrete};
use swarm_core::nfp_strategy::select::SelectionRule;
use swarm_core::util::io::read_spp_input;
use std::path::Path;
use std::time::Duration;

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

/// 결과를 비트 단위로 비교할 수 있는 형태로 굳힌다.
///
/// `f32` 를 `to_bits()` 로 본다 — `==` 는 `-0.0 == 0.0` 이고 `NaN != NaN` 이라
/// 회귀 판정에 쓰면 안 된다.
fn fingerprint(out: &AccretionOutcome) -> Vec<(usize, i64, u32, u32)> {
    out.pieces
        .iter()
        .map(|p| {
            (
                p.item_id,
                p.rotation,
                p.translation.0.to_bits(),
                p.translation.1.to_bits(),
            )
        })
        .collect()
}

struct Fixture {
    cache: NfpLbfCache,
    remaining: Vec<usize>,
    strip_height: f32,
    cap: f32,
    item_rotations: Vec<Vec<i64>>,
}

fn fixture(name: &str) -> Fixture {
    let inst = load(name);
    let library = PreparedLibrary::build(&inst, &[0], 0.02).expect("라이브러리");
    let mut cache = NfpLbfCache::new(library);
    cache.prewarm(Duration::from_secs(30));

    let max_id = inst.items.iter().map(|(i, _)| i.idx()).max().unwrap_or(0);
    let item_rotations: Vec<Vec<i64>> = (0..=max_id).map(|_| vec![0i64]).collect();
    let cap: f32 = inst
        .items
        .iter()
        .map(|(it, q)| it.shape_cd().bbox().width() * (*q as f32))
        .sum();
    let mut remaining = Vec::new();
    for (item, qty) in inst.items.iter() {
        for _ in 0..*qty {
            remaining.push(item.idx());
        }
    }
    Fixture {
        cache,
        remaining,
        strip_height: inst.base_strip.fixed_height,
        cap,
        item_rotations,
    }
}

fn run(f: &Fixture, probes: usize) -> AccretionOutcome {
    accrete(
        &f.cache,
        &f.remaining,
        f.strip_height,
        f.cap,
        PlacementScoreWeights::default(),
        &f.item_rotations,
        probes,
        SelectionRule::MaxContact,
        // 예산을 주지 않는다 — 시간 제한이 걸리면 그 자체로 결과가 흔들린다.
        None,
    )
}

/// 같은 입력을 여러 번 돌려 **비트 단위로 같은 배치**가 나오는지 본다.
///
/// 병렬 축약이 순서에 의존하면 여기서 걸린다. 실행 간 스레드 스케줄링이 달라지므로
/// 반복만으로도 상당한 검출력이 있다.
#[test]
fn accretion_gives_the_same_layout_every_run() {
    let f = fixture("albano.json");
    let base = run(&f, 4096);
    let base_fp = fingerprint(&base);
    assert!(!base_fp.is_empty(), "fixture 가 아무것도 놓지 못했다");

    for attempt in 1..8 {
        let again = run(&f, 4096);
        assert_eq!(
            fingerprint(&again),
            base_fp,
            "{attempt}번째 실행이 다른 배치를 냈다 — 병렬 축약이 순서에 의존한다"
        );
        assert_eq!(again.unplaced.len(), base.unplaced.len());
        assert_eq!(
            again.strip_length.to_bits(),
            base.strip_length.to_bits(),
            "strip 길이가 달라졌다"
        );
    }
}

/// **스레드 수를 바꿔도** 같은 결과여야 한다.
///
/// 청크 경계가 스레드 수에 따라 달라지므로 축약 순서도 달라진다. 반복 실행보다
/// 이쪽이 검출력이 훨씬 높다 — 순서 의존이 있으면 거의 확실히 드러난다.
///
/// 1 스레드 결과가 기준이다. 그게 순차 실행과 같은 답이어야 한다.
#[test]
fn accretion_does_not_depend_on_how_many_threads_run_it() {
    let f = fixture("albano.json");

    let mut reference: Option<Vec<(usize, i64, u32, u32)>> = None;
    for threads in [1usize, 2, 3, 5, 8, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("스레드 풀");
        let out = pool.install(|| run(&f, 4096));
        let fp = fingerprint(&out);
        match &reference {
            None => reference = Some(fp),
            Some(r) => assert_eq!(
                &fp, r,
                "스레드 {threads}개일 때 결과가 달라졌다 — 청크 분할이 답을 바꾼다"
            ),
        }
    }
}

/// 놓은 조각 수와 수요가 맞는가 — 병렬화가 조각을 잃거나 중복 배치하지 않는지.
///
/// 결정론과 별개 축이다. 매번 같은 답을 내면서 **매번 틀릴** 수도 있다.
#[test]
fn accretion_places_each_piece_exactly_once() {
    let f = fixture("albano.json");
    let out = run(&f, 4096);

    let mut placed_count = vec![0usize; f.item_rotations.len()];
    for p in out.pieces.iter() {
        if let Some(c) = placed_count.get_mut(p.item_id) {
            *c += 1;
        } else {
            panic!("범위 밖 item_id {}", p.item_id);
        }
    }
    let mut demand = vec![0usize; f.item_rotations.len()];
    for &id in f.remaining.iter() {
        if let Some(d) = demand.get_mut(id) {
            *d += 1;
        }
    }
    for (id, (&got, &want)) in placed_count.iter().zip(demand.iter()).enumerate() {
        assert!(
            got <= want,
            "item {id} 을 수요({want})보다 많이({got}) 놓았다"
        );
    }
    assert_eq!(
        out.pieces.len() + out.unplaced.len(),
        f.remaining.len(),
        "놓은 것 + 못 놓은 것이 수요와 다르다 — 조각을 잃었다"
    );
}

/// 156-패널에서 스레드 수를 바꿔 확인한다 — **검출력이 가장 높은 검사**다.
///
/// albano 는 24조각이라 한 단계에서 겨루는 후보가 적고, 청크가 1~2개로 끝나 순서
/// 의존이 드러날 기회가 적다. 156조각은 매 단계 수십 개가 겨루고 청크가 여러 개로
/// 갈리므로, 축약이 순서에 의존하면 거의 확실히 답이 갈린다.
///
/// 탐침 예산을 낮춰(1024) 시간을 줄인다 — 결정론은 예산과 무관하고, 오히려 후보가
/// 적을수록 동률이 잦아 순서 의존이 **더** 잘 드러난다.
#[test]
fn accretion_stays_deterministic_on_the_large_garment_marker() {
    let f = fixture("wsjk156.json");

    let mut reference: Option<Vec<(usize, i64, u32, u32)>> = None;
    for threads in [1usize, 3, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("스레드 풀");
        let out = pool.install(|| run(&f, 1024));
        assert_eq!(
            out.pieces.len() + out.unplaced.len(),
            f.remaining.len(),
            "스레드 {threads}개에서 조각을 잃었다"
        );
        let fp = fingerprint(&out);
        match &reference {
            None => reference = Some(fp),
            Some(r) => {
                // 어디서 갈렸는지 알려 준다 — 156개 중 한 자리만 달라도 잡아야 한다.
                if &fp != r {
                    let first = fp
                        .iter()
                        .zip(r.iter())
                        .position(|(a, b)| a != b)
                        .unwrap_or(fp.len().min(r.len()));
                    panic!(
                        "스레드 {threads}개에서 결과가 갈렸다. 처음 다른 자리 = {first}번째 조각 \
                         (전체 {} vs {})",
                        fp.len(),
                        r.len()
                    );
                }
            }
        }
    }
}
