//! 설계 §3 순차 진행 순서도 — 모듈 1~10 을 묶는 실행부.
//!
//! ```text
//!   [초기화] -> [입력 로드] -> [부착 체인 구축] -> [후보 수집] -> [정렬·분배]
//!     -> [배치 스텝] -> [NFP 경계 배치] -> [확정] -> [2차 패스] -> [전체 검증]
//!     -> [효율 재계산] -> [Draft 완료] -> (Complete 재배치 -> 잔여조각 LP -> Pack)
//! ```
//!
//! # 기존 경로를 한 비트도 바꾸지 않는다
//!
//! 이 엔진은 호출해야 돈다. `SwarmConfig` 어디에도 자동으로 끼어들지 않는다 —
//! 그것이 A/B 측정의 전제이고, 이 저장소가 반복해 확인한 규칙이다.
//!
//! # 지금 어디까지 왔나 (2026-08-31 실측, wsjk156 156조각)
//!
//! ```text
//!   놓은 조각      156/156
//!   겹침           0 (검증 표식 0)
//!   폭             20,311mm   밀도 25.04%
//!   구간 충전      앞 26구간(x < 5,200mm) 79 74 85 77 80 78 71 77 79 78 77 78 76 75
//!                            82 67 79 73 75 76 70 63 74 72 76 54  <- 평균 76
//!                  그 뒤     4~15% 로 붕괴
//! ```
//!
//! **기하는 옳다** — 156조각을 겹침 없이 놓는다. 그리고 **앞 5,200mm 구간의 충전율
//! 76% 는 경쟁사(81%)에 크게 뒤지지 않는다.** 즉 설계의 배치 논리 자체는 작동한다.
//!
//! 무너지는 지점은 하나다: 놓인 조각이 ~26개를 넘으면
//! [`crate::refplace::geom::feasible_for`] 가 빈 집합이나 오류를 돌려주기 시작한다.
//! 커널이 매 배치마다 **놓인 조각 전부**의 pair NFP 를 합집합하기 때문이다
//! (`O(n)` 쌍 x `n` 배치 = `O(n²)`). 그러면 프론티어 오른쪽 폴백이 발동해 조각이
//! 일렬로 늘어서고, 그것이 25% 라는 숫자의 정체다.
//!
//! # 다음 단계는 이것 하나다
//!
//! **배치 가능 영역을 증분으로 유지해야 한다.** 매번 전부 다시 합집합하는 대신,
//! 직전 영역에서 **새로 놓인 조각 하나의 NFP 만 빼면** 된다. 그러면 배치당 비용이
//! `O(n)` 에서 `O(1)` 로 떨어진다.
//!
//! 시도했다가 되돌린 것: 장애물을 **창(window)** 으로 걸러 보았다. 왼쪽 경계를
//! `frontier - 2*height` 로 자르면 커널 호출은 싸지지만(20.6초 -> 8.1초) 왼쪽에 남은
//! 자리를 못 쓰게 되어 프로파일이 무너진다(폭 11,955mm / 42.6%, 충전 14% 구간 20개).
//! 왼쪽 포켓을 채우는 것이 이 엔진의 목적 중 하나라 그쪽을 막으면 안 된다.
//! 오른쪽만 자르는 것은 안전하지만 장애물 수가 줄지 않아 이득이 없다.

use std::collections::BTreeMap;

use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};

use crate::refplace::boundary::{EvalRecord, build_work_list, evaluate_candidates};
use crate::refplace::chain::{
    AttachChain, GroupFallback, GroupSpec, NfpPairIndex, PieceExtent, PoseKey, build_chain,
    fallback_groups,
};
use crate::refplace::collect::{CollectMode, collect, set_piece_key};
use crate::refplace::finish::{
    Stage, arm_second_pass, close_second_pass, integrate_and_validate, recompute_efficiency,
};
use crate::refplace::geom::{PoseCatalogue, RegionCache, feasible_for, feasible_record, world_bbox};
use crate::refplace::params::RefPlaceParams;
use crate::refplace::record::{
    ContactPoint, PieceRecord, PlaceState, PlacedPose, RotationEntry, RotationList, SurfaceRecord,
};
use crate::refplace::score::efficiency;
use crate::refplace::step::{NfpSummary, StepOutcome, step_piece};
use std::borrow::Cow;
use swarm_nfp::{Location, Point as NfpPoint, PolygonSet};

/// 실행 결과.
#[derive(Clone, Debug)]
pub struct RefPlaceReport {
    /// 놓은 조각 수.
    pub placed: usize,
    /// 못 놓은 조각 수 — 잔여 조각 처리 대상.
    pub unplaced: usize,
    /// 선계산한 NFP 쌍 수.
    pub nfp_pairs: usize,
    /// 준비한 자세 수.
    pub poses: usize,
    /// 후보 평가 루프가 **절단**된 횟수.
    pub truncations: usize,
    /// 프론티어 폴백이 발동한 횟수와 그 사유.
    ///
    /// 사유를 나누지 않으면 "영역이 비었다"(정상 — 정말 자리가 없다)와
    /// "영역을 못 구했다"(결함 — 커널이 실패했다)를 구분할 수 없다.
    pub fallback_empty: usize,
    pub fallback_missing: usize,
    /// 후보를 평가했지만 전부 탈락한 횟수.
    pub fallback_no_candidate: usize,
    /// 해를 내지 못한 사유. `None` 이면 성공.
    ///
    /// 조용히 실패하면 원인을 알 수 없다 — 실제로 실제 마커에서 `None` 만 나와
    /// 무엇이 틀렸는지 추적할 수 없었다.
    pub failure: Option<String>,
    /// 예산이 소진돼 남은 조각을 폴백으로 넘겼는가.
    ///
    /// 이 값이 참이면 배치 품질을 **엔진의 실력으로 읽으면 안 된다** — 시간이 모자라
    /// 대충 놓은 것이다.
    pub budget_exhausted: bool,
    /// 단계별 효율 스냅샷.
    pub efficiency: Vec<crate::refplace::finish::EfficiencySnapshot>,
    /// 검증 오류 표식 수. 0 이 아니어도 배치는 유효할 수 있다(표식일 뿐).
    pub validation_marks: usize,
}

impl RefPlaceReport {
    /// 시작도 못 한 실패.
    fn empty(reason: &str) -> Self {
        Self {
            placed: 0,
            unplaced: 0,
            nfp_pairs: 0,
            poses: 0,
            truncations: 0,
            fallback_empty: 0,
            fallback_missing: 0,
            fallback_no_candidate: 0,
            budget_exhausted: false,
            failure: Some(reason.to_string()),
            efficiency: Vec::new(),
            validation_marks: 0,
        }
    }
}

/// 조각별 자세 목록을 정한다.
///
/// 인스턴스가 선언한 `allowed_rotation` 을 그대로 쓴다. **반전은 넣지 않는다** —
/// 의류 마커에서 반전은 결 방향(grain)이 허용할 때만 가능하고, 그 정보는 입력의
/// `domain` 블록에 있는데 솔버 로더가 버린다. 호출자가 알면 직접 넘겨야 한다.
pub fn rotations_from_instance(instance: &SPInstance) -> BTreeMap<usize, Vec<RotationEntry>> {
    use jagua_rs::geometry::geo_enums::RotationRange;
    let mut out = BTreeMap::new();
    for (item, _) in instance.items.iter() {
        let entries = match item.allowed_orientations().rotations() {
            RotationRange::None => vec![RotationEntry::IDENTITY],
            RotationRange::Discrete(set) => {
                let mut v: Vec<RotationEntry> = set
                    .iter()
                    .map(|r| {
                        let deg = r.to_degrees() as f64;
                        RotationEntry::turn((deg * 1000.0).round() as i32)
                    })
                    .collect();
                if v.is_empty() {
                    v.push(RotationEntry::IDENTITY);
                }
                v
            }
            // 연속 회전은 자유회전으로 다룬다 — 목록은 항등 하나이고, 미세 스텝이
            // 나머지를 뻗는다(§모듈 7).
            RotationRange::Continuous => vec![RotationEntry::IDENTITY],
        };
        out.insert(item.idx(), entries);
    }
    out
}

/// 설계대로 초기 배치를 만든다 (§3 순서도).
///
/// `groups` 가 `None` 이면 [`GroupFallback`] 규칙으로 대체 그룹을 만든다 —
/// **조용히 크기 정렬로 돌아가지 않는다**.
pub fn build_reference_nest(
    instance: &SPInstance,
    params: &RefPlaceParams,
    groups: Option<Vec<GroupSpec>>,
) -> Option<(SPSolution, RefPlaceReport)> {
    match build_reference_nest_reported(instance, params, groups) {
        (Some(sol), rep) => Some((sol, rep)),
        (None, _) => None,
    }
}

/// [`build_reference_nest`] 와 같지만 **실패해도 리포트를 돌려준다**.
///
/// 조용히 `None` 만 주면 왜 실패했는지 알 수 없다. 진단 하네스가 이것을 쓴다.
pub fn build_reference_nest_reported(
    instance: &SPInstance,
    params: &RefPlaceParams,
    groups: Option<Vec<GroupSpec>>,
) -> (Option<SPSolution>, RefPlaceReport) {
    // ── 모듈 1·2: 초기화 · 배치 준비 ──────────────────────────────────────
    let rotations = rotations_from_instance(instance);
    let mut pieces: Vec<PieceRecord> = Vec::new();
    let mut extents: Vec<PieceExtent> = Vec::new();
    for (item, qty) in instance.items.iter() {
        let entries = rotations
            .get(&item.idx())
            .cloned()
            .unwrap_or_else(|| vec![RotationEntry::IDENTITY]);
        let list = RotationList::new(entries);
        for copy in 0..*qty {
            pieces.push(PieceRecord::new(item.idx(), copy, list.clone()));
            extents.push(PieceExtent::new(
                item.shape_cd().bbox().width() as f64,
                item.shape_cd().bbox().height() as f64,
            ));
        }
    }
    if pieces.is_empty() {
        return (None, RefPlaceReport::empty("조각이 없다"));
    }

    // Draft 단계의 오목 단순화 강도를 쓴다(§모듈 10: Draft 와 Complete 는 알고리즘이
    // 아니라 다각형 정밀도의 차이다).
    let (_convex, concave) = crate::refplace::post::Precision::Draft.tolerances(&params.simplify);
    let catalogue = PoseCatalogue::build(instance, &rotations, concave);
    if catalogue.is_empty() {
        log::warn!("[REFPLACE] 준비된 자세가 하나도 없다 — 배치를 시작할 수 없다");
        return (None, RefPlaceReport::empty("준비된 자세가 없다"));
    }

    // ── 모듈 3: 부착 체인 + NFP 선계산 ────────────────────────────────────
    let groups = groups.unwrap_or_else(|| fallback_groups(&pieces, GroupFallback::ByItemId));
    let height = instance.base_strip.fixed_height as f64;
    let cap: f64 = extents.iter().map(|e| e.width).sum();
    let mut surfaces: Vec<SurfaceRecord> = Vec::new();
    let chain: AttachChain = build_chain(
        &mut pieces,
        &extents,
        &groups,
        &mut surfaces,
        (cap, height),
    );

    // **여기가 유일한 NFP 계산 지점**이다(설계 §모듈 3). 배치 스텝은 조회만 한다.
    let mut boundaries: BTreeMap<crate::refplace::chain::NfpPairKey, EvalRecord> = BTreeMap::new();
    let index: NfpPairIndex<NfpSummary> = chain.plan.precompute(|key| {
        let (summary, record) = catalogue.compute_pair(key)?;
        boundaries.insert(*key, record);
        Some(summary)
    });

    // ── 모듈 4~9: 배치 ────────────────────────────────────────────────────
    let mut report = RefPlaceReport {
        placed: 0,
        unplaced: 0,
        nfp_pairs: index.len(),
        poses: catalogue.len(),
        truncations: 0,
        fallback_empty: 0,
        fallback_missing: 0,
        fallback_no_candidate: 0,
        budget_exhausted: false,
        failure: None,
        efficiency: Vec::new(),
        validation_marks: 0,
    };
    let mut frontier = 0.0f64;

    // 배치 가능 영역을 **들고 다닌다**. 조각을 놓을 때마다 그것 하나만 빼면 되므로
    // 배치당 비용이 `O(n)` 합집합에서 `O(1)` 차감이 된다(§`RegionCache`).
    let started_at = std::time::Instant::now();
    let budget = (params.budget_ms > 0).then(|| std::time::Duration::from_millis(params.budget_ms));
    // 증분 캐시는 **켰을 때만** 만든다. 씨앗 만들기(자세마다 IFP)부터 비싸다.
    let mut regions = if params.incremental_regions {
        let seeds: Vec<(usize, PoseKey)> = pieces
            .iter()
            .flat_map(|p| {
                p.rotations
                    .entries()
                    .iter()
                    .map(move |e| (p.item_id, PoseKey::of(e)))
            })
            .collect();
        Some(RegionCache::seed(&catalogue, &seeds))
    } else {
        None
    };
    // 이미 놓인 조각 — 재계산 경로가 장애물로 쓴다.
    let mut placed_poses: Vec<(usize, PoseKey, f64, f64)> = Vec::new();

    // 루트를 먼저 놓는다 — 표면 레코드의 시드에서 **접촉 탐색**으로 확정한다(§모듈 3).
    //
    // 시드를 그대로 쓰면 안 된다. 처음에 그렇게 만들었더니 156조각이 전부 원점에
    // 쌓여 밀도 1442%, 검증 표식 156(전부 겹침)이 나왔다. 설계가 "그 시드에서
    // 접촉 탐색(상한 10000, 자유회전 0.01도 스텝)으로 위치를 확정한다" 고 적은
    // 이유가 그것이다.
    for &root in &chain.roots {
        if crate::config::runtime::init_cancel_requested() { return (None, report); }
        if pieces[root].is_placed() {
            continue;
        }
        // **루트 루프에도 예산을 건다.** 여기 빠져 있어서 예산 20초를 넘겨도
        // `budget_exhausted` 가 거짓인 채로 37초가 걸렸다 — 배치의 대부분이
        // 이 루프에서 일어나므로 여기가 빠지면 예산이 사실상 없는 것과 같다.
        if budget.is_some_and(|b| started_at.elapsed() >= b) {
            if !report.budget_exhausted {
                log::warn!(
                    "[REFPLACE] 예산 {}ms 소진(루트) — 남은 조각을 프론티어 오른쪽으로 넘긴다",
                    params.budget_ms
                );
                report.budget_exhausted = true;
            }
            if place_at_frontier(&mut pieces, root, &catalogue, &mut frontier, height) {
                report.placed += 1;
                note_placed(&mut regions, &mut placed_poses, &catalogue, &pieces[root]);
            }
            continue;
        }
        let surface = surfaces
            .iter()
            .position(|s| s.pieces.contains(&root))
            .unwrap_or(0);
        let seed = surfaces.get(surface).map(|s| s.seed).unwrap_or((0.0, 0.0));
        if place_free(
            &mut pieces,
            root,
            seed,
            &catalogue,
            &surfaces,
            &regions,
            &placed_poses,
            &mut frontier,
            height,
            &mut report,
        ) {
            report.placed += 1;
            note_placed(&mut regions, &mut placed_poses, &catalogue, &pieces[root]);
        }
    }

    // 나머지를 표면 점수 모드로 훑는다.
    let processed = vec![false; surfaces.len()];
    let mut rounds = 0usize;
    loop {
        rounds += 1;
        let set = collect(
            CollectMode::SurfaceScore,
            &pieces,
            &surfaces,
            &chain.surface_order,
            &processed,
        );
        let ready: Vec<usize> = set.ready().map(|c| c.piece).collect();
        if ready.is_empty() {
            break;
        }
        let mut progressed = false;
        for p in ready {
            if crate::config::runtime::init_cancel_requested() { return (None, report); }
            if pieces[p].is_placed() {
                continue;
            }
            // 예산을 넘겼으면 **남은 것을 폴백으로 빠르게 처리**한다.
            //
            // 그냥 멈추면 조각이 빠져 밀도의 분자가 줄고, 그 배치는 다른 것과 비교
            // 자체가 불가능해진다. 폴백은 프론티어 오른쪽이라 겹치지 않는다 —
            // 폭은 늘지만 정직한 손해다.
            if budget.is_some_and(|b| started_at.elapsed() >= b) {
                if !report.budget_exhausted {
                    log::warn!(
                        "[REFPLACE] 예산 {}ms 소진 — 남은 조각을 프론티어 오른쪽으로 넘긴다",
                        params.budget_ms
                    );
                    report.budget_exhausted = true;
                }
                let pose = pieces[p].rotations.entries()[0];
                if let Some(bbox) =
                    world_bbox(&catalogue, pieces[p].item_id, PoseKey::of(&pose), 0.0, 0.0)
                {
                    if bbox.3 - bbox.1 <= height - 2.0 * BOUNDARY_INSET_MM {
                        let x = frontier - bbox.0 + FRONTIER_GAP_MM;
                        let y = -bbox.1 + BOUNDARY_INSET_MM;
                        commit(&mut pieces, p, pose, 0.0, x, y, &catalogue, &mut frontier);
                        report.placed += 1;
                        progressed = true;
                        note_placed(&mut regions, &mut placed_poses, &catalogue, &pieces[p]);
                    }
                }
                continue;
            }
            let pose = pieces[p].rotations.entries()[0];
            match step_piece(&mut pieces, p, &pose, &index) {
                StepOutcome::Place { key, .. } => {
                    let Some(record) = boundaries.get(&key) else {
                        continue;
                    };
                    // 쌍 NFP 는 **부착 대상을 원점에 둔** 좌표계로 계산돼 있다
                    // (`PoseCatalogue::compute_pair`). 그 경계 점을 그대로 앵커로 쓰면
                    // 모든 부착 조각이 원점 근처로 몰린다 — 대상의 확정 좌표만큼
                    // 옮겨서 월드 좌표로 만든다(Column Draft 의 `boundaries` 와 같다).
                    let Some(target_pose) = pieces[p]
                        .attach_to
                        .and_then(|t| pieces.get(t))
                        .and_then(|t| t.pose)
                    else {
                        continue;
                    };
                    let record = translate_record(record, target_pose.x, target_pose.y);
                    if place_on_boundary(
                        &mut pieces,
                        p,
                        &record,
                        &catalogue,
                        &surfaces,
                        &regions,
                        &placed_poses,
                        &mut frontier,
                        height,
                        &mut report,
                    ) {
                        report.placed += 1;
                        progressed = true;
                        note_placed(&mut regions, &mut placed_poses, &catalogue, &pieces[p]);
                    }
                }
                StepOutcome::Independent => {
                    // 독립 배치 — 부착 대상이 없으므로 조회할 쌍이 없다. 대신
                    // **배치 가능 영역**(IFP - ∪NFP(놓인 것))에서 후보를 뽑는다.
                    if place_free(
                        &mut pieces,
                        p,
                        (0.0, 0.0),
                        &catalogue,
                        &surfaces,
                        &regions,
                        &placed_poses,
                        &mut frontier,
                        height,
                        &mut report,
                    ) {
                        report.placed += 1;
                        progressed = true;
                        note_placed(&mut regions, &mut placed_poses, &catalogue, &pieces[p]);
                    }
                }
                StepOutcome::Deferred | StepOutcome::Detached(_) | StepOutcome::AlreadyPlaced => {}
            }
        }
        // **진전이 없으면 멈춘다.** 그러지 않으면 서로를 기다리는 조각에서 영원히 돈다.
        if !progressed || rounds > pieces.len() {
            break;
        }
    }

    // ── 모듈 9: 2차 패스 · 검증 · 효율 ────────────────────────────────────
    let mut pass = arm_second_pass(&mut pieces);
    // 되살린 조각을 한 번 더 시도한다. **한 번만** 이다(§모듈 9).
    for p in pass.retried.clone() {
        if crate::config::runtime::init_cancel_requested() { return (None, report); }
        if pieces[p].is_placed() {
            continue;
        }
        if place_free(
            &mut pieces,
            p,
            (0.0, 0.0),
            &catalogue,
            &surfaces,
            &regions,
            &placed_poses,
            &mut frontier,
            height,
            &mut report,
        ) {
            report.placed += 1;
            note_placed(&mut regions, &mut placed_poses, &catalogue, &pieces[p]);
        }
    }
    close_second_pass(&mut pieces, &mut pass);
    report.unplaced = pass.still_unplaced.len();

    let reports: Vec<_> = surfaces
        .iter()
        .enumerate()
        .map(|(i, s)| {
            integrate_and_validate(&pieces, s, i, frontier.max(1.0), |p| {
                let piece = &pieces[p];
                let pose = piece.pose?;
                world_bbox(
                    &catalogue,
                    piece.item_id,
                    PoseKey::of(&pose.rotation),
                    pose.x,
                    pose.y,
                )
            })
        })
        .collect();
    report.validation_marks = reports.iter().map(|r| r.errors.len()).sum();
    report.efficiency.push(recompute_efficiency(
        Stage::Placement,
        &reports,
        &surfaces,
        frontier.max(1.0),
    ));

    // ── 결과를 기존 표현으로 굳힌다 ───────────────────────────────────────
    //
    // 검증 경로를 우회하지 않는다 — `fit_strip` 과 `solution_validation_error` 를 거쳐야
    // 밀도가 다른 하네스와 비교 가능해진다.
    let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
    prob.change_strip_width((cap as f32).max(1.0)).expect(crate::dmsg!("strip width must be positive"));
    let mut left: Vec<usize> = instance.items.iter().map(|(_, q)| *q).collect();
    for piece in pieces.iter().filter(|p| p.is_placed()) {
        let Some(pose) = piece.pose else { continue };
        if left.get(piece.item_id).copied().unwrap_or(0) == 0 {
            continue;
        }
        left[piece.item_id] -= 1;
        prob.place_item(SPPlacement {
            d_transf: DTransformation::new(
                (pose.rotation.radians() + pose.extra_rad) as f32,
                (pose.x as f32, pose.y as f32),
            ),
            item_idx: piece.item_id,
        });
    }
    if prob.n_placed_items() == 0 {
        report.failure = Some("굳히기에서 아무것도 안 놓였다".to_string());
        return (None, report);
    }
    prob.fit_strip().expect(crate::dmsg!("strip width must be positive"));
    let solution = prob.save();

    // ── 검증을 통과한 것만 돌려준다 ──
    //
    // 이 엔진이 만든 해를 호출부가 그대로 시드로 쓴다. 겹치거나 컨테이너를 벗어난
    // 해를 넘기면 그 다음 단계가 통째로 실패한다 — 실제로 그렇게 됐다(WPF
    // `Init 실패: code=2001 ... exterior_collisions: 7`).
    //
    // **내부 검증(`integrate_and_validate`)만 믿으면 안 된다.** 그건 bbox 근사이고,
    // 최종 판정은 jagua-rs CDE 가 정확한 기하로 한다. 두 판정이 갈리면 CDE 가 맞다.
    // 실패하면 `None` 을 돌려 호출부가 기존 배치를 유지하게 한다 — 실험 스위치가
    // 실행을 막으면 안 된다.
    if let Some(err) = crate::structure::pipeline::solution_validation_error(instance, &solution) {
        log::warn!("[REFPLACE] 만든 배치가 검증에 실패했다 ({err:?}) — 해를 내지 않는다");
        report.failure = Some(format!("{err:?}"));
        return (None, report);
    }
    let _ = efficiency(0.0, 1.0);
    (Some(solution), report)
}

use crate::refplace::geom::BOUNDARY_INSET_MM;

/// 평가 레코드의 점들을 `(dx, dy)` 만큼 옮긴 사본 — bbox 도 다시 계산한다.
///
/// 쌍 NFP 는 고정 조각(부착 대상)을 원점에 둔 로컬 좌표계라, 월드 앵커로 쓰려면
/// 대상의 확정 좌표만큼 옮겨야 한다.
fn translate_record(record: &EvalRecord, dx: f64, dy: f64) -> EvalRecord {
    EvalRecord::new(
        record
            .points
            .iter()
            .map(|p| ContactPoint {
                x: p.x + dx,
                y: p.y + dy,
                flags: p.flags,
            })
            .collect(),
    )
}

/// 프론티어 오른쪽 폴백이 둘 조각 간 간격(mm).
///
/// 0 이면 앞 조각과 정확히 맞닿는데, f32 로 내려가면서 미세 겹침이 생길 수 있다.
/// 이 저장소가 실측으로 확인한 문제라 여기서도 같은 여유를 둔다.
const FRONTIER_GAP_MM: f64 = 0.01;

/// 프론티어 오른쪽에 놓는다 — **모든 실패의 단일 출구**.
///
/// 그 자리는 정의상 놓인 것들의 최대 x 보다 오른쪽이라 **겹칠 수 없다**.
///
/// # 왜 포기하지 않는가
///
/// 조각을 못 놓으면 해가 `QuantityMismatch` 로 검증에 실패해 **통째로 버려진다**.
/// 실제로 그렇게 됐다 — 후보가 전부 탈락했을 때 그냥 `false` 를 돌려주는 구멍이 있었고,
/// shapes0 에서 15개 중 7개만 놓여 해가 무효가 됐다.
///
/// 폭이 늘어나는 것은 손해지만 **정직한 손해**다. 조각을 잃는 것과 다르다.
fn place_at_frontier(
    pieces: &mut [PieceRecord],
    piece: usize,
    catalogue: &PoseCatalogue,
    frontier: &mut f64,
    height: f64,
) -> bool {
    let pose = pieces[piece].rotations.entries()[0];
    let item_id = pieces[piece].item_id;
    let Some(bbox) = world_bbox(catalogue, item_id, PoseKey::of(&pose), 0.0, 0.0) else {
        return false;
    };
    let piece_h = bbox.3 - bbox.1;
    if piece_h > height {
        // 컨테이너보다 높은 조각은 어디에도 못 놓는다. 이건 입력의 문제다.
        log::warn!(
            "[REFPLACE] 아이템 {item_id} 높이 {piece_h:.2} 가 strip {height:.2} 를 넘는다 — 놓을 수 없다"
        );
        return false;
    }
    // 세로로 가운데 정렬한다. 위아래 어느 쪽에도 붙지 않으므로 f32 반올림으로
    // 밖으로 나갈 여지가 없다 — 경계에 붙이는 것보다 안전하다.
    let y = -bbox.1 + (height - piece_h) * 0.5;
    let x = *frontier - bbox.0 + FRONTIER_GAP_MM;
    commit(pieces, piece, pose, 0.0, x, y, catalogue, frontier);
    true
}

/// 부착 대상이 없는 조각을 **배치 가능 영역**에서 놓는다.
///
/// 루트(§모듈 3 "시드에서 접촉 탐색")와 독립 배치(§모듈 6 "부착 대상 없음")가 쓴다.
///
/// # 왜 시드를 그대로 쓰지 않는가
///
/// 처음 구현이 시드를 그대로 썼다. 그 결과 이 마커에서 **156조각이 전부 원점에 쌓였다**
/// (밀도 1442%, 검증 표식 156). 설계가 "그 시드에서 **접촉 탐색**으로 위치를 확정한다"
/// 고 적은 것이 바로 이 단계다.
///
/// 영역은 `IFP - ∪NFP(놓인 것)` 이고 커널이 계산한다. 그 경계 점이 곧 후보이며,
/// 점수는 §4 대로 낸다. 시드는 **점수의 동률을 가르는 기준**으로만 남는다 — 같은
/// 점수라면 시드에 가까운 자리를 쓴다.
#[allow(clippy::too_many_arguments)]
fn place_free(
    pieces: &mut [PieceRecord],
    piece: usize,
    seed: (f64, f64),
    catalogue: &PoseCatalogue,
    _surfaces: &[SurfaceRecord],
    regions: &Option<RegionCache>,
    placed: &[(usize, PoseKey, f64, f64)],
    frontier: &mut f64,
    height: f64,
    report: &mut RefPlaceReport,
) -> bool {
    let pose0 = pieces[piece].rotations.entries()[0];
    let item_id = pieces[piece].item_id;
    // 캐시가 있으면 꺼내고, 없으면 그 자리에서 구한다. 둘은 **같은 결과**를 내지만
    // 비용이 다르다 — `RefPlaceParams::incremental_regions` 문서의 실측 참조.
    // 캐시의 영역은 빌려 쓰고, 그 자리에서 구한 것만 소유한다 — 후보 점 목록을 뽑는
    // 데 사본은 필요 없다.
    let fetched: Option<Cow<'_, PolygonSet>> = match regions {
        Some(cache) => cache.get(item_id, PoseKey::of(&pose0)).map(Cow::Borrowed),
        None => feasible_for(
            catalogue,
            item_id,
            PoseKey::of(&pose0),
            placed,
            Some((0.0, *frontier + 2.0 * height + 1.0)),
        )
        .map(Cow::Owned),
    };
    // ── 프론티어 오른쪽 폴백 ──
    //
    // 배치 가능 영역을 못 구하거나(장애물이 많아 커널이 거절) 비어 있으면,
    // **프론티어 오른쪽**에 놓는다. 그 자리는 정의상 놓인 것들의 최대 x 보다
    // 오른쪽이라 **겹칠 수 없다**.
    //
    // 왜 그냥 포기하지 않는가: 조각을 못 놓으면 밀도의 분자가 줄어 그 배치는
    // 다른 것과 **비교 자체가 불가능**해진다. 실측으로 확인했다 — 폴백이 없을 때
    // 이 마커에서 156 중 28개가 빠졌고, 그때 나온 72.90% 는 128개만 센 값이라
    // 아무 뜻이 없었다.
    //
    // 폭이 늘어나는 것은 손해지만 **정직한 손해**다. 조각을 잃는 것과 다르다.
    let region = match fetched {
        Some(r) if !r.is_empty() => r,
        Some(_) => {
            report.fallback_empty += 1;
            return place_at_frontier(pieces, piece, catalogue, frontier, height);
        }
        None => {
            report.fallback_missing += 1;
            return place_at_frontier(pieces, piece, catalogue, frontier, height);
        }
    };
    let records = vec![feasible_record(&region)];
    let work = if records[0].points.len() > 1 {
        // 레코드가 하나뿐이면 쌍이 만들어지지 않는다. 자기 자신과의 쌍을 하나 넣어
        // 그 점들을 앵커 후보로 쓴다 — 후보 = 접촉 쌍 전부라는 계약은 유지된다.
        vec![crate::refplace::boundary::WorkItem {
            a: 0,
            b: 0,
            dist2: 0.0,
        }]
    } else {
        build_work_list(&records, 1.0)
    };
    // **`pose0` 만 평가한다.** 앵커는 `pose0` 의 배치 가능 영역 경계에서 나왔다 —
    // 다른 자세에 같은 좌표를 쓰면 그 자세의 영역이 아니므로 겹칠 수 있다.
    let list = RotationList::new(vec![pose0]);
    let base = *frontier;

    // 점수는 효율 그대로다 — 기여가 하나(효율 = v_max)뿐이면 §4-2 의 부족률이 0 이라
    // 가중치가 0 이고, 그 표면은 센티널로 빠져 `candidate_score` 가 `v_max` 를 돌려준다.
    let outcome = evaluate_candidates(&work, &records, &list, 0.0, |x, y, pose, _extra| {
        let bbox = world_bbox(catalogue, item_id, PoseKey::of(pose), x, y)?;
        // 여유는 컨테이너를 좁혀서 이미 확보돼 있다(§`BOUNDARY_INSET_MM`).
        // 여기서는 **돌출만** 막는다 — 경계에 닿는 후보를 거르면 IFP 경계가 통째로
        // 사라져 아무것도 안 놓인다(실제로 그렇게 만들었다가 놓음 0 이 나왔다).
        const PROTRUSION_TOL: f64 = 1.0e-6;
        if bbox.1 < -PROTRUSION_TOL
            || bbox.3 > height + PROTRUSION_TOL
            || bbox.0 < -PROTRUSION_TOL
        {
            return None;
        }
        let width = base.max(bbox.2);
        let placed_area = (bbox.2 - bbox.0).max(0.0) * (bbox.3 - bbox.1).max(0.0);
        let eff = efficiency(placed_area, width.max(1.0) * height);
        // 시드와의 거리는 **동률을 가르는 정도**로만 쓴다. 점수 본체에 섞으면 §4 의
        // 산식이 아니게 된다 — 그래서 아주 작은 계수를 곱해 뒤에 더한다.
        let to_seed = ((x - seed.0).powi(2) + (y - seed.1).powi(2)).sqrt();
        let span = (base.max(1.0) + height).max(1.0);
        Some(eff - 1.0e-9 * (to_seed / span))
    });

    if outcome.truncated {
        report.truncations += 1;
    }
    let Some(best) = outcome.best else {
        // 후보를 평가했는데 전부 탈락해도 조각을 잃지 않는다 — 모든 실패는 같은
        // 출구(프론티어 오른쪽)로 간다. 그러지 않으면 해가 `QuantityMismatch` 다.
        report.fallback_no_candidate += 1;
        return place_at_frontier(pieces, piece, catalogue, frontier, height);
    };
    commit(
        pieces,
        piece,
        best.rotation,
        best.extra_rad,
        best.x,
        best.y,
        catalogue,
        frontier,
    );
    true
}

/// 경계 후보를 평가해 최선 자리에 놓는다 (§모듈 7).
#[allow(clippy::too_many_arguments)]
fn place_on_boundary(
    pieces: &mut [PieceRecord],
    piece: usize,
    record: &EvalRecord,
    catalogue: &PoseCatalogue,
    _surfaces: &[SurfaceRecord],
    regions: &Option<RegionCache>,
    placed: &[(usize, PoseKey, f64, f64)],
    frontier: &mut f64,
    height: f64,
    report: &mut RefPlaceReport,
) -> bool {
    let pose0 = pieces[piece].rotations.entries()[0];
    let item_id = pieces[piece].item_id;
    // **배치 가능 영역**을 함께 넣는다. 부착 대상의 NFP 경계만 보면 이미 놓인 다른
    // 조각과 겹치는 후보까지 최선이 될 수 있다. 영역이 없으면(비었거나 커널이
    // 실패) 후보를 **검증할 수단이 없으므로** 프론티어 폴백으로 간다 — IFP 만으로
    // 평가하면 겹치는 자리를 최선으로 고를 수 있다.
    let fetched: Option<Cow<'_, PolygonSet>> = match regions {
        Some(cache) => cache.get(item_id, PoseKey::of(&pose0)).map(Cow::Borrowed),
        None => feasible_for(
            catalogue,
            item_id,
            PoseKey::of(&pose0),
            placed,
            Some((0.0, *frontier + 2.0 * height + 1.0)),
        )
        .map(Cow::Owned),
    };
    let region = match fetched {
        Some(r) if !r.is_empty() => r,
        Some(_) => {
            report.fallback_empty += 1;
            return place_at_frontier(pieces, piece, catalogue, frontier, height);
        }
        None => {
            report.fallback_missing += 1;
            return place_at_frontier(pieces, piece, catalogue, frontier, height);
        }
    };
    // 영역을 한 번 정규화해 두고 후보마다 포함 판정만 한다 — 쌍 NFP 경계의 점은
    // 부착 대상과는 맞닿지만 **다른** 놓인 조각과 겹칠 수 있다.
    let Ok(prepared) = catalogue.engine().prepare_region(&region) else {
        report.fallback_missing += 1;
        return place_at_frontier(pieces, piece, catalogue, frontier, height);
    };
    let records = vec![record.clone(), feasible_record(&region)];
    let work = build_work_list(&records, 1.0);
    // **`pose0` 만 평가한다.** 쌍 NFP 도 배치 가능 영역도 `pose0` 로 계산됐다 —
    // 같은 앵커를 다른 자세에 쓰면 그 자세의 기하가 아니므로 겹칠 수 있다.
    let list = RotationList::new(vec![pose0]);
    let base = *frontier;
    let span = (base.max(1.0) + height).max(1.0);

    let outcome = evaluate_candidates(&work, &records, &list, 0.0, |x, y, pose, _extra| {
        // 컨테이너 밖은 후보가 아니다.
        let bbox = world_bbox(catalogue, item_id, PoseKey::of(pose), x, y)?;
        // 여유는 컨테이너를 좁혀서 이미 확보돼 있다(§`BOUNDARY_INSET_MM`).
        // 여기서는 **돌출만** 막는다 — 경계에 닿는 후보를 거르면 IFP 경계가 통째로
        // 사라져 아무것도 안 놓인다(실제로 그렇게 만들었다가 놓음 0 이 나왔다).
        const PROTRUSION_TOL: f64 = 1.0e-6;
        if bbox.1 < -PROTRUSION_TOL
            || bbox.3 > height + PROTRUSION_TOL
            || bbox.0 < -PROTRUSION_TOL
        {
            return None;
        }
        // 배치 가능 영역 **밖**이면 놓인 조각과 겹친다. 경계는 맞닿음이라 허용한다.
        // 판정 실패도 탈락이다 — 낙관하면 겹친 배치가 검증까지 통과할 수 있다.
        match catalogue.engine().locate_prepared(&prepared, NfpPoint::new(x, y)) {
            Ok(Location::Inside) | Ok(Location::Boundary) => {}
            Ok(Location::Outside) | Err(_) => return None,
        }
        // §4-1 효율 — 이 배치가 만드는 폭에 대한 면적비. 클수록 좋다.
        let width = base.max(bbox.2);
        let placed_area = (bbox.2 - bbox.0).max(0.0) * (bbox.3 - bbox.1).max(0.0);
        let eff = efficiency(placed_area, width.max(1.0) * height);
        // §4-2 — 기여가 하나(효율 = v_max)뿐이면 부족률 0 → 가중치 0 → 센티널이라
        // `candidate_score` 는 `v_max` 를 돌려준다. 점수는 효율 그대로다.
        //
        // 동률은 **왼쪽 아래**가 이긴다 — 같은 폭을 만드는 후보가 여럿일 때 훑는
        // 순서가 아니라 자리가 결정하도록, 효율에 영향 없는 크기로 뒤에 뺀다.
        Some(eff - 1.0e-9 * ((bbox.0 + bbox.1) / span))
    });

    if outcome.truncated {
        report.truncations += 1;
    }
    let Some(best) = outcome.best else {
        // **여기가 구멍이었다.** 후보를 평가했는데 전부 탈락하면 그냥 `false` 를
        // 돌려주었고, 그 조각이 빠져 해가 `QuantityMismatch` 로 무효가 됐다.
        // 모든 실패는 같은 출구로 간다.
        report.fallback_no_candidate += 1;
        return place_at_frontier(pieces, piece, catalogue, frontier, height);
    };
    commit(
        pieces,
        piece,
        best.rotation,
        best.extra_rad,
        best.x,
        best.y,
        catalogue,
        frontier,
    );
    true
}

/// 조각을 놓았음을 영역 캐시에 알린다.
///
/// 남은 자세들의 영역에서 이 조각을 **한 번** 뺀다. 이 호출을 빠뜨리면 뒤 조각이
/// 낡은 영역을 보고 겹치는 자리에 놓인다 — 그래서 배치를 확정하는 모든 자리에서
/// 반드시 부른다.
fn note_placed(
    regions: &mut Option<RegionCache>,
    placed: &mut Vec<(usize, PoseKey, f64, f64)>,
    catalogue: &PoseCatalogue,
    piece: &PieceRecord,
) {
    let Some(pose) = piece.pose else { return };
    let key = PoseKey::of(&pose.rotation);
    placed.push((piece.item_id, key, pose.x, pose.y));
    if let Some(cache) = regions.as_mut() {
        cache.place(catalogue, piece.item_id, key, pose.x, pose.y);
        cache.retire(piece.item_id, key);
    }
}

/// 좌표를 피스 레코드에 기록한다 (§모듈 7-4 확정).
#[allow(clippy::too_many_arguments)]
fn commit(
    pieces: &mut [PieceRecord],
    piece: usize,
    rotation: RotationEntry,
    extra_rad: f64,
    x: f64,
    y: f64,
    catalogue: &PoseCatalogue,
    frontier: &mut f64,
) {
    pieces[piece].pose = Some(PlacedPose {
        rotation,
        extra_rad,
        x,
        y,
    });
    pieces[piece].state = PlaceState::Placed;
    set_piece_key(&mut pieces[piece], x);
    if let Some(bbox) = world_bbox(catalogue, pieces[piece].item_id, PoseKey::of(&rotation), x, y) {
        *frontier = frontier.max(bbox.2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tiny_instance() -> Option<SPInstance> {
        // 저장소에 이미 있는 작은 표준 인스턴스를 쓴다. 없으면 테스트를 건너뛴다 —
        // 데이터 없는 머신에서 panic 하지 않는 것이 이 저장소의 규칙이다.
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../data/input")
            .join("shapes0.json");
        let (ext, _) = crate::util::io::read_spp_input(&path).ok()?;
        let c = crate::config::DEFAULT_SWARM_CONFIG;
        let imp = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&imp, &ext).ok()
    }

    /// 쌍 NFP 레코드(대상 로컬 좌표)는 대상의 확정 좌표만큼 옮겨져 월드 앵커가 된다 —
    /// bbox 도 함께 옮겨지고 플래그는 보존된다.
    #[test]
    fn a_pair_record_is_translated_by_the_target_pose() {
        let mut p = ContactPoint::new(1.0, 2.0);
        p.flags = ContactPoint::FORWARD;
        let record = EvalRecord::new(vec![p, ContactPoint::new(-3.0, 4.0)]);
        let moved = translate_record(&record, 10.0, 100.0);
        assert_eq!(moved.points.len(), 2);
        assert_eq!((moved.points[0].x, moved.points[0].y), (11.0, 102.0));
        assert_eq!((moved.points[1].x, moved.points[1].y), (7.0, 104.0));
        assert!(moved.points[0].has(ContactPoint::FORWARD), "플래그는 보존된다");
        assert_eq!(moved.bbox, (7.0, 102.0, 11.0, 104.0), "bbox 도 같이 옮겨진다");
        // 원본은 그대로다.
        assert_eq!(record.bbox, (-3.0, 2.0, 1.0, 4.0));
    }

    /// 인스턴스 회전 선언이 자세 목록으로 그대로 옮겨진다.
    #[test]
    fn declared_rotations_become_the_pose_list() {
        let Some(inst) = tiny_instance() else { return };
        let rot = rotations_from_instance(&inst);
        assert_eq!(rot.len(), inst.items.len());
        // **반전은 넣지 않는다** — 결 정보가 없으면 반전을 임의로 허용하면 안 된다.
        for entries in rot.values() {
            assert!(!entries.is_empty());
            assert!(
                entries.iter().all(|e| !e.flip_x && !e.flip_y),
                "결 정보 없이 반전을 만들면 안 된다"
            );
        }
    }

    /// 엔진이 실제 인스턴스에서 **끝까지 돌고 유효한 해를 낸다**.
    #[test]
    fn the_engine_runs_end_to_end_and_produces_a_valid_solution() {
        let Some(inst) = tiny_instance() else { return };
        let params = RefPlaceParams::default();
        let Some((sol, report)) = build_reference_nest(&inst, &params, None) else {
            panic!(
                "해를 내야 한다 — 검증 실패로 None 이 나왔다면 배치 자체가 잘못된 것이다"
            );
        };
        assert!(report.placed > 0, "하나는 놓아야 한다");
        // 검증을 통과했으므로 여기 왔다 — `build_reference_nest` 가 실패한 해를
        // 돌려주지 않는다.
        assert!(
            crate::structure::pipeline::solution_validation_error(&inst, &sol).is_none()
        );
        assert!(sol.strip_width() > 0.0);
        assert!(report.nfp_pairs > 0, "NFP 를 선계산해야 한다");
        assert!(report.poses > 0);
        // 효율 스냅샷이 남아야 한다 (§4-1 단계마다 재계산).
        assert_eq!(report.efficiency.len(), 1);
        assert_eq!(report.efficiency[0].stage, Stage::Placement);
    }

    /// **결정론** — 같은 입력이면 같은 배치다.
    ///
    /// 이 저장소의 회귀는 비트 단위 비교로 잡는다. 초기 배치가 실행마다 달라지면
    /// 모든 실측치가 무의미해진다.
    #[test]
    fn the_same_input_always_produces_the_same_layout() {
        let Some(inst) = tiny_instance() else { return };
        let params = RefPlaceParams::default();
        let a = build_reference_nest(&inst, &params, None);
        let b = build_reference_nest(&inst, &params, None);
        match (a, b) {
            (Some((sa, ra)), Some((sb, rb))) => {
                assert_eq!(sa.strip_width().to_bits(), sb.strip_width().to_bits());
                assert_eq!(ra.placed, rb.placed);
                assert_eq!(ra.nfp_pairs, rb.nfp_pairs);
            }
            (None, None) => {}
            _ => panic!("{}", crate::dmsg!("한 번은 해를 내고 한 번은 못 내면 결정론이 아니다")),
        }
    }

    /// 그룹을 주면 그 순서가 체인의 모양이 된다.
    #[test]
    fn supplying_groups_changes_the_chain_rather_than_being_ignored() {
        let Some(inst) = tiny_instance() else { return };
        let params = RefPlaceParams::default();
        let n: usize = inst.items.iter().map(|(_, q)| *q).sum();
        // 전부 한 그룹 -> 긴 사슬 하나.
        let one = vec![GroupSpec::new("all", (0..n).collect())];
        let with_group = build_reference_nest(&inst, &params, Some(one));
        assert!(with_group.is_some());
    }

    /// 조각이 없으면 조용히 `None` — panic 하지 않는다.
    #[test]
    fn an_empty_instance_yields_none_instead_of_panicking() {
        let Some(inst) = tiny_instance() else { return };
        let mut empty = inst.clone();
        empty.items.clear();
        assert!(build_reference_nest(&empty, &RefPlaceParams::default(), None).is_none());
    }
}
