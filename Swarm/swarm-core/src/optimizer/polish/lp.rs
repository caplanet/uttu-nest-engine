use crate::config::LpCompactionConfig;
use crate::lp_solver::{self, LpProblem, LpRow, LpSolverConfig};
use crate::optimizer::polish::probe::translated;
use crate::optimizer::separator::Separator;
use crate::util::terminator::Terminator;
use itertools::Itertools;
use jagua_rs::entities::PItemKey;
use jagua_rs::geometry::geo_traits::DistanceTo;
use jagua_rs::geometry::primitives::{Edge, Point, Rect, SPolygon};
use ordered_float::OrderedFloat;
use rayon::prelude::*;

/// LP에 참여하는 아이템 하나에 대한 정보.
///
/// Every rigid member keeps its own geometry but shares its representative's
/// translation variables. Ungrouped placements remain independent, even when
/// several copies have the same item id.
struct LpItem {
    pk: PItemKey,
    group_representative: Option<PItemKey>,
    shape: std::sync::Arc<SPolygon>,
    bbox: Rect,
    /// 고정(frozen) 아이템은 이동할 수 없다.
    frozen: bool,
}

impl LpItem {
    fn unit_key(&self) -> PItemKey {
        self.group_representative.unwrap_or(self.pk)
    }
}

/// One deterministic variable layout shared by model construction and decoding.
struct LpVariables {
    slot_of: Vec<usize>,
    representatives: Vec<PItemKey>,
}

impl LpVariables {
    fn new(items: &[LpItem]) -> Self {
        let mut slots = std::collections::HashMap::new();
        let mut representatives = Vec::new();
        let slot_of = items
            .iter()
            .map(|it| {
                if it.frozen {
                    return usize::MAX;
                }
                *slots.entry(it.unit_key()).or_insert_with(|| {
                    let slot = 1 + 2 * representatives.len();
                    representatives.push(it.unit_key());
                    slot
                })
            })
            .collect();
        Self {
            slot_of,
            representatives,
        }
    }
}

/// 정점-모서리(또는 정점-꼭짓점) 접촉 제약 하나.
#[derive(Debug)]
struct ContactRow {
    /// 정점을 가진 아이템 (LpItem 인덱스)
    a: usize,
    /// 모서리를 가진 아이템 (LpItem 인덱스)
    b: usize,
    /// 단위 법선 (`b` 의 경계에서 정점 쪽, 즉 바깥을 향한다)
    n: (f32, f32),
    /// 정점에서 `b` 의 그 모서리(선분)까지의 거리 (>= 0)
    dist: f32,
}

/// LP compaction 패스.
///
/// 아이템별 평행이동 (dx, dy)과 strip 폭 W를 변수로 하는 선형계획을 세우고 W를 최소화한다.
/// 비겹침 조건은 **정점-모서리 분리 제약**으로 선형화한다: 아이템 A의 정점 v가 아이템 B의
/// 모서리 e 를 넘지 않도록 `n·(d_a - d_b) >= -dist` 를 건다. 법선과 거리는 v 에서 가장
/// 가까운 **선분 위의 점** 기준이다([`contact_row`] 참조).
///
/// 두 도형이 평행이동만으로 떨어진 상태에서 겹치는 상태로 넘어가려면 그 전환 순간에
/// 반드시 한쪽 정점이 다른 쪽 모서리 위에 있어야 한다. 따라서 상대 이동 한계(2Δ) 이내의
/// 모든 정점-모서리 쌍에 제약을 걸면 선형화가 보수적으로 성립한다. 다만 쌍당 제약 수를
/// 제한하면(성능상 필요) 이 완전성이 깨질 수 있으므로, **LP 해를 적용한 뒤에는 항상 CDE로
/// 전수 검증하고 실패하면 스텝을 절반씩 줄여 재시도**한다. 그래서 어떤 경우에도 입력 해의
/// 실행 가능성을 깨뜨리지 않는다.
///
/// 막혔을 때 신뢰 영역을 키우는 배율.
const TRUST_GROWTH: f32 = 2.0;
/// 신뢰 영역을 몇 단계까지 키워볼지.
///
/// 5 → 3 (2026-10-01). 접촉 행을 선분 기준으로 고친 뒤에는 L0 가 수렴한 자리에서 더 큰
/// 단계가 얻는 것이 없다 — 수렴 시점의 이동량은 신뢰 영역보다 훨씬 작고, 단계를 키워
/// 추가되는 행은 89mm 넘게 떨어진 쌍이라 그 이동량에서는 걸리지 않는다. 실측(wsjk156,
/// 폭 6337.6 → 6301.35)에서 L3·L4 는 채택 0회에 패스당 약 7.6초(2.3s + 5.3s 시간 초과)를
/// 썼고, L2 의 채택은 0.018mm 한 번이었다. 그 꼬리가 라운드마다 반복되어 폴리시가 예산을
/// 다 쓰고 잔여 예산 회수(reclaim)가 돌지 못했다.
const N_TRUST_LEVELS: u32 = 3;
/// 신뢰 영역을 몇 단계까지 **줄여**볼지 (`level` 의 하한).
///
/// 모델을 못 푼 경우의 대응이다. `2^-3` 이면 기본 비율의 1/8 이고,
/// clearance 도 같은 비율로 줄어 접촉 행 수가 급감한다.
const MIN_TRUST_LEVEL: i32 = -3;
/// 채택 없이 연속으로 돌 수 있는 최대 반복 수 — 진동 방지용 절대 상한.
///
/// 키우기(3) + 줄이기(3) 를 다 소진해도 이 값에 먼저 닿지 않는다. 즉 실패가 없는
/// 실행에서는 이 상한이 **작동하지 않으며**, 기존 동작이 그대로 보존된다.
const MAX_STALL: usize = 9;

/// LP 반복 한 번의 결과. **각 갈래는 서로 다른 대응을 요구한다.**
///
/// # 왜 나눴나
///
/// 예전에는 전부 `Option<f32>` 의 `None` 이었다. 그래서 "수치적으로 못 풀었다" 와
/// "이 스텝 크기로는 개선 여지가 없다" 가 구분되지 않았고, 양쪽 모두에 신뢰 영역을
/// **키우는** 대응이 나갔다.
///
/// 그게 왜 나쁜가: `clearance = trust x 2.83` 이라 신뢰 영역을 키우면 접촉 쌍 판정
/// 반경이 같이 커지고, 접촉 행이 폭발한다. 그 행들은 상당수가 법선이 같고 우변만
/// 다른 선형종속 행이라 단체법 기저가 퇴화한다. 즉 **수치 실패에 대한 대응이 하필
/// 수치 실패를 악화시키는 방향**이었다.
///
/// 실측(wsjk156, 폭 6379.767): `4 steps applied / solver_fail 5` 로 패스가 40회
/// 예산 중 9회쯤에서 조기 종료됐다. level 4 면 clearance 가 1,444mm — 스트립 길이의
/// 1/4 이라 거의 모든 쌍이 접촉 쌍이 된다.
#[derive(Debug, Clone, Copy, PartialEq)]
enum IterOutcome {
    /// 폭이 줄어 채택됐다. 값은 **직전** 폭.
    Applied(f32),
    /// 모델은 제대로 풀렸는데 이 스텝으로는 더 못 줄인다. -> 신뢰 영역을 키운다.
    NoImprovement,
    /// No effective movable unit exists; independent of solver availability.
    NoMovable,
    /// 모델을 풀지 못했다(기저 퇴화·시간 초과·행 수 초과). -> 신뢰 영역을 **줄인다**.
    ///
    /// 같은 모델을 다시 풀면 결정적으로 같은 실패가 나오므로, 재시도하려면
    /// 모델 자체가 달라져야 한다. 신뢰 영역을 줄이면 clearance 가 줄어
    /// 행 수와 퇴화가 함께 줄어든다 — 실제로 다른 모델이 된다.
    Intractable,
    /// 모델은 풀렸고 해가 신뢰 영역 안쪽인데, 어떤 λ 로도 적용하지 못했다.
    /// 키워도 같은 해가 나온다([`step_is_interior`]). -> 패스를 끝낸다.
    Blocked,
    /// 더 돌 이유가 없다(중단 요청, 백엔드 부재 등). -> 패스를 끝낸다.
    Abort,
}

/// 반복 스케줄 상태 — 신뢰 영역 단계와 연속 무채택 횟수.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Schedule {
    /// 신뢰 영역 지수. 실제 비율은 `base_ratio * 2^level` 이다.
    level: i32,
    /// 채택 없이 연속으로 돈 횟수.
    stall: usize,
    /// 이 패스에서 **풀지 못한** 가장 낮은 단계. 이 단계 이상으로는 다시 키우지 않는다.
    ///
    /// 기억이 없으면 `L2 무개선 → L3 시간 초과 → L2 무개선 → L3 시간 초과 …` 가
    /// [`MAX_STALL`] 에 닿을 때까지 되풀이된다. 못 푼 단계는 배치가 조금 바뀌어도
    /// 행 수가 거의 같아 또 못 푼다 — 실측(wsjk156, 폭 6337.6, Polish 60s)에서 첫
    /// 패스 16회 풀이 중 마지막 7회(약 28초, 5초 초과 3회)가 이 꼬리였다.
    ceiling: i32,
    /// 마지막 채택 이후 무개선으로 끝난 가장 높은 단계 (없으면 `i32::MIN`).
    /// 줄여서 내려간 단계가 이미 무개선이었다면 다시 풀어도 같은 답이다.
    exhausted: i32,
}

impl Schedule {
    const START: Self = Self {
        level: 0,
        stall: 0,
        ceiling: N_TRUST_LEVELS as i32,
        exhausted: i32::MIN,
    };

    /// 한 번의 결과를 받아 다음 상태를 정한다. `None` 이면 패스를 끝낸다.
    ///
    /// **순수 함수로 뺀 이유**: 이 전이 규칙이 이 모듈에서 가장 틀리기 쉬운 부분인데,
    /// 루프 안에 인라인으로 두면 LP 를 실제로 풀지 않고서는 검증할 수 없다.
    /// 여기서는 표로 단정할 수 있다([`tests`] 의 `schedule_*`).
    fn advance(self, outcome: IterOutcome) -> Option<Self> {
        let remember = !legacy_schedule();
        match outcome {
            // 개선됨 → 다시 가장 촘촘한 단계부터. 단, 못 푼 단계 아래에서 시작한다.
            IterOutcome::Applied(_) => Some(Self {
                level: if remember {
                    self.ceiling.saturating_sub(1).min(0)
                } else {
                    0
                },
                stall: 0,
                ceiling: self.ceiling,
                exhausted: i32::MIN,
            }),
            IterOutcome::NoImprovement => {
                let next = Self {
                    level: self.level + 1,
                    stall: self.stall + 1,
                    ceiling: self.ceiling,
                    exhausted: self.exhausted.max(self.level),
                };
                // 키우기 상한. `stall` 상한은 진동 방지용 절대 보루라 보통 여기서
                // 먼저 걸린다(무개선만 이어지면 `N_TRUST_LEVELS` 회에 끝난다).
                // 못 푼 단계(`ceiling`)에 닿아도 끝낸다 — 다시 풀어도 또 못 푼다.
                if next.level >= N_TRUST_LEVELS as i32
                    || next.stall >= MAX_STALL
                    || (remember && next.level >= self.ceiling)
                {
                    None
                } else {
                    Some(next)
                }
            }
            IterOutcome::Intractable => {
                // **키우지 않는다.** 키우면 clearance 가 커져 행이 더 늘고 더 잘 실패한다.
                // 같은 모델을 다시 풀면 결정적으로 같은 실패이므로, 재시도가 의미를
                // 가지려면 모델 자체가 달라져야 한다 — 줄이면 실제로 다른 문제가 된다.
                if self.level <= MIN_TRUST_LEVEL || self.stall + 1 >= MAX_STALL {
                    return None;
                }
                let next = Self {
                    level: self.level - 1,
                    stall: self.stall + 1,
                    ceiling: self.ceiling.min(self.level),
                    exhausted: self.exhausted,
                };
                // 내려갈 단계가 이미 무개선이었다 → 더 볼 것이 없다.
                if remember && next.level <= self.exhausted {
                    None
                } else {
                    Some(next)
                }
            }
            IterOutcome::Blocked | IterOutcome::NoMovable | IterOutcome::Abort => None,
        }
    }
}

/// I-04: clearance 배율의 안전 하한 = 2√2.
/// dx, dy 가 각각 ±trust 박스 경계이므로 두 아이템의 상대 변위는 최대 2√2·trust 다.
/// 제약 생성 반경이 이보다 좁으면 (clearance, 2√2·trust] 거리의 쌍이 무제약이 되어
/// 대각 이동으로 겹칠 수 있고, 백트래킹 실패(λ 반감 → 이득 상실)가 늘어난다.
const MIN_CLEARANCE_FACTOR: f32 = 2.8285;

/// I-05: 접촉 제약이 남겨야 하는 최소 간격(mm, 모델 단위).
/// 마진 없이 등식 접촉까지 밀면 f64 해 → f32 적용 반올림으로 미세 관입이 생겨
/// λ 반감 백트래킹이 전멸할 수 있다 (`initial/compact.rs` 의 `LP_MARGIN` 과 동일 원리).
/// 이미 이 값보다 가까운 쌍은 벌어질 자리가 있을 때만 이 값까지 되돌린다
/// ([`MARGIN_SLACK_COST`]). 고정 벽과의 간격은 유지만 요구한다([`contact_rhs`]).
const LP_CONTACT_MARGIN: f32 = 0.05;
/// I-05: 기존 간격 대비 남겨야 하는 최소 비율 (간격의 90% 이내로만 축소 허용).
const LP_KEEP_RATIO: f32 = 0.1;

/// 반환값: 실제로 적용된 LP 반복 횟수
///
/// 최적의 신뢰 영역 크기는 인스턴스마다 다르다. 작은 스텝은 선형화가 정확해 더 촘촘히
/// 밀어 넣지만 얕은 지역해에 갇히고, 큰 스텝은 배치를 크게 재배열하는 대신 근사 오차가 커진다.
/// 그래서 **가장 촘촘한 단계에서 시작해, 막힐 때마다 신뢰 영역을 두 배로 키우고,
/// 한 번이라도 개선되면 다시 가장 촘촘한 단계로 돌아가는** 다단계 스케줄을 쓴다.
pub fn lp_compaction_pass(
    sep: &mut Separator,
    cfg: &LpCompactionConfig,
    term: &impl Terminator,
) -> usize {
    lp_compaction_pass_with_solver(sep, cfg, &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG, term)
}

/// [`lp_compaction_pass`]에 솔버 선택을 얹은 진입점.
///
/// # 경로는 하나다
///
/// 한때 여기에는 갈래가 둘 있었다. 기본 설정(microlp 단체법 + 순수 LP)은 microlp
/// API 로 모델을 직접 세우는 인라인 코드를 탔고, 다른 백엔드나 QP 만 [`LpProblem`]
/// 으로 옮겨 담았다. 새 표현이 검증되기 전까지 실측이 쌓인 기본 경로를 건드리지
/// 않으려던 것이다.
///
/// 그 대가는 **같은 모델을 두 벌 유지하는 것**이었다 — 한쪽만 고치면 조용히 어긋나고,
/// 어긋난 것을 알아챌 방법도 없었다. 새 표현은 `tests/lp_backends.rs` 5종(microlp·
/// clarabel 양쪽에서 압축 성공 + CDE 검증 통과)으로 검증됐고, 합치기 직전에 두 경로가
/// **같은 제약 집합·같은 해**를 내는 것을 `tests::both_paths_agree_on_the_same_model`
/// 과 `tests::both_paths_reach_the_same_objective_and_the_same_step` 으로 확인했다.
/// 그래서 인라인 경로를 지웠다.
///
/// 옛 인라인 모델은 사라지지 않고 그 두 테스트 안에 **손으로 옮겨 적은 한 벌**로
/// 남아 있다. [`build_generic_model`] 을 고치면 거기서 먼저 걸린다.
pub fn lp_compaction_pass_with_solver(
    sep: &mut Separator,
    cfg: &LpCompactionConfig,
    solver: &LpSolverConfig,
    term: &impl Terminator,
) -> usize {
    lp_compaction_pass_report(sep, cfg, solver, term).applied
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PassExit {
    ScheduleExhausted,
    Blocked,
    NoMovable,
    Incomplete,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct PassReport {
    pub applied: usize,
    pub exit: PassExit,
}

impl PassReport {
    pub fn proves_completion(self, solver: &LpSolverConfig) -> bool {
        self.exit == PassExit::NoMovable
            || (solver.backend.is_always_available()
                && matches!(self.exit, PassExit::ScheduleExhausted | PassExit::Blocked))
    }
}

pub(super) fn lp_compaction_pass_report(
    sep: &mut Separator,
    cfg: &LpCompactionConfig,
    solver: &LpSolverConfig,
    term: &impl Terminator,
) -> PassReport {
    let base_ratio = cfg.trust_region_ratio.max(f32::EPSILON);
    run_pass(cfg.n_iterations, term, |level| {
        let ratio = base_ratio * TRUST_GROWTH.powi(level);
        let trust = sep.prob.strip_width() * ratio;
        lp_iteration(sep, cfg, solver, trust, term)
    })
}

// Share the real schedule with outcome tests. A count of zero alone never
// proves completion: limits, cancellation and failed models retain retries.
fn run_pass(
    n_iterations: usize,
    term: &impl Terminator,
    mut iteration: impl FnMut(i32) -> IterOutcome,
) -> PassReport {
    let mut report = PassReport {
        applied: 0,
        exit: PassExit::Incomplete,
    };
    let mut sched = Schedule::START;
    let mut incomplete = false;
    for _ in 0..n_iterations.max(1) {
        if term.kill() {
            return report;
        }
        let outcome = iteration(sched.level);
        if matches!(outcome, IterOutcome::Applied(_)) {
            report.applied += 1;
        }
        incomplete |= matches!(outcome, IterOutcome::Intractable | IterOutcome::Abort);
        match sched.advance(outcome) {
            Some(next) => sched = next,
            None => {
                // A killed backtrack may otherwise look like Blocked. Any
                // earlier Intractable also invalidates a lowered ceiling.
                if !incomplete && !term.kill() {
                    report.exit = match outcome {
                        IterOutcome::NoMovable => PassExit::NoMovable,
                        IterOutcome::NoImprovement => PassExit::ScheduleExhausted,
                        IterOutcome::Blocked => PassExit::Blocked,
                        _ => PassExit::Incomplete,
                    };
                }
                return report;
            }
        }
    }
    report // Configured iteration cap, with an unexhausted schedule.
}

/// LP 1회: 모델 수립 → 풀이 → 백트래킹 적용.
///
/// 반환값은 [`IterOutcome`] 이다. **호출부가 대응을 고르려면 "못 줄였다" 와
/// "못 풀었다" 가 구분돼야 한다** — 전자는 신뢰 영역을 키우고, 후자는 줄인다.
fn lp_iteration(
    sep: &mut Separator,
    cfg: &LpCompactionConfig,
    solver: &LpSolverConfig,
    trust: f32,
    term: &impl Terminator,
) -> IterOutcome {
    let width_before = sep.prob.strip_width();
    let container = sep.prob.layout().container().outer_cd().bbox();
    // I-04: 하한을 상대 변위 상한(2√2·trust)에 맞춘다.
    let clearance = trust * cfg.clearance_factor.max(MIN_CLEARANCE_FACTOR);

    let items: Vec<LpItem> = sep
        .prob
        .layout()
        .placed_items()
        .iter()
        .map(|(pk, pi)| LpItem {
            pk,
            group_representative: sep.place_groups.group_id_of(pi.item().idx()).map(|_| {
                sep.placement_key(sep.place_groups.representative(pi.item().idx()))
                    .expect(crate::dmsg!("validated rigid group must have a placed representative"))
            }),
            shape: pi.shape().clone(),
            bbox: pi.shape().bbox(),
            frozen: sep.frozen_mask.get(pi.item().idx()).copied().unwrap_or(false)
                || sep.place_groups.members(pi.item().idx()).iter().any(|member| {
                    sep.frozen_mask
                        .get(member.item_id)
                        .copied()
                        .unwrap_or(false)
                }),
        })
        .collect();

    let variables = LpVariables::new(&items);
    let n_movable = variables.representatives.len();
    if n_movable == 0 {
        // 신뢰 영역을 어떻게 바꿔도 달라지지 않는다 — 더 돌지 않는다.
        return IterOutcome::NoMovable;
    }
    // 폭 변수 W 는 **가변 아이템의 우측 끝**이다 — 스트립 폭이 아니다.
    //
    // 스트립은 `우측 끝 x 1.00001 + offset` 으로 맞춰지므로([`fit_strip_with_frozen`]) 스트립
    // 폭을 상한으로 쓰면 모델에 실제로는 없는 공간이 생긴다(wsjk156: 0.063 + 0.2mm). 그러면
    // 아무것도 움직이지 않아도 W 가 그만큼 «줄어» 아래의 무개선 판정이 수렴점에서 걸리지 않고,
    // 우측 끝을 그 공간으로 밀어내는 해는 폭이 늘어 어떤 λ 로도 채택되지 않는다.
    // Frozen poses remain fixed but are permitted outside the physical strip.
    //
    // [`fit_strip_with_frozen`]: crate::util::frozen::fit_strip_with_frozen
    let right_before = if legacy_width_bound() {
        width_before
    } else {
        items
            .iter()
            .filter(|it| !it.frozen)
            .map(|it| it.bbox.x_max)
            .fold(f32::NEG_INFINITY, f32::max)
    };
    let w_lower = (right_before - trust).max(f32::EPSILON);
    if w_lower >= right_before {
        // No positive representable width decrease in this trust region.
        return IterOutcome::NoImprovement;
    }
    // 좌측으로 당기는 항이 폭 목적항을 압도하지 않도록 아이템 수로 정규화한다.
    let pull = (cfg.left_pull_weight / n_movable as f32) as f64;

    // 2. 접촉(비겹침) 제약
    // 이 아래로 제약 생성(O(쌍))·풀이·백트래킹 검증이 이어지는데, 대형 인스턴스에서는
    // 한 번에 수 초가 걸릴 수 있다. 중단 요청이 들어왔다면 시작하기 전에 빠져나간다.
    if term.kill() {
        return IterOutcome::Abort;
    }
    let build_rows = || build_contact_rows(&items, clearance, cfg, term);
    let rows_result = sep
        .thread_pool
        .as_ref()
        .map_or_else(build_rows, |pool| pool.install(build_rows));
    let rows = match rows_result {
        Ok(rows) => rows,
        Err(ContactBuildError::Cancelled) => return IterOutcome::Abort,
        Err(ContactBuildError::Limit) => return IterOutcome::Intractable,
    };
    if rows.len() > cfg.max_constraints {
        // 키우면 clearance 가 커져 행이 **더** 늘어난다. 줄여야 하는 쪽이다.
        return IterOutcome::Intractable;
    }

    // 3. 모델 — 컨테이너 제약과 접촉 제약을 백엔드 중립 표현으로 담는다.
    let model = build_generic_model(
        &items,
        &rows,
        container,
        w_lower,
        right_before,
        trust,
        pull,
        cfg,
        solver,
    );

    // 4. 풀이
    if term.kill() {
        return IterOutcome::Abort;
    }
    let Some(backend) = lp_solver::backend_for(solver) else {
        // 폴백이 꺼져 있고 백엔드를 쓸 수 없다. 신뢰 영역을 어떻게 바꿔도 백엔드는
        // 생기지 않으므로, 남은 반복을 태우지 않고 패스를 끝낸다.
        return IterOutcome::Abort;
    };
    // Pass the remaining phase/slice budget to every backend. Microlp also
    // polls cancellation between resumable slices; external backends use their deadline.
    //
    // **합치면서 바뀐 유일한 동작이 여기다.** 옛 인라인 경로는 Terminator 데드라인이
    // 있을 때만 제한을 걸었고, 없으면 단체법을 무제한으로 돌렸다. 이제는 데드라인이
    // 없어도 [`LpSolverConfig::time_limit_ms`](기본 5s)가 한 번의 풀이를 자른다 —
    // 그 필드가 원래 약속하던 계약이고, 기본 경로만 계약 밖에 있을 이유가 없다.
    //
    // 실측(wsjk156 156-패널, NFP-LBF ordering 3 / 탐침 1024 시드 배치 위에서
    // `DEFAULT_POLISH_CONFIG.lp` 압축 1회):
    //
    // ```text
    // 제한 없음(옛 동작)   폭 7583.8721 -> 7581.3110,  11 스텝,  43.3s
    // 제한 5s (지금)       폭 7583.8721 -> 7581.3115,  11 스텝,  27.2s
    // ```
    //
    // 5s 를 넘긴 풀이가 버려지면서 폭이 0.0005mm(상대 6.6e-8) 손해였고 패스는
    // 16초 짧아졌다. 옛 동작이 필요하면 `time_limit_ms` 를 키우면 그대로 돌아온다
    // (같은 하네스에서 600s 로 두면 7581.3110 이 정확히 재현된다).
    let limit = match term.timeout_at() {
        Some(deadline) => deadline
            .checked_duration_since(jagua_rs::Instant::now())
            .unwrap_or_default()
            .min(solver.time_limit()),
        None => solver.time_limit(),
    };
    if limit.is_zero() {
        return IterOutcome::Abort;
    }
    let sol = backend.solve_with_control(&model, limit, &|| term.kill());
    if !sol.status.is_usable() {
        // 시간 제한이든 진짜 실패든 **대응은 같다**: 둘 다 "이 모델을 못 풀었다" 이고,
        // 모델을 줄이면 행 수와 퇴화가 함께 줄어 양쪽 모두 나아진다.
        return IterOutcome::Intractable;
    }

    // 폭 기준의 상대 오차 (f32::EPSILON은 폭 크기에 비해 의미가 없다)
    let w_eps = width_before * 1e-6;
    // 변수 배치: 0 = W, 이후 가변 unit마다 (dx, dy).
    // 이 순서는 `build_generic_model` 이 만든 것과 **반드시 같아야 한다**.
    let Some(&w_value) = sol.x.first() else {
        return IterOutcome::Intractable;
    };
    let mut deltas: Vec<(PItemKey, (f32, f32))> = Vec::with_capacity(n_movable);
    let mut slot = 1usize;
    for &pk in &variables.representatives {
        match (sol.x.get(slot), sol.x.get(slot + 1)) {
            (Some(&dx), Some(&dy)) => deltas.push((pk, (dx as f32, dy as f32))),
            // 백엔드가 짧은 해를 돌려준 경우 — 조용히 포기한다.
            _ => return IterOutcome::Intractable,
        }
        slot += 2;
    }
    // 줄일 폭도 옮길 조각도 없다 — 적용과 인증을 거칠 것 없이 무개선이다.
    //
    // 폭 이득이 없어도 **이동이 있으면 적용한다.** 같은 폭에서의 좌측 정리와 마진 복원은
    // 다음 반복이 줄일 자리를 만든다. 폭 이득만 보고 여기서 돌아가면 저장 배치 10개 중
    // 6개에서 최종 폭이 0.1 ~ 0.47mm 늘었다. 실제 채택 여부는 아래 백트래킹이 CDE 로
    // 검증한 뒤의 폭으로 정한다.
    if w_value as f32 >= right_before - w_eps
        && deltas
            .iter()
            .all(|&(_, (dx, dy))| dx.abs() < w_eps && dy.abs() < w_eps)
    {
        return IterOutcome::NoImprovement;
    }

    // 5. 백트래킹 적용
    // 선형화가 완전하지 않을 수 있으므로 적용 후 반드시 검증한다.
    let saved = sep.prob.save();
    // Restoring the tracker snapshot avoids rebuilding the whole collision
    // tracker on every rejected λ (an O(n²) scan per backtrack).
    let ct_saved = sep.ct.save();
    let mut lambda = 1.0f32;
    for _ in 0..=cfg.max_backtracks {
        // 각 시도는 전체 레이아웃 이동 + CDE 전수 검증이라 비싸다.
        // 루프 머리에서만 확인하면 레이아웃은 항상 `saved` 상태(정상)이므로 안전하게 빠져나간다.
        if term.kill() {
            break;
        }
        if !apply_deltas_with_control(sep, &deltas, lambda, &mut || term.kill()) {
            sep.rollback(&saved, Some(&ct_saved));
            return IterOutcome::Abort;
        }
        if sep.ct.get_total_loss() == 0.0 {
            sep.refit_strip();
            if term.kill() {
                sep.rollback(&saved, Some(&ct_saved));
                return IterOutcome::Abort;
            }
            let after = sep.prob.strip_width();
            // Surrogate loss is only a proposal. Certify original geometry,
            // frozen poses and rigid offsets before retaining this transaction.
            let certified =
                after <= width_before && sep.solution_is_certified(&sep.prob.save(), &saved);
            if certified {
                return if after < width_before - w_eps {
                    IterOutcome::Applied(width_before)
                } else {
                    IterOutcome::NoImprovement
                };
            }
        }
        // 겹침 발생 → 되돌리고 스텝을 절반으로
        sep.rollback(&saved, Some(&ct_saved));
        lambda *= 0.5;
    }
    // 백트래킹을 다 써도 겹침이 남았다. 솔버는 정상이었고 선형화 근사가 어긋난 경우다.
    //
    // 해가 신뢰 영역 안쪽에 있었다면 키워도 같은 최적해가 나온다([`step_is_interior`]) —
    // 같은 스텝을 같은 λ 들로 다시 적용해 같은 실패를 반복할 뿐이므로 패스를 끝낸다.
    if !legacy_blocked() && step_is_interior(&deltas, right_before - w_value as f32, trust) {
        return IterOutcome::Blocked;
    }
    // 신뢰 영역에 걸린 해였다면 **기존대로 키운다.** 이 모듈의 다단계 스케줄은 원래 이
    // 상황에 더 과감한 재배열을 시도하도록 설계돼 있다(모듈 문서 참조) — 얕은 지역해를
    // 벗어나려는 의도적 동작이므로 건드리지 않는다.
    IterOutcome::NoImprovement
}

/// 해가 신뢰 영역에 걸리지 않았는가 — 어느 이동량도, 폭 감소량도 `0.9·trust` 미만인가.
///
/// # 왜 이것이 «키워도 소용없다» 를 뜻하는가
///
/// 신뢰 영역을 키우면 모델에서 달라지는 것은 둘뿐이다. 변수 상자(`±trust`, 폭 하한)가
/// 넓어지고, 판정 반경(`clearance >= 2.8285·trust`)이 커져 **더 먼** 쌍의 행이 추가된다.
///
/// * 추가되는 행은 거리가 옛 반경(`2.8285·trust`)보다 크고 우변이 `-0.9·dist` 다. 상대
///   이동이 `2√2·0.9·trust` 미만인 이 해는 그 행들을 전부 만족한다 — 여기서 0.9 가 나온다.
/// * 기존 행은 그대로이고(쌍당 상한은 가까운 것부터 채운다), 상자는 이 해에서 활성이 아니다.
///
/// 그러므로 이 해는 키운 모델에서도 실행 가능하고, 볼록 문제에서 비활성 제약을 풀어도
/// 최적값은 내려가지 않으므로 여전히 최적이다. 실측(wsjk156, 폭 6280.356, 슬라이드 뒤):
/// L0·L1·L2 가 같은 해(폭 -0.407mm, 최대 이동 2.67mm / 신뢰 영역 31.4mm)를 냈고 λ 반감
/// 6회가 세 번 똑같이 실패했다 — 패스당 `backtracks 18`, 풀이 3회 중 2회가 헛돌았다.
fn step_is_interior(deltas: &[(PItemKey, (f32, f32))], width_drop: f32, trust: f32) -> bool {
    let limit = 0.9 * trust;
    width_drop < limit
        && deltas
            .iter()
            .all(|&(_, (dx, dy))| dx.abs() < limit && dy.abs() < limit)
}

/// 이동량을 `lambda` 비율로 적용한다.
fn apply_deltas_with_control(
    sep: &mut Separator,
    deltas: &[(PItemKey, (f32, f32))],
    lambda: f32,
    should_stop: &mut dyn FnMut() -> bool,
) -> bool {
    let moves: Vec<_> = deltas
        .iter()
        .filter_map(|&(pk, (dx, dy))| {
            if dx == 0.0 && dy == 0.0 {
                return None;
            }
            let dt = sep.prob.layout().placed_items().get(pk)?.d_transf();
            Some((pk, translated(dt, (dx * lambda, dy * lambda), 1.0)))
        })
        .collect();
    sep.move_items_with_control(&moves, should_stop)
}

#[derive(Debug, PartialEq, Eq)]
enum ContactBuildError {
    Cancelled,
    Limit,
}

/// Read-only pairs are evaluated in bounded batches. Indexed collection keeps
/// row order identical across thread counts; only model construction is parallel.
fn build_contact_rows(
    items: &[LpItem],
    clearance: f32,
    cfg: &LpCompactionConfig,
    term: &impl Terminator,
) -> Result<Vec<ContactRow>, ContactBuildError> {
    let order = (0..items.len())
        .sorted_by_key(|&i| OrderedFloat(items[i].bbox.x_min))
        .collect_vec();
    let mut rows = Vec::new();
    let mut pairs = Vec::with_capacity(64);
    for (oi, &i) in order.iter().enumerate() {
        if term.kill() {
            return Err(ContactBuildError::Cancelled);
        }
        for &j in &order[oi + 1..] {
            if term.kill() {
                return Err(ContactBuildError::Cancelled);
            }
            if items[j].bbox.x_min > items[i].bbox.x_max + clearance {
                break;
            }
            if items[i].unit_key() == items[j].unit_key()
                || (items[i].frozen && items[j].frozen)
                || !bboxes_within(items[i].bbox, items[j].bbox, clearance)
            {
                continue;
            }
            pairs.push((i, j));
            if pairs.len() == 64 {
                append_contact_batch(items, &pairs, clearance, cfg, term, &mut rows)?;
                pairs.clear();
            }
        }
    }
    append_contact_batch(items, &pairs, clearance, cfg, term, &mut rows)?;
    Ok(rows)
}

fn append_contact_batch(
    items: &[LpItem],
    pairs: &[(usize, usize)],
    clearance: f32,
    cfg: &LpCompactionConfig,
    term: &impl Terminator,
    rows: &mut Vec<ContactRow>,
) -> Result<(), ContactBuildError> {
    let evaluate = |&(i, j): &(usize, usize)| {
        let mut local = Vec::new();
        if !collect_pair_rows_with_control(items, i, j, clearance, cfg, &mut local, term)
            || !collect_pair_rows_with_control(items, j, i, clearance, cfg, &mut local, term)
        {
            return Err(ContactBuildError::Cancelled);
        }
        Ok(local)
    };
    let work: usize = pairs
        .iter()
        .map(|&(i, j)| {
            items[i]
                .shape
                .n_vertices()
                .saturating_mul(items[j].shape.n_vertices())
        })
        .sum();
    let batch: Vec<Result<Vec<ContactRow>, ContactBuildError>> =
        // Only fan out inside an installed pool; from the global pool the batch
        // would otherwise block the caller thread behind unrelated work.
        if pairs.len() >= 4
            && work >= 16_384
            && rayon::current_num_threads() > 1
            && rayon::current_thread_index().is_some()
        {
            pairs.par_iter().map(evaluate).collect()
        } else {
            pairs.iter().map(evaluate).collect()
        };
    for result in batch {
        let local = result?;
        if local.len() > cfg.max_constraints.saturating_sub(rows.len()) {
            return Err(ContactBuildError::Limit);
        }
        rows.extend(local);
    }
    Ok(())
}

fn bboxes_within(a: Rect, b: Rect, clearance: f32) -> bool {
    a.x_min - clearance <= b.x_max
        && b.x_min - clearance <= a.x_max
        && a.y_min - clearance <= b.y_max
        && b.y_min - clearance <= a.y_max
}

/// 중복 제거 on/off — **측정 전용** 스위치. 테스트 빌드에만 존재한다.
///
/// A/B 를 깨끗하게 하려면 두 팔이 **같은 시작 레이아웃**을 받아야 한다. 시간 예산으로
/// 만든 배치를 팔마다 새로 만들면 속도 차이가 배치 차이로 번져 무엇을 재는지 알 수
/// 없게 된다(이 저장소가 `scale_orderings` 로 한 번 겪은 오염이다). 그래서 한 프로세스
/// 안에서 같은 해를 복원해 두 번 돌릴 수 있도록 스위치를 두었다.
#[cfg(test)]
static DEDUP_ENABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

#[cfg(test)]
fn dedup_enabled() -> bool {
    DEDUP_ENABLED.load(std::sync::atomic::Ordering::Relaxed)
}

/// 생산 빌드에는 분기 자체가 남지 않는다 — 컴파일러가 통째로 지운다.
#[cfg(not(test))]
const fn dedup_enabled() -> bool {
    true
}

/// 옛 동작 재현 스위치 — **측정 전용**, 테스트 빌드에만 존재한다([`dedup_enabled`] 와 같은 이유).
///
/// 비트 0: 접촉 행을 옛 «무한 직선» 방식으로 만든다([`contact_row_line`]).
/// 비트 1: 스케줄이 못 푼 단계를 기억하지 않는다([`Schedule::advance`]).
/// 비트 2: 마진보다 좁은 접촉을 되돌리지 않는다([`MARGIN_SLACK_COST`]).
/// 비트 3: 적용에 실패한 안쪽 해에서도 신뢰 영역을 키운다([`step_is_interior`]).
/// 비트 4: 폭 변수의 기준을 우측 끝이 아니라 스트립 폭으로 둔다([`lp_iteration`]).
#[cfg(test)]
static LEGACY_AB: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0);

#[cfg(test)]
fn legacy_line_rows() -> bool {
    LEGACY_AB.load(std::sync::atomic::Ordering::Relaxed) & 1 != 0
}

#[cfg(not(test))]
const fn legacy_line_rows() -> bool {
    false
}

#[cfg(test)]
fn legacy_schedule() -> bool {
    LEGACY_AB.load(std::sync::atomic::Ordering::Relaxed) & 2 != 0
}

#[cfg(not(test))]
const fn legacy_schedule() -> bool {
    false
}

#[cfg(test)]
fn legacy_margin_rows() -> bool {
    LEGACY_AB.load(std::sync::atomic::Ordering::Relaxed) & 4 != 0
}

#[cfg(not(test))]
const fn legacy_margin_rows() -> bool {
    false
}

#[cfg(test)]
fn legacy_blocked() -> bool {
    LEGACY_AB.load(std::sync::atomic::Ordering::Relaxed) & 8 != 0
}

#[cfg(not(test))]
const fn legacy_blocked() -> bool {
    false
}

#[cfg(test)]
fn legacy_width_bound() -> bool {
    LEGACY_AB.load(std::sync::atomic::Ordering::Relaxed) & 16 != 0
}

#[cfg(not(test))]
const fn legacy_width_bound() -> bool {
    false
}

/// 마진보다 좁은 접촉을 마진까지 되돌리지 **못한** 양(슬랙) 1mm 의 목적 비용. 폭 W 의 계수는 1.0.
///
/// # 무엇을 고치는가
///
/// 간격이 [`LP_CONTACT_MARGIN`] 보다 좁은 접촉의 행은 예전에 우변이 0 이었다 — «더 가까워지지만
/// 마라». 그러면 LP 는 그 접촉을 **간격 그대로 둔 채 모서리를 따라 미끄러지는** 해를 낸다.
/// 간격이 f32 해상도(좌표 5000mm 에서 0.0005mm)보다 좁으면 적용할 때 좌표 반올림만으로
/// 정점이 모서리를 넘고, 스텝 크기와 무관하므로 λ 를 줄여도 똑같이 실패한다.
///
/// 그런 접촉을 만드는 것은 슬라이드 패스와 압축 단계다. 슬라이드는 충돌 직전까지 민다 —
/// 모서리와 거의 나란한 방향으로 밀리면 LP 가 남긴 0.05mm 가 0.0005mm 아래로 닫힌다. 실측
/// (wsjk156): 슬라이드 한 번에 간격 < 0.001mm 인 행이 1 → 11 개가 됐고, 그 뒤 LP 패스는
/// 전부 `0 steps / backtracks 18` 이었다. 압축 결과에 그런 접촉이 있으면 첫 패스부터
/// 스텝이 깎인다(λ 1/2 → 1/16 → 막힘, 폭 6287.23 에서 정지; 복원을 켜면 백트래킹 0 회로 6277.94).
///
/// # 어떻게
///
/// 간격 < 마진인 행을 `n·(d_a - d_b) + s >= 마진 - dist`, `0 <= s <= 마진 - dist` 로 바꾼다.
/// `s` 가 상한이면 예전 행과 같으므로 **현재 배치(d = 0)는 여전히 실행 가능**하다. 벌어질
/// 자리가 있는 접촉은 마진을 되찾고, 양쪽이 막힌 접촉만 슬랙으로 남는다.
///
/// 계수가 1 보다 커야 한다. 폭을 정하는 사슬 위의 접촉은 벌리는 만큼 폭이 늘어나는데, 그
/// 접촉을 벌리지 않고 얻은 폭 이득은 어차피 적용되지 않는다. 1.5 / 4 / 16 은 6개 배치에서
/// 결과가 비트 단위로 같았다(0.5 는 하나에서 0.26mm 나빴다).
///
/// # 실측
///
/// wsjk156, 저장된 배치 10개(Compress 5 · Explore 5)에 같은 시작 배치로 Polish 60s:
/// 복원 없음 대비 9개에서 폭이 줄었다(0.59 ~ 15.65mm, 평균 3.4mm ≈ 0.043%p), 1개는 +0.13mm.
/// Polish 본체 시간은 그대로다(양쪽 모두 5 ~ 9s 안팎에서 스스로 끝난다).
/// 되돌리는 목표를 수치 안전에 필요한 최소치(0.006mm)로만 잡으면 8개에서 이보다 못했다.
///
/// **폭 변수의 기준이 우측 끝이어야 성립한다**([`lp_iteration`]). 스트립 폭이 기준이면 모델에
/// 실제로는 없는 공간이 0.26mm 있어, 계수가 1 보다 큰 슬랙이 우측 끝 조각을 그리로 밀어낸다.
const MARGIN_SLACK_COST: f64 = 4.0;

/// 접촉 제약 `n·(d_a - d_b) >= rhs` 의 우변.
///
/// # 중복 제거의 근거가 여기 있다
///
/// `keep = max(0.1·dist, min(0.05, dist))` 이므로
/// `dist - keep` 는 **dist 에 대해 단조 비감소**다:
///
/// ```text
/// dist <= 0.05        keep = dist        -> dist - keep = 0
/// 0.05 < dist < 0.5   keep = 0.05        -> dist - 0.05      (증가)
/// dist >= 0.5         keep = 0.1·dist    -> 0.9·dist         (증가)
/// ```
///
/// 따라서 우변 `-(dist - keep)` 은 dist 에 대해 **단조 비증가**이고,
/// 법선이 같은 행들 중 **dist 가 가장 작은 것이 가장 강한 제약**이다.
/// 나머지는 그것에 함의되므로 지워도 실행 가능 영역이 변하지 않는다.
/// (`contact_rhs_is_monotone_in_distance` 가 이 성질을 지킨다.)
fn contact_rhs(dist: f32) -> f64 {
    let keep = (dist * LP_KEEP_RATIO).max(LP_CONTACT_MARGIN.min(dist));
    -((dist - keep) as f64)
}

/// 법선 양자화 격자. 성분당 `1e-4` — 각도로 약 0.0057도다.
///
/// 이보다 성기게 잡으면 서로 다른 방향의 제약이 접혀 선형화가 헐거워지고,
/// 촘촘하게 잡으면 정규화 반올림 때문에 같은 직선 모서리에서 나온 행들이
/// 서로 다른 키를 받아 접히지 않는다 — 접으려는 대상이 바로 그것이다.
const NORMAL_QUANT: f32 = 10_000.0;

/// 법선을 격자에 맞춰 정수 키로. 같은 방향이면 같은 키를 받는다.
fn normal_key(n: (f32, f32)) -> (i32, i32) {
    (
        (n.0 * NORMAL_QUANT).round() as i32,
        (n.1 * NORMAL_QUANT).round() as i32,
    )
}

/// **dist 오름차순으로 정렬된** 후보에서 방향이 겹치는 행을 접고 상한까지 자른다.
///
/// # 왜 접는가
///
/// 조각 A 의 여러 정점이 B 의 **같은 직선 모서리**에 붙어 있으면, 법선이 같고
/// 우변 상수만 다른 행이 정점 수만큼 생긴다. 선형종속 행이고, 단체법 기저를
/// 퇴화시켜 `Singular matrix` 를 만든다. 실측(wsjk156, 폭 6379.767)에서
/// `4 steps applied / solver_fail 5` 로 압축 패스가 조기 종료된 원인이었다.
///
/// # 무엇을 남기는가
///
/// 입력이 dist 오름차순이므로 각 방향의 **첫 번째**가 가장 강한 제약이다
/// ([`contact_rhs`] 참조). 그것만 남긴다.
///
/// # 정확한가
///
/// 법선이 **정확히** 같은 행끼리는 실행 가능 영역이 변하지 않는다 — 나머지가
/// 남은 것에 함의되기 때문이다. 양자화 때문에 미세하게 다른 방향(<0.006도)이
/// 함께 접힐 수 있고, 그만큼은 근사다. 이 모듈은 이미 같은 이유로 근사를 안고
/// 있으며(쌍당 행 상한 `max_constraints_per_pair`), **적용 후 CDE 전수 검증과
/// λ 반감 백트래킹**이 그것을 받아낸다. 중복 제거도 같은 안전망 안에 있다.
///
/// 부수 효과 하나: 상한이 "가장 가까운 8행" 이 아니라 "가장 가까운 **서로 다른
/// 방향** 8개" 를 뜻하게 된다. 같은 예산으로 더 많은 방향을 덮는다.
///
/// # 실측 — 효과는 상태에 크게 좌우된다
///
/// 병리적 기하에서는 결정적이다. 정점 10개가 한 직선 모서리에 붙은 쌍에서
/// **12행 -> 3행**, 그중 10행이 완전히 같은 법선이었다
/// (`dedup_collapses_a_run_of_vertices_on_one_straight_edge`).
///
/// 그런데 실제 마커(wsjk156, NFP-LBF 시드 위 압축 1회)에서는 미미했다:
///
/// ```text
///              rows_max   backtracks   스텝   폭
/// 중복 제거 전   89,434       31         11    7581.3086
/// 중복 제거 후   87,417       31         11    7581.3096
/// ```
///
/// 행이 2.3% 줄었을 뿐이다. 이유는 `max_constraints_per_pair`(기본 8)가 이미
/// 쌍마다 잘라내고 있어서, 접을 수 있는 대상이 그 8개 안으로 한정되기 때문이다.
/// **`backtracks` 가 31 로 완전히 같다** — 선형화의 보수성은 그대로다.
///
/// 압축된 상태에서도 같았다(`dedup_ab_on_a_compressed_layout`, 같은 시작 레이아웃
/// 을 두 팔에 복원해 먹인 A/B):
///
/// ```text
/// 시작 6682.9146mm (밀도 76.12%)
///              rows_max   backtracks   스텝   폭
/// 끔            85,396       20          9    6671.5781
/// 켬            83,381       20          9    6671.5796
/// ```
///
/// 시간은 신호로 쓸 수 없었다. 같은 코드가 36.4s / 41.6s / 48.7s 로 흔들린다
/// (5s 제한에 걸리는 풀이 3회가 들어 있다).
///
/// # 확인하지 못한 것 — 이득의 증거가 없다
///
/// 이 변경의 표적인 `Singular matrix` 는 폭 6379mm 에서 보고됐다. 세 상태
/// (7583 / 7071 / 6682mm) 어디서도 재현하지 못했다 — 전부 `solver_fail: 0` 이다.
/// 따라서 **이 변경이 그 실패를 없앤다는 것은 실측되지 않았다.**
///
/// 재 본 것을 그대로 적으면: 행 −2.3~2.4%, `backtracks` 동일, 스텝 동일,
/// 폭 차이 +0.001~0.0015mm(해당 크기에서 f32 2~3 ULP). **중립이다.**
/// 행 감소가 이 정도에 그치는 이유는 `max_constraints_per_pair`(기본 8)가 이미
/// 쌍마다 잘라내고 있어 접을 대상이 그 8개 안으로 한정되기 때문이다.
///
/// 남겨 둔 근거는 셋이다: 법선이 정확히 같은 행끼리는 **실행 가능 영역이 변하지
/// 않고**(`contact_rhs_is_monotone_in_distance`), 세 상태 모두에서 회귀가 없었으며,
/// 병리적 기하에서는 12행 -> 3행으로 확실히 작동한다
/// (`dedup_collapses_a_run_of_vertices_on_one_straight_edge`).
/// 실제 실행에서 `solver_fail` 이 줄지 않으면 되돌릴 것 — 호출부 한 줄이다.
fn dedup_by_direction(sorted_by_dist: Vec<ContactRow>, cap: usize) -> Vec<ContactRow> {
    if !dedup_enabled() {
        // 중복 제거 이전 동작: 가장 가까운 것부터 상한까지.
        let mut rows = sorted_by_dist;
        rows.truncate(cap);
        return rows;
    }
    let mut kept: Vec<ContactRow> = Vec::with_capacity(cap.min(sorted_by_dist.len()));
    // 상한이 작아(기본 8) 선형 탐색이 해시보다 싸다. 쌍마다 도는 자리라 할당도 아깝다.
    let mut keys: Vec<(i32, i32)> = Vec::with_capacity(cap);
    for row in sorted_by_dist {
        if kept.len() >= cap {
            break;
        }
        let key = normal_key(row.n);
        if keys.contains(&key) {
            continue;
        }
        keys.push(key);
        kept.push(row);
    }
    kept
}

/// 아이템 `a`의 정점들과 아이템 `b`의 모서리들 사이의 근접 쌍에 대해 제약을 만든다.
#[cfg(test)]
fn collect_pair_rows(
    items: &[LpItem],
    a: usize,
    b: usize,
    clearance: f32,
    cfg: &LpCompactionConfig,
    out: &mut Vec<ContactRow>,
) {
    collect_pair_rows_with_control(
        items,
        a,
        b,
        clearance,
        cfg,
        out,
        &crate::util::terminator::BasicTerminator::new(),
    );
}

fn collect_pair_rows_with_control(
    items: &[LpItem],
    a: usize,
    b: usize,
    clearance: f32,
    cfg: &LpCompactionConfig,
    out: &mut Vec<ContactRow>,
    term: &impl Terminator,
) -> bool {
    let (sa, sb) = (&items[a].shape, &items[b].shape);
    let b_centroid = sb.centroid();
    let sq_clearance = clearance * clearance;

    let mut candidates: Vec<ContactRow> = vec![];

    // Consecutive polygon edges are spatially local. Skip whole small blocks
    // while retaining the original edge order and equal-distance tie policy.
    let edges: Vec<_> = sb.edge_iter().collect();
    let edge_blocks: Vec<[f32; 4]> = edges
        .chunks(8)
        .map(|block| {
            block.iter().fold(
                [
                    f32::INFINITY,
                    f32::NEG_INFINITY,
                    f32::INFINITY,
                    f32::NEG_INFINITY,
                ],
                |mut b, e| {
                    b[0] = b[0].min(e.x_min());
                    b[1] = b[1].max(e.x_max());
                    b[2] = b[2].min(e.y_min());
                    b[3] = b[3].max(e.y_max());
                    b
                },
            )
        })
        .collect();

    for vi in 0..sa.n_vertices() {
        if term.kill() {
            return false;
        }
        let v = sa.vertex(vi);
        // b의 bbox에서 clearance 밖에 있는 정점은 볼 필요 없다
        if v.0 < sb.bbox().x_min - clearance
            || v.0 > sb.bbox().x_max + clearance
            || v.1 < sb.bbox().y_min - clearance
            || v.1 > sb.bbox().y_max + clearance
        {
            continue;
        }
        for (block_index, block) in edges.chunks(8).enumerate() {
            if block_index % 8 == 0 && term.kill() {
                return false;
            }
            let bounds = edge_blocks[block_index];
            if v.0 < bounds[0] - clearance
                || v.0 > bounds[1] + clearance
                || v.1 < bounds[2] - clearance
                || v.1 > bounds[3] + clearance
            {
                continue;
            }
            for (offset, e) in block.iter().enumerate() {
                if v.0 < e.x_min() - clearance
                    || v.0 > e.x_max() + clearance
                    || v.1 < e.y_min() - clearance
                    || v.1 > e.y_max() + clearance
                {
                    continue;
                }
                if e.sq_distance_to(&v) > sq_clearance {
                    continue;
                }
                let Some(row) =
                    contact_row_at(a, b, v, &edges, block_index * 8 + offset, b_centroid)
                else {
                    continue;
                };
                if dedup_enabled() {
                    // Retain only the strongest bounded set of distinct directions.
                    // Equal-distance ties retain vertex/edge traversal order.
                    let key = normal_key(row.n);
                    if let Some(pos) = candidates.iter().position(|r| normal_key(r.n) == key) {
                        if candidates[pos].dist <= row.dist {
                            continue;
                        }
                        candidates.remove(pos);
                    }
                    let pos = candidates.partition_point(|r| r.dist <= row.dist);
                    if pos < cfg.max_constraints_per_pair {
                        candidates.insert(pos, row);
                        candidates.truncate(cfg.max_constraints_per_pair);
                    }
                } else {
                    candidates.push(row);
                }
            }
        }
    }

    // 가까운 접촉부터 우선. 같은 방향의 중복을 접고 쌍당 상한까지 자른다
    // (→ 적용 후 CDE 검증이 필수인 이유).
    candidates.sort_by_key(|r| OrderedFloat(r.dist));
    out.extend(dedup_by_direction(candidates, cfg.max_constraints_per_pair));
    true
}

/// 정점이 모서리 직선의 재료 쪽으로 이만큼(mm)까지 들어가 있어도 접촉(거리 0)으로 본다.
/// f32 좌표(수천 mm)에서 정확히 맞닿은 정점의 부호 있는 거리가 ±수 µm 로 흔들리기 때문이다.
const CONTACT_SIDE_TOLERANCE: f32 = 0.01;

/// 정점 `v` 와 `edges[i]` 사이의 제약 하나. `edges` 는 CCW 다각형의 모서리 순서 그대로다.
fn contact_row_at(
    a: usize,
    b: usize,
    v: Point,
    edges: &[Edge],
    i: usize,
    b_centroid: Point,
) -> Option<ContactRow> {
    let e = edges.get(i)?;
    if legacy_line_rows() {
        return contact_row_line(a, b, v, e.start, e.end, b_centroid);
    }
    let next = edges.get((i + 1) % edges.len())?;
    contact_row(a, b, v, e.start, e.end, next.end)
}

/// 정점 `v` 가 모서리 `(p, q)` 를 넘지 못하게 하는 제약을 만든다. `r` 은 다음 모서리의 끝점.
///
/// # 무엇을 기준으로 하는가
///
/// `v` 에서 가장 가까운 **선분 위의 점**이다. 세 경우로 갈린다.
///
/// * 발이 선분 안, `v` 가 바깥쪽 → 모서리의 바깥 법선과 수직 거리. 진짜 마주보는 접촉이다.
/// * 발이 선분 안, `v` 가 재료 쪽 → 행을 만들지 않는다. `v` 는 `b` 밖에 있으므로 이 모서리와
///   `v` 사이에는 `b` 의 다른 모서리가 있고, 그쪽 행이 관입을 막는다.
/// * 발이 끝점 `q` 너머이고 `q` 가 볼록 꼭짓점이며 `v` 가 그 꼭짓점 영역 안 → 꼭짓점 `q`
///   에서 `v` 로 향하는 방향과 그 거리. 꼭짓점 영역 안에서는 이 방향이 `q` 의 법선 원뿔에
///   들어 있어, `q` 를 지나는 수직선이 `b` 의 국소 받침선이 된다.
///   시작점 `p` 쪽은 앞 모서리의 이 분기가 맡으므로 여기서는 만들지 않는다.
///
/// # 왜 무한 직선이 아닌가
///
/// 예전에는 모서리의 **무한 직선**까지 거리를 쓰고 `v` 가 어느 쪽에 있든 그쪽을 유지시켰다
/// ([`contact_row_line`]). 그러면 `v` 가 모서리의 연장선 근처에 있기만 해도 — 실제 선분은
/// 수십 mm 떨어져 있어도 — 거리 ≈ 0 인 행이 생겨 그 법선 방향 이동이 통째로 막힌다.
/// 곡선 조각은 모서리가 많아 거의 모든 방향이 이렇게 핀 되고, 쌍당 «거리 작은 8개» 선택에서
/// 이 가짜 행이 진짜 마주보는 행을 밀어낸다. 실측(wsjk156, 폭 6337.6): 신뢰 영역 31mm 에서
/// 스텝당 0.1~0.3mm, 24스텝에 백트래킹 54회.
fn contact_row(a: usize, b: usize, v: Point, p: Point, q: Point, r: Point) -> Option<ContactRow> {
    let (ex, ey) = (q.0 - p.0, q.1 - p.1);
    let len = (ex * ex + ey * ey).sqrt();
    if len <= f32::EPSILON || !len.is_finite() {
        return None;
    }
    // CCW 다각형의 바깥 법선.
    let n_out = (ey / len, -ex / len);
    let t = ((v.0 - p.0) * ex + (v.1 - p.1) * ey) / (len * len);
    if t < 0.0 {
        return None;
    }
    if t <= 1.0 {
        let dist = n_out.0 * (v.0 - p.0) + n_out.1 * (v.1 - p.1);
        if dist < -CONTACT_SIDE_TOLERANCE {
            return None;
        }
        return Some(ContactRow {
            a,
            b,
            n: n_out,
            dist: dist.max(0.0),
        });
    }

    // 끝점 q 의 꼭짓점 영역.
    let (fx, fy) = (r.0 - q.0, r.1 - q.1);
    let flen = (fx * fx + fy * fy).sqrt();
    if flen <= f32::EPSILON || !flen.is_finite() {
        return None;
    }
    // 오목 꼭짓점에는 바깥 꼭짓점 영역이 없다 (CCW 에서 좌회전만 볼록).
    if ex * fy - ey * fx <= 0.0 {
        return None;
    }
    let (ux, uy) = (v.0 - q.0, v.1 - q.1);
    // 다음 모서리 위(또는 너머)에 발이 떨어지면 그 모서리의 행이 맡는다.
    if ux * fx + uy * fy >= 0.0 {
        return None;
    }
    let dist = (ux * ux + uy * uy).sqrt();
    if !dist.is_finite() {
        return None;
    }
    let n = if dist > f32::EPSILON {
        (ux / dist, uy / dist)
    } else {
        // 꼭짓점끼리 맞닿았다 → 두 모서리 바깥 법선의 이등분 방향.
        let (bx, by) = (n_out.0 + fy / flen, n_out.1 - fx / flen);
        let blen = (bx * bx + by * by).sqrt();
        if blen <= f32::EPSILON {
            return None;
        }
        (bx / blen, by / blen)
    };
    Some(ContactRow { a, b, n, dist })
}

/// 옛 방식: 모서리 (p, q)의 **무한 직선**에 대해 정점이 현재 쪽을 유지하게 한다.
/// 측정용 스위치([`legacy_line_rows`])로만 도달한다.
fn contact_row_line(
    a: usize,
    b: usize,
    v: Point,
    p: Point,
    q: Point,
    b_centroid: Point,
) -> Option<ContactRow> {
    let (ex, ey) = (q.0 - p.0, q.1 - p.1);
    let len = (ex * ex + ey * ey).sqrt();
    if len <= f32::EPSILON || !len.is_finite() {
        return None;
    }
    // 모서리에 수직인 단위 벡터
    let mut n = (-ey / len, ex / len);
    let mut dist = n.0 * (v.0 - p.0) + n.1 * (v.1 - p.1);

    if dist.abs() <= f32::EPSILON {
        // 접촉 중이라 부호로 방향을 정할 수 없다 → b의 중심에서 멀어지는 쪽을 바깥으로 본다.
        let out = n.0 * (v.0 - b_centroid.0) + n.1 * (v.1 - b_centroid.1);
        if out < 0.0 {
            n = (-n.0, -n.1);
        }
        dist = 0.0;
    } else if dist < 0.0 {
        n = (-n.0, -n.1);
        dist = -dist;
    }

    Some(ContactRow { a, b, n, dist })
}

/// 압축 모델을 백엔드 중립 표현([`LpProblem`])으로 담는다. **모델을 만드는 곳은
/// 여기 하나뿐이다** — 어떤 백엔드로 풀든 같은 제약 집합을 본다.
///
/// 변수 배치는 호출부가 해를 되읽는 순서와 **반드시 같아야 한다**.
///
/// ```text
/// 0            폭 W          목적 계수 1.0,  경계 [w_lower, w_upper]
/// 1 + 2k       dx_k          목적 계수 pull, 경계 [-trust, trust]
/// 2 + 2k       dy_k          목적 계수 0,    경계 [-trust, trust]
/// 그 뒤        s_r           목적 계수 [`MARGIN_SLACK_COST`], 경계 [0, 마진 - dist_r]
/// ```
///
/// `k` counts movable units. Every member of a rigid unit uses the same pair;
/// frozen units have no translation variables. `s_r` 는 간격이 마진보다 좁은 접촉 행마다
/// 하나씩 붙는 슬랙이고, 해를 되읽을 때는 쓰지 않는다.
///
/// # 2차 항이 W 에도 걸리는 것에 대하여
///
/// [`LpProblem::quadratic`]은 모든 변수에 같은 계수를 거는 대각 형태다. 정규화의
/// 본래 목적은 "같은 폭이면 덜 움직이는 해"이므로 `dx, dy` 에만 걸고 싶지만,
/// W 에 걸리는 `(mu/2)W²` 은 양의 정의역에서 **단조 증가**라 폭을 줄이는 방향을
/// 강화할 뿐 뒤집지 않는다. 1차항이 이미 W 를 최소화하고 있으므로 방향은 안전하다.
/// 상대 가중치는 바뀌므로, 계수를 키울 때는 A/B 로 확인할 것.
#[allow(clippy::too_many_arguments)]
fn build_generic_model(
    items: &[LpItem],
    rows: &[ContactRow],
    container: Rect,
    w_lower: f32,
    w_upper: f32,
    trust: f32,
    pull: f64,
    cfg: &LpCompactionConfig,
    solver: &LpSolverConfig,
) -> LpProblem {
    let variables = LpVariables::new(items);
    let slot_of = &variables.slot_of;
    let n_movable = variables.representatives.len();
    let n = 1 + 2 * n_movable;
    let mut objective = vec![0.0; n];
    let mut lower = vec![0.0; n];
    let mut upper = vec![0.0; n];

    objective[0] = 1.0;
    lower[0] = w_lower as f64;
    upper[0] = w_upper as f64;
    for k in 0..n_movable {
        objective[1 + 2 * k] = pull;
        lower[1 + 2 * k] = -trust as f64;
        upper[1 + 2 * k] = trust as f64;
        lower[2 + 2 * k] = -trust as f64;
        upper[2 + 2 * k] = trust as f64;
    }

    let mut constraints = Vec::with_capacity(4 * items.len() + rows.len());

    // 컨테이너 제약. 고정 벽(좌·하·상)에는 접촉 행과 같은 여유([`contact_rhs`])를 둔다.
    //
    // 여유 없이 `dx >= x_min - bbox.x_min` 로 두면 좌측 당김 목적 때문에 왼쪽 열 조각이
    // 벽에 **정확히** 붙는 해가 나오고, jagua 는 벽과의 정확한 접촉을 충돌로 본다. 그래서
    // λ = 1 적용이 매번 실패하고 전 조각이 λ = 0.5 로 물러났다 — 실측(wsjk156, 폭 6337.6)
    // 에서 20스텝 연속 `backtrack 1`, 스텝마다 LP 가 찾은 이득의 절반만 얻었다.
    // 우측 벽은 폭 W 가 변수이고 적용 뒤 strip 을 다시 맞추므로 해당이 없다.
    for (i, it) in items.iter().enumerate().filter(|(_, it)| !it.frozen) {
        let dx = slot_of[i];
        let dy = dx + 1;
        // 좌측 벽: dx >= -(간격 - 여유)
        constraints.push(LpRow::greater_or_equal(
            vec![dx],
            vec![1.0],
            contact_rhs((it.bbox.x_min - container.x_min).max(0.0)),
        ));
        // 우측 벽(가변): dx - W <= -bbox.x_max
        constraints.push(LpRow::less_or_equal(
            vec![dx, 0],
            vec![1.0, -1.0],
            -(it.bbox.x_max as f64),
        ));
        // 하단 / 상단 — 높이는 고정이다.
        constraints.push(LpRow::greater_or_equal(
            vec![dy],
            vec![1.0],
            contact_rhs((it.bbox.y_min - container.y_min).max(0.0)),
        ));
        constraints.push(LpRow::less_or_equal(
            vec![dy],
            vec![1.0],
            -contact_rhs((container.y_max - it.bbox.y_max).max(0.0)),
        ));
    }

    // 접촉(비겹침) 제약 — n·(d_a - d_b) >= -(dist - keep)
    for row in rows.iter() {
        let mut cols = Vec::with_capacity(4);
        let mut vals = Vec::with_capacity(4);
        // Aggregation is necessary when both endpoints share variables;
        // duplicate column entries are not portable across LP backends.
        let mut add = |col, value| {
            if let Some(index) = cols.iter().position(|&existing| existing == col) {
                vals[index] += value;
            } else {
                cols.push(col);
                vals.push(value);
            }
        };
        if let Some(&a) = slot_of.get(row.a)
            && a != usize::MAX
        {
            add(a, row.n.0 as f64);
            add(a + 1, row.n.1 as f64);
        }
        if let Some(&b) = slot_of.get(row.b)
            && b != usize::MAX
        {
            add(b, -row.n.0 as f64);
            add(b + 1, -row.n.1 as f64);
        }
        if cols.is_empty() || vals.iter().all(|&value| value == 0.0) {
            // 양쪽 다 고정이면 제약이 무의미하다 (그리고 빈 행은 백엔드가 거절한다).
            continue;
        }
        if row.dist < LP_CONTACT_MARGIN && !legacy_margin_rows() {
            // 마진보다 좁다 → 되돌리되, 못 되돌리는 만큼은 슬랙이 받는다([`MARGIN_SLACK_COST`]).
            let lacking = (LP_CONTACT_MARGIN - row.dist) as f64;
            cols.push(objective.len());
            vals.push(1.0);
            objective.push(MARGIN_SLACK_COST);
            lower.push(0.0);
            upper.push(lacking);
            constraints.push(LpRow::greater_or_equal(cols, vals, lacking));
        } else {
            constraints.push(LpRow::greater_or_equal(cols, vals, contact_rhs(row.dist)));
        }
    }

    let _ = cfg; // 행 상한은 호출부가 이미 확인했다.

    LpProblem {
        objective,
        quadratic: if solver.uses_quadratic() {
            solver.quadratic_regularization as f64
        } else {
            0.0
        },
        lower,
        upper,
        rows: constraints,
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn issue8_report_distinguishes_completion_from_caps_and_incomplete_solves() {
        use crate::util::terminator::BasicTerminator;
        use IterOutcome::*;
        let cases: &[(&[IterOutcome], PassExit, usize)] = &[
            (
                &[NoImprovement, NoImprovement, NoImprovement],
                PassExit::ScheduleExhausted,
                0,
            ),
            (&[Blocked], PassExit::Blocked, 0),
            (&[NoMovable], PassExit::NoMovable, 0),
            (&[Abort], PassExit::Incomplete, 0),
            (&[Intractable, NoImprovement], PassExit::Incomplete, 0),
            (&[NoImprovement, Intractable], PassExit::Incomplete, 0),
            (&[NoImprovement], PassExit::Incomplete, 0),
            (&[Applied(10.)], PassExit::Incomplete, 1),
        ];
        for &(outcomes, expected, applied) in cases {
            let mut seen = Vec::new();
            let report = run_pass(outcomes.len(), &BasicTerminator::new(), |level| {
                let outcome = outcomes[seen.len()];
                seen.push(level);
                outcome
            });
            assert_eq!(
                (report.exit, report.applied),
                (expected, applied),
                "{outcomes:?}"
            );
            assert_eq!(seen.len(), outcomes.len());
        }
        let mut solver = crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG;
        for backend in [
            crate::lp_solver::LpSolverKind::Clp,
            crate::lp_solver::LpSolverKind::Xpress,
        ] {
            solver.backend = backend;
            assert!(
                !PassReport {
                    applied: 0,
                    exit: PassExit::ScheduleExhausted
                }
                .proves_completion(&solver)
            );
            assert!(
                PassReport {
                    applied: 0,
                    exit: PassExit::NoMovable
                }
                .proves_completion(&solver)
            );
        }
    }

    #[test]
    fn issue8_cancelled_terminal_backtrack_cannot_prove_completion() {
        use std::sync::atomic::{AtomicBool, Ordering};
        struct Stop(AtomicBool);
        impl Terminator for Stop {
            fn kill(&self) -> bool {
                self.0.load(Ordering::Relaxed)
            }
            fn new_timeout(&mut self, _: std::time::Duration) {}
            fn timeout_at(&self) -> Option<jagua_rs::Instant> {
                None
            }
        }
        let term = Stop(AtomicBool::new(false));
        let report = run_pass(40, &term, |_| {
            term.0.store(true, Ordering::Relaxed);
            IterOutcome::Blocked
        });
        assert_eq!(report.exit, PassExit::Incomplete);
    }

    use super::*;
    use crate::lp_solver::{LpBackend, LpStatus, microlp_backend::MicrolpBackend};
    use crate::util::terminator::BasicTerminator;
    use std::sync::Arc;
    use std::time::Duration;

    /// 축 정렬 사각형 하나. `SPolygon::new` 이 방향을 CCW 로 맞춰 준다.
    fn rect_poly(x: f32, y: f32, w: f32, h: f32) -> Arc<SPolygon> {
        Arc::new(
            SPolygon::new(vec![
                Point(x, y),
                Point(x + w, y),
                Point(x + w, y + h),
                Point(x, y + h),
            ])
            .expect(crate::dmsg!("사각형은 유효한 단순 다각형이다")),
        )
    }

    /// 등가성 검사에 쓰는 모델 파라미터. 실제 `lp_iteration` 이 계산하는 것과
    /// 같은 식으로 만든다 — 여기서 어긋나면 검사 자체가 무의미하다.
    struct Setup {
        items: Vec<LpItem>,
        rows: Vec<ContactRow>,
        container: Rect,
        w_lower: f32,
        width_before: f32,
        trust: f32,
        pull: f64,
        cfg: LpCompactionConfig,
    }

    /// 조각 4개 중 하나는 고정. 컨테이너 제약·접촉 제약·고정 아이템 건너뛰기가
    /// 한 모델 안에 전부 들어가도록 서로 가깝게 놓는다.
    fn setup() -> Setup {
        let container = Rect::try_new(0.0, 0.0, 40.0, 16.0).expect(crate::dmsg!("컨테이너"));
        // (x, y, w, h, frozen)
        let specs = [
            (1.0f32, 1.0f32, 4.0f32, 4.0f32, false),
            (5.2, 1.4, 4.0, 4.0, false),
            (1.4, 5.3, 5.0, 3.0, true),
            (9.6, 2.0, 3.0, 6.0, false),
        ];
        let mut keys: slotmap::SlotMap<PItemKey, ()> = slotmap::SlotMap::with_key();
        let items: Vec<LpItem> = specs
            .iter()
            .map(|&(x, y, w, h, frozen)| {
                let shape = rect_poly(x, y, w, h);
                let bbox = shape.bbox();
                LpItem {
                    pk: keys.insert(()),
                    group_representative: None,
                    shape,
                    bbox,
                    frozen,
                }
            })
            .collect();

        let mut cfg = crate::config::DEFAULT_POLISH_CONFIG.lp;
        cfg.n_iterations = 1;
        cfg.trust_region_ratio = 0.2;

        let width_before = 20.0f32;
        let trust = width_before * cfg.trust_region_ratio;
        let clearance = trust * cfg.clearance_factor.max(MIN_CLEARANCE_FACTOR);
        let rows = build_contact_rows(&items, clearance, &cfg, &BasicTerminator::new())
            .expect(crate::dmsg!("중단 요청이 없으므로 항상 Some"));
        // 등가성 검사는 fixture 가 실제로 만든 행만큼만 본다. 그래서 **어떤 종류의
        // 행이 생겼는지**를 여기서 못 박는다 — 조각 좌표를 나중에 손대다 한쪽 분기가
        // 조용히 빠지면, 두 경로가 갈려도 테스트는 통과해 버린다.
        let is_frozen = |i: usize| items.get(i).map(|it| it.frozen).unwrap_or(false);
        let n_mixed = rows
            .iter()
            .filter(|r| is_frozen(r.a) != is_frozen(r.b))
            .count();
        let n_free = rows
            .iter()
            .filter(|r| !is_frozen(r.a) && !is_frozen(r.b))
            .count();
        assert!(
            n_mixed > 0,
            "고정 아이템이 낀 접촉 행이 없다 — 변수 없는 쪽을 건너뛰는 분기가 안 돈다"
        );
        assert!(
            n_free > 0,
            "가변끼리의 접촉 행이 없다 — 양쪽에 계수가 붙는 분기가 안 돈다"
        );
        // 마진보다 좁은 행에는 슬랙 변수가 붙는다. 인라인 경로에 없던 형태라 등가성 검사의
        // 대상이 아니다 — 그쪽은 `narrow_contact_*` 테스트가 본다.
        assert!(
            rows.iter().all(|r| r.dist >= LP_CONTACT_MARGIN),
            "fixture 에 마진보다 좁은 접촉이 생겼다"
        );

        let w_lower = (width_before - trust).max(f32::EPSILON);
        let n_movable = items.iter().filter(|it| !it.frozen).count();
        let pull = (cfg.left_pull_weight / n_movable as f32) as f64;

        Setup {
            items,
            rows,
            container,
            w_lower,
            width_before,
            trust,
            pull,
            cfg,
        }
    }

    /// 제약 하나를 자료구조와 무관하게 비교하기 위한 정규형:
    /// `(정렬된 (열, 계수) 목록, 하한, 상한)`.
    type Canon = (Vec<(usize, i64)>, i64, i64);

    /// f64 를 비교 가능한 정수로. 모델 계수는 전부 f32 에서 올라온 값이라
    /// 1e-9 눈금이면 두 경로의 "같은 식" 이 정확히 같은 정수로 떨어진다.
    fn q(v: f64) -> i64 {
        if v.is_infinite() {
            return if v > 0.0 { i64::MAX } else { i64::MIN };
        }
        (v * 1.0e9).round() as i64
    }

    fn canon(cols: &[usize], vals: &[f64], lower: f64, upper: f64) -> Canon {
        let mut terms: Vec<(usize, i64)> = cols
            .iter()
            .zip(vals.iter())
            .map(|(&c, &v)| (c, q(v)))
            .collect();
        terms.sort_unstable();
        (terms, q(lower), q(upper))
    }

    /// **합치기 전 인라인 microlp 경로가 만들던 제약 집합을 손으로 옮겨 적은 것.**
    ///
    /// 인라인 경로는 microlp `Problem` 안에 제약을 바로 밀어 넣었고 그 자료구조는
    /// 밖에서 읽을 수 없다. 그래서 "옛 경로가 무엇을 넣었는가" 를 여기 한 벌 남겨
    /// 두고 [`build_generic_model`] 이 만드는 행과 대조한다. 나중에 일반화 표현을
    /// 손대면 여기서 먼저 걸린다.
    fn inline_path_rows(s: &Setup) -> Vec<Canon> {
        let mut slot = vec![usize::MAX; s.items.len()];
        let mut k = 0;
        for (i, it) in s.items.iter().enumerate() {
            if !it.frozen {
                if let Some(cell) = slot.get_mut(i) {
                    *cell = 1 + 2 * k;
                }
                k += 1;
            }
        }
        let slot_of = |i: usize| slot.get(i).copied().unwrap_or(usize::MAX);
        let inf = f64::INFINITY;
        let mut out = vec![];
        for (i, it) in s.items.iter().enumerate() {
            if it.frozen {
                continue;
            }
            let (dx, dy) = (slot_of(i), slot_of(i) + 1);
            // 고정 벽에는 접촉 행과 같은 여유를 둔다 (옛 인라인 경로에는 없던 것).
            let wall = |gap: f32| {
                let keep = (gap * LP_KEEP_RATIO).max(LP_CONTACT_MARGIN.min(gap));
                (gap - keep) as f64
            };
            out.push(canon(
                &[dx],
                &[1.0],
                -wall(it.bbox.x_min - s.container.x_min),
                inf,
            ));
            out.push(canon(&[dx, 0], &[1.0, -1.0], -inf, -(it.bbox.x_max as f64)));
            out.push(canon(
                &[dy],
                &[1.0],
                -wall(it.bbox.y_min - s.container.y_min),
                inf,
            ));
            out.push(canon(
                &[dy],
                &[1.0],
                -inf,
                wall(s.container.y_max - it.bbox.y_max),
            ));
        }
        for row in s.rows.iter() {
            let mut cols = vec![];
            let mut vals = vec![];
            if slot_of(row.a) != usize::MAX {
                cols.push(slot_of(row.a));
                vals.push(row.n.0 as f64);
                cols.push(slot_of(row.a) + 1);
                vals.push(row.n.1 as f64);
            }
            if slot_of(row.b) != usize::MAX {
                cols.push(slot_of(row.b));
                vals.push(-row.n.0 as f64);
                cols.push(slot_of(row.b) + 1);
                vals.push(-row.n.1 as f64);
            }
            let keep = (row.dist * LP_KEEP_RATIO).max(LP_CONTACT_MARGIN.min(row.dist));
            out.push(canon(&cols, &vals, -((row.dist - keep) as f64), inf));
        }
        out
    }

    fn generic_model(s: &Setup) -> LpProblem {
        build_generic_model(
            &s.items,
            &s.rows,
            s.container,
            s.w_lower,
            s.width_before,
            s.trust,
            s.pull,
            &s.cfg,
            &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
        )
    }

    #[test]
    fn rigid_unit_model_uses_all_member_bounds_and_aggregates_internal_contacts() {
        let mut s = setup();
        let anchor = s.items[0].pk;
        s.items[0].group_representative = Some(anchor);
        s.items[1].group_representative = Some(anchor);
        s.rows = vec![ContactRow {
            a: 0,
            b: 1,
            n: (1.0, 0.0),
            dist: 0.2,
        }];
        let model = generic_model(&s);
        // Two movable units (members 0+1, singleton 3), one frozen item.
        assert_eq!(model.n_vars(), 5);
        // Three movable members retain all four bounds each. The contact
        // within a rigid unit cancels completely instead of duplicating columns.
        assert_eq!(model.n_rows(), 12);
        assert_eq!(model.rows[1].cols, vec![1, 0]);
        assert_eq!(model.rows[5].cols, vec![1, 0]);
        assert_eq!(model.rows[5].upper, -(s.items[1].bbox.x_max as f64));
        assert_eq!(model.rows[9].cols, vec![3, 0]);
        assert!(model.rows.iter().all(|row| {
            row.cols
                .iter()
                .enumerate()
                .all(|(i, col)| !row.cols[..i].contains(col))
        }));
    }

    #[test]
    fn internal_rigid_contacts_do_not_consume_the_contact_budget() {
        let mut s = setup();
        s.items.truncate(2);
        let anchor = s.items[0].pk;
        for it in &mut s.items {
            it.group_representative = Some(anchor);
        }
        s.cfg.max_constraints = 0;
        assert!(
            build_contact_rows(&s.items, 10., &s.cfg, &BasicTerminator::new())
                .unwrap()
                .is_empty()
        );
    }

    /// 조각 0 의 오른변에 조각 1 의 정점이 간격 `dist` 로 붙어 있다는 행 하나.
    fn narrow_contact(dist: f32) -> ContactRow {
        ContactRow {
            a: 1,
            b: 0,
            n: (1.0, 0.0),
            dist,
        }
    }

    /// 마진보다 좁은 접촉의 행에는 슬랙이 붙고, 슬랙이 상한이면 옛 행(우변 0)과 같다 —
    /// 그래서 현재 배치는 언제나 실행 가능하다.
    #[test]
    fn narrow_contact_row_gets_a_slack_that_keeps_the_current_layout_feasible() {
        let mut s = setup();
        s.rows = vec![narrow_contact(0.01), narrow_contact(LP_CONTACT_MARGIN)];
        let model = generic_model(&s);
        let n_movable = s.items.iter().filter(|it| !it.frozen).count();
        let slack = 1 + 2 * n_movable;
        assert_eq!(model.n_vars(), slack + 1, "좁은 행 하나에만 슬랙이 붙는다");
        assert!(model.validate().is_ok(), "{:?}", model.validate());

        let lacking = (LP_CONTACT_MARGIN - 0.01) as f64;
        assert_eq!(q(model.objective[slack]), q(MARGIN_SLACK_COST));
        assert_eq!(
            (q(model.lower[slack]), q(model.upper[slack])),
            (0, q(lacking))
        );

        let [narrow, at_margin] = &model.rows[model.n_rows() - 2..] else {
            panic!("{}", crate::dmsg!("접촉 행 두 개가 마지막에 온다"));
        };
        // dx_1 - dx_0 + s >= 마진 - dist. d = 0, s = 상한에서 등식으로 성립한다.
        assert_eq!(
            canon(&narrow.cols, &narrow.vals, narrow.lower, narrow.upper),
            canon(
                &[3, 4, 1, 2, slack],
                &[1.0, 0.0, -1.0, 0.0, 1.0],
                lacking,
                f64::INFINITY
            )
        );
        // 마진에 닿아 있는 행은 예전 그대로다: 더 가까워지지만 않으면 된다.
        assert_eq!(
            canon(
                &at_margin.cols,
                &at_margin.vals,
                at_margin.lower,
                at_margin.upper
            ),
            canon(&[3, 4, 1, 2], &[1.0, 0.0, -1.0, 0.0], 0.0, f64::INFINITY)
        );
    }

    /// 벌어질 자리가 있으면 마진을 되찾고, 양쪽이 막혀 있으면 슬랙이 받아 모델이 풀린다.
    #[test]
    fn narrow_contact_is_restored_to_the_margin_when_there_is_room() {
        let mut s = setup();
        let n_movable = s.items.iter().filter(|it| !it.frozen).count();
        let slack = 1 + 2 * n_movable;

        s.rows = vec![narrow_contact(0.0)];
        let sol = MicrolpBackend.solve(&generic_model(&s), Duration::from_secs(30));
        assert_eq!(sol.status, LpStatus::Optimal);
        let gap = sol.x[3] - sol.x[1];
        assert!((gap - LP_CONTACT_MARGIN as f64).abs() < 1e-9, "간격 {gap}");
        assert!(sol.x[slack].abs() < 1e-9, "슬랙 {}", sol.x[slack]);

        // 같은 쌍을 반대 방향으로도 묶으면 어느 쪽으로도 벌릴 수 없다.
        let mut opposite = narrow_contact(0.0);
        opposite.n = (-1.0, 0.0);
        s.rows.push(opposite);
        let sol = MicrolpBackend.solve(&generic_model(&s), Duration::from_secs(30));
        assert_eq!(
            sol.status,
            LpStatus::Optimal,
            "막힌 접촉이 모델을 실행 불가능하게 만들었다"
        );
        let unrestored = sol.x[slack] + sol.x[slack + 1];
        assert!(
            (unrestored - 2.0 * LP_CONTACT_MARGIN as f64).abs() < 1e-9,
            "슬랙 합 {unrestored}"
        );
    }

    /// 1단계: **두 경로가 같은 제약 집합을 만든다** (개수·계수·우변).
    #[test]
    fn both_paths_agree_on_the_same_model() {
        let s = setup();
        let model = generic_model(&s);
        let expected = inline_path_rows(&s);

        assert_eq!(
            model.rows.len(),
            expected.len(),
            "제약 개수가 다르다: 일반화 {} vs 인라인 {}",
            model.rows.len(),
            expected.len()
        );

        let actual: Vec<Canon> = model
            .rows
            .iter()
            .map(|r| canon(&r.cols, &r.vals, r.lower, r.upper))
            .collect();
        for (i, (a, e)) in actual.iter().zip(expected.iter()).enumerate() {
            assert_eq!(a, e, "제약 {i} 이 인라인 경로와 다르다");
        }

        // 변수 쪽도 같아야 한다: 0 = W, 이후 가변 아이템마다 (dx, dy).
        let n_movable = s.items.iter().filter(|it| !it.frozen).count();
        assert_eq!(model.n_vars(), 1 + 2 * n_movable);
        assert_eq!(model.objective.first().map(|&v| q(v)), Some(q(1.0)));
        assert_eq!(
            model.lower.first().map(|&v| q(v)),
            Some(q(s.w_lower as f64))
        );
        assert_eq!(
            model.upper.first().map(|&v| q(v)),
            Some(q(s.width_before as f64))
        );
        for k in 0..n_movable {
            assert_eq!(
                model.objective.get(1 + 2 * k).map(|&v| q(v)),
                Some(q(s.pull)),
                "dx{k} 목적 계수"
            );
            assert_eq!(
                model.objective.get(2 + 2 * k).map(|&v| q(v)),
                Some(0),
                "dy{k} 목적 계수"
            );
            for v in [1 + 2 * k, 2 + 2 * k] {
                assert_eq!(model.lower.get(v).map(|&x| q(x)), Some(q(-s.trust as f64)));
                assert_eq!(model.upper.get(v).map(|&x| q(x)), Some(q(s.trust as f64)));
            }
        }
        // 기본 설정은 순수 LP 다. 2차항이 새로 붙으면 단체법 백엔드는 경고 한 줄만
        // 남기고 1차항만 푼다 — 모델이 조용히 다른 문제가 되는데 행 비교로는 안 잡힌다.
        assert_eq!(q(model.quadratic), 0, "기본 설정에서 2차항이 붙었다");
        assert!(model.validate().is_ok(), "{:?}", model.validate());
    }

    /// 2단계: **두 경로의 해가 같은 목적값·같은 스텝을 낸다.**
    ///
    /// 인라인 경로를 microlp API 로 그대로 다시 세워 풀고, 일반화 표현을 같은
    /// 백엔드로 푼 결과와 대조한다. 변수와 제약을 넣는 순서까지 같으므로 단체법은
    /// 같은 꼭짓점에 도착해야 한다 — 퇴화한 LP 라도 결과가 갈릴 여지가 없다.
    #[test]
    fn both_paths_reach_the_same_objective_and_the_same_step() {
        use microlp::{ComparisonOp, LinearExpr, OptimizationDirection, Problem, SolveOutcome};

        let s = setup();

        // ── 인라인 경로 재현 ──
        let mut lp = Problem::new(OptimizationDirection::Minimize);
        let w_var = lp.add_var(1.0, (s.w_lower as f64, s.width_before as f64));
        let mut vars = vec![None; s.items.len()];
        for (i, it) in s.items.iter().enumerate() {
            if it.frozen {
                continue;
            }
            let dx = lp.add_var(s.pull, (-s.trust as f64, s.trust as f64));
            let dy = lp.add_var(0.0, (-s.trust as f64, s.trust as f64));
            if let Some(cell) = vars.get_mut(i) {
                *cell = Some((dx, dy));
            }
        }
        let var_of = |i: usize| vars.get(i).copied().flatten();
        // 고정 벽의 여유 — `inline_path_rows` 와 같은 식.
        let wall = |gap: f32| {
            let keep = (gap * LP_KEEP_RATIO).max(LP_CONTACT_MARGIN.min(gap));
            (gap - keep) as f64
        };
        for (i, it) in s.items.iter().enumerate() {
            let Some((dx, dy)) = var_of(i) else { continue };
            let mut left = LinearExpr::empty();
            left.add(dx, 1.0);
            lp.add_constraint(
                left,
                ComparisonOp::Ge,
                -wall(it.bbox.x_min - s.container.x_min),
            );
            let mut right = LinearExpr::empty();
            right.add(dx, 1.0);
            right.add(w_var, -1.0);
            lp.add_constraint(right, ComparisonOp::Le, -(it.bbox.x_max as f64));
            let mut bottom = LinearExpr::empty();
            bottom.add(dy, 1.0);
            lp.add_constraint(
                bottom,
                ComparisonOp::Ge,
                -wall(it.bbox.y_min - s.container.y_min),
            );
            let mut top = LinearExpr::empty();
            top.add(dy, 1.0);
            lp.add_constraint(
                top,
                ComparisonOp::Le,
                wall(s.container.y_max - it.bbox.y_max),
            );
        }
        for row in s.rows.iter() {
            let mut e = LinearExpr::empty();
            if let Some((dx, dy)) = var_of(row.a) {
                e.add(dx, row.n.0 as f64);
                e.add(dy, row.n.1 as f64);
            }
            if let Some((dx, dy)) = var_of(row.b) {
                e.add(dx, -row.n.0 as f64);
                e.add(dy, -row.n.1 as f64);
            }
            let keep = (row.dist * LP_KEEP_RATIO).max(LP_CONTACT_MARGIN.min(row.dist));
            lp.add_constraint(e, ComparisonOp::Ge, -((row.dist - keep) as f64));
        }
        let inline_sol = match lp.solve() {
            Ok(SolveOutcome::Solution(sol)) => sol,
            Ok(SolveOutcome::Interrupted(_)) => panic!("{}", crate::dmsg!("인라인 경로가 제한에 걸렸다")),
            Err(e) => panic!("인라인 경로가 실패했다: {e:?}"),
        };
        let inline_obj = inline_sol.objective();
        let inline_w = inline_sol[w_var];
        let inline_d: Vec<(f64, f64)> = vars
            .iter()
            .filter_map(|v| v.map(|(dx, dy)| (inline_sol[dx], inline_sol[dy])))
            .collect();

        // ── 일반화 경로 ──
        let model = generic_model(&s);
        let sol = MicrolpBackend.solve(&model, Duration::from_secs(30));
        assert_eq!(sol.status, LpStatus::Optimal, "일반화 경로 상태");

        assert!(
            (sol.objective - inline_obj).abs() < 1.0e-9,
            "목적값이 다르다: 인라인 {inline_obj} vs 일반화 {}",
            sol.objective
        );
        let generic_w = sol.x.first().copied().expect(crate::dmsg!("W 는 0번 변수다"));
        assert!(
            (generic_w - inline_w).abs() < 1.0e-9,
            "폭이 다르다: 인라인 {inline_w} vs 일반화 {generic_w}"
        );
        assert_eq!(sol.x.len(), 1 + 2 * inline_d.len(), "해 길이");
        for (k, &(dx, dy)) in inline_d.iter().enumerate() {
            let gdx = sol.x.get(1 + 2 * k).copied().unwrap_or(f64::NAN);
            let gdy = sol.x.get(2 + 2 * k).copied().unwrap_or(f64::NAN);
            assert!(
                (gdx - dx).abs() < 1.0e-9 && (gdy - dy).abs() < 1.0e-9,
                "아이템 {k} 이동량이 다르다: 인라인 ({dx}, {dy}) vs 일반화 ({gdx}, {gdy})"
            );
        }
    }

    // ────────────────────────────────────────────────────────────────────
    // 실 인스턴스 계측 (여성WSjk 156-패널). 자동 회귀가 아니라 **측정**이라
    // `#[ignore]` 다 — 한 번 도는 데 수십 초가 걸린다. 인라인 microlp 경로를
    // 지우기 전후로 압축 결과(폭)가 같은지 재는 데 쓴 하네스가 이것이다.
    //
    //     cargo test -p swarm-core --release \
    //       --features "simd,capmods,adaptive,structure-first" \
    //       --lib optimizer::polish::lp::tests::lp_compaction -- --ignored --nocapture
    //
    // 노브: LP_MEASURE_ORDERINGS(기본 3), LP_MEASURE_PROBES(기본 1024),
    //       LP_MEASURE_TIME_LIMIT_MS(기본 5000 = DEFAULT_LP_TIME_LIMIT_MS).
    // ────────────────────────────────────────────────────────────────────

    fn env_usize(key: &str, default: usize) -> usize {
        std::env::var(key)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    }

    /// 실제 마커에 NFP-LBF 로 배치를 하나 만들고 그 위에 압축을 건다.
    /// 폭 전/후와 적용 스텝 수를 찍는다 — 그 숫자가 합치기 전후 비교의 대상이다.
    ///
    /// 합치기 전/후 실측 (ordering 3, 탐침 1024, seed 0):
    ///
    /// ```text
    /// 인라인 경로 (합치기 전)          7583.8721 -> 7581.3110,  11 스텝,  41.7s
    /// 합친 경로, time_limit 600s      7583.8721 -> 7581.3110,  11 스텝,  43.3s
    /// 합친 경로, time_limit 5s(기본)   7583.8721 -> 7581.3115,  11 스텝,  27.2s
    /// ```
    ///
    /// 제한을 풀면 폭이 **정확히** 재현된다 — 모델과 해 읽기는 그대로라는 뜻이다.
    /// 기본 5s 제한에서만 0.0005mm 차이가 나는데, 그건 `lp_iteration` 의 제한 계산
    /// 주석에 적은 대로 한 번의 풀이가 예산을 넘겨 버려진 결과다.
    ///
    /// # [`IterOutcome`] 도입 후 (같은 조건, 재측정)
    ///
    /// ```text
    /// 7583.8721 -> 7581.3086,  11 스텝,  36.4s
    /// LpPassStats { backtracks: 31, interrupted: 3, solver_fail: 0,
    ///               skipped_rows_cap: 0, shrinks: 3 }
    /// ```
    ///
    /// 위의 7581.3115 에서 **0.0029mm 더 좁아졌고 스텝 수는 11 로 같다.**
    ///
    /// 바뀐 이유는 하나다: 시간 제한에 걸린 풀이 3회(`interrupted: 3`)가 예전에는
    /// 신뢰 영역을 **키우는** 대응을 받았고, 이제는 줄이는 대응을 받는다
    /// (`shrinks: 3`). 시간 초과는 "모델이 무거웠다" 는 뜻이라 키우면 반드시 또
    /// 넘긴다 — 줄이는 쪽이 맞고, 실측도 손해가 아니라 미세한 이득이었다.
    ///
    /// 즉 **"실패도 시간 초과도 없는 실행"** 에서만 결과가 비트 단위로 같다.
    /// 그 조건은 `schedule_without_failures_behaves_exactly_as_before` 가 지킨다.
    #[test]
    #[ignore]
    fn lp_compaction_on_the_real_marker() {
        use crate::nfp_lbf::{NfpLbfConfig, build_best_initial_nest};
        use jagua_rs::probs::spp::entities::SPProblem;
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        let path =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json");
        let (ext, _) = crate::util::io::read_spp_input(&path).expect(crate::dmsg!("wsjk156.json 읽기"));
        let c = crate::config::DEFAULT_SWARM_CONFIG;
        let imp = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        let instance =
            jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect(crate::dmsg!("wsjk156 import"));

        let nfp_cfg = NfpLbfConfig {
            n_orderings: env_usize("LP_MEASURE_ORDERINGS", 3),
            cache_budget: Duration::from_secs(10),
            total_budget: Duration::from_secs(900),
            concave_simplify_tolerance: 0.02,
            max_candidate_probes: env_usize("LP_MEASURE_PROBES", 1024),
            ..NfpLbfConfig::default()
        };
        let mut seed_rng = Xoshiro256PlusPlus::seed_from_u64(0);
        let nest = build_best_initial_nest(&instance, &nfp_cfg, &mut seed_rng).expect(crate::dmsg!("초기 배치"));

        let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
        prob.restore(&nest.solution);
        let mut sep = Separator::new(
            instance.clone(),
            prob,
            Xoshiro256PlusPlus::seed_from_u64(1),
            c.expl_cfg.separator_config,
        );

        let solver = crate::lp_solver::LpSolverConfig {
            time_limit_ms: env_usize(
                "LP_MEASURE_TIME_LIMIT_MS",
                crate::lp_solver::DEFAULT_LP_TIME_LIMIT_MS as usize,
            ) as u64,
            ..crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG
        };

        let width_before = sep.prob.strip_width();
        let t = std::time::Instant::now();
        let applied = lp_compaction_pass_with_solver(
            &mut sep,
            &crate::config::DEFAULT_POLISH_CONFIG.lp,
            &solver,
            &BasicTerminator::new(),
        );
        let elapsed = t.elapsed();
        let width_after = sep.prob.strip_width();

        println!(
            "[측정] wsjk156  폭 {width_before:.4} -> {width_after:.4} \
             ({:+.4}%),  적용 {applied} 스텝,  {} ms,  제한 {} ms",
            (width_after / width_before - 1.0) * 100.0,
            elapsed.as_millis(),
            solver.time_limit_ms,
        );

        // 압축이 배치를 망가뜨리지 않았다는 것만 자동으로 지킨다. 폭 자체는
        // 인스턴스·초기배치에 딸린 값이라 상수로 박으면 곧 거짓이 된다.
        assert!(
            width_after <= width_before + 1.0e-4,
            "압축이 폭을 늘렸다: {width_before} -> {width_after}"
        );
        assert!(
            sep.prob.layout().is_collision_free(),
            "압축 후 배치가 유효해야 한다"
        );
    }

    // ── 반복 스케줄 전이 규칙 ────────────────────────────────────────────
    //
    // 이 규칙이 이 모듈에서 가장 틀리기 쉽다. LP 를 풀지 않고 표로 단정한다.

    use super::{IterOutcome, MAX_STALL, MIN_TRUST_LEVEL, N_TRUST_LEVELS, Schedule};

    /// 결과 목록을 차례로 먹여 마지막 상태(또는 종료)를 돌려준다.
    fn run(outcomes: &[IterOutcome]) -> (Vec<Schedule>, bool) {
        let mut sched = Schedule::START;
        let mut seen = vec![sched];
        for &o in outcomes {
            match sched.advance(o) {
                Some(next) => {
                    sched = next;
                    seen.push(sched);
                }
                None => return (seen, true),
            }
        }
        (seen, false)
    }

    /// 수치 실패가 하나도 없으면 무개선마다 level 이 1씩 오르고,
    /// [`N_TRUST_LEVELS`] 에 닿으면 패스가 끝난다.
    #[test]
    fn schedule_without_failures_grows_one_level_at_a_time() {
        let n = N_TRUST_LEVELS as usize;
        let (seen, ended) = run(&vec![IterOutcome::NoImprovement; n]);
        assert!(ended, "무개선 {n} 회면 끝나야 한다");
        let levels: Vec<i32> = seen.iter().map(|s| s.level).collect();
        assert_eq!(
            levels,
            (0..N_TRUST_LEVELS as i32).collect::<Vec<_>>(),
            "level 은 1씩만 올라야 한다"
        );
        // `stall` 상한이 이 경로에 끼어들면 안 된다 — 끼어들면 기존 동작이 바뀐다.
        assert!(
            (N_TRUST_LEVELS as usize) < MAX_STALL,
            "MAX_STALL 이 N_TRUST_LEVELS 보다 작거나 같으면 무개선 경로의 종료 시점이 바뀐다"
        );
    }

    /// 채택되면 가장 촘촘한 단계로 되돌아간다. 다만 못 푼 단계가 있었다면 그 **아래**에서
    /// 다시 시작한다 — 0 으로 돌아가 같은 초과를 되풀이하지 않는다.
    #[test]
    fn schedule_resets_to_the_finest_solvable_level_after_any_improvement() {
        let mut grown = Schedule::START;
        for o in [IterOutcome::NoImprovement, IterOutcome::NoImprovement] {
            grown = grown.advance(o).expect(crate::dmsg!("아직 끝나면 안 된다"));
        }
        assert_ne!(grown, Schedule::START, "전제: 상태가 움직여 있어야 한다");
        assert_eq!(
            grown.advance(IterOutcome::Applied(1.0)),
            Some(Schedule::START)
        );

        let mut shrunk = Schedule::START;
        for o in [IterOutcome::Intractable, IterOutcome::Intractable] {
            shrunk = shrunk.advance(o).expect(crate::dmsg!("아직 끝나면 안 된다"));
        }
        let after = shrunk
            .advance(IterOutcome::Applied(1.0))
            .expect(crate::dmsg!("채택은 패스를 끝내지 않는다"));
        assert_eq!((after.level, after.stall), (-2, 0));
        assert_eq!(after.ceiling, -1, "못 푼 단계(-1)는 기억한다");
    }

    /// **못 푼 단계를 기억한다.** `L2 무개선 → L3 초과 → L2 무개선 → L3 초과 …` 의
    /// 꼬리가 [`MAX_STALL`] 까지 되풀이되지 않는다.
    #[test]
    fn schedule_does_not_regrow_into_a_level_it_could_not_solve() {
        // L0·L1 무개선 → L2 초과 → L1 은 이미 무개선이었으므로 끝.
        let (seen, ended) = run(&[
            IterOutcome::NoImprovement,
            IterOutcome::NoImprovement,
            IterOutcome::Intractable,
        ]);
        assert!(ended, "이미 무개선이던 단계로는 내려가지 않는다");
        assert_eq!(
            seen.iter().map(|s| s.level).collect::<Vec<_>>(),
            vec![0, 1, 2]
        );

        // L0 초과 → L-1 무개선 → 다시 L0 로 키우지 않고 끝.
        let (seen, ended) = run(&[IterOutcome::Intractable, IterOutcome::NoImprovement]);
        assert!(ended, "못 푼 단계로 다시 키우지 않는다");
        assert_eq!(
            seen.iter().map(|s| s.level).collect::<Vec<_>>(),
            vec![0, -1]
        );

        // 채택 뒤에도 천장은 남는다: L1 초과를 겪었으면 L0 무개선에서 끝난다.
        let (seen, ended) = run(&[IterOutcome::NoImprovement, IterOutcome::Intractable]);
        assert!(ended);
        assert_eq!(seen.last().map(|s| s.level), Some(1));
        let capped = Schedule {
            ceiling: 1,
            ..Schedule::START
        };
        let after = capped
            .advance(IterOutcome::Applied(1.0))
            .expect(crate::dmsg!("채택은 패스를 끝내지 않는다"));
        assert_eq!((after.level, after.ceiling), (0, 1));
        assert_eq!(after.advance(IterOutcome::NoImprovement), None);
    }

    /// **이 수정의 핵심.** 못 푼 경우에는 신뢰 영역을 키우지 **않는다** — 줄인다.
    ///
    /// 키우면 `clearance = trust x 2.83` 이 같이 커져 접촉 행이 폭발하고,
    /// 선형종속 행이 늘어 단체법 기저가 더 잘 퇴화한다. 실패에 대한 대응이
    /// 실패를 악화시키던 것이 원래 결함이었다.
    #[test]
    fn schedule_shrinks_the_trust_region_when_the_model_cannot_be_solved() {
        let (seen, _) = run(&[IterOutcome::Intractable; 3]);
        let levels: Vec<i32> = seen.iter().map(|s| s.level).collect();
        assert_eq!(
            levels,
            vec![0, -1, -2, -3],
            "실패마다 한 단계씩 줄어야 한다"
        );
        assert!(
            levels.windows(2).all(|w| w[1] < w[0]),
            "실패에 신뢰 영역이 커지면 안 된다"
        );
    }

    /// 하한에 닿으면 끝낸다. 같은 모델을 다시 풀어봐야 결정적으로 같은 실패다.
    #[test]
    fn schedule_stops_once_the_model_cannot_shrink_further() {
        let n = (-MIN_TRUST_LEVEL) as usize;
        let (seen, ended) = run(&vec![IterOutcome::Intractable; n + 1]);
        assert!(ended, "하한을 지나면 패스를 끝내야 한다");
        assert_eq!(
            seen.last().map(|s| s.level),
            Some(MIN_TRUST_LEVEL),
            "하한 아래로 내려가면 안 된다"
        );
    }

    /// 키우기와 줄이기가 번갈아 나와도 예산을 무한정 태우지 않는다.
    ///
    /// 이 보루가 없으면 `NoImprovement -> Intractable -> ...` 가 level 을 0 근처에서
    /// 왕복시켜 두 상한 중 어느 쪽에도 닿지 않는다.
    #[test]
    fn schedule_terminates_when_growing_and_shrinking_alternate() {
        let alternating: Vec<IterOutcome> = (0..MAX_STALL * 4)
            .map(|i| {
                if i % 2 == 0 {
                    IterOutcome::NoImprovement
                } else {
                    IterOutcome::Intractable
                }
            })
            .collect();
        let (seen, ended) = run(&alternating);
        assert!(ended, "진동해도 반드시 끝나야 한다");
        assert!(
            seen.len() <= MAX_STALL + 1,
            "무채택 연속 {} 회를 넘겨 돌았다",
            MAX_STALL
        );
    }

    /// 어떤 결과 조합에서도 패스는 유한하게 끝난다.
    #[test]
    fn schedule_always_terminates_for_any_sequence_of_non_improvements() {
        let kinds = [IterOutcome::NoImprovement, IterOutcome::Intractable];
        // 무채택 결과만 나오는 모든 길이-9 조합을 전수로 돌린다.
        for mask in 0..(1u32 << MAX_STALL) {
            let seq: Vec<IterOutcome> = (0..MAX_STALL)
                .map(|i| kinds[((mask >> i) & 1) as usize])
                .collect();
            let (seen, ended) = run(&seq);
            assert!(ended, "무채택만 {MAX_STALL} 회면 끝나야 한다: {seen:?}");
            assert!(
                seen.iter().all(|s| s.level >= MIN_TRUST_LEVEL),
                "하한을 뚫었다: {seen:?}"
            );
            assert!(
                seen.iter().all(|s| s.level < N_TRUST_LEVELS as i32),
                "상한을 뚫었다: {seen:?}"
            );
        }
    }

    /// 중단 요청·백엔드 부재는 즉시 끝낸다. 신뢰 영역을 바꿔도 달라지지 않는 상황이라,
    /// 예전처럼 5단계를 태우고 끝나면 그만큼이 순수한 낭비다.
    #[test]
    fn schedule_aborts_immediately() {
        assert_eq!(Schedule::START.advance(IterOutcome::Abort), None);
        let deep = Schedule {
            level: 3,
            stall: 1,
            ..Schedule::START
        };
        assert_eq!(deep.advance(IterOutcome::Abort), None);
    }

    /// 신뢰 영역 안쪽 해를 적용하지 못했으면 어느 단계에서든 패스를 끝낸다 —
    /// 키운 모델의 최적해도 같은 해라, 무개선처럼 다음 단계로 올라가면 같은 실패만 반복한다.
    #[test]
    fn schedule_ends_the_pass_when_an_interior_step_cannot_be_applied() {
        assert_eq!(Schedule::START.advance(IterOutcome::Blocked), None);
        let shrunk = Schedule {
            level: -1,
            stall: 1,
            ceiling: 0,
            ..Schedule::START
        };
        assert_eq!(shrunk.advance(IterOutcome::Blocked), None);
    }

    /// 0.9·trust 가 경계다. 이동량 하나라도, 또는 폭 감소량이 거기에 닿으면 신뢰 영역이
    /// 해를 붙잡고 있었을 수 있으므로 «키워도 같은 해» 라고 말할 수 없다.
    #[test]
    fn a_step_is_interior_only_when_nothing_reaches_the_trust_bound() {
        let mut keys: slotmap::SlotMap<PItemKey, ()> = slotmap::SlotMap::with_key();
        let mut step = |dx: f32, dy: f32| (keys.insert(()), (dx, dy));
        let trust = 10.0;
        let small = [step(2.0, -3.0), step(-8.9, 0.0)];
        assert!(step_is_interior(&small, 0.4, trust));
        // 폭이 신뢰 영역만큼 줄었다 = 폭 하한에 걸렸다.
        assert!(!step_is_interior(&small, 9.5, trust));
        // 한 아이템이 상자 경계에 있다 (음의 방향 포함).
        assert!(!step_is_interior(
            &[step(2.0, -3.0), step(0.0, -10.0)],
            0.4,
            trust
        ));
        assert!(!step_is_interior(&[step(9.0, 0.0)], 0.4, trust));
        // 움직일 것이 없어도 폭만으로 판정한다.
        assert!(step_is_interior(&[], 0.0, trust));
    }

    // ── 접촉 행 중복 제거 ────────────────────────────────────────────────

    use super::{ContactRow, NORMAL_QUANT, contact_rhs, dedup_by_direction, normal_key};

    fn row(n: (f32, f32), dist: f32) -> ContactRow {
        ContactRow {
            a: 0,
            b: 1,
            n,
            dist,
        }
    }

    /// **중복 제거의 근거.** 우변이 dist 에 대해 단조 비증가여야, 법선이 같은 행들
    /// 중 dist 가 가장 작은 것이 가장 강한 제약이 된다. 이 성질이 깨지면
    /// `dedup_by_direction` 이 더 약한 행을 남기고 겹침을 허용하게 된다.
    #[test]
    fn contact_rhs_is_monotone_in_distance() {
        // 세 구간(dist<=0.05, 0.05..0.5, >=0.5)을 모두 지나도록 훑는다.
        let mut prev = contact_rhs(0.0);
        let mut d = 0.0f32;
        while d <= 5.0 {
            let cur = contact_rhs(d);
            assert!(
                cur <= prev + 1e-12,
                "dist {d} 에서 우변이 커졌다: {prev} -> {cur} (가까운 행이 더 약해진다)"
            );
            prev = cur;
            d += 0.001;
        }
        // 구간 경계값도 직접 확인한다.
        assert_eq!(contact_rhs(0.0), 0.0);
        assert_eq!(
            contact_rhs(0.05),
            0.0,
            "0.05 이하는 기존 간격 유지만 요구한다"
        );
        assert!(contact_rhs(0.2) < 0.0);
        assert!(contact_rhs(1.0) < contact_rhs(0.2));
    }

    /// 같은 방향은 접고, **가장 가까운(=가장 강한)** 행을 남긴다.
    #[test]
    fn dedup_keeps_the_strictest_row_per_direction() {
        // dist 오름차순이 이 함수의 전제다.
        let input = vec![
            row((1.0, 0.0), 0.10),
            row((1.0, 0.0), 0.30),
            row((0.0, 1.0), 0.40),
            row((1.0, 0.0), 0.90),
            row((0.0, 1.0), 1.20),
        ];
        let kept = dedup_by_direction(input, 8);
        assert_eq!(kept.len(), 2, "방향은 둘뿐이다");
        assert_eq!(kept[0].n, (1.0, 0.0));
        assert_eq!(kept[0].dist, 0.10, "각 방향의 최소 dist 를 남겨야 한다");
        assert_eq!(kept[1].n, (0.0, 1.0));
        assert_eq!(kept[1].dist, 0.40);
        // 남은 행이 실제로 더 강한 제약인지 우변으로 확인한다.
        assert!(contact_rhs(0.10) >= contact_rhs(0.30));
    }

    /// 반대 방향 법선은 **접으면 안 된다** — `n` 과 `-n` 은 서로 다른 제약이다.
    /// (한쪽만 남기면 조각이 반대편으로 통과할 수 있다.)
    #[test]
    fn dedup_never_folds_opposite_normals() {
        let kept = dedup_by_direction(vec![row((1.0, 0.0), 0.1), row((-1.0, 0.0), 0.2)], 8);
        assert_eq!(kept.len(), 2, "n 과 -n 은 다른 제약이다");
        assert_ne!(normal_key((1.0, 0.0)), normal_key((-1.0, 0.0)));
    }

    /// 상한은 "가장 가까운 N행" 이 아니라 "가장 가까운 **서로 다른 방향** N개" 다.
    /// 같은 예산으로 더 많은 방향을 덮는 것이 이 변경의 부수 효과다.
    #[test]
    fn dedup_spends_the_cap_on_distinct_directions() {
        // 같은 방향 5개가 앞에 오고 다른 방향 2개가 뒤에 온다.
        let mut input: Vec<ContactRow> = (0..5)
            .map(|i| row((1.0, 0.0), 0.01 * (i + 1) as f32))
            .collect();
        input.push(row((0.0, 1.0), 0.5));
        input.push(row((0.6, 0.8), 0.6));

        let kept = dedup_by_direction(input, 2);
        assert_eq!(kept.len(), 2);
        // 예전에는 같은 방향 두 개를 담았을 자리다.
        assert_eq!(kept[0].n, (1.0, 0.0));
        assert_eq!(kept[1].n, (0.0, 1.0), "두 번째 자리는 다른 방향이 가져간다");
    }

    #[test]
    fn dedup_handles_degenerate_caps_and_inputs() {
        assert!(dedup_by_direction(vec![], 8).is_empty());
        assert!(dedup_by_direction(vec![row((1.0, 0.0), 0.1)], 0).is_empty());
        assert_eq!(dedup_by_direction(vec![row((1.0, 0.0), 0.1)], 1).len(), 1);
    }

    /// 양자화 격자의 해상도를 못박는다. 너무 성기면 서로 다른 방향이 접혀
    /// 선형화가 헐거워지고, 너무 촘촘하면 접으려던 중복이 안 접힌다.
    #[test]
    fn normal_key_folds_near_identical_directions_only() {
        // 정규화 반올림 수준의 차이(1e-6)는 접힌다 — 이게 접으려는 대상이다.
        assert_eq!(normal_key((1.0, 0.0)), normal_key((1.0 - 1e-6, 1e-6)));
        // 격자 한 칸(1e-4)을 넘는 차이는 구분된다.
        let step = 2.0 / NORMAL_QUANT;
        assert_ne!(normal_key((1.0, 0.0)), normal_key((1.0, step)));
        // 육안으로 명백히 다른 방향은 당연히 구분된다.
        assert_ne!(normal_key((1.0, 0.0)), normal_key((0.0, 1.0)));
        assert_ne!(normal_key((0.6, 0.8)), normal_key((0.8, 0.6)));
    }

    /// **실제 기하로 메커니즘을 증명한다.**
    ///
    /// 단순화를 거친 의류 조각은 거의 일직선인 모서리에 정점이 여러 개 놓인다.
    /// 그 조각이 다른 조각의 직선 모서리에 붙으면, 정점 하나마다 **법선이 같고
    /// 우변만 다른 행**이 생긴다 — 선형종속 행이고 단체법 기저를 퇴화시킨다.
    ///
    /// 오른쪽 변이 10개 정점으로 잘게 나뉜 조각 A 를 조각 B 의 왼쪽 변에 붙인다.
    #[test]
    fn dedup_collapses_a_run_of_vertices_on_one_straight_edge() {
        use std::sync::Arc;

        // A: x=4 인 오른쪽 변이 정점 10개로 나뉜 다각형.
        const N_SUBDIV: usize = 10;
        let mut pts = vec![Point(0.0, 0.0)];
        for i in 0..N_SUBDIV {
            pts.push(Point(4.0, i as f32 * 10.0 / (N_SUBDIV - 1) as f32));
        }
        pts.push(Point(0.0, 10.0));
        let a_shape = Arc::new(SPolygon::new(pts).expect(crate::dmsg!("단순 다각형")));
        // B: A 의 오른쪽 변에 딱 붙는 사각형.
        let b_shape = rect_poly(4.0, 0.0, 4.0, 10.0);

        let mut keys: slotmap::SlotMap<PItemKey, ()> = slotmap::SlotMap::with_key();
        let items: Vec<LpItem> = [a_shape, b_shape]
            .into_iter()
            .map(|shape| LpItem {
                pk: keys.insert(()),
                group_representative: None,
                bbox: shape.bbox(),
                shape,
                frozen: false,
            })
            .collect();

        // 상한을 넉넉히 열어 둔다 — 상한이 잘라서 줄어든 것과 구분하기 위해서다.
        let cfg = LpCompactionConfig {
            max_constraints_per_pair: 64,
            ..crate::config::DEFAULT_POLISH_CONFIG.lp
        };
        let mut rows = vec![];
        collect_pair_rows(&items, 0, 1, 1.0, &cfg, &mut rows);

        assert!(!rows.is_empty(), "붙어 있으므로 접촉 행이 나와야 한다");
        // 남은 행의 방향은 서로 달라야 한다 — 그게 중복 제거의 정의다.
        let keys_seen: Vec<(i32, i32)> = rows.iter().map(|r| normal_key(r.n)).collect();
        let mut uniq = keys_seen.clone();
        uniq.sort_unstable();
        uniq.dedup();
        assert_eq!(
            keys_seen.len(),
            uniq.len(),
            "같은 방향의 행이 남아 있다: {keys_seen:?}"
        );
        // 세분된 정점 10개가 B 의 같은 왼쪽 변에 붙어 있으므로, 중복 제거가 없으면
        // 그 방향만으로 10행이 나온다. 방향 수는 그보다 훨씬 적어야 한다.
        assert!(
            rows.len() < N_SUBDIV,
            "정점 {N_SUBDIV} 개가 한 모서리에 붙었는데 행이 {} 개다 — 접히지 않았다",
            rows.len()
        );
    }

    /// **압축된 상태에서의 중복 제거 A/B.** 이것이 결판을 내는 측정이다.
    ///
    /// # 왜 별도 하네스인가
    ///
    /// `lp_compaction_on_the_real_marker` 는 NFP-LBF 시드(폭 7583mm) 위에서 잰다.
    /// 그런데 `Singular matrix` 는 훨씬 압축된 상태(보고된 폭 6379mm)에서 났다.
    /// 조각이 촘촘히 맞물릴수록 쌍당 후보가 많아져 상한이 실제로 물리고, 그때
    /// 가장 가까운 8행이 한 모서리에 몰린 중복일 가능성이 커진다 — 중복 제거가
    /// 의미를 갖는 구간이 거기다.
    ///
    /// # 오염을 막는 법
    ///
    /// 탐색·압축을 **한 번만** 돌려 시작 레이아웃을 하나 만들고, 그것을 복원해
    /// 두 팔에 똑같이 먹인다. 팔마다 새로 만들면 속도 차이가 배치 차이로 번져
    /// 무엇을 재는지 알 수 없게 된다.
    ///
    /// ```powershell
    /// $env:SWARM_INPUT='C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json'
    /// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
    ///   --lib dedup_ab_on_a_compressed_layout -- --ignored --nocapture --test-threads=1
    /// ```
    #[test]
    #[ignore]
    fn dedup_ab_on_a_compressed_layout() {
        use crate::optimizer::optimize;
        use jagua_rs::probs::spp::entities::SPProblem;
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;
        use std::sync::atomic::Ordering as AtomicOrdering;

        let path = match std::env::var("SWARM_INPUT") {
            Ok(p) => std::path::PathBuf::from(p),
            Err(_) => {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json")
            }
        };
        let (ext, _) = crate::util::io::read_spp_input(&path).expect(crate::dmsg!("입력 읽기"));
        let mut c = crate::config::DEFAULT_SWARM_CONFIG;
        let imp = crate::util::io::new_importer(c.cde_config, c.poly_simpl_tolerance, c.min_item_separation, c.narrow_concavity_cutoff_ratio);
        let instance = jagua_rs::probs::spp::io::import_instance(&imp, &ext).expect(crate::dmsg!("import"));

        // ── 1. 시작 레이아웃을 **한 번만** 만든다 (polish 는 끈다) ──
        c.expl_cfg.time_limit = Duration::from_secs(env_usize("AB_EXPLORE_SECS", 30) as u64);
        c.cmpr_cfg.time_limit = Duration::from_secs(env_usize("AB_COMPRESS_SECS", 15) as u64);
        c.polish_cfg.enabled = false;
        c.polish_cfg.time_limit = Duration::ZERO;
        // 결정성을 위해 워커 1개.
        c.expl_cfg.separator_config.n_workers = 1;
        c.cmpr_cfg.separator_config.n_workers = 1;

        let mut term = BasicTerminator::new();
        let base = optimize(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(0),
            None,
            &mut term,
            &c.expl_cfg,
            &c.cmpr_cfg,
            &c.polish_cfg,
            None,
            c.lbf_x_multiplier,
            c.lbf_y_multiplier,
            &c.lbf_raster,
        );
        println!(
            "\n[A/B] 시작 레이아웃  폭 {:.4}  밀도 {:.3}%  ({})",
            base.strip_width(),
            base.density() * 100.0,
            path.display()
        );

        // ── 2. 같은 레이아웃에 두 팔을 먹인다 ──
        let mut results = vec![];
        for (label, on) in [("중복제거 끔", false), ("중복제거 켬", true)] {
            DEDUP_ENABLED.store(on, AtomicOrdering::Relaxed);

            let mut prob = SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
            prob.restore(&base);
            let mut sep = Separator::new(
                instance.clone(),
                prob,
                Xoshiro256PlusPlus::seed_from_u64(1),
                c.expl_cfg.separator_config,
            );
            let before = sep.prob.strip_width();
            let t = std::time::Instant::now();
            let applied = lp_compaction_pass_with_solver(
                &mut sep,
                &crate::config::DEFAULT_POLISH_CONFIG.lp,
                &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
                &BasicTerminator::new(),
            );
            let after = sep.prob.strip_width();
            println!(
                "[A/B] {label}  폭 {before:.4} -> {after:.4}  적용 {applied} 스텝  {} ms",
                t.elapsed().as_millis()
            );
            assert!(
                sep.prob.layout().is_collision_free(),
                "{label}: 압축 후 배치가 유효해야 한다"
            );
            results.push((label, after, applied));
        }
        // 기본값으로 되돌린다 — 같은 프로세스의 다른 테스트에 새지 않게.
        DEDUP_ENABLED.store(true, AtomicOrdering::Relaxed);

        let (off, on) = (&results[0], &results[1]);
        println!(
            "\n[A/B] 판정  스텝 {} -> {},  폭 {:+.4}mm\n",
            off.2,
            on.2,
            on.1 - off.1,
        );
    }

    // ── 선분 기준 접촉 행 ────────────────────────────────────────────────

    use super::{CONTACT_SIDE_TOLERANCE, contact_row, contact_row_line};

    /// B = [0,10]² (CCW). 아랫변 (0,0)→(10,0), 오른변 (10,0)→(10,10), 윗변 (10,10)→(0,10).
    const SQ: [Point; 4] = [
        Point(0.0, 0.0),
        Point(10.0, 0.0),
        Point(10.0, 10.0),
        Point(0.0, 10.0),
    ];

    /// 마주보는 정점: 바깥 법선과 수직 거리.
    #[test]
    fn facing_vertex_gets_the_outward_normal_and_the_segment_distance() {
        // 오른변 (10,0)→(10,10) 의 오른쪽 0.3mm.
        let r = contact_row(0, 1, Point(10.3, 4.0), SQ[1], SQ[2], SQ[3]).expect(crate::dmsg!("마주본다"));
        assert!(
            (r.n.0 - 1.0).abs() < 1e-6 && r.n.1.abs() < 1e-6,
            "{:?}",
            r.n
        );
        assert!((r.dist - 0.3).abs() < 1e-5);
        // 정확히 맞닿았거나 반올림으로 아주 조금 들어간 정점은 거리 0 의 접촉이다.
        let touching = contact_row(
            0,
            1,
            Point(10.0 - 0.5 * CONTACT_SIDE_TOLERANCE, 4.0),
            SQ[1],
            SQ[2],
            SQ[3],
        )
        .expect(crate::dmsg!("접촉"));
        assert_eq!(touching.dist, 0.0);
        assert!((touching.n.0 - 1.0).abs() < 1e-6);
    }

    /// **이 수정의 핵심.** 모서리의 연장선 근처에 있을 뿐인 정점에는 그 모서리의
    /// 법선으로 된 행이 생기지 않는다. 옛 방식은 여기서 거리 ≈ 0 의 가짜 행을 만들었다.
    #[test]
    fn a_vertex_beside_the_extension_of_an_edge_is_not_pinned_by_that_edge() {
        // A 의 정점이 B 오른변 바로 옆(x=10.05), 높이는 B 윗변 직선에서 0.02mm 위.
        // 윗변 (10,10)→(0,10) 의 발은 선분 밖(t<0)이다.
        let v = Point(10.05, 10.02);
        assert!(
            contact_row(0, 1, v, SQ[2], SQ[3], SQ[0]).is_none(),
            "시작점 쪽 꼭짓점 영역은 앞 모서리가 맡는다"
        );
        // 옛 방식은 법선 (0,1), 거리 0.02 — 아래로 0.02mm 도 못 내려가게 묶는다.
        let legacy = contact_row_line(0, 1, v, SQ[2], SQ[3], Point(5.0, 5.0)).expect(crate::dmsg!("옛 행"));
        assert!(legacy.dist < 0.03 && legacy.n.1.abs() > 0.99, "{legacy:?}");

        // 같은 정점은 오른변의 끝점 (10,10) 의 꼭짓점 영역에 있다 → 꼭짓점 행 하나.
        let corner = contact_row(0, 1, v, SQ[1], SQ[2], SQ[3]).expect(crate::dmsg!("꼭짓점 영역"));
        let d = (0.05f32 * 0.05 + 0.02 * 0.02).sqrt();
        assert!((corner.dist - d).abs() < 1e-5, "{corner:?}");
        assert!(corner.n.0 > 0.9 && corner.n.1 > 0.3, "{corner:?}");
    }

    /// 발이 선분 안인데 재료 쪽에 있는 정점(= 조각 건너편)은 행을 만들지 않는다.
    #[test]
    fn a_vertex_on_the_material_side_of_an_edge_gets_no_row_from_it() {
        // B 왼쪽 바깥 (-3, 5): 오른변 (10,0)→(10,10) 기준으로는 재료 쪽 13mm.
        assert!(contact_row(0, 1, Point(-3.0, 5.0), SQ[1], SQ[2], SQ[3]).is_none());
        // 옛 방식은 법선을 뒤집어 «왼쪽에 머물러라» 는 행을 만들었다.
        assert!(contact_row_line(0, 1, Point(-3.0, 5.0), SQ[1], SQ[2], Point(5.0, 5.0)).is_some());
    }

    /// 오목 꼭짓점에는 바깥 꼭짓점 영역이 없다.
    #[test]
    fn reflex_corners_produce_no_vertex_row() {
        // (0,0)→(4,0) 다음이 (4,0)→(4,-4): CCW 에서 우회전 = 오목.
        let (p, q, r) = (Point(0.0, 0.0), Point(4.0, 0.0), Point(4.0, -4.0));
        assert!(contact_row(0, 1, Point(5.0, -1.0), p, q, r).is_none());
    }

    /// 꼭짓점끼리 정확히 맞닿으면(t == 1) 모서리 분기가 거리 0 의 접촉으로 잡는다.
    #[test]
    fn coincident_corners_are_a_zero_distance_contact() {
        let r = contact_row(0, 1, SQ[2], SQ[1], SQ[2], SQ[3]).expect(crate::dmsg!("접촉"));
        assert_eq!(r.dist, 0.0);
        assert!((r.n.0 - 1.0).abs() < 1e-6 && r.n.1.abs() < 1e-6);
    }

    /// 접촉 행은 현재 배치(d = 0)에서 항상 성립해야 한다 — 그래야 LP 가 언제나 실행 가능하다.
    #[test]
    fn every_row_holds_at_zero_displacement() {
        let items = super::parallel_regression::fixture();
        let cfg = crate::config::DEFAULT_POLISH_CONFIG.lp;
        let rows = build_contact_rows(&items, 12.0, &cfg, &BasicTerminator::new()).unwrap();
        assert!(!rows.is_empty());
        for r in &rows {
            assert!(r.dist >= 0.0 && r.dist.is_finite(), "{r:?}");
            assert!(
                ((r.n.0 * r.n.0 + r.n.1 * r.n.1).sqrt() - 1.0).abs() < 1e-4,
                "{r:?}"
            );
            assert!(contact_rhs(r.dist) <= 0.0, "{r:?}");
        }
    }

    /// **저장된 압축 해 위에서 접촉 행·스케줄 A/B.**
    ///
    /// 탐색·압축은 시간 종료라 같은 시드로도 배치가 재현되지 않는다. 그래서 FullBench 가
    /// 남긴 `*.Compress.json` 을 그대로 읽어 모든 팔에 같은 시작 배치를 먹인다.
    ///
    /// 노브: `AB_SOLUTION`(필수, 해 JSON), `AB_INPUT`(기본 wsjk156), `AB_OVERLAP_MM`(기본 0.4),
    /// `AB_POLISH_SECS`(기본 60; 0 이면 Polish 팔 생략), `AB_LP_PASS`(기본 1; 0 이면 LP 단독 팔 생략),
    /// `AB_ARMS`(기본 `31,30,28,20,4,0`; 돌릴 팔만 쉼표로), `AB_TRACE`(1 이면 반복별 추적).
    ///
    /// ```powershell
    /// $env:AB_SOLUTION='...\seed1.Compress.json'
    /// cargo test -p swarm-core --release --features "simd,capmods,adaptive,structure-first" `
    ///   --lib contact_rows_ab_on_a_saved_layout -- --ignored --nocapture --test-threads=1
    /// ```
    #[test]
    #[ignore]
    fn contact_rows_ab_on_a_saved_layout() {
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;
        use std::sync::atomic::Ordering as AtomicOrdering;

        let Ok(solution_path) = std::env::var("AB_SOLUTION") else {
            println!("[A/B] AB_SOLUTION 이 없어 건너뛴다");
            return;
        };
        // AB_TRACE=1 이면 반복별 (단계, 결과, 폭, 누적 백트래킹, 풀이 시간)을 찍는다.
        // 패스 루프는 반복별 기록을 남기지 않으므로 여기서 한 번 더 돈다.
        let trace = env_usize("AB_TRACE", 0) != 0;
        let input = std::env::var("AB_INPUT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/input/wsjk156.json")
            });
        let overlap: f32 = std::env::var("AB_OVERLAP_MM")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0.4);
        let polish_secs = env_usize("AB_POLISH_SECS", 60) as u64;
        let lp_pass = env_usize("AB_LP_PASS", 1) != 0;

        let (ext, _) = crate::util::io::read_spp_input(&input).expect(crate::dmsg!("입력 읽기"));
        let c = crate::config::DEFAULT_SWARM_CONFIG;
        let instance = crate::util::io::import_spp_instance(
            &ext,
            c.cde_config,
            c.poly_simpl_tolerance,
            (overlap > 0.0).then_some(-overlap),
            c.narrow_concavity_cutoff_ratio,
        )
        .expect(crate::dmsg!("import"));
        let ext_sol: jagua_rs::probs::spp::io::ext_repr::ExtSPSolution =
            serde_json::from_str(&std::fs::read_to_string(&solution_path).expect(crate::dmsg!("해 읽기")))
                .expect(crate::dmsg!("해 JSON"));
        let start = crate::util::frozen::import_solution(&instance, &ext_sol);
        let start_sol = start.save();
        println!(
            "\n[A/B] 시작 배치  폭 {:.4}  밀도 {:.3}%  ({solution_path})",
            start_sol.strip_width(),
            start_sol.density() * 100.0
        );

        let new_sep = || {
            let mut sep_cfg = c.cmpr_cfg.separator_config;
            sep_cfg.n_workers = 3;
            Separator::new(
                instance.clone(),
                start.clone(),
                Xoshiro256PlusPlus::seed_from_u64(1),
                sep_cfg,
            )
        };
        {
            let sep = new_sep();
            assert_eq!(sep.ct.get_total_loss(), 0.0, "시작 배치에 충돌이 있다");
            assert!(
                sep.solution_is_certified(&start_sol, &start_sol),
                "시작 배치 인증 실패"
            );
        }

        // 팔 번호는 [`LEGACY_AB`] 비트 마스크다 — 0 이 현재 동작.
        let arms = std::env::var("AB_ARMS").unwrap_or_else(|_| "31,30,28,20,4,0".into());
        for (label, legacy) in [
            ("옛 행 + 옛 스케줄", 31u8),
            ("새 행 + 옛 스케줄", 30),
            ("새 행 + 새 스케줄", 28),
            ("… + 막힘 종료", 20),
            ("… + 막힘 종료 + 우측 끝 기준", 4),
            ("… + 막힘 종료 + 우측 끝 기준 + 마진 복원", 0),
        ] {
            if !arms.split(',').any(|a| a.trim().parse() == Ok(legacy)) {
                continue;
            }
            LEGACY_AB.store(legacy, AtomicOrdering::Relaxed);
            if trace {
                let mut sep = new_sep();
                let cfg = crate::config::DEFAULT_POLISH_CONFIG.lp;
                let mut sched = Schedule::START;
                for iter in 0..cfg.n_iterations {
                    let trust = sep.prob.strip_width()
                        * cfg.trust_region_ratio
                        * TRUST_GROWTH.powi(sched.level);
                    let t = std::time::Instant::now();
                    let outcome = lp_iteration(
                        &mut sep,
                        &cfg,
                        &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
                        trust,
                        &BasicTerminator::new(),
                    );
                    println!(
                        "[A/B]     {label} iter {iter:2} L{:+} trust {trust:7.2}  {outcome:?}  폭 {:.4}  {:.2}s",
                        sched.level,
                        sep.prob.strip_width(),
                        t.elapsed().as_secs_f32()
                    );
                    match sched.advance(outcome) {
                        Some(next) => sched = next,
                        None => break,
                    }
                }
            }
            if lp_pass {
                let mut sep = new_sep();
                let t = std::time::Instant::now();
                let applied = lp_compaction_pass_with_solver(
                    &mut sep,
                    &crate::config::DEFAULT_POLISH_CONFIG.lp,
                    &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
                    &BasicTerminator::new(),
                );
                let after = sep.prob.save();
                println!(
                    "[A/B] {label}  LP 패스 1회  폭 {:.4} -> {:.4} ({:+.3}mm)  {applied} 스텝  {:.1}s",
                    start_sol.strip_width(),
                    after.strip_width(),
                    after.strip_width() - start_sol.strip_width(),
                    t.elapsed().as_secs_f32()
                );
                assert!(
                    sep.solution_is_certified(&after, &start_sol),
                    "{label}: LP 결과 인증 실패"
                );
            }
            if polish_secs > 0 {
                let mut sep = new_sep();
                let mut polish_cfg = crate::config::DEFAULT_POLISH_CONFIG;
                polish_cfg.time_limit = Duration::from_secs(polish_secs);
                // 회수(reclaim)는 여기서 돌지 않는다 — 폴리시 본체만 잰다.
                let mut term = BasicTerminator::new();
                term.new_timeout(polish_cfg.time_limit);
                let t = std::time::Instant::now();
                let polished = crate::optimizer::polish::polish_phase_with_nfp(
                    &instance,
                    &mut sep,
                    &start_sol,
                    &mut None,
                    &term,
                    &polish_cfg,
                    &crate::config::DEFAULT_NFP_INTEGRATION_CONFIG,
                    &crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
                );
                println!(
                    "[A/B] {label}  Polish {polish_secs}s  폭 {:.4} -> {:.4} ({:+.3}mm, {:+.4}%p)  실제 {:.1}s",
                    start_sol.strip_width(),
                    polished.strip_width(),
                    polished.strip_width() - start_sol.strip_width(),
                    (polished.density() - start_sol.density()) * 100.0,
                    t.elapsed().as_secs_f32()
                );
                assert!(
                    sep.solution_is_certified(&polished, &start_sol),
                    "{label}: Polish 결과 인증 실패"
                );
            }
        }
        LEGACY_AB.store(0, AtomicOrdering::Relaxed);
    }
}

#[cfg(test)]
mod parallel_regression {
    use super::*;
    use crate::util::terminator::BasicTerminator;
    pub(super) fn fixture() -> Vec<LpItem> {
        let mut keys: slotmap::SlotMap<PItemKey, ()> = slotmap::SlotMap::with_key();
        (0..16)
            .map(|i| {
                let vertices = (0..128)
                    .map(|v| {
                        let a = v as f32 * std::f32::consts::TAU / 128.0;
                        Point(
                            (i % 4) as f32 * 10.2 + 5.0 * a.cos(),
                            (i / 4) as f32 * 10.2 + 5.0 * a.sin(),
                        )
                    })
                    .collect();
                let shape = std::sync::Arc::new(SPolygon::new(vertices).unwrap());
                LpItem {
                    pk: keys.insert(()),
                    group_representative: None,
                    bbox: shape.bbox(),
                    shape,
                    frozen: i == 0,
                }
            })
            .collect()
    }
    fn signature(rows: &[ContactRow]) -> Vec<(usize, usize, u32, u32, u32)> {
        rows.iter()
            .map(|r| (r.a, r.b, r.n.0.to_bits(), r.n.1.to_bits(), r.dist.to_bits()))
            .collect()
    }
    #[test]
    fn contact_rows_are_identical_across_thread_counts_and_caps_stop_early() {
        let items = fixture();
        let cfg = crate::config::DEFAULT_POLISH_CONFIG.lp;
        let mut reference = None;
        for workers in [1, 2, 4] {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(workers)
                .build()
                .unwrap();
            let rows = pool
                .install(|| build_contact_rows(&items, 8.0, &cfg, &BasicTerminator::new()))
                .unwrap();
            assert!(rows.len() > 100);
            let current = signature(&rows);
            if let Some(expected) = &reference {
                assert_eq!(&current, expected);
            } else {
                reference = Some(current);
            }
        }
        let mut capped = cfg;
        capped.max_constraints = 1;
        assert!(matches!(
            build_contact_rows(&items, 8.0, &capped, &BasicTerminator::new()),
            Err(ContactBuildError::Limit)
        ));
    }
    #[test]
    fn bounded_direction_selection_matches_full_collect_sort_dedup() {
        let items = fixture();
        for cap in [0, 1, 2, 4, 8, 128] {
            let mut cfg = crate::config::DEFAULT_POLISH_CONFIG.lp;
            cfg.max_constraints_per_pair = cap;
            for a in 0..items.len() {
                for b in 0..items.len() {
                    if a == b {
                        continue;
                    }
                    let mut expected = Vec::new();
                    reference_pair_rows(&items, a, b, 12.0, &cfg, &mut expected);
                    let mut actual = Vec::new();
                    assert!(collect_pair_rows_with_control(
                        &items,
                        a,
                        b,
                        12.0,
                        &cfg,
                        &mut actual,
                        &BasicTerminator::new()
                    ));
                    assert_eq!(
                        signature(&actual),
                        signature(&expected),
                        "cap={cap}, pair=({a},{b})"
                    );
                }
            }
        }
    }
    #[test]
    #[ignore = "isolated contact-generation microbenchmark; run without other builds"]
    fn contact_generation_parallel_benchmark() {
        let items = fixture();
        let cfg = crate::config::DEFAULT_POLISH_CONFIG.lp;
        for workers in [1, 2, 4] {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(workers)
                .build()
                .unwrap();
            pool.install(|| build_contact_rows(&items, 8.0, &cfg, &BasicTerminator::new()))
                .unwrap();
            let mut samples = Vec::new();
            let mut n = 0;
            for _ in 0..9 {
                let start = std::time::Instant::now();
                n = pool
                    .install(|| build_contact_rows(&items, 8.0, &cfg, &BasicTerminator::new()))
                    .unwrap()
                    .len();
                samples.push(start.elapsed().as_secs_f64() * 1000.0);
            }
            samples.sort_by(f64::total_cmp);
            println!(
                "contacts workers={workers} rows={n} median_ms={:.3}",
                samples[4]
            );
        }
    }
    fn reference_pair_rows(
        items: &[LpItem],
        a: usize,
        b: usize,
        clearance: f32,
        cfg: &LpCompactionConfig,
        out: &mut Vec<ContactRow>,
    ) {
        let (sa, sb) = (&items[a].shape, &items[b].shape);
        let b_centroid = sb.centroid();
        let sq_clearance = clearance * clearance;

        let mut candidates: Vec<ContactRow> = vec![];
        let edges: Vec<_> = sb.edge_iter().collect();

        for vi in 0..sa.n_vertices() {
            let v = sa.vertex(vi);
            // b의 bbox에서 clearance 밖에 있는 정점은 볼 필요 없다
            if v.0 < sb.bbox().x_min - clearance
                || v.0 > sb.bbox().x_max + clearance
                || v.1 < sb.bbox().y_min - clearance
                || v.1 > sb.bbox().y_max + clearance
            {
                continue;
            }
            for (i, e) in edges.iter().enumerate() {
                if v.0 < e.x_min() - clearance
                    || v.0 > e.x_max() + clearance
                    || v.1 < e.y_min() - clearance
                    || v.1 > e.y_max() + clearance
                {
                    continue;
                }
                if e.sq_distance_to(&v) > sq_clearance {
                    continue;
                }
                let Some(row) = contact_row_at(a, b, v, &edges, i, b_centroid) else {
                    continue;
                };
                candidates.push(row);
            }
        }

        // 가까운 접촉부터 우선. 같은 방향의 중복을 접고 쌍당 상한까지 자른다
        // (→ 적용 후 CDE 검증이 필수인 이유).
        candidates.sort_by_key(|r| OrderedFloat(r.dist));
        out.extend(dedup_by_direction(candidates, cfg.max_constraints_per_pair));
    }
}
