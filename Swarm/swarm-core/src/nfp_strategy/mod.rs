//! NFP 초기 배치 전략 — 일곱 가지를 설정으로 골라 분기 실행한다.
//!
//! # 이 모듈이 기존 `nfp_lbf` 와 갈리는 지점
//!
//! `nfp_lbf` 는 **하나의** 배치 규칙(순서 고정 + 좌하단 우선)을 구현한다.
//! 이 모듈은 그 규칙을 **정책으로 분리**해 갈아 끼운다.
//!
//! ```text
//! nfp_lbf::SequentialPlacer   후보 오라클  — "여기에 놓을 수 있나"
//!         |                                   (NFP 커널 + 캐시. 검증된 코드라 포크하지 않는다)
//!         v
//! nfp_strategy                정책        — "무엇을 어디에 놓을까"
//!   rotations   ① 회전 집합을 인스턴스 선언에서 뽑는다
//!   cluster     ② 잘 맞물리는 쌍을 묶어 하나로 배치한다
//!   select      ③ 접촉 최대 / ⑥ best-fit 조각 선택
//!   (pocket)    ④ 프론티어 안쪽부터 채운다 (오라클의 탐침 순서로 구현)
//!   beam        ⑤ 부분 배치를 B개 유지한다
//!   sliding     ⑦ 배치 후 좌하단으로 민다
//! ```
//!
//! 판단 로직은 전부 이 모듈에서 새로 쓴다. 오라클에서 가져오는 것은 "이 좌표가
//! 실행 가능한가" 하나뿐이다.
//!
//! # 왜 후보 오라클은 새로 쓰지 않았나
//!
//! 후보 열거는 NFP 커널·마스크·준비영역·SoA 가 얽힌 이 코드베이스에서 가장
//! 성능·정확성이 민감한 부분이고, 실측으로 여러 번 교정된 곳이다. 두 벌로 갈라
//! 두면 한쪽만 고쳐지는 순간 조용히 어긋난다. 그래서 **정책만** 새로 쓴다.

pub mod accrete;
pub mod beam;
pub mod cluster;
pub mod config;
/// **Jostling — 배선하지 마라.** 배치 결과를 순서로 되먹여 재배치하는 기법인데,
/// 여성WSjk 156-패널에서 **이득이 없다**(71.75% -> 71.19%, -0.56%p).
///
/// 되먹인 순서가 출발 순서를 이긴 적이 63패스 중 **0회**다. 최선 ordering(71.75%)에서
/// 출발시켜도 18패스 전부 졌다 — 출발점 문제가 아니라 되먹임 자체가 진다.
/// 원인: x 되먹임이 "큰 조각 먼저" 라는 사전지식을 버린다. 띠 단위로 완화하면
/// 손실이 5%p 회복되지만, 띠를 무한대로 키우면 그건 초기 순서 자체라 정의상 못 넘는다.
///
/// 회전 자유도가 있거나 수요가 큰 인스턴스에서 다시 재 보라고 남긴 코드다.
pub mod jostle;
/// 꼬리 회수 + 저밀도 띠 재구성. **이득 있음** — 5초 예산에서 71.75% -> 73.27%(+1.52%p).
///
/// albano 에서는 더 크다(폭 12080.7 -> 11038.0). 구조상 결과를 나쁘게 만들 수 없다
/// (라운드마다 저장하고 strip 이 안 줄면 원복한다).
pub mod reflow;
pub mod pipeline;
pub mod rotations;
pub mod select;
pub mod sliding;

pub use config::{
    DEFAULT_BEAM_WIDTH, DEFAULT_BEST_FIT_LOOKAHEAD, DEFAULT_CLUSTER_MIN_GAIN,
    DEFAULT_CONTINUOUS_ROTATION_STEP_DEG, DEFAULT_NFP_STRATEGY_CONFIG, DEFAULT_REFLOW_ROUNDS,
    DEFAULT_SLIDING_ROUNDS, NfpStrategyConfig,
};

use jagua_rs::Instant;

use crate::nfp_lbf::placement::Aabb;
use crate::nfp_lbf::{PlacedPiece, SequentialPlacer};

/// 한 조각을 한 자세로 한 자리에 놓는 선택지.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnchorOption {
    /// 이 선택지의 조각.
    pub item_id: usize,
    /// 적용할 회전(도).
    pub rotation: i64,
    /// 앵커 평행이동(mm).
    pub x: f64,
    pub y: f64,
    /// 놓았을 때의 월드 bbox.
    pub world: Aabb,
    /// **정확히** 맞닿는 기존 조각 수. 근사가 아니다.
    ///
    /// 오라클이 feasible 판정을 하면서 obstacle 마다 `Location::Boundary` 를 이미
    /// 세므로 **공짜로 얻는 값**이다. 그런데 오래도록 전략 계층 진입점에서 버려졌고,
    /// ③ 접촉 최대화와 ⑤ 빔은 bbox 근사인 `contact` 하나만 보고 있었다.
    ///
    /// 근사와 정확값의 관계: `contact` 는 맞닿은 **길이**의 bbox 근사이고, 이 값은
    /// 맞닿은 **조각 수**의 정확값이다. 서로를 대체하지 않는다 — 조각 수가 같을 때
    /// 길이가 가른다.
    pub touches: u16,
}

impl AnchorOption {
    /// 배치 결과로 굳힌다.
    pub fn to_piece(self) -> PlacedPiece {
        PlacedPiece {
            item_id: self.item_id,
            rotation: self.rotation,
            translation: (self.x as f32, self.y as f32),
        }
    }
}

/// 선택지 하나의 평가 지표. **전부 mm 단위 실제 값**이다.
///
/// 정규화하지 않는 이유는 축마다 나누는 기준이 달라지면 같은 1mm 차이가 축에 따라
/// 동률이 되기도 하고 아니기도 하기 때문이다. 그 함정은 이미 한 번 밟았다.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AnchorMetrics {
    /// 이 조각을 놓은 뒤의 strip 길이(mm).
    pub strip_length: f64,
    /// 기존 조각·strip 경계와 맞닿는 길이의 근사(mm). 클수록 밀착이다.
    pub contact: f64,
    /// 이 배치가 새로 만드는 못 쓰는 공백의 근사(mm²).
    pub waste: f64,
    /// **정확히** 맞닿는 기존 조각 수 ([`AnchorOption::touches`]).
    pub touches: u16,
    /// 월드 좌하단.
    pub x: f64,
    pub y: f64,
}

/// 전략이 공유하는 실행 문맥.
///
/// 오라클(`placer`)을 빌려 쓰되, 무엇을 어디에 놓을지는 전략이 정한다.
pub struct StrategyContext<'a, 'c> {
    /// 후보 오라클. `candidates_for` 로 feasible 앵커만 받아 온다.
    pub placer: &'a mut SequentialPlacer<'c>,
    /// 켜진 전략들.
    pub cfg: NfpStrategyConfig,
    /// 아이템별 허용 회전(도). 비어 있으면 오라클 기본값을 쓴다.
    pub item_rotations: Vec<Vec<i64>>,
    /// 예산. 넘기면 전략은 즉시 반환해야 한다.
    pub deadline: Option<Instant>,
}

impl<'a, 'c> StrategyContext<'a, 'c> {
    pub fn new(
        placer: &'a mut SequentialPlacer<'c>,
        cfg: NfpStrategyConfig,
        item_rotations: Vec<Vec<i64>>,
        deadline: Option<Instant>,
    ) -> Self {
        Self {
            placer,
            cfg: cfg.normalized(),
            item_rotations,
            deadline,
        }
    }

    /// 예산이 남았는가.
    ///
    /// Init 취소 요청도 예산 소진으로 본다 — 전략은 deadline 과 같은 경로로 즉시 접는다.
    pub fn has_budget(&self) -> bool {
        if crate::config::runtime::init_cancel_requested() {
            return false;
        }
        match self.deadline {
            Some(d) => Instant::now() < d,
            None => true,
        }
    }

    /// 이 조각에 허용된 회전(도).
    pub fn rotations_for(&self, item_id: usize) -> Vec<i64> {
        match self.item_rotations.get(item_id) {
            Some(rots) if !rots.is_empty() => rots.clone(),
            _ => self.placer.allowed_rotations(item_id),
        }
    }

    /// 현재 프론티어 = 배치된 조각들의 오른쪽 끝(mm). 빈 배치면 `0`.
    pub fn frontier(&self, placed: &[PlacedPiece]) -> f64 {
        self.placer
            .world_bounds(placed)
            .iter()
            .map(|b| b.max_x)
            .fold(0.0_f64, f64::max)
    }

    /// 이 조각을 놓을 수 있는 **모든** 선택지를 모은다 (허용 회전 전부).
    ///
    /// ④ 포켓 우선이 켜져 있으면 오라클의 탐침 순서가 프론티어 안쪽을 먼저
    /// 전량 훑도록 전환된다 — 안쪽에서 자리를 찾으면 바깥은 보지 않는다.
    pub fn options_for(&mut self, item_id: usize, placed: &[PlacedPiece]) -> Vec<AnchorOption> {
        let frontier = self.frontier(placed);
        self.options_for_at(item_id, placed, frontier)
    }

    /// [`Self::options_for`] 와 같되 프론티어를 호출부가 넘긴다.
    ///
    /// `frontier` 는 `placed` 의 월드 bbox `max_x` 최대값이어야 한다. 호출부가 그 값을
    /// 이미 들고 있으면(빔 노드의 `strip_length`, best-fit 의 `frontier`) 여기서
    /// `world_bounds` 를 다시 만들 이유가 없다 — 조각마다 배치 전체를 다시 접는 비용이다.
    pub fn options_for_at(
        &mut self,
        item_id: usize,
        placed: &[PlacedPiece],
        frontier: f64,
    ) -> Vec<AnchorOption> {
        self.placer.set_probe_frontier(frontier);
        self.placer.set_pocket_first(self.cfg.pocket_first);

        let mut out = Vec::new();
        for rotation in self.rotations_for(item_id) {
            let Some((bounds, _)) = self.placer.shape_bounds_area(item_id, rotation) else {
                continue;
            };
            for cand in self.placer.candidates_for(item_id, rotation, placed) {
                let (x, y) = (cand.translation.0 as f64, cand.translation.1 as f64);
                out.push(AnchorOption {
                    item_id,
                    rotation,
                    x,
                    y,
                    world: bounds.translated(x, y),
                    // 오라클이 feasible 판정에서 이미 센 값이다. 버리지 않는다.
                    touches: cand.touch_count,
                });
            }
        }
        // 오라클의 탐침 순서를 되돌린다 — 다른 호출자가 영향을 받지 않게.
        self.placer.set_pocket_first(false);
        out
    }

    /// 선택지 하나를 평가한다.
    ///
    /// `placed_boxes` 는 호출부가 한 번만 만들어 재사용한다 — 후보마다 다시 만들면
    /// 그것만으로 O(후보 x 배치)가 된다.
    pub fn metrics(
        &self,
        option: &AnchorOption,
        placed_boxes: &[Aabb],
        frontier: f64,
    ) -> AnchorMetrics {
        AnchorMetrics {
            strip_length: frontier.max(option.world.max_x),
            contact: contact_length(&option.world, placed_boxes, self.placer.strip_height() as f64),
            waste: waste_area(&option.world, placed_boxes),
            touches: option.touches,
            x: option.world.min_x,
            y: option.world.min_y,
        }
    }
}

/// 두 bbox 가 맞닿은 길이의 근사(mm), strip 위·아래 경계 포함.
///
/// 정확한 접촉 길이는 폴리곤 경계 교차 길이지만, 후보마다 그걸 구하면 배치 1회가
/// 초 단위로 늘어난다. 후보 좌표 자체가 NFP 경계에서 나오므로 **실제 형상은 이미
/// 접촉**해 있고, 여기서 필요한 것은 "어느 후보가 더 많이 맞닿았나" 라는 **순서**다.
/// bbox 근사는 그 순서를 대체로 보존한다.
///
/// 간극 `g` 는 `CONTACT_BAND` 안에서 선형으로 감쇠시킨다 — 정확히 0 만 세면
/// f32 반올림 때문에 접촉이 통째로 사라진다.
fn contact_length(world: &Aabb, placed: &[Aabb], strip_height: f64) -> f64 {
    let mut total = 0.0;
    let width = (world.max_x - world.min_x).max(0.0);

    // strip 아래/위 경계. 경계 밖으로 나갈 수 없으므로 간극은 음수가 될 수 없지만,
    // f32 반올림으로 미세한 음수가 나올 수 있어 `proximity` 가 눕혀 준다.
    total += width * proximity(world.min_y, f64::NEG_INFINITY);
    total += width * proximity(strip_height - world.max_y, f64::NEG_INFINITY);

    for b in placed {
        // 세로 변끼리 마주보는 경우 — 겹치는 y 구간이 접촉 길이다.
        let y_overlap = (world.max_y.min(b.max_y) - world.min_y.max(b.min_y)).max(0.0);
        if y_overlap > 0.0 {
            total += y_overlap * proximity(world.min_x - b.max_x, b.min_x - world.max_x);
        }
        // 가로 변끼리 마주보는 경우.
        let x_overlap = (world.max_x.min(b.max_x) - world.min_x.max(b.min_x)).max(0.0);
        if x_overlap > 0.0 {
            total += x_overlap * proximity(world.min_y - b.max_y, b.min_y - world.max_y);
        }
    }
    total
}

/// 두 bbox 가 한 축에서 얼마나 가까운지를 `0..=1` 로. 1이 맞닿음이다.
///
/// # bbox 가 **겹칠 때**가 핵심이다
///
/// 예전 구현은 `(0.0..=CONTACT_BAND).contains(&gap)` 로 걸렀는데, 이러면 두 bbox 가
/// 겹칠 때(`gap < 0`) 조건이 거짓이 되어 **접촉을 0으로 친다**. 그런데 후보는 이미
/// NFP 로 feasible 판정을 통과했으므로 **실제 도형은 겹치지 않는다** — bbox 만 겹친다.
/// 그건 두 조각이 서로의 오목한 부분에 **맞물려 들어갔다**는 뜻이고, 접촉이 없기는커녕
/// 가장 좋은 배치다.
///
/// 의류 패널은 대부분 오목하므로 이 경우가 흔하다. 실측(탐침 8192, wsjk156):
/// 후보 105,109개 중 **76.2%가 접촉 0** 이었고, ③ 접촉 최대화가 고른 후보의 57.6%가
/// 접촉 0 이었다 — 규칙이 볼 신호 자체가 없었다는 뜻이다.
///
/// 그래서 음수 간극을 0 으로 눕혀 **맞물림에 최대 가중치**를 준다.
///
/// # `NaN` 은 어떻게 되는가
///
/// 예전 문서는 "`NaN` 은 `max` 두 번을 거쳐 0 이 되고, 그 후보는 접촉 없음으로
/// 취급된다" 고 적었다. **코드와 정반대다.** `f64::max` 는 `NaN` 을 무시하고 다른
/// 피연산자를 돌려주므로 `NaN.max(x).max(0.0)` 은 `NaN` 이 아니라 `max(x, 0)` 이 되고,
/// 특히 `gap_a` 와 `gap_b` 가 **둘 다** `NaN` 이면 `NaN.max(NaN).max(0.0) == 0.0` —
/// 즉 접촉 없음이 아니라 **간극 0 = 최대 접촉**이 된다.
///
/// 방향으로 보면 그게 맞다: 간극을 계산할 수 없는 후보는 좌표가 이미 망가진 것이고,
/// 그런 후보가 접촉 점수까지 최대로 받는 것은 곤란하다. 그래서 여기서 명시적으로
/// 걸러 "접촉 없음" 으로 눕힌다. 문서가 말하던 동작을 코드로 만든 것이다.
#[inline]
fn proximity(gap_a: f64, gap_b: f64) -> f64 {
    /// 맞닿았다고 볼 간극 폭(mm).
    ///
    /// 후보 좌표는 NFP 경계에서 나오므로 실제 도형은 정확히 접촉한다. 이 밴드는
    /// bbox 근사와 f32 반올림이 만드는 오차를 흡수하기 위한 것이지 "가까우면 봐준다"가
    /// 아니다.
    const CONTACT_BAND: f64 = 0.5;

    // 비유한 값은 접촉 없음. `max` 는 `NaN` 을 무시하므로 여기서 먼저 걸러야 한다.
    if !gap_a.is_finite() && !gap_b.is_finite() {
        return 0.0;
    }
    let gap = gap_a.max(gap_b).max(0.0);
    if gap > CONTACT_BAND {
        return 0.0;
    }
    1.0 - gap / CONTACT_BAND
}

/// 이 배치가 왼쪽·아래에 남기는 못 쓰는 공백의 근사(mm²).
///
/// 가장 가까운 왼쪽/아래 이웃(없으면 strip 경계)까지의 간극을 직사각형으로 잡는다.
/// 조각이 이웃에서 멀리 떨어질수록 커진다.
fn waste_area(world: &Aabb, placed: &[Aabb]) -> f64 {
    let h = (world.max_y - world.min_y).max(0.0);
    let w = (world.max_x - world.min_x).max(0.0);

    // 왼쪽으로 가장 가까운 이웃의 오른쪽 끝.
    let left = placed
        .iter()
        .filter(|b| (world.max_y.min(b.max_y) - world.min_y.max(b.min_y)) > 0.0)
        .filter(|b| b.max_x <= world.min_x)
        .map(|b| b.max_x)
        .fold(0.0_f64, f64::max);

    // 아래로 가장 가까운 이웃의 위쪽 끝.
    let below = placed
        .iter()
        .filter(|b| (world.max_x.min(b.max_x) - world.min_x.max(b.min_x)) > 0.0)
        .filter(|b| b.max_y <= world.min_y)
        .map(|b| b.max_y)
        .fold(0.0_f64, f64::max);

    (world.min_x - left).max(0.0) * h + (world.min_y - below).max(0.0) * w
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bbox(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Aabb {
        Aabb {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    #[test]
    fn touching_neighbours_score_higher_contact_than_distant_ones() {
        let neighbour = bbox(0.0, 0.0, 10.0, 10.0);
        let strip_h = 100.0;

        let touching = contact_length(&bbox(10.0, 0.0, 20.0, 10.0), &[neighbour], strip_h);
        let apart = contact_length(&bbox(30.0, 0.0, 40.0, 10.0), &[neighbour], strip_h);
        assert!(
            touching > apart,
            "맞닿은 배치가 더 큰 접촉을 가져야 한다: {touching} vs {apart}"
        );
    }

    #[test]
    fn interlocking_pieces_score_maximum_contact_not_zero() {
        // 두 bbox 가 겹치는 배치. 후보는 NFP 로 feasible 판정을 통과했으므로 실제
        // 도형은 겹치지 않는다 — 서로의 오목한 부분에 맞물려 들어간 것이다.
        // 예전 구현은 이걸 접촉 0 으로 쳐서 ③ 접촉 최대화가 볼 신호를 없앴다.
        let neighbour = bbox(0.0, 0.0, 10.0, 10.0);
        let interlocked = contact_length(&bbox(8.0, 0.0, 18.0, 10.0), &[neighbour], 100.0);
        let touching = contact_length(&bbox(10.0, 0.0, 20.0, 10.0), &[neighbour], 100.0);
        let apart = contact_length(&bbox(30.0, 0.0, 40.0, 10.0), &[neighbour], 100.0);

        assert!(interlocked > 0.0, "맞물린 배치의 접촉이 0이면 안 된다");
        assert!(
            interlocked >= touching,
            "맞물림이 단순 맞닿음보다 낮게 평가되면 안 된다: {interlocked} vs {touching}"
        );
        // 맞물리면 **두 축 모두**에서 맞닿으므로 단순 맞닿음보다 커야 정상이다.
        // 세로변 10mm + 가로변 2mm(파고든 깊이) + 바닥 10mm = 22, 맞닿음은 10 + 10 = 20.
        assert!(
            (interlocked - 22.0).abs() < 1.0e-9 && (touching - 20.0).abs() < 1.0e-9,
            "접촉 분해가 바뀌었다: 맞물림 {interlocked}, 맞닿음 {touching}"
        );
        assert!(interlocked > apart);
    }

    /// 간극을 계산할 수 없는 후보는 **접촉 없음**이다.
    ///
    /// 예전에는 이름("treated_as_no_contact")과 docstring 은 "접촉 없음" 이라고 하면서
    /// 단정은 `1.0`(= 최대 접촉)을 요구했다. 셋이 서로 어긋나 있었고, 실제 코드는
    /// 단정을 따랐다 — `f64::max` 는 `NaN` 을 무시하므로 `NaN.max(NaN).max(0.0)` 이
    /// `0.0`(간극 0 = 맞닿음)이 됐기 때문이다.
    ///
    /// 좌표가 이미 망가진 후보가 접촉 점수를 최대로 받는 것은 곤란하다. 이름과 문서가
    /// 말하던 쪽으로 코드를 맞추고, 단정도 그것으로 고쳤다.
    #[test]
    fn a_non_finite_gap_is_treated_as_no_contact_instead_of_propagating() {
        assert_eq!(
            proximity(f64::NAN, f64::NAN),
            0.0,
            "간극을 못 재는 후보가 최대 접촉을 받으면 안 된다"
        );
        assert_eq!(proximity(f64::INFINITY, f64::NEG_INFINITY), 0.0);
        // 겹침(음수 간극)은 최대 가중치.
        assert_eq!(proximity(-5.0, -3.0), 1.0);
        // 밴드 밖은 0.
        assert_eq!(proximity(1.0, f64::NEG_INFINITY), 0.0);
        // 밴드 안은 선형 감쇠.
        assert!((proximity(0.25, f64::NEG_INFINITY) - 0.5).abs() < 1.0e-9);
    }

    #[test]
    fn the_strip_floor_counts_as_contact() {
        // 바닥에 붙은 조각은 이웃이 없어도 접촉이 있다.
        let on_floor = contact_length(&bbox(0.0, 0.0, 10.0, 5.0), &[], 100.0);
        let floating = contact_length(&bbox(0.0, 50.0, 10.0, 55.0), &[], 100.0);
        assert!(on_floor > 0.0);
        assert_eq!(floating, 0.0);
    }

    #[test]
    fn waste_grows_with_the_gap_to_the_left_neighbour() {
        let neighbour = bbox(0.0, 0.0, 10.0, 10.0);
        let snug = waste_area(&bbox(10.0, 0.0, 20.0, 10.0), &[neighbour]);
        let gapped = waste_area(&bbox(25.0, 0.0, 35.0, 10.0), &[neighbour]);
        assert!(snug < gapped, "간극이 크면 낭비가 커야 한다");
        assert_eq!(snug, 0.0, "딱 붙으면 낭비가 없다");
    }

    #[test]
    fn an_option_converts_to_a_placed_piece_without_losing_the_pose() {
        let option = AnchorOption {
            item_id: 7,
            rotation: 180,
            x: 12.5,
            y: 3.25,
            world: bbox(12.5, 3.25, 22.5, 13.25),
            touches: 0,
        };
        let piece = option.to_piece();
        assert_eq!(piece.item_id, 7);
        assert_eq!(piece.rotation, 180);
        assert_eq!(piece.translation, (12.5, 3.25));
    }
}
