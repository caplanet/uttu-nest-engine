use crate::optimizer::separator::{SeparationRepairConfig, SeparatorConfig};
use crate::sample::search::SampleConfig;
use jagua_rs::collision_detection::CDEConfig;
use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
use std::time::Duration;

/// 원본 외곽선 겹침이 `piece_overlap_mm` 허용값을 넘는 배치의 처리 방식.
///
/// 겹침을 허용하면 탐색은 각 조각을 허용값의 절반만큼 줄인 충돌 도형으로 한다.
/// 줄인 도형은 겹치지 않아도, 원본 외곽선은 날카로운 꼭짓점에서 허용값보다 깊이
/// 겹칠 수 있다(줄이기가 꼭짓점을 변보다 더 깎는다).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OriginalOverlapPolicy {
    /// 실현 불가능으로 보고 버린다 (기본, 기존 동작). 원본 외곽선 가드가
    /// 구성·탐색·인증 전 과정에서 초과를 막고 보정한다.
    #[default]
    Infeasible,
    /// 실현 가능으로 보고 채택한다. 원본 외곽선 가드를 끄고, 겹침은 줄인 충돌
    /// 도형으로만 판정한다. 원단 경계(원단 밖 허용 mm) 검사는 그대로다.
    Feasible,
}

/// V2 place groups(패턴 그룹)를 엔진에 어떻게 넘길지.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlaceGroupMode {
    /// 그룹을 **다중 폴리곤 아이템 하나**로 만든다(기본). 멤버 외곽선을 힌트 자세대로
    /// 합친 아이템을 엔진이 보통 조각처럼 다루고, 결과는 멤버별 자세로 풀어 돌려준다.
    /// 멤버 사이 틈에는 다른 조각이 들어갈 수 있고 멤버 상대 자세는 정의상 정확하다.
    #[default]
    MultiPolygon,
    /// 멤버를 별도 조각으로 두고 상대 좌표로 함께 옮긴다(1.0.2 까지의 방식).
    Rigid,
}

#[derive(Debug, Clone, Copy)]
pub struct SwarmConfig {
    pub rng_seed: Option<usize>,
    pub expl_cfg: ExplorationConfig,
    pub cmpr_cfg: CompressionConfig,
    /// LBF constructor: X-axis placement bias multiplier.
    /// Higher values pull items more strongly toward the left edge. Default: 10.0
    pub lbf_x_multiplier: f32,
    /// LBF constructor: Y-axis placement bias multiplier.
    /// Higher values pull items more strongly toward the bottom edge. Default: 1.0
    pub lbf_y_multiplier: f32,
    /// Configuration for the collision detection engine.
    /// See [`CDEConfig`] for more details.
    pub cde_config: CDEConfig,
    /// Defines the polygon simplification tolerance: maximum allowable inflation of items when simplifying their shape.
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub poly_simpl_tolerance: Option<f32>,
    /// Absolute cumulative boundary deviation after clearance/overlap offset.
    /// Replaces area-ratio simplification and unbounded narrow-pocket closing.
    pub simplify_max_deviation_mm: Option<f32>,
    /// 아이템과 다른 hazard 사이의 최소 간격. `None`이면 비활성.
    ///
    /// **음수를 주면 겹침 허용(piece overlap)이 된다.** jagua-rs importer가 이 값의
    /// 절반을 각 도형의 offset으로 적용하므로(`offset = separation / 2`), 음수면 도형이
    /// 안쪽으로 줄어들어 두 조각이 최대 `|separation|`만큼 물릴 수 있다.
    ///
    /// 이 축소는 **import 시점**에 일어나 CDE와 NFP가 같은 도형을 보므로, 겹침이
    /// 충돌 판정을 깨뜨리지 않는다. 실제 원단에서만 그만큼 물린다.
    ///
    /// 실측 (여성WSjk 156-패널, NFP-LBF ordering 1회):
    ///
    /// ```text
    /// None    strip 7686.5   밀도 66.18%
    /// -0.4    strip 7466.4   밀도 68.13%   <- strip -2.9%, 밀도 +1.95%p
    /// -1.0    strip 7598.7   밀도 66.95%   <- 너무 줄이면 오히려 손해
    /// ```
    ///
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub min_item_separation: Option<f32>,
    /// 조각 겹침 허용(`min_item_separation < 0`)일 때, **원본** 외곽선끼리의 겹침이
    /// 허용값을 넘는 배치를 어떻게 볼지. 기본은 [`OriginalOverlapPolicy::Infeasible`]
    /// (기존 동작). [`crate::original_overlap::apply_policy`] 참조.
    pub original_overlap_policy: OriginalOverlapPolicy,
    /// place groups 처리 방식. 기본 [`PlaceGroupMode::MultiPolygon`].
    pub place_group_mode: PlaceGroupMode,
    /// 원단 경계에서 조각의 **원본** 외곽선이 넘어서도 되는 최대 거리(mm).
    /// `None` 이면 검사·보정 없음(기존 동작).
    ///
    /// 겹침을 허용하면 각 조각의 충돌 도형이 `overlap/2` 만큼 침식된다. 침식은
    /// 볼록 꼭짓점을 변에서처럼 `r` 이 아니라 `r/sin(θ/2)` 만큼 끌어당기므로,
    /// 침식된 도형이 (이미 같은 양만큼 줄여 둔) 컨테이너 벽에 닿으면 **원본**
    /// 꼭짓점은 `r · (1/sin(θ/2) − 1)` 만큼 원단 밖으로 나간다. 이 값은 각이
    /// 날카로울수록 무한히 커지고, 원본 외곽선을 경계와 대조하는 검사는 이
    /// 코드베이스 어디에도 없다.
    ///
    /// 실측(여성WSjk 156-패널, 겹침 0.8mm): 최소 꼭짓각 31.7° → 이론상 최대
    /// 1.065mm, 실제 산출물에서 0.352mm 관측.
    ///
    /// 이 값을 주면 컨테이너를 그만큼 **더** 줄여 초과량을 예산 안으로 눌러 준다.
    /// 대가는 원단 폭이고, 1524mm 기준 여유 1mm 당 약 0.066% 의 밀도다.
    pub fabric_edge_margin_mm: Option<f32>,
    /// 초기 구성 직후 hole fill + LP 압축에 줄 예산(ms). `0` 이면 하지 않는다(기본).
    ///
    /// 두 패스는 지금까지 Polish 에서만 돌았다. 그때는 배치가 굳어 있고 Explore 가
    /// 이미 그 배치를 전제로 폭을 정한 뒤다. 구성 직후에 돌리면 Explore 가 더 나은
    /// 출발점에서 시작한다.
    pub init_compaction_ms: u64,
    /// 구성 **도중** 몇 조각마다 다질지. `0` 이면 하지 않는다(기본).
    ///
    /// 참조 네스터의 77% 초기 배치는 조각을 하나 놓을 때마다 hole fill·LP 압축·
    /// 접촉 위치 탐색이 함께 도는 것으로 보인다. 우리는 셋 다 Polish 에서만 돌아서,
    /// 다음 조각이 **다져지지 않은** 프론티어를 보고 자리를 골랐다.
    /// 구간 완성 폭(mm). `None`(기본)이면 구간 제한 없이 기존대로 배치한다.
    pub init_band_width_mm: Option<f32>,
    /// 구간 경계에서 목록 뒤쪽의 작은 조각까지 훑을 때의 면적 상한 비율. `0` = 훑지 않음.
    pub init_band_fill_ratio: f32,
    pub init_settle_every: usize,
    /// 구성 도중 다지기 한 번에 줄 예산(ms).
    pub init_settle_budget_ms: u64,
    /// Defines a maximum distance and area of a concavity to be considered "narrow" (which will be closed).
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub narrow_concavity_cutoff_ratio: Option<(f32, f32)>,
    /// 압축 단계 이후 수행되는 결정론적 후처리(polish) 단계 설정.
    pub polish_cfg: PolishConfig,
    /// LBF 초기 배치의 래스터 NFP 후보 생성 설정. capmods 빌드에서만 반영.
    pub lbf_raster: RasterNfpConfig,
    /// 인스턴스 로드 시 각 아이템의 허용 회전 목록에 180° 반전을 추가한다.
    /// 결방향이 없는(non-nap) 원단 전용 — 기본 false.
    /// [`crate::util::io::augment_orientations_180`]을 import 전에 적용하는 호출자만 반영한다.
    pub allow_180_rotation: bool,
    /// Experimental P0-P5 structure-first pipeline. Runtime-disabled by
    /// default and only executable in a `structure-first` feature build.
    pub structure: crate::structure::config::StructureFirstConfig,
    /// 정체 감지 기반 적응형 국소 최적해 탈출 기법
    /// (Stagnation-Triggered Adaptive Local-Optimum Escape).
    ///
    /// 켜면 탐색 단계가 [`crate::escape::escape_driven_exploration_phase`]로 교체된다.
    /// `adaptive` feature의 `exploration_phase_adaptive`보다 우선한다.
    pub escape_enabled: bool,
    /// 위 탈출 기법의 세부 설정.
    pub escape: crate::escape::AdaptiveEscapeConfig,
    /// NFP Cache -> Multiple Item Ordering -> NFP Sequential Placement 로
    /// Best Initial Nest를 만들어 탐색 단계의 seed로 넣는다.
    pub nfp_lbf: NfpLbfSeedConfig,
    /// 탐색 단계에서 NFP를 어디까지 쓸지. 항목마다 독립적으로 켜고 끈다.
    pub nfp_integration: NfpIntegrationConfig,
    /// NFP **초기 배치** 전략 일곱 가지. 항목마다 독립적으로 켜고 끈다.
    ///
    /// [`crate::nfp_strategy`] 참조. 전부 끄면 기존 NFP-LBF 경로가 그대로 돈다.
    pub nfp_strategy: crate::nfp_strategy::NfpStrategyConfig,
    /// 압축 단계 LP·QP 를 어떤 엔진으로 풀지.
    ///
    /// [`crate::lp_solver`] 참조. 기본은 기존 동작(microlp 단체법, 순수 LP)이다.
    pub lp_solver: crate::lp_solver::LpSolverConfig,
    /// **실험** — 레퍼런스 설계 초기배치 엔진(`refplace`).
    ///
    /// 기본 꺼짐이고, 켜면 기존 초기 배치를 **대체**한다(시드 비교가 아니다).
    /// 아직 품질이 기존 경로에 크게 못 미친다 — [`RefPlaceSeedConfig`] 문서의 실측 참조.
    pub refplace: RefPlaceSeedConfig,
    /// LBF 초기 배치의 **순회 방향**.
    ///
    /// 크기 점수의 정의는 그대로 두고 순서만 바꾼다 — [`crate::optimizer::lbf::LbfOrderMode`].
    /// 기본은 현행 동작(큰 것부터)이라, 지정하지 않으면 한 비트도 달라지지 않는다.
    pub lbf_order: crate::optimizer::lbf::LbfOrderMode,
}

/// 탐색(exploration) 단계의 NFP 활용 범위.
///
/// 기존 Swarm 탐색은 "이 조각이 어디로 갈 수 있나"를 전부 **근사**로 답한다 —
/// 난수 샘플링, bbox MTV, 래스터 격자. NFP는 그 질문에 **정확히** 답한다.
///
/// 다만 NFP 질의는 근사보다 비싸고, 탐색은 초기 배치보다 질의를 훨씬 많이 한다.
/// 그래서 전부 기본 꺼짐이며, 항목마다 따로 켜서 A/B로 재 봐야 한다.
///
/// [`SwarmConfig`]가 `Copy`라 이 구조체도 `Copy`다.
#[derive(Debug, Clone, Copy)]
pub struct NfpIntegrationConfig {
    /// 탐색 단계를 **순수 NFP 순서 탐색**으로 교체한다.
    ///
    /// 겹침을 한 번도 허용하지 않고, 탐색 공간을 위치가 아니라 배치 순서로 잡는다.
    /// ordering을 계속 변형(swap / move / block reorder)하며 prefix를 재사용해
    /// 재배치하고, 주기적으로 LP 압축으로 미세 조정한다.
    ///
    /// 기존 탐색을 대체하는 **네 번째 선택지**다. `escape_enabled`보다 우선한다.
    /// 연속 완화(겹침 허용)가 주는 마지막 몇 %를 잃을 수 있으므로 반드시 A/B로 확인할 것.
    pub ordering_exploration: bool,

    /// ① separator의 위치 샘플러를 NFP 후보로 교체한다.
    ///
    /// 기존은 컨테이너에 난수를 뿌리고 CDE로 거른다. 배치가 조밀해질수록 유효 위치의
    /// 면적 비율이 급감해 좁은 포켓을 확률적으로 놓친다. NFP는 유효 위치를 정확히
    /// 열거하고, 후보가 전부 접촉 위치라 자동으로 밀착된다.
    pub nfp_sampler: bool,

    /// ② Conflict Burst의 분리 벡터를 정확한 NFP 기반으로 바꾼다.
    ///
    /// 현재 `EscapeVector = sum w_ij x SeparationVector`의 SeparationVector가
    /// **bbox MTV 근사**다. 오목한 의류 패턴에서는 실제 최소 분리 방향과 크게 어긋난다.
    pub exact_separation_vector: bool,

    /// ③ hole filling을 `NfpEngine::feasible_region` 기반으로 바꾼다.
    ///
    /// 현재는 래스터 격자로 포켓을 찾아 격자 해상도에 걸리는 좁은 포켓을 놓친다.
    /// `feasible_region`은 "이 조각이 들어갈 수 있는 영역"을 정확히 돌려준다.
    pub nfp_hole_filling: bool,

    /// ④ Ruin & Recreate의 재삽입을 NFP 기반으로 바꾼다.
    ///
    /// 현재 재삽입은 샘플링 + CDE 검증이라 ①과 같은 한계를 갖는다.
    /// NFP-LBF의 순차 배치 로직을 재사용해 정확한 최선 위치를 찾는다.
    pub nfp_reinsertion: bool,
}

/// NFP 기반 초기 배치(NFP-LBF) 시드 설정.
///
/// ```text
/// NFP Cache
///    v
/// Multiple Item Ordering
///    v
/// NFP 기반 Sequential Placement
///    v
/// Strip Length 최소 후보 우선
///    v
/// LB + Contact + Void 평가
///    v
/// Best Initial Nest
///    v
/// Swarm / Local Improvement
/// ```
///
/// [`SwarmConfig`]가 `Copy`라서 이 구조체도 `Copy`여야 한다. 실행 시점에
/// [`Self::to_config`]로 `Vec`을 가진 [`crate::nfp_lbf::NfpLbfConfig`]를 만든다.
#[derive(Debug, Clone, Copy)]
pub struct NfpLbfSeedConfig {
    /// 끄면 기존 LBF 생성기(`optimizer/lbf.rs`)를 그대로 쓴다.
    pub enabled: bool,
    /// 평가할 ordering 수. 문서 목표: `Cold start 10초 이내에 NFP 캐시 + 30~50개 평가`.
    pub n_orderings: usize,
    /// NFP 콜드 캐시 생성 예산(ms). 문서 목표 `< 2 sec`, 최악 `< 5 sec`.
    pub cache_budget_ms: u64,
    /// 초기 배치 탐색 전체 예산(ms). 문서 목표 `Cold start 10초 이내`.
    pub total_budget_ms: u64,
    /// 회전 상태에 180도를 추가한다. 문서 기본 조건은 "회전 필요없고 쌍별 페어로만"이라
    /// 기본 `false`(0도 단일)다.
    pub allow_180: bool,
    /// 순서가 뒷부분만 달라졌을 때 앞의 배치를 그대로 재사용한다.
    pub enable_prefix_reuse: bool,
    /// NFP 라이브러리 **전용** 오목(concave) 단순화 강도 (원본 면적 대비 허용 증가 비율).
    ///
    /// 볼록 분해 조각 수가 `오목 정점 수 + 1`을 따라가므로, NFP 비용을 줄이는 레버는
    /// 오목 정점뿐이다. 전역 [`SwarmConfig::poly_simpl_tolerance`]를 올리면 볼록
    /// 정점까지 함께 뭉개져 형상 충실도만 잃으므로 강도를 분리한다.
    ///
    /// 결과 도형은 원본의 상위집합이라 NFP가 금지영역을 과대평가한다 - 안전하다.
    /// `0.0`이면 비활성.
    pub concave_simplify_tolerance: f32,
    /// 배치 후보 선택에 쓸 가중치. 기본값은 문서 수식의 기본 가중치이고
    /// `score_first` 가 꺼져 있어 사전식 순서가 실제 선택을 한다.
    pub score_weights: crate::nfp_lbf::PlacementScoreWeights,
    /// 아이템 하나당 feasible 판정을 시도할 raw 후보 수 상한. `0`이면 기본값.
    ///
    /// **초기 배치 품질을 좌우하는 값이다.** 후보 수는 배치가 진행될수록 폭증하는데
    /// (여성WSjk 156-패널: 20번째 1,607개 -> 155번째 13,514개) 예산이 고정이면 후기
    /// 아이템이 후보의 7.5%만 보고 자리를 정한다.
    ///
    /// 예전 기본값 1024에서는 NFP-LBF가 기존 LBF보다 **넓은** 결과를 내서 시드가 항상
    /// 기각됐다. 실측표는 [`crate::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES`] 참조.
    pub max_candidate_probes: usize,
}

impl NfpLbfSeedConfig {
    /// 실행 시점 설정으로 펼친다.
    ///
    /// `strategy`를 **인자로 받는다.** [`SwarmConfig::nfp_strategy`]가 단일 출처이고,
    /// 여기에 사본을 두면 두 값이 어긋날 수 있다. 인자로 받으면 호출부가 빠뜨릴 수
    /// 없다 — 컴파일러가 막는다. 이 코드베이스는 이미 "설정을 실어 보내는 걸 잊어서
    /// 기능이 통째로 죽는" 결함을 겪었다.
    pub fn to_config(
        self,
        strategy: crate::nfp_strategy::NfpStrategyConfig,
    ) -> crate::nfp_lbf::NfpLbfConfig {
        crate::nfp_lbf::NfpLbfConfig {
            n_orderings: self.n_orderings,
            cache_budget: Duration::from_millis(self.cache_budget_ms),
            total_budget: Duration::from_millis(self.total_budget_ms),
            weights: self.score_weights,
            rotations: if self.allow_180 {
                vec![0, 180]
            } else {
                vec![0]
            },
            custom_order: None,
            enable_prefix_reuse: self.enable_prefix_reuse,
            concave_simplify_tolerance: self.concave_simplify_tolerance,
            max_candidate_probes: if self.max_candidate_probes == 0 {
                crate::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES
            } else {
                self.max_candidate_probes
            },
            strategy,
            // **꺼짐이 기본이다.** 켜면 조절 판정이 벽시계를 읽어 같은 입력이
            // 머신·부하에 따라 다른 배치를 낸다 — 회귀를 비트 단위로 잡는 이
            // 코드베이스에서는 그 자체가 결함이다.
            adaptive_probe_budget: false,
        }
    }
}

/// LBF(초기 배치) 단계의 래스터 NFP 후보 생성 설정. capmods 전용.
///
/// 레이아웃과 아이템을 같은 격자로 래스터화해 "겹치지 않는 격자 위치"를 직접 열거한다.
/// 균등 난수 샘플링이 좁은 포켓을 확률적으로 놓치는 문제를 없애는 것이 목적이다.
///
/// **래스터는 후보를 제안할 뿐 판정하지 않는다.** 열거된 후보는 전부 기존 CDE
/// (`LBFEvaluator`)로 정확히 검증한 뒤에만 채택되므로, 격자 해상도가 어떻든 잘못된
/// 배치가 통과할 수 없다. 격자는 후보의 품질(=탐색 효율)에만 영향을 준다.
#[derive(Debug, Clone, Copy)]
pub struct RasterNfpConfig {
    pub enabled: bool,
    /// 셀 크기 = (가장 작은 아이템의 최소 변 길이) × 이 비율.
    /// 작을수록 촘촘하지만 격자 비용이 제곱으로 늘어난다. 기본값 0.12.
    pub cell_ratio: f32,
    /// 격자 셀 총 개수 상한. 초과 시 셀 크기를 키워 맞춘다.
    pub max_cells: usize,
    /// CDE로 검증할 래스터 후보 수 (회전 전체 합산). 기본값 64.
    pub n_candidates: usize,
    /// **양쪽 모서리 앵커링을 적용할 앞쪽 조각 수.** 0 이면 꺼짐(기존 좌·하단 규칙).
    ///
    /// 켜진 동안에는 아래 모서리뿐 아니라 위 모서리에 붙는 자리도 같은 값으로 친다.
    ///
    /// # 왜
    ///
    /// LBF 손실의 y 항은 y 가 작을수록 좋다 — 그래서 모든 조각이 아래부터 쌓인다.
    /// 실측(여성WSjk 156-패널 초기 배치, 겹침 0.8mm): 하단 모서리에 닿는 조각 16개,
    /// 상단 6개로 2.7배 편향이다. 참조 네스터의 77% 초기 배치는 큰 패널을 위·아래에
    /// 번갈아 붙여 **프론티어를 둘** 만들고 그 사이를 채운다.
    ///
    /// # 왜 "앞쪽 N개" 인가
    ///
    /// 전 구간에 적용하면 모든 조각이 위·아래로 몰려 **가운데가 빈다** — 실측에서
    /// Init 이 73.350% → 72.696% 로 나빠졌다. 두 프론티어를 만드는 것은 큰 패널
    /// 몇 개의 일이고, 그 뒤의 채우기는 기존 좌·하단 규칙이 맞다.
    pub double_sided_first_n: usize,
    /// 래스터 후보와 함께 유지할 난수 샘플 수. 래스터가 놓치는 경우의 보험이며
    /// 0이면 래스터 후보만 쓴다. 기본값 1000.
    pub n_random_samples: usize,
    /// **홀 우선 부착** — 이미 배치된 조각들에 둘러싸인 빈 주머니(포켓) 안의 후보를
    /// 시드 순서 앞쪽으로 보낸다.
    ///
    /// # 왜 필요한가
    ///
    /// 래스터 후보는 LBF 손실(좌·하단 우선)로만 정렬된다. 포켓은 정의상 이미
    /// 배치된 조각들 **사이**에 있으므로 x 가 크고, 따라서 손실이 크다 — 후보
    /// 상한(`n_candidates`)에 걸려 잘려 나간다. 정작 그 자리가 밀도에 가장 크게
    /// 기여하는데도 시드가 한 번도 시도하지 않는다.
    ///
    /// 폴리시 단계의 [`crate::optimizer::polish::holes`] 가 사후에 같은 일을 하지만,
    /// 그때는 배치가 이미 굳어 있어 넣을 수 있는 조각이 얼마 남지 않는다. 초기
    /// 배치에서 먼저 시도하는 것이 이 옵션이다.
    ///
    /// 포켓 판정은 폴리시와 같은 정의를 쓴다: **strip 우측 경계에 닿지 않는
    /// 4-연결 빈 영역**. 우측에 닿는 영역은 packing frontier 바깥의 열린 만이다.
    ///
    /// # 순서만으로는 아무 일도 일어나지 않는다
    ///
    /// 처음에는 후보 목록 앞쪽에 놓기만 했다. 결과가 **한 비트도 안 바뀌었다** —
    /// 호출부가 시드 전체를 `LBFEvaluator`(좌·하단 손실 최소)로 채점해 하나만 고르는데,
    /// 포켓은 x 가 커서 손실이 크므로 순서와 무관하게 argmin 에서 진다. 진단
    /// 카운터([`crate::sample::raster_nfp::hole_stats`])가 포켓 후보 203개가 실제로
    /// 만들어졌음을 보여줘서 "배선이 끊긴 것" 과 구분할 수 있었다.
    ///
    /// 그래서 `optimizer::lbf` 가 포켓 구간을 **먼저 따로** 평가하고, 통과하면
    /// 그것을 채택한다. 그때부터 수치가 움직였다.
    ///
    /// # 실측 (LBF 단독, seed 0, share 0.5)
    ///
    /// ```text
    /// 인스턴스     폭 변화        밀도 변화   포켓 배치
    /// wsjk156      -52.85mm  -0.73%  +0.52%p     18   <- 지정 데이터
    /// swim          -6.54mm  -0.09%  +0.06%p      -
    /// shirts        +0.55mm  +0.80%  -0.62%p     27   <- 나빠진다
    /// fu/blaz1/albano/dagli/marques  변화 없음 (포켓이 안 생긴다)
    /// ```
    ///
    /// **기본을 끔으로 둔 이유가 이 표다.** 의류 마커처럼 조각이 크고 오목해
    /// 진짜 주머니가 생기는 인스턴스에서는 확실히 이득이고, `shirts` 처럼 조각이
    /// 작고 촘촘한 인스턴스에서는 포켓을 메우느라 프론티어가 나빠진다.
    pub hole_first: bool,
    /// 홀 우선 모드에서 포켓 후보에 내줄 후보 슬롯의 최대 비율 (0.0 ~ 1.0).
    ///
    /// 1.0 이면 포켓 후보가 `n_candidates` 를 전부 차지할 수 있다. 포켓 후보가
    /// 전부 CDE 에서 떨어지면 그 아이템은 시드를 통째로 날리므로, 기본값은 절반만
    /// 내주고 나머지는 기존 LBF 순서로 채운다.
    ///
    /// 살아 있는 노브다(`shirts` 실측: 0.25/0.50/1.00 에서 앞자리 채택
    /// 643/961/1349, 실제 배치 21/27/27). 다만 지금까지 잰 두 인스턴스에서는
    /// 최종 폭이 값에 따라 달라지지 않았다 — 포켓 후보가 상한에 닿을 만큼
    /// 많지 않거나(wsjk156), 많아도 결과가 같았다(shirts).
    pub hole_share: f32,
}

/// 탐색 단계 축소 스텝의 적응 방식.
///
/// 두 축이 있다. **밀도 기반**은 "지금 얼마나 꽉 찼는가"라는 사전지식으로 스텝을 정하는
/// 개루프이고, **실패 피드백**은 "방금 그 스텝이 과했는가"를 보고 고치는 폐루프다.
/// 서로 독립이므로 하나만 쓸 수도, 곱해서 쓸 수도 있다.
///
/// 실효 스텝은 `shrink_step × density_mult × step_scale`이다.
/// 이 모드가 밀도 배율과 기존 성공/실패 피드백 중 무엇을 사용할지 정한다.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShrinkAdaptMode {
    /// 적응 없음. `shrink_step`을 그대로 쓴다 (upstream 원본 동작).
    Fixed,
    /// 기존 밀도 기반: `mult = clamp(1 + gain × (1 − density), 1, max_mult)`.
    ///
    /// 기본 밀도 제안식이다. 밀도의 **절대값**을 쓰기 때문에
    /// 도달 가능 밀도가 다른 인스턴스끼리 같은 gain이 다르게 동작한다. 기본 gain 4.0에서는
    /// `max_mult` clamp가 `density ≤ 0`에서만 걸려 사실상 죽은 옵션이 된다.
    DensityLegacy,
    /// 개선된 밀도 기반: 런 **자신의 시작 밀도**를 기준으로 정규화한다.
    ///
    /// ```text
    /// progress = clamp((density − density_start) / (1 − density_start), 0, 1)
    /// mult     = 1 + (max_mult − 1) × (1 − progress)^gain
    /// ```
    ///
    /// 시작 시 `max_mult`, 이론 최대 밀도에 붙으면 1로 수렴한다. 기준이 인스턴스마다
    /// 자기 자신이므로 절대 밀도에 의존하지 않고, `max_mult`가 실제로 살아 있는 값이 된다.
    /// `gain`은 감쇠 모양이다 — 1이면 선형, 클수록 같은 양의 진행도에서 더 빨리 감소한다.
    DensitySlack,
    /// 밀도 배율은 1로 두고 명시적 실패 피드백 계수를 사용한다.
    Feedback,
    /// [`ShrinkAdaptMode::DensitySlack`] × 실패 피드백.
    DensitySlackFeedback,
}

impl ShrinkAdaptMode {
    /// 밀도 배율을 계산하는 모드인가.
    pub fn uses_density(self) -> bool {
        matches!(
            self,
            ShrinkAdaptMode::DensityLegacy
                | ShrinkAdaptMode::DensitySlack
                | ShrinkAdaptMode::DensitySlackFeedback
        )
    }

    /// 실패 피드백 배율을 계산하는 모드인가.
    pub fn uses_feedback(self) -> bool {
        matches!(
            self,
            ShrinkAdaptMode::Feedback | ShrinkAdaptMode::DensitySlackFeedback
        )
    }

    /// C ABI 정수 코드에서 복원한다. 알 수 없는 값은 `None`.
    pub fn from_code(code: i32) -> Option<Self> {
        match code {
            0 => Some(ShrinkAdaptMode::Fixed),
            1 => Some(ShrinkAdaptMode::DensityLegacy),
            2 => Some(ShrinkAdaptMode::DensitySlack),
            3 => Some(ShrinkAdaptMode::Feedback),
            4 => Some(ShrinkAdaptMode::DensitySlackFeedback),
            _ => None,
        }
    }

    /// C ABI 정수 코드.
    pub fn code(self) -> i32 {
        match self {
            ShrinkAdaptMode::Fixed => 0,
            ShrinkAdaptMode::DensityLegacy => 1,
            ShrinkAdaptMode::DensitySlack => 2,
            ShrinkAdaptMode::Feedback => 3,
            ShrinkAdaptMode::DensitySlackFeedback => 4,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ExplorationConfig {
    /// Opt-in bounded repairs after separation stagnation; Explore only.
    pub repair: SeparationRepairConfig,
    pub shrink_step: f32,
    pub time_limit: Duration,
    pub max_conseq_failed_attempts: Option<usize>,
    pub solution_pool_distribution_stddev: f32,
    pub separator_config: SeparatorConfig,
    pub large_item_ch_area_cutoff_percentile: f32,
    /// 파괴-재생성(ruin & recreate) 설정. adaptive 빌드에서만 반영.
    pub rnr: RuinRecreateConfig,
    /// 축소 스텝 적응 방식. 밀도 기반과 실패 피드백 중 무엇을 쓸지 고른다.
    /// adaptive 전용. 기본 밀도 제안은 [`ShrinkAdaptMode::DensityLegacy`].
    pub shrink_adapt_mode: ShrinkAdaptMode,
    /// 밀도 기반 적응의 강도. 모드마다 의미가 다르다.
    ///
    /// - [`ShrinkAdaptMode::DensityLegacy`]: `mult = clamp(1 + gain × (1 − density), 1, max_mult)`.
    ///   0.0이면 밀도 배율이 1이다.
    /// - [`ShrinkAdaptMode::DensitySlack`]: 감쇠 지수. `mult = 1 + (max_mult − 1) × (1 − progress)^gain`.
    ///   1이면 선형 감쇠, 클수록 같은 진행도에서 더 빨리 감소한다. 0이면 `max_mult` 고정.
    ///
    /// adaptive 빌드에서만 반영. 기본값 4.0.
    pub adaptive_shrink_gain: f32,
    /// 적응형 shrink의 `shrink_step` 대비 최대 배율 상한. adaptive 전용. 기본값 5.0.
    ///
    /// 주의 — `mult = clamp(1 + gain * (1 - density), 1, max_mult)` 에서 기본 gain 4.0이면
    /// `density <= 0` 에서만 상한에 닿는다. 즉 **기본값에서는 이 clamp가 발동하지 않는다.**
    /// gain을 10 이상으로 올려야 의미가 생긴다.
    pub adaptive_shrink_max_mult: f32,
    /// 실패 피드백 shrink: 어떤 폭에서 분리에 처음 실패하면 축소 스텝 배율에 곱하는 감쇠 계수.
    ///
    /// 밀도 기반은 밀도만 보는 **개루프**라 "이 스텝이 과했다"는 사실을 학습하지 못한다.
    /// 이 값은 압축 단계의 [`ShrinkDecayStrategy::FailureBased`]와 같은 관례로 닫힌 루프를
    /// 만든다 — 해당 폭에서 처음 실패하면 감쇠하고, 성공하면 한 단계 회복한다.
    ///
    /// 같은 폭에서의 반복 실패는 세지 않는다(폭 하나당 한 번). 그러지 않으면 교란 재시도만으로
    /// 스텝이 순식간에 하한까지 무너진다.
    ///
    /// [`ExplorationConfig::shrink_adapt_mode`]가 피드백을 쓰는 모드일 때만 반영된다.
    /// `0 < r < 1` 범위여야 하며, 벗어나면 기본값으로 정규화된다. adaptive 전용. 기본값 0.6.
    pub shrink_fail_decay: f32,
    /// 실패 피드백 shrink: 스텝 배율의 하한. adaptive 전용. 기본값 0.05.
    pub shrink_scale_min: f32,
    /// 폭 진동: 같은 폭에서 이만큼 연속 실패하면 폭을 일시적으로 되돌렸다가 재접근한다.
    /// 0이면 비활성. adaptive 전용. 기본값 4.
    pub osc_fail_threshold: usize,
    /// 폭 진동: 되돌릴 폭 (base `shrink_step`의 배수). adaptive 전용. 기본값 3.0.
    ///
    /// 주의 — 실제 확장량은 `min(shrink_step * osc_expand_steps, headroom/2)` 이고
    /// `headroom = best_width - current_width` 는 항상 "직전 shrink 한 스텝"이다.
    /// 따라서 이 값이 `adaptive_shrink_max_mult / 2` 를 넘으면 언제나 headroom 쪽에
    /// 잘려 값이 무의미해진다. 최선 feasible 폭을 넘는 탈출이 필요하면
    /// [`ExplorationConfig::relief_fail_threshold`] 를 쓴다.
    pub osc_expand_steps: f32,
    /// 폭 완화 복구(width relief): 같은 폭에서 이만큼 연속 실패하면, 그 폭의 최선
    /// 충돌 해를 **그대로 둔 채** 폭을 조금씩 넓혀 가며 충돌이 풀릴 때까지 재분리한다.
    /// 풀리면 넓어진 그 배치에서 정상 축소를 재개한다.
    ///
    /// 폭 진동([`ExplorationConfig::osc_fail_threshold`])과의 차이 — 진동은 확장량이
    /// 최선 feasible 폭까지 남은 여유의 절반으로 clamp 되어 그 폭을 절대 넘지 못한다.
    /// 완화 복구의 상한은 `relief_max_ratio` 뿐이라 최선 feasible 폭을 **넘어설 수
    /// 있다**. 인컴번트(`feasible_sols`)는 따로 보존되므로 결과는 나빠지지 않는다.
    ///
    /// 0이 아니면 이 값이 폭 탈출 전략을 결정하고 `osc_fail_threshold` 는 무시된다.
    /// 0이면 비활성(기존 폭 진동 동작). adaptive 전용. 기본값 0.
    pub relief_fail_threshold: usize,
    /// 폭 완화 복구: 라운드 1회의 확장 비율(현재 폭 대비). adaptive 전용. 기본값 0.002.
    pub relief_step: f32,
    /// 폭 완화 복구: 막힌 폭 대비 누적 확장 상한 비율. adaptive 전용. 기본값 0.03.
    pub relief_max_ratio: f32,
    /// 폭 완화 복구: 한 번의 복구에서 시도할 확장 라운드 수 상한. adaptive 전용. 기본값 12.
    pub relief_max_rounds: usize,
    /// 폭 완화 복구: 확장 라운드 1회에 허용할 separation strike 한도.
    /// 탐색이 아니라 "매듭이 풀리는지"만 보므로 짧게 잡는다. adaptive 전용. 기본값 1.
    pub relief_strike_limit: usize,
    /// 폭 완화 복구: 확장 라운드 1회의 벽시계 예산. `Duration::ZERO`면 제한 없음.
    ///
    /// strike 한도만으로는 한 strike 안에서 `iter_no_imprv_limit`(대형 인스턴스는 수백 회)
    /// 를 다 태울 수 있어, 초 단위 예산에서는 복구 한 번이 예산 전체를 먹을 수 있다.
    /// 이 값을 주면 라운드마다 그만큼만 쓰고 다음 폭으로 넘어간다.
    /// adaptive 전용. 기본값 `Duration::ZERO`(제한 없음).
    pub relief_round_budget: Duration,
    /// ALNS disruption 연산자 가중치의 지수 감쇠 계수 (0~0.999 로 잘린다).
    /// 교란 뒤 분리 결과로 `w ← decay·w + (1−decay)·score` 로 갱신한다. score 는 0~10 이다
    /// (겹침 해소 10 / loss 10% 이상 감소 5 / 감소 1 을 소요 시간과 인증된 폭 이득으로 보정).
    /// 클수록 과거 성과를 오래 기억한다. 가중치는 R&R 차례에 어느 R&R 전략을 쓸지 고르는 데
    /// 쓰이므로, R&R 이 꺼져 있으면(`rnr.period == 0`) 영향이 없다. adaptive 전용. 기본값 0.85.
    pub alns_decay: f32,
    /// ALNS 가중치 하한 (탐험 유지 — 어떤 연산자도 완전히 죽지 않게).
    /// 연속 실패한 연산자도 이 값 아래로는 내려가지 않는다. adaptive 전용. 기본값 0.1.
    pub alns_min_weight: f32,
    /// ★ 탐색 루프의 분리 1회 벽시계 상한. `Duration::ZERO` = 제한 없음(기본).
    ///
    /// 벽 근처 폭에서 분리 한 번이 수십 초를 쓰는데(여성WSjk 156, 3코어: 37초 실패 후
    /// 교란 1회, 재분리 89초 성공), 그 동안 호스트 화면은 «개선 없음» 으로 보인다. 상한이
    /// 오면 그 폭을 «지금 배치로는 못 푼다» 로 보고 실패 분기(풀·교란·진동)로 넘긴다 —
    /// 즉 막힌 폭을 **건너뛰고** 다른 후보를 본다. adaptive 전용.
    ///
    /// # 실측 결과 — 기각 (`docs/EXPLORE_STALL_AND_STOP_2026-09-30.md`)
    ///
    /// 20초 상한(+포트폴리오 4) 3시드 240초: 평균 **−0.5%p**. 벽 근처의 긴 분리는
    /// 대부분 결국 성공하는 분리라, 끊으면 그 성공을 버린다. 기본 꺼짐.
    pub sep_time_cap: Duration,
    /// ★ 분리 1회 상한을 **실행 중 관측한 성공 시간**에서 정한다. 기본 꺼짐. adaptive 전용.
    ///
    /// 고정 상한(`sep_time_cap`)은 마커마다 맞는 값이 다르다. 2026-10-01 실측(3코어, 480초)에서
    /// 성공하는 분리는 babel 겉감 200조각이 10초 안, 여성WSjk 156 이 25초 안에 끝났고, 실패하는
    /// 분리는 각각 40초·56초까지 갔다. 상한을 «이 실행에서 가장 오래 걸린 성공 × 1.25» 로 두면
    /// 두 마커에서 12초·30초 근처가 된다(고정 상한 실험에서 손해가 없던 값).
    ///
    /// 켜면 `sep_time_cap` 은 무시된다. 규칙은 `explore_mods::SepCapTuner` 에 있다 — 성공 표본이
    /// 모이기 전에는 상한이 없고, 상한에 걸린 실패가 이어지면 상한을 늘린다(짧은 상한에 갇히지
    /// 않게).
    pub sep_cap_adaptive: bool,
    /// ★ 분리 실패 뒤 교란 후보를 몇 개 탐침할지. `0`/`1` = 종전(후보 1개, 탐침 없음).
    ///
    /// 종전에는 ALNS 가 고른 교란 **하나**를 적용하고 곧바로 전체 예산의 분리에 들어간다.
    /// 그 교란이 나쁘면(무작위 큰 조각 교환이 대부분이다) 수십 초를 통째로 잃는다.
    /// `K > 1` 이면 같은 출발 해에 서로 다른 교란 K개를 적용하고 각각 `probe_budget` 만큼만
    /// 분리해 본 뒤, 남은 충돌(loss)이 가장 작은 후보에서 본 분리를 이어간다. adaptive 전용.
    ///
    /// # 실측 결과 — 잡음 안 (`docs/EXPLORE_STALL_AND_STOP_2026-09-30.md`)
    ///
    /// K=4 × 2초, 3시드 240초: 평균 −0.2%p (시드 잡음 0.6%p 안). 240초 안에 분리 실패가
    /// 1~2회뿐이라 발동 기회가 적다. 기본 꺼짐.
    pub portfolio_k: usize,
    /// ★ 후보 하나의 탐침 예산. `Duration::ZERO` 면 2초. adaptive 전용.
    pub probe_budget: Duration,
}

/// 파괴-재생성(LNS ruin & recreate) 설정. adaptive 빌드에서만 반영된다.
///
/// 세로 밴드 하나를 통째로 비우고 그 아이템들을 regret 순서로 다시 삽입한다.
/// 국소 최적 탈출용. `period`가 정한 disruption cadence마다 네 R&R 전략 중
/// 하나를 ALNS 성공률 가중치로 선택한다.
#[derive(Debug, Clone, Copy)]
pub struct RuinRecreateConfig {
    /// 0이면 R&R 비활성. 1 이상이면 매 `period`번째 disruption에서
    /// R&R 연산자(우측/저밀도/파편화/랜덤 밴드) 하나를 실행한다. 기본값 3.
    pub period: usize,
    /// 파괴할 세로 밴드의 폭 (strip 폭 대비 비율). 기본값 0.18.
    pub band_fraction: f32,
    /// 한 번에 파괴할 아이템 수 상한. 기본값 40.
    pub max_items: usize,
    /// 밴드 안 이동 가능 조각을 **전부** 파괴한다 (`max_items` 상한과 무작위
    /// truncate 를 건너뛴다). 기본값 false.
    ///
    /// 기존 경로는 밴드에서 `max_items` 개를 무작위로 골라 남긴다. 남은 조각이
    /// 장애물로 버티므로 재배치가 "구간 전체를 다시 놓는" 것이 아니라 "빈틈에
    /// 끼워 넣는" 것이 된다. 이 플래그는 그 제약을 없앤다.
    pub full_band: bool,
    /// 재삽입 LBF 평가기의 x/y 편향. `(0,0)` 이면 내부 기본값 `(10.0, 1.0)`.
    ///
    /// 기본값은 좌측 우선(x 가중 10배)이라 재삽입이 언제나 **같은 좌하단 규칙**을
    /// 재현한다. 구간을 실제로 다시 **배열**하려면 이 규칙 자체를 바꿔야 한다.
    /// 예: `(1.0, 10.0)` 은 세로(아래)부터 채운다.
    pub recreate_bias: (f32, f32),
    /// 축소 직후 우측 밴드를 통째로 다시 배열한다. 기본값 false.
    ///
    /// ALNS disruption 경로의 R&R 은 **실패한 분리** 뒤에만 돌기 때문에 이 마커에서
    /// 480초 동안 1~6회만 실행된다(실측). 축소는 같은 예산에서 13~14회 일어나므로,
    /// 축소 직후에 걸면 같은 연산자가 한 자릿수 더 자주 실행된다. 좁아진 폭에
    /// 밀려난 우측 조각들을 분리 루프에 넘기기 전에 새 폭 기준으로 다시 놓는다.
    pub post_shrink_rearrange: bool,
    /// 재배열을 유지할 최소 "빈자리 착석률". `0.0` 이면 검사하지 않는다(기본값).
    ///
    /// 재삽입은 충돌 없는 자리를 못 찾은 조각을 **원위치**로 되돌린다. 그런데 그
    /// 원위치는 방금 다른 조각이 먼저 앉았을 수 있어서, 이 폴백은 없던 겹침을
    /// 새로 만든다. 배치가 성길 때는 거의 모든 조각이 빈자리를 찾으므로 진짜
    /// 재배열이 되지만, 조밀해지면 대부분 폴백으로 떨어져 "재배열은 없고 겹침만
    /// 늘어난" 상태가 된다 — 실측에서 축소 직후 총 loss 가 4배로 뛰었다.
    ///
    /// 이 값은 그 경계를 밀도 상수로 고정하지 않고 결과로 판정한다. 빈자리에 앉은
    /// 조각 비율이 이 값 미만이면 트랜잭션을 통째로 롤백하고 0을 반환한다. 즉
    /// 성길 때는 통과하고 조밀해지면 저절로 꺼진다.
    pub min_clear_reinsert_ratio: f32,
    /// ★ 재삽입에서 재탐색이 찾은 자리를 버리고 **원위치**를 쓴다. 기본값 false.
    ///
    /// 재탐색은 «좌측 우선» 규칙으로 새 자리를 찾는데, 좌측은 이미 조밀해서 찾은 자리가
    /// 원위치보다 나쁠 수 있다 — loss 폭증(실측 x2.8~15.8)의 직접 원인이다. 켜면 R&R 이
    /// «아무것도 옮기지 않는» 상태가 되어 loss 악화가 0 이 된다. 즉 이 노브는 «손해 차단»
    /// 이지 «개선» 이 아니다. 재배열이 이득을 내려면 재탐색 규칙 자체가 «원위치보다 나은
    /// 자리» 를 찾아야 한다.
    pub prefer_original_position: bool,
    /// ★ 재삽입에서 **원위치를 후보로 포함**한다. 기본값 false.
    ///
    /// `prefer_original_position` 이 «재탐색 결과를 무조건 버린다» 인 반면, 이 노브는
    /// 재탐색 자리와 원위치를 **같은 LBF 점수로 비교**해 더 나은 쪽을 쓴다. 그리고
    /// 충돌 없는 자리를 못 찾은 조각은 **가장 먼저** 원위치에 앉혀 다른 조각이 그
    /// 자리를 차지하지 못하게 한다.
    ///
    /// # 왜
    ///
    /// 종전 재삽입은 자리를 못 찾은 조각을 regret `-inf` 로 **마지막에** 원위치로
    /// 돌려보낸다. 그 원위치는 앞서 앉은 조각이 이미 차지했을 수 있어 깊은 겹침이
    /// 생긴다 — 2026-09-14 실측의 «착석률 60% 이상인데 loss 가 3~8배» 는 나머지
    /// 40% 가 이렇게 겹친 결과로 설명된다. 켜면 조각은 «원위치보다 나쁘지 않은
    /// 빈자리» 로만 옮겨지므로 R&R 이 loss 를 늘리는 경로가 사실상 사라진다.
    pub original_candidate: bool,
    /// ★ 밴드 시작점 후보를 **실제 빈틈(포켓) 위치에서** 추가로 뽑는다. 기본값 false.
    ///
    /// 종전 후보는 `N_DENSITY_BANDS = 8` 개의 균등 간격뿐이었다. `span / 7` 이
    /// 1524mm 원단에서 약 **190mm** 이므로, 조각 하나 크기(수십~백여 mm)의 빈틈은
    /// **두 후보 사이에 완전히 빠진다.** 그래서 `LowDensity`/`Fragmented` 가
    /// «가장 빈 밴드» 를 골라도 그 밴드가 실제 최대 빈틈을 담고 있으리라는 보장이
    /// 없었다.
    ///
    /// 켜면 `OccupancyGrid::pockets()` 가 돌려주는 연결 성분(닫힌 주머니 + 우측
    /// 열린 만)을 **면적 내림차순**으로 정렬해, 상위 포켓의 좌측 경계를 밴드
    /// 시작점 후보로 추가한다. 균등 후보는 그대로 남으므로 «빈틈이 없는 구간» 도
    /// 계속 선택 가능하다 — 후보 집합이 넓어질 뿐 선택 규칙은 바뀌지 않는다.
    pub pocket_band_candidates: bool,
    /// `pocket_band_candidates` 로 추가할 포켓 개수 상한. 기본값 24.
    ///
    /// 포켓 수는 조각 수에 비례하므로 상한이 없으면 후보가 수백 개가 되어
    /// `Fragmented`/`GapTarget` 의 점수 계산(후보 × 포켓)이 급증한다.
    pub pocket_band_max: usize,
    /// ★ 빈틈 격자의 셀 크기 배율. 1.0 = 종전 동작, 작을수록 촘촘. 기본값 1.0.
    ///
    /// 종전 셀 크기는 `max(최소조각 최소변 × 0.5, √(원단면적 / 100000))` 이었다.
    /// 앞 항이 **하한**이라 최소 조각의 절반보다 얇은 포켓은 래스터에 **아예
    /// 안 잡힌다.** 그런 포켓은 «아무것도 못 들어가는 틈» 이 아니라 **«작은
    /// 조각은 들어가는 틈»** 이므로, 후보에서 사라지면 채울 기회 자체가 없어진다.
    ///
    /// 이 배율을 낮추면 하한이 내려가 좁은 포켓이 잡힌다. 대가는 셀 수 증가다
    /// (`FRAG_GRID_MAX_CELLS` 상한이 있어 폭주하지는 않는다).
    pub pocket_grid_cell_scale: f32,
    /// ★ 축소 직후 재배열의 **loss 악화 허용 배율**. `<=0` = 검사 안 함(기본값).
    ///
    /// 실측(2026-09-14, Explore 90초 seed 1): `full_band=1` 로 52개를 부수고
    /// 재삽입하면 매번 `requires_separation=true` 로 끝나고 **loss 가 4.6~10.5배**로
    /// 폭증한다(21395→144498, 32144→187065). 그런데 `min_clear_reinsert_ratio`
    /// 가드는 **한 번도 발동하지 않는다** — 52개가 «충돌 없이» 앉았기 때문이다.
    /// 즉 악화의 원인은 «폴백»이 아니라 **재배열 자체**다.
    ///
    /// 이 값은 재배열 **전후의 총 loss** 를 직접 비교한다. 재배열 후 loss 가
    /// `재배열 전 loss × 이 배율` 을 넘으면 트랜잭션을 롤백하고 0을 반환한다.
    /// 예: `2.0` 이면 loss 가 2배를 넘을 때 되돌린다.
    ///
    /// ★ 왜 loss 먼저인가: R&R 의 재삽입은 «각 조각이 합법» 만 보장하고
    /// **조합의 품질**은 보지 않는다. loss 는 그 조합 품질의 대리 지표다.
    /// `post_shrink_rearrange` 경로는 재배열 후 width 만 보고 loss 회복을 분리
    /// 루프에 맡기는데, 분리가 못 따라가면 그대로 손해가 남는다.
    pub max_loss_growth: f32,
    /// ★ 축소 직후 재배열에 쓸 **파괴 전략**. `0` = `Rightmost`(종전 하드코딩).
    ///
    /// 실측(2026-09-14): `Rightmost` 는 **우측 끝 = packing frontier** 를 통째로
    /// 재배열한다. 그 구간은 «가장 잘 맞춰진» 조각들이라, 꺼내서 좌측 우선으로
    /// 다시 놓으면 원래보다 나쁜 자리에 앉는다. 결과적으로 loss 가 항상
    /// **3.7~9.1배** 폭증하고, `max_loss_growth=2.0` 가드는 **43회 전부 거부**한다.
    ///
    /// 값: `0`=Rightmost `1`=LowDensity `2`=Fragmented `3`=GapTarget
    /// `4`=DensitySeam `5`=Random
    ///
    /// `LowDensity`/`Fragmented` 는 «비어 있는 곳» 을 고르므로 frontier 를
    /// 건드리지 않는다 — 이쪽이 재배열 이득을 낼 여지가 있다.
    pub post_strategy: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct CompressionConfig {
    /// Shrink-step schedule `(start, stop)`. `start` is the larger initial
    /// fraction and `stop` is the smaller termination threshold.
    pub shrink_range: (f32, f32),
    pub time_limit: Duration,
    pub shrink_decay: ShrinkDecayStrategy,
    pub separator_config: SeparatorConfig,
}

/// Converts two positive finite shrink values to the internal `(start, stop)`
/// order. External ABI fields retain their legacy `min`/`max` names, so both
/// caller orders are accepted at the boundary.
#[inline]
pub fn normalize_shrink_range(range: (f32, f32)) -> Option<(f32, f32)> {
    let (a, b) = range;
    if !a.is_finite() || !b.is_finite() || a <= 0.0 || b <= 0.0 {
        return None;
    }
    Some((a.max(b), a.min(b)))
}

/// 후처리(polish) 단계 설정.
///
/// 탐색/압축이 끝난 **feasible** 해를 입력으로 받아, 확률적 재배치 대신
/// 결정론적인 정리 패스를 적용한다. 모든 패스는 "검증된 이동만 반영"하므로
/// 입력 해의 실행 가능성(feasibility)을 절대 깨뜨리지 않는다.
#[derive(Debug, Clone, Copy)]
pub struct PolishConfig {
    /// 전체 후처리 단계 on/off
    pub enabled: bool,
    /// LP/slide retry budget, capped by any shorter parent deadline or external stop.
    pub time_limit: Duration,
    /// Legacy field retained for API compatibility; active LP/slide rounds run
    /// until the time budget or stop, regardless of this value or improvement.
    pub n_rounds: usize,
    pub slide: SlideConfig,
    pub hole_fill: HoleFillConfig,
    pub lp: LpCompactionConfig,
    /// NFP 로 같은 높이 근처의 가장 왼쪽 빈자리를 찾아 조각을 옮긴다.
    pub nfp_left: NfpLeftConfig,
    /// 폴리시가 남긴 예산을 압축 재진입으로 회수한다.
    pub reclaim: ReclaimConfig,
}

/// Polish 의 **NFP 왼쪽 재배치** 패스 설정.
///
/// slide 는 조각을 똑바로 밀다가 처음 막히는 곳에서 멈춘다. 위아래 조각과 비스듬히
/// 맞물린 조각은 그래서 왼쪽 빈자리를 두고도 움직이지 못한다. 이 패스는 주변 조각과의
/// NFP 를 수평선 몇 개로 잘라, 그 높이들에서 겹치지 않는 가장 왼쪽 자리로 조각을 옮긴다
/// (막힌 구간을 건너뛴 빈자리도 포함한다). 모든 이동은 CDE 와 원본 외곽 계약으로 검증한다.
///
/// slide 를 보강하는 패스라 `slide.enabled` 일 때만, 남은 Polish 예산이 2초 이상인
/// 호출에서만 돈다.
#[derive(Debug, Clone, Copy)]
pub struct NfpLeftConfig {
    pub enabled: bool,
    /// 현재 높이에서 위아래로 살펴볼 범위 (조각 높이 대비 비율).
    pub y_band_ratio: f32,
    /// 현재 높이 위·아래 각각에 둘 수평선 수.
    pub n_lines_each_side: usize,
    /// 왼쪽으로 찾아볼 최대 거리 (조각 폭 대비 배수).
    pub reach_ratio: f32,
    /// 이보다 덜 움직이는 자리는 버린다 (mm).
    pub min_gain_mm: f32,
    /// 조각 하나에서 검증해 볼 후보 수 상한.
    pub max_verify: usize,
}

/// **잔여 예산 회수** — 폴리시가 수렴하고 남긴 시간을 압축 재진입에 쓴다.
///
/// # 왜 필요한가
///
/// 폴리시는 시계가 아니라 **고정점**에서 끝난다. 실측(52조각 OPS2 마커, 예산 60s):
/// 2.4~7.4초만 쓰고 종료한다. 그 잔여는 지금 그냥 버려진다 — 단계마다 자기
/// timeout 만 걸고 이월이 없기 때문이다.
///
/// # 왜 하필 압축인가
///
/// 폴리시를 더 돌리는 것은 **이득이 없다**. 같은 압축 결과에서 출발해 재 봤다:
///
/// ```text
/// 기본                2.4s ( 3.9%)  폭 3595.77
/// 라운드 100          8.6s (14.4%)  폭 3595.77   동일
/// LP 반복 400         8.8s (14.7%)  폭 3595.77   동일
/// NFP 홀채움 ON      60.0s (100 %)  폭 3595.77   동일 — 예산은 다 쓰지만 소득 없음
/// 신뢰영역 x4        60.0s (100 %)  폭 3599.09   나빠짐
/// ```
///
/// 정제 계열에 시간을 더 부으면 예산은 소진되지만 폭은 그대로거나 나빠진다.
/// 반면 **압축을 폴리시 결과 위에서 다시 돌리면** 더 좁은 폭에서 축소 스케줄을
/// 새로 시작한다. 실측 4회(짝지은 within-run 측정):
///
/// ```text
/// -17.11mm / +0.00mm / +0.00mm / -8.00mm    적중 2/4, 평균 -6.28mm
/// ```
///
/// 기대치는 낮다. 다만 **한 번도 나빠지지 않는다** — 폭이 실제로 줄었을 때만
/// 교체하기 때문이다. 최악이 "이미 버려지던 시간을 헛씀" 이다.
///
/// # 총 소요 시간은 늘지 않는다
///
/// 회수 루프의 마감은 **폴리시가 이미 받은 예산의 끝**이다. 호출자가 준 적 없는
/// 시간은 쓰지 않는다.
/// 잔여 예산을 **무엇에** 쓸지.
///
/// # 두 계열은 성격이 다르다
///
/// 폴리시가 멈춘 지점은 **정제로는 못 벗어나는 고정점**이다. 실측으로 확인했다:
/// 라운드 100, LP 반복 400, 정확 NFP 홀채움(예산 100% 소진) 모두 폭이 완전히
/// 동일했고, 신뢰 영역을 키우면 오히려 나빠졌다.
///
/// * **정제(refine)** — 이미 좁은 해를 더 조인다. 압축 재진입이 여기 속한다.
///   더 좁아진 폭에서 축소 스케줄을 새로 시작하므로 고정점 자체는 그대로다.
/// * **탈출(escape)** — 배치를 일부러 **부수고** 다시 만든다. 조합 자체를 바꾸므로
///   고정점을 벗어날 수 있지만, 더 나빠진 채 끝날 수도 있다(그래서 폭 게이트가 있다).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReclaimMode {
    /// **압축 재진입** (기본). 폴리시 결과 위에서 압축을 다시 돌린다.
    ///
    /// 실측(52조각 OPS2 마커, 짝지은 within-run) 4회:
    /// `-17.11 / +0.00 / +0.00 / -8.00 mm`. 적중 2/4, 평균 -6.28mm.
    #[default]
    CompressReentry,
    /// **Ruin & Recreate 교란 후 재탐색.**
    ///
    /// 사이클마다 밴드 하나를 부수고(`lns::ruin_and_recreate`) 분리로 겹침을 푼 뒤,
    /// 남은 시간으로 탐색을 재개해 폭을 줄인다. `structure` 의 P4 repair 사이클과
    /// 같은 뼈대다(그쪽이 검증된 사용례다).
    ///
    /// R&R 자체는 폭을 줄이지 않는다 — **재탐색이 폭을 줄이는 부분**이고, R&R 은
    /// 그 재탐색이 같은 지역해로 되돌아가지 않게 만드는 교란이다.
    RuinRecreate,
    /// **적응형 탈출 탐색.**
    ///
    /// 정체를 감지해 단계적으로 강한 교란을 거는 탐색으로 잔여 시간을 돌린다
    /// (`escape::escape_driven_exploration_phase`).
    Escape,
}

impl ReclaimMode {
    /// C ABI 정수에서. 범위 밖은 기본값(압축 재진입)이다 — 경계에서 죽이지 않는다.
    pub fn from_raw(v: i32) -> Self {
        match v {
            1 => Self::RuinRecreate,
            2 => Self::Escape,
            _ => Self::CompressReentry,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::CompressReentry => "압축 재진입",
            Self::RuinRecreate => "R&R 교란+재탐색",
            Self::Escape => "적응형 탈출 탐색",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ReclaimConfig {
    /// **기본 켜짐.** 끄면 회수 루프는 `sep` 을 건드리지 않고 즉시 반환하므로,
    /// 끈 실행은 이 기능이 없던 때와 한 비트도 다르지 않다.
    ///
    /// # 켜면 재현성이 달라진다
    ///
    /// 사이클 수가 **실측 잔여 시간**의 함수다. 같은 시드라도 빠른 기계에서 더
    /// 많이 돌아 결과가 달라진다. 재현 가능한 실행이 필요하면 [`Self::max_cycles`]
    /// 를 고정하라 — 그러면 기계 속도와 무관해진다.
    pub enabled: bool,
    /// 한 사이클에서 남은 시간의 몇 할을 압축에 줄지 (나머지는 폴리시).
    ///
    /// 압축이 폭을 줄이고 폴리시가 그 결과를 다듬는 구조라 압축 쪽이 커야 한다.
    pub compress_share: f32,
    /// 사이클 수 상한. `0` 이면 예산이 다할 때까지.
    ///
    /// **재현성이 필요하면 이 값을 쓴다.** 사이클 수가 실측 잔여 시간의 함수라,
    /// 빠른 기계에서 더 많이 돌아 결과가 달라진다. 고정하면 그 흔들림이 사라진다.
    pub max_cycles: usize,
    /// 남은 시간이 이보다 적으면 시작하지 않는다.
    ///
    /// 압축 한 번은 스트립을 줄이고 분리를 돌려야 하므로, 지나치게 짧은 조각은
    /// 실패만 하고 끝난다.
    pub min_slice_ms: u64,
    /// 잔여 시간을 무엇에 쓸지. 기본은 [`ReclaimMode::CompressReentry`].
    pub mode: ReclaimMode,
    /// 탈출 계열에서 한 사이클에 부술 밴드 폭 (strip 폭 대비 비율).
    ///
    /// `structure` P4 의 검증된 값을 그대로 가져왔다.
    pub ruin_band_fraction: f32,
    /// 탈출 계열에서 한 사이클에 부술 조각 수 상한.
    pub ruin_max_items: usize,
    /// **탈출 계열에서** 개선 없는 사이클을 몇 번까지 참을지.
    ///
    /// 압축 재진입은 결정론적이라 한 번 실패하면 끝이다(같은 입력에 같은 결과).
    /// 탈출 계열은 **확률적**이라 한 번 못 찾았다고 수렴한 게 아니다 — 첫 실패에
    /// 멈추면 예산을 크게 남긴다(실측: R&R 이 60초 중 24초만 쓰고 36초를 남겼다).
    ///
    /// `structure` P4 의 `p4_stagnation_cycles` 와 같은 역할이고 기본값도 같다.
    pub stagnation_cycles: usize,
}

/// 회수 루프 계측치.
#[derive(Debug, Clone, Copy, Default)]
pub struct ReclaimStats {
    /// 실제로 돈 사이클 수.
    pub cycles: usize,
    /// 폭이 줄어 채택된 사이클 수.
    pub accepted: usize,
    /// 회수 전 폭.
    pub width_before: f32,
    /// 회수 후 폭.
    pub width_after: f32,
    /// 끝내 못 쓰고 남긴 시간.
    pub unused_ms: u64,
    /// 종료 시점의 연속 무개선 사이클 수.
    ///
    /// `cycles` 와 `accepted` 만으로는 "예산이 다해서 끝났는지" 와 "정체 상한에
    /// 걸려 끝났는지" 를 구분할 수 없다.
    pub stagnant: usize,
}

/// 결정론적 좌측/하단 슬라이딩(compaction pass) 설정.
#[derive(Debug, Clone, Copy)]
pub struct SlideConfig {
    pub enabled: bool,
    /// 한 라운드 안에서 슬라이딩을 반복할 횟수 (더 이상 움직이지 않으면 조기 종료)
    pub n_passes: usize,
    /// 좌측 슬라이딩 후 하단 슬라이딩도 수행할지
    pub slide_down: bool,
    /// 실제로 이동을 반영할 최소 거리 (아이템 최소 변 길이 대비 비율)
    pub min_move_ratio: f32,
    /// 접촉 지점을 찾기 위한 전방 스캔 스텝 (아이템 최소 변 길이 대비 비율)
    pub scan_step_ratio: f32,
    /// 스캔으로 좁힌 구간을 이분탐색할 횟수
    pub n_bisections: usize,
    /// 접촉 시 남겨둘 안전 간격 (아이템 최소 변 길이 대비 비율)
    pub safety_ratio: f32,
    /// 왼쪽이 막힌 조각은 위·아래로 비스듬한 방향으로도 밀어 보고 가장 왼쪽으로
    /// 가는 방향을 고른다.
    pub diagonal: bool,
    /// 겹침 허용 모드에서 원본 외곽 겹침 계약을 지키는 지점에서 멈춘다. 끄면 줄어든
    /// 충돌 도형이 맞닿을 때까지 밀어, 그 라운드가 원본 검증에서 통째로 되돌려질 수 있다.
    pub original_aware: bool,
}

/// 타깃형 hole filling(빈 포켓 탐지 후 조각 이동) 설정.
#[derive(Debug, Clone, Copy)]
pub struct HoleFillConfig {
    pub enabled: bool,
    /// occupancy grid 셀 크기 (가장 작은 아이템의 최소 변 길이 대비 비율)
    pub cell_size_ratio: f32,
    /// grid 셀 총 개수 상한 (셀 크기는 이 상한에 맞춰 자동으로 커진다)
    pub max_cells: usize,
    /// 포켓의 여유 면적이 아이템 면적의 몇 배 이상이어야 시도할지
    pub min_pocket_area_ratio: f32,
    /// 개별 재배치 후보 수 상한 (작은 면적 우선).
    /// 공동 배정은 이 한도 안에서 가장 우측의 강체 단위를 최대 4개 사용한다.
    pub max_candidates: usize,
    /// 후보 선정 구간: strip 우측 몇 %에 걸쳐 있는 아이템을 대상으로 할지
    pub candidate_band: f32,
    /// 포켓 하나당 평가할 랜덤 샘플 수
    pub n_samples_per_pocket: usize,
    /// 아이템 하나당 시도할 포켓 수 상한 (왼쪽 포켓 우선)
    pub max_pockets_per_item: usize,
    /// 좌표 하강으로 정제할 상위 샘플 수
    pub n_coord_descents: usize,
}

/// LP compaction 설정.
///
/// 강체 단위별 평행이동 (dx, dy)과 strip 폭 W를 변수로 하는 선형계획을 세우고,
/// 접촉 쌍에 대해 정점-모서리 분리 제약을 선형화하여 폭을 최소화한다.
/// 비볼록 도형이라 선형화가 완전하지 않으므로, 해를 적용한 뒤 반드시 CDE로
/// 전수 검증하고 실패 시 스텝을 절반씩 줄여 재시도한다(백트래킹).
#[derive(Debug, Clone, Copy)]
pub struct LpCompactionConfig {
    pub enabled: bool,
    /// LP 수립 → 적용 반복 횟수
    pub n_iterations: usize,
    /// 신뢰 영역의 **기본(가장 촘촘한) 단계** 크기, strip 폭 대비 비율.
    /// 개선이 막히면 내부적으로 2배씩 키워 가며 재시도한다.
    pub trust_region_ratio: f32,
    /// 좌측으로 당기는 목적항 가중치 (폭 W의 계수 1.0 대비)
    pub left_pull_weight: f32,
    /// 접촉 제약을 생성할 간극 임계값 (신뢰 영역 대비 배수)
    pub clearance_factor: f32,
    /// 아이템 쌍당 생성할 제약 수 상한
    pub max_constraints_per_pair: usize,
    /// LP 전체 제약 수 상한 (초과 시 LP 단계를 건너뛴다)
    pub max_constraints: usize,
    /// 해 적용 시 스텝을 절반으로 줄여 재시도할 최대 횟수
    pub max_backtracks: usize,
}

#[derive(Debug, Clone, Copy)]
pub enum ShrinkDecayStrategy {
    /// The shrink ratio decays linearly with time
    TimeBased,
    /// The shrink ratio decays by a fixed ratio every time it fails to compress into a feasible solution
    FailureBased(f32),
}

pub const DEFAULT_SWARM_CONFIG: SwarmConfig = SwarmConfig {
    rng_seed: None,
    expl_cfg: ExplorationConfig {
        repair: SeparationRepairConfig::DEFAULT,
        shrink_step: 0.001,
        time_limit: Duration::from_secs(9 * 60),
        max_conseq_failed_attempts: None,
        solution_pool_distribution_stddev: 0.25,
        separator_config: SeparatorConfig {
            iter_no_imprv_limit: 200,
            strike_limit: 3,
            n_workers: 3,
            sample_config: SampleConfig {
                n_container_samples: 50,
                n_focussed_samples: 25,
                n_coord_descents: 3,
                left_bias_gain: 2.0,
                tilt_range: 0.0,
            },
            swap_period: 0,
            swap_attempts: 8,
            swap_area_ratio: (0.4, 2.5),
            // 기본 꺼짐. `NfpIntegrationConfig::nfp_sampler` 에서 전파된다.
            nfp_sampler: false,
            gls: crate::optimizer::separator::GlsWeightConfig::DEFAULT,
            restart_window: 0,
            iter_floor_override: 0,
        },
        large_item_ch_area_cutoff_percentile: 0.75,
        rnr: RuinRecreateConfig {
            period: 0,
            band_fraction: 0.18,
            max_items: 40,
            full_band: false,
            recreate_bias: (0.0, 0.0),
            post_shrink_rearrange: false,
            min_clear_reinsert_ratio: 0.0,
            prefer_original_position: false,
            original_candidate: false,
            // ★ 기본 꺼짐 — 후보 집합 확장은 선택 규칙을 바꾸므로 측정으로만 켠다.
            pocket_band_candidates: false,
            pocket_band_max: 24,
            pocket_grid_cell_scale: 1.0,
            max_loss_growth: 0.0,
            post_strategy: 0,
        },
        shrink_adapt_mode: ShrinkAdaptMode::DensityLegacy,
        adaptive_shrink_gain: 4.0,
        adaptive_shrink_max_mult: 5.0,
        shrink_fail_decay: 0.6,
        shrink_scale_min: 0.05,
        osc_fail_threshold: 4,
        osc_expand_steps: 3.0,
        relief_fail_threshold: 0,
        relief_step: 0.002,
        relief_max_ratio: 0.03,
        relief_max_rounds: 12,
        relief_strike_limit: 1,
        relief_round_budget: Duration::ZERO,
        alns_decay: 0.85,
        alns_min_weight: 0.1,
        sep_time_cap: Duration::ZERO,
        sep_cap_adaptive: false,
        portfolio_k: 0,
        probe_budget: Duration::ZERO,
    },
    cmpr_cfg: CompressionConfig {
        shrink_range: (0.0005, 0.00001),
        time_limit: Duration::from_secs(60),
        shrink_decay: ShrinkDecayStrategy::TimeBased,
        separator_config: SeparatorConfig {
            iter_no_imprv_limit: 100,
            strike_limit: 5,
            n_workers: 3,
            sample_config: SampleConfig {
                n_container_samples: 50,
                n_focussed_samples: 25,
                n_coord_descents: 3,
                left_bias_gain: 2.0,
                tilt_range: 0.0,
            },
            swap_period: 0,
            swap_attempts: 8,
            swap_area_ratio: (0.4, 2.5),
            // 기본 꺼짐. `NfpIntegrationConfig::nfp_sampler` 에서 전파된다.
            nfp_sampler: false,
            gls: crate::optimizer::separator::GlsWeightConfig::DEFAULT,
            restart_window: 0,
            iter_floor_override: 0,
        },
    },
    cde_config: CDEConfig {
        quadtree_depth: 4,
        cd_threshold: 64,
        item_surrogate_config: SPSurrogateConfig {
            n_pole_limits: [(64, 0.0), (16, 0.8), (8, 0.9)],
            ff_pole_area_ratio: 0.5,
            n_ff_piers: 0,
        },
    },
    poly_simpl_tolerance: Some(0.001),
    simplify_max_deviation_mm: None,
    narrow_concavity_cutoff_ratio: Some((0.01, 0.01)),
    min_item_separation: None,
    original_overlap_policy: OriginalOverlapPolicy::Infeasible,
    place_group_mode: PlaceGroupMode::MultiPolygon,
    fabric_edge_margin_mm: None,
    init_compaction_ms: 0,
    init_band_width_mm: None,
    init_band_fill_ratio: 0.0,
    init_settle_every: 0,
    init_settle_budget_ms: 200,
    lbf_x_multiplier: 10.0,
    lbf_y_multiplier: 1.0,
    polish_cfg: DEFAULT_POLISH_CONFIG,
    lbf_raster: DEFAULT_RASTER_NFP_CONFIG,
    allow_180_rotation: false,
    structure: crate::structure::config::DEFAULT_STRUCTURE_FIRST_CONFIG,
    escape_enabled: false,
    escape: DEFAULT_ADAPTIVE_ESCAPE_CONFIG,
    nfp_lbf: DEFAULT_NFP_LBF_SEED_CONFIG,
    nfp_integration: DEFAULT_NFP_INTEGRATION_CONFIG,
    nfp_strategy: crate::nfp_strategy::DEFAULT_NFP_STRATEGY_CONFIG,
    lp_solver: crate::lp_solver::DEFAULT_LP_SOLVER_CONFIG,
    refplace: DEFAULT_REFPLACE_SEED_CONFIG,
    lbf_order: crate::optimizer::lbf::LbfOrderMode::LargestFirst,
};

/// 전부 꺼짐. 각 항목은 근사를 정확한 NFP로 바꾸는 대신 질의 비용을 올리므로,
/// 반드시 A/B로 밀도와 시간을 재 보고 켜야 한다.
pub const DEFAULT_NFP_INTEGRATION_CONFIG: NfpIntegrationConfig = NfpIntegrationConfig {
    ordering_exploration: false,
    nfp_sampler: false,
    exact_separation_vector: false,
    nfp_hole_filling: false,
    nfp_reinsertion: false,
};

/// 문서가 정한 임계값을 그대로 담은 기본 탈출 설정.
pub const DEFAULT_ADAPTIVE_ESCAPE_CONFIG: crate::escape::AdaptiveEscapeConfig =
    crate::escape::AdaptiveEscapeConfig {
        stagnation: crate::escape::StagnationConfig {
            window_floor: crate::escape::STAGNATION_WINDOW_FLOOR,
            window_item_factor: crate::escape::STAGNATION_WINDOW_ITEM_FACTOR,
            efficiency_eps: crate::escape::STAGNATION_EFFICIENCY_EPS,
            diversity_threshold: crate::escape::STAGNATION_DIVERSITY_THRESHOLD,
        },
        conflict_burst: crate::escape::ConflictBurstConfig {
            remove_ratio_min: crate::escape::CONFLICT_REMOVE_RATIO_MIN,
            remove_ratio_max: crate::escape::CONFLICT_REMOVE_RATIO_MAX,
            local_move_mm: crate::escape::LOCAL_MOVE_MM,
            escape_move_mm: crate::escape::ESCAPE_MOVE_MM,
            escape_move_bbox_ratio: crate::escape::ESCAPE_MOVE_BBOX_RATIO,
            max_push_items: 16,
        },
        enable_conflict_burst: true,
        enable_swarm_diversification: true,
        max_infeasible_pool: 32,
    };

/// 문서 목표치를 그대로 담은 기본 NFP-LBF 시드 설정.
///
/// ```text
/// NFP cold cache          2.0 sec
/// LBF ordering 1회       100~200 ms
/// 50 orderings            5~10 sec
/// ```
/// 레퍼런스 설계 초기배치 엔진(`crate::refplace`) 스위치.
///
/// # 실험 단계다 — 켜기 전에 읽을 것
///
/// `레퍼런스-초기배치-설계.md` 를 그대로 구현한 별도 엔진이다. 구조는 완성됐고
/// 겹침 없는 유효한 배치를 내지만, **품질이 아직 기존 경로에 크게 못 미친다**.
///
/// 실측 (wsjk156 156조각, 2026-08-31):
///
/// ```text
///   기존 NFP-LBF                     폭  7,090mm   밀도 71.75%
///   전략 ⑧+⑦+⑨                      폭  6,900mm   밀도 73.72%
///   경쟁사                            폭  6,183mm   밀도 82.28%
///   refplace (이 엔진)                폭 20,312mm   밀도 25.04%   <- 여기
/// ```
///
/// 앞 5,200mm 구간의 충전율은 76%로 경쟁사(81%)에 크게 뒤지지 않는다. 무너지는 지점은
/// 하나다 — 배치가 진행되면 `subtract_obstacle` 이 실패하기 시작해(실측 46회) 남은
/// 조각이 프론티어 오른쪽 폴백으로 일렬로 밀린다. 그 원인은 아직 규명 중이다.
///
/// Legacy step Init compares the older reference constructor with its existing
/// complete certified layout. Partial, invalid or worse candidates are rejected.
/// Explicit Column Draft remains the primary constructor for fresh inputs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RefPlaceSeedConfig {
    /// Select bottom-to-top Column Draft instead of the older reference engine.
    pub column_draft: bool,
    /// Enables the selected initial constructor/candidate. Default off.
    pub enabled: bool,
    /// 그룹 정의가 없을 때 쓸 대체 규칙. `0` = 조각 하나가 한 그룹,
    /// `1` = 같은 `item_id` 사본끼리 한 그룹.
    ///
    /// 설계의 제1원칙이 "순서 = 그룹 정의 기반" 인데 입력의 그룹 메타데이터
    /// (`domain` 블록)를 솔버 로더가 버리므로, 지금은 이 대체 규칙만 쓸 수 있다.
    pub group_fallback: u8,
}

/// 기본값 — **꺼짐**.
pub const DEFAULT_REFPLACE_SEED_CONFIG: RefPlaceSeedConfig = RefPlaceSeedConfig {
    column_draft: false,
    enabled: false,
    group_fallback: 1,
};

impl Default for RefPlaceSeedConfig {
    fn default() -> Self {
        DEFAULT_REFPLACE_SEED_CONFIG
    }
}

impl RefPlaceSeedConfig {
    /// 대체 규칙을 코어 타입으로.
    pub fn fallback(&self) -> crate::refplace::GroupFallback {
        match self.group_fallback {
            1 => crate::refplace::GroupFallback::ByItemId,
            _ => crate::refplace::GroupFallback::EachPieceAlone,
        }
    }
}

pub const DEFAULT_NFP_LBF_SEED_CONFIG: NfpLbfSeedConfig = NfpLbfSeedConfig {
    enabled: false,
    n_orderings: crate::nfp_lbf::DEFAULT_N_ORDERINGS,
    cache_budget_ms: crate::nfp_lbf::DEFAULT_CACHE_BUDGET_MS,
    total_budget_ms: crate::nfp_lbf::DEFAULT_TOTAL_BUDGET_MS,
    allow_180: false,
    enable_prefix_reuse: true,
    concave_simplify_tolerance: crate::nfp_lbf::DEFAULT_CONCAVE_SIMPLIFY_TOLERANCE,
    score_weights: crate::nfp_lbf::PlacementScoreWeights {
        w1_strip_length: crate::nfp_lbf::DEFAULT_SCORE_W1_STRIP_LENGTH,
        w2_waste: crate::nfp_lbf::DEFAULT_SCORE_W2_WASTE,
        w3_contact: crate::nfp_lbf::DEFAULT_SCORE_W3_CONTACT,
        w4_y: crate::nfp_lbf::DEFAULT_SCORE_W4_Y,
        score_first: false,
        length_tie_mm: 0.0,
    },
    max_candidate_probes: crate::nfp_lbf::DEFAULT_MAX_CANDIDATE_PROBES,
};

/// 기본값은 **켜짐**이다 (capmods 빌드에서만 동작).
///
/// 래스터 NFP는 LBF 초기해를 확실히 개선한다 (실물 마커 여성WSjk 기준 초기 폭 −3.2%,
/// 표준 인스턴스 −0.4~−1.8%, 비용 0.2초 미만).
///
/// > **주의 — 근거를 재현할 수 없다.** 아래가 인용하는 `docs/bench/2026-08-07_REPORT.md`
/// > 는 **저장소에 없다.** 기본 **켜짐**인 기능의 근거가 확인 불가능한 상태다. 이 값을
/// > 신뢰하기 전에 `tests/competitor_gap.rs` 로 다시 재고 이 문단을 실측표로 바꿀 것.
///
/// 처음 측정에서는 탐색 단계가 초기해를 다시 짜는 탓에 최종 밀도 이득이 확인되지 않아
/// 기본 꺼짐이었으나,
/// 이후 후반부 항목에 exact NFP 접촉 슬라이딩(`optimizer/lbf.rs`, 배치 진행률 90% 이상)을
/// 덧붙이면서 기본값을 켜짐으로 바꿨다.
pub const DEFAULT_RASTER_NFP_CONFIG: RasterNfpConfig = RasterNfpConfig {
    enabled: true,
    cell_ratio: 0.12,
    max_cells: 250_000,
    n_candidates: 64,
    double_sided_first_n: 0,
    // 난수 샘플은 줄이지 않는다: 래스터 후보는 난수 탐색을 **대체**하는 게 아니라
    // 보강하는 쪽이 확실히 좋았다 (줄이면 오히려 초기 폭이 나빠졌다).
    n_random_samples: 1000,
    // 기본은 꺼짐. 켜면 격자 전수 스캔에서 조기 종료를 포기하므로 시드 시간이 늘고,
    // 후보 예산의 일부를 포켓에 내준다. 실측으로 이득을 확인하고 켤 것.
    hole_first: false,
    hole_share: 0.5,
};

pub const DEFAULT_POLISH_CONFIG: PolishConfig = PolishConfig {
    enabled: true,
    time_limit: Duration::from_secs(20),
    n_rounds: 3,
    reclaim: ReclaimConfig {
        // 기본 켜짐. 폴리시가 남긴 예산은 지금 그냥 버려지고, 회수는 폭이 실제로
        // 줄었을 때만 교체하므로 결과가 나빠질 수 없다 — 최악이 "이미 버려지던
        // 시간을 헛씀" 이다. 실측 이득은 적중 2/4, 평균 -6.28mm 로 크지 않다.
        //
        // 대가는 재현성이다: 사이클 수가 실측 잔여 시간의 함수라 기계 속도에 따라
        // 결과가 달라진다. 재현이 필요하면 `max_cycles` 를 고정한다.
        enabled: true,
        compress_share: 0.75,
        max_cycles: 0,
        min_slice_ms: 500,
        mode: ReclaimMode::CompressReentry,
        // structure P4 의 검증된 값 (p4_ruin_fraction 0.20 / p4_max_items 48).
        ruin_band_fraction: 0.20,
        ruin_max_items: 48,
        stagnation_cycles: 2,
    },
    slide: SlideConfig {
        enabled: true,
        n_passes: 3,
        slide_down: true,
        min_move_ratio: 0.002,
        scan_step_ratio: 0.5,
        n_bisections: 12,
        safety_ratio: 0.0,
        diagonal: false,
        original_aware: false,
    },
    nfp_left: NfpLeftConfig {
        // 기본 켜짐. 앱 설정(LP 켬, Polish 6초)에서 시드 8개 평균 길이 -0.64mm. 이긴
        // 시드에서는 -4.6mm 까지 줄였고, 진 시드는 +0.06mm 이내였다.
        enabled: true,
        y_band_ratio: 0.25,
        n_lines_each_side: 4,
        reach_ratio: 3.0,
        min_gain_mm: 0.5,
        max_verify: 8,
    },
    hole_fill: HoleFillConfig {
        enabled: false,
        cell_size_ratio: 0.25,
        max_cells: 400_000,
        min_pocket_area_ratio: 1.0,
        max_candidates: 64,
        candidate_band: 0.25,
        n_samples_per_pocket: 200,
        max_pockets_per_item: 8,
        n_coord_descents: 3,
    },
    lp: LpCompactionConfig {
        enabled: true,
        n_iterations: 40,
        trust_region_ratio: 0.005,
        left_pull_weight: 0.02,
        clearance_factor: 2.0,
        max_constraints_per_pair: 8,
        max_constraints: 200_000,
        max_backtracks: 5,
    },
};

/// Per-run dynamic exploration control. Legacy global functions set defaults
/// for future runs; a running handle always owns its own atomic control.
pub mod runtime {
    use std::cell::RefCell;
    use std::sync::{
        Arc,
        atomic::{AtomicU32, Ordering},
    };
    const NOT_SET: u32 = u32::MAX;
    static DEFAULT_SHRINK_STEP: AtomicU32 = AtomicU32::new(NOT_SET);
    thread_local! { static ACTIVE: RefCell<Option<Arc<RunControl>>> = const { RefCell::new(None) }; }

    #[derive(Debug)]
    pub struct RunControl {
        shrink_step: AtomicU32,
    }
    impl Default for RunControl {
        fn default() -> Self {
            Self {
                shrink_step: AtomicU32::new(NOT_SET),
            }
        }
    }
    impl RunControl {
        pub fn from_defaults() -> Arc<Self> {
            Arc::new(Self {
                shrink_step: AtomicU32::new(DEFAULT_SHRINK_STEP.load(Ordering::Relaxed)),
            })
        }
        pub fn set_shrink_step(&self, step: f32) -> bool {
            if !step.is_finite() || step <= 0.0 || step >= 1.0 {
                return false;
            }
            self.shrink_step.store(step.to_bits(), Ordering::Relaxed);
            true
        }
        pub fn clear_shrink_step(&self) {
            self.shrink_step.store(NOT_SET, Ordering::Relaxed);
        }
        pub fn shrink_step(&self) -> Option<f32> {
            decode(self.shrink_step.load(Ordering::Relaxed))
        }
    }
    fn decode(bits: u32) -> Option<f32> {
        (bits != NOT_SET).then(|| f32::from_bits(bits))
    }

    /// Must be dropped on the entering thread, including during unwinding.
    pub struct Scope {
        previous: Option<Arc<RunControl>>,
        _thread: std::marker::PhantomData<std::rc::Rc<()>>,
    }
    impl Drop for Scope {
        fn drop(&mut self) {
            ACTIVE.with(|slot| *slot.borrow_mut() = self.previous.take());
        }
    }
    pub fn enter(control: Arc<RunControl>) -> Scope {
        Scope {
            previous: ACTIVE.with(|slot| slot.replace(Some(control))),
            _thread: std::marker::PhantomData,
        }
    }
    pub fn current_control() -> Arc<RunControl> {
        ACTIVE
            .with(|slot| slot.borrow().clone())
            .unwrap_or_else(RunControl::from_defaults)
    }
    pub fn enter_if_unset(control: Arc<RunControl>) -> Scope {
        let active = ACTIVE.with(|slot| slot.borrow().clone());
        enter(active.unwrap_or(control))
    }
    /// Legacy API: defaults for future runs only, never a running handle.
    pub fn set_shrink_step(step: f32) -> bool {
        if !step.is_finite() || step <= 0.0 || step >= 1.0 {
            return false;
        }
        DEFAULT_SHRINK_STEP.store(step.to_bits(), Ordering::Relaxed);
        true
    }
    pub fn clear_shrink_step() {
        DEFAULT_SHRINK_STEP.store(NOT_SET, Ordering::Relaxed);
    }
    pub fn shrink_step_override() -> Option<f32> {
        decode(DEFAULT_SHRINK_STEP.load(Ordering::Relaxed))
    }
    pub fn effective_shrink_step(configured: f32) -> f32 {
        ACTIVE
            .with(|slot| slot.borrow().as_ref().and_then(|c| c.shrink_step()))
            .unwrap_or(configured)
    }

    // Cancellation is broadcast to Init calls that have already entered. A later
    // Init must neither erase an older call's stop request nor inherit that stop.
    static INIT_CANCEL_EPOCH: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    thread_local! {
        static INIT_TOKEN: std::cell::Cell<Option<InitCancelToken>> = const { std::cell::Cell::new(None) };
    }

    #[derive(Clone, Copy, Debug)]
    pub struct InitCancelToken(u64);
    impl InitCancelToken {
        pub fn is_cancelled(self) -> bool {
            INIT_CANCEL_EPOCH.load(Ordering::Acquire) != self.0
        }
    }
    pub struct InitCancelScope {
        previous: Option<InitCancelToken>,
        // Restores caller-thread state, so the scope must not move to another thread.
        _thread_bound: std::marker::PhantomData<std::rc::Rc<()>>,
    }
    impl Drop for InitCancelScope {
        fn drop(&mut self) {
            INIT_TOKEN.with(|slot| slot.set(self.previous));
        }
    }
    pub fn begin_init() -> InitCancelScope {
        let token = InitCancelToken(INIT_CANCEL_EPOCH.load(Ordering::Acquire));
        InitCancelScope {
            previous: INIT_TOKEN.with(|slot| slot.replace(Some(token))),
            _thread_bound: std::marker::PhantomData,
        }
    }
    /// Capture on the Init caller before dispatching work to Rayon.
    pub fn init_cancel_token() -> Option<InitCancelToken> {
        INIT_TOKEN.with(|slot| slot.get())
    }
    pub fn request_init_cancel() {
        INIT_CANCEL_EPOCH.fetch_add(1, Ordering::AcqRel);
    }
    pub fn init_cancel_requested() -> bool {
        init_cancel_token().is_some_and(InitCancelToken::is_cancelled)
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        #[test]
        fn init_cancellation_survives_later_calls_and_reaches_worker_tokens() {
            assert!(!init_cancel_requested());
            let outer = begin_init();
            let token = init_cancel_token().unwrap();
            assert!(!token.is_cancelled());
            use crate::util::terminator::{BasicTerminator, InitTerminator, Terminator};
            let term = InitTerminator::new(BasicTerminator::new());
            std::thread::spawn(request_init_cancel).join().unwrap();
            std::thread::spawn(move || assert!(term.kill() && term.stop_requested())).join().unwrap();
            assert!(init_cancel_requested());
            std::thread::spawn(move || assert!(token.is_cancelled())).join().unwrap();
            // A new Init after stop gets a fresh epoch, but cannot clear outer's request.
            {
                let _later = begin_init();
                assert!(!init_cancel_requested());
                assert!(token.is_cancelled());
            }
            assert!(init_cancel_requested());
            drop(outer);
            assert!(!init_cancel_requested());
            let _restart = begin_init();
            assert!(!init_cancel_requested());
        }

        #[test]
        fn concurrent_controls_are_isolated_and_invalid_updates_are_rejected() {
            let a = Arc::new(RunControl::default());
            let b = Arc::new(RunControl::default());
            a.set_shrink_step(0.02);
            b.set_shrink_step(0.04);
            std::thread::scope(|threads| {
                for (control, expected) in [(a.clone(), 0.02), (b.clone(), 0.04)] {
                    threads.spawn(move || {
                        let _scope = enter(control);
                        for _ in 0..1000 {
                            assert_eq!(effective_shrink_step(0.001), expected);
                        }
                    });
                }
            });
            for bad in [0.0, -0.1, 1.0, f32::NAN, f32::INFINITY] {
                assert!(!a.set_shrink_step(bad));
            }
            assert_eq!(a.shrink_step(), Some(0.02));
            let _scope = enter(a.clone());
            {
                let _nested = enter(b);
                assert_eq!(effective_shrink_step(0.001), 0.04);
            }
            assert_eq!(effective_shrink_step(0.001), 0.02);
            a.clear_shrink_step();
            assert_eq!(effective_shrink_step(0.001), 0.001);
        }
    }
}
