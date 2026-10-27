//! NFP 기반 Sequential Placement.
//!
//! ```text
//! 아이템 1 배치
//!    ↓
//! 아이템 2
//! → 기존 배치 아이템들과 NFP 생성/조회
//! → 모든 충돌 금지영역 합성
//! → feasible boundary 후보 평가
//! → LB 기준 최적 위치 선택
//!    ↓
//! 아이템 3
//! → 동일 반복
//!    ↓
//! ...
//! 아이템 156
//! ```
//!
//! 중요한 건 단순히 좌하단으로 조금씩 밀어보는 게 아니라, **NFP를 이용해서 실제로
//! 가능한 후보 위치들을 직접 평가하는 것**이다.
//!
//! 새 아이템 `P`를 넣을 때 기존 배치가 `A, B, C`라면:
//!
//! ```text
//! NFP(A,P)
//! NFP(B,P)
//! NFP(C,P)
//!        ↓
//! 충돌 불가 영역 구성
//!        ↓
//! Strip boundary까지 고려
//!        ↓
//! Feasible candidate 추출
//!        ↓
//! 각 candidate 평가
//! ```
//!
//! > **후보 점들은 보통 NFP vertex, NFP edge와 다른 NFP edge의 교점, strip boundary와
//! > NFP의 교점 등에서 나온다.**
//!
//! `k`번째 아이템을 넣을 때 필요한 pair는
//!
//! ```text
//! P_k ↔ P_1
//! P_k ↔ P_2
//! ...
//! P_k ↔ P_(k-1)
//! ```
//!
//! 이고 모두 cache hit라면 실제 배치에서는 **NFP 계산보다 candidate evaluation이 핵심
//! 비용**이 된다. 그래서 이 모듈은 후보 생성·중복 제거·사전 정렬·상한 절단을 전부
//! 명시적으로 관리한다.
//!
//! # 좌표 규약
//!
//! [`swarm_nfp`]의 configuration space 규약은 `NFP(F, M) = F + (-M)`이다. 즉
//! 고정 형상 `F`가 원점에 놓였을 때 `NFP(F, M)`은 이동 형상 `M`의 **앵커 평행이동**이
//! 금지되는 영역이다. `F`가 `t_F`에 놓여 있으면 금지 영역은 `t_F + NFP(F, M)`이 된다.
//! 이 모듈이 다루는 후보 좌표는 전부 그 앵커 평행이동(mm) 공간의 점이다.
//!
//! `jagua-rs`는 `f32` mm, `swarm-nfp`는 `f64` mm를 쓴다. 내부 계산은 전부 `f64`로
//! 하고 공개 타입(`PlacementCandidate`, `PlacedPiece`)에만 `f32`로 내려 담는다.

use std::cmp::Ordering;
use std::collections::HashSet;
use std::hash::{BuildHasherDefault, Hasher};
#[cfg(feature = "simd")]
use std::simd::{Select, Simd};
use std::sync::Arc;

use jagua_rs::Instant;
use swarm_nfp::{
    GRID_SCALE, Location, NfpEngine, Point as NfpPoint, Polygon as NfpPolygon, PolygonSet,
    PreparedRegion, Ring as NfpRing,
};

use crate::nfp_lbf::COORD_TIE_EPS;
use crate::nfp_lbf::cache::{GeometryHash, MaskCell, NfpKey, NfpLbfCache, NfpMask};
use crate::nfp_lbf::score::{CandidateScore, PlacementScoreWeights, ScoreTerms, compare_by_priority};

// ────────────────────────────────────────────────────────────────────────────
// 상한 상수
// ────────────────────────────────────────────────────────────────────────────

/// NFP edge × NFP edge 교점 상한.
///
/// 모든 NFP 쌍의 모든 edge 쌍을 도는 것은 `O(n²·e²)`로 폭발한다. bbox 사전 필터를
/// 통과한 쌍만 판정하고, 그래도 이 상한에 도달하면 즉시 중단한다.
pub const MAX_EDGE_INTERSECTIONS: usize = 4096;

/// 한 아이템당 전체 평가까지 끌고 갈 feasible 후보 수 상한.
pub const MAX_EVALUATED_CANDIDATES: usize = 256;

/// 부유조각(floating parts) 분리 배치를 쓸지.
///
/// 결·위치 제약이 없어 자세가 자유로운 조각을 별도 집합으로 떼어 **뒤로 미룬다**.
/// 제약이 강한 조각이 먼저 자리를 잡고, 자유도가 높은 조각이 남은 공백을 채운다.
pub const FLOAT_PARTS_ENABLED: bool = true;

/// 부유조각에 주는 탐침 예산 배수.
///
/// 부유조각이 채울 공백은 프론티어가 아니라 이미 채워진 왼쪽 안쪽에 있다.
/// 기본 예산으로는 그 후보에 닿기 전에 예산이 끝나므로 배수를 준다.
pub const FLOAT_FILL_PROBE_SCALE: usize = 3;

/// 프론티어 폴백에서 기존 배치 오른쪽 끝과 새 조각 사이에 두는 간격(mm).
///
/// 배치 간극([`crate::nfp_lbf::cache`]의 clearance)보다 넉넉히 잡아, f32로 내려간 뒤에도
/// 접촉조차 일어나지 않게 한다.
pub const FRONTIER_FALLBACK_GAP_MM: f64 = 0.1;

/// feasible 판정(= `NfpEngine::locate` 호출)을 시도할 raw 후보 수 상한의 **기본값**.
///
/// 배치가 빽빽해지면 유효 후보가 [`MAX_EVALUATED_CANDIDATES`]에 한참 못 미치므로
/// (여성WSjk 156-패널에서 아이템당 25~88개) 이 예산을 **매번 전부 소진**한다.
/// 따라서 이 값이 곧 배치 시간이고, 동시에 배치 품질이다.
///
/// 더 중요한 건 **후보 수가 배치 진행에 따라 폭증한다**는 점이다. 여성WSjk 156-패널에서
/// 20번째 아이템의 raw 후보는 1,607개지만 155번째는 13,514개다. 예산이 고정이면
/// 후기 아이템은 후보의 7.5%만 보고 자리를 정한다 — 커버리지 붕괴다.
///
/// 실측 (여성WSjk 156-패널, ordering 50개 전량 평가, `concave_simplify_tolerance = 0.02`):
///
/// ```text
/// 예산      strip 폭    밀도      LBF(7325.05 / 69.45%) 대비   전체 시간
///  1024     7479.09    68.02%    -1.43%p                       8 s
///  2048     7277.71    69.90%    +0.45%p                      10 s
///  4096     7082.27    71.83%    +2.38%p                      12 s
///  8192     7057.59    72.08%    +2.63%p                      14 s
/// 16384     7054.85    72.11%    +2.66%p                      15 s
/// ```
///
/// **1024에서는 NFP-LBF가 기존 LBF에 진다.** 시드는 더 좁을 때만 채택되므로
/// (`ffi_step.rs`의 시드 블록) 그대로 기각되고, 화면에는 LBF 결과가 그대로 남는다.
/// "NFP를 켜도 결과가 똑같다"는 증상의 실제 원인이 이것이었다.
///
/// 품질은 4096에서 사실상 포화하고, 그 위로는 0.3%p를 더 얻자고 시간을 쓴다.
/// 시간이 예산에 정비례하지 않는 이유는 초과 탐침 대부분이 [`MAX_EVALUATED_CANDIDATES`]
/// 도달 전에 빠르게 기각되기 때문이다.
///
/// 예전 docstring에 있던 "4096 = 19.90초" 표는 `NfpMask` / `locate_prepared` 도입
/// **이전** 측정이라 더 이상 유효하지 않다. 그 낡은 수치가 1024를 고른 근거였다.
///
/// 실행 시점 값은 [`crate::nfp_lbf::NfpLbfConfig::max_candidate_probes`]로 바꾼다.
pub const DEFAULT_MAX_CANDIDATE_PROBES: usize = 4096;

/// 좌측 우선 패스가 프론티어보다 왼쪽 구간에 쓸 예산의 **분모**.
///
/// 후보는 x 오름차순이고 배치가 진행되면 왼쪽은 이미 꽉 차 있다. 실측에서 20번째
/// 이후 아이템은 좌측 순차 탐침 512회의 **hit이 0건**이었다 — 예산 절반을 태우고
/// feasible 영역에 닿지도 못한다.
///
/// 그렇다고 왼쪽을 통째로 버리면 안 된다. 이미 배치된 조각 사이의 **구멍**은 실제로
/// 왼쪽에 있고, 그게 NFP를 쓰는 이유 중 하나다. 그래서 왼쪽은 버리지 않고
/// `predecessor 예산 / 이 값`만큼만 **균등 간격**으로 훑는다.
///
/// # 기본 경로에서는 이 값이 아무 것도 정하지 않는다
///
/// 기본 경로의 구간 1 은 **의도적으로 비활성**이다(`candidates_for` 의 해당 분기 참조).
/// 살려서 재 봤더니 전부 나빠졌기 때문이다 — ⑧ 누적배치 72.93% -> 72.46%,
/// ⑨ 꼬리회수 73.53% -> 72.21%. 헛탐침에 예산의 1/8 을 태우는 것이 그만큼 손해다.
///
/// 그래서 이 상수가 실제로 쓰이는 곳은 **④ 포켓 우선 모드**뿐이다. 거기서는 구간을
/// 프론티어에 **가까운 쪽부터** 훑고 예산도 `POCKET_PROBE_DIVISOR` 로 더 적게 쓴다.
pub const LEFT_REGION_PROBE_DIVISOR: usize = 8;

/// 접촉도(Contact) 계산에서 "맞닿았다"고 볼 간극 폭(mm).
///
/// 후보 점은 NFP 경계에서 나오므로 실제 형상은 정확히 접촉하지만, 여기서 쓰는
/// bbox 근사는 그만큼 정밀하지 않다. 간극 `g`에 대해 `1 - g / CONTACT_BAND_MM`으로
/// 선형 감쇠시켜 근사한다.
pub const CONTACT_BAND_MM: f64 = 1.0;

/// [`SequentialPlacer::candidates_for`]의 후보 생성원이 없음을 뜻하는 표식.
const NO_GENERATOR: u32 = u32::MAX;

// ────────────────────────────────────────────────────────────────────────────
// 공개 타입
// ────────────────────────────────────────────────────────────────────────────

/// 후보 점이 어느 기하 출처에서 나왔는지.
///
/// ```text
/// NFP vertex
/// NFP edge ∩ NFP edge
/// strip boundary(IFP) ∩ NFP edge
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CandidateSource {
    /// 평행이동된 `NFP(o, item)`의 ring 정점.
    NfpVertex,
    /// 서로 다른 두 NFP의 edge 교점.
    NfpEdgeIntersection,
    /// IFP(strip 경계) 정점 또는 IFP edge와 NFP edge의 교점.
    StripBoundaryIntersection,
}

/// 아직 feasible 판정만 끝난 앵커 평행이동 후보.
#[derive(Clone, Copy, Debug)]
pub struct PlacementCandidate {
    /// 앵커 평행이동(mm).
    pub translation: (f32, f32),
    /// 이 후보를 만들어 낸 기하 출처.
    pub source: CandidateSource,
    /// 이 자리에 놓았을 때 **정확히 맞닿는 기존 조각의 수**.
    ///
    /// 근사가 아니다. feasible 판정이 obstacle 마다 이미 계산하는
    /// `Location::Boundary` 를 세기만 한 값이고, 그건 "앵커 점이
    /// `t_k + NFP(k, item)` 의 경계 위에 있다" = **조각이 `placed[k]` 에 정확히
    /// 맞닿았다** 와 같은 말이다.
    ///
    /// 왜 이게 필요한가: 전략 계층의 접촉 지표는 bbox 근사라 오목한 의류 패널이
    /// 실제로 맞물려도 접촉을 놓친다. 그 근사를 대체하지는 못하지만(길이를 모른다),
    /// **몇 조각에 닿았는가**는 정확하므로 1순위 판별에 쓸 수 있다.
    ///
    /// strip 경계와의 접촉은 세지 않는다 — 그건 IFP 판정이라 obstacle 이 아니다.
    pub touch_count: u16,
}

/// 확정 배치된 한 조각.
#[derive(Clone, Copy, Debug)]
pub struct PlacedPiece {
    /// 인스턴스 아이템 id.
    pub item_id: usize,
    /// 적용된 회전각(도, 정수).
    pub rotation: i64,
    /// 앵커 평행이동(mm).
    pub translation: (f32, f32),
}

/// 한 ordering을 끝까지 배치한 결과.
#[derive(Clone, Debug, Default)]
pub struct SequentialPlacement {
    /// 배치된 조각들(배치 순서 그대로).
    pub pieces: Vec<PlacedPiece>,
    /// 배치된 모든 조각의 `x_max` 최대값(mm).
    pub strip_length: f32,
    /// 배치 성공 개수(= `pieces.len()`).
    pub placed: usize,
    /// 자리를 찾지 못한 아이템 id들.
    pub unplaced: Vec<usize>,
}

/// NFP 캐시를 빌려 하나의 ordering을 순차 배치하는 실행기.
///
/// SoA 버퍼(`*_soa`)는 **한 번 만들고 계속 재사용**한다. 156개짜리 배치의 마지막
/// 아이템이면 obstacle 155개 × 후보 수백 개를 매번 훑으므로, 아이템마다 버퍼를
/// 다시 할당하면 그것만으로도 무시 못 할 비용이 된다.
pub struct SequentialPlacer<'a> {
    /// 이번 아이템에 적용할 탐침 예산 배수 (부유조각에서 임시로 올린다).
    probe_scale: usize,
    /// raw 후보 탐침 예산. [`DEFAULT_MAX_CANDIDATE_PROBES`] 참조.
    max_candidate_probes: usize,
    /// 포켓 우선 모드. [`Self::set_pocket_first`] 참조.
    pocket_first: bool,
    /// 이번 아이템 탐침에서 "프론티어"로 볼 x(mm). 이보다 충분히 왼쪽인 후보는
    /// 순차가 아니라 균등 간격으로만 훑는다. `0.0`이면 구간 분리를 하지 않는다.
    ///
    /// [`Self::best_placement`]가 아이템마다 설정한다. `candidates_for`를 직접
    /// 부르는 외부 호출자(`optimizer::lns`)는 건드리지 않으므로 기존 동작이 유지된다.
    probe_frontier: f64,
    /// `item_id`별 허용 회전(도). 길이가 2 이상이면 그 조각은 **부유조각**이다.
    /// 비워 두면 분리가 비활성되고 라이브러리 전체 회전을 쓴다.
    item_rotations: Vec<Vec<i64>>,
    cache: &'a mut NfpLbfCache,
    /// strip의 고정 변(y 방향, mm).
    strip_height: f32,
    /// IFP 계산에 쓸 x 방향 상한(mm).
    strip_width_cap: f32,
    /// importer 가 컨테이너에 적용한 **물리적** 인셋(mm). 겹침이 꺼져 있으면 0 이다.
    ///
    /// 구성형 배치기는 원단 치수를 줄어들지 않은 값에서 가져오므로 이 값을 빼 주지
    /// 않으면 첫 조각과 모든 열 시작 조각을 실제 컨테이너 밖에 놓는다.
    strip_inset: f32,
    weights: PlacementScoreWeights,
    engine: NfpEngine,
    /// 현재 아이템의 obstacle bbox SoA(= `collect_obstacles` 결과와 인덱스 1:1).
    obs_bbox_soa: AabbSoA,
    /// 이미 배치된 조각들의 월드 bbox SoA(= `world_boxes` 결과와 인덱스 1:1).
    placed_bbox_soa: AabbSoA,
    /// obstacle별 edge SoA. 바깥 `Vec`도 용량을 유지한 채 재사용한다.
    obs_edge_soa: Vec<EdgeSoA>,
    /// IFP edge SoA.
    ifp_edge_soa: EdgeSoA,
}

// ────────────────────────────────────────────────────────────────────────────
// 내부 기하 보조 타입
// ────────────────────────────────────────────────────────────────────────────

/// 축 정렬 경계 상자(mm, `f64`).
///
/// 전략 계층([`crate::nfp_strategy`])이 후보 평가에 그대로 쓰므로 공개한다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl Aabb {
    pub fn width(self) -> f64 {
        self.max_x - self.min_x
    }

    pub fn height(self) -> f64 {
        self.max_y - self.min_y
    }

    pub fn area(self) -> f64 {
        (self.width().max(0.0)) * (self.height().max(0.0))
    }

    pub fn translated(self, dx: f64, dy: f64) -> Self {
        Self {
            min_x: self.min_x + dx,
            min_y: self.min_y + dy,
            max_x: self.max_x + dx,
            max_y: self.max_y + dy,
        }
    }

    fn contains(self, x: f64, y: f64, eps: f64) -> bool {
        x >= self.min_x - eps && x <= self.max_x + eps && y >= self.min_y - eps && y <= self.max_y + eps
    }
}

/// 하나의 장애물이 만든 금지 영역.
///
/// # 왜 로컬 좌표로 들고 있나
///
/// 예전에는 캐시된 NFP 전체를 `translate_set`으로 월드 좌표에 **복사**했다. 아이템 하나를
/// 놓을 때마다 obstacle 155개 × 정점 수만큼 새 `PolygonSet`을 할당했고, 여성WSjk
/// 156-패널 ordering 1회에 12,090번이었다.
///
/// 대신 NFP는 캐시의 로컬 좌표 그대로(`Arc` 공유, 복사 0회) 두고, 질의할 때 점에서
/// 평행이동을 뺀다.
///
/// ```text
/// 기존:  locate( translate(NFP, t),  p )
/// 변경:  locate( NFP,               p - t )
/// ```
///
/// # 격자 양자화 등가성
///
/// `locate`는 도형 정점과 질의점을 **둘 다** 0.001mm 격자로 반올림한다. 그래서 뺄셈을
/// `f64` mm에서 하면 결과가 옛 경로와 1격자 어긋날 수 있다. 정확히 같게 하려고 뺄셈을
/// **정수 격자에서** 한다.
///
/// ```text
/// 기존 판정:  round(GRID·p)            vs  round(GRID·(v + t)) = V + round(GRID·t)
/// 새 판정  :  round(GRID·p) - T        vs  V,   T = round(GRID·t)
/// ```
///
/// NFP 정점은 커널이 정수 격자 위에서 뱉으므로 `GRID·v = V`가 정확한 정수이고,
/// 위 두 판정은 `T = round(GRID·t)`인 한 **비트 단위로 동일**하다. 유일한 예외는
/// `GRID·t`의 소수부가 정확히 `0.5`라 반올림이 부동소수 오차에 걸리는 경우인데,
/// 그때는 [`Obstacle::world`]로 옛 경로를 그대로 쓴다([`GRID_TIE_MARGIN`] 참조).
///
/// # 왜 정규화 영역을 따로 들고 있나
///
/// `NfpEngine::locate`는 **호출마다** 영역을 `canonical_paths_from_polygon_set`으로
/// 통째로 재정규화한다. 정점 40개짜리 point-in-polygon 한 번이 약 36 µs 걸린 이유가
/// 그것이고, 여성WSjk 156-패널 3-ordering 배치의 `locate` 63,588회가 전체 4.67초 중
/// 약 2.3초를 차지했다. 캐시가 방향(정규/부호 반전)마다 [`PreparedRegion`]을 한 번만
/// 만들어 두므로 여기서는 `Arc`만 들고 다니고 `locate_prepared`로 질의한다.
/// 결과는 `locate`와 정의상 동일하다.
struct Obstacle {
    /// 캐시가 소유한 **로컬 좌표** NFP.
    region: Arc<PolygonSet>,
    /// 로컬 좌표계 점유 마스크. `locate` 단축 경로다.
    mask: Arc<NfpMask>,
    /// `region`을 한 번만 정규화해 둔 질의용 영역(캐시 소유, `Arc` 공유).
    /// `None`이면 정규화가 실패한 것이며 그때는 `locate`로 되돌아간다.
    prepared: Option<Arc<PreparedRegion>>,
    /// 월드 평행이동(mm).
    tx: f64,
    ty: f64,
    /// 평행이동의 0.001mm 격자 정수값.
    tx_q: i64,
    ty_q: i64,
    /// 월드 bbox.
    bbox: Aabb,
    /// 격자 반올림이 애매할 때만 채워지는 월드 좌표 사본(옛 경로 폴백).
    world: Option<PolygonSet>,
    /// `world`를 한 번만 정규화해 둔 질의용 영역. 이 영역은 평행이동에 딸린
    /// 인스턴스별 사본이라 캐시에 넣을 수 없으므로 여기서 소유한다.
    world_prepared: Option<PreparedRegion>,
}

impl Obstacle {
    /// 후보 생성이 훑을 **로컬 좌표** ring 집합. 월드 좌표는 여기에 `(tx, ty)`를 더해 얻는다.
    #[inline]
    fn local_rings(&self) -> &PolygonSet {
        &self.region
    }
}

/// `GRID·t`의 소수부가 `±0.5`에 이만큼 가까우면 격자 반올림이 애매하다고 본다.
///
/// 옛 경로는 `round(GRID·(v + t))`를 계산했고 그 부동소수 오차는 좌표 20,000mm 기준
/// 5e-9 수준이다. 여유를 1e-5로 잡으면 오차보다 세 자릿수 넉넉하면서, 실제로 걸릴
/// 확률은 좌표 10만 개에 1개 수준이다.
const GRID_TIE_MARGIN: f64 = 1.0e-5;

/// 정규화 척도. 항끼리 자릿수가 크게 달라 가중합이 한 항에 지배되는 것을 막는다.
#[derive(Clone, Copy, Debug)]
struct Scales {
    length: f32,
    height: f32,
    area: f32,
    perimeter: f32,
}

// ────────────────────────────────────────────────────────────────────────────
// SequentialPlacer
// ────────────────────────────────────────────────────────────────────────────

impl<'a> SequentialPlacer<'a> {
    /// `strip_width_cap`은 IFP 계산에 쓸 x 방향 상한이다. 넉넉히 잡는다.
    pub fn new(
        cache: &'a mut NfpLbfCache,
        strip_height: f32,
        strip_width_cap: f32,
        weights: PlacementScoreWeights,
    ) -> Self {
        Self::with_item_rotations(cache, strip_height, strip_width_cap, weights, Vec::new())
    }

    /// `item_rotations`는 `item_id`로 인덱싱한 **아이템별 허용 회전**(도)이다.
    /// 길이가 2 이상인 아이템은 부유조각으로 분류된다. 비워 두면 분리가 비활성된다.
    pub fn with_item_rotations(
        cache: &'a mut NfpLbfCache,
        strip_height: f32,
        strip_width_cap: f32,
        weights: PlacementScoreWeights,
        item_rotations: Vec<Vec<i64>>,
    ) -> Self {
        let strip_inset = cache.library().strip_inset();
        Self {
            probe_scale: 1,
            max_candidate_probes: DEFAULT_MAX_CANDIDATE_PROBES,
            pocket_first: false,
            probe_frontier: 0.0,
            item_rotations,
            cache,
            strip_height,
            strip_width_cap,
            strip_inset,
            weights,
            engine: NfpEngine::new(),
            obs_bbox_soa: AabbSoA::default(),
            placed_bbox_soa: AabbSoA::default(),
            obs_edge_soa: Vec::new(),
            ifp_edge_soa: EdgeSoA::default(),
        }
    }

    /// raw 후보 탐침 예산을 바꾼다. `0`은 무시하고 기본값을 유지한다.
    ///
    /// 이 값이 곧 배치 품질이자 시간이다. 근거는 [`DEFAULT_MAX_CANDIDATE_PROBES`]의 실측표.
    pub fn with_probe_budget(mut self, probes: usize) -> Self {
        if probes > 0 {
            self.max_candidate_probes = probes;
        }
        self
    }

    /// 현재 탐침 예산.
    pub fn probe_budget(&self) -> usize {
        self.max_candidate_probes
    }

    /// 탐침 예산을 실행 중에 바꾼다. `0`은 무시한다.
    ///
    /// ordering 사이에서 남은 시간에 맞춰 조절하는 데 쓴다
    /// ([`crate::nfp_lbf::search`]의 예산 되먹임). 배치 **도중**에 바꾸면 같은 해가
    /// 조각마다 다른 정밀도로 만들어져 재현이 안 되므로, ordering 경계에서만 부를 것.
    pub fn set_probe_budget(&mut self, probes: usize) {
        if probes > 0 {
            self.max_candidate_probes = probes;
        }
    }

    /// 빌려 쓰는 NFP 캐시.
    pub fn cache(&self) -> &NfpLbfCache {
        &*self.cache
    }

    // ── 전략 계층(`crate::nfp_strategy`)이 쓰는 후보 오라클 표면 ──────────
    //
    // 아래 넷은 **읽기 접근자와 상태 설정자일 뿐** 선택 로직을 담지 않는다.
    // 전략 계층은 `candidates_for`(이미 공개)로 feasible 앵커만 받아 가고,
    // "어느 후보를 고를지 / 어느 조각을 먼저 놓을지"는 전부 자기 쪽에서 정한다.

    /// `(item_id, rotation)`의 앵커 기준 bbox와 실제 면적(mm²).
    ///
    /// 앵커 기준이므로 월드 bbox는 여기에 평행이동을 더하면 된다.
    pub fn shape_bounds_area(&self, item_id: usize, rotation: i64) -> Option<(Aabb, f64)> {
        self.shape_metrics(item_id, rotation)
    }

    /// 이 아이템에 실제로 허용된 회전 목록(도).
    pub fn allowed_rotations(&self, item_id: usize) -> Vec<i64> {
        self.rotations_for(item_id)
    }

    /// 배치된 조각들의 월드 bbox.
    pub fn world_bounds(&self, placed: &[PlacedPiece]) -> Vec<Aabb> {
        self.world_boxes(placed)
    }

    /// 다음 `candidates_for` 호출이 쓸 탐침 구간 분리 기준(mm).
    ///
    /// `0.0`이면 구간을 나누지 않는다. 전략 계층이 아이템마다 직접 설정한다.
    pub fn set_probe_frontier(&mut self, frontier: f64) {
        self.probe_frontier = if frontier.is_finite() && frontier > 0.0 {
            frontier
        } else {
            0.0
        };
    }

    /// 포켓 우선 모드. 켜면 `candidates_for`가 프론티어 **안쪽** 후보를 먼저
    /// 전량 훑고, 거기서 하나도 못 찾았을 때만 프론티어 바깥으로 나간다.
    ///
    /// 기본은 꺼짐이며, 그때는 안쪽에 예산의 `1/LEFT_REGION_PROBE_DIVISOR`만 쓴다.
    pub fn set_pocket_first(&mut self, on: bool) {
        self.pocket_first = on;
    }

    /// strip의 고정 변(mm).
    /// 작업 strip 폭 상한(mm). 후보 점수의 길이 정규화 분모이기도 하다.
    ///
    /// ⑧ 누적 배치가 오라클을 거치지 않고 캐시를 직접 쓰므로, 같은 상한을 받아야
    /// 두 경로의 점수가 같은 눈금이 된다.
    pub fn strip_width_cap(&self) -> f32 {
        self.strip_width_cap
    }

    /// 후보 점수 가중치. ⑧ 누적 배치가 같은 가중치를 써야 비교가 성립한다.
    pub fn weights(&self) -> PlacementScoreWeights {
        self.weights
    }

    pub fn strip_height(&self) -> f32 {
        self.strip_height
    }

    /// 주어진 아이템·회전에 대해 문서가 지정한 세 출처에서 feasible 후보를 뽑는다.
    ///
    /// ```text
    /// 1) NFP vertex
    /// 2) NFP edge ∩ NFP edge
    /// 3) strip boundary(IFP) ∩ NFP edge, 그리고 IFP 정점
    /// ```
    ///
    /// feasible 판정은
    /// * (a) 어떤 `NFP(o, item)`의 **내부**에도 있지 않고(경계는 허용 — 접촉 배치),
    /// * (b) IFP의 내부 또는 경계에 있어야 한다.
    pub fn candidates_for(
        &mut self,
        item_id: usize,
        rotation: i64,
        placed: &[PlacedPiece],
    ) -> Vec<PlacementCandidate> {
        // 호스트의 Init 취소가 탐침 루프까지 닿도록 한다(아이템/탐침 단위로만 폴링한다).
        let should_stop = || crate::config::runtime::init_cancel_requested();
        self.candidates_for_with_control(item_id, rotation, placed, &should_stop).unwrap_or_default()
    }

    // None denotes interruption or a failed obstacle NFP (see `collect_obstacles`),
    // never a geometrically complete partial list.
    fn candidates_for_with_control(
        &mut self,
        item_id: usize,
        rotation: i64,
        placed: &[PlacedPiece],
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> Option<Vec<PlacementCandidate>> {
        if should_stop() { return None; }
        let Some(hash_item) = self.cache.library().hash_of_item(item_id) else {
            return Some(Vec::new());
        };

        // ── IFP: strip 안에 아이템이 완전히 들어가는 앵커 영역 ──
        let Some((ifp, ifp_mask, ifp_prepared)) = self.cache.inner_fit_view_prepared_with_inset(
            hash_item,
            rotation,
            self.strip_width_cap,
            self.strip_height,
            self.strip_inset,
        ) else {
            return Some(Vec::new());
        };
        if ifp.is_empty() {
            return Some(Vec::new());
        }
        let Some(ifp_bbox) = set_bounds(&ifp) else {
            return Some(Vec::new());
        };

        // ── 기존 배치 아이템별 금지 영역 ──
        if should_stop() { return None; }
        let obstacles = self.collect_obstacles(hash_item, rotation, placed, should_stop)?;
        // bbox·edge를 SoA로 한 번 재배치해 둔다(버퍼는 재사용, 재할당 없음).
        // bbox SoA는 **전체** obstacle을 담는다 — feasible 판정이 전부를 봐야 한다.
        self.obs_bbox_soa.load_obstacles(&obstacles);

        // 후보를 생성할 obstacle 부분집합.
        //
        // 주석이 오래도록 `CANDIDATE_BAND_ITEM_FACTOR` 를 근거로 가리켰는데 **그 상수는
        // 저장소 어디에도 없다** — 되돌려진 "프론티어 밴드 제한" 실험의 잔재다.
        // 지금 `select_candidate_generators` 는 밴드로 자르지 않고 **전체를 돌려준다**.
        // 근거는 그 함수의 문서에 있다.
        let generators = select_candidate_generators(&obstacles, ifp_bbox);
        self.load_obstacle_edges_subset(&obstacles, &generators);
        self.ifp_edge_soa.load_from_set(&ifp);

        // ── 후보 수집 ──
        // 4번째 성분은 이 후보를 만든 obstacle 인덱스다([`NO_GENERATOR`] = 없음).
        // NFP 정점에서 나온 후보는 그 obstacle에 대해 **반드시 경계**이므로
        // feasible 판정에서 `locate` 한 번을 확실히 건너뛸 수 있다.
        let mut raw: Vec<(f64, f64, CandidateSource, u32)> = Vec::new();
        // bbox 사전 필터가 통과시킨 인덱스를 담는 스크래치. 루프마다 재할당하지 않는다.
        let mut hits: Vec<usize> = Vec::new();

        // 1) NFP vertex
        //
        // 후보 **생성**은 여전히 월드 좌표가 필요하다. 다만 `PolygonSet`을 통째로
        // 복사하지 않고 정점을 훑으면서 평행이동만 더한다 (`fl(v + t)`는 옛
        // `translate_set` 결과와 비트 단위로 같다).
        for &oi in &generators {
                if should_stop() { return None; }
            let o = &obstacles[oi];
            for ring in set_rings(o.local_rings()) {
                for v in &ring.vertices {
                    raw.push((v.x + o.tx, v.y + o.ty, CandidateSource::NfpVertex, oi as u32));
                }
            }
        }

        // 2) NFP edge ∩ NFP edge — bbox 겹치는 NFP 쌍 → bbox 겹치는 edge 쌍만
        //
        // edge 하나(`ea`) 대 상대 edge 전부의 bbox 사전 필터는 순수 비교 4개짜리
        // 작업이라 벡터화가 잘 먹는다. 통과한 인덱스만 오름차순으로 모아 스칼라
        // `segment_intersection`에 넘기므로 후보가 쌓이는 **순서까지 기존과 같다**
        // (중복 제거가 "먼저 본 출처"를 남기므로 순서 보존이 곧 결과 보존이다).
        let eps = COORD_TIE_EPS as f64;
        let mut n_intersections = 0usize;
        'pairs: for gi in 0..generators.len() {
                if should_stop() { return None; }
            for gj in (gi + 1)..generators.len() {
                let (i, j) = (generators[gi], generators[gj]);
                if !aabb_overlaps(obstacles[i].bbox, obstacles[j].bbox, eps) {
                    continue;
                }
                let (ea_set, eb_set) = (&self.obs_edge_soa[gi], &self.obs_edge_soa[gj]);
                for ai in 0..ea_set.len() {
                    edge_bbox_hits(eb_set, ea_set.bbox_of(ai), eps, &mut hits);
                    let ea = ea_set.edge_of(ai);
                    for &bi in hits.iter() {
                        if let Some((x, y)) = segment_intersection(&ea, &eb_set.edge_of(bi)) {
                            raw.push((x, y, CandidateSource::NfpEdgeIntersection, NO_GENERATOR));
                            n_intersections += 1;
                            if n_intersections >= MAX_EDGE_INTERSECTIONS {
                                break 'pairs;
                            }
                        }
                    }
                }
            }
        }

        // 3) IFP 정점 + IFP edge ∩ NFP edge
        for ring in set_rings(&ifp) {
            for v in &ring.vertices {
                raw.push((v.x, v.y, CandidateSource::StripBoundaryIntersection, NO_GENERATOR));
            }
        }
        let mut n_boundary = 0usize;
        'boundary: for (gi, &oi) in generators.iter().enumerate() {
                if should_stop() { return None; }
            if !aabb_overlaps(obstacles[oi].bbox, ifp_bbox, eps) {
                continue;
            }
            let eb_set = &self.obs_edge_soa[gi];
            for ai in 0..self.ifp_edge_soa.len() {
                edge_bbox_hits(eb_set, self.ifp_edge_soa.bbox_of(ai), eps, &mut hits);
                let ea = self.ifp_edge_soa.edge_of(ai);
                for &bi in hits.iter() {
                    if let Some((x, y)) = segment_intersection(&ea, &eb_set.edge_of(bi)) {
                        raw.push((x, y, CandidateSource::StripBoundaryIntersection, NO_GENERATOR));
                        n_boundary += 1;
                        if n_boundary >= MAX_EDGE_INTERSECTIONS {
                            break 'boundary;
                        }
                    }
                }
            }
        }

        // ── 0.001mm 격자 중복 제거 (먼저 본 출처를 유지) ──
        let mut seen: GridKeySet = HashSet::with_capacity_and_hasher(raw.len(), Default::default());
        let mut unique: Vec<(f64, f64, CandidateSource, u32)> = Vec::with_capacity(raw.len());
        for (x, y, src, origin) in raw {
            if !x.is_finite() || !y.is_finite() {
                continue;
            }
            if seen.insert(grid_key(x, y)) {
                unique.push((x, y, src, origin));
            }
        }

        // ── 저렴한 사전 정렬: x 오름차순 → y 오름차순 ──
        unique.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
        });

        // ── feasible 판정 ──
        //
        // 후보는 x 오름차순이라 배치가 빽빽해지면 **왼쪽 후보가 통째로 기각**된다.
        // 예전에는 그대로 앞에서부터 훑다가 예산을 소진하고 후보 0개로 끝났다
        // (여성WSjk 156-패널에서 45개가 이 사유로 미배치). 그래서 세 구간으로 나눈다.
        //
        //   1. 프론티어보다 왼쪽 — **균등 간격**으로 예산의 `1/LEFT_REGION_PROBE_DIVISOR`.
        //      배치된 조각 사이의 구멍은 여기 있으므로 버리지 않되, 순차로 태우지도 않는다.
        //   2. 프론티어 이후 — 왼쪽부터 순서대로 (LBF 본래 의도: 좌하단 우선).
        //   3. 남은 예산 — 후보 전체에 균등 간격. 어느 구간이 막혀도 열린 자리를 만난다.
        //
        // 세 경로 모두 같은 `is_feasible`을 쓰므로 판정 자체는 달라지지 않는다.
        //
        // 구간 분리 이전에는 좌측 순차 패스가 예산의 절반을 쓰고 **hit이 0건**이었다.
        // 20번째 이후 아이템에서 feasible 후보는 전부 프론티어 근처에 있는데, 좌측
        // 512회 탐침은 x가 그 근처에 닿지도 못한 채 끝났다. 실효 예산이 절반이었던 셈이다.
        let mut out: Vec<PlacementCandidate> = Vec::with_capacity(MAX_EVALUATED_CANDIDATES.min(unique.len()));
        let mut probed: Vec<bool> = vec![false; unique.len()];
        let mut probes = 0usize;

        let try_index =
            |idx: usize,
             probed: &mut Vec<bool>,
             probes: &mut usize,
             out: &mut Vec<PlacementCandidate>,
             this: &Self,
             hits: &mut Vec<usize>| {
                if probed[idx] {
                    return;
                }
                probed[idx] = true;
                *probes += 1;
                let (x, y, src, origin) = unique[idx];
                if let Some(touch_count) = this.feasible_touch_count(
                    x,
                    y,
                    origin,
                    &ifp,
                    &ifp_mask,
                    ifp_prepared.as_deref(),
                    ifp_bbox,
                    &obstacles,
                    hits,
                ) {
                    out.push(PlacementCandidate {
                        translation: (x as f32, y as f32),
                        source: src,
                        touch_count,
                    });
                }
            };

        let probe_budget = self
            .max_candidate_probes
            .saturating_mul(self.probe_scale.max(1));

        // 프론티어 왼쪽 구간의 끝. `probe_frontier`가 0이면(외부 호출자) 구간 분리를
        // 하지 않고 예전처럼 전부 순차 패스로 넘긴다.
        //
        // 경계는 `프론티어 - 조각의 앵커 기준 우측 끝`이다. 앵커 x가 그보다 왼쪽이면
        // 조각 전체가 프론티어 안쪽, 즉 이미 채워진 영역에 들어간다는 뜻이다.
        //
        // 첫 가지를 `!(x > 0.0)`으로 쓴다 — 교정 전 코드가 `if probe_frontier > 0.0`
        // 였으므로, NaN이 들어와도 기본 경로가 **글자 그대로 같은 가지**로 가야 한다.
        // (`probe_frontier`는 지금 NaN이 될 수 없다: 설정자는 `is_finite`를 걸고,
        //  내부 대입은 `fold(0.0, f64::max)` 결과다. 그래도 기준선 측정이 통째로
        //  걸린 경로라 동치성을 문법 수준에서 못박아 둔다.)
        let left_end = if !(self.probe_frontier > 0.0) {
            0
        } else if self.pocket_first {
            // ④ 포켓 우선 전용 경계 — 교정본.
            //
            // 예전에는 여기서도 `ifp_bbox.width()`를 뺐다. 그런데 `ifp_bbox`는 조각이
            // 아니라 **앵커 가용 영역**의 bbox다. 폭이 `strip_width_cap - 조각폭`인데
            // `strip_width_cap = Σ(모든 조각 bbox 폭)`이라(`worst_case_strip_width`)
            // 조각폭보다 두 자릿수 크다. wsjk156 실측: Σ폭 = 51141.09 mm,
            // 조각의 anchor max_x는 수백 mm, 프론티어는 아무리 커도 ~7.3e3 mm
            // (LBF 단독 폭 7325.05가 상한). 즉 `boundary`가 **항상** 음수라
            // `left_end`가 모든 호출에서 0이었고, 구간 1이 통째로 죽어 있었다
            // (④가 기준선과 소수점까지 같았던 이유).
            //
            // 월드 bbox는 `bounds.translated(x, y)`이므로 조각의 우측 끝은
            // `앵커x + bounds.max_x`다. 이것이 프론티어 안이어야 "포켓"이다.
            match self.shape_metrics(item_id, rotation) {
                Some((bounds, _)) => pocket_left_end(&unique, self.probe_frontier, bounds.max_x),
                // 조각 bbox를 못 얻으면 "안쪽"을 판정할 근거가 없다. 0을 돌려
                // 기본 경로와 같은 상태(구간 1 비활성)로 물러난다.
                //
                // 예전 `unwrap_or(0.0)`은 경계를 프론티어 **자체**로 만들어, 프론티어
                // 밖으로 튀어나오는 후보까지 포켓으로 오인하고 그걸로 조기 반환까지
                // 할 수 있었다 — 폴백이 안전한 쪽이 아니라 반대쪽으로 틀어져 있었다.
                None => 0,
            }
        } else {
            // 기본 경로도 **포켓 모드와 같은 식**을 쓴다.
            //
            // # 예전에는 왜 달랐나
            //
            // 여기서 빼던 `ifp_bbox.width()` 는 조각 폭이 아니라 **IFP 폭**이다.
            // 둘은 `IFP폭 = strip폭 − 조각폭` 관계라 의미가 정반대고, 그래서
            // `boundary = frontier − IFP폭` 은 거의 언제나 크게 음수가 되어
            // `partition_point` 가 0 을 돌려줬다 — **구간 1이 통째로 죽어 있었다.**
            // (④ 포켓 우선이 "효과 0" 으로 측정된 원인도 같은 식이었다.)
            //
            // 당시에는 "고치면 기존 측정치가 무효가 된다" 는 이유로 미뤄 두고,
            // "되살릴지는 재측정과 함께 정해야 한다" 고 적어 두었다. 그 재측정을 했다.
            // # 재측정 결과 — **되살리지 않는다**
            //
            // 포켓 모드와 같은 식(`pocket_left_end(&unique, frontier, bounds.max_x)`)으로
            // 바꿔 구간 1 을 실제로 살려 재 봤다. 결과는 **전부 나빠졌다**
            // (wsjk156, 탐침 8192, `SPSolution::density`, 2026-08-31):
            //
            // ```text
            //                    죽은 채로       되살림
            //   ⑧ 누적배치        72.93%        72.46%   -0.47%p
            //   ⑧+⑦             73.09%        72.95%   -0.14%p
            //   ⑨ 꼬리회수        73.53%        72.21%   -1.32%p
            //   ⑧+⑦+⑨           73.72%        73.72%    변화 없음
            // ```
            //
            // 이유는 이 상수의 원래 근거와 같다 — 배치가 진행되면 프론티어 왼쪽은 이미
            // 차 있어 **hit 이 거의 없다.** 구간 1 을 살리면 예산의 1/8 을 그 헛탐침에
            // 태우고, 정작 자리가 있는 프론티어 근처 탐침이 그만큼 줄어든다.
            // (④ 포켓 우선이 이득을 내는 것은 구간을 **프론티어에 가까운 쪽부터**
            //  훑고 예산도 적게 쓰기 때문이다 — 같은 구간이라도 훑는 방식이 다르다.)
            //
            // 그래서 기본 경로의 구간 1 은 **의도적으로 비활성**이다. `0` 을 그대로
            // 돌려주고, 왜 그런지를 여기 남긴다. 되살리려면 위 표를 다시 뒤집는
            // 측정을 먼저 보여야 한다.
            let _ = ifp_bbox;
            0
        };

        // 1) 프론티어 왼쪽.
        //
        // 기본 모드는 균등 간격으로 예산의 1/LEFT_REGION_PROBE_DIVISOR 만 쓴다.
        // **포켓 우선 모드**(`set_pocket_first`)는 아래 세 가지가 다르다.
        //   (a) 프론티어에 **가까운 쪽부터** 훑는다(오른쪽 -> 왼쪽).
        //   (b) 예산을 조금만 쓴다(`POCKET_PROBE_DIVISOR`).
        //   (c) 구간을 끝까지 다 훑고 자리를 찾았을 때만 조기 반환한다.
        // 근거는 각 항목 주석의 실측값.
        if left_end > 0 && self.pocket_first {
            // (b) 포켓 패스가 쓸 예산.
            //
            // 교정 전에는 `left_end`가 늘 0이라 이 위험이 드러나지 않았다. 교정 후
            // 좌측 구간은 wsjk156에서 후보의 **97.2%**다(left_end 누적 7,908,003 /
            // unique 누적 8,135,637). 즉 "포켓"은 특별한 소수 구간이 아니라 후보 거의
            // 전부다. 여기에 예산을 크게 물리면 구간 3(전역 균등 스캔)이 굶어 죽고,
            // 그 순간 배치가 나빠진다. 예산 4096, 12 ordering 실측:
            //
            //   1/2   71.59%   1/4  71.43%   1/8  71.79%
            //   1/16  71.85%   1/32 71.85%   1/64 71.85%   1/128 71.70%
            //
            // 1/16~1/64가 평평하다 = 그 이상 주어도 포켓을 더 못 찾는다. 하한을 넉넉히
            // 잡아 1/16을 쓴다. (기준선 71.75%)
            const POCKET_PROBE_DIVISOR: usize = 16;
            let pocket_budget = (probe_budget / POCKET_PROBE_DIVISOR).max(1);

            // (a) 프론티어에 가까운 쪽부터.
            //
            // `unique`는 x 오름차순이므로 앞에서부터 훑으면 x=0 근처, 즉 **가장 오래
            // 전에 채워져 남을 자리가 없는** 영역을 먼저 태운다. 이건 구간 분리를
            // 도입해 없앤 바로 그 병리다. 방금 생긴 구멍은 프론티어 바로 뒤에 있다.
            // 실측(예산 1/2, 조기반환 끔): 정방향 71.38% vs 역방향 71.59%.
            // 좌측 hit 수도 역방향이 2.5배다(누적 10,265 vs 4,181 / 1,850회 호출).
            //
            // 구간을 실제로 끝까지 봤는지는 (c)가 쓰므로 **플래그로 직접** 기록한다.
            // 예전에는 `probes < pocket_budget`으로 대신 판정했는데, 그건
            // `left_end == pocket_budget`인 경우(= 구간을 정확히 다 훑고 예산도 딱
            // 맞은 경우)를 "중간에 끊겼다"로 잘못 읽는다.
            let mut scanned_whole_region = true;
            for k in 0..left_end {
                if should_stop() { return None; }
                if out.len() >= MAX_EVALUATED_CANDIDATES || probes >= pocket_budget {
                    scanned_whole_region = false;
                    break;
                }
                let Some(idx) = left_end.checked_sub(k + 1) else {
                    scanned_whole_region = false;
                    break;
                };
                try_index(idx, &mut probed, &mut probes, &mut out, self, &mut hits);
            }

            // (c) 구간을 **끝까지** 훑었을 때만 조기 반환한다.
            //
            // 예산에 걸려 중간에 끊겼다면 "안쪽에 이보다 나은 자리가 없다"가 성립하지
            // 않는다. 그때 바로 반환하면 구간 2·3이 찾아 줄 더 좋은 포켓을 잃는다.
            // 실측: 무조건 반환 71.71% vs 여기 조건 71.85%.
            //
            // 점수 1순위가 `strip_length = max(프론티어, 조각 우측 끝)`이라, 포켓
            // 후보는 항상 최소값으로 동률이 된다(`select::cmp_by_rule`은 1순위가
            // 다르면 즉시 결판내는 사전식 비교다). 즉 **포켓이 후보에 들어 있기만
            // 하면** 포켓 아닌 후보에게 지는 일은 없다. 갈리는 것은 **포켓끼리의
            // 2순위**다 — 기본 규칙(`LeftBottom`)은 `x`가 작을수록, ③(`MaxContact`)은
            // 접촉이 클수록 이긴다. 구간 3의 균등 스캔은 `left_end` **안쪽**까지
            // 훑으므로 이 역방향 패스가 못 본 포켓을 더 찾아낸다. 조기 반환이 잃는
            // 것이 바로 그 후보들이다.
            if scanned_whole_region && !out.is_empty() {
                return Some(out);
            }
        } else if left_end > 0 {
            let left_budget = (probe_budget / LEFT_REGION_PROBE_DIVISOR).max(1);
            let stride = (left_end / left_budget).max(1);
            let mut idx = 0usize;
            while idx < left_end {
                if should_stop() { return None; }
                if out.len() >= MAX_EVALUATED_CANDIDATES || probes >= left_budget {
                    break;
                }
                try_index(idx, &mut probed, &mut probes, &mut out, self, &mut hits);
                idx += stride;
            }
        }

        // 2) 프론티어 이후 — 왼쪽부터 순서대로.
        let sequential_budget = probes + probe_budget / 2;
        for idx in left_end..unique.len() {
                if should_stop() { return None; }
            if out.len() >= MAX_EVALUATED_CANDIDATES || probes >= sequential_budget {
                break;
            }
            try_index(idx, &mut probed, &mut probes, &mut out, self, &mut hits);
        }

        // 3) 균등 간격 스캔 패스 — 어느 구간이 막혀도 열린 자리를 찾는다.
        if out.len() < MAX_EVALUATED_CANDIDATES && unique.len() > probes {
            let remaining = probe_budget.saturating_sub(probes).max(1);
            let stride = (unique.len() / remaining).max(1);
            let mut idx = 0usize;
            while idx < unique.len() {
                if should_stop() { return None; }
                if out.len() >= MAX_EVALUATED_CANDIDATES || probes >= probe_budget {
                    break;
                }
                try_index(idx, &mut probed, &mut probes, &mut out, self, &mut hits);
                idx += stride;
            }
        }

        if should_stop() { return None; }
        Some(out)
    }

    /// 하나의 ordering을 순차 배치한다.
    ///
    /// `prefix`가 `Some`이고 `prefix_len > 0`이면 `sequence[..prefix_len]`에 해당하는
    /// 앞부분 배치를 **그대로 재사용**하고 나머지만 새로 배치한다.
    ///
    /// > 156개 중 80번째 이후 순서만 바뀌었다면 앞의 79개 배치를 그대로 재사용
    ///
    /// `deadline`을 넘기면 남은 아이템을 전부 `unplaced`로 처리하고 즉시 반환한다.
    pub fn place_sequence(
        &mut self,
        sequence: &[usize],
        prefix: Option<&SequentialPlacement>,
        prefix_len: usize,
        deadline: Option<Instant>,
    ) -> SequentialPlacement {
        self.place_sequence_with_control(sequence, prefix, prefix_len, deadline, &|| false)
    }

    /// Adds phase Stop to the existing absolute deadline and Init cancellation.
    /// The current item's incomplete search is discarded on interruption.
    pub fn place_sequence_with_control(
        &mut self,
        sequence: &[usize],
        prefix: Option<&SequentialPlacement>,
        prefix_len: usize,
        deadline: Option<Instant>,
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> SequentialPlacement {
        let stopped = || should_stop()
            || deadline.is_some_and(|dl| Instant::now() >= dl)
            || crate::config::runtime::init_cancel_requested();
        let (mut placed, mut unplaced, start) = reuse_prefix(sequence, prefix, prefix_len);

        // ── 부유조각(floating parts) 분리 배치 ────────────────────────────
        //
        // 결(grain)이나 위치 제약이 걸린 조각은 자세를 고를 여지가 없다. 반대로 제약이
        // 없어 자유롭게 회전할 수 있는 조각은 어디든 자세를 맞춰 넣을 수 있다.
        //
        // 그래서 **제약이 강한 조각을 먼저** 놓는다(most-constrained-first). 자유도가
        // 높은 부유조각은 뒤로 미뤄, 앞선 배치가 만들어 놓은 공백에 자기 자세를 맞춰
        // 채워 넣게 한다. 반대로 하면 부유조각이 좋은 자리를 먼저 먹고, 자세를 못 바꾸는
        // 제약 조각이 갈 곳을 잃는다.
        //
        // 두 집합 모두 ordering 전략이 정한 **상대 순서를 그대로 보존**한다.
        let (constrained, floating) = self.split_by_constraint(&sequence[start..]);

        let mut expired = false;
        let order: Vec<usize> = constrained.into_iter().chain(floating).collect();
        for item_id in order {
            if expired || stopped() {
                expired = true;
                unplaced.push(item_id);
                continue;
            }
            // 부유조각이 채울 공백은 프론티어가 아니라 안쪽이라 탐침 예산을 더 준다.
            self.probe_scale = if self.is_floating(item_id) {
                FLOAT_FILL_PROBE_SCALE
            } else {
                1
            };
            let rots = self.rotations_for(item_id);
            let chosen = self.best_placement(item_id, &rots, &placed, &stopped);
            self.probe_scale = 1;
            if stopped() { expired = true; unplaced.push(item_id); continue; }
            match chosen {
                Some(piece) => placed.push(piece),
                None => unplaced.push(item_id),
            }
        }

        let strip_length = self.total_strip_length(&placed);
        SequentialPlacement {
            placed: placed.len(),
            pieces: placed,
            strip_length,
            unplaced,
        }
    }

    // ── 내부 구현 ──────────────────────────────────────────────────────────


    /// 배치 순서를 (제약 조각, 부유조각)으로 가른다.
    ///
    /// 판정 근거는 `Item::allowed_rotation`이다.
    ///
    /// ```text
    /// RotationRange::None            -> 제약 (자세 고정)
    /// RotationRange::Discrete([r])   -> 제약 (선택지 1개)
    /// RotationRange::Discrete([..])  -> 부유 (선택지 2개 이상)
    /// RotationRange::Continuous      -> 부유 (자유 회전)
    /// ```
    ///
    /// 의류 마커에서 결방향이 지정된 조각은 허용 회전이 하나뿐이라 제약으로 분류되고,
    /// 결 제약이 없는(non-nap) 조각만 부유조각이 된다.
    ///
    /// 두 목록 모두 입력 순서를 보존한다. 한쪽이 비면 분리에 의미가 없으므로 원본을 돌려준다.
    fn split_by_constraint(&self, sequence: &[usize]) -> (Vec<usize>, Vec<usize>) {
        if !FLOAT_PARTS_ENABLED {
            return (sequence.to_vec(), Vec::new());
        }
        let mut constrained = Vec::with_capacity(sequence.len());
        let mut floating = Vec::new();
        for &id in sequence {
            if self.is_floating(id) {
                floating.push(id);
            } else {
                constrained.push(id);
            }
        }
        if floating.is_empty() || constrained.is_empty() {
            return (sequence.to_vec(), Vec::new());
        }
        (constrained, floating)
    }

    /// 이 아이템이 부유조각(자세 제약 없음)인가.
    fn is_floating(&self, item_id: usize) -> bool {
        self.item_rotations
            .get(item_id)
            .is_some_and(|rots| rots.len() > 1)
    }

    /// 이 아이템에 실제로 허용된 회전 목록.
    ///
    /// 라이브러리 전체 회전 목록이 아니라 **아이템별** 허용 회전이다. 결방향이 걸린
    /// 조각에 라이브러리의 180도를 적용하면 원단 결을 어기게 된다.
    fn rotations_for(&self, item_id: usize) -> Vec<i64> {
        match self.item_rotations.get(item_id) {
            Some(rots) if !rots.is_empty() => rots.clone(),
            _ => {
                let lib = self.cache.library().rotations().to_vec();
                if lib.is_empty() { vec![0] } else { lib }
            }
        }
    }

    /// 한 아이템에 대해 허용 회전 전체를 훑어 최선의 배치를 고른다.
    fn best_placement(
        &mut self,
        item_id: usize,
        rotations: &[i64],
        placed: &[PlacedPiece],
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> Option<PlacedPiece> {
        if should_stop() { return None; }
        let placed_boxes = self.world_boxes(placed);
        // 이웃 bbox 순회(waste/contact)는 후보마다 반복되므로 SoA로 한 번만 옮겨 둔다.
        self.placed_bbox_soa.load_boxes(&placed_boxes);
        let current_len = placed_boxes
            .iter()
            .map(|b| b.max_x)
            .fold(0.0_f64, f64::max);
        // 탐침 구간 분리 기준. `candidates_for`가 읽는다.
        self.probe_frontier = current_len;

        let mut best: Option<(PlacedPiece, ScoreTerms, CandidateScore)> = None;

        for &rotation in rotations {
            if should_stop() { return None; }
            let Some((bounds, area)) = self.shape_metrics(item_id, rotation) else {
                continue;
            };
            let scales = Scales {
                length: self.strip_width_cap.max(1.0),
                height: self.strip_height.max(1.0),
                area: (area.max(1.0)) as f32,
                perimeter: (2.0 * (bounds.width() + bounds.height())).max(1.0) as f32,
            };

            // `None`은 중단이거나 obstacle NFP 계산 실패다. 중단이면 포기하고, 실패면
            // 이 회전만 건너뛴다(다음 회전 → 없으면 프론티어 폴백).
            let Some(candidates) =
                self.candidates_for_with_control(item_id, rotation, placed, should_stop)
            else {
                if should_stop() { return None; }
                continue;
            };
            for cand in candidates {
                if should_stop() { return None; }
                let (tx, ty) = (cand.translation.0 as f64, cand.translation.1 as f64);
                let world = bounds.translated(tx, ty);

                let raw = ScoreTerms {
                    strip_length: current_len.max(world.max_x) as f32,
                    waste: waste_of(world, area, &placed_boxes, &self.placed_bbox_soa) as f32,
                    contact: contact_of(
                        world,
                        &placed_boxes,
                        &self.placed_bbox_soa,
                        self.strip_height as f64,
                    ) as f32,
                    x: world.min_x as f32,
                    y: world.min_y as f32,
                };
                // 4단 우선순위는 **실제 mm 값**으로 비교한다.
                //
                // `STRIP_LENGTH_TIE_EPS` / `COORD_TIE_EPS`는 mm 단위 float 노이즈
                // 허용치다. 정규화된 값에 그대로 적용하면 축마다 나누는 기준이 달라
                // (strip_length·x는 strip_width_cap, y는 strip_height) 같은 1 mm 차이가
                // 축에 따라 동률이 되기도 하고 아니기도 한다. 문서의 우선순위는
                // "Strip Length 최소 -> x 최소 -> y 최소 -> 접촉도 최대"라는 실제 크기의
                // 사전식 순서이므로 raw로 비교해야 한다.
                //
                // 정규화는 차원이 다른 네 항을 하나의 가중합으로 더하는 `Score(p)`에만
                // 쓴다 (문서의 "근사 점수" 문단).
                let terms = normalize_terms(raw, scales);
                let score = self.weights.score(terms);

                let better = match &best {
                    None => true,
                    // 길이 동률 구간 안에서만 맞물림 품질로 고른다. 길이 차이가
                    // 허용치를 넘으면 언제나 짧은 쪽 — 접촉이 길이를 이길 수 없다.
                    Some((_, best_raw, best_score)) if self.weights.length_tie_mm > 0.0 => {
                        let tol = self.weights.length_tie_mm;
                        if (raw.strip_length - best_raw.strip_length).abs() > tol {
                            raw.strip_length < best_raw.strip_length
                        } else {
                            let a = self.weights.fit_value(score.terms);
                            let b = self.weights.fit_value(best_score.terms);
                            match a.partial_cmp(&b).unwrap_or(Ordering::Equal) {
                                Ordering::Less => true,
                                Ordering::Greater => false,
                                Ordering::Equal => {
                                    compare_by_priority(&raw, best_raw) == Ordering::Less
                                }
                            }
                        }
                    }
                    Some((_, best_raw, best_score)) if self.weights.score_first => {
                        // 가중합 우선. 접촉 길이와 낭비 면적이 실제로 자리를 고른다.
                        // 사전식 순서는 동률 처리로 내려간다 — 정규화된 점수가 같은
                        // 두 후보 사이에서는 여전히 짧은 strip, 왼쪽, 아래가 이긴다.
                        match score
                            .value
                            .partial_cmp(&best_score.value)
                            .unwrap_or(Ordering::Equal)
                        {
                            Ordering::Less => true,
                            Ordering::Greater => false,
                            Ordering::Equal => {
                                compare_by_priority(&raw, best_raw) == Ordering::Less
                            }
                        }
                    }
                    Some((_, best_raw, best_score)) => {
                        match compare_by_priority(&raw, best_raw) {
                            Ordering::Less => true,
                            Ordering::Greater => false,
                            // 4순위까지 동률이면 가중합 점수로 최종 결정한다.
                            Ordering::Equal => score
                                .value
                                .partial_cmp(&best_score.value)
                                .unwrap_or(Ordering::Equal)
                                == Ordering::Less,
                        }
                    }
                };
                if better {
                    best = Some((
                        PlacedPiece {
                            item_id,
                            rotation,
                            translation: cand.translation,
                        },
                        raw,
                        score,
                    ));
                }
            }
        }

        // 후보를 하나도 못 찾았으면 **배치를 포기하지 않고** 우측 프론티어 밖에 놓는다.
        //
        // NFP-LBF는 구성(constructive) 휴리스틱이다. 조각을 남기면 그 해는 수요를 못 채워
        // `solution_validation_error`의 `QuantityMismatch`로 통째로 버려진다 — 실제로
        // 여성WSjk 156-패널에서 45개가 남아 시드가 전부 폐기됐다. 이미 배치된 모든 조각의
        // 오른쪽 끝 너머는 정의상 아무와도 겹치지 않으므로, 그 자리는 항상 유효하다.
        // strip 길이는 늘지만 완전한 해를 얻고, 그 뒤 탐색 단계가 줄여 나간다.
        if should_stop() { return None; }
        if best.is_none() {
            return self.frontier_fallback(item_id, rotations, current_len);
        }

        best.map(|(piece, _, _)| piece)
    }

    /// 우측 프론티어 밖 배치. 겹칠 수 없으므로 항상 유효하다.
    ///
    /// strip 높이에 들어가는 회전이 하나도 없으면 `None`이다 (진짜로 놓을 수 없는 조각).
    fn frontier_fallback(
        &mut self,
        item_id: usize,
        rotations: &[i64],
        current_len: f64,
    ) -> Option<PlacedPiece> {
        let gap = FRONTIER_FALLBACK_GAP_MM;
        let strip_h = self.strip_height as f64;
        // IFP 경로와 같은 물리·수치 여백을 쓴다. 폴백만 y=0 에 붙이면 겹침이 켜졌을
        // 때 그 조각 하나가 해 전체를 무효로 만든다.
        let inset_x = self.strip_inset as f64 + super::cache::ifp_inset(self.strip_width_cap as f64);
        let physical_y = self.strip_inset as f64;
        let available_height = strip_h - 2.0 * physical_y;

        // 높이가 가장 낮은 회전을 고른다 — 이후 조각이 위에 쌓일 여지가 커진다.
        let mut best: Option<(f64, PlacedPiece)> = None;
        for &rotation in rotations {
            let Some((bounds, _)) = self.shape_metrics(item_id, rotation) else {
                continue;
            };
            let h = bounds.max_y - bounds.min_y;
            let height_slack = available_height - h;
            let roundoff = 2.0 * f32::EPSILON as f64 * strip_h.abs().max(h).max(1.0);
            if available_height <= 0.0 || height_slack < -roundoff {
                continue; // 어떤 위치에서도 strip 높이를 넘는다
            }
            // 위아래에 정확히 닿는 배치는 합법이다. 수치 여백을 넣을 자리가 없다는
            // 이유로 들어가는 조각을 버리지 않도록 여유를 남은 만큼만 쓴다.
            let inset_y = physical_y + super::cache::ifp_inset(strip_h).min(height_slack * 0.5);
            // 앵커 기준 bbox를 (current_len + gap, inset_y)에 좌하단 정렬한다.
            let tx = (current_len + gap).max(inset_x) - bounds.min_x;
            let ty = inset_y - bounds.min_y;
            if !tx.is_finite() || !ty.is_finite() {
                continue;
            }
            let candidate = PlacedPiece {
                item_id,
                rotation,
                translation: (tx as f32, ty as f32),
            };
            if best.as_ref().is_none_or(|(best_h, _)| h < *best_h) {
                best = Some((h, candidate));
            }
        }

        best.map(|(_, piece)| piece)
    }

    /// 기존 배치 아이템별 `t_o + NFP(o, item)`를 모은다.
    ///
    /// NFP는 **복사하지 않는다**. 로컬 좌표 `Arc`와 평행이동만 들고 다니며, 월드 좌표가
    /// 필요한 곳(후보 생성, bbox)에서만 정점에 평행이동을 더한다. `Obstacle`의 문서 참조.
    fn collect_obstacles(
        &mut self,
        hash_item: GeometryHash,
        rotation: i64,
        placed: &[PlacedPiece],
        should_stop: &(impl Fn() -> bool + Sync),
    ) -> Option<Vec<Obstacle>> {
        let mut out = Vec::with_capacity(placed.len());
        for piece in placed {
            if should_stop() { return None; }
            let Some(hash_o) = self.cache.library().hash_of_item(piece.item_id) else {
                continue;
            };
            let key = NfpKey::new(hash_o, piece.rotation, hash_item, rotation);
            // 정규화 영역까지 함께 받는다 — `locate`의 호출당 재정규화를 없애는 자리다.
            // NFP 계산 실패 — obstacle 하나를 빠뜨린 채 후보를 만들면 겹침 배치가 나온다.
            // 이 회전의 후보 집합을 통째로 포기한다(호출자가 다음 회전으로 넘어간다).
            let Some((region, mask, prepared)) = self.cache.view_prepared_or_compute(key) else {
                return None;
            };
            if should_stop() { return None; }
            let tx = piece.translation.0 as f64;
            let ty = piece.translation.1 as f64;
            // 로컬 bbox를 평행이동한 값은 `set_bounds(translate_set(..))`와 비트 단위로
            // 같다 — `fl(v + t)`가 `v`에 대해 단조라 min/max의 자리가 바뀌지 않는다.
            let Some((lx, ly, hx, hy)) = mask.bounds() else {
                continue;
            };
            let bbox = Aabb {
                min_x: lx,
                min_y: ly,
                max_x: hx,
                max_y: hy,
            }
            .translated(tx, ty);

            let (tx_q, tx_exact) = grid_translation(tx);
            let (ty_q, ty_exact) = grid_translation(ty);
            // 격자 반올림이 애매하면 옛 경로(월드 좌표 사본)를 쓴다 — 결과 동일성이 최우선이다.
            let world = if tx_exact && ty_exact {
                None
            } else {
                Some(translate_set(&region, tx, ty))
            };
            // 폴백 경로도 질의는 반복된다 — 정규화를 obstacle당 한 번으로 끝낸다.
            let world_prepared = world
                .as_ref()
                .and_then(|w| self.engine.prepare_region(w).ok());

            out.push(Obstacle {
                region,
                mask,
                prepared,
                tx,
                ty,
                tx_q,
                ty_q,
                bbox,
                world,
                world_prepared,
            });
        }
        if should_stop() { return None; }
        Some(out)
    }


    /// `generators`가 가리키는 obstacle의 edge만 SoA로 적재한다.
    ///
    /// `self.obs_edge_soa[k]`는 **generators[k]** 의 edge다 (obstacle 인덱스가 아니다).
    fn load_obstacle_edges_subset(&mut self, obstacles: &[Obstacle], generators: &[usize]) {
        while self.obs_edge_soa.len() < generators.len() {
            self.obs_edge_soa.push(EdgeSoA::default());
        }
        for (k, &oi) in generators.iter().enumerate() {
            let o = &obstacles[oi];
            self.obs_edge_soa[k].load_from_set_translated(o.local_rings(), o.tx, o.ty);
        }
    }

    /// 후보 점 `p`의 feasible 판정.
    ///
    /// `hits`는 bbox 사전 필터를 통과한 obstacle 인덱스를 담는 재사용 스크래치다.
    /// SIMD 경로에서도 인덱스는 **오름차순**으로 나오므로 `locate` 호출 순서가
    /// 스칼라와 같고, 따라서 판정 결과도 완전히 같다.
    ///
    /// # `locate` → `locate_prepared`
    ///
    /// 마스크가 `Boundary`로 넘긴 나머지는 여전히 정확한 점 위치 질의가 필요하다.
    /// 그 질의를 `NfpEngine::locate`로 하면 **호출마다** 영역이 재정규화된다 —
    /// 여성WSjk 156-패널 3-ordering에서 63,588회 × 약 36 µs ≈ 2.3초로, 남은 4.67초의
    /// 절반쯤이었다. 영역은 캐시가 한 번만 정규화해 두므로 여기서는
    /// `locate_prepared`로 질의한다. 두 함수는 정의상 결과가 같으므로 판정은
    /// 비트 단위로 동일하고, 사라지는 것은 재정규화뿐이다.
    /// (`*_prepared`가 `None`인 영역은 정규화가 실패한 것이라 예전 `locate` 경로를
    ///  그대로 쓴다. `locate`도 같은 정규화에서 `Err`를 내므로 결과는 역시 같다.)
    #[allow(clippy::too_many_arguments)]
    /// feasible 하면 **정확히 맞닿는 조각 수**를, 아니면 `None`.
    ///
    /// 접촉 수는 덤이 아니라 **이미 계산하던 값**이다. 이 판정은 obstacle 마다
    /// `Location` 을 구하는데, `Boundary` 가 곧 접촉이다. 예전에는 그걸 버리고
    /// `bool` 만 돌려줬다 — 그래서 전략 계층이 bbox 근사로 접촉을 다시 추정해야 했고,
    /// 오목한 패널이 실제로 맞물려도 놓쳤다(실측: 후보의 76.2%가 접촉 0으로 나왔다).
    ///
    /// 비용은 `u16` 증가 연산뿐이다.
    #[allow(clippy::too_many_arguments)]
    fn feasible_touch_count(
        &self,
        x: f64,
        y: f64,
        generator: u32,
        ifp: &PolygonSet,
        ifp_mask: &NfpMask,
        ifp_prepared: Option<&PreparedRegion>,
        ifp_bbox: Aabb,
        obstacles: &[Obstacle],
        hits: &mut Vec<usize>,
    ) -> Option<u16> {
        let eps = COORD_TIE_EPS as f64;
        // 정확히 맞닿은 조각 수. `Location::Boundary` 가 곧 접촉이다.
        let mut touches: u16 = 0;
        // (b) IFP 안(내부 또는 경계)이어야 한다. bbox로 먼저 싸게 걸러낸다.
        if !ifp_bbox.contains(x, y, eps) {
            return None;
        }
        // IFP 판정은 탐침마다 반드시 일어난다 — 마스크가 가장 크게 먹히는 자리다.
        // IFP는 이미 월드 좌표라 평행이동 보정이 필요 없다.
        match ifp_mask.classify(x, y) {
            MaskCell::Inside => {}
            MaskCell::Outside => return None,
            MaskCell::Boundary => {
                let located = match ifp_prepared {
                    Some(prepared) => self.engine.locate_prepared(prepared, NfpPoint::new(x, y)),
                    None => self.engine.locate(ifp, NfpPoint::new(x, y)),
                };
                match located {
                    Ok(Location::Inside) | Ok(Location::Boundary) => {}
                    _ => return None,
                }
            }
        }
        // (a) 어떤 NFP 내부에도 있으면 안 된다. 경계는 접촉 배치라 허용한다.
        bbox_hits(&self.obs_bbox_soa, x, y, eps, hits);
        if hits.is_empty() {
            // bbox 사전 필터가 아무것도 안 걸렀다 = 근처에 조각이 없다 = 접촉 0.
            // (strip 경계와의 접촉은 여기서 세지 않는다 — 그건 IFP 판정이다.)
            return Some(0);
        }
        // 질의점을 격자 정수로 한 번만 올려 둔다. obstacle마다 정수 뺄셈만 하면 된다.
        let (px_q, py_q) = match (grid_round(x), grid_round(y)) {
            (Some(px), Some(py)) => (px, py),
            // 격자에 못 올리는 좌표는 `locate`가 어차피 Err를 준다 = 기각.
            _ => return None,
        };
        for &k in hits.iter() {
            let o = &obstacles[k];
            // 이 후보를 만든 obstacle이라면 점이 그 NFP의 **정점 그 자체**다.
            //
            // 후보 좌표는 `fl(v + t)`이고 질의 좌표는 `round(GRID·(v+t)) - round(GRID·t)`
            // = `GRID·v`(커널 출력 정점은 격자 위의 정확한 정수다). 따라서
            // `point_in_ring`의 `on_segment`가 반드시 걸려 `Boundary`가 나온다 —
            // 접촉 배치는 허용이므로 결과를 바꾸지 않고 `locate` 한 번을 통째로 아낀다.
            // (격자 반올림이 애매해 월드 폴백을 쓰는 obstacle은 이 등식이 성립하지
            //  않으므로 제외한다.)
            if k as u32 == generator && o.world.is_none() {
                // 이 후보는 이 obstacle 의 NFP 정점 그 자체다 = **정의상 경계 접촉**이다.
                // `locate` 를 건너뛰는 것이지 접촉이 없는 게 아니므로 여기서 센다.
                touches = touches.saturating_add(1);
                continue;
            }
            if let Some(world) = o.world.as_ref() {
                // 격자 반올림이 애매한 obstacle — 옛 경로 그대로(질의 영역만 미리 정규화).
                let located = match o.world_prepared.as_ref() {
                    Some(prepared) => self.engine.locate_prepared(prepared, NfpPoint::new(x, y)),
                    None => self.engine.locate(world, NfpPoint::new(x, y)),
                };
                match located {
                    Ok(Location::Inside) => return None,
                    Ok(Location::Boundary) => {
                        touches = touches.saturating_add(1);
                        continue;
                    }
                    Ok(Location::Outside) => continue,
                    Err(_) => return None,
                }
            }
            // 로컬 좌표 질의점. 정수 격자에서 빼므로 옛 경로와 판정이 완전히 같다.
            let lx = (px_q - o.tx_q) as f64 / GRID_SCALE as f64;
            let ly = (py_q - o.ty_q) as f64 / GRID_SCALE as f64;
            match o.mask.classify(lx, ly) {
                // 셀 전체가 밖 — `locate` 호출 없이 확정.
                MaskCell::Outside => {}
                // 셀 전체가 안 — 즉시 기각.
                MaskCell::Inside => return None,
                // 경계가 지나갈 수 있다 — 정확한 판정은 여전히 점 위치 질의가 한다.
                MaskCell::Boundary => {
                    let located = match o.prepared.as_deref() {
                        Some(prepared) => {
                            self.engine.locate_prepared(prepared, NfpPoint::new(lx, ly))
                        }
                        None => self.engine.locate(&o.region, NfpPoint::new(lx, ly)),
                    };
                    match located {
                        Ok(Location::Inside) => return None,
                        Ok(Location::Boundary) => touches = touches.saturating_add(1),
                        Ok(Location::Outside) => {}
                        Err(_) => return None,
                    }
                }
            }
        }
        Some(touches)
    }

    /// 아이템·회전의 (앵커 기준 bbox, 면적 mm²).
    fn shape_metrics(&self, item_id: usize, rotation: i64) -> Option<(Aabb, f64)> {
        let library = self.cache.library();
        let hash = library.hash_of_item(item_id)?;
        let shape = library.shape(hash, rotation)?;
        let b = shape.bounds();
        Some((
            Aabb {
                min_x: b.min.x,
                min_y: b.min.y,
                max_x: b.max.x,
                max_y: b.max.y,
            },
            shape.area_mm2(),
        ))
    }

    /// 배치된 조각들의 월드 bbox 목록.
    fn world_boxes(&self, placed: &[PlacedPiece]) -> Vec<Aabb> {
        placed
            .iter()
            .filter_map(|p| {
                self.shape_metrics(p.item_id, p.rotation)
                    .map(|(b, _)| b.translated(p.translation.0 as f64, p.translation.1 as f64))
            })
            .collect()
    }

    /// 배치된 모든 조각의 `x_max` 최대값.
    fn total_strip_length(&self, placed: &[PlacedPiece]) -> f32 {
        self.world_boxes(placed)
            .iter()
            .map(|b| b.max_x)
            .fold(0.0_f64, f64::max) as f32
    }
}

// ────────────────────────────────────────────────────────────────────────────
// ④ 포켓 우선 — 구간 경계
// ────────────────────────────────────────────────────────────────────────────

/// 포켓(= 프론티어 안쪽) 후보 구간의 끝 인덱스.
///
/// `sorted`는 `candidates_for`가 정렬해 둔 x 오름차순 후보 목록이고, `.0`이 앵커 x다.
/// `anchor_max_x`는 조각 bbox의 **앵커 기준** 우측 끝이다.
///
/// # 왜 `anchor_max_x`인가
///
/// 월드 bbox가 `bounds.translated(x, y)`이므로 앵커 x에 놓인 조각의 우측 끝은
/// `x + anchor_max_x`다. 그게 프론티어를 넘지 않아야 "프론티어를 밀지 않는 자리",
/// 즉 포켓이다. 따라서 경계는 `프론티어 - anchor_max_x`.
///
/// # 왜 함수로 떼어냈나
///
/// 여기에 `ifp_bbox.width()`(= `strip_width_cap - 조각폭`)를 넣는 실수가 ④를
/// 통째로 no-op으로 만든 적이 있다. wsjk156에서 `strip_width_cap = Σ 조각폭 =
/// 51141.09 mm`인데 프론티어는 최대 ~7.3e3 mm라, 경계가 **항상** 음수 →
/// `left_end`가 늘 0 → 포켓 분기에 한 번도 들어가지 않았다. 두 값은 부호까지
/// 뒤집혀 있어 눈으로는 구분이 안 되므로, 테스트가 붙는 자리로 꺼내 둔다.
fn pocket_left_end(sorted: &[(f64, f64, CandidateSource, u32)], frontier: f64, anchor_max_x: f64) -> usize {
    if !(frontier > 0.0) || !anchor_max_x.is_finite() {
        return 0;
    }
    let boundary = frontier - anchor_max_x;
    sorted.partition_point(|c| c.0 < boundary)
}

// ────────────────────────────────────────────────────────────────────────────
// prefix 재사용
// ────────────────────────────────────────────────────────────────────────────

/// `sequence[..prefix_len]`에 해당하는 앞부분 배치를 복원한다.
///
/// `prefix.pieces`에는 **배치 성공한 조각만** 들어 있으므로 인덱스가 `sequence`와
/// 1:1로 맞지 않는다. `item_id`를 순서대로 그리디 매칭해 성공/실패를 재구성한다.
///
/// 반환값은 `(재사용 배치, 재사용 구간의 unplaced, 새로 배치할 시작 인덱스)`.
fn reuse_prefix(
    sequence: &[usize],
    prefix: Option<&SequentialPlacement>,
    prefix_len: usize,
) -> (Vec<PlacedPiece>, Vec<usize>, usize) {
    let mut placed = Vec::new();
    let mut unplaced = Vec::new();

    let Some(pfx) = prefix else {
        return (placed, unplaced, 0);
    };
    let n = prefix_len.min(sequence.len());
    if n == 0 {
        return (placed, unplaced, 0);
    }

    let mut j = 0usize;
    for (i, &item_id) in sequence.iter().take(n).enumerate() {
        if j < pfx.pieces.len() && pfx.pieces[j].item_id == item_id {
            placed.push(pfx.pieces[j]);
            j += 1;
        } else if pfx.unplaced.contains(&item_id) {
            unplaced.push(item_id);
        } else {
            // 접두가 이 아이템을 모른다(순서가 갈라졌다) — 재사용은 여기서 끝내고
            // 나머지는 정상 배치한다. 모르는 아이템을 unplaced로 꾸며내면 안 된다.
            return (placed, unplaced, i);
        }
    }
    (placed, unplaced, n)
}

// ────────────────────────────────────────────────────────────────────────────
// 점수 항 근사
// ────────────────────────────────────────────────────────────────────────────

/// `Waste(p)` 근사 — 이 위치에 놓았을 때 새로 생기는 사용 불가 공백(mm²).
///
/// 두 성분의 합이다.
///
/// ```text
/// (1) bbox 잉여   : 조각 bbox 면적 − 조각 실제 면적
/// (2) 좌측 사각지대: (x_min − 같은 y 띠의 기존 frontier) × bbox 높이
/// ```
///
/// (2)는 "왼쪽에 남겨두고 건너뛴" 폭을 벌점으로 매긴다. frontier가 없으면 strip
/// 좌단(`x = 0`)을 기준으로 삼는다.
///
/// simd feature가 켜지면 실사용 경로는 [`waste_of`]가 SoA 벡터 커널로 대체하고,
/// 이 함수는 그 정확성을 고정하는 기준으로 테스트에서만 호출된다.
#[cfg_attr(feature = "simd", allow(dead_code))]
fn approx_waste(world: Aabb, item_area: f64, placed: &[Aabb]) -> f64 {
    let bbox_slack = (world.area() - item_area).max(0.0);

    let mut frontier = 0.0_f64;
    for b in placed {
        // y 구간이 겹치는 기존 조각만 이 띠의 frontier에 기여한다.
        let lo = world.min_y.max(b.min_y);
        let hi = world.max_y.min(b.max_y);
        if hi - lo > COORD_TIE_EPS as f64 {
            frontier = frontier.max(b.max_x);
        }
    }
    let gap = (world.min_x - frontier).max(0.0);
    bbox_slack + gap * world.height().max(0.0)
}

/// `Contact(p)` 근사 — 기존 패턴 및 strip boundary와 맞닿는 길이(mm).
///
/// bbox 근사이므로 간극 `g`에 대해 `1 − g / CONTACT_BAND_MM`으로 선형 감쇠시킨다.
/// strip 상·하단(`y = 0`, `y = strip_height`)과 좌단(`x = 0`)도 접촉으로 센다.
///
/// simd feature가 켜지면 실사용 경로는 [`contact_of`]가 SoA 벡터 커널로 대체한다.
#[cfg_attr(feature = "simd", allow(dead_code))]
fn approx_contact(world: Aabb, placed: &[Aabb], strip_height: f64) -> f64 {
    let mut contact = 0.0_f64;

    for b in placed {
        // 수직 면 접촉(좌/우) — 겹치는 y 길이만큼
        let y_overlap = (world.max_y.min(b.max_y) - world.min_y.max(b.min_y)).max(0.0);
        if y_overlap > 0.0 {
            contact += y_overlap * proximity(axis_gap(b.min_x - world.max_x, world.min_x - b.max_x));
        }
        // 수평 면 접촉(상/하) — 겹치는 x 길이만큼
        let x_overlap = (world.max_x.min(b.max_x) - world.min_x.max(b.min_x)).max(0.0);
        if x_overlap > 0.0 {
            contact += x_overlap * proximity(axis_gap(b.min_y - world.max_y, world.min_y - b.max_y));
        }
    }

    // strip boundary
    contact += world.width().max(0.0) * proximity(world.min_y.abs());
    contact += world.width().max(0.0) * proximity((strip_height - world.max_y).abs());
    contact += world.height().max(0.0) * proximity(world.min_x.abs());

    contact
}

/// 한 축에서 두 bbox 사이의 **실제 간극**(mm). 겹치면 `0`.
///
/// # 왜 `.abs()` 가 아닌가
///
/// 예전에는 양쪽 방향의 차이를 각각 `.abs()` 해서 두 항으로 더했다. 그러면 두 bbox 가
/// **겹칠 때**(둘 다 음수) 겹침 깊이가 그대로 "간극" 이 되어, 깊이 맞물린 조각의
/// 접촉이 **0으로 계산된다.**
///
/// 그런데 여기 오는 후보는 이미 NFP 로 feasible 판정을 통과했다 — **실제 도형은 겹치지
/// 않는다.** bbox 만 겹치는 것이고, 그건 두 조각이 서로의 오목한 부분에 **맞물려
/// 들어갔다**는 뜻이다. 접촉이 없기는커녕 가장 좋은 배치다.
///
/// 의류 패널은 대부분 오목해서 이 경우가 흔하다. 전략 계층에서 같은 결함을 고칠 때 잰
/// 값: 후보 105,109개 중 **76.2%가 접촉 0** 이었다
/// ([`crate::nfp_strategy`] 의 `proximity` 문서 참조). 오라클 쪽은 그때 고치지 않아
/// 같은 결함이 남아 있었다.
///
/// 두 방향 차이 중 **큰 쪽**이 실제 간극이다(둘 다 양수일 수는 없다 — 상대가 왼쪽에
/// 있으면서 동시에 오른쪽에 있을 수 없다). 그것을 0 으로 하한 잡으면 겹침이 맞닿음이 된다.
#[inline]
fn axis_gap(near: f64, far: f64) -> f64 {
    near.max(far).max(0.0)
}

/// 간극 `g`(mm)를 `[0, 1]` 접촉 계수로 바꾼다.
#[inline]
fn proximity(gap: f64) -> f64 {
    if gap >= CONTACT_BAND_MM {
        0.0
    } else {
        1.0 - gap / CONTACT_BAND_MM
    }
}

/// 각 항을 자기 척도로 나눠 자릿수를 맞춘다.
///
/// **항마다** 배율이 다르다 — `strip_length`·`x` 는 `scales.length`(= `strip_width_cap`),
/// `y` 는 `scales.height`, `waste` 는 면적, `contact` 는 둘레로 나눈다. 한 항 안에서는
/// 모든 후보에 같은 양수 배율이 걸리지만, **항 사이의 상대 크기는 바뀐다.**
///
/// 그래서 이 결과를 [`compare_by_priority`] 에 넣으면 안 된다. 우선순위는 축을 넘나드는
/// 사전식 비교(strip length -> x -> y -> 접촉)라, 축마다 다른 배율이 걸리면 같은 1mm
/// 차이가 축에 따라 동률이 되기도 하고 아니기도 한다. 실제로 그렇게 만들었다가
/// 한 줄 배치가 나왔다(strip 126.9 -> 고친 뒤 42.0).
///
/// 호출부는 우선순위를 **raw mm** 로 비교하고, 이 정규화는 차원이 다른 네 항을 하나의
/// 가중합으로 더하는 `Score(p)` 에만 쓴다.
fn normalize_terms(raw: ScoreTerms, scales: Scales) -> ScoreTerms {
    ScoreTerms {
        strip_length: raw.strip_length / scales.length,
        waste: raw.waste / scales.area,
        contact: raw.contact / scales.perimeter,
        x: raw.x / scales.length,
        y: raw.y / scales.height,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// PolygonSet 유틸
// ────────────────────────────────────────────────────────────────────────────

/// outer + holes를 모두 훑는다.
fn set_rings(set: &PolygonSet) -> impl Iterator<Item = &NfpRing> {
    set.components
        .iter()
        .flat_map(|c| std::iter::once(&c.outer).chain(c.holes.iter()))
}

/// 모든 ring의 닫힌 edge를 `[x1, y1, x2, y2]`로 펼친다.
///
/// 실사용 경로는 [`EdgeSoA::load_from_set`](EdgeSoA)이 같은 순서로 SoA에 직접 채운다.
/// 이 AoS 버전은 그 순서 규약을 고정하는 참조 구현으로 테스트에만 남겨 둔다.
#[cfg(test)]
fn set_edges(set: &PolygonSet) -> Vec<[f64; 4]> {
    let mut out = Vec::new();
    for ring in set_rings(set) {
        let n = ring.vertices.len();
        if n < 2 {
            continue;
        }
        for i in 0..n {
            let a = ring.vertices[i];
            let b = ring.vertices[(i + 1) % n];
            out.push([a.x, a.y, b.x, b.y]);
        }
    }
    out
}

/// 집합 전체의 bbox. 정점이 하나도 없으면 `None`.
fn set_bounds(set: &PolygonSet) -> Option<Aabb> {
    let mut acc: Option<Aabb> = None;
    for ring in set_rings(set) {
        for v in &ring.vertices {
            acc = Some(match acc {
                None => Aabb {
                    min_x: v.x,
                    min_y: v.y,
                    max_x: v.x,
                    max_y: v.y,
                },
                Some(b) => Aabb {
                    min_x: b.min_x.min(v.x),
                    min_y: b.min_y.min(v.y),
                    max_x: b.max_x.max(v.x),
                    max_y: b.max_y.max(v.y),
                },
            });
        }
    }
    acc
}

/// 컴포넌트와 hole 구조를 그대로 보존하며 평행이동한다.
///
/// `NfpEngine::translate_region`은 매번 재정규화를 하므로, 후보 추출·점 조회 용도의
/// 값싼 평행이동은 여기서 직접 처리한다.
fn translate_set(set: &PolygonSet, dx: f64, dy: f64) -> PolygonSet {
    PolygonSet::new(
        set.components
            .iter()
            .map(|c| {
                NfpPolygon::with_holes(
                    translate_ring(&c.outer, dx, dy),
                    c.holes.iter().map(|h| translate_ring(h, dx, dy)).collect(),
                )
            })
            .collect(),
    )
}

fn translate_ring(ring: &NfpRing, dx: f64, dy: f64) -> NfpRing {
    NfpRing {
        vertices: ring
            .vertices
            .iter()
            .map(|p| NfpPoint::new(p.x + dx, p.y + dy))
            .collect(),
    }
}

/// `swarm_nfp`의 `quantize_mm`과 **같은 식**으로 mm를 0.001mm 격자 정수로 올린다.
///
/// 격자에 못 올리는 값(비유한/과대)은 `None`. 그런 좌표는 `locate`도 `Err`를 준다.
#[inline]
fn grid_round(v: f64) -> Option<i64> {
    let scaled = v * GRID_SCALE as f64;
    if !scaled.is_finite() || scaled.abs() > 9.0e15 {
        return None;
    }
    Some(scaled.round() as i64)
}

/// 평행이동을 격자 정수로 올리고, 그 반올림이 **애매하지 않은지** 함께 돌려준다.
///
/// 옛 경로는 정점마다 `round(GRID·(v + t))`를 계산했다. `GRID·v`가 정확한 정수이므로
/// 그 값은 `GRID·v + round(GRID·t + η)`이고, `η`는 부동소수 오차(5e-9 수준)다.
/// 소수부가 `0.5`에서 [`GRID_TIE_MARGIN`]만큼 떨어져 있으면 `η`가 반올림을 못 바꾸므로
/// `round(GRID·t)` 하나로 정점 전체를 옮긴 것과 결과가 같다.
#[inline]
fn grid_translation(t: f64) -> (i64, bool) {
    let scaled = t * GRID_SCALE as f64;
    if !scaled.is_finite() || scaled.abs() > 9.0e15 {
        return (0, false);
    }
    let rounded = scaled.round();
    let residual = (scaled - rounded).abs();
    (rounded as i64, residual <= 0.5 - GRID_TIE_MARGIN)
}

/// 0.001mm 격자 키. 중복 후보 제거용.
#[inline]
fn grid_key(x: f64, y: f64) -> (i64, i64) {
    let s = GRID_SCALE as f64;
    ((x * s).round() as i64, (y * s).round() as i64)
}

/// 격자 키 `(i64, i64)` 전용의 싼 결정적 해셔.
///
/// 후보 중복 제거는 아이템·회전마다 수천 번 `insert`하므로 SipHash의 키 설정·블록
/// 처리 비용이 그대로 보인다. 입력이 신뢰된 정수 두 개뿐이라 HashDoS 방어는 필요 없고,
/// 곱셈 혼합 한 번이면 hashbrown의 상위 7비트 태그와 하위 인덱스 비트가 충분히 섞인다.
#[derive(Default)]
struct GridKeyHasher(u64);

impl Hasher for GridKeyHasher {
    #[inline]
    fn finish(&self) -> u64 {
        let h = self.0;
        h ^ (h >> 32)
    }

    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u64(b as u64);
        }
    }

    #[inline]
    fn write_i64(&mut self, i: i64) {
        self.write_u64(i as u64);
    }

    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.0 = (self.0.rotate_left(29) ^ i).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}

type GridKeySet = HashSet<(i64, i64), BuildHasherDefault<GridKeyHasher>>;

#[inline]
/// 후보를 생성할 obstacle 목록.
///
/// 한때 프론티어 근처로 부분집합을 잡아 봤지만(밴드 3배 + 최소 24개) 여성WSjk 156-패널에서
/// 시간은 20.4초 -> 18.6초로 거의 줄지 않은 반면 strip 길이가 7112 -> 7568로 나빠졌다.
/// 비용의 지배항이 edge x edge 교점이 아니었기 때문이다. 그래서 전체를 그대로 쓴다.
fn select_candidate_generators(obstacles: &[Obstacle], _ifp_bbox: Aabb) -> Vec<usize> {
    (0..obstacles.len()).collect()
}

fn aabb_overlaps(a: Aabb, b: Aabb, eps: f64) -> bool {
    a.min_x <= b.max_x + eps
        && b.min_x <= a.max_x + eps
        && a.min_y <= b.max_y + eps
        && b.min_y <= a.max_y + eps
}

/// 두 선분의 bbox가 겹치는지. edge 쌍 사전 필터의 **참조 구현**.
///
/// 실사용 경로는 [`edge_bbox_hits`]가 같은 비교식을 SoA 위에서 한 번에 판정한다.
#[cfg(test)]
#[inline]
fn segment_bbox_overlaps(a: &[f64; 4], b: &[f64; 4], eps: f64) -> bool {
    let (a_min_x, a_max_x) = min_max(a[0], a[2]);
    let (a_min_y, a_max_y) = min_max(a[1], a[3]);
    let (b_min_x, b_max_x) = min_max(b[0], b[2]);
    let (b_min_y, b_max_y) = min_max(b[1], b[3]);
    a_min_x <= b_max_x + eps
        && b_min_x <= a_max_x + eps
        && a_min_y <= b_max_y + eps
        && b_min_y <= a_max_y + eps
}

#[inline]
fn min_max(a: f64, b: f64) -> (f64, f64) {
    if a <= b { (a, b) } else { (b, a) }
}

/// 두 선분의 교점. 평행/공선은 `None`을 돌려준다.
///
/// 공선 겹침에서 나오는 점은 이미 NFP vertex 후보로 들어와 있으므로 중복 처리할
/// 필요가 없다.
fn segment_intersection(a: &[f64; 4], b: &[f64; 4]) -> Option<(f64, f64)> {
    const DENOM_EPS: f64 = 1.0e-12;
    const PARAM_EPS: f64 = 1.0e-9;

    let rx = a[2] - a[0];
    let ry = a[3] - a[1];
    let sx = b[2] - b[0];
    let sy = b[3] - b[1];

    let denom = rx * sy - ry * sx;
    if denom.abs() <= DENOM_EPS {
        return None;
    }

    let qpx = b[0] - a[0];
    let qpy = b[1] - a[1];
    let t = (qpx * sy - qpy * sx) / denom;
    let u = (qpx * ry - qpy * rx) / denom;

    if t < -PARAM_EPS || t > 1.0 + PARAM_EPS || u < -PARAM_EPS || u > 1.0 + PARAM_EPS {
        return None;
    }
    let x = a[0] + t * rx;
    let y = a[1] + t * ry;
    if x.is_finite() && y.is_finite() {
        Some((x, y))
    } else {
        None
    }
}

// ────────────────────────────────────────────────────────────────────────────
// SoA 버퍼
// ────────────────────────────────────────────────────────────────────────────

/// bbox 묶음의 SoA(Structure of Arrays) 레이아웃.
///
/// AoS(`Vec<Aabb>`)는 한 lane을 채우려고 32B stride로 흩어진 필드를 긁어와야 한다.
/// 축별로 연속 배열을 두면 벡터 로드 한 번에 `W`개가 들어온다.
#[derive(Clone, Debug, Default)]
#[repr(align(32))]
struct AabbSoA {
    min_x: Vec<f64>,
    min_y: Vec<f64>,
    max_x: Vec<f64>,
    max_y: Vec<f64>,
}

impl AabbSoA {
    #[inline]
    fn len(&self) -> usize {
        self.min_x.len()
    }

    #[inline]
    fn clear(&mut self) {
        self.min_x.clear();
        self.min_y.clear();
        self.max_x.clear();
        self.max_y.clear();
    }

    #[inline]
    fn push(&mut self, b: Aabb) {
        self.min_x.push(b.min_x);
        self.min_y.push(b.min_y);
        self.max_x.push(b.max_x);
        self.max_y.push(b.max_y);
    }

    /// `clear` + `push`이므로 용량이 유지된다 — 아이템마다 재할당하지 않는다.
    fn load_boxes(&mut self, boxes: &[Aabb]) {
        self.clear();
        for b in boxes {
            self.push(*b);
        }
    }

    fn load_obstacles(&mut self, obstacles: &[Obstacle]) {
        self.clear();
        for o in obstacles {
            self.push(o.bbox);
        }
    }
}

/// edge 묶음의 SoA. 끝점과 함께 **미리 계산한 edge bbox**를 같이 들고 다닌다.
///
/// bbox를 미리 두면 사전 필터 안쪽 루프에서 `min_max`를 반복하지 않아도 되고,
/// 축별 연속 배열이라 그대로 벡터 로드가 된다.
#[derive(Clone, Debug, Default)]
#[repr(align(32))]
struct EdgeSoA {
    x1: Vec<f64>,
    y1: Vec<f64>,
    x2: Vec<f64>,
    y2: Vec<f64>,
    min_x: Vec<f64>,
    min_y: Vec<f64>,
    max_x: Vec<f64>,
    max_y: Vec<f64>,
}

impl EdgeSoA {
    #[inline]
    fn len(&self) -> usize {
        self.x1.len()
    }

    fn clear(&mut self) {
        self.x1.clear();
        self.y1.clear();
        self.x2.clear();
        self.y2.clear();
        self.min_x.clear();
        self.min_y.clear();
        self.max_x.clear();
        self.max_y.clear();
    }

    /// 모든 ring의 닫힌 edge를 [`set_edges`]와 **같은 순서로** 담는다.
    ///
    /// `-0.0 + 0.0 = +0.0`처럼 0의 부호가 바뀌지 않도록, 평행이동 없는 경로는
    /// 덧셈을 아예 하지 않는다.
    fn load_from_set(&mut self, set: &PolygonSet) {
        self.clear();
        for ring in set_rings(set) {
            let n = ring.vertices.len();
            if n < 2 {
                continue;
            }
            for i in 0..n {
                let a = ring.vertices[i];
                let b = ring.vertices[(i + 1) % n];
                let (min_x, max_x) = min_max(a.x, b.x);
                let (min_y, max_y) = min_max(a.y, b.y);
                self.x1.push(a.x);
                self.y1.push(a.y);
                self.x2.push(b.x);
                self.y2.push(b.y);
                self.min_x.push(min_x);
                self.min_y.push(min_y);
                self.max_x.push(max_x);
                self.max_y.push(max_y);
            }
        }
    }

    /// [`Self::load_from_set`]과 같지만 정점에 `(dx, dy)`를 더하며 담는다.
    ///
    /// obstacle NFP는 캐시의 **로컬 좌표**로 들고 다니므로(`Obstacle` 문서 참조),
    /// 월드 edge가 필요한 후보 생성 경로에서만 여기서 평행이동한다.
    /// `PolygonSet`을 통째로 복사하던 옛 경로와 값이 비트 단위로 같다.
    fn load_from_set_translated(&mut self, set: &PolygonSet, dx: f64, dy: f64) {
        self.clear();
        for ring in set_rings(set) {
            let n = ring.vertices.len();
            if n < 2 {
                continue;
            }
            for i in 0..n {
                let a = ring.vertices[i];
                let b = ring.vertices[(i + 1) % n];
                let (ax, ay) = (a.x + dx, a.y + dy);
                let (bx, by) = (b.x + dx, b.y + dy);
                let (min_x, max_x) = min_max(ax, bx);
                let (min_y, max_y) = min_max(ay, by);
                self.x1.push(ax);
                self.y1.push(ay);
                self.x2.push(bx);
                self.y2.push(by);
                self.min_x.push(min_x);
                self.min_y.push(min_y);
                self.max_x.push(max_x);
                self.max_y.push(max_y);
            }
        }
    }

    /// `i`번 edge를 `[x1, y1, x2, y2]`로.
    #[inline]
    fn edge_of(&self, i: usize) -> [f64; 4] {
        [self.x1[i], self.y1[i], self.x2[i], self.y2[i]]
    }

    /// `i`번 edge의 bbox를 `[min_x, min_y, max_x, max_y]`로.
    #[inline]
    fn bbox_of(&self, i: usize) -> [f64; 4] {
        [self.min_x[i], self.min_y[i], self.max_x[i], self.max_y[i]]
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 스칼라 참조 커널 — SIMD 경로의 tail 처리와 non-simd 빌드가 함께 쓴다.
// ────────────────────────────────────────────────────────────────────────────

/// `from`번째부터 `soa[i].contains(x, y, eps)`인 인덱스를 오름차순으로 덧붙인다.
///
/// 비교식은 [`Aabb::contains`]와 글자 그대로 같다. 경계에 정확히 걸친 후보의
/// 판정이 바뀌면 안 되기 때문이다.
#[inline]
fn bbox_hits_scalar(soa: &AabbSoA, x: f64, y: f64, eps: f64, from: usize, out: &mut Vec<usize>) {
    for i in from..soa.len() {
        if x >= soa.min_x[i] - eps
            && x <= soa.max_x[i] + eps
            && y >= soa.min_y[i] - eps
            && y <= soa.max_y[i] + eps
        {
            out.push(i);
        }
    }
}

/// `from`번째부터 질의 bbox `q = [min_x, min_y, max_x, max_y]`와 겹치는 edge
/// 인덱스를 오름차순으로 덧붙인다. 비교식은 [`segment_bbox_overlaps`]와 같다.
#[inline]
fn edge_bbox_hits_scalar(
    soa: &EdgeSoA,
    q: [f64; 4],
    eps: f64,
    from: usize,
    out: &mut Vec<usize>,
) {
    for i in from..soa.len() {
        if q[0] <= soa.max_x[i] + eps
            && soa.min_x[i] <= q[2] + eps
            && q[1] <= soa.max_y[i] + eps
            && soa.min_y[i] <= q[3] + eps
        {
            out.push(i);
        }
    }
}

/// `approx_waste`의 frontier 부분(= y 띠가 겹치는 이웃의 `max_x` 최대값)을
/// `from`번째부터 이어서 계산한다. `max`만 쓰므로 누산 오차가 없다.
#[cfg(feature = "simd")]
#[inline]
fn frontier_scalar(soa: &AabbSoA, min_y: f64, max_y: f64, eps: f64, from: usize, init: f64) -> f64 {
    let mut frontier = init;
    for i in from..soa.len() {
        let lo = min_y.max(soa.min_y[i]);
        let hi = max_y.min(soa.max_y[i]);
        if hi - lo > eps {
            frontier = frontier.max(soa.max_x[i]);
        }
    }
    frontier
}

/// `approx_contact`의 이웃 항만 `from`번째부터 이어서 계산한다.
#[cfg(feature = "simd")]
#[inline]
fn contact_neighbours_scalar(soa: &AabbSoA, world: Aabb, from: usize) -> f64 {
    let mut contact = 0.0_f64;
    for i in from..soa.len() {
        let (b_min_x, b_min_y) = (soa.min_x[i], soa.min_y[i]);
        let (b_max_x, b_max_y) = (soa.max_x[i], soa.max_y[i]);

        let y_overlap = (world.max_y.min(b_max_y) - world.min_y.max(b_min_y)).max(0.0);
        if y_overlap > 0.0 {
            contact += y_overlap * proximity(axis_gap(b_min_x - world.max_x, world.min_x - b_max_x));
        }
        let x_overlap = (world.max_x.min(b_max_x) - world.min_x.max(b_min_x)).max(0.0);
        if x_overlap > 0.0 {
            contact += x_overlap * proximity(axis_gap(b_min_y - world.max_y, world.min_y - b_max_y));
        }
    }
    contact
}

// ────────────────────────────────────────────────────────────────────────────
// SIMD 커널 (feature = "simd")
//
// 폭 디스패치는 크레이트 관례를 그대로 따른다: avx512f → W=16, avx2 → W=8,
// 그 외 → W=4. 다만 여기 lane 타입은 `f64`다(mm 좌표 계산은 전부 f64이고,
// 경계 허용오차 판정을 스칼라와 **비트 단위로 동일**하게 유지해야 한다).
// 즉 W=16이면 f64 레지스터 2개 분량이 되며, portable_simd가 알아서 쪼갠다.
// ────────────────────────────────────────────────────────────────────────────

#[cfg(feature = "simd")]
type F64xN<const W: usize> = Simd<f64, W>;

// region: bbox 포함 판정 (후보 점 1개 vs bbox 전부)

/// 후보 점이 들어가는 bbox 인덱스를 **오름차순**으로 모은다.
#[cfg(feature = "simd")]
#[inline]
fn bbox_hits(soa: &AabbSoA, x: f64, y: f64, eps: f64, out: &mut Vec<usize>) {
    out.clear();
    #[cfg(target_arch = "x86_64")]
    {
        if std::arch::is_x86_feature_detected!("avx512f") {
            unsafe { bbox_hits_16(soa, x, y, eps, out) };
            return;
        }
        if std::arch::is_x86_feature_detected!("avx2") {
            unsafe { bbox_hits_8(soa, x, y, eps, out) };
            return;
        }
    }
    bbox_hits_4(soa, x, y, eps, out);
}

/// simd feature가 꺼지면 스칼라 그대로.
#[cfg(not(feature = "simd"))]
#[inline]
fn bbox_hits(soa: &AabbSoA, x: f64, y: f64, eps: f64, out: &mut Vec<usize>) {
    out.clear();
    bbox_hits_scalar(soa, x, y, eps, 0, out);
}

#[cfg(feature = "simd")]
#[inline(always)]
fn bbox_hits_kernel<const W: usize>(
    soa: &AabbSoA,
    x: f64,
    y: f64,
    eps: f64,
    out: &mut Vec<usize>,
) {
    use std::simd::prelude::SimdPartialOrd;

    let n = soa.len();
    let xv = F64xN::<W>::splat(x);
    let yv = F64xN::<W>::splat(y);
    let ev = F64xN::<W>::splat(eps);
    let chunks = n / W;

    for c in 0..chunks {
        let i = c * W;
        let min_x = F64xN::<W>::from_slice(&soa.min_x[i..i + W]);
        let max_x = F64xN::<W>::from_slice(&soa.max_x[i..i + W]);
        let min_y = F64xN::<W>::from_slice(&soa.min_y[i..i + W]);
        let max_y = F64xN::<W>::from_slice(&soa.max_y[i..i + W]);

        let mask = xv.simd_ge(min_x - ev)
            & xv.simd_le(max_x + ev)
            & yv.simd_ge(min_y - ev)
            & yv.simd_le(max_y + ev);

        let mut bits = mask.to_bitmask();
        while bits != 0 {
            let lane = bits.trailing_zeros() as usize;
            out.push(i + lane);
            bits &= bits - 1;
        }
    }
    bbox_hits_scalar(soa, x, y, eps, chunks * W, out);
}

#[cfg(feature = "simd")]
fn bbox_hits_4(soa: &AabbSoA, x: f64, y: f64, eps: f64, out: &mut Vec<usize>) {
    bbox_hits_kernel::<4>(soa, x, y, eps, out)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn bbox_hits_8(soa: &AabbSoA, x: f64, y: f64, eps: f64, out: &mut Vec<usize>) {
    bbox_hits_kernel::<8>(soa, x, y, eps, out)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx512f")]
unsafe fn bbox_hits_16(soa: &AabbSoA, x: f64, y: f64, eps: f64, out: &mut Vec<usize>) {
    bbox_hits_kernel::<16>(soa, x, y, eps, out)
}

// endregion

// region: edge bbox 사전 필터 (edge 1개 vs edge 전부)

/// 질의 edge bbox와 겹치는 edge 인덱스를 **오름차순**으로 모은다.
#[cfg(feature = "simd")]
#[inline]
fn edge_bbox_hits(soa: &EdgeSoA, q: [f64; 4], eps: f64, out: &mut Vec<usize>) {
    out.clear();
    #[cfg(target_arch = "x86_64")]
    {
        if std::arch::is_x86_feature_detected!("avx512f") {
            unsafe { edge_bbox_hits_16(soa, q, eps, out) };
            return;
        }
        if std::arch::is_x86_feature_detected!("avx2") {
            unsafe { edge_bbox_hits_8(soa, q, eps, out) };
            return;
        }
    }
    edge_bbox_hits_4(soa, q, eps, out);
}

#[cfg(not(feature = "simd"))]
#[inline]
fn edge_bbox_hits(soa: &EdgeSoA, q: [f64; 4], eps: f64, out: &mut Vec<usize>) {
    out.clear();
    edge_bbox_hits_scalar(soa, q, eps, 0, out);
}

#[cfg(feature = "simd")]
#[inline(always)]
fn edge_bbox_hits_kernel<const W: usize>(
    soa: &EdgeSoA,
    q: [f64; 4],
    eps: f64,
    out: &mut Vec<usize>,
) {
    use std::simd::prelude::SimdPartialOrd;

    let n = soa.len();
    let q_min_x = F64xN::<W>::splat(q[0]);
    let q_min_y = F64xN::<W>::splat(q[1]);
    let q_max_x = F64xN::<W>::splat(q[2]);
    let q_max_y = F64xN::<W>::splat(q[3]);
    let ev = F64xN::<W>::splat(eps);
    let chunks = n / W;

    for c in 0..chunks {
        let i = c * W;
        let min_x = F64xN::<W>::from_slice(&soa.min_x[i..i + W]);
        let max_x = F64xN::<W>::from_slice(&soa.max_x[i..i + W]);
        let min_y = F64xN::<W>::from_slice(&soa.min_y[i..i + W]);
        let max_y = F64xN::<W>::from_slice(&soa.max_y[i..i + W]);

        let mask = q_min_x.simd_le(max_x + ev)
            & min_x.simd_le(q_max_x + ev)
            & q_min_y.simd_le(max_y + ev)
            & min_y.simd_le(q_max_y + ev);

        let mut bits = mask.to_bitmask();
        while bits != 0 {
            let lane = bits.trailing_zeros() as usize;
            out.push(i + lane);
            bits &= bits - 1;
        }
    }
    edge_bbox_hits_scalar(soa, q, eps, chunks * W, out);
}

#[cfg(feature = "simd")]
fn edge_bbox_hits_4(soa: &EdgeSoA, q: [f64; 4], eps: f64, out: &mut Vec<usize>) {
    edge_bbox_hits_kernel::<4>(soa, q, eps, out)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn edge_bbox_hits_8(soa: &EdgeSoA, q: [f64; 4], eps: f64, out: &mut Vec<usize>) {
    edge_bbox_hits_kernel::<8>(soa, q, eps, out)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx512f")]
unsafe fn edge_bbox_hits_16(soa: &EdgeSoA, q: [f64; 4], eps: f64, out: &mut Vec<usize>) {
    edge_bbox_hits_kernel::<16>(soa, q, eps, out)
}

// endregion

// region: approx_waste / approx_contact 의 이웃 bbox 순회

/// simd 빌드 — SoA 위에서 벡터화한다.
#[cfg(feature = "simd")]
#[inline]
fn waste_of(world: Aabb, item_area: f64, placed: &[Aabb], soa: &AabbSoA) -> f64 {
    debug_assert_eq!(placed.len(), soa.len());
    let _ = placed;
    let bbox_slack = (world.area() - item_area).max(0.0);
    let frontier = frontier_dispatch(soa, world.min_y, world.max_y, COORD_TIE_EPS as f64);
    let gap = (world.min_x - frontier).max(0.0);
    bbox_slack + gap * world.height().max(0.0)
}

/// non-simd 빌드 — 기존 스칼라 함수를 그대로 호출한다.
#[cfg(not(feature = "simd"))]
#[inline]
fn waste_of(world: Aabb, item_area: f64, placed: &[Aabb], soa: &AabbSoA) -> f64 {
    let _ = soa;
    approx_waste(world, item_area, placed)
}

/// simd 빌드 — 이웃 항만 벡터화하고 strip 경계 항은 스칼라 그대로 더한다.
#[cfg(feature = "simd")]
#[inline]
fn contact_of(world: Aabb, placed: &[Aabb], soa: &AabbSoA, strip_height: f64) -> f64 {
    debug_assert_eq!(placed.len(), soa.len());
    let _ = placed;
    let mut contact = contact_dispatch(soa, world);
    contact += world.width().max(0.0) * proximity(world.min_y.abs());
    contact += world.width().max(0.0) * proximity((strip_height - world.max_y).abs());
    contact += world.height().max(0.0) * proximity(world.min_x.abs());
    contact
}

/// non-simd 빌드 — 기존 스칼라 함수를 그대로 호출한다.
#[cfg(not(feature = "simd"))]
#[inline]
fn contact_of(world: Aabb, placed: &[Aabb], soa: &AabbSoA, strip_height: f64) -> f64 {
    let _ = soa;
    approx_contact(world, placed, strip_height)
}

#[cfg(feature = "simd")]
#[inline]
fn frontier_dispatch(soa: &AabbSoA, min_y: f64, max_y: f64, eps: f64) -> f64 {
    #[cfg(target_arch = "x86_64")]
    {
        if std::arch::is_x86_feature_detected!("avx512f") {
            return unsafe { frontier_16(soa, min_y, max_y, eps) };
        }
        if std::arch::is_x86_feature_detected!("avx2") {
            return unsafe { frontier_8(soa, min_y, max_y, eps) };
        }
    }
    frontier_4(soa, min_y, max_y, eps)
}

#[cfg(feature = "simd")]
#[inline(always)]
fn frontier_kernel<const W: usize>(soa: &AabbSoA, min_y: f64, max_y: f64, eps: f64) -> f64 {
    use std::simd::prelude::{SimdFloat, SimdPartialOrd};

    let lo_v = F64xN::<W>::splat(min_y);
    let hi_v = F64xN::<W>::splat(max_y);
    let ev = F64xN::<W>::splat(eps);
    let zero = F64xN::<W>::splat(0.0);
    // 스칼라와 마찬가지로 frontier의 시작값은 0.0이다(= strip 좌단).
    let mut acc = zero;
    let chunks = soa.len() / W;

    for c in 0..chunks {
        let i = c * W;
        let b_min_y = F64xN::<W>::from_slice(&soa.min_y[i..i + W]);
        let b_max_y = F64xN::<W>::from_slice(&soa.max_y[i..i + W]);
        let b_max_x = F64xN::<W>::from_slice(&soa.max_x[i..i + W]);

        let lo = lo_v.simd_max(b_min_y);
        let hi = hi_v.simd_min(b_max_y);
        // y 띠가 겹치지 않는 lane은 0.0으로 눌러 두면 최대값에 영향이 없다.
        acc = acc.simd_max((hi - lo).simd_gt(ev).select(b_max_x, zero));
    }
    // `max` 축약은 순서에 무관하므로 여기까지는 스칼라와 **정확히 같은 값**이다.
    frontier_scalar(soa, min_y, max_y, eps, chunks * W, acc.reduce_max())
}

#[cfg(feature = "simd")]
fn frontier_4(soa: &AabbSoA, min_y: f64, max_y: f64, eps: f64) -> f64 {
    frontier_kernel::<4>(soa, min_y, max_y, eps)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn frontier_8(soa: &AabbSoA, min_y: f64, max_y: f64, eps: f64) -> f64 {
    frontier_kernel::<8>(soa, min_y, max_y, eps)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx512f")]
unsafe fn frontier_16(soa: &AabbSoA, min_y: f64, max_y: f64, eps: f64) -> f64 {
    frontier_kernel::<16>(soa, min_y, max_y, eps)
}

#[cfg(feature = "simd")]
#[inline]
fn contact_dispatch(soa: &AabbSoA, world: Aabb) -> f64 {
    #[cfg(target_arch = "x86_64")]
    {
        if std::arch::is_x86_feature_detected!("avx512f") {
            return unsafe { contact_16(soa, world) };
        }
        if std::arch::is_x86_feature_detected!("avx2") {
            return unsafe { contact_8(soa, world) };
        }
    }
    contact_4(soa, world)
}

/// [`proximity`]의 벡터판. `gap >= CONTACT_BAND_MM`이면 0, 아니면 선형 감쇠.
#[cfg(feature = "simd")]
#[inline(always)]
fn proximity_simd<const W: usize>(gap: F64xN<W>) -> F64xN<W> {
    use std::simd::prelude::SimdPartialOrd;

    let band = F64xN::<W>::splat(CONTACT_BAND_MM);
    let one = F64xN::<W>::splat(1.0);
    let zero = F64xN::<W>::splat(0.0);
    gap.simd_lt(band).select(one - gap / band, zero)
}

#[cfg(feature = "simd")]
#[inline(always)]
fn contact_kernel<const W: usize>(soa: &AabbSoA, world: Aabb) -> f64 {
    use std::simd::prelude::SimdFloat;

    let zero = F64xN::<W>::splat(0.0);
    let w_min_x = F64xN::<W>::splat(world.min_x);
    let w_min_y = F64xN::<W>::splat(world.min_y);
    let w_max_x = F64xN::<W>::splat(world.max_x);
    let w_max_y = F64xN::<W>::splat(world.max_y);

    let mut acc = zero;
    let chunks = soa.len() / W;

    for c in 0..chunks {
        let i = c * W;
        let b_min_x = F64xN::<W>::from_slice(&soa.min_x[i..i + W]);
        let b_min_y = F64xN::<W>::from_slice(&soa.min_y[i..i + W]);
        let b_max_x = F64xN::<W>::from_slice(&soa.max_x[i..i + W]);
        let b_max_y = F64xN::<W>::from_slice(&soa.max_y[i..i + W]);

        // 겹침 길이가 0이면 그 항은 곱해서 0이 되므로 스칼라의 `if` 분기와 같다.
        let y_overlap = (w_max_y.simd_min(b_max_y) - w_min_y.simd_max(b_min_y)).simd_max(zero);
        let x_overlap = (w_max_x.simd_min(b_max_x) - w_min_x.simd_max(b_min_x)).simd_max(zero);

        // 스칼라의 `axis_gap` 과 **비트 단위로 같은 식**이어야 한다.
        // `simd_max` 는 스칼라 `f64::max` 와 같은 결과를 준다(NaN 취급 포함).
        let gap_x = (b_min_x - w_max_x).simd_max(w_min_x - b_max_x).simd_max(zero);
        let gap_y = (b_min_y - w_max_y).simd_max(w_min_y - b_max_y).simd_max(zero);
        acc += y_overlap * proximity_simd(gap_x);
        acc += x_overlap * proximity_simd(gap_y);
    }

    acc.reduce_sum() + contact_neighbours_scalar(soa, world, chunks * W)
}

#[cfg(feature = "simd")]
fn contact_4(soa: &AabbSoA, world: Aabb) -> f64 {
    contact_kernel::<4>(soa, world)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx2")]
unsafe fn contact_8(soa: &AabbSoA, world: Aabb) -> f64 {
    contact_kernel::<8>(soa, world)
}

#[cfg(all(feature = "simd", target_arch = "x86_64"))]
#[target_feature(enable = "avx512f")]
unsafe fn contact_16(soa: &AabbSoA, world: Aabb) -> f64 {
    contact_kernel::<16>(soa, world)
}

// endregion

// ────────────────────────────────────────────────────────────────────────────
// 단위 테스트 — 외부 데이터 없이 순수 계산만 검증한다.
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// 겹침을 켰을 때 importer 가 컨테이너를 줄인다. 구성형 배치기는 원단 치수를
    /// 줄어들지 않은 값에서 가져오므로, 그 인셋을 모르면 첫 조각을 컨테이너 밖에
    /// 놓고 해 전체가 `exterior_collisions` 로 기각된다 — 겹침 0.8mm 에서 NFP
    /// 구성형 경로가 통째로 죽던 원인이다.
    fn offset_strip_instance(separation: f32) -> jagua_rs::probs::spp::entities::SPInstance {
        let ext = serde_json::from_value(serde_json::json!({
            "name": "nfp-physical-strip",
            "strip_height": 10.0,
            "items": [{
                "id": 0, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0.0, "y_min": 0.0, "width": 10.0, "height": 4.0}}
            }]
        }))
        .unwrap();
        crate::util::io::import_spp_instance(
            &ext,
            crate::config::DEFAULT_SWARM_CONFIG.cde_config,
            None,
            Some(separation),
            None,
        )
        .unwrap()
    }

    #[test]
    fn nfp_candidates_respect_the_imported_strip_inset() {
        // 음수 = 겹침 허용, 양수 = 간격 확보. 둘 다 컨테이너를 바꾼다.
        for separation in [-0.8, 0.8] {
            let instance = offset_strip_instance(separation);
            let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone()).expect(crate::dmsg!("strip forms a valid container"));
            prob.change_strip_width(100.0).expect(crate::dmsg!("strip width must be positive"));
            let bounds = prob.layout().container().outer_cd().bbox();
            let lib = crate::nfp_lbf::cache::PreparedLibrary::build(&instance, &[0], 0.0).unwrap();
            let hash = lib.hash_of_item(0).unwrap();
            let mut cache = NfpLbfCache::new(lib);
            // 세 레거시 캐시를 먼저 채운다. 인셋 인지 질의가 이 origin-local 항목을
            // 재사용하면 샘플러와 배치기가 같은 슬롯에 다른 기하를 덮어쓰게 된다.
            let legacy = cache.inner_fit_view_prepared(hash, 0, 100.0, 10.0).unwrap();
            let mut placer =
                SequentialPlacer::new(&mut cache, 10.0, 100.0, PlacementScoreWeights::default());
            let candidates = placer.candidates_for(0, 0, &[]);
            assert!(!candidates.is_empty());
            for p in candidates {
                let world = placer.world_bounds(&[PlacedPiece {
                    item_id: 0,
                    rotation: 0,
                    translation: p.translation,
                }])[0];
                assert!(
                    world.min_x >= bounds.x_min as f64 - 1e-4
                        && world.max_x <= bounds.x_max as f64 + 1e-4
                        && world.min_y >= bounds.y_min as f64 - 1e-4
                        && world.max_y <= bounds.y_max as f64 + 1e-4,
                    "separation {separation}: candidate {world:?}, container {bounds:?}"
                );
            }
            drop(placer);
            let again = cache.inner_fit_view_prepared(hash, 0, 100.0, 10.0).unwrap();
            assert!(std::sync::Arc::ptr_eq(&legacy.0, &again.0));
            assert!(std::sync::Arc::ptr_eq(&legacy.1, &again.1));
            assert!(std::sync::Arc::ptr_eq(
                legacy.2.as_ref().unwrap(),
                again.2.as_ref().unwrap()
            ));
        }
    }

    #[test]
    fn nfp_frontier_fallback_respects_the_imported_strip_inset() {
        let instance = offset_strip_instance(-0.8);
        let lib = crate::nfp_lbf::cache::PreparedLibrary::build(&instance, &[0], 0.0).unwrap();
        let mut cache = NfpLbfCache::new(lib);
        let mut placer =
            SequentialPlacer::new(&mut cache, 10.0, 100.0, PlacementScoreWeights::default());
        let p = placer.frontier_fallback(0, &[0], 0.0).unwrap();
        let b = placer.world_bounds(&[p])[0];
        assert!(b.min_x >= 0.3999 && b.min_y >= 0.3999 && b.max_y <= 9.6001, "{b:?}");
    }

    fn aabb(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Aabb {
        Aabb {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    #[test]
    fn limit_constants_match_the_document() {
        assert_eq!(MAX_EDGE_INTERSECTIONS, 4096);
        assert_eq!(MAX_EVALUATED_CANDIDATES, 256);
    }

    /// x 오름차순 후보 목록. `pocket_left_end`는 `.0`만 본다.
    fn cands(xs: &[f64]) -> Vec<(f64, f64, CandidateSource, u32)> {
        xs.iter()
            .map(|&x| (x, 0.0, CandidateSource::NfpVertex, NO_GENERATOR))
            .collect()
    }

    #[test]
    fn pocket_boundary_keeps_only_anchors_whose_piece_ends_inside_the_frontier() {
        // 조각 우측 끝이 `x + 100`. 프론티어 1000 -> 경계 900.
        let c = cands(&[0.0, 500.0, 899.9, 900.0, 950.0, 1200.0]);
        // 900.0은 놓으면 우측 끝이 정확히 1000 = 프론티어라 밀지는 않지만,
        // `partition_point(< boundary)`가 배제한다. 경계 한 점을 어느 쪽으로 넣든
        // 구간 3이 다시 훑으므로 후보를 잃지 않는다 — 여기서는 보수적으로 배제한다.
        assert_eq!(pocket_left_end(&c, 1000.0, 100.0), 3);
    }

    #[test]
    fn pocket_boundary_is_empty_before_anything_is_placed() {
        let c = cands(&[0.0, 10.0]);
        // 프론티어 0 = 배치 전. 포켓이라는 개념 자체가 없다.
        assert_eq!(pocket_left_end(&c, 0.0, 100.0), 0);
        assert_eq!(pocket_left_end(&c, f64::NAN, 100.0), 0);
        assert_eq!(pocket_left_end(&c, 1000.0, f64::NAN), 0);
    }

    /// ④를 통째로 no-op으로 만들었던 그 실수의 회귀 방지.
    ///
    /// `ifp_bbox.width()`는 `strip_width_cap - 조각폭`이고 wsjk156에서
    /// `strip_width_cap = Σ 조각폭 = 51141.09 mm`다. 프론티어는 아무리 커도
    /// LBF 단독 폭 7325.05 mm를 넘지 않으므로 경계가 **항상** 음수가 된다.
    #[test]
    fn ifp_width_as_the_boundary_would_kill_the_whole_pocket_region() {
        let c = cands(&[0.0, 100.0, 3000.0, 7000.0]);
        let frontier = 7325.05_f64; // 이 인스턴스에서 가능한 최대 프론티어
        let anchor_max_x = 352.75_f64; // 조각의 앵커 기준 우측 끝
        let ifp_width = 51141.09 - anchor_max_x; // 옛 코드가 뺐던 값

        assert_eq!(pocket_left_end(&c, frontier, ifp_width), 0, "옛 식은 구간이 빈다");
        assert!(
            pocket_left_end(&c, frontier, anchor_max_x) > 0,
            "교정된 식은 포켓 구간을 실제로 만든다"
        );
    }

    /// 포켓 패스가 프론티어에 **가까운 쪽부터** 훑는지 — 인덱스 생성 규칙만 본다.
    ///
    /// 앞에서부터 훑으면 x=0 근처, 즉 가장 먼저 채워져 자리가 남을 리 없는 곳을
    /// 먼저 태운다. 그게 구간 분리를 도입해 없앤 병리다.
    #[test]
    fn pocket_scan_walks_from_the_frontier_backwards() {
        let left_end = 4usize;
        let order: Vec<usize> = (0..left_end)
            .filter_map(|k| left_end.checked_sub(k + 1))
            .collect();
        assert_eq!(order, vec![3, 2, 1, 0]);
        // `left_end == 0`이면 한 번도 돌지 않는다(언더플로 없음).
        assert!((0..0usize).filter_map(|k| 0usize.checked_sub(k + 1)).next().is_none());
    }

    #[test]
    fn segment_intersection_finds_the_crossing() {
        let a = [0.0, 0.0, 1.0, 1.0];
        let b = [0.0, 1.0, 1.0, 0.0];
        let (x, y) = segment_intersection(&a, &b).expect(crate::dmsg!("두 대각선은 교차한다"));
        assert!((x - 0.5).abs() < 1e-12);
        assert!((y - 0.5).abs() < 1e-12);
    }

    #[test]
    fn parallel_segments_have_no_intersection() {
        let a = [0.0, 0.0, 1.0, 0.0];
        let b = [0.0, 1.0, 1.0, 1.0];
        assert!(segment_intersection(&a, &b).is_none());
    }

    #[test]
    fn intersection_outside_the_span_is_rejected() {
        // 직선으로는 (2, 0)에서 만나지만 두 선분 구간 밖이다.
        let a = [0.0, 0.0, 1.0, 0.0];
        let b = [2.0, 1.0, 2.0, 2.0];
        assert!(segment_intersection(&a, &b).is_none());
    }

    #[test]
    fn segment_bbox_prefilter_works() {
        let a = [0.0, 0.0, 1.0, 1.0];
        let far = [10.0, 10.0, 11.0, 11.0];
        let near = [0.5, 0.0, 0.5, 2.0];
        assert!(!segment_bbox_overlaps(&a, &far, 0.0));
        assert!(segment_bbox_overlaps(&a, &near, 0.0));
    }

    #[test]
    fn aabb_overlap_includes_touching() {
        assert!(aabb_overlaps(aabb(0.0, 0.0, 2.0, 2.0), aabb(1.0, 1.0, 3.0, 3.0), 0.0));
        assert!(!aabb_overlaps(aabb(0.0, 0.0, 1.0, 1.0), aabb(2.0, 2.0, 3.0, 3.0), 0.0));
        // 접하는 경우는 겹침으로 본다(접촉 배치를 살려야 한다).
        assert!(aabb_overlaps(aabb(0.0, 0.0, 1.0, 1.0), aabb(1.0, 0.0, 2.0, 1.0), 0.0));
    }

    #[test]
    fn grid_key_quantizes_to_micron_grid() {
        assert_eq!(grid_key(1.0, 2.0), (1_000, 2_000));
        // 0.0004mm 차이는 같은 격자 칸이다.
        assert_eq!(grid_key(1.0, 2.0), grid_key(1.0004, 2.0));
        // 0.001mm 차이는 다른 칸이다.
        assert_ne!(grid_key(1.0, 2.0), grid_key(1.001, 2.0));
    }

    #[test]
    fn rectangular_piece_has_no_bbox_slack() {
        // 10 × 5 사각형을 원점에 두면 bbox 면적 = 실제 면적, frontier 간극도 0.
        let w = approx_waste(aabb(0.0, 0.0, 10.0, 5.0), 50.0, &[]);
        assert!(w.abs() < 1e-9, "waste = {w}");
    }

    #[test]
    fn left_dead_zone_is_penalised() {
        // frontier가 없으므로 x = 0 기준, x_min = 4 → 4 × 5 = 20mm² 벌점.
        let w = approx_waste(aabb(4.0, 0.0, 14.0, 5.0), 50.0, &[]);
        assert!((w - 20.0).abs() < 1e-9, "waste = {w}");
    }

    #[test]
    fn frontier_shortens_the_dead_zone() {
        let placed = [aabb(0.0, 0.0, 4.0, 5.0)];
        // frontier = 4, x_min = 6 → 2 × 5 = 10mm²
        let w = approx_waste(aabb(6.0, 0.0, 16.0, 5.0), 50.0, &placed);
        assert!((w - 10.0).abs() < 1e-9, "waste = {w}");
    }

    #[test]
    fn disjoint_y_band_does_not_reach_the_frontier() {
        let placed = [aabb(0.0, 100.0, 40.0, 105.0)];
        // y 띠가 전혀 겹치지 않으므로 frontier = 0 → 6 × 5 = 30mm²
        let w = approx_waste(aabb(6.0, 0.0, 16.0, 5.0), 50.0, &placed);
        assert!((w - 30.0).abs() < 1e-9, "waste = {w}");
    }

    #[test]
    fn floor_contact_adds_the_full_width() {
        let floor = approx_contact(aabb(0.0, 0.0, 10.0, 5.0), &[], 100.0);
        let lifted = approx_contact(aabb(0.0, 50.0, 10.0, 55.0), &[], 100.0);
        // 바닥 접촉 10mm + 좌단 접촉 5mm = 15mm
        assert!((floor - 15.0).abs() < 1e-9, "floor = {floor}");
        // 떠 있는 쪽은 좌단 접촉 5mm만 남는다.
        assert!((lifted - 5.0).abs() < 1e-9, "lifted = {lifted}");
        assert!(floor > lifted);
    }

    #[test]
    fn neighbour_contact_is_counted() {
        let placed = [aabb(0.0, 0.0, 10.0, 5.0)];
        // x = 10에 맞붙이면 y 겹침 5mm가 그대로 접촉이 된다(+ 바닥 10mm).
        let touching = approx_contact(aabb(10.0, 0.0, 20.0, 5.0), &placed, 100.0);
        let apart = approx_contact(aabb(30.0, 0.0, 40.0, 5.0), &placed, 100.0);
        assert!((touching - 15.0).abs() < 1e-9, "touching = {touching}");
        assert!((apart - 10.0).abs() < 1e-9, "apart = {apart}");
    }

    #[test]
    fn proximity_decays_to_zero_at_band_width() {
        assert!((proximity(0.0) - 1.0).abs() < 1e-12);
        assert!((proximity(CONTACT_BAND_MM / 2.0) - 0.5).abs() < 1e-12);
        assert!(proximity(CONTACT_BAND_MM).abs() < 1e-12);
        assert!(proximity(CONTACT_BAND_MM * 10.0).abs() < 1e-12);
    }

    #[test]
    fn normalisation_is_a_positive_scaling() {
        let scales = Scales {
            length: 2000.0,
            height: 1500.0,
            area: 500.0,
            perimeter: 120.0,
        };
        let a = ScoreTerms {
            strip_length: 100.0,
            waste: 10.0,
            contact: 30.0,
            x: 5.0,
            y: 7.0,
        };
        let b = ScoreTerms {
            strip_length: 200.0,
            waste: 10.0,
            contact: 30.0,
            x: 5.0,
            y: 7.0,
        };
        let na = normalize_terms(a, scales);
        let nb = normalize_terms(b, scales);
        assert!(na.strip_length < nb.strip_length);
        assert!((na.strip_length - 100.0 / 2000.0).abs() < 1e-9);
        assert!((na.waste - 10.0 / 500.0).abs() < 1e-9);
        assert!((na.contact - 30.0 / 120.0).abs() < 1e-9);
        assert!((na.y - 7.0 / 1500.0).abs() < 1e-9);
    }

    #[test]
    fn grid_translation_matches_the_kernel_rounding() {
        // 격자에 정확히 올라간 평행이동은 언제나 "애매하지 않다".
        for mm in [0.0, 1.0, -2.5, 137.456, -9_876.543] {
            let (q, exact) = grid_translation(mm);
            assert!(exact, "{mm} 은 격자 tie가 아니다");
            assert_eq!(q, (mm * GRID_SCALE as f64).round() as i64);
        }
        // 소수부가 정확히 0.5인 값은 tie로 걸러져 옛 경로(월드 사본)로 넘어간다.
        // 0.0625 = 1/16 -> 62.5 격자 단위.
        let (_, exact) = grid_translation(0.0625);
        assert!(!exact, "GRID·t 소수부가 0.5면 반올림이 부동소수 오차에 걸린다");
        // 비유한 값도 폴백으로 넘긴다.
        assert!(!grid_translation(f64::NAN).1);
        assert!(!grid_translation(f64::INFINITY).1);
    }

    #[test]
    fn local_query_equals_the_translated_query_on_the_grid() {
        // `Obstacle` 문서의 등가식을 직접 확인한다.
        //
        //   round(GRID·p) - round(GRID·t)  ==  round(GRID·(v + t)) - ... 를 거치지 않고,
        //   "월드 정점을 양자화한 값"에서 "평행이동을 양자화한 값"을 뺀 것이
        //   "로컬 정점을 양자화한 값"과 같은지 본다.
        let engine = NfpEngine::new();
        for &t in &[0.0_f64, 12.3456, -987.6543, 1_234.5] {
            let (tq, exact) = grid_translation(t);
            assert!(exact, "테스트 평행이동은 tie가 아니어야 한다");
            for &v in &[0.0_f64, 0.001, -0.003, 42.117, -1_003.999] {
                // 커널 출력 정점은 항상 0.001mm 격자 위의 값이다.
                let world = v + t;
                let pq = grid_round(world).expect(crate::dmsg!("격자에 올라간다"));
                let local = (pq - tq) as f64 / GRID_SCALE as f64;
                // 로컬 좌표로 되돌린 값이 원래 로컬 정점과 같은 격자 칸이어야 한다.
                assert_eq!(grid_round(local), grid_round(v), "v={v} t={t}");
                // 커널의 양자화도 같은 정수로 떨어져야 한다.
                let probe = PolygonSet::from_outer(NfpRing::new(vec![
                    NfpPoint::new(local - 1.0, local - 1.0),
                    NfpPoint::new(local + 1.0, local - 1.0),
                    NfpPoint::new(local + 1.0, local + 1.0),
                ]));
                assert!(engine.locate(&probe, NfpPoint::new(local, local)).is_ok());
            }
        }
    }

    #[test]
    fn translation_preserves_holes() {
        let outer = NfpRing::new(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]);
        let hole = NfpRing::new(vec![(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]);
        let set = PolygonSet::new(vec![NfpPolygon::with_holes(outer, vec![hole])]);
        let moved = translate_set(&set, 5.0, -3.0);

        assert_eq!(moved.components.len(), 1);
        assert_eq!(moved.components[0].holes.len(), 1);
        assert_eq!(moved.ring_count(), 2);

        let b = set_bounds(&moved).expect(crate::dmsg!("정점이 있다"));
        assert!((b.min_x - 5.0).abs() < 1e-12);
        assert!((b.min_y + 3.0).abs() < 1e-12);
        assert!((b.max_x - 15.0).abs() < 1e-12);
        assert!((b.max_y - 7.0).abs() < 1e-12);
    }

    #[test]
    fn set_edges_yields_one_edge_per_vertex() {
        let outer = NfpRing::new(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]);
        let hole = NfpRing::new(vec![(2.0, 2.0), (4.0, 2.0), (4.0, 4.0)]);
        let set = PolygonSet::new(vec![NfpPolygon::with_holes(outer, vec![hole])]);
        assert_eq!(set_edges(&set).len(), 4 + 3);
    }

    #[test]
    fn empty_set_has_no_bounds() {
        assert!(set_bounds(&PolygonSet::new(Vec::new())).is_none());
    }

    #[test]
    fn no_prefix_places_everything_fresh() {
        let (placed, unplaced, start) = reuse_prefix(&[1, 2, 3], None, 2);
        assert!(placed.is_empty());
        assert!(unplaced.is_empty());
        assert_eq!(start, 0);
    }

    #[test]
    fn zero_prefix_len_reuses_nothing() {
        let pfx = SequentialPlacement {
            pieces: vec![PlacedPiece {
                item_id: 1,
                rotation: 0,
                translation: (0.0, 0.0),
            }],
            strip_length: 10.0,
            placed: 1,
            unplaced: Vec::new(),
        };
        let (placed, unplaced, start) = reuse_prefix(&[1, 2, 3], Some(&pfx), 0);
        assert!(placed.is_empty());
        assert!(unplaced.is_empty());
        assert_eq!(start, 0);
    }

    #[test]
    fn prefix_reuse_skips_unplaced_items() {
        // sequence = [1, 2, 3, 4], 앞 3개 중 2번은 배치에 실패했던 상황.
        let pfx = SequentialPlacement {
            pieces: vec![
                PlacedPiece {
                    item_id: 1,
                    rotation: 0,
                    translation: (0.0, 0.0),
                },
                PlacedPiece {
                    item_id: 3,
                    rotation: 180,
                    translation: (12.0, 0.0),
                },
                PlacedPiece {
                    item_id: 4,
                    rotation: 0,
                    translation: (24.0, 0.0),
                },
            ],
            strip_length: 34.0,
            placed: 3,
            unplaced: vec![2],
        };
        let (placed, unplaced, start) = reuse_prefix(&[1, 2, 3, 4], Some(&pfx), 3);
        assert_eq!(placed.len(), 2);
        assert_eq!(placed[0].item_id, 1);
        assert_eq!(placed[1].item_id, 3);
        assert_eq!(placed[1].rotation, 180);
        assert_eq!(unplaced, vec![2]);
        assert_eq!(start, 3);
    }

    #[test]
    fn prefix_reuse_stops_at_the_first_unknown_item() {
        // 접두는 [1, 3]을 배치했고 unplaced는 없다. 새 순서 [1, 9, 3, 4]의 9는 접두가
        // 모르는 아이템이므로 재사용은 1에서 끝나고 9부터 정상 배치해야 한다.
        let pfx = SequentialPlacement {
            pieces: vec![
                PlacedPiece {
                    item_id: 1,
                    rotation: 0,
                    translation: (0.0, 0.0),
                },
                PlacedPiece {
                    item_id: 3,
                    rotation: 0,
                    translation: (12.0, 0.0),
                },
            ],
            strip_length: 24.0,
            placed: 2,
            unplaced: Vec::new(),
        };
        let (placed, unplaced, start) = reuse_prefix(&[1, 9, 3, 4], Some(&pfx), 3);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0].item_id, 1);
        assert!(unplaced.is_empty());
        assert_eq!(start, 1);
    }

    #[test]
    fn prefix_len_is_clamped_to_the_sequence() {
        let pfx = SequentialPlacement {
            pieces: vec![PlacedPiece {
                item_id: 7,
                rotation: 0,
                translation: (1.0, 2.0),
            }],
            strip_length: 5.0,
            placed: 1,
            unplaced: Vec::new(),
        };
        let (placed, unplaced, start) = reuse_prefix(&[7], Some(&pfx), 99);
        assert_eq!(placed.len(), 1);
        assert!(unplaced.is_empty());
        assert_eq!(start, 1);
    }

    #[test]
    fn candidate_sources_are_distinct() {
        assert_ne!(CandidateSource::NfpVertex, CandidateSource::NfpEdgeIntersection);
        assert_ne!(
            CandidateSource::NfpEdgeIntersection,
            CandidateSource::StripBoundaryIntersection
        );
        let c = PlacementCandidate {
            translation: (1.5, 2.5),
            source: CandidateSource::NfpVertex,
            touch_count: 3,
        };
        assert_eq!(c.source, CandidateSource::NfpVertex);
        // 접촉 수는 feasible 판정이 이미 계산하던 `Location::Boundary` 를 센 값이다.
        assert_eq!(c.touch_count, 3);
    }

    #[test]
    fn default_placement_is_empty() {
        let sp = SequentialPlacement::default();
        assert_eq!(sp.placed, 0);
        assert!(sp.pieces.is_empty());
        assert!(sp.unplaced.is_empty());
        assert_eq!(sp.strip_length, 0.0);
    }

    #[test]
    fn edge_soa_matches_the_aos_reference_order() {
        let outer = NfpRing::new(vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]);
        let hole = NfpRing::new(vec![(2.0, 2.0), (4.0, 2.0), (4.0, 4.0)]);
        let set = PolygonSet::new(vec![NfpPolygon::with_holes(outer, vec![hole])]);

        let aos = set_edges(&set);
        let mut soa = EdgeSoA::default();
        soa.load_from_set(&set);

        assert_eq!(soa.len(), aos.len());
        for (i, e) in aos.iter().enumerate() {
            assert_eq!(soa.edge_of(i), *e, "edge {i}");
            let (min_x, max_x) = min_max(e[0], e[2]);
            let (min_y, max_y) = min_max(e[1], e[3]);
            assert_eq!(soa.bbox_of(i), [min_x, min_y, max_x, max_y], "bbox {i}");
        }
    }

    /// 버퍼 재사용이 이전 아이템의 행을 흘리거나 매번 재할당하지 않는지.
    #[test]
    fn soa_buffers_are_reused_without_stale_rows() {
        let big: Vec<Aabb> = (0..64).map(|k| aabb(k as f64, 0.0, k as f64 + 1.0, 1.0)).collect();
        let mut soa = AabbSoA::default();
        soa.load_boxes(&big);
        let cap = soa.min_x.capacity();

        soa.load_boxes(&[aabb(5.0, 5.0, 6.0, 6.0)]);
        assert_eq!(soa.len(), 1);
        assert_eq!(soa.min_x[0], 5.0);
        assert_eq!(soa.min_x.capacity(), cap, "재할당 없이 용량을 재사용해야 한다");
    }

    /// non-simd 빌드에서도 `bbox_hits` / `edge_bbox_hits`가 스칼라 술어와 일치한다.
    #[test]
    fn hit_collectors_match_the_scalar_predicates() {
        let eps = COORD_TIE_EPS as f64;
        let boxes = [
            aabb(0.0, 0.0, 10.0, 10.0),
            aabb(20.0, 0.0, 30.0, 10.0),
            aabb(-5.0, -5.0, 5.0, 5.0),
        ];
        let mut soa = AabbSoA::default();
        soa.load_boxes(&boxes);

        let mut hits = Vec::new();
        for &(x, y) in &[(0.0, 0.0), (25.0, 5.0), (10.0, 10.0), (100.0, 100.0)] {
            bbox_hits(&soa, x, y, eps, &mut hits);
            let want: Vec<usize> = (0..boxes.len())
                .filter(|&i| boxes[i].contains(x, y, eps))
                .collect();
            assert_eq!(hits, want, "p=({x}, {y})");
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// SIMD ↔ 스칼라 동치성 검증
//
// 후보 필터(불리언)는 **완전히 같아야** 하고, 점수 항(f64 누산)은 순서 차이로
// 생기는 미세 오차만 허용한다. 판정(불리언/순위)을 바꾸는 오차는 실패로 본다.
// ────────────────────────────────────────────────────────────────────────────

#[cfg(all(test, feature = "simd"))]
mod simd_parity_tests {
    use super::*;

    /// 외부 의존성 없이 재현 가능한 결정적 LCG.
    struct Lcg(u64);

    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
        }

        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + self.next() * (hi - lo)
        }
    }

    /// tail(= `n % W`)까지 밟도록 lane 폭 4/8/16의 경계 전후를 모두 훑는다.
    const SIZES: [usize; 10] = [0, 1, 3, 4, 7, 8, 15, 16, 33, 155];

    fn random_boxes(n: usize, seed: u64) -> Vec<Aabb> {
        let mut rng = Lcg(seed);
        (0..n)
            .map(|_| {
                let min_x = rng.range(-5.0, 120.0);
                let min_y = rng.range(-5.0, 60.0);
                Aabb {
                    min_x,
                    min_y,
                    max_x: min_x + rng.range(0.5, 25.0),
                    max_y: min_y + rng.range(0.5, 25.0),
                }
            })
            .collect()
    }

    fn soa_of(boxes: &[Aabb]) -> AabbSoA {
        let mut soa = AabbSoA::default();
        soa.load_boxes(boxes);
        soa
    }

    fn rel_err(a: f64, b: f64) -> f64 {
        let scale = a.abs().max(b.abs()).max(1.0);
        (a - b).abs() / scale
    }

    #[test]
    fn simd_bbox_hits_match_the_scalar_predicate() {
        let eps = COORD_TIE_EPS as f64;
        for &n in &SIZES {
            let boxes = random_boxes(n, 0x5EED_0001 ^ n as u64);
            let soa = soa_of(&boxes);
            let mut rng = Lcg(0x00C0_FFEE ^ n as u64);
            let mut hits = Vec::new();

            for _ in 0..64 {
                let (x, y) = (rng.range(-10.0, 130.0), rng.range(-10.0, 70.0));
                bbox_hits(&soa, x, y, eps, &mut hits);
                let want: Vec<usize> = (0..n).filter(|&i| boxes[i].contains(x, y, eps)).collect();
                assert_eq!(hits, want, "n={n} p=({x}, {y})");
            }
        }
    }

    /// 경계에 **정확히** 걸친 점은 접촉 배치를 살리는 케이스라 어긋나면 안 된다.
    #[test]
    fn simd_bbox_hits_agree_on_exact_boundaries() {
        let eps = COORD_TIE_EPS as f64;
        let boxes = random_boxes(37, 0x0B01_DFACE);
        let soa = soa_of(&boxes);
        let mut hits = Vec::new();

        for b in &boxes {
            // 꼭짓점·변 위·eps 바로 안팎을 모두 찔러 본다.
            let probes = [
                (b.min_x, b.min_y),
                (b.max_x, b.max_y),
                (b.min_x - eps, b.min_y),
                (b.max_x + eps, b.max_y),
                (b.min_x - eps * 1.000_001, b.min_y),
                (b.max_x + eps * 1.000_001, b.max_y),
                (b.min_x, (b.min_y + b.max_y) * 0.5),
            ];
            for (x, y) in probes {
                bbox_hits(&soa, x, y, eps, &mut hits);
                let want: Vec<usize> = (0..boxes.len())
                    .filter(|&i| boxes[i].contains(x, y, eps))
                    .collect();
                assert_eq!(hits, want, "p=({x}, {y})");
            }
        }
    }

    #[test]
    fn simd_edge_bbox_hits_match_the_scalar_predicate() {
        let eps = COORD_TIE_EPS as f64;
        let mut rng = Lcg(0x1234_5678_9ABC_DEF0);

        for &n in &SIZES {
            let mut soa = EdgeSoA::default();
            let mut aos: Vec<[f64; 4]> = Vec::new();
            for _ in 0..n {
                let e = [
                    rng.range(-5.0, 120.0),
                    rng.range(-5.0, 60.0),
                    rng.range(-5.0, 120.0),
                    rng.range(-5.0, 60.0),
                ];
                let (min_x, max_x) = min_max(e[0], e[2]);
                let (min_y, max_y) = min_max(e[1], e[3]);
                soa.x1.push(e[0]);
                soa.y1.push(e[1]);
                soa.x2.push(e[2]);
                soa.y2.push(e[3]);
                soa.min_x.push(min_x);
                soa.min_y.push(min_y);
                soa.max_x.push(max_x);
                soa.max_y.push(max_y);
                aos.push(e);
            }
            assert_eq!(soa.len(), n);

            let mut hits = Vec::new();
            for _ in 0..32 {
                let q = [
                    rng.range(-5.0, 120.0),
                    rng.range(-5.0, 60.0),
                    rng.range(-5.0, 120.0),
                    rng.range(-5.0, 60.0),
                ];
                let (qmin_x, qmax_x) = min_max(q[0], q[2]);
                let (qmin_y, qmax_y) = min_max(q[1], q[3]);
                edge_bbox_hits(&soa, [qmin_x, qmin_y, qmax_x, qmax_y], eps, &mut hits);

                let want: Vec<usize> = (0..n)
                    .filter(|&i| segment_bbox_overlaps(&q, &aos[i], eps))
                    .collect();
                assert_eq!(hits, want, "n={n}");
            }
        }
    }

    #[test]
    fn simd_waste_matches_the_scalar_reference() {
        for &n in &SIZES {
            let boxes = random_boxes(n, 0x0000_A57F ^ n as u64);
            let soa = soa_of(&boxes);
            let mut rng = Lcg(0xDEAD_BEEF ^ n as u64);

            for _ in 0..32 {
                let min_x = rng.range(0.0, 120.0);
                let min_y = rng.range(0.0, 55.0);
                let world = Aabb {
                    min_x,
                    min_y,
                    max_x: min_x + rng.range(1.0, 30.0),
                    max_y: min_y + rng.range(1.0, 20.0),
                };
                let area = world.area() * rng.range(0.4, 1.0);

                let want = approx_waste(world, area, &boxes);
                let got = waste_of(world, area, &boxes, &soa);
                // frontier는 max 축약이라 순서에 무관하다 — 사실상 완전 일치해야 한다.
                assert!(rel_err(want, got) < 1e-14, "n={n} want={want} got={got}");
            }
        }
    }

    #[test]
    fn simd_contact_matches_the_scalar_reference() {
        for &n in &SIZES {
            let boxes = random_boxes(n, 0x00C0_07AC ^ n as u64);
            let soa = soa_of(&boxes);
            let mut rng = Lcg(0xFEED_FACE ^ n as u64);

            for _ in 0..32 {
                let min_x = rng.range(0.0, 120.0);
                let min_y = rng.range(0.0, 55.0);
                let world = Aabb {
                    min_x,
                    min_y,
                    max_x: min_x + rng.range(1.0, 30.0),
                    max_y: min_y + rng.range(1.0, 20.0),
                };

                let want = approx_contact(world, &boxes, 60.0);
                let got = contact_of(world, &boxes, &soa, 60.0);
                // f64 누산 순서만 다르다 — 상대오차는 반올림 수준이어야 한다.
                assert!(rel_err(want, got) < 1e-12, "n={n} want={want} got={got}");
            }
        }
    }

    /// 딱 붙은 이웃(간극 0)에서 접촉 길이가 흔들리면 4순위 순위가 뒤집힐 수 있다.
    #[test]
    fn simd_contact_is_exact_for_touching_neighbours() {
        let boxes: Vec<Aabb> = (0..40)
            .map(|k| {
                let x = k as f64 * 10.0;
                Aabb {
                    min_x: x,
                    min_y: 0.0,
                    max_x: x + 10.0,
                    max_y: 5.0,
                }
            })
            .collect();
        let soa = soa_of(&boxes);
        let world = Aabb {
            min_x: 100.0,
            min_y: 5.0,
            max_x: 110.0,
            max_y: 10.0,
        };

        let want = approx_contact(world, &boxes, 60.0);
        let got = contact_of(world, &boxes, &soa, 60.0);
        assert!(rel_err(want, got) < 1e-13, "want={want} got={got}");
        assert!(want > 0.0);
    }

    /// 런타임 디스패치는 이 CPU가 가진 폭 하나만 밟는다. 커널은 폭에 제네릭한
    /// 한 벌이므로 W=4/8/16을 **직접** 인스턴스화해 전부 검증한다(portable_simd가
    /// 하드웨어에 없는 폭도 에뮬레이트하므로 AVX-512 없는 기계에서도 돈다).
    #[test]
    fn every_lane_width_matches_the_scalar_reference() {
        let eps = COORD_TIE_EPS as f64;

        macro_rules! check_width {
            ($w:literal) => {
                for &n in &SIZES {
                    let boxes = random_boxes(n, 0x0071_D7AF ^ n as u64);
                    let soa = soa_of(&boxes);
                    let mut rng = Lcg(0x0071_5EED ^ n as u64);
                    let mut hits = Vec::new();

                    for _ in 0..16 {
                        let (x, y) = (rng.range(-10.0, 130.0), rng.range(-10.0, 70.0));
                        hits.clear();
                        bbox_hits_kernel::<$w>(&soa, x, y, eps, &mut hits);
                        let want: Vec<usize> =
                            (0..n).filter(|&i| boxes[i].contains(x, y, eps)).collect();
                        assert_eq!(hits, want, "W={} n={n}", $w);

                        let min_x = rng.range(0.0, 120.0);
                        let min_y = rng.range(0.0, 55.0);
                        let world = Aabb {
                            min_x,
                            min_y,
                            max_x: min_x + rng.range(1.0, 30.0),
                            max_y: min_y + rng.range(1.0, 20.0),
                        };

                        let f_want = frontier_scalar(&soa, world.min_y, world.max_y, eps, 0, 0.0);
                        let f_got = frontier_kernel::<$w>(&soa, world.min_y, world.max_y, eps);
                        assert_eq!(f_want, f_got, "frontier W={} n={n}", $w);

                        let c_want = contact_neighbours_scalar(&soa, world, 0);
                        let c_got = contact_kernel::<$w>(&soa, world);
                        assert!(
                            rel_err(c_want, c_got) < 1e-12,
                            "contact W={} n={n} want={c_want} got={c_got}",
                            $w
                        );
                    }
                }
            };
        }

        check_width!(4);
        check_width!(8);
        check_width!(16);
    }

    /// 4단 우선순위의 4순위(접촉도 최대) 판정이 SIMD/스칼라에서 같아야 한다.
    #[test]
    fn simd_contact_preserves_the_priority_4_ordering() {
        let boxes = random_boxes(101, 0x0A11_1CE5);
        let soa = soa_of(&boxes);
        let mut rng = Lcg(0x0BAD_5EED);

        for _ in 0..200 {
            let mk = |rng: &mut Lcg| {
                let min_x = rng.range(0.0, 120.0);
                let min_y = rng.range(0.0, 55.0);
                Aabb {
                    min_x,
                    min_y,
                    max_x: min_x + rng.range(1.0, 30.0),
                    max_y: min_y + rng.range(1.0, 20.0),
                }
            };
            let a = mk(&mut rng);
            let b = mk(&mut rng);

            let (sa, sb) = (
                approx_contact(a, &boxes, 60.0) as f32,
                approx_contact(b, &boxes, 60.0) as f32,
            );
            let (va, vb) = (
                contact_of(a, &boxes, &soa, 60.0) as f32,
                contact_of(b, &boxes, &soa, 60.0) as f32,
            );

            let scalar_ord = ScoreTerms {
                contact: sa,
                ..Default::default()
            };
            let scalar_ord2 = ScoreTerms {
                contact: sb,
                ..Default::default()
            };
            let simd_ord = ScoreTerms {
                contact: va,
                ..Default::default()
            };
            let simd_ord2 = ScoreTerms {
                contact: vb,
                ..Default::default()
            };
            assert_eq!(
                compare_by_priority(&scalar_ord, &scalar_ord2),
                compare_by_priority(&simd_ord, &simd_ord2),
                "contact scalar=({sa}, {sb}) simd=({va}, {vb})"
            );
        }
    }
}
