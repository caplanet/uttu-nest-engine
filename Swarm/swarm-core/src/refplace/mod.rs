//! 레퍼런스 초기 배치 엔진 — `레퍼런스-초기배치-설계.md` 를 그대로 구현한다.
//!
//! # 왜 기존 `initial` / `nfp_lbf` 와 별도인가
//!
//! 아키텍처가 다르다. 같은 "NFP 경계에서 후보를 뽑는다" 라도 무엇을 먼저 놓을지,
//! 후보를 어떻게 모을지, 무엇으로 점수를 매길지가 전부 다르다:
//!
//! ```text
//!                    이 모듈(설계 문서)              기존 nfp_lbf / initial
//!   배치 순서        그룹 정의 기반 부착 체인          면적·직경·오목도 정렬
//!   후보 수집        부착 대상의 NFP 경계 (3모드)      {미배치} x {배치됨} 전역
//!   회전             조각별 (x축,y축) 쌍 목록          허용 회전 집합
//!   점수             효율 + 표면 간 균형 (8배 감점)    무차원 가중합 4단 우선순위
//!   정밀도           Draft -> Complete 2단계           단일
//! ```
//!
//! 섞으면 어느 쪽 계약도 지킬 수 없다. 그래서 별도 모듈로 두고, 기존 경로는 한 비트도
//! 건드리지 않는다.
//!
//! # 모듈 구성 (설계 §2 의 모듈 번호를 그대로 따른다)
//!
//! ```text
//!   params    모듈 1  전역 파라미터 테이블 (§5 표 그대로)
//!   record    모듈 2  피스/표면 레코드, 회전 목록, 간격 설정
//!   chain     모듈 3  부착 체인 구축 + NFP 양방향 선계산
//!   collect   모듈 4·5 후보 수집 3모드, 정렬·분배
//!   step      모듈 6  배치 스텝 (NFP 이진탐색 조회, 보류 체인)
//!   boundary  모듈 7  NFP 경계 배치 후보 평가 루프 (상한 10000)
//!   score     모듈 8  점수 산식 (§4 그대로)
//!   finish    모듈 9  검증·2차 패스·효율 재계산
//!   post      모듈 10 Complete 재배치 / 잔여조각 LP / Pack
//! ```
//!
//! # 기본은 **꺼짐**이다
//!
//! 이 엔진은 설정으로 켜야 돈다. 켜지 않으면 기존 초기 배치 경로가 그대로다 —
//! 그것이 A/B 측정의 전제이고, 이 저장소가 반복해 확인한 규칙이다.

pub mod boundary;
pub mod chain;
pub mod collect;
pub mod columns;
pub mod driver;
pub mod finish;
pub mod geom;
pub mod params;
pub mod post;
pub mod record;
pub mod score;
pub mod step;

pub use chain::{
    AttachChain, GroupFallback, GroupSpec, NfpLookup, NfpPairIndex, NfpPairKey, NfpPairPlan,
    PieceExtent, PoseKey, build_chain,
};
pub use collect::{Candidate, CandidateSet, CollectMode, collect};
pub use boundary::{BoundaryCandidate, BoundaryOutcome, EvalRecord, WorkItem, evaluate_candidates};
pub use finish::{EfficiencySnapshot, SecondPass, Stage, SurfaceError, SurfaceReport};
pub use geom::{PoseCatalogue, orientation_of, world_bbox};
pub use driver::{
    RefPlaceReport, build_reference_nest, build_reference_nest_reported, rotations_from_instance,
};
pub use params::RefPlaceParams;
pub use post::{JoinOutcome, LpOutcome, PackDirection, Precision, ResidualLp};
pub use step::{NfpSummary, ReadCode, StepGuard, StepOutcome, step_piece};
pub use record::{
    ContactPoint, GapBehaviour, GapSettings, GapType, PieceRecord, PlaceState, PlacedPose,
    RotationEntry, RotationList, SurfaceRecord,
};
