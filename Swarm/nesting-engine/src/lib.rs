//! 신규 초기 배치 엔진 (`C:\Caplanet\Ref\신규엔진-구현-지시서-ChatGPT.md`).
//!
//! 지시서 §12 의 1~8 단계를 담는다.
//!
//! | 단계 | 모듈 |
//! |---|---|
//! | 좌표·자료구조·입출력 (§0, §1) | [`geom`], [`io`] |
//! | 전처리·NFP (§2) | [`nfp`] |
//! | Draft (§3) — LP 없음 | [`draft`], [`gate`] |
//! | 검증·게이트 (§8) | [`validate`] |
//! | Complete (§4) | [`complete`] |
//! | 재시작 + 압축 LP (§0.1.2, §5) | [`compact`], [`lp`] |
//! | 잔여조각 LP (§6), 밀어내기 압축 (§5.11) | [`residual`], [`pushin`] |
//! | 상태 코드 (§5.12) | [`state`] |
//! | 전체 순서·20초 예산 (§0.1, §4.1) | [`pipeline`] |
//! | 초기 배치 **이후** 단계 선택 (§12.1.32) | [`chain`] |
//!
//! 기존 `swarm` 엔진과 코드를 공유하지 않는다. §13 "기존 코드를 참고하되 명세와
//! 충돌하면 명세를 따른다" 와, 사용자의 "따로 진행" 지시 때문이다.

pub mod chain;
pub mod compact;
pub mod complete;
pub mod draft;
pub mod gate;
pub mod geom;
pub mod io;
pub mod lp;
pub mod nfp;
pub mod pipeline;
pub mod pushin;
pub mod residual;
pub mod state;
pub mod validate;
