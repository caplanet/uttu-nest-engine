#![cfg_attr(feature = "simd", feature(portable_simd))]

/// 하드닝(디컴파일 방지, C-7): panic/expect/unreachable 의 메시지 텍스트를
/// release 빌드(`debug_assertions` off)에서 `""` 로 치환해 **컴파일에서 제외**한다.
/// 내부 알고리즘을 드러내는 서술형 메시지가 바이너리 문자열로 남지 않는다.
/// dev/test 빌드(`debug_assertions` on)에서는 전체 텍스트를 유지한다(진단용).
/// 패닉 "발생 위치"는 `.cargo/config.toml` 의 `-Z location-detail=none` 이 별도로 제거한다.
#[macro_export]
macro_rules! dmsg {
    ($s:literal) => {{
        #[cfg(debug_assertions)]
        {
            $s
        }
        #[cfg(not(debug_assertions))]
        {
            ""
        }
    }};
}

use jagua_rs::Instant;
use std::sync::LazyLock;

pub mod config;
#[cfg(feature = "init-profile")]
pub mod init_profile;
pub mod consts;
// 하드닝: 도메인 시드 유도를 cpguard VM 으로 가상화(매직 상수 은닉). swarm-core/swarm 공용.
pub mod protected_seed;
// 런타임 안티탬퍼(선택 feature). 후킹/인젝션 탐지 + 호스트 텔레메트리/점진저하 신호.
#[cfg(feature = "antitamper")]
pub mod tamper;
pub mod escape;
pub mod eval;
pub mod initial;
pub mod lp_solver;
pub mod nfp_lbf;
pub mod nfp_strategy;
pub mod optimizer;
pub mod fabric_edge;
pub mod original_overlap;
pub mod prepared_geometry;
pub mod quantify;
pub mod refplace;
pub mod sample;
pub mod structure;
pub mod util;

// 시간 측정
pub static EPOCH: LazyLock<Instant> = LazyLock::new(Instant::now);
