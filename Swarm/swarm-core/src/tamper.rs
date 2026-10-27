//! cpswcore 런타임 안티탬퍼 통합(선택 feature `antitamper`). cpguard::antitamper 의 얇은 래퍼.
//!
//! 배치 원칙: **run 당 1회 스캔**(비핫). 결과를 캐시해 phase/FFI 에서 재사용한다. 핫루프 금지.
//! 반응은 호스트 텔레메트리(크래시 없음)와, 선택적으로 점진 저하 신호(`antitamper-degrade`).

use cpguard::antitamper;
use std::sync::{Mutex, OnceLock};

pub use cpguard::antitamper::TamperReport;
// telemetry 는 antitamper feature 와 항상 함께 켜진다(swarm-core/antitamper = cpguard/antitamper+telemetry).
pub use cpguard::antitamper::telemetry::TamperCallback;

static LAST: Mutex<Option<TamperReport>> = Mutex::new(None);
// 무거운 스캔(모듈 열거 + 메모리 워크 + .text 해시)은 프로세스당 1회만 캐시한다.
static HEAVY: OnceLock<TamperReport> = OnceLock::new();

/// run 당 1회 호출(비핫 경로). `watch` = 프롤로그 후킹 검사할 export 함수 주소들.
///
/// 성능: 무거운 부분(모듈/메모리/.text)은 **프로세스당 1회**만 수행해 캐시하므로, init 을
/// 여러 번 반복해도 총비용은 1회(≈ 수~십수 ms)다. **값싼 프롤로그 후킹 검사만 run 마다** 신선하게
/// 수행한다(마이크로초). 의심되면 호스트 콜백으로 보고(크래시 없음).
pub fn run_scan(watch: &[usize]) {
    let heavy = HEAVY.get_or_init(|| antitamper::scan(&[]));
    let hooked_now = antitamper::check_prologues(watch);
    let mut r = heavy.clone();
    r.hooked_prologue |= hooked_now;
    antitamper::telemetry::report(&r);
    if let Ok(mut g) = LAST.lock() {
        *g = Some(r);
    }
}

/// 마지막 스캔의 탐지 점수(0=clean). 캐시 미존재 시 0.
pub fn last_score() -> u32 {
    LAST.lock()
        .ok()
        .and_then(|g| g.as_ref().map(|r| r.score()))
        .unwrap_or(0)
}

/// 호스트(C#/Java)가 탬퍼 보고 콜백을 등록.
pub fn set_callback(cb: antitamper::telemetry::TamperCallback, user_data: usize) {
    antitamper::telemetry::set_tamper_callback(cb, user_data);
}

/// 점진 저하 품질 배수(clean=1.0, 탬퍼 시 0.7 까지). 앱이 비핫 지점에서 knob 에 곱한다.
/// ⚠ 양날: 오탐 시 정상 사용자 결과가 나빠진다. `antitamper-degrade` feature 전용.
#[cfg(feature = "antitamper-degrade")]
pub fn quality_factor() -> f64 {
    LAST.lock()
        .ok()
        .and_then(|g| g.as_ref().map(antitamper::degrade::quality_factor))
        .unwrap_or(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    // 정상(clean) 프로세스에서는 탐지 score 가 0 이어야 하고, run_scan 오버헤드를 실측 출력한다.
    #[test]
    fn clean_scan_and_overhead() {
        let watch = [run_scan as usize, last_score as usize];
        run_scan(&watch); // warm + 캐시
        assert_eq!(last_score(), 0, "unexpected tamper score in a clean test process");

        let n = 20u32;
        let t = Instant::now();
        for _ in 0..n {
            run_scan(&watch);
        }
        eprintln!("[bench] run_scan avg = {:?} over {n} iters", t.elapsed() / n);

        let t2 = Instant::now();
        for _ in 0..n {
            let _ = antitamper::self_text_digest();
        }
        eprintln!("[bench] self_text_digest avg = {:?}", t2.elapsed() / n);
    }
}
