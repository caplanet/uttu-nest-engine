//! Rust 로그를 호스트(C#/WPF)로 넘기는 콜백 경계.
//!
//! # 왜 필요한가
//!
//! `log` 크레이트는 **로거가 등록되지 않으면 모든 매크로가 no-op**이다.
//! 이 워크스페이스에서 로거를 설치하는 곳은 [`swarm_core::util::io::init_logger`]
//! 하나뿐이고, 그건 CLI 바이너리만 부른다. 즉 cdylib(`cpswcore.dll`)로
//! 올라온 호스트에서는 `log::info!` / `log::warn!` 이 **한 줄도 나가지 않았다**.
//!
//! 그래서 이런 진단이 전부 보이지 않았다:
//!
//! ```text
//! [INIT] NFP-LBF 시드 채택: 폭 7479.090 -> 7082.270 ...
//! [INIT] NFP-LBF 시드 기각: 폭 ... >= 기존 LBF ... — 기존 배치를 유지한다
//! [NFP-LBF] 탐침 예산 4096/아이템, ordering 50개, 예산 10000 ms
//! ```
//!
//! 로그가 안 보이니 "설정이 전달되지 않는다"와 "전달됐지만 결과가 기각됐다"를
//! 구분할 방법이 없었다. 이 모듈이 그 눈을 만들어 준다.
//!
//! # 사용법 (C#)
//!
//! ```csharp
//! // 델리게이트를 필드로 잡아 둬야 GC가 수거하지 않는다.
//! _logSink = (level, msg, _) => Log(level, Marshal.PtrToStringUTF8(msg));
//! SwarmNative.swarm_set_log_callback(_logSink, UIntPtr.Zero, 3 /* Info */);
//! ```
//!
//! # 스레드
//!
//! 콜백은 **아무 스레드에서나** 불린다 (rayon 워커 포함). 동시 호출도 일어난다.
//! 호스트 쪽 핸들러는 스레드 안전해야 하고, UI를 직접 만지면 안 된다.
//!
//! # 레벨
//!
//! `log` 의존성이 `release_max_level_info` 로 잡혀 있어 **릴리스 빌드에서는
//! `debug!` / `trace!` 가 아예 컴파일되지 않는다**. `max_level` 을 4/5로 올려도
//! 릴리스에서는 Info 까지만 나온다. 이는 설계된 동작이다 — 배치 루프 안쪽의
//! `debug!` 는 초당 수만 줄이라 콜백으로 흘리면 그 자체가 병목이 된다.

use std::ffi::CString;
use std::os::raw::c_char;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Mutex, OnceLock};

use log::{Level, LevelFilter, Log, Metadata, Record};

/// 호스트가 등록하는 로그 수신 함수.
///
/// * `level` — 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Trace
/// * `message` — NUL 종료 UTF-8. **콜백이 반환하는 즉시 무효**다. 보관하려면 복사할 것.
/// * `user_data` — [`swarm_set_log_callback`]에 넘긴 값을 그대로 돌려준다.
pub type SwarmLogCallback =
    extern "C" fn(level: i32, message: *const c_char, user_data: usize);

/// 등록된 싱크. `None`이면 로거는 설치돼 있어도 아무것도 하지 않는다.
static SINK: Mutex<Option<(SwarmLogCallback, usize)>> = Mutex::new(None);

/// `log::set_logger`는 프로세스당 한 번만 성공한다. 재등록은 싱크만 갈아 끼운다.
static INSTALLED: OnceLock<bool> = OnceLock::new();

/// 콜백이 패닉하거나 메시지에 NUL이 섞여 버린 줄 수. 진단용.
static DROPPED: AtomicUsize = AtomicUsize::new(0);

struct FfiLogger;

static LOGGER: FfiLogger = FfiLogger;

impl Log for FfiLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        // 레벨 판정은 `log::set_max_level`이 이미 해 준다. 여기서 또 걸지 않는다.
        true
    }

    fn log(&self, record: &Record) {
        // 싱크가 없으면 포맷 비용조차 치르지 않는다.
        let Ok(guard) = SINK.lock() else {
            // 다른 스레드가 콜백 안에서 패닉해 뮤텍스가 오염된 경우.
            DROPPED.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let Some((callback, user_data)) = *guard else {
            return;
        };
        drop(guard);

        let text = format!("{}", record.args());
        // 메시지 안의 NUL은 C 문자열을 잘라 먹는다. 버리지 말고 치환한다.
        let sanitized = if text.as_bytes().contains(&0) {
            text.replace('\0', "\u{fffd}")
        } else {
            text
        };
        let Ok(c_msg) = CString::new(sanitized) else {
            DROPPED.fetch_add(1, Ordering::Relaxed);
            return;
        };

        // 호스트 콜백이 언와인드하면 FFI 경계를 넘어 UB가 된다. 잡아서 버린다.
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            callback(level_to_i32(record.level()), c_msg.as_ptr(), user_data);
        }));
        if result.is_err() {
            DROPPED.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn flush(&self) {}
}

#[inline]
fn level_to_i32(level: Level) -> i32 {
    match level {
        Level::Error => 1,
        Level::Warn => 2,
        Level::Info => 3,
        Level::Debug => 4,
        Level::Trace => 5,
    }
}

#[inline]
fn i32_to_filter(level: i32) -> LevelFilter {
    match level {
        i32::MIN..=0 => LevelFilter::Off,
        1 => LevelFilter::Error,
        2 => LevelFilter::Warn,
        3 => LevelFilter::Info,
        4 => LevelFilter::Debug,
        _ => LevelFilter::Trace,
    }
}

/// 로그 콜백을 등록한다.
///
/// * `callback` — `None`(=널 포인터)이면 등록을 해제한다. 로거 자체는 남지만 아무것도
///   호출하지 않는다.
/// * `user_data` — 콜백에 그대로 되돌려 줄 값. 쓰지 않으면 0.
/// * `max_level` — 0=Off, 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Trace.
///   릴리스 빌드는 `release_max_level_info` 때문에 3을 넘어서 나오지 않는다.
///
/// 반환값은 **1 = 성공, 0 = 실패**다. 실패는 로거 설치가 거부된 경우뿐이며
/// (다른 로거가 이미 프로세스에 설치돼 있음) 그때도 싱크는 갱신되지 않는다.
///
/// 여러 번 불러도 안전하다. 두 번째부터는 싱크와 레벨만 갈아 끼운다.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_set_log_callback(
    callback: Option<SwarmLogCallback>,
    user_data: usize,
    max_level: i32,
) -> i32 {
    let installed = *INSTALLED.get_or_init(|| log::set_logger(&LOGGER).is_ok());
    if !installed {
        return 0;
    }

    match SINK.lock() {
        Ok(mut guard) => *guard = callback.map(|cb| (cb, user_data)),
        Err(_) => return 0,
    }

    log::set_max_level(if callback.is_some() {
        i32_to_filter(max_level)
    } else {
        LevelFilter::Off
    });
    1
}

/// 콜백으로 넘기지 못하고 버린 줄 수 (호스트 패닉 / NUL 오염 / 뮤텍스 오염).
///
/// 정상 동작에서는 0이다. 0이 아니면 호스트 핸들러를 의심할 것.
#[unsafe(no_mangle)]
pub extern "C" fn swarm_log_dropped_count() -> u64 {
    DROPPED.load(Ordering::Relaxed) as u64
}

/// 호스트가 런타임 탬퍼(후킹/인젝션) 보고 콜백을 등록(선택 feature `antitamper`).
///
/// 콜백은 **의심 탐지 시에만** `(score, user_data)` 로 호출된다. score 가 클수록 강한 신호.
/// 호스트는 라이선스/세션 차원에서 대응하라 — 콜백 안에서 호스트를 크래시시키지 말 것.
/// 스캔은 `swarm_init`/`swarm_init_from_json` 에서 run 당 1회 수행된다(비핫).
#[cfg(feature = "antitamper")]
#[unsafe(no_mangle)]
pub extern "C" fn swarm_set_tamper_callback(
    callback: swarm_core::tamper::TamperCallback,
    user_data: usize,
) {
    swarm_core::tamper::set_callback(callback, user_data);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU32;

    static SEEN: AtomicU32 = AtomicU32::new(0);
    static LAST_LEVEL: AtomicU32 = AtomicU32::new(0);

    extern "C" fn sink(level: i32, message: *const c_char, user_data: usize) {
        assert!(!message.is_null());
        assert_eq!(user_data, 0xABCD);
        // NUL 종료 문자열로 안전하게 읽히는지 확인한다.
        let text = unsafe { std::ffi::CStr::from_ptr(message) };
        assert!(text.to_str().is_ok(), "메시지는 유효한 UTF-8이어야 한다");
        LAST_LEVEL.store(level as u32, Ordering::SeqCst);
        SEEN.fetch_add(1, Ordering::SeqCst);
    }

    /// 등록 -> 로그 -> 해제 -> 로그. 해제 후에는 콜백이 불리지 않아야 한다.
    ///
    /// `log::set_logger`가 프로세스당 1회라 테스트를 나누면 서로를 방해한다.
    /// 그래서 한 테스트에 몰아 넣는다.
    #[test]
    fn callback_receives_lines_and_stops_after_clear() {
        assert_eq!(swarm_set_log_callback(Some(sink), 0xABCD, 3), 1);

        SEEN.store(0, Ordering::SeqCst);
        log::info!("탐침 예산 {}/아이템", 4096);
        assert_eq!(SEEN.load(Ordering::SeqCst), 1, "Info는 전달되어야 한다");
        assert_eq!(LAST_LEVEL.load(Ordering::SeqCst), 3);

        log::warn!("시드 기각");
        assert_eq!(SEEN.load(Ordering::SeqCst), 2);
        assert_eq!(LAST_LEVEL.load(Ordering::SeqCst), 2);

        // 레벨을 Error로 조이면 Info/Warn은 막힌다.
        assert_eq!(swarm_set_log_callback(Some(sink), 0xABCD, 1), 1);
        log::info!("무시되어야 한다");
        log::warn!("무시되어야 한다");
        assert_eq!(SEEN.load(Ordering::SeqCst), 2, "레벨 아래는 전달되지 않는다");
        log::error!("전달되어야 한다");
        assert_eq!(SEEN.load(Ordering::SeqCst), 3);

        // 해제.
        assert_eq!(swarm_set_log_callback(None, 0, 3), 1);
        log::error!("해제 후에는 전달되지 않는다");
        assert_eq!(SEEN.load(Ordering::SeqCst), 3);

        assert_eq!(swarm_log_dropped_count(), 0, "정상 경로에서 버린 줄은 없어야 한다");
    }

    #[test]
    fn level_mapping_is_stable() {
        assert_eq!(level_to_i32(Level::Error), 1);
        assert_eq!(level_to_i32(Level::Warn), 2);
        assert_eq!(level_to_i32(Level::Info), 3);
        assert_eq!(level_to_i32(Level::Debug), 4);
        assert_eq!(level_to_i32(Level::Trace), 5);

        assert_eq!(i32_to_filter(0), LevelFilter::Off);
        assert_eq!(i32_to_filter(-1), LevelFilter::Off);
        assert_eq!(i32_to_filter(3), LevelFilter::Info);
        assert_eq!(i32_to_filter(99), LevelFilter::Trace);
    }
}
