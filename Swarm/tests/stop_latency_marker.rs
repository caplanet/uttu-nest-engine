//! 실제 마커에서 Init 취소와 Explore/Compress/Polish 의 Stop 응답 시간을 잰다.
//!
//! 실행 (마커 경로는 환경변수로 준다):
//! ```powershell
//! $env:SWARM_MARKER='C:\...\marker.json'
//! cargo test --release --features "simd capmods adaptive structure-first" `
//!   --test stop_latency_marker -- --ignored --nocapture --test-threads=1
//! ```
//!
//! 기준: 각 단계에서 Stop 요청 뒤 반환까지 2초 이내. 사용자가 관찰한 «Init/Polish 에서
//! Stop 이 오래 걸린다» 를 재현·검증하는 테스트다.

use cpswcore::ffi::{CSwarmConfig, swarm_get_default_config};
use cpswcore::ffi_log::swarm_set_log_callback;
use cpswcore::ffi_step::*;
use std::ffi::CStr;
use std::os::raw::c_char;
use std::ffi::CString;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const STOP_BOUND: Duration = Duration::from_secs(2);

fn marker_json() -> Option<CString> {
    let path = std::env::var("SWARM_MARKER").ok()?;
    let json = std::fs::read_to_string(&path).expect("marker must be readable");
    Some(CString::new(json).expect("json contains NUL"))
}

fn config(explore_s: f32, compress_s: f32, polish_s: f32) -> CSwarmConfig {
    let mut cfg: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut cfg);
    cfg.rng_seed = 1;
    cfg.fabric_tolerance.piece_overlap_mm = 0.8;
    cfg.exploration.time_limit_secs = explore_s;
    cfg.compression.time_limit_secs = compress_s;
    cfg.polish.time_limit_secs = polish_s;
    cfg.polish.enabled = if polish_s > 0.0 { 1 } else { 0 };
    cfg
}

/// `after` 뒤에 `stop` 을 호출하고, 요청 시각을 기록한다.
fn arm<F: FnOnce() + Send + 'static>(after: Duration, stop: F) -> Arc<Mutex<Option<Instant>>> {
    let requested = Arc::new(Mutex::new(None));
    let slot = requested.clone();
    thread::spawn(move || {
        thread::sleep(after);
        *slot.lock().unwrap() = Some(Instant::now());
        stop();
    });
    requested
}

fn latency_ms(requested: &Arc<Mutex<Option<Instant>>>) -> Option<f32> {
    let requested = (*requested.lock().unwrap())?;
    Some(Instant::now().duration_since(requested).as_secs_f32() * 1000.0)
}

/// 포인터를 스레드 간에 넘기기 위한 래퍼. 핸들은 `Send + Sync` 다.
#[derive(Clone, Copy)]
struct HandlePtr(*mut SwarmStepHandle);
unsafe impl Send for HandlePtr {}
unsafe impl Sync for HandlePtr {}
impl HandlePtr {
    // 클로저가 필드(`.0`)만 잡아 `Send` 를 잃지 않도록 래퍼 전체를 통해 꺼낸다.
    fn get(self) -> *mut SwarmStepHandle {
        self.0
    }
}

#[test]
#[ignore]
fn init_cancel_returns_promptly_on_the_real_marker() {
    let Some(json) = marker_json() else {
        println!("SWARM_MARKER 가 없어 건너뛴다");
        return;
    };
    // 단계별 시각을 보기 위해 [INIT]/[NFP-LBF] 로그 줄만 콘솔에 찍고, jagua import
    // (협조 취소가 불가능한 구간) 의 소요 시간을 읽어 둔다.
    static IMPORT_MS: Mutex<u128> = Mutex::new(0);
    extern "C" fn sink(_level: i32, line: *const c_char, _user: usize) {
        let text = unsafe { CStr::from_ptr(line) }.to_string_lossy();
        if text.contains("[INIT]") || text.contains("[NFP-LBF]") || text.contains("[LBF]") {
            println!("    {text}");
        }
        if let Some(rest) = text.strip_prefix("[INIT] import + simplification: ") {
            if let Some(ms) = rest.split_whitespace().next().and_then(|v| v.parse::<u128>().ok()) {
                *IMPORT_MS.lock().unwrap() = ms;
            }
        }
    }
    swarm_set_log_callback(Some(sink), 0, 3);
    // 미세 래스터 + NFP-LBF 시드를 켜서 Init 을 일부러 길게 만든다 (실측 9~15초).
    let mut cfg = config(1.0, 1.0, 0.0);
    cfg.lbf_raster.cell_ratio = 0.02;
    cfg.lbf_raster.max_cells = 4_000_000;
    cfg.lbf_raster.n_random_samples = 4000;
    cfg.nfp_lbf.enabled = 1;

    let requested = arm(Duration::from_millis(1500), || swarm_cancel_init());
    let started = Instant::now();
    let h = swarm_init_from_json(json.as_ptr(), &cfg, std::ptr::null(), 0);
    let total = started.elapsed();
    let latency = latency_ms(&requested);
    println!(
        "init: total {:.0} ms, cancel latency {:?} ms, handle null = {}, code = {}",
        total.as_secs_f32() * 1000.0,
        latency,
        h.is_null(),
        swarm_get_last_error_code()
    );
    if !h.is_null() {
        swarm_free(h);
        // Init 이 1.5초 전에 끝났으면 취소 경로를 재지 못한 것 — 통과로 둔다.
        assert!(total < Duration::from_millis(1500), "init completed after the cancel request");
        return;
    }
    assert_eq!(swarm_get_last_error_code(), 2002, "cancelled init must report code 2002");
    let latency = latency.expect("cancel was requested");
    // jagua 의 import/단순화는 협조 취소가 안 되는 구간이라 그 시간은 하한으로 빼 준다.
    // 취소 요청이 import 도중에 들어오면 지연 = (import 잔여) + 우리 쪽 응답이다.
    let import_ms = *IMPORT_MS.lock().unwrap() as f32;
    let bound = STOP_BOUND.as_secs_f32() * 1000.0 + import_ms;
    println!("init: import floor {import_ms:.0} ms -> bound {bound:.0} ms");
    assert!(latency < bound, "init cancel took {latency:.0} ms (bound {bound:.0} ms)");
}

#[test]
#[ignore]
fn phases_stop_promptly_on_the_real_marker() {
    let Some(json) = marker_json() else {
        println!("SWARM_MARKER 가 없어 건너뛴다");
        return;
    };
    // 각 단계가 Stop 없이도 끝나지 않을 만큼 긴 예산을 준다.
    let cfg = config(600.0, 600.0, 600.0);
    let h = swarm_init_from_json(json.as_ptr(), &cfg, std::ptr::null(), 0);
    assert!(!h.is_null(), "init failed: {}", swarm_get_last_error_code());
    let hp = HandlePtr(h);

    let mut worst = 0.0f32;
    for (name, run, after) in [
        ("explore", swarm_alg_phase as extern "C" fn(*mut SwarmStepHandle) -> i32, Duration::from_secs(20)),
        ("compress", swarm_cp_phase, Duration::from_secs(8)),
        ("polish", swarm_refine_phase, Duration::from_secs(8)),
    ] {
        // Stop 은 핸들 수명 동안 영구적이라 단계마다 새 핸들이 필요하다.
        let h = if name == "explore" {
            hp.get()
        } else {
            let fresh = swarm_init_from_json(json.as_ptr(), &cfg, std::ptr::null(), 0);
            assert!(!fresh.is_null());
            fresh
        };
        if name == "polish" {
            // Polish 는 Compress 뒤에만 허용된다. 짧은 Compress 를 먼저 끝낸다.
            let mut short = cfg;
            short.compression.time_limit_secs = 3.0;
            let warm = swarm_init_from_json(json.as_ptr(), &short, std::ptr::null(), 0);
            assert!(!warm.is_null());
            swarm_free(h);
            assert_eq!(swarm_cp_phase(warm), 0, "compress before polish failed");
            // 이후 Polish 는 원래(긴) 예산으로 돈다: 핸들 설정은 Init 시점에 고정되므로
            // 여기서는 polish 예산만 긴 config 로 새 핸들을 만들 수 없다 — 대신 짧은
            // Compress 를 가진 config 의 polish 예산(600초)이 그대로 쓰인다.
            let hp3 = HandlePtr(warm);
            let requested = arm(after, move || swarm_request_stop(hp3.get()));
            let started = Instant::now();
            let rc = run(warm);
            let total = started.elapsed();
            let latency = latency_ms(&requested);
            println!(
                "{name}: rc {rc}, total {:.1} s, stop latency {:?} ms",
                total.as_secs_f32(),
                latency
            );
            swarm_free(warm);
            if let Some(latency) = latency {
                worst = worst.max(latency);
            }
            continue;
        }
        let hp2 = HandlePtr(h);
        let requested = arm(after, move || swarm_request_stop(hp2.get()));
        let started = Instant::now();
        let rc = run(h);
        let total = started.elapsed();
        let latency = latency_ms(&requested);
        println!(
            "{name}: rc {rc}, total {:.1} s, stop latency {:?} ms",
            total.as_secs_f32(),
            latency
        );
        swarm_free(h);
        if let Some(latency) = latency {
            worst = worst.max(latency);
        }
    }
    assert!(
        worst < STOP_BOUND.as_secs_f32() * 1000.0,
        "worst stop latency {worst:.0} ms exceeds {STOP_BOUND:?}"
    );
}

/// Polish 안의 어느 단계(LP·슬라이드·홀 채움)에서 Stop 이 들어오든 응답이 빠른지,
/// 요청 시각을 바꿔 가며 잰다.
#[test]
#[ignore]
fn polish_stop_latency_sweep_on_the_real_marker() {
    let Some(json) = marker_json() else {
        println!("SWARM_MARKER 가 없어 건너뛴다");
        return;
    };
    let mut cfg = config(1.0, 3.0, 600.0);
    cfg.compression.time_limit_secs = 3.0;
    // 폴리시 로그 줄을 경과 시각과 함께 찍는다 — Stop 이 어느 하위 단계에 걸렸는지 본다.
    static T0: Mutex<Option<Instant>> = Mutex::new(None);
    extern "C" fn sink(_level: i32, line: *const c_char, _user: usize) {
        let text = unsafe { CStr::from_ptr(line) }.to_string_lossy();
        if text.contains("[PLSH]") || text.contains("[HOLE]") || text.contains("[LP") || text.contains("[SLIDE") || text.contains("[RECLAIM") {
            let t = T0.lock().unwrap().map(|t| t.elapsed().as_millis()).unwrap_or(0);
            println!("    +{t:>6} ms {text}");
        }
    }
    swarm_set_log_callback(Some(sink), 0, 3);
    let mut worst = 0.0f32;
    for after_ms in [8000u64, 12000] {
        let h = swarm_init_from_json(json.as_ptr(), &cfg, std::ptr::null(), 0);
        assert!(!h.is_null());
        assert_eq!(swarm_cp_phase(h), 0);
        let hp = HandlePtr(h);
        let requested = arm(Duration::from_millis(after_ms), move || swarm_request_stop(hp.get()));
        let started = Instant::now();
        *T0.lock().unwrap() = Some(started);
        let rc = swarm_refine_phase(h);
        let total = started.elapsed();
        let latency = latency_ms(&requested);
        println!(
            "polish stop at {after_ms} ms: rc {rc}, total {:.1} s, latency {:?} ms",
            total.as_secs_f32(),
            latency
        );
        swarm_free(h);
        if let Some(l) = latency {
            worst = worst.max(l);
        }
    }
    assert!(
        worst < STOP_BOUND.as_secs_f32() * 1000.0,
        "worst polish stop latency {worst:.0} ms"
    );
}
