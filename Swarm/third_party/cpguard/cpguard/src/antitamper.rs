//! 런타임 안티탬퍼 탐지 (feature `antitamper`).
//!
//! 동적 계측(인라인 후킹, DLL 인젝션/수동매핑, Frida 등)을 **스스로 탐지**하는 프리미티브.
//! 실제 OS 탐지는 **Windows 전용**(`#[cfg(windows)]`)이고, 그 외 플랫폼에서는 clean 을 반환한다.
//!
//! 설계 원칙(호스트 DLL):
//! - **크래시 금지.** 반응은 `telemetry`(호스트 콜백 보고) 또는 `degrade`(점진 저하 신호)로.
//! - 가장 강한 반응은 분기 없는 **무결성→VM 키 결합**([`integrity_key`]): 코드가 패치되면
//!   키가 틀어져 바이트코드 복호가 조용히 깨진다(탐지 boolean 이 없어 패치 제거 대상이 없음).
//!
//! 한계: 같은 프로세스 공격자는 탐지 자체를 무력화할 수 있다. 비용을 올리는 억제책이다.
//! 적용 시 대상 AV/EDR **오탐 테스트**와 호스트 스모크 테스트를 동반할 것.

use alloc::string::String;
use alloc::vec::Vec;

/// 탐지 요약.
#[derive(Debug, Default, Clone)]
pub struct TamperReport {
    /// 감시 함수 프롤로그에서 인라인 후킹 서명이 발견됨.
    pub hooked_prologue: bool,
    /// 의심 모듈명(frida/gum/detour 등).
    pub suspicious_modules: Vec<String>,
    /// EXECUTE 권한의 private(비이미지) 커밋 영역 수(수동매핑/셸코드 지표).
    pub exec_private_regions: usize,
    /// 자기 `.text` 해시(코드 패치 시 변함).
    pub text_digest: u64,
}

impl TamperReport {
    /// 하나라도 의심 신호가 있는가.
    pub fn is_suspicious(&self) -> bool {
        self.hooked_prologue || !self.suspicious_modules.is_empty() || self.exec_private_regions > 0
    }
    /// 탐지 강도 점수(0=clean). telemetry/degrade 가 활용.
    pub fn score(&self) -> u32 {
        let mut s = 0u32;
        if self.hooked_prologue {
            s = s.saturating_add(100);
        }
        s = s.saturating_add((self.suspicious_modules.len() as u32).saturating_mul(50));
        s = s.saturating_add(self.exec_private_regions as u32);
        s
    }
}

/// 주어진 함수 포인터들의 프롤로그(첫 16바이트)에서 인라인 후킹 서명을 검사.
///
/// 안전: 유효한 함수 주소만 넘길 것(코드 페이지 읽기). 0 은 건너뛴다.
pub fn check_prologues(fns: &[usize]) -> bool {
    for &f in fns {
        if f == 0 {
            continue;
        }
        let bytes = unsafe { core::slice::from_raw_parts(f as *const u8, 16) };
        if is_hook_prologue(bytes) {
            return true;
        }
    }
    false
}

fn is_hook_prologue(b: &[u8]) -> bool {
    if b.is_empty() {
        return false;
    }
    match b[0] {
        0xE9 | 0xEB => true,                                 // jmp rel32 / rel8
        0xCC => true,                                        // int3 (software bp)
        0xFF if b.len() >= 2 && b[1] == 0x25 => true,        // jmp [rip+disp32]
        0x68 if b.len() >= 6 && b[5] == 0xC3 => true,        // push imm32; ret
        0x48 if b.len() >= 12 && b[1] == 0xB8 && b[10] == 0xFF && b[11] == 0xE0 => true, // mov rax,imm64; jmp rax
        _ => false,
    }
}

/// 자기 모듈 **인메모리** `.text` 해시. 코드가 패치/후킹되면 값이 변한다. (Windows 전용; 그 외 0)
///
/// ⚠ ASLR/relocation 주의: 인메모리 `.text` 는 로드 베이스에 따라 일부 바이트(절대주소
/// relocation)가 **프로세스마다 달라질 수 있다**. RIP-상대 위주의 Rust 코드는 `.text`
/// relocation 이 거의 없어 보통 안정적이지만 **보장은 아니다**. [`integrity_key`] 처럼
/// "다른 프로세스에서 측정한 기대값" 과 비교하려면, 먼저 **프로세스를 두 번 띄워 값이 같은지
/// 확인**하라. 불안정하면 on-disk `.text` 해시나 [`check_prologues`](특정 함수 프롤로그 비교)를
/// 쓸 것. (같은 프로세스 내 비교/키 혼합에는 안정적이다.)
pub fn self_text_digest() -> u64 {
    imp::self_text_digest()
}

/// 로드 모듈/메모리 스캔 + 프롤로그 검사를 종합한 리포트.
pub fn scan(watch_fns: &[usize]) -> TamperReport {
    let mut r = TamperReport {
        hooked_prologue: check_prologues(watch_fns),
        ..Default::default()
    };
    imp::augment(&mut r);
    r.text_digest = imp::self_text_digest();
    r
}

/// VM 복호 키에 자기 무결성을 섞는다(분기 없는 탬퍼 반응).
///
/// `expected_digest` 는 **신뢰 빌드에서 측정해 임베드**한 `self_text_digest()` 값이다.
/// 코드가 패치되지 않았다면 `self_text_digest() == expected_digest` 라 결과가 `base_key`,
/// 패치되었다면 키가 틀어져 바이트코드 복호가 깨진다.
///
/// ⚠ [`self_text_digest`] 의 ASLR 주의 참고: 사용 전 **같은 바이너리를 두 번 실행해
/// self_text_digest 가 동일한지 반드시 확인**하라(다르면 정상 빌드도 복호가 깨진다). 불안정하면
/// 이 방식 대신 `scan()`/`check_prologues` 기반 탐지(telemetry/degrade)를 쓸 것.
///
/// ```ignore
/// // 1) 신뢰 빌드에서 한 번: println!("{:#x}", cpguard::antitamper::self_text_digest());
/// // 2) 그 값을 EXPECTED 로 임베드:
/// let key = cpguard::antitamper::integrity_key(BASE_KEY, EXPECTED_DIGEST);
/// let prot = cpguard::vm::Protected::new(ENC.to_vec(), key, OP_SEED, N);
/// ```
pub fn integrity_key(base_key: u64, expected_digest: u64) -> u64 {
    base_key ^ (self_text_digest() ^ expected_digest)
}

// ───────────────────────── Windows 구현 ─────────────────────────
#[cfg(all(windows, feature = "antitamper"))]
mod imp {
    use super::TamperReport;
    use alloc::string::String;
    use alloc::vec::Vec;
    use windows_sys::Win32::Foundation::{CloseHandle, HMODULE, INVALID_HANDLE_VALUE};
    use windows_sys::Win32::System::Diagnostics::ToolHelp::{
        CreateToolhelp32Snapshot, Module32FirstW, Module32NextW, MODULEENTRY32W, TH32CS_SNAPMODULE,
    };
    use windows_sys::Win32::System::LibraryLoader::{
        GetModuleHandleExW, GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
    };
    use windows_sys::Win32::System::Memory::{
        VirtualQuery, MEMORY_BASIC_INFORMATION, MEM_COMMIT, MEM_PRIVATE, PAGE_EXECUTE,
        PAGE_EXECUTE_READ, PAGE_EXECUTE_READWRITE, PAGE_EXECUTE_WRITECOPY,
    };

    const SUSPICIOUS: &[&str] = &[
        "frida", "gum", "gadget", "detour", "easyhook", "mhook", "pin.dll",
    ];

    fn fnv1a(bytes: &[u8]) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        for &b in bytes {
            h ^= b as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }

    pub fn self_text_digest() -> u64 {
        unsafe {
            let mut hmod: HMODULE = core::ptr::null_mut();
            let anchor = self_text_digest as usize;
            let ok = GetModuleHandleExW(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                    | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                anchor as *const u16,
                &mut hmod,
            );
            if ok == 0 || hmod.is_null() {
                return 0;
            }
            text_section_digest(hmod as *const u8).unwrap_or(0)
        }
    }

    unsafe fn text_section_digest(base: *const u8) -> Option<u64> {
        if base.is_null() {
            return None;
        }
        let e_lfanew = *(base.add(0x3C) as *const i32);
        if e_lfanew <= 0 {
            return None;
        }
        let nt = base.offset(e_lfanew as isize);
        if *(nt as *const u32) != 0x0000_4550 {
            // "PE\0\0"
            return None;
        }
        let file_hdr = nt.add(4);
        let num_sections = *(file_hdr.add(2) as *const u16) as usize;
        let size_opt_hdr = *(file_hdr.add(16) as *const u16) as usize;
        let mut sec = file_hdr.add(20).add(size_opt_hdr);
        for _ in 0..num_sections {
            let name = core::slice::from_raw_parts(sec, 8);
            let vsize = *(sec.add(8) as *const u32) as usize;
            let vaddr = *(sec.add(12) as *const u32) as usize;
            if name.len() >= 5 && &name[..5] == b".text" && vsize > 0 {
                let bytes = core::slice::from_raw_parts(base.add(vaddr), vsize);
                return Some(fnv1a(bytes));
            }
            sec = sec.add(40);
        }
        None
    }

    pub fn augment(r: &mut TamperReport) {
        r.suspicious_modules = scan_modules();
        r.exec_private_regions = scan_exec_private();
    }

    fn scan_modules() -> Vec<String> {
        let mut out = Vec::new();
        unsafe {
            let snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, 0);
            if snap == INVALID_HANDLE_VALUE {
                return out;
            }
            let mut me: MODULEENTRY32W = core::mem::zeroed();
            me.dwSize = core::mem::size_of::<MODULEENTRY32W>() as u32;
            let mut ok = Module32FirstW(snap, &mut me);
            while ok != 0 {
                let name = u16z_to_string(&me.szModule);
                let lower = name.to_ascii_lowercase();
                if SUSPICIOUS.iter().any(|p| lower.contains(p)) {
                    out.push(name);
                }
                ok = Module32NextW(snap, &mut me);
            }
            CloseHandle(snap);
        }
        out
    }

    fn scan_exec_private() -> usize {
        let mut count = 0usize;
        let mut addr: usize = 0;
        unsafe {
            loop {
                let mut mbi: MEMORY_BASIC_INFORMATION = core::mem::zeroed();
                let got = VirtualQuery(
                    addr as *const _,
                    &mut mbi,
                    core::mem::size_of::<MEMORY_BASIC_INFORMATION>(),
                );
                if got == 0 {
                    break;
                }
                let exec = mbi.Protect
                    & (PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE
                        | PAGE_EXECUTE_WRITECOPY)
                    != 0;
                if mbi.State == MEM_COMMIT && mbi.Type == MEM_PRIVATE && exec {
                    count += 1;
                }
                let next = (mbi.BaseAddress as usize).wrapping_add(mbi.RegionSize);
                if next <= addr {
                    break;
                }
                addr = next;
            }
        }
        count
    }

    fn u16z_to_string(buf: &[u16]) -> String {
        let len = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
        String::from_utf16_lossy(&buf[..len])
    }
}

// ───────────────────────── 비(非)Windows 폴백 ─────────────────────────
#[cfg(not(all(windows, feature = "antitamper")))]
mod imp {
    use super::TamperReport;
    pub fn self_text_digest() -> u64 {
        0
    }
    pub fn augment(_r: &mut TamperReport) {}
}

// ───────────────────────── 텔레메트리(선택 feature) ─────────────────────────
#[cfg(feature = "telemetry")]
pub mod telemetry {
    //! 탐지 결과를 호스트에 **보고**만 한다(크래시 없음). 호스트(C#/Java)가 라이선스/세션을 판단.
    use super::TamperReport;
    use core::sync::atomic::{AtomicUsize, Ordering};

    /// `score`, `user_data` 를 받는 C ABI 콜백.
    pub type TamperCallback = extern "C" fn(score: u32, user_data: usize);

    static CB: AtomicUsize = AtomicUsize::new(0);
    static USER: AtomicUsize = AtomicUsize::new(0);

    /// 호스트가 탬퍼 보고 콜백을 등록.
    pub fn set_tamper_callback(cb: TamperCallback, user_data: usize) {
        USER.store(user_data, Ordering::SeqCst);
        CB.store(cb as usize, Ordering::SeqCst);
    }

    /// 리포트가 의심스러우면 콜백으로 score 를 보고.
    pub fn report(r: &TamperReport) {
        if !r.is_suspicious() {
            return;
        }
        let p = CB.load(Ordering::SeqCst);
        if p != 0 {
            let cb: TamperCallback = unsafe { core::mem::transmute(p) };
            cb(r.score(), USER.load(Ordering::SeqCst));
        }
    }
}

// ───────────────────────── 점진 저하 신호(선택 feature) ─────────────────────────
#[cfg(feature = "degrade")]
pub mod degrade {
    //! 점진 저하 "신호". 앱이 자기 품질 knob(반복수/샘플수 등)에 섞어 쓴다.
    //!
    //! ⚠ 양날: **오탐 시 정상 사용자 결과가 나빠진다.** 탐지 신뢰도가 높을 때만, 보수적으로.
    use super::TamperReport;

    /// clean 이면 0(무저하). 탬퍼 시 결정론적이지만 눈에 안 띄는 섭동 시드.
    pub fn perturbation(r: &TamperReport) -> u64 {
        if !r.is_suspicious() {
            return 0;
        }
        (r.text_digest ^ (r.score() as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)).rotate_left(17)
    }

    /// 품질 배수: clean=1.0, 탬퍼가 심할수록 0.7 까지 선형 감소. 앱이 knob 에 곱해 쓴다.
    pub fn quality_factor(r: &TamperReport) -> f64 {
        if !r.is_suspicious() {
            return 1.0;
        }
        let s = r.score().min(300) as f64;
        1.0 - (s / 300.0) * 0.3
    }
}
