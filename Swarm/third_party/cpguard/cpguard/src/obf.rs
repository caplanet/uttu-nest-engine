//! 런타임 난독화 지원 루틴 (매크로가 생성한 코드가 호출). no_std 호환(alloc 필요).

use alloc::string::String;
use alloc::vec::Vec;
use core::hint::black_box;

/// 항상 `true` 지만 컴파일타임에 상수로 접히지 않는 불투명 술어.
#[inline(never)]
pub fn opaque_true() -> bool {
    let a = black_box(0x9E37_79B9_u64);
    let b = black_box(0x8521_u64);
    ((a | 1).wrapping_add(b.wrapping_mul(2)) & 1) == 1
}

/// `#[obfuscate]` FSM 의 시작 상태(불투명하게 0 을 반환).
#[inline(never)]
pub fn opaque_seed() -> u32 {
    let a = black_box(0xA5A5_u32);
    // (a ^ a) == 0 이지만 컴파일러는 증명 못 함.
    a ^ black_box(a)
}

/// `#[obfuscate]` FSM 의 다음 상태(불투명하게 `next` 를 반환).
#[inline(never)]
pub fn opaque_next(_cur: u32, next: u32) -> u32 {
    let z = black_box(0u32);
    next.wrapping_add(z)
}

/// 도달해서는 안 되는 가짜 분기의 종단. 반환 타입 `!`.
#[cfg(feature = "std")]
#[inline(never)]
#[cold]
pub fn never() -> ! {
    let _ = black_box(0xDEAD_BEEF_u64);
    std::process::abort()
}

/// no_std: abort 가 없으므로 영구 루프(실제로는 도달하지 않음).
#[cfg(not(feature = "std"))]
#[inline(never)]
#[cold]
pub fn never() -> ! {
    let _ = black_box(0xDEAD_BEEF_u64);
    loop {
        core::hint::spin_loop();
    }
}

/// `obf_str!` 가 생성한 암호화 바이트를 런타임에 복호화한다.
#[doc(hidden)]
pub fn deobf(enc: &[u8], key: u64) -> String {
    let v: Vec<u8> = enc
        .iter()
        .enumerate()
        .map(|(i, &b)| b ^ ((key >> ((i % 8) * 8)) as u8) ^ (i as u8).wrapping_mul(31))
        .collect();
    String::from_utf8_lossy(&v).into_owned()
}
