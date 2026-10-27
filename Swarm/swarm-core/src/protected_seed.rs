//! 하드닝: 도메인별 시드 유도(`base ^ MAGIC`)를 cpguard 마이크로-VM 으로 가상화한다.
//!
//! 도메인 태그 매직 상수는 네이티브 코드에 평문으로 존재하지 않는다 — 아래 `SEED_ENC` 는
//! 빌드 호스트에서 암호화·순열되어 생성된 바이트코드이고(상수는 그 안에만), 런타임에만 VM
//! 인터프리터가 복원해 실행한다. run 당 극소수 회 호출되므로 VM 비용은 무시할 만하다.
//!
//! `SEED_ENC` 재생성: `cpguard` 워크스페이스의 `cargo run --example gen_seed`.

use cpguard::vm::Protected;
use std::sync::OnceLock;

// cpguard/examples/gen_seed.rs 로 생성(매직 상수 평문 없음).
// cpguard v0.2 인코딩(오피코드 aliasing + 레지스터 치환 포함)으로 재생성.
pub(crate) const SEED_ENC: [u8; 41] = [
    0xc6, 0x18, 0x73, 0xf7, 0xc8, 0x31, 0x3f, 0x04, 0x00, 0xa7, 0x57, 0x71, 0xc6, 0xa2, 0x98, 0x17,
    0x05, 0xde, 0xb6, 0xba, 0x54, 0x2c, 0x98, 0xc8, 0x80, 0xf3, 0x86, 0xe3, 0x28, 0xdb, 0xa8, 0xf3,
    0x6b, 0x21, 0xdb, 0xda, 0xac, 0x64, 0xc8, 0x99, 0x19,
];
pub(crate) const SEED_KEY: u64 = 0x7b16_9d4c_2e58_a1f3;
pub(crate) const SEED_OP_SEED: u64 = 0x33aa_55cc_1177_ee22;

fn program() -> &'static Protected {
    static PROG: OnceLock<Protected> = OnceLock::new();
    PROG.get_or_init(|| Protected::new(SEED_ENC.to_vec(), SEED_KEY, SEED_OP_SEED, 2))
}

fn derive(base: u64) -> [u64; 2] {
    // 빌드타임에 고정 생성된 결정론적 바이트코드라 런타임 실패가 없다.
    // (평문 매직 폴백을 두면 상수가 다시 노출되므로 두지 않는다.)
    let out = program().run(&[base]).expect("seed vm");
    [out[0], out[1]]
}

/// candidate-duel 결정론적 시드. (기존: `base ^ <duel 태그>`)
#[inline]
pub fn seed_duel(base: u64) -> u64 {
    derive(base)[0]
}

/// nfp-lbf 결정론적 시드. (기존: `base ^ <nfp 태그>`)
#[inline]
pub fn nfp_lbf(base: u64) -> u64 {
    derive(base)[1]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_original_derivation() {
        // 원본 상수는 테스트에만 등장(릴리스 바이너리엔 없음).
        const DUEL: u64 = 0x5345_4544_4455_454C;
        const NFP: u64 = 0x4E46_505F_4C42_4600;
        for base in [0u64, 1, 41, 0xDEAD_BEEF, u64::MAX, 0x0123_4567_89AB_CDEF] {
            assert_eq!(seed_duel(base), base ^ DUEL, "duel base={base:#x}");
            assert_eq!(nfp_lbf(base), base ^ NFP, "nfp base={base:#x}");
        }
    }
}
