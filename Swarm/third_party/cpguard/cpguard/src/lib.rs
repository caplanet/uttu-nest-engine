//! # cpguard
//!
//! 소스를 가진 Rust 프로젝트 전반에 **재사용 가능한 컴파일타임 코드 보호 툴킷**.
//! (특정 프로젝트 전용이 아니라, 어떤 크레이트든 의존성으로 추가해 쓸 수 있다.)
//!
//! 세 가지 보호 수단을 제공한다:
//!
//! 1. [`vm`] — **마이크로-VM**. 핵심 정수/비트/f64 루틴을 바이트코드로 가상화한다. 바이너리에는
//!    인터프리터 + 암호화·순열·aliasing 된 바이트코드만 남아, 보호 루틴의 **정적 디컴파일이
//!    무의미**해진다. (가장 강함)
//! 2. [`macro@obfuscate`] — 함수/메서드에 **제어흐름 난독화**(FSM 디스패치 + 불투명 술어 + 가짜 분기).
//! 3. [`macro@obf_str`] — 문자열 리터럴 **컴파일타임 암호화**.
//!
//! build.rs 에서 바이트코드를 생성·임베드하는 헬퍼는 [`codegen`](crate::codegen) 참고(std 필요).
//!
//! ## 빠른 예시 — 임계 수식을 VM 으로 보호
//! ```
//! use cpguard::vm::{Asm, Protected, reg::*};
//! let key = 0x1122_3344_5566_7788;      // 복호 키
//! let op_seed = 0x00C0_FFEE;            // 빌드별 다형성 시드(오피코드/레지스터/aliasing)
//! let mut a = Asm::new();
//! a.input(R0, 0);
//! a.input(R1, 1);
//! a.mul(R2, R0, R1);
//! a.ldi(R3, 13);
//! a.rotl(R4, R0, R3);
//! a.xor(R5, R2, R4);
//! a.output(0, R5);
//! a.halt();
//! let enc = a.finish_encrypted(key, op_seed).unwrap();
//! let prot = Protected::new(enc, key, op_seed, 1);
//!
//! let a_in = 0xABCD_u64; let b_in = 0x1357_u64;
//! let expect = (a_in.wrapping_mul(b_in)) ^ a_in.rotate_left(13);
//! assert_eq!(prot.run(&[a_in, b_in]).unwrap()[0], expect);
//! ```
//!
//! ## 한계 (정직하게)
//! - 클라이언트에서 실행되는 모든 보호는 **동적 분석·메모리 덤프**로 우회 가능하다.
//!   cpguard 는 **정적 분석·자동 복원**을 어렵게 하는 것이 목적이다.
//! - VM 은 정수/비트/f64 스칼라 중심의 **작은 임계 루틴**용이다. 부동소수 벡터·힙·외부 호출·
//!   대형 함수 전체 가상화는 범위 밖이다. VM 은 네이티브보다 느리므로 **비핫 경로**에만 쓸 것.
//! - `#[obfuscate]` 는 의미를 보존한다(자유 함수/메서드).
//!
//! ## no_std
//! 기본 feature `std` 를 끄면(`default-features = false`) `alloc` 만으로 동작한다.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

// `#[obfuscate]` 매크로가 생성하는 `::cpguard::...` 경로가 이 크레이트 내부에서도
// 해석되도록 자기 자신을 crate 이름으로 참조한다.
extern crate self as cpguard;

pub mod obf;
pub mod vm;

#[cfg(feature = "std")]
pub mod codegen;

/// 런타임 안티탬퍼 탐지(후킹/인젝션). feature `antitamper`(기본 off), 실제 탐지는 Windows 전용.
/// 반응은 선택 feature `telemetry`(호스트 콜백 보고) / `degrade`(점진 저하 신호)로.
#[cfg(feature = "antitamper")]
pub mod antitamper;

pub use cpguard_macros::{obf_str, obfuscate};
pub use obf::{deobf, never, opaque_next, opaque_seed, opaque_true};
pub use vm::{reg, Asm, AsmError, Label, Protected, VmError};
