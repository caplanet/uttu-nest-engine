//! build.rs 용 코드젠 헬퍼 (std 필요).
//!
//! VM 바이트코드는 **반드시 빌드타임에 생성·암호화**해 임베드해야 한다(런타임 빌드는 상수 누출).
//! 이 모듈이 그 과정을 한 줄로 만든다.
//!
//! # build.rs 예시
//! ```no_run
//! use cpguard::vm::{Asm, reg::*};
//! fn main() {
//!     let (key, op_seed) = (0x1122_3344_5566_7788, 0x00C0_FFEE);
//!     let mut a = Asm::new();
//!     a.input(R0, 0);
//!     a.ldi(R1, 0xDEAD_BEEF);   // 숨기고 싶은 임계 상수
//!     a.xor(R2, R0, R1);
//!     a.output(0, R2);
//!     a.halt();
//!     let out = std::path::Path::new(&std::env::var("OUT_DIR").unwrap()).join("prog.bin");
//!     cpguard::codegen::emit_encrypted_file(a, key, op_seed, out).unwrap();
//! }
//! ```
//! 그리고 소스에서:
//! ```ignore
//! static ENC: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/prog.bin"));
//! let prot = cpguard::vm::Protected::new(ENC.to_vec(), KEY, OP_SEED, 1);
//! ```

use crate::vm::{Asm, AsmError};
use std::fmt::Write as _;
use std::io;
use std::path::Path;

/// `Asm` 을 암호화·다형 인코딩하여 파일로 저장(보통 `OUT_DIR`).
pub fn emit_encrypted_file(
    asm: Asm,
    key: u64,
    op_seed: u64,
    path: impl AsRef<Path>,
) -> io::Result<()> {
    let enc = asm
        .finish_encrypted(key, op_seed)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;
    std::fs::write(path, &enc)
}

/// `Asm` 을 암호화하여 **Rust const 소스 문자열**로 반환(소스에 직접 임베드용).
///
/// `NAME`, `NAME_KEY`, `NAME_OP_SEED` 세 const 를 생성한다.
pub fn emit_rust_const(
    asm: Asm,
    key: u64,
    op_seed: u64,
    name: &str,
) -> Result<String, AsmError> {
    let enc = asm.finish_encrypted(key, op_seed)?;
    let mut s = String::new();
    let _ = writeln!(s, "pub const {name}: [u8; {}] = [", enc.len());
    for (i, b) in enc.iter().enumerate() {
        if i % 16 == 0 {
            s.push_str("    ");
        }
        let _ = write!(s, "0x{b:02x}, ");
        if i % 16 == 15 {
            s.push('\n');
        }
    }
    s.push_str("\n];\n");
    let _ = writeln!(s, "pub const {name}_KEY: u64 = {key:#018x};");
    let _ = writeln!(s, "pub const {name}_OP_SEED: u64 = {op_seed:#018x};");
    Ok(s)
}
