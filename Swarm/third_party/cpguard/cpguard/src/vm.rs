//! 마이크로-VM: 핵심 정수/비트/f64 루틴을 바이트코드로 "가상화"한다. no_std(alloc) 호환.
//!
//! 보호 계층:
//! - **키스트림 XOR 암호화**: 정적 이미지에 평문 바이트코드가 없다.
//! - **오피코드 aliasing(핸들러 셔플)**: 한 연산이 빌드마다 다른 바이트로, 또 같은 빌드 안에서도
//!   여러 바이트로 인코딩된다 → 패턴 매칭으로 "이 바이트 = 이 연산" 을 못 잡는다.
//! - **레지스터 치환**: 레지스터 인덱스 바이트도 빌드별로 재배치된다.
//! 모두 `op_seed` 하나에서 유도되므로 API 는 그대로다.
//!
//! # 한계
//! - 라이선스/무결성 체크, 키·시드 유도, 체크섬/해시, 작은 임계 수식·판정에 적합.
//!   힙·외부 호출·대형 함수 전체 가상화는 범위 밖. VM 은 느리므로 **비핫 경로**에만.
//! - 런타임 메모리 덤프로 레지스터/스크래치 값은 관찰될 수 있다(정적 복원 방어 전용).

use alloc::vec::Vec;
use core::fmt;

// ───────────────────────── 논리 오피코드 ─────────────────────────
pub(crate) const OP_HALT: u8 = 0x00;
pub(crate) const OP_LDI: u8 = 0x01;
pub(crate) const OP_MOV: u8 = 0x02;
pub(crate) const OP_ADD: u8 = 0x03;
pub(crate) const OP_SUB: u8 = 0x04;
pub(crate) const OP_MUL: u8 = 0x05;
pub(crate) const OP_UDIV: u8 = 0x06;
pub(crate) const OP_UMOD: u8 = 0x07;
pub(crate) const OP_AND: u8 = 0x08;
pub(crate) const OP_OR: u8 = 0x09;
pub(crate) const OP_XOR: u8 = 0x0A;
pub(crate) const OP_SHL: u8 = 0x0B;
pub(crate) const OP_SHR: u8 = 0x0C;
pub(crate) const OP_ROTL: u8 = 0x0D;
pub(crate) const OP_ROTR: u8 = 0x0E;
pub(crate) const OP_NOT: u8 = 0x0F;
pub(crate) const OP_NEG: u8 = 0x10;
pub(crate) const OP_ADDI: u8 = 0x11;
pub(crate) const OP_XORI: u8 = 0x12;
pub(crate) const OP_LD: u8 = 0x13;
pub(crate) const OP_ST: u8 = 0x14;
pub(crate) const OP_IN: u8 = 0x15;
pub(crate) const OP_OUT: u8 = 0x16;
pub(crate) const OP_JMP: u8 = 0x17;
pub(crate) const OP_JZ: u8 = 0x18;
pub(crate) const OP_JNZ: u8 = 0x19;
pub(crate) const OP_JLTU: u8 = 0x1A;
pub(crate) const OP_JEQ: u8 = 0x1B;
pub(crate) const OP_FADD: u8 = 0x20;
pub(crate) const OP_FSUB: u8 = 0x21;
pub(crate) const OP_FMUL: u8 = 0x22;
pub(crate) const OP_FDIV: u8 = 0x23;
pub(crate) const OP_FSQRT: u8 = 0x24;
pub(crate) const OP_FNEG: u8 = 0x25;
pub(crate) const OP_FABS: u8 = 0x26;
pub(crate) const OP_ITOF: u8 = 0x27;
pub(crate) const OP_FTOI: u8 = 0x28;
pub(crate) const OP_JFLT: u8 = 0x29;
pub(crate) const OP_JFEQ: u8 = 0x2A;

const N_OPS: usize = 0x2B; // 사용 중인 논리 오피코드 수(0x00..=0x2A)
const ALIASES: usize = 3; // 연산당 alias 바이트 수 (N_OPS*ALIASES <= 256)

/// 레지스터 이름 상수 (r0..r15).
pub mod reg {
    pub const R0: u8 = 0;
    pub const R1: u8 = 1;
    pub const R2: u8 = 2;
    pub const R3: u8 = 3;
    pub const R4: u8 = 4;
    pub const R5: u8 = 5;
    pub const R6: u8 = 6;
    pub const R7: u8 = 7;
    pub const R8: u8 = 8;
    pub const R9: u8 = 9;
    pub const R10: u8 = 10;
    pub const R11: u8 = 11;
    pub const R12: u8 = 12;
    pub const R13: u8 = 13;
    pub const R14: u8 = 14;
    pub const R15: u8 = 15;
}

const SCRATCH: usize = 256;

// ───────────────────────── 암호화 ─────────────────────────
fn keystream_xor(data: &mut [u8], key: u64) {
    let mut state = key ^ 0xA5A5_5A5A_DEAD_C0DE;
    for byte in data.iter_mut() {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        *byte ^= (state >> 56) as u8;
    }
}

// ───────────────────────── 다형성(Poly): aliasing + 레지스터 치환 ─────────────────────────
fn perm256(seed: u64) -> [u8; 256] {
    let mut p = [0u8; 256];
    for (i, s) in p.iter_mut().enumerate() {
        *s = i as u8;
    }
    let mut st = seed ^ 0x243F_6A88_85A3_08D3;
    for i in (1..256).rev() {
        st = st
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let j = ((st >> 33) as usize) % (i + 1);
        p.swap(i, j);
    }
    p
}

fn perm16(seed: u64) -> [u8; 16] {
    let mut p = [0u8; 16];
    for (i, s) in p.iter_mut().enumerate() {
        *s = i as u8;
    }
    let mut st = seed ^ 0x9E37_79B9_7F4A_7C15;
    for i in (1..16).rev() {
        st = st
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let j = ((st >> 33) as usize) % (i + 1);
        p.swap(i, j);
    }
    p
}

/// 빌드별 인코딩(오피코드 alias 표 + 레지스터 치환).
struct Poly {
    op_alias: [[u8; ALIASES]; N_OPS], // 논리op -> alias 바이트들
    op_inv: [u8; 256],                // 바이트 -> 논리op (0xFF=미사용)
    reg_perm: [u8; 16],               // 논리reg -> 물리 바이트
    reg_inv: [u8; 16],                // 물리 -> 논리reg
}

impl Poly {
    fn from_seed(op_seed: u64) -> Self {
        let pool = perm256(op_seed);
        let mut op_alias = [[0u8; ALIASES]; N_OPS];
        let mut op_inv = [0xFFu8; 256];
        for op in 0..N_OPS {
            for k in 0..ALIASES {
                let b = pool[op * ALIASES + k];
                op_alias[op][k] = b;
                op_inv[b as usize] = op as u8;
            }
        }
        let reg_perm = perm16(op_seed);
        let mut reg_inv = [0u8; 16];
        for (i, &v) in reg_perm.iter().enumerate() {
            reg_inv[v as usize] = i as u8;
        }
        Poly {
            op_alias,
            op_inv,
            reg_perm,
            reg_inv,
        }
    }

    fn identity() -> Self {
        let mut op_alias = [[0u8; ALIASES]; N_OPS];
        let mut op_inv = [0xFFu8; 256];
        for op in 0..N_OPS {
            op_alias[op] = [op as u8; ALIASES];
            op_inv[op] = op as u8;
        }
        let mut id16 = [0u8; 16];
        for (i, s) in id16.iter_mut().enumerate() {
            *s = i as u8;
        }
        Poly {
            op_alias,
            op_inv,
            reg_perm: id16,
            reg_inv: id16,
        }
    }
}

// ───────────────────────── 어셈블러 ─────────────────────────
/// 바이트코드 빌더. 라벨로 분기 대상을 지정하고 2-pass 로 해결한다.
#[derive(Default)]
pub struct Asm {
    code: Vec<u8>,
    op_positions: Vec<usize>,  // 오피코드 바이트 위치(alias 대상)
    reg_positions: Vec<usize>, // 레지스터 바이트 위치(치환 대상)
    fixups: Vec<(usize, usize)>,
    labels: Vec<Option<u32>>,
}

/// 분기 대상 라벨 핸들.
#[derive(Clone, Copy)]
pub struct Label(usize);

impl Asm {
    pub fn new() -> Self {
        Asm::default()
    }
    pub fn label(&mut self) -> Label {
        self.labels.push(None);
        Label(self.labels.len() - 1)
    }
    pub fn bind(&mut self, l: Label) {
        self.labels[l.0] = Some(self.code.len() as u32);
    }

    fn op(&mut self, opcode: u8) {
        self.op_positions.push(self.code.len());
        self.code.push(opcode);
    }
    fn rg(&mut self, reg: u8) {
        self.reg_positions.push(self.code.len());
        self.code.push(reg & 0x0f);
    }
    fn imm(&mut self, v: u64) {
        self.code.extend_from_slice(&v.to_le_bytes());
    }
    fn u16v(&mut self, v: u16) {
        self.code.extend_from_slice(&v.to_le_bytes());
    }
    fn target(&mut self, l: Label) {
        self.fixups.push((self.code.len(), l.0));
        self.code.extend_from_slice(&0u32.to_le_bytes());
    }

    pub fn ldi(&mut self, rd: u8, imm: u64) {
        self.op(OP_LDI);
        self.rg(rd);
        self.imm(imm);
    }
    pub fn fimm(&mut self, rd: u8, v: f64) {
        self.ldi(rd, v.to_bits());
    }
    pub fn mov(&mut self, rd: u8, rs: u8) {
        self.op(OP_MOV);
        self.rg(rd);
        self.rg(rs);
    }
    fn a3(&mut self, op: u8, rd: u8, ra: u8, rb: u8) {
        self.op(op);
        self.rg(rd);
        self.rg(ra);
        self.rg(rb);
    }
    fn a2(&mut self, op: u8, rd: u8, ra: u8) {
        self.op(op);
        self.rg(rd);
        self.rg(ra);
    }
    pub fn add(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_ADD, rd, ra, rb);
    }
    pub fn sub(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_SUB, rd, ra, rb);
    }
    pub fn mul(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_MUL, rd, ra, rb);
    }
    pub fn udiv(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_UDIV, rd, ra, rb);
    }
    pub fn umod(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_UMOD, rd, ra, rb);
    }
    pub fn and(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_AND, rd, ra, rb);
    }
    pub fn or(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_OR, rd, ra, rb);
    }
    pub fn xor(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_XOR, rd, ra, rb);
    }
    pub fn shl(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_SHL, rd, ra, rb);
    }
    pub fn shr(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_SHR, rd, ra, rb);
    }
    pub fn rotl(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_ROTL, rd, ra, rb);
    }
    pub fn rotr(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_ROTR, rd, ra, rb);
    }
    pub fn not(&mut self, rd: u8, ra: u8) {
        self.a2(OP_NOT, rd, ra);
    }
    pub fn neg(&mut self, rd: u8, ra: u8) {
        self.a2(OP_NEG, rd, ra);
    }
    pub fn addi(&mut self, rd: u8, ra: u8, imm: u64) {
        self.op(OP_ADDI);
        self.rg(rd);
        self.rg(ra);
        self.imm(imm);
    }
    pub fn xori(&mut self, rd: u8, ra: u8, imm: u64) {
        self.op(OP_XORI);
        self.rg(rd);
        self.rg(ra);
        self.imm(imm);
    }
    pub fn load(&mut self, rd: u8, addr: u8) {
        self.a2(OP_LD, rd, addr);
    }
    pub fn store(&mut self, addr: u8, rs: u8) {
        self.a2(OP_ST, addr, rs);
    }
    pub fn input(&mut self, rd: u8, idx: u16) {
        self.op(OP_IN);
        self.rg(rd);
        self.u16v(idx);
    }
    pub fn output(&mut self, idx: u16, rs: u8) {
        self.op(OP_OUT);
        self.u16v(idx);
        self.rg(rs);
    }
    pub fn jmp(&mut self, l: Label) {
        self.op(OP_JMP);
        self.target(l);
    }
    pub fn jz(&mut self, ra: u8, l: Label) {
        self.op(OP_JZ);
        self.rg(ra);
        self.target(l);
    }
    pub fn jnz(&mut self, ra: u8, l: Label) {
        self.op(OP_JNZ);
        self.rg(ra);
        self.target(l);
    }
    pub fn jltu(&mut self, ra: u8, rb: u8, l: Label) {
        self.op(OP_JLTU);
        self.rg(ra);
        self.rg(rb);
        self.target(l);
    }
    pub fn jeq(&mut self, ra: u8, rb: u8, l: Label) {
        self.op(OP_JEQ);
        self.rg(ra);
        self.rg(rb);
        self.target(l);
    }
    pub fn fadd(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_FADD, rd, ra, rb);
    }
    pub fn fsub(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_FSUB, rd, ra, rb);
    }
    pub fn fmul(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_FMUL, rd, ra, rb);
    }
    pub fn fdiv(&mut self, rd: u8, ra: u8, rb: u8) {
        self.a3(OP_FDIV, rd, ra, rb);
    }
    pub fn fsqrt(&mut self, rd: u8, ra: u8) {
        self.a2(OP_FSQRT, rd, ra);
    }
    pub fn fneg(&mut self, rd: u8, ra: u8) {
        self.a2(OP_FNEG, rd, ra);
    }
    pub fn fabs(&mut self, rd: u8, ra: u8) {
        self.a2(OP_FABS, rd, ra);
    }
    pub fn itof(&mut self, rd: u8, ra: u8) {
        self.a2(OP_ITOF, rd, ra);
    }
    pub fn ftoi(&mut self, rd: u8, ra: u8) {
        self.a2(OP_FTOI, rd, ra);
    }
    pub fn jflt(&mut self, ra: u8, rb: u8, l: Label) {
        self.op(OP_JFLT);
        self.rg(ra);
        self.rg(rb);
        self.target(l);
    }
    pub fn jfeq(&mut self, ra: u8, rb: u8, l: Label) {
        self.op(OP_JFEQ);
        self.rg(ra);
        self.rg(rb);
        self.target(l);
    }
    pub fn halt(&mut self) {
        self.op(OP_HALT);
    }

    /// 라벨 해결한 **평문/논리** 바이트코드 (테스트/도구용).
    pub fn finish(mut self) -> Result<Vec<u8>, AsmError> {
        self.resolve()?;
        Ok(self.code)
    }

    /// 라벨 해결 → 다형 인코딩(op_seed: aliasing + 레지스터 치환) → XOR(key). 배포용.
    pub fn finish_encrypted(mut self, key: u64, op_seed: u64) -> Result<Vec<u8>, AsmError> {
        self.resolve()?;
        let poly = Poly::from_seed(op_seed);
        for (i, &pos) in self.op_positions.iter().enumerate() {
            let logical = self.code[pos] as usize;
            self.code[pos] = poly.op_alias[logical][i % ALIASES];
        }
        for &pos in &self.reg_positions {
            let logical = (self.code[pos] & 0x0f) as usize;
            self.code[pos] = poly.reg_perm[logical];
        }
        keystream_xor(&mut self.code, key);
        Ok(self.code)
    }

    fn resolve(&mut self) -> Result<(), AsmError> {
        for (pos, label_id) in &self.fixups {
            let addr = self.labels[*label_id].ok_or(AsmError::UnboundLabel(*label_id))?;
            self.code[*pos..*pos + 4].copy_from_slice(&addr.to_le_bytes());
        }
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum AsmError {
    UnboundLabel(usize),
}
impl fmt::Display for AsmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AsmError::UnboundLabel(id) => write!(f, "unbound label #{id}"),
        }
    }
}
#[cfg(feature = "std")]
impl std::error::Error for AsmError {}

// ───────────────────────── 인터프리터 ─────────────────────────
#[derive(Debug, PartialEq, Eq)]
pub enum VmError {
    OutOfFuel,
    BadPc,
    BadOpcode(u8),
    BadOutput,
}
impl fmt::Display for VmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VmError::OutOfFuel => write!(f, "out of fuel"),
            VmError::BadPc => write!(f, "bad pc"),
            VmError::BadOpcode(o) => write!(f, "bad opcode {o:#x}"),
            VmError::BadOutput => write!(f, "bad output slot"),
        }
    }
}
#[cfg(feature = "std")]
impl std::error::Error for VmError {}

struct Cursor<'a> {
    code: &'a [u8],
    pc: usize,
    reg_inv: &'a [u8; 16],
}
impl<'a> Cursor<'a> {
    fn u8(&mut self) -> Result<u8, VmError> {
        let v = *self.code.get(self.pc).ok_or(VmError::BadPc)?;
        self.pc += 1;
        Ok(v)
    }
    fn reg(&mut self) -> Result<usize, VmError> {
        Ok(self.reg_inv[(self.u8()? & 0x0f) as usize] as usize)
    }
    fn u16(&mut self) -> Result<u16, VmError> {
        let b = self.code.get(self.pc..self.pc + 2).ok_or(VmError::BadPc)?;
        self.pc += 2;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }
    fn u32(&mut self) -> Result<u32, VmError> {
        let b = self.code.get(self.pc..self.pc + 4).ok_or(VmError::BadPc)?;
        self.pc += 4;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }
    fn u64(&mut self) -> Result<u64, VmError> {
        let b = self.code.get(self.pc..self.pc + 8).ok_or(VmError::BadPc)?;
        self.pc += 8;
        let mut a = [0u8; 8];
        a.copy_from_slice(b);
        Ok(u64::from_le_bytes(a))
    }
}

/// 보호된(암호화·다형) 바이트코드 프로그램.
pub struct Protected {
    enc: Vec<u8>,
    key: u64,
    op_seed: u64,
    n_out: usize,
    fuel: u64,
}

impl Protected {
    pub fn new(enc: Vec<u8>, key: u64, op_seed: u64, n_out: usize) -> Self {
        Protected {
            enc,
            key,
            op_seed,
            n_out,
            fuel: 1_000_000,
        }
    }
    pub fn with_fuel(mut self, fuel: u64) -> Self {
        self.fuel = fuel;
        self
    }
    pub fn run(&self, inputs: &[u64]) -> Result<Vec<u64>, VmError> {
        let mut code = self.enc.clone();
        keystream_xor(&mut code, self.key);
        let poly = Poly::from_seed(self.op_seed);
        interpret(&code, inputs, self.n_out, self.fuel, &poly)
    }
    pub fn run_f64(&self, inputs: &[f64]) -> Result<Vec<f64>, VmError> {
        let bits: Vec<u64> = inputs.iter().map(|x| x.to_bits()).collect();
        Ok(self.run(&bits)?.into_iter().map(f64::from_bits).collect())
    }
}

/// 평문/논리 바이트코드를 직접 실행(테스트/도구용, 다형성 없음).
pub fn run_bytecode(
    code: &[u8],
    inputs: &[u64],
    n_out: usize,
    fuel: u64,
) -> Result<Vec<u64>, VmError> {
    interpret(code, inputs, n_out, fuel, &Poly::identity())
}

#[cpguard_macros::obfuscate]
fn interpret(
    code: &[u8],
    inputs: &[u64],
    n_out: usize,
    fuel: u64,
    poly: &Poly,
) -> Result<Vec<u64>, VmError> {
    let mut r = [0u64; 16];
    let mut scratch = [0u64; SCRATCH];
    let mut out: Vec<u64> = alloc::vec![0u64; n_out];
    let mut cur = Cursor {
        code,
        pc: 0,
        reg_inv: &poly.reg_inv,
    };
    let mut steps = 0u64;

    loop {
        if steps >= fuel {
            return Err(VmError::OutOfFuel);
        }
        steps += 1;

        let raw = cur.u8()?;
        let op = poly.op_inv[raw as usize];
        match op {
            OP_HALT => break,
            OP_LDI => {
                let rd = cur.reg()?;
                r[rd] = cur.u64()?;
            }
            OP_MOV => {
                let (d, s) = (cur.reg()?, cur.reg()?);
                r[d] = r[s];
            }
            OP_ADD => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a].wrapping_add(r[b]);
            }
            OP_SUB => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a].wrapping_sub(r[b]);
            }
            OP_MUL => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a].wrapping_mul(r[b]);
            }
            OP_UDIV => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = if r[b] == 0 { 0 } else { r[a] / r[b] };
            }
            OP_UMOD => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = if r[b] == 0 { 0 } else { r[a] % r[b] };
            }
            OP_AND => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a] & r[b];
            }
            OP_OR => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a] | r[b];
            }
            OP_XOR => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a] ^ r[b];
            }
            OP_SHL => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a].wrapping_shl((r[b] & 63) as u32);
            }
            OP_SHR => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a].wrapping_shr((r[b] & 63) as u32);
            }
            OP_ROTL => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a].rotate_left((r[b] & 63) as u32);
            }
            OP_ROTR => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = r[a].rotate_right((r[b] & 63) as u32);
            }
            OP_NOT => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                r[d] = !r[a];
            }
            OP_NEG => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                r[d] = r[a].wrapping_neg();
            }
            OP_ADDI => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                let imm = cur.u64()?;
                r[d] = r[a].wrapping_add(imm);
            }
            OP_XORI => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                let imm = cur.u64()?;
                r[d] = r[a] ^ imm;
            }
            OP_LD => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                r[d] = scratch[(r[a] as usize) % SCRATCH];
            }
            OP_ST => {
                let (a, s) = (cur.reg()?, cur.reg()?);
                scratch[(r[a] as usize) % SCRATCH] = r[s];
            }
            OP_IN => {
                let rd = cur.reg()?;
                let idx = cur.u16()? as usize;
                r[rd] = inputs.get(idx).copied().unwrap_or(0);
            }
            OP_OUT => {
                let idx = cur.u16()? as usize;
                let rs = cur.reg()?;
                *out.get_mut(idx).ok_or(VmError::BadOutput)? = r[rs];
            }
            OP_JMP => {
                let t = cur.u32()? as usize;
                cur.pc = t;
            }
            OP_JZ => {
                let a = cur.reg()?;
                let t = cur.u32()? as usize;
                if r[a] == 0 {
                    cur.pc = t;
                }
            }
            OP_JNZ => {
                let a = cur.reg()?;
                let t = cur.u32()? as usize;
                if r[a] != 0 {
                    cur.pc = t;
                }
            }
            OP_JLTU => {
                let (a, b) = (cur.reg()?, cur.reg()?);
                let t = cur.u32()? as usize;
                if r[a] < r[b] {
                    cur.pc = t;
                }
            }
            OP_JEQ => {
                let (a, b) = (cur.reg()?, cur.reg()?);
                let t = cur.u32()? as usize;
                if r[a] == r[b] {
                    cur.pc = t;
                }
            }
            OP_FADD => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = (f64::from_bits(r[a]) + f64::from_bits(r[b])).to_bits();
            }
            OP_FSUB => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = (f64::from_bits(r[a]) - f64::from_bits(r[b])).to_bits();
            }
            OP_FMUL => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = (f64::from_bits(r[a]) * f64::from_bits(r[b])).to_bits();
            }
            OP_FDIV => {
                let (d, a, b) = (cur.reg()?, cur.reg()?, cur.reg()?);
                r[d] = (f64::from_bits(r[a]) / f64::from_bits(r[b])).to_bits();
            }
            OP_FSQRT => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                let x = f64::from_bits(r[a]);
                // std=하드웨어 sqrt, no_std=libm. 둘 다 IEEE 정확 반올림이라 결과가 동일하다.
                #[cfg(feature = "std")]
                let s = x.sqrt();
                #[cfg(not(feature = "std"))]
                let s = libm::sqrt(x);
                r[d] = s.to_bits();
            }
            OP_FNEG => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                // 부호 비트 토글(core 의 Neg 로도 되지만 no_std 안전하게 비트 연산).
                r[d] = r[a] ^ 0x8000_0000_0000_0000;
            }
            OP_FABS => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                // 부호 비트 클리어(std 불필요).
                r[d] = r[a] & 0x7FFF_FFFF_FFFF_FFFF;
            }
            OP_ITOF => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                r[d] = ((r[a] as i64) as f64).to_bits();
            }
            OP_FTOI => {
                let (d, a) = (cur.reg()?, cur.reg()?);
                r[d] = (f64::from_bits(r[a]) as i64) as u64;
            }
            OP_JFLT => {
                let (a, b) = (cur.reg()?, cur.reg()?);
                let t = cur.u32()? as usize;
                if f64::from_bits(r[a]) < f64::from_bits(r[b]) {
                    cur.pc = t;
                }
            }
            OP_JFEQ => {
                let (a, b) = (cur.reg()?, cur.reg()?);
                let t = cur.u32()? as usize;
                if f64::from_bits(r[a]) == f64::from_bits(r[b]) {
                    cur.pc = t;
                }
            }
            _ => return Err(VmError::BadOpcode(raw)),
        }
    }
    Ok(out)
}
