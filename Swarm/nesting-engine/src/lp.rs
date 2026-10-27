//! 자체 LP 표현 — 압축 LP(§5)와 잔여조각 LP(§6)가 함께 쓴다.
//!
//! # 계약 (§12.1.22 AJ8, §12.1.24 AL2)
//!
//! 원천은 외부 LP 라이브러리(Clp 계열)를 정적 링크하고 래퍼 네 개로 행을 쌓는다. 래퍼와
//! 목적값 환산식(A×B−C)은 옮기지 않는다(AC5·AC7). 옮기는 것은 값 규약뿐이고, 풀이는
//! 워크스페이스가 이미 쓰는 `microlp` 에 맡긴다.
//!
//! - 행 API: `new_row → add_coef ×N → set_bounds(행, 하한, 상한)`. 3번째 인자가 하한이다.
//!   행 `None`(원천의 −1) = 마지막 행.
//! - `add_coef` 는 `|v| < 1e−6` 계수를 조용히 버린다. 축에 평행한 모서리에서 행의 항이
//!   줄어드는 것이 정상이다 — 행 자체와 경계는 남는다.
//! - 모서리에서 나온 행은 하한이 전부 `FREE_LOWER = −1e30` 이고 상한만 구속한다.
//! - 목적: 최소화, 길이 변수에만 0.8 을 **한 번** 곱한다, 오프셋 없음. 폭은 해 벡터의 L 을
//!   그대로 읽는다 — 목적값 `0.8·L` 은 진단용이다.
//!
//! # 모서리 반평면 (§5.3)
//!
//! 모서리 `(x1,y1)→(x2,y2)` 와 현재 상대 위치 `r` 에서 `dy=(y2−y1)K`, `negdx=(x1−x2)K`,
//! `c=(y2·x1−x2·y1)K`, `d=c−rx·dy−ry·negdx` (K=0.01). 대수적으로 `d = K·cross(p2−p1, r−p1)`
//! 이어서 정수 외적으로 정확히 계산한다. 계수 4개의 자리는 (i.x, j.x, i.y, j.y) 다(제15차 Z3).
//! 1행 `(−dy, +dy, −negdx, +negdx)` 는 `n·(Δ_j − Δ_i)`, `n = (dy, negdx)` 이고
//! `d(r + Δ) = d(r) − n·Δ` 가 성립한다.
//!
//! # 신뢰영역 우변 — 지시서가 스스로 모순된다
//!
//! - [`TrustScheme::Spec25`]: 제25차 3분기(§5.10, AL1). d>0.001 → (2d, −1e−5),
//!   |d|≤0.001 → (d, −2d), d<−0.001 또는 NaN → (d, d).
//! - [`TrustScheme::AsmM1`]: 같은 절이 근거로 인용한 기계어(2326–2344행)를 x87 스택 규칙대로
//!   따라가면 나오는 값. 제8차 표이자 §5.9 표, 6상 솔버 검산을 통과한 유일한 값이다.
//!   d>0.001 → (2d, −1e−5), |d|≤0.001 또는 NaN → (1e−5, 1e−5), d<−0.001 → (1e−5, −2d).
//! - [`TrustScheme::Sound`] (기본): 기하적으로 안전한 판. 현재 `r` 이 모서리의 비금지 쪽
//!   (d ≥ 0)이면 1행만 넣고 상한을 d 로 둔다 — 상대 이동이 그 모서리의 직선을 넘지 못한다.
//!   금지 쪽(d < 0) 모서리는 직선 경로가 NFP 안으로 들어가는 입구가 될 수 없어서 행이 필요
//!   없다(직선은 한 직선을 한 번만 건넌다). 도달 범위 안의 모든 모서리에 이렇게 걸면 해의
//!   끝점은 언제나 NFP 밖이다 — 실수 해 기준이다. 격자 절사 뒤에도 성립하는 것은 한 축 패스
//!   ([`sound_axis_row`], 정수 경계)뿐이고, 2D 이동(동시 패스·잔여조각 LP)은 절사가 경사
//!   모서리를 한 격자 넘을 수 있어 적용 뒤 겹침 검사가 받아낸다.
//!
//! 두 지시서 판은 1행 상한 2d 로 "현재 간격만큼 선을 넘어도 된다"는 신뢰영역을 쓴다. 겹칠 수
//! 있고, 그러면 스텝 반감·롤백(§5.7 함정 6)이 받아낸다. 2행(`+dy,+B,+negdx,+A`, A·B 는 좌표
//! 합)·3·4행·조건부 5행은 [`SpecRows`] 로 켠다.

use crate::geom::Pt;
use microlp::{ComparisonOp, LinearExpr, OptimizationDirection, Problem, SolveOutcome, Variable};
use std::time::Duration;

/// 모서리 방향·상수항 배율 (`K_SCALE`).
pub const K_SCALE: f64 = 0.01;
/// 이격 판정 임계 (`TRUST_CLEAR`).
pub const TRUST_CLEAR: f64 = 0.001;
/// 접촉 대역 하한 (`TRUST_HI`).
pub const TRUST_HI: f64 = -0.001;
/// 기본 상한 (`NEAR_ZERO`).
pub const NEAR_ZERO: f64 = 1e-5;
/// 이격 시 2행 상한 (`NEAR_ZERO_N`).
pub const NEAR_ZERO_N: f64 = -1e-5;
/// 접촉·겹침 배율 (`TRUST_F`).
pub const TRUST_F: f64 = -2.0;
/// 조건부 5행 경로 B 하한 (−K_SCALE). `TRUST_HI` 와 다른 상수다.
pub const FIFTH_ROW_THR: f64 = -0.01;
/// 행 하한 (`FREE_LOWER`).
pub const FREE_LOWER: f64 = -1e30;
/// ±무한 대용.
pub const INF: f64 = 1e30;
/// 계수 버림 임계 (`add_coef`).
pub const COEF_DROP: f64 = 1e-6;
/// 길이 변수 목적계수 — 한 번만 곱한다 (AL2).
pub const OBJ_LEN: f64 = 0.8;

/// 이 값 이상이면 무한으로 본다.
const INF_EDGE: f64 = 1e29;

#[derive(Clone, Debug)]
pub struct Row {
    pub coefs: Vec<(usize, f64)>,
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug, Default)]
pub struct Lp {
    col_lo: Vec<f64>,
    col_hi: Vec<f64>,
    obj: Vec<f64>,
    rows: Vec<Row>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LpStatus {
    /// 원천 솔버 상태 0.
    Optimal,
    /// 원천 솔버 상태 3 — 시간 상한. 상태 `0x100`(부분 완료)으로 옮긴다.
    LimitReached,
    Infeasible,
    Unbounded,
    Error,
}

#[derive(Clone, Debug)]
pub struct Solved {
    pub status: LpStatus,
    pub x: Vec<f64>,
    /// 목적값(0.8 이 곱해진 값) — 진단용.
    pub objective: f64,
    pub message: String,
}

impl Solved {
    fn fail(status: LpStatus, message: impl Into<String>) -> Self {
        Self { status, x: Vec::new(), objective: f64::NAN, message: message.into() }
    }
}

fn to_bound(v: f64) -> f64 {
    if v <= -INF_EDGE {
        f64::NEG_INFINITY
    } else if v >= INF_EDGE {
        f64::INFINITY
    } else {
        v
    }
}

impl Lp {
    /// 열 `ncols` 개, 경계는 전부 (−∞, ∞), 목적계수 0.
    pub fn new(ncols: usize) -> Self {
        Self {
            col_lo: vec![-INF; ncols],
            col_hi: vec![INF; ncols],
            obj: vec![0.0; ncols],
            rows: Vec::new(),
        }
    }

    /// 행 버퍼 예약 (§5.1 용량 산식 값). 실제 행은 필요한 만큼 늘어난다 — 경계 없는 쓰기는
    /// 하지 않는다(E6).
    pub fn reserve_rows(&mut self, capacity: usize) {
        self.rows.reserve(capacity);
    }

    pub fn ncols(&self) -> usize {
        self.col_lo.len()
    }

    pub fn add_col(&mut self, lo: f64, hi: f64, obj: f64) -> usize {
        self.col_lo.push(lo);
        self.col_hi.push(hi);
        self.obj.push(obj);
        self.col_lo.len() - 1
    }

    pub fn set_col_bounds(&mut self, col: usize, lo: f64, hi: f64) {
        self.col_lo[col] = lo;
        self.col_hi[col] = hi;
    }

    pub fn set_obj(&mut self, col: usize, c: f64) {
        self.obj[col] = c;
    }

    pub fn new_row(&mut self) {
        self.rows.push(Row { coefs: Vec::new(), lower: FREE_LOWER, upper: INF });
    }

    /// `|v| < 1e−6` 이면 아무것도 하지 않는다. 열 번호는 받은 그대로 저장한다.
    pub fn add_coef(&mut self, col: usize, v: f64) {
        if v.abs() < COEF_DROP {
            return;
        }
        debug_assert!(col < self.col_lo.len(), "열 {col} 이 범위 밖이다");
        self.rows.last_mut().expect("new_row 를 먼저 불러야 한다").coefs.push((col, v));
    }

    /// `row == None` 이면 마지막 행.
    pub fn set_bounds(&mut self, row: Option<usize>, lower: f64, upper: f64) {
        let r = row.unwrap_or(self.rows.len() - 1);
        self.rows[r].lower = lower;
        self.rows[r].upper = upper;
    }

    pub fn rows(&self) -> &[Row] {
        &self.rows
    }

    /// 매 반복 새 솔버 객체를 만들고 남은 시간을 상한으로 준다 (§5.7, §5.10).
    pub fn solve(&self, limit: Duration) -> Solved {
        if limit.is_zero() {
            return Solved::fail(LpStatus::LimitReached, "남은 시간 0");
        }
        let mut p = Problem::new(OptimizationDirection::Minimize);
        p.set_time_limit(limit);
        let vars: Vec<Variable> = (0..self.col_lo.len())
            .map(|c| p.add_var(self.obj[c], (to_bound(self.col_lo[c]), to_bound(self.col_hi[c]))))
            .collect();
        for row in &self.rows {
            if row.lower.is_nan() || row.upper.is_nan() {
                continue; // 상한 NaN 행은 무효다 (§5.9 [바]).
            }
            let lo_free = row.lower <= -INF_EDGE;
            let hi_free = row.upper >= INF_EDGE;
            if lo_free && hi_free {
                continue;
            }
            // 같은 열이 두 번 오면 합친다 — 솔버는 한 식에 같은 변수를 두 번 받지 않는다.
            let mut coefs = row.coefs.clone();
            coefs.sort_by_key(|c| c.0);
            let mut merged: Vec<(usize, f64)> = Vec::with_capacity(coefs.len());
            for (c, v) in coefs {
                match merged.last_mut() {
                    Some(last) if last.0 == c => last.1 += v,
                    _ => merged.push((c, v)),
                }
            }
            if merged.is_empty() {
                if (!lo_free && row.lower > 0.0) || (!hi_free && row.upper < 0.0) {
                    return Solved::fail(LpStatus::Infeasible, "항이 모두 버려진 행의 경계가 0 을 배제한다");
                }
                continue;
            }
            let expr: LinearExpr = merged.iter().map(|&(c, v)| (vars[c], v)).collect();
            if !lo_free && !hi_free && row.lower == row.upper {
                p.add_constraint(expr, ComparisonOp::Eq, row.upper);
            } else {
                if !hi_free {
                    p.add_constraint(expr.clone(), ComparisonOp::Le, row.upper);
                }
                if !lo_free {
                    p.add_constraint(expr, ComparisonOp::Ge, row.lower);
                }
            }
        }
        match p.solve() {
            Ok(SolveOutcome::Solution(sol)) => Solved {
                status: LpStatus::Optimal,
                x: vars.iter().map(|&v| sol.var_value(v)).collect(),
                objective: sol.objective(),
                message: String::new(),
            },
            Ok(SolveOutcome::Interrupted(_)) => Solved::fail(LpStatus::LimitReached, "시간 상한"),
            Err(microlp::Error::Infeasible) => Solved::fail(LpStatus::Infeasible, "불능"),
            Err(microlp::Error::Unbounded) => Solved::fail(LpStatus::Unbounded, "무계"),
            Err(e) => Solved::fail(LpStatus::Error, format!("{e:?}")),
        }
    }
}

/// NFP 모서리 하나의 §5.3 양. `p1→p2` 는 지시서 규약(d > 0 = 이격) 방향이어야 한다.
#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub p1: Pt,
    pub p2: Pt,
    pub dy: f64,
    pub negdx: f64,
    pub c: f64,
}

impl Edge {
    pub fn new(p1: Pt, p2: Pt) -> Edge {
        let dy = (p2.y - p1.y) as f64 * K_SCALE;
        let negdx = (p1.x - p2.x) as f64 * K_SCALE;
        let c = (p2.y as i128 * p1.x as i128 - p2.x as i128 * p1.y as i128) as f64 * K_SCALE;
        Edge { p1, p2, dy, negdx, c }
    }

    /// `cross(p2−p1, r−p1)` — 정수로 정확하다. 부호가 곧 d 의 부호다.
    pub fn cross(&self, r: Pt) -> i128 {
        (self.p2.x - self.p1.x) as i128 * (r.y - self.p1.y) as i128
            - (self.p2.y - self.p1.y) as i128 * (r.x - self.p1.x) as i128
    }

    /// `d = c − rx·dy − ry·negdx = K·cross(p2−p1, r−p1)`.
    pub fn d(&self, r: Pt) -> f64 {
        self.cross(r) as f64 * K_SCALE
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrustScheme {
    Sound,
    AsmM1,
    Spec25,
}

/// 지시서 두 판의 (1행 상한 = `[esp]`/local_54, 2행 상한 = ATR/dVar5).
pub fn trust_rhs(d: f64, scheme: TrustScheme) -> (f64, f64) {
    match scheme {
        TrustScheme::Spec25 => {
            if d > TRUST_CLEAR {
                (2.0 * d, NEAR_ZERO_N)
            } else if d >= TRUST_HI {
                (d, d * TRUST_F)
            } else {
                (d, d) // d < −0.001 또는 NaN
            }
        }
        TrustScheme::AsmM1 | TrustScheme::Sound => {
            if d.is_nan() {
                (NEAR_ZERO, NEAR_ZERO)
            } else if d > TRUST_CLEAR {
                (2.0 * d, NEAR_ZERO_N)
            } else if d >= TRUST_HI {
                (NEAR_ZERO, NEAR_ZERO)
            } else {
                (NEAR_ZERO, d * TRUST_F)
            }
        }
    }
}

/// 한 축만 움직이는 패스(§5.7 축 분해)의 안전 행. `axis` 0 = x 패스(계수 dy), 1 = y 패스
/// (계수 negdx). 넣은 행 수(0 또는 1)를 돌려준다.
///
/// 1행 `a·(Δ_j − Δ_i) ≤ d` 는 상대 이동 `Δrel ≤ d/a` (a > 0) 또는 `Δrel ≥ d/a` (a < 0) 다.
/// 조각마다 이동을 절사(§5.7 함정 5)하면 상대 이동이 분수 경계를 최대 1격자 넘을 수 있어서,
/// 경계를 격자 정수로 내린다(a > 0 은 내림, a < 0 은 올림). 절사는 단조라서 `Δ_j − Δ_i ≤ m`
/// (m 정수) 이면 `trunc Δ_j − trunc Δ_i ≤ m` 도 성립한다 — 적용한 배치가 모서리를 넘지 않는다.
/// 경계는 외적(정수)으로 정확히 계산한다.
pub fn sound_axis_row(lp: &mut Lp, slots: [Option<usize>; 4], e: &Edge, r: Pt, axis: usize) -> usize {
    sound_axis_row_scaled(lp, slots, e, r, axis, 1.0)
}

/// ★ §12.1.40 AU140.14 — `sound_axis_row` 의 «제약 배율» 판.
///
/// `scale` 을 `CONSTR_SCALE = 0.5` 로 주면 우변(= 이 축으로 모서리까지 다가갈 수 있는
/// 거리 `a·m`)이 절반이 된다 — 즉 행이 **조여진다**(«완화» 가 아니다). 원천
/// `FUN_1004a140` 이 `_DAT_10154458 = 0.5` 를 제약에 곱하는 자리와 같은지는 미확인이다.
/// `1.0` 이면 종전 동작과 완전히 동일하다.
pub fn sound_axis_row_scaled(
    lp: &mut Lp,
    slots: [Option<usize>; 4],
    e: &Edge,
    r: Pt,
    axis: usize,
    scale: f64,
) -> usize {
    let cross = e.cross(r);
    if cross < 0 {
        return 0; // 금지 쪽 — 직선 경로의 입구가 아니다
    }
    let comp: i128 = if axis == 0 { (e.p2.y - e.p1.y) as i128 } else { (e.p1.x - e.p2.x) as i128 };
    if comp == 0 {
        return 0; // 이 축 이동과 무관하다 (0 ≤ d 는 이미 성립)
    }
    let m = if comp > 0 { cross.div_euclid(comp) } else { -(cross.div_euclid(-comp)) };
    let a = comp as f64 * K_SCALE;
    let (si, sj) = if axis == 0 { (slots[0], slots[1]) } else { (slots[2], slots[3]) };
    lp.new_row();
    if let Some(c) = si {
        lp.add_coef(c, -a);
    }
    if let Some(c) = sj {
        lp.add_coef(c, a);
    }
    lp.set_bounds(None, FREE_LOWER, a * m as f64 * scale);
    1
}

/// 지시서 판에서 1행 말고 더 넣을 행.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SpecRows {
    /// 2행 `(+dy, +B, +negdx, +A)`, A = x_B + y_A, B = y_B + x_A (제10차 U2·U4).
    pub row2: bool,
    /// 3·4행 — 좌표 쌍, 상한 재계산 (§5.9).
    pub rows34: bool,
    /// 조건부 5행 — 쌍 보조열 두 개 (제19·23차).
    pub fifth: bool,
    /// ★ §12.1.40 AU140.14 — 원천 «제약 배율» `_DAT_10154458 = 0.5` 를 `Sound` 경로
    /// 우변에 적용한다(`FUN_1004a140`). 우리 비겹침 제약이 원천보다 2배 빡빡해
    /// «미세 이동» 이 막히던 문제를 푼다. 기본 `false` 는 종전 동작 보존.
    pub constraint_relax: bool,
}

/// 모서리 하나의 행들을 넣는다. 넣은 행 수를 돌려준다. `slots` 는 (i.x, j.x, i.y, j.y) 열이고
/// `None` 은 고정 조각(계수를 넣지 않는다).
pub fn edge_rows(
    lp: &mut Lp,
    scheme: TrustScheme,
    extra: SpecRows,
    slots: [Option<usize>; 4],
    e: &Edge,
    r: Pt,
    aux: Option<[usize; 2]>,
) -> usize {
    let put = |lp: &mut Lp, coefs: [f64; 4]| {
        for (s, v) in slots.iter().zip(coefs) {
            if let Some(c) = s {
                lp.add_coef(*c, v);
            }
        }
    };
    let d = e.d(r);
    if scheme == TrustScheme::Sound {
        if e.cross(r) < 0 {
            return 0;
        }
        lp.new_row();
        put(lp, [-e.dy, e.dy, -e.negdx, e.negdx]);
        // ★ §12.1.40 AU140.14 — 원천 «제약 배율» `_DAT_10154458 = 0.5` (`FUN_1004a140`).
        //
        //   AU136·AU137 이 이 상수를 판독해 두고도 **구현에서 쓰지 않았다**(미사용 상수였다).
        //   AU140.11~AU140.13 실측이 «LP 가 폭 결정 조각을 1mm 도 못 움직인다»(`dx=0`,
        //   `하한=-750`)를 보였고, 그 원인은 **우리 비겹침 제약이 원천보다 2배 빡빡**한 것이다.
        //
        //   원천은 우변을 `0.5` 배로 **완화**해 «미세 이동» 을 허용한다. 이 여유가 있어야
        //   `L`(폭) 을 조금씩 줄이는 해가 성립한다. 종전 동작은 `constraint_relax == false`
        //   로 보존한다.
        //
        // ★ 주의 — 이름과 달리 이 배율은 행을 **조인다**. 행은 `Σ(계수·이동) ≤ d` 이고
        //   `d ≥ 0` 은 «현재 이격» 이다: 우변이 접근 가능 거리다. `0.5·d` 면 모서리까지
        //   절반만 다가갈 수 있으므로 허용 이동이 **줄어든다**(완화가 아니다). 원천의
        //   0.5 가 어느 항에 붙는지는 재확인이 필요하다. 실측에서 값 변화가 없던 이유다.
        let rhs = if extra.constraint_relax {
            d * crate::compact::CONSTR_SCALE
        } else {
            d
        };
        lp.set_bounds(None, FREE_LOWER, rhs);
        return 1;
    }
    let (u1, u2) = trust_rhs(d, scheme);
    let mut n = 1;
    lp.new_row();
    put(lp, [-e.dy, e.dy, -e.negdx, e.negdx]);
    lp.set_bounds(None, FREE_LOWER, u1);
    if extra.row2 {
        let a = (e.p2.x + e.p1.y) as f64;
        let b = (e.p2.y + e.p1.x) as f64;
        lp.new_row();
        put(lp, [e.dy, b, e.negdx, a]);
        lp.set_bounds(None, FREE_LOWER, u2);
        n += 1;
    }
    if extra.rows34 {
        let dv = r.y as f64 * e.dy - r.x as f64 * e.negdx;
        let mut u3 = (e.negdx * e.p1.x as f64 + dv) - e.dy * e.p1.y as f64;
        if u3 <= 0.0 {
            u3 = NEAR_ZERO;
        }
        let mut u4 = e.dy * e.p2.y as f64 - (e.negdx * e.p2.x as f64 + dv);
        if u4 < 0.0 {
            u4 = NEAR_ZERO;
        }
        lp.new_row();
        put(lp, [-e.negdx, e.negdx, -e.dy, e.dy]);
        lp.set_bounds(None, FREE_LOWER, u3);
        lp.new_row();
        put(lp, [e.negdx, -e.negdx, e.dy, -e.dy]);
        lp.set_bounds(None, FREE_LOWER, u4);
        n += 2;
    }
    if extra.fifth
        && let Some([ca, cb]) = aux
    {
        // v1, v2 = 시작 꼭짓점 정수 좌표, v3 = 2행의 A 와 같은 누적값 (제19·23차, 검토 지적).
        let (v1, v2) = (e.p1.x as f64, e.p1.y as f64);
        let v3 = (e.p2.x + e.p1.y) as f64;
        let coef = if d > TRUST_CLEAR {
            Some(-1.0)
        } else if d >= FIFTH_ROW_THR {
            Some(1.0)
        } else {
            None // d < −0.01 또는 NaN — 행 없음
        };
        if let Some(s) = coef {
            lp.new_row();
            lp.add_coef(ca, s * v1);
            lp.add_coef(cb, s * v2);
            lp.set_bounds(None, FREE_LOWER, -v3);
            n += 1;
        }
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-12
    }

    #[test]
    fn add_coef_drops_tiny_values_strictly() {
        let mut lp = Lp::new(4);
        lp.new_row();
        lp.add_coef(0, 5e-7);
        lp.add_coef(1, -0.0);
        lp.add_coef(2, -9.99e-7);
        lp.add_coef(3, 1e-6);
        lp.set_bounds(None, FREE_LOWER, 1.0);
        assert_eq!(lp.rows()[0].coefs, vec![(3, 1e-6)]);
        assert_eq!(lp.rows()[0].lower, FREE_LOWER);
    }

    #[test]
    fn set_bounds_none_targets_the_last_row() {
        let mut lp = Lp::new(1);
        lp.new_row();
        lp.new_row();
        lp.set_bounds(None, -1.0, 2.0);
        lp.set_bounds(Some(0), 3.0, 4.0);
        assert_eq!((lp.rows()[1].lower, lp.rows()[1].upper), (-1.0, 2.0));
        assert_eq!((lp.rows()[0].lower, lp.rows()[0].upper), (3.0, 4.0));
    }

    /// §5.9 예제 1.
    #[test]
    fn edge_quantities_follow_example_one() {
        let e = Edge::new(Pt::new(100, 0), Pt::new(100, 50));
        assert!(close(e.dy, 0.5) && close(e.negdx, 0.0) && close(e.c, 50.0));
        for (rx, d) in [(0, 50.0), (100, 0.0), (99, 0.5), (101, -0.5), (200, -50.0), (-100, 100.0)] {
            let r = Pt::new(rx, 0);
            assert!(close(e.d(r), d), "rx={rx}: {}", e.d(r));
            assert!(close(e.c - rx as f64 * e.dy, d));
        }
    }

    #[test]
    fn spec25_trust_table() {
        for (d, a, b) in [
            (50.0, 100.0, -1e-5),
            (0.0005, 0.0005, -0.001),
            (0.0, 0.0, 0.0),
            (-0.0005, -0.0005, 0.001),
            (-0.5, -0.5, -0.5),
            (0.001, 0.001, -0.002),
            (-0.001, -0.001, 0.002),
        ] {
            let (u1, u2) = trust_rhs(d, TrustScheme::Spec25);
            assert!(close(u1, a) && close(u2, b), "d={d}: ({u1}, {u2})");
        }
        let (u1, u2) = trust_rhs(f64::NAN, TrustScheme::Spec25);
        assert!(u1.is_nan() && u2.is_nan());
    }

    /// 제8차 6상 표 (§5.9 예제 1 표 값) — 기계어 스택 해석과 같다.
    #[test]
    fn asm_trust_table_matches_the_six_phase_check() {
        for (d, a, b) in [
            (0.0, 1e-5, 1e-5),
            (-0.5, 1e-5, 1.0),
            (0.5, 1.0, -1e-5),
            (-50.0, 1e-5, 100.0),
            (50.0, 100.0, -1e-5),
            (100.0, 200.0, -1e-5),
        ] {
            let (u1, u2) = trust_rhs(d, TrustScheme::AsmM1);
            assert!(close(u1, a) && close(u2, b), "d={d}: ({u1}, {u2})");
        }
        assert_eq!(trust_rhs(f64::NAN, TrustScheme::AsmM1), (1e-5, 1e-5));
    }

    #[test]
    fn solves_a_tiny_lp_and_reports_the_physical_length() {
        // min 0.8·L, L ≥ x + 5, x ∈ [−3, 3] → x = −3, L = 2, 목적값 1.6.
        let mut lp = Lp::new(2);
        lp.set_col_bounds(0, -3.0, 3.0);
        lp.set_col_bounds(1, 0.0, INF);
        lp.set_obj(1, OBJ_LEN);
        lp.new_row();
        lp.add_coef(0, 1.0);
        lp.add_coef(1, -1.0);
        lp.set_bounds(None, FREE_LOWER, -5.0);
        let s = lp.solve(Duration::from_secs(5));
        assert_eq!(s.status, LpStatus::Optimal);
        assert!((s.x[1] - 2.0).abs() < 1e-7, "L = {}", s.x[1]);
        assert!((s.objective - 1.6).abs() < 1e-7);
    }

    #[test]
    fn an_emptied_row_that_excludes_zero_is_infeasible() {
        let mut lp = Lp::new(1);
        lp.new_row();
        lp.add_coef(0, 1e-9);
        lp.set_bounds(None, FREE_LOWER, -1.0);
        assert_eq!(lp.solve(Duration::from_secs(1)).status, LpStatus::Infeasible);
    }

    /// 회귀 ① — 접촉(d=0)에서 파고드는 쪽은 막히고 떨어지는 쪽은 열린다.
    #[test]
    fn sound_row_blocks_penetration_and_allows_separation() {
        // 고정 i 의 NFP 오른쪽 변 (x=400, y −400..400) 을 지시서 방향(자유 쪽이 d>0)으로.
        let e = Edge::new(Pt::new(400, 400), Pt::new(400, -400));
        let r = Pt::new(400, 0);
        assert_eq!(e.cross(r), 0);
        for (obj, want) in [(1.0, -0.0), (-1.0, 100.0)] {
            let mut lp = Lp::new(1); // j 의 Δx
            lp.set_col_bounds(0, -100.0, 100.0);
            lp.set_obj(0, obj);
            let n = edge_rows(&mut lp, TrustScheme::Sound, SpecRows::default(), [None, Some(0), None, None], &e, r, None);
            assert_eq!(n, 1);
            let s = lp.solve(Duration::from_secs(1));
            assert_eq!(s.status, LpStatus::Optimal);
            assert!((s.x[0] - want).abs() < 1e-7, "obj {obj}: {}", s.x[0]);
        }
    }

    /// 경사 모서리의 분수 경계를 정수로 내린다. 모서리 (0,0)→(3,10) 의 왼쪽이 자유 쪽이고
    /// r = (0, 5) 는 외적 15 로 자유 쪽이다. x 방향 상대 이동 한계는 15/10 = 1.5 → 정수 1.
    #[test]
    fn axis_rows_round_the_relative_bound_toward_safety() {
        let e = Edge::new(Pt::new(0, 0), Pt::new(3, 10));
        let r = Pt::new(0, 5);
        assert!(e.cross(r) > 0);
        let mut lp = Lp::new(1);
        lp.set_col_bounds(0, -10.0, 10.0);
        lp.set_obj(0, -1.0); // Δx_j 를 최대화
        assert_eq!(sound_axis_row(&mut lp, [None, Some(0), None, None], &e, r, 0), 1);
        let s = lp.solve(Duration::from_secs(1));
        assert_eq!(s.status, LpStatus::Optimal);
        // 원래 경계: d/a = cross/comp. 정수로 내린 경계 이하여야 한다.
        let comp = (e.p2.y - e.p1.y) as f64;
        let exact = e.cross(r) as f64 / comp;
        assert!(s.x[0] <= exact.floor() + 1e-9 || comp < 0.0, "Δ={} 정확 경계={exact}", s.x[0]);
        // 절사 후의 점도 NFP 반평면 안에 있다.
        let moved = Pt::new(s.x[0].trunc() as i64, 0);
        assert!(e.cross(r + moved) >= 0, "절사 후 모서리를 넘었다: {moved:?}");
    }

    #[test]
    fn sound_skips_edges_whose_line_is_behind_the_reference_point() {
        let e = Edge::new(Pt::new(400, 400), Pt::new(400, -400));
        let mut lp = Lp::new(1);
        let n = edge_rows(&mut lp, TrustScheme::Sound, SpecRows::default(), [None, Some(0), None, None], &e, Pt::new(300, 0), None);
        assert_eq!(n, 0);
    }

    #[test]
    fn spec_rows_produce_four_rows_plus_the_conditional_fifth() {
        let e = Edge::new(Pt::new(100, 0), Pt::new(100, 50));
        let extra = SpecRows { row2: true, rows34: true, fifth: true, ..Default::default() };
        let slots = [Some(0), Some(1), Some(2), Some(3)];
        // rx=0 → d=50 (경로 A), rx=100 → d=0 (경로 B), rx=101 → d=−0.5 < −0.01 (5행 없음).
        for (rx, rows) in [(0, 5), (100, 5), (101, 4), (200, 4)] {
            let mut lp = Lp::new(6);
            let n = edge_rows(&mut lp, TrustScheme::AsmM1, extra, slots, &e, Pt::new(rx, 0), Some([4, 5]));
            assert_eq!(n, rows, "rx={rx}");
            assert!(lp.rows().iter().all(|r| r.lower == FREE_LOWER));
        }
        // 2행은 1행의 부호 반전이 아니다.
        let mut lp = Lp::new(4);
        edge_rows(&mut lp, TrustScheme::AsmM1, SpecRows { row2: true, ..Default::default() }, slots, &e, Pt::new(0, 0), None);
        let (r1, r2) = (&lp.rows()[0].coefs, &lp.rows()[1].coefs);
        assert_ne!(r2, &r1.iter().map(|&(c, v)| (c, -v)).collect::<Vec<_>>());
    }
}
