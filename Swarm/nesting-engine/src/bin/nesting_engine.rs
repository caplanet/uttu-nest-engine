//! 신규 Init 엔진 — 전처리 → Draft → 재시작+LP → Complete → 잔여조각 LP → 밀어내기 압축 →
//! 최종 검증 (§0.1, §4.1, §8.1, §11).
//!
//! ```text
//! nesting-engine <input.json> [--overlap 0.8] [--fabric-budget 0.1] [--time 20]
//!     [--order declaration|area] [--global off|fallback|always] [--no-worklist]
//!     [--lp-rows sound|asm|spec25] [--spec-row2] [--spec-rows34] [--spec-fifth]
//!     [--y-pass off|zero|down] [--pull 0.0] [--step 750] [--margin 2000] [--iters 20]
//!     [--no-lp] [--no-complete] [--complete-wait 5] [--no-residual] [--out result.json]
//!     [--chain complete|swarm|both|none] [--warmstart out.json]
//! ```
//!
//! §8.1: 밀도는 겹침 검증과 쌍으로만 보고한다. 미배치가 있으면 숨기지 않는다.

use anyhow::{Context, Result, bail};
use nesting_engine::chain::{self, Chain, Refine};
use nesting_engine::compact::YPass;
use nesting_engine::lp::TrustScheme;
use nesting_engine::pipeline::{self, PipelineConfig};
use nesting_engine::{draft, io};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// 초 단위 인자. 음수·NaN·무한·0 은 오류로 돌려준다(패닉하지 않는다).
fn secs(name: &str, v: &str) -> Result<Duration> {
    let s: f64 = v.parse().with_context(|| format!("{name} 값 '{v}' 을 읽지 못했다"))?;
    match Duration::try_from_secs_f64(s) {
        Ok(d) if !d.is_zero() => Ok(d),
        _ => bail!("{name} 는 양의 유한한 초여야 한다: {v}"),
    }
}

/// §11.2 단계별 목표 게이트.
fn gate_of(stage: &str) -> Option<f64> {
    match stage {
        "Draft" => Some(0.70),
        "Draft+LP" | "Complete" | "Final" => Some(0.74),
        _ => None,
    }
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut input: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    let mut warmstart: Option<PathBuf> = None;
    let mut overlap: f64 = 0.8;
    // Swarm warm start 용 원단 벽 여유 (mm). 지시서 기본 0.
    let mut edge_inset: f64 = 0.0;
    let mut mirror = false;
    let mut cfg = PipelineConfig::default();
    while let Some(a) = args.next() {
        let mut val = |name: &str| args.next().with_context(|| format!("{name} 값이 없다"));
        match a.as_str() {
            "--overlap" => overlap = val("--overlap")?.parse()?,
            "--edge-inset" => edge_inset = val("--edge-inset")?.parse()?,
            "--fabric-budget" => cfg.fabric_budget_mm = val("--fabric-budget")?.parse()?,
            "--time" => cfg.budget = secs("--time", &val("--time")?)?,
            "--order" => {
                cfg.draft.order = match val("--order")?.as_str() {
                    "declaration" => draft::Order::Declaration,
                    "area" => draft::Order::AreaDesc,
                    // 참조 구현 다중 시작점(§12.1.33 AU27).
                    "seed" | "seedsweep" => draft::Order::SeedSweep,
                    "leftfree" => draft::Order::LeftFree,
                    // ★★★ AU172 — 원천 정렬 키 (형상 치수 오름차순).
                    "shapewidth" | "width" => draft::Order::ShapeWidthAsc,
                    other => bail!("알 수 없는 순서 '{other}'"),
                }
            }
            "--global" => {
                cfg.draft.global = match val("--global")?.as_str() {
                    "off" => draft::Global::Off,
                    "fallback" => draft::Global::Fallback,
                    "always" => draft::Global::Always,
                    other => bail!("알 수 없는 전역 워크리스트 모드 '{other}'"),
                }
            }
            "--seed-trials" => cfg.draft.seed_trials = val("--seed-trials")?.parse()?,
            // 참조 구현 적응적 시작점(§12.1.33 AU39) — 직전 배치의 접촉 쌍에서 다음 시작점.
            "--adaptive-seeds" => cfg.draft.adaptive_seeds = true,
            "--adaptive-uncapped" => cfg.draft.adaptive_uncapped = true,
            // §12.1.34 AU86 — 전역 후보를 «좁힌 범위» 만 볼지 (원천 이진 탐색 근사).
            "--narrow-global" => cfg.draft.wide_global = false,
            "--global-window" => cfg.draft.global_window = val("--global-window")?.parse()?,
            // §12.1.34 AU90 — 매 반복 정렬 근사: 배치 후 좌표 기준 재배치 회차.
            "--reorder-rounds" => cfg.draft.reorder_rounds = val("--reorder-rounds")?.parse()?,
            // ★ §12.1.34 AU101 — 원천 «기준값 기반 배치» 재작성 경로.
            "--canonical" => cfg.draft.canonical = true,
            // ★★★ §12.1.77 AU182 — **«확정 최적 조합» 프리셋**.
            //
            // 실측(20초 예산)에서 **밀도 75% 를 넘기면서 가장 빨랐던 조합**이다:
            //   `75.417% / 6745.21mm / 11.49초 / 효율 6.56 %/s`
            //
            // 아래 5개 플래그를 **한 번에** 켠다 — 개별 지정과 결과가 동일하다.
            //   `--canonical-slack --canonical-scan-score --canonical-rounds 6
            //    --canonical-window 24 --canonical-family-lp`
            //
            // ★ 배포 기본 경로(`--canonical` 없이 74.298%/4.6초)는 **손상되지 않는다** —
            //   이 플래그를 준 경우에만 켜진다.
            //
            // ★ 회차를 6 으로 잡은 근거: 원천 `+0x394 = 20` 이 «설계 상한» 이지만
            //   시간 효율은 6 회차가 최고다(6 → 6.56, 12 → 4.36, 20 → 3.87 %/s).
            //   20 회차까지 돌리면 13.1초를 더 써서 0.056pp 만 얻는다.
            //   더 높은 밀도가 필요하면 `--canonical-rounds` 를 개별로 덮어써라(뒤에 온 인자가 이긴다).
            "--best" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_slack = true;
                cfg.draft.canonical_scan = true;
                cfg.draft.canonical_scan_score = true;
                cfg.draft.canonical_rounds = 6;
                cfg.draft.canonical_window = 24;
                cfg.draft.canonical_group_lp = true;
                cfg.draft.canonical_family_lp = true;
                if cfg.draft.canonical_lp.is_none() {
                    cfg.draft.canonical_lp = Some(Box::new(cfg.lp.clone()));
                }
            }
            // ★★★ §AU186 — **최고 밀도 프리셋**. `--best`(75.417%) 대신
            //   `canonical` 을 **끄고** 시작점 적응 확장(무제한)으로 간다.
            //
            // 실측(AU186, wsjk156, 순차 1회):
            //   `75.887% / 6703.47mm / 99.54s / 효율 0.76 %/s`
            //   vs `--best` 75.417% / 6745.24mm / 11.45s
            //   → **+0.470pp, −41.77mm, 시간 8.7배**
            //
            // ★ 왜 `--best` 에 얹지 않는가: `canonical=true` 면 `run_with` 가
            //   `order != SeedSweep` 분기로 **즉시 반환**(draft.rs:2612)해
            //   시작점 탐색 자체를 하지 않는다. 실측 확인: `--best --adaptive-seeds
            //   --adaptive-uncapped` = 75.417%/11.86s (무력화).
            //
            // ★ 시간 예산 주의: 99.5초는 지시서 20초(§0.1.1)의 5배다. `--time` 을
            //   함께 주지 않으면 기본 20초에서 잘린다. 측정용 경로다.
            "--max-density" => {
                cfg.draft.order = draft::Order::SeedSweep;
                cfg.draft.adaptive_seeds = true;
                cfg.draft.adaptive_uncapped = true;
            }
            // ★★★ §12.1.79 AU188 — **재시작 이중 루프** (원천 `FUN_1004ac70`).
            //   `--max-density` 는 «시작점 목록을 무제한으로 늘리는» 1중 루프였다.
            //   원천은 `+0x3e0`(목록 상한 20)에 도달하면 «배치 종료» 하고 **재시작**한다
            //   (`+0x17c` 회, 런타임 계산 = 부품 슬롯 수). 이 플래그가 그 이중 구조를 켠다.
            //   `--restart-cap N` 으로 `+0x3e0`(기본 20)을 조절한다.
            "--restart-loop" => {
                cfg.draft.order = draft::Order::SeedSweep;
                cfg.draft.adaptive_seeds = true;
                cfg.draft.restart_loop = true;
                cfg.draft.seed_list_cap = 20;
            }
            "--restart-cap" => {
                let v: usize = val("--restart-cap")?.parse()?;
                if v == 0 {
                    bail!("--restart-cap 은 1 이상이어야 한다");
                }
                cfg.draft.restart_loop = true;
                cfg.draft.seed_list_cap = v;
            }
            // ★★★ §12.1.80 AU189 — **시작점별 방향 왕복** (원천 `FUN_1002a930`).
            //   시작점 1개마다 정방향 sweep 과 역방향 sweep 을 둘 다 돌리고 폭이
            //   좁은 쪽을 채택한다. 역방향은 LP 목적계수를 설정하지 않는다
            //   (원천 `+0x368 == 2` — `chunk_003.c:20670`).
            // ★★★ §12.1.81 AU190 — **원천 파이프라인 이식** (`--autonest-native`).
            //   `--max-density` 는 `canonical_rounds = 0` 이라 LP 압축 블록을 통째로
            //   건너뛴다. 이 플래그가 그 게이트를 열어 «배치 + LP» 를 원천
            //   `FUN_1004ae60` 처럼 한 묶음으로 돌린다.
            "--autonest-native" => {
                cfg.draft.native_lp = true;
                cfg.draft.order = draft::Order::SeedSweep;
                cfg.draft.adaptive_seeds = true;
                cfg.draft.adaptive_uncapped = true;
                cfg.draft.canonical_slack = true;
                // ★ `sweep` 의 LP 는 `canonical_lp` 에서 `lp_with_cap` 을 만든다.
                //   이걸 비워두면 `lp_round` 가 `None` 이 되어 LP 가 안 돈다(AU190 계측).
                cfg.draft.canonical_lp = Some(Box::new(cfg.lp.clone()));
                // ★ AU191 철회 — 국소 LP(`native_prop`)는 98% 겹침 기각으로 값이
                //   안 바뀌고 시간만 6배 쓴다(70초 75.343% vs AU190 150초 76.361%).
                //   원천은 `+0x158`~`+0x164` **전체 구간**에 돌리는데, 우리 `n-3`
                //   근사로는 그 효과가 안 나온다. 정확 구간을 이식할 때까지 끈다.
                //   `--native-prop` 로 개별 시험 가능.
            }
            "--native-prop" => {
                cfg.draft.native_prop = true;
            }
            "--bidir" => {
                cfg.draft.bidir_swap = true;
            }
            // ★★★ §AU185 — `+0x37c`(허용오차 동적계산) — `+0x58` 진동 2배.
            //   `--best` 에는 넣지 않는다(AU185 실측: 값 변화 없음).
            //   대조 실험용 개별 플래그만 둔다.
            "--canonical-dyn-tol" => cfg.draft.canonical_dyn_tol = true,
            // ★ §12.1.43 AU145 — «배치 도중 개선»: 첫 배치를 3분할해 경계마다 개선 패스.
            "--canonical-midpass" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_midpass = true;
                if cfg.draft.canonical_lp.is_none() {
                    cfg.draft.canonical_lp = Some(Box::new(cfg.lp.clone()));
                }
            }
            // ★ §12.1.43 AU145.5 — 첫 바퀴 전역 폴백 끄기(«배치 도중» 생성).
            // ★ §12.1.43 AU146 — 구간 분할 임계 `+0x3cc` (0 = 종전 동작).
            // ★ §12.1.43 AU147 — 첫 바퀴 «목표 폭 상한» 배수 (0 = 해제).
            "--cap-mult" => {
                cfg.draft.canonical_cap_mult = val("--cap-mult")?.parse()?;
            }
            // ★ §12.1.43 AU150 — 상한 배수 퍼밀 (예: 2500 = ×2.5).
            "--cap-permille" => {
                cfg.draft.canonical_cap_permille = val("--cap-permille")?.parse()?;
            }
            // ★ §12.1.43 AU148 — 상한 점진 확대 (매 바퀴 k+wave).
            // ★ §12.1.43 AU149 — 배치 상한을 LP 압축 목표 폭으로도 전달.
            "--cap-to-lp" => {
                cfg.draft.canonical_cap_to_lp = true;
            }
            "--cap-grow" => {
                cfg.draft.canonical_cap_grow = true;
            }
            // ★★★ §12.1.69 AU174 — `canonical_pass` 의 «좌우 이웃 창» 크기.
            //   원천 `FUN_100284e0`/`FUN_10028530` 은 «이진 탐색으로 찾은 범위 전체» 를
            //   훑는다. 우리는 그 범위를 `window` 로 «근사» 하고 있었다(기본 8).
            //   창을 넓히면 원천에 더 가까워진다 — 다만 시간이 늘어난다.
            "--canonical-window" => {
                cfg.draft.canonical_window = val("--canonical-window")?.parse()?;
            }
            // ★★★ §12.1.82 AU201 — **«제약 여유 침식»** (원천 `FUN_1005f170` 뒷부분).
            //   시작점 조각에 제약이 있으면 여유 테이블(`+0x8c`/`+0x90`)을 누적
            //   감소시킨다. 감소량 = `−round(제약개수 ∓ 0.5)`. 좌·우 네 칸 모두 같은 값.
            //   기본 꺼짐 — 배포 경로(`--best`)는 손상되지 않는다.
            "--canonical-slack-erode" => cfg.draft.canonical_slack_erode = true,
            // ★ §12.1.86 AU205 — 원천 `FUN_1005f8e0`+`FUN_1005f9c0` 의
            //   «미결 쌍 수렴 반복». 기본 꺼짐.
            "--canonical-pair-converge" => cfg.draft.canonical_pair_converge = true,
            // ★★★ §12.1.61 AU170 — ⑤-a «그룹 확장 밀어넣기» (`FUN_1004b520`).
            //   원천 회차 루프의 Complete 이후 단계. 그룹 구간만 LP 로 다시 푼다.
            "--canonical-group-lp" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_group_lp = true;
            }
            // ★★★ §12.1.61 AU170 — ⑤-b «부품 일괄 이동» (`FUN_1004bae0`).
            //   원천은 «미배치가 남았을 때» 이쪽을 탄다. 우리는 전부 배치하므로
            //   기본 경로는 ⑤-a 이고, 이 플래그는 대조 실험용이다.
            "--canonical-family-lp" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_family_lp = true;
            }
            // ★★ §12.1.45 AU152 — 바퀴 사이 «미배치 국소 재정렬».
            "--canonical-local-reorder" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_midpass = true;
                cfg.draft.canonical_local_reorder = true;
            }
            // ★★ §12.1.46 AU153 — 원천 점수 함수 («우향 여유 + 배치 y»).
            "--canonical-score" => {
                cfg.draft.canonical_score = true;
            }
            // ★★ §12.1.47 AU154 — «키 정렬 + 누적 최대 이진 탐색» 범위 필터.
            "--canonical-scan" => {
                cfg.draft.canonical_scan = true;
            }
            // ★★ §12.1.48 AU155 — scan 범위 안에서만 원천 점수 적용.
            "--canonical-scan-score" => {
                cfg.draft.canonical_scan = true;
                cfg.draft.canonical_scan_score = true;
            }
            // ★★ §12.1.49 AU156 — y 축 쌍 검사 (`FUN_10028590`).
            "--canonical-yscan" => {
                cfg.draft.canonical_yscan = true;
            }
            // ★★★ §12.1.50 AU157 — «쌍 전용 y 축 검사» (`FUN_10028590`).
            "--canonical-pair-y" => {
                cfg.draft.canonical_pair_y = true;
            }
            // ★★★ §12.1.51 AU158 — 여유 테이블 갱신 (`FUN_10022270`, 각 단계 직후).
            "--canonical-slack" => {
                cfg.draft.canonical_slack = true;
            }
            "--canonical-slice" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_slice_thr = val("--canonical-slice")?.parse()?;
            }
            "--canonical-midpass-late-global" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_midpass = true;
                cfg.draft.canonical_midpass_late_global = true;
                if cfg.draft.canonical_lp.is_none() {
                    cfg.draft.canonical_lp = Some(Box::new(cfg.lp.clone()));
                }
            }
            "--canonical-rounds" => {
                cfg.draft.canonical = true;
                cfg.draft.canonical_rounds = val("--canonical-rounds")?.parse()?;
                // ★ §12.1.34 AU115 — 3단 교대: 회차를 주면 ② LP 도 함께 돌린다.
                if cfg.draft.canonical_lp.is_none() {
                    cfg.draft.canonical_lp = Some(Box::new(cfg.lp.clone()));
                }
            }
            // 각 회차의 LP 반복 상한 (교대 예산 조절).
            "--canonical-iters" => {
                let it: usize = val("--canonical-iters")?.parse()?;
                let mut lp = cfg.draft.canonical_lp.take().map(|b| *b).unwrap_or(cfg.lp.clone());
                lp.max_iter = it;
                cfg.draft.canonical_lp = Some(Box::new(lp));
            }
            // ★ §12.1.40 AU140.9 — 원천 `+0x78` «목표 폭» 전진을 LP L 제약에 강제한다.
            //   매 패스 목표를 `현재폭 − step` 으로 낮춰 «폭을 줄이려는 압력» 을 만든다.
            "--target-squeeze" => {
                cfg.lp.target_squeeze = true;
                if let Some(lp) = cfg.draft.canonical_lp.as_deref_mut() {
                    lp.target_squeeze = true;
                }
            }
            // ★ §12.1.40 AU140.14 — 원천 «제약 배율» `0.5`(`FUN_1004a140`) 적용.
            "--constraint-relax" => {
                cfg.lp.constraint_relax = true;
                if let Some(lp) = cfg.draft.canonical_lp.as_deref_mut() {
                    lp.constraint_relax = true;
                }
            }
            // ★ §12.1.40 AU140.17 — 목표 폭 «한 걸음» 폭 (step 배수). 클수록 더 넓게 민다.
            "--squeeze-span" => {
                let sp: i64 = val("--squeeze-span")?.parse()?;
                if sp < 1 {
                    bail!("--squeeze-span 은 1 이상이어야 한다: {sp}");
                }
                cfg.lp.target_squeeze = true;
                cfg.lp.squeeze_span = sp;
                if let Some(lp) = cfg.draft.canonical_lp.as_deref_mut() {
                    lp.target_squeeze = true;
                    lp.squeeze_span = sp;
                }
            }
            "--no-worklist" => cfg.draft.use_worklist = false,
            "--lp-rows" => {
                cfg.lp.trust = match val("--lp-rows")?.as_str() {
                    "sound" => TrustScheme::Sound,
                    "asm" => TrustScheme::AsmM1,
                    "spec25" => TrustScheme::Spec25,
                    other => bail!("알 수 없는 LP 행 판 '{other}'"),
                }
            }
            "--spec-row2" => cfg.lp.spec_rows.row2 = true,
            "--spec-rows34" => cfg.lp.spec_rows.rows34 = true,
            "--spec-fifth" => cfg.lp.spec_rows.fifth = true,
            "--y-pass" => {
                cfg.lp.y_pass = match val("--y-pass")?.as_str() {
                    "off" => YPass::Off,
                    "zero" => YPass::Zero,
                    "down" => YPass::Down,
                    other => bail!("알 수 없는 y 패스 '{other}'"),
                }
            }
            "--pull" => cfg.lp.dx_pull = val("--pull")?.parse()?,
            "--joint" => cfg.lp.joint = true,
            "--joint-safe" => {
                cfg.lp.joint = true;
                cfg.lp.joint_safe = true;
            }
            "--partial-repair" => cfg.lp.partial_repair = true,
            "--step" => cfg.lp.step_box = val("--step")?.parse()?,
            "--margin" => cfg.lp.active_margin = val("--margin")?.parse()?,
            "--iters" => cfg.lp.max_iter = val("--iters")?.parse()?,
            "--no-lp" => cfg.lp_enabled = false,
            "--no-complete" => cfg.complete = false,
            "--complete-wait" => cfg.complete_wait = secs("--complete-wait", &val("--complete-wait")?)?,
            "--no-residual" => cfg.residual = false,
            "--chain" => {
                let v = val("--chain")?;
                cfg.chain = Chain::parse(&v).with_context(|| {
                    format!("알 수 없는 chain '{v}' (complete|swarm|both|none)")
                })?;
            }
            // ★★★ §12.1.64 AU172 — **개선 단계** 선택 (초기 배치의 `--chain` 과 자매 축).
            //
            //   대응 관계:
            //     [초기 배치]                        [개선 단계]
            //     Draft → Complete   (신규 엔진) ←→  Complete 이후 ④~⑦  (autonest)
            //     swarm LBF (init)   (swarm)     ←→  exp → comp → polish (swarm)
            //
            //   `--chain`  = 초기 배치 (Draft→Complete | swarm LBF)
            //   `--refine` = 개선 단계 (autonest | swarm | both | none)
            //
            //   ★ 두 축의 `swarm` 은 뜻이 다르다 — `--chain swarm` 은 초기 배치를 LBF 로
            //   하고, `--refine swarm` 은 «개선 단계» 를 exp/comp/polish 로 한다.
            "--refine" => {
                let v = val("--refine")?;
                cfg.refine = Refine::parse(&v).with_context(|| {
                    format!("알 수 없는 refine '{v}' (autonest|swarm|both|none)")
                })?;
            }
            "--warmstart" => warmstart = Some(val("--warmstart")?.into()),
            "--mirror" => mirror = true,
            "--out" => out = Some(val("--out")?.into()),
            other if input.is_none() => input = Some(other.into()),
            other => bail!("알 수 없는 인자 '{other}'"),
        }
    }
    let input = input.context("입력 JSON 경로가 필요하다")?;
    if !overlap.is_finite() || overlap < 0.0 {
        bail!("--overlap 은 0 이상의 유한한 mm 여야 한다: {overlap}");
    }
    if !cfg.fabric_budget_mm.is_finite() || cfg.fabric_budget_mm < 0.0 {
        bail!("--fabric-budget 은 0 이상의 유한한 mm 여야 한다: {}", cfg.fabric_budget_mm);
    }
    if cfg.lp.max_iter == 0 {
        bail!("--iters 는 1 이상이어야 한다");
    }
    if cfg.budget > pipeline::TOTAL_BUDGET {
        eprintln!("[경고] --time {:.1}초는 지시서 상한 20초(§0.1.1)를 넘는다 — 측정용으로만 쓸 것", cfg.budget.as_secs_f64());
    }
    if !cfg.lp.step_margin_ok() {
        bail!(
            "§0.1.3 필수 부등식 위반: 2 × STEP_BOX({}) < ACTIVE_MARGIN({}) 이어야 한다",
            cfg.lp.step_box,
            cfg.lp.active_margin
        );
    }
    // `--chain` 은 `pipeline` 의 Complete 게이트(`Chain::runs_complete`)에만 걸린다. swarm LBF
    // 초기 배치(`swarm`/`both`)는 이 독립 크레이트 안에서 돌 수 없다 — warm start 내보내기는
    // `--refine swarm|both` 가 담당한다.
    if cfg.chain.exports_warmstart() {
        eprintln!(
            "[경고] --chain {:?}: swarm LBF 초기 배치는 이 엔진 밖이다. Complete 는 {} — warm start 는 --refine swarm|both 로 낸다",
            cfg.chain,
            if cfg.chain.runs_complete() { "돈다" } else { "건너뛴다" }
        );
    }

    // 측정 시작 = 전처리 진입 (§0.1.1).
    let t0 = Instant::now();
    if !edge_inset.is_finite() || edge_inset < 0.0 {
        bail!("--edge-inset 은 0 이상의 유한한 mm 여야 한다: {edge_inset}");
    }
    let inst = Arc::new(io::load_with(&input, overlap, edge_inset)?);
    let t_load = t0.elapsed();
    // §12.1.34 AU115 — 출력 요약에 쓰이므로 실행 전에 설정값을 보관한다.
    let res = pipeline::run(Arc::clone(&inst), cfg.clone(), t0);
    let total = t0.elapsed();

    let out = out.unwrap_or_else(|| input.with_extension("init.json"));
    // §12.1.27 후속: `--mirror` 는 출력 좌표계를 x 반전한다. 골든 대조가 «x 미러 적용»
    // 기준이므로(REFPLACE 보고서 12-1) 검증 도구와 좌표계를 맞출 때 쓴다.
    // **최적화 기법이 아니다** — 왼쪽 벽이든 오른쪽 벽이든 같은 문제라 폭은 안 줄어든다.
    let mirror_x = if mirror { Some(inst.used_width_mm(&res.placements)) } else { None };
    io::write_solution_mirrored(&out, &inst, &res.placements, mirror_x)?;

    // §12.1.32 — chain=swarm/both 이면 같은 배치를 swarm 가 읽는 warm start JSON 으로도 쓴다.
    // `--edge-inset` 을 준 경우 그 값을 그대로 쓴다(swarm 의 원단 포함 검사가 원본 벽에
    // 딱 붙은 배치를 통째로 버리므로, 여유를 준 만큼 warm start 도 같은 여유를 갖는다).
    let ws_path = if cfg.refine.exports_warmstart() {
        let p = warmstart.unwrap_or_else(|| chain::default_warmstart_path(&input));
        chain::write_warmstart(&p, &input, &inst, &res.placements, mirror_x, total)?;
        Some(p)
    } else {
        None
    };

    println!("단계별 (§11.2 게이트):");
    for s in &res.stages {
        let gate = gate_of(s.name)
            .map(|g| {
                let hit = s.density >= g && s.certified;
                format!("게이트 ≥{:.2} {}", g, if hit { "통과" } else { "미달" })
            })
            .unwrap_or_default();
        println!(
            "  [{:<9}] 밀도 {:>7.3}% · 폭 {:>8.2}mm · 배치 {}/{} · 겹침 {} · 원단밖 {:.4}mm · {} · {}{} · {:.2}s — {}",
            s.name,
            s.density * 100.0,
            s.width_mm,
            s.placed,
            s.total,
            s.overlaps,
            s.overrun_mm,
            if s.certified { "인증" } else { "인증 실패" },
            if s.adopted { "채택" } else { "미채택" },
            if gate.is_empty() { String::new() } else { format!(" · {gate}") },
            s.at_ms as f64 / 1000.0,
            s.note
        );
    }
    let r = &res.final_report;
    println!("eff: {:.3}; placed: {}", r.density, r.placed);
    // §11.2 목표(0.77) 도달까지의 시간 — 파이프라인이 기록한 단계 시각에서 찾는다.
    let goal = 0.77;
    let hit = res.stages.iter().find(|s| s.density >= goal && s.certified);
    match hit {
        Some(s) => println!(
            "★ 목표 도달: {:.3}% ≥ {:.0}% · {} 단계 · {:.2}s (전체 {:.2}s)",
            s.density * 100.0,
            goal * 100.0,
            s.name,
            s.at_ms as f64 / 1000.0,
            total.as_secs_f64()
        ),
        None => {
            let best = res.stages.iter().filter(|s| s.certified)
                .max_by(|a, b| a.density.total_cmp(&b.density));
            println!(
                "★ 목표 미달: 최고 {:.3}% ({}) · {:.2}s — 목표 {:.0}% 까지 {:.3}pp 부족",
                best.map(|s| s.density * 100.0).unwrap_or(0.0),
                best.map(|s| s.name).unwrap_or("없음"),
                best.map(|s| s.at_ms as f64 / 1000.0).unwrap_or(0.0),
                goal * 100.0,
                goal - best.map(|s| s.density).unwrap_or(0.0)
            );
        }
    }
    println!(
        "overlap_check: {} (겹침 {}건, 검사 쌍 {}) · 원단 밖 좌 {:.4} 우 {:.4} 하 {:.4} 상 {:.4} mm (허용 {})",
        if r.passes(cfg.fabric_budget_mm) { "PASS" } else { "FAIL" },
        r.overlaps.len(),
        r.pairs_checked,
        r.overrun[0],
        r.overrun[1],
        r.overrun[2],
        r.overrun[3],
        cfg.fabric_budget_mm
    );
    if let Some(lp) = &res.lp {
        println!(
            "압축 LP: 폭 {:.2} → {:.2} mm · 재시작 {} · 반복 {} · 점수배열(L, mm) {:?}",
            lp.width_before_mm,
            lp.width_after_mm,
            lp.restarts,
            lp.iterations,
            lp.score_hist.iter().map(|w| (w * 100.0).round() / 100.0).collect::<Vec<_>>()
        );
    }
    if let Some(b) = &res.complete_boxes {
        let n = b.iter().filter(|&&(a, c)| a != c).count();
        println!("Complete 이동변수 상자: 항목 {}개 · 전체 {}개 형상", n, b.len());
    }
    if let Some(lp) = &res.complete_lp {
        println!("Complete 압축 LP: 폭 {:.2} → {:.2} mm · 반복 {} · 종료: {}", lp.width_before_mm, lp.width_after_mm, lp.iterations, lp.stop);
    }
    println!(
        "잔여조각 LP: 잔여 {} · 배치 {} · LP {}회 · 최종 상태 0x{:x} · 최고 폭 {:?}",
        res.residual.residual,
        res.residual.placed,
        res.residual.lp_solves,
        res.commit.state.code(),
        res.commit.best_score
    );
    println!(
        "시간: 전처리 {:.2}s · 전체 {:.2}s (상한 {:.0}s) · 순서 {:?} · 전역 {:?} · LP 행 {:?} · STEP {} · MARGIN {} · 겹침 {overlap}mm",
        t_load.as_secs_f64(),
        total.as_secs_f64(),
        cfg.budget.as_secs_f64(),
        cfg.draft.order,
        cfg.draft.global,
        cfg.lp.trust,
        cfg.lp.step_box,
        cfg.lp.active_margin,
    );
    for (a, b, area) in r.overlaps.iter().take(5) {
        println!("   겹침 items {a}/{b}: 침식 후 {area:.6} mm²");
    }
    println!("결과: {}", out.display());
    if let Some(p) = &ws_path {
        println!("warm start: {}", p.display());
        println!(
            "  ─ 위 파일을 swarm 에 넘기면 explore → compress → polish 가 이어받는다 \
             (입력 JSON 의 `solution` 키를 swarm 가 자동 인식)."
        );
        println!(
            "  ─ 읽기 계약: swarm-core `read_spp_input` (`solution` 키) — 예시 tests/warm_start.rs \
             `init_with_solution_skips_lbf_placement`"
        );
    }
    Ok(())
}
