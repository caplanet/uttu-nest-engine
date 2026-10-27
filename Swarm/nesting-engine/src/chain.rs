//! 초기 배치 **이후** 단계 선택 (§12.1.32).
//!
//! ```text
//!               ┌─────────────────────────────────────────┐
//!   Draft ──────┤ chain = complete | swarm | both | none  │──── 최종 검증
//!   (신규 엔진)  └─────────────────────────────────────────┘
//! ```
//!
//! 신규 엔진은 초기 배치까지만 책임진다. 그 뒤를 무엇으로 이어갈지는 **품질·시간이
//! 서로 다르므로 선택 사항**이다:
//!
//! | 선택 | 내용 | 실측 (wsjk156, 겹침 0.8mm) |
//! |---|---|---|
//! | `complete` | 신규 엔진의 Complete → 잔여조각 → 밀어내기 (지시서 §4~§6) | 74.30% / 6846.82mm |
//! | `swarm` | 완성된 초기해를 warm start JSON 으로 내보내 swarm exp→comp→polish 가 잇는다 | 77.97% / 6524.46mm |
//! | `both` | 둘 다 실행하고 좋은 쪽 채택 | 위 둘 중 우세 |
//! | `none` | Draft 만 (초기 배치 순수 측정용) | 74.30% |
//!
//! # 왜 swarm 이 더 좋은가
//!
//! Complete 는 **같은 배치 루프를 정밀도만 바꿔 재실행**한다(§4). 원천에도 Draft 전용
//! 코드 경로가 없다는 것이 U-H 분석으로 확정됐다(`sfnt.dll` `0x10033b50` 의 모드 문자열은
//! 로그 라벨 전용이고 `+0x1c8` 을 읽는 곳이 그 함수 하나뿐). 즉 Complete 는 초기해의
//! **구조를 바꾸지 못한다** — 실측에서도 Draft 보다 −1.3pp 나빴다.
//!
//! 반면 swarm 의 explore 는 해를 **다른 구조로 재구성**한다: 정답 배치를 넣어도 6200.90 →
//! 7044.80 으로 일단 흩뜨린 뒤 6490.09 까지 다시 좁힌다. 그 재구성 능력이 초기해의 국소
//! 최적을 탈출시킨다.
//!
//! # warm start 계약
//!
//! `nesting-engine` 은 swarm 에 링크하지 않는다(독립 크레이트). 대신 swarm 가 읽는
//! 형식의 JSON 을 **파일로** 내보내고, 호출자가 그 파일을 swarm 에 넘긴다.
//! swarm 는 입력 JSON 에 `solution` 키가 있으면 그 배치에서 출발한다
//! (`swarm-core/src/util/io.rs::read_spp_input`).

use crate::io::{Instance, Placement};
use anyhow::{Context, Result};
use std::path::{Path, PathBuf};
use std::time::Duration;

/// 초기 배치 이후를 무엇으로 잇는가.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chain {
    /// 신규 엔진의 Complete → 잔여조각 → 밀어내기 (지시서 본문).
    Complete,
    /// 완성된 초기해를 warm start JSON 으로 내보내 swarm 에 넘긴다.
    Swarm,
    /// 둘 다 실행하고 좋은 쪽 채택.
    Both,
    /// Draft 만.
    None,
}

/// ★★★ §12.1.64 AU172 — **개선 단계(refine)** 를 무엇으로 하는가.
///
/// ## 대응 관계 (사용자 지시로 확정)
///
/// ```text
///   [초기 배치 — 둘 중 택일]              [개선 단계 — 둘 중 택일]
///   Draft → Complete   (신규 엔진)   ←→   Complete 이후 단계 (⑤-a ⑤-b ⑥ ⑦)
///   swarm LBF (init)   (swarm)       ←→   swarm exp → comp → polish
/// ```
///
/// **`Draft → Complete` 전체가 swarm 의 `LBF 배치(init)` 에 대응**한다. 즉 Complete 는
/// «초기 배치» 쪽에 속하고, 그 **뒤**부터가 exp/comp/polish 와 대응하는 자리다.
/// 두 열은 각각 «택일» 이며 서로 독립이다:
///
/// - 초기 배치 = `--chain draft|swarm-lbf|both|none`
/// - 개선 단계 = `--refine autonest|swarm|both|none`
///
/// ## 원천 근거
///
/// 원천 회차 루프(`FUN_1004ad80`)는 초기 배치가 끝난 뒤 **개선 단계**를 돈다:
///
/// ```c
/// FUN_1004ae60();                         // Draft → LP → Complete  (= 초기 배치)
/// if (수렴) return 0;
/// FUN_100bc990/ab0/700(+0xc);             // ④ LP 재구성
/// FUN_1004b520();  또는  FUN_1004bae0();  // ⑤-a 그룹 LP / ⑤-b 부품 이동
/// FUN_10024cd0();                         // ⑥ 경계 재계산
/// FUN_10022270(0x200);                    // ⑦ 여유 갱신
/// ```
///
/// 이 ④~⑦ 이 swarm 의 `explore → compress → polish` 와 **같은 자리**를 차지한다.
///
/// ★ 이름 주의: `Chain::Swarm`(초기 배치를 LBF 로)과 `Refine::Swarm`(개선 단계를
/// exp/comp/polish 로)은 **다른 뜻**이다. 축을 분리해 두어 모호함을 없앤다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refine {
    /// 원천 회차 루프의 개선 단계 (④ LP 재구성 / ⑤-a ⑤-b / ⑥ / ⑦).
    AutoNest,
    /// swarm 의 exp → comp → polish 로 개선한다.
    Swarm,
    /// 둘 다 실행하고 좋은 쪽 채택.
    Both,
    /// 초기 배치에서 멈춘다 (대조 측정용).
    None,
}

impl Refine {
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "autonest" | "auto" => Refine::AutoNest,
            "swarm" => Refine::Swarm,
            "both" => Refine::Both,
            "none" => Refine::None,
            _ => return None,
        })
    }

    /// 원천 Complete 이후 단계(⑤-a/⑤-b/⑥/⑦)를 도는가.
    pub fn runs_autonest(self) -> bool {
        matches!(self, Refine::AutoNest | Refine::Both)
    }

    /// Complete 이후를 swarm 으로 넘기는가.
    pub fn exports_warmstart(self) -> bool {
        matches!(self, Refine::Swarm | Refine::Both)
    }
}

impl Chain {
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "complete" => Chain::Complete,
            "swarm" => Chain::Swarm,
            "both" => Chain::Both,
            "none" => Chain::None,
            _ => return None,
        })
    }

    /// 신규 엔진의 후처리(Complete·잔여조각·밀어내기)를 돌리는가.
    pub fn runs_complete(self) -> bool {
        matches!(self, Chain::Complete | Chain::Both)
    }

    /// swarm warm start JSON 을 내보내는가.
    pub fn exports_warmstart(self) -> bool {
        matches!(self, Chain::Swarm | Chain::Both)
    }
}

/// warm start JSON 의 기본 경로 — 입력 옆에 `.<name>.warmstart.json`.
pub fn default_warmstart_path(input: &Path) -> PathBuf {
    let stem = input.file_stem().and_then(|s| s.to_str()).unwrap_or("instance");
    input.with_file_name(format!("{stem}.warmstart.json"))
}

/// 인스턴스 + 배치를 swarm 가 읽는 **warm start JSON** 으로 쓴다.
///
/// 원본 입력 JSON 을 그대로 두고 `solution` 키만 얹는다 — swarm 의 `ExtSPOutput`
/// 은 인스턴스 필드가 flatten 되어 있고 `solution` 이 별도 키이므로, 이 형태가
/// 곧 "인스턴스 + 해" 출력이다(`tests/warm_start.rs::init_with_solution_skips_lbf_placement`).
///
/// `edge_inset_mm` 는 **호출자가 `io::load_with` 에 준 값과 같아야** 한다. swarm 은
/// 자기 충돌 형상(겹침 허용치 절반만큼 줄이고 외곽을 부푼다)의 bbox 로 원단 포함을
/// 검사하므로, 원본이 벽에 딱 붙은 배치는 "outside the fixed strip bounds" 로
/// 통째로 버려진다(`src/ffi_step.rs` legacy_seed_problem).
///
/// `solution` 은 jagua-rs 0.8.1 의 `ExtSPSolution`/`ExtLayout` 그대로다 — 두 구조체에
/// serde 기본값이 **없으므로** `strip_width`·`density`·`run_time_sec`·`layout.density`
/// 를 빠뜨리면 swarm 쪽 역직렬화가 조용히 실패해 warm start 가 무시된다.
/// 폭은 `Instance::used_width_mm`, 밀도는 배치된 원본 면적 합 / (폭 × 높이) 다.
/// `run_time_sec` 는 `u64` 라 정수 초로 내보낸다.
pub fn write_warmstart(
    path: &Path,
    input: &Path,
    inst: &Instance,
    placements: &[Placement],
    mirror_x: Option<f64>,
    run_time: Duration,
) -> Result<PathBuf> {
    let text = std::fs::read_to_string(input).with_context(|| format!("{}", input.display()))?;
    let mut doc: serde_json::Value = serde_json::from_str(&text).context("입력 JSON")?;
    let obj = doc.as_object_mut().context("입력 JSON 최상위가 객체가 아니다")?;

    let items: Vec<serde_json::Value> = placements
        .iter()
        .map(|p| {
            let (rot, t) = inst.transformation_mirrored(p, mirror_x);
            serde_json::json!({
                "item_id": inst.pieces[p.piece].item_id,
                "transformation": { "rotation": rot, "translation": t }
            })
        })
        .collect();
    let width = inst.used_width_mm(placements);
    let placed_area: f64 = placements.iter().map(|p| inst.shapes[p.shape].area_mm2).sum();
    let density = if width > 0.0 && inst.strip_height_mm > 0.0 {
        placed_area / (width * inst.strip_height_mm)
    } else {
        0.0
    };

    obj.insert(
        "solution".into(),
        serde_json::json!({
            "strip_width": width,
            "density": density,
            "run_time_sec": run_time.as_secs(),
            "layout": { "container_id": 0, "placed_items": items, "density": density },
        }),
    );
    std::fs::write(path, serde_json::to_string_pretty(&doc)?)?;
    Ok(path.to_path_buf())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Pt;

    fn tiny() -> (PathBuf, PathBuf) {
        let dir = std::env::temp_dir();
        let pid = std::process::id();
        let input = dir.join(format!("chain_in_{pid}.json"));
        std::fs::write(
            &input,
            r#"{"name":"t","strip_height":10,"items":[
                {"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[4,0],[4,6],[0,6]]}},
                {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[3,0],[3,3],[0,3]]}}
            ]}"#,
        )
        .unwrap();
        (input, dir.join(format!("chain_out_{pid}.json")))
    }

    #[test]
    fn chain_parse_covers_every_arm() {
        assert_eq!(Chain::parse("complete"), Some(Chain::Complete));
        assert_eq!(Chain::parse("swarm"), Some(Chain::Swarm));
        assert_eq!(Chain::parse("both"), Some(Chain::Both));
        assert_eq!(Chain::parse("none"), Some(Chain::None));
        assert_eq!(Chain::parse("nope"), None);
    }

    #[test]
    fn chain_arms_select_the_right_halves() {
        assert!(Chain::Complete.runs_complete() && !Chain::Complete.exports_warmstart());
        assert!(!Chain::Swarm.runs_complete() && Chain::Swarm.exports_warmstart());
        assert!(Chain::Both.runs_complete() && Chain::Both.exports_warmstart());
        assert!(!Chain::None.runs_complete() && !Chain::None.exports_warmstart());
    }

    #[test]
    fn refine_parse_covers_every_arm() {
        assert_eq!(Refine::parse("autonest"), Some(Refine::AutoNest));
        assert_eq!(Refine::parse("auto"), Some(Refine::AutoNest));
        assert_eq!(Refine::parse("swarm"), Some(Refine::Swarm));
        assert_eq!(Refine::parse("both"), Some(Refine::Both));
        assert_eq!(Refine::parse("none"), Some(Refine::None));
        assert_eq!(Refine::parse("nope"), None);
    }

    /// ★ 이름이 겹쳐도 **뜻이 다르다** — `Chain::Swarm` 은 초기 배치를 넘기고,
    /// `Refine::Swarm` 은 Complete 이후만 넘긴다.
    #[test]
    fn refine_arms_are_distinct_from_chain_arms() {
        assert!(Refine::AutoNest.runs_autonest() && !Refine::AutoNest.exports_warmstart());
        assert!(!Refine::Swarm.runs_autonest() && Refine::Swarm.exports_warmstart());
        assert!(Refine::Both.runs_autonest() && Refine::Both.exports_warmstart());
        assert!(!Refine::None.runs_autonest() && !Refine::None.exports_warmstart());
    }

    /// warm start JSON 은 원본 입력의 모든 키를 보존하고 `solution` 만 얹는다.
    /// `solution` 에는 jagua-rs `ExtSPSolution`/`ExtLayout` 의 필수 키(`strip_width`,
    /// `density`, `run_time_sec`, `layout.density`)가 **모두** 있어야 한다.
    #[test]
    fn warmstart_keeps_instance_and_adds_solution() {
        let (input, out) = tiny();
        let inst = crate::io::load(&input, 0.0).unwrap();
        let placed = vec![
            Placement { piece: 0, shape: inst.pieces[0].shapes[0], pos: Pt::new(0, 0) },
            Placement { piece: 1, shape: inst.pieces[1].shapes[0], pos: Pt::new(400, 0) },
            Placement { piece: 2, shape: inst.pieces[2].shapes[0], pos: Pt::new(0, 600) },
        ];
        write_warmstart(&out, &input, &inst, &placed, None, Duration::from_secs_f64(2.7)).unwrap();

        let doc: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(&out).unwrap()).unwrap();
        // 인스턴스 필드 보존
        assert_eq!(doc["strip_height"], 10.0);
        assert_eq!(doc["items"].as_array().unwrap().len(), 2);
        // solution 추가
        let sol = &doc["solution"];
        let pi = sol["layout"]["placed_items"].as_array().unwrap();
        assert_eq!(pi.len(), 3);
        assert_eq!(pi[0]["item_id"], 0);
        assert_eq!(pi[2]["transformation"]["translation"][1], 6.0);
        // jagua-rs 필수 키 — 폭 8 (4×6 두 개가 x=0, x=4), 면적 24+24+9 = 57, 밀도 57 / (8·10).
        assert!((sol["strip_width"].as_f64().unwrap() - 8.0).abs() < 1e-9);
        let density = sol["density"].as_f64().unwrap();
        assert!((density - 57.0 / 80.0).abs() < 1e-9, "{density}");
        assert_eq!(sol["layout"]["density"], sol["density"]);
        assert_eq!(sol["run_time_sec"], 2, "u64 정수 초");
        let _ = std::fs::remove_file(&input);
        let _ = std::fs::remove_file(&out);
    }

    #[test]
    fn default_path_sits_next_to_input() {
        let p = default_warmstart_path(Path::new("C:/x/wsjk156.json"));
        assert_eq!(p.file_name().unwrap().to_str().unwrap(), "wsjk156.warmstart.json");
    }
}
