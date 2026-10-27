//! 실행 모드와 상태 코드 (§5.12).
//!
//! 원천은 컨텍스트의 16비트 필드에 상태를 적는다. 여기서는 값만 옮긴 열거형으로 둔다
//! (§13.1: 원천 바이트 배치를 강제하지 않는다).

/// 상태 코드 8종 (§5.12). 값은 원천 그대로.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum EngineState {
    /// 정상 — 계속.
    Normal = 0x00,
    /// 방향/시드 재탐색 — 다음 방향으로.
    Research = 0x20,
    /// 중간 — 계속.
    Intermediate = 0x30,
    /// 이번 회 실패 — 상위 루프로.
    RoundFailed = 0x40,
    /// 정상 종료(성공).
    Success = 0x50,
    /// 최종 실패.
    FinalFailed = 0x60,
    /// 부분 완료 — 재시작 진입. LP 가 시간 상한에 걸리면(솔버 상태 3) 이 값이 된다.
    Partial = 0x100,
    /// 압축 단계 완료 — 압축 스냅샷 확정.
    CompactDone = 0x200,
}

impl EngineState {
    pub fn code(self) -> u16 {
        self as u16
    }
}

/// 실행 모드 6종 (§5.12). 후보 수집에서 구현 대상은 모드 1 뿐이다 — 2·4·5·6 은 원천의
/// 측정·초기화 전용이다.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Default = 1,
    RefValue = 2,
    Special = 3,
    Bench = 4,
    TimeSliced = 5,
    Init = 6,
}

impl Mode {
    /// `MODE.설정(param)`: `mode = (param != 0) + 1`.
    pub fn from_param(param: i32) -> Mode {
        if param != 0 { Mode::RefValue } else { Mode::Default }
    }

    /// 모드 {4, 5} 는 대기 상태를 무시하고 진행한다.
    pub fn ignores_waiting(self) -> bool {
        matches!(self, Mode::Bench | Mode::TimeSliced)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_match_the_source_values() {
        let all = [
            (EngineState::Normal, 0x00),
            (EngineState::Research, 0x20),
            (EngineState::Intermediate, 0x30),
            (EngineState::RoundFailed, 0x40),
            (EngineState::Success, 0x50),
            (EngineState::FinalFailed, 0x60),
            (EngineState::Partial, 0x100),
            (EngineState::CompactDone, 0x200),
        ];
        for (s, c) in all {
            assert_eq!(s.code(), c);
        }
    }

    #[test]
    fn mode_is_param_not_zero_plus_one() {
        assert_eq!(Mode::from_param(0), Mode::Default);
        assert_eq!(Mode::from_param(5), Mode::RefValue);
        assert!(Mode::Bench.ignores_waiting() && Mode::TimeSliced.ignores_waiting());
        assert!(!Mode::Default.ignores_waiting());
    }
}
