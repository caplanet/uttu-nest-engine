/// Reserved settings for source and ABI compatibility. Auxiliary repair is removed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SeparationRepairConfig {
    /// **접촉 방향.** 충돌 단위 **하나만** 움직여 본다.
    ///
    /// 그 단위와 충돌 중이거나 가장 가까운 이웃 4개를 골라, 두 외곽선의 정점→변 최근접 벡터
    /// (도형당 정점·변 32개까지 표본)에서 이동 후보를 만든다: 접촉점을 살짝 넘긴 이동, 그 절반,
    /// 반대 방향. 여기에 원단 네 벽에 맞추는 이동과 ±`max_move_mm` 축 방향 이동을 더한다.
    /// 정확한 NFP 탐색이 아니라 후보 생성기이고, 채택은 실제 충돌 평가가 정한다.
    pub direction_enabled: bool,
    /// **작은 집단 LP.** 가장 심하게 충돌하는 단위와 그 주변 단위들을 **동시에** 움직인다.
    ///
    /// `2 × max_move_mm` 안의 가까운 단위를 `max_units - 1` 개까지 묶고, 그 주변 조각은 고정
    /// 장애물로 넣는다(조각 128개 상한). 쌍마다 최소 침투 축 방향의 분리 조건을 걸고 «남은 침투
    /// (슬랙) + 작은 이동 비용» 을 최소화하는 선형계획을 microlp 로 푼다. 해를 1배·0.5배·0.25배로
    /// 줄여 가며 적용해 본다.
    ///
    /// 도형을 볼록 외곽으로 근사하므로 오목한 조각에서는 지나치게 보수적일 수 있다.
    /// Polish 의 폭 축소 LP 와는 다른 모델이다 — 폭은 건드리지 않는다.
    pub cluster_lp_enabled: bool,
    /// **차단 연쇄.** 한 단위를 밀었더니 새로 부딪히는 이웃(차단 조각)까지 따라 민다.
    ///
    /// 접촉 방향 후보로 한 단위를 옮겨 보고, 채택이 안 되면 그 이동으로 쌍 loss 가 가장 많이
    /// 늘어난 이웃을 찾아(지금까지 충돌이 없던 조각도 대상) 같은 이동 또는 접촉 후보로 함께
    /// 옮긴다. 이를 `max_units` 개까지 재귀한다. 중간의 충돌 증가는 사본에만 있다.
    pub chain_enabled: bool,
    /// 복구를 켜는 정체 반복 수. 최소 loss 대비 0.5% 이상 개선이 없었던 분리 반복이 이만큼
    /// 쌓여야 한다. 1 ~ 10000, 기본 24. 작으면 자주, 크면 드물게 돈다.
    pub stagnation_iterations: u32,
    /// 분리 호출 한 번에서 돌릴 복구 패스 수 상한. 1 ~ 8, 기본 2.
    pub max_passes: u32,
    /// 이동 단위 수 상한. 세 곳에 쓰인다: 패스가 대상으로 삼는 충돌 단위 수, 집단 LP 가 함께
    /// 푸는 단위 수, 차단 연쇄의 최대 길이. 2 ~ 8, 기본 4.
    pub max_units: u32,
    /// 패스 하나에서 평가할 배치 후보 수 상한(후보 하나 = 배치 사본 하나 + 충돌 재계산).
    /// 여러 방법을 켜면 접촉 방향은 `상한 / 켠 방법 수` 까지만 쓰고 나머지를 뒤 방법에 남긴다.
    /// 1 ~ 256, 기본 64.
    pub max_evaluations: u32,
    /// 패스 하나의 벽시계 예산(ms). 세 방법이 나눠 쓰고, 넘기면 그 패스는 아무것도 바꾸지 않고
    /// 끝난다. 1 ~ 200, 기본 20.
    pub time_budget_ms: u32,
    /// 한 단위가 한 패스에서 움직일 수 있는 최대 거리(mm, 유클리드). 집단 LP 의 이웃 반경
    /// (`2 ×` 이 값)도 정한다. **겹침 허용량이 아니다** — `piece_overlap_mm` 과 무관하다.
    /// 0 초과 100 이하, 기본 2. 유한한 양수가 아니면 기본값으로 되돌린다.
    pub max_move_mm: f32,
}

impl SeparationRepairConfig {
    pub const DEFAULT: Self = Self {
        direction_enabled: false,
        cluster_lp_enabled: false,
        chain_enabled: false,
        stagnation_iterations: 24,
        max_passes: 2,
        max_units: 4,
        max_evaluations: 64,
        time_budget_ms: 20,
        max_move_mm: 2.0,
    };

    pub fn enabled(&self) -> bool {
        false
    }

    /// 작업 예산을 문서화된 범위로 자른다. 이동 한도가 유한한 양수가 아니면 기본 2mm 로 되돌린다.
    /// C 경계에서 온 값이라 그대로 믿을 수 없다.
    pub fn sanitized(self) -> Self {
        Self {
            stagnation_iterations: self.stagnation_iterations.clamp(1, 10_000),
            max_passes: self.max_passes.clamp(1, 8),
            max_units: self.max_units.clamp(2, 8),
            max_evaluations: self.max_evaluations.clamp(1, 256),
            time_budget_ms: self.time_budget_ms.clamp(1, 200),
            max_move_mm: if self.max_move_mm.is_finite() && self.max_move_mm > 0.0 {
                self.max_move_mm.min(100.0)
            } else {
                Self::DEFAULT.max_move_mm
            },
            ..self
        }
    }
}

impl Default for SeparationRepairConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn defaults_are_opt_in_with_bounded_budgets() {
        let c = SeparationRepairConfig::default();
        assert!(!c.enabled());
        assert_eq!(
            (
                c.stagnation_iterations,
                c.max_passes,
                c.max_units,
                c.max_evaluations,
                c.time_budget_ms
            ),
            (24, 2, 4, 64, 20)
        );
        assert_eq!(c.max_move_mm, 2.0);
        assert_eq!(c, c.sanitized());
    }
    #[test]
    fn hostile_budgets_and_nonfinite_moves_are_bounded() {
        let c = SeparationRepairConfig {
            direction_enabled: true,
            cluster_lp_enabled: true,
            chain_enabled: true,
            stagnation_iterations: 0,
            max_passes: u32::MAX,
            max_units: 0,
            max_evaluations: u32::MAX,
            time_budget_ms: u32::MAX,
            max_move_mm: f32::NAN,
        }
        .sanitized();
        assert!(!c.enabled());
        assert_eq!(
            (
                c.stagnation_iterations,
                c.max_passes,
                c.max_units,
                c.max_evaluations,
                c.time_budget_ms
            ),
            (1, 8, 2, 256, 200)
        );
        assert_eq!(c.max_move_mm, 2.0);
        for movement in [f32::INFINITY, f32::NEG_INFINITY, 0.0, -1.0] {
            assert_eq!(
                SeparationRepairConfig {
                    max_move_mm: movement,
                    ..c
                }
                .sanitized()
                .max_move_mm,
                2.0
            );
        }
        assert_eq!(
            SeparationRepairConfig {
                max_move_mm: 150.0,
                ..c
            }
            .sanitized()
            .max_move_mm,
            100.0
        );
    }
}
