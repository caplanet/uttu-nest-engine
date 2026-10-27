use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};

/// Trait for listeners that can receive solutions during the optimization process
pub trait SolutionListener {
    fn report(&mut self, report: ReportType, solution: &SPSolution, instance: &SPInstance);

    /// A complete immutable worker result certified on the listener's owner
    /// thread, while the other workers finish the same synchronous round.
    fn report_certified_early(&mut self, solution: &SPSolution, instance: &SPInstance) {
        self.report(ReportType::ExplFeas, solution, instance);
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReportType {
    /// 탐색 단계에서 찾은 실행 가능한 해(feasible)가 포함되어 있다.
    ExplFeas,

    /// 탐색 단계에서 찾은 실행 불가능한 해(infeasible solution)가 포함되어 있다.
    ExplInfeas,

    /// 탐색 단계에서 도출된, 이전 해보다 실행 가능 조건에 더 근접한 중간 해가 포함되어 있다.
    ExplImproving,

    /// 압축 단계에서 찾은 실행 가능한 해가 포함되어 있다.
    CmprFeas,

    /// 최종적으로 도출된 해가 포함되어 있다.
    Final,
}
