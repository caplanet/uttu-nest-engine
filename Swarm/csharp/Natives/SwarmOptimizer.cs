using System.Runtime.InteropServices;
using System.Text;

/// <summary>
/// <see cref="Marshal.PtrToStringUTF8"/>
/// </summary>
internal static class MarshalExtension
{
    /// <summary>
    /// UTF-8로 인코딩된 널 종단 문자열을 <see cref="IntPtr"/>에서 읽어 <see cref="string"/>으로 변환한다.
    ///
    /// <para>구현 방식:</para>
    /// <list type="number">
    ///   <item><description>포인터가 <see cref="IntPtr.Zero"/>이면 <c>null</c> 반환</description></item>
    ///   <item><description><see cref="Marshal.ReadByte"/>로 널 바이트(0)를 만날 때까지 바이트 길이를 측정</description></item>
    ///   <item><description><see cref="Marshal.Copy"/>로 원시 바이트 배열을 복사</description></item>
    ///   <item><description><see cref="Encoding.UTF8.GetString"/>으로 <see cref="string"/> 변환</description></item>
    /// </list>
    /// </summary>
    /// <param name="ptr">UTF-8 널 종단 문자열을 가리키는 포인터</param>
    /// <returns>변환된 문자열. <paramref name="ptr"/>가 <see cref="IntPtr.Zero"/>이면 <c>null</c>.</returns>
    public static string? PtrToStringUTF8(IntPtr ptr)
    {
        if (ptr == IntPtr.Zero)
        {
            return null;
        }

        int _len = 0;
        while (Marshal.ReadByte(ptr, _len) != 0)
        {
            _len++;
        }

        byte[] _bytes = new byte[_len];
        Marshal.Copy(ptr, _bytes, 0, _len);
        return Encoding.UTF8.GetString(_bytes);
    }
}

namespace Uttu.NestingV2
{
    /// <summary>
    /// 최적화 콜백에서 전달되는 솔루션 보고의 유형.
    /// Rust 측 <c>ReportType</c> enum과 1:1 매핑.
    /// </summary>
    public enum ReportType
    {
        /// <summary>탐색(Exploration) 단계에서 발견된 실현 가능(Feasible) 솔루션</summary>
        ExplFeas = 0,
        /// <summary>탐색 단계에서 발견된 실현 불가능(Infeasible) 솔루션</summary>
        ExplInfeas = 1,
        /// <summary>탐색 단계의 개선 중인 중간 솔루션</summary>
        ExplImproving = 2,
        /// <summary>압축(Compression) 단계에서 발견된 실현 가능 솔루션</summary>
        CmprFeas = 3,
        /// <summary>최종 솔루션</summary>
        Final = 4
    }

    /// <summary>
    /// 압축(Compression) 단계의 축소 감쇠 전략.
    /// Rust 측 <c>ShrinkDecayStrategy</c> enum과 1:1 매핑.
    /// </summary>
    public enum ShrinkDecayStrategy
    {
        /// <summary>시간 경과에 따라 축소 비율이 선형으로 감소한다.</summary>
        TimeBased = 0,
        /// <summary>실패할 때마다 축소 비율이 고정 비율(<see cref="CompressionConfig.ShrinkDecayRatio"/>)만큼 감소한다.</summary>
        FailureBased = 1
    }

    /// <summary>
    /// 중첩 분리기(Separator) 알고리즘의 설정.
    /// 충돌하는 아이템들을 물리적으로 분리하는 알고리즘의 동작을 제어한다.
    /// </summary>
    public class SeparatorConfig
    {
        /// <summary>개선 없이 포기할 최대 반복 횟수. 기본값 200.</summary>
        public int IterNoImprovementLimit { get; set; } = 200;

        /// <summary>최대 스트라이크(실패한 분리 시도) 횟수. 기본값 3.</summary>
        public int StrikeLimit { get; set; } = 3;

        /// <summary>병렬 워커 수. 기본값 3.</summary>
        public int NumWorkers { get; set; } = 3;

        /// <summary>이동당 컨테이너 전체 랜덤 샘플 수. 기본값 50.</summary>
        public int NumContainerSamples { get; set; } = 50;

        /// <summary>이동당 집중(Focussed) 랜덤 샘플 수. 기본값 25.</summary>
        public int NumFocussedSamples { get; set; } = 25;

        /// <summary>좌표 하강(Coordinate Descent) 개선 횟수. 기본값 3.</summary>
        public int NumCoordDescents { get; set; } = 3;

        // ── capmods 전용 설정 (capmods feature 없이 빌드된 네이티브에서는 무시됨) ──

        /// <summary>Reserved compatibility setting; periodic swap was removed.</summary>
        public int SwapPeriod { get; set; } = 0;

        /// <summary>스왑 패스 1회당 시도 수. 기본값 8, 네이티브 상한 1024.</summary>
        public int SwapAttempts { get; set; } = 8;

        /// <summary>스왑 파트너로 허용할 면적비 하한(기준 아이템 대비). 기본값 0.4.</summary>
        public float SwapAreaRatioMin { get; set; } = 0.4f;

        /// <summary>스왑 파트너로 허용할 면적비 상한(기준 아이템 대비). 기본값 2.5.</summary>
        public float SwapAreaRatioMax { get; set; } = 2.5f;

        /// <summary>컨테이너 전역 샘플의 좌측 편향 최대 강도. 0 = 균등 분포. 기본값 2.0.</summary>
        public float LeftBiasGain { get; set; } = 2.0f;

        /// <summary>관리 설정을 네이티브 <see cref="SwarmNative.CSeparatorConfig"/>로 변환한다.</summary>
        internal SwarmNative.CSeparatorConfig ToNative()
        {
            return new SwarmNative.CSeparatorConfig
            {
                iter_no_imprv_limit = IterNoImprovementLimit,
                strike_limit = StrikeLimit,
                n_workers = NumWorkers,
                n_container_samples = NumContainerSamples,
                n_focussed_samples = NumFocussedSamples,
                n_coord_descents = NumCoordDescents,
                swap_period = SwapPeriod,
                swap_attempts = SwapAttempts,
                swap_area_ratio_min = SwapAreaRatioMin,
                swap_area_ratio_max = SwapAreaRatioMax,
                left_bias_gain = LeftBiasGain
            };
        }
    }

    /// <summary>
    /// Explore 의 분리가 정체했을 때만 시도하는 국소 충돌 복구. 세 방법 모두 기본 꺼짐.
    /// <para>
    /// 분리 한 번 안에서, 전체 loss 가 그때까지의 최소값보다 0.5% 이상 낮아지지 못한 반복이
    /// <see cref="StagnationIterations"/> 번 쌓이면 복구 패스를 한 번 돌린다(분리당 최대
    /// <see cref="MaxPasses"/> 번). 패스는 충돌이 큰 이동 단위(강체 그룹 = 한 단위)를
    /// <see cref="MaxUnits"/> 개 골라, 켜진 방법을 접촉 방향 → 집단 LP → 차단 연쇄 순으로
    /// 시도하고 처음 채택된 후보에서 끝낸다.
    /// </para>
    /// <para>
    /// 후보는 배치 사본에서만 평가한다. 가중 loss 가 줄고, 비가중 loss 가 늘지 않고, 스트립 폭이
    /// 그대로일 때만 반영하므로 거부된 후보는 배치를 바꾸지 않는다. 회전은 바꾸지 않는다.
    /// Init·Compress·Polish 의 분리에는 적용되지 않는다. 밀도 이득은 측정 기록이 없다.
    /// </para>
    /// </summary>
    public sealed class SeparationRepairConfig
    {
        /// <summary>
        /// 접촉 방향. 충돌 단위 하나만 옮겨 본다. 충돌 중이거나 가까운 이웃 4개와의 정점·변 접촉에서
        /// 이동 후보(접촉점을 살짝 넘김, 절반, 반대 방향)를 만들고, 원단 벽 맞춤과 축 방향 이동을 더한다.
        /// </summary>
        public bool DirectionEnabled { get; set; } = false;

        /// <summary>
        /// 작은 집단 LP. 가장 심하게 충돌하는 단위와 <c>2 × MaxMoveMm</c> 안의 주변 단위들을
        /// 선형계획으로 동시에 옮긴다. 도형을 볼록 외곽으로 근사하므로 오목한 조각에서는 보수적이다.
        /// </summary>
        public bool ClusterLpEnabled { get; set; } = false;

        /// <summary>
        /// 차단 연쇄. 한 단위를 옮겼을 때 새로 부딪히는 이웃(충돌이 없던 조각 포함)까지 따라 옮긴다.
        /// 연쇄 길이는 <see cref="MaxUnits"/> 까지다.
        /// </summary>
        public bool ChainEnabled { get; set; } = false;

        /// <summary>복구를 켜는 정체 반복 수. 1~10000, 기본 24. 작으면 자주, 크면 드물게 돈다.</summary>
        public int StagnationIterations { get; set; } = 24;

        /// <summary>분리 호출 한 번에서 돌릴 복구 패스 수 상한. 1~8, 기본 2.</summary>
        public int MaxPasses { get; set; } = 2;

        /// <summary>
        /// 이동 단위 수 상한. 패스가 대상으로 삼는 충돌 단위 수, 집단 LP 가 함께 푸는 단위 수,
        /// 차단 연쇄의 최대 길이에 모두 쓰인다. 2~8, 기본 4.
        /// </summary>
        public int MaxUnits { get; set; } = 4;

        /// <summary>
        /// 패스 하나에서 평가할 배치 후보 수 상한. 여러 방법을 켜면 접촉 방향은
        /// <c>상한 / 켠 방법 수</c> 까지만 쓴다. 1~256, 기본 64.
        /// </summary>
        public int MaxEvaluations { get; set; } = 64;

        /// <summary>패스 하나의 벽시계 예산(ms). 넘기면 그 패스는 아무것도 바꾸지 않는다. 1~200, 기본 20.</summary>
        public int TimeBudgetMs { get; set; } = 20;

        /// <summary>
        /// 한 단위가 한 패스에서 움직일 수 있는 최대 거리(mm). 겹침 허용량이 아니다.
        /// 0 초과 100 이하, 기본 2.
        /// </summary>
        public float MaxMoveMm { get; set; } = 2.0f;

        internal SwarmNative.CSeparationRepairConfig ToNative() => new()
        {
            direction_enabled = DirectionEnabled ? 1 : 0,
            cluster_lp_enabled = ClusterLpEnabled ? 1 : 0,
            chain_enabled = ChainEnabled ? 1 : 0,
            stagnation_iterations = (uint)Math.Clamp(StagnationIterations, 1, 10_000),
            max_passes = (uint)Math.Clamp(MaxPasses, 1, 8),
            max_units = (uint)Math.Clamp(MaxUnits, 2, 8),
            max_evaluations = (uint)Math.Clamp(MaxEvaluations, 1, 256),
            time_budget_ms = (uint)Math.Clamp(TimeBudgetMs, 1, 200),
            max_move_mm = float.IsFinite(MaxMoveMm) && MaxMoveMm > 0 ? Math.Min(MaxMoveMm, 100.0f) : 2.0f,
        };

        internal static SeparationRepairConfig FromNative(SwarmNative.CSeparationRepairConfig c) => new()
        {
            DirectionEnabled = c.direction_enabled != 0,
            ClusterLpEnabled = c.cluster_lp_enabled != 0,
            ChainEnabled = c.chain_enabled != 0,
            StagnationIterations = (int)Math.Clamp(c.stagnation_iterations, 1u, 10_000u),
            MaxPasses = (int)Math.Clamp(c.max_passes, 1u, 8u),
            MaxUnits = (int)Math.Clamp(c.max_units, 2u, 8u),
            MaxEvaluations = (int)Math.Clamp(c.max_evaluations, 1u, 256u),
            TimeBudgetMs = (int)Math.Clamp(c.time_budget_ms, 1u, 200u),
            MaxMoveMm = float.IsFinite(c.max_move_mm) && c.max_move_mm > 0 ? Math.Min(c.max_move_mm, 100.0f) : 2.0f,
        };
    }

    /// <summary>
    /// 탐색(Exploration) 단계의 설정.
    /// 전체 스트립 폭에서 시작하여 점진적으로 폭을 줄여가며 실현 가능한 배치를 찾는 단계를 제어한다.
    /// </summary>
    public class ExplorationConfig
    {
        public SeparationRepairConfig Repair { get; set; } = new();
        /// <summary>스트립 폭 축소 스텝 비율. 예: 0.001 = 0.1%씩 축소. 기본값 0.001.</summary>
        public float ShrinkStep { get; set; } = 0.001f;

        /// <summary>탐색 단계 시간 제한(초). 기본값 540초(9분).</summary>
        public float TimeLimitSecs { get; set; } = 540f; // 9 minutes

        /// <summary>최대 연속 실패 시도 횟수. <c>null</c>이면 무제한. 기본값 <c>null</c>.</summary>
        public int? MaxConseqFailedAttempts { get; set; } = null;

        /// <summary>솔루션 풀 분포의 표준편차. 기본값 0.25.</summary>
        public float SolutionPoolDistributionStdDev { get; set; } = 0.25f;

        /// <summary>"대형 아이템" 판정을 위한 볼록 껍질(Convex Hull) 면적 컷오프 백분위. 기본값 0.75.</summary>
        public float LargeItemChAreaCutoffPercentile { get; set; } = 0.75f;

        /// <summary>중첩 분리기 설정. 기본값 <c>new SeparatorConfig()</c>.</summary>
        public SeparatorConfig Separator { get; set; } = new SeparatorConfig();

        // ── adaptive 전용 설정 (adaptive feature 없이 빌드된 네이티브에서는 무시됨) ──

        /// <summary>Reserved compatibility setting; ruin and recreate was removed.</summary>
        public int RnrPeriod { get; set; } = 0;

        /// <summary>파괴할 세로 밴드 폭(스트립 폭 대비 비율, 0~1). 기본값 0.18.</summary>
        public float RnrBandFraction { get; set; } = 0.18f;

        /// <summary>한 번에 파괴할 아이템 수 상한. 기본값 40.</summary>
        public int RnrMaxItems { get; set; } = 40;

        /// <summary>밀도 기반 적응형 shrink의 적응 강도. 0 = 고정 스텝. 기본값 4.0.</summary>
        public float AdaptiveShrinkGain { get; set; } = 4.0f;

        /// <summary>
        /// 적응형 shrink의 <see cref="ShrinkStep"/> 대비 최대 배율 상한. 기본값 5.0.
        /// <para>
        /// 기본 <see cref="AdaptiveShrinkGain"/> 4.0에서는 <c>density &lt;= 0</c>에서만 이 상한에
        /// 닿으므로 실질적으로 발동하지 않는다. gain을 10 이상으로 올려야 의미가 생긴다.
        /// </para>
        /// </summary>
        public float AdaptiveShrinkMaxMult { get; set; } = 5.0f;

        /// <summary>
        /// 축소 스텝 실패 피드백: 어떤 폭에서 분리에 처음 실패하면 스텝 배율에 곱하는 감쇠 계수.
        /// 새 feasible 폭에 도달하면 한 단계 되돌려 받는다(압축 단계와 같은 AIMD 관례).
        /// <para>
        /// <see cref="AdaptiveShrinkGain"/>은 밀도만 보는 개루프라 "이 스텝이 과했다"를 학습하지
        /// 못한다. 이 값은 그 루프를 닫는다 — 큰 스텝으로 빠르게 내려가다 벽에 닿으면 자동으로
        /// 미세 스텝으로 전환된다. 한 폭에서의 <b>첫</b> 실패만 센다.
        /// </para>
        /// (0, 1) 범위여야 동작한다. 1.0 = 비활성(기본값).
        /// </summary>
        public float ShrinkFailDecay { get; set; } = 1.0f;

        /// <summary>축소 스텝 실패 피드백: 스텝 배율의 하한. 기본값 0.05.</summary>
        public float ShrinkScaleMin { get; set; } = 0.05f;

        /// <summary>폭 진동: 같은 폭에서 이만큼 연속 실패하면 폭을 되돌렸다가 재접근. 0 = 비활성. 기본값 4.</summary>
        public int OscFailThreshold { get; set; } = 4;

        /// <summary>
        /// 폭 진동: 되돌릴 폭(<see cref="ShrinkStep"/> 배수). 기본값 3.0.
        /// <para>
        /// 실제 확장량은 <c>min(ShrinkStep × 이 값, headroom / 2)</c>이고 headroom(최선
        /// feasible 폭까지 남은 여유)은 언제나 "직전 축소 한 스텝"이다. 따라서 이 값이
        /// <see cref="AdaptiveShrinkMaxMult"/>의 절반을 넘으면 항상 headroom 쪽에 잘려
        /// 값이 무의미해진다. 최선 feasible 폭을 넘는 탈출이 필요하면
        /// <see cref="ReliefFailThreshold"/>를 쓴다.
        /// </para>
        /// </summary>
        public float OscExpandSteps { get; set; } = 3.0f;

        /// <summary>
        /// 폭 완화 복구: 한 폭에서 이만큼 연속 실패하면, 그 폭의 최선 충돌 배치를 유지한 채
        /// 폭을 조금씩(<see cref="ReliefStep"/>) 넓혀 가며 충돌이 풀릴 때까지 재분리한다.
        /// 확장은 충돌이 몰린 위치에 삽입되고, 풀리면 넓어진 그 배치에서 정상 축소를 재개한다.
        /// <para>
        /// 폭 진동(<see cref="OscFailThreshold"/>)과 달리 최선 feasible 폭에 막히지 않는다.
        /// 인컴번트 해는 그대로 보존되고 더 넓은 단계 결과는 단계 경계에서 거부되므로
        /// 최종 결과가 나빠지지 않는다.
        /// </para>
        /// 0보다 크면 복구가 켜지고 <see cref="OscFailThreshold"/>는 무시된다.
        /// 0 = 비활성(기본값, 기존 폭 진동 유지).
        /// </summary>
        public int ReliefFailThreshold { get; set; } = 0;

        /// <summary>폭 완화 복구: 라운드 1회 확장 비율(현재 폭 대비). 기본값 0.002 (0.2%).</summary>
        public float ReliefStep { get; set; } = 0.002f;

        /// <summary>폭 완화 복구: 막힌 폭 대비 누적 확장 상한. 기본값 0.03 (3%).</summary>
        public float ReliefMaxRatio { get; set; } = 0.03f;

        /// <summary>폭 완화 복구: 복구 1회의 확장 라운드 상한. 기본값 12.</summary>
        public int ReliefMaxRounds { get; set; } = 12;

        /// <summary>
        /// 폭 완화 복구: 확장 라운드 1회에 허용할 separation strike 한도.
        /// 탐색이 아니라 "이 폭이면 풀리는가"만 보므로 짧게 잡는다. 기본값 1.
        /// </summary>
        public int ReliefStrikeLimit { get; set; } = 1;

        /// <summary>
        /// 폭 완화 복구: 확장 라운드 1회의 벽시계 예산(ms). 0 = 무제한(기본값).
        /// <para>
        /// <see cref="ReliefStrikeLimit"/>만으로는 한 strike가 단계의
        /// <c>IterNoImprovementLimit</c>(대형 마커는 수백 회)를 전부 태울 수 있어, 초 단위
        /// 예산에서는 복구 한 번이 예산 전체를 먹을 수 있다. 3~5초 예산이라면 200~500을 준다.
        /// </para>
        /// </summary>
        public int ReliefRoundBudgetMs { get; set; } = 0;

        /// <summary>
        /// ALNS 교란 연산자 가중치의 지수 감쇠 계수(0~0.999). 기본값 0.85.
        /// 교란 뒤 분리 결과로 <c>가중치 ← decay·가중치 + (1−decay)·점수</c> 로 갱신한다
        /// (점수 0~10: 겹침 해소 10, loss 10% 이상 감소 5, 감소 1). 클수록 과거 성과를 오래 기억한다.
        /// R&amp;R 이 꺼져 있으면(RnrPeriod = 0) 연산자가 스왑 하나뿐이라 영향이 없다. adaptive 전용.
        /// </summary>
        public float AlnsDecay { get; set; } = 0.85f;

        /// <summary>
        /// ALNS 연산자 가중치 하한. 기본값 0.1. 연속 실패한 연산자도 이 값 아래로는 내려가지 않아
        /// 다시 선택될 여지가 남는다. adaptive 전용.
        /// </summary>
        public float AlnsMinWeight { get; set; } = 0.1f;

        /// <summary>관리 설정을 네이티브 <see cref="SwarmNative.CExplorationConfig"/>로 변환한다.
        /// <see cref="MaxConseqFailedAttempts"/>가 <c>null</c>이면 네이티브 측에서 -1로 전달된다(None 의미).</summary>
        internal SwarmNative.CExplorationConfig ToNative()
        {
            return new SwarmNative.CExplorationConfig
            {
                shrink_step = ShrinkStep,
                time_limit_secs = TimeLimitSecs,
                max_conseq_failed_attempts = MaxConseqFailedAttempts.HasValue ? MaxConseqFailedAttempts.Value : -1,
                solution_pool_distribution_stddev = SolutionPoolDistributionStdDev,
                large_item_ch_area_cutoff_percentile = LargeItemChAreaCutoffPercentile,
                separator = Separator.ToNative(),
                rnr_period = RnrPeriod,
                rnr_band_fraction = RnrBandFraction,
                rnr_max_items = RnrMaxItems,
                adaptive_shrink_gain = AdaptiveShrinkGain,
                adaptive_shrink_max_mult = AdaptiveShrinkMaxMult,
                osc_fail_threshold = OscFailThreshold,
                osc_expand_steps = OscExpandSteps,
                alns_decay = AlnsDecay,
                alns_min_weight = AlnsMinWeight
            };
        }

        /// <summary>축소 스텝 실패 피드백 설정을 네이티브 ABI-tail 블록으로 변환한다.
        /// 폭 완화 복구와 마찬가지로 C ABI에서는 tail이지만 의미상 탐색 단계 설정이다.</summary>
        internal SwarmNative.CShrinkFeedbackConfig ToNativeShrinkFeedback()
        {
            return new SwarmNative.CShrinkFeedbackConfig
            {
                fail_decay = ShrinkFailDecay,
                scale_min = ShrinkScaleMin
            };
        }

        /// <summary>폭 완화 복구 설정을 네이티브 ABI-tail 블록으로 변환한다.
        /// C ABI에서는 <see cref="SwarmNative.CSwarmConfig.width_relief"/>에 있지만
        /// 의미상 탐색 단계 설정이라 관리 API에서는 이 클래스에 둔다.</summary>
        internal SwarmNative.CWidthReliefConfig ToNativeWidthRelief()
        {
            return new SwarmNative.CWidthReliefConfig
            {
                fail_threshold = ReliefFailThreshold,
                step = ReliefStep,
                max_ratio = ReliefMaxRatio,
                max_rounds = ReliefMaxRounds,
                strike_limit = ReliefStrikeLimit,
                round_budget_ms = ReliefRoundBudgetMs
            };
        }
    }

    /// <summary>
    /// 압축(Compression) 단계의 설정.
    /// 탐색 단계에서 찾은 솔루션을 미세 조정하여 스트립 폭을 더 압축하는 단계를 제어한다.
    /// </summary>
    public class CompressionConfig
    {
        /// <summary>호환 필드 1. 두 양수 값은 순서와 무관하며 유효 시작값은 큰 값이다. 기본값 0.0005.</summary>
        public float ShrinkRangeMin { get; set; } = 0.0005f;

        /// <summary>호환 필드 2. 두 양수 값은 순서와 무관하며 유효 정지 임계값은 작은 값이다. 기본값 0.00001.</summary>
        public float ShrinkRangeMax { get; set; } = 0.00001f;

        /// <summary>압축 단계 시간 제한(초). 기본값 60초.</summary>
        public float TimeLimitSecs { get; set; } = 60f;

        /// <summary>축소 감쇠 전략. 기본값 <see cref="ShrinkDecayStrategy.TimeBased"/>.</summary>
        public ShrinkDecayStrategy ShrinkDecay { get; set; } = ShrinkDecayStrategy.TimeBased;

        /// <summary><see cref="ShrinkDecayStrategy.FailureBased"/> 전략의 감쇠 비율. 기본값 0.9.</summary>
        public float ShrinkDecayRatio { get; set; } = 0.9f;

        /// <summary>
        /// 중첩 분리기 설정.
        /// 압축 단계는 탐색 단계와 다른 기본값을 사용한다:
        /// <c>IterNoImprovementLimit=100, StrikeLimit=5, NumWorkers=3, NumContainerSamples=50, NumFocussedSamples=25, NumCoordDescents=3</c>.
        /// </summary>
        public SeparatorConfig Separator { get; set; } = new SeparatorConfig
        {
            IterNoImprovementLimit = 100,
            StrikeLimit = 5,
            NumWorkers = 3,
            NumContainerSamples = 50,
            NumFocussedSamples = 25,
            NumCoordDescents = 3
        };

        /// <summary>관리 설정을 네이티브 <see cref="SwarmNative.CCompressionConfig"/>로 변환한다.</summary>
        internal SwarmNative.CCompressionConfig ToNative()
        {
            return new SwarmNative.CCompressionConfig
            {
                shrink_range_min = ShrinkRangeMin,
                shrink_range_max = ShrinkRangeMax,
                time_limit_secs = TimeLimitSecs,
                shrink_decay_strategy = (int)ShrinkDecay,
                shrink_decay_ratio = ShrinkDecayRatio,
                separator = Separator.ToNative()
            };
        }
    }

    /// <summary>
    /// 후처리(Polish) 단계의 설정.
    /// 압축 단계가 끝난 실행 가능한 해에 결정론적 정리 패스를 적용한다.
    /// <para>
    /// 1) <see cref="LpEnabled"/> — 아이템별 평행이동과 스트립 폭을 변수로 하는 선형계획으로 큰 방향을 먼저 잡는다.
    /// 2) <see cref="SlideEnabled"/> — LP 뒤 남은 작은 틈을 좌측/하단 슬라이딩으로 회수한다.
    /// 활성 LP/slide는 지정한 시간 예산 만료 또는 정지 요청까지 반복한다.
    /// </para>
    /// <para>모든 이동은 충돌 검증을 거치므로 결과가 압축 단계보다 나빠지지 않으며, 고정된 아이템은 이동하지 않는다.</para>
    /// </summary>
    public class PolishConfig
    {
        /// <summary>후처리 단계 사용 여부. 기본값 true.</summary>
        public bool Enabled { get; set; } = true;

        /// <summary>후처리 단계 전체 시간 제한(초). 기본값 20초.</summary>
        public float TimeLimitSecs { get; set; } = 20f;

        /// <summary>기존 API 호환용. 활성 LP/slide는 시간 예산 또는 정지 요청까지 반복한다.</summary>
        public int NumRounds { get; set; } = 3;

        /// <summary>좌측 슬라이딩 패스 사용 여부. 기본값 true.</summary>
        public bool SlideEnabled { get; set; } = true;

        /// <summary>한 라운드 안에서 슬라이딩을 반복할 횟수. 기본값 3.</summary>
        public int SlideNumPasses { get; set; } = 3;

        /// <summary>하단 방향 슬라이딩도 수행할지. 기본값 true.</summary>
        public bool SlideDown { get; set; } = true;

        /// <summary>접촉 시 남겨둘 안전 간격(아이템 최소 변 길이 대비 비율). 기본값 0.</summary>
        public float SlideSafetyRatio { get; set; } = 0f;

        /// <summary>Reserved compatibility setting; hole filling was removed.</summary>
        public bool HoleFillEnabled { get; set; } = false;

        /// <summary>점유 격자 셀 크기(가장 작은 아이템 최소 변 길이 대비 비율). 기본값 0.25.</summary>
        public float HoleFillCellSizeRatio { get; set; } = 0.25f;

        /// <summary>재배치 후보 아이템 수 상한. 기본값 64.</summary>
        public int HoleFillMaxCandidates { get; set; } = 64;

        /// <summary>후보 선정 구간: 스트립 우측 몇 %를 대상으로 할지(0.0~1.0). 기본값 0.25.</summary>
        public float HoleFillCandidateBand { get; set; } = 0.25f;

        /// <summary>포켓 하나당 평가할 랜덤 샘플 수. 기본값 200.</summary>
        public int HoleFillNumSamples { get; set; } = 200;

        /// <summary>LP compaction 패스 사용 여부. 기본값 true.</summary>
        public bool LpEnabled { get; set; } = true;

        /// <summary>LP 수립 → 적용 반복 횟수. 기본값 40.</summary>
        public int LpNumIterations { get; set; } = 40;

        /// <summary>신뢰 영역의 기본(가장 촘촘한) 단계 크기(스트립 폭 대비 비율). 개선이 막히면 2배씩 확대된다. 기본값 0.005.</summary>
        public float LpTrustRegionRatio { get; set; } = 0.005f;

        /// <summary>좌측으로 당기는 목적항 가중치. 기본값 0.02.</summary>
        public float LpLeftPullWeight { get; set; } = 0.02f;

        /// <summary>관리 설정을 네이티브 <see cref="SwarmNative.CPolishConfig"/>로 변환한다.</summary>
        internal SwarmNative.CPolishConfig ToNative()
        {
            return new SwarmNative.CPolishConfig
            {
                enabled = Enabled ? 1 : 0,
                time_limit_secs = TimeLimitSecs,
                n_rounds = NumRounds,

                slide_enabled = SlideEnabled ? 1 : 0,
                slide_n_passes = SlideNumPasses,
                slide_down = SlideDown ? 1 : 0,
                slide_safety_ratio = SlideSafetyRatio,

                hole_fill_enabled = HoleFillEnabled ? 1 : 0,
                hole_fill_cell_size_ratio = HoleFillCellSizeRatio,
                hole_fill_max_candidates = HoleFillMaxCandidates,
                hole_fill_candidate_band = HoleFillCandidateBand,
                hole_fill_n_samples = HoleFillNumSamples,

                lp_enabled = LpEnabled ? 1 : 0,
                lp_n_iterations = LpNumIterations,
                lp_trust_region_ratio = LpTrustRegionRatio,
                lp_left_pull_weight = LpLeftPullWeight
            };
        }
    }

    /// <summary>
    /// LBF 초기 배치의 래스터 NFP 후보 생성 설정 (capmods 빌드 전용).
    /// <para>
    /// 레이아웃과 아이템을 같은 격자로 래스터화해 "겹치지 않는 격자 위치"를 직접 열거하고,
    /// 난수 샘플과 함께 CDE 검증에 넘긴다. 좁은 포켓을 난수가 확률적으로 놓치는 문제를 줄인다.
    /// </para>
    /// <para>
    /// 초기 폭은 확실히 줄고(실물 마커 기준 −3.2%), 배치 진행률 90% 이상인 후반부 아이템에는
    /// exact NFP 접촉 슬라이딩까지 덧붙는다. <see cref="Enabled"/> 기본값은 <c>true</c>다.
    /// </para>
    /// </summary>
    public class RasterNfpConfig
    {
        /// <summary>래스터 후보 생성 사용 여부. 기본값 true.</summary>
        public bool Enabled { get; set; } = true;

        /// <summary>셀 크기 = (가장 작은 아이템의 최소 변 길이) × 이 비율. 기본값 0.12.</summary>
        public float CellRatio { get; set; } = 0.12f;

        /// <summary>격자 셀 총 개수 상한(초과 시 셀 크기가 자동으로 커진다). 기본값 250000.</summary>
        public int MaxCells { get; set; } = 250_000;

        /// <summary>CDE로 검증할 래스터 후보 수(회전 전체 합산). 기본값 64.</summary>
        public int NumCandidates { get; set; } = 64;

        /// <summary>
        /// 래스터 후보와 병행할 난수 샘플 수. 0이면 래스터 후보만 쓴다. 기본값 1000.
        /// 난수 샘플을 줄여 래스터로 대체하면 오히려 나빠지므로 보강용으로만 쓸 것.
        /// </summary>
        public int NumRandomSamples { get; set; } = 1000;

        /// <summary>관리 설정을 네이티브 <see cref="SwarmNative.CRasterNfpConfig"/>로 변환한다.</summary>
        internal SwarmNative.CRasterNfpConfig ToNative()
        {
            return new SwarmNative.CRasterNfpConfig
            {
                enabled = Enabled ? 1 : 0,
                cell_ratio = CellRatio,
                max_cells = MaxCells,
                n_candidates = NumCandidates,
                n_random_samples = NumRandomSamples
            };
        }
    }

    /// <summary>
    /// 실험적 P0-P5 structure-first 대체 파이프라인 설정.
    /// <para>
    /// 자동 motif는 되돌릴 수 있는 생성 힌트이며 사용자 <c>PlaceGroupID</c>의
    /// 강체 계약과는 별개다. <see cref="Enabled"/> 기본값은 <c>false</c>이므로
    /// 기존 Explore/Compress/Polish 동작은 바뀌지 않는다.
    /// </para>
    /// </summary>
    public class StructureFirstConfig
    {
        public bool Enabled { get; set; } = false;
        public bool MotifEnabled { get; set; } = true;
        public int MaxActiveMotifs { get; set; } = 64;
        public int ConstructionStarts { get; set; } = 8;
        public float P1TimeLimitSecs { get; set; } = 10f;
        public bool IncludeLegacyP1Baseline { get; set; } = true;
        public float ProfileRowHeightRatio { get; set; } = 0.12f;
        public int ProfileMaxRows { get; set; } = 512;
        public int ProfileTopK { get; set; } = 3;
        public int ItemBeamWidth { get; set; } = 12;
        public int CandidatePoolCap { get; set; } = 96;
        public int SlideRefineSteps { get; set; } = 24;
        public int PairMaxOffsetCandidates { get; set; } = 256;
        public int PairMaxOrientationWork { get; set; } = 2048;
        public int DiagnosticsMaxRasterCells { get; set; } = 250000;
        public int DiagnosticsMaxPairComparisons { get; set; } = 250000;
        /// <summary>Structure 및 기존 Explore/Compress/Polish 중간 결과 callback 간격(ms). 기본값은 2000ms, 최소값은 500ms.</summary>
        public int ProgressCallbackIntervalMs { get; set; } = 2000;

        public float P2TimeLimitSecs { get; set; } = 5f;
        /// <summary>P2 축소 범위 호환 필드 1. 두 양수 값은 순서와 무관하며 큰 값부터 시작한다.</summary>
        public float P2ShrinkRangeMin { get; set; } = 0.003f;
        /// <summary>P2 축소 범위 호환 필드 2. 두 양수 값은 순서와 무관하며 작은 값에서 정지한다.</summary>
        public float P2ShrinkRangeMax { get; set; } = 0.00005f;
        public float P2FailureDecay { get; set; } = 0.55f;

        public float P3SlackRatio { get; set; } = 0.05f;
        /// <summary><c>null</c>이면 일반 Exploration 시간 제한을 사용한다.</summary>
        public float? P3TimeLimitSecs { get; set; } = null;

        public int P4Cycles { get; set; } = 3;
        public float P4CycleTimeLimitSecs { get; set; } = 5f;
        public float P4RuinFraction { get; set; } = 0.20f;
        public int P4MaxItems { get; set; } = 48;
        public int P4StagnationCycles { get; set; } = 2;

        public bool P5SnapRotations { get; set; } = true;

        internal SwarmNative.CStructureFirstConfig ToNative()
        {
            return new SwarmNative.CStructureFirstConfig
            {
                enabled = Enabled ? 1 : 0,
                motif_enabled = MotifEnabled ? 1 : 0,
                max_active_motifs = MaxActiveMotifs,
                construction_starts = ConstructionStarts,
                p1_time_limit_secs = P1TimeLimitSecs,
                p1_include_legacy_baseline = IncludeLegacyP1Baseline ? 1 : 0,
                profile_row_height_ratio = ProfileRowHeightRatio,
                profile_max_rows = ProfileMaxRows,
                profile_top_k = ProfileTopK,
                item_beam_width = ItemBeamWidth,
                candidate_pool_cap = CandidatePoolCap,
                slide_refine_steps = SlideRefineSteps,
                pair_max_offset_candidates = PairMaxOffsetCandidates,
                pair_max_orientation_work = PairMaxOrientationWork,
                diagnostics_max_raster_cells = DiagnosticsMaxRasterCells,
                diagnostics_max_pair_comparisons = DiagnosticsMaxPairComparisons,
                progress_callback_interval_ms = ProgressCallbackIntervalMs,
                p2_time_limit_secs = P2TimeLimitSecs,
                p2_shrink_range_min = P2ShrinkRangeMin,
                p2_shrink_range_max = P2ShrinkRangeMax,
                p2_failure_decay = P2FailureDecay,
                p3_slack_ratio = P3SlackRatio,
                p3_time_limit_secs = P3TimeLimitSecs ?? -1f,
                p4_cycles = P4Cycles,
                p4_cycle_time_limit_secs = P4CycleTimeLimitSecs,
                p4_ruin_fraction = P4RuinFraction,
                p4_max_items = P4MaxItems,
                p4_stagnation_cycles = P4StagnationCycles,
                p5_snap_rotations = P5SnapRotations ? 1 : 0
            };
        }
    }

    /// <summary>
    /// 진행 callback JSON에 포함할 추가 진단을 제어합니다.
    /// <para>이 설정은 Solver 내부 충돌 처리와 exact 검증을 비활성화하지 않습니다.</para>
    /// </summary>
    public class CallbackOptionsConfig
    {
        /// <summary>callback 레이아웃의 충돌 loss를 계산해 포함할지 여부. 기본값은 <c>true</c>.</summary>
        public bool CollectLoss { get; set; } = true;

        /// <summary>callback의 아이템별 충돌 상태를 계산해 포함할지 여부. 기본값은 <c>true</c>.</summary>
        public bool CollectCollisionStatus { get; set; } = true;

        internal SwarmNative.CCallbackOptionsConfig ToNative() => new()
        {
            collect_loss = CollectLoss ? 1 : 0,
            collect_collision_status = CollectCollisionStatus ? 1 : 0
        };
    }

    /// <summary>
    /// N06 독립 초기배치 엔진 설정 (<c>swarm_init_phase</c>).
    /// 구성(GRASP) → 슬라이드 재압축 → [폭 축소 → 겹침 분리(rescue·정체 종료·미세 CD) → 재압축]을
    /// kick(step 재확장)으로 시간 예산까지 반복한다. 기본값은 N06 확정 조합이다.
    /// 여성WSjk 인스턴스 실측: 60초 1시드 약 76%, 300초 8시드 최선 76.66%, 1800초 최선 77.75%.
    /// </summary>
    public class InitialPlacementConfig
    {
        /// <summary>초기배치 페이즈 사용 여부. 기본값 false.</summary>
        public bool Enabled { get; set; } = false;

        /// <summary>전체 시간 예산(초). 기본값 60.</summary>
        public float TimeLimitSecs { get; set; } = 60f;

        /// <summary>GRASP 구성 반복 수 — 최소 폭 구성을 채택. 기본값 1.</summary>
        public int Grasp { get; set; } = 1;

        /// <summary>kick(step 재확장 재시도) 횟수. 0 = kick 없음. 기본값 100000(사실상 시간 예산이 상한).</summary>
        public int Kicks { get; set; } = 100_000;

        /// <summary>축소 시작 step (스트립 폭 대비 비율). 기본값 0.005 (0.5%).</summary>
        public float Step0Frac { get; set; } = 0.005f;

        /// <summary>축소 최소 step (시작 폭 대비 비율). 기본값 0.0005 (0.05%).</summary>
        public float MinFrac { get; set; } = 0.0005f;

        /// <summary>분리(겹침 해소) 반복 상한. 기본값 3000.</summary>
        public int IterCap { get; set; } = 3000;

        /// <summary>분리 이동 후보 표본 수 기저. 기본값 50.</summary>
        public int SamplesBase { get; set; } = 50;

        /// <summary>GLS 쌍 가중치 증가율. 기본값 1.1.</summary>
        public float Growth { get; set; } = 1.1f;

        /// <summary>rescue(겹친 조각 전수 재배치) 발동 주기 — 정체 반복 수. 0 = 끔. 기본값 300.</summary>
        public int RescueAfter { get; set; } = 300;

        /// <summary>정체 기반 종료 — 무개선 연속 상한. 0 = 반복 수 종료(구 동작). 기본값 1500.</summary>
        public int StallCap { get; set; } = 1500;

        /// <summary>좌표 하강 최소 스텝(mm). 기본값 0.03.</summary>
        public float CdMin { get; set; } = 0.03f;

        /// <summary>슬라이드 재압축 패스 상한. 0 = 재압축 끔. 기본값 6.</summary>
        public int CompactPasses { get; set; } = 6;

        /// <summary>reseat(최우측 폭 결정 조각 재배치) 대상 수. 0 = 끔. 기본값 4.</summary>
        public int ReseatTop { get; set; } = 4;

        /// <summary>배치 후보 평가 상한. 기본값 32.</summary>
        public int CandEvalCap { get; set; } = 32;

        /// <summary>좌하단 점수 X 가중치. 기본값 500.</summary>
        public float LbfXWeight { get; set; } = 500f;

        /// <summary>좌하단 점수 Y 가중치. 기본값 0.1.</summary>
        public float LbfYWeight { get; set; } = 0.1f;

        /// <summary>관리 설정을 네이티브 <see cref="SwarmNative.CInitialPlacementConfig"/>로 변환한다.</summary>
        internal SwarmNative.CInitialPlacementConfig ToNative()
        {
            return new SwarmNative.CInitialPlacementConfig
            {
                enabled = Enabled ? 1 : 0,
                time_limit_secs = TimeLimitSecs,
                grasp = Grasp,
                kicks = Kicks,
                step0_frac = Step0Frac,
                min_frac = MinFrac,
                iter_cap = IterCap,
                samples_base = SamplesBase,
                growth = Growth,
                rescue_after = RescueAfter,
                stall_cap = StallCap,
                cd_min = CdMin,
                compact_passes = CompactPasses,
                reseat_top = ReseatTop,
                cand_eval_cap = CandEvalCap,
                lbf_x_weight = LbfXWeight,
                lbf_y_weight = LbfYWeight
            };
        }
    }

    /// <summary>
    /// Swarm 최적화기 전체 설정.
    /// 탐색(Exploration) · 압축(Compression) · 후처리(Polish) 단계의 설정과
    /// 전처리(다각형 단순화, 아이템 간격, 오목 컷오프), LBF 초기 배치 설정을 포함한다.
    /// </summary>
    /// <summary>축소 직후 구간(밴드)을 통째로 다시 배열한다 (ABI v17). 전부 기본 꺼짐.</summary>
    public class BandRearrangeSettings
    {
        /// <summary>축소에 성공할 때마다 우측 밴드를 통째로 다시 배열한다.
        /// <para>
        /// 기존 Ruin &amp; Recreate 는 분리 실패 뒤에만 돌아 8분 실행당 5~6회뿐이다.
        /// 축소는 같은 예산에서 52~82회 일어나므로 훨씬 자주 실행된다.
        /// </para></summary>
        public bool PostShrink { get; set; }

        /// <summary>밴드 안 이동 가능 조각을 전부 파괴한다(상한·무작위 truncate 해제).</summary>
        public bool FullBand { get; set; }

        /// <summary>재삽입 LBF 평가기의 x 편향. 0 이하 = 기본값 10.</summary>
        public float RecreateXMult { get; set; }

        /// <summary>재삽입 LBF 평가기의 y 편향. 0 이하 = 기본값 1.</summary>
        public float RecreateYMult { get; set; }

        /// <summary>재배열을 유지할 최소 "빈자리 착석률"(0~1). 0 이하 = 검사 안 함.</summary>
        public float MinClearReinsertRatio { get; set; }

        /// <summary>
        /// 축소 직후 재배열의 loss 악화 허용 배율. 1 이하 = 검사 안 함(기본).
        /// <para>
        /// 재배열 뒤의 충돌 loss 가 <c>직전 × 이 배율</c> 을 넘으면 재배열 전체를 되돌린다.
        /// 재삽입은 조각을 하나씩 덜 겹치는 자리에 놓을 뿐 조합을 보지 않아서, 밴드를 통째로
        /// 부수면 loss 가 몇 배로 늘 수 있다(실측 3 ~ 10배). 2.0 처럼 낮게 잡으면 재배열이 거의
        /// 전부 거부되어 기능을 끈 것과 같아진다.
        /// </para>
        /// </summary>
        public float MaxLossGrowth { get; set; }

        internal SwarmNative.CBandRearrangeConfig ToNative() => new()
        {
            post_shrink = PostShrink ? 1 : 0,
            full_band = FullBand ? 1 : 0,
            recreate_x_mult = RecreateXMult,
            recreate_y_mult = RecreateYMult,
            min_clear_reinsert_ratio = MinClearReinsertRatio,
            max_loss_growth = MaxLossGrowth,
        };
    }

    /// <summary>GLS 쌍 가중치의 학습률과 상한 (ABI v17). 전부 0 = 네이티브 상수.</summary>
    public class GlsWeightSettings
    {
        /// <summary>충돌 중인 쌍의 최소 증가율. 1 이하 = 기본값 1.2.</summary>
        public float MinInc { get; set; }
        /// <summary>충돌 중인 쌍의 최대 증가율. MinInc 미만 = 기본값 2.0.</summary>
        public float MaxInc { get; set; }
        /// <summary>충돌하지 않는 쌍의 감쇠율. 0 이하 또는 1 초과 = 기본값 0.95.</summary>
        public float Decay { get; set; }
        /// <summary>가중치 상한. 1 이하 = 기본값 1e6.</summary>
        public float MaxWeight { get; set; }

        internal SwarmNative.CGlsWeightConfig ToNative() => new()
        {
            min_inc = MinInc,
            max_inc = MaxInc,
            decay = Decay,
            max_weight = MaxWeight,
        };
    }

    /// <summary>초기 구성기의 배치 규칙 (ABI v17). 전부 기본 꺼짐.</summary>
    public class ConstructorSettings
    {
        /// <summary>위·아래 양쪽 모서리 앵커링을 적용할 앞쪽 조각 수. 0 = 끔.
        /// <para>이 마커 실측에서는 부호가 음수였다(73.350% → 71.902%).</para></summary>
        public int DoubleSidedFirstN { get; set; }

        /// <summary>구성 도중 몇 조각마다 hole fill + LP 압축으로 다질지. 0 = 끔.</summary>
        public int SettleEvery { get; set; }

        /// <summary>구성 도중 다지기 한 번의 예산(ms). 0 이하 = 기본값 200.</summary>
        public int SettleBudgetMs { get; set; }

        /// <summary>구성 직후 한 번만 다질 예산(ms). 0 이하 = 끔.</summary>
        public int PostInitCompactionMs { get; set; }

        /// <summary><b>구간 완성 폭(mm).</b> 0 이하 = 끔.
        /// <para>
        /// 현재 세로 구간이 더 받아들이지 못할 때까지 프론티어를 오른쪽으로 밀지 않는다.
        /// 이 마커 실측에서 300mm 가 초기 밀도 73.350% → 73.903% 로 가장 컸다.
        /// </para></summary>
        public float BandWidthMm { get; set; }

        /// <summary>NFP 구성형 배치에서 가중합 점수를 1순위로 쓸지. 이 마커에서는 음수였다.</summary>
        public bool NfpScoreFirst { get; set; }

        /// <summary>낭비 면적 가중치 w2. 음수 = 기본값 0.35.</summary>
        public float NfpW2Waste { get; set; } = -1f;
        /// <summary>접촉 길이 가중치 w3. 음수 = 기본값 0.25.</summary>
        public float NfpW3Contact { get; set; } = -1f;
        /// <summary>세로 위치 가중치 w4. 음수 = 기본값 0.10.</summary>
        public float NfpW4Y { get; set; } = -1f;

        /// <summary>길이 동률 허용치(mm). 0 이하 = 끔. 이 마커에서는 음수였다.</summary>
        public float NfpLengthTieMm { get; set; }

        /// <summary>구간 경계에서 목록 뒤쪽의 작은 조각까지 훑을 때의 면적 상한 비율.
        /// 0 이하 = 훑지 않음. 이 마커에서는 어떤 값이든 음수였다.</summary>
        public float BandFillRatio { get; set; }

        internal SwarmNative.CConstructorConfig ToNative() => new()
        {
            double_sided_first_n = DoubleSidedFirstN,
            settle_every = SettleEvery,
            settle_budget_ms = SettleBudgetMs,
            post_init_compaction_ms = PostInitCompactionMs,
            band_width_mm = BandWidthMm,
            nfp_score_first = NfpScoreFirst ? 1 : 0,
            nfp_w2_waste = NfpW2Waste,
            nfp_w3_contact = NfpW3Contact,
            nfp_w4_y = NfpW4Y,
            nfp_length_tie_mm = NfpLengthTieMm,
            band_fill_ratio = BandFillRatio,
        };
    }

    /// <summary>
    /// 자유 순서와 반복을 지원하는 최적화 단계. Initial/Structure는 별도 경로이다.
    /// </summary>
    [System.Text.Json.Serialization.JsonConverter(typeof(System.Text.Json.Serialization.JsonStringEnumConverter<OptimizationPhase>))]
    public enum OptimizationPhase { Explore, Compress, Polish }

    /// <summary>관리 실행 순서. 입력 순서와 중복을 보존하고 실행 전에 전체 목록을 검증한다.</summary>
    public static class OptimizationPhaseSequence
    {
        /// <summary>기본 순서: Explore → Compress → Polish.</summary>
        public static OptimizationPhase[] Default =>
            [OptimizationPhase.Explore, OptimizationPhase.Compress, OptimizationPhase.Polish];

        public static string DisplayName(OptimizationPhase phase) => phase.ToString();

        public static OptimizationPhase[] Parse(string? text)
        {
            if (string.IsNullOrWhiteSpace(text))
                throw new ArgumentException("실행 순서를 입력하세요. 예: Explore → Explore → Compress → Polish");
            var tokens = text.Replace("->", ",").Replace("→", ",").Split(',');
            var result = new OptimizationPhase[tokens.Length];
            for (int i = 0; i < tokens.Length; i++)
            {
                string token = tokens[i].Trim();
                if (!Enum.GetNames<OptimizationPhase>().Any(name => string.Equals(name, token, StringComparison.OrdinalIgnoreCase)) ||
                    !Enum.TryParse(token, true, out result[i]))
                    throw new ArgumentException($"실행 순서 {i + 1}번째 '{token}'는 잘못된 단계입니다. Explore, Compress, Polish를 사용하세요.");
            }
            return result;
        }

        public static OptimizationPhase[] Validate(IEnumerable<OptimizationPhase>? sequence)
        {
            var result = sequence?.ToArray();
            if (result == null || result.Length == 0 || result.Any(phase => !Enum.IsDefined(phase)))
                throw new ArgumentException("실행 순서는 Explore, Compress, Polish 중 하나 이상이어야 합니다. 중복은 허용됩니다.");
            return result;
        }

        /// <summary>각 호출은 동기적으로 끝난 후 다음 호출을 시작한다. 중단 또는 첫 오류에서 나머지를 생략한다.</summary>
        public static int Run(IEnumerable<OptimizationPhase> sequence, Func<bool> shouldStop,
            Func<OptimizationPhase, int> invoke, Action<OptimizationPhase, int, int>? starting = null,
            Action<OptimizationPhase, int, int, int>? completed = null)
        {
            var phases = Validate(sequence);
            for (int i = 0; i < phases.Length; i++)
            {
                if (shouldStop()) return 0;
                starting?.Invoke(phases[i], i + 1, phases.Length);
                if (shouldStop()) return 0;
                int result = invoke(phases[i]);
                completed?.Invoke(phases[i], i + 1, phases.Length, result);
                if (result != 0 || shouldStop()) return result;
            }
            return 0;
        }
    }

    public class SwarmConfig
    {
        /// <summary>난수 생성기 시드. 0이면 무작위 시드를 사용한다. 기본값 0.</summary>
        public ulong RngSeed { get; set; } = 0;

        /// <summary>
        /// 실행할 최적화 단계의 순서. 같은 단계를 여러 번 넣을 수 있다(예: Explore → Explore →
        /// Compress → Polish). 네이티브 설정으로 넘어가지 않는 관리 측 값이며, 단계별 API 를
        /// 이 순서대로 호출하는 쪽(WPF 샘플)이 읽는다. Structure 를 켜면 쓰지 않는다.
        /// </summary>
        public OptimizationPhase[] PhaseSequence { get; set; } = OptimizationPhaseSequence.Default;

        /// <summary>다각형 단순화 허용 오차. <c>null</c>이면 비활성. 기본값 0.001.</summary>
        public float? PolySimplTolerance { get; set; } = 0.001f;
        /// <summary>양수 mm 한도. 설정하면 면적 비율 단순화와 좁은 홈 닫기를 대체한다.</summary>
        public float? SimplifyMaxDeviationMm { get; set; }

        /// <summary>아이템 간 최소 간격. <c>null</c>이면 비활성. 기본값 <c>null</c>.</summary>
        public float? MinItemSeparation { get; set; } = null;

        /// <summary>좁은 오목(Narrow Concavity) 컷오프 거리. <c>null</c>이면 비활성. 기본값 0.01.</summary>
        public float? NarrowConcavityCutoffDist { get; set; } = 0.01f;

        /// <summary>좁은 오목(Narrow Concavity) 컷오프 면적. <c>null</c>이면 비활성. 기본값 0.01.</summary>
        public float? NarrowConcavityCutoffArea { get; set; } = 0.01f;

        /// <summary>LBF 초기 배치 X축 bias multiplier. 기본값 10.0</summary>
        public float? lbf_x_multiplier { get; set; } = 10.0f;

        /// <summary>LBF 초기 배치 Y축 bias multiplier. 기본값 1.0</summary>
        public float? lbf_y_multiplier { get; set; } = 1.0f;

        /// <summary>탐색 단계 설정. 기본값 <c>new ExplorationConfig()</c>.</summary>
        public ExplorationConfig Exploration { get; set; } = new ExplorationConfig();

        /// <summary>압축 단계 설정. 기본값 <c>new CompressionConfig()</c>.</summary>
        public CompressionConfig Compression { get; set; } = new CompressionConfig();

        /// <summary>후처리 단계 설정. 기본값 <c>new PolishConfig()</c>.</summary>
        public PolishConfig Polish { get; set; } = new PolishConfig();

        /// <summary>LBF 래스터 NFP 설정 (capmods 빌드 전용). 기본값 <c>new RasterNfpConfig()</c> (활성, cell ratio 0.12, 후보 64).</summary>
        public RasterNfpConfig LbfRaster { get; set; } = new RasterNfpConfig();

        /// <summary>
        /// 각 아이템의 허용 회전에 180° 반전을 추가한다. 결방향 없는(non-nap) 원단 전용.
        /// 기본값 <c>false</c>. (여성WSjk 60초 기준 밀도 +0.8%p)
        /// </summary>
        public bool Allow180Rotation { get; set; } = false;

        /// <summary>
        /// discrete 회전 아이템에 허용하는 소각도 틸트 (도 단위, 허용 회전 기준 ±값).
        /// 0 = 비활성(기본값). 결방향 등으로 정확한 각도가 필요한 원단에서는 0으로 둘 것.
        /// (여성WSjk 60초 기준 ±2°에서 밀도 +2.2%p)
        /// </summary>
        public float TiltRangeDeg { get; set; } = 0f;

        /// <summary>P0-P5 structure-first 설정. 기본값은 비활성.</summary>
        public StructureFirstConfig StructureFirst { get; set; } = new StructureFirstConfig();

        /// <summary>진행 callback JSON 진단 설정.</summary>
        public CallbackOptionsConfig CallbackOptions { get; set; } = new CallbackOptionsConfig();

        /// <summary>N06 초기배치 페이즈 설정. 기본값은 비활성.</summary>
        public InitialPlacementConfig InitialPlacement { get; set; } = new InitialPlacementConfig();

        /// <summary>
        /// 관리 설정을 네이티브 <see cref="SwarmNative.CSwarmConfig"/>로 변환한다.
        ///
        /// <para>변환 규칙:</para>
        /// <list type="bullet">
        ///   <item><description><c>float?</c>가 <c>null</c>이면 <c>-1f</c>로 매핑 (네이티브 측에서 음수 = "None" 의미)</description></item>
        ///   <item><description>하위 설정 객체도 재귀적으로 <c>ToNative()</c> 호출</description></item>
        /// </list>
        /// </summary>
        internal SwarmNative.CSwarmConfig ToNative()
        {
            return new SwarmNative.CSwarmConfig
            {
                rng_seed = RngSeed,
                poly_simpl_tolerance = SimplifyMaxDeviationMm.HasValue ? -1f : PolySimplTolerance ?? -1f,
                min_item_separation = MinItemSeparation ?? -1f,
                narrow_concavity_cutoff_dist = NarrowConcavityCutoffDist ?? -1f,
                narrow_concavity_cutoff_area = NarrowConcavityCutoffArea ?? -1f,
                lbf_x_multiplier = lbf_x_multiplier ?? 10.0f,
                lbf_y_multiplier = lbf_y_multiplier ?? 1.0f,
                exploration = Exploration.ToNative(),
                compression = Compression.ToNative(),
                polish = Polish.ToNative(),
                lbf_raster = LbfRaster.ToNative(),
                allow_180_rotation = Allow180Rotation ? 1 : 0,
                tilt_range_deg = TiltRangeDeg,
                structure_first = StructureFirst.ToNative(),
                callback_options = CallbackOptions.ToNative(),
                initial_placement = InitialPlacement.ToNative(),
                width_relief = Exploration.ToNativeWidthRelief(),
                repair = Exploration.Repair.ToNative(),
                shrink_feedback = Exploration.ToNativeShrinkFeedback(),
                escape = Escape.ToNative(),
                nfp_lbf = NfpLbf.ToNative(),
                fabric_tolerance = FabricTolerance.ToNative(),
                nfp_integration = NfpIntegration.ToNative(),
                nfp_lbf_tuning = NfpLbf.ToNativeTuning(),
                nfp_strategy = NfpStrategy.ToNative(),
                lp_solver = LpSolver.ToNative(),
                nfp_strategy2 = NfpStrategy.ToNative2(),
                refplace = RefPlace.ToNative(SimplifyMaxDeviationMm),
                lbf_order = new SwarmNative.CLbfOrderConfig
                {
                    mode = (int)LbfOrder,
                    _reserved0 = 0,
                    _reserved1 = 0,
                    _reserved2 = 0
                },
                hole_first = new SwarmNative.CHoleFirstConfig
                {
                    enabled = HoleFirst ? 1 : 0,
                    share = HoleFirstShare,
                    _reserved0 = 0,
                    _reserved1 = 0
                },
                reclaim = new SwarmNative.CReclaimConfig
                {
                    enabled = ReclaimLeftover ? 1 : 0,
                    compress_share = ReclaimCompressShare,
                    max_cycles = ReclaimMaxCycles,
                    min_slice_ms = ReclaimMinSliceMs,
                    mode = (int)ReclaimMode,
                    ruin_band_fraction = ReclaimRuinBandFraction,
                    ruin_max_items = ReclaimRuinMaxItems,
                    stagnation_cycles = ReclaimStagnationCycles
                },
                band_rearrange = BandRearrange.ToNative(),
                fabric_edge = new SwarmNative.CFabricEdgeConfig
                {
                    margin_mm = FabricEdgeMarginMm ?? 0f
                },
                gls = Gls.ToNative(),
                constructor = Constructor.ToNative(),
                original_overlap = new SwarmNative.COriginalOverlapConfig
                {
                    excess_policy = (int)OriginalOverlapPolicy
                },
                place_group = new SwarmNative.CPlaceGroupConfig
                {
                    mode = (int)PlaceGroupMode
                }
            };
        }

        /// <summary>ABI v17 — 축소 직후 구간(밴드) 통째 재배열. 기본 꺼짐.</summary>
        public BandRearrangeSettings BandRearrange { get; set; } = new();

        /// <summary>ABI v17 — 조각의 <b>원본</b> 외곽선이 원단 밖으로 넘어도 되는 거리(mm).
        /// <c>null</c>이면 검사하지 않는다(기본).
        /// <para>
        /// 겹침을 허용하면 충돌 도형이 침식되는데, 침식은 볼록 꼭짓점을 변보다 깊이
        /// 끌어당긴다. 그래서 컨테이너를 같은 양만큼 줄여도 원본 꼭짓점은 밖으로 나간다.
        /// 값을 주면 공개 직전에 넘은 조각만 안쪽으로 민다.
        /// </para></summary>
        public float? FabricEdgeMarginMm { get; set; }

        /// <summary>ABI v19 — 원본 외곽선 겹침이 겹침 허용값을 넘는 배치의 처리.
        /// 기본 <see cref="OriginalOverlapPolicy.Infeasible"/>(버림, 기존 동작).
        /// <para>겹침 허용(<c>FabricTolerance.PieceOverlapMm</c>)이 0 이면 아무 효과가 없다.</para></summary>
        public OriginalOverlapPolicy OriginalOverlapPolicy { get; set; } = OriginalOverlapPolicy.Infeasible;

        /// <summary>ABI v20 — V2 place groups(패턴 그룹) 처리 방식.
        /// 기본 <see cref="PlaceGroupMode.MultiPolygon"/>: 그룹마다 멤버 외곽선을 합친 아이템 하나.</summary>
        public PlaceGroupMode PlaceGroupMode { get; set; } = PlaceGroupMode.MultiPolygon;

        /// <summary>ABI v17 — GLS 가중치 학습률. 기본값은 네이티브 상수.</summary>
        public GlsWeightSettings Gls { get; set; } = new();

        /// <summary>ABI v17 — 초기 구성기의 배치 규칙. 기본 꺼짐.</summary>
        public ConstructorSettings Constructor { get; set; } = new();

        /// <summary><b>실험</b> — 레퍼런스 설계 초기배치 엔진. 기본값은 꺼짐.</summary>
        public RefPlaceSettings RefPlace { get; set; } = new();

        /// <summary>
        /// LBF 초기배치의 <b>순회 방향</b>. 기본값은 현행 동작(큰 것부터).
        /// </summary>
        public LbfOrderMode LbfOrder { get; set; } = LbfOrderMode.LargestFirst;

        /// <summary>
        /// <b>홀 우선 부착</b> — 이미 배치된 조각들에 둘러싸인 빈 주머니(포켓)에
        /// 먼저 조각을 넣는다. 기본 꺼짐.
        /// <para>
        /// 실측: wsjk156 폭 −0.73% / 밀도 +0.52%p, shirts는 +0.80% / −0.62%p로 나빠진다.
        /// 조각이 크고 오목한 의류 마커에서 켤 것.
        /// </para>
        /// </summary>
        public bool HoleFirst { get; set; }

        /// <summary>포켓 후보에 내줄 후보 슬롯 비율 (0.0~1.0). 0 이하면 네이티브 기본값 0.5.</summary>
        public float HoleFirstShare { get; set; } = 0.5f;

        /// <summary>
        /// <b>폴리시 잔여 예산 회수</b> — 폴리시가 일찍 수렴하면 남은 시간 동안
        /// 압축↔폴리시를 반복한다. <b>기본 켜짐.</b>
        /// <para>
        /// 총 소요 시간은 늘지 않는다(마감이 폴리시 예산의 끝). 실측 4회에서
        /// 적중 2/4, 평균 −6.28mm, 한 번도 나빠지지 않았다.
        /// </para>
        /// <para>
        /// <b>재현성 주의</b>: 사이클 수가 실측 잔여 시간의 함수라 기계 속도에 따라
        /// 결과가 달라진다. 재현이 필요하면 <see cref="ReclaimMaxCycles"/> 를 고정하라.
        /// </para>
        /// </summary>
        public bool ReclaimLeftover { get; set; } = true;

        /// <summary>한 사이클에서 압축이 받을 몫 (0.0~1.0). 0 이하면 네이티브 기본값 0.75.</summary>
        public float ReclaimCompressShare { get; set; } = 0.75f;

        /// <summary>사이클 수 상한. 0 = 예산까지. 재현성이 필요하면 고정한다.</summary>
        public int ReclaimMaxCycles { get; set; }

        /// <summary>남은 시간이 이보다 적으면 시작하지 않는다(ms). 0 이하면 기본값 500.</summary>
        public int ReclaimMinSliceMs { get; set; } = 500;

        /// <summary>잔여 시간을 무엇에 쓸지. 기본은 압축 재진입.</summary>
        public ReclaimMode ReclaimMode { get; set; } = ReclaimMode.CompressReentry;

        /// <summary>탈출 계열에서 부술 밴드 폭 (strip 폭 대비). 0 이하면 기본값 0.20.</summary>
        public float ReclaimRuinBandFraction { get; set; } = 0.20f;

        /// <summary>탈출 계열에서 부술 조각 수 상한. 0 이하면 기본값 48.</summary>
        public int ReclaimRuinMaxItems { get; set; } = 48;

        /// <summary>
        /// 탈출 계열에서 개선 없는 사이클을 몇 번까지 참을지. 0 이하면 기본값 2.
        /// 압축 재진입은 결정론적이라 이 값과 무관하게 첫 실패에서 멈춘다.
        /// </summary>
        public int ReclaimStagnationCycles { get; set; } = 2;

        /// <summary>정체 감지 기반 적응형 국소 최적해 탈출 설정. 기본값은 꺼짐.</summary>
        public EscapeSettings Escape { get; set; } = new();

        /// <summary>NFP 기반 초기 배치(NFP-LBF) 시드 설정. 기본값은 꺼짐.</summary>
        public NfpLbfSettings NfpLbf { get; set; } = new();

        /// <summary>원단 허용오차(조각 겹침) 설정. 기본값은 꺼짐.</summary>
        public FabricToleranceSettings FabricTolerance { get; set; } = new();

        /// <summary>NFP 초기배치 전략 7종. 기본값은 전부 꺼짐.</summary>
        public NfpStrategySettings NfpStrategy { get; set; } = new();

        /// <summary>압축 단계 LP·QP 솔버 선택. 기본값은 기존 동작(단체법, 순수 LP).</summary>
        public LpSolverSettings LpSolver { get; set; } = new();

        /// <summary>탐색 단계의 NFP 활용 범위. 기본값은 전부 꺼짐.</summary>
        public NfpIntegrationSettings NfpIntegration { get; set; } = new();
    }

    /// <summary>
    /// 정체 감지 기반 적응형 국소 최적해 탈출 (Stagnation-Triggered Adaptive Local-Optimum Escape).
    /// <para>
    /// 켜면 탐색 단계가 탈출 파이프라인으로 교체된다. 0/음수 필드는 네이티브가 코어 기본값으로 보정한다.
    /// </para>
    /// </summary>
    public class EscapeSettings
    {
        /// <summary>탈출 기법 사용 여부. 기본 꺼짐.</summary>
        public bool Enabled { get; set; }

        /// <summary>정체 판정 창의 하한. 0 이하 = 기본값(100).</summary>
        public int WindowFloor { get; set; }

        /// <summary>정체 판정 창의 패턴 수 배수. 0 이하 = 기본값(3).</summary>
        public int WindowItemFactor { get; set; }

        /// <summary>효율 개선량 임계(비율). 0 이하 = 기본값(0.0001 = 0.01%p).</summary>
        public float EfficiencyEps { get; set; }

        /// <summary>후보 다양성 임계. 0 이하 = 기본값(0.05).</summary>
        public float DiversityThreshold { get; set; }

        /// <summary>Conflict Burst 사용 여부. 기본 켜짐.</summary>
        public bool EnableConflictBurst { get; set; } = true;

        /// <summary>Swarm 다양화 사용 여부. 기본 켜짐.</summary>
        public bool EnableSwarmDiversification { get; set; } = true;

        /// <summary>Collision Push 최대 대상 수. 0 이하 = 기본값(16).</summary>
        public int MaxPushItems { get; set; }

        /// <summary>infeasible 풀 상한. 0 이하 = 기본값(32).</summary>
        public int MaxInfeasiblePool { get; set; }

        internal SwarmNative.CEscapeConfig ToNative()
        {
            return new SwarmNative.CEscapeConfig
            {
                enabled = Enabled ? 1 : 0,
                window_floor = WindowFloor,
                window_item_factor = WindowItemFactor,
                efficiency_eps = EfficiencyEps,
                diversity_threshold = DiversityThreshold,
                enable_conflict_burst = EnableConflictBurst ? 1 : 0,
                enable_swarm_diversification = EnableSwarmDiversification ? 1 : 0,
                max_push_items = MaxPushItems,
                max_infeasible_pool = MaxInfeasiblePool
            };
        }
    }

    /// <summary>
    /// NFP 기반 초기 배치(NFP-LBF) 시드 설정.
    /// <para>
    /// NFP Cache -&gt; Multiple Item Ordering -&gt; NFP Sequential Placement 로 초기 배치를 만들어
    /// 탐색 단계의 seed 로 넣는다. 실패하면 기존 LBF 생성기로 자동 폴백한다.
    /// </para>
    /// </summary>
    public class NfpLbfSettings
    {
        /// <summary>NFP-LBF 시드 사용 여부. 기본 꺼짐(기존 LBF 생성기 유지).</summary>
        public bool Enabled { get; set; }

        /// <summary>평가할 ordering 수. 0 이하 = 기본값(50).</summary>
        public int Orderings { get; set; }

        /// <summary>NFP 콜드 캐시 생성 예산(ms). 0 이하 = 기본값(5000).</summary>
        public int CacheBudgetMs { get; set; }

        /// <summary>초기 배치 탐색 전체 예산(ms). 0 이하 = 기본값(10000).</summary>
        public int TotalBudgetMs { get; set; }

        /// <summary>0/180도 회전을 함께 쓸지. 결방향 없는(non-nap) 원단 전용. 기본 꺼짐.</summary>
        public bool Allow180 { get; set; }

        /// <summary>앞부분이 같은 ordering 의 배치를 재사용할지. 기본 켜짐.</summary>
        public bool EnablePrefixReuse { get; set; } = true;

        /// <summary>
        /// NFP 라이브러리 전용 오목 단순화 강도(원본 면적 대비 허용 증가 비율).
        /// 음수 = 기본값(0.0 = 비활성). 결과 도형은 원본의 상위집합이라 안전하다.
        /// 실측: 0.02 에서 캐시 생성 2677ms -&gt; 162ms, 밀도 손해 없음.
        /// </summary>
        public float ConcaveSimplifyTolerance { get; set; }

        /// <summary>
        /// 아이템 하나당 feasible 판정을 시도할 raw 후보 수 상한. 0 이하 = 기본값(4096).
        /// <para>
        /// <b>초기 배치 품질을 좌우하는 값이다.</b> 후보 수는 배치가 진행될수록 폭증하는데
        /// (여성WSjk 156-패널: 20번째 1,607개 → 155번째 13,514개) 예산이 고정이면 후기
        /// 아이템이 후보의 7.5%만 보고 자리를 정한다.
        /// </para>
        /// <para>
        /// 실측 (여성WSjk 156-패널, ordering 50개, <b>오목 단순화 0.02</b>,
        /// LBF = 7325.05 / 69.45% — <b>단, 이 LBF 기준선은 벤치가 좌측편향 계수를
        /// 프로덕션(10.0)이 아닌 1.0 으로 만든 값이라 실제보다 나쁘다. 프로덕션 계수로
        /// 재면 70.61% 이므로 아래 델타는 약 1.16%p 부풀려져 있다</b>):
        /// 1024 → 7479.09/68.02% (<b>LBF에 져서 시드가 기각된다</b>),
        /// 2048 → 7277.71/69.90%, 4096 → 7082.27/71.83% (기본값, 여기서 포화),
        /// 8192 → 7057.59/72.08%. 전체 시간은 8초 → 14초.
        /// </para>
        /// </summary>
        public int MaxCandidateProbes { get; set; }

        internal SwarmNative.CNfpLbfTuningConfig ToNativeTuning()
        {
            return new SwarmNative.CNfpLbfTuningConfig
            {
                max_candidate_probes = MaxCandidateProbes
            };
        }

        internal SwarmNative.CNfpLbfConfig ToNative()
        {
            return new SwarmNative.CNfpLbfConfig
            {
                enabled = Enabled ? 1 : 0,
                n_orderings = Orderings,
                cache_budget_ms = CacheBudgetMs,
                total_budget_ms = TotalBudgetMs,
                allow_180 = Allow180 ? 1 : 0,
                enable_prefix_reuse = EnablePrefixReuse ? 1 : 0,
                concave_simplify_tolerance = ConcaveSimplifyTolerance
            };
        }
    }

    /// <summary>
    /// NFP <b>초기배치</b> 전략 7종. 항목마다 독립적으로 켜고 끈다.
    /// <para>전부 꺼짐이 기본값이며, 그때는 기존 NFP-LBF 경로가 한 비트도 바뀌지 않는다.</para>
    /// </summary>
    public class NfpStrategySettings
    {
        /// <summary>① 회전 후보를 인스턴스가 선언한 <c>allowed_orientations</c> 에서 그대로 뽑는다.</summary>
        public bool InstanceRotations { get; set; }

        /// <summary>① 연속 회전 샘플 간격(도). 0 이하 = 기본값(90).</summary>
        public int ContinuousRotationStepDeg { get; set; }

        /// <summary>② 잘 맞물리는 조각 쌍을 하나로 묶어 배치한다.</summary>
        public bool PairwiseClustering { get; set; }

        /// <summary>② 쌍 채택 최소 충전율 이득. 0 이하 = 기본값(0.05).</summary>
        public float ClusterMinGain { get; set; }

        /// <summary>③ 후보를 접촉 최대 기준으로 고른다.</summary>
        public bool ContactMaximization { get; set; }

        /// <summary>④ 프론티어를 늘리기 전에 내부 구멍부터 채운다.</summary>
        public bool PocketFirst { get; set; }

        /// <summary>⑤ 부분 배치를 여러 개 유지한다.</summary>
        public bool BeamSearch { get; set; }

        /// <summary>⑤ 빔 폭. 0 이하 = 기본값(4). 시간이 이 값에 정비례한다.</summary>
        public int BeamWidth { get; set; }

        /// <summary>⑥ 매 단계 가장 잘 맞는 조각을 고른다(순서 고정 없음).</summary>
        public bool BestFitSelection { get; set; }

        /// <summary>⑥ 매 단계 평가할 조각 종류 수 상한. 0 이하 = 기본값(12).</summary>
        public int BestFitLookahead { get; set; }

        /// <summary>⑦ 배치 후 각 조각을 좌하단으로 민다.</summary>
        public bool SlidingCompaction { get; set; }

        /// <summary>⑦ 슬라이딩 라운드 수. 0 이하 = 기본값(2).</summary>
        public int SlidingRounds { get; set; }

        /// <summary>
        /// ⑧ 순서를 정하지 않고 매 단계 가장 잘 맞물리는 조각을 고른다(누적 배치).
        /// <para>
        /// 켜면 ordering 을 <b>1개만</b> 평가한다 — 순서를 쓰지 않으므로 더 돌아도 같은
        /// 결과다. 실측(wsjk156, 탐침 8192): 71.75% → 72.93%, ⑦ 을 얹으면 73.09%.
        /// </para>
        /// </summary>
        public bool Accretion { get; set; }

        /// <summary>
        /// ⑨ 꼬리의 조각을 빼서 앞쪽 빈 곳으로 되돌린다.
        /// 실측(wsjk156, 5초 예산): 71.75% → 73.27% (+1.52%p).
        /// </summary>
        public bool TailReflow { get; set; }

        /// <summary>⑨ 꼬리 회수 라운드 수. 0 이하 = 기본값(2).</summary>
        public int ReflowRounds { get; set; }

        /// <summary>
        /// ⑧·⑨ 를 담는 ABI v11 tail block.
        /// <para>
        /// ⑦까지와 별도 구조체인 이유는 ABI 호환이다 — 기존 블록을 늘리면 뒤따르는
        /// <c>lp_solver</c> 오프셋이 밀려 구버전 호스트가 깨진다.
        /// </para>
        /// </summary>
        internal SwarmNative.CNfpStrategy2Config ToNative2()
        {
            return new SwarmNative.CNfpStrategy2Config
            {
                accretion = Accretion ? 1 : 0,
                tail_reflow = TailReflow ? 1 : 0,
                reflow_rounds = ReflowRounds,
                _reserved = 0
            };
        }

        internal SwarmNative.CNfpStrategyConfig ToNative()
        {
            return new SwarmNative.CNfpStrategyConfig
            {
                instance_rotations = InstanceRotations ? 1 : 0,
                continuous_rotation_step_deg = ContinuousRotationStepDeg,
                pairwise_clustering = PairwiseClustering ? 1 : 0,
                cluster_min_gain = ClusterMinGain,
                contact_maximization = ContactMaximization ? 1 : 0,
                pocket_first = PocketFirst ? 1 : 0,
                beam_search = BeamSearch ? 1 : 0,
                beam_width = BeamWidth,
                best_fit_selection = BestFitSelection ? 1 : 0,
                best_fit_lookahead = BestFitLookahead,
                sliding_compaction = SlidingCompaction ? 1 : 0,
                sliding_rounds = SlidingRounds
            };
        }
    }

    /// <summary>
    /// <b>실험</b> — 레퍼런스 설계 초기배치 엔진(<c>refplace</c>) 스위치.
    /// <para>
    /// 설계 문서를 그대로 구현한 별도 엔진이다. 구조는 완성됐고 겹침 없는 유효한 배치를
    /// 내지만 <b>품질이 아직 기존 경로에 크게 못 미친다</b>:
    /// </para>
    /// <code>
    ///   기존 NFP-LBF        폭  7,090mm   밀도 71.75%
    ///   전략 8+7+9          폭  6,900mm   밀도 73.72%
    ///   경쟁사              폭  6,183mm   밀도 82.28%
    ///   refplace (이 엔진)  폭 20,312mm   밀도 25.04%
    /// </code>
    /// <para>
    /// 앞 5,200mm 구간의 충전율은 76%로 경쟁사(81%)에 크게 뒤지지 않는다. 무너지는
    /// 지점은 하나다 — 배치가 진행되면 NFP 차감이 실패하기 시작해(실측 46회) 남은
    /// 조각이 프론티어 오른쪽으로 일렬로 밀린다. 원인은 규명 중이다.
    /// </para>
    /// </summary>
    public class RefPlaceSettings
    {
        /// <summary>Use bottom-to-top column Draft for a fresh Init. Requires Enabled.</summary>
        public bool ColumnDraft { get; set; }
        /// <summary>
        /// 켜면 기존 초기 배치를 이 엔진으로 <b>대체</b>한다. 기본 꺼짐.
        /// <para>
        /// 시드로 비교해 좋은 쪽을 고르지 않는 이유: 지금 품질 차이가 커서 비교로 넣으면
        /// 언제나 기존 쪽이 이겨 이 엔진이 도는지조차 확인할 수 없다.
        /// </para>
        /// </summary>
        public bool Enabled { get; set; }

        /// <summary>
        /// 그룹 정의가 없을 때 쓸 대체 규칙.
        /// 0 = 조각 하나가 한 그룹(체인 없음), 1 = 같은 조각의 사본끼리 한 그룹(기본).
        /// </summary>
        public int GroupFallback { get; set; } = 1;

        internal SwarmNative.CRefPlaceConfig ToNative(float? simplifyMaxDeviationMm = null)
        {
            if (simplifyMaxDeviationMm is float mm && (!float.IsFinite(mm) || mm <= 0))
                throw new ArgumentOutOfRangeException(nameof(simplifyMaxDeviationMm), "최대 이탈(mm)은 유한한 양수여야 합니다.");
            return new SwarmNative.CRefPlaceConfig
            {
                enabled = Enabled ? 1 : 0,
                group_fallback = GroupFallback,
                _reserved0 = ColumnDraft ? 1 : 0,
                simplify_max_deviation_mm = simplifyMaxDeviationMm ?? 0f
            };
        }
    }

    /// <summary>
    /// LBF 초기배치가 조각을 훑는 <b>방향</b>.
    /// <para>
    /// <b>기준 크기가 "면적"이 아니다.</b> 정렬 점수는 <c>볼록껍질 면적 × 직경</c> 이라,
    /// 오목한 부분이 메워진 면적에 직경이 곱해진다 — 길쭉한 조각이 앞으로 온다.
    /// 이 설정은 그 <b>점수의 정의는 건드리지 않고 순회 방향만</b> 바꾼다.
    /// </para>
    /// <para>
    /// 큰 것부터 넣으면 큰 조각이 좋은 자리를 먼저 잡고 작은 조각이 틈을 메운다.
    /// 중간부터는 양쪽을 번갈아 내보내 골격과 채움이 배치 내내 섞인다. 어느 쪽이
    /// 좋은지는 마커마다 다르므로 재 보는 수밖에 없다.
    /// </para>
    /// </summary>
    /// <summary>
    /// 원본 외곽선 겹침이 겹침 허용값(<c>piece_overlap_mm</c>)을 넘는 배치의 처리 (ABI v19).
    /// <para>
    /// 겹침을 허용하면 탐색은 조각을 허용값의 절반만큼 줄인 도형으로 하고, 결과를 원본
    /// 외곽선으로 다시 검증한다. 날카로운 꼭짓점은 줄이기에서 더 깎이므로, 줄인 도형이
    /// 겹치지 않아도 원본 외곽선은 허용값보다 깊이 겹칠 수 있다. 네이티브 로그
    /// <c>[ORIGINAL:VALIDATE] original overlap exceeds configured piece_overlap_mm</c> 가 그 경우다.
    /// </para>
    /// </summary>
    public enum OriginalOverlapPolicy
    {
        /// <summary>실현 불가능으로 보고 버린다. 기본값이자 기존 동작.</summary>
        Infeasible = 0,
        /// <summary>실현 가능으로 보고 채택한다. 원본 외곽선 검증을 끄고, 겹침은 줄인 도형으로만 판정한다.</summary>
        Feasible = 1,
    }

    /// <summary>
    /// V2 place groups(패턴 그룹)를 엔진에 넘기는 방식 (ABI v20).
    /// </summary>
    public enum PlaceGroupMode
    {
        /// <summary>그룹을 멤버 외곽선을 힌트 자세대로 합친 아이템 하나로 다룬다. 기본값.
        /// 멤버 상대 자세는 정의상 정확하고, 멤버 사이 틈에는 다른 조각이 들어갈 수 있다.</summary>
        MultiPolygon = 0,
        /// <summary>멤버를 별도 조각으로 두고 상대 좌표로 함께 옮긴다. 1.0.2 까지의 방식.</summary>
        Rigid = 1,
    }

    public enum LbfOrderMode
    {
        /// <summary>큰 것부터. 현행 동작이자 기본값.</summary>
        LargestFirst = 0,
        /// <summary>작은 것부터. 위 순서를 그대로 뒤집는다.</summary>
        SmallestFirst = 1,
        /// <summary>중간부터, 갈림길에서 <b>큰 쪽</b>을 먼저 (m, m-1, m+1, m-2, …).</summary>
        MiddleOutLargeFirst = 2,
        /// <summary>중간부터, 갈림길에서 <b>작은 쪽</b>을 먼저 (m, m+1, m-1, m+2, …).</summary>
        MiddleOutSmallFirst = 3,
        /// <summary>
        /// <b>방향성 오목도</b>부터. 크기가 아니라 <c>10·S_L + S_B + 2·S_C + 0.5·S_D</c>
        /// (왼쪽·아래로 열린 오목부 가중)로 정렬한다. 점수가 면적 정규화돼 크기와 무관하다.
        /// </summary>
        ConcavityFirst = 4,
        /// <summary>크기 × (1 + 오목도). 큰 것부터의 골격을 유지하며 오목한 조각을 앞당긴다.</summary>
        SizeTimesConcavity = 5
    }

    /// <summary>
    /// 폴리시 잔여 시간을 <b>무엇에</b> 쓸지.
    /// <para>
    /// 폴리시가 멈춘 지점은 <b>정제로는 못 벗어나는 고정점</b>이다 — 라운드 100,
    /// LP 반복 400, 정확 NFP 홀채움(예산 100% 소진) 모두 폭이 완전히 동일했다.
    /// </para>
    /// </summary>
    public enum ReclaimMode
    {
        /// <summary>
        /// <b>압축 재진입</b> (기본). 정제 계열 — 더 좁아진 폭에서 축소 스케줄을
        /// 새로 시작한다. 실측 4회: −17.11 / +0.00 / +0.00 / −8.00mm.
        /// </summary>
        CompressReentry = 0,
        /// <summary>
        /// <b>Ruin &amp; Recreate 교란 후 재탐색.</b> 탈출 계열 — 밴드 하나를 부수고
        /// 분리로 겹침을 푼 뒤 탐색을 재개한다. R&amp;R 자체는 폭을 줄이지 않는다;
        /// 재탐색이 폭을 줄이고 R&amp;R 은 같은 지역해로 되돌아가지 않게 하는 교란이다.
        /// </summary>
        RuinRecreate = 1,
        /// <summary>
        /// <b>적응형 탈출 탐색.</b> 정체를 감지해 단계적으로 강한 교란을 거는
        /// 탐색에 잔여를 돌린다.
        /// </summary>
        Escape = 2
    }

    /// <summary>어떤 엔진으로 압축 LP·QP 를 풀지.</summary>
    public enum LpSolverBackend
    {
        /// <summary>단체법, 순수 Rust. 항상 사용 가능. LP 전용. (기본값)</summary>
        Microlp = 0,
        /// <summary>내점법(barrier), 순수 Rust. <b>QP 를 푼다.</b></summary>
        Clarabel = 1,
        /// <summary>COIN-OR Clp. 런타임에 <c>clp.dll</c> / <c>libClp.so</c> 를 찾는다. LP 전용.</summary>
        Clp = 2,
        /// <summary>FICO Xpress. 런타임에 <c>xprs.dll</c> / <c>libxprs.so</c>. LP+QP. 라이선스 필요.</summary>
        Xpress = 3,
    }

    /// <summary>
    /// 압축 단계 LP·QP 솔버 선택.
    /// <para>
    /// Clp 와 Xpress 는 빌드 시점에 링크하지 않고 <b>런타임에 동적 로딩</b>한다.
    /// 그래야 SDK 없는 환경에서도 빌드가 깨지지 않는다. 라이브러리가 없으면
    /// <see cref="Fallback"/> 에 따라 microlp 로 물러나거나 압축을 건너뛴다.
    /// </para>
    /// </summary>
    public class LpSolverSettings
    {
        /// <summary>어떤 엔진으로 풀지. 기본값은 기존 동작(단체법).</summary>
        public LpSolverBackend Backend { get; set; } = LpSolverBackend.Microlp;

        /// <summary>
        /// 2차 정규화 계수. 0 이하면 순수 LP.
        /// <para>QP 를 푸는 백엔드(Clarabel, Xpress)에서만 의미가 있다.</para>
        /// </summary>
        public float QuadraticRegularization { get; set; }

        /// <summary>백엔드를 쓸 수 없을 때 microlp 로 물러날지. 기본 켜짐.</summary>
        public bool Fallback { get; set; } = true;

        /// <summary>한 번의 풀이 제한 시간(ms). 0 이하 = 기본값(5000).</summary>
        public int TimeLimitMs { get; set; }

        internal SwarmNative.CLpSolverConfig ToNative()
        {
            return new SwarmNative.CLpSolverConfig
            {
                backend = (int)Backend,
                quadratic_regularization = QuadraticRegularization,
                fallback = Fallback ? 1 : 0,
                time_limit_ms = TimeLimitMs
            };
        }
    }

    /// <summary>
    /// 탐색(exploration) 단계의 NFP 활용 범위.
    /// <para>전부 기본 꺼짐. 항목마다 따로 켜서 A/B 로 밀도와 시간을 재 봐야 한다.</para>
    /// </summary>
    public class NfpIntegrationSettings
    {
        /// <summary>탐색 단계를 순수 NFP 순서 탐색으로 교체한다. 적응형 탈출보다 우선한다.</summary>
        public bool OrderingExploration { get; set; }

        /// <summary>① separator 위치 샘플러를 NFP 후보로 교체한다.</summary>
        public bool NfpSampler { get; set; }

        /// <summary>② Conflict Burst 분리 벡터를 정확한 NFP 기반으로 바꾼다.</summary>
        public bool ExactSeparationVector { get; set; }

        /// <summary>③ hole filling 을 feasible_region 기반으로 바꾼다.</summary>
        public bool NfpHoleFilling { get; set; }

        /// <summary>④ Ruin &amp; Recreate 재삽입을 NFP 기반으로 바꾼다.</summary>
        public bool NfpReinsertion { get; set; }

        internal SwarmNative.CNfpIntegrationConfig ToNative()
        {
            return new SwarmNative.CNfpIntegrationConfig
            {
                ordering_exploration = OrderingExploration ? 1 : 0,
                nfp_sampler = NfpSampler ? 1 : 0,
                exact_separation_vector = ExactSeparationVector ? 1 : 0,
                nfp_hole_filling = NfpHoleFilling ? 1 : 0,
                nfp_reinsertion = NfpReinsertion ? 1 : 0
            };
        }
    }

    /// <summary>원단 허용오차 — 조각 사이 겹침.</summary>
    public class FabricToleranceSettings
    {
        /// <summary>
        /// 조각 사이에 허용할 겹침(mm). 0 이하 = 비활성(기본).
        /// 0보다 크면 <see cref="SwarmConfig.MinItemSeparation"/> 을 이긴다.
        /// 실측: 0.4 에서 strip 길이 -2.9%, 밀도 +1.95%p.
        /// </summary>
        public float PieceOverlapMm { get; set; }

        internal SwarmNative.CFabricToleranceConfig ToNative()
        {
            return new SwarmNative.CFabricToleranceConfig
            {
                piece_overlap_mm = PieceOverlapMm
            };
        }
    }

    /// <summary>
    /// <see cref="SwarmOptimizer.SolutionFound"/> 이벤트의 이벤트 Arguments.
    /// 최적화 진행 중 발견된 솔루션의 스칼라 정보와 JSON 문자열을 전달한다.
    /// </summary>
    public class SolutionEventArgs : EventArgs
    {
        public sealed record ParsedPlacement(
            int ItemId,
            float RotationDegrees,
            float TranslationX,
            float TranslationY);

        public sealed class PlaceCollisionStatus
        {
            /// <summary>Place Item ID</summary>
            [System.Text.Json.Serialization.JsonPropertyName("item_id")]
            public int ItemID { get; set; }

            /// <summary>충돌 여부</summary>
            [System.Text.Json.Serialization.JsonPropertyName("collision")]
            public bool IsCollision { get; set; }
        }

        /// <summary>보고 유형</summary>
        public ReportType ReportType { get; }

        /// <summary>현재 스트립 폭</summary>
        public float StripWidth { get; }

        /// <summary>현재 밀도(효율)</summary>
        public float Density { get; }

        /// <summary>
        /// 솔루션 전용 JSON 문자열.
        /// <c>strip_width</c>, <c>layout.placed_items</c>, <c>density</c> 필드를 포함한다.
        /// <see cref="SwarmOptimizer.EmitIntermediateJson"/>가 <c>false</c>이거나
        /// 실현 가능성 조사 보고일 때는 <c>null</c>이 전달된다.
        /// </summary>
        public string? SolutionJson { get; }

        /// <summary>
        /// 현재 레이아웃의 총 충돌 loss. 0이면 충돌 없음.
        /// <see cref="SolutionJson"/>이 <c>null</c>이면 의미 없음.
        /// </summary>
        public float Loss { get; }

        /// <summary>callback JSON에 loss가 실제로 포함되었는지 여부.</summary>
        public bool HasLoss { get; }

        /// <summary>각 Place의 충돌 상태를 가져옵니다.</summary>
        public IReadOnlyList<PlaceCollisionStatus> CollisionStatus { get; }

        /// <summary>callback JSON에 collision_status가 실제로 포함되었는지 여부.</summary>
        public bool HasCollisionStatus { get; }

        /// <summary>결합 JSON을 첫 파싱할 때 추출한 렌더링용 배치입니다.</summary>
        public IReadOnlyList<ParsedPlacement> ParsedPlacements { get; }

        /// <summary>결합 JSON solution의 strip width입니다.</summary>
        public float? ParsedStripWidth { get; }

        /// <summary>
        /// <see cref="SolutionEventArgs"/>의 새 인스턴스를 초기화한다.
        /// </summary>
        /// <param name="reportType">보고 유형</param>
        /// <param name="stripWidth">현재 스트립 폭</param>
        /// <param name="density">현재 밀도</param>
        /// <param name="solutionJson">솔루션 전용 JSON</param>
        /// <param name="loss">총 충돌 loss</param>
        /// <param name="collisionStatus">각 Place의 충돌 상태</param>
        public SolutionEventArgs(
            ReportType reportType,
            float stripWidth,
            float density,
            string? solutionJson,
            float loss,
            IReadOnlyList<PlaceCollisionStatus>? collisionStatus = null,
            bool hasLoss = true,
            bool hasCollisionStatus = true,
            IReadOnlyList<ParsedPlacement>? parsedPlacements = null,
            float? parsedStripWidth = null)
        {
            this.ReportType = reportType;
            this.StripWidth = stripWidth;
            this.Density = density;
            this.SolutionJson = solutionJson;
            this.Loss = loss;
            this.HasLoss = hasLoss;
            this.CollisionStatus = collisionStatus ?? Array.Empty<PlaceCollisionStatus>();
            this.HasCollisionStatus = hasCollisionStatus;
            this.ParsedPlacements = parsedPlacements ?? Array.Empty<ParsedPlacement>();
            this.ParsedStripWidth = parsedStripWidth;
        }

        /// <summary>
        /// 네이티브 콜백에서 받은 결합 JSON을 파싱하여 <see cref="SolutionEventArgs"/>를 생성한다.
        /// </summary>
        public static SolutionEventArgs FromCombinedJson(
            ReportType reportType,
            float stripWidth,
            float density,
            string? combinedJson)
        {
            if (string.IsNullOrWhiteSpace(combinedJson))
            {
                return new SolutionEventArgs(
                    reportType, stripWidth, density, null, 0,
                    hasLoss: false, hasCollisionStatus: false);
            }

            try
            {
                using var _doc = System.Text.Json.JsonDocument.Parse(combinedJson);
                var _root = _doc.RootElement;

                string? _solutionJson = null;
                float? _parsedStripWidth = null;
                var _parsedPlacements = new List<ParsedPlacement>();
                if (_root.TryGetProperty("solution", out var _solProp))
                {
                    _solutionJson = _solProp.GetRawText();
                    if (_solProp.TryGetProperty("strip_width", out var _stripWidthProp))
                    {
                        _parsedStripWidth = _stripWidthProp.GetSingle();
                    }
                    if (_solProp.TryGetProperty("layout", out var _layout)
                        && _layout.TryGetProperty("placed_items", out var _placedItems))
                    {
                        foreach (var placement in _placedItems.EnumerateArray())
                        {
                            var transformation = placement.GetProperty("transformation");
                            var translation = transformation.GetProperty("translation");
                            _parsedPlacements.Add(new ParsedPlacement(
                                placement.GetProperty("item_id").GetInt32(),
                                transformation.GetProperty("rotation").GetSingle(),
                                translation[0].GetSingle(),
                                translation[1].GetSingle()));
                        }
                    }
                }

                bool _hasLoss = _root.TryGetProperty("loss", out var _lossProp);
                float _loss = 0;
                if (_hasLoss)
                    _loss = _lossProp.GetSingle();

                bool _hasCollisionStatus = _root.TryGetProperty("collision_status", out var _statusProp);
                var _collisionStatus = _hasCollisionStatus
                    ? System.Text.Json.JsonSerializer.Deserialize<List<PlaceCollisionStatus>>(_statusProp.GetRawText())
                    : null;

                return new SolutionEventArgs(
                    reportType, stripWidth, density, _solutionJson, _loss, _collisionStatus,
                    _hasLoss, _hasCollisionStatus, _parsedPlacements, _parsedStripWidth);
            }
            catch
            {
                // 파싱 실패 시 JSON 없이 반환
                return new SolutionEventArgs(
                    reportType, stripWidth, density, null, 0f,
                    hasLoss: false, hasCollisionStatus: false);
            }
        }
    }

    /// <summary>
    /// 네이티브 진행 callback에서 복사된 원시 보고를 받는 관리 delegate입니다.
    /// </summary>
    /// <param name="reportType">보고 유형</param>
    /// <param name="stripWidth">현재 스트립 폭(mm)</param>
    /// <param name="density">현재 밀도(0~1)</param>
    /// <param name="combinedJson">
    /// callback 포인터의 유효 시간 안에 복사된 결합 JSON입니다.
    /// JSON이 첨부되지 않은 보고는 빈 문자열입니다.
    /// </param>
    /// <remarks>
    /// 이 delegate는 네이티브 callback 스레드에서 호출되므로 파싱이나 UI 갱신을 직접 수행하지 말고
    /// 관리 큐에 복사본을 전달한 뒤 즉시 반환해야 합니다. delegate에서 발생한 예외는 ABI 경계를
    /// 넘어가지 않도록 <see cref="SwarmStepOptimizer"/>가 차단합니다.
    /// </remarks>
    public delegate void RawSolutionCallback(
        ReportType reportType,
        float stripWidth,
        float density,
        string combinedJson);

    /// <summary>
    /// Swarm Rust cdylib에 대한 관리 래퍼.
    /// 2D 불규칙 스트립 패킹(2D Irregular Strip Packing) 최적화를 위한
    /// 깔끔한 C# API를 제공한다.
    ///
    /// <para><b>사용 패턴:</b></para>
    /// <list type="bullet">
    ///   <item><description><b>단계별 API:</b> <see cref="SwarmStepOptimizer"/>로 초기화 후 탐색과 압축을 순서대로 실행한다.</description></item>
    ///   <item><description><b>설정:</b> <see cref="GetDefaultConfig"/>로 기본값을 가져와 수정하거나,
    ///     <c>new SwarmConfig { ... }</c>로 직접 구성한다.</description></item>
    /// </list>
    ///
    /// </summary>
    public static class SwarmOptimizer
    {
        /// <summary>
        /// 이후 생성할 솔버의 기본 축소 스텝을 설정한다. 실행 중 변경은 SwarmStepOptimizer.TrySetShrinkStep을 사용한다.
        /// </summary>
        /// <param name="step">현재 폭 대비 축소 비율 (0 &lt; step &lt; 1).</param>
        /// <returns>적용 성공 여부. 범위 밖 값이면 <c>false</c>이고 기존 값이 유지된다.</returns>
        public static bool TrySetShrinkStep(double step)
        {
            SwarmNative.EnsureCompatible();
            return SwarmNative.swarm_set_shrink_step((float)step) != 0;
        }

        /// <summary>이후 생성할 솔버의 기본 축소 스텝을 해제한다.</summary>
        public static void ClearShrinkStep()
        {
            SwarmNative.EnsureCompatible();
            SwarmNative.swarm_clear_shrink_step();
        }

        /// <summary>현재 축소 스텝 오버라이드. 설정되어 있지 않으면 <c>null</c>.</summary>
        public static double? CurrentShrinkStep()
        {
            SwarmNative.EnsureCompatible();
            float v = SwarmNative.swarm_get_shrink_step();
            return v > 0.0f ? v : null;
        }

        // ── 네이티브 로그 ──────────────────────────────────────────────────
        //
        // Rust 의 `log` 크레이트는 로거가 등록되지 않으면 모든 매크로가 no-op 이다.
        // cdylib 경로에는 로거를 설치하는 코드가 없어서, `[INIT] NFP-LBF 시드 채택/기각`
        // 같은 진단이 한 줄도 나오지 않았다. 아래가 그 눈을 만들어 준다.

        /// <summary>델리게이트를 GC 로부터 지킨다. 이게 없으면 네이티브가 죽은 포인터를 부른다.</summary>
        private static SwarmNative.SwarmLogCallback? _logCallback;

        /// <summary>등록된 관리 쪽 수신자. 여러 스레드에서 동시에 불린다.</summary>
        private static Action<int, string>? _logSink;

        /// <summary>
        /// 네이티브 로그를 관리 코드로 받는다. 이걸 부르기 전까지 네이티브는 아무 로그도 내지 않는다.
        /// <para>
        /// <paramref name="sink"/>는 <b>아무 스레드에서나, 동시에</b> 불린다(rayon 워커 포함).
        /// 스레드 안전해야 하고, UI 를 직접 만지면 안 된다. 예외를 던지면 그 줄은 버려지고
        /// <see cref="DroppedLogCount"/>가 올라간다.
        /// </para>
        /// </summary>
        /// <param name="sink">
        /// <c>(level, message)</c> 수신자. level 은 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Trace.
        /// <c>null</c>이면 등록을 해제한다.
        /// </param>
        /// <param name="maxLevel">
        /// 0=Off, 1=Error, 2=Warn, 3=Info(기본), 4=Debug, 5=Trace.
        /// 릴리스 빌드는 <c>release_max_level_info</c> 때문에 3 을 넘어서 나오지 않는다.
        /// </param>
        /// <returns>1 = 성공, 0 = 실패(다른 로거가 이미 프로세스에 설치됨).</returns>
        public static bool SetLogSink(Action<int, string>? sink, int maxLevel = 3)
        {
            SwarmNative.EnsureCompatible();
            _logSink = sink;
            if (sink is null)
            {
                int cleared = SwarmNative.swarm_set_log_callback(null, UIntPtr.Zero, 0);
                _logCallback = null;
                return cleared != 0;
            }

            // 델리게이트는 한 번만 만들어 재사용한다. 매번 새로 만들면 이전 것이 수거된다.
            _logCallback ??= OnNativeLog;
            return SwarmNative.swarm_set_log_callback(_logCallback, UIntPtr.Zero, maxLevel) != 0;
        }

        private static void OnNativeLog(int level, IntPtr message, UIntPtr userData)
        {
            var sink = _logSink;
            if (sink is null || message == IntPtr.Zero)
            {
                return;
            }
            // 네이티브 경계를 넘어 예외가 나가면 UB 다. 여기서 삼킨다.
            // (네이티브도 catch_unwind 로 한 번 더 막지만, 여기서 막는 편이 싸다.)
            try
            {
                string? text = Marshal.PtrToStringUTF8(message);
                if (!string.IsNullOrEmpty(text))
                {
                    sink(level, text);
                }
            }
            catch
            {
                // 로그 한 줄 때문에 최적화를 죽일 수는 없다.
            }
        }

        /// <summary>
        /// 콜백으로 넘기지 못하고 버린 로그 줄 수. 정상 동작에서는 0 이다.
        /// </summary>
        public static ulong DroppedLogCount
        {
            get
            {
                SwarmNative.EnsureCompatible();
                return SwarmNative.swarm_log_dropped_count();
            }
        }

        /// <summary>
        /// 네이티브 Swarm 라이브러리의 버전 문자열을 가져온다.
        /// 내부적으로 <see cref="SwarmNative.swarm_version"/>을 호출하고
        /// 반환된 문자열 포인터를 <see cref="MarshalExtension.PtrToStringUTF8"/>로 변환한다.
        /// (Rust 측 문자열은 정적 수명이므로 <c>swarm_free_string</c> 호출이 필요 없다.)
        /// </summary>
        public static string Version
        {
            get
            {
                SwarmNative.EnsureCompatible();
                IntPtr _ptr = SwarmNative.swarm_version();
                return MarshalExtension.PtrToStringUTF8(_ptr) ?? "unknown";
            }
        }

        /// <summary>
        /// 네이티브 기본 설정값을 가져와 관리 객체로 변환한다.
        /// 내부적으로 <see cref="SwarmNative.swarm_get_default_config"/>를 호출한 뒤
        /// <see cref="ConfigConverter.FromNative"/>로 변환한다.
        /// </summary>
        /// <returns>네이티브 기본값으로 채워진 <see cref="SwarmConfig"/></returns>
        public static SwarmConfig GetDefaultConfig()
        {
            SwarmNative.EnsureCompatible();
            var _native = new SwarmNative.CSwarmConfig();
            SwarmNative.swarm_get_default_config(ref _native);
            return ConfigConverter.FromNative(_native);
        }

    }

    /// <summary>
    /// 단계별(Step-by-step) API Wrapper.
    /// 페이즈별로 실행하며, <see cref="RequestStop"/>으로 실행 중인 최적화를 즉시 중단할 수 있다.
    ///
    /// <para><b>사용 패턴:</b></para>
    /// <code>
    /// using var _step = new SwarmStepOptimizer();
    /// _step.Init(inputPath, outputDir);
    /// _step.SetCallback(callback, emitJson: true);
    /// _step.ExplorePhase();   // 백그라운드 스레드에서 실행
    /// _step.CompressPhase();  // 백그라운드 스레드에서 실행
    /// _step.PolishPhase();    // 백그라운드 스레드에서 실행 (선택)
    /// var _json = _step.GetSolutionJson();
    /// // 도중에 중단하려면: _step.RequestStop();
    /// </code>
    /// </summary>
    public class SwarmStepOptimizer : IDisposable
    {
        private readonly object _CallbackSync = new();
        private IntPtr _SwarmHandle;
        private SwarmNative.SolutionCallback? _CallbackDelegate;
        private bool _IsDisposed;
        private bool _PhaseRunning;
        private int _StopWaiters;
        private int _ActiveSolutionGetters;
        private ErrorSnapshot _LastErrorSnapshot = ErrorSnapshot.None;

        private sealed class ErrorSnapshot
        {
            public static readonly ErrorSnapshot None = new(0, string.Empty);

            public ErrorSnapshot(int code, string message)
            {
                this.Code = code;
                this.Message = message;
            }

            public int Code { get; }

            public string Message { get; }
        }

        /// <summary>Swarm 네이티브 핸들. <see cref="Init"/> Or <see cref="InitFromJson(string, SwarmConfig, CPlacedHint[])"/> 호출 후 유효.</summary>
        public IntPtr SwarmHandle => this._SwarmHandle;

        /// <summary>초기화 여부</summary>
        public bool IsInitialized => this._SwarmHandle != IntPtr.Zero;

        /// <summary>가장 최근 Init, phase 또는 solution export 실패 코드(기존 호환 이름). 성공 시 0.</summary>
        public int LastInitErrorCode => Volatile.Read(ref this._LastErrorSnapshot).Code;

        /// <summary>가장 최근 Init, phase 또는 solution export 실패 상세(기존 호환 이름).</summary>
        public string LastInitError => Volatile.Read(ref this._LastErrorSnapshot).Message;

        /// <summary>
        /// 가장 최근 Init, phase 또는 solution export 실패 코드.
        /// 기존 <see cref="LastInitErrorCode"/>와 같은 저장소를 사용한다.
        /// </summary>
        public int LastErrorCode => this.LastInitErrorCode;

        /// <summary>가장 최근 Init, phase 또는 solution export 실패 상세.</summary>
        public string LastError => this.LastInitError;

        private void ClearLastError()
        {
            Volatile.Write(ref this._LastErrorSnapshot, ErrorSnapshot.None);
        }

        private void SetManagedError(int code, string message)
        {
            Volatile.Write(ref this._LastErrorSnapshot, new ErrorSnapshot(code, message));
        }

        private void CaptureNativeError()
        {
            int _code = SwarmNative.swarm_get_last_error_code();
            string _message;
            var _ptr = SwarmNative.swarm_get_last_error_message();
            if (_ptr == IntPtr.Zero)
            {
                _message = string.Empty;
            }
            else
            {
                try
                {
                    _message = MarshalExtension.PtrToStringUTF8(_ptr) ?? string.Empty;
                }
                finally
                {
                    SwarmNative.swarm_free_string(_ptr);
                }
            }
            this.SetManagedError(_code, _message);
        }

        private void CaptureLastInitError()
        {
            if (this._SwarmHandle != IntPtr.Zero)
            {
                this.ClearLastError();
                return;
            }
            this.CaptureNativeError();
        }

        /// <summary>현재 각 Place의 충돌 상태를 가져온다.</summary>
        public string GetCollisionStatusJson()
        {
            if (this._SwarmHandle == IntPtr.Zero)
            {
                return string.Empty;
            }

            var _ptr = SwarmNative.swarm_get_collision_status_json(this._SwarmHandle);
            if (_ptr == IntPtr.Zero)
            {
                return string.Empty;
            }

            try
            {
                return MarshalExtension.PtrToStringUTF8(_ptr) ?? string.Empty;
            }
            finally
            {
                SwarmNative.swarm_free_string(_ptr);
            }
        }

        /// <summary>현재 솔루션의 각 Place 충돌 상태를 관리 객체 목록으로 가져온다.</summary>
        public IReadOnlyList<SolutionEventArgs.PlaceCollisionStatus> GetCollisionStatus()
        {
            if (this._SwarmHandle == IntPtr.Zero)
            {
                return Array.Empty<SolutionEventArgs.PlaceCollisionStatus>();
            }

            UIntPtr _nativeColCount;
            IntPtr _ptr = SwarmNative.swarm_get_collision_status(this._SwarmHandle, out _nativeColCount);
            ulong _count64 = _nativeColCount.ToUInt64();
            if (_ptr == IntPtr.Zero || _count64 == 0)
            {
                return Array.Empty<SolutionEventArgs.PlaceCollisionStatus>();
            }

            int _count = checked((int)_count64);
            int _elementSize = Marshal.SizeOf<CCollisionStatus>();
            var _statuses = new List<SolutionEventArgs.PlaceCollisionStatus>(_count);
            try
            {
                for (int i = 0; i < _count; i++)
                {
                    var _nativeStruct = Marshal.PtrToStructure<CCollisionStatus>(IntPtr.Add(_ptr, checked(i * _elementSize)));
                    _statuses.Add(new SolutionEventArgs.PlaceCollisionStatus
                    {
                        ItemID = checked((int)_nativeStruct.ItemId.ToUInt64()),
                        IsCollision = _nativeStruct.Collision != 0
                    });
                }
                return _statuses;
            }
            finally
            {
                SwarmNative.swarm_free_collision_status(_ptr, _nativeColCount);
            }
        }

        /// <summary>
        /// 기본 설정으로 입력 파일을 로드해 초기화한다.
        /// </summary>
        /// <param name="inputPath">입력 JSON 파일 경로</param>
        /// <param name="hints">배치 힌트 리스트 (null 가능)</param>
        /// <returns>성공 시 true</returns>
        public bool Init(string inputPath, CPlacedHint[]? hints = null)
        {
            if (this._IsDisposed)
            {
                throw new ObjectDisposedException(nameof(SwarmStepOptimizer));
            }
            SwarmNative.EnsureCompatible();
            // 영값 구조체를 넘기면 시간 제한이 0이 되어 모든 페이즈가 즉시 끝난다. 반드시 네이티브 기본값을 받아서 쓴다.
            var _defaultCfg = new SwarmNative.CSwarmConfig();
            SwarmNative.swarm_get_default_config(ref _defaultCfg);
            this._SwarmHandle = SwarmNative.swarm_init(inputPath, ref _defaultCfg, hints ?? Array.Empty<CPlacedHint>(), hints?.Length ?? 0);
            this.CaptureLastInitError();
            return this._SwarmHandle != IntPtr.Zero;
        }

        /// <summary>
        /// 사용자 설정으로 초기화한다.
        /// </summary>
        /// <param name="inputPath">입력 JSON 파일 경로</param>
        /// <param name="config">최적화 설정</param>
        /// <param name="hints">배치 힌트 리스트 (null 가능)</param>
        /// <returns>성공 시 true</returns>
        public bool Init(string inputPath, SwarmConfig config, CPlacedHint[]? hints = null)
        {
            if (this._IsDisposed)
            {
                throw new ObjectDisposedException(nameof(SwarmStepOptimizer));
            }
            SwarmNative.EnsureCompatible();
            var nativeCfg = config.ToNative();
            this._SwarmHandle = SwarmNative.swarm_init(inputPath, ref nativeCfg, hints ?? Array.Empty<CPlacedHint>(), hints?.Length ?? 0);
            this.CaptureLastInitError();
            return this._SwarmHandle != IntPtr.Zero;
        }

        /// <summary>
        /// JSON 문자열로부터 최적화기를 초기화한다.
        /// </summary>
        /// <param name="inputJson">입력 JSON 문자열 (인스턴스 또는 솔루션 포함 출력 형식)</param>
        /// <param name="config">최적화 설정 (null이면 네이티브 기본값)</param>
        /// <param name="hints">배치 힌트 리스트 (null 가능)</param>
        /// <returns>성공 시 true</returns>
        public bool InitFromJson(
            string inputJson,
            SwarmConfig? config = null,
            CPlacedHint[]? hints = null)
        {
            if (this._IsDisposed)
            {
                throw new ObjectDisposedException(nameof(SwarmStepOptimizer));
            }
            SwarmNative.EnsureCompatible();
            var _nativeCfg = (config ?? SwarmOptimizer.GetDefaultConfig()).ToNative();
            this._SwarmHandle = SwarmNative.swarm_init_from_json(inputJson, ref _nativeCfg, hints ?? Array.Empty<CPlacedHint>(), hints?.Length ?? 0);
            this.CaptureLastInitError();
            return this._SwarmHandle != IntPtr.Zero;
        }

        /// <summary>
        /// 진행 상황 콜백을 등록한다. <see cref="ExplorePhase"/> 호출 전에 호출해야 한다.
        /// 기존 API 호환을 위해 결합 JSON을 callback 스레드에서 <see cref="SolutionEventArgs"/>로
        /// 변환한다. 큰 JSON을 비동기 처리해야 하는 UI/서버 코드는 <see cref="SetRawCallback"/>을
        /// 사용해 원문을 작업 큐로 넘기는 것이 좋다.
        /// </summary>
        /// <param name="callback">콜백</param>
        /// <param name="emitJson">중간 JSON 포함 여부</param>
        public void SetCallback(EventHandler<SolutionEventArgs>? callback, bool emitJson)
        {
            if (callback == null)
            {
                this.SetRawCallback(null, emitJson: false);
                return;
            }

            this.SetRawCallback(
                (reportType, stripWidth, density, combinedJson) =>
                    callback.Invoke(
                        this,
                        SolutionEventArgs.FromCombinedJson(
                            reportType,
                            stripWidth,
                            density,
                            combinedJson)),
                emitJson);
        }

        /// <summary>
        /// 진행 상황의 결합 JSON 원문을 복사해 전달하는 저비용 callback을 등록합니다.
        /// <see cref="ExplorePhase"/>, <see cref="CompressPhase"/>, <see cref="PolishPhase"/>,
        /// <see cref="StructurePhase"/> 호출 전에 등록해야 합니다.
        /// </summary>
        /// <param name="callback">
        /// 복사된 원시 보고를 받을 callback입니다. <see langword="null"/>이면 기존 callback을 해제합니다.
        /// callback은 네이티브 스레드에서 실행되므로 큐 적재 외의 무거운 작업을 수행하면 안 됩니다.
        /// </param>
        /// <param name="emitJson">중간 결합 JSON 포함 여부</param>
        public void SetRawCallback(RawSolutionCallback? callback, bool emitJson)
        {
            lock (this._CallbackSync)
            {
                if (this._IsDisposed)
                {
                    throw new ObjectDisposedException(nameof(SwarmStepOptimizer));
                }
                if (this._PhaseRunning)
                {
                    throw new InvalidOperationException(
                        "실행 중인 native phase가 반환된 뒤 callback을 변경해야 합니다.");
                }
                if (this._SwarmHandle == IntPtr.Zero)
                {
                    return;
                }

                if (callback == null)
                {
                    // 네이티브가 더 이상 delegate를 호출하지 않도록 해제한 뒤 GC root를 제거한다.
                    SwarmNative.swarm_set_callback(this._SwarmHandle, null, IntPtr.Zero, 0);
                    this._CallbackDelegate = null;
                    return;
                }

                SwarmNative.SolutionCallback nativeCallback = CreateRawCallbackBridge(callback);
                // 등록 호출 전에 field에 보관하여 네이티브가 즉시 참조해도 delegate가 살아 있도록 한다.
                this._CallbackDelegate = nativeCallback;
                SwarmNative.swarm_set_callback(
                    this._SwarmHandle,
                    nativeCallback,
                    IntPtr.Zero,
                    emitJson ? 1 : 0);
            }
        }

        /// <summary>
        /// 네이티브 포인터를 callback 유효 시간 안에 관리 문자열로 복사하고 ABI 경계의 모든 관리 예외를
        /// 차단하는 bridge를 만듭니다.
        /// </summary>
        internal static SwarmNative.SolutionCallback CreateRawCallbackBridge(RawSolutionCallback callback)
        {
            ArgumentNullException.ThrowIfNull(callback);

            return (reportType, stripWidth, density, solutionJson, userData) =>
            {
                try
                {
                    string combinedJson = solutionJson == IntPtr.Zero
                        ? string.Empty
                        : MarshalExtension.PtrToStringUTF8(solutionJson) ?? string.Empty;
                    callback((ReportType)reportType, stripWidth, density, combinedJson);
                }
                catch
                {
                    // 관리 예외가 unmanaged callback 경계를 통과하면 프로세스가 종료될 수 있다.
                    // 소비자 오류는 이 경계에서 반드시 차단한다.
                }
            };
        }

        /// <summary>
        /// N06 초기배치(Initial Placement) 페이즈를 실행한다. 초기화 직후에만 유효하며
        /// LBF 초기해를 N06 엔진 결과로 교체한다. 동기 블로킹 호출이므로
        /// 백그라운드 스레드에서 호출해야 한다.
        /// <see cref="RequestStop"/>으로 중단 가능하다.
        /// </summary>
        /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
        public int InitialPhase()
        {
            return this.RunPhase(SwarmNative.swarm_init_phase);
        }

        /// <summary>
        /// 탐색(Exploration) 페이즈를 실행한다. 동기 블로킹 호출이므로
        /// 백그라운드 스레드에서 호출해야 한다.
        /// 탐색·압축·후처리는 순서와 횟수가 자유다 — 어느 호출이든 직전 호출이 확정한
        /// 인증 해에서 출발한다. Structure 이후에는 호출할 수 없다.
        /// <see cref="RequestStop"/>으로 중단 가능하다.
        /// </summary>
        /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
        public int ExplorePhase()
        {
            return this.RunPhase(SwarmNative.swarm_alg_phase);
        }

        /// <summary>
        /// 압축(Compression) 페이즈를 실행한다. 동기 블로킹 호출이므로
        /// 백그라운드 스레드에서 호출해야 한다.
        /// <see cref="RequestStop"/>으로 중단 가능하다.
        /// </summary>
        /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
        public int CompressPhase()
        {
            return this.RunPhase(SwarmNative.swarm_cp_phase);
        }

        /// <summary>
        /// 후처리(Polish) 페이즈를 실행한다. 동기 블로킹 호출이므로
        /// 백그라운드 스레드에서 호출해야 한다.
        /// <para>
        /// 압축 결과에 대해 (1) 각 도형을 접촉할 때까지 좌측/하단으로 미는 결정론적 슬라이딩,
        /// (2) 빈 포켓을 찾아 작은 조각을 옮겨 넣는 hole filling,
        /// (3) LP compaction을 순서대로 적용한다. 모든 이동은 충돌 검증을 거치므로
        /// 결과가 압축 단계보다 나빠지지 않는다.
        /// </para>
        /// <see cref="CompressPhase"/> 이후에 호출해야 하며,
        /// <see cref="RequestStop"/>으로 중단 가능하다.
        /// </summary>
        /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
        public int PolishPhase()
        {
            return this.RunPhase(SwarmNative.swarm_refine_phase);
        }

        /// <summary>
        /// 초기화 직후 P0-P5 structure-first 대체 파이프라인을 동기 실행한다.
        /// 설정의 <see cref="StructureFirstConfig.Enabled"/>가 켜져 있어야 하며,
        /// 실행 중 <see cref="RequestStop"/>으로 중단할 수 있다.
        /// </summary>
        /// <returns>성공 시 0, 호출 순서/설정 또는 실행 오류 시 0이 아닌 값.</returns>
        public int StructurePhase()
        {
            return this.RunPhase(SwarmNative.swarm_gp_phase);
        }

        private int RunPhase(Func<IntPtr, int> phase)
        {
            IntPtr handle;
            lock (this._CallbackSync)
            {
                if (this._IsDisposed)
                {
                    this.SetManagedError(3000, "Phase rejected: the Swarm optimizer is disposed.");
                    return -1;
                }
                if (this._SwarmHandle == IntPtr.Zero)
                {
                    this.SetManagedError(3000, "Phase rejected: the Swarm optimizer is not initialized.");
                    return -1;
                }
                if (this._PhaseRunning)
                {
                    this.SetManagedError(3000, "Phase rejected: another Swarm phase is already running.");
                    return -1;
                }
                this._PhaseRunning = true;
                handle = this._SwarmHandle;
            }

            try
            {
                int result = phase(handle);
                if (result == 0)
                {
                    this.ClearLastError();
                }
                else
                {
                    this.CaptureNativeError();
                }
                return result;
            }
            finally
            {
                GC.KeepAlive(this._CallbackDelegate);
                lock (this._CallbackSync)
                {
                    this._PhaseRunning = false;
                    Monitor.PulseAll(this._CallbackSync);
                }
            }
        }

        /// <summary>
        /// 최근 structure-first 실행의 P0-P5 체크포인트와 진단 보고서 JSON을 반환한다.
        /// 실행 전이거나 보고서가 없으면 빈 문자열을 반환한다.
        /// </summary>
        public string GetStructureReportJson()
        {
            if (this._SwarmHandle == IntPtr.Zero)
            {
                return string.Empty;
            }

            IntPtr _ptr = SwarmNative.swarm_get_structure_report_json(this._SwarmHandle);
            if (_ptr == IntPtr.Zero)
            {
                return string.Empty;
            }

            try
            {
                return MarshalExtension.PtrToStringUTF8(_ptr) ?? string.Empty;
            }
            finally
            {
                SwarmNative.swarm_free_string(_ptr);
            }
        }

        /// <summary>
        /// 최종 솔루션 JSON을 반환한다.
        /// </summary>
        /// <returns>JSON 문자열. 실패 시 null.</returns>
        public string GetSolutionJson()
        {
            IntPtr _handle;
            lock (this._CallbackSync)
            {
                if (this._IsDisposed)
                {
                    this.SetManagedError(3000, "Solution export rejected: the Swarm optimizer is disposed.");
                    return string.Empty;
                }
                if (this._SwarmHandle == IntPtr.Zero)
                {
                    this.SetManagedError(3000, "Solution export rejected: the Swarm optimizer is not initialized.");
                    return string.Empty;
                }
                _handle = this._SwarmHandle;
                this._ActiveSolutionGetters++;
            }

            IntPtr _ptr = IntPtr.Zero;
            try
            {
                _ptr = SwarmNative.swarm_get_solution_json(_handle);
                if (_ptr == IntPtr.Zero)
                {
                    this.CaptureNativeError();
                    return string.Empty;
                }

                string _json = MarshalExtension.PtrToStringUTF8(_ptr) ?? string.Empty;
                this.ClearLastError();
                return _json;
            }
            finally
            {
                if (_ptr != IntPtr.Zero)
                {
                    SwarmNative.swarm_free_string(_ptr);
                }
                lock (this._CallbackSync)
                {
                    this._ActiveSolutionGetters--;
                    Monitor.PulseAll(this._CallbackSync);
                }
            }
        }

        /// <summary>이 솔버의 실행 중 축소 스텝만 변경한다.</summary>
        public bool TrySetShrinkStep(double step)
        {
            lock (this._CallbackSync)
            {
                return !this._IsDisposed && this._SwarmHandle != IntPtr.Zero
                    && SwarmNative.swarm_set_handle_shrink_step(this._SwarmHandle, (float)step) != 0;
            }
        }

        public void ClearShrinkStep()
        {
            lock (this._CallbackSync)
            {
                if (!this._IsDisposed && this._SwarmHandle != IntPtr.Zero)
                    SwarmNative.swarm_clear_handle_shrink_step(this._SwarmHandle);
            }
        }

        public double? CurrentShrinkStep()
        {
            lock (this._CallbackSync)
            {
                if (this._IsDisposed || this._SwarmHandle == IntPtr.Zero) return null;
                float value = SwarmNative.swarm_get_handle_shrink_step(this._SwarmHandle);
                return value > 0 ? value : null;
            }
        }

        /// <summary>
        /// 실행 중인 페이즈에 즉시 중단을 요청한다.
        /// 백그라운드 스레드에서 실행 중인 <see cref="ExplorePhase"/> 또는
        /// <see cref="CompressPhase"/>가 다음 반복문 확인 시 종료된다.
        /// UI 스레드에서 호출 가능하다. 중단 요청은 이 핸들의 수명 동안 유지된다.
        /// </summary>
        public void RequestStop()
        {
            lock (this._CallbackSync)
            {
                if (this._SwarmHandle != IntPtr.Zero)
                {
                    SwarmNative.swarm_request_stop(this._SwarmHandle);
                }
            }
        }

        /// <summary>네이티브 Init 이 <see cref="CancelInit"/>으로 중단됐을 때의 오류 코드.</summary>
        public const int InitCancelledErrorCode = 2002;

        /// <summary>
        /// 진행 중인 <see cref="InitFromJson(string, SwarmConfig, CPlacedHint[])"/>/<see cref="Init(string, SwarmConfig, CPlacedHint[])"/>를
        /// 취소한다. Init 은 핸들이 생기기 전이라 <see cref="RequestStop"/>이 닿지 않는다. 취소된 Init 은
        /// false 를 돌려주고 <see cref="LastErrorCode"/>가 <see cref="InitCancelledErrorCode"/>가 된다.
        /// 어떤 스레드에서든 호출 가능하다.
        /// </summary>
        public void CancelInit()
        {
            SwarmNative.swarm_cancel_init();
        }

        /// <summary>마지막 Init 이 사용자 취소로 끝났는가.</summary>
        public bool LastInitWasCancelled => this.LastErrorCode == InitCancelledErrorCode;

        /// <summary>
        /// 실행 중인 페이즈에 중단을 요청하고, 실제로 페이즈가 종료될 때까지 블록한다.
        /// 실행 중인 페이즈가 없으면 즉시 반환한다. 중단 요청은 이 핸들의 수명 동안 유지된다.
        /// 솔루션 콜백 안에서 호출하면 자기 자신을 기다리지 않고 중단 요청만 기록한 뒤 반환한다.
        /// </summary>
        public void RequestStopAndWait()
        {
            IntPtr handle;
            lock (this._CallbackSync)
            {
                if (this._SwarmHandle == IntPtr.Zero)
                {
                    return;
                }
                handle = this._SwarmHandle;
                this._StopWaiters++;
            }
            try
            {
                // callback이 RequestStop을 다시 호출할 수 있으므로 blocking native wait 동안
                // callback gate를 잡지 않는다. Dispose는 _StopWaiters가 0이 될 때까지 기다린다.
                SwarmNative.swarm_request_stop_and_wait(handle);
            }
            finally
            {
                lock (this._CallbackSync)
                {
                    this._StopWaiters--;
                    Monitor.PulseAll(this._CallbackSync);
                }
            }
        }

        /// <summary>
        /// 고정할 item_id 목록을 설정한다. ExplorePhase/CompressPhase 호출 전에 호출해야 한다.
        /// 고정된 아이템은 separation 중에 초기 위치에서 이동하지 않는다.
        /// </summary>
        /// <param name="itemIds">고정할 item ID 배열</param>
        public void FreezeItems(int[] itemIds)
        {
            if (this._SwarmHandle != IntPtr.Zero && itemIds != null && itemIds.Length > 0)
            {
                SwarmNative.swarm_freeze_items(this._SwarmHandle, itemIds, itemIds.Length);
            }
        }

        /// <summary>
        /// C#에서 계산한 바운딩 박스 배열로 AABB 1차 필터 + CDE 정밀 검사.
        /// handle이 null이면 AABB-only, handle이 있으면 실제 폴리곤 충돌만 반환한다.
        /// </summary>
        /// <param name="handle">SwarmStepOptimizer의 내부 핸들 (또는 null)</param>
        /// <param name="bounds">CItemBounds 배열 (item_id, min_x, min_y, max_x, max_y)</param>
        /// <returns>실제 충돌 중인 item_id 배열 (중복 없음, 정렬됨). 충돌 없으면 빈 배열.</returns>
        public static int[] FindBoundCollisions(IntPtr handle, CItemBounds[] bounds)
        {
            if (bounds == null || bounds.Length <= 1)
            {
                return Array.Empty<int>();
            }

            SwarmNative.EnsureCompatible();
            IntPtr _ptr = SwarmNative.swarm_find_bound_collisions(handle, bounds, bounds.Length);
            if (_ptr == IntPtr.Zero)
            {
                return Array.Empty<int>();
            }

            string _json = MarshalExtension.PtrToStringUTF8(_ptr) ?? string.Empty;
            SwarmNative.swarm_free_string(_ptr);

            try
            {
                return System.Text.Json.JsonSerializer.Deserialize<int[]>(_json) ?? Array.Empty<int>();
            }
            catch
            {
                return Array.Empty<int>();
            }
        }

        /// <summary>
        /// 실행 중인 페이즈를 중단하고 기다린 뒤 핸들과 관련 리소스를 해제한다.
        /// 솔루션 콜백 안에서 직접 호출하지 말고 콜백이 반환된 뒤 다른 스레드에서 호출해야 한다.
        /// </summary>
        public void Dispose()
        {
            IntPtr handle;
            lock (this._CallbackSync)
            {
                if (this._IsDisposed)
                {
                    return;
                }
                this._IsDisposed = true;
                handle = this._SwarmHandle;
            }

            if (handle != IntPtr.Zero)
            {
                SwarmNative.swarm_request_stop_and_wait(handle);
                lock (this._CallbackSync)
                {
                    while (this._PhaseRunning || this._StopWaiters > 0 || this._ActiveSolutionGetters > 0)
                    {
                        Monitor.Wait(this._CallbackSync);
                    }
                    SwarmNative.swarm_free(handle);
                    this._SwarmHandle = IntPtr.Zero;
                }
            }

            lock (this._CallbackSync)
            {
                this._CallbackDelegate = null;
            }
        }
    }

    /// <summary>
    /// 네이티브 <see cref="SwarmNative.CSwarmConfig"/>를 관리 <see cref="SwarmConfig"/>로 변환하는
    /// 내부 정적 헬퍼 클래스.
    ///
    /// <para>변환 규칙:</para>
    /// <list type="bullet">
    ///   <item><description>네이티브 값 ≥ 0이면 해당 값을 사용, 음수(-1f 등)이면 <c>null</c>로 복원</description></item>
    ///   <item><description>하위 설정 객체도 재귀적으로 변환</description></item>
    /// </list>
    /// </summary>
    // ── Helper: convert native config to managed ────────────────

    internal static class ConfigConverter
    {
        /// <summary>네이티브 <see cref="SwarmNative.CSeparatorConfig"/>를 관리 <see cref="SeparatorConfig"/>로 변환한다.</summary>
        private static SeparatorConfig SeparatorFromNative(SwarmNative.CSeparatorConfig native)
        {
            return new SeparatorConfig
            {
                IterNoImprovementLimit = native.iter_no_imprv_limit,
                StrikeLimit = native.strike_limit,
                NumWorkers = native.n_workers,
                NumContainerSamples = native.n_container_samples,
                NumFocussedSamples = native.n_focussed_samples,
                NumCoordDescents = native.n_coord_descents,
                SwapPeriod = native.swap_period,
                SwapAttempts = native.swap_attempts,
                SwapAreaRatioMin = native.swap_area_ratio_min,
                SwapAreaRatioMax = native.swap_area_ratio_max,
                LeftBiasGain = native.left_bias_gain
            };
        }

        /// <summary>
        /// 네이티브 <see cref="SwarmNative.CSwarmConfig"/>를 관리 <see cref="SwarmConfig"/>로 변환한다.
        /// </summary>
        /// <param name="nativeConfig">네이티브 설정 구조체</param>
        /// <returns>변환된 관리 설정 객체</returns>
        internal static SwarmConfig FromNative(SwarmNative.CSwarmConfig nativeConfig)
        {
            return new SwarmConfig
            {
                RngSeed = nativeConfig.rng_seed,
                PolySimplTolerance = nativeConfig.poly_simpl_tolerance >= 0 ? nativeConfig.poly_simpl_tolerance : null,
                SimplifyMaxDeviationMm = float.IsFinite(nativeConfig.refplace.simplify_max_deviation_mm) && nativeConfig.refplace.simplify_max_deviation_mm > 0 ? nativeConfig.refplace.simplify_max_deviation_mm : null,
                MinItemSeparation = nativeConfig.min_item_separation >= 0 ? nativeConfig.min_item_separation : null,
                NarrowConcavityCutoffDist = nativeConfig.narrow_concavity_cutoff_dist >= 0 ? nativeConfig.narrow_concavity_cutoff_dist : null,
                NarrowConcavityCutoffArea = nativeConfig.narrow_concavity_cutoff_area >= 0 ? nativeConfig.narrow_concavity_cutoff_area : null,
                lbf_x_multiplier = nativeConfig.lbf_x_multiplier > 0 ? nativeConfig.lbf_x_multiplier : 10.0f,
                lbf_y_multiplier = nativeConfig.lbf_y_multiplier > 0 ? nativeConfig.lbf_y_multiplier : 1.0f,
                Exploration = new ExplorationConfig
                {
                    Repair = SeparationRepairConfig.FromNative(nativeConfig.repair),
                    ShrinkStep = nativeConfig.exploration.shrink_step,
                    TimeLimitSecs = nativeConfig.exploration.time_limit_secs,
                    MaxConseqFailedAttempts = nativeConfig.exploration.max_conseq_failed_attempts >= 0
                        ? nativeConfig.exploration.max_conseq_failed_attempts
                        : null,
                    SolutionPoolDistributionStdDev = nativeConfig.exploration.solution_pool_distribution_stddev,
                    LargeItemChAreaCutoffPercentile = nativeConfig.exploration.large_item_ch_area_cutoff_percentile,
                    Separator = SeparatorFromNative(nativeConfig.exploration.separator),
                    RnrPeriod = nativeConfig.exploration.rnr_period,
                    RnrBandFraction = nativeConfig.exploration.rnr_band_fraction,
                    RnrMaxItems = nativeConfig.exploration.rnr_max_items,
                    AdaptiveShrinkGain = nativeConfig.exploration.adaptive_shrink_gain,
                    AdaptiveShrinkMaxMult = nativeConfig.exploration.adaptive_shrink_max_mult,
                    OscFailThreshold = nativeConfig.exploration.osc_fail_threshold,
                    OscExpandSteps = nativeConfig.exploration.osc_expand_steps,
                    AlnsDecay = nativeConfig.exploration.alns_decay,
                    AlnsMinWeight = nativeConfig.exploration.alns_min_weight,
                    // 폭 완화 복구는 C ABI-tail 블록에 있다.
                    ReliefFailThreshold = nativeConfig.width_relief.fail_threshold,
                    ReliefStep = nativeConfig.width_relief.step,
                    ReliefMaxRatio = nativeConfig.width_relief.max_ratio,
                    ReliefMaxRounds = nativeConfig.width_relief.max_rounds,
                    ReliefStrikeLimit = nativeConfig.width_relief.strike_limit,
                    ReliefRoundBudgetMs = nativeConfig.width_relief.round_budget_ms,
                    ShrinkFailDecay = nativeConfig.shrink_feedback.fail_decay,
                    ShrinkScaleMin = nativeConfig.shrink_feedback.scale_min
                },
                Compression = new CompressionConfig
                {
                    ShrinkRangeMin = nativeConfig.compression.shrink_range_min,
                    ShrinkRangeMax = nativeConfig.compression.shrink_range_max,
                    TimeLimitSecs = nativeConfig.compression.time_limit_secs,
                    ShrinkDecay = (ShrinkDecayStrategy)nativeConfig.compression.shrink_decay_strategy,
                    ShrinkDecayRatio = nativeConfig.compression.shrink_decay_ratio,
                    Separator = SeparatorFromNative(nativeConfig.compression.separator)
                },
                Polish = new PolishConfig
                {
                    Enabled = nativeConfig.polish.enabled != 0,
                    TimeLimitSecs = nativeConfig.polish.time_limit_secs,
                    NumRounds = nativeConfig.polish.n_rounds,

                    SlideEnabled = nativeConfig.polish.slide_enabled != 0,
                    SlideNumPasses = nativeConfig.polish.slide_n_passes,
                    SlideDown = nativeConfig.polish.slide_down != 0,
                    SlideSafetyRatio = nativeConfig.polish.slide_safety_ratio,

                    HoleFillEnabled = nativeConfig.polish.hole_fill_enabled != 0,
                    HoleFillCellSizeRatio = nativeConfig.polish.hole_fill_cell_size_ratio,
                    HoleFillMaxCandidates = nativeConfig.polish.hole_fill_max_candidates,
                    HoleFillCandidateBand = nativeConfig.polish.hole_fill_candidate_band,
                    HoleFillNumSamples = nativeConfig.polish.hole_fill_n_samples,

                    LpEnabled = nativeConfig.polish.lp_enabled != 0,
                    LpNumIterations = nativeConfig.polish.lp_n_iterations,
                    LpTrustRegionRatio = nativeConfig.polish.lp_trust_region_ratio,
                    LpLeftPullWeight = nativeConfig.polish.lp_left_pull_weight
                },
                LbfRaster = new RasterNfpConfig
                {
                    Enabled = nativeConfig.lbf_raster.enabled != 0,
                    CellRatio = nativeConfig.lbf_raster.cell_ratio,
                    MaxCells = nativeConfig.lbf_raster.max_cells,
                    NumCandidates = nativeConfig.lbf_raster.n_candidates,
                    NumRandomSamples = nativeConfig.lbf_raster.n_random_samples
                },
                Allow180Rotation = nativeConfig.allow_180_rotation != 0,
                TiltRangeDeg = nativeConfig.tilt_range_deg > 0f ? nativeConfig.tilt_range_deg : 0f,
                StructureFirst = new StructureFirstConfig
                {
                    Enabled = nativeConfig.structure_first.enabled != 0,
                    MotifEnabled = nativeConfig.structure_first.motif_enabled != 0,
                    MaxActiveMotifs = nativeConfig.structure_first.max_active_motifs,
                    ConstructionStarts = nativeConfig.structure_first.construction_starts,
                    P1TimeLimitSecs = nativeConfig.structure_first.p1_time_limit_secs,
                    IncludeLegacyP1Baseline = nativeConfig.structure_first.p1_include_legacy_baseline != 0,
                    ProfileRowHeightRatio = nativeConfig.structure_first.profile_row_height_ratio,
                    ProfileMaxRows = nativeConfig.structure_first.profile_max_rows,
                    ProfileTopK = nativeConfig.structure_first.profile_top_k,
                    ItemBeamWidth = nativeConfig.structure_first.item_beam_width,
                    CandidatePoolCap = nativeConfig.structure_first.candidate_pool_cap,
                    SlideRefineSteps = nativeConfig.structure_first.slide_refine_steps,
                    PairMaxOffsetCandidates = nativeConfig.structure_first.pair_max_offset_candidates,
                    PairMaxOrientationWork = nativeConfig.structure_first.pair_max_orientation_work,
                    DiagnosticsMaxRasterCells = nativeConfig.structure_first.diagnostics_max_raster_cells,
                    DiagnosticsMaxPairComparisons = nativeConfig.structure_first.diagnostics_max_pair_comparisons,
                    ProgressCallbackIntervalMs = nativeConfig.structure_first.progress_callback_interval_ms,
                    P2TimeLimitSecs = nativeConfig.structure_first.p2_time_limit_secs,
                    P2ShrinkRangeMin = nativeConfig.structure_first.p2_shrink_range_min,
                    P2ShrinkRangeMax = nativeConfig.structure_first.p2_shrink_range_max,
                    P2FailureDecay = nativeConfig.structure_first.p2_failure_decay,
                    P3SlackRatio = nativeConfig.structure_first.p3_slack_ratio,
                    P3TimeLimitSecs = nativeConfig.structure_first.p3_time_limit_secs >= 0f
                        ? nativeConfig.structure_first.p3_time_limit_secs
                        : null,
                    P4Cycles = nativeConfig.structure_first.p4_cycles,
                    P4CycleTimeLimitSecs = nativeConfig.structure_first.p4_cycle_time_limit_secs,
                    P4RuinFraction = nativeConfig.structure_first.p4_ruin_fraction,
                    P4MaxItems = nativeConfig.structure_first.p4_max_items,
                    P4StagnationCycles = nativeConfig.structure_first.p4_stagnation_cycles,
                    P5SnapRotations = nativeConfig.structure_first.p5_snap_rotations != 0
                },
                CallbackOptions = new CallbackOptionsConfig
                {
                    CollectLoss = nativeConfig.callback_options.collect_loss != 0,
                    CollectCollisionStatus = nativeConfig.callback_options.collect_collision_status != 0
                }
            };
        }
    }
}
