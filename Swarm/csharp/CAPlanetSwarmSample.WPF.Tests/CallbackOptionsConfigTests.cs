using System.Runtime.InteropServices;
using Uttu.NestingV2;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public sealed class CallbackOptionsConfigTests
{
    [Fact]
    public void MmSimplificationUsesExistingAbiSlotAndOverridesAreaMode()
    {
        var config = new SwarmConfig { SimplifyMaxDeviationMm = 0.3f, PolySimplTolerance = 0.01f };
        var native = config.ToNative();
        Assert.Equal(0, native.refplace.enabled);
        Assert.Equal(0.3f, native.refplace.simplify_max_deviation_mm);
        Assert.Equal(-1f, native.poly_simpl_tolerance);
        Assert.Equal(12, Marshal.OffsetOf<SwarmNative.CRefPlaceConfig>("simplify_max_deviation_mm").ToInt32());
        config.SimplifyMaxDeviationMm = null;
        Assert.Equal(0.01f, config.ToNative().poly_simpl_tolerance);
        config.SimplifyMaxDeviationMm = float.NaN;
        Assert.Throws<System.ArgumentOutOfRangeException>(() => config.ToNative());
    }
    [Fact]
    public void ManagedAbiTailMatchesNativeLayout()
    {
        Assert.Equal(8, Marshal.SizeOf<SwarmNative.CCallbackOptionsConfig>());
        Assert.Equal(17 * 4, Marshal.SizeOf<SwarmNative.CInitialPlacementConfig>());
        Assert.Equal(6 * 4, Marshal.SizeOf<SwarmNative.CWidthReliefConfig>());
        Assert.Equal(2 * 4, Marshal.SizeOf<SwarmNative.CShrinkFeedbackConfig>());
        Assert.Equal(9 * 4, Marshal.SizeOf<SwarmNative.CEscapeConfig>());
        Assert.Equal(7 * 4, Marshal.SizeOf<SwarmNative.CNfpLbfConfig>());
        // ABI v2 필드 오프셋은 그대로 보존된다.
        Assert.Equal(404, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("callback_options").ToInt32());
        // ABI v3 tail: initial_placement 블록 — 오프셋 유지.
        Assert.Equal(412, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("initial_placement").ToInt32());
        // ABI v4 tail: width_relief 블록 — 오프셋 유지.
        Assert.Equal(480, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("width_relief").ToInt32());
        // ABI v5 tail: shrink_feedback 블록 — 오프셋 유지.
        Assert.Equal(504, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("shrink_feedback").ToInt32());
        // ABI v6 tail: escape / nfp_lbf 블록.
        Assert.Equal(512, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("escape").ToInt32());
        Assert.Equal(548, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("nfp_lbf").ToInt32());
        Assert.Equal(576, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("fabric_tolerance").ToInt32());
        Assert.Equal(4, Marshal.SizeOf<SwarmNative.CFabricToleranceConfig>());
        Assert.Equal(580, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("nfp_integration").ToInt32());
        Assert.Equal(20, Marshal.SizeOf<SwarmNative.CNfpIntegrationConfig>());
        // ABI v9 tail: NFP-LBF 탐침 예산. CNfpLbfConfig 를 늘리면 뒤 블록 오프셋이 전부
        // 밀리므로, 새 노브는 구조체 끝에 블록으로 덧붙였다.
        Assert.Equal(600, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("nfp_lbf_tuning").ToInt32());
        Assert.Equal(4 * 4, Marshal.SizeOf<SwarmNative.CNfpLbfTuningConfig>());
        // ABI v10 tail: 초기배치 전략 7종 + LP/QP 솔버 선택.
        Assert.Equal(616, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("nfp_strategy").ToInt32());
        Assert.Equal(12 * 4, Marshal.SizeOf<SwarmNative.CNfpStrategyConfig>());
        Assert.Equal(664, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("lp_solver").ToInt32());
        Assert.Equal(4 * 4, Marshal.SizeOf<SwarmNative.CLpSolverConfig>());
        // ABI v11 tail: 초기배치 전략 ⑧ 누적 배치 / ⑨ 꼬리 회수.
        // CNfpStrategyConfig(616, 48B) 를 늘리지 않고 새 블록으로 붙였다 —
        // 중간 블록을 늘렸다면 lp_solver 가 664 에서 밀려 v10 호스트가 조용히 깨진다.
        Assert.Equal(680, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("nfp_strategy2").ToInt32());
        Assert.Equal(4 * 4, Marshal.SizeOf<SwarmNative.CNfpStrategy2Config>());
        // ABI v12 tail: 레퍼런스 설계 초기배치 엔진(실험). 앞 블록 오프셋은 전부 그대로다.
        Assert.Equal(696, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("refplace").ToInt32());
        Assert.Equal(4 * 4, Marshal.SizeOf<SwarmNative.CRefPlaceConfig>());
        // ABI v13 tail: LBF 배치 순서. 앞 블록 오프셋은 전부 그대로다.
        Assert.Equal(712, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("lbf_order").ToInt32());
        Assert.Equal(4 * 4, Marshal.SizeOf<SwarmNative.CLbfOrderConfig>());
        // ABI v14 tail: 홀 우선 부착. 앞 블록 오프셋은 전부 그대로다.
        Assert.Equal(728, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("hole_first").ToInt32());
        Assert.Equal(4 * 4, Marshal.SizeOf<SwarmNative.CHoleFirstConfig>());
        // ABI v15 tail: 잔여 예산 회수. 앞 블록 오프셋은 전부 그대로다.
        Assert.Equal(744, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("reclaim").ToInt32());
        Assert.Equal(8 * 4, Marshal.SizeOf<SwarmNative.CReclaimConfig>());
        Assert.Equal(776, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("repair").ToInt32());
        Assert.Equal(36, Marshal.SizeOf<SwarmNative.CSeparationRepairConfig>());
        Assert.Equal(952, Marshal.SizeOf<SwarmNative.CSwarmConfig>());
    }

    /// <summary>실험 엔진 스위치가 네이티브 tail block 까지 흘러가고, 기본은 꺼짐이다.</summary>
    [Fact]
    public void TheExperimentalEngineIsOffByDefaultAndReachesTheNativeTail()
    {
        var config = new SwarmConfig();
        var off = config.ToNative().refplace;
        Assert.Equal(0, off.enabled);
        Assert.Equal(1, off.group_fallback);

        config.RefPlace.Enabled = true;
        config.RefPlace.GroupFallback = 0;
        var on = config.ToNative().refplace;
        Assert.Equal(1, on.enabled);
        Assert.Equal(0, on.group_fallback);
        config.RefPlace.ColumnDraft = true;
        Assert.Equal(1, config.ToNative().refplace._reserved0);
    }

    /// <summary>
    /// 일곱 전략과 솔버 선택이 관리 설정에서 네이티브 tail block 까지 실제로 흘러가는지 본다.
    /// <para>
    /// 이런 배선이 빠졌을 때 WPF 에서 옵션을 켜도 아무 일도 일어나지 않는다 —
    /// 실제로 한 번 겪은 결함이라 테스트로 고정한다.
    /// </para>
    /// </summary>
    [Fact]
    public void EveryStrategyFlagReachesTheNativeTailBlock()
    {
        var config = new SwarmConfig();

        // 기본값은 전부 꺼짐이어야 한다 — 기존 경로가 바뀌면 안 된다.
        var off = config.ToNative().nfp_strategy;
        Assert.Equal(0, off.instance_rotations);
        Assert.Equal(0, off.pairwise_clustering);
        Assert.Equal(0, off.contact_maximization);
        Assert.Equal(0, off.pocket_first);
        Assert.Equal(0, off.beam_search);
        Assert.Equal(0, off.best_fit_selection);
        Assert.Equal(0, off.sliding_compaction);

        // ⑧⑨는 v11 tail block 에 있다. 기본값은 둘 다 꺼짐.
        var off2 = config.ToNative().nfp_strategy2;
        Assert.Equal(0, off2.accretion);
        Assert.Equal(0, off2.tail_reflow);

        config.NfpStrategy.InstanceRotations = true;
        config.NfpStrategy.PairwiseClustering = true;
        config.NfpStrategy.ContactMaximization = true;
        config.NfpStrategy.PocketFirst = true;
        config.NfpStrategy.BeamSearch = true;
        config.NfpStrategy.BeamWidth = 6;
        config.NfpStrategy.BestFitSelection = true;
        config.NfpStrategy.BestFitLookahead = 20;
        config.NfpStrategy.SlidingCompaction = true;
        config.NfpStrategy.SlidingRounds = 3;
        config.NfpStrategy.ContinuousRotationStepDeg = 45;
        config.NfpStrategy.ClusterMinGain = 0.12f;
        config.NfpStrategy.Accretion = true;
        config.NfpStrategy.TailReflow = true;
        config.NfpStrategy.ReflowRounds = 7;

        var on = config.ToNative().nfp_strategy;
        Assert.Equal(1, on.instance_rotations);
        Assert.Equal(45, on.continuous_rotation_step_deg);
        Assert.Equal(1, on.pairwise_clustering);
        Assert.Equal(0.12f, on.cluster_min_gain);
        Assert.Equal(1, on.contact_maximization);
        Assert.Equal(1, on.pocket_first);
        Assert.Equal(1, on.beam_search);
        Assert.Equal(6, on.beam_width);
        Assert.Equal(1, on.best_fit_selection);
        Assert.Equal(20, on.best_fit_lookahead);
        Assert.Equal(1, on.sliding_compaction);
        Assert.Equal(3, on.sliding_rounds);

        // ⑧⑨는 별도 tail block 이다. 같은 설정 객체에서 함께 실려야 한다 —
        // 두 블록으로 나뉜 탓에 한쪽만 채우고 잊는 것이 정확히 이 코드베이스의 함정이다.
        var on2 = config.ToNative().nfp_strategy2;
        Assert.Equal(1, on2.accretion);
        Assert.Equal(1, on2.tail_reflow);
        Assert.Equal(7, on2.reflow_rounds);
    }

    /// <summary>솔버 선택과 QP 계수가 그대로 네이티브로 넘어간다.</summary>
    [Fact]
    public void LpSolverChoiceReachesTheNativeTailBlock()
    {
        var config = new SwarmConfig();

        // 기본값은 기존 동작 — 단체법, 순수 LP, 폴백 켜짐.
        var def = config.ToNative().lp_solver;
        Assert.Equal((int)LpSolverBackend.Microlp, def.backend);
        Assert.Equal(0f, def.quadratic_regularization);
        Assert.Equal(1, def.fallback);

        config.LpSolver.Backend = LpSolverBackend.Xpress;
        config.LpSolver.QuadraticRegularization = 0.25f;
        config.LpSolver.Fallback = false;
        config.LpSolver.TimeLimitMs = 1234;

        var native = config.ToNative().lp_solver;
        Assert.Equal(3, native.backend);
        Assert.Equal(0.25f, native.quadratic_regularization);
        Assert.Equal(0, native.fallback);
        Assert.Equal(1234, native.time_limit_ms);
    }

    /// <summary>
    /// 탐침 예산이 관리 설정에서 네이티브 tail block 까지 실제로 흘러가는지 본다.
    /// <para>
    /// 이 배선이 빠졌을 때 WPF 에서 NFP 를 켜도 초기 배치가 전혀 바뀌지 않았다.
    /// 값이 0 이면 네이티브가 기본값(4096)으로 보정한다.
    /// </para>
    /// </summary>
    [Fact]
    public void MaxCandidateProbesReachesTheNativeTailBlock()
    {
        var config = new SwarmConfig();
        // 기본값은 "네이티브에 맡김"(0)이 아니라 명시적 0 이다 — 네이티브가 4096 으로 채운다.
        Assert.Equal(0, config.ToNative().nfp_lbf_tuning.max_candidate_probes);

        config.NfpLbf.MaxCandidateProbes = 8192;
        Assert.Equal(8192, config.ToNative().nfp_lbf_tuning.max_candidate_probes);

        // 예약 필드는 반드시 0 이어야 한다.
        var native = config.ToNative().nfp_lbf_tuning;
        Assert.Equal(0, native._reserved0);
        Assert.Equal(0, native._reserved1);
        Assert.Equal(0, native._reserved2);
    }

    [Fact]
    public void InitialPlacementDefaultsAreDisabledWithConfirmedN06Values()
    {
        var config = new SwarmConfig();

        Assert.False(config.InitialPlacement.Enabled);
        var native = config.ToNative().initial_placement;
        Assert.Equal(0, native.enabled);
        Assert.Equal(300, native.rescue_after);
        Assert.Equal(1500, native.stall_cap);
        Assert.Equal(0.03f, native.cd_min);
        Assert.Equal(6, native.compact_passes);
        Assert.Equal(4, native.reseat_top);
        Assert.Equal(500f, native.lbf_x_weight);
        Assert.Equal(0.1f, native.lbf_y_weight);
    }

    [Fact]
    public void PublicDefaultsPreserveExistingCallbackDiagnostics()
    {
        var config = new SwarmConfig();

        Assert.True(config.CallbackOptions.CollectLoss);
        Assert.True(config.CallbackOptions.CollectCollisionStatus);
        Assert.Equal(1, config.ToNative().callback_options.collect_loss);
        Assert.Equal(1, config.ToNative().callback_options.collect_collision_status);
    }

    [Fact]
    public void OptionsMapIndependently()
    {
        var config = new SwarmConfig
        {
            CallbackOptions = new CallbackOptionsConfig
            {
                CollectLoss = true,
                CollectCollisionStatus = false
            }
        };

        var native = config.ToNative();

        Assert.Equal(1, native.callback_options.collect_loss);
        Assert.Equal(0, native.callback_options.collect_collision_status);
    }
}
