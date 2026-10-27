using Uttu.NestingV2;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

/// <summary>
/// 폭 완화 복구 설정의 관리 → 네이티브 매핑 테스트.
/// 이 설정은 의미상 탐색 단계에 속하지만 C ABI에서는 tail 블록이라,
/// <see cref="SwarmConfig.ToNative"/>가 두 위치를 정확히 잇는지 확인한다.
/// </summary>
public sealed class WidthReliefConfigTests
{
    [Fact]
    public void DefaultsKeepReliefOffSoLegacyOscillationStillRuns()
    {
        var config = new SwarmConfig();

        Assert.Equal(0, config.Exploration.ReliefFailThreshold);
        Assert.Equal(4, config.Exploration.OscFailThreshold);

        var native = config.ToNative().width_relief;
        Assert.Equal(0, native.fail_threshold);
        Assert.Equal(0.002f, native.step);
        Assert.Equal(0.03f, native.max_ratio);
        Assert.Equal(12, native.max_rounds);
        Assert.Equal(1, native.strike_limit);
        Assert.Equal(0, native.round_budget_ms);
    }

    [Fact]
    public void ExplorationReliefFieldsMapIntoTheTailBlock()
    {
        var config = new SwarmConfig();
        config.Exploration.ReliefFailThreshold = 2;
        config.Exploration.ReliefStep = 0.005f;
        config.Exploration.ReliefMaxRatio = 0.06f;
        config.Exploration.ReliefMaxRounds = 20;
        config.Exploration.ReliefStrikeLimit = 2;
        config.Exploration.ReliefRoundBudgetMs = 250;

        var native = config.ToNative().width_relief;
        Assert.Equal(2, native.fail_threshold);
        Assert.Equal(0.005f, native.step);
        Assert.Equal(0.06f, native.max_ratio);
        Assert.Equal(20, native.max_rounds);
        Assert.Equal(2, native.strike_limit);
        Assert.Equal(250, native.round_budget_ms);
    }

    [Fact]
    public void ReliefSurvivesANativeRoundTrip()
    {
        var config = new SwarmConfig();
        config.Exploration.ReliefFailThreshold = 1;
        config.Exploration.ReliefStep = 0.004f;
        config.Exploration.ReliefMaxRatio = 0.02f;
        config.Exploration.ReliefMaxRounds = 8;
        config.Exploration.ReliefStrikeLimit = 3;
        config.Exploration.ReliefRoundBudgetMs = 400;

        var restored = ConfigConverter.FromNative(config.ToNative()).Exploration;
        Assert.Equal(1, restored.ReliefFailThreshold);
        Assert.Equal(0.004f, restored.ReliefStep);
        Assert.Equal(0.02f, restored.ReliefMaxRatio);
        Assert.Equal(8, restored.ReliefMaxRounds);
        Assert.Equal(3, restored.ReliefStrikeLimit);
        Assert.Equal(400, restored.ReliefRoundBudgetMs);
        // 진동 설정은 그대로 남는다 (네이티브가 복구를 우선할 뿐 값은 보존한다).
        Assert.Equal(config.Exploration.OscFailThreshold, restored.OscFailThreshold);
    }
}
