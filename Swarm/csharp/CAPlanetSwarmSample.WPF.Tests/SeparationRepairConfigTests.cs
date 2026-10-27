using System.Runtime.InteropServices;
using System.Text.Json;
using Uttu.NestingV2;
using Xunit;
namespace CAPlanetSwarmSample.WPF.Tests;
public sealed class SeparationRepairConfigTests
{
    [Fact]
    public void RepairAppendsToTheAbiWithoutMovingReclaim()
    {
        Assert.Equal(744, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("reclaim").ToInt32());
        Assert.Equal(952, Marshal.SizeOf<SwarmNative.CSwarmConfig>());
        Assert.Equal(776, Marshal.OffsetOf<SwarmNative.CSwarmConfig>("repair").ToInt32());
        Assert.Equal(36, Marshal.SizeOf<SwarmNative.CSeparationRepairConfig>());
    }
    [Fact]
    public void DefaultsAreOptInAndBudgetsReachTheTail()
    {
        var c = new SwarmConfig().ToNative().repair;
        Assert.Equal(0, c.direction_enabled);
        Assert.Equal(0, c.cluster_lp_enabled);
        Assert.Equal(0, c.chain_enabled);
        Assert.Equal(24u, c.stagnation_iterations);
        Assert.Equal(2u, c.max_passes);
        Assert.Equal(4u, c.max_units);
        Assert.Equal(64u, c.max_evaluations);
        Assert.Equal(20u, c.time_budget_ms);
        Assert.Equal(2f, c.max_move_mm);
    }
    [Fact]
    public void ExplicitSettingsSurviveNativeAndJsonRoundtrips()
    {
        var config = new SwarmConfig();
        config.Exploration.Repair = new SeparationRepairConfig {
            DirectionEnabled = true, ClusterLpEnabled = true, ChainEnabled = true,
            StagnationIterations = 31, MaxPasses = 3, MaxUnits = 6,
            MaxEvaluations = 99, TimeBudgetMs = 40, MaxMoveMm = 3.5f,
        };
        var serialized = JsonSerializer.Serialize(config);
        var restoredJson = JsonSerializer.Deserialize<SwarmConfig>(serialized)!;
        var r = ConfigConverter.FromNative(restoredJson.ToNative()).Exploration.Repair;
        Assert.True(r.DirectionEnabled);
        Assert.True(r.ClusterLpEnabled);
        Assert.True(r.ChainEnabled);
        Assert.Equal(31, r.StagnationIterations);
        Assert.Equal(3, r.MaxPasses);
        Assert.Equal(6, r.MaxUnits);
        Assert.Equal(99, r.MaxEvaluations);
        Assert.Equal(40, r.TimeBudgetMs);
        Assert.Equal(3.5f, r.MaxMoveMm);
    }
    [Fact]
    public void HostileBudgetsAreClampedBeforeUnsignedAbiConversion()
    {
        var config = new SwarmConfig();
        config.Exploration.Repair = new SeparationRepairConfig {
            DirectionEnabled = true, StagnationIterations = -5, MaxPasses = 999,
            MaxUnits = 0, MaxEvaluations = -1, TimeBudgetMs = 1000, MaxMoveMm = float.NaN,
        };
        var r = ConfigConverter.FromNative(config.ToNative()).Exploration.Repair;
        Assert.True(r.DirectionEnabled);
        Assert.Equal(1, r.StagnationIterations);
        Assert.Equal(8, r.MaxPasses);
        Assert.Equal(2, r.MaxUnits);
        Assert.Equal(1, r.MaxEvaluations);
        Assert.Equal(200, r.TimeBudgetMs);
        Assert.Equal(2f, r.MaxMoveMm);
        config.Exploration.Repair.MaxMoveMm = 999;
        Assert.Equal(100f, config.ToNative().repair.max_move_mm);
    }
}
