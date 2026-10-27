using Uttu.NestingV2;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public class OptimizationPhaseSequenceTests
{
    [Fact]
    public void DefaultIsExploreCompressPolish()
    {
        Assert.Equal(
            new[] { OptimizationPhase.Explore, OptimizationPhase.Compress, OptimizationPhase.Polish },
            new SwarmConfig().PhaseSequence);
    }

    [Theory]
    [InlineData("Explore → Explore → Compress → Polish", 4)]
    [InlineData("explore -> polish -> EXPLORE", 3)]
    [InlineData("Compress, Compress", 2)]
    public void ParseKeepsOrderAndRepeats(string text, int count)
    {
        var parsed = OptimizationPhaseSequence.Parse(text);
        Assert.Equal(count, parsed.Length);
        Assert.Equal(parsed, OptimizationPhaseSequence.Validate(parsed));
    }

    [Theory]
    [InlineData("")]
    [InlineData("Explore → Repack")]
    [InlineData("Explore → 3")]
    public void ParseRejectsEmptyAndUnknownPhases(string text) =>
        Assert.Throws<ArgumentException>(() => OptimizationPhaseSequence.Parse(text));

    [Fact]
    public void ValidateRejectsEmptySequence()
    {
        Assert.Throws<ArgumentException>(() => OptimizationPhaseSequence.Validate([]));
        Assert.Throws<ArgumentException>(() => OptimizationPhaseSequence.Validate(null));
    }

    [Fact]
    public void RunCallsEveryOccurrenceInOrder()
    {
        var calls = new List<string>();
        int result = OptimizationPhaseSequence.Run(
            [OptimizationPhase.Explore, OptimizationPhase.Explore, OptimizationPhase.Compress, OptimizationPhase.Polish],
            () => false,
            phase => { calls.Add(phase.ToString()); return 0; },
            (phase, ordinal, total) => calls.Add($"start {ordinal}/{total}"),
            (phase, ordinal, total, code) => calls.Add($"done {ordinal}/{total}:{code}"));
        Assert.Equal(0, result);
        Assert.Equal(
            new[]
            {
                "start 1/4", "Explore", "done 1/4:0",
                "start 2/4", "Explore", "done 2/4:0",
                "start 3/4", "Compress", "done 3/4:0",
                "start 4/4", "Polish", "done 4/4:0",
            },
            calls);
    }

    [Fact]
    public void RunStopsAtTheFirstFailureAndOnStop()
    {
        var calls = new List<OptimizationPhase>();
        int failed = OptimizationPhaseSequence.Run(
            [OptimizationPhase.Explore, OptimizationPhase.Compress, OptimizationPhase.Polish],
            () => false,
            phase => { calls.Add(phase); return phase == OptimizationPhase.Compress ? -1 : 0; });
        Assert.Equal(-1, failed);
        Assert.Equal(new[] { OptimizationPhase.Explore, OptimizationPhase.Compress }, calls);

        calls.Clear();
        bool stop = false;
        int stopped = OptimizationPhaseSequence.Run(
            [OptimizationPhase.Explore, OptimizationPhase.Compress],
            () => stop,
            phase => { calls.Add(phase); stop = true; return 0; });
        Assert.Equal(0, stopped);
        Assert.Equal(new[] { OptimizationPhase.Explore }, calls);
    }
}
