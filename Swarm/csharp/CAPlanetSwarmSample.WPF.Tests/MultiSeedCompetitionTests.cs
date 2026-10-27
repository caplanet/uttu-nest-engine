using System.Collections.Concurrent;
using System.Text.Json;
using System.Text.Json.Serialization;
using CAPlanetSwarmSample.WPF.Services;
using Uttu.NestingV2;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public class MultiSeedCompetitionTests
{
    [Theory]
    [InlineData("0", true)]
    [InlineData("02", true)]
    [InlineData("18446744073709551615", true)]
    [InlineData("18446744073709551616", false)]
    [InlineData("-1", false)]
    [InlineData("1.0", false)]
    [InlineData(" 2", false)]
    [InlineData("２", false)]
    [InlineData("", false)]
    public void SeedEditorOnlyAcceptsUnsignedAsciiIntegers(string text, bool expected) =>
        Assert.Equal(expected, MultiSeedConfiguration.TryParseSeed(text, out _));

    [Fact]
    public void ExplicitSeedListKeepsGapsOrderAndSettings()
    {
        var config = new SwarmConfig { RngSeed = 999 };
        config.Exploration.Separator.NumWorkers = 17;
        var copies = MultiSeedConfiguration.Create(config, new ulong[] { 42, 1, ulong.MaxValue });
        Assert.Equal(new ulong[] { 42, 1, ulong.MaxValue }, copies.Select(c => c.RngSeed));
        Assert.All(copies, c => Assert.Equal(17, c.Exploration.Separator.NumWorkers));
        Assert.Equal(999UL, config.RngSeed);
        AssertIndependentObjects(copies[0], copies[1]);
    }

    [Fact]
    public void ExplicitZeroIsRandomWithoutCollidingWithFixedSeeds()
    {
        var copies = MultiSeedConfiguration.Create(new(), new ulong[] { 7, 0, 1, ulong.MaxValue });
        Assert.Equal(7UL, copies[0].RngSeed);
        Assert.Equal(1UL, copies[2].RngSeed);
        Assert.Equal(ulong.MaxValue, copies[3].RngSeed);
        Assert.Equal(4, copies.Select(c => c.RngSeed).Distinct().Count());
        Assert.DoesNotContain(copies, c => c.RngSeed == 0);
        Assert.Throws<ArgumentException>(() => MultiSeedConfiguration.Create(new(), Array.Empty<ulong>()));
        Assert.Throws<ArgumentException>(() => MultiSeedConfiguration.Create(new(), new ulong[] { 2, 2 }));
        Assert.Throws<ArgumentException>(() => MultiSeedConfiguration.Create(new(), new ulong[] { 1, 2, 3, 4, 5, 6, 7 }));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(6)]
    public void CreatesRequestedCountWithoutChangingUserWorkerCounts(int count)
    {
        var config = new SwarmConfig { RngSeed = 91 };
        config.Exploration.Separator.NumWorkers = 17;
        config.Compression.Separator.NumWorkers = 13;
        var copies = MultiSeedConfiguration.Create(config, count);
        Assert.Equal(count, copies.Count);
        Assert.Equal(91UL, config.RngSeed);
        for (int i = 0; i < count; i++)
        {
            Assert.Equal(91UL + (ulong)i, copies[i].RngSeed);
            Assert.Equal(17, copies[i].Exploration.Separator.NumWorkers);
            Assert.Equal(13, copies[i].Compression.Separator.NumWorkers);
            Assert.NotSame(config, copies[i]);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(7)]
    [InlineData(-1)]
    public void RejectsUnsupportedCount(int count) =>
        Assert.Throws<ArgumentOutOfRangeException>(() => MultiSeedConfiguration.Create(new(), count));

    [Fact]
    public void RandomSeedResolvesToUniqueConcreteNonzeroValues()
    {
        var copies = MultiSeedConfiguration.Create(new() { RngSeed = 0 }, 6);
        Assert.Equal(6, copies.Select(x => x.RngSeed).Distinct().Count());
        Assert.DoesNotContain(copies, x => x.RngSeed == 0);
    }

    [Fact]
    public void SeedOverflowWrapsToOne()
    {
        var copies = MultiSeedConfiguration.Create(new() { RngSeed = ulong.MaxValue - 1 }, 4);
        Assert.Equal(new ulong[] { ulong.MaxValue - 1, ulong.MaxValue, 1, 2 }, copies.Select(x => x.RngSeed));
    }

    [Fact]
    public void DeepClonePreservesAllSettingsIncludingInfinityAndRepeatedPhases()
    {
        var config = new SwarmConfig
        {
            RngSeed = 7,
            PolySimplTolerance = float.PositiveInfinity,
            PhaseSequence = [OptimizationPhase.Polish, OptimizationPhase.Compress, OptimizationPhase.Polish]
        };
        config.Exploration.Separator.NumWorkers = 19;
        config.CallbackOptions.CollectLoss = false;
        var options = new JsonSerializerOptions { NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals };
        string original = JsonSerializer.Serialize(config, options);
        var copies = MultiSeedConfiguration.Create(config, 2);
        copies[1].RngSeed = 7;
        Assert.All(copies, c => Assert.Equal(original, JsonSerializer.Serialize(c, options)));
        AssertIndependentObjects(config, copies[0]);
        AssertIndependentObjects(copies[0], copies[1]);
        copies[0].PhaseSequence[0] = OptimizationPhase.Explore;
        copies[0].Exploration.Separator.NumWorkers = 1;
        Assert.Equal(OptimizationPhase.Polish, copies[1].PhaseSequence[0]);
        Assert.Equal(19, config.Exploration.Separator.NumWorkers);
        Assert.Equal(19, copies[1].Exploration.Separator.NumWorkers);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(3)]
    [InlineData(4)]
    public void CertifiedReportsMayOmitDiagnostics(int type)
    {
        var winner = Competition().TryOffer(1, Report(type: type, diagnostics: false), "Compress");
        Assert.NotNull(winner);
        Assert.Equal(22UL, winner.Seed);
        Assert.Equal(1, winner.Revision);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(5)]
    [InlineData(-1)]
    public void NoncertifiedReportsCannotWinEvenWithZeroDiagnostics(int type) =>
        Assert.Null(Competition().TryOffer(0, Report(type: type), "Explore"));

    [Fact]
    public void ImprovingReportWinsWithCompleteZeroCollisionStatus()
    {
        var winner = Competition().TryOffer(0, Report(type: (int)ReportType.ExplImproving), "Explore");
        Assert.NotNull(winner);
        Assert.Equal(ReportType.ExplImproving, winner.Report.ReportType);
    }

    [Theory]
    [InlineData(false, null)]
    [InlineData(true, "collision")]
    [InlineData(true, "missing-status")]
    [InlineData(true, "wrong-status-id")]
    [InlineData(true, "missing-json")]
    public void ImprovingReportCannotWinWithoutCompleteZeroCollisionStatus(bool diagnostics, string? fault) =>
        Assert.Null(Competition().TryOffer(0,
            Report(type: (int)ReportType.ExplImproving, diagnostics: diagnostics, fault: fault), "Explore"));

    [Theory]
    [InlineData("loss")]
    [InlineData("nan-loss")]
    [InlineData("infinite-loss")]
    [InlineData("collision")]
    [InlineData("missing-status")]
    [InlineData("wrong-status-id")]
    [InlineData("wrong-quantity")]
    [InlineData("missing-piece")]
    [InlineData("extra-piece")]
    [InlineData("nan-x")]
    [InlineData("infinite-y")]
    [InlineData("nan-rotation")]
    [InlineData("nan-width")]
    [InlineData("zero-width")]
    [InlineData("negative-width")]
    [InlineData("infinite-width")]
    [InlineData("nan-density")]
    [InlineData("zero-density")]
    [InlineData("negative-density")]
    [InlineData("infinite-density")]
    [InlineData("missing-parsed-width")]
    [InlineData("nan-parsed-width")]
    [InlineData("different-parsed-width")]
    [InlineData("missing-json")]
    public void InvalidCandidateCannotReplaceCertifiedWinner(string fault)
    {
        var competition = Competition();
        var incumbent = competition.TryOffer(0, Report(width: 100), "Init", true);
        Assert.NotNull(incumbent);
        Assert.Null(competition.TryOffer(1, Report(width: 80, fault: fault), "Compress", true));
        Assert.Same(incumbent, competition.Winner);
    }

    [Fact]
    public void PositiveDensityAboveOnePermittedForConfiguredOverlap()
    {
        Assert.NotNull(Competition().TryOffer(0, Report(fault: "density-over-one"), "Final"));
    }

    [Fact]
    public void SmallFloatWidthRoundingDifferenceIsPermitted()
    {
        Assert.NotNull(Competition().TryOffer(0, Report(fault: "rounded-width"), "Final"));
    }

    [Fact]
    public void OnlyStrictlyShorterWinnerAdvancesRevisionAndCompletedWinnerPersists()
    {
        var competition = Competition();
        var first = competition.TryOffer(0, Report(width: 100), "Final", true, "structure-one");
        Assert.NotNull(first);
        Assert.True(first.IsFinal);
        Assert.Equal("structure-one", first.StructureReportJson);
        Assert.Null(competition.TryOffer(1, Report(width: 100), "Compress"));
        Assert.Null(competition.TryOffer(1, Report(width: 101), "Final", true));
        Assert.True(competition.IsCurrent(first));
        var second = competition.TryOffer(1, Report(width: 99), "Polish");
        Assert.NotNull(second);
        Assert.Equal(2, second.Revision);
        Assert.False(competition.IsCurrent(first));
        Assert.True(competition.IsCurrent(second));
        Assert.False(competition.IsCurrent(second with { SeedIndex = 0 }));
    }

    [Fact]
    public void TiedOtherSeedCannotReplaceWinnerOrAttachItsFinalMetadata()
    {
        var competition = Competition();
        var original = competition.TryOffer(0, Report(), "Init");
        Assert.NotNull(original);
        Assert.Null(competition.TryOffer(1, Report(), "Final", true, "other-seed-structure"));
        Assert.Same(original, competition.Winner);
        Assert.False(original.IsFinal);
        Assert.Null(original.StructureReportJson);
    }

    [Fact]
    public void SameSeedIdenticalLayoutEnrichesFinalMetadataWithoutReplacingGeometry()
    {
        var competition = Competition();
        var original = competition.TryOffer(0, Report(), "Init");
        Assert.NotNull(original);
        var enriched = competition.TryOffer(0, Report(type: 4), "Structure Final", true, "structure-final");
        Assert.NotNull(enriched);
        Assert.Equal(2, enriched.Revision);
        Assert.Equal(original.Seed, enriched.Seed);
        Assert.Same(original.Report, enriched.Report);
        Assert.Equal("Structure Final", enriched.Phase);
        Assert.True(enriched.IsFinal);
        Assert.Equal("structure-final", enriched.StructureReportJson);
        Assert.False(competition.IsCurrent(original));
        Assert.True(competition.IsCurrent(enriched));
        Assert.Null(competition.TryOffer(0, Report(), "Another Label", true, "structure-final"));
        Assert.Same(enriched, competition.Winner);
    }

    [Fact]
    public void SameSeedMetadataEnrichmentNeverClearsFinalOrStructureAndOnlyChangesWhenNeeded()
    {
        var competition = Competition();
        var original = competition.TryOffer(0, Report(), "Live", false, "structure-live");
        Assert.NotNull(original);
        var final = competition.TryOffer(0, Report(), "Final", true);
        Assert.NotNull(final);
        Assert.True(final.IsFinal);
        Assert.Equal("structure-live", final.StructureReportJson);
        var updated = competition.TryOffer(0, Report(), "Late callback", false, "structure-final");
        Assert.NotNull(updated);
        Assert.Equal(3, updated.Revision);
        Assert.True(updated.IsFinal);
        Assert.Equal("Final", updated.Phase);
        Assert.Equal("structure-final", updated.StructureReportJson);
        Assert.Same(original.Report, updated.Report);
        Assert.Null(competition.TryOffer(0, Report(), "Late callback"));
        Assert.Same(updated, competition.Winner);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SameSeedEqualLengthDifferentNonfinalLayoutCannotAttachMetadata(bool duplicatePlacement)
    {
        var competition = Competition();
        var original = competition.TryOffer(0, Report(), "Init");
        Assert.NotNull(original);
        var other = Report();
        var placements = (List<SolutionEventArgs.ParsedPlacement>)other.ParsedPlacements;
        placements[0] = duplicatePlacement ? placements[1] : placements[0] with { TranslationX = 1 };
        Assert.Null(competition.TryOffer(0, other, "Boundary", false, "different-layout"));
        Assert.Same(original, competition.Winner);
    }

    [Fact]
    public void SameSeedCertifiedFinalTieReplacesNonfinalGeometryAndItsMetadataTogetherOnce()
    {
        var competition = Competition();
        var original = competition.TryOffer(0, Report(width: 7072.05f), "Structure live", false, "live-structure");
        Assert.NotNull(original);
        var finalReport = ChangedLayoutReport(7072.05f);
        var final = competition.TryOffer(0, finalReport, "Structure Final", true, "final-structure");
        Assert.NotNull(final);
        Assert.Equal(2, final.Revision);
        Assert.True(final.IsFinal);
        Assert.Equal(original.Seed, final.Seed);
        Assert.NotSame(original.Report, final.Report);
        Assert.Equal(finalReport.ParsedPlacements, final.Report.ParsedPlacements);
        Assert.Equal(finalReport.SolutionJson, final.Report.SolutionJson);
        Assert.Equal("final-structure", final.StructureReportJson);
        Assert.Equal("Structure Final", final.Phase);
        Assert.False(competition.IsCurrent(original));
        Assert.True(competition.IsCurrent(final));
        Assert.Null(competition.TryOffer(0, Report(width: 7072.05f), "Another final", true, "different-final"));
        Assert.Same(final, competition.Winner);
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(0, true)]
    public void ChangedFinalTieCannotReplaceAnotherSeedOrBypassCertification(int seedIndex, bool invalid)
    {
        var competition = Competition();
        var original = competition.TryOffer(0, Report(), "Live");
        Assert.NotNull(original);
        var finalReport = ChangedLayoutReport(100);
        if (invalid) finalReport.CollisionStatus[0].IsCollision = true;
        Assert.Null(competition.TryOffer(seedIndex, finalReport, "Final", true, "final-structure"));
        Assert.Same(original, competition.Winner);
    }

    private static SolutionEventArgs ChangedLayoutReport(float width)
    {
        string json = JsonSerializer.Serialize(new
        {
            solution = new
            {
                strip_width = width,
                density = 0.5f,
                layout = new
                {
                    placed_items = new[]
                    {
                        new { item_id = 7, transformation = new { rotation = 0f, translation = new[] { 1f, 0f } } },
                        new { item_id = 7, transformation = new { rotation = 180f, translation = new[] { 20f, 0f } } },
                        new { item_id = 9, transformation = new { rotation = 0f, translation = new[] { 40f, 0f } } }
                    }
                }
            },
            loss = 0,
            collision_status = new[]
            {
                new { item_id = 7, collision = false },
                new { item_id = 7, collision = false },
                new { item_id = 9, collision = false }
            }
        });
        return SolutionEventArgs.FromCombinedJson(ReportType.Final, width, 0.5f, json);
    }

    [Fact]
    public void SameLayoutWithReorderedPlacementsCanReceiveItsStructureMetadata()
    {
        var competition = Competition();
        var original = competition.TryOffer(0, Report(), "Init");
        Assert.NotNull(original);
        var reordered = Report();
        ((List<SolutionEventArgs.ParsedPlacement>)reordered.ParsedPlacements).Reverse();
        var enriched = competition.TryOffer(0, reordered, "Boundary", false, "structure");
        Assert.NotNull(enriched);
        Assert.Equal("Init", enriched.Phase);
        Assert.Same(original.Report, enriched.Report);
        Assert.Equal("structure", enriched.StructureReportJson);
    }

    [Fact]
    public void CopiesDemandSeedsAndRetainedMutableReportLists()
    {
        var seeds = new ulong[] { 11, 22 };
        var demand = new Dictionary<int, int> { [7] = 2, [9] = 1 };
        var competition = new MultiSeedCompetition(seeds, demand);
        seeds[0] = 50;
        demand[7] = 99;
        var report = Report();
        var winner = competition.TryOffer(0, report, "Init");
        Assert.NotNull(winner);
        Assert.Equal(11UL, winner.Seed);
        ((List<SolutionEventArgs.ParsedPlacement>)report.ParsedPlacements).Clear();
        report.CollisionStatus[0].IsCollision = true;
        ((List<SolutionEventArgs.PlaceCollisionStatus>)report.CollisionStatus).Clear();
        Assert.Equal(3, winner.Report.ParsedPlacements.Count);
        Assert.Equal(3, winner.Report.CollisionStatus.Count);
        Assert.All(winner.Report.CollisionStatus, s => Assert.False(s.IsCollision));
    }

    [Fact]
    public void ConcurrentOffersRetainGlobalShortestAndUniqueMonotonicRevisions()
    {
        var competition = Competition();
        var winners = new ConcurrentBag<SeedWinner>();
        Parallel.For(0, 100, i =>
        {
            var winner = competition.TryOffer(i % 2, Report(width: 200 - i), "Compress");
            if (winner != null) winners.Add(winner);
        });
        Assert.NotNull(competition.Winner);
        Assert.Equal(101, competition.Winner.Report.StripWidth);
        Assert.Equal(Enumerable.Range(1, winners.Count).Select(i => (long)i), winners.Select(w => w.Revision).Order());
        var ordered = winners.OrderBy(w => w.Revision).ToArray();
        for (int i = 1; i < ordered.Length; i++)
            Assert.True(ordered[i].Report.StripWidth < ordered[i - 1].Report.StripWidth);
    }

    private static MultiSeedCompetition Competition() => new([11, 22], new Dictionary<int, int> { [7] = 2, [9] = 1 });

    private static SolutionEventArgs Report(float width = 100, int type = 0, bool diagnostics = true, string? fault = null)
    {
        var placements = new List<SolutionEventArgs.ParsedPlacement>
        {
            new(7, 0, 0, 0), new(7, 180, 20, 0), new(9, 0, 40, 0)
        };
        var statuses = new List<SolutionEventArgs.PlaceCollisionStatus>
        {
            new() { ItemID = 7 }, new() { ItemID = 7 }, new() { ItemID = 9 }
        };
        float loss = 0, density = 0.5f;
        float? parsedWidth = width;
        string? json = JsonSerializer.Serialize(new
        {
            strip_width = width,
            density,
            layout = new
            {
                placed_items = placements.Select(p => new
                {
                    item_id = p.ItemId,
                    transformation = new { rotation = p.RotationDegrees, translation = new[] { p.TranslationX, p.TranslationY } }
                })
            }
        });
        switch (fault)
        {
            case "loss": loss = 0.01f; break;
            case "nan-loss": loss = float.NaN; break;
            case "infinite-loss": loss = float.PositiveInfinity; break;
            case "collision": statuses[0].IsCollision = true; break;
            case "missing-status": statuses.RemoveAt(0); break;
            case "wrong-status-id": statuses[0].ItemID = 9; break;
            case "wrong-quantity": placements[0] = placements[0] with { ItemId = 9 }; break;
            case "missing-piece": placements.RemoveAt(0); break;
            case "extra-piece": placements.Add(new(10, 0, 60, 0)); break;
            case "nan-x": placements[0] = placements[0] with { TranslationX = float.NaN }; break;
            case "infinite-y": placements[0] = placements[0] with { TranslationY = float.PositiveInfinity }; break;
            case "nan-rotation": placements[0] = placements[0] with { RotationDegrees = float.NaN }; break;
            case "nan-width": width = float.NaN; break;
            case "zero-width": width = 0; break;
            case "negative-width": width = -1; break;
            case "infinite-width": width = float.PositiveInfinity; break;
            case "nan-density": density = float.NaN; break;
            case "zero-density": density = 0; break;
            case "negative-density": density = -0.1f; break;
            case "density-over-one": density = 1.01f; break;
            case "infinite-density": density = float.PositiveInfinity; break;
            case "missing-parsed-width": parsedWidth = null; break;
            case "nan-parsed-width": parsedWidth = float.NaN; break;
            case "different-parsed-width": parsedWidth = width - 1; break;
            case "rounded-width": parsedWidth = width + 0.00001f; break;
            case "missing-json": json = null; break;
        }
        return new((ReportType)type, width, density, json, loss, statuses, diagnostics, diagnostics, placements, parsedWidth);
    }

    private static void AssertIndependentObjects(object original, object copy)
    {
        if (original is string || original.GetType().IsValueType) return;
        Assert.NotSame(original, copy);
        if (original is Array) return;
        foreach (var property in original.GetType().GetProperties().Where(p => p.CanRead && p.CanWrite))
        {
            var left = property.GetValue(original);
            var right = property.GetValue(copy);
            if (left != null && right != null) AssertIndependentObjects(left, right);
        }
    }
}
