using CAPlanetSwarmSample.WPF.Data;
using CAPlanetSwarmSample.WPF.Services;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public sealed class StructureReportInspectorTests
{
    [Fact]
    public void Parse_Schema3SingleReport_ExtractsCoreMetrics()
    {
        string json = CoreReport("p3_explore", 123.5, 0.81, 2.4, 3, 2);

        StructureReportSummary result = StructureReportInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.False(result.IsGroupedReport);
        Assert.Equal("legacy", result.NestingMode);
        Assert.Equal(123.5, result.FinalStripWidth);
        Assert.Equal(0.81, result.FinalDensity);
        Assert.Equal("p3_explore", result.BestStage);
        Assert.Equal(1, result.LaneCount);
        Assert.Equal(3, result.P1StartsAttempted);
        Assert.Equal(2, result.P1StartsCompleted);
    }

    [Fact]
    public void Parse_GroupedSchema3Report_AggregatesLanesAndMandatoryGaps()
    {
        string lane0 = GroupedLane(0, "[0,1]", CoreReport("p2_compress", 50, 0.7, 1.2, 2, 1));
        string lane1 = GroupedLane(1, "[2]", CoreReport("p5_finish", 60, 0.8, 1.8, 1, 1), "user_place_groups_active");
        string json = $$"""
            {
              "schema_version":3,
              "nesting_mode":"size_regions",
              "group_workers":2,
              "quantum_ms":50,
              "fixed_prefix_item_count":1,
              "fixed_prefix_right_mm":20,
              "locked_prefix_gap_mm":7,
              "required_region_gap_mm":10,
              "lane_count":2,
              "lanes":[{{lane0}},{{lane1}}],
              "final_strip_width":130,
              "final_density":0.75
            }
            """;

        StructureReportSummary result = StructureReportInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.True(result.IsGroupedReport);
        Assert.Equal("size_regions", result.NestingMode);
        Assert.Equal(2, result.LaneCount);
        Assert.Equal(2, result.GroupWorkers);
        Assert.Contains("workers 2 · quantum", result.DisplayText);
        Assert.Equal(10, result.RequiredRegionGapMm);
        Assert.Equal(3, result.P1StartsAttempted);
        Assert.Equal(2, result.P1StartsCompleted);
        Assert.Equal(1, result.FallbackLaneCount);
        Assert.Contains("p2_compress", result.BestStage);
        Assert.Contains("p5_finish", result.BestStage);
    }

    [Theory]
    [InlineData(2, "workers 2 (요청 4)")]
    [InlineData(4, "workers 4 · quantum")]
    public void Parse_GroupedReport_DisplaysEffectiveWorkers(int effectiveWorkers, string expected)
    {
        string lane = GroupedLane(0, "[0]", CoreReport("p5_finish", 50, 0.7, 1.2, 1, 1));
        string json = $$"""
            {
              "schema_version":3,
              "nesting_mode":"size_regions",
              "group_workers":4,
              "effective_group_workers":{{effectiveWorkers}},
              "quantum_ms":50,
              "lane_count":1,
              "lanes":[{{lane}}],
              "final_strip_width":50,
              "final_density":0.7
            }
            """;

        StructureReportSummary result = StructureReportInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal(4, result.GroupWorkers);
        Assert.Equal(effectiveWorkers, result.EffectiveGroupWorkers);
        Assert.Contains(expected, result.DisplayText);
    }

    [Fact]
    public void Parse_FixedOnlySizeRegionsReport_AllowsZeroMovableLanes()
    {
        string json = """
            {
              "schema_version":3,
              "nesting_mode":"size_regions",
              "group_workers":1,
              "quantum_ms":50,
              "fixed_prefix_item_count":2,
              "fixed_prefix_right_mm":40,
              "locked_prefix_gap_mm":7,
              "required_region_gap_mm":0,
              "lane_count":0,
              "lanes":[],
              "final_strip_width":40,
              "final_density":0.5
            }
            """;

        StructureReportSummary result = StructureReportInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.True(result.IsGroupedReport);
        Assert.Equal("size_regions", result.NestingMode);
        Assert.Equal(0, result.LaneCount);
        Assert.Equal("fixed_prefix", result.BestStage);
        Assert.Equal(2, result.FixedPrefixItemCount);
        Assert.Equal(40, result.FinalStripWidth);
    }

    [Theory]
    [InlineData("")]
    [InlineData("{")]
    [InlineData("{\"schema_version\":2}")]
    [InlineData("{\"schema_version\":3,\"lanes\":[]}")]
    public void Parse_InvalidReport_ReturnsFailure(string json)
    {
        StructureReportSummary result = StructureReportInspector.Parse(json);

        Assert.False(result.IsValid);
        Assert.False(string.IsNullOrWhiteSpace(result.ErrorMessage));
    }

    [Fact]
    public void ProgressTracker_UsesForcedSequenceWithoutClaimingLaneMilestones()
    {
        var tracker = new StructureProgressTracker();

        Assert.Equal(StructureUiStage.P1Construct, tracker.Observe(0));
        Assert.Equal(StructureUiStage.P1Construct, tracker.Observe(0));
        Assert.Equal(2, tracker.P1CallbackCount);
        Assert.Equal(StructureUiStage.P2Compress, tracker.Observe(3));
        Assert.Equal(StructureUiStage.P3Explore, tracker.Observe(0));
        Assert.Equal(StructureUiStage.P4Refine, tracker.Observe(2));
        Assert.Equal(StructureUiStage.P5Finish, tracker.Observe(3));
        Assert.Equal(StructureUiStage.Completed, tracker.Observe(4));
        Assert.Equal("Structure 완료", tracker.DisplayText);
    }

    [Theory]
    [InlineData(true, "{\"schema_version\":3}", false)]
    [InlineData(false, null, false)]
    [InlineData(false, "", false)]
    [InlineData(false, "   ", false)]
    [InlineData(false, "{\"schema_version\":3}", true)]
    public void ExportCommandCondition_RequiresCompletedNonEmptyReport(
        bool isRunning,
        string? reportJson,
        bool expected)
    {
        Assert.Equal(expected, StructureReportExportService.CanExport(isRunning, reportJson));
    }

    [Fact]
    public void ExportFileName_SanitizesProjectNameAndContainsMilliseconds()
    {
        var completedAt = new DateTimeOffset(2026, 8, 20, 9, 8, 7, 123, TimeSpan.FromHours(9));

        string fileName = StructureReportExportService.BuildFileName("A:B/C", completedAt);

        Assert.Equal("A_B_C_structure_report_20260820_090807_123.json", fileName);
    }

    [Fact]
    public void ExportSave_WritesRawReportAndPreservesAnExistingFile()
    {
        string directory = Path.Combine(
            Path.GetTempPath(),
            "capswarm-wpf-report-tests",
            Guid.NewGuid().ToString("N"));
        string report = CoreReport("p5_finish", 100, 0.9, 3, 2, 2);
        var completedAt = new DateTimeOffset(2026, 8, 20, 9, 8, 7, 123, TimeSpan.FromHours(9));

        try
        {
            StructureReportExportResult first = StructureReportExportService.Save(
                report,
                directory,
                "marker",
                completedAt);
            StructureReportExportResult second = StructureReportExportService.Save(
                report,
                directory,
                "marker",
                completedAt);

            Assert.True(first.IsSuccess, first.ErrorMessage);
            Assert.True(second.IsSuccess, second.ErrorMessage);
            Assert.NotEqual(first.FilePath, second.FilePath);
            Assert.Equal(report, File.ReadAllText(first.FilePath!));
            Assert.EndsWith("_2.json", second.FilePath);
        }
        finally
        {
            if (Directory.Exists(directory))
            {
                Directory.Delete(directory, recursive: true);
            }
        }
    }

    private static string CoreReport(
        string bestStage,
        double width,
        double density,
        double elapsed,
        int attempted,
        int completed) => $$"""
            {
              "schema_version":3,
              "motif_singles":2,
              "motif_pairs":1,
              "motif_blocks":1,
              "motif_bands":0,
              "construction_starts_attempted":{{attempted}},
              "construction_starts_completed":{{completed}},
              "checkpoints":[{"stage":"p1_construct"},{"stage":"{{bestStage}}"}],
              "repair_cycles":[{"cycle":0}],
              "best_stage":"{{bestStage}}",
              "final_width":{{width}},
              "final_density":{{density}},
              "elapsed_secs":{{elapsed}}
            }
            """;

    private static string GroupedLane(
        int index,
        string itemIds,
        string report,
        string? fallbackReason = null) => $$"""
            {
              "schema_version":3,
              "lane_index":{{index}},
              "global_item_ids":{{itemIds}},
              "place_group_count":{{(fallbackReason is null ? 0 : 1)}},
              "frozen_item_count":0,
              "p0_p1_soft_structure_applied":{{(fallbackReason is null ? "true" : "false")}},
              "fallback_reason":{{(fallbackReason is null ? "null" : $"\"{fallbackReason}\"")}},
              "report":{{report}}
            }
            """;
}
