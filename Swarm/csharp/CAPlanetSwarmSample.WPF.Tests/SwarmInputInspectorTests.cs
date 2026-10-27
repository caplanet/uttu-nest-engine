using CAPlanetSwarmSample.WPF.Data;
using CAPlanetSwarmSample.WPF.Services;
using System.Text.Json;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public sealed class SwarmInputInspectorTests
{
    [Fact]
    public void Parse_LegacyInstance_ReportsOneWholeItemLane()
    {
        const string json = """
            {
              "name":"legacy",
              "items":[
                {"id":0},
                {"id":1,"placed_hint":{"item_id":1,"is_freeze":1}}
              ]
            }
            """;

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal(1, result.SchemaVersion);
        Assert.Equal("legacy", result.NestingMode);
        Assert.Equal(1, result.LaneCount);
        Assert.Equal(2, result.ItemCount);
        Assert.Equal(1, result.PlacedHintCount);
        Assert.Equal(1, result.FrozenHintCount);
    }

    [Fact]
    public void Parse_LegacyCamelCaseHint_IsCountedLikeExecutionParser()
    {
        const string json = """
            {
              "name":"legacy-camel",
              "items":[
                {"id":0,"placedHint":{"itemId":0,"isFreeze":true}}
              ]
            }
            """;

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal(1, result.PlacedHintCount);
        Assert.Equal(1, result.FrozenHintCount);
    }

    [Fact]
    public void Parse_LegacyOutputEnvelope_UsesNestedInstance()
    {
        const string json = """
            {
              "instance":{"items":[{"id":7}]},
              "solution":{"layout":{"placed_items":[]}}
            }
            """;

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal(1, result.ItemCount);
    }

    [Fact]
    public void Parse_V2AllItems_AppliesSchedulerDefaultsAndPlaceGroup()
    {
        string json = V2Envelope(
            "all_items",
            "[]",
            """
            [
              {"id":0,"placed_hint":{"item_id":0,"is_freeze":0}},
              {"id":1,"placed_hint":{"item_id":1,"is_freeze":0}}
            ]
            """,
            "[{\"id\":42,\"member_item_ids\":[0,1]}]",
            nestingExtras: string.Empty);

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal(2, result.SchemaVersion);
        Assert.Equal("all_items", result.NestingMode);
        Assert.Equal(1, result.LaneCount);
        // group_workers 를 생략한 입력이므로 스케줄러 기본값이 적용된다.
        // 그 기본값은 실행 머신의 논리 코어 수라 상수로 단정할 수 없다.
        Assert.Equal(SwarmInputInspector.DefaultGroupWorkers, result.GroupWorkers);
        Assert.Equal(SwarmInputInspector.DefaultQuantumMs, result.QuantumMs);
        Assert.Single(result.PlaceGroups);
        Assert.Equal([0UL, 1UL], result.PlaceGroups[0].MemberItemIds);
    }

    [Fact]
    public void Parse_PartialPlaceGroupFreeze_ReportsExplicitAndEffectiveLocks()
    {
        string json = V2Envelope(
            "all_items",
            "[]",
            """
            [
              {"id":0,"placed_hint":{"item_id":0,"is_freeze":1}},
              {"id":1,"placed_hint":{"item_id":1,"is_freeze":0}},
              {"id":2,"placed_hint":{"item_id":2,"is_freeze":1}}
            ]
            """,
            "[{\"id\":42,\"member_item_ids\":[0,1]}]",
            nestingExtras: string.Empty);

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal(2, result.FrozenHintCount);
        Assert.Equal(3, result.EffectiveFrozenItemCount);
        Assert.Contains("잠금 명시 2 / 적용 3", result.DisplayText);
    }

    [Fact]
    public void Parse_V2SizeRegions_ReportsRegionsAppliedGapsAndScheduler()
    {
        string json = V2Envelope(
            "size_regions",
            """
            [
              {"id":"S+M","kind":"size_group","item_ids":[0,1],"gap_after_mm":10.0},
              {"id":"L","kind":"size_group","item_ids":[2],"gap_after_mm":5.0},
              {"id":"XL","kind":"remainder","item_ids":[3],"gap_after_mm":99.0}
            ]
            """,
            "[{\"id\":0},{\"id\":1},{\"id\":2},{\"id\":3}]",
            "[]",
            "\"group_workers\":2,\"quantum_ms\":125,\"locked_prefix_gap_mm\":7.5");

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal("size_regions", result.NestingMode);
        Assert.Equal(3, result.LaneCount);
        Assert.Equal(2, result.GroupWorkers);
        Assert.Equal(125, result.QuantumMs);
        Assert.Equal(7.5, result.LockedPrefixGapMm);
        Assert.Equal(15.0, result.TotalRegionGapMm);
        Assert.Contains("gap 99 mm 미적용", result.DisplayText);
    }

    [Fact]
    public void Parse_FixedOnlyRegion_IsExcludedFromMovableLanesAndAppliedGaps()
    {
        string json = V2Envelope(
            "size_regions",
            """
            [
              {"id":"locked","kind":"size_bundle","item_ids":[0],"gap_after_mm":10.0},
              {"id":"M","kind":"size_bundle","item_ids":[1],"gap_after_mm":5.0},
              {"id":"L","kind":"size_bundle","item_ids":[2],"gap_after_mm":99.0}
            ]
            """,
            """
            [
              {"id":0,"placed_hint":{"item_id":0,"is_freeze":1}},
              {"id":1},
              {"id":2}
            ]
            """,
            "[]",
            string.Empty);

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.True(result.IsValid, result.ErrorMessage);
        Assert.Equal(3, result.Regions.Count);
        Assert.Equal(2, result.LaneCount);
        Assert.Equal(5.0, result.TotalRegionGapMm);
        Assert.Equal(0, result.Regions[0].MovableItemCount);
        Assert.Equal(0, result.Regions[0].AppliedGapAfterMm);
        Assert.Equal(5.0, result.Regions[1].AppliedGapAfterMm);
        Assert.Contains("fixed prefix only", result.DisplayText);
    }

    [Theory]
    [InlineData("{")]
    [InlineData("[]")]
    [InlineData("{\"schema_version\":9}")]
    public void Parse_InvalidJsonOrSchema_ReturnsFailureInsteadOfThrowing(string json)
    {
        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.False(result.IsValid);
        Assert.False(string.IsNullOrWhiteSpace(result.ErrorMessage));
    }

    [Fact]
    public void Parse_PlaceGroupCrossingSizeRegions_ReturnsFailure()
    {
        string json = V2Envelope(
            "size_regions",
            """
            [
              {"id":"S","kind":"size_bundle","item_ids":[0],"gap_after_mm":0},
              {"id":"M","kind":"size_bundle","item_ids":[1],"gap_after_mm":0}
            ]
            """,
            """
            [
              {"id":0,"placed_hint":{"item_id":0,"is_freeze":0}},
              {"id":1,"placed_hint":{"item_id":1,"is_freeze":0}}
            ]
            """,
            "[{\"id\":12,\"member_item_ids\":[0,1]}]",
            string.Empty);

        SwarmInputSummary result = SwarmInputInspector.Parse(json);

        Assert.False(result.IsValid);
        Assert.Contains("여러 size region", result.ErrorMessage);
    }

    [Fact]
    public void PrepareForRun_UseHintsOff_RemovesOnlyStandaloneMovableHints()
    {
        string json = V2Envelope(
            "all_items",
            "[]",
            """
            [
              {"id":0,"placed_hint":{"item_id":0,"is_freeze":0,"tr_x":1}},
              {"id":1,"placed_hint":{"item_id":1,"is_freeze":1,"tr_x":2}},
              {"id":2,"placed_hint":{"item_id":2,"is_freeze":0,"tr_x":3}},
              {"id":3,"placed_hint":{"item_id":3,"is_freeze":0,"tr_x":4}}
            ]
            """,
            "[{\"id\":77,\"member_item_ids\":[2,3]}]",
            string.Empty);

        SwarmInputPreparationResult result = SwarmInputInspector.PrepareForRun(json, useHints: false);

        Assert.True(result.IsSuccess, result.ErrorMessage);
        Assert.Equal(1, result.RemovedMovableHintCount);
        Assert.Equal(3, result.RetainedRequiredHintCount);
        using JsonDocument document = JsonDocument.Parse(result.Json);
        JsonElement.ArrayEnumerator items = document.RootElement
            .GetProperty("instance")
            .GetProperty("items")
            .EnumerateArray();
        JsonElement[] values = items.ToArray();
        Assert.False(values[0].TryGetProperty("placed_hint", out _));
        Assert.True(values[1].TryGetProperty("placed_hint", out _));
        Assert.True(values[2].TryGetProperty("placed_hint", out _));
        Assert.True(values[3].TryGetProperty("placed_hint", out _));
    }

    [Fact]
    public void PrepareForRun_UseHintsOn_PreservesOriginalJson()
    {
        const string json = "{\"schema_version\":2}";

        SwarmInputPreparationResult result = SwarmInputInspector.PrepareForRun(json, useHints: true);

        Assert.True(result.IsSuccess);
        Assert.Equal(json, result.Json);
    }

    [Fact]
    public void PrepareForRun_V2GroupWorkersOverride_UpdatesOnlySchedulerSetting()
    {
        string json = V2Envelope(
            "size_regions",
            "[{\"id\":\"S\",\"kind\":\"size_group\",\"item_ids\":[0],\"gap_after_mm\":0}]",
            "[{\"id\":0}]",
            "[]",
            "\"group_workers\":1,\"quantum_ms\":125");

        SwarmInputPreparationResult result = SwarmInputInspector.PrepareForRun(
            json,
            useHints: true,
            groupWorkersOverride: 4);

        Assert.True(result.IsSuccess, result.ErrorMessage);
        Assert.Equal(4, result.AppliedGroupWorkers);
        using JsonDocument document = JsonDocument.Parse(result.Json);
        JsonElement nesting = document.RootElement.GetProperty("nesting");
        Assert.Equal(4, nesting.GetProperty("group_workers").GetInt32());
        Assert.Equal(125, nesting.GetProperty("quantum_ms").GetInt32());
    }

    [Fact]
    public void PrepareForRun_LegacyGroupWorkersOverride_IsIgnored()
    {
        const string json = "{\"items\":[{\"id\":0}]}";

        SwarmInputPreparationResult result = SwarmInputInspector.PrepareForRun(
            json,
            useHints: true,
            groupWorkersOverride: 4);

        Assert.True(result.IsSuccess, result.ErrorMessage);
        Assert.Equal(json, result.Json);
        Assert.Null(result.AppliedGroupWorkers);
    }

    [Fact]
    public void PrepareForRun_InvalidGroupWorkersOverride_IsRejected()
    {
        string json = V2Envelope("all_items", "[]", "[{\"id\":0}]", "[]", string.Empty);

        SwarmInputPreparationResult result = SwarmInputInspector.PrepareForRun(
            json,
            useHints: true,
            groupWorkersOverride: 0);

        Assert.False(result.IsSuccess);
        Assert.Contains("1 이상", result.ErrorMessage);
    }

    private static string V2Envelope(
        string mode,
        string regions,
        string items,
        string placeGroups,
        string nestingExtras)
    {
        string optionalProperties = string.IsNullOrWhiteSpace(nestingExtras)
            ? string.Empty
            : $",{nestingExtras}";
        return $$"""
            {
              "schema_version":2,
              "nesting":{
                "mode":"{{mode}}"{{optionalProperties}}
              },
              "instance":{"items":{{items}}},
              "regions":{{regions}},
              "place_groups":{{placeGroups}}
            }
            """;
    }
}
