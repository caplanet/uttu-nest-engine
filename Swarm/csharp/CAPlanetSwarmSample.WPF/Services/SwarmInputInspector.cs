using CAPlanetSwarmSample.WPF.Data;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace CAPlanetSwarmSample.WPF.Services;

/// <summary>
/// legacy V1과 grouped V2 입력을 native 실행 전에 읽기 전용으로 요약하고,
/// UI의 Use Hints 정책을 V2 JSON에 적용한다.
/// </summary>
public static class SwarmInputInspector
{
    private const int GroupedSchemaVersion = 2;
    // These fallbacks apply only when a V2 input omits the scheduler fields.
    // Explicit input values remain part of the execution contract.
    /// <summary>
    /// V2 입력이 <c>group_workers</c>를 생략했을 때 적용하는 기본 워커 수.
    /// <para>
    /// 실행 머신의 논리 코어 수를 따르므로 값이 머신마다 다르다. 이 기본값을 검증하는
    /// 코드(테스트 포함)는 상수를 하드코딩하지 말고 반드시 이 속성을 참조해야 한다.
    /// </para>
    /// </summary>
    public static int DefaultGroupWorkers { get; } = Math.Max(1, Environment.ProcessorCount);
    /// <summary>
    /// V2 입력이 <c>quantum_ms</c>를 생략했을 때 적용하는 기본 스케줄러 퀀텀(ms).
    /// 이 기본값을 검증하는 코드는 상수를 하드코딩하지 말고 이 속성을 참조해야 한다.
    /// </summary>
    public static long DefaultQuantumMs => 1000;

    private static readonly JsonSerializerOptions OutputOptions = new()
    {
        WriteIndented = true,
        Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping
    };

    /// <summary>
    /// 입력 JSON을 분석한다. native validator를 대체하지 않지만, UI에 잘못된 grouping
    /// 정보를 표시하지 않도록 schema, region 배정 및 PlaceGroup의 핵심 계약을 확인한다.
    /// </summary>
    /// <param name="json">Swarm Init에 전달할 JSON.</param>
    /// <returns>화면 표시용 입력 요약. 잘못된 입력도 예외 대신 실패 결과로 반환한다.</returns>
    public static SwarmInputSummary Parse(string json)
    {
        if (string.IsNullOrWhiteSpace(json))
        {
            return SwarmInputSummary.Invalid("JSON이 비어 있습니다.");
        }

        try
        {
            using var document = JsonDocument.Parse(json);
            JsonElement root = document.RootElement;
            if (root.ValueKind != JsonValueKind.Object)
            {
                return SwarmInputSummary.Invalid("JSON root는 object여야 합니다.");
            }

            if (!root.TryGetProperty("schema_version", out JsonElement schemaElement))
            {
                return ParseLegacy(root);
            }

            if (!schemaElement.TryGetInt32(out int schemaVersion))
            {
                return SwarmInputSummary.Invalid("schema_version은 정수여야 합니다.");
            }

            if (schemaVersion != GroupedSchemaVersion)
            {
                return SwarmInputSummary.Invalid($"지원하지 않는 schema_version {schemaVersion}입니다.");
            }

            return ParseV2(root);
        }
        catch (JsonException exception)
        {
            return SwarmInputSummary.Invalid($"JSON 구문 오류: {exception.Message}");
        }
        catch (InvalidOperationException exception)
        {
            return SwarmInputSummary.Invalid(exception.Message);
        }
        catch (OverflowException exception)
        {
            return SwarmInputSummary.Invalid($"숫자 범위를 벗어났습니다: {exception.Message}");
        }
    }

    /// <summary>
    /// Use Hints와 grouped scheduler 설정을 적용한 실행용 JSON을 만든다.
    /// </summary>
    /// <remarks>
    /// V2에서 Use Hints가 꺼지면 독립적으로 움직이는 item의 movable hint만 제거한다.
    /// 잠금 hint와 PlaceGroup의 상대 pose를 정의하는 모든 member hint는 실행 계약이므로
    /// 반드시 보존한다. <c>quantum_ms</c>도 입력 계약을 그대로 유지하며 이 메서드는 변경하지 않는다.
    /// V1의 외부 <c>CPlacedHint[]</c> 필터링은 호출자가 담당한다.
    /// </remarks>
    /// <param name="json">원본 입력 JSON.</param>
    /// <param name="useHints">사용자 warm-start hint를 사용할지 여부.</param>
    /// <param name="groupWorkersOverride">
    /// V2 <c>nesting.group_workers</c> 실행값. <see langword="null"/>이면 입력값을 유지한다.
    /// legacy V1에는 적용하지 않는다.
    /// </param>
    /// <returns>native Init에 전달할 JSON과 제거/보존 건수.</returns>
    public static SwarmInputPreparationResult PrepareForRun(
        string json,
        bool useHints,
        int? groupWorkersOverride = null)
    {
        if (string.IsNullOrWhiteSpace(json))
        {
            return PreparationFailure("JSON이 비어 있습니다.");
        }

        if (groupWorkersOverride is < 1)
        {
            return PreparationFailure("Group Workers는 1 이상이어야 합니다.");
        }

        if (useHints && groupWorkersOverride is null)
        {
            return new SwarmInputPreparationResult
            {
                IsSuccess = true,
                Json = json
            };
        }

        SwarmInputSummary summary = Parse(json);
        if (!summary.IsValid)
        {
            return PreparationFailure(summary.ErrorMessage ?? "입력 JSON을 분석하지 못했습니다.");
        }

        // legacy hint는 별도 CPlacedHint[] 인자로 제어한다. JSON 자체는 변경하지 않는다.
        if (summary.SchemaVersion == 1)
        {
            return new SwarmInputPreparationResult
            {
                IsSuccess = true,
                Json = json
            };
        }

        try
        {
            JsonNode? parsed = JsonNode.Parse(json);
            if (parsed is not JsonObject root ||
                root["instance"] is not JsonObject instance ||
                instance["items"] is not JsonArray items)
            {
                return PreparationFailure("V2 instance.items를 찾을 수 없습니다.");
            }

            bool groupWorkersChanged = false;
            if (groupWorkersOverride is int configuredGroupWorkers)
            {
                if (root["nesting"] is not JsonObject nesting)
                {
                    return PreparationFailure("V2 nesting object를 찾을 수 없습니다.");
                }

                int currentGroupWorkers = nesting["group_workers"]?.GetValue<int>()
                    ?? DefaultGroupWorkers;
                groupWorkersChanged = currentGroupWorkers != configuredGroupWorkers;
                nesting["group_workers"] = configuredGroupWorkers;
            }

            HashSet<ulong> poseRequiredItemIds = useHints
                ? []
                : summary.PlaceGroups.SelectMany(group => group.MemberItemIds).ToHashSet();
            int removed = 0;
            int retainedRequired = 0;

            if (!useHints)
            {
                foreach (JsonNode? node in items)
                {
                    if (node is not JsonObject item || item["placed_hint"] is not JsonObject hint)
                    {
                        continue;
                    }

                    ulong itemId = ReadNodeUInt64(item["id"], "instance.items[].id");
                    bool isFrozen = ReadFreezeFlag(hint);
                    if (isFrozen || poseRequiredItemIds.Contains(itemId))
                    {
                        retainedRequired++;
                        continue;
                    }

                    item.Remove("placed_hint");
                    removed++;
                }
            }

            if (removed == 0 && !groupWorkersChanged)
            {
                return new SwarmInputPreparationResult
                {
                    IsSuccess = true,
                    Json = json,
                    RetainedRequiredHintCount = retainedRequired,
                    AppliedGroupWorkers = groupWorkersOverride
                };
            }

            return new SwarmInputPreparationResult
            {
                IsSuccess = true,
                Json = root.ToJsonString(OutputOptions),
                RemovedMovableHintCount = removed,
                RetainedRequiredHintCount = retainedRequired,
                AppliedGroupWorkers = groupWorkersOverride
            };
        }
        catch (Exception exception) when (exception is JsonException or InvalidOperationException or FormatException or OverflowException)
        {
            return PreparationFailure($"Use Hints 정책 적용 실패: {exception.Message}");
        }
    }

    private static SwarmInputSummary ParseLegacy(JsonElement root)
    {
        JsonElement instance = root.TryGetProperty("instance", out JsonElement nestedInstance) &&
                               nestedInstance.ValueKind == JsonValueKind.Object
            ? nestedInstance
            : root;
        if (!instance.TryGetProperty("items", out JsonElement items) ||
            items.ValueKind != JsonValueKind.Array)
        {
            return SwarmInputSummary.Invalid("legacy V1 입력에는 items 배열이 필요합니다.");
        }

        (int hintCount, int frozenCount) = CountHints(items);
        return new SwarmInputSummary
        {
            IsValid = true,
            SchemaVersion = 1,
            NestingMode = "legacy",
            LaneCount = 1,
            ItemCount = items.GetArrayLength(),
            PlacedHintCount = hintCount,
            FrozenHintCount = frozenCount,
            EffectiveFrozenItemCount = frozenCount,
            HintedLayout = HintedLayoutAnalyzer.Analyze(instance)
        };
    }

    private static SwarmInputSummary ParseV2(JsonElement root)
    {
        JsonElement nesting = RequireObject(root, "nesting");
        JsonElement instance = RequireObject(root, "instance");
        JsonElement items = RequireArray(instance, "items");
        string mode = RequireString(nesting, "mode");
        if (mode is not ("all_items" or "size_regions"))
        {
            return SwarmInputSummary.Invalid("nesting.mode는 all_items 또는 size_regions여야 합니다.");
        }

        int groupWorkers = ReadOptionalInt32(nesting, "group_workers", DefaultGroupWorkers);
        if (groupWorkers < 1)
        {
            return SwarmInputSummary.Invalid("nesting.group_workers는 1 이상이어야 합니다.");
        }

        long quantumMs = ReadOptionalInt64(nesting, "quantum_ms", DefaultQuantumMs);
        if (quantumMs is < 1 or > 60_000)
        {
            return SwarmInputSummary.Invalid("nesting.quantum_ms는 1..60000 범위여야 합니다.");
        }

        double lockedPrefixGapMm = ReadOptionalDouble(nesting, "locked_prefix_gap_mm", 0);
        if (!double.IsFinite(lockedPrefixGapMm) || lockedPrefixGapMm < 0)
        {
            return SwarmInputSummary.Invalid("nesting.locked_prefix_gap_mm는 유한한 0 이상의 값이어야 합니다.");
        }

        var itemIds = new HashSet<ulong>();
        var hintedItemIds = new HashSet<ulong>();
        var explicitlyFrozenItemIds = new HashSet<ulong>();
        int frozenCount = 0;
        foreach (JsonElement item in items.EnumerateArray())
        {
            if (item.ValueKind != JsonValueKind.Object ||
                !item.TryGetProperty("id", out JsonElement idElement) ||
                !idElement.TryGetUInt64(out ulong itemId))
            {
                return SwarmInputSummary.Invalid("instance.items의 모든 item에는 정수 id가 필요합니다.");
            }

            if (!itemIds.Add(itemId))
            {
                return SwarmInputSummary.Invalid($"item id {itemId}가 중복되었습니다.");
            }

            if (item.TryGetProperty("placed_hint", out JsonElement hint))
            {
                if (hint.ValueKind != JsonValueKind.Object)
                {
                    return SwarmInputSummary.Invalid($"item {itemId}의 placed_hint는 object여야 합니다.");
                }

                hintedItemIds.Add(itemId);
                if (ReadFreezeFlag(hint))
                {
                    frozenCount++;
                    explicitlyFrozenItemIds.Add(itemId);
                }
            }
        }

        IReadOnlyList<SwarmRegionSummary> regions = ParseRegions(root, itemIds, mode, out string? regionError);
        if (regionError is not null)
        {
            return SwarmInputSummary.Invalid(regionError);
        }

        IReadOnlyList<SwarmPlaceGroupSummary> placeGroups = ParsePlaceGroups(
            root,
            itemIds,
            hintedItemIds,
            regions,
            mode,
            out string? groupError);
        if (groupError is not null)
        {
            return SwarmInputSummary.Invalid(groupError);
        }

        var effectiveFrozenItemIds = new HashSet<ulong>(explicitlyFrozenItemIds);
        foreach (SwarmPlaceGroupSummary group in placeGroups)
        {
            if (group.MemberItemIds.Any(explicitlyFrozenItemIds.Contains))
            {
                effectiveFrozenItemIds.UnionWith(group.MemberItemIds);
            }
        }

        IReadOnlyList<SwarmRegionSummary> effectiveRegions = AnnotateEffectiveRegions(
            regions,
            effectiveFrozenItemIds);
        int laneCount = mode == "all_items"
            ? 1
            : effectiveRegions.Count(region => region.MovableItemCount > 0);

        return new SwarmInputSummary
        {
            IsValid = true,
            SchemaVersion = GroupedSchemaVersion,
            NestingMode = mode,
            LaneCount = laneCount,
            ItemCount = items.GetArrayLength(),
            GroupWorkers = groupWorkers,
            QuantumMs = quantumMs,
            LockedPrefixGapMm = lockedPrefixGapMm,
            Regions = effectiveRegions,
            PlaceGroups = placeGroups,
            PlacedHintCount = hintedItemIds.Count,
            FrozenHintCount = frozenCount,
            EffectiveFrozenItemCount = effectiveFrozenItemIds.Count,
            HintedLayout = HintedLayoutAnalyzer.Analyze(instance)
        };
    }

    private static IReadOnlyList<SwarmRegionSummary> AnnotateEffectiveRegions(
        IReadOnlyList<SwarmRegionSummary> regions,
        IReadOnlySet<ulong> frozenItemIds)
    {
        int[] movableCounts = regions
            .Select(region => region.ItemIds.Count(itemId => !frozenItemIds.Contains(itemId)))
            .ToArray();
        int lastMovableRegion = Array.FindLastIndex(movableCounts, count => count > 0);

        return regions
            .Select((region, index) => new SwarmRegionSummary
            {
                Id = region.Id,
                Kind = region.Kind,
                ItemIds = region.ItemIds,
                GapAfterMm = region.GapAfterMm,
                MovableItemCount = movableCounts[index],
                AppliedGapAfterMm = movableCounts[index] > 0 && index != lastMovableRegion
                    ? region.GapAfterMm
                    : 0
            })
            .ToArray();
    }

    private static IReadOnlyList<SwarmRegionSummary> ParseRegions(
        JsonElement root,
        HashSet<ulong> itemIds,
        string mode,
        out string? error)
    {
        error = null;
        if (!root.TryGetProperty("regions", out JsonElement regionsElement))
        {
            regionsElement = default;
        }
        else if (regionsElement.ValueKind != JsonValueKind.Array)
        {
            error = "regions는 배열이어야 합니다.";
            return [];
        }

        var regions = new List<SwarmRegionSummary>();
        var assigned = new HashSet<ulong>();
        if (regionsElement.ValueKind == JsonValueKind.Array)
        {
            foreach (JsonElement region in regionsElement.EnumerateArray())
            {
                if (region.ValueKind != JsonValueKind.Object)
                {
                    error = "regions의 각 항목은 object여야 합니다.";
                    return [];
                }

                string id = RequireString(region, "id");
                string kind = RequireString(region, "kind");
                JsonElement regionItems = RequireArray(region, "item_ids");
                var ids = new List<ulong>();
                foreach (JsonElement idElement in regionItems.EnumerateArray())
                {
                    if (!idElement.TryGetUInt64(out ulong itemId))
                    {
                        error = $"region {id}의 item_ids에는 정수만 사용할 수 있습니다.";
                        return [];
                    }

                    if (!itemIds.Contains(itemId))
                    {
                        error = $"region {id}가 존재하지 않는 item {itemId}를 참조합니다.";
                        return [];
                    }

                    if (!assigned.Add(itemId))
                    {
                        error = $"item {itemId}가 둘 이상의 region에 배정되었습니다.";
                        return [];
                    }

                    ids.Add(itemId);
                }

                double gap = ReadOptionalDouble(region, "gap_after_mm", 0);
                if (!double.IsFinite(gap) || gap < 0)
                {
                    error = $"region {id}의 gap_after_mm는 유한한 0 이상의 값이어야 합니다.";
                    return [];
                }

                regions.Add(new SwarmRegionSummary
                {
                    Id = id,
                    Kind = kind,
                    ItemIds = ids,
                    GapAfterMm = gap
                });
            }
        }

        if (mode == "all_items" && regions.Count != 0)
        {
            error = "all_items 모드에는 regions를 선언할 수 없습니다.";
            return [];
        }

        if (mode == "size_regions")
        {
            if (regions.Count == 0)
            {
                error = "size_regions 모드에는 region이 하나 이상 필요합니다.";
                return [];
            }

            if (!assigned.SetEquals(itemIds))
            {
                ulong missing = itemIds.First(id => !assigned.Contains(id));
                error = $"item {missing}가 어떤 region에도 배정되지 않았습니다.";
                return [];
            }
        }

        return regions;
    }

    private static IReadOnlyList<SwarmPlaceGroupSummary> ParsePlaceGroups(
        JsonElement root,
        HashSet<ulong> itemIds,
        HashSet<ulong> hintedItemIds,
        IReadOnlyList<SwarmRegionSummary> regions,
        string mode,
        out string? error)
    {
        error = null;
        if (!root.TryGetProperty("place_groups", out JsonElement groupsElement))
        {
            return [];
        }

        if (groupsElement.ValueKind != JsonValueKind.Array)
        {
            error = "place_groups는 배열이어야 합니다.";
            return [];
        }

        var result = new List<SwarmPlaceGroupSummary>();
        var groupIds = new HashSet<long>();
        var assignedMembers = new HashSet<ulong>();
        Dictionary<ulong, int> regionByItem = regions
            .SelectMany((region, index) => region.ItemIds.Select(itemId => (itemId, index)))
            .ToDictionary(pair => pair.itemId, pair => pair.index);

        foreach (JsonElement group in groupsElement.EnumerateArray())
        {
            if (group.ValueKind != JsonValueKind.Object ||
                !group.TryGetProperty("id", out JsonElement idElement) ||
                !idElement.TryGetInt64(out long groupId) ||
                groupId <= 0)
            {
                error = "PlaceGroup id는 양의 정수여야 합니다.";
                return [];
            }

            if (!groupIds.Add(groupId))
            {
                error = $"PlaceGroup id {groupId}가 중복되었습니다.";
                return [];
            }

            JsonElement membersElement = RequireArray(group, "member_item_ids");
            var members = new List<ulong>();
            foreach (JsonElement memberElement in membersElement.EnumerateArray())
            {
                if (!memberElement.TryGetUInt64(out ulong itemId))
                {
                    error = $"PlaceGroup {groupId}의 member_item_ids에는 정수만 사용할 수 있습니다.";
                    return [];
                }

                if (!itemIds.Contains(itemId))
                {
                    error = $"PlaceGroup {groupId}가 존재하지 않는 item {itemId}를 참조합니다.";
                    return [];
                }

                if (!assignedMembers.Add(itemId))
                {
                    error = $"item {itemId}가 둘 이상의 PlaceGroup에 속합니다.";
                    return [];
                }

                if (!hintedItemIds.Contains(itemId))
                {
                    error = $"PlaceGroup {groupId} member {itemId}에는 상대 pose용 placed_hint가 필요합니다.";
                    return [];
                }

                members.Add(itemId);
            }

            if (members.Count < 2)
            {
                error = $"PlaceGroup {groupId}에는 member가 두 개 이상 필요합니다.";
                return [];
            }

            if (mode == "size_regions" && members.Select(member => regionByItem[member]).Distinct().Skip(1).Any())
            {
                error = $"PlaceGroup {groupId}가 여러 size region에 걸쳐 있습니다.";
                return [];
            }

            result.Add(new SwarmPlaceGroupSummary
            {
                Id = groupId,
                MemberItemIds = members
            });
        }

        return result;
    }

    private static (int HintCount, int FrozenCount) CountHints(JsonElement items)
    {
        int hints = 0;
        int frozen = 0;
        foreach (JsonElement item in items.EnumerateArray())
        {
            if ((!item.TryGetProperty("placed_hint", out JsonElement hint)
                    && !item.TryGetProperty("placedHint", out hint)) ||
                hint.ValueKind != JsonValueKind.Object)
            {
                continue;
            }

            hints++;
            if (ReadFreezeFlag(hint))
            {
                frozen++;
            }
        }

        return (hints, frozen);
    }

    private static bool ReadFreezeFlag(JsonElement hint)
    {
        if (!hint.TryGetProperty("is_freeze", out JsonElement freeze) &&
            !hint.TryGetProperty("isFreeze", out freeze))
        {
            return false;
        }

        return freeze.ValueKind switch
        {
            JsonValueKind.True => true,
            JsonValueKind.False => false,
            JsonValueKind.Number when freeze.TryGetInt64(out long value) => value != 0,
            _ => throw new InvalidOperationException("placed_hint.is_freeze는 정수 또는 bool이어야 합니다.")
        };
    }

    private static bool ReadFreezeFlag(JsonObject hint)
    {
        JsonNode? freeze = hint["is_freeze"] ?? hint["isFreeze"];
        if (freeze is null)
        {
            return false;
        }

        if (freeze is JsonValue value)
        {
            if (value.TryGetValue<bool>(out bool boolValue))
            {
                return boolValue;
            }

            if (value.TryGetValue<long>(out long integerValue))
            {
                return integerValue != 0;
            }

            if (value.TryGetValue<int>(out int intValue))
            {
                return intValue != 0;
            }
        }

        throw new InvalidOperationException("placed_hint.is_freeze는 정수 또는 bool이어야 합니다.");
    }

    private static ulong ReadNodeUInt64(JsonNode? node, string propertyPath)
    {
        if (node is JsonValue value)
        {
            if (value.TryGetValue<ulong>(out ulong unsignedValue))
            {
                return unsignedValue;
            }

            if (value.TryGetValue<long>(out long longValue) && longValue >= 0)
            {
                return (ulong)longValue;
            }

            if (value.TryGetValue<int>(out int intValue) && intValue >= 0)
            {
                return (ulong)intValue;
            }
        }

        throw new InvalidOperationException($"{propertyPath}는 정수여야 합니다.");
    }

    private static JsonElement RequireObject(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            value.ValueKind != JsonValueKind.Object)
        {
            throw new InvalidOperationException($"{propertyName} object가 필요합니다.");
        }

        return value;
    }

    private static JsonElement RequireArray(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            value.ValueKind != JsonValueKind.Array)
        {
            throw new InvalidOperationException($"{propertyName} 배열이 필요합니다.");
        }

        return value;
    }

    private static string RequireString(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            value.ValueKind != JsonValueKind.String)
        {
            throw new InvalidOperationException($"{propertyName} 문자열이 필요합니다.");
        }

        return value.GetString()!;
    }

    private static int ReadOptionalInt32(JsonElement parent, string propertyName, int defaultValue)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value))
        {
            return defaultValue;
        }

        if (!value.TryGetInt32(out int result))
        {
            throw new InvalidOperationException($"{propertyName}은 정수여야 합니다.");
        }

        return result;
    }

    private static long ReadOptionalInt64(JsonElement parent, string propertyName, long defaultValue)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value))
        {
            return defaultValue;
        }

        if (!value.TryGetInt64(out long result))
        {
            throw new InvalidOperationException($"{propertyName}은 정수여야 합니다.");
        }

        return result;
    }

    private static double ReadOptionalDouble(JsonElement parent, string propertyName, double defaultValue)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value))
        {
            return defaultValue;
        }

        if (!value.TryGetDouble(out double result))
        {
            throw new InvalidOperationException($"{propertyName}은 숫자여야 합니다.");
        }

        return result;
    }

    private static SwarmInputPreparationResult PreparationFailure(string message) => new()
    {
        IsSuccess = false,
        ErrorMessage = message
    };
}
