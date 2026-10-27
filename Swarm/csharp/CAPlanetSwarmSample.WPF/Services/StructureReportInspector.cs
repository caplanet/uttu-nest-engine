using CAPlanetSwarmSample.WPF.Data;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;

namespace CAPlanetSwarmSample.WPF.Services;

/// <summary>native가 반환한 Structure schema 3 감사 JSON을 WPF 표시 모델로 변환한다.</summary>
public static class StructureReportInspector
{
    private const int SupportedSchemaVersion = 3;

    /// <summary>
    /// legacy 단일 report와 grouped top/lane report를 모두 분석한다.
    /// </summary>
    /// <param name="json">swarm_get_structure_report_json 반환 문자열.</param>
    /// <returns>예외를 노출하지 않는 UI용 보고서 요약.</returns>
    public static StructureReportSummary Parse(string json)
    {
        if (string.IsNullOrWhiteSpace(json))
        {
            return StructureReportSummary.Invalid("보고서 JSON이 비어 있습니다.");
        }

        try
        {
            using var document = JsonDocument.Parse(json);
            JsonElement root = document.RootElement;
            if (root.ValueKind != JsonValueKind.Object)
            {
                return StructureReportSummary.Invalid("보고서 root는 object여야 합니다.");
            }

            int schemaVersion = ReadRequiredInt32(root, "schema_version");
            if (schemaVersion != SupportedSchemaVersion)
            {
                return StructureReportSummary.Invalid($"지원하지 않는 Structure schema_version {schemaVersion}입니다.");
            }

            return root.TryGetProperty("lanes", out JsonElement lanes)
                ? ParseGrouped(root, lanes)
                : ParseSingle(root);
        }
        catch (JsonException exception)
        {
            return StructureReportSummary.Invalid($"보고서 JSON 구문 오류: {exception.Message}");
        }
        catch (InvalidOperationException exception)
        {
            return StructureReportSummary.Invalid(exception.Message);
        }
        catch (OverflowException exception)
        {
            return StructureReportSummary.Invalid($"보고서 숫자 범위를 벗어났습니다: {exception.Message}");
        }
    }

    private static StructureReportSummary ParseSingle(JsonElement root)
    {
        CoreReport core = ParseCoreReport(root);
        var lane = new StructureLaneReportSummary
        {
            LaneIndex = 0,
            SoftStructureApplied = true,
            BestStage = core.BestStage,
            FinalWidth = core.FinalWidth,
            FinalDensity = core.FinalDensity,
            ElapsedSeconds = core.ElapsedSeconds,
            MotifCount = core.MotifCount,
            P1StartsAttempted = core.P1StartsAttempted,
            P1StartsCompleted = core.P1StartsCompleted,
            RepairCycleCount = core.RepairCycleCount,
            CheckpointStages = core.CheckpointStages
        };

        return new StructureReportSummary
        {
            IsValid = true,
            SchemaVersion = SupportedSchemaVersion,
            NestingMode = "legacy",
            IsGroupedReport = false,
            FinalStripWidth = core.FinalWidth,
            FinalDensity = core.FinalDensity,
            ElapsedSeconds = core.ElapsedSeconds,
            BestStage = core.BestStage,
            MotifCount = core.MotifCount,
            P1StartsAttempted = core.P1StartsAttempted,
            P1StartsCompleted = core.P1StartsCompleted,
            RepairCycleCount = core.RepairCycleCount,
            Lanes = [lane]
        };
    }

    private static StructureReportSummary ParseGrouped(JsonElement root, JsonElement lanesElement)
    {
        if (lanesElement.ValueKind != JsonValueKind.Array)
        {
            return StructureReportSummary.Invalid("grouped report의 lanes는 배열이어야 합니다.");
        }

        string nestingMode = ReadRequiredString(root, "nesting_mode");
        if (nestingMode is not ("all_items" or "size_regions"))
        {
            return StructureReportSummary.Invalid("grouped report의 nesting_mode가 올바르지 않습니다.");
        }

        var lanes = new List<StructureLaneReportSummary>();
        foreach (JsonElement laneElement in lanesElement.EnumerateArray())
        {
            if (laneElement.ValueKind != JsonValueKind.Object)
            {
                return StructureReportSummary.Invalid("grouped report의 lane은 object여야 합니다.");
            }

            JsonElement coreElement = RequireObject(laneElement, "report");
            CoreReport core = ParseCoreReport(coreElement);
            lanes.Add(new StructureLaneReportSummary
            {
                LaneIndex = ReadRequiredInt32(laneElement, "lane_index"),
                GlobalItemIds = ReadUInt64Array(laneElement, "global_item_ids"),
                PlaceGroupCount = ReadOptionalInt32(laneElement, "place_group_count", 0),
                FrozenItemCount = ReadOptionalInt32(laneElement, "frozen_item_count", 0),
                SoftStructureApplied = ReadOptionalBoolean(laneElement, "p0_p1_soft_structure_applied", true),
                FallbackReason = ReadOptionalString(laneElement, "fallback_reason"),
                BestStage = core.BestStage,
                FinalWidth = core.FinalWidth,
                FinalDensity = core.FinalDensity,
                ElapsedSeconds = core.ElapsedSeconds,
                MotifCount = core.MotifCount,
                P1StartsAttempted = core.P1StartsAttempted,
                P1StartsCompleted = core.P1StartsCompleted,
                RepairCycleCount = core.RepairCycleCount,
                CheckpointStages = core.CheckpointStages
            });
        }

        int declaredLaneCount = ReadOptionalInt32(root, "lane_count", lanes.Count);
        if (declaredLaneCount != lanes.Count)
        {
            return StructureReportSummary.Invalid($"lane_count {declaredLaneCount}와 lanes 배열 길이 {lanes.Count}가 다릅니다.");
        }

        // size_regions는 모든 item이 frozen prefix에 속하면 movable lane이 0개일 수 있다.
        // 이 경우에도 native grouped report는 완전한 최종 해를 나타내므로 실패로 취급하지 않는다.
        string bestStages = lanes.Count == 0
            ? "fixed_prefix"
            : string.Join(",", lanes
                .Select(lane => lane.BestStage)
                .Distinct(StringComparer.Ordinal));

        return new StructureReportSummary
        {
            IsValid = true,
            SchemaVersion = SupportedSchemaVersion,
            NestingMode = nestingMode,
            IsGroupedReport = true,
            GroupWorkers = ReadRequiredInt32(root, "group_workers"),
            EffectiveGroupWorkers = root.TryGetProperty("effective_group_workers", out _)
                ? ReadRequiredInt32(root, "effective_group_workers") : null,
            QuantumMs = ReadRequiredInt64(root, "quantum_ms"),
            FixedPrefixItemCount = ReadOptionalInt32(root, "fixed_prefix_item_count", 0),
            FixedPrefixRightMm = ReadOptionalDouble(root, "fixed_prefix_right_mm", 0),
            LockedPrefixGapMm = ReadOptionalDouble(root, "locked_prefix_gap_mm", 0),
            RequiredRegionGapMm = ReadOptionalDouble(root, "required_region_gap_mm", 0),
            FinalStripWidth = ReadRequiredDouble(root, "final_strip_width"),
            FinalDensity = ReadRequiredDouble(root, "final_density"),
            ElapsedSeconds = lanes.Count == 0 ? 0 : lanes.Max(lane => lane.ElapsedSeconds),
            BestStage = bestStages,
            MotifCount = lanes.Sum(lane => lane.MotifCount),
            P1StartsAttempted = lanes.Sum(lane => lane.P1StartsAttempted),
            P1StartsCompleted = lanes.Sum(lane => lane.P1StartsCompleted),
            RepairCycleCount = lanes.Sum(lane => lane.RepairCycleCount),
            Lanes = lanes
        };
    }

    private static CoreReport ParseCoreReport(JsonElement report)
    {
        int schemaVersion = ReadRequiredInt32(report, "schema_version");
        if (schemaVersion != SupportedSchemaVersion)
        {
            throw new InvalidOperationException($"lane Structure schema_version {schemaVersion}은 지원하지 않습니다.");
        }

        var checkpointStages = new List<string>();
        if (report.TryGetProperty("checkpoints", out JsonElement checkpoints))
        {
            if (checkpoints.ValueKind != JsonValueKind.Array)
            {
                throw new InvalidOperationException("checkpoints는 배열이어야 합니다.");
            }

            foreach (JsonElement checkpoint in checkpoints.EnumerateArray())
            {
                checkpointStages.Add(ReadRequiredString(checkpoint, "stage"));
            }
        }

        int repairCycles = 0;
        if (report.TryGetProperty("repair_cycles", out JsonElement repairCyclesElement))
        {
            if (repairCyclesElement.ValueKind != JsonValueKind.Array)
            {
                throw new InvalidOperationException("repair_cycles는 배열이어야 합니다.");
            }

            repairCycles = repairCyclesElement.GetArrayLength();
        }

        int motifCount =
            ReadOptionalInt32(report, "motif_singles", 0) +
            ReadOptionalInt32(report, "motif_pairs", 0) +
            ReadOptionalInt32(report, "motif_blocks", 0) +
            ReadOptionalInt32(report, "motif_bands", 0);

        return new CoreReport(
            ReadRequiredString(report, "best_stage"),
            ReadRequiredDouble(report, "final_width"),
            ReadRequiredDouble(report, "final_density"),
            ReadRequiredDouble(report, "elapsed_secs"),
            motifCount,
            ReadOptionalInt32(report, "construction_starts_attempted", 0),
            ReadOptionalInt32(report, "construction_starts_completed", 0),
            repairCycles,
            checkpointStages);
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

    private static IReadOnlyList<ulong> ReadUInt64Array(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement array) ||
            array.ValueKind != JsonValueKind.Array)
        {
            throw new InvalidOperationException($"{propertyName} 배열이 필요합니다.");
        }

        var result = new List<ulong>();
        foreach (JsonElement element in array.EnumerateArray())
        {
            if (!element.TryGetUInt64(out ulong value))
            {
                throw new InvalidOperationException($"{propertyName}에는 정수만 사용할 수 있습니다.");
            }

            result.Add(value);
        }

        return result;
    }

    private static string ReadRequiredString(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            value.ValueKind != JsonValueKind.String)
        {
            throw new InvalidOperationException($"{propertyName} 문자열이 필요합니다.");
        }

        return value.GetString()!;
    }

    private static string? ReadOptionalString(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            value.ValueKind == JsonValueKind.Null)
        {
            return null;
        }

        if (value.ValueKind != JsonValueKind.String)
        {
            throw new InvalidOperationException($"{propertyName}은 문자열 또는 null이어야 합니다.");
        }

        return value.GetString();
    }

    private static int ReadRequiredInt32(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            !value.TryGetInt32(out int result))
        {
            throw new InvalidOperationException($"{propertyName} 정수가 필요합니다.");
        }

        return result;
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

    private static long ReadRequiredInt64(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            !value.TryGetInt64(out long result))
        {
            throw new InvalidOperationException($"{propertyName} 정수가 필요합니다.");
        }

        return result;
    }

    private static double ReadRequiredDouble(JsonElement parent, string propertyName)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value) ||
            !value.TryGetDouble(out double result) ||
            !double.IsFinite(result))
        {
            throw new InvalidOperationException($"{propertyName} 유한 숫자가 필요합니다.");
        }

        return result;
    }

    private static double ReadOptionalDouble(JsonElement parent, string propertyName, double defaultValue)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value))
        {
            return defaultValue;
        }

        if (!value.TryGetDouble(out double result) || !double.IsFinite(result))
        {
            throw new InvalidOperationException($"{propertyName}은 유한 숫자여야 합니다.");
        }

        return result;
    }

    private static bool ReadOptionalBoolean(JsonElement parent, string propertyName, bool defaultValue)
    {
        if (!parent.TryGetProperty(propertyName, out JsonElement value))
        {
            return defaultValue;
        }

        return value.ValueKind switch
        {
            JsonValueKind.True => true,
            JsonValueKind.False => false,
            _ => throw new InvalidOperationException($"{propertyName}은 bool이어야 합니다.")
        };
    }

    private sealed record CoreReport(
        string BestStage,
        double FinalWidth,
        double FinalDensity,
        double ElapsedSeconds,
        int MotifCount,
        int P1StartsAttempted,
        int P1StartsCompleted,
        int RepairCycleCount,
        IReadOnlyList<string> CheckpointStages);
}
