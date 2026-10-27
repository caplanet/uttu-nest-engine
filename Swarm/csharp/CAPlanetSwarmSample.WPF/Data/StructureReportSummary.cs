using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;

namespace CAPlanetSwarmSample.WPF.Data;

/// <summary>Structure schema 3 감사 보고서의 WPF 표시용 요약.</summary>
public sealed class StructureReportSummary
{
    /// <summary>보고서를 성공적으로 분석했는지 여부.</summary>
    public bool IsValid { get; init; }

    /// <summary>보고서 분석 실패 메시지. 성공하면 <see langword="null"/>.</summary>
    public string? ErrorMessage { get; init; }

    /// <summary>Structure 보고서 schema 버전.</summary>
    public int SchemaVersion { get; init; }

    /// <summary>legacy, all_items 또는 size_regions 실행 모드.</summary>
    public string NestingMode { get; init; } = "unknown";

    /// <summary>보고서가 grouped top-level envelope인지 여부.</summary>
    public bool IsGroupedReport { get; init; }

    /// <summary>grouped scheduler의 최대 병렬 lane 수. legacy 보고서이면 <see langword="null"/>.</summary>
    public int? GroupWorkers { get; init; }

    /// <summary>CPU 제한을 적용한 실제 병렬 lane 수. 이전 보고서에는 없을 수 있다.</summary>
    public int? EffectiveGroupWorkers { get; init; }

    /// <summary>grouped scheduler의 시간 조각(ms). legacy 보고서이면 <see langword="null"/>.</summary>
    public long? QuantumMs { get; init; }

    /// <summary>잠금 prefix에 포함된 item 수.</summary>
    public int FixedPrefixItemCount { get; init; }

    /// <summary>잠금 prefix의 오른쪽 끝 좌표(mm).</summary>
    public double FixedPrefixRightMm { get; init; }

    /// <summary>잠금 prefix 뒤에 요구된 간격(mm).</summary>
    public double LockedPrefixGapMm { get; init; }

    /// <summary>모든 region 사이에 추가된 필수 간격 합(mm).</summary>
    public double RequiredRegionGapMm { get; init; }

    /// <summary>최종 strip 폭(mm).</summary>
    public double FinalStripWidth { get; init; }

    /// <summary>최종 밀도. 0..1 비율.</summary>
    public double FinalDensity { get; init; }

    /// <summary>단일 report의 경과 시간 또는 grouped report에서 가장 오래 걸린 lane 시간(초).</summary>
    public double ElapsedSeconds { get; init; }

    /// <summary>최종 incumbent를 만든 단계. lane마다 다르면 쉼표로 결합한다.</summary>
    public string BestStage { get; init; } = "unknown";

    /// <summary>P0가 생성한 motif 총수.</summary>
    public int MotifCount { get; init; }

    /// <summary>P1 construction start 시도 총수.</summary>
    public int P1StartsAttempted { get; init; }

    /// <summary>완료된 P1 construction start 총수.</summary>
    public int P1StartsCompleted { get; init; }

    /// <summary>P4 repair cycle 총수.</summary>
    public int RepairCycleCount { get; init; }

    /// <summary>lane별 Structure 요약.</summary>
    public IReadOnlyList<StructureLaneReportSummary> Lanes { get; init; } = [];

    /// <summary>보고서에 포함된 lane 수.</summary>
    public int LaneCount => Lanes.Count;

    /// <summary>사용자 PlaceGroup 또는 잠금 정책 때문에 soft P0/P1이 fallback된 lane 수.</summary>
    public int FallbackLaneCount => Lanes.Count(lane => !string.IsNullOrWhiteSpace(lane.FallbackReason));

    /// <summary>좌측 패널에 표시할 핵심 지표.</summary>
    public string DisplayText
    {
        get
        {
            if (!IsValid)
            {
                return $"Structure report 분석 실패: {ErrorMessage ?? "알 수 없는 오류"}";
            }

            var builder = new StringBuilder();
            builder.AppendLine($"완료 · best {FormatStage(BestStage)} · lane {LaneCount}");
            builder.Append("폭 ");
            builder.Append(FormatNumber(FinalStripWidth));
            builder.Append(" mm · 밀도 ");
            builder.Append((FinalDensity * 100).ToString("0.00", CultureInfo.InvariantCulture));
            builder.AppendLine("%");
            builder.AppendLine($"P1 starts {P1StartsCompleted}/{P1StartsAttempted} · motifs {MotifCount} · P4 cycles {RepairCycleCount}");
            if (IsGroupedReport)
            {
                int? workers = EffectiveGroupWorkers ?? GroupWorkers;
                string requested = EffectiveGroupWorkers.HasValue && EffectiveGroupWorkers != GroupWorkers
                    ? $" (요청 {GroupWorkers})" : string.Empty;
                builder.AppendLine($"workers {workers}{requested} · quantum {QuantumMs} ms");
                builder.Append("prefix ");
                builder.Append(FixedPrefixItemCount);
                builder.Append(" items @ ");
                builder.Append(FormatNumber(FixedPrefixRightMm));
                builder.Append(" mm · +");
                builder.Append(FormatNumber(LockedPrefixGapMm));
                builder.AppendLine(" mm");
                builder.Append($"required region gap {FormatNumber(RequiredRegionGapMm)} mm · ");
            }
            builder.Append($"경과 {FormatNumber(ElapsedSeconds)} s");

            if (FallbackLaneCount > 0)
            {
                builder.AppendLine();
                builder.Append($"soft P0/P1 fallback lane {FallbackLaneCount}");
            }

            return builder.ToString();
        }
    }

    /// <summary>보고서 분석 실패 결과를 생성한다.</summary>
    public static StructureReportSummary Invalid(string message) => new()
    {
        IsValid = false,
        ErrorMessage = message
    };

    private static string FormatStage(string stage) => stage
        .Replace("p0_preprocess", "P0")
        .Replace("p1_construct", "P1")
        .Replace("p2_compress", "P2")
        .Replace("p3_explore", "P3")
        .Replace("p4_refine", "P4")
        .Replace("p5_finish", "P5");

    private static string FormatNumber(double value) =>
        value.ToString("0.###", CultureInfo.InvariantCulture);
}

/// <summary>grouped Structure 보고서의 단일 lane 요약.</summary>
public sealed class StructureLaneReportSummary
{
    /// <summary>0부터 시작하는 lane 순번.</summary>
    public int LaneIndex { get; init; }

    /// <summary>lane에 속한 전역 item ID 목록.</summary>
    public IReadOnlyList<ulong> GlobalItemIds { get; init; } = [];

    /// <summary>lane 내부의 PlaceGroup 수.</summary>
    public int PlaceGroupCount { get; init; }

    /// <summary>lane의 잠금 item 수.</summary>
    public int FrozenItemCount { get; init; }

    /// <summary>soft motif P0/P1 경로가 실제 적용됐는지 여부.</summary>
    public bool SoftStructureApplied { get; init; }

    /// <summary>soft P0/P1을 사용하지 않은 이유. 적용됐다면 <see langword="null"/>.</summary>
    public string? FallbackReason { get; init; }

    /// <summary>lane의 최종 incumbent를 만든 단계.</summary>
    public string BestStage { get; init; } = "unknown";

    /// <summary>lane의 최종 폭(mm).</summary>
    public double FinalWidth { get; init; }

    /// <summary>lane의 최종 밀도. 0..1 비율.</summary>
    public double FinalDensity { get; init; }

    /// <summary>lane Structure 경과 시간(초).</summary>
    public double ElapsedSeconds { get; init; }

    /// <summary>P0에서 생성한 motif 수.</summary>
    public int MotifCount { get; init; }

    /// <summary>P1 construction start 시도 수.</summary>
    public int P1StartsAttempted { get; init; }

    /// <summary>P1 construction start 완료 수.</summary>
    public int P1StartsCompleted { get; init; }

    /// <summary>P4 repair cycle 수.</summary>
    public int RepairCycleCount { get; init; }

    /// <summary>완료된 checkpoint stage 목록.</summary>
    public IReadOnlyList<string> CheckpointStages { get; init; } = [];
}

/// <summary>ABI의 generic ReportType을 이용해 추정한 현재 Structure 단계.</summary>
public enum StructureUiStage
{
    /// <summary>Structure 실행 전.</summary>
    Idle,
    /// <summary>Init baseline 또는 P1 construction 구간.</summary>
    P1Construct,
    /// <summary>P2 압축 구간.</summary>
    P2Compress,
    /// <summary>P3 탐색 구간.</summary>
    P3Explore,
    /// <summary>P4 gap refine 구간.</summary>
    P4Refine,
    /// <summary>P5 finish/polish 구간.</summary>
    P5Finish,
    /// <summary>최종 callback을 받은 완료 상태.</summary>
    Completed
}

/// <summary>
/// Structure callback의 generic ReportType만으로 표시 가능한 보수적 진행 상태.
/// lane completion과 ordinary P1 preview는 ABI에서 구분되지 않으므로 callback 수만 제공한다.
/// </summary>
public sealed class StructureProgressTracker
{
    /// <summary>현재 추정 단계.</summary>
    public StructureUiStage CurrentStage { get; private set; } = StructureUiStage.Idle;

    /// <summary>이번 Structure 실행에서 받은 전체 callback 수.</summary>
    public int CallbackCount { get; private set; }

    /// <summary>P1로 해석된 callback 수. baseline, lane 완료와 preview가 모두 포함될 수 있다.</summary>
    public int P1CallbackCount { get; private set; }

    /// <summary>현재 진행 상황을 명시적으로 '추정'이라고 표시하는 UI 문자열.</summary>
    public string DisplayText => CurrentStage switch
    {
        StructureUiStage.Idle => "Structure 대기",
        StructureUiStage.P1Construct => $"P1 Construct 실행 중 · callback {P1CallbackCount} (단계 추정)",
        StructureUiStage.P2Compress => "P2 Compress 실행 중 (단계 추정)",
        StructureUiStage.P3Explore => "P3 Explore 실행 중 (단계 추정)",
        StructureUiStage.P4Refine => "P4 Refine 실행 중 (단계 추정)",
        StructureUiStage.P5Finish => "P5 Finish 실행 중 (단계 추정)",
        StructureUiStage.Completed => "Structure 완료",
        _ => "Structure 상태 알 수 없음"
    };

    /// <summary>새 실행을 위해 상태와 callback 건수를 초기화한다.</summary>
    public void Reset()
    {
        CurrentStage = StructureUiStage.Idle;
        CallbackCount = 0;
        P1CallbackCount = 0;
    }

    /// <summary>
    /// native ReportType 정수값을 관찰한다. 0=ExplFeas, 2=ExplImproving,
    /// 3=CmprFeas, 4=Final 계약을 사용한다.
    /// </summary>
    /// <param name="reportType">callback의 ReportType 정수값.</param>
    /// <returns>관찰 후의 추정 단계.</returns>
    public StructureUiStage Observe(int reportType)
    {
        CallbackCount++;
        switch (reportType)
        {
            case 4:
                CurrentStage = StructureUiStage.Completed;
                break;
            case 2:
                CurrentStage = StructureUiStage.P4Refine;
                break;
            case 3:
                CurrentStage = (int)CurrentStage >= (int)StructureUiStage.P3Explore
                    ? StructureUiStage.P5Finish
                    : StructureUiStage.P2Compress;
                break;
            case 0:
                if (CurrentStage is StructureUiStage.Idle or StructureUiStage.P1Construct)
                {
                    CurrentStage = StructureUiStage.P1Construct;
                    P1CallbackCount++;
                }
                else if (CurrentStage == StructureUiStage.P2Compress)
                {
                    CurrentStage = StructureUiStage.P3Explore;
                }
                break;
        }

        return CurrentStage;
    }
}
