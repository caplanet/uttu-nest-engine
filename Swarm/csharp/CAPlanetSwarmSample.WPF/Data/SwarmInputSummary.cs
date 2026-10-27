using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;

namespace CAPlanetSwarmSample.WPF.Data;

/// <summary>
/// Swarm 입력 JSON에서 UI에 표시할 그룹 실행 계약을 요약한다.
/// </summary>
public sealed class SwarmInputSummary
{
    /// <summary>입력 구조를 WPF에서 안전하게 읽었는지 여부.</summary>
    public bool IsValid { get; init; }

    /// <summary>파싱 또는 최소 계약 검증 실패 메시지. 성공하면 <see langword="null"/>.</summary>
    public string? ErrorMessage { get; init; }

    /// <summary>입력 schema 버전. legacy V1은 1이다.</summary>
    public int SchemaVersion { get; init; }

    /// <summary>원본 instance에 선언된 item 종류 수.</summary>
    public int ItemCount { get; init; }

    /// <summary>
    /// 실행 모드. legacy V1은 <c>legacy</c>, V2는 <c>all_items</c> 또는
    /// <c>size_regions</c>이다.
    /// </summary>
    public string NestingMode { get; init; } = "unknown";

    /// <summary>동시에 실행할 수 있는 size-region lane 수. V1이면 <see langword="null"/>.</summary>
    public int? GroupWorkers { get; init; }

    /// <summary>size-region scheduler의 시간 조각(ms). V1이면 <see langword="null"/>.</summary>
    public long? QuantumMs { get; init; }

    /// <summary>잠금 prefix 뒤에 적용하는 간격(mm). V1이면 <see langword="null"/>.</summary>
    public double? LockedPrefixGapMm { get; init; }

    /// <summary>입력에 선언된 size region 목록. <c>all_items</c>에서는 비어 있다.</summary>
    public IReadOnlyList<SwarmRegionSummary> Regions { get; init; } = [];

    /// <summary>입력에 선언된 강체 PlaceGroup 목록.</summary>
    public IReadOnlyList<SwarmPlaceGroupSummary> PlaceGroups { get; init; } = [];

    /// <summary>입력에 명시된 placed_hint 수.</summary>
    public int PlacedHintCount { get; init; }

    /// <summary>입력에서 <c>is_freeze != 0</c>인 잠금 hint 수.</summary>
    public int FrozenHintCount { get; init; }

    /// <summary>
    /// PlaceGroup의 일부 멤버 잠금을 그룹 전체로 전파한 뒤 실제 잠금되는 item 수.
    /// V1에는 PlaceGroup이 없으므로 <see cref="FrozenHintCount"/>와 같다.
    /// </summary>
    public int EffectiveFrozenItemCount { get; init; }

    /// <summary>
    /// 잠금 prefix item을 제외한 뒤 실제로 생성되는 movable lane 수.
    /// legacy와 all_items는 1이며, fixed-only size_regions는 0일 수 있다.
    /// </summary>
    public int LaneCount { get; init; }

    /// <summary>
    /// <c>placed_hint</c>가 지정한 배치의 효율과 겹침입니다. 힌트가 없으면
    /// <see cref="HintedLayoutMetrics.IsAvailable"/>가 false입니다.
    /// </summary>
    public HintedLayoutMetrics HintedLayout { get; init; } =
        HintedLayoutMetrics.Unavailable("분석 전");

    /// <summary>
    /// movable region 사이에 실제 적용되는 전체 후행 간격(mm). 잠금만 있는 region은
    /// lane에서 제외되고, 마지막 movable region의 <c>gap_after_mm</c>는 적용하지 않는다.
    /// </summary>
    public double TotalRegionGapMm => Regions.Sum(region => region.AppliedGapAfterMm);

    /// <summary>좌측 UI에 바로 표시할 수 있는 여러 줄 요약.</summary>
    public string DisplayText
    {
        get
        {
            if (!IsValid)
            {
                return $"입력 분석 실패: {ErrorMessage ?? "알 수 없는 오류"}";
            }

            var builder = new StringBuilder();
            builder.Append($"Schema V{SchemaVersion} · {FormatMode(NestingMode)}");
            builder.AppendLine($" · Item {ItemCount}");

            if (SchemaVersion == 1)
            {
                builder.AppendLine("영역 1 (전체 Item · 기존 실행)");
            }
            else
            {
                if (NestingMode == "size_regions")
                {
                    builder.Append($"영역 {Regions.Count} · movable lane {LaneCount} · workers {GroupWorkers}");
                }
                else
                {
                    builder.Append($"Lane {LaneCount} · workers {GroupWorkers}");
                }
                builder.AppendLine($" · quantum {QuantumMs} ms");
                builder.Append("간격: region ");
                builder.Append(FormatNumber(TotalRegionGapMm));
                builder.Append(" mm · locked prefix ");
                builder.Append(FormatNumber(LockedPrefixGapMm ?? 0));
                builder.AppendLine(" mm");
            }

            builder.Append($"PlaceGroup {PlaceGroups.Count} · hints {PlacedHintCount}");
            builder.Append($" (잠금 명시 {FrozenHintCount} / 적용 {EffectiveFrozenItemCount})");

            // 힌트가 배치를 담고 있으면 그 배치의 효율과 겹침을 바로 보여준다.
            // 상용 마커를 변환한 입력에서는 이 값이 실행 결과와 비교할 기준선이다.
            if (HintedLayout.DisplayLine is string layoutLine)
            {
                builder.AppendLine();
                builder.Append(layoutLine);
            }

            if (Regions.Count > 0)
            {
                for (int index = 0; index < Regions.Count; index++)
                {
                    SwarmRegionSummary region = Regions[index];
                    builder.AppendLine();
                    builder.Append($"  {region.Id} [{region.Kind}] {region.ItemCount} items");
                    if (region.MovableItemCount == 0)
                    {
                        builder.Append(" · fixed prefix only");
                    }
                    else if (region.MovableItemCount != region.ItemCount)
                    {
                        builder.Append($" · movable {region.MovableItemCount}");
                    }

                    if (region.AppliedGapAfterMm > 0)
                    {
                        builder.Append($" · +{FormatNumber(region.AppliedGapAfterMm)} mm");
                    }
                    else if (region.GapAfterMm > 0)
                    {
                        builder.Append($" · gap {FormatNumber(region.GapAfterMm)} mm 미적용");
                    }
                }
            }

            if (PlaceGroups.Count > 0)
            {
                foreach (var group in PlaceGroups)
                {
                    builder.AppendLine();
                    builder.Append($"  PlaceGroup {group.Id}: ");
                    builder.Append(string.Join(", ", group.MemberItemIds));
                }
            }

            return builder.ToString();
        }
    }

    /// <summary>입력 분석 실패 결과를 생성한다.</summary>
    /// <param name="message">사용자에게 표시할 실패 원인.</param>
    /// <returns>실패 상태의 요약.</returns>
    public static SwarmInputSummary Invalid(string message) => new()
    {
        IsValid = false,
        ErrorMessage = message
    };

    private static string FormatMode(string mode) => mode switch
    {
        "legacy" => "기존 전체 네스팅",
        "all_items" => "V2 전체 Item",
        "size_regions" => "사이즈 영역",
        _ => mode
    };

    private static string FormatNumber(double value) =>
        value.ToString("0.###", CultureInfo.InvariantCulture);
}

/// <summary>하나의 size region과 그 후행 간격을 표시하기 위한 요약.</summary>
public sealed class SwarmRegionSummary
{
    /// <summary>converter가 생성한 region 식별자.</summary>
    public string Id { get; init; } = string.Empty;

    /// <summary><c>size_group</c>, <c>size_bundle</c>, <c>remainder</c> 등의 region 종류.</summary>
    public string Kind { get; init; } = string.Empty;

    /// <summary>region에 속한 전역 item ID 목록.</summary>
    public IReadOnlyList<ulong> ItemIds { get; init; } = [];

    /// <summary>이 region 뒤에 더하는 간격(mm).</summary>
    public double GapAfterMm { get; init; }

    /// <summary>잠금 prefix item을 제외하고 이 region의 lane에 남는 item 수.</summary>
    public int MovableItemCount { get; init; }

    /// <summary>실제 다음 movable lane 앞에 적용되는 후행 간격(mm).</summary>
    public double AppliedGapAfterMm { get; init; }

    /// <summary>region에 포함된 item 종류 수.</summary>
    public int ItemCount => ItemIds.Count;
}

/// <summary>여러 Place Item을 강체로 묶는 PlaceGroup 입력 요약.</summary>
public sealed class SwarmPlaceGroupSummary
{
    /// <summary>양수인 PlaceGroup 식별자.</summary>
    public long Id { get; init; }

    /// <summary>같이 이동하고 회전하는 전역 item ID 목록.</summary>
    public IReadOnlyList<ulong> MemberItemIds { get; init; } = [];
}

/// <summary>Use Hints 정책을 적용해 실행용 JSON을 준비한 결과.</summary>
public sealed class SwarmInputPreparationResult
{
    /// <summary>JSON 준비 성공 여부.</summary>
    public bool IsSuccess { get; init; }

    /// <summary>native Init에 전달할 JSON. 실패하면 빈 문자열이다.</summary>
    public string Json { get; init; } = string.Empty;

    /// <summary>Use Hints OFF로 제거한 standalone movable hint 수.</summary>
    public int RemovedMovableHintCount { get; init; }

    /// <summary>잠금 또는 PlaceGroup pose 계약 때문에 보존한 hint 수.</summary>
    public int RetainedRequiredHintCount { get; init; }

    /// <summary>
    /// 실행 JSON에 적용한 V2 <c>nesting.group_workers</c>. 입력값을 그대로 사용하거나
    /// legacy V1이면 <see langword="null"/>이다.
    /// </summary>
    public int? AppliedGroupWorkers { get; init; }

    /// <summary>준비 실패 메시지. 성공하면 <see langword="null"/>.</summary>
    public string? ErrorMessage { get; init; }
}
