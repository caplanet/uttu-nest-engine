using System.Globalization;

namespace CAPlanetSwarmSample.WPF.Data;

/// <summary>
/// <c>placed_hint</c>가 지정한 배치를 그대로 놓았을 때의 효율과 겹침입니다.
/// <para>
/// 입력 JSON은 도형을 각자 원점으로 정규화해 담기 때문에 배치 정보가 힌트에만 남습니다.
/// 상용 마커를 변환한 입력이라면 이 값이 곧 <b>상용 배치의 효율</b>이므로, 실행 결과와
/// 나란히 놓고 비교할 기준선이 됩니다.
/// </para>
/// <para>
/// 겹침은 별도로 표시합니다. 원본 마커가 0.025 mm 격자로 양자화돼 있으면 접촉면에
/// 미세한 관통이 남는데, 겹침 허용이 0이면 힌트 배치 전체가 검증에 걸려 버려집니다.
/// 그래서 "몇 쌍이 얼마나 깊게" 겹치는지가 곧 필요한 겹침 허용값이 됩니다.
/// </para>
/// </summary>
public sealed class HintedLayoutMetrics
{
    /// <summary>계산이 가능했는지 여부입니다. 힌트나 도형이 없으면 false입니다.</summary>
    public bool IsAvailable { get; init; }

    /// <summary>계산할 수 없었던 이유입니다. <see cref="IsAvailable"/>가 false일 때만 채워집니다.</summary>
    public string? UnavailableReason { get; init; }

    /// <summary>힌트로 배치된 조각 수입니다.</summary>
    public int PlacedCount { get; init; }

    /// <summary>힌트가 없어 배치에서 빠진 조각 수입니다.</summary>
    public int MissingHintCount { get; init; }

    /// <summary>배치된 조각들의 실면적 합(mm²)입니다.</summary>
    public double PieceAreaMm2 { get; init; }

    /// <summary>배치가 차지한 스트립 길이(mm)입니다. 원점부터 가장 오른쪽 끝까지입니다.</summary>
    public double LengthMm { get; init; }

    /// <summary>스트립 폭(mm). 입력의 <c>strip_height</c>입니다.</summary>
    public double StripHeightMm { get; init; }

    /// <summary>효율(%) = 조각 면적 합 / (길이 × 폭).</summary>
    public double EfficiencyPct { get; init; }

    /// <summary>실제로 관통이 있는 조각 쌍의 수입니다.</summary>
    public int OverlapPairCount { get; init; }

    /// <summary>가장 깊은 관통 깊이(mm)입니다. 겹침 허용값을 정하는 기준입니다.</summary>
    public double MaxPenetrationMm { get; init; }

    /// <summary>스트립 폭([0, strip_height])을 벗어난 조각 수입니다.</summary>
    public int OutOfStripCount { get; init; }

    /// <summary>겹침 검사를 건너뛰었는지 여부입니다. 조각이 너무 많으면 생략합니다.</summary>
    public bool OverlapCheckSkipped { get; init; }

    /// <summary>계산 불가 상태를 만듭니다.</summary>
    /// <param name="reason">사용자에게 보여줄 이유입니다.</param>
    /// <returns>사용할 수 없는 지표입니다.</returns>
    public static HintedLayoutMetrics Unavailable(string reason) => new()
    {
        IsAvailable = false,
        UnavailableReason = reason
    };

    /// <summary>UI 한 줄 요약입니다. 계산 불가면 null을 돌려 표시를 생략하게 합니다.</summary>
    public string? DisplayLine
    {
        get
        {
            if (!IsAvailable)
            {
                return null;
            }

            var text = new System.Text.StringBuilder();
            text.Append($"힌트 배치: 효율 {Format(EfficiencyPct)}%");
            text.Append($" · {PlacedCount}개 · 길이 {Format(LengthMm)} × {Format(StripHeightMm)} mm");

            if (OverlapCheckSkipped)
            {
                text.Append(" · 겹침 검사 생략(조각 과다)");
            }
            else if (OverlapPairCount == 0)
            {
                text.Append(" · 겹침 없음");
            }
            else
            {
                text.Append($" · 겹침 {OverlapPairCount}쌍 (최대 {Format(MaxPenetrationMm)} mm)");
            }

            if (OutOfStripCount > 0)
            {
                text.Append($" · 폭 이탈 {OutOfStripCount}개");
            }

            if (MissingHintCount > 0)
            {
                text.Append($" · 힌트 없음 {MissingHintCount}개");
            }

            return text.ToString();
        }
    }

    /// <summary>효율만 담은 짧은 문자열입니다. 항상 보이는 패널용입니다.</summary>
    public string EfficiencyText => IsAvailable ? $"{Format(EfficiencyPct)} %" : "-";

    /// <summary>겹침 상태를 담은 짧은 문자열입니다.</summary>
    public string OverlapText
    {
        get
        {
            if (!IsAvailable)
            {
                return "-";
            }

            if (OverlapCheckSkipped)
            {
                return "검사 생략";
            }

            return OverlapPairCount == 0
                ? "없음"
                : $"{OverlapPairCount}쌍 · 최대 {Format(MaxPenetrationMm)} mm";
        }
    }

    /// <summary>조각 수와 배치 크기를 담은 짧은 문자열입니다.</summary>
    public string ExtentText => IsAvailable
        ? $"{PlacedCount}개 · {Format(LengthMm)} × {Format(StripHeightMm)} mm"
        : "-";

    private static string Format(double value) =>
        value.ToString("0.###", CultureInfo.InvariantCulture);
}
