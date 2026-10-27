using System.Text.Json;

using CAPlanetSwarmSample.WPF.Data;
using CAPlanetSwarmSample.WPF.Services;

using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public sealed class HintedLayoutAnalyzerTests
{
    /// <summary>축정렬 사각형 하나를 아이템 JSON 조각으로 만듭니다.</summary>
    private static string Rect(int id, double w, double h, double trX, double trY) =>
        $$"""
          {
            "id": {{id}},
            "shape": { "type": "simple_polygon", "data": [[0,0],[{{w}},0],[{{w}},{{h}}],[0,{{h}}]] },
            "placed_hint": { "item_id": {{id}}, "is_freeze": 0, "tr_x": {{trX}}, "tr_y": {{trY}}, "rot_angle_in_rad": 0.0 }
          }
          """;

    private static HintedLayoutMetrics Analyze(string itemsJson, double stripHeight = 100)
    {
        using JsonDocument document = JsonDocument.Parse(
            $$"""{ "items": [{{itemsJson}}], "strip_height": {{stripHeight}} }""");
        return HintedLayoutAnalyzer.Analyze(document.RootElement);
    }

    [Fact]
    public void 힌트가없으면_사용불가다()
    {
        using JsonDocument document = JsonDocument.Parse(
            """{ "items": [{ "id": 0, "shape": { "type": "simple_polygon", "data": [[0,0],[1,0],[1,1]] } }], "strip_height": 10 }""");

        HintedLayoutMetrics metrics = HintedLayoutAnalyzer.Analyze(document.RootElement);

        Assert.False(metrics.IsAvailable);
        Assert.Null(metrics.DisplayLine);
        Assert.Equal("placed_hint가 없습니다.", metrics.UnavailableReason);
    }

    [Fact]
    public void 겹치지않는두조각의_효율을_계산한다()
    {
        // 20x50 두 개를 나란히: 면적 2000, 길이 40, 폭 100 -> 2000/4000 = 50%
        HintedLayoutMetrics metrics = Analyze(
            $"{Rect(0, 20, 50, 0, 0)},{Rect(1, 20, 50, 20, 0)}");

        Assert.True(metrics.IsAvailable);
        Assert.Equal(2, metrics.PlacedCount);
        Assert.Equal(2000, metrics.PieceAreaMm2, 6);
        Assert.Equal(40, metrics.LengthMm, 6);
        Assert.Equal(50.0, metrics.EfficiencyPct, 6);
        Assert.Equal(0, metrics.OverlapPairCount);
        Assert.Equal(0, metrics.OutOfStripCount);
    }

    [Fact]
    public void 맞닿기만한_두조각은_겹침이아니다()
    {
        // x=20 에서 변이 정확히 맞닿는다. 접촉은 겹침으로 세지 않는다.
        HintedLayoutMetrics metrics = Analyze(
            $"{Rect(0, 20, 50, 0, 0)},{Rect(1, 20, 50, 20, 0)}");

        Assert.Equal(0, metrics.OverlapPairCount);
        Assert.Equal(0.0, metrics.MaxPenetrationMm);
    }

    [Fact]
    public void 관통한_두조각의_깊이를_잰다()
    {
        // 두 번째를 2mm 왼쪽으로 밀어 2mm 관통시킨다.
        HintedLayoutMetrics metrics = Analyze(
            $"{Rect(0, 20, 50, 0, 0)},{Rect(1, 20, 50, 18, 0)}");

        Assert.Equal(1, metrics.OverlapPairCount);
        Assert.Equal(2.0, metrics.MaxPenetrationMm, 6);
    }

    [Fact]
    public void 십자로_교차하면_정점이안에없어도_겹침이다()
    {
        // 가로 막대와 세로 막대가 십자로 겹친다. 어느 정점도 상대 내부에 없다.
        string horizontal = Rect(0, 60, 10, 0, 25);
        string vertical = Rect(1, 10, 60, 25, 0);
        HintedLayoutMetrics metrics = Analyze($"{horizontal},{vertical}");

        Assert.Equal(1, metrics.OverlapPairCount);
    }

    [Fact]
    public void 스트립폭을_벗어난_조각을_센다()
    {
        HintedLayoutMetrics metrics = Analyze(Rect(0, 20, 50, 0, 60), stripHeight: 100);

        Assert.Equal(1, metrics.OutOfStripCount);
        Assert.Contains("폭 이탈 1개", metrics.DisplayLine);
    }

    [Fact]
    public void 회전힌트는_배치복원이안되므로_제외한다()
    {
        string rotated = """
            {
              "id": 1,
              "shape": { "type": "simple_polygon", "data": [[0,0],[20,0],[20,50],[0,50]] },
              "placed_hint": { "item_id": 1, "is_freeze": 0, "tr_x": 30, "tr_y": 0, "rot_angle_in_rad": 1.5707963 }
            }
            """;
        HintedLayoutMetrics metrics = Analyze($"{Rect(0, 20, 50, 0, 0)},{rotated}");

        Assert.Equal(1, metrics.PlacedCount);
        Assert.Equal(1, metrics.MissingHintCount);
        Assert.Contains("힌트 없음 1개", metrics.DisplayLine);
    }

    [Fact]
    public void CamelCase_힌트도_읽는다()
    {
        string camel = """
            {
              "id": 0,
              "shape": { "type": "simple_polygon", "data": [[0,0],[20,0],[20,50],[0,50]] },
              "placedHint": { "itemId": 0, "isFreeze": 0, "trX": 10, "trY": 0, "rotAngleInRad": 0.0 }
            }
            """;
        HintedLayoutMetrics metrics = Analyze(camel);

        Assert.True(metrics.IsAvailable);
        Assert.Equal(30, metrics.LengthMm, 6);
    }

    [Fact]
    public void 표시줄에_효율과_겹침이_들어간다()
    {
        HintedLayoutMetrics metrics = Analyze(
            $"{Rect(0, 20, 50, 0, 0)},{Rect(1, 20, 50, 18, 0)}");

        string line = Assert.IsType<string>(metrics.DisplayLine);
        Assert.StartsWith("힌트 배치: 효율 ", line);
        Assert.Contains("겹침 1쌍", line);
        Assert.Contains("mm", line);
    }

    [Fact]
    public void 항상보이는패널용_짧은문자열을_만든다()
    {
        HintedLayoutMetrics metrics = Analyze(
            $"{Rect(0, 20, 50, 0, 0)},{Rect(1, 20, 50, 18, 0)}");

        Assert.Equal("52.632 %", metrics.EfficiencyText);
        Assert.Equal("1쌍 · 최대 2 mm", metrics.OverlapText);
        Assert.Equal("2개 · 38 × 100 mm", metrics.ExtentText);
    }

    [Fact]
    public void 겹침이없으면_없음으로_표시한다()
    {
        HintedLayoutMetrics metrics = Analyze(
            $"{Rect(0, 20, 50, 0, 0)},{Rect(1, 20, 50, 20, 0)}");

        Assert.Equal("없음", metrics.OverlapText);
    }

    [Fact]
    public void 사용불가면_짧은문자열이_대시다()
    {
        HintedLayoutMetrics metrics = HintedLayoutMetrics.Unavailable("테스트");

        Assert.Equal("-", metrics.EfficiencyText);
        Assert.Equal("-", metrics.OverlapText);
        Assert.Equal("-", metrics.ExtentText);
    }

    [Fact]
    public void 요약_표시문에_힌트배치줄이_붙는다()
    {
        string json =
            $$"""{ "items": [{{Rect(0, 20, 50, 0, 0)}}], "strip_height": 100 }""";

        SwarmInputSummary summary = SwarmInputInspector.Parse(json);

        Assert.True(summary.IsValid);
        Assert.Equal(1, summary.SchemaVersion);
        Assert.True(summary.HintedLayout.IsAvailable);
        Assert.Contains("힌트 배치: 효율 ", summary.DisplayText);
    }
}
