using System.Text.Json;

using CAPlanetSwarmSample.WPF.Data;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public sealed class SwarmInputPlacesTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Parse_OrgID는64비트로읽고_Hint에는순차ID를유지한다(bool v2)
    {
        const string instance = """{"items":[{"id":0,"OrgID":9223372036854775807,"placed_hint":{"item_id":0,"is_freeze":1}},{"id":1,"OrgID":9007199254740993}]}""";
        var input = SwarmInputPlaces.Parse(v2 ? "{\"instance\":" + instance + "}" : instance);
        Assert.Equal(long.MaxValue, input.Items[0].OrgID);
        Assert.Equal(9007199254740993L, input.Items[1].OrgID);
        Assert.Equal(0, Assert.Single(input.ToNativeHints(false)).ItemId);
    }

    [Theory]
    [InlineData("{}")]
    [InlineData("{\"OrgID\":null}")]
    public void Parse_OrgID가없으면새ID를만들지않는다(string fields)
    {
        string item = fields.Replace("{", "{\"id\":0,").Replace(",}", "}");
        Assert.Null(Assert.Single(SwarmInputPlaces.Parse("{\"items\":[" + item + "]}").Items).OrgID);
    }

    [Theory]
    [InlineData("1.5")]
    [InlineData("9223372036854775808")]
    [InlineData("\"123\"")]
    [InlineData("true")]
    public void Parse_잘못된OrgID를거부한다(string value)
    {
        Assert.Throws<JsonException>(() => SwarmInputPlaces.Parse("{\"items\":[{\"id\":0,\"OrgID\":" + value + "}]}"));
    }

    [Fact]
    public void Parse_Hint가없으면_Null이고_NativeHint를만들지않는다()
    {
        SwarmInputPlaces input = SwarmInputPlaces.Parse("""
            { "items": [{ "id": 7 }] }
            """);

        SwarmInputPlaceItem item = Assert.Single(input.Items);
        Assert.Equal(7, item.ItemId);
        Assert.Null(item.PlacedHint);
        Assert.Empty(input.ToNativeHints(useMovableHints: true));
    }

    [Fact]
    public void Parse_CanonicalSnakeCaseHint의모든값을읽는다()
    {
        SwarmInputPlaces input = SwarmInputPlaces.Parse("""
            {
              "items": [{
                "id": 3,
                "placed_hint": {
                  "item_id": 3,
                  "is_freeze": 1,
                  "tr_x": 12.5,
                  "tr_y": 4.25,
                  "rot_angle_in_rad": 1.5,
                  "stripe_interval_y": 8.0,
                  "stripe_interval_x": 9.0
                }
              }]
            }
            """);

        var hint = Assert.Single(input.ToNativeHints(useMovableHints: false));
        Assert.Equal(3, hint.ItemId);
        Assert.Equal(1, hint.IsFreeze);
        Assert.Equal(12.5f, hint.TrX);
        Assert.Equal(4.25f, hint.TrY);
        Assert.Equal(1.5f, hint.RotAngleInRad);
        Assert.Equal(8f, hint.StripeIntervalY);
        Assert.Equal(9f, hint.StripeIntervalX);
    }

    [Fact]
    public void Parse_LegacyCamelCaseHint와PlacedHint이름을읽는다()
    {
        SwarmInputPlaces input = SwarmInputPlaces.Parse("""
            {
              "items": [{
                "id": 4,
                "placedHint": {
                  "itemId": 4,
                  "isFreeze": true,
                  "trX": 1.25,
                  "trY": 2.5,
                  "rotAngleInRad": 0.75,
                  "stripeIntervalY": 3.0,
                  "stripeIntervalX": 6.0
                }
              }]
            }
            """);

        var hint = Assert.Single(input.ToNativeHints(useMovableHints: true));
        Assert.Equal(4, hint.ItemId);
        Assert.Equal(1, hint.IsFreeze);
        Assert.Equal(1.25f, hint.TrX);
        Assert.Equal(2.5f, hint.TrY);
        Assert.Equal(0.75f, hint.RotAngleInRad);
        Assert.Equal(3f, hint.StripeIntervalY);
        Assert.Equal(6f, hint.StripeIntervalX);
    }

    [Fact]
    public void ToNativeHints_UseHintsOff에서도Freeze는유지한다()
    {
        SwarmInputPlaces input = SwarmInputPlaces.Parse("""
            {
              "items": [
                { "id": 1, "placed_hint": { "item_id": 1, "is_freeze": 0, "tr_x": 10 } },
                { "id": 2, "placed_hint": { "item_id": 2, "is_freeze": 1, "tr_x": 20 } },
                { "id": 3 }
              ]
            }
            """);

        var withoutMovable = Assert.Single(input.ToNativeHints(useMovableHints: false));
        Assert.Equal(2, withoutMovable.ItemId);
        Assert.Equal(2, input.ToNativeHints(useMovableHints: true).Length);
    }

    [Fact]
    public void Parse_V2Envelope의InstanceItems를읽고_HintItemId가없으면ItemId를사용한다()
    {
        SwarmInputPlaces input = SwarmInputPlaces.Parse("""
            {
              "schema_version": 2,
              "instance": {
                "items": [{
                  "id": 42,
                  "placed_hint": { "is_freeze": 1, "tr_x": 5.0 }
                }]
              }
            }
            """);

        var hint = Assert.Single(input.ToNativeHints(useMovableHints: false));
        Assert.Equal(42, hint.ItemId);
        Assert.Equal(5f, hint.TrX);
    }

    [Fact]
    public void Parse_PlacedHint가Object가아니면명확히실패한다()
    {
        JsonException exception = Assert.Throws<JsonException>(() =>
            SwarmInputPlaces.Parse("""
                { "items": [{ "id": 1, "placed_hint": 123 }] }
                """));

        Assert.Contains("placed_hint는 객체", exception.Message);
    }

    [Fact]
    public void Parse_HintItemId가외부ItemId와다르면명확히실패한다()
    {
        JsonException exception = Assert.Throws<JsonException>(() =>
            SwarmInputPlaces.Parse("""
                {
                  "items": [{
                    "id": 1,
                    "placed_hint": { "item_id": 2, "is_freeze": 1 }
                  }]
                }
                """));

        Assert.Contains("외부 item.id와 다릅니다", exception.Message);
    }
}
