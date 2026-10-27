using System.Runtime.InteropServices;
using Uttu.NestingV2;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public sealed class RawSolutionCallbackTests
{
    [Fact]
    public void BridgeCopiesUtf8PayloadBeforeNativeBufferIsFreed()
    {
        const string expected = "{\"solution\":{\"name\":\"한글 callback\"},\"loss\":0.25}";
        string? copied = null;
        var bridge = SwarmStepOptimizer.CreateRawCallbackBridge(
            (reportType, stripWidth, density, combinedJson) =>
            {
                Assert.Equal(ReportType.CmprFeas, reportType);
                Assert.Equal(123.5f, stripWidth);
                Assert.Equal(0.75f, density);
                copied = combinedJson;
            });

        IntPtr nativeUtf8 = Marshal.StringToCoTaskMemUTF8(expected);
        try
        {
            bridge((int)ReportType.CmprFeas, 123.5f, 0.75f, nativeUtf8, IntPtr.Zero);
        }
        finally
        {
            Marshal.FreeCoTaskMem(nativeUtf8);
        }

        Assert.Equal(expected, copied);
    }

    [Fact]
    public void BridgeMapsNullPayloadToEmptyString()
    {
        string? copied = null;
        var bridge = SwarmStepOptimizer.CreateRawCallbackBridge(
            (_, _, _, combinedJson) => copied = combinedJson);

        bridge((int)ReportType.ExplFeas, 10f, 0.5f, IntPtr.Zero, IntPtr.Zero);

        Assert.Equal(string.Empty, copied);
    }

    [Fact]
    public void BridgeNeverLetsManagedConsumerExceptionCrossAbiBoundary()
    {
        var bridge = SwarmStepOptimizer.CreateRawCallbackBridge(
            (_, _, _, _) => throw new InvalidOperationException("consumer failure"));

        Exception? escaped = Record.Exception(
            () => bridge((int)ReportType.Final, 10f, 1f, IntPtr.Zero, IntPtr.Zero));

        Assert.Null(escaped);
    }

    [Fact]
    public void CombinedJsonParsingRemainsAvailableForCompatibilityCallback()
    {
        const string combined = """
            {
              "solution": { "strip_width": 42.0, "layout": { "placed_items": [] } },
              "loss": 1.5,
              "collision_status": [ { "item_id": 7, "collision": true } ]
            }
            """;

        SolutionEventArgs result = SolutionEventArgs.FromCombinedJson(
            ReportType.ExplImproving,
            42f,
            0.9f,
            combined);

        Assert.Equal(ReportType.ExplImproving, result.ReportType);
        Assert.Equal(1.5f, result.Loss);
        Assert.True(result.HasLoss);
        Assert.NotNull(result.SolutionJson);
        Assert.Single(result.CollisionStatus);
        Assert.True(result.HasCollisionStatus);
        Assert.Equal(7, result.CollisionStatus[0].ItemID);
        Assert.True(result.CollisionStatus[0].IsCollision);
    }

    [Fact]
    public void CombinedJsonDistinguishesDisabledDiagnosticsFromZeroValues()
    {
        const string combined = """
            {
              "solution": { "strip_width": 42.0, "layout": { "placed_items": [] } }
            }
            """;

        SolutionEventArgs result = SolutionEventArgs.FromCombinedJson(
            ReportType.ExplFeas,
            42f,
            0.9f,
            combined);

        Assert.NotNull(result.SolutionJson);
        Assert.False(result.HasLoss);
        Assert.Equal(0f, result.Loss);
        Assert.False(result.HasCollisionStatus);
        Assert.Empty(result.CollisionStatus);
    }

    [Fact]
    public void EmptyPayloadDoesNotAttemptJsonParsing()
    {
        SolutionEventArgs result = SolutionEventArgs.FromCombinedJson(
            ReportType.ExplFeas,
            10f,
            0.5f,
            string.Empty);

        Assert.Null(result.SolutionJson);
        Assert.Empty(result.ParsedPlacements);
        Assert.False(result.HasLoss);
        Assert.False(result.HasCollisionStatus);
    }

    [Fact]
    public void CombinedJsonExtractsRenderPlacementsDuringItsSingleParse()
    {
        const string combined = """
            {
              "solution": {
                "strip_width": 42.0,
                "layout": {
                  "placed_items": [
                    {"item_id":7,"transformation":{"rotation":90,"translation":[12,34]}}
                  ]
                }
              }
            }
            """;

        SolutionEventArgs result = SolutionEventArgs.FromCombinedJson(
            ReportType.ExplFeas, 42f, 0.9f, combined);

        Assert.Equal(42f, result.ParsedStripWidth);
        SolutionEventArgs.ParsedPlacement placement = Assert.Single(result.ParsedPlacements);
        Assert.Equal(7, placement.ItemId);
        Assert.Equal(90f, placement.RotationDegrees);
        Assert.Equal(12f, placement.TranslationX);
        Assert.Equal(34f, placement.TranslationY);
    }
}
