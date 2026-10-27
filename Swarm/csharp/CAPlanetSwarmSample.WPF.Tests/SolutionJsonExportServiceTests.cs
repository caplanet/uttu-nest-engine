using System.Text.Json;
using CAPlanetSwarmSample.WPF.Services;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public class SolutionJsonExportServiceTests
{
    private const string Input = """{"name":"marker","strip_height":1524,"items":[{"id":0,"OrgID":9223372036854775807,"OrgTransform":{"angle":30,"origin":[1,2]}},{"id":1,"OrgID":9007199254740993}]}""";
    private const string Result = """{"strip_width":100,"layout":{"placed_items":[{"item_id":1,"transformation":{"rotation":90,"translation":[10,20]}},{"item_id":0,"transformation":{"rotation":0,"translation":[0,0]}}]}}""";

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void UsesInputIdentityNotResultOrderAndPreservesTransforms(bool v2, bool wrapped)
    {
        string output = SolutionJsonExportService.Create(v2 ? "{\"instance\":" + Input + "}" : Input,
            wrapped ? "{\"solution\":" + Result + ",\"loss\":0}" : Result);
        using var document = JsonDocument.Parse(output);
        var root = wrapped ? document.RootElement.GetProperty("solution") : document.RootElement;
        var items = root.GetProperty("layout").GetProperty("placed_items");
        Assert.Equal(9007199254740993L, items[0].GetProperty("OrgID").GetInt64());
        Assert.Equal(long.MaxValue, items[1].GetProperty("OrgID").GetInt64());
        Assert.Equal(90, items[0].GetProperty("transformation").GetProperty("rotation").GetDouble());
        Assert.Equal(30, items[1].GetProperty("OrgTransform").GetProperty("angle").GetDouble());
        Assert.Equal(1524, root.GetProperty("source_strip_height").GetDouble());
        Assert.Equal(1, root.GetProperty("swarm_export_version").GetInt32());
    }

    [Fact]
    public void RejectsMissingAndDuplicateInputIds()
    {
        Assert.Throws<JsonException>(() => SolutionJsonExportService.Create(Input, Result.Replace("item_id\":1", "item_id\":2")));
        Assert.Throws<JsonException>(() => SolutionJsonExportService.Create(Input.Replace("id\":1", "id\":0"), Result));
    }

    [Fact]
    public void OldInputRemainsExportableWithoutInventedOrgId()
    {
        using var doc = JsonDocument.Parse(SolutionJsonExportService.Create("""{"items":[{"id":0},{"id":1}]}""", Result));
        Assert.All(doc.RootElement.GetProperty("layout").GetProperty("placed_items").EnumerateArray(), item => Assert.False(item.TryGetProperty("OrgID", out _)));
    }

    [Fact]
    public void ReportsMissingImportMetadataInsteadOfClaimingCompatibility()
    {
        string partial = SolutionJsonExportService.Create(Input, Result);
        Assert.Contains("1개", SolutionJsonExportService.GetUttuImportWarning(partial));
        string complete = SolutionJsonExportService.Create(Input.Replace("\"OrgID\":9007199254740993", "\"OrgID\":9007199254740993,\"OrgTransform\":{}"), Result);
        Assert.Null(SolutionJsonExportService.GetUttuImportWarning(complete));
        Assert.Null(SolutionJsonExportService.GetUttuImportWarning("{\"solution\":" + complete + "}"));
    }
}
