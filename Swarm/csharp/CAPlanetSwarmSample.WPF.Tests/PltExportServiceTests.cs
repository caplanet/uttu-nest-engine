using CAPlanetSwarmSample.WPF.Services;
using System.Globalization;
using System.Text.Json;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public class PltExportServiceTests
{
    private const string Input = """{"strip_height":100,"items":[{"id":7,"shape":{"type":"simple_polygon","data":[[10,20],[14,20],[14,23],[10,23]]}}]}""";
    private const string Pose = """{"item_id":7,"transformation":{"rotation":90,"translation":[50,60]}}""";
    private static string Solution(string poses = Pose) => "{\"strip_width\":120,\"layout\":{\"placed_items\":[" + poses + "]}}";

    [Fact]
    public void UsesOriginalCoordinatesDegreesAndFortyUnitsPerMm()
    {
        string plt = PltExportService.Create(Input, Solution());
        Assert.StartsWith("NE120,100;\r\nIN;", plt);
        Assert.Contains("PU1200,2800;\r\nPD1200,2960;\r\nPD1080,2960;\r\nPD1080,2800;\r\nPD1200,2800;\r\nPU;", plt);
        Assert.Contains("PU0,0;\r\nPD4800,0;\r\nPD4800,4000;\r\nPD0,4000;\r\nPD0,0;", plt);
        Assert.EndsWith("PU;\r\nSP0;\r\n", plt);
    }

    [Fact]
    public void SupportsV2WrappedSolutionsAndRepeatedItemDemand()
    {
        string plt = PltExportService.Create("{\"instance\":" + Input + "}", "{\"solution\":" + Solution(Pose + "," + Pose) + "}");
        Assert.Equal(2, plt.Split("PU1200,2800;").Length - 1);
    }

    [Fact]
    public void DoesNotClipNegativeFrozenCoordinatesOrFlipTheYAxis()
    {
        const string input = """{"strip_height":10,"items":[{"id":7,"shape":{"type":"rectangle","data":{"x_min":0,"y_min":0,"width":3,"height":2}}}]}""";
        string pose = Pose.Replace("90", "0").Replace("[50,60]", "[-5,-2]");
        string plt = PltExportService.Create(input, Solution(pose));
        Assert.Contains("PU-200,-80;\r\nPD-80,-80;\r\nPD-80,0;", plt);
    }

    [Fact]
    public void OutputIsCultureIndependentAndReferenceHeaderCompatible()
    {
        var before = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");
            string plt = PltExportService.Create(Input, Solution());
            Assert.Contains("PW0.350,8;", plt);
            Assert.Contains("VS32,8;", plt);
            Assert.DoesNotContain("PW0,350", plt);
        }
        finally { CultureInfo.CurrentCulture = before; }
    }

    [Fact]
    public void RejectsMissingGeometryEmptyOutputAndOverflow()
    {
        Assert.Throws<JsonException>(() => PltExportService.Create(Input, Solution(Pose.Replace("item_id\":7", "item_id\":9"))));
        Assert.Throws<JsonException>(() => PltExportService.Create(Input, Solution("")));
        Assert.Throws<JsonException>(() => PltExportService.Create(Input, Solution(Pose.Replace("[50,60]", "[1e100,60]"))));
    }

    [Fact]
    public void InvalidExportLeavesNoFileAndExistingFileIsPreserved()
    {
        string directory = Path.Combine(Path.GetTempPath(), "swarm-plt-" + Guid.NewGuid().ToString("N"));
        string path = Path.Combine(directory, "test.plt");
        try
        {
            Assert.Throws<JsonException>(() => PltExportService.Save(Input, Solution(""), path));
            Assert.False(File.Exists(path));
            PltExportService.Save(Input, Solution(), path);
            string saved = File.ReadAllText(path);
            Assert.Throws<IOException>(() => PltExportService.Save(Input, Solution(), path));
            Assert.Equal(saved, File.ReadAllText(path));
            Assert.Single(Directory.GetFiles(directory));
        }
        finally
        {
            if (File.Exists(path)) File.Delete(path);
            if (Directory.Exists(directory)) Directory.Delete(directory);
        }
    }
}
