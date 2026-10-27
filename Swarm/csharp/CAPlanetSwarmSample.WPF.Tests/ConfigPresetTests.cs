using System.Text.Json;
using CAPlanetSwarmSample.WPF.Services;
using Xunit;

namespace CAPlanetSwarmSample.WPF.Tests;

public class ConfigPresetTests
{
    [Fact]
    public void ExportOnlySelectedSavedPresetAndUseFilenameForNames()
    {
        string root = Path.Combine(Path.GetTempPath(), "swarm-preset-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(root);
        try
        {
            var store = new ConfigPresetStore(Path.Combine(root, "store"));
            var first = store.Add(ConfigPresetCodec.Capture("First", new Settings { Workers = 3 }, ["Explore"]));
            var other = store.Add(ConfigPresetCodec.Capture("Other", new Settings { Workers = 7 }, ["Polish"]));
            string file = Path.Combine(root, "Custom 2026.json");
            var saved = store.Export(first.Id, file);
            var doc = ConfigPresetCodec.ReadFile(file);
            Assert.Equal("Custom 2026", saved.Name);
            Assert.Equal(saved.Document.Name, doc.Name);
            Assert.Equal(3, doc.Settings["Workers"].GetInt32());
            Assert.Equal("Other", store.Load().Entries.Single(e => e.Id == other.Id).Name);
            Assert.Equal(2, store.Load().Entries.Count);
            Assert.Throws<InvalidDataException>(() => store.Add(ConfigPresetCodec.ReadImportFile(file), rejectDuplicateName: true));
            Assert.Throws<InvalidDataException>(() => store.Export(first.Id, Path.Combine(root, "OTHER.json")));
            Assert.False(File.Exists(Path.Combine(root, "OTHER.json")));
            string renamedFile = Path.Combine(root, "다른 파일 이름.json");
            File.Copy(file, renamedFile);
            var imported = store.Add(ConfigPresetCodec.ReadImportFile(renamedFile), rejectDuplicateName: true);
            Assert.Equal("다른 파일 이름", imported.Name);
            Assert.Equal(3, imported.Document.Settings["Workers"].GetInt32());
            Assert.Equal(3, store.Load().Entries.Count);
        }
        finally
        {
            foreach (string path in Directory.EnumerateFiles(root, "*", SearchOption.AllDirectories)) File.Delete(path);
            foreach (string path in Directory.EnumerateDirectories(root)) Directory.Delete(path);
            Directory.Delete(root);
        }
    }

    public enum Mode { First, Second }
    public class Settings
    {
        public bool Enabled { get; set; }
        public double DisabledOptionValue { get; set; } = 1.25;
        public ulong Seed { get; set; } = ulong.MaxValue;
        public Mode Strategy { get; set; } = Mode.Second;
        public int Workers { get; set; } = 3;
        public string Computed => "not a setting";
        public object Command { get; set; } = new();
    }

    [Fact]
    public void RoundTripPreservesDisabledValuesEnumsSeedAndRepeatedPhaseOrder()
    {
        var source = new Settings();
        var doc = ConfigPresetCodec.Capture("시험", source, ["Explore", "Polish", "Explore"]);
        var loaded = ConfigPresetCodec.Read(ConfigPresetCodec.Write(doc));
        var target = new Settings { DisabledOptionValue = 99, Seed = 1, Strategy = Mode.First, Workers = 8 };
        ConfigPresetCodec.Apply(loaded, target);
        Assert.False(target.Enabled);
        Assert.Equal(1.25, target.DisabledOptionValue);
        Assert.Equal(ulong.MaxValue, target.Seed);
        Assert.Equal(Mode.Second, target.Strategy);
        Assert.Equal(3, target.Workers);
        Assert.Equal(new[] { "Explore", "Polish", "Explore" }, loaded.PhaseSequence);
        Assert.DoesNotContain("Command", loaded.Settings.Keys);
        Assert.DoesNotContain("Computed", loaded.Settings.Keys);
    }

    [Theory]
    [InlineData("{\"format\":\"other\",\"version\":1}")]
    [InlineData("{\"format\":\"swarm-custom-config-preset\",\"version\":2}")]
    [InlineData("{\"format\":\"swarm-custom-config-preset\",\"version\":1,\"version\":1}")]
    [InlineData("[]")]
    public void RejectsUnrecognizedOrAmbiguousDocuments(string json) =>
        Assert.ThrowsAny<Exception>(() => ConfigPresetCodec.Read(json));

    [Fact]
    public void InvalidPropertyDoesNotPartiallyApply()
    {
        var doc = ConfigPresetCodec.Capture("bad", new Settings { Workers = 8 }, ["Explore"]);
        doc.Settings["Strategy"] = JsonSerializer.SerializeToElement("NotAMode");
        var target = new Settings();
        Assert.ThrowsAny<Exception>(() => ConfigPresetCodec.Apply(doc, target));
        Assert.Equal(3, target.Workers);
    }

    [Fact]
    public void UnknownSettingsAndNonfiniteValuesAreRejected()
    {
        Assert.ThrowsAny<Exception>(() => ConfigPresetCodec.Capture("bad", new Settings { DisabledOptionValue = double.NaN }, ["Explore"]));
        var doc = ConfigPresetCodec.Capture("bad", new Settings(), ["Explore"]);
        doc.Settings["Typo"] = JsonSerializer.SerializeToElement(3);
        Assert.ThrowsAny<Exception>(() => ConfigPresetCodec.Apply(doc, new Settings()));
        Assert.ThrowsAny<Exception>(() => ConfigPresetCodec.Capture("bad", new Settings(), ["Unknown"]));
    }

    [Fact]
    public void StorePersistsAddsDistinctNamesAndDeletesOnlyOwnedCopy()
    {
        string directory = Path.Combine(Path.GetTempPath(), "swarm-preset-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(directory);
        try
        {
            var store = new ConfigPresetStore(directory);
            var doc = ConfigPresetCodec.Capture("원본", new Settings(), ["Explore"]);
            string external = Path.Combine(directory, "import-source.json");
            File.WriteAllText(external, ConfigPresetCodec.Write(doc));
            var first = store.Add(doc);
            var second = store.Add(ConfigPresetCodec.Read(File.ReadAllText(external)));
            Assert.NotEqual(first.Id, second.Id);
            Assert.Equal("원본 (2)", second.Name);
            Assert.Equal(2, new ConfigPresetStore(directory).Load().Entries.Count);
            store.Delete(first.Id);
            Assert.Single(store.Load().Entries);
            Assert.True(File.Exists(external));
            File.WriteAllText(Path.Combine(directory, Guid.NewGuid().ToString("N") + ".json"), "broken");
            var result = store.Load();
            Assert.Single(result.Entries);
            Assert.Single(result.Errors);
        }
        finally
        {
            // Only the fresh GUID-named test directory owns these flat files.
            foreach (string file in Directory.EnumerateFiles(directory)) File.Delete(file);
            Directory.Delete(directory);
        }
    }
}
