using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace CAPlanetSwarmSample.WPF.Services;

public sealed record ConfigPresetDocument
{
    public string Format { get; init; } = "swarm-custom-config-preset";
    public int Version { get; init; } = 2;
    public string Name { get; init; } = string.Empty;
    public Dictionary<string, JsonElement> Settings { get; init; } = new(StringComparer.Ordinal);
    public string[] PhaseSequence { get; init; } = [];
}

/// <summary>Stores editable UI values, including inactive options, instead of lossy native structs.</summary>
public static class ConfigPresetCodec
{
    public const int MaximumBytes = 1024 * 1024;
    private static readonly JsonSerializerOptions Options = new()
    {
        WriteIndented = true,
        PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
        Converters = { new JsonStringEnumConverter(allowIntegerValues: false) }
    };

    private static PropertyInfo[] Properties(object settings) => settings.GetType()
        .GetProperties(BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly)
        .Where(p => p.CanRead && p.SetMethod?.IsPublic == true && p.GetIndexParameters().Length == 0
            && Supported(p.PropertyType)).OrderBy(p => p.Name, StringComparer.Ordinal).ToArray();

    private static bool Supported(Type type)
    {
        type = Nullable.GetUnderlyingType(type) ?? type;
        return type.IsEnum || type == typeof(bool) || type == typeof(int) || type == typeof(uint)
            || type == typeof(long) || type == typeof(ulong) || type == typeof(float) || type == typeof(double);
    }

    public static ConfigPresetDocument Capture(string name, object settings, IEnumerable<string> phases)
    {
        var values = new Dictionary<string, JsonElement>(StringComparer.Ordinal);
        foreach (var property in Properties(settings))
        {
            object? value = property.GetValue(settings);
            ValidateValue(value, property.Name);
            values.Add(property.Name, JsonSerializer.SerializeToElement(value, property.PropertyType, Options));
        }
        var doc = new ConfigPresetDocument { Name = name.Trim(), Settings = values, PhaseSequence = phases.ToArray() };
        Validate(doc);
        return doc;
    }

    public static string Write(ConfigPresetDocument doc)
    {
        Validate(doc);
        return JsonSerializer.Serialize(doc, Options);
    }

    public static ConfigPresetDocument Read(string json)
    {
        if (Encoding.UTF8.GetByteCount(json) > MaximumBytes) throw new InvalidDataException("프리셋 JSON은 1MB 이하여야 합니다.");
        using var parsed = JsonDocument.Parse(json);
        RejectDuplicateKeys(parsed.RootElement);
        var result = JsonSerializer.Deserialize<ConfigPresetDocument>(json, Options)
            ?? throw new InvalidDataException("프리셋 JSON이 비어 있습니다.");
        // A plain/native config must not silently deserialize to an empty preset.
        if (!parsed.RootElement.TryGetProperty("format", out _) || !parsed.RootElement.TryGetProperty("version", out _)
            || !parsed.RootElement.TryGetProperty("settings", out _) || !parsed.RootElement.TryGetProperty("phase_sequence", out _))
            throw new InvalidDataException("Custom Config 프리셋 JSON 형식이 아닙니다.");
        Validate(result);
        return result;
    }

    public static ConfigPresetDocument ReadFile(string path)
    {
        if (new FileInfo(path).Length > MaximumBytes) throw new InvalidDataException("프리셋 JSON은 1MB 이하여야 합니다.");
        return Read(File.ReadAllText(path));
    }

    public static ConfigPresetDocument ReadImportFile(string path)
    {
        var doc = ReadFile(path) with { Name = Path.GetFileNameWithoutExtension(path) };
        Validate(doc);
        return doc;
    }

    public static void Apply(ConfigPresetDocument doc, object target)
    {
        Validate(doc);
        var properties = Properties(target).ToDictionary(p => p.Name, StringComparer.Ordinal);
        var assignments = new List<(PropertyInfo Property, object? Value)>();
        foreach (var (name, json) in doc.Settings)
        {
            if (!properties.TryGetValue(name, out var property))
                throw new InvalidDataException($"이 버전에서 지원하지 않는 설정입니다: {name}");
            object? value = json.Deserialize(property.PropertyType, Options);
            if (value is null && Nullable.GetUnderlyingType(property.PropertyType) is null)
                throw new InvalidDataException($"설정값이 비어 있습니다: {name}");
            ValidateValue(value, name);
            assignments.Add((property, value));
        }
        // Decode and validate every field before touching the target. The UI uses a
        // new staging view model and publishes it only after phase validation too.
        foreach (var (property, value) in assignments) property.SetValue(target, value);
    }

    private static void ValidateValue(object? value, string name)
    {
        if (value is float f && !float.IsFinite(f) || value is double d && !double.IsFinite(d)
            || value is Enum e && !Enum.IsDefined(e.GetType(), e))
            throw new InvalidDataException($"유효하지 않은 설정값: {name}");
    }

    private static void Validate(ConfigPresetDocument doc)
    {
        if (doc.Format != "swarm-custom-config-preset" || doc.Version != 2)
            throw new InvalidDataException("지원하지 않는 프리셋 형식 또는 버전입니다.");
        if (string.IsNullOrWhiteSpace(doc.Name) || doc.Name.Length > 120)
            throw new InvalidDataException("프리셋 이름은 1~120자로 입력하세요.");
        if (doc.Settings is null || doc.Settings.Count == 0 || doc.Settings.Count > 1024)
            throw new InvalidDataException("프리셋 설정 목록이 비어 있거나 너무 큽니다.");
        if (doc.PhaseSequence is null || doc.PhaseSequence.Length > 128
            || doc.PhaseSequence.Any(p => p is not ("Explore" or "Compress" or "Polish")))
            throw new InvalidDataException("프리셋 실행 순서가 올바르지 않습니다.");
    }

    private static void RejectDuplicateKeys(JsonElement element)
    {
        if (element.ValueKind == JsonValueKind.Object)
        {
            var names = new HashSet<string>(StringComparer.Ordinal);
            foreach (var property in element.EnumerateObject())
            {
                if (!names.Add(property.Name)) throw new InvalidDataException($"중복 JSON 항목: {property.Name}");
                RejectDuplicateKeys(property.Value);
            }
        }
        else if (element.ValueKind == JsonValueKind.Array)
            foreach (var item in element.EnumerateArray()) RejectDuplicateKeys(item);
    }
}

public sealed record ConfigPresetEntry(Guid Id, ConfigPresetDocument Document)
{
    public string Name => Document.Name;
}
public sealed record ConfigPresetLoadResult(IReadOnlyList<ConfigPresetEntry> Entries, IReadOnlyList<string> Errors);

/// <summary>Each managed GUID file is one preset. Imported source files never become deletion targets.</summary>
public sealed class ConfigPresetStore(string directory)
{
    private static readonly object Gate = new();
    public string DirectoryPath { get; } = Path.GetFullPath(directory);
    private string FilePath(Guid id) => Path.Combine(DirectoryPath, id.ToString("N") + ".json");

    public ConfigPresetLoadResult Load()
    {
        lock (Gate)
        {
            var entries = new List<ConfigPresetEntry>();
            var errors = new List<string>();
            if (!Directory.Exists(DirectoryPath)) return new(entries, errors);
            foreach (string path in Directory.EnumerateFiles(DirectoryPath, "*.json"))
            {
                if (!Guid.TryParseExact(Path.GetFileNameWithoutExtension(path), "N", out var id)) continue;
                try { entries.Add(new(id, ConfigPresetCodec.ReadFile(path))); }
                catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or JsonException or InvalidDataException)
                { errors.Add($"{Path.GetFileName(path)}: {ex.Message}"); }
            }
            return new(entries.OrderBy(e => e.Name, StringComparer.CurrentCultureIgnoreCase).ThenBy(e => e.Id).ToArray(), errors);
        }
    }

    public ConfigPresetEntry Add(ConfigPresetDocument doc, bool rejectDuplicateName = false)
    {
        lock (Gate)
        {
            var names = Load().Entries.Select(e => e.Name).ToHashSet(StringComparer.OrdinalIgnoreCase);
            string baseName = doc.Name.Trim(), name = baseName;
            if (rejectDuplicateName && names.Contains(name))
                throw new InvalidDataException($"'{name}' 이름의 프리셋이 이미 있어 불러올 수 없습니다.");
            for (int suffix = 2; names.Contains(name); suffix++) name = $"{baseName[..Math.Min(baseName.Length, 108)]} ({suffix})";
            var entry = new ConfigPresetEntry(Guid.NewGuid(), doc with { Name = name });
            string json = ConfigPresetCodec.Write(entry.Document);
            Directory.CreateDirectory(DirectoryPath);
            string temporary = FilePath(entry.Id) + ".tmp";
            try
            {
                File.WriteAllText(temporary, json, new UTF8Encoding(false));
                File.Move(temporary, FilePath(entry.Id));
            }
            finally { if (File.Exists(temporary)) File.Delete(temporary); }
            return entry;
        }
    }

    public void Delete(Guid id)
    {
        lock (Gate) File.Delete(FilePath(id));
    }

    /// <summary>Export exactly one saved preset, renaming that preset to the chosen filename.</summary>
    public ConfigPresetEntry Export(Guid id, string path)
    {
        lock (Gate)
        {
            var entries = Load().Entries;
            var current = entries.SingleOrDefault(e => e.Id == id)
                ?? throw new InvalidDataException("선택된 프리셋이 삭제되었습니다. 목록을 새로 선택하세요.");
            string name = Path.GetFileNameWithoutExtension(path);
            if (entries.Any(e => e.Id != id && string.Equals(e.Name, name, StringComparison.OrdinalIgnoreCase)))
                throw new InvalidDataException($"'{name}' 이름의 다른 프리셋이 이미 있습니다.");
            string fullPath = Path.GetFullPath(path);
            if (fullPath.StartsWith(DirectoryPath.TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar,
                StringComparison.OrdinalIgnoreCase))
                throw new InvalidDataException("내부 프리셋 저장 폴더 밖의 파일을 선택하세요.");
            var renamed = current with { Document = current.Document with { Name = name } };
            string json = ConfigPresetCodec.Write(renamed.Document);
            WriteAtomic(fullPath, json);
            WriteAtomic(FilePath(id), json);
            return renamed;
        }
    }

    private static void WriteAtomic(string path, string json)
    {
        string temporary = path + "." + Guid.NewGuid().ToString("N") + ".tmp";
        try
        {
            File.WriteAllText(temporary, json, new UTF8Encoding(false));
            File.Move(temporary, path, overwrite: true);
        }
        finally { if (File.Exists(temporary)) File.Delete(temporary); }
    }
}
