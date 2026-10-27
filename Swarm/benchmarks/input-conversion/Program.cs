using System.Diagnostics;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Unicode;

// Usage: dotnet run -c Release -- <absolute DLL> <output.json> <fixture.json> ...
// Start point: the SAME float-coordinate objects, already loaded, with demand expanded.
// End point: owned native ValidatedInput. No geometry import, LBF, or solver phase runs.
if (args.Length < 3) throw new ArgumentException("DLL output.json fixture.json [fixture.json ...]");
var dll = Path.GetFullPath(args[0]);
NativeLibrary.SetDllImportResolver(Assembly.GetExecutingAssembly(), (name, _, _) =>
    name == "cpswcore" ? NativeLibrary.Load(dll) : 0);
if (IntPtr.Size != 8 || Marshal.SizeOf<Hint>() != 40 || Marshal.SizeOf<Item>() != 144 || Marshal.SizeOf<Input>() != 104)
    throw new InvalidOperationException("This benchmark's interop layout requires the documented x64 ABI.");
var pretty = Options(true);
var compact = Options(false);
if (Environment.GetEnvironmentVariable("SWARM_BENCH_MODE") == "init")
{
    foreach (var file in args.Skip(2)) InitTiming.Run(dll, args[1], file, Load(file), pretty);
    return;
}
VerifyInterop(compact);
var results = new List<object>();
const int warmups = 12, samples = 81;
foreach (var file in args.Skip(2))
{
    var model = Load(file);
    string prettyJson = JsonSerializer.Serialize(model, pretty);
    string compactJson = JsonSerializer.Serialize(model, compact);
    using var packed = new Packed(model);
    using var prettyUtf8 = new PinnedBytes(Encoding.UTF8.GetBytes(prettyJson + '\0'));
    using var compactUtf8 = new PinnedBytes(Encoding.UTF8.GetBytes(compactJson + '\0'));
    var cases = new (string Name, Func<(nint Handle, IDisposable? Buffers)> Run)[] {
        ("native_json_pretty", () => (Native.InputFromUtf8(prettyUtf8.Address), null)),
        ("native_json_compact", () => (Native.InputFromUtf8(compactUtf8.Address), null)),
        ("native_struct", () => (Native.swarm_input_from_struct(in packed.Input), null)),
        ("csharp_json_pretty", () => (Native.swarm_input_from_json(JsonSerializer.Serialize(model, pretty)), null)),
        ("csharp_json_compact", () => (Native.swarm_input_from_json(JsonSerializer.Serialize(model, compact)), null)),
        ("csharp_struct", () => { var p = new Packed(model); return (Native.swarm_input_from_struct(in p.Input), p); }),
    };
    var times = cases.Select(_ => new List<double>()).ToArray();
    var allocations = cases.Select(_ => new List<long>()).ToArray();
    // Warm every path including JIT, serializer metadata and native allocations.
    // Rotate order between rounds; exclude freeing the completed native input on BOTH paths.
    for (int round = -warmups; round < samples; round++)
    {
        for (int offset = 0; offset < cases.Length; offset++)
        {
            int index = ((round + warmups) + offset) % cases.Length;
            long allocated = GC.GetAllocatedBytesForCurrentThread();
            long start = Stopwatch.GetTimestamp();
            var value = cases[index].Run();
            double milliseconds = Stopwatch.GetElapsedTime(start).TotalMilliseconds;
            long bytes = GC.GetAllocatedBytesForCurrentThread() - allocated;
            if (value.Handle == 0) throw new InvalidOperationException($"{cases[index].Name}: native error {Native.swarm_get_last_error_code()}");
            Native.swarm_input_free(value.Handle);
            value.Buffers?.Dispose();
            if (round >= 0) { times[index].Add(milliseconds); allocations[index].Add(bytes); }
        }
    }
    var rows = cases.Select((c, i) => new {
        c.Name, MedianMs = Percentile(times[i], 0.5), P95Ms = Percentile(times[i], 0.95),
        MeanMs = times[i].Average(), ManagedBytesMedian = Percentile(allocations[i].Select(x => (double)x).ToList(), 0.5),
        SamplesMs = times[i], ManagedBytes = allocations[i]
    }).ToArray();
    var entry = new {
        File = Path.GetFullPath(file), SourceSha256 = Hash(file),
        Items = model.Instance.Items.Length,
        Vertices = model.Instance.Items.Sum(x => x.Shape.Data.Length),
        PrettyJsonBytes = Encoding.UTF8.GetByteCount(prettyJson), CompactJsonBytes = Encoding.UTF8.GetByteCount(compactJson),
        Rows = rows
    };
    results.Add(entry);
    Console.WriteLine($"{Path.GetFileName(file)}: {entry.Items} items / {entry.Vertices} vertices / {entry.PrettyJsonBytes:N0} JSON bytes");
    foreach (var row in rows) Console.WriteLine($"  {row.Name,-22} median {row.MedianMs,10:F4} ms  p95 {row.P95Ms,10:F4} ms  managed {row.ManagedBytesMedian,12:N0} B");
}
var report = new {
    Utc = DateTimeOffset.UtcNow, Dll = dll, DllSha256 = Hash(dll),
    OS = RuntimeInformation.OSDescription, Runtime = RuntimeInformation.FrameworkDescription,
    Architecture = RuntimeInformation.ProcessArchitecture.ToString(), ProcessorCount = Environment.ProcessorCount,
    TieredCompilation = Environment.GetEnvironmentVariable("DOTNET_TieredCompilation") ?? "default",
    InteropSmokePassed = true,
    StopwatchFrequency = Stopwatch.Frequency, Warmups = warmups, Samples = samples,
    Boundary = "Same C# float-coordinate objects -> owned native ValidatedInput; native_* starts at prepared UTF-8/descriptors. Input destruction excluded in all cases. No geometry import/solver.",
    Results = results
};
File.WriteAllText(args[1], JsonSerializer.Serialize(report, new JsonSerializerOptions { WriteIndented = true }));

static JsonSerializerOptions Options(bool indented) => new() {
    WriteIndented = indented, PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
    Encoder = JavaScriptEncoder.Create(UnicodeRanges.All)
};
static string Hash(string file) => Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(file)));
static double Percentile(List<double> values, double p) {
    var sorted = values.Order().ToArray();
    return sorted[(int)Math.Ceiling(p * sorted.Length) - 1];
}
static void VerifyInterop(JsonSerializerOptions options) {
    var model = new Envelope { Instance = new Instance { Name = "interop-한글", StripHeight = 50,
        Items = [new PolygonItem { Id = 0, AllowedOrientations = [0],
            Shape = new Shape { Data = [[0,0], [10,0], [10,8], [0,8]] } }] } };
    using var packed = new Packed(model);
    using var viaJson = Solve(Native.swarm_input_from_json(JsonSerializer.Serialize(model, options)));
    using var viaStruct = Solve(Native.swarm_input_from_struct(in packed.Input));
    var jsonSolution = viaJson.RootElement;
    var structSolution = viaStruct.RootElement;
    foreach (string property in new[] { "strip_width", "density", "layout" })
        if (!JsonElement.DeepEquals(jsonSolution.GetProperty(property), structSolution.GetProperty(property)))
            throw new InvalidOperationException($"C# interop parity failed: {property}");
    if (structSolution.GetProperty("layout").GetProperty("placed_items").GetArrayLength() != 1)
        throw new InvalidOperationException("C# interop did not place the input polygon.");
    Console.WriteLine("C# interop smoke: JSON/direct initial layout, width and density match.");
}
static JsonDocument Solve(nint input) {
    if (input == 0) throw new InvalidOperationException($"prepare error {Native.swarm_get_last_error_code()}");
    if (Native.swarm_get_abi_info(out var abi) != 0 || abi.Version != 20 || abi.ConfigSize < 8)
    {
        Native.swarm_input_free(input);
        throw new InvalidOperationException("Smoke check requires config ABI 20.");
    }
    nint config = Marshal.AllocHGlobal(checked((int)abi.ConfigSize));
    nint handle;
    try {
        Native.swarm_get_default_config(config);
        // ABI 20's first field is uint64_t rng_seed. Keep both smoke solves deterministic.
        Marshal.WriteInt64(config, 0, 1);
        handle = Native.swarm_init_from_input(input, config); // consumes input even on failure
    } finally { Marshal.FreeHGlobal(config); }
    if (handle == 0) throw new InvalidOperationException($"init error {Native.swarm_get_last_error_code()}");
    nint text = 0;
    try {
        text = Native.swarm_get_solution_json(handle);
        if (text == 0) throw new InvalidOperationException($"export error {Native.swarm_get_last_error_code()}");
        return JsonDocument.Parse(Marshal.PtrToStringUTF8(text)!);
    } finally {
        if (text != 0) Native.swarm_free_string(text);
        Native.swarm_free(handle);
    }
}
static Envelope Load(string file) {
    using var doc = JsonDocument.Parse(File.ReadAllText(file));
    var root = doc.RootElement;
    var items = new List<PolygonItem>();
    foreach (var item in root.GetProperty("items").EnumerateArray()) {
        var shape = item.GetProperty("shape");
        if (shape.GetProperty("type").GetString() != "simple_polygon")
            throw new InvalidOperationException("This benchmark fixture loader accepts simple_polygon only.");
        var points = shape.GetProperty("data").EnumerateArray()
            .Select(p => new[] { p[0].GetSingle(), p[1].GetSingle() }).ToArray();
        float[]? angles = item.TryGetProperty("allowed_orientations", out var a) && a.ValueKind != JsonValueKind.Null
            ? a.EnumerateArray().Select(x => x.GetSingle()).ToArray() : null;
        // Legacy [] means fixed at zero; direct ABI uses explicit discrete rotations.
        if (angles is { Length: 0 }) angles = [0];
        if (item.TryGetProperty("orientation", out _))
            throw new InvalidOperationException("Use a legacy-orientation fixture or extend this benchmark loader explicitly.");
        int demand = item.GetProperty("demand").GetInt32();
        for (int n = 0; n < demand; n++) items.Add(new PolygonItem {
            Id = (ulong)items.Count, AllowedOrientations = angles, Shape = new Shape { Data = points }
        });
    }
    return new Envelope { Instance = new Instance {
        Name = root.GetProperty("name").GetString()!, StripHeight = root.GetProperty("strip_height").GetSingle(),
        MinItemSeparation = root.TryGetProperty("min_item_separation", out var sep) ? sep.GetSingle() : 0,
        Items = items.ToArray()
    } };
}

internal sealed class Envelope {
    public int SchemaVersion { get; init; } = 2;
    public Nesting Nesting { get; init; } = new();
    public required Instance Instance { get; init; }
}
internal sealed class Nesting {
    public string Mode { get; init; } = "all_items";
    public int GroupWorkers { get; init; } = 1;
    public ulong QuantumMs { get; init; } = 250;
    public float LockedPrefixGapMm { get; init; }
}
internal sealed class Instance {
    public required string Name { get; init; }
    public float StripHeight { get; init; }
    public float MinItemSeparation { get; init; }
    public required PolygonItem[] Items { get; init; }
}
internal sealed class PolygonItem {
    public ulong Id { get; init; }
    public ulong Demand { get; init; } = 1;
    public float[]? AllowedOrientations { get; init; }
    public required Shape Shape { get; init; }
}
internal sealed class Shape {
    public string Type { get; init; } = "simple_polygon";
    public required float[][] Data { get; init; }
}
internal sealed class PinnedBytes : IDisposable {
    private readonly GCHandle pin;
    public nint Address => pin.AddrOfPinnedObject();
    public PinnedBytes(byte[] data) => pin = GCHandle.Alloc(data, GCHandleType.Pinned);
    public void Dispose() => pin.Free();
}
internal sealed unsafe class Packed : IDisposable {
    private readonly List<GCHandle> pins = [];
    public readonly Input Input;
    public Packed(Envelope model) {
        try {
            var source = model.Instance.Items;
            var points = new Point[source.Sum(x => x.Shape.Data.Length)];
            var angles = new float[source.Sum(x => x.AllowedOrientations?.Length ?? 0)];
            var items = new Item[source.Length];
            nint pointBase = Pin(points), angleBase = Pin(angles);
            int pi = 0, ai = 0;
            for (int i = 0; i < source.Length; i++) {
                var s = source[i];
                items[i] = new Item { Id = s.Id, Demand = 1, ShapeKind = 1, MinQuality = -1,
                    Vertices = pointBase + pi * sizeof(Point), VertexCount = (nuint)s.Shape.Data.Length,
                    RotationKind = s.AllowedOrientations == null ? 2u : 0u,
                    Angles = angleBase + ai * sizeof(float), AngleCount = (nuint)(s.AllowedOrientations?.Length ?? 0) };
                foreach (var p in s.Shape.Data) points[pi++] = new Point { X = p[0], Y = p[1] };
                if (s.AllowedOrientations != null) foreach (float a in s.AllowedOrientations) angles[ai++] = a;
            }
            byte[] name = Encoding.UTF8.GetBytes(model.Instance.Name);
            Input = new Input { AbiVersion = 1, StructSize = (uint)sizeof(Input), Name = Pin(name), NameLength = (nuint)name.Length,
                StripHeight = model.Instance.StripHeight, MinSeparation = model.Instance.MinItemSeparation,
                Items = Pin(items), ItemCount = (nuint)items.Length, Workers = 1, QuantumMs = 250 };
        } catch { Dispose(); throw; }
    }
    private nint Pin(Array array) {
        var pin = GCHandle.Alloc(array, GCHandleType.Pinned);
        pins.Add(pin);
        return pin.AddrOfPinnedObject();
    }
    public void Dispose() { foreach (var pin in pins) pin.Free(); pins.Clear(); }
}
