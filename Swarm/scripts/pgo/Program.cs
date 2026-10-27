using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Reflection;
using System.Security.Cryptography;
using System.Text.Json;
using Uttu.NestingV2;

if (args.Length != 8)
    throw new ArgumentException("DLL input.json seed overlap_mm explore_seconds compress_seconds polish_seconds output_prefix");
var dllPath = Path.GetFullPath(args[0]);
var inputPath = Path.GetFullPath(args[1]);
var prefix = Path.GetFullPath(args[7]);
Directory.CreateDirectory(Path.GetDirectoryName(prefix)!);
// Register before SwarmNative's static constructor. Never fall back to another checkout DLL.
var library = NativeLibrary.Load(dllPath);
NativeLibrary.SetDllImportResolver(typeof(SwarmNative).Assembly,
    (name, assembly, searchPath) => name == "cpswcore" ? library : IntPtr.Zero);
SwarmNative.EnsureCompatible();
var exports = (string[])typeof(SwarmNative).GetField("RequiredExports", BindingFlags.NonPublic | BindingFlags.Static)!.GetValue(null)!;
foreach (var export in exports) _ = NativeLibrary.GetExport(library, export);
var cfg = new SwarmNative.CSwarmConfig();
SwarmNative.swarm_get_default_config(ref cfg);
cfg.rng_seed = ulong.Parse(args[2], CultureInfo.InvariantCulture);
cfg.poly_simpl_tolerance = 0.01f;
cfg.fabric_tolerance.piece_overlap_mm = float.Parse(args[3], CultureInfo.InvariantCulture);
cfg.exploration.time_limit_secs = float.Parse(args[4], CultureInfo.InvariantCulture);
cfg.compression.time_limit_secs = float.Parse(args[5], CultureInfo.InvariantCulture);
cfg.polish.time_limit_secs = float.Parse(args[6], CultureInfo.InvariantCulture);
cfg.exploration.separator.n_workers = 3;
cfg.compression.separator.n_workers = 3;
cfg.polish.enabled = 1;
cfg.polish.lp_enabled = 1;
cfg.polish.slide_enabled = 1;
cfg.polish.hole_fill_enabled = 1;
var input = File.ReadAllText(inputPath);
using var inputDoc = JsonDocument.Parse(input);
var expected = inputDoc.RootElement.GetProperty("items").EnumerateArray()
    .ToDictionary(x => x.GetProperty("id").GetInt32(), x => x.GetProperty("demand").GetInt32());
var steps = new List<object>();
var timer = Stopwatch.StartNew();
var handle = SwarmNative.swarm_init_from_json(input, ref cfg, [], 0);
if (handle == IntPtr.Zero) throw new InvalidOperationException("Init: " + LastError());
var callbackCount = 0;
var callbackJsonCount = 0;
// Callback pointers belong to native code and must not be freed by managed code.
SwarmNative.SolutionCallback callback = (kind, width, density, ptr, data) =>
{
    Interlocked.Increment(ref callbackCount);
    if (ptr != IntPtr.Zero && Marshal.PtrToStringUTF8(ptr) is { Length: > 0 })
        Interlocked.Increment(ref callbackJsonCount);
};
double lastWidth = double.PositiveInfinity;
try
{
    Checkpoint("Init", timer.Elapsed.TotalSeconds);
    SwarmNative.swarm_set_callback(handle, callback, IntPtr.Zero, 1);
    foreach (var (name, phase) in new (string, Func<IntPtr, int>)[] {
        ("Explore", SwarmNative.swarm_alg_phase),
        ("Compress", SwarmNative.swarm_cp_phase),
        ("Polish", SwarmNative.swarm_refine_phase) })
    {
        timer.Restart();
        var rc = phase(handle);
        if (rc != 0) throw new InvalidOperationException($"{name}: {rc}: {LastError()}");
        Checkpoint(name, timer.Elapsed.TotalSeconds);
    }
}
finally
{
    SwarmNative.swarm_free(handle);
    GC.KeepAlive(callback);
    // Keep the DLL loaded until process exit so LLVM's profile atexit hook can flush.
}
File.WriteAllText(prefix + ".summary.json", JsonSerializer.Serialize(new {
    dll = dllPath, sha256 = Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(dllPath))),
    input = inputPath, input_sha256 = Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(inputPath))),
    seed = cfg.rng_seed, overlap_mm = cfg.fabric_tolerance.piece_overlap_mm, polysimpl = cfg.poly_simpl_tolerance,
    config = cfg, requiredExports = exports.Length, callbackCount, callbackJsonCount, steps
}, new JsonSerializerOptions { WriteIndented = true, IncludeFields = true }));

string LastError() => Marshal.PtrToStringUTF8(SwarmNative.swarm_get_last_error_message()) ?? "no error message";
string TakeString(IntPtr ptr)
{
    if (ptr == IntPtr.Zero) throw new InvalidOperationException(LastError());
    try { return Marshal.PtrToStringUTF8(ptr)!; }
    finally { SwarmNative.swarm_free_string(ptr); }
}
void Checkpoint(string name, double seconds)
{
    var json = TakeString(SwarmNative.swarm_get_solution_json(handle));
    File.WriteAllText(prefix + "." + name + ".json", json);
    using var doc = JsonDocument.Parse(json);
    var layout = doc.RootElement.GetProperty("layout");
    var actual = layout.GetProperty("placed_items").EnumerateArray()
        .GroupBy(x => x.GetProperty("item_id").GetInt32()).ToDictionary(g => g.Key, g => g.Count());
    if (expected.Count != actual.Count || expected.Any(kv => !actual.TryGetValue(kv.Key, out var n) || n != kv.Value))
        throw new InvalidOperationException(name + ": missing/duplicate placements");
    var width = doc.RootElement.GetProperty("strip_width").GetDouble();
    if (!double.IsFinite(width) || width <= 0 || width > lastWidth + 0.01)
        throw new InvalidOperationException($"{name}: invalid/increased length {lastWidth} -> {width}");
    lastWidth = width;
    var collisions = TakeString(SwarmNative.swarm_get_collision_status_json(handle));
    File.WriteAllText(prefix + "." + name + ".collisions.json", collisions);
    using var status = JsonDocument.Parse(collisions);
    var count = status.RootElement.EnumerateArray().Count(x => x.GetProperty("collision").GetBoolean());
    if (count != 0) throw new InvalidOperationException($"{name}: {count} colliding placements");
    steps.Add(new { phase = name, seconds, width, placements = actual.Values.Sum(), collisions = count });
    Console.WriteLine($"{name}: {seconds:F3}s, length={width:F3}mm, placements={actual.Values.Sum()}, collisions={count}");
}
