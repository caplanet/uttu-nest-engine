using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Uttu.NestingV2;

if (args.Length is < 5 or > 6) throw new ArgumentException("absolute_Dll input.json nonzero_seed explore_seconds output_prefix [off|direction|lp|chain|all]");
if (!Path.IsPathFullyQualified(args[0])) throw new ArgumentException("DLL path must be absolute");
var dll = Path.GetFullPath(args[0]);
var inputPath = Path.GetFullPath(args[1]);
var seed = ulong.Parse(args[2], CultureInfo.InvariantCulture);
var seconds = float.Parse(args[3], CultureInfo.InvariantCulture);
if (seed == 0 || !float.IsFinite(seconds) || seconds <= 0) throw new ArgumentException("Seed and seconds must be positive");
var prefix = Path.GetFullPath(args[4]);
Directory.CreateDirectory(Path.GetDirectoryName(prefix)!);
var serializer = new JsonSerializerOptions { WriteIndented = true, IncludeFields = true };
var dllHash = Hash(File.ReadAllBytes(dll));
var inputBytes = File.ReadAllBytes(inputPath);
var inputHash = Hash(inputBytes);
var input = File.ReadAllText(inputPath);
using var inputDoc = JsonDocument.Parse(input);
var expected = inputDoc.RootElement.GetProperty("items").EnumerateArray()
    .Where(x => x.GetProperty("demand").GetInt32() > 0)
    .ToDictionary(x => x.GetProperty("id").GetInt32(), x => x.GetProperty("demand").GetInt32());
var count = expected.Values.Sum();

// Install before SwarmNative's static constructor: no DLL search fallback.
var library = NativeLibrary.Load(dll);
NativeLibrary.SetDllImportResolver(typeof(SwarmNative).Assembly,
    (name, assembly, searchPath) => name == "cpswcore" ? library : IntPtr.Zero);
SwarmNative.EnsureCompatible();
var exports = (string[])typeof(SwarmNative).GetField("RequiredExports", BindingFlags.NonPublic | BindingFlags.Static)!.GetValue(null)!;
foreach (var export in exports) _ = NativeLibrary.GetExport(library, export);
object?[] abiArgs = [null];
var abiRc = (int)typeof(SwarmNative).GetMethod("swarm_get_abi_info", BindingFlags.NonPublic | BindingFlags.Static)!.Invoke(null, abiArgs)!;
if (abiRc != 0) throw new InvalidOperationException("ABI query failed");
var cfg = new SwarmNative.CSwarmConfig();
SwarmNative.swarm_get_default_config(ref cfg);
cfg.rng_seed = seed; // Same Init seed per A/B pair. ABI has no post-Init reseed operation.
cfg.poly_simpl_tolerance = 0.01f;
cfg.fabric_tolerance.piece_overlap_mm = 2.2f;
cfg.exploration.time_limit_secs = seconds;
cfg.exploration.separator.n_workers = 3;
cfg.callback_options.collect_loss = 1;
cfg.callback_options.collect_collision_status = 1;
var repairMode = args.Length == 6 ? args[5] : "off";
if (repairMode is not ("off" or "direction" or "lp" or "chain" or "all")) throw new ArgumentException("Unknown repair mode");
cfg.repair.direction_enabled = repairMode is "direction" or "all" ? 1 : 0;
cfg.repair.cluster_lp_enabled = repairMode is "lp" or "all" ? 1 : 0;
cfg.repair.chain_enabled = repairMode is "chain" or "all" ? 1 : 0;
var raw = new List<RawCallback>();
var trace = new List<TracePoint>();
var errors = new List<string>();
var phaseTimer = new Stopwatch();
var nativeLogs = new List<string>();
SwarmNative.SwarmLogCallback logCallback = (_, ptr, _) => {
    try { var line = Marshal.PtrToStringUTF8(ptr); if (line != null) lock(nativeLogs) nativeLogs.Add(line); }
    catch { /* Never unwind through a native callback. */ }
};
if (args.Length == 6) SwarmNative.swarm_set_log_callback(logCallback, UIntPtr.Zero, 3);
double initSeconds = 0, exploreSeconds = 0;
Snapshot? initial = null, final = null;
var handle = IntPtr.Zero;
SwarmNative.SolutionCallback callback = (kind, width, density, ptr, _) =>
{
    // Borrowed pointer: copy only; never free, parse, or reenter the handle here.
    try
    {
        var time = phaseTimer.Elapsed.TotalSeconds;
        var json = ptr == IntPtr.Zero ? null : Marshal.PtrToStringUTF8(ptr);
        lock (raw) raw.Add(new RawCallback(time, kind, width, density, json));
    }
    catch (Exception ex) { lock (errors) errors.Add("Callback capture: " + ex.Message); }
};
try
{
    var timer = Stopwatch.StartNew();
    handle = SwarmNative.swarm_init_from_json(input, ref cfg, [], 0);
    initSeconds = timer.Elapsed.TotalSeconds;
    if (handle == IntPtr.Zero) throw new InvalidOperationException("Init: " + LastError());
    initial = Checkpoint("Init");
    SwarmNative.swarm_set_callback(handle, callback, IntPtr.Zero, 1);
    phaseTimer.Start();
    var rc = SwarmNative.swarm_alg_phase(handle);
    phaseTimer.Stop();
    exploreSeconds = phaseTimer.Elapsed.TotalSeconds;
    if (rc != 0) throw new InvalidOperationException($"Explore rc={rc}: {LastError()}");
    final = Checkpoint("Explore");
    var best = initial.width;
    foreach (var c in raw)
    {
        if (!double.IsFinite(c.width) || c.width <= 0) throw new InvalidOperationException("Invalid callback width");
        if (c.json is null)
        {
            trace.Add(new(c.seconds, c.kind, c.width, null, false, null, null));
            continue;
        }
        using var doc = JsonDocument.Parse(c.json);
        var root = doc.RootElement;
        var solution = root.GetProperty("solution");
        var snapshot = Inspect(solution);
        var collisions = CollisionCount(root.GetProperty("collision_status"));
        var loss = root.GetProperty("loss").GetDouble();
        if (Math.Abs(c.width - snapshot.width) > Tolerance(snapshot.width)) throw new InvalidOperationException("Callback scalar/JSON width mismatch");
        // Feasible/final JSON must have zero native collision status and loss.
        var certified = collisions == 0 && loss == 0;
        if (c.kind is 0 or 3 or 4 && !certified) throw new InvalidOperationException("Feasible callback has collision/loss");
        if (certified)
        {
            if (snapshot.width > best + Tolerance(best)) throw new InvalidOperationException("Certified callback incumbent increased");
            best = Math.Min(best, snapshot.width);
        }
        trace.Add(new(c.seconds, c.kind, c.width, snapshot.width, certified, collisions, snapshot.pose_sha256));
    }
    if (final.width > best + Tolerance(best)) throw new InvalidOperationException("Final GetSolution lost an earlier certified incumbent");
    if (Hash(File.ReadAllBytes(dll)) != dllHash) throw new InvalidOperationException("DLL changed during run");
}
catch (Exception ex) { errors.Add(ex.ToString()); }
finally
{
    if (handle != IntPtr.Zero) SwarmNative.swarm_free(handle);
    GC.KeepAlive(callback);
    if (args.Length == 6) SwarmNative.swarm_set_log_callback(null, UIntPtr.Zero, 0);
    GC.KeepAlive(logCallback);
    // DLL intentionally remains loaded through process exit (LLVM PGO atexit).
}
File.WriteAllText(prefix + ".callbacks.jsonl", string.Join("\n", raw.Select(x => JsonSerializer.Serialize(x))) + "\n");
if (args.Length == 6) File.WriteAllLines(prefix + ".native.log", nativeLogs);
File.WriteAllText(prefix + ".summary.json", JsonSerializer.Serialize(new {
    passed = errors.Count == 0, errors, dll, dll_sha256 = dllHash, input = inputPath,
    input_sha256 = inputHash, seed, requested_explore_seconds = seconds,
    init_seconds = initSeconds, explore_seconds = exploreSeconds,
    initial, final, abi = abiArgs[0], required_exports = exports, config = cfg,
    callback_count = raw.Count, trace,
    measurement_note = "Init and Explore timed separately; callback capture and native diagnostics are included in Explore. Trace JSON is throttled; first observed target time is an upper bound. No Compress/Polish calls."
}, serializer));
Console.WriteLine($"passed={errors.Count == 0} Init={initSeconds:F3}s Explore={exploreSeconds:F3}s width={final?.width}");
if (errors.Count != 0) { Console.Error.WriteLine(string.Join("\n", errors)); Environment.ExitCode = 1; }

string Hash(byte[] bytes) => Convert.ToHexString(SHA256.HashData(bytes));
double Tolerance(double width) => Math.Max(0.00001, Math.Abs(width) * 0.0000002); // f32 serialization/ULP only
string LastError()
{
    var ptr = SwarmNative.swarm_get_last_error_message();
    if (ptr == IntPtr.Zero) return "no error";
    try { return Marshal.PtrToStringUTF8(ptr) ?? "no error"; }
    finally { SwarmNative.swarm_free_string(ptr); }
}
string TakeString(IntPtr ptr)
{
    if (ptr == IntPtr.Zero) throw new InvalidOperationException(LastError());
    try { return Marshal.PtrToStringUTF8(ptr)!; }
    finally { SwarmNative.swarm_free_string(ptr); }
}
int CollisionCount(JsonElement status)
{
    if (status.GetArrayLength() != count) throw new InvalidOperationException("Collision-status placement count mismatch");
    return status.EnumerateArray().Count(x => x.GetProperty("collision").GetBoolean());
}
Snapshot Inspect(JsonElement solution)
{
    var width = solution.GetProperty("strip_width").GetDouble();
    if (!double.IsFinite(width) || width <= 0) throw new InvalidOperationException("Invalid strip width");
    var placed = solution.GetProperty("layout").GetProperty("placed_items");
    var actual = placed.EnumerateArray().GroupBy(x => x.GetProperty("item_id").GetInt32()).ToDictionary(x => x.Key, x => x.Count());
    if (actual.Count != expected.Count || expected.Any(kv => !actual.TryGetValue(kv.Key, out var n) || n != kv.Value)) throw new InvalidOperationException("Missing/duplicate/wrong item IDs or demand");
    var poses = placed.EnumerateArray().Select(x => {
        var tr = x.GetProperty("transformation");
        var rotation = tr.GetProperty("rotation").GetDouble();
        var translation = tr.GetProperty("translation").EnumerateArray().Select(v => v.GetDouble()).ToArray();
        if (!double.IsFinite(rotation) || translation.Length != 2 || translation.Any(v => !double.IsFinite(v))) throw new InvalidOperationException("Invalid pose");
        return string.Join("|", x.GetProperty("item_id").GetInt32().ToString(CultureInfo.InvariantCulture), rotation.ToString("R", CultureInfo.InvariantCulture), translation[0].ToString("R", CultureInfo.InvariantCulture), translation[1].ToString("R", CultureInfo.InvariantCulture));
    }).OrderBy(x => x, StringComparer.Ordinal);
    return new Snapshot(width, count, Hash(Encoding.UTF8.GetBytes(string.Join("\n", poses))));
}
Snapshot Checkpoint(string name)
{
    var solution = TakeString(SwarmNative.swarm_get_solution_json(handle));
    var collisions = TakeString(SwarmNative.swarm_get_collision_status_json(handle));
    File.WriteAllText(prefix + "." + name + ".json", solution);
    File.WriteAllText(prefix + "." + name + ".collisions.json", collisions);
    using var doc = JsonDocument.Parse(solution);
    using var status = JsonDocument.Parse(collisions);
    var snapshot = Inspect(doc.RootElement);
    if (CollisionCount(status.RootElement) != 0) throw new InvalidOperationException(name + " has native collision status");
    return snapshot;
}
record RawCallback(double seconds, int kind, double width, double density, string? json);
record Snapshot(double width, int placements, string pose_sha256);
record TracePoint(double seconds, int kind, double scalar_width, double? width, bool certified, int? collisions, string? pose_sha256);
