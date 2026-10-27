// Full-pipeline bench: Init -> Explore -> Compress -> Polish, with JSON config overrides.
// Reports per-phase width/time and final efficiency against the *input* polygon area.
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Uttu.NestingV2;

// Optional trailing "--log" captures the native info-level log so feature-firing can be
// audited (width relief / SEP-REPAIR / ALNS). Off by default: registering a sink adds
// per-line formatting cost inside the measured phases, so comparison runs must not use it.
var captureLog = args.Length > 0 && args[^1] == "--log";
if (captureLog) args = args[..^1];
if (args.Length is < 8 or > 9)
    throw new ArgumentException("absolute_dll input.json seed explore_s compress_s polish_s overlap_mm out_prefix [overrides.json] [--log]");
if (!Path.IsPathFullyQualified(args[0])) throw new ArgumentException("DLL path must be absolute");

var dll = Path.GetFullPath(args[0]);
var inputPath = Path.GetFullPath(args[1]);
var seed = ulong.Parse(args[2], CultureInfo.InvariantCulture);
var exploreS = float.Parse(args[3], CultureInfo.InvariantCulture);
var compressS = float.Parse(args[4], CultureInfo.InvariantCulture);
var polishS = float.Parse(args[5], CultureInfo.InvariantCulture);
var overlapMm = float.Parse(args[6], CultureInfo.InvariantCulture);
var prefix = Path.GetFullPath(args[7]);
var overridesPath = args.Length == 9 ? Path.GetFullPath(args[8]) : null;
if (seed == 0) throw new ArgumentException("Seed 0 means 'random' in this ABI; pass a nonzero seed for reproducibility");
if (!float.IsFinite(exploreS) || exploreS <= 0) throw new ArgumentException("explore_s must be positive");
if (!float.IsFinite(overlapMm) || overlapMm < 0) throw new ArgumentException("overlap_mm must be >= 0");
Directory.CreateDirectory(Path.GetDirectoryName(prefix)!);

var serializer = new JsonSerializerOptions { WriteIndented = true, IncludeFields = true };
var dllHash = Hash(File.ReadAllBytes(dll));
var inputBytes = File.ReadAllBytes(inputPath);
var inputHash = Hash(inputBytes);
var input = Encoding.UTF8.GetString(inputBytes);
using var inputDoc = JsonDocument.Parse(input);
var root = inputDoc.RootElement;
var stripHeight = root.GetProperty("strip_height").GetDouble();
var expected = root.GetProperty("items").EnumerateArray()
    .Where(x => x.GetProperty("demand").GetInt32() > 0)
    .ToDictionary(x => x.GetProperty("id").GetInt32(), x => x.GetProperty("demand").GetInt32());
var count = expected.Values.Sum();
// Exact input-polygon area (shoelace, outer ring), independent of the engine's simplified geometry.
double inputArea = 0;
foreach (var item in root.GetProperty("items").EnumerateArray())
{
    var demand = item.GetProperty("demand").GetInt32();
    if (demand <= 0) continue;
    if (!item.TryGetProperty("shape", out var shape) || !shape.TryGetProperty("data", out var data)) continue;
    double a = 0;
    var pts = data.EnumerateArray().Select(p => p.EnumerateArray().Select(v => v.GetDouble()).ToArray()).ToArray();
    for (var i = 0; i < pts.Length; i++)
    {
        var p = pts[i];
        var q = pts[(i + 1) % pts.Length];
        a += p[0] * q[1] - q[0] * p[1];
    }
    inputArea += Math.Abs(a) / 2.0 * demand;
}
if (inputArea <= 0) throw new InvalidOperationException("Could not compute input polygon area");

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
cfg.rng_seed = seed;
// poly_simpl_tolerance stays at the native product default (0.001). The older explore_bench
// pinned it to 0.01 for speed; that inflates collision geometry and caps achievable density,
// so it is an explicit arm here rather than a silent baseline choice.
cfg.fabric_tolerance.piece_overlap_mm = overlapMm;
cfg.exploration.time_limit_secs = exploreS;
cfg.exploration.separator.n_workers = 3;
cfg.compression.time_limit_secs = compressS;
cfg.compression.separator.n_workers = 3;
cfg.polish.time_limit_secs = polishS;
cfg.polish.enabled = polishS > 0 ? 1 : 0;
cfg.callback_options.collect_loss = 1;
cfg.callback_options.collect_collision_status = 1;

var applied = new List<string>();
if (overridesPath is not null)
{
    using var od = JsonDocument.Parse(File.ReadAllText(overridesPath));
    foreach (var prop in od.RootElement.EnumerateObject())
    {
        if (prop.Name.StartsWith('_')) continue; // comment keys
        SetPath(ref cfg, prop.Name, prop.Value);
        applied.Add(prop.Name + "=" + prop.Value.ToString());
    }
}

var raw = new List<RawCallback>();
var trace = new List<TracePoint>();
var errors = new List<string>();
var nativeLogs = new List<string>();
SwarmNative.SwarmLogCallback logCallback = (_, ptr, _) =>
{
    try { var line = Marshal.PtrToStringUTF8(ptr); if (line != null) lock (nativeLogs) nativeLogs.Add(line); }
    catch { /* Never unwind through a native callback. */ }
};
if (captureLog) SwarmNative.swarm_set_log_callback(logCallback, UIntPtr.Zero, 3);
var phaseTimer = new Stopwatch();
using var measuredProcess = Process.GetCurrentProcess();
TimeSpan CpuNow() { measuredProcess.Refresh(); return measuredProcess.TotalProcessorTime; }
var cpuSeconds = new Dictionary<string, double>();
var checkpoints = new Dictionary<string, object>();
var startedUtc = DateTimeOffset.UtcNow;
var phaseName = "Init";
Snapshot? initial = null, afterInitial = null, afterExplore = null, afterCompress = null, final = null;
double initSeconds = 0, initialPhaseSeconds = 0, exploreSeconds = 0, compressSeconds = 0, polishSeconds = 0;
var handle = IntPtr.Zero;

SwarmNative.SolutionCallback callback = (kind, width, density, ptr, _) =>
{
    // Borrowed pointer: copy only; never free, parse, or reenter the handle here.
    try
    {
        var time = phaseTimer.Elapsed.TotalSeconds;
        var json = ptr == IntPtr.Zero ? null : Marshal.PtrToStringUTF8(ptr);
        string phase;
        lock (raw) { phase = phaseName; raw.Add(new RawCallback(time, phase, kind, width, density, json)); }
    }
    catch (Exception ex) { lock (errors) errors.Add("Callback capture: " + ex.Message); }
};

try
{
    Console.WriteLine($"Starting Init: {dll}");
    var cpuStart = CpuNow();
    var timer = Stopwatch.StartNew();
    handle = SwarmNative.swarm_init_from_json(input, ref cfg, [], 0);
    initSeconds = timer.Elapsed.TotalSeconds;
    cpuSeconds["init"] = (CpuNow() - cpuStart).TotalSeconds;
    if (handle == IntPtr.Zero) throw new InvalidOperationException("Init: " + LastError());
    initial = Checkpoint("Init");
    SwarmNative.swarm_set_callback(handle, callback, IntPtr.Zero, 1);

    // N06 attach constructor is a separate opt-in phase: swarm_init only builds the LBF
    // layout, so enabling initial_placement in config alone does nothing (measured: identical
    // Init width). Call it when the caller enabled it.
    if (cfg.initial_placement.enabled != 0)
    {
        phaseName = "Initial";
        phaseTimer.Restart();
        var irc = SwarmNative.swarm_init_phase(handle);
        phaseTimer.Stop();
        initialPhaseSeconds = phaseTimer.Elapsed.TotalSeconds;
        if (irc != 0) throw new InvalidOperationException($"Initial rc={irc}: {LastError()}");
        afterInitial = Checkpoint("Initial");
    }

    // structure_first replaces the adaptive Explore with the P0-P5 pipeline; it is a
    // different constructor+search, not a knob on this one, so it takes the Explore slot.
    var useStructure = cfg.structure_first.enabled != 0;
    phaseName = "Explore";
    Console.WriteLine($"Init complete ({initSeconds:F3}s); starting Explore");
    cpuStart = CpuNow();
    phaseTimer.Restart();
    var rc = useStructure
        ? SwarmNative.swarm_gp_phase(handle)
        : SwarmNative.swarm_alg_phase(handle);
    phaseTimer.Stop();
    exploreSeconds = phaseTimer.Elapsed.TotalSeconds;
    cpuSeconds["explore"] = (CpuNow() - cpuStart).TotalSeconds;
    if (rc != 0) throw new InvalidOperationException($"{(useStructure ? "Structure" : "Explore")} rc={rc}: {LastError()}");
    afterExplore = Checkpoint("Explore");

    if (compressS > 0)
    {
        phaseName = "Compress";
        Console.WriteLine($"Explore complete ({exploreSeconds:F3}s, width={afterExplore.width}); starting Compress");
        cpuStart = CpuNow();
        phaseTimer.Restart();
        rc = SwarmNative.swarm_cp_phase(handle);
        phaseTimer.Stop();
        compressSeconds = phaseTimer.Elapsed.TotalSeconds;
        cpuSeconds["compress"] = (CpuNow() - cpuStart).TotalSeconds;
        if (rc != 0) throw new InvalidOperationException($"Compress rc={rc}: {LastError()}");
        afterCompress = Checkpoint("Compress");
    }
    else afterCompress = afterExplore;

    if (polishS > 0)
    {
        phaseName = "Polish";
        phaseTimer.Restart();
        rc = SwarmNative.swarm_refine_phase(handle);
        phaseTimer.Stop();
        polishSeconds = phaseTimer.Elapsed.TotalSeconds;
        if (rc != 0) throw new InvalidOperationException($"Polish rc={rc}: {LastError()}");
        final = Checkpoint("Polish");
    }
    else final = afterCompress;

    foreach (var c in raw)
    {
        if (!double.IsFinite(c.width) || c.width <= 0) throw new InvalidOperationException("Invalid callback width");
        if (c.json is null) { trace.Add(new(c.seconds, c.phase, c.kind, c.width, null, false, null)); continue; }
        using var doc = JsonDocument.Parse(c.json);
        var r = doc.RootElement;
        var snapshot = Inspect(r.GetProperty("solution"));
        var collisions = CollisionCount(r.GetProperty("collision_status"));
        var loss = r.GetProperty("loss").GetDouble();
        if (Math.Abs(c.width - snapshot.width) > Tolerance(snapshot.width)) throw new InvalidOperationException("Callback scalar/JSON width mismatch");
        var certified = collisions == 0 && loss == 0;
        if (c.kind is 0 or 3 or 4 && !certified) throw new InvalidOperationException("Feasible callback has collision/loss");
        trace.Add(new(c.seconds, c.phase, c.kind, c.width, snapshot.width, certified, collisions));
    }
    if (final.width > initial.width + Tolerance(initial.width)) throw new InvalidOperationException("Final width worse than Init");
    if (Hash(File.ReadAllBytes(dll)) != dllHash) throw new InvalidOperationException("DLL changed during run");
}
catch (Exception ex) { errors.Add(ex.ToString()); }
finally
{
    if (handle != IntPtr.Zero) SwarmNative.swarm_free(handle);
    GC.KeepAlive(callback);
    if (captureLog) SwarmNative.swarm_set_log_callback(null, UIntPtr.Zero, 0);
    GC.KeepAlive(logCallback);
}
if (captureLog) File.WriteAllLines(prefix + ".native.log", nativeLogs);

double Eff(double? w) => w is null or <= 0 ? 0 : inputArea / (w.Value * stripHeight);
var result = new
{
    passed = errors.Count == 0,
    started_utc = startedUtc,
    cpu_seconds = cpuSeconds,
    checkpoints,
    errors,
    dll,
    dll_sha256 = dllHash,
    input = inputPath,
    input_sha256 = inputHash,
    seed,
    overlap_mm = overlapMm,
    requested = new { explore_s = exploreS, compress_s = compressS, polish_s = polishS },
    overrides = applied,
    input_area_mm2 = inputArea,
    strip_height = stripHeight,
    seconds = new { init = initSeconds, initial_phase = initialPhaseSeconds, explore = exploreSeconds, compress = compressSeconds, polish = polishSeconds },
    width = new { init = initial?.width, initial_phase = afterInitial?.width, explore = afterExplore?.width, compress = afterCompress?.width, final = final?.width },
    efficiency = new { init = Eff(initial?.width), initial_phase = Eff(afterInitial?.width), explore = Eff(afterExplore?.width), compress = Eff(afterCompress?.width), final = Eff(final?.width) },
    abi = abiArgs[0],
    config = cfg,
    callback_count = raw.Count,
    trace,
    measurement_note = "Efficiency = exact input polygon area / (final strip width x strip height). Phases timed separately; callback capture is inside each phase's time. Trace times are throttled upper bounds.",
};
File.WriteAllText(prefix + ".summary.json", JsonSerializer.Serialize(result, serializer));
File.WriteAllText(prefix + ".callbacks.jsonl", string.Join("\n", raw.Select(x => JsonSerializer.Serialize(new { x.seconds, x.phase, x.kind, x.width, x.density }))) + "\n");
Console.WriteLine(FormattableString.Invariant(
    $"passed={errors.Count == 0} seed={seed} init={initial?.width:F2}({Eff(initial?.width) * 100:F3}%) n06={afterInitial?.width:F2} explore={afterExplore?.width:F2}({Eff(afterExplore?.width) * 100:F3}%) compress={afterCompress?.width:F2}({Eff(afterCompress?.width) * 100:F3}%) final={final?.width:F2}({Eff(final?.width) * 100:F3}%) t={initSeconds:F1}/{exploreSeconds:F1}/{compressSeconds:F1}/{polishSeconds:F1}"));
if (errors.Count != 0) { Console.Error.WriteLine(string.Join("\n", errors)); Environment.ExitCode = 1; }

static void SetPath(ref SwarmNative.CSwarmConfig cfg, string path, JsonElement value)
{
    var parts = path.Split('.');
    object cur = cfg;
    var chain = new List<(object obj, FieldInfo fi)>();
    for (var i = 0; i < parts.Length; i++)
    {
        var fi = cur.GetType().GetField(parts[i], BindingFlags.Public | BindingFlags.Instance)
                 ?? throw new ArgumentException($"Unknown config field '{parts[i]}' in path '{path}'");
        if (i == parts.Length - 1) { fi.SetValue(cur, Coerce(value, fi.FieldType, path)); break; }
        chain.Add((cur, fi));
        cur = fi.GetValue(cur)!;
    }
    for (var i = chain.Count - 1; i >= 0; i--) { chain[i].fi.SetValue(chain[i].obj, cur); cur = chain[i].obj; }
    cfg = (SwarmNative.CSwarmConfig)cur;
}

static object Coerce(JsonElement value, Type type, string path)
{
    if (type == typeof(int)) return value.GetInt32();
    if (type == typeof(uint)) return value.GetUInt32();
    if (type == typeof(long)) return value.GetInt64();
    if (type == typeof(ulong)) return value.GetUInt64();
    if (type == typeof(float)) return (float)value.GetDouble();
    if (type == typeof(double)) return value.GetDouble();
    throw new ArgumentException($"Unsupported config field type {type.Name} at '{path}'");
}

string Hash(byte[] bytes) => Convert.ToHexString(SHA256.HashData(bytes));
double Tolerance(double width) => Math.Max(0.00001, Math.Abs(width) * 0.0000002);
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
    if (actual.Count != expected.Count || expected.Any(kv => !actual.TryGetValue(kv.Key, out var n) || n != kv.Value))
        throw new InvalidOperationException("Missing/duplicate/wrong item IDs or demand");
    foreach (var x in placed.EnumerateArray())
    {
        var tr = x.GetProperty("transformation");
        if (!double.IsFinite(tr.GetProperty("rotation").GetDouble())) throw new InvalidOperationException("Invalid rotation");
        if (tr.GetProperty("translation").EnumerateArray().Any(v => !double.IsFinite(v.GetDouble()))) throw new InvalidOperationException("Invalid translation");
    }
    return new Snapshot(width, count);
}
Snapshot Checkpoint(string name)
{
    var solution = TakeString(SwarmNative.swarm_get_solution_json(handle));
    var collisions = TakeString(SwarmNative.swarm_get_collision_status_json(handle));
    File.WriteAllText(prefix + "." + name + ".json", solution);
    using var doc = JsonDocument.Parse(solution);
    using var status = JsonDocument.Parse(collisions);
    var snapshot = Inspect(doc.RootElement);
    var collisionCount = CollisionCount(status.RootElement);
    checkpoints[name] = new { snapshot.placements, native_collision_count = collisionCount,
        solution_sha256 = Hash(Encoding.UTF8.GetBytes(solution)),
        native_density = doc.RootElement.TryGetProperty("density", out var density) ? density.GetDouble() : (double?)null };
    File.WriteAllText(prefix + "." + name + ".collisions.json", collisions);
    if (collisionCount != 0) throw new InvalidOperationException(name + " has native collision status");
    return snapshot;
}
record RawCallback(double seconds, string phase, int kind, double width, double density, string? json);
record Snapshot(double width, int placements);
record TracePoint(double seconds, string phase, int kind, double scalar_width, double? width, bool certified, int? collisions);
