using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text.Json;

internal static class InitTiming
{
    public static void Run(string dll, string output, string file, Envelope model, JsonSerializerOptions options)
    {
        if (Native.swarm_get_abi_info(out var abi) != 0 || abi.Version != 20)
            throw new InvalidOperationException("Init timing requires ABI 20.");
        string original = File.ReadAllText(file); // Disk I/O and source model loading are outside timing.
        var rows = new List<object>();
        var requested = Environment.GetEnvironmentVariable("SWARM_INIT_CASE");
        foreach (string name in new[] { "original_legacy_json", "v2_json_pretty", "v2_struct" })
        {
            if (requested is not null && requested != name) continue;
            nint config = Marshal.AllocHGlobal(checked((int)abi.ConfigSize));
            nint handle = 0;
            Packed? packed = null;
            try
            {
                // Clear padding as well so the recorded config hash is reproducible.
                Marshal.Copy(new byte[abi.ConfigSize], 0, config, checked((int)abi.ConfigSize));
                Native.swarm_get_default_config(config);
                Marshal.WriteInt64(config, 0, 1);
                var configBytes = new byte[abi.ConfigSize];
                Marshal.Copy(config, configBytes, 0, configBytes.Length);
                Console.WriteLine($"START {name} {DateTimeOffset.Now:O}; items={model.Instance.Items.Length}; seed=1; native defaults");
                long prepareStart = Stopwatch.GetTimestamp();
                string? json = name == "original_legacy_json" ? original :
                    name == "v2_json_pretty" ? JsonSerializer.Serialize(model, options) : null;
                if (name == "v2_struct") packed = new Packed(model);
                double prepareMs = Stopwatch.GetElapsedTime(prepareStart).TotalMilliseconds;
                var process = Process.GetCurrentProcess();
                TimeSpan cpuStart = process.TotalProcessorTime;
                long initStart = Stopwatch.GetTimestamp();
                handle = packed is null
                    ? Native.swarm_init_from_json(json!, config, 0, 0)
                    : Native.swarm_init_from_struct(in packed.Input, config);
                double initMs = Stopwatch.GetElapsedTime(initStart).TotalMilliseconds;
                double cpuMs = (process.TotalProcessorTime - cpuStart).TotalMilliseconds;
                int errorCode = Native.swarm_get_last_error_code();
                string? error = null;
                int? placed = null;
                double? width = null, density = null;
                string? layoutHash = null;
                if (handle == 0)
                {
                    nint text = Native.swarm_get_last_error_message();
                    try { error = Marshal.PtrToStringUTF8(text); }
                    finally { if (text != 0) Native.swarm_free_string(text); }
                }
                else
                {
                    nint text = Native.swarm_get_solution_json(handle); // Export/validation is outside Init timing.
                    try
                    {
                        if (text == 0) throw new InvalidOperationException("Initial solution export failed.");
                        using var doc = JsonDocument.Parse(Marshal.PtrToStringUTF8(text)!);
                        var root = doc.RootElement;
                        var layout = root.GetProperty("layout");
                        placed = layout.GetProperty("placed_items").GetArrayLength();
                        width = root.GetProperty("strip_width").GetDouble();
                        density = root.GetProperty("density").GetDouble();
                        layoutHash = Convert.ToHexString(SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(layout.GetRawText())));
                    }
                    finally { if (text != 0) Native.swarm_free_string(text); }
                }
                rows.Add(new { Name = name, PrepareMs = prepareMs, InitMs = initMs,
                    TotalMs = prepareMs + initMs, CpuMs = cpuMs, Success = handle != 0,
                    ErrorCode = errorCode, Error = error, PlacedItems = placed, StripWidth = width,
                    Density = density, LayoutSha256 = layoutHash,
                    ConfigSha256 = Convert.ToHexString(SHA256.HashData(configBytes)) });
                File.WriteAllText(output, JsonSerializer.Serialize(new {
                    Utc = DateTimeOffset.UtcNow, Dll = dll, DllSha256 = Hash(dll),
                    File = Path.GetFullPath(file), SourceSha256 = Hash(file),
                    OS = RuntimeInformation.OSDescription, Runtime = RuntimeInformation.FrameworkDescription,
                    ProcessorCount = Environment.ProcessorCount, Abi = abi.Version, ConfigSize = abi.ConfigSize,
                    Config = "swarm_get_default_config with rng_seed=1; no hints; V2 all_items, group_workers=1, quantum_ms=250",
                    Boundary = "Init API entry (including UTF-8 marshaling for JSON) to returned handle with initial placement. File loading, config setup, solution export and destruction excluded. PrepareMs separately measures C# serialization or packing from preloaded model. One measured run per case; no full-size warmup.",
                    Items = model.Instance.Items.Length, Vertices = model.Instance.Items.Sum(x => x.Shape.Data.Length),
                    Rows = rows
                }, new JsonSerializerOptions { WriteIndented = true }));
                Console.WriteLine($"DONE {name}: init={initMs / 1000:F3}s; prepare={prepareMs:F3}ms; cpu={cpuMs / 1000:F3}s; placed={placed}; success={handle != 0}; error={error}");
                if (handle != 0 && placed != model.Instance.Items.Length)
                    throw new InvalidOperationException($"Initial layout has {placed} placements; expected {model.Instance.Items.Length}.");
            }
            finally
            {
                if (handle != 0) Native.swarm_free(handle);
                packed?.Dispose();
                Marshal.FreeHGlobal(config);
            }
        }
    }

    private static string Hash(string path) => Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(path)));
}
