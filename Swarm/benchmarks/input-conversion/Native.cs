using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
internal struct Point { public float X, Y; }
[StructLayout(LayoutKind.Sequential)]
internal struct Hint
{
    public int Ver;
    public long ItemId;
    public int IsFreeze;
    public float X, Y, Rotation, StripeY, StripeX;
}
[StructLayout(LayoutKind.Sequential)]
internal struct Item
{
    public ulong Id, Demand;
    public uint ShapeKind, RotationKind;
    public float X, Y, Width, Height;
    public nint Vertices;
    public nuint VertexCount;
    public nint Angles;
    public nuint AngleCount;
    public float RotationStep;
    public nint ReflectionAxes;
    public nuint ReflectionCount;
    public int MinQuality;
    public uint HasHint;
    public Hint Hint;
}
[StructLayout(LayoutKind.Sequential)]
internal struct Input
{
    public uint AbiVersion, StructSize;
    public nint Name;
    public nuint NameLength;
    public float StripHeight, MinSeparation;
    public nint Items;
    public nuint ItemCount;
    public uint Mode, Workers;
    public ulong QuantumMs;
    public float LockedGap;
    public nint Regions;
    public nuint RegionCount;
    public nint Groups;
    public nuint GroupCount;
}
[StructLayout(LayoutKind.Sequential)]
internal struct AbiInfo { public uint Version, ConfigSize, StructureSize, FeatureFlags; }
internal static partial class Native
{
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern nint swarm_init_from_json([MarshalAs(UnmanagedType.LPUTF8Str)] string json, nint config, nint hints, int hintCount);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern nint swarm_init_from_struct(in Input input, nint config);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern nint swarm_get_last_error_message();
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern int swarm_get_abi_info(out AbiInfo info);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void swarm_get_default_config(nint config);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern nint swarm_input_from_json([MarshalAs(UnmanagedType.LPUTF8Str)] string json);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl, EntryPoint = "swarm_input_from_json")]
    internal static extern nint InputFromUtf8(nint json);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern nint swarm_input_from_struct(in Input input);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void swarm_input_free(nint input);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern int swarm_get_last_error_code();
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern nint swarm_init_from_input(nint input, nint config);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern nint swarm_get_solution_json(nint handle);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void swarm_free_string(nint text);
    [DllImport("cpswcore", CallingConvention = CallingConvention.Cdecl)]
    internal static extern void swarm_free(nint handle);
}
