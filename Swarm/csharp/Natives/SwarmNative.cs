using System.IO;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.InteropServices;

namespace Uttu.NestingV2;

/// <summary>
/// Repr(C) 배치 힌트: 아이템의 시작 위치(와 선택적 고정).
/// <para>
/// 힌트를 보내면 <see cref="TrX"/>/<see cref="TrY"/>/<see cref="RotAngleInRad"/>는
/// <b>항상</b> 시작 위치로 쓰인다. <see cref="IsFreeze"/>는 그 위에 얹히는 별개의 제약이다.
/// 힌트를 주지 않은 아이템은 기존대로 LBF가 배치하므로, 일부에만 힌트를 주면
/// 그 아이템들은 지정 위치에서 출발하고 나머지는 LBF가 채운다.
/// </para>
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct CPlacedHint
{
    /// <summary>Item ID (0-based). 범위를 벗어나면 이 힌트는 무시된다.</summary>
    public int ItemId;
    /// <summary>
    /// 0 = 지정 위치에서 출발하되 탐색/압축이 자유롭게 옮김 (warm start),
    /// 1 = 지정 위치에 고정 (이동 금지).
    /// 좌표 사용 여부와는 무관하다 — 힌트를 보낸 이상 좌표는 항상 쓰인다.
    /// </summary>
    public int IsFreeze;
    /// <summary>아이템 bbox 좌하단이 놓일 x (mm)</summary>
    public float TrX;
    /// <summary>아이템 bbox 좌하단이 놓일 y (mm)</summary>
    public float TrY;
    public float RotAngleInRad;
    public float StripeIntervalY;
    public float StripeIntervalX;
}

/// <summary>아이템 바운딩 박스. Rust의 CItemBounds와 1:1 매핑.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct CItemBounds
{
    public int ItemId;
    public float MinX;
    public float MinY;
    public float MaxX;
    public float MaxY;
}

/// <summary>Rust의 CCollisionStatus와 1:1로 매핑되는 충돌 상태.</summary>
[StructLayout(LayoutKind.Sequential)]
internal struct CCollisionStatus
{
    public UIntPtr ItemId;
    public byte Collision;
}

/// <summary>
/// Swarm Rust cdylib(cpswcore.dll / libcpswcore.so / libcpswcore.dylib)에 대한
/// P/Invoke 바인딩을 제공하는 내부 정적 클래스.
///
/// <para>주요 기능</para>
/// <list type="bullet">
///   <item>
///     <description>정적 생성자에서 <see cref="NativeLibrary.SetDllImportResolver"/>를
///     등록하여 "cpswcore"이라는 논리적 DLL 이름을 플랫폼별 실제 파일명으로
///     해석하고, Rust 빌드 출력 디렉터리(target/debug, target/release)를 포함한 여러
///     위치에서 DLL을 탐색한다.</description>
///   </item>
///   <item>
///     <description>Rust 측 C 구조체와 1:1로 매핑되는 <see cref="StructLayoutAttribute"/>
///     구조체들을 정의한다. 모든 필드는 <see cref="LayoutKind.Sequential"/>로 배치된다.</description>
///   </item>
///   <item>
///     <description>Rust cdylib가 C ABI(<see cref="CallingConvention.Cdecl"/>)로
///     노출하는 모든 함수에 대한 <see cref="DllImportAttribute"/> 선언을 포함한다.</description>
///   </item>
///   <item>
///     <description>콜백 델리게이트 <see cref="SolutionCallback"/>을 정의하여
///     최적화 진행 상황을 C# 측에서 실시간으로 수신할 수 있게 한다.</description>
///   </item>
/// </list>
///
/// <para>사용처</para>
/// <para>이 클래스는 internal로 선언되어 외부에서 직접 P/Invoke를 호출할 필요 없이
/// <see cref="SwarmOptimizer"/>가 내부적으로 사용한다.</para>
/// </summary>
internal static class SwarmNative
{
    /// <summary>DllImport에 사용할 DLL 이름. 플랫폼별 실제 파일명은 리졸버에서 결정된다.</summary>
    private const string DllName = "cpswcore";

    private const uint ExpectedAbiVersion = 20;
    private const NativeFeatureFlags RequiredNativeFeatures =
        NativeFeatureFlags.Simd |
        NativeFeatureFlags.Capmods |
        NativeFeatureFlags.Adaptive |
        NativeFeatureFlags.StructureFirst;

    private static readonly string[] RequiredExports =
    {
        "swarm_get_abi_info",
        "swarm_get_default_config",
        "swarm_free_string",
        "swarm_get_last_error_code",
        "swarm_get_last_error_message",
        "swarm_version",
        "swarm_set_callback",
        "swarm_get_collision_status_json",
        "swarm_get_collision_status",
        "swarm_free_collision_status",
        "swarm_init",
        "swarm_init_from_json",
        "swarm_init_phase",
        "swarm_alg_phase",
        "swarm_cp_phase",
        "swarm_refine_phase",
        "swarm_gp_phase",
        "swarm_get_structure_report_json",
        "swarm_get_solution_json",
        "swarm_request_stop",
        "swarm_cancel_init",
        "swarm_is_stop_requested",
        "swarm_set_shrink_step",
        "swarm_clear_shrink_step",
        "swarm_get_shrink_step",
        "swarm_set_handle_shrink_step",
        "swarm_clear_handle_shrink_step",
        "swarm_get_handle_shrink_step",
        "swarm_set_log_callback",
        "swarm_log_dropped_count",
        "swarm_request_stop_and_wait",
        "swarm_freeze_items",
        "swarm_find_bound_collisions",
        "swarm_free"
    };

    private static readonly object CompatibilityLock = new();
    private static volatile bool _CompatibilityValidated;

    [Flags]
    private enum NativeFeatureFlags : uint
    {
        None = 0,
        Simd = 1 << 0,
        Capmods = 1 << 1,
        Adaptive = 1 << 2,
        StructureFirst = 1 << 3,
        Jni = 1 << 4
    }

    // 로컬 네이티브 라이브러리를 우선하도록 리졸버를 등록하고 플랫폼별 파일 이름 처리를 추가.
    static SwarmNative()
    {
        try
        {
            NativeLibrary.SetDllImportResolver(Assembly.GetExecutingAssembly(), DllImportResolver);
        }
        catch
        {
            // Ignore on platforms where SetDllImportResolver might not be available
        }
    }

    /// <summary>
    /// 커스텀 DLL 임포트 리졸버.
    /// <see cref="DllName"/>을 플랫폼별 실제 파일명으로 해석하고,
    /// 아래 순서로 DLL을 탐색한다:
    ///
    /// <list type="number">
    ///   <item><description>어셈블리와 같은 디렉터리 (<c>AppContext.BaseDirectory</c>)</description></item>
    ///   <item><description>개발 checkout에서 상위 디렉터리의 <c>Cargo.toml</c>을 찾은 후
    ///     <c>target/release</c>, <c>target/dev-fast</c>, <c>target/debug</c> Cargo 출력을 검색</description></item>
    ///   <item><description>모든 후보가 실패하면 OS 기본 DLL 검색으로 위임한다.</description></item>
    /// </list>
    /// </summary>
    private static IntPtr DllImportResolver(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
    {
        if (!string.Equals(libraryName, DllName, StringComparison.OrdinalIgnoreCase))
        {
            return IntPtr.Zero;
        }

        string baseDir = AppContext.BaseDirectory ?? Path.GetDirectoryName(assembly.Location) ?? Directory.GetCurrentDirectory();
        string dllName = RuntimeInformation.IsOSPlatform(OSPlatform.Windows) ? "cpswcore.dll" :
                         RuntimeInformation.IsOSPlatform(OSPlatform.OSX) ? "libcpswcore.dylib" : "libcpswcore.so";
        var failures = new List<string>();

        foreach (string file in GetNativeLibraryCandidates(baseDir, assembly, dllName))
        {
            try
            {
                string fullPath = Path.GetFullPath(file);
                if (!File.Exists(fullPath))
                {
                    continue;
                }
                if (!NativeLibrary.TryLoad(fullPath, out var handle))
                {
                    failures.Add($"{fullPath}: native load failed");
                    continue;
                }

                if (HasRequiredExports(handle, out string missingExport))
                {
                    return handle;
                }

                NativeLibrary.Free(handle);
                failures.Add($"{fullPath}: missing export '{missingExport}'");
            }
            catch (Exception ex)
            {
                failures.Add($"{file}: {ex.Message}");
            }
        }

        try
        {
            // Preserve the runtime's assembly/deps.json-aware probing before
            // falling back to the process-wide OS search. A bad explicit
            // checkout candidate must not mask a valid packaged runtime asset.
            if (NativeLibrary.TryLoad(dllName, assembly, searchPath, out var fallbackHandle))
            {
                if (HasRequiredExports(fallbackHandle, out string missingExport))
                {
                    return fallbackHandle;
                }

                NativeLibrary.Free(fallbackHandle);
                failures.Add($"OS search result: missing export '{missingExport}'");
            }
        }
        catch (Exception ex)
        {
            failures.Add($"runtime search result: {ex.Message}");
        }

        try
        {
            if (NativeLibrary.TryLoad(dllName, out var fallbackHandle))
            {
                if (HasRequiredExports(fallbackHandle, out string missingExport))
                {
                    return fallbackHandle;
                }

                NativeLibrary.Free(fallbackHandle);
                failures.Add($"OS search result: missing export '{missingExport}'");
            }
        }
        catch (Exception ex)
        {
            failures.Add($"OS search result: {ex.Message}");
        }

        if (failures.Count > 0)
        {
            throw new DllNotFoundException(
                $"A native {dllName} was found but is incompatible with this managed client. " +
                $"Rebuild the Rust library with the standard feature set. {string.Join(" | ", failures)}");
        }

        return IntPtr.Zero;
    }

    private static List<string> GetNativeLibraryCandidates(string baseDir, Assembly assembly, string dllName)
    {
        var candidates = new List<string>();
        var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        static void AddCandidate(List<string> result, HashSet<string> known, string path)
        {
            string fullPath = Path.GetFullPath(path);
            if (known.Add(fullPath))
            {
                result.Add(fullPath);
            }
        }

        AddCandidate(candidates, seen, Path.Combine(baseDir, dllName));
        string assemblyDir = Path.GetDirectoryName(assembly.Location) ?? baseDir;
        AddCandidate(candidates, seen, Path.Combine(assemblyDir, dllName));

        for (DirectoryInfo? directory = new(baseDir); directory != null; directory = directory.Parent)
        {
            if (!File.Exists(Path.Combine(directory.FullName, "Cargo.toml")))
            {
                continue;
            }

            AddCandidate(candidates, seen, Path.Combine(directory.FullName, "target", "release", dllName));
            AddCandidate(candidates, seen, Path.Combine(directory.FullName, "target", "dev-fast", dllName));
            AddCandidate(candidates, seen, Path.Combine(directory.FullName, "target", "debug", dllName));
            break;
        }

        return candidates;
    }

    private static bool HasRequiredExports(IntPtr handle, out string missingExport)
    {
        foreach (string exportName in RequiredExports)
        {
            if (!NativeLibrary.TryGetExport(handle, exportName, out _))
            {
                missingExport = exportName;
                return false;
            }
        }

        missingExport = string.Empty;
        return true;
    }

    /// <summary>
    /// 관리 구조체를 네이티브에 넘기기 전 ABI 버전, 크기, 필수 feature를 한 번 검증한다.
    /// 구 ABI DLL은 <c>swarm_get_abi_info</c> export가 없으므로 명시적으로 차단된다.
    /// </summary>
    internal static void EnsureCompatible()
    {
        if (_CompatibilityValidated)
        {
            return;
        }

        lock (CompatibilityLock)
        {
            if (_CompatibilityValidated)
            {
                return;
            }

            int result = swarm_get_abi_info(out CSwarmAbiInfo abiInfo);
            if (result != 0)
            {
                throw new InvalidOperationException($"swarm_get_abi_info failed with code {result}.");
            }

            uint managedConfigSize = checked((uint)Marshal.SizeOf<CSwarmConfig>());
            uint managedStructureSize = checked((uint)Marshal.SizeOf<CStructureFirstConfig>());
            if (abiInfo.abi_version != ExpectedAbiVersion ||
                abiInfo.swarm_config_size != managedConfigSize ||
                abiInfo.structure_config_size != managedStructureSize)
            {
                throw new InvalidOperationException(
                    "Swarm native ABI mismatch. " +
                    $"managed ABI={ExpectedAbiVersion}, config={managedConfigSize}, structure={managedStructureSize}; " +
                    $"native ABI={abiInfo.abi_version}, config={abiInfo.swarm_config_size}, structure={abiInfo.structure_config_size}. " +
                    "Rebuild and redeploy cpswcore with this managed client.");
            }

            var nativeFeatures = (NativeFeatureFlags)abiInfo.feature_flags;
            var missingFeatures = RequiredNativeFeatures & ~nativeFeatures;
            if (missingFeatures != NativeFeatureFlags.None)
            {
                throw new InvalidOperationException(
                    $"Swarm native library is missing required features: {missingFeatures}. " +
                    $"Compiled features: {nativeFeatures}.");
            }

            IntPtr versionPointer = swarm_version();
            string? nativeVersion = versionPointer == IntPtr.Zero
                ? null
                : Marshal.PtrToStringUTF8(versionPointer);
            if (string.IsNullOrWhiteSpace(nativeVersion))
            {
                throw new InvalidOperationException("Swarm native library returned an empty product version.");
            }

            _CompatibilityValidated = true;
        }
    }

    // -- Structs

    /// <summary>Rust <c>CSwarmAbiInfo</c>와 1:1로 매핑되는 고정 크기 ABI handshake.</summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct CSwarmAbiInfo
    {
        public uint abi_version;
        public uint swarm_config_size;
        public uint structure_config_size;
        public uint feature_flags;
    }

    /// <summary>
    /// 중첩 분리기(Separator) 알고리즘의 설정.
    /// Rust 측 <c>SeparatorConfig</c> 구조체와 1:1 매핑.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CSeparatorConfig
    {
        /// <summary>개선 없이 포기할 최대 반복 횟수</summary>
        public int iter_no_imprv_limit;
        /// <summary>최대 스트라이크(실패한 분리 시도) 횟수</summary>
        public int strike_limit;
        /// <summary>병렬 워커 수</summary>
        public int n_workers;
        /// <summary>이동당 컨테이너 전체 랜덤 샘플 수</summary>
        public int n_container_samples;
        /// <summary>이동당 집중 랜덤 샘플 수</summary>
        public int n_focussed_samples;
        /// <summary>좌표 하강(Coordinate Descent) 개선 횟수</summary>
        public int n_coord_descents;

        /// <summary>스왑(교환) 이웃 연산 주기(내부 루프 iteration 단위). 0 = 비활성, 음수 = 기본값(20)</summary>
        public int swap_period;
        /// <summary>스왑 패스 1회당 시도 수. 0 이하 = 기본값(8)</summary>
        public int swap_attempts;
        /// <summary>스왑 파트너 면적비 하한. 0 이하 = 기본값(0.4)</summary>
        public float swap_area_ratio_min;
        /// <summary>스왑 파트너 면적비 상한. 하한보다 작으면 기본값(2.5)</summary>
        public float swap_area_ratio_max;
        /// <summary>컨테이너 전역 샘플의 좌측 편향 최대 강도. 0 = 균등 분포, 음수 = 기본값(2.0)</summary>
        public float left_bias_gain;
    }

    /// <summary>
    /// 탐색(Exploration) 단계의 설정.
    /// Rust 측 <c>ExplorationConfig</c> 구조체와 1:1 매핑.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CExplorationConfig
    {
        /// <summary>스트립 폭 축소 스텝 비율 (예: 0.001 = 0.1%)</summary>
        public float shrink_step;
        /// <summary>탐색 단계 시간 제한(초)</summary>
        public float time_limit_secs;
        /// <summary>최대 연속 실패 시도 횟수. 0 이하면 무제한(None)</summary>
        public int max_conseq_failed_attempts; // <= 0 = None
        /// <summary>솔루션 풀 분포의 표준편차</summary>
        public float solution_pool_distribution_stddev;
        /// <summary>"대형 아이템" 판정을 위한 볼록 껍질 면적 컷오프 백분위</summary>
        public float large_item_ch_area_cutoff_percentile;
        /// <summary>중첩 분리기 설정</summary>
        public CSeparatorConfig separator;

        // ── adaptive 전용 (adaptive feature 없이 빌드된 라이브러리에서는 무시됨) ──

        /// <summary>파괴-재생성(ruin &amp; recreate) 주기(disruption 횟수 단위). 0 = 비활성, 음수 = 기본값(3)</summary>
        public int rnr_period;
        /// <summary>파괴할 세로 밴드 폭(스트립 폭 대비 비율, 0~1). 0 이하 = 기본값(0.18)</summary>
        public float rnr_band_fraction;
        /// <summary>한 번에 파괴할 아이템 수 상한. 0 이하 = 기본값(40)</summary>
        public int rnr_max_items;
        /// <summary>밀도 기반 적응형 shrink의 적응 강도. 0 = 고정 스텝, 음수 = 기본값(4.0)</summary>
        public float adaptive_shrink_gain;
        /// <summary>적응형 shrink의 최대 배율 상한. 1 미만 = 기본값(5.0)</summary>
        public float adaptive_shrink_max_mult;
        /// <summary>폭 진동: 같은 폭 연속 실패 임계값. 0 = 비활성, 음수 = 기본값(4)</summary>
        public int osc_fail_threshold;
        /// <summary>폭 진동: 되돌릴 폭(base shrink_step 배수). 0 이하 = 기본값(3.0)</summary>
        public float osc_expand_steps;
        /// <summary>ALNS 연산자 가중치 지수 감쇠 계수(0~1). 범위 밖 = 기본값(0.85)</summary>
        public float alns_decay;
        /// <summary>ALNS 연산자 가중치 하한. 음수 = 기본값(0.1)</summary>
        public float alns_min_weight;
    }

    /// <summary>
    /// 압축(Compression) 단계의 설정.
    /// Rust 측 <c>CompressionConfig</c> 구조체와 1:1 매핑.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CCompressionConfig
    {
        /// <summary>최소 축소 비율</summary>
        public float shrink_range_min;
        /// <summary>최대 축소 비율</summary>
        public float shrink_range_max;
        /// <summary>압축 단계 시간 제한(초)</summary>
        public float time_limit_secs;
        /// <summary>
        /// 축소 감쇠 전략.
        /// 0 = <see cref="ShrinkDecayStrategy.TimeBased"/> (시간에 따라 선형 감쇠),
        /// 1 = <see cref="ShrinkDecayStrategy.FailureBased"/> (실패 시 고정 비율로 감쇠)
        /// </summary>
        public int shrink_decay_strategy; // 0 = TimeBased, 1 = FailureBased
        /// <summary>FailureBased 전략의 감쇠 비율</summary>
        public float shrink_decay_ratio;
        /// <summary>중첩 분리기 설정</summary>
        public CSeparatorConfig separator;
    }

    /// <summary>
    /// 후처리(Polish) 단계의 설정.
    /// Rust 측 <c>PolishConfig</c> 구조체와 1:1 매핑.
    /// <para>
    /// 활성 LP/slide 패스를 시간 예산 만료 또는 정지 요청까지 반복한다.
    /// 1) lp : 아이템별 평행이동과 strip 폭을 변수로 하는 선형계획을 풀어 큰 폭을 먼저 줄인다.
    /// 2) slide : LP 뒤 남은 작은 틈을 왼쪽(옵션에 따라 아래쪽) 접촉까지 민다.
    /// hole fill 필드는 ABI 호환을 위해 보존되며 동작하지 않는다.
    /// </para>
    /// <para>
    /// 두 패스 모두 이동 전에 충돌을 검증하므로 결과가 이전보다 나빠지지 않으며,
    /// 고정(freeze)된 아이템은 이동하지 않는다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CPolishConfig
    {
        /// <summary>후처리 단계 사용 여부. 0이면 비활성.</summary>
        public int enabled;
        /// <summary>후처리 단계 전체 시간 제한(초)</summary>
        public float time_limit_secs;
        /// <summary>호환용 필드. 활성 LP/slide 반복은 시간 예산 또는 정지 요청으로 끝난다.</summary>
        public int n_rounds;

        /// <summary>좌측 슬라이딩 패스 사용 여부. 0이면 비활성.</summary>
        public int slide_enabled;
        /// <summary>한 라운드 안에서 슬라이딩을 반복할 횟수</summary>
        public int slide_n_passes;
        /// <summary>하단 방향 슬라이딩도 수행할지. 0이면 좌측 방향만.</summary>
        public int slide_down;
        /// <summary>접촉 시 남겨둘 안전 간격(아이템 최소 변 길이 대비 비율)</summary>
        public float slide_safety_ratio;

        /// <summary>빈 포켓 채우기 패스 사용 여부. 0이면 비활성.</summary>
        public int hole_fill_enabled;
        /// <summary>점유 격자 셀 크기(가장 작은 아이템 최소 변 길이 대비 비율)</summary>
        public float hole_fill_cell_size_ratio;
        /// <summary>재배치 후보 아이템 수 상한</summary>
        public int hole_fill_max_candidates;
        /// <summary>후보 선정 구간: strip 우측 몇 %를 대상으로 할지(0.0~1.0)</summary>
        public float hole_fill_candidate_band;
        /// <summary>포켓 하나당 평가할 랜덤 샘플 수</summary>
        public int hole_fill_n_samples;

        /// <summary>LP compaction 패스 사용 여부. 0이면 비활성.</summary>
        public int lp_enabled;
        /// <summary>LP 수립 → 적용 반복 횟수</summary>
        public int lp_n_iterations;
        /// <summary>신뢰 영역의 기본(가장 촘촘한) 단계 크기(strip 폭 대비 비율). 개선이 막히면 2배씩 확대된다.</summary>
        public float lp_trust_region_ratio;
        /// <summary>좌측으로 당기는 목적항 가중치</summary>
        public float lp_left_pull_weight;
    }

    /// <summary>
    /// P0-P5 structure-first 대체 파이프라인 설정.
    /// 공개 PlaceGroup과 달리 P0 motif는 해체 가능한 내부 힌트이다.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CStructureFirstConfig
    {
        public int enabled;
        public int motif_enabled;
        public int max_active_motifs;
        public int construction_starts;
        public float profile_row_height_ratio;
        public int profile_top_k;

        public float p2_time_limit_secs;
        public float p2_shrink_range_min;
        public float p2_shrink_range_max;
        public float p2_failure_decay;

        public float p3_slack_ratio;
        public float p3_time_limit_secs;

        public int p4_cycles;
        public float p4_cycle_time_limit_secs;
        public float p4_ruin_fraction;
        public int p4_max_items;
        public int p4_stagnation_cycles;

        public int p5_snap_rotations;

        public float p1_time_limit_secs;
        public int profile_max_rows;
        public int item_beam_width;
        public int candidate_pool_cap;
        public int slide_refine_steps;
        public int p1_include_legacy_baseline;
        public int pair_max_offset_candidates;
        public int pair_max_orientation_work;
        public int diagnostics_max_raster_cells;
        public int diagnostics_max_pair_comparisons;
        /// <summary>Structure 중간 결과 간격(ms). 500 미만은 네이티브에서 500으로 보정된다.</summary>
        public int progress_callback_interval_ms;
    }

    /// <summary>
    /// 진행 callback JSON에 포함할 선택적 진단 설정입니다.
    /// Solver 내부 충돌 처리와 exact 검증에는 영향을 주지 않습니다.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CCallbackOptionsConfig
    {
        /// <summary>0이면 callback용 loss 계산과 필드 출력을 생략합니다.</summary>
        public int collect_loss;
        /// <summary>0이면 callback용 아이템별 충돌 계산과 필드 출력을 생략합니다.</summary>
        public int collect_collision_status;
    }

    /// <summary>
    /// Swarm 최적화기 전체 설정.
    /// Rust 측 <c>SwarmConfig</c> 구조체와 1:1 매핑.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CSwarmConfig
    {
        /// <summary>난수 생성기 시드. 0이면 무작위 시드를 사용한다.</summary>
        public ulong rng_seed; // 0 = random
        /// <summary>다각형 단순화 허용 오차. 음수이면 비활성(None).</summary>
        public float poly_simpl_tolerance; // negative = None
        /// <summary>아이템 간 최소 간격. 음수이면 비활성(None).</summary>
        public float min_item_separation;  // negative = None
        /// <summary>좁은 오목(Narrow Concavity) 컷오프 거리. 음수이면 비활성(None).</summary>
        public float narrow_concavity_cutoff_dist; // negative = None
        /// <summary>좁은 오목(Narrow Concavity) 컷오프 면적. 음수이면 비활성(None).</summary>
        public float narrow_concavity_cutoff_area; // negative = None
        /// <summary>LBF 초기 배치 X축 bias multiplier. 기본값 10.0</summary>
        public float lbf_x_multiplier;
        /// <summary>LBF 초기 배치 Y축 bias multiplier. 기본값 1.0</summary>
        public float lbf_y_multiplier;
        /// <summary>탐색 단계 설정</summary>
        public CExplorationConfig exploration;
        /// <summary>압축 단계 설정</summary>
        public CCompressionConfig compression;
        /// <summary>후처리 단계 설정</summary>
        public CPolishConfig polish;
        /// <summary>LBF 래스터 NFP 설정 (capmods 빌드 전용)</summary>
        public CRasterNfpConfig lbf_raster;
        /// <summary>0 = 비활성(기본값). 그 외 = 각 아이템의 허용 회전에 180° 반전을 추가한다. 결방향 없는(non-nap) 원단 전용.</summary>
        public int allow_180_rotation;
        /// <summary>discrete 회전 아이템에 허용하는 소각도 틸트 (도 단위, ±값). 0 이하 = 비활성(기본값). 결방향 원단에서는 0으로 둘 것.</summary>
        public float tilt_range_deg;
        /// <summary>ABI-tail P0-P5 structure-first 설정. 기본값은 비활성.</summary>
        public CStructureFirstConfig structure_first;
        /// <summary>ABI-tail callback payload 진단 설정.</summary>
        public CCallbackOptionsConfig callback_options;
        /// <summary>ABI-tail N06 초기배치(점진 축소·분리) 설정. 기본값은 비활성.</summary>
        public CInitialPlacementConfig initial_placement;
        /// <summary>ABI-tail 폭 완화 복구 설정. 기본값은 비활성.</summary>
        public CWidthReliefConfig width_relief;
        /// <summary>ABI-tail 축소 스텝 실패 피드백 설정. 기본값은 비활성.</summary>
        public CShrinkFeedbackConfig shrink_feedback;
        /// <summary>ABI-tail 정체 감지 기반 적응형 국소 최적해 탈출 설정. 기본값은 비활성.</summary>
        public CEscapeConfig escape;
        /// <summary>ABI-tail NFP 기반 초기 배치(NFP-LBF) 시드 설정. 기본값은 비활성.</summary>
        public CNfpLbfConfig nfp_lbf;

        /// <summary>ABI-tail 원단 허용오차 설정. 기본값은 비활성.</summary>
        public CFabricToleranceConfig fabric_tolerance;

        /// <summary>ABI-tail 탐색 단계 NFP 활용 설정. 기본값은 전부 비활성.</summary>
        public CNfpIntegrationConfig nfp_integration;

        /// <summary>ABI-tail NFP-LBF 탐침 예산 튜닝. 기본값은 네이티브 기본치(4096).</summary>
        public CNfpLbfTuningConfig nfp_lbf_tuning;

        /// <summary>ABI-tail NFP 초기배치 전략 7종. 기본값은 전부 비활성.</summary>
        public CNfpStrategyConfig nfp_strategy;

        /// <summary>ABI-tail LP·QP 솔버 선택. 기본값은 기존 동작(단체법, 순수 LP).</summary>
        public CLpSolverConfig lp_solver;

        /// <summary>ABI-tail 초기배치 전략 ⑧·⑨ (ABI v11). 기본값은 둘 다 비활성.</summary>
        public CNfpStrategy2Config nfp_strategy2;

        /// <summary>ABI-tail 레퍼런스 설계 초기배치 엔진 (ABI v12). <b>실험</b>, 기본 비활성.</summary>
        public CRefPlaceConfig refplace;

        /// <summary>ABI-tail LBF 배치 순서 (ABI v13). 기본 0 = 현행 동작(큰 것부터).</summary>
        public CLbfOrderConfig lbf_order;

        /// <summary>ABI-tail 홀 우선 부착 (ABI v14). 기본 0 = 꺼짐.</summary>
        public CHoleFirstConfig hole_first;

        /// <summary>ABI-tail 잔여 예산 회수 (ABI v15). <b>기본 켜짐.</b>
        /// zeroed 버퍼를 넘기는 구 호스트는 0(꺼짐)을 받아 예전 동작 그대로다.</summary>
        public CReclaimConfig reclaim;
        /// <summary>ABI v16 tail: Explore stagnation repair; disabled by default.</summary>
        public CSeparationRepairConfig repair;
        /// <summary>ABI v17 tail: 구간(밴드) 통째 재배열. 전부 0 = 꺼짐이 기본.</summary>
        public CBandRearrangeConfig band_rearrange;
        /// <summary>ABI v17 tail: 원단 경계 여유. 0 = 제한 없음이 기본.</summary>
        public CFabricEdgeConfig fabric_edge;
        /// <summary>ABI v17 tail: GLS 가중치 학습률. 전부 0 = 네이티브 상수가 기본.</summary>
        public CGlsWeightConfig gls;
        /// <summary>ABI v17 tail: 구성기 배치 규칙. 0 = 기존 좌·하단 규칙이 기본.</summary>
        public CConstructorConfig constructor;
        /// <summary>ABI v18 tail: 탐색 정체 대응(재출발 창·R&amp;R 원위치·탈출 창). 전부 0 = v17 동작.</summary>
        public CSearchTuningConfig search;
        /// <summary>ABI v19 tail: 원본 외곽선 겹침 초과 처리. 0 = 실현 불가능(기존 동작).
        /// 정렬 여유(오프셋 948)에 들어가 구조체 크기(952)는 그대로다.</summary>
        public COriginalOverlapConfig original_overlap;
        /// <summary>ABI v20 tail: place group 처리 방식. 0 = 다중 폴리곤 아이템(기본), 1 = 강체 이동.
        /// 구조체 크기가 960 으로 늘어난다(8바이트 정렬).</summary>
        public CPlaceGroupConfig place_group;
    }

    /// <summary>
    /// V2 place groups(패턴 그룹) 처리 방식. Rust <c>CPlaceGroupConfig</c>와 1:1 (ABI v20).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CPlaceGroupConfig
    {
        /// <summary>0 = 다중 폴리곤 아이템(기본): 그룹을 멤버 외곽선을 합친 아이템 하나로 넘긴다.
        /// 1 = 강체 이동: 멤버를 별도 조각으로 두고 함께 옮긴다(기존 동작). 그 밖의 값은 0.</summary>
        public int mode;
    }

    /// <summary>
    /// 원본 외곽선 겹침이 겹침 허용값(<c>piece_overlap_mm</c>)을 넘는 배치의 처리.
    /// Rust <c>COriginalOverlapConfig</c>와 1:1 (ABI v19).
    /// <para>탐색은 조각을 허용값의 절반만큼 줄인 도형으로 하고, 결과를 원본 외곽선으로 다시 검증한다.
    /// 날카로운 꼭짓점은 더 깎이므로 원본 외곽선이 허용값보다 깊이 겹칠 수 있다.</para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct COriginalOverlapConfig
    {
        /// <summary>0 = 실현 불가능(기본): 그런 배치를 버린다. 1 = 실현 가능: 원본 외곽선 검증을 끄고 채택한다.
        /// 그 밖의 값은 0 으로 본다.</summary>
        public int excess_policy;
    }

    /// <summary>
    /// 탐색 단계 정체 대응 노브. Rust <c>CSearchTuningConfig</c>와 1:1 (ABI v18).
    /// <para>전부 0 이면 네이티브 기본값(= v17 동작)이다.</para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CSearchTuningConfig
    {
        /// <summary>Explore 분리기의 정체 재출발 창(라운드). 0 이하 = 꺼짐.
        /// <para>한 strike 안에서 이 라운드 수 동안 개선이 없으면 인컴번트로 되돌아가 다른
        /// 난수 궤적으로 계속한다. strike 판정과 포기 시점은 바뀌지 않는다.</para></summary>
        public int sep_restart_window;
        /// <summary>Explore 분리기 iter_no_imprv_limit 의 capmods 하한(아이템 수 x 2.5)을 덮어쓴다. 0 이하 = 종전.</summary>
        public int sep_iter_floor;
        /// <summary>Compress 분리기의 정체 재출발 창. 0 이하 = 꺼짐.</summary>
        public int cmpr_restart_window;
        /// <summary>예약(RESERVED) — 이 빌드의 코어는 아직 소비하지 않는다(미구현).
        /// 의도: 0 = 꺼짐. 그 외 = R&amp;R 재삽입에서 원위치를 후보로 포함한다.</summary>
        public int rnr_original_candidate;
        /// <summary>0 = 종전. 그 외 = 탈출 파이프라인의 정체 창을 window_floor 절대값으로 쓴다.</summary>
        public int escape_window_absolute;
        /// <summary>Explore 루프의 분리 1회 벽시계 상한(ms). 0 = 제한 없음.
        /// 상한이 오면 그 폭을 못 푼다고 보고 실패 분기(풀·교란·진동)로 넘긴다 — 막힌 폭 건너뛰기.
        /// <para>-1 = 적응형 상한: 이 실행에서 가장 오래 걸린 성공 분리의 1.25배를 상한으로 쓴다.
        /// 그 밖의 음수는 제한 없음.</para></summary>
        public int sep_time_cap_ms;
        /// <summary>예약(RESERVED) — 이 빌드의 코어는 아직 소비하지 않는다(미구현).
        /// 의도: 분리 실패 뒤 탐침할 교란 후보 수. 1 이하 = 종전(후보 1개).</summary>
        public int portfolio_k;
        /// <summary>예약(RESERVED) — 이 빌드의 코어는 아직 소비하지 않는다(미구현).
        /// 의도: 교란 후보 하나의 탐침 예산(ms). 0 이하 = 기본 2000.</summary>
        public int probe_budget_ms;
    }

    /// <summary>
    /// 초기 구성기의 배치 규칙. Rust <c>CConstructorConfig</c>와 1:1 (ABI v17).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CConstructorConfig
    {
        /// <summary>0 = 꺼짐(기본, 좌·하단 규칙). 그 외 = 위·아래 양쪽 모서리 앵커링.
        /// <para>
        /// 기본 규칙은 y 가 작을수록 좋아서 조각이 아래부터만 쌓인다. 실측으로 하단
        /// 접촉 16개 대 상단 6개였다. 켜면 y 항이 가까운 가로 모서리까지의 거리가 된다.
        /// </para></summary>
        public int double_sided_first_n;
        /// <summary>구성 도중 몇 조각마다 hole fill + LP 압축으로 다질지. 0 이하 = 하지 않음.</summary>
        public int settle_every;
        /// <summary>구성 도중 다지기 한 번의 예산(ms). 0 이하 = 기본값 200.</summary>
        public int settle_budget_ms;
        /// <summary>구성 직후 한 번만 다질 예산(ms). 0 이하 = 하지 않음.</summary>
        public int post_init_compaction_ms;
        /// <summary>구간 완성 폭(mm). 0 이하 = 끔. 현재 세로 구간이 다 찰 때까지
        /// 프론티어를 오른쪽으로 밀지 않는다.</summary>
        public float band_width_mm;
        /// <summary>NFP 구성형 배치에서 가중합 점수를 1순위로 쓸지. 0 = 끔.
        /// <para>기본 선택은 strip length → x → y → 접촉의 사전식 순서라, x 가 연속값이라
        /// 동률이 거의 없어 접촉·낭비 가중치가 사실상 아무것도 결정하지 못한다.</para></summary>
        public int nfp_score_first;
        /// <summary>낭비 면적 가중치 w2. 음수 = 기본값 0.35.</summary>
        public float nfp_w2_waste;
        /// <summary>접촉 길이 가중치 w3. 음수 = 기본값 0.25.</summary>
        public float nfp_w3_contact;
        /// <summary>세로 위치 가중치 w4. 음수 = 기본값 0.10.</summary>
        public float nfp_w4_y;
        /// <summary>길이 동률 허용치(mm). 0 이하 = 끔. strip 길이 차이가 이 값 이하인
        /// 후보들만 동률로 보고 그 안에서 맞물림 품질로 고른다.</summary>
        public float nfp_length_tie_mm;
        /// <summary>구간 경계에서 목록 뒤쪽의 작은 조각까지 훑어 구간을 채울 때의
        /// 면적 상한(가장 큰 조각 대비 비율). 0 이하 = 훑지 않음.</summary>
        public float band_fill_ratio;
    }

    /// <summary>
    /// GLS(Guided Local Search) 쌍 가중치의 학습률과 상한. Rust <c>CGlsWeightConfig</c>와 1:1 (ABI v17).
    /// <para>
    /// 기존 상수(증가 1.2~2.0, 상한 1e6, 감쇠 0.95)에서는 충돌이 지속되는 쌍의 가중치가
    /// 20~76 라운드 만에 상한에 닿는다. 벽에서의 분리 한 번은 천 라운드를 넘기므로, 그
    /// 뒤로는 어려운 쌍이 전부 1e6 이고 한 번도 충돌한 적 없는 쌍은 1.0 이라 GLS 의
    /// 변별력이 사라진다.
    /// </para>
    /// <para>전부 0 이면 네이티브 상수를 그대로 쓴다.</para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CGlsWeightConfig
    {
        /// <summary>충돌 중인 쌍의 최소 증가율. 1 이하 = 기본값 1.2.</summary>
        public float min_inc;
        /// <summary>충돌 중인 쌍의 최대 증가율. min_inc 미만 = 기본값 2.0.</summary>
        public float max_inc;
        /// <summary>충돌하지 않는 쌍의 감쇠율. 0 이하 또는 1 초과 = 기본값 0.95.</summary>
        public float decay;
        /// <summary>가중치 상한. 1 이하 = 기본값 1e6.</summary>
        public float max_weight;
    }

    /// <summary>
    /// 원본 외곽선이 원단 밖으로 나가는 양을 제한한다. Rust <c>CFabricEdgeConfig</c>와 1:1 (ABI v17).
    /// <para>
    /// 겹침 <c>d</c> 를 허용하면 조각의 충돌 도형이 <c>d/2</c> 만큼 침식되고 컨테이너도 같은
    /// 양만큼 줄어든다. 그래도 원본 외곽선은 밖으로 나간다 — 침식이 변을 <c>r</c> 만큼
    /// 들이는 동안 볼록 꼭짓점은 <c>r/sin(θ/2)</c> 만큼 들이기 때문이다. 초과량은
    /// <c>r·(1/sin(θ/2) − 1)</c> 이고 각이 날카로울수록 한없이 커진다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CFabricEdgeConfig
    {
        /// <summary>원본 외곽선이 원단 밖으로 넘어도 되는 최대 거리(mm).
        /// 0 이하 = 제한 없음(기본). 값을 주면 컨테이너를 그만큼 더 줄인다.</summary>
        public float margin_mm;
    }

    /// <summary>
    /// 구간(밴드)을 통째로 다시 배열하는 연산자. Rust <c>CBandRearrangeConfig</c>와 1:1 (ABI v17).
    /// <para>
    /// 기존 Ruin &amp; Recreate 는 ALNS disruption 경로에 있고, 그 경로는 <b>분리 실패</b>
    /// 뒤에만 돈다. 여성WSjk 156-패널 실측(Explore 480초, 3시드)에서 분리는 13~14회
    /// 성공하고 4회만 실패하므로 R&amp;R 전체가 실행당 1~6회밖에 돌지 않는다.
    /// <c>post_shrink</c> 를 켜면 같은 연산자가 축소에 성공할 때마다 돌아 실행 횟수가
    /// 성공 횟수(13~14회)까지 올라간다.
    /// </para>
    /// <para>전부 0 이면 v16 과 동작이 한 비트도 다르지 않다.</para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CBandRearrangeConfig
    {
        /// <summary>0 = 꺼짐(기본). 그 외 = 축소 성공 직후 우측 밴드를 통째로 다시 배열.</summary>
        public int post_shrink;
        /// <summary>0 = 꺼짐(기본). 그 외 = 밴드 안 이동 가능 조각을 전부 파괴
        /// (<c>rnr_max_items</c> 상한과 무작위 truncate 를 건너뛴다).</summary>
        public int full_band;
        /// <summary>재삽입 LBF 평가기의 x 편향. 0 이하 = 기본값 10.0.</summary>
        public float recreate_x_mult;
        /// <summary>재삽입 LBF 평가기의 y 편향. 0 이하 = 기본값 1.0.
        /// 둘 다 양수일 때만 반영된다. 예: (1, 10) 은 아래부터 채운다.</summary>
        public float recreate_y_mult;
        /// <summary>재배열을 유지할 최소 "빈자리 착석률" (0~1). 0 이하 = 검사 안 함.
        /// <para>
        /// 충돌 없는 자리에 앉은 조각 비율이 이 값 미만이면 재배열 전체를 롤백한다.
        /// 배치가 성길 때는 통과하고 조밀해지면 저절로 꺼지는 자동 스위치다.
        /// </para></summary>
        public float min_clear_reinsert_ratio;
        /// <summary>ABI v18 tail: 0 = 꺼짐(기본). 그 외 = 밴드 시작점 후보에
        /// <b>실제 빈틈(포켓)</b> 위치를 면적 내림차순으로 추가한다.
        /// <para>
        /// 종전 후보는 8개 균등 간격뿐이라 1524mm 원단에서 간격이 약 190mm 였다.
        /// 조각 하나 크기의 빈틈은 두 후보 사이에 완전히 빠진다 —
        /// <c>LowDensity</c>/<c>Fragmented</c> 가 «가장 빈 밴드» 를 골라도 그 밴드가
        /// 최대 빈틈을 담고 있으리라는 보장이 없었던 이유다.
        /// </para></summary>
        public int pocket_band;
        /// <summary><c>pocket_band</c> 로 추가할 포켓 개수 상한. 0 이하 = 기본값 24.</summary>
        public int pocket_band_max;
        /// <summary>빈틈 격자 셀 크기 배율. 0 이하 = 1.0(종전 동작). 작을수록 촘촘해져
        /// 최소 조각의 절반보다 얇은 좁은 포켓이 잡힌다.</summary>
        public float pocket_grid_cell_scale;
        /// <summary>ABI v18 tail: 축소 직후 재배열의 loss 악화 허용 배율.
        /// 0 이하 = 검사 안 함(기본). 재배열 후 loss 가 <c>전 × 이 배율</c> 을 넘으면 롤백.
        /// <para>
        /// 실측: <c>full_band=1</c> 로 52개를 부수고 재삽입하면 loss 가 4.6~10.5배로
        /// 폭증하는데 <c>min_clear_reinsert_ratio</c> 가드는 발동하지 않았다
        /// (조각들이 «충돌 없이» 앉았기 때문). 악화 원인은 폴백이 아니라 재배열 자체다.
        /// </para></summary>
        public float max_loss_growth;
        /// <summary>ABI v18 tail: 축소 직후 재배열의 파괴 전략.
        /// 0=Rightmost(기본) 1=LowDensity 2=Fragmented 3=GapTarget 4=DensitySeam 5=Random.
        /// <para>
        /// 실측: <c>Rightmost</c> 는 packing frontier(=가장 잘 맞춰진 우측 구간)를
        /// 통째로 재배열해 loss 를 3.7~9.1배로 폭증시킨다. <c>LowDensity</c>/
        /// <c>Fragmented</c> 는 «비어 있는 곳» 을 고르므로 frontier 를 안 건드린다.
        /// </para></summary>
        public int post_strategy;
    }

    /// <summary>
    /// Explore 분리 정체 복구. Rust <c>CSeparationRepairConfig</c>와 1:1 (ABI v16).
    /// 각 항목의 의미는 <see cref="SeparationRepairConfig"/> 참조.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CSeparationRepairConfig
    {
        /// <summary>접촉 방향 사용 여부. 0 = 꺼짐(기본).</summary>
        public int direction_enabled;
        /// <summary>작은 집단 LP 사용 여부. 0 = 꺼짐(기본).</summary>
        public int cluster_lp_enabled;
        /// <summary>차단 연쇄 사용 여부. 0 = 꺼짐(기본).</summary>
        public int chain_enabled;
        /// <summary>복구를 켜는 정체 반복 수. 1~10000, 기본 24.</summary>
        public uint stagnation_iterations;
        /// <summary>분리 한 번당 복구 패스 수 상한. 1~8, 기본 2.</summary>
        public uint max_passes;
        /// <summary>이동 단위 수 상한. 2~8, 기본 4.</summary>
        public uint max_units;
        /// <summary>패스 하나의 후보 평가 수 상한. 1~256, 기본 64.</summary>
        public uint max_evaluations;
        /// <summary>패스 하나의 벽시계 예산(ms). 1~200, 기본 20.</summary>
        public uint time_budget_ms;
        /// <summary>단위당 최대 이동 거리(mm). 0 초과 100 이하, 기본 2.</summary>
        public float max_move_mm;
    }

    /// <summary>
    /// 폴리시 잔여 예산을 압축 재진입으로 회수한다. Rust <c>CReclaimConfig</c>와 1:1 (ABI v15).
    /// <para>
    /// 폴리시는 시계가 아니라 <b>고정점</b>에서 끝난다 — 실측(52조각 OPS2 마커)에서
    /// 60초 예산 중 2.4~7.4초만 쓰고, 나머지는 버려진다. 켜면 남은 시간 동안
    /// 압축↔폴리시를 반복한다.
    /// </para>
    /// <para>
    /// <b>총 소요 시간은 늘지 않는다.</b> 마감이 폴리시가 이미 받은 예산의 끝이다.
    /// 폴리시가 예산을 다 쓰는 인스턴스(156조각 마커: 97~100%)에서는 자동 no-op.
    /// </para>
    /// <para>
    /// 실측 4회: −17.11 / +0.00 / +0.00 / −8.00 mm. 적중 2/4, 평균 −6.28mm.
    /// <b>한 번도 나빠지지 않는다</b> — 폭이 실제로 줄었을 때만 교체한다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CReclaimConfig
    {
        /// <summary>0 = 꺼짐, 그 외 = 켬. 네이티브 기본값은 <b>1</b>.</summary>
        public int enabled;
        /// <summary>한 사이클에서 압축이 받을 몫 (0.0~1.0). 0 이하면 기본값 0.75.</summary>
        public float compress_share;
        /// <summary>
        /// 사이클 수 상한. 0 = 예산까지.
        /// <b>재현성이 필요하면 고정한다</b> — 사이클 수가 실측 잔여 시간의 함수라
        /// 기계 속도에 따라 결과가 달라진다.
        /// </summary>
        public int max_cycles;
        /// <summary>남은 시간이 이보다 적으면 시작하지 않는다(ms). 0 이하면 기본값 500.</summary>
        public int min_slice_ms;
        /// <summary>
        /// 잔여를 무엇에 쓸지. <b>0 = 압축 재진입(기본)</b>, 1 = R&amp;R 교란+재탐색,
        /// 2 = 적응형 탈출 탐색. 범위 밖은 0.
        /// </summary>
        public int mode;
        /// <summary>탈출 계열에서 부술 밴드 폭 (strip 폭 대비). 0 이하면 기본값 0.20.</summary>
        public float ruin_band_fraction;
        /// <summary>탈출 계열에서 부술 조각 수 상한. 0 이하면 기본값 48.</summary>
        public int ruin_max_items;
        /// <summary>
        /// <b>탈출 계열에서</b> 개선 없는 사이클을 몇 번까지 참을지. 0 이하면 기본값 2.
        /// 압축 재진입은 결정론적이라 이 값과 무관하게 첫 실패에서 멈춘다.
        /// </summary>
        public int stagnation_cycles;
    }

    /// <summary>
    /// 초기배치의 <b>홀 우선 부착</b>. Rust <c>CHoleFirstConfig</c>와 1:1 (ABI v14).
    /// <para>
    /// 래스터 격자 전수 스캔은 이미 배치된 조각들에 둘러싸인 빈 주머니(포켓) 안의
    /// 위치도 만들어 낸다. 기존 LBF 는 좌·하단 손실이 최소인 후보 하나만 고르므로
    /// x 가 큰 포켓 후보는 언제나 진다. 켜면 포켓 구간을 먼저 따로 평가하고,
    /// CDE 를 통과하면 그것을 채택한다.
    /// </para>
    /// <para>
    /// 실측(LBF 단독, seed 0, share 0.5): wsjk156 폭 −0.73% / 밀도 +0.52%p,
    /// swim −0.09% / +0.06%p, shirts <b>+0.80% / −0.62%p</b>(나빠짐).
    /// 인스턴스에 따라 갈리므로 기본은 꺼짐이다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CHoleFirstConfig
    {
        /// <summary>0 = 꺼짐(기본), 그 외 = 켬.</summary>
        public int enabled;
        /// <summary>포켓 후보에 내줄 후보 슬롯 비율 (0.0~1.0). 0 이하면 기본값 0.5.</summary>
        public float share;
        /// <summary>예약 필드. 구조체 크기를 16B 로 유지한다.</summary>
        public int _reserved0;
        /// <summary>예약 필드.</summary>
        public int _reserved1;
    }

    /// <summary>
    /// LBF 초기배치의 순회 방향. Rust <c>CLbfOrderConfig</c>와 1:1 (ABI v13).
    /// <para>
    /// LBF 는 조각을 <c>볼록껍질 면적 × 직경</c> 내림차순으로 넣는다. 이 블록은 그
    /// <b>점수의 정의는 그대로 두고 순회 방향만</b> 바꾼다 — 그래야 A/B 에서
    /// "순서 효과" 와 "크기 정의 효과" 가 섞이지 않는다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CLbfOrderConfig
    {
        /// <summary>
        /// 0 = 큰 것부터(기본, 현행), 1 = 작은 것부터,
        /// 2 = 중간부터 큰 쪽 먼저, 3 = 중간부터 작은 쪽 먼저.
        /// 범위 밖 값은 코어가 0 으로 되돌린다.
        /// </summary>
        public int mode;
        /// <summary>예약 필드. 구조체 크기를 16B 로 유지한다.</summary>
        public int _reserved0;
        /// <summary>예약 필드.</summary>
        public int _reserved1;
        /// <summary>예약 필드.</summary>
        public int _reserved2;
    }

    /// <summary>
    /// 레퍼런스 설계 초기배치 엔진 스위치. Rust <c>CRefPlaceConfig</c>와 1:1 (ABI v12).
    /// <para>
    /// <b>실험이다.</b> 설계 문서를 그대로 구현한 별도 엔진이고 겹침 없는 유효한 배치를
    /// 내지만, 품질이 아직 기존 경로에 크게 못 미친다 (wsjk156 기준 25.04% vs 71.75%).
    /// 켜면 기존 초기 배치를 <b>대체</b>한다 — 시드로 비교해 좋은 쪽을 고르는 것이 아니다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CRefPlaceConfig
    {
        /// <summary>켜면 기존 초기 배치를 이 엔진으로 대체한다. 기본 0(꺼짐).</summary>
        public int enabled;
        /// <summary>
        /// 그룹 정의가 없을 때 쓸 대체 규칙. 0 = 조각 하나가 한 그룹, 1 = item_id 별 묶음.
        /// <para>
        /// 설계의 제1원칙이 "순서 = 그룹 정의 기반" 인데, 입력의 그룹 메타데이터
        /// (<c>domain</c> 블록)를 솔버 로더가 버리므로 지금은 이 대체 규칙만 쓸 수 있다.
        /// </para>
        /// </summary>
        public int group_fallback;
        /// <summary>기존 ABI 슬롯: 1 = Column Draft, 0 = 종전 설계 엔진.</summary>
        public int _reserved0;
        /// <summary>기존 예약 슬롯. 전역 단순화 최대 이탈(mm), 양수만 활성. enabled와 독립.</summary>
        public float simplify_max_deviation_mm;
    }

    /// <summary>
    /// NFP 초기배치 전략 ⑧ 누적 배치 / ⑨ 꼬리 회수. Rust <c>CNfpStrategy2Config</c>와
    /// 1:1 매핑 (ABI v11).
    /// <para>
    /// 기존 <c>CNfpStrategyConfig</c>(offset 616)를 늘리지 않고 <b>새 블록</b>으로 붙였다.
    /// 중간 블록을 늘리면 뒤따르는 <c>lp_solver</c>(664) 오프셋이 밀려 v10 호스트가
    /// 조용히 깨진다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CNfpStrategy2Config
    {
        /// <summary>
        /// ⑧ 배치 순서를 정하지 않고, 매 단계 남은 조각 전체 × 가능한 자리 전체에서
        /// 가장 잘 맞물리는 (조각, 자리) 한 쌍을 고른다.
        /// <para>
        /// ⑥ best-fit 과 다르다 — ⑥은 ordering 을 시작점으로 남기지만 ⑧은 ordering 을
        /// 아예 쓰지 않는다. 그래서 켜면 ordering 을 <b>1개만</b> 평가한다.
        /// 실측(wsjk156, 탐침 8192): 71.75% → 72.93%, ⑦ 슬라이딩을 얹으면 73.09%.
        /// 26초 걸리던 ②+③+④+⑦(72.18%)보다 좋으면서 훨씬 싸다.
        /// </para>
        /// </summary>
        public int accretion;
        /// <summary>
        /// ⑨ 오른쪽 끝(꼬리)의 조각을 빼서 앞쪽에 남은 빈 곳으로 되돌린다.
        /// <para>
        /// 그리디 배치는 앞을 조이고 뒤를 굶긴다. 경쟁사 마커와 200mm 구간별 충전율을
        /// 대조하면 앞 1/3 은 −1.7%p 인데 뒤 1/3 이 −15.4%p 로 벌어진다.
        /// ⑨는 정확히 그 자리를 친다. 실측(5초 예산): 71.75% → 73.27% (+1.52%p).
        /// </para>
        /// </summary>
        public int tail_reflow;
        /// <summary>⑨ 꼬리 회수 라운드 수. 0 이하 = 기본값(2).</summary>
        public int reflow_rounds;
        /// <summary>예약 필드. 구조체 크기를 4의 배수로 유지한다.</summary>
        public int _reserved;
    }

    /// <summary>
    /// NFP 초기배치 전략 7종. Rust <c>CNfpStrategyConfig</c>와 1:1 매핑 (ABI v10).
    /// <para>
    /// 전부 <c>0 = 끔</c>. 조절 노브는 0 이하면 네이티브 기본값으로 떨어진다.
    /// 각 전략은 파이프라인의 다른 지점에 붙으므로 대부분 함께 켤 수 있다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CNfpStrategyConfig
    {
        /// <summary>
        /// ① 회전 후보를 <c>allow_180</c> 플래그가 아니라 인스턴스 선언에서 그대로 뽑는다.
        /// <para>
        /// 기존 LBF 생성기는 이미 그렇게 한다. NFP-LBF 만 0도(또는 0/180도)로 묶여 있어
        /// 회전이 허용된 인스턴스에서 구조적으로 불리했다.
        /// 실측(albano, <c>allowed_orientations=[0,180]</c>): 끄면 LBF 대비 −1.51%p 로
        /// <b>지고</b>, 켜면 +8.57%p 로 이긴다. 회전 선언이 <c>[0]</c> 뿐인 인스턴스는 영향 없음.
        /// </para>
        /// </summary>
        public int instance_rotations;
        /// <summary>① 연속 회전을 몇 도 간격으로 샘플링할지. 0 이하 = 기본값(90).</summary>
        public int continuous_rotation_step_deg;
        /// <summary>
        /// ② 잘 맞물리는 조각 쌍을 하나의 super-piece 로 묶어 배치한다.
        /// 의류 마커의 좌/우 대칭 패널과 소매가 대상이다.
        /// </summary>
        public int pairwise_clustering;
        /// <summary>② 쌍을 채택할 최소 충전율 이득. 0 이하 = 기본값(0.05).</summary>
        public float cluster_min_gain;
        /// <summary>
        /// ③ 후보를 "가장 왼쪽" 이 아니라 "접촉 최대" 로 고른다 (Gomes &amp; Oliveira).
        /// <para>
        /// 기존 우선순위는 사실상 <c>argmin(x)</c> 로 붕괴한다 — 폭 W 가 상수라
        /// <c>strip_length = max(frontier, x+W)</c> 가 x 의 단조 증가 함수이고,
        /// 따라서 1순위가 2순위를 절대 뒤집지 못한다.
        /// 실측(탐침 8192): 72.08% → 72.48%. 탐침 예산이 낮으면 이득이 없다.
        /// </para>
        /// </summary>
        public int contact_maximization;
        /// <summary>④ 프론티어를 늘리기 전에 이미 배치된 영역 안쪽의 구멍부터 채운다.</summary>
        public int pocket_first;
        /// <summary>⑤ 부분 배치를 1개가 아니라 <c>beam_width</c> 개 유지한다.</summary>
        public int beam_search;
        /// <summary>⑤ 빔 폭. 0 이하 = 기본값(4). <b>시간이 이 값에 정비례한다.</b></summary>
        public int beam_width;
        /// <summary>⑥ 순서를 고정하지 않고 매 단계 가장 잘 맞는 조각을 고른다.</summary>
        public int best_fit_selection;
        /// <summary>⑥ 매 단계 평가할 조각 종류 수 상한. 0 이하 = 기본값(12).</summary>
        public int best_fit_lookahead;
        /// <summary>⑦ 배치 후 각 조각을 막힐 때까지 좌하단으로 민다.</summary>
        public int sliding_compaction;
        /// <summary>⑦ 슬라이딩 라운드 수. 0 이하 = 기본값(2).</summary>
        public int sliding_rounds;
    }

    /// <summary>
    /// 압축 단계 LP·QP 솔버 선택. Rust <c>CLpSolverConfig</c>와 1:1 매핑 (ABI v10).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CLpSolverConfig
    {
        /// <summary>
        /// 0 = microlp(단체법, 순수 Rust, 항상 사용 가능, LP 전용),
        /// 1 = clarabel(내점법, 순수 Rust, <b>QP 지원</b>),
        /// 2 = COIN-OR Clp(런타임에 <c>clp.dll</c> 을 찾는다, LP 전용),
        /// 3 = FICO Xpress(런타임에 <c>xprs.dll</c>, LP+QP, 라이선스 필요).
        /// 범위 밖이면 0.
        /// </summary>
        public int backend;
        /// <summary>
        /// 2차 정규화 계수. 0 이하면 순수 LP.
        /// <para>
        /// 순수 LP 는 목적이 폭 최소화 하나뿐이라 폭이 같은 해가 무수히 많고, 그중
        /// 아무거나 고르면 조각이 불필요하게 멀리 튄다. 2차항을 얹으면 <b>같은 폭이면
        /// 덜 움직이는</b> 해가 유일하게 뽑힌다. LP 전용 백엔드(0, 2)에서는 무시된다.
        /// </para>
        /// </summary>
        public float quadratic_regularization;
        /// <summary>
        /// 선택한 백엔드를 쓸 수 없을 때 microlp 로 물러날지. 0 이면 압축을 건너뛴다.
        /// 상용 솔버 결과를 재현해야 하는 검증 실행에서는 조용한 폴백이 해로우므로 끌 수 있다.
        /// </summary>
        public int fallback;
        /// <summary>한 번의 풀이에 허용할 시간(ms). 0 이하 = 기본값(5000).</summary>
        public int time_limit_ms;
    }

    /// <summary>
    /// NFP-LBF 순차 배치의 탐침 예산. Rust <c>CNfpLbfTuningConfig</c>와 1:1 매핑 (ABI v9).
    /// <para>
    /// <see cref="CNfpLbfConfig"/>를 늘리면 뒤따르는 tail block의 오프셋이 전부 밀려
    /// 기존 호스트가 깨지므로, 새 노브는 구조체 끝에 블록으로 덧붙인다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CNfpLbfTuningConfig
    {
        /// <summary>
        /// 아이템 하나당 feasible 판정을 시도할 raw 후보 수 상한. 0 이하 = 기본값(4096).
        /// <para>
        /// <b>초기 배치 품질을 좌우하는 값이다.</b> 후보 수는 배치가 진행될수록 폭증하는데
        /// (여성WSjk 156-패널: 20번째 1,607개 → 155번째 13,514개) 예산이 고정이면 후기
        /// 아이템이 후보의 7.5%만 보고 자리를 정한다.
        /// </para>
        /// <para>
        /// 실측 (여성WSjk 156-패널, ordering 50개, <b>오목 단순화 0.02</b>,
        /// LBF = 7325.05 / 69.45% — <b>단, 이 LBF 기준선은 벤치가 좌측편향 계수를
        /// 프로덕션(10.0)이 아닌 1.0 으로 만든 값이라 실제보다 나쁘다. 프로덕션 계수로
        /// 재면 70.61% 이므로 아래 델타는 약 1.16%p 부풀려져 있다</b>):
        /// 1024 → 7479.09/68.02% (LBF에 진다 → 시드 기각),
        /// 2048 → 7277.71/69.90%, 4096 → 7082.27/71.83% (기본값, 여기서 포화),
        /// 8192 → 7057.59/72.08%.
        /// </para>
        /// </summary>
        public int max_candidate_probes;
        /// <summary>예약. 반드시 0으로 둘 것.</summary>
        public int _reserved0;
        /// <summary>예약. 반드시 0으로 둘 것.</summary>
        public int _reserved1;
        /// <summary>예약. 반드시 0으로 둘 것.</summary>
        public int _reserved2;
    }

    /// <summary>
    /// 정체 감지 기반 적응형 국소 최적해 탈출. Rust <c>CEscapeConfig</c>와 1:1 매핑.
    /// <para>
    /// 최근 <c>max(window_floor, window_item_factor × 패턴 수)</c>회 동안 효율 개선이
    /// <c>efficiency_eps</c> 미만이고 후보 배치의 다양성이 <c>diversity_threshold</c>
    /// 미만이면 정체로 보고, 탐색 단계를 탈출 기반 경로로 교체한다.
    /// </para>
    /// <para>
    /// <c>enabled != 0</c>이어야 동작한다. 나머지 0/음수 필드는 네이티브에서
    /// 코어 기본값으로 보정된다 (각 필드 주석 참조).
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CEscapeConfig
    {
        /// <summary>0 = 비활성(기본값). 그 외 = 탈출 기반 탐색 단계를 사용한다.</summary>
        public int enabled;
        /// <summary>정체 관측 창의 하한. 0 이하 = 기본값(100).</summary>
        public int window_floor;
        /// <summary>정체 관측 창의 패턴 수 배수. 0 이하 = 기본값(3).</summary>
        public int window_item_factor;
        /// <summary>정체 판정 효율 개선 임계값. 0 이하 = 기본값(0.0001 = 0.01%p).</summary>
        public float efficiency_eps;
        /// <summary>후보 배치 다양성 임계값. 0 이하 = 기본값(0.05).</summary>
        public float diversity_threshold;
        /// <summary>0 = 비활성. 그 외 = Conflict Burst + Partial Rebuild 경로 사용(기본 켜짐).</summary>
        public int enable_conflict_burst;
        /// <summary>0 = 비활성. 그 외 = Hard 탈출에서 하위 후보해를 Elite 기준으로 재초기화(기본 켜짐).</summary>
        public int enable_swarm_diversification;
        /// <summary>Conflict Burst Stage 1에서 강제 분리할 아이템 수 상한. 0 이하 = 기본값(16).</summary>
        public int max_push_items;
        /// <summary>infeasible 해 풀의 크기 상한. 0 이하 = 기본값(32).</summary>
        public int max_infeasible_pool;
    }

    /// <summary>
    /// NFP 기반 초기 배치(NFP-LBF) 시드. Rust <c>CNfpLbfConfig</c>와 1:1 매핑.
    /// <para>
    /// NFP Cache → Multiple Item Ordering → NFP Sequential Placement → Best Initial Nest.
    /// <c>enabled != 0</c>이면 기존 LBF 생성기 대신 여기서 만든 최선 초기해를 탐색 단계의
    /// seed로 넣는다. 나머지 0/음수 필드는 네이티브에서 코어 기본값으로 보정된다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CNfpLbfConfig
    {
        /// <summary>0 = 비활성(기본값, 기존 LBF 생성기 유지). 그 외 = NFP-LBF 시드 사용.</summary>
        public int enabled;
        /// <summary>평가할 ordering 수. 0 이하 = 기본값(50).</summary>
        public int n_orderings;
        /// <summary>NFP 콜드 캐시 생성 예산(ms). 0 이하 = 기본값(5000).</summary>
        public int cache_budget_ms;
        /// <summary>초기 배치 탐색 전체 예산(ms). 0 이하 = 기본값(10000).</summary>
        public int total_budget_ms;
        /// <summary>0 = 0도 단일(기본값). 그 외 = 0/180도. 결방향 없는(non-nap) 원단 전용.</summary>
        public int allow_180;
        /// <summary>0 = 비활성. 그 외 = 앞부분이 같은 ordering의 배치를 재사용(기본 켜짐).</summary>
        public int enable_prefix_reuse;
        /// <summary>NFP 라이브러리 전용 오목 단순화 강도(원본 면적 대비 허용 증가 비율).
        /// 음수 = 기본값(0.0 = 비활성). 결과 도형은 원본의 상위집합이라 안전하다.</summary>
        public float concave_simplify_tolerance;
    }

    /// <summary>
    /// 원단 허용오차 — 조각 사이 겹침 (ABI v7 tail block).
    /// Rust <c>CFabricToleranceConfig</c>와 1:1 대응한다.
    /// <para>
    /// <c>min_item_separation</c>은 음수가 곧 겹침 허용이지만, C ABI에서는 그 필드의
    /// 음수가 이미 "설정 안 함"을 뜻하므로 겹침에 도달할 수 없다. 그래서 별도 필드로 뺀다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CFabricToleranceConfig
    {
        /// <summary>
        /// 조각 사이에 허용할 겹침(mm). 0 이하 = 비활성(기본값).
        /// 0보다 크면 이 값이 <c>min_item_separation</c>을 이긴다.
        /// 네이티브가 도형을 이 값의 절반씩 안쪽으로 줄이므로 CDE와 NFP가 같은 도형을 본다.
        /// </summary>
        public float piece_overlap_mm;
    }

    /// <summary>
    /// 탐색(exploration) 단계의 NFP 활용 범위 (ABI v8 tail block).
    /// <para>
    /// 기존 Swarm 탐색은 "이 조각이 어디로 갈 수 있나"를 난수 샘플링·bbox 근사·래스터 격자로
    /// 답한다. NFP는 정확히 답하지만 질의가 더 비싸다. 그래서 전부 기본 꺼짐이고, 항목마다
    /// 따로 켜서 밀도와 시간을 A/B로 재 봐야 한다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CNfpIntegrationConfig
    {
        /// <summary>탐색 단계를 순수 NFP 순서 탐색으로 교체. 0 = 끔. escape 보다 우선한다.</summary>
        public int ordering_exploration;
        /// <summary>① separator 위치 샘플러를 NFP 후보로 교체. 0 = 끔.</summary>
        public int nfp_sampler;
        /// <summary>② Conflict Burst 분리 벡터를 정확한 NFP 기반으로. 0 = 끔(bbox MTV 근사 유지).</summary>
        public int exact_separation_vector;
        /// <summary>③ hole filling 을 feasible_region 기반으로. 0 = 끔(래스터 격자 유지).</summary>
        public int nfp_hole_filling;
        /// <summary>④ R&amp;R 재삽입을 NFP 기반으로. 0 = 끔(샘플링 + CDE 유지).</summary>
        public int nfp_reinsertion;
    }

    /// <summary>
    /// 탐색 단계 축소 스텝의 폐루프 제어. Rust <c>CShrinkFeedbackConfig</c>와 1:1 매핑.
    /// <para>
    /// <c>adaptive_shrink_gain</c>은 밀도만 보는 개루프라 "이 스텝이 과했다"를 학습하지 못한다.
    /// 밀도는 폭이 줄면 단조 증가하므로 배율이 사실상 고정 스케줄이 된다. 이 블록은 압축 단계가
    /// 이미 갖고 있는 피드백을 탐색에도 넣는다 — 어떤 폭을 분리하지 못하면 스텝을 물리고,
    /// 새 feasible 폭에 도달하면 되돌려 받는다.
    /// </para>
    /// <para>
    /// 한 폭에서의 <b>첫</b> 실패만 센다. 같은 폭에서의 교란 재시도까지 세면 스텝이 순식간에
    /// 하한으로 무너진다. adaptive 빌드 전용.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CShrinkFeedbackConfig
    {
        /// <summary>한 폭에서 첫 분리 실패 시 스텝 배율에 곱할 값. (0,1) 범위여야 동작하며
        /// 1.0 또는 범위 밖 = 비활성(기본값).</summary>
        public float fail_decay;
        /// <summary>스텝 배율의 하한. 0 이하 또는 1 초과 = 기본값(0.05).</summary>
        public float scale_min;
    }

    /// <summary>
    /// 탐색 단계 폭 완화 복구 설정. Rust <c>CWidthReliefConfig</c>와 1:1 매핑.
    /// <para>
    /// 분리가 풀지 못하는 폭에 갇혔을 때, 같은 폭을 무한 재시도하는 대신 폭을 조금씩
    /// 넓혀 탈출한다. 한 폭에서 <c>fail_threshold</c>회 연속 실패하면 그 폭의 최선 충돌
    /// 배치를 유지한 채 현재 폭의 +step, +2*step, ... 으로 재분리하며(확장은 충돌이 몰린
    /// 위치에 삽입된다) 충돌이 풀리거나 <c>max_ratio</c>에 닿을 때까지 반복한다. 풀리면
    /// 그 배치에서 정상 축소를 재개한다.
    /// </para>
    /// <para>
    /// <c>osc_fail_threshold</c>(폭 진동)와 달리 최선 feasible 폭에 막히지 않는다.
    /// 인컴번트 해는 그대로 보존되고 더 넓은 단계 결과는 단계 경계에서 거부되므로
    /// 최종 결과가 나빠지지 않는다. <c>fail_threshold &gt; 0</c>이면 복구가 켜지고
    /// <c>osc_fail_threshold</c>는 무시된다. adaptive 빌드 전용.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CWidthReliefConfig
    {
        /// <summary>한 폭에서 이만큼 연속 실패하면 복구 발동. 0 = 비활성(기본값, 기존 폭 진동 유지).</summary>
        public int fail_threshold;
        /// <summary>라운드 1회 확장 비율(현재 폭 대비). 0 이하 = 기본값(0.002 = 0.2%).</summary>
        public float step;
        /// <summary>막힌 폭 대비 누적 확장 상한. 0 이하 = 기본값(0.03 = 3%).</summary>
        public float max_ratio;
        /// <summary>복구 1회의 확장 라운드 상한. 0 이하 = 기본값(12).</summary>
        public int max_rounds;
        /// <summary>확장 라운드 1회의 separation strike 한도. 0 이하 = 기본값(1).</summary>
        public int strike_limit;
        /// <summary>확장 라운드 1회의 벽시계 예산(ms). 0 이하 = 무제한(기본값).
        /// strike 한도만으로는 한 strike가 단계의 <c>iter_no_imprv_limit</c>를 전부 태울 수 있어
        /// 초 단위 예산에서는 복구 한 번이 예산을 다 먹는다. 이 값으로 라운드마다 끊는다.</summary>
        public int round_budget_ms;
    }

    /// <summary>
    /// N06 독립 초기배치 엔진 설정. Rust <c>CInitialPlacementConfig</c>와 1:1 매핑.
    /// <para>
    /// 구성(GRASP) → 슬라이드 재압축 → [폭 축소 → 겹침 분리(rescue) → 재압축] 반복(kick).
    /// 0/음수 필드는 네이티브에서 기본값으로 보정된다 (각 필드 주석 참조).
    /// Legacy(단일 스트립) 입력 전용이며 Grouped(V2) 입력에서는 오류를 반환한다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CInitialPlacementConfig
    {
        /// <summary>0 = 비활성(기본값), 그 외 = <c>swarm_init_phase</c> 사용 가능.</summary>
        public int enabled;
        /// <summary>전체 시간 예산(초). 0 이하 = 기본값(60).</summary>
        public float time_limit_secs;
        /// <summary>GRASP 구성 반복 수(최소 폭 채택). 0 이하 = 기본값(1).</summary>
        public int grasp;
        /// <summary>kick(step 재확장 재시도) 횟수. 음수 = 기본값(100000 = 사실상 예산 제한), 0 = kick 없음.</summary>
        public int kicks;
        /// <summary>축소 시작 step 비율. 0 이하 = 기본값(0.005 = 0.5%).</summary>
        public float step0_frac;
        /// <summary>축소 최소 step 비율. 0 이하 = 기본값(0.0005 = 0.05%).</summary>
        public float min_frac;
        /// <summary>분리 반복 상한. 0 이하 = 기본값(3000).</summary>
        public int iter_cap;
        /// <summary>분리 이동 표본 수 기저. 0 이하 = 기본값(50).</summary>
        public int samples_base;
        /// <summary>GLS 가중치 증가율. 1.0 이하 = 기본값(1.1).</summary>
        public float growth;
        /// <summary>rescue(전수 재배치) 발동 주기(정체 반복 수). 음수 = 기본값(300), 0 = 끔.</summary>
        public int rescue_after;
        /// <summary>정체 기반 종료: 무개선 연속 상한. 음수 = 기본값(1500), 0 = 반복 수 종료(구 동작).</summary>
        public int stall_cap;
        /// <summary>좌표 하강 최소 스텝(mm). 0 이하 = 기본값(0.03).</summary>
        public float cd_min;
        /// <summary>슬라이드 재압축 패스 상한. 음수 = 기본값(6), 0 = 재압축 끔.</summary>
        public int compact_passes;
        /// <summary>reseat(최우측 조각 재배치) 대상 수. 음수 = 기본값(4), 0 = 끔.</summary>
        public int reseat_top;
        /// <summary>배치 후보 평가 상한. 0 이하 = 기본값(32).</summary>
        public int cand_eval_cap;
        /// <summary>좌하단 점수 X 가중치. 0 이하 = 기본값(500).</summary>
        public float lbf_x_weight;
        /// <summary>좌하단 점수 Y 가중치. 음수 = 기본값(0.1).</summary>
        public float lbf_y_weight;
    }

    /// <summary>
    /// LBF 초기 배치의 래스터 NFP 후보 생성 설정. Rust의 CRasterNfpConfig와 1:1 매핑.
    /// <para>
    /// 레이아웃과 아이템을 같은 격자로 래스터화해 겹치지 않는 격자 위치를 직접 열거한다.
    /// 모든 후보는 정밀 CDE로 재검증되므로 격자 해상도는 품질에만 영향을 준다.
    /// 초기 폭 후보를 보강하며 기본값은 켜짐이다. 최종 밀도 효과는 입력별 ablation으로 확인한다.
    /// capmods 없이 빌드된 네이티브에서는 무시된다.
    /// </para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CRasterNfpConfig
    {
        /// <summary>0 = 비활성, 그 외 = 활성(기본값).</summary>
        public int enabled;
        /// <summary>셀 크기 = (가장 작은 아이템 최소 변) × 이 비율. 0 이하 = 기본값(0.12).</summary>
        public float cell_ratio;
        /// <summary>격자 셀 총 개수 상한. 0 이하 = 기본값(250000).</summary>
        public int max_cells;
        /// <summary>CDE로 검증할 래스터 후보 수. 0 이하 = 기본값(64).</summary>
        public int n_candidates;
        /// <summary>래스터 후보와 병행할 난수 샘플 수. 0 = 래스터만, 음수 = 기본값(1000).</summary>
        public int n_random_samples;
    }

    // -- Callback delegate

    /// <summary>
    /// 최적화 진행 상황을 실시간으로 수신하는 콜백 델리게이트.
    /// Rust 측에서 <c>extern "C" fn(...)</c>로 호출되므로
    /// <see cref="CallingConvention.Cdecl"/> 호출 규약을 사용한다.
    ///
    /// <para><paramref name="solution_json"/>은 <paramref name="emit_intermediate_json"/>이
    /// 1이고 JSON이 첨부된 보고일 때만 유효한 UTF-8 결합 JSON 포인터이며, 그 외에는 NULL이다.
    /// 포인터는 callback이 반환되는 순간 무효가 되므로 보관하려면 callback 안에서 복사해야 한다.</para>
    /// </summary>
    /// <param name="report_type">보고 유형 (<see cref="ReportType"/> enum 값). Structure checkpoint는 P1/P3=0, P2=3, P4=2, P5=4.</param>
    /// <param name="strip_width">현재 스트립 폭</param>
    /// <param name="density">현재 밀도</param>
    /// <param name="solution_json">
    /// <c>{ solution, loss, collision_status }</c> combined JSON UTF-8 문자열 포인터
    /// (또는 <see cref="IntPtr.Zero"/>). 콜백 안에서는 문자열을 복사만 하고
    /// getter/phase/free API를 재진입 호출하지 않는다.
    /// </param>
    /// <param name="user_data"><see cref="swarm_set_callback"/>에 전달한 사용자 데이터 포인터</param>
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void SolutionCallback(
        int report_type,
        float strip_width,
        float density,
        IntPtr solution_json,
        IntPtr user_data
    );

    /// <summary>네이티브 ABI 버전, 구조체 크기, 컴파일 feature 플래그를 가져온다.</summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    private static extern int swarm_get_abi_info(out CSwarmAbiInfo info);

    /// <summary>
    /// 기본 설정값을 가져온다.
    /// <paramref name="config"/>에 Rust 측 기본 <c>SwarmConfig</c> 값이 채워진다.
    /// </summary>
    /// <param name="config">기본값을 받을 구조체 (참조 전달)</param>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_get_default_config(ref CSwarmConfig config);

    /// <summary>
    /// Rust 측에서 할당된 문자열을 해제한다.
    /// <see cref="swarm_get_solution_json"/>, <see cref="swarm_get_collision_status_json"/>,
    /// <see cref="swarm_find_bound_collisions"/>가 반환한 포인터에만 사용한다.
    /// (<see cref="swarm_version"/>은 정적 문자열이므로 해제하면 안 된다.)
    /// </summary>
    /// <param name="ptr">해제할 문자열 포인터</param>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_free_string(IntPtr ptr);

    /// <summary>현재 스레드의 마지막 Init, phase 또는 solution export 오류 코드. 0이면 오류 없음.</summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_get_last_error_code();

    /// <summary>
    /// 현재 스레드의 마지막 Init, phase 또는 solution export 오류 UTF-8 문자열.
    /// 반환 포인터는 <see cref="swarm_free_string"/>으로 해제해야 한다.
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_get_last_error_message();

    /// <summary>
    /// 라이브러리 버전 문자열을 가져온다.
    /// 반환된 <see cref="IntPtr"/>는 Rust 측 정적 수명의 UTF-8 문자열을 가리키므로
    /// 해제해서는 안 된다.
    /// </summary>
    /// <returns>버전 문자열 UTF-8 포인터</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_version();

    // -- Step-by-step API

    /// <summary>
    /// 단계별(Step-by-step) API용 진행 상황 콜백을 등록한다.
    /// <see cref="SwarmStepOptimizer"/>가 내부적으로 사용한다.
    /// </summary>
    /// <param name="handle">최적화기 핸들</param>
    /// <param name="callback">진행 상황 콜백</param>
    /// <param name="user_data">콜백에 전달할 사용자 데이터 포인터</param>
    /// <param name="emit_intermediate_json">중간 JSON 문자열을 콜백에 포함할지 여부 (0=미포함, 1=포함)</param>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_set_callback(
        IntPtr handle,
        SolutionCallback? callback,
        IntPtr user_data,
        int emit_intermediate_json
    );

    /// <summary>현재 솔루션의 각 Place의 충돌 상태 JSON을 가져온다.</summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_get_collision_status_json(IntPtr handle);

    /// <summary>현재 솔루션의 각 Place의 충돌 상태 배열을 가져온다.</summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_get_collision_status(
        IntPtr handle,
        out UIntPtr count
    );

    /// <summary>swarm_get_collision_status가 반환한 배열을 해제한다.</summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_free_collision_status(
        IntPtr statuses,
        UIntPtr count
    );

    /// <summary>
    /// 단계별 API: 최적화기를 초기화한다.
    /// 입력 파일을 로드하고 LBF 초기해를 빌드한 뒤 핸들을 반환한다.
    /// </summary>
    /// <param name="input_path">입력 JSON 파일 경로</param>
    /// <param name="config">최적화 설정</param>
    /// <param name="hints">배치 힌트 배열 (빈 배열 가능)</param>
    /// <param name="hint_count"><paramref name="hints"/> 길이</param>
    /// <returns>핸들 포인터. 실패 시 <see cref="IntPtr.Zero"/>.</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_init(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string input_path,
        ref CSwarmConfig config,
        CPlacedHint[] hints,
        int hint_count
    );

    /// <summary>
    /// JSON 문자열로부터 초기화한다 (파일 입력과 동일한 형식).
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_init_from_json(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string input_json,
        ref CSwarmConfig config,
        CPlacedHint[] hints,
        int hint_count
    );

    /// <summary>
    /// 단계별 API: N06 초기배치(Initial Placement) 페이즈를 실행한다.
    /// 초기화 직후(Phase=Init)에만 호출 가능한 선택 단계로,
    /// LBF 초기해를 N06 엔진(GRASP 구성 → 재압축 → 점진 축소·분리·kick)의
    /// 결과로 교체한다. 종료 후에도 Phase=Init이 유지되어
    /// <see cref="swarm_alg_phase"/>/<see cref="swarm_cp_phase"/>를 이어 붙일 수 있다.
    /// Grouped(V2) 입력에서는 오류(3004)를 반환한다.
    /// <see cref="swarm_request_stop"/>으로 중단할 수 있다.
    /// </summary>
    /// <param name="handle">핸들</param>
    /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_init_phase(IntPtr handle);

    /// <summary>
    /// 단계별 API: 탐색(Exploration) 페이즈를 실행한다.
    /// 탐색·압축·후처리는 순서와 횟수가 자유다(예: Explore → Explore → Compress → Polish → Explore).
    /// 어느 호출이든 직전 호출이 확정한 인증 해에서 출발해 자기 시간만큼 돌고, 결과가 나빠지지 않는다.
    /// Structure 이후에는 호출할 수 없다.
    /// <see cref="swarm_request_stop"/>으로 중단할 수 있다.
    /// </summary>
    /// <param name="handle">핸들</param>
    /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_alg_phase(IntPtr handle);

    /// <summary>
    /// 단계별 API: 압축(Compression) 페이즈를 실행한다.
    /// 초기화 직후(탐색 생략, LBF + Compress) 또는
    /// <see cref="swarm_alg_phase"/> 이후 중 어느 쪽에서든 호출할 수 있다.
    /// 탐색을 생략하면 LBF 초기해에서 바로 압축을 시작한다.
    /// <see cref="swarm_request_stop"/>으로 중단할 수 있다.
    /// </summary>
    /// <param name="handle">핸들</param>
    /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_cp_phase(IntPtr handle);

    /// <summary>
    /// 단계별 API: 후처리(Polish) 페이즈를 실행한다.
    /// 좌측 슬라이딩 · 빈 포켓 채우기 · LP compaction을 순서대로 적용한다.
    /// <see cref="swarm_cp_phase"/> 이후에 호출해야 하며,
    /// <see cref="swarm_request_stop"/>으로 중단할 수 있다.
    /// </summary>
    /// <param name="handle">핸들</param>
    /// <returns>성공 시 0, 실패 시 0이 아닌 값</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_refine_phase(IntPtr handle);

    /// <summary>
    /// 단계별 API: P0-P5 structure-first 대체 파이프라인을 실행한다.
    /// 초기화 직후에만 호출할 수 있고 <c>structure_first.enabled</c>가 켜져 있어야 한다.
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_gp_phase(IntPtr handle);

    /// <summary>
    /// 최근 structure-first 실행의 체크포인트/진단 보고서 JSON을 반환한다.
    /// 반환 포인터는 <see cref="swarm_free_string"/>으로 해제해야 한다.
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_get_structure_report_json(IntPtr handle);

    /// <summary>
    /// 단계별 API: 최종 솔루션 JSON을 반환한다.
    /// 호출자는 반환된 포인터를 <see cref="swarm_free_string"/>로 해제해야 한다.
    /// </summary>
    /// <param name="handle">핸들</param>
    /// <returns>JSON 문자열 포인터. 실패 시 <see cref="IntPtr.Zero"/>.</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_get_solution_json(IntPtr handle);

    /// <summary>
    /// 단계별 API: 실행 중인 페이즈에 즉시 중단을 요청한다.
    /// 백그라운드 스레드에서 실행 중인 <see cref="swarm_alg_phase"/> 또는
    /// <see cref="swarm_cp_phase"/>가 다음 반복문 확인 시 종료된다.
    /// 어떤 스레드에서든 호출 가능하다. 중단 요청은 이 핸들의 수명 동안 유지된다.
    /// </summary>
    /// <param name="handle">핸들</param>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_request_stop(IntPtr handle);

    /// <summary>
    /// 진행 중인 <see cref="swarm_init_from_json"/>을 취소한다. 어떤 스레드에서든 호출 가능하다.
    /// <para>Init 은 핸들이 생기기 전이라 <see cref="swarm_request_stop"/>이 닿지 않는다. 이 플래그는
    /// 프로세스 전역이며 다음 <see cref="swarm_init_from_json"/>이 시작할 때 지워진다. 취소된 Init 은
    /// NULL 을 돌려주고 오류 코드 2002 를 남긴다 — 실패가 아니라 사용자 중단으로 다룰 것.</para>
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_cancel_init();

    /// <summary>stop 플래그 상태를 읽는다. 0이 아니면 stop 요청된 상태.</summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_is_stop_requested(IntPtr handle);

    /// <summary>Default for subsequently created handles; does not update running handles.</summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_set_shrink_step(float step);
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_clear_shrink_step();
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern float swarm_get_shrink_step();

    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_set_handle_shrink_step(IntPtr handle, float step);
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_clear_handle_shrink_step(IntPtr handle);
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern float swarm_get_handle_shrink_step(IntPtr handle);

    /// <summary>
    /// 네이티브 로그 수신 델리게이트.
    /// </summary>
    /// <param name="level">1=Error, 2=Warn, 3=Info, 4=Debug, 5=Trace.</param>
    /// <param name="message">
    /// NUL 종료 UTF-8 포인터. <b>콜백이 반환하는 즉시 무효</b>가 되므로 보관하려면
    /// <see cref="Marshal.PtrToStringUTF8(IntPtr)"/>로 복사할 것.
    /// </param>
    /// <param name="userData"><c>swarm_set_log_callback</c>에 넘긴 값이 그대로 돌아온다.</param>
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void SwarmLogCallback(int level, IntPtr message, UIntPtr userData);

    /// <summary>
    /// 네이티브 로그를 관리 코드로 넘길 콜백을 등록한다.
    /// <para>
    /// <b>이걸 부르기 전까지 네이티브 로그는 한 줄도 나오지 않는다.</b> Rust의
    /// <c>log</c> 크레이트는 로거가 등록되지 않으면 모든 매크로가 no-op이고,
    /// cdylib 경로에는 로거를 설치하는 코드가 없었다.
    /// </para>
    /// <para>
    /// <b>델리게이트는 호출자가 살려 둬야 한다.</b> 필드에 담지 않고 넘기면 GC가
    /// 수거해 네이티브가 죽은 포인터를 호출한다.
    /// </para>
    /// <para>
    /// 콜백은 <b>아무 스레드에서나, 동시에</b> 불린다(rayon 워커 포함). 핸들러는
    /// 스레드 안전해야 하고 UI를 직접 만지면 안 된다.
    /// </para>
    /// </summary>
    /// <param name="callback">수신 델리게이트. <c>null</c>이면 등록을 해제한다.</param>
    /// <param name="userData">콜백에 그대로 되돌려 줄 값. 쓰지 않으면 <see cref="UIntPtr.Zero"/>.</param>
    /// <param name="maxLevel">
    /// 0=Off, 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Trace.
    /// 릴리스 빌드는 <c>release_max_level_info</c> 때문에 3을 넘어서 나오지 않는다.
    /// </param>
    /// <returns>1 = 성공, 0 = 실패(다른 로거가 이미 프로세스에 설치됨).</returns>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int swarm_set_log_callback(
        SwarmLogCallback? callback,
        UIntPtr userData,
        int maxLevel);

    /// <summary>
    /// 콜백으로 넘기지 못하고 버린 로그 줄 수. 정상 동작에서는 0이다.
    /// 0이 아니면 관리 쪽 핸들러가 예외를 던지고 있다는 뜻이다.
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong swarm_log_dropped_count();

    /// <summary>
    /// 실행 중인 페이즈에 중단을 요청하고, 실제로 페이즈가 종료될 때까지 블록한다.
    /// 실행 중인 페이즈가 없으면 즉시 반환한다. 중단 요청은 이 핸들의 수명 동안 유지된다.
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_request_stop_and_wait(IntPtr handle);

    /// <summary>
    /// 고정할 item_id 목록을 설정한다. swarm_alg_phase 호출 전에 호출해야 한다.
    /// 고정된 아이템은 separation 중에 이동하지 않는다.
    /// ids: int 배열 포인터, count: 배열 길이
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_freeze_items(IntPtr handle, int[] ids, int count);


    /// <summary>
    /// C#에서 계산한 바운딩 박스 배열을 받아 AABB 1차 필터 → CDE 정밀 검사로
    /// 실제 충돌 중인 아이템 ID만 반환한다.
    /// handle: SwarmStepHandle 포인터. null이면 AABB-only 모드.
    /// 반환값: JSON int 배열 ([id1, id2, ...]). 충돌 없으면 IntPtr.Zero.
    /// 호출 후 반드시 swarm_free_string으로 해제해야 한다.
    /// </summary>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr swarm_find_bound_collisions(
        IntPtr handle,
        [In] CItemBounds[] bounds,
        int count
    );

    /// <summary>
    /// 단계별 API: 실행 중인 페이즈를 중단하고 활성 호출이 끝날 때까지 기다린 뒤
    /// 핸들과 관련 리소스를 해제한다. 페이즈 스레드의 콜백 안에서 부르면 교착 대신
    /// 정지만 요청하고 해제하지 않은 채 돌아온다(핸들 누수) — 콜백 밖에서 부를 것.
    /// </summary>
    /// <param name="handle">핸들</param>
    [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void swarm_free(IntPtr handle);
}
