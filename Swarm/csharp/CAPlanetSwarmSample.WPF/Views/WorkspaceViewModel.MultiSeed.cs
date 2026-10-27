using CAPlanetSwarmSample.WPF.Data;
using CAPlanetSwarmSample.WPF.Services;
using ReactiveUI.SourceGenerators;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Threading.Channels;
using Uttu.NestingV2;

namespace CAPlanetSwarmSample.WPF.Views;

/// <summary>
/// 다중 시드 동시 실행. 시드마다 독립 핸들로 같은 설정을 돌리고, 인증된 결과 중 가장 짧은
/// 배치를 화면에 올린다. 시드가 하나면 기존 단일 실행 경로(<c>RunPipelineAsync</c>)를 쓴다.
/// </summary>
public partial class WorkspaceViewModel
{
    // Each seed owns its handle. Registration also serializes stop/setter access against disposal.
    private readonly HashSet<SwarmStepOptimizer> _MultiSeedHandles = new();
    private float? _MultiSeedShrinkOverride;
    [Reactive] public partial string ResolvedSeedsText { get; set; } = string.Empty;
    [Reactive] public partial string WinningSeedText { get; set; } = string.Empty;
    [Reactive] public partial string MultiSeedStatus { get; set; } = string.Empty;

    /// <summary>실행 중인 모든 시드에 중지를 전달한다. 실행 중인 시드가 있었으면 <c>true</c>.</summary>
    private bool RequestMultiSeedStop()
    {
        lock (_OptimizerSync)
        {
            foreach (var handle in _MultiSeedHandles)
            {
                // Init 은 핸들이 생기기 전이라 phase stop 이 닿지 않는다 — 전역 Init 취소를 건다.
                if (handle.IsInitialized) handle.RequestStop();
                else handle.CancelInit();
            }
            return _MultiSeedHandles.Count > 0;
        }
    }

    private async Task WaitForMultiSeedStopAsync(Task completion)
    {
        // 중지 확인 직후 뒤늦게 Init 에 들어간 시드와
        // 방금 Init 을 마치고 단계에 진입한 시드까지 멈추도록 끝날 때까지 다시 요청한다.
        while (!completion.IsCompleted)
        {
            RequestMultiSeedStop();
            await Task.WhenAny(completion, Task.Delay(100)).ConfigureAwait(false);
        }
        await completion.ConfigureAwait(false);
    }

    // Native calls are blocking. Dedicated callers let all seeds enter Init/phases promptly,
    // without consuming the thread pool needed by JSON consumers and the UI dispatcher.
    private static Task<T> NativeSeedCall<T>(Func<T> call) => Task.Factory.StartNew(
        call, CancellationToken.None, TaskCreationOptions.LongRunning, TaskScheduler.Default);

    private async Task RunMultiSeedAsync(StringBuilder log, SwarmConfig common, IReadOnlyList<ulong> seeds,
        string input, bool useHints, int? groupWorkers, int generation, CancellationToken token)
    {
        lock (_OptimizerSync) _MultiSeedShrinkOverride = null;
        if (!common.StructureFirst.Enabled)
            common.PhaseSequence = OptimizationPhaseSequence.Validate(common.PhaseSequence);
        var preparation = SwarmInputInspector.PrepareForRun(input, useHints, groupWorkers);
        if (!preparation.IsSuccess) throw new InvalidDataException(preparation.ErrorMessage);
        var summary = SwarmInputInspector.Parse(preparation.Json);
        if (!summary.IsValid) throw new InvalidDataException(summary.ErrorMessage);
        InputSummary = summary;
        var hints = summary.SchemaVersion != 1
            ? Array.Empty<CPlacedHint>()
            : SwarmInputPlaces.Parse(input).ToNativeHints(useHints);
        // 단일 실행과 같은 힌트 준비 정책을 모든 시드에 적용한다 (RunPipelineAsync 의 주석 참조).
        if (hints.Length > 0)
        {
            if (common.PolySimplTolerance is float simpl && simpl >= 0)
            {
                common.PolySimplTolerance = null;
                AppendRunLine(log, $"[Seeds] Hint 설정 보정: PolySimpl {simpl} → 끔");
            }
            if (common.FabricTolerance is { PieceOverlapMm: <= 0 })
            {
                common.FabricTolerance.PieceOverlapMm = HintPieceOverlapMm;
                AppendRunLine(log, $"[Seeds] Hint 설정 보정: Piece Overlap → {HintPieceOverlapMm}mm");
            }
        }
        var configs = MultiSeedConfiguration.Create(common, seeds);
        using var document = JsonDocument.Parse(preparation.Json);
        var root = document.RootElement;
        var instance = root.TryGetProperty("instance", out var nested) ? nested : root;
        var demand = instance.GetProperty("items").EnumerateArray()
            .Where(item => !item.TryGetProperty("demand", out var q) || q.GetInt32() > 0).ToDictionary(
            item => item.GetProperty("id").GetInt32(),
            item => item.TryGetProperty("demand", out var quantity) ? quantity.GetInt32() : 1);
        var competition = new MultiSeedCompetition(configs.Select(c => c.RngSeed).ToArray(), demand);
        string[] statuses = configs.Select(c => $"{c.RngSeed}: 준비").ToArray();
        ResolvedSeedsText = "실행 시드: " + string.Join(", ", configs.Select(c => c.RngSeed));
        AppendRunLine(log, $"[Seeds] {ResolvedSeedsText}; 공통 Config 복제, 시드별 워커 설정 유지");
        if (seeds.Contains(0UL)) AppendRunLine(log, $"[Seeds] 입력 0(무작위) → 실제 시드 {configs[seeds.ToList().IndexOf(0)].RngSeed}");
        bool sink = SwarmOptimizer.SetLogSink((level, message) =>
        {
            AppendRunLine(log, $"[native shared:{NativeLogLabel(level)}] {message}");
            _ = PublishRunLogAsync(log, generation);
        });
        if (!sink) AppendRunLine(log, "[Seeds] 공유 native 로그 싱크 설치 실패");
        try
        {
            await Task.WhenAll(configs.Select((config, index) => Task.Run(() => RunSeedAsync(index, config))));
            AppendRunLine(log, competition.Winner is { } winner
                ? $"[Seeds] 최상 인증 결과 seed={winner.Seed}, 길이={winner.Report.StripWidth:F3}mm, 효율={winner.Report.Density:P3}"
                : "[Seeds] 인증된 전체 배치 결과 없음");
        }
        finally
        {
            if (sink) SwarmOptimizer.SetLogSink(null);
            // Native logs are bounded; repeat the seed mapping after the run so it
            // remains available even when verbose logs evicted the startup entries.
            AppendRunLine(log, $"[Seeds] 종료 · {ResolvedSeedsText}");
            if (seeds.Contains(0UL)) AppendRunLine(log, $"[Seeds] 입력 0(무작위) → 실제 시드 {configs[seeds.ToList().IndexOf(0)].RngSeed}");
        }

        async Task Status(int index, string status)
        {
            await RunOnMainThreadAsync(() =>
            {
                if (_IsDisposed || generation != Volatile.Read(ref _RunGeneration)) return;
                statuses[index] = $"{configs[index].RngSeed}: {status}";
                MultiSeedStatus = string.Join(" · ", statuses);
                StructureProgressText = "다중 시드 · 인증된 최상 결과 표시";
            });
        }

        async Task Offer(int index, SolutionEventArgs report, string phase, bool final = false,
            string? structure = null)
        {
            if (token.IsCancellationRequested || generation != Volatile.Read(ref _RunGeneration)) return;
            var winner = competition.TryOffer(index, report, phase, final, structure);
            if (winner is null) return;
            AppendRunLine(log, $"[Seed {winner.Seed}] 최상 갱신 #{winner.Revision}: {phase}, {report.StripWidth:F3}mm, {report.Density:P3}");
            var snapshot = await Task.Run(() => BuildCallbackRenderSnapshot(winner.Report, new HashSet<int>()));
            bool applied = false;
            try
            {
                await RunOnMainThreadAsync(() =>
                {
                    // Parsing of an older winner may finish after a newer seed has won.
                    if (_IsDisposed || token.IsCancellationRequested
                        || generation != Volatile.Read(ref _RunGeneration) || !competition.IsCurrent(winner)) return;
                    LastSolutionJson = winner.Report.SolutionJson!;
                    RenderedStripWidth = snapshot.StripWidth;
                    RenderedMaxY = snapshot.MaxY;
                    RenderedDensity = winner.Report.Density;
                    RenderedCollisions = 0;
                    RenderedLoss = 0;
                    LastStructureReportJson = winner.StructureReportJson ?? string.Empty;
                    StructureReportSummary = string.IsNullOrEmpty(LastStructureReportJson)
                        ? null : StructureReportInspector.Parse(LastStructureReportJson);
                    WinningSeedText = $"최상 시드 {winner.Seed} · {winner.Phase} · {winner.Report.Density:P2}";
                    ReplaceRenderedItems(snapshot.Items);
                    applied = true;
                });
            }
            finally { if (!applied) DisposeRenderItems(snapshot.Items); }
        }

        async Task RunSeedAsync(int index, SwarmConfig config)
        {
            var handle = new SwarmStepOptimizer();
            lock (_OptimizerSync) _MultiSeedHandles.Add(handle);
            Channel<RawCallbackReport>? channel = null;
            Task? consumer = null;
            string phase = "Init";
            bool failed = false;
            int completedPhases = 0;
            int plannedPhases = config.StructureFirst.Enabled ? 1 : config.PhaseSequence.Length;
            void Write(string line) => AppendRunLine(log, $"[Seed {config.RngSeed}] {line}");
            void Error(string line)
            {
                Write("[ERROR] " + line.Replace("\r", " ").Replace("\n", " "));
                _ = PublishRunLogAsync(log, generation);
            }
            bool Stopped() => _IsStopRequested || _IsDisposed || token.IsCancellationRequested;
            async Task Boundary(bool final = false)
            {
                // GetSolutionJson itself performs original/native certification. Empty/error
                // exports and incomplete collision-status arrays fail closed in the selector.
                string json = handle.GetSolutionJson();
                if (string.IsNullOrWhiteSpace(json)) { Error($"{phase} 인증/내보내기 실패: {handle.LastError}"); return; }
                using var solution = JsonDocument.Parse(json);
                float width = solution.RootElement.GetProperty("strip_width").GetSingle();
                float density = solution.RootElement.GetProperty("density").GetSingle();
                string statusesJson = handle.GetCollisionStatusJson();
                var report = SolutionEventArgs.FromCombinedJson(ReportType.Final, width, density,
                    "{\"solution\":" + json + ",\"collision_status\":" + statusesJson + "}");
                string? structure = config.StructureFirst.Enabled && phase == "Structure"
                    ? handle.GetStructureReportJson() : null;
                Write($"{phase} 인증 snapshot: {width:R}mm, 조각={report.ParsedPlacements.Count}, " +
                    $"충돌={report.CollisionStatus.Count(s => s.IsCollision)}/{report.CollisionStatus.Count}");
                await Offer(index, report, phase, final, structure);
            }
            try
            {
                await Status(index, phase);
                Write(phase + " 시작");
                bool initialized = await NativeSeedCall(() => !Stopped()
                    && handle.InitFromJson(preparation.Json, config, hints));
                if (!initialized)
                {
                    if (Stopped() || handle.LastInitWasCancelled)
                    {
                        Write("Init 중단됨");
                        await Status(index, $"Init 중단 (0/{plannedPhases})");
                        return;
                    }
                    throw new InvalidOperationException($"Init 실패 {handle.LastErrorCode}: {handle.LastError}");
                }
                lock (_OptimizerSync)
                    if (_MultiSeedShrinkOverride is float step) handle.TrySetShrinkStep(step);
                await Boundary();
                if (Stopped()) { await Status(index, $"중단 (0/{plannedPhases})"); return; }
                channel = Channel.CreateUnbounded<RawCallbackReport>(new UnboundedChannelOptions
                    { SingleReader = true, SingleWriter = false, AllowSynchronousContinuations = false });
                var writer = channel.Writer;
                consumer = Consume();
                handle.SetRawCallback((type, width, density, json) =>
                {
                    // Grouped Explore 는 인증된 중간 해를 ExplImproving 으로 보낸다. 레거시의
                    // ExplImproving 은 JSON 이 없으므로 큐에 넣지 않는다. 충돌 검사는 경쟁기에서 한다.
                    if (type is ReportType.ExplFeas or ReportType.CmprFeas or ReportType.Final
                        || (type is ReportType.ExplImproving && !string.IsNullOrEmpty(json)))
                        writer.TryWrite(new RawCallbackReport(type, width, density, json, Volatile.Read(ref phase)));
                }, emitJson: true);
                await RunOnMainThreadAsync(() =>
                {
                    if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration) && !_PhaseClock.IsRunning)
                        StartPhaseClock();
                });
                Write($"실행 목록: {plannedPhases}단계 · " + (config.StructureFirst.Enabled
                    ? "Structure" : string.Join(" → ", config.PhaseSequence.Select(OptimizationPhaseSequence.DisplayName))));
                if (config.StructureFirst.Enabled)
                    failed = !await Phase("Structure", handle.StructurePhase, 1);
                else
                {
                    if (config.InitialPlacement.Enabled && !Stopped())
                        await Phase("Initial", handle.InitialPhase); // 실패해도 Init 해가 그대로 남는다.
                    for (int phaseIndex = 0; phaseIndex < config.PhaseSequence.Length; phaseIndex++)
                    {
                        if (Stopped()) break;
                        var next = config.PhaseSequence[phaseIndex];
                        if (!await Phase(OptimizationPhaseSequence.DisplayName(next), () => next switch
                        {
                            OptimizationPhase.Explore => handle.ExplorePhase(),
                            OptimizationPhase.Compress => handle.CompressPhase(),
                            OptimizationPhase.Polish => handle.PolishPhase(),
                            _ => throw new ArgumentOutOfRangeException()
                        }, phaseIndex + 1)) { failed = true; break; }
                    }
                }
                await DetachRawCallbackQueueAsync(handle, channel, consumer);
                channel = null;
                await Boundary(final: true);
                Write($"실행 목록 종료: {completedPhases}/{plannedPhases}단계 완료" +
                    (Stopped() ? " · 사용자 중지" : failed ? " · 단계 오류로 종료" : " · 전체 완료"));
                await Status(index, Stopped() ? $"중단 ({completedPhases}/{plannedPhases})"
                    : failed ? $"실패 ({completedPhases}/{plannedPhases}, 인증 결과 유지)"
                    : $"완료 ({completedPhases}/{plannedPhases})");

                async Task<bool> Phase(string name, Func<int> call, int ordinal = 0)
                {
                    if (Stopped()) return false;
                    Volatile.Write(ref phase, name);
                    // Each occurrence is distinct: Compress -> Compress must display 2/4 then 3/4.
                    // Configured seconds are a maximum budget, not a mandatory waiting period.
                    string position = ordinal > 0 ? $" [단계 {ordinal}/{plannedPhases}]" : string.Empty;
                    await Status(index, ordinal > 0 ? $"{ordinal}/{plannedPhases} {name}" : name);
                    Write(name + " 시작" + position);
                    var timer = System.Diagnostics.Stopwatch.StartNew();
                    int result = await NativeSeedCall(() => Stopped() ? -1 : call());
                    timer.Stop();
                    string error = result == 0 ? string.Empty : $"; native={handle.LastErrorCode}: {handle.LastError}";
                    Write($"{name} 종료 code={result}{error}{position} · 실제 {timer.Elapsed.TotalSeconds:F3}초");
                    if (result == 0 && ordinal > 0) completedPhases++;
                    if (result != 0 && !Stopped()) Error($"{name} 실패 code={result}{error}");
                    await Boundary();
                    return result == 0;
                }
                async Task Consume()
                {
                    await foreach (var raw in channel.Reader.ReadAllAsync().ConfigureAwait(false))
                    {
                        if (token.IsCancellationRequested) continue;
                        try
                        {
                            var report = SolutionEventArgs.FromCombinedJson(raw.ReportType,
                                raw.StripWidth, raw.Density, raw.CombinedJson);
                            await Offer(index, report, raw.Phase ?? "Callback");
                        }
                        catch (Exception ex) { Error($"{raw.Phase ?? "Callback"} 콜백 거부: {ex.Message}"); }
                    }
                }
            }
            catch (Exception ex)
            {
                Error($"{phase} 실패: {ex}");
                Write($"실행 목록 오류 종료: {completedPhases}/{plannedPhases}단계 완료");
                await Status(index, $"실패 ({completedPhases}/{plannedPhases}, 다른 시드 계속)");
            }
            finally
            {
                try
                {
                    if (channel is not null && consumer is not null)
                        await DetachRawCallbackQueueAsync(handle, channel, consumer);
                }
                finally
                {
                    lock (_OptimizerSync) _MultiSeedHandles.Remove(handle);
                    handle.Dispose();
                }
                await PublishRunLogAsync(log, generation);
            }
        }
    }
}
