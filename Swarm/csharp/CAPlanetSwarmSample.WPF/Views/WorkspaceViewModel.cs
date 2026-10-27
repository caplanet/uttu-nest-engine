using CAPlanetSwarmSample.WPF.Data;
using CAPlanetSwarmSample.WPF.DockSites;
using CAPlanetSwarmSample.WPF.Services;
using ReactiveUI;
using ReactiveUI.Builder;
using ReactiveUI.SourceGenerators;
using SkiaSharp;
using Uttu.NestingV2;
using System;
using System.Collections.Generic;
using System.IO;
using System.Reactive.Concurrency;
using System.Reactive.Disposables;
using System.Reactive.Disposables.Fluent;
using System.Reactive.Linq;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Channels;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace CAPlanetSwarmSample.WPF.Views;

/// <summary>
/// 한 배치 아이템의 불변 렌더링 snapshot입니다.
/// geometry에서 파생되는 path, bounds, label 위치를 한 번만 계산해 paint frame의 반복 작업을 없앱니다.
/// </summary>
public sealed class PlacedItemRender : IDisposable
{
    private bool _IsDisposed;

    public PlacedItemRender(
        int itemId,
        List<(float X, float Y)> polygon,
        SKColor color,
        SKColor? borderColor = null)
    {
        ArgumentNullException.ThrowIfNull(polygon);
        if (polygon.Count < 3)
        {
            throw new ArgumentException("렌더링 polygon은 점이 3개 이상이어야 합니다.", nameof(polygon));
        }

        ItemId = itemId;
        Polygon = polygon;
        Color = color;
        BorderColor = borderColor ?? SKColors.White.WithAlpha(140);
        Label = itemId.ToString(System.Globalization.CultureInfo.InvariantCulture);

        float minX = float.PositiveInfinity;
        float minY = float.PositiveInfinity;
        float maxX = float.NegativeInfinity;
        float maxY = float.NegativeInfinity;
        float centerX = 0;
        float centerY = 0;

        using var builder = new SKPathBuilder();
        builder.MoveTo(polygon[0].X, polygon[0].Y);
        foreach (var point in polygon.Skip(1))
        {
            builder.LineTo(point.X, point.Y);
        }
        builder.Close();
        Path = builder.Detach();

        foreach (var (x, y) in polygon)
        {
            minX = MathF.Min(minX, x);
            minY = MathF.Min(minY, y);
            maxX = MathF.Max(maxX, x);
            maxY = MathF.Max(maxY, y);
            centerX += x;
            centerY += y;
        }

        Bounds = new SKRect(minX, minY, maxX, maxY);
        LabelCenter = new SKPoint(centerX / polygon.Count, centerY / polygon.Count);
    }

    /// <summary>Item ID.</summary>
    public int ItemId { get; }

    /// <summary>SVG 내보내기와 진단에서 사용하는 원본 world 좌표입니다.</summary>
    public List<(float X, float Y)> Polygon { get; }

    /// <summary>매 frame 재생성하지 않고 바로 그리는 캐시 path입니다.</summary>
    public SKPath Path { get; }

    /// <summary>viewport culling과 전체 높이 계산에 사용하는 world bounds입니다.</summary>
    public SKRect Bounds { get; }

    /// <summary>item ID를 배치할 캐시 중심점입니다.</summary>
    public SKPoint LabelCenter { get; }

    public string Label { get; }

    public SKColor Color { get; }

    public SKColor BorderColor { get; }

    public void Dispose()
    {
        if (_IsDisposed)
        {
            return;
        }

        _IsDisposed = true;
        Path.Dispose();
    }
}

public partial class WorkspaceViewModel : ReactiveObject, IDisposable
{
    private sealed record RenderSnapshot(
        List<PlacedItemRender> Items,
        float StripWidth,
        float MaxY,
        float? Density);

    /// <summary>
    /// 네이티브 callback 포인터에서 관리 문자열로 복사된 뒤 파싱 큐에 적재되는 원시 보고입니다.
    /// </summary>
    private sealed record RawCallbackReport(
        ReportType ReportType,
        float StripWidth,
        float Density,
        string CombinedJson,
        string? Phase = null);

    #region Define

    private static readonly SKColor[] Colors = {
        SKColor.Parse("#E57373"), SKColor.Parse("#64B5F6"), SKColor.Parse("#81C784"),
        SKColor.Parse("#FFD54F"), SKColor.Parse("#BA68C8"), SKColor.Parse("#4DD0E1"),
        SKColor.Parse("#FF8A65"), SKColor.Parse("#A1887F")
    };

    #endregion

    #region Field

    private readonly CompositeDisposable _Subscriptions = new();

    private readonly object _OptimizerSync = new();

    private readonly StructureProgressTracker _StructureProgress = new();

    private SwarmStepOptimizer? _StepOptimizer;

    private volatile bool _IsStopRequested;

    private volatile bool _IsDisposed;

    private int _RunGeneration;

    private int _DisposeStarted;

    private TaskCompletionSource<bool> _RunCompletion = CompletedRunCompletion();

    private long _SnapshotSequence;

    private long _LastAppliedSnapshotSequence;

    private CancellationTokenSource _SnapshotCancellation = new();

    private int _LogPublishScheduled;

    private int _LogPublishDirty;

    private const int MaxRunLogChars = 128 * 1024;

    private const int RetainedRunLogChars = 96 * 1024;

    private Dictionary<int, List<(float X, float Y)>> _Shapes = new();

    private Dictionary<int, CPlacedHint> _Hints = new();

    #endregion

    #region Property

    public MainWindowViewModel ParentViewModel { get; }

    public DockDocumentItemViewModel DockingItem { get; }

    public SwarmInputData InputData { get; }

    public string InputFullPath => Path.Combine(InputData.DirectoryPath, InputData.FileName + "." + InputData.FileExtension);

    [Reactive] public partial OptimizerOptions SelectedOption { get; set; } = OptimizerOptions.Simple;

    [Reactive] public partial bool ShowLeftPanel { get; set; } = true;

    [Reactive] public partial bool ShowCollisions { get; set; } = false;

    /// <summary>
    /// 이동 가능한 warm-start 힌트를 사용할지 여부입니다. 잠금 힌트와 PlaceGroup 상대 pose는 항상 유지합니다.
    /// </summary>
    [Reactive] public partial bool UseHints { get; set; } = true;

    public GridLength LeftPanelWidth => ShowLeftPanel ? new GridLength(1, GridUnitType.Auto) : new GridLength(0);

    public double LeftPanelMinWidth => ShowLeftPanel ? 400 : 0;

    public GridLength SplitterWidth => ShowLeftPanel ? GridLength.Auto : new GridLength(0);

    [Reactive] public partial string OutputLog { get; set; } = string.Empty;

    [Reactive] public partial bool IsRunning { get; set; } = false;

    /// <summary>
    /// Init 직후부터 마지막 phase(Polish, 또는 Structure) 종료까지의 경과 시간 표시.
    /// Run 시작 시 비우고, Init 이 끝나 phase 가 시작될 때 시계를 켠 뒤, 실행이 끝나면
    /// 마지막 값으로 고정한다(중단·실패도 그 시점에서 멈춘다).
    /// </summary>
    [Reactive] public partial string PhaseElapsedText { get; set; } = string.Empty;

    private readonly System.Diagnostics.Stopwatch _PhaseClock = new();
    private System.Windows.Threading.DispatcherTimer? _PhaseTicker;

    private static string FormatPhaseElapsed(TimeSpan elapsed)
        => elapsed.TotalHours >= 1
            ? $"{(long)elapsed.TotalHours}:{elapsed:mm\\:ss}"
            : elapsed.ToString(@"m\:ss\.f");

    /// <summary>Init 이 끝나 phase 가 시작되는 시점에 부른다 (UI 스레드).</summary>
    private void StartPhaseClock()
    {
        _PhaseTicker?.Stop();
        _PhaseClock.Restart();
        PhaseElapsedText = FormatPhaseElapsed(TimeSpan.Zero);
        // UI 스레드의 DispatcherTimer — 바인딩 갱신이 곧바로 화면에 반영된다.
        _PhaseTicker ??= new System.Windows.Threading.DispatcherTimer(
            TimeSpan.FromMilliseconds(100),
            System.Windows.Threading.DispatcherPriority.Background,
            (_, _) =>
            {
                if (_PhaseClock.IsRunning)
                    PhaseElapsedText = FormatPhaseElapsed(_PhaseClock.Elapsed);
            },
            Application.Current.Dispatcher);
        _PhaseTicker.Start();
    }

    /// <summary>phase 가 전부 끝났거나 중단됐을 때 부른다 (UI 스레드). 마지막 값을 남긴다.</summary>
    private void StopPhaseClock()
    {
        if (_PhaseClock.IsRunning)
        {
            _PhaseClock.Stop();
            PhaseElapsedText = FormatPhaseElapsed(_PhaseClock.Elapsed);
        }
        _PhaseTicker?.Stop();
    }

    [Reactive] public partial List<PlacedItemRender>? RenderedItems { get; set; } = new();

    [Reactive] public partial float RenderedStripWidth { get; set; } = 0f;

    /// <summary>현재 snapshot의 전체 높이입니다. paint 때 모든 꼭짓점을 다시 순회하지 않습니다.</summary>
    [Reactive] public partial float RenderedMaxY { get; set; } = 0f;

    [Reactive] public partial float RenderedDensity { get; set; } = 0f;

    [Reactive] public partial int RenderedCollisions { get; set; } = 0;

    /// <summary>
    /// 마지막 렌더 대상 해의 loss 입니다. <b>현재는 화면에 표시하지 않습니다.</b>
    /// <para>
    /// 네이티브가 loss 를 실어 보내는 콜백은 JSON 적격 보고뿐인데(<c>src/ffi_step.rs</c> 의
    /// <c>json_eligible</c>), 그 대상은 <c>CmprFeas</c> 와 <c>Final</c> 로 모두 검증을 통과한
    /// <b>실행 가능한</b> 해다. 따라서 이 값은 구조적으로 항상 0 이며, 그대로 표시하면
    /// "겹침 없음"을 늘 참이라고 주장하는 화면이 된다.
    /// </para>
    /// <para>
    /// 실행 중 현재 해의 loss 를 정말 보여 주려면 네이티브에 별도 채널이 필요하다
    /// (스칼라 게터 또는 콜백 payload 의 loss 필드). 겹친 배치를 JSON 으로 다시 흘리는
    /// 방식은 과거에 되돌린 결정이므로 쓰지 말 것 — <c>src/ffi_step.rs</c> 주석 참조.
    /// 입력 JSON 이 담은 배치의 겹침은 <see cref="InputSummary"/> 의
    /// HintedLayout 패널이 이미 보여 준다.
    /// </para>
    /// </summary>
    [Reactive] public partial float RenderedLoss { get; set; } = 0f;

    [Reactive] public partial string LastSolutionJson { get; set; } = string.Empty;

    /// <summary>현재 입력 파일의 Swarm schema, lane, gap 및 PlaceGroup 요약.</summary>
    [Reactive] public partial SwarmInputSummary InputSummary { get; set; } =
        SwarmInputSummary.Invalid("입력 분석 전");

    /// <summary>generic callback 순서를 이용해 보수적으로 추정한 Structure 현재 단계.</summary>
    [Reactive] public partial string StructureProgressText { get; set; } = "Structure 대기";

    /// <summary>완료된 최근 Structure schema 3 감사 보고서의 화면 표시용 요약.</summary>
    [Reactive] public partial StructureReportSummary? StructureReportSummary { get; set; }

    /// <summary>완료된 최근 Structure 감사 JSON 원문. 별도 report 파일 저장에 사용한다.</summary>
    [Reactive] public partial string LastStructureReportJson { get; set; } = string.Empty;

    /// <summary>
    /// UI에서 설정 가능한 네스팅 설정값 (CustomConfig 선택 시 사용).
    /// 프리셋을 적용하면 새 인스턴스로 통째로 교체된다.
    /// </summary>
    private NestingConfigViewModel _Config = new();
    public NestingConfigViewModel Config
    {
        get => _Config;
        private set => this.RaiseAndSetIfChanged(ref _Config, value);
    }

    public string OutputDir => Path.Combine(InputData.DirectoryPath, "output");

    #endregion

    /// <summary>
    /// 실행 중에 적용할 폭 축소 스텝 (현재 폭 대비 비율).
    /// </summary>
    /// <remarks>
    /// 유효 범위는 0 &lt; step &lt; 1 이다. 네이티브가 범위를 다시 검사하므로 잘못된 값은
    /// 적용되지 않고 기존 값이 유지된다.
    /// </remarks>
    private double _ShrinkStepInput = 0.001;

    public double ShrinkStepInput
    {
        get => this._ShrinkStepInput;
        set => this.RaiseAndSetIfChanged(ref this._ShrinkStepInput, value);
    }

    private string _ShrinkStepStatus = string.Empty;

    /// <summary>마지막 적용 결과를 사용자에게 보여 주는 짧은 문구.</summary>
    public string ShrinkStepStatus
    {
        get => this._ShrinkStepStatus;
        set => this.RaiseAndSetIfChanged(ref this._ShrinkStepStatus, value);
    }

    #region Command

    public ICommand RunCommand { get; }

    public ICommand StopCommand { get; }

    /// <summary>탐색 단계의 폭 축소 스텝을 실행 중에 적용한다.</summary>
    public ICommand ApplyShrinkStepCommand { get; }

    public ICommand ScrollToTopCommand { get; }

    public ICommand ScrollToEndCommand { get; }

    public ICommand ExportStructureReportCommand { get; }

    #endregion

    #region Constructor

    public WorkspaceViewModel(MainWindowViewModel parentViewModel, SwarmInputData inputData)
    {
        ParentViewModel = parentViewModel;
        InputData = inputData;
        InitializePresets();
        InitializeSeeds();
        SelectedOption = OptimizerOptions.Simple;
        OutputLog = string.Empty;
        InputSummary = LoadInputSummary();
        if (InputSummary.SchemaVersion == 2 && InputSummary.GroupWorkers is int inputGroupWorkers)
        {
            // Custom Config의 초기값은 입력 계약을 그대로 따릅니다. 사용자가 값을
            // 변경했을 때만 실행 직전 V2 nesting.group_workers를 덮어씁니다.
            Config.GroupWorkers = inputGroupWorkers;
        }
        LoadShapes();

        DockingItem = new DockDocumentItemViewModel(this) { Title = inputData.ProjectName };

        var _canRun = this.WhenAnyValue(x => x.IsRunning, x => x.SeedCount, (running, count) => !running && count > 0);
        RunCommand = ReactiveCommand.CreateFromTask(RunAsync, _canRun, RxSchedulers.MainThreadScheduler);

        var _canStop = this.WhenAnyValue(x => x.IsRunning);
        StopCommand = ReactiveCommand.CreateFromTask(StopAsync, _canStop, RxSchedulers.MainThreadScheduler);

        // CustomConfig 를 고르면 이 저장소의 실측에서 가장 좋았던 값으로 채운다.
        // 한 번만 채우고, 그 뒤에는 사용자가 바꾼 값을 덮어쓰지 않는다 — 탭을
        // 오갈 때마다 편집 내용이 사라지면 쓸 수 없는 화면이 된다.
        var _recommendedApplied = false;
        this.WhenAnyValue(x => x.SelectedOption)
            .Where(option => option == OptimizerOptions.CustomConfig)
            .Subscribe(_ =>
            {
                if (_recommendedApplied)
                {
                    return;
                }
                _recommendedApplied = true;
                Config.ApplyRecommended();
            });

        // 실행 중이 아니어도 미리 설정해 둘 수 있다 — 다음 실행부터 반영된다.
        ApplyShrinkStepCommand = ReactiveCommand.Create(ApplyShrinkStep, outputScheduler: RxSchedulers.MainThreadScheduler);

        var _canExportStructureReport = this.WhenAnyValue(
            x => x.IsRunning,
            x => x.LastStructureReportJson,
            StructureReportExportService.CanExport);
        ExportStructureReportCommand = ReactiveCommand.Create(
            ExportStructureReport,
            _canExportStructureReport,
            RxSchedulers.MainThreadScheduler);

        this.ScrollToTopCommand = ReactiveCommand.Create<object>((o) =>
        {
            if (o is TextBox _tb)
            {
                _tb.ScrollToHome();
            }
        });

        this.ScrollToEndCommand = ReactiveCommand.Create<object>((o) =>
        {
            if (o is TextBox _tb)
            {
                _tb.ScrollToEnd();
            }
        });

        this._Subscriptions.Add((IDisposable)this.RunCommand);
        this._Subscriptions.Add((IDisposable)this.StopCommand);
        this._Subscriptions.Add((IDisposable)this.ApplyShrinkStepCommand);
        this._Subscriptions.Add((IDisposable)this.ScrollToTopCommand);
        this._Subscriptions.Add((IDisposable)this.ScrollToEndCommand);
        this._Subscriptions.Add((IDisposable)this.ExportStructureReportCommand);

        this.WhenAnyValue(x => x.ShowLeftPanel)
            .Subscribe(_ =>
            {
                this.RaisePropertyChanged(nameof(LeftPanelWidth));
                this.RaisePropertyChanged(nameof(LeftPanelMinWidth));
                this.RaisePropertyChanged(nameof(SplitterWidth));
            })
            .DisposeWith(this._Subscriptions);
    }

    #endregion

    /// <summary>
    /// Use Hints 로 배치를 받아들일 때 쓰는 겹침 허용(mm)입니다.
    /// <para>
    /// 상용 마커 변환본(0.025mm 격자)의 실측 최대 관통 깊이가 0.0954mm 라 0.1 이면 충분하고,
    /// 더 크게 잡으면 오히려 실패합니다 — 겹침 허용은 컨테이너도 같은 양만큼 줄이므로
    /// 원단 가장자리에 닿은 조각이 밖으로 나갑니다(실측: 0.2mm 에서 경계 위반 1건).
    /// </para>
    /// </summary>
    private const float HintPieceOverlapMm = 0.1f;

    private SwarmInputSummary LoadInputSummary()
    {
        try
        {
            return SwarmInputInspector.Parse(InputData.JsonSnapshot);
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException)
        {
            return SwarmInputSummary.Invalid($"입력 파일 읽기 실패: {exception.Message}");
        }
    }

    private void ExportStructureReport()
    {
        StructureReportExportResult result = StructureReportExportService.Save(
            LastStructureReportJson,
            OutputDir,
            InputData.ProjectName);
        if (result.IsSuccess)
        {
            AppendLog($"Structure report JSON 저장: {result.FilePath}");
        }
        else
        {
            AppendLog($"Structure report JSON 저장 실패: {result.ErrorMessage}");
        }
    }

    /// <summary>입력 JSON에서 shape 데이터 + hint 로드</summary>
    private void LoadShapes()
    {
        try
        {
            string json = InputData.JsonSnapshot;
            SwarmInputPlaces input = SwarmInputPlaces.Parse(json);
            var parsedHints = input.Items
                .Where(item => item.PlacedHint.HasValue)
                .ToDictionary(item => item.ItemId, item => item.PlacedHint!.Value);

            using var doc = JsonDocument.Parse(json);
            var root = doc.RootElement;
            var instance = root.TryGetProperty("instance", out var nestedInstance)
                ? nestedInstance
                : root;

            var parsedShapes = new Dictionary<int, List<(float X, float Y)>>();
            var unsupportedShapeIds = new List<int>();

            if (!instance.TryGetProperty("items", out JsonElement items)
                || items.ValueKind != JsonValueKind.Array)
            {
                throw new JsonException("Swarm 입력에 items 배열이 없습니다.");
            }

            foreach (var item in items.EnumerateArray())
            {
                int id = item.TryGetProperty("id", out JsonElement idElement)
                    && idElement.TryGetInt32(out int parsedId)
                    ? parsedId
                    : -1;
                if (id >= 0
                    && item.TryGetProperty("shape", out JsonElement shape)
                    && TryParseShape(shape, out List<(float X, float Y)> points))
                {
                    parsedShapes[id] = points;
                }
                else
                {
                    unsupportedShapeIds.Add(id);
                }
            }

            _Shapes = parsedShapes;
            _Hints = parsedHints;
            if (unsupportedShapeIds.Count > 0)
            {
                OutputLog = $"화면에서 지원하지 않는 shape item: {string.Join(", ", unsupportedShapeIds)}{Environment.NewLine}";
            }
        }
        catch (Exception exception)
        {
            OutputLog = $"입력 표시 실패: {exception.Message}{Environment.NewLine}";
            Console.WriteLine(exception);
        }
        RenderInputShapes();
    }

    private static bool TryParseShape(JsonElement shape, out List<(float X, float Y)> points)
    {
        points = [];
        if (shape.ValueKind != JsonValueKind.Object
            || !shape.TryGetProperty("type", out JsonElement typeElement)
            || !shape.TryGetProperty("data", out JsonElement data))
        {
            return false;
        }
        string? type = typeElement.GetString();
        if (type == "simple_polygon" && data.ValueKind == JsonValueKind.Array)
        {
            foreach (JsonElement point in data.EnumerateArray())
            {
                if (point.ValueKind != JsonValueKind.Array
                    || point.GetArrayLength() < 2
                    || !point[0].TryGetSingle(out float x)
                    || !point[1].TryGetSingle(out float y)
                    || !float.IsFinite(x)
                    || !float.IsFinite(y))
                {
                    points = [];
                    return false;
                }
                points.Add((x, y));
            }
            return points.Count >= 3;
        }

        if (type == "rectangle" && data.ValueKind == JsonValueKind.Object)
        {
            if (!data.TryGetProperty("x_min", out JsonElement xMinElement)
                || !data.TryGetProperty("y_min", out JsonElement yMinElement)
                || !data.TryGetProperty("width", out JsonElement widthElement)
                || !data.TryGetProperty("height", out JsonElement heightElement)
                || !xMinElement.TryGetSingle(out float minX)
                || !yMinElement.TryGetSingle(out float minY)
                || !widthElement.TryGetSingle(out float width)
                || !heightElement.TryGetSingle(out float height)
                || !float.IsFinite(minX)
                || !float.IsFinite(minY)
                || !float.IsFinite(width)
                || !float.IsFinite(height)
                || width <= 0
                || height <= 0)
            {
                return false;
            }
            float maxX = minX + width;
            float maxY = minY + height;
            points = [(minX, minY), (maxX, minY), (maxX, maxY), (minX, maxY)];
            return true;
        }

        return false;
    }
    private void RenderInputShapes()
    {
        var renders = new List<PlacedItemRender>();
        float maxX = 0;
        float maxY = 0;
        foreach (var (id, shape) in _Shapes)
        {
            List<(float, float)> xf = shape;
            if (_Hints.TryGetValue(id, out var h))
            {
                float minX = float.MaxValue, minY = float.MaxValue;
                foreach (var (x, y) in shape)
                {
                    if (x < minX) minX = x;
                    if (y < minY) minY = y;
                }

                float cos = MathF.Cos(h.RotAngleInRad), sin = MathF.Sin(h.RotAngleInRad);
                xf = new List<(float, float)>();
                foreach (var (x, y) in shape)
                {
                    float rx = x - minX, ry = y - minY;
                    xf.Add((rx * cos - ry * sin + h.TrX, rx * sin + ry * cos + h.TrY));
                }
            }
            var render = new PlacedItemRender(
                id,
                xf,
                Colors[Math.Abs(id % Colors.Length)]);
            renders.Add(render);
            maxX = MathF.Max(maxX, render.Bounds.Right);
            maxY = MathF.Max(maxY, render.Bounds.Bottom);
        }
        RenderedStripWidth = maxX > 0 ? maxX : 1000;
        RenderedMaxY = maxY;
        ReplaceRenderedItems(renders);
    }

    private RenderSnapshot ParseRenderSnapshot(
        string json,
        bool parseDensity,
        IReadOnlySet<int>? collisionItemIds)
    {
        using var doc = JsonDocument.Parse(json);
        var root = doc.RootElement;
        float stripWidth = root.GetProperty("strip_width").GetSingle();
        var items = root.GetProperty("layout").GetProperty("placed_items");
        float? density = null;
        if (parseDensity && root.TryGetProperty("density", out JsonElement densityElement))
        {
            density = densityElement.GetSingle();
        }

        var renders = new List<PlacedItemRender>();
        float maxY = 0;
        try
        {
            foreach (var pi in items.EnumerateArray())
            {
                int id = pi.GetProperty("item_id").GetInt32();
                if (!_Shapes.TryGetValue(id, out var shape)) continue;

                var trans = pi.GetProperty("transformation");
                float rot = trans.GetProperty("rotation").GetSingle();
                var tr = trans.GetProperty("translation");
                float tx = tr[0].GetSingle(), ty = tr[1].GetSingle();

                float rad = rot * MathF.PI / 180f;
                float cos = MathF.Cos(rad), sin = MathF.Sin(rad);
                var xf = new List<(float, float)>(shape.Count);
                foreach (var (x, y) in shape)
                {
                    xf.Add((x * cos - y * sin + tx, x * sin + y * cos + ty));
                }

                var render = new PlacedItemRender(
                    id,
                    xf,
                    Colors[Math.Abs(id % Colors.Length)],
                    collisionItemIds?.Contains(id) == true
                        ? SKColor.Parse("#8B0000")
                        : SKColors.White.WithAlpha(140));
                renders.Add(render);
                maxY = MathF.Max(maxY, render.Bounds.Bottom);
            }
        }
        catch
        {
            DisposeRenderItems(renders);
            throw;
        }

        return new RenderSnapshot(renders, stripWidth, maxY, density);
    }

    private async Task ApplySolutionJsonAsync(
        string json,
        bool parseDensity,
        float? reportedDensity,
        IReadOnlyCollection<int> collisionItemIds,
        int generation,
        CancellationToken cancellationToken)
    {
        long sequence = Interlocked.Increment(ref _SnapshotSequence);
        int[] collisions = collisionItemIds.Distinct().ToArray();
        RenderSnapshot snapshot = await Task.Run(
            () => ParseRenderSnapshot(json, parseDensity, collisions.ToHashSet()),
            cancellationToken);

        await ApplyPreparedSolutionAsync(
            json,
            snapshot,
            reportedDensity,
            collisions,
            generation,
            sequence,
            cancellationToken);
    }

    private async Task ApplySolutionJsonAsync(
        string json,
        RenderSnapshot snapshot,
        float? reportedDensity,
        IReadOnlyCollection<int> collisionItemIds,
        int generation,
        CancellationToken cancellationToken)
    {
        long sequence = Interlocked.Increment(ref _SnapshotSequence);
        int[] collisions = collisionItemIds.Distinct().ToArray();
        await ApplyPreparedSolutionAsync(
            json,
            snapshot,
            reportedDensity,
            collisions,
            generation,
            sequence,
            cancellationToken);
    }

    private async Task ApplyPreparedSolutionAsync(
        string json,
        RenderSnapshot snapshot,
        float? reportedDensity,
        int[] collisions,
        int generation,
        long sequence,
        CancellationToken cancellationToken)
    {

        bool applied = false;
        await RunOnMainThreadAsync(() =>
        {
            if (_IsDisposed
                || generation != Volatile.Read(ref _RunGeneration)
                || sequence < _LastAppliedSnapshotSequence
                || cancellationToken.IsCancellationRequested)
            {
                return;
            }

            _LastAppliedSnapshotSequence = sequence;
            LastSolutionJson = json;
            RenderedStripWidth = snapshot.StripWidth;
            RenderedMaxY = snapshot.MaxY;
            RenderedCollisions = collisions.Length;
            if (reportedDensity.HasValue)
            {
                RenderedDensity = reportedDensity.Value;
            }
            else if (snapshot.Density.HasValue)
            {
                RenderedDensity = snapshot.Density.Value;
            }

            // RenderedItems를 마지막에 교체해 View가 완성된 snapshot만 한 번 invalidate하도록 합니다.
            ReplaceRenderedItems(snapshot.Items);
            applied = true;
        });

        if (!applied)
        {
            DisposeRenderItems(snapshot.Items);
        }
    }

    private void ReplaceRenderedItems(List<PlacedItemRender>? next)
    {
        List<PlacedItemRender>? previous = RenderedItems;
        RenderedItems = next;
        if (!ReferenceEquals(previous, next))
        {
            DisposeRenderItems(previous);
        }
    }

    private static void DisposeRenderItems(IEnumerable<PlacedItemRender>? items)
    {
        if (items is null)
        {
            return;
        }

        foreach (PlacedItemRender item in items)
        {
            item.Dispose();
        }
    }

    private static Task RunOnMainThreadAsync(Action action)
    {
        var dispatcher = Application.Current?.Dispatcher;
        if (dispatcher is null || dispatcher.CheckAccess())
        {
            action();
            return Task.CompletedTask;
        }

        return dispatcher.InvokeAsync(action).Task;
    }

    public void AppendLog(string line)
    {
        _ = RunOnMainThreadAsync(() =>
        {
            string prepended = string.Concat(line, Environment.NewLine, OutputLog);
            OutputLog = TrimNewestLogPrefix(prepended);
        });
    }

    private static string TrimNewestLogPrefix(string log)
    {
        if (log.Length <= MaxRunLogChars)
        {
            return log;
        }

        int retainedLength = Math.Min(RetainedRunLogChars, log.Length);
        int lineBoundary = log.LastIndexOf('\n', retainedLength - 1, retainedLength);
        return lineBoundary >= 0
            ? log[..(lineBoundary + 1)]
            : log[..retainedLength];
    }

    public string GetMakeSvg()
    {
        if (RenderedItems == null) { return string.Empty; }

        var path = Path.Combine(OutputDir, $"{InputData.ProjectName}.svg");
        Directory.CreateDirectory(OutputDir);
        var sb = new StringBuilder();
        float h = 0;
        foreach (var ri in RenderedItems)
        {
            foreach (var (_, y) in ri.Polygon) if (y > h) h = y;
        }

        float w = RenderedStripWidth;
        sb.AppendLine($"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {w:F1} {h:F1}\" width=\"{w/2:F1}\" height=\"{h/2:F1}\">");
        sb.AppendLine($"<rect width=\"{w:F1}\" height=\"{h:F1}\" fill=\"#232325\"/>");

        foreach (var ri in RenderedItems)
        {
            sb.Append($"<polygon points=\"");
            foreach (var (x, y) in ri.Polygon)
            {
                sb.Append($"{x:F2},{y:F2} ");
            }

            var c = ri.Color;
            var border = ri.BorderColor;
            sb.AppendLine($"\" fill=\"#{c.Red:X2}{c.Green:X2}{c.Blue:X2}\" stroke=\"#{border.Red:X2}{border.Green:X2}{border.Blue:X2}\" stroke-width=\"1.2\" stroke-opacity=\"{border.Alpha / 255f:F2}\"/>");
        }
        // strip
        sb.AppendLine($"<line x1=\"{w:F1}\" y1=\"0\" x2=\"{w:F1}\" y2=\"{h:F1}\" stroke=\"#e94560\" stroke-width=\"2\" stroke-dasharray=\"8,4\"/>");
        sb.AppendLine($"<text x=\"4\" y=\"52\" fill=\"#ffffff\" font-size=\"48\" font-family=\"Consolas\">Strip: {w:F1}  Density: {RenderedDensity*100:F1}%</text>");
        sb.AppendLine("</svg>");

        return sb.ToString();
    }

    /// <summary>Init 또는 현재 native phase에 취소를 요청합니다.</summary>
    private async Task StopAsync()
    {
        _IsStopRequested = true;
        if (RequestMultiSeedStop())
        {
            AppendLog("모든 시드 중단 요청: 실행 중인 단계와 콜백 정리를 기다립니다. Init 중인 시드에도 중지를 전달합니다.");
            Task completion;
            lock (_OptimizerSync) completion = _RunCompletion.Task;
            await WaitForMultiSeedStopAsync(completion);
            AppendLog("모든 시드 중단 완료 · 마지막 인증 배치 유지");
            return;
        }
        SwarmStepOptimizer? optimizer;
        lock (_OptimizerSync)
        {
            optimizer = _StepOptimizer;
        }

        if (optimizer is null || !optimizer.IsInitialized)
        {
            // Init 은 핸들이 생기기 전이라 phase stop 이 닿지 않는다. 네이티브 구성 루프를
            // 조각 경계에서 끊는 전역 취소를 건다. (optimizer 가 아직 없으면 RunAsync 가
            // _IsStopRequested 를 보고 phase 진입을 막는다.)
            optimizer?.CancelInit();
            AppendLog("중단 요청: Init 을 취소합니다.");
            return;
        }

        AppendLog("중단 요청 중...");
        await Task.Run(optimizer.RequestStopAndWait);
        AppendLog("중단 완료");
    }

    private async Task RunAsync()
    {
        if (_IsDisposed || IsRunning || SeedCount == 0)
        {
            return;
        }

        int generation = Interlocked.Increment(ref _RunGeneration);
        var runCompletion = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        lock (_OptimizerSync)
        {
            _RunCompletion = runCompletion;
        }
        _SnapshotCancellation.Cancel();
        _SnapshotCancellation.Dispose();
        _SnapshotCancellation = new CancellationTokenSource();
        CancellationToken snapshotToken = _SnapshotCancellation.Token;
        _IsStopRequested = false;
        _LastAppliedSnapshotSequence = 0;

        IsRunning = true;
        StopPhaseClock();
        PhaseElapsedText = string.Empty;
        OutputLog = string.Empty;
        LastSolutionJson = string.Empty;
        LastStructureReportJson = string.Empty;
        StructureReportSummary = null;
        ReplaceRenderedItems(null);
        RenderedStripWidth = 0;
        RenderedMaxY = 0;
        RenderedDensity = 0;
        RenderedCollisions = 0;
        RenderedLoss = 0;
        MultiSeedStatus = string.Empty;
        WinningSeedText = string.Empty;
        ResolvedSeedsText = string.Empty;
        lock (_StructureProgress)
        {
            _StructureProgress.Reset();
        }
        StructureProgressText = "Structure 설정 확인 중";

        bool useHints = UseHints;
        OptimizerOptions selectedOption = SelectedOption;
        int? groupWorkersOverride = selectedOption == OptimizerOptions.CustomConfig
            ? Config.GroupWorkers
            : null;
        var log = new StringBuilder();

        try
        {
            SwarmConfig config = selectedOption == OptimizerOptions.CustomConfig
                ? Config.ToSwarmConfig()
                : SwarmOptimizer.GetDefaultConfig();
            // 실행 순서는 Structure 를 켜지 않은 실행에서만 쓴다. 비어 있으면 여기서 실행 준비 오류가 난다.
            if (!config.StructureFirst.Enabled)
            {
                config.PhaseSequence = OptimizationPhaseSequence.Validate(config.PhaseSequence);
            }
            StructureProgressText = config.StructureFirst.Enabled
                ? "Structure 시작 준비 · Init baseline 대기"
                : "Structure OFF · " + string.Join(" → ", config.PhaseSequence.Select(OptimizationPhaseSequence.DisplayName));
            Directory.CreateDirectory(OutputDir);
            string title = selectedOption == OptimizerOptions.CustomConfig
                ? "=== Custom Config (Step API) ==="
                : "=== Default Config ===";
            AppendRunLine(log, title);
            ulong[] seeds = SeedValues.ToArray();
            if (seeds.Length == 1)
            {
                // 시드 하나: 기존 단일 실행 경로. 상단 시드 목록의 값이 설정의 시드를 덮어쓴다.
                config.RngSeed = seeds[0];
                ResolvedSeedsText = seeds[0] == 0 ? "실행 시드: 무작위" : $"실행 시드: {seeds[0]}";
                await Task.Run(
                    () => RunPipelineAsync(
                        log,
                        config,
                        useHints,
                        groupWorkersOverride,
                        generation,
                        snapshotToken),
                    snapshotToken);
            }
            else
            {
                // 시드 여러 개: 시드마다 독립 핸들로 동시에 돌리고 인증된 최상 결과를 표시한다.
                await RunMultiSeedAsync(
                    log,
                    config,
                    seeds,
                    InputData.JsonSnapshot,
                    useHints,
                    groupWorkersOverride,
                    generation,
                    snapshotToken);
            }
        }
        catch (OperationCanceledException) when (snapshotToken.IsCancellationRequested)
        {
            AppendRunLine(log, "실행 결과 표시가 취소되었습니다.");
        }
        catch (Exception exception)
        {
            AppendRunLine(log, $"오류: {exception}");
        }
        finally
        {
            try
            {
                await PublishRunLogAsync(log, generation);
                await RunOnMainThreadAsync(() =>
                {
                    if (generation == Volatile.Read(ref _RunGeneration) && !_IsDisposed)
                    {
                        StopPhaseClock();
                        IsRunning = false;
                    }
                });
            }
            finally
            {
                runCompletion.TrySetResult(true);
            }
        }
    }

    private static TaskCompletionSource<bool> CompletedRunCompletion()
    {
        var completion = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        completion.SetResult(true);
        return completion;
    }

    private void OnSolutionFound(
        StringBuilder log,
        SolutionEventArgs eventArgs,
        bool structureEnabled,
        int generation,
        CancellationToken cancellationToken)
    {
        if (_IsDisposed || generation != Volatile.Read(ref _RunGeneration))
        {
            return;
        }

        string message =
            $"  [{eventArgs.ReportType,-12}] width={eventArgs.StripWidth:F3}, density={eventArgs.Density * 100:F2}%";
        if (!string.IsNullOrEmpty(eventArgs.SolutionJson))
        {
            message += $"  json={eventArgs.SolutionJson.Length} chars";
        }

        AppendRunLine(log, message);
        _ = PublishRunLogAsync(log, generation);

        if (eventArgs.HasLoss)
        {
            float loss = eventArgs.Loss;
            _ = RunOnMainThreadAsync(() =>
            {
                if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                {
                    RenderedLoss = loss;
                }
            });
        }

        if (structureEnabled)
        {
            string progressText;
            lock (_StructureProgress)
            {
                _StructureProgress.Observe((int)eventArgs.ReportType);
                progressText = _StructureProgress.DisplayText;
            }
            _ = RunOnMainThreadAsync(() =>
            {
                if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                {
                    StructureProgressText = progressText;
                }
            });
        }

        if (string.IsNullOrEmpty(eventArgs.SolutionJson))
        {
            return;
        }

        string solutionJson = eventArgs.SolutionJson;
        int[] collisionIds = eventArgs.CollisionStatus
            .Where(status => status.IsCollision)
            .Select(status => status.ItemID)
            .ToArray();
        float density = eventArgs.Density;

        // 결합 JSON 파싱은 raw callback 큐에서 이미 수행됐다. 솔루션 geometry 파싱도 별도 작업으로
        // 넘겨 연속 callback 전달과 UI thread를 막지 않으며, sequence gate가 늦게 끝난 snapshot을 버린다.
        _ = ApplyCallbackSolutionAsync(
            eventArgs,
            collisionIds,
            generation,
            cancellationToken,
            log);
    }

    private async Task ApplyCallbackSolutionAsync(
        SolutionEventArgs eventArgs,
        int[] collisionIds,
        int generation,
        CancellationToken cancellationToken,
        StringBuilder log)
    {
        try
        {
            await ApplySolutionJsonAsync(
                eventArgs.SolutionJson!,
                BuildCallbackRenderSnapshot(eventArgs, collisionIds.ToHashSet()),
                eventArgs.Density,
                collisionIds,
                generation,
                cancellationToken);
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
        }
        catch (Exception exception)
        {
            AppendRunLine(log, $"  [Callback 표시 실패] {exception.Message}");
            await PublishRunLogAsync(log, generation);
        }
    }

    private RenderSnapshot BuildCallbackRenderSnapshot(
        SolutionEventArgs eventArgs,
        IReadOnlySet<int> collisionItemIds)
    {
        var renders = new List<PlacedItemRender>();
        float maxY = 0;
        try
        {
            foreach (SolutionEventArgs.ParsedPlacement placement in eventArgs.ParsedPlacements)
            {
                if (!_Shapes.TryGetValue(placement.ItemId, out var shape))
                {
                    continue;
                }

                float rad = placement.RotationDegrees * MathF.PI / 180f;
                float cos = MathF.Cos(rad), sin = MathF.Sin(rad);
                var transformed = new List<(float X, float Y)>(shape.Count);
                foreach (var (x, y) in shape)
                {
                    transformed.Add((
                        x * cos - y * sin + placement.TranslationX,
                        x * sin + y * cos + placement.TranslationY));
                }

                var render = new PlacedItemRender(
                    placement.ItemId,
                    transformed,
                    Colors[Math.Abs(placement.ItemId % Colors.Length)],
                    collisionItemIds.Contains(placement.ItemId)
                        ? SKColor.Parse("#8B0000")
                        : SKColors.White.WithAlpha(140));
                renders.Add(render);
                maxY = MathF.Max(maxY, render.Bounds.Bottom);
            }
        }
        catch
        {
            DisposeRenderItems(renders);
            throw;
        }

        return new RenderSnapshot(
            renders,
            eventArgs.ParsedStripWidth ?? eventArgs.StripWidth,
            maxY,
            eventArgs.Density);
    }

    /// <summary>
    /// raw callback을 수신 순서대로 파싱합니다. 네이티브 callback 스레드는 이 메서드를 실행하지 않고
    /// <see cref="ChannelWriter{T}.TryWrite(T)"/>까지만 수행합니다.
    /// </summary>
    private async Task ProcessRawCallbackQueueAsync(
        ChannelReader<RawCallbackReport> reader,
        StringBuilder log,
        bool structureEnabled,
        int generation,
        CancellationToken cancellationToken,
        Action onSolutionSnapshot)
    {
        await foreach (RawCallbackReport rawReport in reader.ReadAllAsync())
        {
            if (_IsDisposed
                || generation != Volatile.Read(ref _RunGeneration)
                || cancellationToken.IsCancellationRequested)
            {
                continue;
            }

            SolutionEventArgs eventArgs;
            try
            {
                eventArgs = await Task.Run(
                    () => SolutionEventArgs.FromCombinedJson(
                        rawReport.ReportType,
                        rawReport.StripWidth,
                        rawReport.Density,
                        rawReport.CombinedJson),
                    cancellationToken).ConfigureAwait(false);
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                continue;
            }

            if (_IsDisposed || generation != Volatile.Read(ref _RunGeneration))
            {
                continue;
            }

            OnSolutionFound(
                log,
                eventArgs,
                structureEnabled,
                generation,
                cancellationToken);
            if (!string.IsNullOrEmpty(eventArgs.SolutionJson))
            {
                onSolutionSnapshot();
            }
        }
    }

    /// <summary>
    /// phase 반환 뒤 네이티브 callback을 해제하고, 이미 복사된 raw 보고의 파싱 전달이 끝날 때까지
    /// 기다립니다. 이 순서로 최종 snapshot이 이전 callback보다 항상 높은 sequence를 갖습니다.
    /// </summary>
    private static async Task DetachRawCallbackQueueAsync(
        SwarmStepOptimizer optimizer,
        Channel<RawCallbackReport> channel,
        Task callbackConsumer)
    {
        try
        {
            optimizer.SetRawCallback(null, emitJson: false);
        }
        finally
        {
            // unregister가 실패해도 reader가 영원히 대기하지 않도록 writer는 반드시 닫는다.
            channel.Writer.TryComplete();
        }

        await callbackConsumer.ConfigureAwait(false);
    }

    private async Task RunPipelineAsync(
        StringBuilder log,
        SwarmConfig config,
        bool useHints,
        int? groupWorkersOverride,
        int generation,
        CancellationToken cancellationToken)
    {
        int snapshots = 0;
        string inputJson = InputData.JsonSnapshot;
        SwarmInputSummary summary = SwarmInputInspector.Parse(inputJson);
        if (!summary.IsValid)
        {
            throw new InvalidDataException(summary.ErrorMessage ?? "Swarm 입력을 분석하지 못했습니다.");
        }
        await RunOnMainThreadAsync(() =>
        {
            if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
            {
                InputSummary = summary;
            }
        });

        SwarmInputPreparationResult preparation = SwarmInputInspector.PrepareForRun(
            inputJson,
            useHints,
            groupWorkersOverride);
        if (!preparation.IsSuccess)
        {
            throw new InvalidDataException(preparation.ErrorMessage ?? "실행용 Swarm 입력을 준비하지 못했습니다.");
        }

        SwarmInputSummary executionSummary = SwarmInputInspector.Parse(preparation.Json);
        if (!executionSummary.IsValid)
        {
            throw new InvalidDataException(
                executionSummary.ErrorMessage ?? "준비된 Swarm 입력을 분석하지 못했습니다.");
        }
        await RunOnMainThreadAsync(() =>
        {
            if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
            {
                InputSummary = executionSummary;
            }
        });

        CPlacedHint[] hints = executionSummary.SchemaVersion == 1
            ? SwarmInputPlaces.Parse(inputJson).ToNativeHints(useHints)
            : [];

        // Use Hints 를 켜고 입력이 실제 배치를 담고 있으면, 그 좌표를 **쓸 수 있는**
        // 충돌 설정으로 맞춘다.
        //
        // 상용 마커를 변환한 입력은 조각이 원단 폭에 정확히 꽉 차 있다. 기본값인
        // PolySimpl 0.001 은 충돌 도형을 최대 0.9mm 부풀리므로 그 배치는 "원단 밖"
        // 으로 판정되고, 네이티브는 힌트를 통째로 버린 뒤 LBF 로 처음부터 다시 짓는다.
        // 분리기로도 못 고친다 — 부풀린 도형이 원단보다 커서 밀어 넣을 자리가 없다.
        //
        // 겹침 허용 0.1mm 는 0.025mm 격자 양자화가 남긴 미세 관통을 흡수한다.
        // 더 크게 주면 오히려 실패한다: 겹침 허용은 컨테이너도 같이 줄이므로
        // 가장자리 조각이 밖으로 나간다(실측: 0.2mm 에서 경계 위반 1건).
        //
        // 사용자가 직접 정한 값은 건드리지 않는다. 기본값일 때만 맞춘다.
        if (hints.Length > 0)
        {
            if (config.PolySimplTolerance is float simpl && simpl >= 0f)
            {
                config.PolySimplTolerance = null;
                AppendRunLine(
                    log,
                    $"  Hint 설정 보정: PolySimpl {simpl} → 끔 " +
                    "(도형 팽창이 힌트 배치를 원단 밖으로 밀어냅니다)");
            }

            if (config.FabricTolerance is { PieceOverlapMm: <= 0f })
            {
                config.FabricTolerance.PieceOverlapMm = HintPieceOverlapMm;
                AppendRunLine(
                    log,
                    $"  Hint 설정 보정: Piece Overlap → {HintPieceOverlapMm} mm " +
                    "(격자 양자화가 남긴 미세 관통 흡수)");
            }
        }

        AppendRunLine(
            log,
            $"  Schema=V{executionSummary.SchemaVersion}, Mode={executionSummary.NestingMode}, UseHints={useHints}, NativeHints={hints.Length}" +
            (executionSummary.GroupWorkers is int workers ? $", GroupWorkers={workers}" : string.Empty));
        if (preparation.RemovedMovableHintCount > 0 || preparation.RetainedRequiredHintCount > 0)
        {
            AppendRunLine(
                log,
                $"  Hint 정책: movable 제거 {preparation.RemovedMovableHintCount}, lock/PlaceGroup pose 유지 {preparation.RetainedRequiredHintCount}");
        }

        var optimizer = new SwarmStepOptimizer();
        Channel<RawCallbackReport>? callbackChannel = null;
        Task? callbackConsumer = null;
        bool callbackDetached = false;
        SwarmStepOptimizer? previous;
        lock (_OptimizerSync)
        {
            previous = _StepOptimizer;
            _StepOptimizer = optimizer;
        }

        // 네이티브 로그를 실행 로그로 끌어온다.
        //
        // Rust 의 `log` 크레이트는 로거가 등록되지 않으면 모든 매크로가 no-op 이다.
        // 이걸 부르기 전까지 `[INIT] NFP-LBF 시드 채택/기각` 같은 진단이 한 줄도
        // 나오지 않았고, 그래서 "설정이 전달되지 않았다"와 "전달됐지만 기각됐다"를
        // 구분할 수 없었다.
        //
        // 싱크는 rayon 워커를 포함한 아무 스레드에서나 불린다. `AppendRunLine` 이
        // StringBuilder 를 lock 하므로 그대로 넘겨도 안전하다.
        bool logSinkInstalled = SwarmOptimizer.SetLogSink(
            (level, message) =>
            {
                AppendRunLine(log, $"  [native:{NativeLogLabel(level)}] {message}");
                _ = PublishRunLogAsync(log, generation);
            });
        if (!logSinkInstalled)
        {
            AppendRunLine(log, "  경고: 네이티브 로그 싱크를 설치하지 못했습니다 (다른 로거가 이미 설치됨).");
        }

        try
        {
            previous?.Dispose();
            var initClock = System.Diagnostics.Stopwatch.StartNew();
            AppendRunLine(log, "  [Init] 시작: 전처리 → 초기 배치 → 검증 → 화면 표시");
            await PublishRunLogAsync(log, generation);
            InitialPreviewResult initResult = await InitialPreviewFlow.RunAsync(
                Task.Run(() => !_IsStopRequested && !cancellationToken.IsCancellationRequested
                    && optimizer.InitFromJson(preparation.Json, config, hints)),
                async () =>
                {
                    AppendRunLine(log, $"  [Init] 네이티브 완료 {initClock.Elapsed.TotalSeconds:F2}초; 결과 표시 준비");
                    string initialJson = optimizer.GetSolutionJson();
                    if (string.IsNullOrWhiteSpace(initialJson))
                        throw new InvalidDataException("Init은 성공했지만 초기 배치 JSON이 비어 있습니다.");
                    int[] initialCollisions = config.CallbackOptions.CollectCollisionStatus
                        ? optimizer.GetCollisionStatus()
                            .Where(status => status.IsCollision)
                            .Select(status => status.ItemID).ToArray()
                        : Array.Empty<int>();
                    await ApplySolutionJsonAsync(initialJson, parseDensity: true,
                        reportedDensity: null, initialCollisions, generation, cancellationToken);
                    AppendRunLine(log, $"  [Init] 초기 배치 표시 완료 {initClock.Elapsed.TotalSeconds:F2}초");
                    await PublishRunLogAsync(log, generation);
                },
                () => _IsDisposed || generation != Volatile.Read(ref _RunGeneration)
                    || cancellationToken.IsCancellationRequested,
                () => _IsStopRequested,
                async elapsed =>
                {
                    string status = _IsStopRequested
                        ? $"Init 중단 처리 중 · 네이티브 정리 대기 {elapsed.TotalSeconds:F0}초"
                        : $"Init 진행 중 · 전처리/배치/검증 포함 {elapsed.TotalSeconds:F0}초";
                    await RunOnMainThreadAsync(() =>
                    {
                        if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                            StructureProgressText = status;
                    });
                    await PublishRunLogAsync(log, generation);
                }, optimizer.CancelInit);
            if (initResult == InitialPreviewResult.Failed)
            {
                if (_IsStopRequested || optimizer.LastInitWasCancelled)
                {
                    AppendRunLine(log, $"  -> Init 중단됨 ({initClock.Elapsed.TotalSeconds:F2}초)");
                    return;
                }
                AppendRunLine(
                    log,
                    $"  -> Init 실패: code={optimizer.LastErrorCode}, {optimizer.LastError}");
                return;
            }

            if (initResult == InitialPreviewResult.Obsolete) return;
            if (initResult == InitialPreviewResult.Stopped)
            {
                AppendRunLine(log, "  -> 중단 요청에 따라 초기 배치를 표시하고 종료합니다. 다음 phase는 시작하지 않습니다.");
                await RunOnMainThreadAsync(() =>
                {
                    if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                        StructureProgressText = "Init 결과 표시 완료 · 중단됨";
                });
                return;
            }

            callbackChannel = Channel.CreateUnbounded<RawCallbackReport>(
                new UnboundedChannelOptions
                {
                    SingleReader = true,
                    // size-region lane이 병렬 보고할 수 있으므로 writer를 단일 스레드로 가정하지 않는다.
                    SingleWriter = false,
                    AllowSynchronousContinuations = false
                });
            ChannelWriter<RawCallbackReport> callbackWriter = callbackChannel.Writer;
            callbackConsumer = ProcessRawCallbackQueueAsync(
                callbackChannel.Reader,
                log,
                config.StructureFirst.Enabled,
                generation,
                cancellationToken,
                () => Interlocked.Increment(ref snapshots));

            optimizer.SetRawCallback((reportType, stripWidth, density, combinedJson) =>
            {
                // native callback 포인터는 반환 즉시 무효가 된다. SwarmStepOptimizer가 이미 관리
                // 문자열로 복사했으므로 여기서는 비차단 TryWrite만 수행하고 즉시 반환한다.
                if (!_IsDisposed
                    && generation == Volatile.Read(ref _RunGeneration)
                    && !cancellationToken.IsCancellationRequested)
                {
                    callbackWriter.TryWrite(
                        new RawCallbackReport(reportType, stripWidth, density, combinedJson));
                }
            }, emitJson: true);

            if (_IsStopRequested || _IsDisposed)
            {
                optimizer.RequestStop();
                AppendRunLine(log, "  -> phase 시작 전에 중단되었습니다.");
                return;
            }

            // Init 이후 phase의 경과 시간만 표시한다.
            await RunOnMainThreadAsync(() =>
            {
                if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                    StartPhaseClock();
            });

            if (config.StructureFirst.Enabled)
            {
                AppendRunLine(
                    log,
                    $"  [Structure First] 시작 (Motif={config.StructureFirst.MotifEnabled})...");
                int result = optimizer.StructurePhase();
                await DetachRawCallbackQueueAsync(
                    optimizer,
                    callbackChannel,
                    callbackConsumer).ConfigureAwait(false);
                callbackDetached = true;
                if (_IsStopRequested || _IsDisposed)
                {
                    AppendRunLine(log, "  [Structure First] 중단됨");
                    await RunOnMainThreadAsync(() =>
                    {
                        if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                        {
                            StructureProgressText = "Structure 중단됨";
                        }
                    });
                }
                else if (result == 0)
                {
                    string reportJson = optimizer.GetStructureReportJson();
                    StructureReportSummary reportSummary = StructureReportInspector.Parse(reportJson);
                    if (reportSummary.IsValid)
                    {
                        await RunOnMainThreadAsync(() =>
                        {
                            if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                            {
                                LastStructureReportJson = reportJson;
                                StructureReportSummary = reportSummary;
                                StructureProgressText = "Structure 완료 · schema 3 report 준비됨";
                            }
                        });
                        AppendRunLine(log, $"  [Structure First] 완료, report={reportJson.Length} chars");
                    }
                    else
                    {
                        AppendRunLine(
                            log,
                            $"  [Structure First] 완료, report 분석 실패: {reportSummary.ErrorMessage}");
                        await RunOnMainThreadAsync(() =>
                        {
                            if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                            {
                                StructureReportSummary = reportSummary;
                                StructureProgressText = "Structure 완료 · report 분석 실패";
                            }
                        });
                    }
                }
                else
                {
                    AppendRunLine(
                        log,
                        $"  [Structure First] 실패: code={optimizer.LastErrorCode}, {optimizer.LastError}");
                    int errorCode = optimizer.LastErrorCode;
                    await RunOnMainThreadAsync(() =>
                    {
                        if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                        {
                            StructureProgressText = $"Structure 실패 · code {errorCode}";
                        }
                    });
                }
            }
            else
            {
                AppendRunLine(
                    log,
                    $"  RngSeed={config.RngSeed}, ExplTime={config.Exploration.TimeLimitSecs}s, CmprTime={config.Compression.TimeLimitSecs}s, CallbackLoss={config.CallbackOptions.CollectLoss}, CallbackCollision={config.CallbackOptions.CollectCollisionStatus}");
                await RunLegacyPhasesAsync(optimizer, config, log);
                await DetachRawCallbackQueueAsync(
                    optimizer,
                    callbackChannel,
                    callbackConsumer).ConfigureAwait(false);
                callbackDetached = true;
            }

            string finalJson = optimizer.GetSolutionJson();
            if (!string.IsNullOrWhiteSpace(finalJson))
            {
                int[] finalCollisions = (config.StructureFirst.Enabled
                        || config.CallbackOptions.CollectCollisionStatus)
                    ? optimizer.GetCollisionStatus()
                        .Where(status => status.IsCollision)
                        .Select(status => status.ItemID)
                        .ToArray()
                    : Array.Empty<int>();
                if (config.StructureFirst.Enabled && finalCollisions.Length > 0)
                {
                    AppendRunLine(
                        log,
                        $"  [Structure First] 최종 인증 거부: 충돌 item {string.Join(", ", finalCollisions)}");
                    throw new InvalidDataException(
                        "Structure가 충돌 상태의 최종 해를 반환하여 화면/내보내기에 적용하지 않았습니다.");
                }
                await ApplySolutionJsonAsync(
                    finalJson,
                    parseDensity: true,
                    reportedDensity: null,
                    finalCollisions,
                    generation,
                    cancellationToken);
                AppendRunLine(log, $"최종 솔루션 JSON: {finalJson.Length} chars");
            }

            AppendRunLine(log, $"  -> {Volatile.Read(ref snapshots)} snapshots");
        }
        finally
        {
            // 싱크는 `log` StringBuilder 를 캡처하고 있다. 실행이 끝나면 반드시 떼어
            // 다음 실행의 로그가 이전 버퍼로 새지 않게 한다.
            if (logSinkInstalled)
            {
                ulong dropped = SwarmOptimizer.DroppedLogCount;
                SwarmOptimizer.SetLogSink(null);
                if (dropped > 0)
                {
                    AppendRunLine(log, $"  경고: 네이티브 로그 {dropped}줄이 전달되지 못했습니다.");
                }
            }

            // phase 호출이 반환된 뒤 callback을 먼저 해제하고 raw queue를 닫아야 delegate와 native
            // JSON 포인터 수명이 끝난 뒤에도 남은 관리 문자열을 안전하게 처리할 수 있다.
            try
            {
                if (!callbackDetached && callbackChannel is not null && callbackConsumer is not null)
                {
                    await DetachRawCallbackQueueAsync(
                        optimizer,
                        callbackChannel,
                        callbackConsumer).ConfigureAwait(false);
                }
            }
            finally
            {
                try
                {
                    optimizer.Dispose();
                }
                finally
                {
                    lock (_OptimizerSync)
                    {
                        if (ReferenceEquals(_StepOptimizer, optimizer))
                        {
                            _StepOptimizer = null;
                        }
                    }
                }
            }
        }
    }

    private async Task RunLegacyPhasesAsync(
        SwarmStepOptimizer optimizer,
        SwarmConfig config,
        StringBuilder log)
    {
        // N06 초기배치 페이즈 — 켜져 있으면 LBF 초기해를 N06 엔진 결과로 교체한 뒤
        // 기존 Explore→Compress→Polish 를 그 해에서 잇는다. 실패해도(예: Grouped 입력)
        // LBF 초기해가 그대로 남아 있으므로 이후 단계는 계속 진행한다.
        if (config.InitialPlacement.Enabled)
        {
            if (_IsStopRequested || _IsDisposed)
            {
                AppendRunLine(log, "  [Initial] 중단됨");
                return;
            }

            AppendRunLine(
                log,
                $"  [Initial] 시작... (예산 {config.InitialPlacement.TimeLimitSecs}s, rescue {config.InitialPlacement.RescueAfter}, stall {config.InitialPlacement.StallCap}, kick {config.InitialPlacement.Kicks})");
            int initialResult = optimizer.InitialPhase();
            if (_IsStopRequested || _IsDisposed)
            {
                AppendRunLine(log, "  [Initial] 중단됨");
                return;
            }
            AppendRunLine(
                log,
                initialResult == 0
                    ? "  [Initial] 완료: True"
                    : $"  [Initial] 실패(code={optimizer.LastErrorCode}): {optimizer.LastError} — LBF 초기해로 계속");
        }

        if (_IsStopRequested || _IsDisposed)
        {
            AppendRunLine(log, "  [실행 순서] 중단됨");
            return;
        }

        // 설정한 순서대로 실행한다. 같은 단계가 여러 번 있을 수 있고, 각 호출은 직전 호출이
        // 남긴 인증된 배치에서 출발한다. 중단하거나 한 단계가 실패하면 나머지는 생략한다.
        OptimizationPhaseSequence.Run(
            config.PhaseSequence,
            () => _IsStopRequested || _IsDisposed,
            phase => phase switch
            {
                OptimizationPhase.Explore => optimizer.ExplorePhase(),
                OptimizationPhase.Compress => optimizer.CompressPhase(),
                OptimizationPhase.Polish => optimizer.PolishPhase(),
                _ => throw new ArgumentOutOfRangeException(nameof(phase))
            },
            (phase, ordinal, total) =>
            {
                float budget = phase switch
                {
                    OptimizationPhase.Explore => config.Exploration.TimeLimitSecs,
                    OptimizationPhase.Compress => config.Compression.TimeLimitSecs,
                    _ => config.Polish.TimeLimitSecs
                };
                AppendRunLine(log, $"  [{ordinal}/{total} {OptimizationPhaseSequence.DisplayName(phase)}] 시작... (이번 호출 예산 {budget}s)");
            },
            (phase, ordinal, total, result) =>
            {
                string name = OptimizationPhaseSequence.DisplayName(phase);
                if (result != 0)
                    AppendRunLine(log, $"  [{ordinal}/{total} {name}] 실패(code={optimizer.LastErrorCode}): {optimizer.LastError} — 나머지 단계 중단");
                else
                    AppendRunLine(log, _IsStopRequested || _IsDisposed
                        ? $"  [{ordinal}/{total} {name}] 중단됨 — 나머지 단계 중단"
                        : $"  [{ordinal}/{total} {name}] 완료: True");
            });
        await Task.CompletedTask;
    }

    /// <summary>네이티브 로그 레벨(1=Error … 5=Trace)을 짧은 라벨로 바꾼다.</summary>
    private static string NativeLogLabel(int level) => level switch
    {
        1 => "ERROR",
        2 => "WARN",
        3 => "INFO",
        4 => "DEBUG",
        _ => "TRACE",
    };

    private static void AppendRunLine(StringBuilder log, string line)
    {
        lock (log)
        {
            log.Insert(0, Environment.NewLine);
            log.Insert(0, line);
            if (log.Length > MaxRunLogChars)
            {
                int retainedLength = Math.Min(RetainedRunLogChars, log.Length);
                int lineBoundary = -1;
                for (int index = retainedLength - 1; index >= 0; index--)
                {
                    if (log[index] == '\n')
                    {
                        lineBoundary = index;
                        break;
                    }
                }

                int removeFrom = lineBoundary >= 0 ? lineBoundary + 1 : retainedLength;
                log.Remove(removeFrom, log.Length - removeFrom);
            }
        }
    }

    private async Task PublishRunLogAsync(StringBuilder log, int generation)
    {
        Interlocked.Exchange(ref _LogPublishDirty, 1);
        if (Interlocked.CompareExchange(ref _LogPublishScheduled, 1, 0) != 0)
        {
            return;
        }

        try
        {
            do
            {
                Interlocked.Exchange(ref _LogPublishDirty, 0);
                await Task.Delay(100).ConfigureAwait(false);
                string snapshot;
                lock (log)
                {
                    snapshot = log.ToString();
                }

                await RunOnMainThreadAsync(() =>
                {
                    if (!_IsDisposed && generation == Volatile.Read(ref _RunGeneration))
                    {
                        OutputLog = snapshot;
                    }
                });
            }
            while (Interlocked.Exchange(ref _LogPublishDirty, 0) != 0);
        }
        finally
        {
            Interlocked.Exchange(ref _LogPublishScheduled, 0);
            if (Volatile.Read(ref _LogPublishDirty) != 0)
            {
                _ = PublishRunLogAsync(log, generation);
            }
        }
    }

    #region Destructor

    public async Task DisposeAsync()
    {
        if (Interlocked.Exchange(ref _DisposeStarted, 1) != 0)
        {
            Task existing;
            lock (_OptimizerSync)
            {
                existing = _RunCompletion.Task;
            }
            await existing.ConfigureAwait(false);
            return;
        }

        _IsDisposed = true;
        _IsStopRequested = true;
        Interlocked.Increment(ref _RunGeneration);
        _SnapshotCancellation.Cancel();

        SwarmStepOptimizer? optimizer;
        Task runCompletion;
        lock (_OptimizerSync)
        {
            optimizer = _StepOptimizer;
            runCompletion = _RunCompletion.Task;
        }

        optimizer?.RequestStop();
        bool stoppingSeeds = RequestMultiSeedStop();
        await RunOnMainThreadAsync(StopPhaseClock).ConfigureAwait(false);
        if (stoppingSeeds) await WaitForMultiSeedStopAsync(runCompletion).ConfigureAwait(false);
        else await runCompletion.ConfigureAwait(false);

        lock (_OptimizerSync)
        {
            optimizer = _StepOptimizer;
            _StepOptimizer = null;
        }

        optimizer?.Dispose();
        _SnapshotCancellation.Dispose();
        _Subscriptions.Dispose();
        DisposeRenderItems(RenderedItems);
        RenderedItems = null;
    }

    public void Dispose()
    {
        _ = DisposeAsync();
    }

    /// <summary>
    /// 입력한 축소 스텝을 네이티브에 적용한다.
    /// </summary>
    /// <remarks>
    /// 탐색이 도는 도중에 눌러도 된다. 다음 축소 시점부터 반영되고, 진행 중인 separation은
    /// 중단 없이 끝까지 간다. 네이티브가 범위(0 &lt; step &lt; 1)를 검사해 실패하면
    /// 기존 값이 그대로 유지된다.
    /// </remarks>
    private void ApplyShrinkStep()
    {
        try
        {
            bool applied;
            lock (_OptimizerSync)
            {
                if (_MultiSeedHandles.Count > 0)
                {
                    if (!double.IsFinite(ShrinkStepInput) || ShrinkStepInput <= 0 || ShrinkStepInput >= 1)
                    {
                        this.ShrinkStepStatus = "거부됨 · 0 < step < 1 범위여야 합니다.";
                        return;
                    }
                    // Init 중인 시드는 Init 이 끝난 직후 같은 값을 적용한다.
                    _MultiSeedShrinkOverride = (float)ShrinkStepInput;
                    int count = 0;
                    foreach (var handle in _MultiSeedHandles)
                        if (handle.IsInitialized && handle.TrySetShrinkStep(ShrinkStepInput)) count++;
                    this.ShrinkStepStatus = $"{count}/{_MultiSeedHandles.Count} 시드에 적용 · Init 중인 시드도 완료 후 적용";
                    return;
                }
                if (_StepOptimizer is null || !_StepOptimizer.IsInitialized)
                {
                    this.ShrinkStepStatus = "실행 중인 솔버가 없습니다.";
                    return;
                }
                applied = _StepOptimizer.TrySetShrinkStep(this.ShrinkStepInput);
            }
            this.ShrinkStepStatus = applied
                ? $"적용됨 · shrink step {this.ShrinkStepInput:0.#####}"
                : $"거부됨 · 0 < step < 1 범위여야 합니다 (입력 {this.ShrinkStepInput:0.#####})";
        }
        catch (Exception ex)
        {
            this.ShrinkStepStatus = $"실패 · {ex.Message}";
        }
    }

    #endregion
}
