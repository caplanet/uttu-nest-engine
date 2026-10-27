using SkiaSharp;
using SkiaSharp.Views.Desktop;
using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Threading;

namespace CAPlanetSwarmSample.WPF.Views;

/// <summary>
/// 네스팅 결과를 software Skia canvas에 표시합니다.
/// geometry는 ViewModel snapshot에서 캐시하고, 이 View는 viewport에 보이는 항목만 가볍게 합성합니다.
/// </summary>
public partial class WorkspaceView : UserControl
{
    // 시드 입력칸은 0~ulong.MaxValue 정수만 받는다. 붙여넣기·IME 입력도 같은 규칙으로 거른다.
    private static bool IsSeedEditValid(TextBox box, string input)
    {
        string candidate = box.Text.Remove(box.SelectionStart, box.SelectionLength).Insert(box.SelectionStart, input);
        return candidate.Length == 0 || Services.MultiSeedConfiguration.TryParseSeed(candidate, out _);
    }

    private void OnSeedTextInput(object sender, TextCompositionEventArgs e) =>
        e.Handled = sender is TextBox box && !IsSeedEditValid(box, e.Text);

    private void OnSeedPreviewKeyDown(object sender, KeyEventArgs e)
    {
        if (e.Key == Key.Space) e.Handled = true;
    }

    private void OnSeedPaste(object sender, DataObjectPastingEventArgs e)
    {
        if (sender is not TextBox box || !e.SourceDataObject.GetDataPresent(DataFormats.UnicodeText)
            || e.SourceDataObject.GetData(DataFormats.UnicodeText) is not string text || !IsSeedEditValid(box, text))
            e.CancelCommand();
    }

    // 다른 마커 탭에서 만든 프리셋도 목록을 열 때 보이게 한다.
    private void OnPresetDropDownOpened(object? sender, EventArgs e)
    {
        if (DataContext is WorkspaceViewModel workspace) workspace.RefreshPresets();
    }

    private const double MaximumFramesPerSecond = 60d;
    private const float GridStep = 1000f;
    private const float MinimumLabelExtentPixels = 24f;

    private static readonly SKColor BgColor = SKColor.Parse("#232325");
    private static readonly SKColor GridColor = SKColor.Parse("#2a2a4a");
    private static readonly SKColor StripEdgeColor = SKColor.Parse("#e94560");

    private readonly DispatcherTimer _RedrawTimer;
    private readonly DispatcherTimer _ZoomIdleTimer;
    private RenderResources? _RenderResources;
    private bool _RenderPending;
    private bool _IsZoomInteractionActive;
    private float _Zoom = 1f;
    private float _PanX;
    private float _PanY;
    private bool _IsPanning;
    private Point _LastMousePosition;

    public WorkspaceView()
    {
        InitializeComponent();

        // MouseMove와 연속 callback이 빠르게 들어와도 WPF render queue에는 60fps까지만 요청합니다.
        _RedrawTimer = new DispatcherTimer(DispatcherPriority.Render)
        {
            Interval = TimeSpan.FromSeconds(1d / MaximumFramesPerSecond)
        };
        _RedrawTimer.Tick += OnRedrawTimerTick;

        // wheel 입력이 잠시 멎은 뒤 한 번 더 그려 고비용 shadow를 자연스럽게 복원합니다.
        _ZoomIdleTimer = new DispatcherTimer(DispatcherPriority.Background)
        {
            Interval = TimeSpan.FromMilliseconds(180)
        };
        _ZoomIdleTimer.Tick += OnZoomIdleTimerTick;

        Loaded += OnLoaded;
        Unloaded += OnUnloaded;
        DataContextChanged += OnDataContextChanged;
        Canvas.MouseWheel += OnMouseWheel;
        Canvas.MouseLeftButtonDown += OnMouseDown;
        Canvas.MouseLeftButtonUp += OnMouseUp;
        Canvas.MouseMove += OnMouseMove;
    }

    private void OnLoaded(object sender, RoutedEventArgs e) => RequestRender();

    private void OnUnloaded(object sender, RoutedEventArgs e)
    {
        _RedrawTimer.Stop();
        _ZoomIdleTimer.Stop();
        _RenderPending = false;
        _IsZoomInteractionActive = false;
        _RenderResources?.Dispose();
        _RenderResources = null;
    }

    private void OnMouseWheel(object sender, MouseWheelEventArgs e)
    {
        _Zoom *= e.Delta > 0 ? 1.1f : 0.9f;
        _Zoom = Math.Clamp(_Zoom, 0.1f, 20f);
        BeginZoomInteraction();
        RequestRender();
    }

    private void OnMouseDown(object sender, MouseButtonEventArgs e)
    {
        if (e.ClickCount == 2)
        {
            _Zoom = 1f;
            _PanX = _PanY = 0f;
            BeginZoomInteraction();
            RequestRender();
            return;
        }

        if (e.LeftButton == MouseButtonState.Pressed)
        {
            _IsPanning = true;
            _LastMousePosition = e.GetPosition(Canvas);
            Canvas.CaptureMouse();
            RequestRender();
        }
    }

    private void OnMouseUp(object sender, MouseButtonEventArgs e)
    {
        _IsPanning = false;
        Canvas.ReleaseMouseCapture();

        // 이동 중 생략했던 shadow를 정지한 화면에서 다시 표시합니다.
        RequestRender();
    }

    private void OnMouseMove(object sender, MouseEventArgs e)
    {
        if (!_IsPanning)
        {
            return;
        }

        Point position = e.GetPosition(Canvas);
        _PanX += (float)(position.X - _LastMousePosition.X);
        _PanY += (float)(position.Y - _LastMousePosition.Y);
        _LastMousePosition = position;
        RequestRender();
    }

    private void BeginZoomInteraction()
    {
        _IsZoomInteractionActive = true;
        _ZoomIdleTimer.Stop();
        _ZoomIdleTimer.Start();
    }

    private void OnZoomIdleTimerTick(object? sender, EventArgs e)
    {
        _ZoomIdleTimer.Stop();
        _IsZoomInteractionActive = false;
        RequestRender();
    }

    private void RequestRender()
    {
        _RenderPending = true;
        if (IsLoaded && !_RedrawTimer.IsEnabled)
        {
            _RedrawTimer.Start();
        }
    }

    private void OnRedrawTimerTick(object? sender, EventArgs e)
    {
        _RedrawTimer.Stop();
        if (!_RenderPending || !IsLoaded)
        {
            return;
        }

        _RenderPending = false;
        Canvas.InvalidateVisual();
    }

    private void OnDataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
    {
        if (e.OldValue is WorkspaceViewModel oldViewModel)
        {
            oldViewModel.PropertyChanged -= OnViewModelPropertyChanged;
        }

        if (e.NewValue is WorkspaceViewModel newViewModel)
        {
            newViewModel.PropertyChanged += OnViewModelPropertyChanged;
        }

        RequestRender();
    }

    private void OnViewModelPropertyChanged(object? sender, PropertyChangedEventArgs e)
    {
        if (e.PropertyName is nameof(WorkspaceViewModel.RenderedItems)
            or nameof(WorkspaceViewModel.IsRunning))
        {
            RequestRender();
        }
    }

    private void OnPaintSurface(object sender, SKPaintSurfaceEventArgs e)
    {
        SKCanvas canvas = e.Surface.Canvas;
        canvas.Clear(BgColor);

        if (DataContext is not WorkspaceViewModel viewModel
            || viewModel.RenderedItems is not { Count: > 0 } items)
        {
            return;
        }

        const float margin = 40f;
        float availableWidth = e.Info.Width - (2 * margin);
        float availableHeight = e.Info.Height - (2 * margin);
        float stripWidth = viewModel.RenderedStripWidth;
        float maxY = viewModel.RenderedMaxY > 0 ? viewModel.RenderedMaxY : 1000f;
        if (availableWidth <= 0 || availableHeight <= 0 || stripWidth <= 0)
        {
            return;
        }

        float scale = MathF.Min(availableWidth / stripWidth, availableHeight / maxY);
        if (!float.IsFinite(scale) || scale <= 0)
        {
            return;
        }

        RenderResources resources = _RenderResources ??= new RenderResources();
        float inverseScale = 1f / scale;
        float effectiveScale = scale * _Zoom;
        bool drawShadow = !viewModel.IsRunning && !_IsPanning && !_IsZoomInteractionActive;

        resources.LabelFont.Size = Math.Clamp(10f * inverseScale, 8f, 20f);
        resources.GridPaint.StrokeWidth = 0.5f * inverseScale;
        resources.EdgePaint.StrokeWidth = 2f * inverseScale;
        // Skia hairline stays one device pixel wide, including when zoomed in.
        resources.StrokePaint.StrokeWidth = 0f;
        resources.UpdateDashEffect(8f * inverseScale, 4f * inverseScale);
        if (drawShadow)
        {
            resources.UpdateShadowMask(2f * inverseScale);
        }

        canvas.DrawText(
            $"Strip : {stripWidth:F1}    Density : {viewModel.RenderedDensity * 100:F1}%    Collisions : {viewModel.RenderedCollisions}",
            10,
            30,
            SKTextAlign.Left,
            resources.HeaderFont,
            resources.GrayPaint);

        canvas.Save();
        canvas.Translate(margin, e.Info.Height - margin);
        canvas.Scale(scale);
        canvas.Scale(_Zoom);
        canvas.Translate(_PanX / effectiveScale, _PanY / effectiveScale);
        canvas.Scale(1, -1);

        // 화면 영역을 world 좌표로 역변환해 보이지 않는 item은 DrawPath 호출 자체를 생략합니다.
        float viewportLeft = (-margin - _PanX) / effectiveScale;
        float viewportRight = (e.Info.Width - margin - _PanX) / effectiveScale;
        float viewportBottom = (_PanY - margin) / effectiveScale;
        float viewportTop = (e.Info.Height - margin + _PanY) / effectiveScale;
        float cullingPadding = drawShadow ? 8f * inverseScale : 2f * inverseScale;

        DrawGrid(
            canvas,
            resources,
            stripWidth,
            maxY,
            viewportLeft,
            viewportRight,
            viewportBottom,
            viewportTop);

        foreach (PlacedItemRender item in items)
        {
            if (!IntersectsViewport(
                item.Bounds,
                viewportLeft,
                viewportRight,
                viewportBottom,
                viewportTop,
                cullingPadding))
            {
                continue;
            }

            if (drawShadow)
            {
                canvas.Save();
                canvas.Translate(2f * inverseScale, 2f * inverseScale);
                canvas.DrawPath(item.Path, resources.ShadowPaint);
                canvas.Restore();
            }

            resources.FillPaint.Color = item.Color;
            canvas.DrawPath(item.Path, resources.FillPaint);

            resources.StrokePaint.Color = item.BorderColor;
            canvas.DrawPath(item.Path, resources.StrokePaint);

            // 작은 부품에 ID를 억지로 그리면 텍스트 비용과 화면 혼잡만 커집니다.
            float smallestScreenExtent = MathF.Min(item.Bounds.Width, item.Bounds.Height) * effectiveScale;
            if (smallestScreenExtent < MinimumLabelExtentPixels)
            {
                continue;
            }

            canvas.Save();
            canvas.Scale(1, -1);
            canvas.DrawText(
                item.Label,
                item.LabelCenter.X - (6f * inverseScale),
                -item.LabelCenter.Y + (4f * inverseScale),
                SKTextAlign.Left,
                resources.LabelFont,
                resources.WhitePaint);
            canvas.Restore();
        }

        canvas.Restore();
    }

    private static void DrawGrid(
        SKCanvas canvas,
        RenderResources resources,
        float stripWidth,
        float maxY,
        float viewportLeft,
        float viewportRight,
        float viewportBottom,
        float viewportTop)
    {
        float firstX = MathF.Max(0, MathF.Floor(viewportLeft / GridStep) * GridStep);
        float lastX = MathF.Min(stripWidth, viewportRight);
        for (float x = firstX; x <= lastX; x += GridStep)
        {
            canvas.DrawLine(x, 0, x, maxY, resources.GridPaint);
        }

        float firstY = MathF.Max(0, MathF.Floor(viewportBottom / GridStep) * GridStep);
        float lastY = MathF.Min(maxY, viewportTop);
        for (float y = firstY; y <= lastY; y += GridStep)
        {
            canvas.DrawLine(0, y, stripWidth, y, resources.GridPaint);
        }

        canvas.DrawLine(stripWidth, 0, stripWidth, maxY, resources.EdgePaint);
    }

    private static bool IntersectsViewport(
        SKRect bounds,
        float left,
        float right,
        float bottom,
        float top,
        float padding) =>
        bounds.Right >= left - padding
        && bounds.Left <= right + padding
        && bounds.Bottom >= bottom - padding
        && bounds.Top <= top + padding;

    /// <summary>
    /// paint마다 만들 필요가 없는 Skia native resource를 View 수명 동안 재사용합니다.
    /// View가 unload되면 모두 해제하고, 다시 load될 때 지연 생성합니다.
    /// </summary>
    private sealed class RenderResources : IDisposable
    {
        private SKPathEffect? _DashEffect;
        private SKMaskFilter? _ShadowMask;
        private float _DashLength = float.NaN;
        private float _DashGap = float.NaN;
        private float _ShadowSigma = float.NaN;

        public RenderResources()
        {
            LabelTypeface = SKTypeface.FromFamilyName("Consolas");
            LabelFont = new SKFont(LabelTypeface, 10);
            HeaderFont = new SKFont(SKTypeface.Default, 24);
            WhitePaint = new SKPaint { Color = SKColors.White, IsAntialias = true };
            GrayPaint = new SKPaint { Color = SKColors.Gray, IsAntialias = true };
            GridPaint = new SKPaint
            {
                Color = GridColor,
                Style = SKPaintStyle.Stroke,
                IsAntialias = true
            };
            EdgePaint = new SKPaint
            {
                Color = StripEdgeColor,
                Style = SKPaintStyle.Stroke,
                IsAntialias = true
            };
            ShadowPaint = new SKPaint
            {
                Color = SKColors.Black.WithAlpha(80),
                Style = SKPaintStyle.Fill,
                IsAntialias = true
            };
            FillPaint = new SKPaint { Style = SKPaintStyle.Fill, IsAntialias = true };
            StrokePaint = new SKPaint { Style = SKPaintStyle.Stroke, IsAntialias = true };
        }

        public SKTypeface LabelTypeface { get; }
        public SKFont LabelFont { get; }
        public SKFont HeaderFont { get; }
        public SKPaint WhitePaint { get; }
        public SKPaint GrayPaint { get; }
        public SKPaint GridPaint { get; }
        public SKPaint EdgePaint { get; }
        public SKPaint ShadowPaint { get; }
        public SKPaint FillPaint { get; }
        public SKPaint StrokePaint { get; }

        public void UpdateDashEffect(float dashLength, float dashGap)
        {
            if (NearlyEqual(_DashLength, dashLength) && NearlyEqual(_DashGap, dashGap))
            {
                return;
            }

            EdgePaint.PathEffect = null;
            _DashEffect?.Dispose();
            _DashEffect = SKPathEffect.CreateDash([dashLength, dashGap], 0);
            EdgePaint.PathEffect = _DashEffect;
            _DashLength = dashLength;
            _DashGap = dashGap;
        }

        public void UpdateShadowMask(float sigma)
        {
            if (NearlyEqual(_ShadowSigma, sigma))
            {
                return;
            }

            ShadowPaint.MaskFilter = null;
            _ShadowMask?.Dispose();
            _ShadowMask = SKMaskFilter.CreateBlur(SKBlurStyle.Normal, sigma);
            ShadowPaint.MaskFilter = _ShadowMask;
            _ShadowSigma = sigma;
        }

        private static bool NearlyEqual(float left, float right) =>
            MathF.Abs(left - right) <= MathF.Max(0.0001f, MathF.Abs(right) * 0.0001f);

        public void Dispose()
        {
            EdgePaint.PathEffect = null;
            ShadowPaint.MaskFilter = null;
            _DashEffect?.Dispose();
            _ShadowMask?.Dispose();
            StrokePaint.Dispose();
            FillPaint.Dispose();
            ShadowPaint.Dispose();
            EdgePaint.Dispose();
            GridPaint.Dispose();
            GrayPaint.Dispose();
            WhitePaint.Dispose();
            HeaderFont.Dispose();
            LabelFont.Dispose();
            LabelTypeface.Dispose();
        }
    }
}
