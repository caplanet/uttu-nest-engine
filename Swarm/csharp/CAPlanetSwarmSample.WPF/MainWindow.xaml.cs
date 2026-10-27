using ActiproSoftware.Windows.Controls.Docking;
using CAPlanetSwarmSample.WPF.DockSites;
using CAPlanetSwarmSample.WPF.Views;
using System.Diagnostics;
using System.Windows;
using System.Windows.Threading;

namespace CAPlanetSwarmSample.WPF
{
    [ReactiveUI.SourceGenerators.IViewFor<MainWindowViewModel>]
    public partial class MainWindow
    {
        private bool _ShutdownCompleted;
        private bool _ShutdownStarted;

        public static DockSite? DockSiteControl { get; private set; }

        public MainWindow(MainWindowViewModel VM)
        {
            InitializeComponent();
            this.DataContext = VM;
            DockSiteControl = dockSite;
            dockSite.WindowsClosed += async (_, e) =>
            {
                foreach (var window in e.Windows)
                {
                    if (window.DataContext is DockDocumentItemViewModel { Content: WorkspaceViewModel w })
                    {
                        await VM.CloseWorkspaceAsync(w);
                    }
                }
            };
            dockSite.WindowActivated += (_, e) =>
            {
                if (e.Window.DataContext is DockDocumentItemViewModel { Content: WorkspaceViewModel w })
                {
                    VM.CurrentWork = w;
                }
            };
            Closing += (_, e) =>
            {
                if (_ShutdownCompleted)
                {
                    return;
                }

                e.Cancel = true;
                if (_ShutdownStarted)
                {
                    return;
                }
                _ShutdownStarted = true;
                IsEnabled = false;
                _ = CompleteShutdownAsync(VM);
            };
            Closed += (_, _) =>
            {
                DockSiteControl = null;
            };
        }

        private async Task CompleteShutdownAsync(MainWindowViewModel viewModel)
        {
            try
            {
                await viewModel.DisposeAsync();
            }
            catch (Exception exception)
            {
                // 종료 중에는 MessageBox/Window.Show 계열도 다시 창 상태를 건드릴 수 있으므로
                // 진단 출력만 남기고 최종 종료는 계속 진행한다.
                Trace.TraceError($"MainWindow shutdown cleanup failed: {exception}");
            }
            finally
            {
                _ShutdownCompleted = true;

                // Closing 이벤트 프레임이 완전히 반환된 다음 두 번째 Close를 실행해야 한다.
                // DisposeAsync가 동기적으로 완료되더라도 항상 Dispatcher에 지연 등록한다.
                _ = Dispatcher.BeginInvoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(Close));
            }
        }
    }
}
