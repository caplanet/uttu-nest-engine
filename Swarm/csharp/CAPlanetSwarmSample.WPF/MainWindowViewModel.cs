using ActiproSoftware.Windows.Controls.Docking;
using CAPlanetSwarmSample.WPF.DockSites;
using CAPlanetSwarmSample.WPF.Services;
using CAPlanetSwarmSample.WPF.Views;
using ReactiveUI;
using ReactiveUI.SourceGenerators;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Reactive.Concurrency;
using System.Reactive.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;

using static CAPlanetSwarmSample.WPF.MainWindow;

namespace CAPlanetSwarmSample.WPF
{
    public partial class MainWindowViewModel : ReactiveObject, IDisposable
    {
        #region Property

        public DockSiteViewModel DockSite { get; }

        private ChildObservableCollection<DockDocumentItemViewModel> DockDocuments = [];

        private Dictionary<DockDocumentItemViewModel, WorkspaceViewModel> Workspaces = [];

        private bool _IsDisposed;

        #endregion

        #region Command

        [Reactive] public partial WorkspaceViewModel? CurrentWork { get; set; }

        public ICommand OpenInputJsonCommand { get; }

        public ICommand ExportOutputJsonCommand { get; }

        public ICommand ExportOutputSvgCommand { get; }

        public ICommand ExportOutputPltCommand { get; }

        public ICommand ShowAboutCommand { get; }

        #endregion

        #region Constructor

        public MainWindowViewModel()
        {
            this.OpenInputJsonCommand = ReactiveCommand.Create(this.OpenInputJson);

            var _canExportOutputJson = this.WhenAnyValue(
                x => x.CurrentWork,
                x => x.CurrentWork.IsRunning,
                x => x.CurrentWork.LastSolutionJson,
                (work, isRunning, json) => work != null && !isRunning && !string.IsNullOrWhiteSpace(json));
            this.ExportOutputJsonCommand = ReactiveCommand.Create(this.ExportOutputJson, _canExportOutputJson);
            this.ExportOutputPltCommand = ReactiveCommand.CreateFromTask(this.ExportOutputPltAsync, _canExportOutputJson);

            var _canExportOutputSvg = this.WhenAnyValue(
                x => x.CurrentWork,
                x => x.CurrentWork.IsRunning,
                x => x.CurrentWork.RenderedItems,
                (work, isRunning, items) => work != null && !isRunning && items != null && items.Count > 0);
            this.ExportOutputSvgCommand = ReactiveCommand.Create(this.ExportOutputSVG, _canExportOutputSvg);

            this.ShowAboutCommand = ReactiveCommand.Create(() =>
            {
                var _aboutWindow = new AboutWindow();
                _aboutWindow.Owner = Application.Current.MainWindow;
                _aboutWindow.WindowStartupLocation = WindowStartupLocation.CenterOwner;
                _aboutWindow.ShowDialog();
            });
            DockSite = new();
        }

        #endregion

        #region Method

        public void ExportOutputJson()
        {
            if (this.CurrentWork != null && this.CurrentWork.IsRunning == false && !string.IsNullOrEmpty(this.CurrentWork.LastSolutionJson))
            {
                string _json;
                try
                {
                    _json = SolutionJsonExportService.Create(this.CurrentWork.InputData.JsonSnapshot, this.CurrentWork.LastSolutionJson);
                }
                catch (Exception exception)
                {
                    MessageBox.Show(exception.Message, "Swarm 결과 JSON 내보내기 실패", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }
                string _fileName = $"{this.CurrentWork.InputData.ProjectName}_output_{DateTime.Now:yyyyMMdd_HHmmss}";
                string _path = Path.Combine(this.CurrentWork.OutputDir, $"{_fileName}.json");
                Directory.CreateDirectory(this.CurrentWork.OutputDir);
                File.WriteAllText(_path, _json);
                this.CurrentWork.AppendLog($"JSON 내보내기 : {_path}");
                string? importWarning = SolutionJsonExportService.GetUttuImportWarning(_json);
                if (importWarning != null)
                {
                    this.CurrentWork.AppendLog(importWarning);
                    MessageBox.Show(importWarning, "Uttu.Nest 연동 정보 누락", MessageBoxButton.OK, MessageBoxImage.Warning);
                }

                if (File.Exists(_path)
                    && MessageBox.Show($"{_fileName}.json 파일이 저장되었습니다.{Environment.NewLine}해당 폴더를 열까요?", "Swarm 결과 JSON 내보내기", MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
                {
                    Process.Start("explorer.exe", $"/select,\"{_path}\"");
                }
            }
        }

        public async Task ExportOutputPltAsync()
        {
            var work = this.CurrentWork;
            if (work == null || work.IsRunning || string.IsNullOrWhiteSpace(work.LastSolutionJson)) return;
            // Capture one workspace and its immutable snapshots before yielding to file I/O.
            string input = work.InputData.JsonSnapshot;
            string solution = work.LastSolutionJson;
            string name = string.Concat(work.InputData.ProjectName.Select(c => Path.GetInvalidFileNameChars().Contains(c) ? '_' : c));
            string path = Path.Combine(work.OutputDir, $"{name}_output_{DateTime.Now:yyyyMMdd_HHmmss_fff}.plt");
            try
            {
                await Task.Run(() => PltExportService.Save(input, solution, path));
                work.AppendLog($"PLT 내보내기 완료: {path}");
                MessageBox.Show($"PLT 파일을 저장했습니다.{Environment.NewLine}{path}", "Swarm PLT 내보내기", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception exception)
            {
                work.AppendLog($"PLT 내보내기 실패: {exception.Message}");
                MessageBox.Show(exception.Message, "PLT 내보내기 실패", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        public void ExportOutputSVG()
        {
            if (this.CurrentWork != null && this.CurrentWork.IsRunning == false && this.CurrentWork.RenderedItems?.Count > 0)
            {
                string _svgStr = this.CurrentWork.GetMakeSvg();
                string _fileName = $"{this.CurrentWork.InputData.ProjectName}_output_{DateTime.Now:yyyyMMdd_HHmmss}";
                string _path = Path.Combine(this.CurrentWork.OutputDir, $"{_fileName}.svg");
                File.WriteAllText(_path, _svgStr);
                this.CurrentWork.AppendLog($"SVG 내보내기 왆료 : {_path}");

                if (File.Exists(_path)
                    && MessageBox.Show($"{_fileName}.svg 파일이 저장되었습니다.{Environment.NewLine}해당 폴더를 열까요?", "Swarm 결과 SVG 내보내기", MessageBoxButton.YesNo, MessageBoxImage.Question) == MessageBoxResult.Yes)
                {
                    Process.Start("explorer.exe", $"/select,\"{_path}\"");
                }
            }
        }

        public void OpenInputJson()
        {
            string _projectPath = CommonDialogService.OpenSwarmInputFileDialog() ?? string.Empty;

            if (_projectPath.Length == 0)
            {
                //실패 메세지
                return;
            }

            var _result = SwarmIOService.InputJsonLoad(Path.GetFileNameWithoutExtension(_projectPath), _projectPath);

            if (_result.IsSuccess == false)
            {
                MessageBox.Show(
                    _result.ErrorMessage ?? "Swarm 입력 JSON을 불러오지 못했습니다.",
                    "입력 JSON 오류",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
                return;
            }

            var _work = new WorkspaceViewModel(this, _result.InputData!);
            _work.DockingItem.IsOpen = true;
            DockSite.DocumentItems.Add(_work.DockingItem);
            if (DockSiteControl != null)
            {
                var windows = DockSiteControl.DocumentWindows;
                if (windows.Count > 0) windows[windows.Count - 1].Activate();
            }
            DockDocuments.Add(_work.DockingItem);
            Workspaces[_work.DockingItem] = _work;
            CurrentWork = _work;
        }

        /// <summary>
        /// 실제로 닫힌 도킹 문서의 Workspace와 네이티브 리소스를 제거합니다.
        /// 탭 비활성화만으로는 호출하지 않습니다.
        /// </summary>
        public async Task CloseWorkspaceAsync(WorkspaceViewModel workspace)
        {
            if (!Workspaces.Remove(workspace.DockingItem))
            {
                return;
            }

            DockDocuments.Remove(workspace.DockingItem);
            DockSite.DocumentItems.Remove(workspace.DockingItem);
            await workspace.DisposeAsync();

            if (ReferenceEquals(CurrentWork, workspace))
            {
                CurrentWork = Workspaces.Values.LastOrDefault();
            }
        }

        #endregion

        #region Destructor

        public void Dispose()
        {
            _ = DisposeAsync();
        }

        public async Task DisposeAsync()
        {
            if (_IsDisposed)
            {
                return;
            }

            _IsDisposed = true;
            foreach (WorkspaceViewModel workspace in Workspaces.Values.ToArray())
            {
                await workspace.DisposeAsync();
            }

            Workspaces.Clear();
            DockDocuments.Clear();
            DockSite.DocumentItems.Clear();
            CurrentWork = null;

            (OpenInputJsonCommand as IDisposable)?.Dispose();
            (ExportOutputJsonCommand as IDisposable)?.Dispose();
            (ExportOutputSvgCommand as IDisposable)?.Dispose();
            (ExportOutputPltCommand as IDisposable)?.Dispose();
            (ShowAboutCommand as IDisposable)?.Dispose();
        }

        #endregion
    }
}
