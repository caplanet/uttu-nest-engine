using CAPlanetSwarmSample.WPF.Services;
using Microsoft.Win32;
using ReactiveUI;
using ReactiveUI.SourceGenerators;
using System.Collections.ObjectModel;
using System.IO;
using System.Reactive.Linq;
using System.Windows.Input;
using Uttu.NestingV2;

namespace CAPlanetSwarmSample.WPF.Views;

public partial class WorkspaceViewModel
{
    private ConfigPresetStore _PresetStore = new(Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "CAPlanet", "SwarmSample", "Presets"));
    public ObservableCollection<ConfigPresetEntry> Presets { get; } = new();
    [Reactive] public partial ConfigPresetEntry? SelectedPreset { get; set; }
    [Reactive] public partial string PresetName { get; set; } = string.Empty;
    [Reactive] public partial string PresetStatus { get; set; } = string.Empty;
    public ICommand AddPresetCommand { get; private set; } = null!;
    public ICommand DeletePresetCommand { get; private set; } = null!;
    public ICommand ApplyPresetCommand { get; private set; } = null!;
    public ICommand SavePresetJsonCommand { get; private set; } = null!;
    public ICommand LoadPresetJsonCommand { get; private set; } = null!;

    private void InitializePresets()
    {
        var idle = this.WhenAnyValue(x => x.IsRunning, running => !running);
        var selectedIdle = this.WhenAnyValue(x => x.IsRunning, x => x.SelectedPreset,
            (running, selected) => !running && selected != null);
        AddPresetCommand = ReactiveCommand.Create(() => PresetAction(AddCurrentPreset), idle);
        DeletePresetCommand = ReactiveCommand.Create(() => PresetAction(() =>
        {
            if (SelectedPreset is not { } entry) return;
            _PresetStore.Delete(entry.Id);
            RefreshPresets();
            PresetStatus = $"'{entry.Name}' 삭제 완료 · 현재 설정은 유지됩니다.";
        }), selectedIdle);
        ApplyPresetCommand = ReactiveCommand.Create(() => PresetAction(() =>
        {
            if (SelectedPreset is not { } entry) return;
            var candidate = PreparePreset(entry.Document);
            Config = candidate;
            PresetStatus = $"'{entry.Name}' 적용 완료";
        }), selectedIdle);
        SavePresetJsonCommand = ReactiveCommand.Create(() => PresetAction(() =>
        {
            if (SelectedPreset is not { } entry) return;
            var doc = entry.Document;
            string safeName = string.Concat(doc.Name.Select(c => Path.GetInvalidFileNameChars().Contains(c) ? '_' : c));
            var dialog = new SaveFileDialog
            {
                Title = "선택한 프리셋 하나를 JSON으로 저장", Filter = "Swarm 프리셋 JSON (*.json)|*.json",
                DefaultExt = ".json", AddExtension = true, FileName = safeName + ".json", OverwritePrompt = true
            };
            if (dialog.ShowDialog() != true) return;
            ExportSelectedPresetFile(dialog.FileName);
        }), selectedIdle);
        LoadPresetJsonCommand = ReactiveCommand.Create(() => PresetAction(() =>
        {
            var dialog = new OpenFileDialog
            {
                Title = "Custom Config 프리셋 JSON 불러오기", Filter = "Swarm 프리셋 JSON (*.json)|*.json",
                CheckFileExists = true, Multiselect = false
            };
            if (dialog.ShowDialog() == true) ImportPresetFile(dialog.FileName);
        }), idle);
        foreach (var command in new[] { AddPresetCommand, DeletePresetCommand, ApplyPresetCommand, SavePresetJsonCommand, LoadPresetJsonCommand })
            _Subscriptions.Add((IDisposable)command);
        _Subscriptions.Add(this.WhenAnyValue(x => x.SelectedPreset).Subscribe(entry =>
        {
            if (entry is not null) PresetName = entry.Name;
        }));
        RefreshPresets();
    }

    private void PresetAction(Action action)
    {
        if (IsRunning || _IsDisposed) return;
        try { action(); }
        catch (Exception ex) { PresetStatus = "프리셋 처리 실패: " + ex.Message; }
    }

    // Also called on drop-down opening so presets created in another marker tab become visible.
    public void RefreshPresets()
    {
        if (IsRunning || _IsDisposed) return;
        try
        {
            Guid? selectedId = SelectedPreset?.Id;
            var result = _PresetStore.Load();
            Presets.Clear();
            foreach (var entry in result.Entries) Presets.Add(entry);
            SelectedPreset = Presets.FirstOrDefault(e => e.Id == selectedId);
            if (result.Errors.Count > 0)
                PresetStatus = $"읽지 못한 프리셋 {result.Errors.Count}개: {result.Errors[0]}";
        }
        catch (Exception ex) { PresetStatus = "프리셋 목록 읽기 실패: " + ex.Message; }
    }

    private ConfigPresetDocument CaptureCurrentPreset()
    {
        _ = Config.ToSwarmConfig();
        string name = string.IsNullOrWhiteSpace(PresetName)
            ? $"Custom {DateTime.Now:yyyy-MM-dd HHmmss}" : PresetName.Trim();
        return ConfigPresetCodec.Capture(name, Config, Config.PhaseEntries.Select(e => e.Name));
    }

    private static NestingConfigViewModel PreparePreset(ConfigPresetDocument document)
    {
        // Missing fields in an older preset use current defaults, never values left by
        // the previously selected preset. Publish the new model in a single assignment.
        var candidate = new NestingConfigViewModel();
        ConfigPresetCodec.Apply(document, candidate);
        candidate.PhaseEntries.Clear();
        foreach (string phase in document.PhaseSequence)
            candidate.PhaseEntries.Add(new(Enum.Parse<OptimizationPhase>(phase)));
        _ = candidate.ToSwarmConfig();
        return candidate;
    }

    private void AddCurrentPreset()
    {
        var entry = _PresetStore.Add(CaptureCurrentPreset());
        RefreshPresets();
        SelectedPreset = Presets.First(e => e.Id == entry.Id);
        PresetStatus = $"'{entry.Name}' 추가 완료 · 현재 입력값을 저장했습니다.";
    }

    private void ImportPresetFile(string path)
    {
        var document = ConfigPresetCodec.ReadImportFile(path);
        _ = PreparePreset(document);
        var entry = _PresetStore.Add(document, rejectDuplicateName: true);
        RefreshPresets();
        SelectedPreset = Presets.First(e => e.Id == entry.Id);
        PresetStatus = $"'{entry.Name}' 불러옴 · 적용을 눌러 설정을 반영하세요.";
    }

    private void ExportSelectedPresetFile(string path)
    {
        if (IsRunning || _IsDisposed || SelectedPreset is not { } selected) return;
        var entry = _PresetStore.Export(selected.Id, path);
        RefreshPresets();
        SelectedPreset = Presets.First(e => e.Id == entry.Id);
        PresetStatus = $"'{entry.Name}' 프리셋 하나 저장 완료: {Path.GetFileName(path)}";
    }
}
