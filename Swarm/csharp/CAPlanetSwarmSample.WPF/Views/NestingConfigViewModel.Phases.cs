using System;
using System.Collections.ObjectModel;
using System.Linq;
using System.Windows.Input;
using ReactiveUI;
using ReactiveUI.SourceGenerators;
using Uttu.NestingV2;

namespace CAPlanetSwarmSample.WPF.Views;

// Each occurrence has its own identity: deleting the second Explore must not
// remove the first Explore in a repeated sequence.
public sealed class PhaseSequenceEntry(OptimizationPhase phase)
{
    public OptimizationPhase Phase { get; } = phase;
    public string Name => Phase.ToString();
    public string DisplayName => OptimizationPhaseSequence.DisplayName(Phase);
    public override string ToString() => Name;
}

public partial class NestingConfigViewModel
{
    public static ReadOnlyCollection<OptimizationPhase> PhaseOptions { get; } =
        Array.AsReadOnly(new[] { OptimizationPhase.Explore, OptimizationPhase.Compress, OptimizationPhase.Polish });

    public ObservableCollection<PhaseSequenceEntry> PhaseEntries { get; } = new()
    {
        new(OptimizationPhase.Explore),
        new(OptimizationPhase.Compress),
        new(OptimizationPhase.Polish),
    };

    /// <summary>아래 ComboBox에서 고른 단계. 예: Explore를 두 번 추가하면 두 번 실행합니다.</summary>
    [Reactive] public partial OptimizationPhase SelectedPhase { get; set; } = OptimizationPhase.Explore;

    public string PhaseSequenceText => string.Join(" → ", PhaseEntries);

    public string PhaseSequenceValidation
    {
        get
        {
            if (PhaseEntries.Count == 0)
                return "아래 목록에서 단계를 선택하고 + 버튼으로 추가하세요.";
            try { OptimizationPhaseSequence.Parse(PhaseSequenceText); return "위에서 아래로 실행 · 같은 단계 반복 추가 가능"; }
            catch (ArgumentException error) { return error.Message; }
        }
    }

    /// <summary>현재 목록을 실행 순서로 돌려준다. 비어 있거나 잘못됐으면 예외를 던진다.</summary>
    public OptimizationPhase[] PhaseSequence =>
        OptimizationPhaseSequence.Validate(PhaseEntries.Select(entry => entry.Phase));

    private ICommand? _addPhaseCommand;
    private ICommand? _removePhaseCommand;
    public ICommand AddPhaseCommand => _addPhaseCommand ??= ReactiveCommand.Create(() =>
    {
        if (PhaseOptions.Contains(SelectedPhase))
            PhaseEntries.Add(new(SelectedPhase));
    });
    public ICommand RemovePhaseCommand => _removePhaseCommand ??= ReactiveCommand.Create<PhaseSequenceEntry>(entry =>
    {
        if (entry is not null) PhaseEntries.Remove(entry);
    });

    private void InitializePhaseEditor()
    {
        PhaseEntries.CollectionChanged += (_, _) =>
        {
            this.RaisePropertyChanged(nameof(PhaseSequenceText));
            this.RaisePropertyChanged(nameof(PhaseSequenceValidation));
        };
    }
}
