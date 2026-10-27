using CAPlanetSwarmSample.WPF.Services;
using ReactiveUI;
using ReactiveUI.SourceGenerators;
using System.Windows.Input;

namespace CAPlanetSwarmSample.WPF.Views;

public partial class WorkspaceViewModel
{
    private readonly List<ulong> _Seeds = [0];
    public IReadOnlyList<ulong> SeedValues => _Seeds.AsReadOnly();
    public int SeedCount => _Seeds.Count;
    public string SeedListText => string.Join(", ", _Seeds);
    [Reactive] public partial string SeedEntryText { get; set; } = "0";
    [Reactive] public partial string SeedInputStatus { get; set; } = string.Empty;
    public ICommand AddSeedCommand { get; private set; } = null!;
    public ICommand RemoveSeedCommand { get; private set; } = null!;

    private void InitializeSeeds()
    {
        var editable = this.WhenAnyValue(x => x.IsRunning, x => x.SeedEntryText,
            (running, text) => !running && MultiSeedConfiguration.TryParseSeed(text, out _));
        AddSeedCommand = ReactiveCommand.Create(() => EditSeed(add: true), editable);
        RemoveSeedCommand = ReactiveCommand.Create(() => EditSeed(add: false), editable);
        _Subscriptions.Add((IDisposable)AddSeedCommand);
        _Subscriptions.Add((IDisposable)RemoveSeedCommand);
    }

    private void EditSeed(bool add)
    {
        if (IsRunning || _IsDisposed || !MultiSeedConfiguration.TryParseSeed(SeedEntryText, out ulong seed)) return;
        if (add && _Seeds.Contains(seed)) { SeedInputStatus = $"시드 {seed}은 이미 있습니다."; return; }
        if (add && _Seeds.Count >= 6) { SeedInputStatus = "동시 시드는 최대 6개입니다."; return; }
        if (add) _Seeds.Add(seed);
        else if (!_Seeds.Remove(seed)) { SeedInputStatus = $"시드 {seed}이 목록에 없습니다."; return; }
        NotifySeedsChanged();
        SeedInputStatus = _Seeds.Count == 0 ? "실행할 시드를 추가하세요." : string.Empty;
    }

    // Input replacement keeps the current tab's seed list as well as Custom Config.
    public void ReplaceSeeds(IEnumerable<ulong> values)
    {
        if (IsRunning || _IsDisposed) return;
        var seeds = values.Distinct().ToArray();
        if (seeds.Length > 6) throw new ArgumentException("동시 시드는 최대 6개입니다.");
        _Seeds.Clear();
        _Seeds.AddRange(seeds);
        NotifySeedsChanged();
        SeedInputStatus = seeds.Length == 0 ? "실행할 시드를 추가하세요." : string.Empty;
    }

    private void NotifySeedsChanged()
    {
        this.RaisePropertyChanged(nameof(SeedValues));
        this.RaisePropertyChanged(nameof(SeedListText));
        this.RaisePropertyChanged(nameof(SeedCount));
    }
}
