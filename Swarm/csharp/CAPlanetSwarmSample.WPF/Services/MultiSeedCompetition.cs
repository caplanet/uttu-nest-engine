using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using System.Text.Json;
using System.Text.Json.Serialization;
using Uttu.NestingV2;

namespace CAPlanetSwarmSample.WPF.Services;

public static class MultiSeedConfiguration
{
    public static bool TryParseSeed(string? text, out ulong seed)
    {
        seed = 0;
        return !string.IsNullOrEmpty(text) && text.All(c => c is >= '0' and <= '9')
            && ulong.TryParse(text, System.Globalization.NumberStyles.None,
                System.Globalization.CultureInfo.InvariantCulture, out seed);
    }

    /// <summary>Explicit ordered seeds; zero requests one non-conflicting random seed.</summary>
    public static IReadOnlyList<SwarmConfig> Create(SwarmConfig common, IReadOnlyList<ulong> seeds)
    {
        ArgumentNullException.ThrowIfNull(common);
        if (seeds.Count is < 1 or > 6 || seeds.Distinct().Count() != seeds.Count)
            throw new ArgumentException("시드는 중복 없이 1~6개 지정하세요.", nameof(seeds));
        byte[] snapshot = JsonSerializer.SerializeToUtf8Bytes(common, CloneOptions);
        var used = seeds.Where(s => s != 0).ToHashSet();
        var result = new List<SwarmConfig>(seeds.Count);
        Span<byte> bytes = stackalloc byte[sizeof(ulong)];
        foreach (ulong requested in seeds)
        {
            ulong seed = requested;
            if (seed == 0)
            {
                do { RandomNumberGenerator.Fill(bytes); seed = BitConverter.ToUInt64(bytes); }
                while (seed == 0 || used.Contains(seed));
            }
            used.Add(seed);
            var copy = JsonSerializer.Deserialize<SwarmConfig>(snapshot, CloneOptions)!;
            copy.RngSeed = seed;
            result.Add(copy);
        }
        return result.AsReadOnly();
    }

    private static readonly JsonSerializerOptions CloneOptions = new()
    {
        NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals
    };

    /// <summary>Snapshot every managed option, changing only the explicit seed per run.</summary>
    public static IReadOnlyList<SwarmConfig> Create(SwarmConfig common, int count)
    {
        ArgumentNullException.ThrowIfNull(common);
        if (count is < 1 or > 6) throw new ArgumentOutOfRangeException(nameof(count));

        // Serialize once so all independent clones use the same common snapshot.
        byte[] snapshot = JsonSerializer.SerializeToUtf8Bytes(common, CloneOptions);
        ulong seed = common.RngSeed;
        if (seed == 0)
        {
            Span<byte> bytes = stackalloc byte[sizeof(ulong)];
            do
            {
                RandomNumberGenerator.Fill(bytes);
                seed = BitConverter.ToUInt64(bytes);
            } while (seed == 0);
        }

        var configurations = new SwarmConfig[count];
        for (int i = 0; i < count; i++)
        {
            var copy = JsonSerializer.Deserialize<SwarmConfig>(snapshot, CloneOptions)!;
            copy.RngSeed = seed;
            configurations[i] = copy;
            seed = seed == ulong.MaxValue ? 1 : seed + 1;
        }
        return Array.AsReadOnly(configurations);
    }
}

public sealed record SeedWinner(long Revision, int SeedIndex, ulong Seed, SolutionEventArgs Report,
    string Phase, bool IsFinal, string? StructureReportJson);

public sealed class MultiSeedCompetition
{
    private readonly object _sync = new();
    private readonly ulong[] _seeds;
    private readonly Dictionary<int, int> _expectedDemand;
    private readonly long _expectedCount;
    private SeedWinner? _winner;

    public MultiSeedCompetition(IReadOnlyList<ulong> seeds, IReadOnlyDictionary<int, int> expectedDemand)
    {
        ArgumentNullException.ThrowIfNull(seeds);
        ArgumentNullException.ThrowIfNull(expectedDemand);
        _seeds = seeds.ToArray();
        _expectedDemand = new Dictionary<int, int>(expectedDemand);
        if (_seeds.Length is < 1 or > 6 || _seeds.Any(s => s == 0) || _seeds.Distinct().Count() != _seeds.Length)
            throw new ArgumentException("Provide one to six distinct, nonzero seeds.", nameof(seeds));
        if (_expectedDemand.Count == 0 || _expectedDemand.Values.Any(count => count <= 0))
            throw new ArgumentException("Provide positive demand for every expected item.", nameof(expectedDemand));
        _expectedCount = _expectedDemand.Values.Sum(count => (long)count);
    }

    /// <summary>
    /// Only native-certified complete reports can replace the incumbent. The native
    /// strip width is the used length; shorter is better for this fixed input.
    /// </summary>
    public SeedWinner? TryOffer(int seedIndex, SolutionEventArgs report, string phase, bool isFinal = false,
        string? structureReportJson = null)
    {
        if ((uint)seedIndex >= (uint)_seeds.Length) throw new ArgumentOutOfRangeException(nameof(seedIndex));
        ArgumentNullException.ThrowIfNull(report);
        ArgumentNullException.ThrowIfNull(phase);
        if (!IsEligible(report)) return null;

        lock (_sync)
        {
            // Ties never switch seeds. The selected seed may finish at the same
            // length with another certified layout; adopt that final snapshot once.
            if (_winner != null && report.StripWidth >= _winner.Report.StripWidth)
            {
                if (report.StripWidth != _winner.Report.StripWidth || seedIndex != _winner.SeedIndex)
                    return null;
                if (SamePlacements(_winner.Report.ParsedPlacements, report.ParsedPlacements))
                {
                    string? structure = string.IsNullOrWhiteSpace(structureReportJson)
                        ? _winner.StructureReportJson : structureReportJson;
                    bool final = _winner.IsFinal || isFinal;
                    if (final == _winner.IsFinal && structure == _winner.StructureReportJson)
                        return null;
                    _winner = _winner with
                    {
                        Revision = _winner.Revision + 1,
                        IsFinal = final,
                        StructureReportJson = structure,
                        Phase = isFinal ? phase : _winner.Phase
                    };
                    return _winner;
                }
                if (!isFinal || _winner.IsFinal) return null;
                // Different final geometry falls through to copy the entire report
                // and its own metadata. Never attach it to the old live geometry.
            }
            var copy = new SolutionEventArgs(report.ReportType, report.StripWidth, report.Density,
                report.SolutionJson, report.Loss,
                Array.AsReadOnly(report.CollisionStatus.Select(status => new SolutionEventArgs.PlaceCollisionStatus
                {
                    ItemID = status.ItemID,
                    IsCollision = status.IsCollision
                }).ToArray()), report.HasLoss, report.HasCollisionStatus,
                Array.AsReadOnly(report.ParsedPlacements.ToArray()), report.ParsedStripWidth);
            _winner = new SeedWinner((_winner?.Revision ?? 0) + 1, seedIndex, _seeds[seedIndex],
                copy, phase, isFinal, structureReportJson);
            return _winner;
        }
    }

    public SeedWinner? Winner
    {
        get { lock (_sync) return _winner; }
    }

    public bool IsCurrent(SeedWinner winner)
    {
        lock (_sync) return ReferenceEquals(_winner, winner) && _winner != null;
    }

    private bool IsEligible(SolutionEventArgs report)
    {
        if (report.ReportType is not (ReportType.ExplFeas or ReportType.ExplImproving
                or ReportType.CmprFeas or ReportType.Final)
            || string.IsNullOrWhiteSpace(report.SolutionJson)
            || !float.IsFinite(report.StripWidth) || report.StripWidth <= 0
            || !float.IsFinite(report.Density) || report.Density <= 0
            || report.ParsedStripWidth is not float parsedWidth || !float.IsFinite(parsedWidth) || parsedWidth <= 0)
            return false;

        // The callback scalar and parsed JSON describe one snapshot; permit only
        // serialization-scale rounding, never a materially different export length.
        if (Math.Abs((double)report.StripWidth - parsedWidth) > Math.Max(0.0001, report.StripWidth * 0.000001))
            return false;
        // Density may exceed one with configured overlap or frozen overlap exemptions.
        if (report.HasLoss && (!float.IsFinite(report.Loss) || report.Loss != 0)) return false;
        // ExplImproving is not certified by kind: legacy Explore uses it for layouts that still
        // overlap, while grouped Explore uses it for certified snapshots. Require the complete
        // zero-collision status checked below instead of trusting the kind.
        if (report.ReportType == ReportType.ExplImproving && !report.HasCollisionStatus) return false;
        if (report.ParsedPlacements.Count != _expectedCount) return false;

        var remaining = new Dictionary<int, int>(_expectedDemand);
        foreach (var placement in report.ParsedPlacements)
        {
            if (placement == null || !float.IsFinite(placement.RotationDegrees)
                || !float.IsFinite(placement.TranslationX) || !float.IsFinite(placement.TranslationY)
                || !Consume(remaining, placement.ItemId))
                return false;
        }

        // Missing diagnostics are allowed because the report kind is certified.
        // When supplied, status must cover exactly the same demanded instances.
        if (report.HasCollisionStatus)
        {
            if (report.CollisionStatus.Count != _expectedCount) return false;
            remaining = new Dictionary<int, int>(_expectedDemand);
            foreach (var status in report.CollisionStatus)
                if (status == null || status.IsCollision || !Consume(remaining, status.ItemID)) return false;
        }
        return true;
    }

    private static bool Consume(Dictionary<int, int> remaining, int itemId)
    {
        if (!remaining.TryGetValue(itemId, out int count) || count <= 0) return false;
        remaining[itemId] = count - 1;
        return true;
    }

    private static bool SamePlacements(IReadOnlyList<SolutionEventArgs.ParsedPlacement> left,
        IReadOnlyList<SolutionEventArgs.ParsedPlacement> right)
    {
        if (left.Count != right.Count) return false;
        // Native snapshots may enumerate instances differently. Record equality
        // compares ID and all transform floats; counts preserve repeated instances.
        var remaining = new Dictionary<SolutionEventArgs.ParsedPlacement, int>();
        foreach (var placement in left)
            remaining[placement] = remaining.GetValueOrDefault(placement) + 1;
        foreach (var placement in right)
        {
            if (!remaining.TryGetValue(placement, out int count) || count == 0) return false;
            remaining[placement] = count - 1;
        }
        return true;
    }
}
