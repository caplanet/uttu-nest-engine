namespace CAPlanetSwarmSample.WPF.Data;

/// <summary>Structure 감사 JSON 파일 저장 결과.</summary>
public sealed class StructureReportExportResult
{
    /// <summary>파일 저장 성공 여부.</summary>
    public bool IsSuccess { get; init; }

    /// <summary>성공 시 저장된 절대 파일 경로.</summary>
    public string? FilePath { get; init; }

    /// <summary>실패 원인. 성공하면 <see langword="null"/>.</summary>
    public string? ErrorMessage { get; init; }
}
