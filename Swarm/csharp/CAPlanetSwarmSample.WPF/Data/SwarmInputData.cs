namespace CAPlanetSwarmSample.WPF.Data;

public class SwarmInputData
{
    /// <summary>파일을 열 때 한 번 읽어 검증한 불변 JSON snapshot입니다.</summary>
    public string JsonSnapshot { get; set; } = string.Empty;

    public string ProjectName { get; set; } = string.Empty;

    public string DirectoryPath { get; set; } = string.Empty;

    public string FileName { get; set; } = string.Empty;

    public string FileExtension { get; set; } = string.Empty;
}
