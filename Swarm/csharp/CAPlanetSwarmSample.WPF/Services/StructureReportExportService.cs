using CAPlanetSwarmSample.WPF.Data;
using System;
using System.IO;
using System.Globalization;
using System.Linq;
using System.Text;

namespace CAPlanetSwarmSample.WPF.Services;

/// <summary>Structure 감사 JSON의 command 활성화 조건, 파일명 및 저장을 일관되게 처리한다.</summary>
public static class StructureReportExportService
{
    /// <summary>현재 WPF 상태에서 report 저장 command를 실행할 수 있는지 확인한다.</summary>
    /// <param name="isRunning">native phase 실행 중 여부.</param>
    /// <param name="reportJson">가장 최근에 완료된 Structure report JSON.</param>
    public static bool CanExport(bool isRunning, string? reportJson) =>
        !isRunning && !string.IsNullOrWhiteSpace(reportJson);

    /// <summary>프로젝트명과 완료 시각으로 안전한 report 파일명을 만든다.</summary>
    /// <param name="projectName">입력 파일에서 가져온 프로젝트명.</param>
    /// <param name="completedAt">Structure 완료 시각.</param>
    /// <returns>확장자를 포함한 파일명.</returns>
    public static string BuildFileName(string? projectName, DateTimeOffset completedAt)
    {
        string source = string.IsNullOrWhiteSpace(projectName) ? "swarm" : projectName.Trim();
        char[] invalidChars = Path.GetInvalidFileNameChars();
        var builder = new StringBuilder(source.Length);
        foreach (char character in source)
        {
            builder.Append(invalidChars.Contains(character) ? '_' : character);
        }

        string safeProjectName = builder.ToString().Trim(' ', '.');
        if (safeProjectName.Length == 0)
        {
            safeProjectName = "swarm";
        }

        string timestamp = completedAt.ToString("yyyyMMdd_HHmmss_fff", CultureInfo.InvariantCulture);
        return $"{safeProjectName}_structure_report_{timestamp}.json";
    }

    /// <summary>schema 3 report를 검증한 뒤 output 폴더에 원문 그대로 저장한다.</summary>
    /// <param name="reportJson">native가 반환한 Structure report JSON.</param>
    /// <param name="outputDirectory">저장할 output 디렉터리.</param>
    /// <param name="projectName">파일명 앞에 붙일 프로젝트명.</param>
    /// <param name="completedAt">파일명에 사용할 완료 시각. 생략하면 현재 지역 시각.</param>
    /// <returns>성공 시 충돌 없이 만들어진 절대 파일 경로.</returns>
    public static StructureReportExportResult Save(
        string? reportJson,
        string outputDirectory,
        string? projectName,
        DateTimeOffset? completedAt = null)
    {
        if (string.IsNullOrWhiteSpace(reportJson))
        {
            return Failure("저장할 Structure report가 없습니다.");
        }

        StructureReportSummary summary = StructureReportInspector.Parse(reportJson);
        if (!summary.IsValid)
        {
            return Failure(summary.ErrorMessage ?? "Structure report가 올바르지 않습니다.");
        }

        if (string.IsNullOrWhiteSpace(outputDirectory))
        {
            return Failure("output 디렉터리가 비어 있습니다.");
        }

        try
        {
            string directory = Path.GetFullPath(outputDirectory);
            Directory.CreateDirectory(directory);
            string fileName = BuildFileName(projectName, completedAt ?? DateTimeOffset.Now);
            string stem = Path.GetFileNameWithoutExtension(fileName);
            string extension = Path.GetExtension(fileName);

            for (int suffix = 1; suffix <= 10_000; suffix++)
            {
                string candidateName = suffix == 1
                    ? fileName
                    : $"{stem}_{suffix}{extension}";
                string candidatePath = Path.Combine(directory, candidateName);
                try
                {
                    using var stream = new FileStream(
                        candidatePath,
                        FileMode.CreateNew,
                        FileAccess.Write,
                        FileShare.Read);
                    using var writer = new StreamWriter(stream, new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));
                    writer.Write(reportJson);
                    return new StructureReportExportResult
                    {
                        IsSuccess = true,
                        FilePath = candidatePath
                    };
                }
                catch (IOException) when (File.Exists(candidatePath))
                {
                    // 같은 millisecond에 저장한 report가 있으면 _2, _3 순서로 보존한다.
                }
            }

            return Failure("사용 가능한 Structure report 파일명을 만들지 못했습니다.");
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
        {
            return Failure($"Structure report 저장 실패: {exception.Message}");
        }
    }

    private static StructureReportExportResult Failure(string message) => new()
    {
        IsSuccess = false,
        ErrorMessage = message
    };
}
