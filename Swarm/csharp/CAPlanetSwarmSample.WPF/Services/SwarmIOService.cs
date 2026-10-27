using System.IO;

using CAPlanetSwarmSample.WPF.Data;

namespace CAPlanetSwarmSample.WPF.Services;

public static class SwarmIOService
{
    /// <summary>입력 파일을 읽고 WPF가 지원하는 Swarm V1/V2 최소 구조를 검증합니다.</summary>
    /// <param name="projectName">Workspace에 표시할 프로젝트 이름입니다.</param>
    /// <param name="inputJsonFileFullPath">읽을 JSON 파일의 절대 경로입니다.</param>
    /// <returns>성공 여부, Workspace 입력 정보, 실패 시 사용자에게 표시할 상세 원인입니다.</returns>
    public static (bool IsSuccess, SwarmInputData? InputData, string? ErrorMessage) InputJsonLoad(
        string projectName,
        string inputJsonFileFullPath)
    {
        try
        {
            if (!File.Exists(inputJsonFileFullPath))
            {
                return (false, null, $"입력 파일이 없습니다: {inputJsonFileFullPath}");
            }

            string json = File.ReadAllText(inputJsonFileFullPath);
            SwarmInputSummary summary = SwarmInputInspector.Parse(json);
            if (!summary.IsValid)
            {
                return (false, null, summary.ErrorMessage ?? "지원하는 Swarm 입력 형식이 아닙니다.");
            }

            // WPF renderer와 현재 C ABI는 item ID를 Int32로 처리하므로, 화면을 연 뒤 늦게
            // 실패하지 않도록 실제 item/hint 경계도 여기서 한 번 검증한다.
            _ = SwarmInputPlaces.Parse(json);

            string? directoryPath = Path.GetDirectoryName(inputJsonFileFullPath);
            if (string.IsNullOrWhiteSpace(directoryPath))
            {
                return (false, null, "입력 파일의 디렉터리 경로를 확인할 수 없습니다.");
            }

            var inputData = new SwarmInputData
            {
                JsonSnapshot = json,
                ProjectName = projectName,
                DirectoryPath = directoryPath,
                FileName = Path.GetFileNameWithoutExtension(inputJsonFileFullPath),
                FileExtension = Path.GetExtension(inputJsonFileFullPath).TrimStart('.')
            };
            return (true, inputData, null);
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException)
        {
            return (false, null, $"입력 파일을 읽지 못했습니다: {exception.Message}");
        }
        catch (Exception exception)
        {
            return (false, null, $"입력 파일 검증 중 오류가 발생했습니다: {exception.Message}");
        }
    }
}
