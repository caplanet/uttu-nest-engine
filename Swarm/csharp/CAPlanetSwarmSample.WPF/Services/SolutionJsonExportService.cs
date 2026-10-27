using System.Text.Json;
using System.Text.Json.Nodes;

namespace CAPlanetSwarmSample.WPF.Services;

/// <summary>Enriches Export JSON from the immutable input snapshot, without changing solver output.</summary>
public static class SolutionJsonExportService
{
    public static string? GetUttuImportWarning(string exportJson)
    {
        using var document = JsonDocument.Parse(exportJson);
        var root = document.RootElement;
        var solution = root.TryGetProperty("solution", out var wrapped) ? wrapped : root;
        int missing = 0;
        foreach (var item in solution.GetProperty("layout").GetProperty("placed_items").EnumerateArray())
            if (!item.TryGetProperty("OrgID", out var id) || id.ValueKind != JsonValueKind.Number || !id.TryGetInt64(out _)
                || !item.TryGetProperty("OrgTransform", out var frame) || frame.ValueKind != JsonValueKind.Object)
                missing++;
        return missing == 0 ? null : $"Uttu.Nest 가져오기 불가: {missing}개 배치에 OrgID 또는 OrgTransform이 없습니다. 업데이트된 Uttu.Nest에서 입력 JSON을 다시 내보내 실행하세요.";
    }

    public static string Create(string inputJson, string solutionJson)
    {
        var input = JsonNode.Parse(inputJson)?.AsObject() ?? throw new JsonException("입력이 없습니다.");
        var instance = (input["instance"] ?? input).AsObject();
        var items = instance["items"]?.AsArray() ?? throw new JsonException("입력 items가 없습니다.");
        var byId = new Dictionary<int, JsonObject>();
        foreach (var node in items)
        {
            var item = node?.AsObject() ?? throw new JsonException("잘못된 입력 item입니다.");
            int id = item["id"]?.GetValue<int>() ?? throw new JsonException("입력 id가 없습니다.");
            if (!byId.TryAdd(id, item)) throw new JsonException($"중복된 입력 id: {id}");
        }
        var output = JsonNode.Parse(solutionJson)?.AsObject() ?? throw new JsonException("결과가 없습니다.");
        var solution = (output["solution"] ?? output).AsObject();
        var placed = solution["layout"]?["placed_items"]?.AsArray() ?? throw new JsonException("결과 배치 목록이 없습니다.");
        foreach (var node in placed)
        {
            var item = node?.AsObject() ?? throw new JsonException("잘못된 결과 item입니다.");
            int id = item["item_id"]?.GetValue<int>() ?? throw new JsonException("결과 item_id가 없습니다.");
            if (!byId.TryGetValue(id, out var original)) throw new JsonException($"결과 item {id}의 입력이 없습니다.");
            item.Remove("OrgID");
            item.Remove("OrgTransform");
            if (original["OrgID"] is { } org)
            {
                // Preserve all 64 bits: never convert identity to double or the sequential item_id.
                item["OrgID"] = org.GetValue<long>();
                if (original["OrgTransform"] is { } frame)
                    item["OrgTransform"] = frame.DeepClone();
            }
        }
        solution["swarm_export_version"] = 1;
        solution["source_name"] = instance["name"]?.DeepClone();
        solution["source_strip_height"] = instance["strip_height"]?.DeepClone();
        return output.ToJsonString(new JsonSerializerOptions { WriteIndented = true });
    }
}
