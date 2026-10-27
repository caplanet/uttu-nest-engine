using System.Text.Json;

using Uttu.NestingV2;

namespace CAPlanetSwarmSample.WPF.Data;

/// <summary>
/// Swarm V1 입력 또는 V2 envelope에서 읽은 아이템과 선택적 배치 힌트 목록입니다.
/// </summary>
public sealed class SwarmInputPlaces
{
    /// <summary>
    /// 입력의 <c>items</c> 목록입니다. V2 입력이면 <c>instance.items</c>를 가리킵니다.
    /// </summary>
    public IReadOnlyList<SwarmInputPlaceItem> Items { get; private init; } = [];

    /// <summary>
    /// 입력 JSON을 파싱합니다. 정식 snake_case와 이전 WPF가 생성한 camelCase hint 필드를 모두 허용합니다.
    /// </summary>
    /// <param name="json">Swarm V1 입력 또는 V2 envelope JSON입니다.</param>
    /// <returns>아이템 ID와 실제로 존재하는 배치 힌트만 포함한 문서입니다.</returns>
    /// <exception cref="JsonException">입력 구조 또는 힌트 값이 올바르지 않을 때 발생합니다.</exception>
    public static SwarmInputPlaces Parse(string json)
    {
        using JsonDocument document = JsonDocument.Parse(json);
        JsonElement root = document.RootElement;
        if (root.ValueKind != JsonValueKind.Object)
        {
            throw new JsonException("Swarm 입력 root는 객체여야 합니다.");
        }

        JsonElement instance = root.TryGetProperty("instance", out JsonElement nestedInstance)
            ? nestedInstance
            : root;

        if (instance.ValueKind != JsonValueKind.Object)
        {
            throw new JsonException("Swarm instance는 객체여야 합니다.");
        }

        if (!instance.TryGetProperty("items", out JsonElement itemsElement)
            || itemsElement.ValueKind != JsonValueKind.Array)
        {
            throw new JsonException("Swarm 입력에 items 배열이 없습니다.");
        }

        var items = new List<SwarmInputPlaceItem>();
        foreach (JsonElement itemElement in itemsElement.EnumerateArray())
        {
            if (!itemElement.TryGetProperty("id", out JsonElement idElement)
                || !idElement.TryGetInt32(out int itemId))
            {
                throw new JsonException("Swarm item.id는 32비트 정수여야 합니다.");
            }

            CPlacedHint? placedHint = null;
            if (TryGetProperty(itemElement, out JsonElement hintElement, "placed_hint", "placedHint")
                && hintElement.ValueKind != JsonValueKind.Null)
            {
                if (hintElement.ValueKind != JsonValueKind.Object)
                {
                    throw new JsonException($"item {itemId}의 placed_hint는 객체여야 합니다.");
                }

                placedHint = ParseHint(hintElement, itemId);
            }

            items.Add(new SwarmInputPlaceItem
            {
                ItemId = itemId,
                OrgID = ReadOriginalId(itemElement, itemId),
                PlacedHint = placedHint
            });
        }

        return new SwarmInputPlaces { Items = items };
    }

    /// <summary>
    /// V1 네이티브 초기화에 전달할 힌트를 만듭니다. 고정 힌트는 항상 유지하고,
    /// 이동 가능한 warm-start 힌트는 <paramref name="useMovableHints"/>가 true일 때만 유지합니다.
    /// </summary>
    /// <param name="useMovableHints">이동 가능한 warm-start 위치를 사용할지 여부입니다.</param>
    /// <returns>실제로 입력에 존재한 힌트만 담은 배열이며, 없으면 빈 배열입니다.</returns>
    public CPlacedHint[] ToNativeHints(bool useMovableHints)
    {
        return Items
            .Where(item => item.PlacedHint.HasValue)
            .Select(item => item.PlacedHint!.Value)
            .Where(hint => hint.IsFreeze != 0 || useMovableHints)
            .ToArray();
    }

    private static CPlacedHint ParseHint(JsonElement hint, int fallbackItemId)
    {
        var result = new CPlacedHint
        {
            ItemId = ReadOptionalInt32(hint, fallbackItemId, "item_id", "itemId"),
            IsFreeze = ReadOptionalFreeze(hint, 0, "is_freeze", "isFreeze"),
            TrX = ReadOptionalSingle(hint, 0f, "tr_x", "trX"),
            TrY = ReadOptionalSingle(hint, 0f, "tr_y", "trY"),
            RotAngleInRad = ReadOptionalSingle(hint, 0f, "rot_angle_in_rad", "rotAngleInRad"),
            StripeIntervalY = ReadOptionalSingle(hint, 0f, "stripe_interval_y", "stripeIntervalY"),
            StripeIntervalX = ReadOptionalSingle(hint, 0f, "stripe_interval_x", "stripeIntervalX")
        };

        if (result.ItemId != fallbackItemId)
        {
            throw new JsonException(
                $"item {fallbackItemId}의 placed_hint.item_id {result.ItemId}가 외부 item.id와 다릅니다.");
        }

        return result;
    }

    private static long? ReadOriginalId(JsonElement item, int itemId)
    {
        if (!item.TryGetProperty("OrgID", out var value) || value.ValueKind == JsonValueKind.Null)
            return null;
        if (value.ValueKind == JsonValueKind.Number && value.TryGetInt64(out long originalId))
            return originalId;
        throw new JsonException($"item {itemId}의 OrgID는 64비트 정수여야 합니다.");
    }

    private static int ReadOptionalInt32(JsonElement parent, int defaultValue, params string[] names)
    {
        if (!TryGetProperty(parent, out JsonElement value, names))
        {
            return defaultValue;
        }

        if (value.TryGetInt32(out int result))
        {
            return result;
        }

        throw new JsonException($"{string.Join('/', names)}는 32비트 정수여야 합니다.");
    }

    private static int ReadOptionalFreeze(JsonElement parent, int defaultValue, params string[] names)
    {
        if (!TryGetProperty(parent, out JsonElement value, names))
        {
            return defaultValue;
        }

        if (value.ValueKind == JsonValueKind.True)
        {
            return 1;
        }
        if (value.ValueKind == JsonValueKind.False)
        {
            return 0;
        }
        if (value.TryGetInt32(out int result))
        {
            return result;
        }

        throw new JsonException($"{string.Join('/', names)}는 정수 또는 bool이어야 합니다.");
    }

    private static float ReadOptionalSingle(JsonElement parent, float defaultValue, params string[] names)
    {
        if (!TryGetProperty(parent, out JsonElement value, names))
        {
            return defaultValue;
        }

        if (value.TryGetSingle(out float result) && float.IsFinite(result))
        {
            return result;
        }

        throw new JsonException($"{string.Join('/', names)}는 유한한 숫자여야 합니다.");
    }

    private static bool TryGetProperty(
        JsonElement parent,
        out JsonElement value,
        params string[] names)
    {
        foreach (string name in names)
        {
            if (parent.TryGetProperty(name, out value))
            {
                return true;
            }
        }

        value = default;
        return false;
    }
}

/// <summary>Swarm 입력 아이템 하나와 그 아이템에 명시된 선택적 배치 힌트입니다.</summary>
public sealed class SwarmInputPlaceItem
{
    /// <summary>네이티브 item ID입니다.</summary>
    public int ItemId { get; init; }

    /// <summary>Uttu.Nest Place(MkPart)의 원래 ID. 없는 이전 입력은 null이며 네이티브 인덱스로 사용하지 않습니다.</summary>
    public long? OrgID { get; init; }

    /// <summary>
    /// 입력에 <c>placed_hint</c>가 있을 때만 값이 있습니다. null은 fresh LBF 배치를 의미합니다.
    /// </summary>
    public CPlacedHint? PlacedHint { get; init; }
}
