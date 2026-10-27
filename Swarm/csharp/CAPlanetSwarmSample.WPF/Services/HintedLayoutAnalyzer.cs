using System.Text.Json;

using CAPlanetSwarmSample.WPF.Data;

namespace CAPlanetSwarmSample.WPF.Services;

/// <summary>
/// 입력 JSON의 <c>items[*].shape</c>와 <c>items[*].placed_hint</c>로 배치를 복원해
/// 효율과 겹침을 계산합니다. 네이티브를 호출하지 않는 순수 계산이라 파일을 여는
/// 즉시 결과를 보여줄 수 있습니다.
/// </summary>
public static class HintedLayoutAnalyzer
{
    /// <summary>겹침 검사를 수행하는 최대 조각 수입니다. 그 이상이면 효율만 계산합니다.</summary>
    private const int OverlapCheckItemLimit = 4000;

    /// <summary>관통으로 셀 최소 깊이(mm). 부동소수 잡음을 겹침으로 세지 않기 위한 값입니다.</summary>
    private const double PenetrationEpsilonMm = 1e-9;

    /// <summary>스트립 폭 이탈로 셀 최소 초과량(mm)입니다.</summary>
    private const double StripMarginEpsilonMm = 1e-6;

    /// <summary>
    /// 인스턴스 요소에서 힌트 배치의 효율과 겹침을 계산합니다.
    /// </summary>
    /// <param name="instance">
    /// <c>items</c>와 <c>strip_height</c>를 가진 인스턴스 요소입니다.
    /// V2 입력이면 <c>instance</c>, V1이면 root를 넘깁니다.
    /// </param>
    /// <returns>계산 결과입니다. 힌트나 도형이 없으면 사용 불가 상태를 돌려줍니다.</returns>
    public static HintedLayoutMetrics Analyze(JsonElement instance)
    {
        if (instance.ValueKind != JsonValueKind.Object ||
            !instance.TryGetProperty("items", out JsonElement items) ||
            items.ValueKind != JsonValueKind.Array)
        {
            return HintedLayoutMetrics.Unavailable("items 배열이 없습니다.");
        }

        double stripHeight = ReadStripHeight(instance);

        var polygons = new List<double[]>();     // [x0,y0,x1,y1,...] 배치 좌표
        var boxes = new List<double[]>();        // [minX,minY,maxX,maxY]
        int missingHints = 0;
        int itemCount = 0;

        foreach (JsonElement item in items.EnumerateArray())
        {
            itemCount++;
            if (!TryReadHint(item, out double trX, out double trY))
            {
                missingHints++;
                continue;
            }

            if (!TryReadShape(item, out double[]? ring) || ring is null)
            {
                missingHints++;
                continue;
            }

            double minX = double.MaxValue, minY = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue;
            for (int i = 0; i < ring.Length; i += 2)
            {
                ring[i] += trX;
                ring[i + 1] += trY;
                minX = Math.Min(minX, ring[i]);
                maxX = Math.Max(maxX, ring[i]);
                minY = Math.Min(minY, ring[i + 1]);
                maxY = Math.Max(maxY, ring[i + 1]);
            }

            polygons.Add(ring);
            boxes.Add([minX, minY, maxX, maxY]);
        }

        if (polygons.Count == 0)
        {
            return HintedLayoutMetrics.Unavailable(
                itemCount == 0 ? "item이 없습니다." : "placed_hint가 없습니다.");
        }

        // 효율 --------------------------------------------------------------
        // 길이는 원점부터 가장 오른쪽 끝까지로 잡는다. 스트립 패킹의 스트립은 x=0에서
        // 시작하므로, 배치가 x>0에서 시작해도 그 앞의 빈 공간은 낭비로 세는 것이 맞다.
        double area = polygons.Sum(SignedAreaAbs);
        double layoutMinX = boxes.Min(b => b[0]);
        double layoutMaxX = boxes.Max(b => b[2]);
        double length = layoutMaxX - Math.Min(0.0, layoutMinX);
        double efficiency = (length > 0 && stripHeight > 0)
            ? 100.0 * area / (length * stripHeight)
            : 0.0;

        int outOfStrip = 0;
        if (stripHeight > 0)
        {
            outOfStrip = boxes.Count(b =>
                b[1] < -StripMarginEpsilonMm || b[3] > stripHeight + StripMarginEpsilonMm);
        }

        // 겹침 --------------------------------------------------------------
        bool skipOverlap = polygons.Count > OverlapCheckItemLimit;
        int overlapPairs = 0;
        double maxPenetration = 0.0;
        if (!skipOverlap)
        {
            (overlapPairs, maxPenetration) = CountOverlaps(polygons, boxes);
        }

        return new HintedLayoutMetrics
        {
            IsAvailable = true,
            PlacedCount = polygons.Count,
            MissingHintCount = missingHints,
            PieceAreaMm2 = area,
            LengthMm = length,
            StripHeightMm = stripHeight,
            EfficiencyPct = efficiency,
            OverlapPairCount = overlapPairs,
            MaxPenetrationMm = maxPenetration,
            OutOfStripCount = outOfStrip,
            OverlapCheckSkipped = skipOverlap
        };
    }

    private static double ReadStripHeight(JsonElement instance)
    {
        if (instance.TryGetProperty("strip_height", out JsonElement value) ||
            instance.TryGetProperty("stripHeight", out value))
        {
            if (value.ValueKind == JsonValueKind.Number && value.TryGetDouble(out double height))
            {
                return height;
            }
        }

        return 0.0;
    }

    private static bool TryReadHint(JsonElement item, out double trX, out double trY)
    {
        trX = 0;
        trY = 0;
        if (item.ValueKind != JsonValueKind.Object)
        {
            return false;
        }

        if ((!item.TryGetProperty("placed_hint", out JsonElement hint) &&
             !item.TryGetProperty("placedHint", out hint)) ||
            hint.ValueKind != JsonValueKind.Object)
        {
            return false;
        }

        trX = ReadDouble(hint, "tr_x", "trX");
        trY = ReadDouble(hint, "tr_y", "trY");

        // 회전이 걸린 힌트는 평행이동만으로 배치를 복원할 수 없다. 잘못된 수치를
        // 보여주느니 그 조각을 빼는 편이 낫다.
        double rotation = ReadDouble(hint, "rot_angle_in_rad", "rotAngleInRad");
        return Math.Abs(rotation) <= 1e-12 && double.IsFinite(trX) && double.IsFinite(trY);
    }

    private static double ReadDouble(JsonElement parent, string snake, string camel)
    {
        if ((parent.TryGetProperty(snake, out JsonElement value) ||
             parent.TryGetProperty(camel, out value)) &&
            value.ValueKind == JsonValueKind.Number &&
            value.TryGetDouble(out double result))
        {
            return result;
        }

        return 0.0;
    }

    /// <summary>도형을 평탄한 [x0,y0,x1,y1,...] 배열로 읽습니다.</summary>
    private static bool TryReadShape(JsonElement item, out double[]? ring)
    {
        ring = null;
        if (!item.TryGetProperty("shape", out JsonElement shape) ||
            shape.ValueKind != JsonValueKind.Object ||
            !shape.TryGetProperty("data", out JsonElement data) ||
            data.ValueKind != JsonValueKind.Array)
        {
            return false;
        }

        int n = data.GetArrayLength();
        if (n < 3)
        {
            return false;
        }

        var flat = new double[n * 2];
        int index = 0;
        foreach (JsonElement point in data.EnumerateArray())
        {
            if (point.ValueKind != JsonValueKind.Array || point.GetArrayLength() < 2)
            {
                return false;
            }

            JsonElement px = point[0];
            JsonElement py = point[1];
            if (!px.TryGetDouble(out double x) || !py.TryGetDouble(out double y))
            {
                return false;
            }

            flat[index++] = x;
            flat[index++] = y;
        }

        ring = flat;
        return true;
    }

    private static double SignedAreaAbs(double[] ring)
    {
        double sum = 0;
        int n = ring.Length / 2;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            sum += ring[i * 2] * ring[j * 2 + 1] - ring[j * 2] * ring[i * 2 + 1];
        }

        return Math.Abs(sum) / 2.0;
    }

    /// <summary>
    /// 관통이 있는 조각 쌍 수와 최대 관통 깊이를 셉니다.
    /// <para>
    /// bbox가 겹치는 쌍만 본 뒤, 한쪽 정점이 다른 쪽 내부에 들어갔는지로 판정합니다.
    /// 정점이 들어가지 않아도 변이 서로 교차하면 겹친 것이므로 그 경우도 셉니다.
    /// 깊이는 내부로 들어간 정점에서 상대 경계까지의 거리로, 필요한 겹침 허용값과
    /// 같은 단위입니다.
    /// </para>
    /// <para>
    /// x축 정렬 후 스위프로 후보를 줄여, 조각이 길게 늘어선 마커에서도 실질 O(n·k)입니다.
    /// </para>
    /// </summary>
    private static (int PairCount, double MaxPenetration) CountOverlaps(
        List<double[]> polygons,
        List<double[]> boxes)
    {
        int n = polygons.Count;
        int[] order = [.. Enumerable.Range(0, n).OrderBy(i => boxes[i][0])];

        int pairs = 0;
        double maxDepth = 0.0;
        var active = new List<int>();

        foreach (int i in order)
        {
            double[] bi = boxes[i];

            // 오른쪽 끝이 현재 조각의 왼쪽보다 앞선 조각은 더 이상 겹칠 수 없다.
            active.RemoveAll(j => boxes[j][2] < bi[0]);

            foreach (int j in active)
            {
                double[] bj = boxes[j];
                if (bj[3] < bi[1] || bi[3] < bj[1])
                {
                    continue;
                }

                // (1) 한쪽 정점이 상대 안으로 들어간 경우. 실제 마커의 양자화 관통은
                //     거의 전부 여기서 잡히고, 깊이도 이 값이 정확하다.
                double depth = Math.Max(
                    Penetration(polygons[i], polygons[j]),
                    Penetration(polygons[j], polygons[i]));

                if (depth > PenetrationEpsilonMm)
                {
                    pairs++;
                    maxDepth = Math.Max(maxDepth, depth);
                    continue;
                }

                // (2) 정점이 들어가지 않아도 변이 실제로 교차하면 겹친 것이다.
                //     스쳐 지나가는 형태라 깊이는 0으로 둔다.
                if (EdgesCross(polygons[i], polygons[j]))
                {
                    pairs++;
                    continue;
                }

                // (3) 경계가 서로 정확히 맞물려 정점도 교차도 없는 경우
                //     (예: 높이가 같은 두 사각형이 x로만 겹칠 때). bbox 교집합의
                //     중심이 양쪽 내부면 겹친 것이고, 그때 깊이는 교집합의 짧은 변으로
                //     근사한다. 이 경로로만 쓰는 근사라 (1)의 정확한 깊이를 흐리지 않는다.
                double ox0 = Math.Max(bi[0], bj[0]);
                double oy0 = Math.Max(bi[1], bj[1]);
                double ox1 = Math.Min(bi[2], bj[2]);
                double oy1 = Math.Min(bi[3], bj[3]);
                if (ox1 - ox0 <= PenetrationEpsilonMm || oy1 - oy0 <= PenetrationEpsilonMm)
                {
                    continue;
                }

                double cx = (ox0 + ox1) / 2.0;
                double cy = (oy0 + oy1) / 2.0;
                if (Contains(polygons[i], cx, cy) && Contains(polygons[j], cx, cy))
                {
                    pairs++;
                    maxDepth = Math.Max(maxDepth, Math.Min(ox1 - ox0, oy1 - oy0));
                }
            }

            active.Add(i);
        }

        return (pairs, maxDepth);
    }

    /// <summary><paramref name="probe"/>의 정점 중 <paramref name="target"/> 안에 있는 것의 최대 침투 깊이입니다.</summary>
    private static double Penetration(double[] probe, double[] target)
    {
        double best = 0.0;
        for (int i = 0; i < probe.Length; i += 2)
        {
            double x = probe[i];
            double y = probe[i + 1];
            if (!Contains(target, x, y))
            {
                continue;
            }

            double nearest = double.MaxValue;
            int n = target.Length / 2;
            for (int k = 0; k < n; k++)
            {
                int m = (k + 1) % n;
                nearest = Math.Min(nearest, PointSegmentDistance(
                    x, y,
                    target[k * 2], target[k * 2 + 1],
                    target[m * 2], target[m * 2 + 1]));
            }

            best = Math.Max(best, nearest);
        }

        return best;
    }

    private static bool Contains(double[] ring, double x, double y)
    {
        bool inside = false;
        int n = ring.Length / 2;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            double yi = ring[i * 2 + 1];
            double yj = ring[j * 2 + 1];
            if (yi > y == yj > y)
            {
                continue;
            }

            double xi = ring[i * 2];
            double xj = ring[j * 2];
            double cross = xi + (y - yi) * (xj - xi) / (yj - yi);
            if (x < cross)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    private static double PointSegmentDistance(
        double px, double py, double ax, double ay, double bx, double by)
    {
        double vx = bx - ax;
        double vy = by - ay;
        double lengthSquared = vx * vx + vy * vy;
        double t = lengthSquared > 0
            ? Math.Clamp(((px - ax) * vx + (py - ay) * vy) / lengthSquared, 0.0, 1.0)
            : 0.0;
        double dx = px - (ax + t * vx);
        double dy = py - (ay + t * vy);
        return Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>두 링의 변이 실제로 교차하는지 봅니다. 정점이 서로 안에 없어도 겹칠 수 있습니다.</summary>
    private static bool EdgesCross(double[] a, double[] b)
    {
        int na = a.Length / 2;
        int nb = b.Length / 2;
        for (int i = 0; i < na; i++)
        {
            int i2 = (i + 1) % na;
            double ax = a[i * 2], ay = a[i * 2 + 1];
            double bx = a[i2 * 2], by = a[i2 * 2 + 1];
            for (int j = 0; j < nb; j++)
            {
                int j2 = (j + 1) % nb;
                if (SegmentsIntersect(
                        ax, ay, bx, by,
                        b[j * 2], b[j * 2 + 1], b[j2 * 2], b[j2 * 2 + 1]))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private static bool SegmentsIntersect(
        double p0x, double p0y, double p1x, double p1y,
        double q0x, double q0y, double q1x, double q1y)
    {
        double d1 = Cross(q0x, q0y, q1x, q1y, p0x, p0y);
        double d2 = Cross(q0x, q0y, q1x, q1y, p1x, p1y);
        double d3 = Cross(p0x, p0y, p1x, p1y, q0x, q0y);
        double d4 = Cross(p0x, p0y, p1x, p1y, q1x, q1y);

        // 접촉(공선/끝점 맞닿음)은 겹침이 아니므로 진짜 교차만 센다.
        return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
               ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
    }

    private static double Cross(
        double ax, double ay, double bx, double by, double px, double py) =>
        (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}
