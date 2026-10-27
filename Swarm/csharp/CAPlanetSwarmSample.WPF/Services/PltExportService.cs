using System.IO;
using System.Text;
using System.Text.Json;

namespace CAPlanetSwarmSample.WPF.Services;

/// <summary>Plots original input contours at Swarm's public (degree/mm) solution poses.</summary>
public static class PltExportService
{
    // HPGL's default plotter unit is 0.025 mm. Matches the supplied UTTU PLT.
    public const double UnitsPerMillimeter = 40;
    private readonly record struct Point(double X, double Y);
    private readonly record struct PlotPoint(int X, int Y);

    public static string Create(string inputJson, string solutionJson)
    {
        using var input = JsonDocument.Parse(inputJson);
        using var output = JsonDocument.Parse(solutionJson);
        var instance = input.RootElement.TryGetProperty("instance", out var nested) ? nested : input.RootElement;
        var solution = output.RootElement.TryGetProperty("solution", out var wrapped) ? wrapped : output.RootElement;
        double height = Positive(instance.GetProperty("strip_height"));
        double width = Positive(solution.GetProperty("strip_width"));
        var shapes = new Dictionary<int, Point[]>();
        foreach (var item in instance.GetProperty("items").EnumerateArray())
        {
            int id = item.GetProperty("id").GetInt32();
            if (!shapes.TryAdd(id, ReadShape(item.GetProperty("shape"))))
                throw new JsonException($"중복된 item ID: {id}");
        }

        var contours = new List<PlotPoint[]>();
        double maxX = width, maxY = height;
        foreach (var placed in solution.GetProperty("layout").GetProperty("placed_items").EnumerateArray())
        {
            int id = placed.GetProperty("item_id").GetInt32();
            if (!shapes.TryGetValue(id, out var original))
                throw new JsonException($"결과 item {id}의 원본 도형이 없습니다.");
            var pose = placed.GetProperty("transformation");
            double angle = Number(pose.GetProperty("rotation")) * Math.PI / 180;
            var translation = ReadPoint(pose.GetProperty("translation"));
            double sin = Math.Sin(angle), cos = Math.Cos(angle);
            var contour = original.Select(p =>
            {
                double x = cos * p.X - sin * p.Y + translation.X;
                double y = sin * p.X + cos * p.Y + translation.Y;
                maxX = Math.Max(maxX, x); maxY = Math.Max(maxY, y);
                return new PlotPoint(ToUnits(x), ToUnits(y));
            }).ToArray();
            if (contour.Distinct().Count() < 3)
                throw new JsonException($"item {id}가 PLT의 0.025mm 격자에서 소실됩니다.");
            contours.Add(contour);
        }
        if (contours.Count == 0) throw new JsonException("내보낼 배치 결과가 없습니다.");

        var text = new StringBuilder();
        void Line(string s) => text.Append(s).Append("\r\n");
        string Pair(PlotPoint p) => FormattableString.Invariant($"{p.X},{p.Y}");
        Line(FormattableString.Invariant($"NE{Math.Ceiling(maxX)},{Math.Ceiling(maxY)};"));
        Line("IN;");
        for (int pen = 1; pen <= 8; pen++) Line($"VS32,{pen};");
        Line("WU0;");
        for (int pen = 1; pen <= 8; pen++) Line($"PW0.350,{pen};");
        Line("PA;");
        Line("PU;");
        Line("SP1;");
        void Contour(PlotPoint[] points)
        {
            Line($"PU{Pair(points[0])};");
            for (int i = 1; i < points.Length; i++)
                if (points[i] != points[i - 1]) Line($"PD{Pair(points[i])};");
            if (points[^1] != points[0]) Line($"PD{Pair(points[0])};");
            Line("PU;");
        }
        foreach (var contour in contours) Contour(contour);
        Contour([new(0, 0), new(ToUnits(width), 0), new(ToUnits(width), ToUnits(height)), new(0, ToUnits(height))]);
        Line("SP0;");
        return text.ToString();
    }

    /// <summary>Validate completely before writing; never leaves a partial PLT or overwrites another export.</summary>
    public static void Save(string inputJson, string solutionJson, string path)
    {
        string text = Create(inputJson, solutionJson);
        string destination = Path.GetFullPath(path);
        Directory.CreateDirectory(Path.GetDirectoryName(destination)!);
        string temporary = destination + "." + Guid.NewGuid().ToString("N") + ".tmp";
        try
        {
            File.WriteAllText(temporary, text, Encoding.ASCII);
            File.Move(temporary, destination, overwrite: false);
        }
        finally
        {
            if (File.Exists(temporary)) File.Delete(temporary);
        }
    }

    private static int ToUnits(double mm)
    {
        double units = Math.Round(mm * UnitsPerMillimeter, MidpointRounding.AwayFromZero);
        if (!double.IsFinite(units) || units < int.MinValue || units > int.MaxValue)
            throw new JsonException("좌표가 PLT 정수 범위를 벗어났습니다.");
        return (int)units;
    }

    private static double Number(JsonElement element)
    {
        if (!element.TryGetDouble(out double value) || !double.IsFinite(value))
            throw new JsonException("도형/배치 좌표는 유한한 숫자여야 합니다.");
        return value;
    }

    private static double Positive(JsonElement element)
    {
        double value = Number(element);
        if (value <= 0) throw new JsonException("도형/원단 크기는 양수여야 합니다.");
        return value;
    }

    private static Point ReadPoint(JsonElement point)
    {
        if (point.ValueKind != JsonValueKind.Array || point.GetArrayLength() != 2)
            throw new JsonException("좌표는 [x,y] 배열이어야 합니다.");
        return new(Number(point[0]), Number(point[1]));
    }

    private static Point[] ReadShape(JsonElement shape)
    {
        var data = shape.GetProperty("data");
        switch (shape.GetProperty("type").GetString())
        {
            case "simple_polygon":
                var points = data.EnumerateArray().Select(ReadPoint).ToArray();
                if (points.Length < 3) throw new JsonException("폴리곤 정점이 부족합니다.");
                return points;
            case "rectangle":
                double x = Number(data.GetProperty("x_min")), y = Number(data.GetProperty("y_min"));
                double w = Positive(data.GetProperty("width")), h = Positive(data.GetProperty("height"));
                return [new(x, y), new(x + w, y), new(x + w, y + h), new(x, y + h)];
            default:
                throw new JsonException("PLT 내보내기는 simple_polygon/rectangle 원본 도형을 지원합니다.");
        }
    }
}
