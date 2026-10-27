using System.IO;
using System.Text;

namespace CAPlanetSwarmSample.WPF;

public static class StringExtension
{
    public static string ToReadJsonUTF8(this string jsonFileFullPath)
    {
        try { return File.ReadAllText(jsonFileFullPath, Encoding.UTF8); }
        catch { return string.Empty; }
    }
}
