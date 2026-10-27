"""Extract the current Uttu OuterSimplify into an isolated, reproducible probe."""
from pathlib import Path
import hashlib, json

root = Path(__file__).resolve().parents[1]
out = root / 'target/overlap-margin-audit'
out.mkdir(parents=True, exist_ok=True)
source = Path(r'C:\Caplanet\Products\uttu\domains\Uttu.Nest.Models\Nesting\NestSource.cs')
text = source.read_text(encoding='utf-8-sig')
method = text[text.index('    private static Path64 OuterSimplify'):text.index('    #endregion', text.index('    private static Path64 OuterSimplify'))]
source_input = root / 'data/input/wsjk156.json'
doc = json.loads(source_input.read_text(encoding='utf-8-sig'))
doc['items'].append({'id': max(i['id'] for i in doc['items'])+1, 'demand': 2, 'allowed_orientations': [0], 'shape': {'type':'simple_polygon','data':[[0,0],[40,0],[40,40],[0,40]]}})
(out/'input.json').write_text(json.dumps(doc), encoding='utf-8')
(out/'probe.csproj').write_text('''<Project Sdk="Microsoft.NET.Sdk"><PropertyGroup><OutputType>Exe</OutputType><TargetFramework>net10.0</TargetFramework><ImplicitUsings>enable</ImplicitUsings></PropertyGroup><ItemGroup><Reference Include="Clipper2Lib"><HintPath>C:/Users/mwlee/.nuget/packages/clipper2/2.0.0/lib/netstandard2.0/Clipper2Lib.dll</HintPath></Reference></ItemGroup></Project>''', encoding='utf-8')
program = '''using Clipper2Lib;
using System.Diagnostics;
using System.Text.Json.Nodes;
class Program {
static void Main(string[] args) {
 if(args[0]=="--verify") { Verify(args[1]); return; }
 var original=JsonNode.Parse(File.ReadAllText(args[0]))!;
 foreach(double tol in new double[]{-1,0,0.5,1,2,3,5}) {
  var doc=original.DeepClone();
  foreach(var it in doc["items"]!.AsArray()) {
   if(tol<0) continue;
   var src=new Path64(it!["shape"]!["data"]!.AsArray().Select(p=>new Point64((long)Math.Round(p![0]!.GetValue<double>()*1000),(long)Math.Round(p[1]!.GetValue<double>()*1000))));
   if(!Clipper.IsPositive(src)) src=Clipper.ReversePath(src);
   var union=Clipper.Union(new Paths64{src},FillRule.NonZero).MaxBy(p=>Math.Abs(Clipper.Area(p)))!;
   var simp=OuterSimplify(union,tol*1000);
   var result=Clipper.Union(new Paths64{simp,union},FillRule.NonZero).MaxBy(p=>Math.Abs(Clipper.Area(p)))!;
   result=Clipper.StripDuplicates(result,true);
   result=Clipper.Union(new Paths64{result},FillRule.NonZero).MaxBy(p=>Clipper.Area(p))!;
   result=Clipper.InflatePaths(new Paths64{result},0,JoinType.Miter,EndType.Polygon).MaxBy(p=>Math.Abs(Clipper.Area(p)))!;
   if(!Clipper.IsPositive(result)) result=Clipper.ReversePath(result);
   it["shape"]!["data"]=new JsonArray(result.Select(p=>(JsonNode)new JsonArray(JsonValue.Create(p.X/1000.0),JsonValue.Create(p.Y/1000.0))).ToArray());
  }
  File.WriteAllText(Path.Combine(args[1],$"uttu-{tol.ToString(System.Globalization.CultureInfo.InvariantCulture)}.json"),doc.ToJsonString());
 }
}
'''+method+'''
static void Verify(string directory) {
 var report=JsonNode.Parse(File.ReadAllText(Path.Combine(directory,"analysis.json")))!;
 var result=new JsonArray();
 foreach(var row in report["contacts"]!.AsArray()) {
  var w=row!["worst"]; if(w is null) continue;
  var dx=w["shift"]![0]!.GetValue<double>(); var dy=w["shift"]![1]!.GetValue<double>();
  Path64 Path(string name,bool move) => new Path64(w[name]!.AsArray().Select(p=>new Point64((long)Math.Round((p![0]!.GetValue<double>()+(move?dx:0))*10000),(long)Math.Round((p[1]!.GetValue<double>()+(move?dy:0))*10000))));
  var inter=Clipper.Intersect(new Paths64{Path("a_original",false)},new Paths64{Path("b_original",true)},FillRule.NonZero);
  bool Remains(double depth) { var e=Clipper.InflatePaths(inter,-depth*10000/2,JoinType.Bevel,EndType.Polygon); return e.Count>0&&Math.Abs(Clipper.Area(e))>0; }
  double lo=0,hi=20;
  for(int k=0;k<24;k++) { var mid=(lo+hi)/2; if(Remains(mid))lo=mid;else hi=mid; }
  var cdinter=Clipper.Intersect(new Paths64{Path("a_cd",false)},new Paths64{Path("b_cd",true)},FillRule.NonZero);
  result.Add(new JsonObject { ["uttu_tolerance"]=row["uttu_tolerance"]!.DeepClone(),["overlap"]=row["overlap"]!.DeepClone(),["a"]=w["a"]!.DeepClone(),["b"]=w["b"]!.DeepClone(),["original_exceeds_1mm"]=Remains(1),["uttu_erosion_depth_mm"]=hi,["cd_intersection_area_mm2"]=Math.Abs(Clipper.Area(cdinter))/1e8 });
 }
 File.WriteAllText(Path.Combine(directory,"uttu-witness-verification.json"),result.ToJsonString());
}
}\n'''
(out/'Program.cs').write_text(program,encoding='utf-8')
(out/'provenance.json').write_text(json.dumps({'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'extracted_method_sha256':hashlib.sha256(method.encode()).hexdigest(),'input':str(source_input),'input_sha256':hashlib.sha256(source_input.read_bytes()).hexdigest(),'original_items':len(doc['items'])-1,'synthetic_rectangle_id':doc['items'][-1]['id'],'clipper_version':'2.0.0','scope':'Unbuffered input polygons; orientation 0; current Swarm PolySimpl 0.01; hypothetical Uttu preprocessing. No product settings changed.'},indent=2),encoding='utf-8')
print(out)
