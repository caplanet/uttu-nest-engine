"""Independent GEOS audit of the public transforms saved by the trial."""
from pathlib import Path
import json,math
import shapely
from shapely.geometry import Polygon
from shapely.affinity import affine_transform
from shapely.strtree import STRtree
root=Path(__file__).resolve().parents[1]
trial=Path((root/'target/original-overlap-trial-current.txt').read_text())
input_doc=json.loads((root/'data/input/wsjk156.json').read_text(encoding='utf-8-sig'))
original={i['id']:Polygon(i['shape']['data']) for i in input_doc['items']}
report=[]
for path in sorted(trial.glob('shared-wsjk-seed*.json')):
    doc=json.loads(path.read_text());budget=doc['overlap']
    for row in doc['rows']:
        geometries=[]
        for p in row['after']['poses']:
            s,c=math.sin(p['rotation']),math.cos(p['rotation']);x,y=p['translation']
            geometries.append(affine_transform(original[p['id']],[c,-s,s,c,x,y]))
        tree=STRtree(geometries);violations=[];max_depth=0.;overlapping=0
        for i,a in enumerate(geometries):
            for j in tree.query(a):
                if j<=i:continue
                common=a.intersection(geometries[j])
                if common.is_empty or common.area<=1e-10:continue
                overlapping+=1
                depth=2*shapely.maximum_inscribed_circle(common,tolerance=.0002).length
                max_depth=max(max_depth,depth)
                if not common.buffer(-budget/2,join_style='bevel').is_empty:
                    violations.append({'a':i,'b':int(j),'depth_mm':depth})
        report.append({'file':path.name,'stage':row['stage'],'allowance_mm':budget,'original_pair_overlaps':overlapping,'max_depth_mm':max_depth,'over_budget_pairs':violations,'piece_count':len(geometries),'repair_status':row['repair_status']})
        print(path.name,row['stage'],'max depth',round(max_depth,6),'violations',len(violations),flush=True)
(trial/'independent-original-audit.json').write_text(json.dumps({'shapely':shapely.__version__,'metric':'Original input polygon intersection, half-budget bevel erosion. Max-depth figure is independently sampled maximum inscribed circle diameter.','results':report},indent=2),encoding='utf-8')
assert all(not row['over_budget_pairs'] for row in report)
