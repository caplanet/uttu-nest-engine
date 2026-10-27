"""Independent GEOS checks of all exported phase layouts; never repairs outputs."""
from pathlib import Path
import json,math,hashlib
import shapely
from shapely.geometry import Polygon,LineString,Point,GeometryCollection
from shapely.affinity import affine_transform
from shapely.strtree import STRtree
root=Path(__file__).resolve().parents[1]
trial=Path((root/'target/selective-overlap-current.txt').read_text())
report=[]
for path in sorted([*trial.glob('wsjk156-*.json'),*trial.glob('trousers64-*.json')]):
    doc=json.loads(path.read_text());local={}
    for g in doc['geometry']:
        points=g['original'];n=len(points);p=g['protection']
        features=[LineString([points[i],points[(i+1)%n]]) for i in p['edges']]+[Point(points[i]) for i in p['vertices']]
        local[g['id']]=(Polygon(points),GeometryCollection(features))
    for row in doc['rows']:
        layout=row['layout'];polys=[];features=[];outside=[]
        assert len(layout['poses'])==sum(g['quantity'] for g in doc['geometry'])
        for i,p in enumerate(layout['poses']):
            s,c=math.sin(p['rotation']),math.cos(p['rotation']);x,y=p['translation'];tr=[c,-s,s,c,x,y]
            original,protected=local[p['id']]
            a=affine_transform(original,tr);polys.append(a);features.append(affine_transform(protected,tr))
            x0,y0,x1,y1=a.bounds
            excess=max(0,-x0,-y0,x1-layout['width'],y1-doc['height'])
            if excess>.0006:outside.append(dict(index=i,excess_mm=excess))
        tree=STRtree(polys);budget_bad=[];protected_bad=[];overlaps=0;max_depth=0
        for i,a in enumerate(polys):
            for j in tree.query(a):
                if j<=i:continue
                b=polys[j];common=a.intersection(b)
                if common.area<=1e-10:continue
                overlaps+=1
                depth=2*shapely.maximum_inscribed_circle(common,tolerance=.0002).length
                max_depth=max(max_depth,depth)
                if not common.buffer(-doc['overlap_mm']/2,join_style='bevel').is_empty:
                    budget_bad.append(dict(a=i,b=int(j),depth_mm=depth))
                # Test actual protected segments/vertices against the OTHER
                # polygon interior, independently of Rust's clipped bands.
                # 0.0002 mm removes integer-grid/contact roundoff only.
                if features[i].intersects(b.buffer(-.0002)) or features[j].intersects(a.buffer(-.0002)):
                    protected_bad.append(dict(a=i,b=int(j),depth_mm=depth))
        result=dict(file=path.name,stage=row['stage'],pieces=len(polys),original_overlap_pairs=overlaps,max_depth_mm=max_depth,over_budget=budget_bad,protected_intrusions=protected_bad,outside=outside)
        report.append(result)
        print(path.name,row['stage'],'overlaps',overlaps,'depth',round(max_depth,6),'violations',len(budget_bad)+len(protected_bad)+len(outside),flush=True)
(trial/'independent-selective-audit.json').write_text(json.dumps(dict(shapely=shapely.__version__,coordinate_tolerance_mm=.0002,strip_tolerance_mm=.0006,results=report),indent=2))
assert len(report)==96,(len(report),'expected 24 runs x 4 stages')
assert all(not(r['over_budget'] or r['protected_intrusions'] or r['outside']) for r in report)
