"""Independent GEOS audit of Uttu preprocessing and actual Swarm CD contours.

The pair-contact sweep is a counterexample search, not a nesting benchmark or
proof over all rotations/translations. Depth is twice intersection inradius.
"""
import json, math, sys
from pathlib import Path
import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.affinity import translate

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'target/overlap-margin-audit'
doc = json.loads((OUT/'input.json').read_text())
original = {i['id']: Polygon(i['shape']['data']) for i in doc['items']}
assert all(p.is_valid and p.area > 0 for p in original.values())
ids = list(original)
synthetic = ids[-1]
real_ids = ids[:-1]
padding = []
for tol in ['-1','0','0.5','1','2','3','5']:
    processed = json.loads((OUT/f'uttu-{tol}.json').read_text())
    rows=[]
    distances=[]
    for item in processed['items'][:-1]:
        p=original[item['id']]
        s=Polygon(item['shape']['data'])
        assert s.is_valid
        points=shapely.points(np.asarray(shapely.segmentize(p.exterior,0.5).coords))
        d=shapely.distance(points,s.boundary)
        signed=np.where(shapely.covers(s,points),d,-d)
        distances.append(signed)
        rows.append({'id':item['id'],'vertices':len(item['shape']['data']),'max_padding_sampled':float(signed.max()),'min_padding_sampled':float(signed.min()),'outside_area':p.difference(s).area})
    ds=np.concatenate(distances)
    padding.append({'tolerance':tol,'vertices':sum(r['vertices'] for r in rows),'boundary_samples':len(ds),'fraction_padding_under_0_1':float(np.mean(ds<0.1)),'fraction_padding_at_least_2':float(np.mean(ds>=2)),'max_padding_sampled':float(ds.max()),'min_padding_sampled':float(ds.min()),'p50_padding':float(np.median(ds)),'p95_padding':float(np.percentile(ds,95)),'items':rows})
    print('padding',tol,padding[-1]['vertices'],padding[-1]['p95_padding'],flush=True)

(OUT/'padding.json').write_text(json.dumps(padding,indent=2),encoding='utf-8')
if '--padding-only' in sys.argv:
    raise SystemExit(0)

def extremum(poly, axis, high):
    pts=np.asarray(poly.exterior.coords)[:-1]
    value=float(pts[:,axis].max() if high else pts[:,axis].min())
    other=float(pts[np.abs(pts[:,axis]-value)<1e-5,1-axis].mean())
    return value,other

def depth(inter):
    if inter.is_empty or inter.area<1e-10:
        return 0.0
    # GEOS largest inscribed circle, diameter. Numeric tolerance is radius error.
    circle=shapely.maximum_inscribed_circle(inter,tolerance=0.0005)
    return 2*circle.length

results=[]
for case in json.loads((OUT/'swarm-geometry.json').read_text()):
    if 'error' in case:
        results.append(case)
        continue
    cd={i['id']:Polygon(i['cd']) for i in case['items']}
    invalid=[i for i,p in cd.items() if not p.is_valid or p.is_empty]
    worst=None
    violations=0
    checked=0
    depths=[]
    rectangle_depth=0
    # Both axis directions, deterministic cyclic real pairs. At these placements
    # CD bounding boxes are separated by 0.002mm: continuous CD non-overlap.
    pairs=[(i,real_ids[(n+1)%len(real_ids)]) for n,i in enumerate(real_ids)] + [(synthetic,synthetic)]
    for a,b in pairs:
        if a in invalid or b in invalid:
            continue
        for axis in (0,1):
            for forward in (True,False):
                av,at=extremum(cd[a],axis,forward)
                bv,bt=extremum(cd[b],axis,not forward)
                shift=[0.,0.]
                shift[axis]=av-bv+(0.002 if forward else -0.002)
                shift[1-axis]=at-bt
                moved_cd=translate(cd[b],*shift)
                assert not cd[a].intersects(moved_cd)
                inter=original[a].intersection(translate(original[b],*shift))
                value=depth(inter)
                if a==synthetic:
                    rectangle_depth=max(rectangle_depth,value)
                    continue
                checked+=1
                depths.append(value)
                violations+=value>1.002
                if worst is None or value>worst['depth']:
                    worst={'a':a,'b':b,'axis':axis,'forward':forward,'shift':shift,'depth':value,'intersection_area':inter.area,'a_cd':list(cd[a].exterior.coords),'b_cd':list(cd[b].exterior.coords),'a_original':list(original[a].exterior.coords),'b_original':list(original[b].exterior.coords)}
    result={'uttu_tolerance':case['uttu_tolerance'],'overlap':case['overlap'],'invalid_cd_ids':invalid,'checked_real_contacts':checked,'violations_over_1mm_with_0_002_numeric_margin':violations,'max_depth':max(depths,default=0),'p95_depth':float(np.percentile(depths,95)) if depths else None,'rectangle_depth':rectangle_depth,'worst':worst}
    results.append(result)
    print('contacts',result['uttu_tolerance'],result['overlap'],'max',round(result['max_depth'],4),'violations',violations,'rectangle',rectangle_depth,flush=True)

report={'shapely_version':shapely.__version__,'geos_version':shapely.geos_version_string,'metric':'2 * maximum inscribed circle radius of original polygon intersection; circle radius tolerance 0.0005mm; violation reporting guard 0.002mm','scope':'156 original input polygons, no added Buffer, orientation 0, cyclic pair extremal alignment contacts, CD bounding-box clearance 0.002mm. Test-only hypothetical Uttu preprocessing followed by actual Swarm importer PolySimpl=0.01, narrow-concavity closure disabled. Not an exhaustive maximum or full nesting benchmark.','padding':padding,'contacts':results}
(OUT/'analysis.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
