import copy,hashlib,json,math,subprocess,time
from pathlib import Path
import shapely
from shapely.geometry import Polygon
from shapely.affinity import affine_transform
from shapely.strtree import STRtree
root=Path(__file__).resolve().parents[2];w=root/'target/overlap-reduction-trial'
source=json.loads((w/'input.json').read_text(encoding='utf8'));items={r['id']:r for r in source['items']}
before=json.loads((w/'before.json').read_text(encoding='utf8'));result=json.loads((w/'local22-result.json').read_text(encoding='utf8'))
def audit(name):
 doc=json.loads((w/name).read_text(encoding='utf8'));geoms=[];rows=doc['layout']['placed_items'];brows={r['item_id']:r for r in before['layout']['placed_items']}
 check=copy.deepcopy(doc)
 assert len(rows)==len(items)==len({r['item_id'] for r in rows})
 for i,r in enumerate(rows):
  b=brows[r['item_id']];item=items[r['item_id']]
  assert r['OrgID']==item['OrgID'] and r['OrgTransform']==item['OrgTransform']
  assert r['transformation']['rotation']==b['transformation']['rotation']
  check['layout']['placed_items'][i]['transformation']['translation']=b['transformation']['translation']
  t=r['transformation'];angle=math.radians(t['rotation']);c,s=math.cos(angle),math.sin(angle);x,y=t['translation'];m=item['OrgTransform']['margin']
  geoms.append(affine_transform(Polygon(item['shape']['data']),[c,-s,s,c,x+m,y+m]))
  previous=affine_transform(Polygon(item['shape']['data']),[c,-s,s,c,*[v+m for v in b['transformation']['translation']]])
  q=geoms[-1].bounds;old=previous.bounds
  assert q[0]>=min(0.,old[0])-1e-6 and q[1]>=min(0.,old[1])-1e-6 and q[2]<=max(before['strip_width'],old[2])+1e-6 and q[3]<=max(source['strip_height'],old[3])+1e-6, ('original boundary worsened',r['OrgID'],q,old)
 assert check==before, 'Fields other than translations changed'
 tree=STRtree(geoms);pairs=[];counts={str(t):0 for t in [1,1.5,2,2.2,3]}
 for i,g in enumerate(geoms):
  for jj in tree.query(g):
   j=int(jj)
   if j<=i:continue
   common=g.intersection(geoms[j])
   if common.area<=1e-8:continue
   d=2*shapely.maximum_inscribed_circle(common,tolerance=.0001).length
   pairs.append({'org_ids':[rows[i]['OrgID'],rows[j]['OrgID']],'estimated_depth_mm':d,'area_mm2':common.area})
   for threshold in counts:
    counts[threshold]+=int(not common.buffer(-float(threshold)/2,join_style='bevel').is_empty)
 return {'file':name,'sha256':hashlib.sha256((w/name).read_bytes()).hexdigest(),'max_estimated_depth_mm':max(p['estimated_depth_mm'] for p in pairs),'total_overlap_area_mm2':sum(p['area_mm2'] for p in pairs),'overlapping_pairs':len(pairs),'pairs_above_threshold_bevel_erosion':counts,'strip_width_mm':doc['strip_width'],'density':doc['density'],'max_translation_mm':max(math.dist(r['transformation']['translation'],brows[r['item_id']]['transformation']['translation']) for r in rows),'changed_pieces':sum(r['transformation']['translation']!=brows[r['item_id']]['transformation']['translation'] for r in rows),'original_world_bounds':[min(g.bounds[0] for g in geoms),min(g.bounds[1] for g in geoms),max(g.bounds[2] for g in geoms),max(g.bounds[3] for g in geoms)],'top_pairs':sorted(pairs,key=lambda p:p['estimated_depth_mm'],reverse=True)[:10]}
metrics=[audit(f) for f in ['before.json','coupled-seed22.json','local22-result.json']]
assert metrics[-1]['max_estimated_depth_mm']<2.2
assert metrics[-1]['pairs_above_threshold_bevel_erosion']['2.2']==0
checks=[]
def native_suite(constraints,mutations):
 path=w/'verification-constraints.json';path.write_text(json.dumps(constraints),encoding='utf8')
 p=subprocess.Popen([str(root/'target/release/examples/overlap_trial_native.exe'),str(w/'input.json'),str(w/'local22-result.json'),str(path),'2.2'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,encoding='utf8')
 ready=json.loads(p.stdout.readline());assert ready['baseline_certified'],ready
 try:
  for name,doc,expected,message in mutations:
   p.stdin.write(json.dumps(doc)+'\n');p.stdin.flush();response=json.loads(p.stdout.readline());assert response['valid']==expected,(name,response)
   if message:assert message in response['error'],response
   checks.append({'test':name,**response})
 finally:p.stdin.close();p.wait()
mutations=[('saved JSON fresh native certification',result,True,None)]
for name,field,value,needle in [('width changed','strip_width',result['strip_width']+.01,'strip changed')]:
 d=copy.deepcopy(result);d[field]=value;mutations.append((name,d,False,needle))
d=copy.deepcopy(result);d['layout']['placed_items'][0]['transformation']['rotation']+=90;mutations.append(('rotation changed',d,False,'rotation changed'))
d=copy.deepcopy(result);d['layout']['placed_items'][0]['transformation']['translation'][0]+=10000;mutations.append(('outside strip',d,False,'LayoutInfeasible'))
d=copy.deepcopy(result);d['layout']['placed_items'].pop();mutations.append(('missing piece',d,False,'placement count mismatch'))
d=copy.deepcopy(result);d['layout']['placed_items'][0]['item_id']=d['layout']['placed_items'][1]['item_id'];mutations.append(('duplicate piece',d,False,'unknown or duplicate item'))
native_suite({'frozen_item_ids':[],'place_groups':[]},mutations)
first=result['layout']['placed_items'][0]['item_id'];second=result['layout']['placed_items'][1]['item_id'];d=copy.deepcopy(result);d['layout']['placed_items'][0]['transformation']['translation'][0]+=.1
native_suite({'frozen_item_ids':[first],'place_groups':[]},[('frozen unchanged',result,True,None),('frozen moved',d,False,'frozen pose changed')])
native_suite({'frozen_item_ids':[],'place_groups':[{'id':1,'item_ids':[first,second]}]},[('group unchanged',result,True,None),('group relative pose changed',d,False,'group relative pose changed')])
report={'piece_overlap_mm':2.2,'poly_simpl':.01,'metrics':metrics,'native_checks':checks,'metadata_preserved':True,'scope':'Original Swarm INPUT contours; Uttu.Nest source and UI were not accessed. Depth is estimated inscribed-circle diameter, with separate bevel-erosion thresholds and production native certification.'}
(w/'verification22.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(report,ensure_ascii=True,indent=2))
