"""Experimental local linearized collision separation; export only certified poses."""
import copy,json,math,subprocess,time,struct
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
from shapely.geometry import Polygon,box
from shapely.affinity import translate
from shapely.strtree import STRtree
root=Path(__file__).resolve().parents[2];w=root/'target/overlap-reduction-trial';start=time.perf_counter()
before=json.loads((w/'before.json').read_text(encoding='utf8'));doc=json.loads((w/'trial90-result.json').read_text(encoding='utf8'));rows=doc['layout']['placed_items'];base={p['item_id']:p for p in before['layout']['placed_items']}
p=subprocess.Popen([str(root/'target/release/examples/overlap_trial_native.exe'),str(w/'input.json'),str(w/'before.json'),str(w/'constraints.json'),'2.2'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,encoding='utf8')
ready=json.loads(p.stdout.readline());raw={g['item_id']:Polygon(g['points']) for g in ready['search_polygons']}
origins=np.array([base[r['item_id']]['transformation']['translation'] for r in rows]);delta=np.array([r['transformation']['translation'] for r in rows])-origins
polys=[raw[r['item_id']].buffer(.003,join_style='mitre') for r in rows];bbox=ready['search_bounds'];n=len(rows);history=[];calls=0;success=False
source=json.loads((w/'input.json').read_text(encoding='utf8'));items={i['id']:i for i in source['items']}
original_bounds=[]
for r,origin in zip(rows,origins):
 assert r['transformation']['rotation']==0, 'This fixture preparation supports zero relative rotations only'
 original_bounds.append(translate(Polygon(items[r['item_id']]['shape']['data']),*origin).bounds)
f32=lambda x:struct.unpack('f',struct.pack('f',x))[0]
try:
 for iteration in range(120):
  geoms=[translate(g,*d) for g,d in zip(polys,delta)];tree=STRtree(geoms);pairs=[]
  for i,g in enumerate(geoms):
   for jj in tree.query(g):
    j=int(jj)
    if j>i:
     a=g.intersection(geoms[j]).area
     if a>1e-9:pairs.append((i,j,a))
  total=sum(a for _,_,a in pairs)+sum(g.difference(box(*bbox)).area for g in geoms)
  print('iteration',iteration,'loss',total,'pairs',len(pairs),'max_move',max(np.linalg.norm(delta,axis=1)),flush=True)
  history.append({'iteration':iteration,'loss':total,'seconds':time.perf_counter()-start})
  if total<.0001:
   candidate=copy.deepcopy(doc)
   for i,r in enumerate(candidate['layout']['placed_items']):r['transformation']['translation']=[f32(v) for v in origins[i]+delta[i]]
   p.stdin.write(json.dumps(candidate)+'\n');p.stdin.flush();result=json.loads(p.stdout.readline());calls+=1;print(result,flush=True)
   if result['valid']:
    (w/'coupled-seed22.json').write_text(json.dumps(candidate,ensure_ascii=False,indent=2),encoding='utf8');success=True;break
  # Linearized pair constraints move both contacting pieces. A shared movement
  # bound minimizes each iteration's largest coordinate change.
  # Nonnegative pair slacks keep a tightly packed contact system solvable;
  # they are penalized, never treated as a certified final state.
  size=4*n+1+len(pairs);A=[];B=[]
  for i,j,area in pairs:
   g,h=geoms[i],geoms[j];eps=.001
   grad=np.array([(translate(g,eps,0).intersection(h).area-translate(g,-eps,0).intersection(h).area)/(2*eps),(translate(g,0,eps).intersection(h).area-translate(g,0,-eps).intersection(h).area)/(2*eps)])
   norm=np.linalg.norm(grad)
   if norm<1e-8:raise RuntimeError('Flat collision gradient')
   r=np.zeros(size);r[2*i:2*i+2]=grad/norm;r[2*j:2*j+2]=-grad/norm;r[4*n+1+len(A)]=-1;A.append(r);B.append(-area/norm-.0002)
  bounds=[]
  for i,g in enumerate(geoms):
   b=g.bounds
   o=original_bounds[i];reserve=.001
   # Also prevent the original contour from extending farther outside the
   # declared strip than this same piece did in the supplied baseline.
   limits=(min(0.,o[0]),min(0.,o[1]),max(before['strip_width'],o[2]),max(source['strip_height'],o[3]))
   bounds.extend([(max(-3.,-5.-delta[i,0],bbox[0]-b[0],limits[0]-o[0]-delta[i,0]+reserve),min(3.,5.-delta[i,0],bbox[2]-b[2],limits[2]-o[2]-delta[i,0]-reserve)),(max(-3.,-5.-delta[i,1],bbox[1]-b[1],limits[1]-o[1]-delta[i,1]+reserve),min(3.,5.-delta[i,1],bbox[3]-b[3],limits[3]-o[3]-delta[i,1]-reserve))])
  for k in range(2*n):
   for sign in [-1,1]:
    r=np.zeros(size);r[k]=sign;r[2*n]=-1;A.append(r);B.append(0.)
    r=np.zeros(size);r[k]=sign;r[2*n+1+k]=-1;A.append(r);B.append(0.)
  bounds.append((0.,3.));bounds.extend([(0.,None)]*(2*n+len(pairs)));objective=np.zeros(size);objective[2*n]=1;objective[2*n+1:4*n+1]=.001;objective[4*n+1:]=1000
  assert all(hi is None or lo<=hi+1e-9 for lo,hi in bounds), [(rows[k//2]['OrgID'],k%2,b) for k,b in enumerate(bounds[:2*n]) if b[0]>b[1]]
  fit=linprog(objective,A_ub=np.array(A),b_ub=np.array(B),bounds=bounds,method='highs')
  if not fit.success:raise RuntimeError(fit.message)
  delta+=fit.x[:2*n].reshape(n,2)
  if time.perf_counter()-start>90:raise RuntimeError('Time limit')
 if not success:raise RuntimeError('No certified candidate')
finally:
 (w/'coupled-seed22-summary.json').write_text(json.dumps({'history':history,'native_calls':calls,'seconds':time.perf_counter()-start,'max_translation_mm':float(max(np.linalg.norm(delta,axis=1))),'certified':success},indent=2),encoding='utf8')
 p.stdin.close();p.wait()


