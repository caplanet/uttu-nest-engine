"""Experimental postprocessing, separate from the production phase pipeline.
Every checkpoint passes production Rust certification before adoption.
"""
import argparse,copy,json,math,os,signal,struct,subprocess,time
from pathlib import Path
import shapely
from shapely.geometry import Polygon
from shapely.affinity import affine_transform,translate
from shapely.strtree import STRtree

parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=90);parser.add_argument('--start',default='before.json');parser.add_argument('--tag',default='trial');parser.add_argument('--stop-after-accepted',type=int,default=0);parser.add_argument('--overlap',type=float,default=2.2);parser.add_argument('--stop-before-adoption',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parents[2];work=root/'target/overlap-reduction-trial';started=time.perf_counter();stop=False
signal.signal(signal.SIGINT,lambda *_:globals().__setitem__('stop',True))
source=json.loads((work/'input.json').read_text(encoding='utf-8-sig'));baseline=json.loads((work/'before.json').read_text(encoding='utf-8-sig'));current=json.loads((work/args.start).read_text(encoding='utf-8-sig'));constraints=json.loads((work/'constraints.json').read_text())
items={p['id']:p for p in source['items']};before={p['item_id']:p for p in baseline['layout']['placed_items']};rows=current['layout']['placed_items'];index={p['item_id']:i for i,p in enumerate(rows)}
assert len(index)==len(rows)==len(items)
assert not source.get('place_groups') and not any('placed_hint' in p for p in source['items']), 'Explicit constraint extraction is required for hinted/grouped input.'
for p in rows:
 assert p['OrgTransform']==items[p['item_id']]['OrgTransform'] and p['OrgID']==items[p['item_id']]['OrgID']
 assert p['transformation']['rotation']==before[p['item_id']]['transformation']['rotation']
assert current['strip_width']==baseline['strip_width']
logfile=(work/(args.tag+'-native-stderr.log')).open('w',encoding='utf-8')
proc=subprocess.Popen([str(root/'target/release/examples/overlap_trial_native.exe'),str(work/'input.json'),str(work/'before.json'),str(work/'constraints.json'),str(args.overlap)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=logfile,text=True,encoding='utf-8',bufsize=1)
ready=json.loads(proc.stdout.readline());assert ready['ready'],ready
(work/(args.tag+'-native-ready.json')).write_text(json.dumps(ready),encoding='utf-8')
native_calls=0;native_seconds=0.;native_rejections=[]
def native(doc):
 global native_calls,native_seconds
 proc.stdin.write(json.dumps(doc,separators=(',',':'))+'\n');proc.stdin.flush();line=proc.stdout.readline();assert line,'native helper exited'
 response=json.loads(line);native_calls+=1;native_seconds+=response['seconds']
 if not response['valid']:native_rejections.append(response['error'])
 return response['valid']
assert native(current)
f32=lambda x:struct.unpack('f',struct.pack('f',x))[0]
pos=[tuple(p['transformation']['translation']) for p in rows];initial_pos=[tuple(before[p['item_id']]['transformation']['translation']) for p in rows]
base_polys=[];search_baseline={p['item_id']:Polygon(p['points']) for p in ready['search_polygons']}
for p in rows:
 pose=p['transformation'];c,s=math.cos(math.radians(pose['rotation'])),math.sin(math.radians(pose['rotation']));x,y=pose['translation']
 base_polys.append(affine_transform(Polygon(items[p['item_id']]['shape']['data']),[c,-s,s,c,x,y]))
geoms=base_polys[:]
search=[translate(search_baseline[p['item_id']],pos[i][0]-initial_pos[i][0],pos[i][1]-initial_pos[i][1]) for i,p in enumerate(rows)]
# Do not make any existing original-boundary overhang worse.
original_limits=[]
for i,p in enumerate(rows):
 b=translate(geoms[i],initial_pos[i][0]-pos[i][0],initial_pos[i][1]-pos[i][1]).bounds
 original_limits.append((min(0.,b[0]),min(0.,b[1]),max(baseline['strip_width'],b[2]),max(source['strip_height'],b[3])))
container=ready['search_bounds'];frozen={index[i] for i,v in enumerate(ready['frozen_mask']) if v}
units=[];assigned=set()
for group in constraints['place_groups']:
 unit=tuple(index[i] for i in group['item_ids']);assigned.update(unit)
 if not frozen.intersection(unit):units.append(unit)
units.extend((i,) for i in range(len(rows)) if i not in assigned and i not in frozen)
def pair_measure(a,b):
 common=a.intersection(b)
 if common.is_empty or common.area<=1e-8:return (0.,0,0.,0.)
 depth=2*shapely.maximum_inscribed_circle(common,tolerance=.0005).length
 excess=common.buffer(-1.,join_style='bevel')
 return (depth,int(not excess.is_empty),excess.area,common.area)
def allpairs():
 tree=STRtree(geoms);values={}
 for i,a in enumerate(geoms):
  for jj in tree.query(a):
   j=int(jj)
   if i<j:
    m=pair_measure(a,geoms[j])
    if m[0]>0:values[i,j]=m
 return values
pairs=allpairs()
def score(values):
 vals=list(values.values());return (max((v[0] for v in vals),default=0.),sum(v[1] for v in vals),sum(v[2] for v in vals),sum(v[3] for v in vals))
def better(a,b):
 if a[0] < b[0]-.0005:return True
 if a[0] > b[0]+1e-9:return False
 if a[1]!=b[1]:return a[1]<b[1]
 if a[2]<b[2]-1e-7:return True
 if a[2]>b[2]+1e-7:return False
 return a[3]<b[3]-1e-7
initial_score=score(pairs);current_score=initial_score
history=[];attempts=0;geom_rejections=0;start_opt=time.perf_counter();output_path=work/(args.tag+'-result.json')
def checkpoint():
 tmp=output_path.with_suffix('.tmp');tmp.write_text(json.dumps(current,ensure_ascii=False,indent=2),encoding='utf-8');os.replace(tmp,output_path)
checkpoint()
reason='time_limit';round_index=0
try:
 while not stop and time.perf_counter()-start_opt<args.seconds:
  accepted_this_round=0;round_index+=1
  priorities={i:max((v[0] for k,v in pairs.items() if i in k),default=0.) for i in range(len(rows))}
  units.sort(key=lambda u:max(priorities[i] for i in u),reverse=True)
  for unit in units:
   if stop or time.perf_counter()-start_opt>=args.seconds:break
   if max(priorities[i] for i in unit)<.01:continue
   tree=STRtree(geoms);search_tree=STRtree(search);touching={k for k in pairs if any(i in k for i in unit)}
   unit_set=set(unit);unaffected={k:v for k,v in pairs.items() if k not in touching};candidate_list=[]
   # Additional translations do not alter any rotation or group-relative offset.
   steps=([1.,.5,.25,.1] if round_index<3 else [.5,.25,.1,.05])
   for distance in steps:
    for direction in range(16):
     if stop or time.perf_counter()-start_opt>=args.seconds:break
     dx=distance*math.cos(direction*math.pi/8);dy=distance*math.sin(direction*math.pi/8);attempts+=1
     trial_pos={i:(f32(pos[i][0]+dx),f32(pos[i][1]+dy)) for i in unit}
     if any(math.dist(trial_pos[i],initial_pos[i])>5.0 for i in unit):continue
     tg={i:translate(geoms[i],trial_pos[i][0]-pos[i][0],trial_pos[i][1]-pos[i][1]) for i in unit}
     ts={i:translate(search[i],trial_pos[i][0]-pos[i][0],trial_pos[i][1]-pos[i][1]) for i in unit}
     valid=True
     for i in unit:
      b=tg[i].bounds;l=original_limits[i]
      if b[0]<l[0]-1e-8 or b[1]<l[1]-1e-8 or b[2]>l[2]+1e-8 or b[3]>l[3]+1e-8:valid=False;break
      b=ts[i].bounds
      if b[0]<container[0]-.0001 or b[1]<container[1]-.0001 or b[2]>container[2]+.001 or b[3]>container[3]+.001:valid=False;break
      for jj in search_tree.query(ts[i]):
       j=int(jj)
       if j not in unit_set and ts[i].intersection(search[j]).area>1e-7:valid=False;break
      if not valid:break
     if not valid:geom_rejections+=1;continue
     neighbors=set(touching)
     for i in unit:
      neighbors.update(tuple(sorted((i,int(j)))) for j in tree.query(tg[i]) if int(j)!=i)
     changes={}
     for i,j in neighbors:
      m=pair_measure(tg.get(i,geoms[i]),tg.get(j,geoms[j]))
      if m[0]>args.overlap:valid=False;break
      if m[0]>0:changes[i,j]=m
     if not valid:geom_rejections+=1;continue
     values=unaffected|changes;s=score(values)
     if better(s,current_score):candidate_list.append((s,trial_pos,tg,ts,values))
   candidate_list.sort(key=lambda c:c[0])
   for s,tp,tg,ts,values in candidate_list:
    if stop or time.perf_counter()-start_opt>=args.seconds:break
    candidate=copy.deepcopy(current)
    for i in unit:candidate['layout']['placed_items'][i]['transformation']['translation']=list(tp[i])
    if not native(candidate):continue
    if args.stop_before_adoption:stop=True;reason='injected_stop_after_validation_before_adoption'
    # An explicit stop arriving during certification leaves the previous checkpoint.
    if stop:break
    current=candidate;current_score=s;pairs=values
    for i in unit:pos[i]=tp[i];geoms[i]=tg[i];search[i]=ts[i]
    accepted_this_round+=1
    history.append({'seconds':time.perf_counter()-start_opt,'round':round_index,'org_ids':[rows[i]['OrgID'] for i in unit],'positions':[list(pos[i]) for i in unit],'score':list(s)})
    checkpoint()
    if len(history)%10==0:print('accepted',len(history),'score',s,'seconds',round(time.perf_counter()-start_opt,2),flush=True)
    if args.stop_after_accepted and len(history)>=args.stop_after_accepted:stop=True;reason='injected_stop_after_certified_checkpoint'
    break
  if accepted_this_round==0 and not stop and time.perf_counter()-start_opt<args.seconds:reason='no_improving_single_unit_move';break
 if stop and reason=='time_limit':reason='external_stop'
 # Reparse exactly the saved file; certification must match the exported artifact.
 saved=json.loads(output_path.read_text(encoding='utf-8'));assert native(saved)
 final_score=score(allpairs());assert not better(initial_score,final_score), (initial_score,final_score)
 summary={'seconds_including_setup':time.perf_counter()-started,'optimization_seconds':time.perf_counter()-start_opt,'reason':reason,'initial_score':initial_score,'final_score':final_score,'score_fields':['max_estimated_depth_mm','pairs_above_2mm','excess_area_after_1mm_erosion','total_overlap_area_mm2'],'accepted':len(history),'trials':attempts,'geometry_rejections':geom_rejections,'native_calls':native_calls,'native_seconds':native_seconds,'native_rejections':native_rejections,'history':history,'output':str(output_path),'max_translation_mm':max(math.dist(a,b) for a,b in zip(pos,initial_pos)),'strip_width':current['strip_width'],'original_run_time_sec':baseline['run_time_sec'],'native_final_certified':True,'constraints':constraints}
 summary['piece_overlap_mm']=args.overlap;summary['poly_simpl']=.01
 (work/(args.tag+'-summary.json')).write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in summary.items() if k not in ['history','native_rejections']},ensure_ascii=True),flush=True)
finally:
 proc.stdin.close();proc.wait(timeout=20);logfile.close()
