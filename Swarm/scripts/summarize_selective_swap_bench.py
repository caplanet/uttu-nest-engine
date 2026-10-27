from pathlib import Path
import json,statistics,hashlib
root=Path(__file__).resolve().parents[1];trial=Path((root/'target/selective-overlap-current.txt').read_text())
docs={p.stem:json.loads(p.read_text()) for p in [*trial.glob('wsjk156-*.json'),*trial.glob('trousers64-*.json')]}
assert len(docs)==24
results=[];comparisons=[]
for case in ['wsjk156','trousers64']:
    for seed in [17,31,73]:
        group={mode:docs[f'{case}-{seed}-{mode}'] for mode in ['off','disrupt','separator','both']}
        base=group['off']['rows'][-1]['layout'];initial=group['off']['rows'][0]['layout']
        assert all(d['rows'][0]['layout']==initial for d in group.values()),'Starting layouts differ'
        for mode,d in group.items():
            final=d['rows'][-1]['layout']
            results.append(dict(case=case,seed=seed,mode=mode,initial_width=initial['width'],width=final['width'],density_percent=final['density']*100,width_reduction_vs_off_mm=base['width']-final['width'],width_reduction_vs_off_percent=(base['width']-final['width'])/base['width']*100,yield_gain_vs_off_pp=(final['density']-base['density'])*100,counts=d['counts'],phase_seconds={r['stage']:r['seconds'] for r in d['rows']},original_vertices=sum(len(g['original']) for g in d['geometry']),search_vertices=sum(len(g['search']) for g in d['geometry']),protected_edges=sum(len(g['protection']['edges']) for g in d['geometry']),protected_vertices=sum(len(g['protection']['vertices']) for g in d['geometry'])))
    for mode in ['disrupt','separator','both']:
        rs=[r for r in results if r['case']==case and r['mode']==mode]
        comparisons.append(dict(case=case,mode=mode,median_width_reduction_percent=statistics.median(r['width_reduction_vs_off_percent'] for r in rs),median_yield_gain_pp=statistics.median(r['yield_gain_vs_off_pp'] for r in rs),better=sum(r['width_reduction_vs_off_mm']>0 for r in rs),worse=sum(r['width_reduction_vs_off_mm']<0 for r in rs),counts={k:sum(r['counts'][k] for r in rs) for k in rs[0]['counts']}))
summary=dict(method='Same initial layouts and seeds, four ablation modes, equal 8/4/2 second phase budgets; time-based comparisons retain scheduling noise. No-op disruption runs cannot establish disruption benefit.',comparisons=comparisons,runs=results)
(trial/'swap-summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(comparisons,indent=2))
