"""Summarize completed runs; missing or uncertified runs are errors."""
import hashlib
import json
import sys
from pathlib import Path
from statistics import mean

sys.stdout.reconfigure(encoding='utf-8')

repo=Path(__file__).resolve().parents[1]
root=repo/'target/separator-variants-trial'
manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8-sig'))
seeds=manifest['seeds']
modes=manifest['modes']
cases=[c['name'] for c in manifest['cases']]
names={'sync':'기존 동기식','noop_empty':'A: 동일 이동·빈 작업 생략',
       'preparation':'B: 준비 목록 공유','shared':'C: 완료 시 공통 최선 배치 인계',
       'kernel':'D: 변 검사 순서 재사용','async':'독립 실행 대조군'}
records={}
for case in cases:
    for mode in modes:
        for seed in seeds:
            f=root/f'{case}-{mode}-{seed}.json'
            j=json.loads(f.read_text(encoding='utf-8'))
            assert j['certified'] and j['mode']==mode and j['seed']==seed
            assert j['budget_seconds']==manifest['seconds']
            assert j['width']<=j['initial_width']
            assert all(a['width']>b['width'] for a,b in zip(j['trace'],j['trace'][1:]))
            assert j['width']==j['trace'][-1]['width']
            if mode=='preparation':
                assert j['trial_counters']['prepared_rounds']==sum(l['metrics']['worker_rounds'] for l in j['lanes'])
            if mode=='noop_empty':
                assert j['trial_counters']['skipped_empty_rounds']>0
            records[case,mode,seed]=j
summary=[]
for case in cases:
    for mode in modes:
        runs=[records[case,mode,s] for s in seeds]
        baseline=[records[case,'sync',s] for s in seeds]
        delta=[r['width']-b['width'] for r,b in zip(runs,baseline)]
        target=0.78 if case=='wsjk' else 0.836
        times=[next((t['received_at'] for t in r['trace'] if t['density']>=target),None) for r in runs]
        summary.append({'case':case,'mode':mode,'name':names[mode],
            'mean_width':mean(r['width'] for r in runs),'mean_density_percent':100*mean(r['density'] for r in runs),
            'mean_delta_mm':mean(delta),'mean_delta_density_pp':100*mean(r['density']-b['density'] for r,b in zip(runs,baseline)),
            'wins':sum(d < -0.001 for d in delta),'ties':sum(abs(d)<=0.001 for d in delta),
            'widths':[r['width'] for r in runs],'deltas':delta,'target_density':target,'target_seconds':times,
            'counters':[r['trial_counters'] for r in runs],
            'rounds':[sum(l['metrics']['worker_rounds'] for l in r['lanes']) for r in runs]})
hashes=json.loads((root/'production-source-hashes.json').read_text(encoding='utf-8'))
changed=[p for p,h in hashes.items() if hashlib.sha256((repo/p).read_bytes()).hexdigest()!=h]
assert not changed,('Production source changed during experiments',changed)
result={'manifest':manifest,'summary':summary,'runs':len(records),'all_certified':True,
        'elapsed_min':min(r['elapsed_seconds'] for r in records.values()),
        'elapsed_max':max(r['elapsed_seconds'] for r in records.values()),
        'cold_validation_min':min(r['cold_validation_seconds'] for r in records.values()),
        'cold_validation_max':max(r['cold_validation_seconds'] for r in records.values()),
        'production_files_checked':len(hashes),'production_unchanged':True}
(root/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
lines=['| 입력 | 방식 | 평균 길이(mm) | 기존 대비(mm) | 평균 수율 | 기존보다 짧음 / 동일 |',
       '|---|---|---:|---:|---:|---:|']
for s in summary:
    lines.append(f"| {s['case']} | {s['name']} | {s['mean_width']:,.3f} | {s['mean_delta_mm']:+.3f} | {s['mean_density_percent']:.3f}% | {s['wins']}/{len(seeds)} · {s['ties']}/{len(seeds)} |")
lines+=['','| 입력 | 방식 | seed '+ ' / '.join(map(str,seeds))+' 길이(mm) | 목표 수율 도달 시간(s) |',
        '|---|---|---|---|']
for s in summary:
    widths=' / '.join(f'{v:,.3f}' for v in s['widths'])
    times=' / '.join('미도달' if t is None else f'{t:.3f}' for t in s['target_seconds'])
    lines.append(f"| {s['case']} | {s['name']} | {widths} | {times} |")
(root/'tables.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print('\n'.join(lines))
print(json.dumps({k:v for k,v in result.items() if k not in ('manifest','summary')},indent=2))
