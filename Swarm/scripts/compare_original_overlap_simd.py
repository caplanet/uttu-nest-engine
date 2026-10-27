"""Interleave baseline / pre-SIMD guard / SIMD guard on frozen identical layouts."""
from pathlib import Path
import subprocess,json,statistics,hashlib
root=Path(__file__).resolve().parents[1]
trial=Path((root/'target/original-overlap-trial-current.txt').read_text())
records=[]
for run,labels in enumerate([['baseline','pre-simd','candidate'],['candidate','pre-simd','baseline']]):
    for label in labels:
        name=f'simd-comparison-{run}-{label}'
        exe=trial/f'{label}-validation.exe'
        with (trial/f'{name}.log').open('w',encoding='utf-8') as log:
            subprocess.run([str(exe),str(trial/'certified-validation-input'),str(trial/f'{name}.json')],cwd=root,stdout=log,stderr=subprocess.STDOUT,check=True)
        rows=json.loads((trial/f'{name}.json').read_text())
        assert all(r['accepted']==r['calls'] for r in rows)
        records.append(dict(run=run,label=label,exe_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),rows=rows))
        print(name,[(r['overlap'],r['mode'],round(r['median_seconds']*1000,3)) for r in rows],flush=True)
summary=[]
for overlap in (1,5):
    for mode in ('unchanged','changing'):
        values={label:statistics.median([time*1000 for record in records if record['label']==label for row in record['rows'] if row['overlap']==overlap and row['mode']==mode for time in row['rounds_seconds_per_call']]) for label in ('baseline','pre-simd','candidate')}
        summary.append(dict(overlap=overlap,mode=mode,ms=values,candidate_vs_baseline=values['candidate']/values['baseline'],simd_change_percent=(values['candidate']/values['pre-simd']-1)*100,calls_per_build=360))
(trial/'simd-interleaved-comparison.json').write_text(json.dumps(dict(records=records,summary=summary),indent=2),encoding='utf-8')
print(json.dumps(summary,indent=2))
