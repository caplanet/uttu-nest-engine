"""Summarize measured artifacts without choosing adoption or rollback."""
from pathlib import Path
import json, statistics, hashlib, zipfile, difflib

root=Path(__file__).resolve().parents[1]
trial=Path((root/'target/original-overlap-trial-current.txt').read_text())
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
widths=[]; rows=[]
for overlap in (1,5):
    for seed in (17,31,73):
        base=read(trial/f'wsjk-seed{seed}-overlap{overlap}.json')
        candidate=read(trial/f'shared-wsjk-seed{seed}-overlap{overlap}.json')
        b=base['rows'][-1]['before']['width'];c=candidate['rows'][-1]['after']['width']
        widths.append(dict(overlap=overlap,seed=seed,baseline_width=b,candidate_width=c,change_percent=(c/b-1)*100))
        rows+=candidate['rows']
validation=[]
for dataset in ('','-certified'):
    base=read(trial/f'baseline{dataset}-validation.json')
    candidate=read(trial/f'candidate{dataset}-validation.json')
    for b,c in zip(base,candidate):
        assert (b['overlap'],b['mode'])==(c['overlap'],c['mode'])
        validation.append(dict(dataset='corrected' if dataset else 'old-layouts',overlap=b['overlap'],mode=b['mode'],baseline_ms=b['median_seconds']*1000,candidate_ms=c['median_seconds']*1000,ratio=c['median_seconds']/b['median_seconds'],baseline_accepted=b['accepted'],candidate_accepted=c['accepted'],calls=b['calls']))
audit=read(trial/'independent-original-audit.json')['results']
summary=dict(decision='pending user choice; no automatic rollback',widths=widths,validation=validation,
    original_check_ms={kind:dict(min=min(r[k]*1000 for r in rows),median=statistics.median(r[k]*1000 for r in rows),max=max(r[k]*1000 for r in rows)) for kind,k in [('cold','cold_check_seconds'),('cached','warm_check_seconds'),('repair','repair_seconds')]},
    audit=dict(layouts=len(audit),placements=sum(r['piece_count'] for r in audit),over_budget_pairs=sum(len(r['over_budget_pairs']) for r in audit),max_depth_by_allowance={str(a):max(r['max_depth_mm'] for r in audit if r['allowance_mm']==a) for a in (1,5)}),
    process={name:read(trial/f'{name}-validation-process.json') for name in ('baseline','candidate','baseline-certified','candidate-certified')},
    interleaved_simd=read(trial/'simd-interleaved-comparison.json')['summary'])
(trial/'comparison-summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')

# Source-only patch against the immutable checkpoint. No restore or deletion.
manifest=read(trial/'manifest.json')
manifest['user_decision_required']=True
manifest['gates']['correctness']='Original intersections must satisfy erosion by half of dynamic piece_overlap_mm; preserve frozen exceptions, groups and cancellation incumbents.'
manifest['gates']['scope']='Trial stays reviewable. User decides retain or rollback after the comparison; no automatic rollback and no Config/default changes.'
(trial/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
changes=[];patch=[]
with zipfile.ZipFile(trial/'source-baseline.zip') as archive:
    for name,old_hash in manifest['source_hashes'].items():
        if not (name.startswith(('src/','swarm-core/src/')) or name in ('Cargo.lock','Cargo.toml','swarm-core/Cargo.toml')):continue
        path=root/name
        if not path.exists():raise RuntimeError(f'Unexpected missing baseline source: {name}')
        data=path.read_bytes();new_hash=hashlib.sha256(data).hexdigest()
        if new_hash==old_hash:continue
        patch+=difflib.unified_diff(archive.read(name).decode('utf-8-sig').splitlines(True),data.decode('utf-8-sig').splitlines(True),fromfile='a/'+name,tofile='b/'+name)
        changes.append(dict(path=name,before=old_hash,after=new_hash))
    for name in ['swarm-core/src/original_overlap.rs','swarm-core/src/original_overlap/kernels.rs','swarm-core/tests/original_overlap_trial.rs']:
        assert name not in manifest['source_hashes']
        data=(root/name).read_bytes()
        patch+=difflib.unified_diff([],data.decode('utf-8-sig').splitlines(True),fromfile='/dev/null',tofile='b/'+name)
        changes.append(dict(path=name,before=None,after=hashlib.sha256(data).hexdigest()))
(root/'docs/SWARM_ORIGINAL_OVERLAP_TRIAL_2026-09-08.patch').write_text(''.join(patch),encoding='utf-8')
(trial/'trial-source-changes.json').write_text(json.dumps(changes,indent=2),encoding='utf-8')
with zipfile.ZipFile(trial/'trial-changed-source.zip','w',zipfile.ZIP_DEFLATED) as archive:
    for change in changes:archive.write(root/change['path'],change['path'])
print(json.dumps(summary,indent=2))
