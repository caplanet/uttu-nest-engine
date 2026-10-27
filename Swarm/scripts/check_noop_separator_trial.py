"""Explain whether skipping a no-op changes the subsequent search trajectory."""
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1]/'target/separator-variants-trial'
rows=[]
for case in ('wsjk','ops2'):
    baseline=json.loads((root/f'{case}-sync_fixed.json').read_text(encoding='utf-8'))
    candidate=json.loads((root/f'{case}-noop_empty_fixed.json').read_text(encoding='utf-8'))
    assert candidate['counters']['skipped_noops']>0
    differences=[]
    for n,(a,b) in enumerate(zip(baseline['rows'],candidate['rows'])):
        changed=[k for k in ('poses','loss','weighted_loss','evals') if a[k]!=b[k]]
        if changed: differences.append({'round':n,'changed':changed})
    rows.append({'case':case,'skipped_noops':candidate['counters']['skipped_noops'],
                 'trajectory_difference_rounds':len(differences),'first_difference':differences[:1],
                 'reported_moves_baseline':sum(r['moves'] for r in baseline['rows']),
                 'reported_moves_noop':sum(r['moves'] for r in candidate['rows'])})
(root/'noop-trajectory.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
print(json.dumps(rows,indent=2))
