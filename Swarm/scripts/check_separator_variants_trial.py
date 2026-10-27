"""Assert geometry and search equivalence before timed trial runs."""
import json
from pathlib import Path

p=Path(__file__).resolve().parents[1]/'target/separator-variants-trial'
results=[]
for case in ('wsjk','ops2'):
    baseline=json.loads((p/f'{case}-sync_fixed.json').read_text(encoding='utf-8'))
    for mode in ('preparation','kernel'):
        candidate=json.loads((p/f'{case}-{mode}_fixed.json').read_text(encoding='utf-8'))
        assert baseline['rows']==candidate['rows'],(case,mode,'fixed-round mismatch')
        if mode=='preparation':
            assert candidate['counters']['prepared_rounds']==80
        results.append({'case':case,'mode':mode,'rounds':80,'equal':True,
                        'baseline_seconds':baseline['seconds'],'variant_seconds':candidate['seconds']})
    empty=json.loads((p/f'{case}-noop_empty_checkempty.json').read_text(encoding='utf-8'))
    assert empty['certified'] and empty['counters']['skipped_empty_rounds']==1
    results.append({'case':case,'mode':'noop_empty','empty_unchanged_certified':True})
(p/'verification.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results,indent=2))
