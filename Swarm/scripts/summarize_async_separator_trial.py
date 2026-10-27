"""Summarize completed isolated trials; do not infer missing target times."""
import json
import statistics as stats
from pathlib import Path

root = Path(__file__).resolve().parents[1] / 'target' / 'async-separator-trial'
manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8-sig'))
summary = {'manifest': manifest, 'cases': []}
for case in manifest['cases']:
    name = case['name']
    modes = {}
    for mode in ('sync', 'async'):
        rows = [json.loads((root / f'{name}-{mode}-{seed}.json').read_text(encoding='utf-8'))
                for seed in manifest['seeds']]
        assert all(r['certified'] for r in rows)
        assert len({r['initial_width'] for r in rows}) == 1
        widths = [r['width'] for r in rows]
        modes[mode] = {
            'mean_width': stats.mean(widths), 'median_width': stats.median(widths),
            'min_width': min(widths), 'max_width': max(widths),
            'mean_density_percent': stats.mean(r['density']*100 for r in rows),
            'mean_elapsed_seconds': stats.mean(r['elapsed_seconds'] for r in rows),
            'targets': {},
            'runs': [{k: r[k] for k in ('seed', 'width', 'density', 'elapsed_seconds', 'lanes')} for r in rows]
        }
        for pct in (78, 80, 82, 83.6, 84, 85):
            times = [next((t['received_at'] for t in r['trace'] if t['density'] >= pct/100), None)
                     for r in rows]
            reached = [t for t in times if t is not None]
            modes[mode]['targets'][str(pct)] = {
                'seconds_by_seed': times, 'reached': len(reached),
                'median_among_reached': stats.median(reached) if reached else None,
            }
        modes[mode]['relative_width_targets'] = {}
        for pct in (0.1, 0.5, 1, 2):
            times = [next((t['received_at'] for t in r['trace']
                           if t['width'] <= r['initial_width']*(1-pct/100)), None)
                     for r in rows]
            modes[mode]['relative_width_targets'][str(pct)] = times
        if mode == 'sync':
            baseline_rows = rows
        else:
            async_rows = rows
    assert all(a['initial_width']==b['initial_width'] for a,b in zip(baseline_rows,async_rows))
    pairs = [{'seed': a['seed'], 'width_delta_mm': b['width']-a['width'],
              'density_delta_pp': (b['density']-a['density'])*100}
             for a,b in zip(baseline_rows,async_rows)]
    p = json.loads((root / f'{name}-barrier-probe.json').read_text(encoding='utf-8'))
    wall = sum(r['seconds'] for r in p['rounds'])
    gap = sum(max(0, r['seconds']-w['done']) for r in p['rounds'] for w in r['workers'])
    summary['cases'].append({
        'name': name, 'initial_width': baseline_rows[0]['initial_width'], 'modes': modes, 'pairs': pairs,
        'mean_width_delta_mm': modes['async']['mean_width']-modes['sync']['mean_width'],
        'mean_density_delta_pp': modes['async']['mean_density_percent']-modes['sync']['mean_density_percent'],
        'async_shorter_count': sum(p['width_delta_mm']<0 for p in pairs),
        'probe': {'rounds': len(p['rounds']), 'wall_seconds': wall,
                  'completion_gap_seconds': gap, 'completion_gap_ratio': gap/(p['workers']*wall)},
    })
(root / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
for case in summary['cases']:
    print(case['name'], json.dumps({k:v for k,v in case.items() if k not in ('modes','name')}, ensure_ascii=False))
    for mode, row in case['modes'].items():
        print(mode, json.dumps({k:v for k,v in row.items() if k!='runs'}, ensure_ascii=False))
