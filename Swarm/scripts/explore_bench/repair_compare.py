"""Compare optional repairs using one DLL, paired seeds/Init and sequential AB/BA runs."""
import argparse
import copy
import json
from pathlib import Path
import re
import subprocess
import sys
from paired import ROOT, compare, sha256


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dll', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--modes', nargs='+', choices=['direction', 'lp', 'chain', 'all'], default=['direction', 'lp', 'chain', 'all'])
    p.add_argument('--seeds', nargs='+', type=int, default=[41])
    p.add_argument('--seconds', type=float, default=20)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    harness = ROOT/'scripts/explore_bench/bin/Release/net10.0/ExploreBench.dll'
    markers = {'wsjk':Path('C:/Caplanet/AiWorks/InputData/여성WSjk_비교용-G_New_SwarmV1.json'),
               'ops2':ROOT/'target/phase-improvements-bench/inputs/ops2.json'}
    fingerprint = sha256(args.dll)
    (args.out/'manifest.json').write_text(json.dumps({'dll':str(args.dll.resolve()),'sha256':fingerprint,
        'harness_sha256':sha256(harness),'arguments':vars(args)},default=str,indent=2),encoding='utf-8')
    rows=[]
    for marker,input_file in markers.items():
        for seed in args.seeds:
            for mode in args.modes:
                order='AB' if len(rows)%2==0 else 'BA'
                summaries={}
                for label in order:
                    actual_mode='off' if label=='A' else mode
                    prefix=args.out/f'{len(rows):02d}-{marker}-{seed}-{mode}-{label}'
                    assert sha256(args.dll)==fingerprint
                    print(f'{prefix.name}: {actual_mode} {args.seconds:g}s',flush=True)
                    with prefix.with_suffix('.log').open('w',encoding='utf-8') as log:
                        subprocess.run(['dotnet',str(harness),str(args.dll.resolve()),str(input_file),str(seed),
                            str(args.seconds),str(prefix),actual_mode],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                            timeout=600,check=True)
                    summaries[label]=json.loads(Path(str(prefix)+'.summary.json').read_text(encoding='utf-8'))
                    summaries[label]['repair_lines']=[x for x in Path(str(prefix)+'.native.log').read_text(encoding='utf-8').splitlines() if '[SEP-REPAIR]' in x]
                a,b=(copy.deepcopy(summaries[k]) for k in 'AB')
                ca=a['config'].pop('repair');cb=b['config'].pop('repair')
                # Only independent algorithm switches may differ; all budgets remain equal.
                switches=('direction_enabled','cluster_lp_enabled','chain_enabled')
                assert all(ca[k]==cb[k] for k in ca if k not in switches)
                row=compare(a,b,marker,seed,args.seconds,0,order)
                row['mode']=mode
                row['repair_passes']=len(b['repair_lines'])
                row['accepted_passes']=sum('accepted=true' in line for line in b['repair_lines'])
                row['repair_elapsed_ms']=sum(float(re.search(r'elapsed_ms=([\d.]+)',line)[1]) for line in b['repair_lines'])
                row['repair_lines']=b['repair_lines']
                row['repair_measurement_note']='Counts/times cover emitted completion logs; early cancellation or preparation abort can return before logging. They are not total repair CPU time.'
                rows.append(row)
                (args.out/'comparison.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
                print(f"delta={row['delta_mm']:+.4f}mm repair={row['accepted_passes']}/{row['repair_passes']}",flush=True)
    print(json.dumps({'pairs':len(rows),'wins':sum(r['delta_mm'] < -0.01 for r in rows),
        'regressions':sum(not r['no_width_regression'] for r in rows)},indent=2))


if __name__=='__main__':
    main()
