"""Compare pre-repair ABI15 DLL with ABI16 repairs-off; each uses matching managed bindings."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
from paired import ROOT, compare, sha256

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--baseline',type=Path,required=True)
p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--out',type=Path,required=True)
p.add_argument('--seed',type=int,default=41)
p.add_argument('--seconds',type=float,default=20)
args=p.parse_args()
args.out.mkdir(parents=True,exist_ok=False)
hosts={'A':ROOT/'target/repair-legacy-host/bin/Release/net10.0/LegacyBench.dll',
       'B':ROOT/'scripts/explore_bench/bin/Release/net10.0/ExploreBench.dll'}
dlls={'A':args.baseline.resolve(),'B':args.candidate.resolve()}
hashes={k:sha256(v) for k,v in dlls.items()}
markers={'wsjk':Path('C:/Caplanet/AiWorks/InputData/여성WSjk_비교용-G_New_SwarmV1.json'),
         'ops2':ROOT/'target/phase-improvements-bench/inputs/ops2.json'}
rows=[]
for marker,input_path in markers.items():
    order='AB' if not rows else 'BA'
    summaries={}
    for label in order:
        prefix=args.out/f'{marker}-{label}'
        assert sha256(dlls[label])==hashes[label]
        print(marker,label,flush=True)
        with prefix.with_suffix('.log').open('w',encoding='utf-8') as log:
            subprocess.run(['dotnet',str(hosts[label]),str(dlls[label]),str(input_path),str(args.seed),str(args.seconds),str(prefix)],
                stdout=log,stderr=subprocess.STDOUT,cwd=ROOT,timeout=600,check=True)
        summaries[label]=json.loads(Path(str(prefix)+'.summary.json').read_text(encoding='utf-8'))
    a,b=(copy.deepcopy(summaries[k]) for k in 'AB')
    repair=b['config'].pop('repair')
    assert not any(repair[k] for k in ('direction_enabled','cluster_lp_enabled','chain_enabled'))
    assert a['abi']['abi_version']==15 and b['abi']['abi_version']==16
    assert a['abi']['feature_flags']==b['abi']['feature_flags']
    a['abi']=b['abi'] # Each original snapshot retains the actual validated ABI.
    row=compare(a,b,marker,args.seed,args.seconds,0,order)
    row['baseline_abi']=summaries['A']['abi'];row['candidate_abi']=summaries['B']['abi']
    row['dll_sha256']=hashes
    row['note']='Whole-build comparison across ABI15/16, each with matching host; log capture off on both. PGO training differs. Cannot isolate algorithm effect.'
    rows.append(row)
    (args.out/'comparison.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    print('delta',row['delta_mm'],flush=True)
