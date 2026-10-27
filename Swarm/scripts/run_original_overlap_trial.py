"""Sequential paired workloads; do not run competing benchmarks in parallel."""
from pathlib import Path
import subprocess,json,hashlib,time,sys
root=Path(__file__).resolve().parents[1]
out=Path((root/'target/original-overlap-trial-current.txt').read_text())
exe=root/'target/release/examples/original_overlap_trial.exe'
records=[]
growth='--growth' in sys.argv
shared='--shared' in sys.argv
for seed in (17,31,73):
    for overlap in ((5,1) if seed!=31 else (1,5)):
        name=('shared-' if shared else ('growth-' if growth else ''))+f'wsjk-seed{seed}-overlap{overlap}'
        cmd=[str(exe),'data/input/wsjk156.json',str(seed),str(overlap),str(out/(name+'.json'))]
        if growth: cmd.append('--growth')
        print('RUN',name,flush=True)
        start=time.perf_counter()
        with (out/(name+'.log')).open('w',encoding='utf-8') as log:
            p=subprocess.run(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=300)
        records.append({'name':name,'seconds':time.perf_counter()-start,'exit':p.returncode})
        print((out/(name+'.log')).read_text(),flush=True)
        if p.returncode: break
    if records[-1]['exit']: break
(out/('shared-benchmark-processes.json' if shared else ('growth-benchmark-processes.json' if growth else 'benchmark-processes.json'))).write_text(json.dumps({'executable_sha256':hashlib.sha256(exe.read_bytes()).hexdigest(),'runs':records},indent=2),encoding='utf-8')
