"""Sequential balanced 4-way ablation; no competing builds during measurements."""
from pathlib import Path
import subprocess,os,json,time,hashlib
root=Path(__file__).resolve().parents[1];trial=Path((root/'target/selective-overlap-current.txt').read_text());exe=trial/'selective-swap-bench.exe'
modes=['off','disrupt','separator','both'];runs=[]
for case,input in [('wsjk156',root/'data/input/wsjk156.json'),('trousers64',trial/'input/trousers-mm.json')]:
    for index,seed in enumerate([17,31,73]):
        order=modes[index:]+modes[:index]
        if index%2:order.reverse()
        for mode in order:
            name=f'{case}-{seed}-{mode}';start=time.perf_counter();env=dict(os.environ,SWAP_PROBE_MODE=mode)
            with (trial/f'{name}.log').open('w',encoding='utf-8') as log:
                p=subprocess.run([str(exe),str(input),str(seed),str(trial/f'{name}.json')],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
            runs.append(dict(name=name,wall_seconds=time.perf_counter()-start,exit=p.returncode))
            print((trial/f'{name}.log').read_text(),flush=True)
            if p.returncode:raise SystemExit(p.returncode)
(trial/'swap-benchmark-runs.json').write_text(json.dumps(dict(executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),runs=runs),indent=2))
