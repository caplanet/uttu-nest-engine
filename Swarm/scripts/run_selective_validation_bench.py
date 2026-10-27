"""Balanced sequential timing, retaining per-process results and executable hashes."""
from pathlib import Path
import json,subprocess,time,hashlib
root=Path(__file__).resolve().parents[1];trial=Path((root/'target/selective-overlap-current.txt').read_text())
runs=[]
for i,label in enumerate(['selective-simd','uniform-baseline','selective-scalar','selective-scalar','uniform-baseline','selective-simd']):
    exe=trial/(label+'-validation.exe');name=f'{label}-timing-{i}';t=time.perf_counter()
    with (trial/(name+'.log')).open('w') as f:
        p=subprocess.run([str(exe),str(trial),str(trial/(name+'.json'))],cwd=root,stdout=f,stderr=subprocess.STDOUT,timeout=240)
    runs.append(dict(label=label,output=name+'.json',wall_seconds=time.perf_counter()-t,exit=p.returncode,executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest()))
    print((trial/(name+'.log')).read_text(),flush=True)
    (trial/'validation-timing-runs.json').write_text(json.dumps(runs,indent=2))
    if p.returncode:raise SystemExit(p.returncode)
