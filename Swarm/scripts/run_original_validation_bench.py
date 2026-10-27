from pathlib import Path
import subprocess,json,time,sys,hashlib
import psutil
root=Path(__file__).resolve().parents[1]
trial=Path((root/'target/original-overlap-trial-current.txt').read_text())
label=sys.argv[1]
assert label in ('baseline','candidate')
certified=len(sys.argv)>2 and sys.argv[2]=='certified'
name=label+('-certified' if certified else '')
inputs=trial/'certified-validation-input' if certified else trial
if certified:
    inputs.mkdir(exist_ok=True)
    for source in trial.glob('shared-wsjk-seed*-overlap*.json'):
        destination=inputs/source.name.removeprefix('shared-')
        if destination.exists():continue # Freeze identical poses for both executables.
        doc=json.loads(source.read_text(encoding='utf-8'))
        for row in doc['rows']:row['before']=row['after']
        destination.write_text(json.dumps(doc),encoding='utf-8')
exe=trial/(label+'-validation.exe')
start=time.perf_counter();peak=0
with (trial/(name+'-validation.log')).open('w',encoding='utf-8') as log:
    proc=subprocess.Popen([str(exe),str(inputs),str(trial/(name+'-validation.json'))],cwd=root,stdout=log,stderr=subprocess.STDOUT)
    monitor=psutil.Process(proc.pid)
    while proc.poll() is None:
        try:peak=max(peak,monitor.memory_info().rss)
        except psutil.NoSuchProcess:pass
        time.sleep(.05)
result={'label':name,'wall_seconds':time.perf_counter()-start,'peak_rss_bytes':peak,'exit':proc.returncode,'executable_sha256':hashlib.sha256(exe.read_bytes()).hexdigest()}
(trial/(name+'-validation-process.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
print((trial/(name+'-validation.log')).read_text())
print(json.dumps(result))
raise SystemExit(proc.returncode)
