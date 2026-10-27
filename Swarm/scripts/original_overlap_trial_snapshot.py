"""Create an immutable trial baseline; never deletes or overwrites source files."""
from pathlib import Path
import datetime,hashlib,json,shutil,zipfile
root=Path(__file__).resolve().parents[1]
stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
out=root/'target'/('original-overlap-trial-'+stamp)
out.mkdir()
paths=[]
for directory in ['src','tests','include','swarm-core/src','swarm-core/tests','swarm-core/examples','swarm-nfp/src','docs','scripts']:
    paths.extend(p for p in (root/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
paths.extend(p for p in root.rglob('Cargo.toml') if 'target' not in p.parts and 'vendor' not in p.parts)
paths.append(root/'Cargo.lock')
paths=sorted(set(paths))
hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
with zipfile.ZipFile(out/'source-baseline.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in paths: z.write(p,p.relative_to(root))
dll=root/'target/release/cpswcore.dll'
shutil.copy2(dll,out/dll.name)
manifest={'created_local':stamp,'cwd':str(root),'source_hashes':hashes,'dll_sha256':hashlib.sha256(dll.read_bytes()).hexdigest(),'user_decision_required':True,'current_contract':'Search and original inspection/correction share existing dynamic piece_overlap_mm; no new Config or fixed 1mm/5mm budget.','gates':{'correctness':'Original pair intersections must satisfy half of the configured allowance erosion; preserve frozen exceptions, group poses, cancellation incumbents.','performance':'Report equal-work validation cost separately from equal-budget nesting quality; a regression above 20% is advisory information for the user, never automatic rollback.','quality':'Report whether baseline layouts also satisfy original validation; do not equate valid and invalid densities.','scope':'No production Config/default changes. Keep the trial reviewable and ask the user whether to retain or roll back after measurements.'}}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
(root/'target/original-overlap-trial-current.txt').write_text(str(out),encoding='utf-8')
print(out)
