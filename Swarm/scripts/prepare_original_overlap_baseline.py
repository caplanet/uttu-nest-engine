"""Extract an isolated original core for same-source validation benchmarking."""
from pathlib import Path
import zipfile,shutil,json,hashlib
root=Path(__file__).resolve().parents[1]
trial=Path((root/'target/original-overlap-trial-current.txt').read_text())
dest=trial/'baseline-core'
dest.mkdir(exist_ok=True)
with zipfile.ZipFile(trial/'source-baseline.zip') as z:
    for name in z.namelist():
        if name.startswith(('swarm-core/src/','swarm-nfp/src/')) or name in ('swarm-core/Cargo.toml','swarm-nfp/Cargo.toml','Cargo.lock'):
            target=(dest/name).resolve()
            assert target.is_relative_to(dest.resolve())
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(z.read(name))
(dest/'Cargo.toml').write_text('[workspace]\nmembers=["swarm-core","swarm-nfp"]\nresolver="3"\n[profile.release]\nopt-level=3\nlto="fat"\n',encoding='utf-8')
example=dest/'swarm-core/examples/original_overlap_validation_bench.rs'
example.parent.mkdir(exist_ok=True)
shutil.copy2(root/'swarm-core/examples/original_overlap_validation_bench.rs',example)
print(dest)
