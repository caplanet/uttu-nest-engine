"""Prepare immutable old/new source copies for equal-pose validation timing."""
from pathlib import Path
import zipfile,shutil,hashlib,json
root=Path(__file__).resolve().parents[1];trial=Path((root/'target/selective-overlap-current.txt').read_text())
for label in ['uniform-baseline','selective-candidate']:
    dest=trial/(label+'-validation-workspace');assert not dest.exists();dest.mkdir()
    if label=='uniform-baseline':
        with zipfile.ZipFile(trial/'source-baseline.zip') as z:
            for name in z.namelist():
                if name=='Cargo.lock' or any(name==f'{c}/Cargo.toml' or name.startswith(f'{c}/src/') for c in ['swarm-core','swarm-nfp']):
                    target=(dest/name).resolve();assert target.is_relative_to(dest.resolve())
                    target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(name))
    else:
        for c in ['swarm-core','swarm-nfp']:
            shutil.copytree(root/c/'src',dest/c/'src');shutil.copy2(root/c/'Cargo.toml',dest/c/'Cargo.toml')
        shutil.copy2(root/'Cargo.lock',dest/'Cargo.lock')
    (dest/'Cargo.toml').write_text('[workspace]\nmembers=["swarm-core","swarm-nfp"]\nresolver="3"\n[profile.release]\nopt-level=3\nlto="fat"\n')
    examples=dest/'swarm-core/examples';examples.mkdir();shutil.copy2(root/'scripts/selective_validation_bench.rs',examples/'selective_validation_bench.rs')
    hashes={p.relative_to(dest).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.rglob('*.rs')}
    (trial/(label+'-validation-source-hashes.json')).write_text(json.dumps(hashes,indent=2))
    print(dest)
