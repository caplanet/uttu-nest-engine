"""Restore only the reviewed selective-overlap patch, preserving subsequent work."""
from pathlib import Path
from datetime import datetime
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parents[1]
trial = root / 'target/selective-overlap-20260908-130235'
baseline = json.loads((trial / 'manifest.json').read_text(encoding='utf-8'))
candidate = json.loads((trial / 'candidate-manifest.json').read_text(encoding='utf-8'))
restore = candidate['changed_existing_files']
remove = ['swarm-core/src/original_overlap/protection.rs', 'swarm-core/tests/selective_overlap.rs']
assert len(restore) == 6
sha = lambda data: hashlib.sha256(data).hexdigest()
before = {}
for name in restore + remove:
    path = (root / name).resolve()
    assert path.is_relative_to(root)
    before[name] = sha(path.read_bytes())
    assert before[name] == candidate['source_hashes'][name], f'Later edits require a merge: {name}'

with zipfile.ZipFile(trial / 'source-baseline.zip') as archive:
    originals = {name: archive.read(name) for name in restore}
    for name, data in originals.items():
        assert sha(data) == baseline['source_hashes'][name], f'Baseline checksum mismatch: {name}'

preserve = {name: sha((root / name).read_bytes()) for name in candidate['source_hashes']
            if name not in restore + remove and (root / name).is_file()}
for directory in ['csharp', 'src', 'swarm-core/src', 'swarm-core/tests']:
    for path in (root / directory).rglob('*'):
        if path.is_file() and not any(p in ['bin','obj'] for p in path.relative_to(root).parts):
            name = path.relative_to(root).as_posix()
            if name not in restore + remove:
                preserve[name] = sha(path.read_bytes())

backup = root / 'target' / ('uniform-overlap-rollback-' + datetime.now().strftime('%Y%m%d-%H%M%S'))
backup.mkdir(exist_ok=False)
with zipfile.ZipFile(backup / 'pre-rollback-changes.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
    for name in restore + remove:
        archive.write(root / name, name)
with zipfile.ZipFile(backup / 'pre-rollback-changes.zip') as archive:
    assert archive.testzip() is None
    assert all(sha(archive.read(name)) == value for name,value in before.items())

for name, data in originals.items():
    (root / name).write_bytes(data)
for name in remove:
    (root / name).unlink()
assert all(sha((root/name).read_bytes()) == baseline['source_hashes'][name] for name in restore)
assert all(sha((root/name).read_bytes()) == value for name,value in preserve.items())
manifest = dict(baseline=str(trial), restored=restore, removed=remove,
                before_hashes=before, restored_hashes={n:sha(d) for n,d in originals.items()},
                preserved_hashes=preserve, decision='User requested uniform original-overlap rollback; piece_overlap_mm remains configurable')
(backup / 'rollback-manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
(root / 'target/uniform-overlap-rollback-current.txt').write_text(str(backup),encoding='utf-8')
print(json.dumps(dict(backup=str(backup), restored=restore, removed=remove, preserved_files=len(preserve)),indent=2))
