"""Immutable checkpoint before protected-edge overlap changes. Never restores files."""
from pathlib import Path
import datetime,hashlib,json,zipfile,shutil
root=Path(__file__).resolve().parents[1]
out=root/'target'/('selective-overlap-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
skip={'target','bin','obj','node_modules','.git','.vs','out','build','__pycache__'}
files=[p for p in root.rglob('*') if p.is_file() and not any(x in skip for x in p.relative_to(root).parts)]
hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
with zipfile.ZipFile(out/'source-baseline.zip','w',zipfile.ZIP_DEFLATED,compresslevel=1) as z:
    for p in files:z.write(p,p.relative_to(root))
with zipfile.ZipFile(out/'source-baseline.zip') as z:assert z.testzip() is None
wpf=root/'csharp/CAPlanetSwarmSample.WPF/bin/x64/Release/net10.0-windows10.0.19041.0'
with zipfile.ZipFile(out/'wpf-release-baseline.zip','w',zipfile.ZIP_DEFLATED,compresslevel=1) as z:
    for p in wpf.rglob('*'):
        if p.is_file():z.write(p,p.relative_to(wpf))
with zipfile.ZipFile(out/'wpf-release-baseline.zip') as z:assert z.testzip() is None
dll=root/'target/release/cpswcore.dll';shutil.copy2(dll,out/dll.name)
manifest={'cwd':str(root),'source_hashes':hashes,'native_sha256':hashlib.sha256(dll.read_bytes()).hexdigest(),'wpf_directory':str(wpf),'wpf_exe_sha256':hashlib.sha256((wpf/'CAPlanetSwarmSample.WPF.exe').read_bytes()).hexdigest(),'contract_before':'Uniform piece_overlap_mm on original polygons with SIMD verification/correction','restore_policy':'Only restore after user request. Compare current hashes to candidate manifest to preserve concurrent changes.'}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
(root/'target/selective-overlap-current.txt').write_text(str(out),encoding='utf-8')
print(out);print('Archived source files:',len(files));print('Native SHA256:',manifest['native_sha256'])
