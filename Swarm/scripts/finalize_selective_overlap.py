"""Record the reviewable candidate; does not restore or deploy outside workspace."""
from pathlib import Path
import ctypes,hashlib,json,zipfile,difflib,shutil
root=Path(__file__).resolve().parents[1];trial=Path((root/'target/selective-overlap-current.txt').read_text())
baseline=json.loads((trial/'manifest.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
changed=[];hashes={};missing=[]
for name,old in baseline['source_hashes'].items():
    p=root/name
    if not p.exists():missing.append(name);continue
    hashes[name]=sha(p)
    if hashes[name]!=old:changed.append(name)
assert not missing,missing
new=['swarm-core/src/original_overlap/protection.rs','swarm-core/tests/selective_overlap.rs','docs/SWARM_SELECTIVE_OVERLAP_SWAP_2026-09-08.md']
new += [p.relative_to(root).as_posix() for p in (root/'scripts').glob('*selective*') if p.is_file() and p.relative_to(root).as_posix() not in baseline['source_hashes']]
new=sorted(set(new));hashes.update({name:sha(root/name) for name in new})
for name in ['Cargo.toml','Cargo.lock','swarm-core/Cargo.toml','swarm-core/src/config.rs','src/ffi.rs']:
    assert hashes[name]==baseline['source_hashes'][name],f'Config/ABI/dependency changed: {name}'
with zipfile.ZipFile(trial/'source-baseline.zip') as z:
    diff=''.join(''.join(difflib.unified_diff(z.read(name).decode('utf-8-sig').replace('\r\n','\n').splitlines(True),(root/name).read_text(encoding='utf-8-sig').splitlines(True),fromfile='baseline/'+name,tofile='candidate/'+name)) for name in changed)
    for name in new:diff+=''.join(difflib.unified_diff([], (root/name).read_text(encoding='utf-8-sig').splitlines(True),fromfile='/dev/null',tofile='candidate/'+name))
(trial/'source-changes.diff').write_text(diff,encoding='utf-8')
with zipfile.ZipFile(trial/'source-candidate-changes.zip','w',zipfile.ZIP_DEFLATED,compresslevel=1) as z:
    for name in changed+new:z.write(root/name,name)
native=root/'target/release/cpswcore.dll';wpf=Path(baseline['wpf_directory']);copied=wpf/native.name
assert sha(native)==sha(copied)
assert sha(native)!=baseline['native_sha256'],'Old native DLL was reused'
class Abi(ctypes.Structure):
    _fields_=[('version',ctypes.c_uint32),('config_size',ctypes.c_uint32),('structure_size',ctypes.c_uint32),('features',ctypes.c_uint32)]
dll=ctypes.CDLL(str(native));f=dll.swarm_get_abi_info;f.argtypes=[ctypes.POINTER(Abi)];f.restype=ctypes.c_int32
info=Abi();assert f(ctypes.byref(info))==0
abi={n:getattr(info,n) for n,_ in Abi._fields_};assert abi==dict(version=16,config_size=816,structure_size=116,features=31),abi
shutil.copy2(native,trial/'cpswcore-selective.dll')
manifest=dict(source_hashes=hashes,changed_existing_files=changed,new_files=new,native=str(native),copied_native=str(copied),native_sha256=sha(native),abi=abi,wpf_exe=str(wpf/'CAPlanetSwarmSample.WPF.exe'),rust_profile='release: opt-level=3, lto=fat',rust_features=['simd','capmods','adaptive','structure-first','jni'],config_defaults_and_abi_unchanged=True,decision='pending user review; no automatic rollback',external_uttu_deployed=False)
(trial/'candidate-manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in manifest.items() if k!='source_hashes'},indent=2))
