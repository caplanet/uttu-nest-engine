"""Check the workspace trial DLL and preserve a review copy, without deployment."""
from pathlib import Path
import ctypes,hashlib,json,shutil
root=Path(__file__).resolve().parents[1]
trial=Path((root/'target/original-overlap-trial-current.txt').read_text())
class Abi(ctypes.Structure):
    _fields_=[('version',ctypes.c_uint32),('config_size',ctypes.c_uint32),('structure_size',ctypes.c_uint32),('features',ctypes.c_uint32)]
path=root/'target/release/cpswcore.dll'
dll=ctypes.CDLL(str(path))
f=dll.swarm_get_abi_info
f.argtypes=[ctypes.POINTER(Abi)];f.restype=ctypes.c_int32
assert f(None)==-1
info=Abi();assert f(ctypes.byref(info))==0
values={name:getattr(info,name) for name,_ in Abi._fields_}
assert values==dict(version=16,config_size=816,structure_size=116,features=15),values
shutil.copy2(path,trial/'cpswcore-trial.dll')
result=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),bytes=path.stat().st_size,abi=values,deployed_to_uttu=False,decision='pending user choice')
(trial/'candidate-abi.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
