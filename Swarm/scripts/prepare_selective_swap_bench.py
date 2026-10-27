"""Copy product source, then add measurement-only SWAP ablation hooks to the copy."""
from pathlib import Path
import shutil,json,hashlib
root=Path(__file__).resolve().parents[1];trial=Path((root/'target/selective-overlap-current.txt').read_text());dest=trial/'bench-workspace-v2'
assert not dest.exists(),'Preserve previous experiment; use a new benchmark directory'
dest.mkdir()
for crate in ('swarm-core','swarm-nfp'):
    shutil.copytree(root/crate/'src',dest/crate/'src')
    shutil.copy2(root/crate/'Cargo.toml',dest/crate/'Cargo.toml')
shutil.copy2(root/'Cargo.lock',dest/'Cargo.lock')
(dest/'Cargo.toml').write_text('[workspace]\nmembers=["swarm-core","swarm-nfp"]\nresolver="3"\n[profile.release]\nopt-level=3\nlto="fat"\n')
source_hashes={p.relative_to(dest).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.rglob('*.rs')}
base=dest/'swarm-core/src'
(base/'swap_probe.rs').write_text('''use std::sync::{OnceLock,atomic::{AtomicUsize,Ordering}};
static MODE:OnceLock<String>=OnceLock::new();
static DISRUPT:AtomicUsize=AtomicUsize::new(0);static SEP_PASS:AtomicUsize=AtomicUsize::new(0);static SEP_ACCEPT:AtomicUsize=AtomicUsize::new(0);static RNR:AtomicUsize=AtomicUsize::new(0);static REPAIRED:AtomicUsize=AtomicUsize::new(0);static IMPROVED:AtomicUsize=AtomicUsize::new(0);
pub fn allowed(kind:&str)->bool {let m=MODE.get_or_init(||std::env::var("SWAP_PROBE_MODE").unwrap());m=="both"||m==kind}
pub fn note_disrupt(applied:bool){if applied {DISRUPT.fetch_add(1,Ordering::Relaxed);}}
pub fn note_separator(n:usize){SEP_PASS.fetch_add(1,Ordering::Relaxed);SEP_ACCEPT.fetch_add(n,Ordering::Relaxed);}
pub fn note_rnr(n:usize){if n>0 {RNR.fetch_add(1,Ordering::Relaxed);}}
pub fn note_repaired(improved:bool){REPAIRED.fetch_add(1,Ordering::Relaxed);if improved {IMPROVED.fetch_add(1,Ordering::Relaxed);}}
pub fn counts()->serde_json::Value{serde_json::json!({"disrupt_applied":DISRUPT.load(Ordering::Relaxed),"separator_passes":SEP_PASS.load(Ordering::Relaxed),"separator_internal_accepts":SEP_ACCEPT.load(Ordering::Relaxed),"rnr_applied":RNR.load(Ordering::Relaxed),"disrupt_repaired_certified":REPAIRED.load(Ordering::Relaxed),"disrupt_width_improvements":IMPROVED.load(Ordering::Relaxed)})}
''')
p=base/'lib.rs';p.write_text(p.read_text()+'\npub mod swap_probe;\n')
p=base/'optimizer/explore.rs';s=p.read_text();signature='pub(crate) fn disrupt_solution(sep: &mut Separator, config: &ExplorationConfig) -> bool {'
assert s.count(signature)==1
s=s.replace(signature,'''pub(crate) fn disrupt_solution(sep: &mut Separator, config: &ExplorationConfig) -> bool {
    if !crate::swap_probe::allowed("disrupt") {return false;}
    let applied=disrupt_solution_measured(sep,config);crate::swap_probe::note_disrupt(applied);applied
}
fn disrupt_solution_measured(sep: &mut Separator, config: &ExplorationConfig) -> bool {''');p.write_text(s)
p=base/'optimizer/lns.rs';s=p.read_text();signature='pub fn swap_pass(sep: &mut Separator, n_attempts: usize, term: &impl Terminator) -> usize {';assert s.count(signature)==1
s=s.replace(signature,'''pub fn swap_pass(sep: &mut Separator, n_attempts: usize, term: &impl Terminator) -> usize {
    if !crate::swap_probe::allowed("separator") {return 0;}
    let n=swap_pass_measured(sep,n_attempts,term);crate::swap_probe::note_separator(n);n
}
fn swap_pass_measured(sep: &mut Separator, n_attempts: usize, term: &impl Terminator) -> usize {''')
s=s.replace('    sep.reinsertion_nfp = cache;','    sep.reinsertion_nfp = cache;\n    crate::swap_probe::note_rnr(result);',1);p.write_text(s)
p=base/'optimizer/explore_mods.rs';s=p.read_text();needle='        if let Some((op, ref_loss)) = pending_op.take() {';assert s.count(needle)==1
s=s.replace(needle,needle+'\n            if op==AdaptiveAlgorithm::SwapDisrupt && total_loss==0. {crate::swap_probe::note_repaired(local_best.0.strip_width()<best_width);}')
p.write_text(s)
(dest/'swarm-core/examples').mkdir();shutil.copy2(root/'scripts/selective_swap_bench.rs',dest/'swarm-core/examples/selective_swap_bench.rs')
(trial/'benchmark-source-baseline-hashes.json').write_text(json.dumps(source_hashes,indent=2))
(trial/'input').mkdir(exist_ok=True)
data=json.loads((root/'data/input/trousers.json').read_text());data['strip_height']*=20;data['name']='trousers-scaled-20'
for item in data['items']:
    assert item['shape']['type']=='simple_polygon'
    item['shape']['data']=[[x*20,y*20] for x,y in item['shape']['data']]
(trial/'input/trousers-mm.json').write_text(json.dumps(data))
print(dest)
