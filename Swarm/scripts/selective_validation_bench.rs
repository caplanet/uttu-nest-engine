//! Same public-API workload for the saved uniform baseline and selective build.
use jagua_rs::{geometry::DTransformation,probs::spp::entities::{SPPlacement,SPProblem},io::import::ext_to_int_transformation};
use swarm_core::{config::DEFAULT_SWARM_CONFIG as C,original_overlap::OriginalOverlapGuard,util::{io::import_spp_instance_with_mm,terminator::BasicTerminator},structure::pipeline::solution_validation_error};
use std::{path::PathBuf,time::Instant,hint::black_box};
fn main(){
 let args:Vec<_>=std::env::args().collect();let trial=PathBuf::from(&args[1]);let output=PathBuf::from(&args[2]);let mut rows=Vec::new();
 for case in ["wsjk156","trousers64"] {for threads in [1,4] {
  let pool=rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
  pool.install(|| {
   let doc:serde_json::Value=serde_json::from_slice(&std::fs::read(trial.join(format!("{case}-17-both.json"))).unwrap()).unwrap();
   let ext=serde_json::from_slice(&std::fs::read(doc["input"].as_str().unwrap()).unwrap()).unwrap();
   let t=Instant::now();let inst=import_spp_instance_with_mm(&ext,C.cde_config,Some(0.01),Some(-5.),None,None).unwrap();let import_s=t.elapsed().as_secs_f64();
   let mut layouts=Vec::new();
   for row in doc["rows"].as_array().unwrap() {
    let s=&row["layout"];let mut p=SPProblem::new(inst.clone());p.change_strip_width(s["width"].as_f64().unwrap() as f32);
    for pose in s["poses"].as_array().unwrap() {
     let id=pose["id"].as_u64().unwrap() as usize;let ext=DTransformation::new(pose["rotation"].as_f64().unwrap() as f32,(pose["translation"][0].as_f64().unwrap() as f32,pose["translation"][1].as_f64().unwrap() as f32));
     p.place_item(SPPlacement{item_id:id,d_transf:ext_to_int_transformation(&ext,&inst.items[id].0.shape_orig.pre_transform)});
    }
    layouts.push(p.save());
   }
   for mode in ["cold_original","cached_original","changing_original","changing_full"] {
    let mut rounds=Vec::new();let mut violations=0;let term=BasicTerminator::new();
    let mut guard=OriginalOverlapGuard::new(&inst);let _=guard.inspect(&layouts[0],&[],&term).unwrap();
    for _ in 0..5 {
     let t=Instant::now();
     for i in 0..20 {
      let s=&layouts[if mode=="cached_original"{0}else{i%layouts.len()}];
      let invalid=match mode {
       "cold_original"=>!OriginalOverlapGuard::new(&inst).inspect(s,&[],&term).unwrap().is_empty(),
       "changing_full"=>solution_validation_error(&inst,s).is_some(),
       _=>!guard.inspect(s,&[],&term).unwrap().is_empty()
      };violations+=usize::from(black_box(invalid));
     }
     rounds.push(t.elapsed().as_secs_f64()/20.);
    }
    rounds.sort_by(f64::total_cmp);
    println!("{case} threads={threads} mode={mode} median_ms={:.5} invalid={violations}/100",rounds[2]*1000.);
    rows.push(serde_json::json!({"case":case,"threads":threads,"mode":mode,"import_seconds":import_s,"median_seconds":rounds[2],"rounds":rounds,"invalid":violations,"calls":100}));
   }
  });
 }}
 std::fs::write(output,serde_json::to_vec_pretty(&rows).unwrap()).unwrap();
}
