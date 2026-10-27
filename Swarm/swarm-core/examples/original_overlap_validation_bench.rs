//! Uses only pre-trial public APIs, so the exact same source can benchmark
//! the immutable baseline and candidate builds without runtime feature toggles.
use jagua_rs::{geometry::DTransformation,probs::spp::entities::{SPPlacement,SPProblem}};
use swarm_core::{config::DEFAULT_SWARM_CONFIG,util::io::import_spp_instance_with_mm,structure::pipeline::solution_validation_error};
use std::{path::PathBuf,time::Instant};
fn main() {
 let args:Vec<_>=std::env::args().collect();let trial=PathBuf::from(&args[1]);let output=PathBuf::from(&args[2]);
 let ext=serde_json::from_slice(&std::fs::read("data/input/wsjk156.json").unwrap()).unwrap();
 let mut reports=Vec::new();
 for overlap in [1.,5.] {
  let inst=import_spp_instance_with_mm(&ext,DEFAULT_SWARM_CONFIG.cde_config,Some(0.01),Some(-overlap),None,None).unwrap();
  let mut layouts=Vec::new();
  for seed in [17,31,73] {
   let doc:serde_json::Value=serde_json::from_slice(&std::fs::read(trial.join(format!("wsjk-seed{seed}-overlap{overlap}.json"))).unwrap()).unwrap();
   for row in doc["rows"].as_array().unwrap() {
    let s=&row["before"];let mut p=SPProblem::new(inst.clone()).expect("strip forms a valid container");p.change_strip_width(s["width"].as_f64().unwrap() as f32).expect("strip width must be positive");
    for pose in s["poses"].as_array().unwrap() {
     let id=pose["id"].as_u64().unwrap() as usize;let r=pose["rotation"].as_f64().unwrap() as f32;
     let (sin,cos)=r.sin_cos();let (px,py)=inst.items[id].0.shape_orig().pre_transform.translation();
     let x=pose["translation"][0].as_f64().unwrap() as f32-cos*px+sin*py;
     let y=pose["translation"][1].as_f64().unwrap() as f32-sin*px-cos*py;
     p.place_item(SPPlacement{item_idx:id,d_transf:DTransformation::new(r,(x,y))});
    }
    layouts.push(p.save());
   }
  }
  for mode in ["unchanged","changing"] {
   let mut rounds=Vec::new();let mut accepted=0;
   for _ in 0..3 {
    let start=Instant::now();
    for i in 0..60 {let sol=if mode=="unchanged" {&layouts[0]} else {&layouts[i%layouts.len()]};accepted+=usize::from(solution_validation_error(&inst,sol).is_none());}
    rounds.push(start.elapsed().as_secs_f64()/60.);
   }
   rounds.sort_by(f64::total_cmp);
   println!("overlap={overlap} mode={mode} median_ms={:.4} accepted={accepted}/180",rounds[1]*1000.);
   reports.push(serde_json::json!({"overlap":overlap,"mode":mode,"median_seconds":rounds[1],"rounds_seconds_per_call":rounds,"accepted":accepted,"calls":180}));
  }
 }
 std::fs::write(output,serde_json::to_vec_pretty(&reports).unwrap()).unwrap();
}
