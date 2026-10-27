//! Compiled only in the isolated benchmark workspace with swap_probe hooks.
use jagua_rs::{entities::Instance,probs::spp::entities::SPSolution};
use rand::{SeedableRng,rngs::Xoshiro256PlusPlus};
use swarm_core::{config::DEFAULT_SWARM_CONFIG as C,consts::LBF_SAMPLE_CONFIG,optimizer::{lbf::LBFBuilder,separator::Separator,place_groups::PlaceGroups},util::{io::import_spp_instance_with_mm,terminator::{BasicTerminator,Terminator}},original_overlap};
use std::{path::PathBuf,time::{Duration,Instant}};
fn main(){
    rayon::ThreadPoolBuilder::new().num_threads(4).build_global().unwrap();
    let args:Vec<_>=std::env::args().collect();let input=PathBuf::from(&args[1]);let seed:u64=args[2].parse().unwrap();let output=PathBuf::from(&args[3]);
    let ext=serde_json::from_slice(&std::fs::read(&input).unwrap()).unwrap();
    let t=Instant::now();let inst=import_spp_instance_with_mm(&ext,C.cde_config,Some(0.01),Some(-5.),None,None).unwrap();let import_s=t.elapsed().as_secs_f64();
    let t=Instant::now();let mut p=LBFBuilder::new(inst.clone(),Xoshiro256PlusPlus::seed_from_u64(seed),LBF_SAMPLE_CONFIG,C.lbf_x_multiplier,C.lbf_y_multiplier).construct().prob;
    let mut term=BasicTerminator::new();term.new_timeout(Duration::from_secs(10));
    assert!(original_overlap::repair_problem(&mut p,&[],&PlaceGroups::default(),&term).unwrap());
    assert!(swarm_core::structure::pipeline::solution_validation_error(&inst,&p.save()).is_none());
    let init_s=t.elapsed().as_secs_f64();
    let mut cfg=C.expl_cfg.separator_config;cfg.n_workers=1;
    let mut sep=Separator::new(inst.clone(),p,Xoshiro256PlusPlus::seed_from_u64(seed+1),cfg);
    sep.configure_reinsertion(&C.nfp_integration);
    let mut stages=vec![("Init",sep.prob.save(),init_s)];
    term.new_timeout(Duration::from_secs(8));let t=Instant::now();
    let sols=swarm_core::optimizer::explore_mods::exploration_phase_adaptive(&inst,&mut sep,&mut None,&term,&C.expl_cfg);
    let explored=sols.last().cloned().unwrap_or_else(||stages[0].1.clone());stages.push(("Explore",explored.clone(),t.elapsed().as_secs_f64()));
    term.new_timeout(Duration::from_secs(4));let t=Instant::now();
    let compressed=swarm_core::optimizer::compress::compression_phase(&inst,&mut sep,&explored,&mut None,&term,&C.cmpr_cfg);stages.push(("Compress",compressed.clone(),t.elapsed().as_secs_f64()));
    term.new_timeout(Duration::from_secs(2));let t=Instant::now();let mut polish=C.polish_cfg;polish.enabled=true;
    let polished=swarm_core::optimizer::polish::polish_phase_with_nfp(&inst,&mut sep,&compressed,&mut None,&term,&polish,&C.nfp_integration,&C.lp_solver);stages.push(("Polish",polished,t.elapsed().as_secs_f64()));
    let export=|s:&SPSolution|{
        assert!(swarm_core::structure::pipeline::solution_validation_error(&inst,s).is_none(),"uncertified stage output");
        let mut poses:Vec<_>=s.layout_snapshot.placed_items.values().map(|p|{let e=jagua_rs::io::export::int_to_ext_transformation(&p.d_transf,&inst.item(p.item_id).shape_orig.pre_transform);serde_json::json!({"id":p.item_id,"rotation":e.rotation(),"translation":e.translation()})}).collect();
        poses.sort_by_key(|p|serde_json::to_string(p).unwrap());
        serde_json::json!({"width":s.strip_width(),"density":s.density(&inst),"poses":poses})
    };
    let geometry:Vec<_>=inst.items.iter().map(|(item,qty)|serde_json::json!({"id":item.id,"quantity":qty,"original":item.shape_orig.shape.vertices.iter().map(|p|[p.0,p.1]).collect::<Vec<_>>(),"protection":original_overlap::protection::profile(item).as_ref(),"search":item.shape_cd.vertices.iter().map(|p|[p.0,p.1]).collect::<Vec<_>>()})).collect();
    let rows:Vec<_>=stages.iter().map(|(name,s,seconds)|serde_json::json!({"stage":name,"seconds":seconds,"layout":export(s)})).collect();
    let result=serde_json::json!({"input":input,"seed":seed,"mode":std::env::var("SWAP_PROBE_MODE").unwrap(),"overlap_mm":5.,"poly_simpl":0.01,"height":inst.base_strip.fixed_height,"import_seconds":import_s,"init_seconds":init_s,"counts":swarm_core::swap_probe::counts(),"geometry":geometry,"rows":rows});
    std::fs::write(&output,serde_json::to_vec(&result).unwrap()).unwrap();
    println!("seed={seed} mode={} width={} density={} counts={}",result["mode"],result["rows"][3]["layout"]["width"],result["rows"][3]["layout"]["density"],result["counts"]);
}
