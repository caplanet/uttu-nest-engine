//! Experimental JSON-lines bridge to the production native validators.
//! It does not modify the optimizer pipeline or accept a candidate implicitly.
use anyhow::{Result, ensure};
use jagua_rs::probs::spp::{
    entities::{SPInstance, SPSolution},
    io::ext_repr::{ExtSPInstance, ExtSPSolution},
};
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use serde_json::{Value, json};
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG as C,
    optimizer::place_groups::{PlaceGroupMember, PlaceGroupSpec, PlaceGroups},
    util::{frozen, io::import_spp_instance_with_mm, terminator::BasicTerminator},
};
use swarm_core::{optimizer::separator::Separator, util::terminator::Terminator};
use std::{
    io::{self, BufRead, Write},
    time::Instant,
};

fn read(path: &str) -> Result<Value> {
    Ok(serde_json::from_slice(&std::fs::read(path)?)?)
}
fn unwrap_solution(v: &Value) -> &Value {
    v.get("solution").unwrap_or(v)
}
fn parse_solution(v: &Value, n: usize) -> Result<ExtSPSolution> {
    let s: ExtSPSolution = serde_json::from_value(unwrap_solution(v).clone())?;
    ensure!(
        s.strip_width.is_finite() && s.strip_width > 0.,
        "invalid strip"
    );
    ensure!(s.layout.placed_items.len() == n, "placement count mismatch");
    let mut seen = vec![false; n];
    for p in &s.layout.placed_items {
        let id = p.item_id as usize;
        ensure!(id < n && !seen[id], "unknown or duplicate item");
        seen[id] = true;
        let t = jagua_rs::geometry::DTransformation::from(p.transformation.clone());
        ensure!(
            t.rotation().is_finite()
                && t.translation().0.is_finite()
                && t.translation().1.is_finite(),
            "nonfinite pose"
        );
    }
    Ok(s)
}
fn certify(
    inst: &SPInstance,
    sol: &SPSolution,
    reference: &SPSolution,
    frozen_ids: &[bool],
    groups: &PlaceGroups,
) -> Result<()> {
    ensure!(sol.strip() == reference.strip(), "strip changed");
    let angles: std::collections::HashMap<_, _> = reference
        .layout_snapshot()
        .placed_items()
        .values()
        .map(|p| (p.item().idx(), p.d_transf().rotation()))
        .collect();
    ensure!(
        sol.layout_snapshot()
            .placed_items()
            .values()
            .all(|p| angles.get(&p.item().idx()) == Some(&p.d_transf().rotation())),
        "rotation changed"
    );
    ensure!(
        swarm_core::util::assertions::frozen_poses_preserved(reference, sol, frozen_ids),
        "frozen pose changed"
    );
    ensure!(
        groups.solution_preserves_relative_poses(sol),
        "group relative pose changed"
    );
    if let Some(e) = swarm_core::structure::pipeline::solution_validation_error_with_frozen(
        inst, sol, frozen_ids,
    ) {
        anyhow::bail!("{e:?}");
    }
    // Fresh guard additionally ensures the final decision is not solely a cache hit.
    let mut guard = swarm_core::original_overlap::OriginalOverlapGuard::new(inst);
    let bad = guard
        .inspect(sol, frozen_ids, &BasicTerminator::new())
        .map_err(anyhow::Error::msg)?;
    ensure!(
        bad.is_empty(),
        "original overlap violation: {} pairs",
        bad.len()
    );
    Ok(())
}
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(
        args.len() == 5,
        "usage: overlap_trial_native INPUT RESULT CONSTRAINTS PIECE_OVERLAP_MM"
    );
    let overlap: f32 = args[4].parse()?;
    ensure!(
        overlap.is_finite() && overlap > 0.,
        "positive overlap required"
    );
    let input = read(&args[1])?;
    let instance_json = input.get("instance").unwrap_or(&input);
    let ext: ExtSPInstance = serde_json::from_value(instance_json.clone())?;
    let constraints = read(&args[3])?;
    let inst = import_spp_instance_with_mm(
        &ext,
        C.cde_config,
        Some(0.01),
        Some(-overlap),
        C.narrow_concavity_cutoff_ratio,
        None,
    )?;
    ensure!(
        inst.items.iter().all(|(_, q)| *q == 1),
        "trial needs expanded demand-one items"
    );
    let baseline = parse_solution(&read(&args[2])?, inst.items.len())?;
    let reference_prob = frozen::import_solution(&inst, &baseline);
    let reference = reference_prob.save();
    let ids: Vec<usize> = serde_json::from_value(constraints["frozen_item_ids"].clone())?;
    ensure!(
        ids.iter().all(|&id| id < inst.items.len()),
        "invalid frozen id"
    );
    let mut mask = frozen::mask(inst.items.len(), &ids);
    let mut specs = Vec::new();
    for g in constraints["place_groups"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("groups array required"))?
    {
        let members = g["item_ids"]
            .as_array()
            .ok_or_else(|| anyhow::anyhow!("group members required"))?
            .iter()
            .map(|v| -> Result<_> {
                let id = v
                    .as_u64()
                    .ok_or_else(|| anyhow::anyhow!("invalid member"))?
                    as usize;
                let p = reference
                    .layout_snapshot()
                    .placed_items()
                    .values()
                    .find(|p| p.item().idx() == id)
                    .ok_or_else(|| anyhow::anyhow!("missing member"))?;
                Ok(PlaceGroupMember {
                    item_id: id,
                    initial_transform: p.d_transf(),
                })
            })
            .collect::<Result<Vec<_>>>()?;
        specs.push(PlaceGroupSpec {
            id: g["id"]
                .as_u64()
                .ok_or_else(|| anyhow::anyhow!("group id"))?,
            members,
        });
    }
    let groups = PlaceGroups::new(&inst, specs)?;
    groups.expand_frozen_mask(&mut mask);
    let error = certify(&inst, &reference, &reference, &mask, &groups)
        .err()
        .map(|e| e.to_string());
    let bbox = reference.layout_snapshot().container().outer_cd().bbox();
    println!(
        "{}",
        json!({"ready":true,"baseline_certified":error.is_none(),"error":error,"frozen_mask":mask,"groups":constraints["place_groups"],"search_bounds":[bbox.x_min,bbox.y_min,bbox.x_max,bbox.y_max],"search_polygons":reference.layout_snapshot().placed_items().values().map(|p|json!({"item_id":p.item().idx(),"points":p.shape().vertices().iter().map(|v|[v.0,v.1]).collect::<Vec<_>>()})).collect::<Vec<_>>()})
    );
    io::stdout().flush()?;
    for line in io::stdin().lock().lines() {
        let started = Instant::now();
        let line = line?;
        let request: Value = serde_json::from_str(&line)?;
        if let Some(doc) = request.get("repair") {
            let outcome = (|| -> Result<Value> {
                let parsed = parse_solution(doc, inst.items.len())?;
                let prob = frozen::import_solution(&inst, &parsed);
                let mut sep = Separator::new(
                    inst.clone(),
                    prob,
                    Xoshiro256PlusPlus::seed_from_u64(request["seed"].as_u64().unwrap_or(42)),
                    C.expl_cfg.separator_config,
                );
                sep.freeze_items(&ids);
                sep.set_place_groups(groups.clone())?;
                let mut term = BasicTerminator::new();
                term.new_timeout(std::time::Duration::from_secs_f64(
                    request["seconds"].as_f64().unwrap_or(30.),
                ));
                let (sol, ct) = sep.separate(&term, &mut None);
                sep.rollback(&sol, Some(&ct));
                certify(&inst, &sol, &reference, &mask, &groups)?;
                let layout =
                    jagua_rs::io::export::export_layout_snapshot(sol.layout_snapshot(), 0);
                // Re-import the exact exported f32 public coordinates before returning them.
                let mut exported = parsed;
                exported.layout = layout;
                let roundtrip = frozen::import_solution(&inst, &exported).save();
                certify(&inst, &roundtrip, &reference, &mask, &groups)?;
                Ok(serde_json::to_value(&exported)?)
            })();
            println!(
                "{}",
                match outcome {
                    Ok(v) =>
                        json!({"valid":true,"solution":v,"seconds":started.elapsed().as_secs_f64()}),
                    Err(e) =>
                        json!({"valid":false,"error":e.to_string(),"seconds":started.elapsed().as_secs_f64()}),
                }
            );
            io::stdout().flush()?;
            continue;
        }
        let outcome = (|| -> Result<_> {
            let v: Value = serde_json::from_str(&line)?;
            let parsed = parse_solution(&v, inst.items.len())?;
            // Keeping the input f32 poses avoids adding an extra public round trip here.
            let prob = frozen::import_solution(&inst, &parsed);
            let sol = prob.save();
            certify(&inst, &sol, &reference, &mask, &groups)?;
            Ok(())
        })();
        println!(
            "{}",
            json!({"valid":outcome.is_ok(),"error":outcome.err().map(|e|e.to_string()),"seconds":started.elapsed().as_secs_f64()})
        );
        io::stdout().flush()?;
    }
    Ok(())
}
