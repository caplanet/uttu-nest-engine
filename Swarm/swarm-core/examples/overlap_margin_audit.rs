//! Geometry-only inspection harness. Does not alter production algorithms or settings.
use jagua_rs::geometry::geo_traits::Transformable;
use swarm_core::{config::DEFAULT_SWARM_CONFIG, util::io::import_spp_instance_with_mm};
fn main() {
    let directory = std::path::PathBuf::from(std::env::args().nth(1).expect("audit directory"));
    if std::env::args().any(|a| a == "--diagnose") {
        let cases: serde_json::Value =
            serde_json::from_slice(&std::fs::read(directory.join("swarm-geometry.json")).unwrap())
                .unwrap();
        let mut diagnostics = Vec::new();
        for case in cases
            .as_array()
            .unwrap()
            .iter()
            .filter(|c| c.get("error").is_some())
        {
            let tolerance = case["uttu_tolerance"].as_str().unwrap();
            let overlap = case["overlap"].as_f64().unwrap() as f32;
            let ext: jagua_rs::probs::spp::io::ext_repr::ExtSPInstance = serde_json::from_slice(
                &std::fs::read(directory.join(format!("uttu-{tolerance}.json"))).unwrap(),
            )
            .unwrap();
            let importer = swarm_core::util::io::new_importer(DEFAULT_SWARM_CONFIG.cde_config, Some(0.01), Some(-overlap), None);
            let offset_only = swarm_core::util::io::new_importer(DEFAULT_SWARM_CONFIG.cde_config, None, Some(-overlap), None);
            let failures: Vec<_> = ext.items.iter().filter_map(|item| importer.import_item(&item.base, item.base.id as usize).err().map(|e| serde_json::json!({"id":item.base.id,"error_prefix":e.to_string().split(": [").next().unwrap(),"also_fails_without_polysimpl":offset_only.import_item(&item.base, item.base.id as usize).is_err()}))).collect();
            println!(
                "diagnostic tolerance={tolerance}, overlap={overlap}: {} failed items",
                failures.len()
            );
            diagnostics.push(serde_json::json!({"uttu_tolerance":tolerance,"overlap":overlap,"failures":failures}));
        }
        std::fs::write(
            directory.join("import-failures.json"),
            serde_json::to_vec_pretty(&diagnostics).unwrap(),
        )
        .unwrap();
        return;
    }
    let mut results = Vec::new();
    for tolerance in ["-1", "0", "0.5", "1", "2", "3", "5"] {
        let ext = serde_json::from_str(
            &std::fs::read_to_string(directory.join(format!("uttu-{tolerance}.json"))).unwrap(),
        )
        .unwrap();
        for overlap in [0.0f32, 1.0, 2.0, 3.0, 5.0] {
            let start = std::time::Instant::now();
            match import_spp_instance_with_mm(&ext, DEFAULT_SWARM_CONFIG.cde_config, Some(0.01), Some(-overlap), None, None) {
                Ok(instance) => {
                    let items: Vec<_> = instance.items.iter().map(|(item,_)| {
                        let external = item.shape_cd().transform_clone(&item.shape_orig().pre_transform.compose().inverse());
                        serde_json::json!({"id":item.idx(),"cd":external.vertices().iter().map(|p|[p.0,p.1]).collect::<Vec<_>>()})
                    }).collect();
                    results.push(serde_json::json!({"uttu_tolerance":tolerance,"overlap":overlap,"items":items,"elapsed_ms":start.elapsed().as_millis()}));
                    println!("tolerance={tolerance} overlap={overlap} imported {} items in {:?}",instance.items.len(),start.elapsed());
                }
                Err(error) => results.push(serde_json::json!({"uttu_tolerance":tolerance,"overlap":overlap,"error":error.to_string()})),
            }
        }
    }
    std::fs::write(
        directory.join("swarm-geometry.json"),
        serde_json::to_vec(&results).unwrap(),
    )
    .unwrap();
}
