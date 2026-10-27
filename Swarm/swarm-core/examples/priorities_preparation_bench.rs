//! Isolated same-process comparison of old lane reimport versus shared geometry.
//! Run with a supplied marker input path; emits JSON records for the report.
use swarm_core::{
    config::DEFAULT_SWARM_CONFIG,
    util::io::{import_spp_instance_with_mm, prepared_spp_subset, read_spp_input},
};
use std::{path::Path, time::Instant};
fn main() {
    let input = std::env::args().nth(1).expect("input JSON path");
    let (ext, _) = read_spp_input(Path::new(&input)).unwrap();
    let lanes: Vec<Vec<usize>> = (0..ext.items.len())
        .collect::<Vec<_>>()
        .chunks(ext.items.len().div_ceil(4))
        .map(|ids| ids.to_vec())
        .collect();
    let mut checksum: Option<(bool, u64, usize)> = None;
    for mm in [None, Some(0.5)] {
        for rep in 0..3 {
            for shared in if rep % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let start = Instant::now();
                let full = import_spp_instance_with_mm(
                    &ext,
                    DEFAULT_SWARM_CONFIG.cde_config,
                    Some(0.01),
                    Some(-2.2),
                    None,
                    mm,
                )
                .unwrap();
                let lane_start = Instant::now();
                let mut area = 0.0f64;
                let mut vertices = 0;
                for ids in &lanes {
                    let lane = if shared {
                        prepared_spp_subset(&full, ids, Some(-2.2)).unwrap()
                    } else {
                        let mut local = ext.clone();
                        local.items = ids
                            .iter()
                            .enumerate()
                            .map(|(id, &global)| {
                                let mut item = ext.items[global].clone();
                                item.base.id = id as u64;
                                item
                            })
                            .collect();
                        import_spp_instance_with_mm(
                            &local,
                            DEFAULT_SWARM_CONFIG.cde_config,
                            Some(0.01),
                            Some(-2.2),
                            None,
                            mm,
                        )
                        .unwrap()
                    };
                    area += lane.item_area() as f64;
                    vertices += lane
                        .items
                        .iter()
                        .map(|(i, _)| i.shape_cd().vertices().len())
                        .sum::<usize>();
                    std::hint::black_box(lane);
                }
                let key = (mm.is_some(), area.to_bits(), vertices);
                if let Some(previous) = checksum {
                    if previous.0 == key.0 {
                        assert_eq!(previous, key);
                    }
                }
                checksum = Some(key);
                println!(
                    "{}",
                    serde_json::json!({"mode":if mm.is_some(){"mm0.5"}else{"area0.01"},
                    "rep":rep,"shared":shared,"items":ext.items.len(),"lanes":lanes.len(),
                    "lane_ms":lane_start.elapsed().as_secs_f64()*1000.,"total_ms":start.elapsed().as_secs_f64()*1000.,
                    "vertices":vertices,"area":area})
                );
            }
        }
    }
}
