use jagua_rs::geometry::primitives::{Point, SPolygon};
use jagua_rs::geometry::shape_modification::ShapeModifyMode;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::util::{io::import_spp_instance_with_mm, simplify_mm::simplify_shape_mm};
use std::time::Instant;

#[test]
fn many_small_removals_cannot_erase_a_deeper_original_notch() {
    let p = SPolygon::new(vec![
        Point(0., 0.),
        Point(100., 0.),
        Point(100., 10.),
        Point(80., 9.9),
        Point(60., 9.7),
        Point(40., 9.9),
        Point(20., 9.7),
        Point(0., 10.),
    ])
    .unwrap();
    let s = simplify_shape_mm(&p, ShapeModifyMode::Inflate, 0.1).unwrap();
    // Repeated local 0.1mm edits must not replace the whole upper chain by y=10.
    assert!(
        s.vertices().iter().any(|p| p.1 > 9.0 && p.1 < 9.81),
        "lost the original 0.3mm-deep notches: {:?}",
        s.vertices()
    );
    assert!(s.area() >= p.area() - 0.001);
}

#[test]
fn mm_import_replaces_area_and_narrow_closing_but_keeps_overlap_offset() {
    let ext=serde_json::from_value(serde_json::json!({"name":"mm-import","strip_height":100,"items":[
        {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[10,0],[10,9.8],[9.8,10],[0,10]]}}
    ]})).unwrap();
    let cfg = DEFAULT_SWARM_CONFIG.cde_config;
    let exact = import_spp_instance_with_mm(&ext, cfg, None, Some(-0.1), None, None).unwrap();
    let zero =
        import_spp_instance_with_mm(&ext, cfg, Some(0.9), Some(-0.1), Some((0.9, 0.9)), Some(0.))
            .unwrap();
    assert_eq!(
        exact.items[0].0.shape_cd().vertices(),
        zero.items[0].0.shape_cd().vertices()
    );
    let mm = import_spp_instance_with_mm(
        &ext,
        cfg,
        Some(0.9),
        Some(-0.1),
        Some((0.9, 0.9)),
        Some(0.3),
    )
    .unwrap();
    assert!(mm.items[0].0.shape_cd().vertices().len() < exact.items[0].0.shape_cd().vertices().len());
    assert_eq!(mm.base_strip.shape_modify_config.offset, Some(0.05));
    assert!(
        mm.items[0]
            .0
            .shape_orig()
            .modify_config
            .simplify_tolerance
            .is_none()
    );
    assert!(import_spp_instance_with_mm(&ext, cfg, None, None, None, Some(f32::NAN)).is_err());
}

#[test]
#[ignore = "Set DRAFT_INPUT and MM_GEOMETRY_REPORT for source-outline audit"]
fn marker_mm_simplification_geometry() {
    let doc: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(std::env::var("DRAFT_INPUT").unwrap()).unwrap(),
    )
    .unwrap();
    let ext = serde_json::from_value(doc).unwrap();
    let mut cases = Vec::new();
    for overlap in [0.0f32, 0.1] {
        let source = import_spp_instance_with_mm(
            &ext,
            DEFAULT_SWARM_CONFIG.cde_config,
            None,
            (overlap > 0.).then_some(-overlap),
            None,
            None,
        )
        .unwrap();
        for (mm, mode) in [
            (0.1f32, ShapeModifyMode::Inflate),
            (0.3, ShapeModifyMode::Inflate),
            (0.5, ShapeModifyMode::Inflate),
            (0.3, ShapeModifyMode::Deflate),
        ] {
            let start = Instant::now();
            let mut rows = Vec::new();
            let mut before = 0;
            let mut after = 0;
            for (item, _) in &source.items {
                let shape = simplify_shape_mm(&item.shape_cd(), mode, mm).unwrap();
                before += item.shape_cd().vertices().len();
                after += shape.vertices().len();
                rows.push(serde_json::json!({"id":item.idx(),"before":item.shape_cd().vertices().iter().map(|p|[p.0,p.1]).collect::<Vec<_>>(),"after":shape.vertices().iter().map(|p|[p.0,p.1]).collect::<Vec<_>>()}));
            }
            let elapsed = start.elapsed().as_millis();
            println!(
                "{mode:?} mm {mm}, overlap {overlap}: vertices {before} -> {after}, simplification {elapsed}ms"
            );
            cases.push(serde_json::json!({"mode":format!("{mode:?}"),"mm":mm,"overlap_mm":overlap,"before":before,"after":after,"elapsed_ms":elapsed,"items":rows}));
        }
    }
    std::fs::write(
        std::env::var("MM_GEOMETRY_REPORT").unwrap(),
        serde_json::to_string(&cases).unwrap(),
    )
    .unwrap();
}

#[test]
#[ignore = "Set MM_EXPECTED_GEOMETRY to a pre-optimization geometry report"]
fn saved_geometry_is_unchanged_by_lazy_candidate_materialization() {
    let cases: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(std::env::var("MM_EXPECTED_GEOMETRY").unwrap()).unwrap(),
    ).unwrap();
    let mut checked = 0;
    for case in cases.as_array().unwrap() {
        let mode = if case["mode"] == "Deflate" { ShapeModifyMode::Deflate } else { ShapeModifyMode::Inflate };
        let mm = case["mm"].as_f64().unwrap() as f32;
        for item in case["items"].as_array().unwrap() {
            let points = |field: &str| item[field].as_array().unwrap().iter()
                .map(|p| Point(p[0].as_f64().unwrap() as f32, p[1].as_f64().unwrap() as f32)).collect::<Vec<_>>();
            let source = SPolygon::new(points("before")).unwrap();
            let actual = simplify_shape_mm(&source, mode, mm).unwrap();
            assert_eq!(actual.vertices(), points("after"), "mode {mode:?}, mm {mm}, item {}", item["id"]);
            checked += 1;
        }
    }
    assert!(checked > 0);
    println!("Exact pre-optimization vertex sequence matches: {checked}");
}
