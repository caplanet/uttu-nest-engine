use jagua_rs::probs::spp::entities::SPInstance;
use swarm_core::config::DEFAULT_SWARM_CONFIG;
use swarm_core::refplace::columns::{
    ColumnDraftConfig, build_column_draft, build_column_draft_with_order,
};
use swarm_core::util::io::import_spp_instance;
use std::time::{Duration, Instant};

fn load(doc: serde_json::Value) -> SPInstance {
    let ext = serde_json::from_value(doc).unwrap();
    import_spp_instance(&ext, DEFAULT_SWARM_CONFIG.cde_config, None, None, None).unwrap()
}

fn rectangles() -> SPInstance {
    load(serde_json::json!({"name":"columns", "strip_height":100,
        "items":[{"id":0,"demand":6,"allowed_orientations":[0],"shape":{
            "type":"simple_polygon","data":[[0,0],[40,0],[40,45],[0,45]]}}]}))
}

#[test]
fn draft_commits_in_piece_order_and_reports_the_literal_area_formula() {
    let instance = load(serde_json::json!({"name":"literal-order", "strip_height":120,
        "items":[
            {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[40,0],[40,30],[0,30]]}},
            {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[40,0],[40,40],[0,40]]}},
            {"id":2,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[10,0],[10,10],[0,10]]}}
        ]}));
    let order = [(0,0),(1,0),(2,0)];
    let (_, report) = build_column_draft_with_order(&instance, Default::default(), &order, false).unwrap();
    assert_eq!(report.placements.iter().map(|p| (p.item_id,p.copy_index)).collect::<Vec<_>>(), order);
    for p in report.placements.iter().skip(1) {
        let score = p.scoring.as_ref().expect("contact search must report its actual score");
        assert_eq!(score.strip_count, 1);
        assert_eq!(score.avg_s, score.placed_area_mm2 / score.strip_count as f64);
        assert_eq!(score.value, score.placed_area_mm2 / (score.piece_count as f64 * score.reference_area_mm2));
        assert_eq!(score.final_score, score.v_max - 8.0 * score.avg_s);
    }
    assert!(report.max_piece_attempts <= 10_000);
}

fn assert_original_fabric_bounds(
    instance: &SPInstance,
    report: &swarm_core::refplace::columns::ColumnDraftReport,
) {
    use jagua_rs::geometry::{DTransformation, geo_traits::TransformableFrom};
    for p in &report.placements {
        let original = &instance.items[p.item_id].0.shape_orig();
        let transform = jagua_rs::io::export::int_to_ext_transformation(
            &DTransformation::new(p.angle_rad, (p.x as f32, p.y as f32)),
            &original.pre_transform,
        );
        let mut raw = original.shape.clone();
        raw.transform_from(&original.shape, &transform.compose());
        let b = raw.bbox();
        assert!(
            b.x_min >= -0.002
                && b.y_min >= -0.002
                && b.x_max <= report.width + 0.002
                && b.y_max <= instance.base_strip.fixed_height + 0.002,
            "source outline crosses fabric: item {}, {:?}",
            p.item_id,
            b
        );
    }
}

#[test]
fn overlap_does_not_allow_acute_source_corners_outside_fabric() {
    let ext = serde_json::from_value(serde_json::json!({"name":"acute-corners","strip_height":100,"items":[
        {"id":0,"demand":4,"allowed_orientations":[0,90],"shape":{"type":"simple_polygon","data":[[0,0],[80,30],[0,60]]}}
    ]})).unwrap();
    let instance = import_spp_instance(
        &ext,
        DEFAULT_SWARM_CONFIG.cde_config,
        Some(0.001),
        Some(-0.3),
        None,
    )
    .unwrap();
    for budget in [Duration::ZERO, Duration::from_secs(2)] {
        let (_, report) = build_column_draft(
            &instance,
            ColumnDraftConfig {
                budget,
                ..Default::default()
            },
        )
        .unwrap();
        assert_original_fabric_bounds(&instance, &report);
    }
}

#[test]
fn column_contour_can_grow_beyond_the_first_piece_width() {
    let instance = load(
        serde_json::json!({"name":"growing-column","strip_height":100,"items":[
            {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[40,0],[40,40],[0,40]]}},
            {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[60,0],[60,40],[0,40]]}}
        ]}),
    );
    let (sol, r) =
        build_column_draft_with_order(&instance, Default::default(), &[(0, 0), (1, 0)], true)
            .unwrap();
    assert_eq!(
        r.columns, 1,
        "wider second piece belongs above the root: {r:?}"
    );
    assert!(sol.strip_width() < 61.0);
}

#[test]
fn overlap_tolerance_changes_contacts_without_opening_fabric_edges() {
    let doc = serde_json::json!({"name":"tolerant-columns","strip_height":100,"items":[
        {"id":0,"demand":2,"allowed_orientations":[0],"shape":{"type":"simple_polygon","data":[[0,0],[40,0],[40,50.05],[0,50.05]]}}
    ]});
    let ext = serde_json::from_value(doc.clone()).unwrap();
    let (_, zero) = build_column_draft(&load(doc), Default::default()).unwrap();
    let instance = import_spp_instance(
        &ext,
        DEFAULT_SWARM_CONFIG.cde_config,
        None,
        Some(-0.2),
        None,
    )
    .unwrap();
    let (sol, overlap) = build_column_draft(&instance, Default::default()).unwrap();
    assert_eq!(zero.columns, 2);
    assert_eq!(
        overlap.columns, 1,
        "0.2mm allowance permits the two 50.05mm pieces in 100mm"
    );
    assert!(sol.strip_width() < 41.0);
    assert!(
        swarm_core::structure::pipeline::solution_validation_error(&instance, &sol).is_none()
    );
}

#[test]
fn diagnostic_order_is_a_permutation_and_is_preserved_even_after_timeout() {
    let instance = rectangles();
    let order = [(0, 5), (0, 2), (0, 3), (0, 0), (0, 1), (0, 4)];
    for budget in [Duration::ZERO, Duration::from_secs(2)] {
        let (_, r) = build_column_draft_with_order(
            &instance,
            ColumnDraftConfig {
                budget,
                ..Default::default()
            },
            &order,
            true,
        )
        .unwrap();
        assert_eq!(
            r.placements
                .iter()
                .map(|p| (p.item_id, p.copy_index))
                .collect::<Vec<_>>(),
            order
        );
    }
    let duplicate = [(0, 0); 6];
    assert!(
        build_column_draft_with_order(&instance, Default::default(), &duplicate, true).is_err()
    );
}

#[test]
#[ignore = "Set DRAFT_INPUT and DRAFT_ORDER_REPORT: order-only diagnostic, not a production strategy"]
fn reference_order_diagnosis_without_position_hints() {
    let raw = std::fs::read_to_string(std::env::var("DRAFT_INPUT").unwrap()).unwrap();
    let mut doc: serde_json::Value = serde_json::from_str(&raw).unwrap();
    let mut rows = Vec::new();
    for it in doc["items"].as_array().unwrap() {
        let h = &it["placed_hint"];
        let angle = h["rot_angle_in_rad"].as_f64().unwrap();
        let (sin, cos) = angle.sin_cos();
        let mut minx = f64::INFINITY;
        let mut miny = f64::INFINITY;
        for v in it["shape"]["data"].as_array().unwrap() {
            let x = v[0].as_f64().unwrap();
            let y = v[1].as_f64().unwrap();
            minx = minx.min(cos * x - sin * y);
            miny = miny.min(sin * x + cos * y);
        }
        for copy in 0..it["demand"].as_u64().unwrap() as usize {
            rows.push((
                minx + h["tr_x"].as_f64().unwrap(),
                miny + h["tr_y"].as_f64().unwrap(),
                it["id"].as_u64().unwrap() as usize,
                copy,
            ));
        }
    }
    // Explicit inference: leftmost bbox first, bottommost tie-break. This is
    // not claimed to recover the original engine's undocumented chronology.
    rows.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then(a.1.total_cmp(&b.1))
            .then((a.2, a.3).cmp(&(b.2, b.3)))
    });
    let order: Vec<_> = rows.iter().map(|r| (r.2, r.3)).collect();
    for it in doc["items"].as_array_mut().unwrap() {
        it.as_object_mut().unwrap().remove("placed_hint");
    }
    let instance = load(doc);
    // Exclude deadline completion as a confounder. These are quality/order
    // diagnostics; their times do not represent the production 8-second cap.
    let cfg = ColumnDraftConfig {
        budget: Duration::from_secs(60),
        ..Default::default()
    };
    let mut runs = Vec::new();
    for mode in ["automatic", "reference_priority", "reference_strict"] {
        let (_, r) = if mode == "automatic" {
            build_column_draft(&instance, cfg)
        } else {
            build_column_draft_with_order(&instance, cfg, &order, mode == "reference_strict")
        }
        .unwrap();
        assert!(
            !r.budget_exhausted,
            "order diagnosis must not include deadline completion"
        );
        assert_eq!(r.placements.len(), order.len());
        if mode == "reference_strict" {
            assert_eq!(
                r.placements
                    .iter()
                    .map(|p| (p.item_id, p.copy_index))
                    .collect::<Vec<_>>(),
                order
            );
        }
        println!(
            "{mode}: width {:.3}, density {:.4}%, columns {}, {}ms, first {:?}",
            r.width,
            r.density * 100.0,
            r.columns,
            r.elapsed_ms,
            r.placements
                .iter()
                .take(12)
                .map(|p| p.item_id)
                .collect::<Vec<_>>()
        );
        runs.push(serde_json::json!({"mode":mode,"report":r}));
    }
    let out = std::env::var("DRAFT_ORDER_REPORT").expect("DRAFT_ORDER_REPORT");
    std::fs::write(out,serde_json::to_string_pretty(&serde_json::json!({
        "inference":"reference bbox xmin ascending, then ymin ascending; not original execution history",
        "reference_order":order,"runs":runs
    })).unwrap()).unwrap();
}

#[test]
fn stacks_up_before_starting_the_next_column_and_preserves_copies() {
    let instance = rectangles();
    let (solution, r) = build_column_draft(&instance, ColumnDraftConfig::default()).unwrap();
    assert_eq!(r.placements.len(), 6);
    assert_eq!(r.columns, 3);
    assert!(solution.strip_width() < 121.0);
    for pair in r.placements.chunks(2) {
        assert_eq!(pair[0].column, pair[1].column);
        assert!(
            pair[1].y >= pair[0].y + 44.99,
            "aligned edges must not hide a material overlap: {r:?}"
        );
        assert!((pair[1].x - pair[0].x).abs() < 0.1);
    }
    assert!(!r.budget_exhausted);
}

#[test]
fn deadline_completion_keeps_every_piece_and_stacks_vertically() {
    let (_, r) = build_column_draft(
        &rectangles(),
        ColumnDraftConfig {
            budget: Duration::ZERO,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(r.placements.len(), 6);
    assert_eq!(r.columns, 3);
    assert!(r.budget_exhausted);
}

#[test]
fn uses_a_fitting_allowed_rotation_and_rejects_impossible_height() {
    let make = |rot: serde_json::Value| {
        load(serde_json::json!({"name":"rotation","strip_height":50,
        "items":[{"id":0,"demand":1,"allowed_orientations":rot,"shape":{
            "type":"simple_polygon","data":[[0,0],[30,0],[30,80],[0,80]]}}]}))
    };
    let (_, r) = build_column_draft(&make(serde_json::json!([0, 90])), Default::default()).unwrap();
    assert!((r.placements[0].angle_rad - std::f32::consts::FRAC_PI_2).abs() < 1e-5);
    assert!(build_column_draft(&make(serde_json::json!([0])), Default::default()).is_err());
}

#[test]
fn concave_contact_can_fill_a_pocket_without_bbox_separation() {
    let instance = load(
        serde_json::json!({"name":"pocket","strip_height":100,"items":[
        {"id":0,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon",
        "data":[[0,0],[80,0],[80,30],[30,30],[30,90],[0,90]]}},
        {"id":1,"demand":1,"allowed_orientations":[0],"shape":{"type":"simple_polygon",
        "data":[[0,0],[40,0],[40,50],[0,50]]}}]}),
    );
    let (solution, r) = build_column_draft(&instance, Default::default()).unwrap();
    assert_eq!(r.placements.len(), 2);
    assert!(
        solution.strip_width() < 81.0,
        "L pocket must be used: {r:?}"
    );
    assert!(r.placements[1].attach_to.is_some());
}

#[test]
#[ignore = "Set DRAFT_INPUT to a reference JSON; hints are not passed to the constructor"]
fn reference_benchmark_without_hints() {
    let started = Instant::now();
    let path = std::env::var("DRAFT_INPUT").expect("DRAFT_INPUT");
    let mut doc: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    for it in doc["items"].as_array_mut().unwrap() {
        it.as_object_mut().unwrap().remove("placed_hint");
    }
    let overlap: f32 = std::env::var("DRAFT_OVERLAP_MM")
        .ok()
        .map(|s| s.parse().unwrap())
        .unwrap_or(0.0);
    assert!(overlap >= 0.0);
    let ext = serde_json::from_value(doc).unwrap();
    let instance = import_spp_instance(
        &ext,
        DEFAULT_SWARM_CONFIG.cde_config,
        None,
        (overlap > 0.0).then_some(-overlap),
        None,
    )
    .unwrap();
    let budget = std::env::var("DRAFT_BUDGET_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8_000);
    let (_, r) = build_column_draft(
        &instance,
        ColumnDraftConfig {
            budget: Duration::from_millis(budget),
            ..Default::default()
        },
    )
    .unwrap();
    println!(
        "Draft end-to-end {}ms; placed {}, columns {}, width {:.3}, density {:.3}%, prep {}ms, core {}ms, pairs {}, NFP failures {}, completion {}",
        started.elapsed().as_millis(),
        r.placements.len(),
        r.columns,
        r.width,
        r.density * 100.0,
        r.preparation_ms,
        r.elapsed_ms,
        r.nfp_pairs,
        r.nfp_failures,
        r.budget_exhausted
    );
    assert_eq!(
        r.placements.len(),
        instance.items.iter().map(|(_, q)| *q).sum::<usize>()
    );
    assert_original_fabric_bounds(&instance, &r);
    if let Ok(out) = std::env::var("DRAFT_REPORT") {
        use jagua_rs::geometry::DTransformation;
        use jagua_rs::geometry::geo_traits::TransformableFrom;
        std::fs::write(&out, serde_json::to_string_pretty(&r).unwrap()).unwrap();
        let h = instance.base_strip.fixed_height;
        let mut svg = format!(
            "<svg xmlns='http://www.w3.org/2000/svg' viewBox='-10 -10 {} {}'><rect x='0' y='0' width='{}' height='{h}' fill='#fafafa' stroke='#222'/>",
            r.width + 20.0,
            h + 20.0,
            r.width
        );
        for p in &r.placements {
            let mut shape = instance.items[p.item_id].0.shape_cd().as_ref().clone();
            let original = shape.clone();
            shape.transform_from(
                &original,
                &DTransformation::new(p.angle_rad, (p.x as f32, p.y as f32)).into(),
            );
            let points = shape
                .vertices()
                .iter()
                .map(|v| format!("{},{}", v.0, h - v.1))
                .collect::<Vec<_>>()
                .join(" ");
            svg.push_str(&format!("<polygon points='{points}' fill='hsl({},55%,75%)' stroke='#333' stroke-width='1.5'/><text x='{}' y='{}' text-anchor='middle' font-size='24'>{}</text>",p.column*47%360,shape.poi().center.0,h-shape.poi().center.1,p.item_id));
        }
        svg.push_str("</svg>");
        std::fs::write(std::path::Path::new(&out).with_extension("svg"), svg).unwrap();
    }
}
