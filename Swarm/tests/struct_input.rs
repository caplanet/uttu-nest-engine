//! Contract checks: direct input must preserve geometry/metadata and JSON validation.
use cpswcore::ffi_input::*;
use cpswcore::ffi_step::*;
use cpswcore::grouped_input;
use std::ffi::CString;

fn fixture() -> (Vec<CInputPoint>, Vec<f32>, Vec<CInputItem>) {
    let points = vec![
        CInputPoint { x: 0.0, y: 0.0 },
        CInputPoint { x: 10.0, y: 0.0 },
        CInputPoint { x: 10.0, y: 8.0 },
        CInputPoint { x: 0.0, y: 8.0 },
    ];
    let angles = vec![0.0, 180.0];
    let items = vec![CInputItem {
        id: 0,
        demand: 1,
        shape_kind: 1,
        vertices: points.as_ptr(),
        vertex_count: points.len(),
        rotation_kind: 0,
        angles: angles.as_ptr(),
        angle_count: angles.len(),
        min_quality: -1,
        ..Default::default()
    }];
    (points, angles, items)
}

fn view(items: &[CInputItem]) -> CSwarmInput {
    CSwarmInput {
        abi_version: 1,
        struct_size: std::mem::size_of::<CSwarmInput>() as u32,
        name: b"test".as_ptr(),
        name_len: 4,
        strip_height: 50.0,
        items: items.as_ptr(),
        item_count: items.len(),
        group_workers: 1,
        quantum_ms: 250,
        ..Default::default()
    }
}

fn json() -> String {
    r#"{"schema_version":2,"nesting":{"mode":"all_items","group_workers":1,"quantum_ms":250},"instance":{"name":"test","strip_height":50,"items":[{"id":0,"demand":1,"allowed_orientations":[0,180],"shape":{"type":"simple_polygon","data":[[0,0],[10,0],[10,8],[0,8]]}}]}}"#.into()
}

#[test]
fn direct_input_matches_json_and_owns_copied_vertices() {
    let (mut points, _angles, items) = fixture();
    let direct = unsafe { convert_input(&view(&items)) }.unwrap();
    points[0].x = 999.0; // native result must not borrow the caller's buffers
    let parsed = grouped_input::parse_and_validate(&json()).unwrap();
    assert_eq!(
        serde_json::to_value(&direct.instance).unwrap(),
        serde_json::to_value(&parsed.instance).unwrap()
    );
    assert_eq!(direct.nesting.quantum_ms, 250);
    assert!(direct.hints.is_empty());
}

#[test]
fn invalid_descriptor_returns_null_and_error_without_dereferencing_null_array() {
    let mut input = view(&[]);
    input.item_count = 1;
    input.items = std::ptr::null();
    let prepared = unsafe { swarm_input_from_struct(&input) };
    assert!(prepared.is_null());
    assert_ne!(swarm_get_last_error_code(), 0);
    input.abi_version = 99;
    assert!(unsafe { swarm_input_from_struct(&input) }.is_null());
    unsafe { swarm_input_free(std::ptr::null_mut()) };
}

#[test]
fn direct_input_uses_same_demand_and_id_validation_as_json() {
    let (_points, _angles, mut items) = fixture();
    items[0].demand = 2;
    assert_eq!(
        unsafe { convert_input(&view(&items)) }.err().unwrap().code,
        grouped_input::InputErrorCode::InvalidItem
    );
    items[0].demand = 1;
    items[0].id = 5;
    assert!(unsafe { convert_input(&view(&items)) }.is_err());
}

#[test]
fn direct_input_rejects_bad_tags_lengths_and_nonfinite_numbers() {
    let (_points, _angles, mut items) = fixture();
    items[0].shape_kind = 99;
    assert!(unsafe { convert_input(&view(&items)) }.is_err());
    items[0].shape_kind = 1;
    items[0].vertex_count = usize::MAX;
    assert!(unsafe { convert_input(&view(&items)) }.is_err());
    items[0].vertex_count = 4;
    items[0].rotation_kind = 99;
    assert!(unsafe { convert_input(&view(&items)) }.is_err());
    items[0].rotation_kind = 0;
    let mut input = view(&items);
    input.strip_height = f32::NAN;
    assert!(unsafe { convert_input(&input) }.is_err());
}

#[test]
fn region_group_and_hint_freeze_propagation_matches_json() {
    let (_points, _angles, mut items) = fixture();
    items.push(items[0]);
    items[1].id = 1;
    for (i, item) in items.iter_mut().enumerate() {
        item.has_hint = 1;
        item.hint = CInputHint {
            item_id: i as i64,
            is_freeze: (i == 0) as i32,
            tr_x: i as f32 * 12.0,
            ..Default::default()
        };
    }
    let ids = [0, 1];
    let group = CInputPlaceGroup {
        id: 1,
        item_ids: ids.as_ptr(),
        item_count: 2,
    };
    let region = CInputRegion {
        id: b"A".as_ptr(),
        id_len: 1,
        kind: b"size_group".as_ptr(),
        kind_len: 10,
        item_ids: ids.as_ptr(),
        item_count: 2,
        gap_after_mm: 3.0,
    };
    let mut input = view(&items);
    input.mode = 1;
    input.regions = &region;
    input.region_count = 1;
    input.place_groups = &group;
    input.place_group_count = 1;
    let direct = unsafe { convert_input(&input) }.unwrap();
    assert_eq!(direct.hints[&1].is_freeze, 1);
    assert_eq!(direct.item_to_region[&1], 0);
    assert_eq!(direct.regions[0].gap_after_mm, 3.0);
    items[1].hint.item_id = 8;
    assert_eq!(
        unsafe { convert_input(&input) }.err().unwrap().code,
        grouped_input::InputErrorCode::InvalidHint
    );
}

#[test]
fn prepared_and_direct_inputs_create_usable_solver_handles() {
    let (_points, _angles, items) = fixture();
    let input = view(&items);
    let prepared = unsafe { swarm_input_from_struct(&input) };
    assert!(!prepared.is_null());
    let handle = unsafe { swarm_init_from_input(prepared, std::ptr::null()) };
    assert!(!handle.is_null());
    // V2 init already builds an initial layout. swarm_init_phase is Legacy-only.
    let result = swarm_get_solution_json(handle);
    assert!(!result.is_null());
    let result_text = unsafe { std::ffi::CStr::from_ptr(result) }
        .to_str()
        .unwrap();
    let doc: serde_json::Value = serde_json::from_str(result_text).unwrap();
    assert_eq!(
        doc["layout"]["placed_items"]
            .as_array()
            .unwrap()
            .len(),
        1
    );
    cpswcore::ffi::swarm_free_string(result);
    swarm_free(handle);
    let handle = unsafe { swarm_init_from_struct(&input, std::ptr::null()) };
    assert!(!handle.is_null());
    swarm_free(handle);
    let text = CString::new(json()).unwrap();
    let prepared = unsafe { swarm_input_from_json(text.as_ptr()) };
    assert!(!prepared.is_null());
    let handle = unsafe { swarm_init_from_input(prepared, std::ptr::null()) };
    assert!(!handle.is_null());
    swarm_free(handle);
}

#[test]
fn rectangle_rotation_reflection_quality_and_hint_are_preserved() {
    let axes = [90.0];
    let item = CInputItem {
        id: 0,
        demand: 1,
        shape_kind: 0,
        rotation_kind: 1,
        x_min: -2.0,
        y_min: 3.0,
        width: 10.0,
        height: 8.0,
        rotation_step: 90.0,
        reflection_axes: axes.as_ptr(),
        reflection_count: 1,
        min_quality: 2,
        has_hint: 1,
        hint: CInputHint {
            ver: 3,
            item_id: 0,
            tr_x: 15.0,
            tr_y: 5.0,
            stripe_interval_x: 2.0,
            stripe_interval_y: 4.0,
            ..Default::default()
        },
        ..Default::default()
    };
    let parsed = unsafe { convert_input(&view(&[item])) }.unwrap();
    let actual = serde_json::to_value(&parsed.instance.items[0]).unwrap();
    assert_eq!(
        actual["shape"],
        serde_json::json!({"type":"rectangle","data":{"x_min":-2.0,"y_min":3.0,"width":10.0,"height":8.0}})
    );
    assert_eq!(
        actual["orientation"],
        serde_json::json!({"rotation":{"mode":"stepped","step":90.0},"reflection_axes":[90.0]})
    );
    assert_eq!(actual["min_quality"], 2);
    assert_eq!(parsed.hints[&0].stripe_interval_y, 4.0);
    assert_eq!(parsed.hints[&0].ver, 3);
    let continuous = CInputItem {
        rotation_kind: 2,
        ..item
    };
    let parsed = unsafe { convert_input(&view(&[continuous])) }.unwrap();
    assert_eq!(
        serde_json::to_value(&parsed.instance.items[0]).unwrap()["orientation"]["rotation"]["mode"],
        "continuous"
    );
}

#[test]
fn invalid_json_utf8_hint_and_input_options_report_errors() {
    assert!(unsafe { swarm_input_from_json(std::ptr::null()) }.is_null());
    let bad_utf8 = [0xffu8, 0];
    assert!(unsafe { swarm_input_from_json(bad_utf8.as_ptr().cast()) }.is_null());
    let (_points, _angles, mut items) = fixture();
    items[0].has_hint = 1;
    items[0].hint.is_freeze = 2;
    assert_eq!(
        unsafe { convert_input(&view(&items)) }.err().unwrap().code,
        grouped_input::InputErrorCode::InvalidHint
    );
    items[0].hint.is_freeze = 0;
    items[0].hint.tr_x = f32::INFINITY;
    assert!(unsafe { convert_input(&view(&items)) }.is_err());
    items[0].has_hint = 0;
    let mut input = view(&items);
    input.group_workers = 0;
    assert_eq!(
        unsafe { convert_input(&input) }.err().unwrap().code,
        grouped_input::InputErrorCode::InvalidConfig
    );
    input.group_workers = 1;
    input.name = bad_utf8.as_ptr();
    input.name_len = 1;
    assert!(unsafe { convert_input(&input) }.is_err());
    input.struct_size -= 1;
    assert!(unsafe { swarm_input_from_struct(&input) }.is_null());
    assert!(unsafe { swarm_init_from_input(std::ptr::null_mut(), std::ptr::null()) }.is_null());
}

#[test]
fn x64_layout_matches_csharp_descriptor_contract() {
    if cfg!(target_pointer_width = "64") {
        assert_eq!(std::mem::size_of::<CInputHint>(), 40);
        assert_eq!(std::mem::size_of::<CInputItem>(), 144);
        assert_eq!(std::mem::size_of::<CSwarmInput>(), 104);
        assert_eq!(std::mem::offset_of!(CInputItem, hint), 104);
        assert_eq!(std::mem::offset_of!(CSwarmInput, regions), 72);
    }
}

#[test]
fn benchmark_fixtures_preserve_every_coordinate_and_orientation() {
    use jagua_rs::io::ext_repr::{ExtRotation, ExtShape};
    let mut paths: Vec<_> = ["shirts", "swim", "trousers", "wsjk156"]
        .into_iter()
        .map(|name| std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join(format!("data/input/{name}.json")))
        .collect();
    // Optional user-supplied fixture; never copy customer geometry into the repository.
    if let Some(path) = std::env::var_os("SWARM_INPUT_PARITY_FILE") {
        paths.push(path.into());
    }
    for path in paths {
        let text = std::fs::read_to_string(&path).unwrap();
        let (mut instance, _) = cpswcore::swarm_core::util::io::parse_spp_input(&text).unwrap();
        instance.items = instance
            .items
            .into_iter()
            .flat_map(|item| {
                (0..item.demand).map(move |_| {
                    let mut copy = item.clone();
                    copy.demand = 1;
                    copy
                })
            })
            .enumerate()
            .map(|(id, mut item)| {
                item.base.id = id as u64;
                item
            })
            .collect();
        let points: Vec<Vec<CInputPoint>> = instance
            .items
            .iter()
            .map(|item| match &item.base.shape {
                ExtShape::SimplePolygon(p) => {
                    p.0.iter().map(|&(x, y)| CInputPoint { x, y }).collect()
                }
                _ => panic!("fixture must contain simple polygons"),
            })
            .collect();
        let items: Vec<CInputItem> = instance
            .items
            .iter()
            .zip(&points)
            .map(|(item, p)| {
                let (rotation_kind, angles, angle_count, rotation_step) =
                    match &item.base.orientation.rotation {
                        ExtRotation::Discrete { angles } => (0, angles.as_ptr(), angles.len(), 0.0),
                        ExtRotation::Stepped { step } => (1, std::ptr::null(), 0, *step),
                        ExtRotation::Continuous {} => (2, std::ptr::null(), 0, 0.0),
                    };
                CInputItem {
                    id: item.base.id,
                    demand: 1,
                    shape_kind: 1,
                    vertices: p.as_ptr(),
                    vertex_count: p.len(),
                    rotation_kind,
                    angles,
                    angle_count,
                    rotation_step,
                    reflection_axes: item.base.orientation.reflection_axes.as_ptr(),
                    reflection_count: item.base.orientation.reflection_axes.len(),
                    min_quality: item.base.min_quality.map_or(-1, |q| q as i32),
                    ..Default::default()
                }
            })
            .collect();
        let input = CSwarmInput {
            name: instance.name.as_ptr(),
            name_len: instance.name.len(),
            strip_height: instance.strip_height,
            min_item_separation: instance.min_item_separation,
            ..view(&items)
        };
        let converted = unsafe { convert_input(&input) }.unwrap();
        let json = serde_json::json!({"schema_version":2,
            "nesting":{"mode":"all_items","group_workers":1,"quantum_ms":250}, "instance":instance})
        .to_string();
        let parsed = grouped_input::parse_and_validate(&json).unwrap();
        assert_eq!(
            serde_json::to_value(&converted.instance).unwrap(),
            serde_json::to_value(&parsed.instance).unwrap(),
            "{}", path.display()
        );
    }
}
