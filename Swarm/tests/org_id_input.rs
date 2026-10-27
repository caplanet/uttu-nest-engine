//! OrgID is input metadata. Native indexing, hints and region membership use id.
use cpswcore::ffi::{swarm_free_string, swarm_get_default_config, CSwarmConfig};
use cpswcore::ffi_step::{swarm_free, swarm_get_solution_json, swarm_init_from_json, CPlacedHint};
use serde_json::{json, Value};
use std::ffi::{CStr, CString};

fn solve(v2: bool, metadata: bool) -> Value {
    let mut items: Vec<Value> = (0..2).map(|id| json!({
        "id":id,"demand":1,"allowed_orientations":[0],
        "shape":{"type":"rectangle","data":{"x_min":0,"y_min":0,"width":4,"height":4}},
        "placed_hint":{"item_id":id,"is_freeze":1,"tr_x":10+id*10,"tr_y":10,"rot_angle_in_rad":0}
    })).collect();
    if metadata {
        items[0]["OrgID"] = json!(i64::MAX);
        items[1]["OrgID"] = json!(9_007_199_254_740_993_i64);
    }
    let instance = json!({"name":"original-identity","strip_height":30,"items":items});
    let input = if v2 { json!({"schema_version":2,
        "nesting":{"mode":"size_regions","group_workers":1,"quantum_ms":1000},
        "instance":instance,"regions":[{"id":"region","kind":"size_group","item_ids":[0,1]}],"place_groups":[]
    }) } else { instance };
    let text = CString::new(input.to_string()).unwrap();
    let hints: Vec<_> = (0..2).map(|id| CPlacedHint {
        item_id:id,is_freeze:1,tr_x:(10+id*10) as f32,tr_y:10.,rot_angle_in_rad:0.,stripe_interval_y:0.,stripe_interval_x:0.
    }).collect();
    let mut config: CSwarmConfig = unsafe { std::mem::zeroed() };
    swarm_get_default_config(&mut config);
    config.poly_simpl_tolerance = -1.;
    let handle = swarm_init_from_json(text.as_ptr(), &config,
        if v2 { std::ptr::null() } else { hints.as_ptr() }, if v2 {0} else {2});
    assert!(!handle.is_null(), "OrgID must not replace the sequential id");
    let ptr = swarm_get_solution_json(handle);
    if ptr.is_null() { swarm_free(handle); panic!("solution export failed"); }
    let result: Value = serde_json::from_str(unsafe {CStr::from_ptr(ptr)}.to_str().unwrap()).unwrap();
    swarm_free_string(ptr);
    swarm_free(handle);
    let mut placed = result["layout"]["placed_items"].as_array().unwrap().clone();
    placed.sort_by_key(|item| item["item_id"].as_u64().unwrap());
    assert_eq!(placed.len(), 2);
    assert_eq!(placed[0]["item_id"], 0);
    assert_eq!(placed[1]["item_id"], 1);
    json!(placed)
}

#[test]
fn legacy_org_id_metadata_preserves_native_ids_and_frozen_poses() {
    assert_eq!(solve(false, false), solve(false, true));
}

#[test]
fn grouped_org_id_metadata_preserves_region_ids_and_frozen_poses() {
    assert_eq!(solve(true, false), solve(true, true));
}
