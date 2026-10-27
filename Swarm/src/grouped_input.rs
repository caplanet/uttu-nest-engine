//! Versioned input contract for all-item and size-region nesting.
//!
//! The legacy Jagua [`ExtSPInstance`] representation intentionally remains
//! accepted by the public FFI.  This module handles only the explicit v2
//! envelope so that region/group fields can never be silently ignored by
//! Serde's default unknown-field behaviour.

use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use serde::Deserialize;
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::fmt::{Display, Formatter};

pub const GROUPED_SCHEMA_VERSION: u32 = 2;
const RECOMMENDED_MIN_QUANTUM_MS: u64 = 100;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NestingMode {
    AllItems,
    SizeRegions,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NestingOptions {
    pub mode: NestingMode,
    #[serde(default = "default_group_workers")]
    pub group_workers: usize,
    #[serde(default = "default_quantum_ms")]
    pub quantum_ms: u64,
    #[serde(default)]
    pub locked_prefix_gap_mm: f32,
}

fn default_group_workers() -> usize {
    std::thread::available_parallelism().map_or(1, |count| count.get())
}

const fn default_quantum_ms() -> u64 {
    1_000
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegionInput {
    pub id: String,
    pub kind: String,
    pub item_ids: Vec<u64>,
    #[serde(default)]
    pub gap_after_mm: f32,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlaceGroupInput {
    pub id: i64,
    pub member_item_ids: Vec<u64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlacedHintInput {
    #[serde(default)]
    pub ver: i32,
    pub item_id: i64,
    pub is_freeze: i32,
    pub tr_x: f32,
    pub tr_y: f32,
    pub rot_angle_in_rad: f32,
    #[serde(default)]
    pub stripe_interval_y: f32,
    #[serde(default)]
    pub stripe_interval_x: f32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EnvelopeWire {
    schema_version: u32,
    nesting: NestingOptions,
    instance: Value,
    #[serde(default)]
    regions: Vec<RegionInput>,
    #[serde(default)]
    place_groups: Vec<PlaceGroupInput>,
}

/// Fully validated v2 contract.  `hints` contains the lock propagation result:
/// if one member of a place group was frozen, every member is frozen here.
pub struct ValidatedInput {
    pub nesting: NestingOptions,
    pub instance: ExtSPInstance,
    pub regions: Vec<RegionInput>,
    pub place_groups: Vec<PlaceGroupInput>,
    pub hints: HashMap<u64, PlacedHintInput>,
    pub item_to_region: HashMap<u64, usize>,
}

impl ValidatedInput {
    pub fn frozen_item_ids(&self) -> HashSet<u64> {
        self.hints
            .iter()
            .filter_map(|(&id, hint)| (hint.is_freeze != 0).then_some(id))
            .collect()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InputErrorCode {
    NotV2,
    UnsupportedSchema,
    InvalidJson,
    InvalidConfig,
    InvalidItem,
    InvalidHint,
    InvalidRegion,
    RegionItemDuplicated,
    RegionItemUnassigned,
    InvalidPlaceGroup,
    PlaceGroupCrossesRegions,
    PlaceGroupMemberPoseMissing,
}

#[derive(Debug, Clone)]
pub struct InputError {
    pub code: InputErrorCode,
    pub message: String,
}

impl InputError {
    pub(crate) fn new(code: InputErrorCode, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }
}

impl Display for InputError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}: {}", self.code, self.message)
    }
}

impl std::error::Error for InputError {}

/// Cheap discriminator used by the FFI before trying the legacy parser.
pub fn is_v2_envelope(value: &str) -> bool {
    #[cfg(test)]
    crate::ffi_step::issue9_count_input_work(false);
    serde_json::from_str::<Value>(value)
        .ok()
        .is_some_and(|v| is_v2_value(&v))
}

pub(crate) fn is_v2_value(value: &Value) -> bool {
    value.get("schema_version").and_then(Value::as_u64).is_some()
}

pub fn parse_and_validate(json: &str) -> Result<ValidatedInput, InputError> {
    #[cfg(test)]
    crate::ffi_step::issue9_count_input_work(false);
    let value: Value = serde_json::from_str(json)
        .map_err(|e| InputError::new(InputErrorCode::InvalidJson, e.to_string()))?;
    parse_and_validate_value(value)
}

pub(crate) fn parse_and_validate_value(value: Value) -> Result<ValidatedInput, InputError> {
    let Some(schema_version) = value.get("schema_version").and_then(Value::as_u64) else {
        return Err(InputError::new(
            InputErrorCode::NotV2,
            "schema_version is missing",
        ));
    };
    if schema_version != GROUPED_SCHEMA_VERSION as u64 {
        return Err(InputError::new(
            InputErrorCode::UnsupportedSchema,
            format!("unsupported schema_version {schema_version}"),
        ));
    }

    let wire: EnvelopeWire = serde_json::from_value(value)
        .map_err(|e| InputError::new(InputErrorCode::InvalidJson, e.to_string()))?;
    debug_assert_eq!(wire.schema_version, GROUPED_SCHEMA_VERSION);

    validate_options(&wire.nesting)?;
    let instance: ExtSPInstance = serde_json::from_value(wire.instance.clone())
        .map_err(|e| InputError::new(InputErrorCode::InvalidItem, e.to_string()))?;
    validate_instance(&instance)?;
    let hints = parse_hints(&wire.instance, &instance)?;
    finish_validation(wire.nesting, instance, wire.regions, wire.place_groups, hints)
}

/// Direct input uses the same semantic checks as the JSON contract.
pub(crate) fn validate_typed(
    nesting: NestingOptions,
    instance: ExtSPInstance,
    regions: Vec<RegionInput>,
    place_groups: Vec<PlaceGroupInput>,
    hints: HashMap<u64, PlacedHintInput>,
) -> Result<ValidatedInput, InputError> {
    validate_options(&nesting)?;
    validate_instance(&instance)?;
    for (&id, hint) in &hints {
        if id >= instance.items.len() as u64 {
            return Err(InputError::new(InputErrorCode::InvalidHint, "hint references missing item"));
        }
        validate_hint(id, hint)?;
    }
    finish_validation(nesting, instance, regions, place_groups, hints)
}

fn finish_validation(
    nesting: NestingOptions,
    instance: ExtSPInstance,
    regions: Vec<RegionInput>,
    place_groups: Vec<PlaceGroupInput>,
    mut hints: HashMap<u64, PlacedHintInput>,
) -> Result<ValidatedInput, InputError> {
    let item_ids: HashSet<u64> = instance.items.iter().map(|item| item.base.id).collect();
    validate_place_groups(&place_groups, &item_ids, &mut hints)?;
    let item_to_region = validate_regions(
        nesting.mode,
        &regions,
        &place_groups,
        &item_ids,
    )?;

    Ok(ValidatedInput {
        nesting,
        instance,
        regions,
        place_groups,
        hints,
        item_to_region,
    })
}

fn validate_options(options: &NestingOptions) -> Result<(), InputError> {
    if options.group_workers == 0 {
        return Err(InputError::new(
            InputErrorCode::InvalidConfig,
            "group_workers must be at least 1",
        ));
    }
    if options.quantum_ms == 0 || options.quantum_ms > 60_000 {
        return Err(InputError::new(
            InputErrorCode::InvalidConfig,
            "quantum_ms must be in 1..=60000",
        ));
    }
    if options.quantum_ms < RECOMMENDED_MIN_QUANTUM_MS {
        log::warn!(
            "grouped quantum_ms={} is below the recommended {}ms minimum and may prevent a lane from completing useful work",
            options.quantum_ms,
            RECOMMENDED_MIN_QUANTUM_MS
        );
    }
    if !options.locked_prefix_gap_mm.is_finite() || options.locked_prefix_gap_mm < 0.0 {
        return Err(InputError::new(
            InputErrorCode::InvalidConfig,
            "locked_prefix_gap_mm must be finite and non-negative",
        ));
    }
    Ok(())
}

fn validate_instance(instance: &ExtSPInstance) -> Result<(), InputError> {
    if !instance.strip_height.is_finite() || instance.strip_height <= 0.0 {
        return Err(InputError::new(
            InputErrorCode::InvalidItem,
            "strip_height must be finite and positive",
        ));
    }
    if instance.items.is_empty() {
        return Err(InputError::new(
            InputErrorCode::InvalidItem,
            "instance must contain at least one item",
        ));
    }
    for (expected, item) in instance.items.iter().enumerate() {
        if item.base.id != expected as u64 {
            return Err(InputError::new(
                InputErrorCode::InvalidItem,
                format!(
                    "v2 item ids must be consecutive from 0; expected {expected}, got {}",
                    item.base.id
                ),
            ));
        }
        if item.demand != 1 {
            return Err(InputError::new(
                InputErrorCode::InvalidItem,
                format!(
                    "v2 requires demand=1 for placement identity; item {} has demand {}",
                    item.base.id, item.demand
                ),
            ));
        }
    }
    Ok(())
}

fn parse_hints(
    raw_instance: &Value,
    instance: &ExtSPInstance,
) -> Result<HashMap<u64, PlacedHintInput>, InputError> {
    let raw_items = raw_instance
        .get("items")
        .and_then(Value::as_array)
        .ok_or_else(|| InputError::new(InputErrorCode::InvalidItem, "instance.items is missing"))?;
    if raw_items.len() != instance.items.len() {
        return Err(InputError::new(
            InputErrorCode::InvalidItem,
            "raw and parsed item counts differ",
        ));
    }

    let mut hints = HashMap::new();
    for (item, raw_item) in instance.items.iter().zip(raw_items) {
        let Some(raw_hint) = raw_item.get("placed_hint") else {
            continue;
        };
        if raw_hint.is_null() {
            continue;
        }
        let hint: PlacedHintInput = serde_json::from_value(raw_hint.clone()).map_err(|e| {
            InputError::new(
                InputErrorCode::InvalidHint,
                format!("item {}: {e}", item.base.id),
            )
        })?;
        validate_hint(item.base.id, &hint)?;
        hints.insert(item.base.id, hint);
    }
    Ok(hints)
}

fn validate_hint(item_id: u64, hint: &PlacedHintInput) -> Result<(), InputError> {
    if hint.item_id < 0 || hint.item_id as u64 != item_id {
        return Err(InputError::new(
            InputErrorCode::InvalidHint,
            format!("item {} contains a hint for item {}", item_id, hint.item_id),
        ));
    }
    if hint.is_freeze != 0 && hint.is_freeze != 1 {
        return Err(InputError::new(
            InputErrorCode::InvalidHint,
            format!("item {} has is_freeze={}, expected 0 or 1", item_id, hint.is_freeze),
        ));
    }
    if ![
        hint.tr_x,
        hint.tr_y,
        hint.rot_angle_in_rad,
        hint.stripe_interval_y,
        hint.stripe_interval_x,
    ]
    .into_iter()
    .all(f32::is_finite)
    {
        return Err(InputError::new(
            InputErrorCode::InvalidHint,
            format!("item {} has a non-finite hint value", item_id),
        ));
    }
    Ok(())
}

fn validate_place_groups(
    groups: &[PlaceGroupInput],
    item_ids: &HashSet<u64>,
    hints: &mut HashMap<u64, PlacedHintInput>,
) -> Result<(), InputError> {
    let mut group_ids = HashSet::new();
    let mut grouped_items = HashSet::new();
    for group in groups {
        if group.id <= 0 || !group_ids.insert(group.id) {
            return Err(InputError::new(
                InputErrorCode::InvalidPlaceGroup,
                format!("place group id {} must be positive and unique", group.id),
            ));
        }
        if group.member_item_ids.len() < 2 {
            return Err(InputError::new(
                InputErrorCode::InvalidPlaceGroup,
                format!("place group {} must contain at least two members", group.id),
            ));
        }
        let mut local = HashSet::new();
        for &item_id in &group.member_item_ids {
            if !item_ids.contains(&item_id) {
                return Err(InputError::new(
                    InputErrorCode::InvalidPlaceGroup,
                    format!("place group {} references missing item {item_id}", group.id),
                ));
            }
            if !local.insert(item_id) || !grouped_items.insert(item_id) {
                return Err(InputError::new(
                    InputErrorCode::InvalidPlaceGroup,
                    format!("item {item_id} occurs in more than one place group"),
                ));
            }
        }

        let freeze_group = group
            .member_item_ids
            .iter()
            .any(|id| hints.get(id).is_some_and(|hint| hint.is_freeze != 0));
        if freeze_group {
            for &item_id in &group.member_item_ids {
                let Some(hint) = hints.get_mut(&item_id) else {
                    return Err(InputError::new(
                        InputErrorCode::PlaceGroupMemberPoseMissing,
                        format!(
                            "frozen place group {} member {item_id} has no placed_hint",
                            group.id
                        ),
                    ));
                };
                hint.is_freeze = 1;
            }
        } else if let Some(item_id) = group
            .member_item_ids
            .iter()
            .find(|id| !hints.contains_key(id))
        {
            return Err(InputError::new(
                InputErrorCode::PlaceGroupMemberPoseMissing,
                format!(
                    "movable place group {} member {item_id} has no placed_hint",
                    group.id
                ),
            ));
        }
    }
    Ok(())
}

fn validate_regions(
    mode: NestingMode,
    regions: &[RegionInput],
    groups: &[PlaceGroupInput],
    item_ids: &HashSet<u64>,
) -> Result<HashMap<u64, usize>, InputError> {
    if mode == NestingMode::AllItems {
        if !regions.is_empty() {
            return Err(InputError::new(
                InputErrorCode::InvalidRegion,
                "all_items mode must not contain regions",
            ));
        }
        return Ok(item_ids.iter().map(|&id| (id, 0)).collect());
    }
    if regions.is_empty() {
        return Err(InputError::new(
            InputErrorCode::InvalidRegion,
            "size_regions mode requires at least one region",
        ));
    }

    let mut region_ids = HashSet::new();
    let mut item_to_region = HashMap::new();
    for (region_index, region) in regions.iter().enumerate() {
        if region.id.trim().is_empty() || !region_ids.insert(region.id.as_str()) {
            return Err(InputError::new(
                InputErrorCode::InvalidRegion,
                format!("region id {:?} is empty or duplicated", region.id),
            ));
        }
        if !matches!(
            region.kind.as_str(),
            "size_group" | "size_bundle" | "remainder"
        ) {
            return Err(InputError::new(
                InputErrorCode::InvalidRegion,
                format!(
                    "region {} has unsupported kind {:?}",
                    region.id, region.kind
                ),
            ));
        }
        if region.item_ids.is_empty() {
            return Err(InputError::new(
                InputErrorCode::InvalidRegion,
                format!("region {} is empty", region.id),
            ));
        }
        if !region.gap_after_mm.is_finite() || region.gap_after_mm < 0.0 {
            return Err(InputError::new(
                InputErrorCode::InvalidRegion,
                format!("region {} has invalid gap_after_mm", region.id),
            ));
        }
        for &item_id in &region.item_ids {
            if !item_ids.contains(&item_id) {
                return Err(InputError::new(
                    InputErrorCode::InvalidRegion,
                    format!("region {} references missing item {item_id}", region.id),
                ));
            }
            if item_to_region.insert(item_id, region_index).is_some() {
                return Err(InputError::new(
                    InputErrorCode::RegionItemDuplicated,
                    format!("item {item_id} occurs in more than one region"),
                ));
            }
        }
    }

    if item_to_region.len() != item_ids.len() {
        let missing = item_ids
            .iter()
            .filter(|id| !item_to_region.contains_key(id))
            .copied()
            .collect::<Vec<_>>();
        return Err(InputError::new(
            InputErrorCode::RegionItemUnassigned,
            format!("items are not assigned to a region: {missing:?}"),
        ));
    }

    for group in groups {
        let first = item_to_region[&group.member_item_ids[0]];
        if group
            .member_item_ids
            .iter()
            .any(|item_id| item_to_region[item_id] != first)
        {
            return Err(InputError::new(
                InputErrorCode::PlaceGroupCrossesRegions,
                format!("place group {} spans multiple size regions", group.id),
            ));
        }
    }
    Ok(item_to_region)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn issue9_snapshot(input: ValidatedInput) -> Value {
        let mut hints = input.hints.iter().map(|(&id, h)| (id, h.ver, h.item_id,
            h.is_freeze, [h.tr_x.to_bits(), h.tr_y.to_bits(), h.rot_angle_in_rad.to_bits(),
            h.stripe_interval_y.to_bits(), h.stripe_interval_x.to_bits()])).collect::<Vec<_>>();
        hints.sort_by_key(|h| h.0);
        let mut frozen = input.frozen_item_ids().into_iter().collect::<Vec<_>>(); frozen.sort();
        let mut membership = input.item_to_region.into_iter().collect::<Vec<_>>(); membership.sort();
        serde_json::json!({"instance":input.instance,
            "nesting":[format!("{:?}", input.nesting.mode), input.nesting.group_workers.to_string(),
                input.nesting.quantum_ms.to_string(), input.nesting.locked_prefix_gap_mm.to_bits().to_string()],
            "regions":input.regions.iter().map(|r| (&r.id,&r.kind,&r.item_ids,r.gap_after_mm.to_bits())).collect::<Vec<_>>(),
            "groups":input.place_groups.iter().map(|g| (g.id,&g.member_item_ids)).collect::<Vec<_>>(),
            "hints":hints,"frozen":frozen,"membership":membership})
    }

    #[test]
    fn issue9_owned_value_and_text_validation_parity() {
        use InputErrorCode::*;
        let base: Value = serde_json::from_str(&envelope(serde_json::json!([
            {"id":"g1","kind":"size_group","item_ids":[0,1],"gap_after_mm":10.0},
            {"id":"rest","kind":"remainder","item_ids":[2],"gap_after_mm":0.0}
        ]), serde_json::json!([{"id":7,"member_item_ids":[0,1]}]))).unwrap();
        let mut cases = vec![("size regions partial lock", base.clone(), None)];
        let mut all = base.clone(); all["nesting"]["mode"] = serde_json::json!("all_items"); all["regions"] = serde_json::json!([]);
        all["instance"]["items"][1]["placed_hint"]["tr_x"] = serde_json::json!(5.0);
        cases.push(("all items overlapping locked group", all, None));
        // Mutations pin validation ordering and expected error families; the two
        // entrypoints must also retain the exact current message.
        let mut add = |label, pointer: &str, value: Value, code| {
            let mut v = base.clone(); *v.pointer_mut(pointer).unwrap() = value;
            cases.push((label, v, Some(code)));
        };
        for version in [0_u64, 1, 3, u64::MAX] { add("unsupported integer", "/schema_version", serde_json::json!(version), UnsupportedSchema); }
        for version in [Value::Null, serde_json::json!(2.0), serde_json::json!(2.5), serde_json::json!(-1), serde_json::json!("2")] {
            add("non unsigned discriminator", "/schema_version", version, NotV2);
        }
        add("workers zero", "/nesting/group_workers", serde_json::json!(0), InvalidConfig);
        for quantum in [0, 60001] { add("invalid quantum", "/nesting/quantum_ms", serde_json::json!(quantum), InvalidConfig); }
        add("negative prefix gap", "/nesting/locked_prefix_gap_mm", serde_json::json!(-1), InvalidConfig);
        add("invalid demand", "/instance/items/0/demand", serde_json::json!(2), InvalidItem);
        add("invalid id", "/instance/items/0/id", serde_json::json!(9), InvalidItem);
        add("invalid hint id", "/instance/items/0/placed_hint/item_id", serde_json::json!(8), InvalidHint);
        add("invalid freeze", "/instance/items/0/placed_hint/is_freeze", serde_json::json!(2), InvalidHint);
        add("negative region gap", "/regions/0/gap_after_mm", serde_json::json!(-1), InvalidRegion);
        add("duplicate region id", "/regions/1/id", serde_json::json!("g1"), InvalidRegion);
        add("duplicate assignment", "/regions/1/item_ids", serde_json::json!([0,2]), RegionItemDuplicated);
        add("one missing assignment", "/regions", serde_json::json!([{ "id":"g1", "kind":"size_group", "item_ids":[0,1], "gap_after_mm":10.0 }]), RegionItemUnassigned);
        add("negative group id", "/place_groups/0/id", serde_json::json!(-7), InvalidPlaceGroup);
        add("missing member pose", "/instance/items/1/placed_hint", Value::Null, PlaceGroupMemberPoseMissing);
        add("crossing regions", "/regions", serde_json::json!([
            {"id":"g1","kind":"size_group","item_ids":[0]}, {"id":"rest","kind":"remainder","item_ids":[1,2]}]), PlaceGroupCrossesRegions);
        drop(add);
        let mut missing = base.clone(); missing.as_object_mut().unwrap().remove("schema_version");
        cases.push(("missing discriminator", missing, Some(NotV2)));
        for (label, pointer, code) in [("unknown root", "", InvalidJson), ("unknown nesting", "/nesting", InvalidJson),
            ("unknown region", "/regions/0", InvalidJson), ("unknown group", "/place_groups/0", InvalidJson),
            ("unknown hint", "/instance/items/0/placed_hint", InvalidHint)] {
            let mut v = base.clone(); v.pointer_mut(pointer).unwrap().as_object_mut().unwrap().insert("unknown".into(), serde_json::json!(true));
            cases.push((label, v, Some(code)));
        }
        for (label, value, expected) in cases {
            let text = value.to_string();
            assert_eq!(is_v2_envelope(&text), is_v2_value(&value), "{label}");
            assert_eq!(is_v2_value(&value), expected != Some(NotV2), "{label}");
            match (parse_and_validate(&text), parse_and_validate_value(value)) {
                (Ok(a), Ok(b)) => {
                    assert!(expected.is_none(), "{label}");
                    assert_eq!(a.frozen_item_ids(), HashSet::from([0,1]), "{label}");
                    assert_eq!(issue9_snapshot(a), issue9_snapshot(b), "{label}");
                }
                (Err(a), Err(b)) => { assert_eq!(Some(a.code.clone()), expected, "{label}"); assert_eq!(a.code,b.code,"{label}"); assert_eq!(a.message,b.message,"{label}"); }
                _ => panic!("text/Value mismatch: {label}"),
            }
        }
    }
    fn item(id: u64, hint: Value) -> Value {
        let mut value = serde_json::json!({
            "id": id,
            "demand": 1,
            "allowed_orientations": [0.0, 180.0],
            "shape": {"type":"rectangle", "data":{"x_min":0.0,"y_min":0.0,"width":10.0,"height":5.0}}
        });
        if !hint.is_null() {
            value["placed_hint"] = hint;
        }
        value
    }

    fn hint(id: u64, frozen: bool, x: f32) -> Value {
        serde_json::json!({
            "item_id": id,
            "is_freeze": if frozen {1} else {0},
            "tr_x": x,
            "tr_y": 0.0,
            "rot_angle_in_rad": 0.0,
            "stripe_interval_y": 0.0,
            "stripe_interval_x": 0.0
        })
    }

    fn envelope(regions: Value, groups: Value) -> String {
        serde_json::json!({
            "schema_version": 2,
            "nesting": {"mode":"size_regions","group_workers":1,"quantum_ms":50,"locked_prefix_gap_mm":10.0},
            "instance": {"name":"t","strip_height":100.0,"items":[
                item(0, hint(0, true, 5.0)), item(1, hint(1, false, 20.0)), item(2, Value::Null)
            ]},
            "regions": regions,
            "place_groups": groups
        })
        .to_string()
    }

    #[test]
    fn omitted_scheduler_fields_use_available_parallelism_and_one_second_quantum() {
        let json = envelope(
            serde_json::json!([
                {"id":"g1","kind":"size_group","item_ids":[0,1],"gap_after_mm":10.0},
                {"id":"rest","kind":"remainder","item_ids":[2],"gap_after_mm":0.0}
            ]),
            serde_json::json!([]),
        );
        let mut root: Value = serde_json::from_str(&json).unwrap();
        let nesting = root["nesting"].as_object_mut().unwrap();
        nesting.remove("group_workers");
        nesting.remove("quantum_ms");

        let parsed = parse_and_validate(&root.to_string()).unwrap();
        let expected_workers = std::thread::available_parallelism().map_or(1, |count| count.get());

        assert_eq!(parsed.nesting.group_workers, expected_workers);
        assert_eq!(parsed.nesting.quantum_ms, 1_000);
    }

    #[test]
    fn explicit_legacy_scheduler_values_are_preserved() {
        let json = envelope(
            serde_json::json!([
                {"id":"g1","kind":"size_group","item_ids":[0,1],"gap_after_mm":10.0},
                {"id":"rest","kind":"remainder","item_ids":[2],"gap_after_mm":0.0}
            ]),
            serde_json::json!([]),
        );

        let parsed = parse_and_validate(&json).unwrap();

        assert_eq!(parsed.nesting.group_workers, 1);
        assert_eq!(parsed.nesting.quantum_ms, 50);
    }

    #[test]
    fn partial_lock_propagates_to_the_whole_place_group() {
        let json = envelope(
            serde_json::json!([
                {"id":"g1","kind":"size_group","item_ids":[0,1],"gap_after_mm":10.0},
                {"id":"rest","kind":"remainder","item_ids":[2],"gap_after_mm":0.0}
            ]),
            serde_json::json!([{"id":7,"member_item_ids":[0,1]}]),
        );
        let parsed = parse_and_validate(&json).unwrap();
        assert_eq!(parsed.frozen_item_ids(), HashSet::from([0, 1]));
    }

    #[test]
    fn a_place_group_spanning_regions_is_rejected() {
        let json = envelope(
            serde_json::json!([
                {"id":"g1","kind":"size_group","item_ids":[0],"gap_after_mm":10.0},
                {"id":"rest","kind":"remainder","item_ids":[1,2],"gap_after_mm":0.0}
            ]),
            serde_json::json!([{"id":7,"member_item_ids":[0,1]}]),
        );
        let error = parse_and_validate(&json).err().unwrap();
        assert_eq!(error.code, InputErrorCode::PlaceGroupCrossesRegions);
    }

    #[test]
    fn a_negative_place_group_id_is_rejected() {
        let json = envelope(
            serde_json::json!([
                {"id":"g1","kind":"size_group","item_ids":[0,1],"gap_after_mm":10.0},
                {"id":"rest","kind":"remainder","item_ids":[2],"gap_after_mm":0.0}
            ]),
            serde_json::json!([{"id":-7,"member_item_ids":[0,1]}]),
        );
        let error = parse_and_validate(&json).err().unwrap();
        assert_eq!(error.code, InputErrorCode::InvalidPlaceGroup);
        assert!(error.message.contains("positive"));
    }

    #[test]
    fn every_item_must_belong_to_exactly_one_region() {
        let json = envelope(
            serde_json::json!([
                {"id":"g1","kind":"size_group","item_ids":[0,1],"gap_after_mm":0.0}
            ]),
            serde_json::json!([{"id":7,"member_item_ids":[0,1]}]),
        );
        let error = parse_and_validate(&json).err().unwrap();
        assert_eq!(error.code, InputErrorCode::RegionItemUnassigned);
    }
}
