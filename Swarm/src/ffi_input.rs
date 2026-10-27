//! Additive, versioned C ABI for V2 input without JSON.
//!
//! Conversion ends at owned `ValidatedInput`, before geometry import or solving.
//! All arrays/UTF-8 strings are borrowed only for the duration of conversion.
use crate::ffi::{CSwarmConfig, DEFAULT_CALLBACK_OPTIONS};
use crate::ffi_step::{self, SwarmStepHandle};
use crate::grouped_input::{
    self, InputError, InputErrorCode, NestingMode, NestingOptions, PlaceGroupInput,
    PlacedHintInput, RegionInput, ValidatedInput,
};
use jagua_rs::io::ext_repr::{ExtItem, ExtOrientation, ExtRotation, ExtSPolygon, ExtShape};
use jagua_rs::probs::spp::io::ext_repr::{ExtItem as SPItem, ExtSPInstance};
use std::collections::HashMap;
use std::ffi::{CStr, c_char};

pub const SWARM_INPUT_ABI_VERSION: u32 = 1;

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct CInputPoint {
    pub x: f32,
    pub y: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct CInputHint {
    pub ver: i32,
    pub item_id: i64,
    pub is_freeze: i32,
    pub tr_x: f32,
    pub tr_y: f32,
    pub rot_angle_in_rad: f32,
    pub stripe_interval_y: f32,
    pub stripe_interval_x: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct CInputItem {
    pub id: u64,
    pub demand: u64,
    /// 0 = rectangle, 1 = simple polygon. Other kinds are rejected.
    pub shape_kind: u32,
    /// 0 = discrete, 1 = stepped, 2 = continuous (angles in degrees).
    pub rotation_kind: u32,
    pub x_min: f32,
    pub y_min: f32,
    pub width: f32,
    pub height: f32,
    pub vertices: *const CInputPoint,
    pub vertex_count: usize,
    pub angles: *const f32,
    pub angle_count: usize,
    pub rotation_step: f32,
    pub reflection_axes: *const f32,
    pub reflection_count: usize,
    /// -1 = no minimum quality override.
    pub min_quality: i32,
    /// 0 = absent, 1 = present.
    pub has_hint: u32,
    pub hint: CInputHint,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct CInputRegion {
    pub id: *const u8,
    pub id_len: usize,
    pub kind: *const u8,
    pub kind_len: usize,
    pub item_ids: *const u64,
    pub item_count: usize,
    pub gap_after_mm: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct CInputPlaceGroup {
    pub id: i64,
    pub item_ids: *const u64,
    pub item_count: usize,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct CSwarmInput {
    pub abi_version: u32,
    pub struct_size: u32,
    pub name: *const u8,
    pub name_len: usize,
    pub strip_height: f32,
    pub min_item_separation: f32,
    pub items: *const CInputItem,
    pub item_count: usize,
    /// 0 = all_items, 1 = size_regions.
    pub mode: u32,
    pub group_workers: u32,
    pub quantum_ms: u64,
    pub locked_prefix_gap_mm: f32,
    pub regions: *const CInputRegion,
    pub region_count: usize,
    pub place_groups: *const CInputPlaceGroup,
    pub place_group_count: usize,
}

/// Opaque, owned input. This is NOT a solver handle.
pub struct SwarmInput {
    input: ValidatedInput,
}

fn invalid(message: impl Into<String>) -> InputError {
    InputError::new(InputErrorCode::InvalidItem, message)
}

// These checks reject malformed descriptors; they cannot verify foreign allocations.
unsafe fn slice<'a, T>(ptr: *const T, len: usize) -> Result<&'a [T], InputError> {
    if len == 0 {
        return Ok(&[]);
    }
    if ptr.is_null()
        || !(ptr as usize).is_multiple_of(std::mem::align_of::<T>())
        || len > isize::MAX as usize / std::mem::size_of::<T>()
    {
        return Err(invalid("null, misaligned, or overflowing input array"));
    }
    Ok(unsafe { std::slice::from_raw_parts(ptr, len) })
}

unsafe fn string(ptr: *const u8, len: usize) -> Result<String, InputError> {
    let bytes = unsafe { slice(ptr, len)? };
    std::str::from_utf8(bytes)
        .map(str::to_owned)
        .map_err(|_| invalid("invalid UTF-8 input string"))
}

fn finite(values: &[f32]) -> Result<(), InputError> {
    if values.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(invalid("non-finite input number"))
    }
}

/// Copies C descriptors into the same validated input used by V2 JSON.
/// Geometry import, simplification and initial placement are deliberately later.
///
/// # Safety
/// Every nonempty array/string must point to readable, aligned storage of the
/// declared length, stable throughout this call. The root must be a full V1 struct.
pub unsafe fn convert_input(input: &CSwarmInput) -> Result<ValidatedInput, InputError> {
    if input.abi_version != SWARM_INPUT_ABI_VERSION
        || input.struct_size as usize != std::mem::size_of::<CSwarmInput>()
    {
        return Err(InputError::new(
            InputErrorCode::UnsupportedSchema,
            "unsupported input ABI version or size",
        ));
    }
    let mode = match input.mode {
        0 => NestingMode::AllItems,
        1 => NestingMode::SizeRegions,
        _ => {
            return Err(InputError::new(
                InputErrorCode::InvalidConfig,
                "invalid nesting mode",
            ));
        }
    };
    finite(&[input.min_item_separation])?;
    let mut items = Vec::new();
    let mut hints = HashMap::new();
    let source = unsafe { slice(input.items, input.item_count)? };
    items.reserve(source.len());
    for item in source {
        let shape = match item.shape_kind {
            0 => {
                finite(&[item.x_min, item.y_min, item.width, item.height])?;
                ExtShape::Rectangle {
                    x_min: item.x_min,
                    y_min: item.y_min,
                    width: item.width,
                    height: item.height,
                }
            }
            1 => {
                let points = unsafe { slice(item.vertices, item.vertex_count)? };
                let vertices = points
                    .iter()
                    .map(|p| {
                        finite(&[p.x, p.y])?;
                        Ok((p.x, p.y))
                    })
                    .collect::<Result<Vec<_>, InputError>>()?;
                ExtShape::SimplePolygon(ExtSPolygon(vertices))
            }
            _ => {
                return Err(invalid(
                    "unsupported shape kind; use rectangle or simple polygon",
                ));
            }
        };
        let rotation = match item.rotation_kind {
            0 => {
                let angles = unsafe { slice(item.angles, item.angle_count)? };
                finite(angles)?;
                ExtRotation::Discrete {
                    angles: angles.to_vec(),
                }
            }
            1 => {
                finite(&[item.rotation_step])?;
                ExtRotation::Stepped {
                    step: item.rotation_step,
                }
            }
            2 => ExtRotation::Continuous {},
            _ => return Err(invalid("unsupported rotation kind")),
        };
        let axes = unsafe { slice(item.reflection_axes, item.reflection_count)? };
        finite(axes)?;
        if item.min_quality < -1 {
            return Err(invalid("min_quality must be -1 or nonnegative"));
        }
        items.push(SPItem {
            base: ExtItem {
                id: item.id,
                shape,
                orientation: ExtOrientation {
                    rotation,
                    reflection_axes: axes.to_vec(),
                },
                min_quality: (item.min_quality >= 0).then_some(item.min_quality as usize),
            },
            demand: item.demand,
        });
        match item.has_hint {
            0 => {}
            1 => {
                let h = item.hint;
                hints.insert(
                    item.id,
                    PlacedHintInput {
                        ver: h.ver,
                        item_id: h.item_id,
                        is_freeze: h.is_freeze,
                        tr_x: h.tr_x,
                        tr_y: h.tr_y,
                        rot_angle_in_rad: h.rot_angle_in_rad,
                        stripe_interval_y: h.stripe_interval_y,
                        stripe_interval_x: h.stripe_interval_x,
                    },
                );
            }
            _ => {
                return Err(InputError::new(
                    InputErrorCode::InvalidHint,
                    "has_hint must be 0 or 1",
                ));
            }
        }
    }
    let regions = unsafe { slice(input.regions, input.region_count)? }
        .iter()
        .map(|r| {
            Ok(RegionInput {
                id: unsafe { string(r.id, r.id_len)? },
                kind: unsafe { string(r.kind, r.kind_len)? },
                item_ids: unsafe { slice(r.item_ids, r.item_count)? }.to_vec(),
                gap_after_mm: r.gap_after_mm,
            })
        })
        .collect::<Result<Vec<_>, InputError>>()?;
    let groups = unsafe { slice(input.place_groups, input.place_group_count)? }
        .iter()
        .map(|g| {
            Ok(PlaceGroupInput {
                id: g.id,
                member_item_ids: unsafe { slice(g.item_ids, g.item_count)? }.to_vec(),
            })
        })
        .collect::<Result<Vec<_>, InputError>>()?;
    grouped_input::validate_typed(
        NestingOptions {
            mode,
            group_workers: input.group_workers as usize,
            quantum_ms: input.quantum_ms,
            locked_prefix_gap_mm: input.locked_prefix_gap_mm,
        },
        ExtSPInstance {
            name: unsafe { string(input.name, input.name_len)? },
            strip_height: input.strip_height,
            min_item_separation: input.min_item_separation,
            items,
        },
        regions,
        groups,
        hints,
    )
}

fn prepare(f: impl FnOnce() -> Result<ValidatedInput, InputError>) -> *mut SwarmInput {
    ffi_step::clear_last_error();
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(Ok(input)) => Box::into_raw(Box::new(SwarmInput { input })),
        Ok(Err(e)) => {
            ffi_step::set_last_error(ffi_step::grouped_input_error_code(&e.code), e.to_string());
            std::ptr::null_mut()
        }
        Err(_) => {
            ffi_step::set_last_error(9000, "panic while preparing input");
            std::ptr::null_mut()
        }
    }
}

/// Prepare V2 JSON only, stopping before geometry import. Legacy JSON init stays unchanged.
/// # Safety
/// `json` must be null or a valid NUL-terminated UTF-8 string, readable for this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn swarm_input_from_json(json: *const c_char) -> *mut SwarmInput {
    prepare(|| {
        if json.is_null() {
            return Err(InputError::new(InputErrorCode::InvalidJson, "null JSON"));
        }
        let text = unsafe { CStr::from_ptr(json) }
            .to_str()
            .map_err(|_| InputError::new(InputErrorCode::InvalidJson, "invalid UTF-8 JSON"))?;
        grouped_input::parse_and_validate(text)
    })
}

/// Prepare direct V2 input. Caller memory can be released immediately after return.
/// # Safety
/// `input` must be null or a readable, aligned V1 struct, with buffers as in `convert_input`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn swarm_input_from_struct(input: *const CSwarmInput) -> *mut SwarmInput {
    prepare(|| {
        let root = unsafe { slice(input, 1)? };
        unsafe { convert_input(&root[0]) }
    })
}

/// Free a prepared input that has not been consumed by `swarm_init_from_input`.
/// # Safety
/// `input` must be null or a live pointer returned by a preparation function, freed once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn swarm_input_free(input: *mut SwarmInput) {
    if !input.is_null() {
        drop(unsafe { Box::from_raw(input) });
    }
}

/// Consume a prepared input and initialize the ordinary solver handle.
/// The input is consumed even on failure. Release the result with `swarm_free`.
/// # Safety
/// `input` must be null or a live prepared input, used once and exclusively.
/// `config` must be null or point to a valid current CSwarmConfig.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn swarm_init_from_input(
    input: *mut SwarmInput,
    config: *const CSwarmConfig,
) -> *mut SwarmStepHandle {
    let _scope = swarm_core::config::runtime::begin_init();
    unsafe { init_prepared(input, config) }
}

// Both entry points establish the cancellation scope before their work begins.
// Do not start a second scope here: it would lose a stop received during conversion.
unsafe fn init_prepared(
    input: *mut SwarmInput,
    config: *const CSwarmConfig,
) -> *mut SwarmStepHandle {
    ffi_step::clear_last_error();
    if input.is_null() {
        ffi_step::set_last_error(1004, "null prepared input");
        return std::ptr::null_mut();
    }
    let owned = unsafe { Box::from_raw(input) };
    #[cfg(feature = "antitamper")]
    swarm_core::tamper::run_scan(&[
        swarm_init_from_input as *const () as usize,
        ffi_step::swarm_init_phase as *const () as usize,
    ]);
    if ffi_step::init_cancelled() {
        return std::ptr::null_mut();
    }
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let (cfg, callbacks) = if config.is_null() {
            (
                crate::config::DEFAULT_SWARM_CONFIG,
                DEFAULT_CALLBACK_OPTIONS,
            )
        } else {
            let c = unsafe { &*config };
            (
                crate::ffi::build_config(c),
                crate::ffi::build_callback_options(c),
            )
        };
        ffi_step::build_grouped_handle(owned.input, cfg, callbacks)
    }));
    match run {
        Ok(Ok(handle)) => {
            if ffi_step::init_cancelled() {
                ffi_step::swarm_free(handle);
                std::ptr::null_mut()
            } else {
                handle
            }
        }
        Ok(Err(e)) => {
            if !ffi_step::init_cancelled() {
                ffi_step::set_last_error(2001, e);
            }
            std::ptr::null_mut()
        }
        Err(_) => {
            ffi_step::set_last_error(9000, "panic while initializing prepared input");
            std::ptr::null_mut()
        }
    }
}

/// Convenience conversion + initialization; existing JSON init exports remain available.
/// # Safety
/// Same pointer requirements as `swarm_input_from_struct` and `swarm_init_from_input`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn swarm_init_from_struct(
    input: *const CSwarmInput,
    config: *const CSwarmConfig,
) -> *mut SwarmStepHandle {
    let _scope = swarm_core::config::runtime::begin_init();
    let prepared = unsafe { swarm_input_from_struct(input) };
    if prepared.is_null() {
        return std::ptr::null_mut();
    }
    unsafe { init_prepared(prepared, config) }
}
