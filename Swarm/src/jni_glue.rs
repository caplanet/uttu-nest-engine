//! Swarm.dll JNI 인터페이스

// JNI 진입점도 C에서 호출되는 경계 shim이다. (ffi.rs와 동일)
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use jni::JNIEnv;
use jni::JavaVM;
use jni::objects::{GlobalRef, JClass, JObject, JString, JValueGen};
use jni::sys::jobject;
use std::ffi::{CStr, CString, c_void};
use std::sync::OnceLock;

use crate::ffi;
use crate::ffi_step;

// -- JVM 핸들
static JVM: OnceLock<JavaVM> = OnceLock::new();

// -- JNI Entry Point
#[unsafe(no_mangle)]
pub extern "system" fn JNI_OnLoad(
    vm: *mut jni::sys::JavaVM,
    _reserved: *mut std::ffi::c_void,
) -> jni::sys::jint {
    // Cache the JavaVM so the progress-callback trampoline can re-enter the JVM.
    if let Ok(java_vm) = unsafe { JavaVM::from_raw(vm) } {
        let _ = JVM.set(java_vm);
    }
    jni::sys::JNI_VERSION_1_6
}

// -- JNI Methods
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_swarm_SwarmOptimizer_optimize(
    mut env: JNIEnv,
    _class: JClass,
    input_path: JString,
    output_dir: JString,
    time_secs: jni::sys::jlong,
    rng_seed: jni::sys::jlong,
) -> jni::sys::jint {
    // Convert Java strings to C strings
    let input_path_c = java_string_to_rust(&mut env, &input_path);
    let output_dir_c = java_string_to_rust(&mut env, &output_dir);

    let (input_ptr, _output_ptr) = match (input_path_c, output_dir_c) {
        (Some(i), Some(o)) => (i, o),
        _ => return -1,
    };

    let mut config: ffi::CSwarmConfig = unsafe { std::mem::zeroed() };
    ffi::swarm_get_default_config(&mut config);
    config.rng_seed = rng_seed as u64;
    config.exploration.time_limit_secs = time_secs as f32;

    let (status, output_json) = run_step_pipeline(
        input_ptr.as_ptr(),
        &config,
        None,
        std::ptr::null_mut(),
        0,
    );
    if !output_json.is_null() {
        ffi::swarm_free_string(output_json);
    }
    status
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_swarm_SwarmOptimizer_optimizeEx(
    mut env: JNIEnv,
    _class: JClass,
    input_path: JString,
    output_dir: JString,
    config_obj: JObject,
) -> jobject {
    let result_class = match env.find_class("com/swarm/OptimizationResult") {
        Ok(c) => c,
        Err(_) => return std::ptr::null_mut(),
    };

    let result = match env.new_object(&result_class, "()V", &[]) {
        Ok(obj) => obj,
        Err(_) => return std::ptr::null_mut(),
    };

    let set_bool = |env: &mut JNIEnv, val: u8| {
        let _ = env.set_field(&result, "success", "Z", JValueGen::Bool(val));
    };

    let input_path_c = match java_string_to_rust(&mut env, &input_path) {
        Some(s) => s,
        None => {
            set_bool(&mut env, 0u8);
            return result.into_raw();
        }
    };

    let _output_dir_c = match java_string_to_rust(&mut env, &output_dir) {
        Some(s) => s,
        None => {
            set_bool(&mut env, 0u8);
            return result.into_raw();
        }
    };

    let config = build_config_from_java(&mut env, &config_obj);

    let (ret_code, output_json) = run_step_pipeline(
        input_path_c.as_ptr(),
        &config,
        None,
        std::ptr::null_mut(),
        0,
    );

    let _ = env.set_field(
        &result,
        "success",
        "Z",
        JValueGen::Bool(if ret_code == 0 { 1u8 } else { 0u8 }),
    );
    let _ = env.set_field(&result, "stripWidth", "F", JValueGen::Float(0.0));
    let _ = env.set_field(&result, "density", "F", JValueGen::Float(0.0));
    let _ = env.set_field(&result, "totalItems", "I", JValueGen::Int(0));
    let _ = env.set_field(
        &result,
        "errorCode",
        "I",
        JValueGen::Int(if ret_code == 0 { 0 } else { jni_error_code() }),
    );

    if !output_json.is_null() {
        if let Ok(json_str) = unsafe { CStr::from_ptr(output_json) }.to_str()
            && let Ok(java_string) = env.new_string(json_str)
        {
            let _ = env.set_field(
                &result,
                "outputJson",
                "Ljava/lang/String;",
                JValueGen::Object(&java_string),
            );
        }
        ffi::swarm_free_string(output_json);
    }
    result.into_raw()
}

// ── Progress callback support ──

/// Context handed to the C callback as `user_data`. Holds a global ref to the
/// Java ProgressListener so it survives the (synchronous) native call.
struct ProgressCtx {
    listener: GlobalRef,
}

/// `extern "C"` trampoline matching `ffi_step::SolutionCallback`.
/// Bridges native progress reports into `ProgressListener.onProgress(...)`.
extern "C" fn progress_trampoline(
    report_type: i32,
    strip_width: f32,
    density: f32,
    solution_json: *const std::os::raw::c_char,
    user_data: *mut c_void,
) {
    if user_data.is_null() {
        return;
    }
    let ctx = unsafe { &*(user_data as *const ProgressCtx) };
    let Some(vm) = JVM.get() else {
        return;
    };
    // Attach the reporting thread (no-op if already attached); the guard does
    // not detach a thread that was already attached.
    if let Ok(mut env) = vm.attach_current_thread() {
        // Convert the (possibly null) solution JSON into a Java String (or null).
        let json_jstr = if solution_json.is_null() {
            None
        } else {
            unsafe { CStr::from_ptr(solution_json) }
                .to_str()
                .ok()
                .and_then(|s| env.new_string(s).ok())
        };
        let null_obj = JObject::null();
        let json_arg: &JObject = match json_jstr.as_ref() {
            Some(js) => js, // &JString -> &JObject via Deref coercion
            None => &null_obj,
        };
        let _ = env.call_method(
            &ctx.listener,
            "onProgress",
            "(IFFLjava/lang/String;)V",
            &[
                JValueGen::Int(report_type),
                JValueGen::Float(strip_width),
                JValueGen::Float(density),
                JValueGen::Object(json_arg),
            ],
        );
        // Clear any pending exception so subsequent JNI calls stay valid.
        if env.exception_check().unwrap_or(false) {
            let _ = env.exception_clear();
        }
    }
}

/// Extended optimization with a real-time progress callback.
///
/// Reads the Java `SwarmConfig` field-by-field (defaults when null) and wires
/// up the progress callback. Pass `null` for `listener` to run without a callback.
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_swarm_SwarmOptimizer_optimizeWithProgress(
    mut env: JNIEnv,
    _class: JClass,
    input_path: JString,
    output_dir: JString,
    config_obj: JObject,
    listener: JObject,
) -> jobject {
    // Create result object
    let result_class = match env.find_class("com/swarm/OptimizationResult") {
        Ok(c) => c,
        Err(_) => return std::ptr::null_mut(),
    };
    let result = match env.new_object(&result_class, "()V", &[]) {
        Ok(obj) => obj,
        Err(_) => return std::ptr::null_mut(),
    };

    let set_fail = |env: &mut JNIEnv| {
        let _ = env.set_field(&result, "success", "Z", JValueGen::Bool(0u8));
    };

    // Convert Java strings
    let input_path_c = match java_string_to_rust(&mut env, &input_path) {
        Some(s) => s,
        None => {
            set_fail(&mut env);
            return result.into_raw();
        }
    };
    let _output_dir_c = match java_string_to_rust(&mut env, &output_dir) {
        Some(s) => s,
        None => {
            set_fail(&mut env);
            return result.into_raw();
        }
    };

    // Build a global ref to the listener so the C callback can use it.
    // `null` listener -> run without a callback.
    let ctx: Option<ProgressCtx> = if listener.is_null() {
        None
    } else {
        env.new_global_ref(&listener)
            .ok()
            .map(|gref| ProgressCtx { listener: gref })
    };

    // Read config from the Java SwarmConfig object (defaults when null)
    let config = build_config_from_java(&mut env, &config_obj);

    let (cb, user_data): (Option<ffi_step::SolutionCallback>, *mut c_void) = match &ctx {
        Some(c) => (
            Some(progress_trampoline as ffi_step::SolutionCallback),
            c as *const ProgressCtx as *mut c_void,
        ),
        None => (None, std::ptr::null_mut()),
    };

    // Synchronous: the callback only fires while the step pipeline is running.
    let (ret_code, output_json) = run_step_pipeline(
        input_path_c.as_ptr(),
        &config,
        cb,
        user_data,
        if cb.is_some() { 1 } else { 0 },
    );

    drop(ctx);

    // Fill result fields
    let _ = env.set_field(
        &result,
        "success",
        "Z",
        JValueGen::Bool(if ret_code == 0 { 1u8 } else { 0u8 }),
    );
    let _ = env.set_field(&result, "stripWidth", "F", JValueGen::Float(0.0));
    let _ = env.set_field(&result, "density", "F", JValueGen::Float(0.0));
    let _ = env.set_field(&result, "totalItems", "I", JValueGen::Int(0));
    let _ = env.set_field(
        &result,
        "errorCode",
        "I",
        JValueGen::Int(if ret_code == 0 { 0 } else { jni_error_code() }),
    );

    // Output JSON
    if !output_json.is_null() {
        if let Ok(json_str) = unsafe { CStr::from_ptr(output_json) }.to_str()
            && let Ok(java_string) = env.new_string(json_str)
        {
            let _ = env.set_field(
                &result,
                "outputJson",
                "Ljava/lang/String;",
                JValueGen::Object(&java_string),
            );
        }
        ffi::swarm_free_string(output_json);
    }

    result.into_raw()
}

// ── Helpers ──

fn run_step_pipeline(
    input_path: *const std::os::raw::c_char,
    config: &ffi::CSwarmConfig,
    callback: Option<ffi_step::SolutionCallback>,
    user_data: *mut c_void,
    emit_intermediate_json: i32,
) -> (i32, *mut std::os::raw::c_char) {
    let handle = ffi_step::swarm_init(input_path, config, std::ptr::null(), 0);
    if handle.is_null() {
        return (-1, std::ptr::null_mut());
    }

    ffi_step::swarm_set_callback(handle, callback, user_data, emit_intermediate_json);
    let status = ffi_step::swarm_alg_phase(handle);
    let status = if status == 0 {
        ffi_step::swarm_cp_phase(handle)
    } else {
        status
    };
    // 후처리 단계
    let status = if status == 0 {
        ffi_step::swarm_refine_phase(handle)
    } else {
        status
    };
    let output_json = if status == 0 {
        ffi_step::swarm_get_solution_json(handle)
    } else {
        std::ptr::null_mut()
    };

    ffi_step::swarm_free(handle);
    (status, output_json)
}

/// Error code for a failed run: the native last-error code when the pipeline
/// recorded one, otherwise the generic -1 (e.g. `swarm_init` returned null
/// without recording a code).
fn jni_error_code() -> i32 {
    match ffi_step::swarm_get_last_error_code() {
        0 => -1,
        code => code,
    }
}

/// Java String -> NUL-terminated `CString`. The call sites hand `.as_ptr()` to
/// `ffi_step::swarm_init`, which reads a C string, so a plain `String` (not
/// NUL-terminated) must never be used here. `None` for a null `JString`,
/// invalid UTF-8, or an embedded NUL; a pending Java exception from
/// `get_string` is cleared so later JNI calls stay valid.
fn java_string_to_rust(env: &mut JNIEnv, js: &JString) -> Option<CString> {
    let s = match env.get_string(js) {
        Ok(s) => s,
        Err(_) => {
            let _ = env.exception_clear();
            return None;
        }
    };
    match s.to_str() {
        Ok(s) => CString::new(s.to_string()).ok(),
        Err(_) => None,
    }
}

// ── Config marshalling (Java SwarmConfig -> CSwarmConfig) ──

// A failed field read (field missing on an older `SwarmConfig` class, wrong
// signature) leaves a Java exception pending, which makes every following JNI
// call invalid. Each `jget_*` clears it and falls back to 0/null; the native
// side substitutes defaults for 0/negative values.

fn jget_int(env: &mut JNIEnv, obj: &JObject, name: &str) -> i32 {
    env.get_field(obj, name, "I")
        .and_then(|v| v.i())
        .unwrap_or_else(|_| {
            let _ = env.exception_clear();
            0
        })
}

fn jget_long(env: &mut JNIEnv, obj: &JObject, name: &str) -> i64 {
    env.get_field(obj, name, "J")
        .and_then(|v| v.j())
        .unwrap_or_else(|_| {
            let _ = env.exception_clear();
            0
        })
}

fn jget_float(env: &mut JNIEnv, obj: &JObject, name: &str) -> f32 {
    env.get_field(obj, name, "F")
        .and_then(|v| v.f())
        .unwrap_or_else(|_| {
            let _ = env.exception_clear();
            0.0
        })
}

fn jget_bool(env: &mut JNIEnv, obj: &JObject, name: &str) -> i32 {
    match env.get_field(obj, name, "Z").and_then(|v| v.z()) {
        Ok(true) => 1,
        Ok(false) => 0,
        Err(_) => {
            let _ = env.exception_clear();
            0
        }
    }
}

fn jget_obj<'local>(
    env: &mut JNIEnv<'local>,
    obj: &JObject,
    name: &str,
    sig: &str,
) -> Option<JObject<'local>> {
    match env.get_field(obj, name, sig).and_then(|v| v.l()) {
        // A null sub-config object means "use defaults", same as a missing field.
        Ok(o) if !o.is_null() => Some(o),
        Ok(_) => None,
        Err(_) => {
            let _ = env.exception_clear();
            None
        }
    }
}

fn read_separator(env: &mut JNIEnv, sep: &JObject) -> ffi::CSeparatorConfig {
    ffi::CSeparatorConfig {
        iter_no_imprv_limit: jget_int(env, sep, "iterNoImprvLimit"),
        strike_limit: jget_int(env, sep, "strikeLimit"),
        n_workers: jget_int(env, sep, "nWorkers"),
        n_container_samples: jget_int(env, sep, "nContainerSamples"),
        n_focussed_samples: jget_int(env, sep, "nFocussedSamples"),
        n_coord_descents: jget_int(env, sep, "nCoordDescents"),
        // capmods 전용 필드. 구형 SwarmConfig 클래스(필드 없음)에서는 jget_*가
        // 0을 돌려주는데, 각 필드의 0/음수는 네이티브 쪽에서 기본값으로 보정된다.
        // 단 swapPeriod는 0 = off 의미이므로 구형 클래스와 섞어 쓰지 말 것.
        swap_period: jget_int(env, sep, "swapPeriod"),
        swap_attempts: jget_int(env, sep, "swapAttempts"),
        swap_area_ratio_min: jget_float(env, sep, "swapAreaRatioMin"),
        swap_area_ratio_max: jget_float(env, sep, "swapAreaRatioMax"),
        left_bias_gain: jget_float(env, sep, "leftBiasGain"),
    }
}

/// Marshal a Java `com.swarm.SwarmConfig` object into a native `CSwarmConfig`.
/// Returns the library default config when `config_obj` is null, and falls back
/// to defaults for any nested object that is unexpectedly null.
fn build_config_from_java<'local>(
    env: &mut JNIEnv<'local>,
    config_obj: &JObject,
) -> ffi::CSwarmConfig {
    let mut cfg: ffi::CSwarmConfig = unsafe { std::mem::zeroed() };
    ffi::swarm_get_default_config(&mut cfg);

    if config_obj.is_null() {
        return cfg;
    }

    const SEP_SIG: &str = "Lcom/swarm/SwarmConfig$SeparatorConfig;";
    const EXPL_SIG: &str = "Lcom/swarm/SwarmConfig$ExplorationConfig;";
    const CMPR_SIG: &str = "Lcom/swarm/SwarmConfig$CompressionConfig;";
    const PLSH_SIG: &str = "Lcom/swarm/SwarmConfig$PolishConfig;";

    // Top-level scalars
    cfg.rng_seed = jget_long(env, config_obj, "rngSeed") as u64;
    cfg.poly_simpl_tolerance = jget_float(env, config_obj, "polySimplTolerance");
    cfg.min_item_separation = jget_float(env, config_obj, "minItemSeparation");
    cfg.narrow_concavity_cutoff_dist = jget_float(env, config_obj, "narrowConcavityCutoffDist");
    cfg.narrow_concavity_cutoff_area = jget_float(env, config_obj, "narrowConcavityCutoffArea");
    cfg.lbf_x_multiplier = jget_float(env, config_obj, "lbfXMultiplier");
    cfg.lbf_y_multiplier = jget_float(env, config_obj, "lbfYMultiplier");

    // Exploration phase
    if let Some(expl) = jget_obj(env, config_obj, "exploration", EXPL_SIG) {
        cfg.exploration.shrink_step = jget_float(env, &expl, "shrinkStep");
        cfg.exploration.time_limit_secs = jget_float(env, &expl, "timeLimitSecs");
        cfg.exploration.max_conseq_failed_attempts =
            jget_int(env, &expl, "maxConseqFailedAttempts");
        cfg.exploration.solution_pool_distribution_stddev =
            jget_float(env, &expl, "solutionPoolDistributionStdDev");
        cfg.exploration.large_item_ch_area_cutoff_percentile =
            jget_float(env, &expl, "largeItemChAreaCutoffPercentile");
        if let Some(sep) = jget_obj(env, &expl, "separator", SEP_SIG) {
            cfg.exploration.separator = read_separator(env, &sep);
        }
        // capmods 전용 필드 (capmods 없이 빌드된 라이브러리에서는 무시됨)
        cfg.exploration.rnr_period = jget_int(env, &expl, "rnrPeriod");
        cfg.exploration.rnr_band_fraction = jget_float(env, &expl, "rnrBandFraction");
        cfg.exploration.rnr_max_items = jget_int(env, &expl, "rnrMaxItems");
        cfg.exploration.adaptive_shrink_gain = jget_float(env, &expl, "adaptiveShrinkGain");
        cfg.exploration.adaptive_shrink_max_mult = jget_float(env, &expl, "adaptiveShrinkMaxMult");
        cfg.exploration.osc_fail_threshold = jget_int(env, &expl, "oscFailThreshold");
        cfg.exploration.osc_expand_steps = jget_float(env, &expl, "oscExpandSteps");
        cfg.exploration.alns_decay = jget_float(env, &expl, "alnsDecay");
        cfg.exploration.alns_min_weight = jget_float(env, &expl, "alnsMinWeight");
        // 폭 완화 복구 (C ABI tail). 자바 필드가 없으면 0 -> 전부 기본값으로 보정된다.
        cfg.width_relief.fail_threshold = jget_int(env, &expl, "reliefFailThreshold");
        cfg.width_relief.step = jget_float(env, &expl, "reliefStep");
        cfg.width_relief.max_ratio = jget_float(env, &expl, "reliefMaxRatio");
        cfg.width_relief.max_rounds = jget_int(env, &expl, "reliefMaxRounds");
        cfg.width_relief.strike_limit = jget_int(env, &expl, "reliefStrikeLimit");
        cfg.width_relief.round_budget_ms = jget_int(env, &expl, "reliefRoundBudgetMs");
        cfg.shrink_feedback.fail_decay = jget_float(env, &expl, "shrinkFailDecay");
        cfg.shrink_feedback.scale_min = jget_float(env, &expl, "shrinkScaleMin");
    }

    // Compression phase
    if let Some(cmpr) = jget_obj(env, config_obj, "compression", CMPR_SIG) {
        cfg.compression.shrink_range_min = jget_float(env, &cmpr, "shrinkRangeMin");
        cfg.compression.shrink_range_max = jget_float(env, &cmpr, "shrinkRangeMax");
        cfg.compression.time_limit_secs = jget_float(env, &cmpr, "timeLimitSecs");
        cfg.compression.shrink_decay_strategy = jget_int(env, &cmpr, "shrinkDecayStrategy");
        cfg.compression.shrink_decay_ratio = jget_float(env, &cmpr, "shrinkDecayRatio");
        if let Some(sep) = jget_obj(env, &cmpr, "separator", SEP_SIG) {
            cfg.compression.separator = read_separator(env, &sep);
        }
    }

    // Polish phase
    if let Some(plsh) = jget_obj(env, config_obj, "polish", PLSH_SIG) {
        cfg.polish.enabled = jget_bool(env, &plsh, "enabled");
        cfg.polish.time_limit_secs = jget_float(env, &plsh, "timeLimitSecs");
        cfg.polish.n_rounds = jget_int(env, &plsh, "nRounds");

        cfg.polish.slide_enabled = jget_bool(env, &plsh, "slideEnabled");
        cfg.polish.slide_n_passes = jget_int(env, &plsh, "slideNPasses");
        cfg.polish.slide_down = jget_bool(env, &plsh, "slideDown");
        cfg.polish.slide_safety_ratio = jget_float(env, &plsh, "slideSafetyRatio");

        cfg.polish.hole_fill_enabled = jget_bool(env, &plsh, "holeFillEnabled");
        cfg.polish.hole_fill_cell_size_ratio = jget_float(env, &plsh, "holeFillCellSizeRatio");
        cfg.polish.hole_fill_max_candidates = jget_int(env, &plsh, "holeFillMaxCandidates");
        cfg.polish.hole_fill_candidate_band = jget_float(env, &plsh, "holeFillCandidateBand");
        cfg.polish.hole_fill_n_samples = jget_int(env, &plsh, "holeFillNSamples");

        cfg.polish.lp_enabled = jget_bool(env, &plsh, "lpEnabled");
        cfg.polish.lp_n_iterations = jget_int(env, &plsh, "lpNIterations");
        cfg.polish.lp_trust_region_ratio = jget_float(env, &plsh, "lpTrustRegionRatio");
        cfg.polish.lp_left_pull_weight = jget_float(env, &plsh, "lpLeftPullWeight");
    }

    // LBF raster NFP (capmods 전용 — capmods 없이 빌드된 라이브러리에서는 무시됨)
    const RSTR_SIG: &str = "Lcom/swarm/SwarmConfig$RasterNfpConfig;";
    if let Some(rstr) = jget_obj(env, config_obj, "lbfRaster", RSTR_SIG) {
        cfg.lbf_raster.enabled = jget_bool(env, &rstr, "enabled");
        cfg.lbf_raster.cell_ratio = jget_float(env, &rstr, "cellRatio");
        cfg.lbf_raster.max_cells = jget_int(env, &rstr, "maxCells");
        cfg.lbf_raster.n_candidates = jget_int(env, &rstr, "nCandidates");
        cfg.lbf_raster.n_random_samples = jget_int(env, &rstr, "nRandomSamples");
    }

    // 회전 자유도
    cfg.allow_180_rotation = jget_bool(env, config_obj, "allow180Rotation");
    cfg.tilt_range_deg = jget_float(env, config_obj, "tiltRangeDeg");
    // ABI v19: absent on older Java classes -> 0 = infeasible (previous behavior).
    cfg.original_overlap.excess_policy = jget_int(env, config_obj, "originalOverlapPolicy");
    // ABI v20: absent on older Java classes -> 0 = multipolygon place groups.
    cfg.place_group.mode = jget_int(env, config_obj, "placeGroupMode");

    // Defensive: clear any pending field-access exception so later JNI calls stay valid.
    if env.exception_check().unwrap_or(false) {
        let _ = env.exception_clear();
    }

    cfg
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_swarm_SwarmOptimizer_isStopRequested(
    _env: JNIEnv,
    _class: JClass,
    handle: jni::sys::jlong,
) -> jni::sys::jint {
    if handle == 0 {
        return 0;
    }
    ffi_step::swarm_is_stop_requested(handle as *const ffi_step::SwarmStepHandle)
        as jni::sys::jint
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_swarm_SwarmOptimizer_stopAndWait(
    _env: JNIEnv,
    _class: JClass,
    handle: jni::sys::jlong,
) {
    if handle != 0 {
        ffi_step::swarm_request_stop_and_wait(handle as *mut ffi_step::SwarmStepHandle);
    }
}
