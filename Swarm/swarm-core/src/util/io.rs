use crate::EPOCH;
use anyhow::{Context, Result};
use clap::Parser;
use jagua_rs::probs::spp::io::ext_repr::{ExtSPInstance, ExtSPSolution};
use log::{Level, LevelFilter, log};
use serde::{Deserialize, Serialize};
use std::fs;
use std::fs::File;
use std::path::Path;
use svg::Document;

#[derive(Parser)]
pub struct MainCli {
    /// Path to input file (mandatory)
    #[arg(
        short = 'i',
        long,
        help = "Path to the input JSON file, or a solution JSON file for warm starting"
    )]
    pub input: String,

    /// Global time limit in seconds (mutually exclusive with -e and -c)
    #[arg(short = 't', long, conflicts_with_all = &["exploration", "compression"], help = "Set a global time limit (in seconds)")]
    pub global_time: Option<u64>,

    /// Exploration time limit in seconds (requires compression time)
    #[arg(
        short = 'e',
        long,
        requires = "compression",
        help = "Set the exploration phase time limit (in seconds)"
    )]
    pub exploration: Option<u64>,

    /// Compression time limit in seconds (requires exploration time)
    #[arg(
        short = 'c',
        long,
        requires = "exploration",
        help = "Set the compression phase time limit (in seconds)"
    )]
    pub compression: Option<u64>,

    /// Enable early and automatic termination
    #[arg(
        short = 'x',
        long,
        help = "Enable early termination of the optimization process"
    )]
    pub early_termination: bool,

    #[arg(short = 's', long, help = "Fixed seed for the random number generator")]
    pub rng_seed: Option<u64>,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct ExtSPOutput {
    #[serde(flatten)]
    pub instance: ExtSPInstance,
    pub solution: ExtSPSolution,
}

pub fn init_logger(level_filter: LevelFilter, log_file_path: &Path) -> Result<()> {
    //remove old log file
    let _ = fs::remove_file(log_file_path);
    fern::Dispatch::new()
        // Perform allocation-free log formatting
        .format(|out, message, record| {
            let handle = std::thread::current();
            let thread_name = handle.name().unwrap_or("-");

            let duration = EPOCH.elapsed();
            let sec = duration.as_secs() % 60;
            let min = (duration.as_secs() / 60) % 60;
            let hours = (duration.as_secs() / 60) / 60;

            let prefix = format!(
                "[{}] [{:0>2}:{:0>2}:{:0>2}] <{}>",
                record.level(),
                hours,
                min,
                sec,
                thread_name,
            );

            out.finish(format_args!("{:<25}{}", prefix, message))
        })
        // Add blanket level filter -
        .level(level_filter)
        .chain(std::io::stdout())
        .chain(fern::log_file(log_file_path)?)
        .apply()?;
    log!(Level::Info, "[EPOCH]: {}", jiff::Timestamp::now());
    Ok(())
}

pub fn write_svg(document: &Document, path: &Path, log_lvl: Level) -> Result<()> {
    //make sure the parent directory exists
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).context("could not create parent directory for svg file")?;
    }
    svg::save(path, document)?;
    log!(
        log_lvl,
        "[IO] svg exported to file://{}",
        fs::canonicalize(path)
            .expect(crate::dmsg!("could not canonicalize path"))
            .to_str()
            .unwrap()
    );
    Ok(())
}

pub fn write_json(json: &impl Serialize, path: &Path, log_lvl: Level) -> Result<()> {
    let file = File::create(path)?;
    serde_json::to_writer_pretty(file, json)?;
    log!(
        log_lvl,
        "[IO] json exported to file://{}",
        fs::canonicalize(path)
            .expect(crate::dmsg!("could not canonicalize path"))
            .to_str()
            .unwrap()
    );
    Ok(())
}

/// 각 아이템의 허용 회전 목록에 180° 반전 각을 추가한다 (결방향 없는 원단 전용).
///
/// `allowed_orientations`가 명시된 아이템에 대해 각 각도 `o`마다 `(o + 180) % 360`을
/// (아직 없으면) 추가한다. 연속 회전(`None`) 아이템은 그대로 둔다.
/// jagua-rs import **전의** 외부 표현에 적용해야 한다.
/// 인스턴스를 import 한다. **겹침 허용일 때 원단 경계는 열지 않는다.**
///
/// (`scale-collapse` 브랜치 899a451/fb4ca00 에서 이식.)
///
/// # 왜 jagua 의 `import_instance` 를 그냥 쓰지 않는가
///
/// `min_item_separation = -X`(겹침 X mm 허용)를 주면 jagua importer 는 그 절반을
/// **아이템과 컨테이너 양쪽에** 적용하는데 모드가 반대다.
///
/// ```text
/// 아이템   Inflate  offset -X/2  ->  도형이 X/2 작아진다   (의도한 것)
/// 컨테이너 Deflate  offset -X/2  ->  경계가 X/2 넓어진다   (의도하지 않은 것)
/// ```
///
/// 두 효과가 겹쳐 조각의 **실제 재단 도형**이 원단 경계를 넘는다. wsjk156 에서
/// 실측하면 2mm 설정에서 아래로 **2.7mm** 나간다(`tests/overlap_semantics.rs`).
///
/// 그건 사용자가 승인한 것과 **다른 종류의 공차**다.
///
/// * 조각끼리의 겹침 — 두 조각이 같은 2mm 를 나눠 쓴다. 둘 다 조금 작게 재단된다.
/// * 경계 초과 — 조각 하나가 **원단에 없는** 부분을 요구한다. 재단 자체가 불가능하다.
///
/// 그래서 컨테이너 오프셋만 부호를 뒤집는다. `|sep|/2` 로 컨테이너를 X/2 **줄이면**,
/// X/2 작아진 충돌 도형이 그 안에 들어갔을 때 실제 도형이 정확히 공칭 경계에 닿는다.
///
/// 양수 `min_item_separation`(정상 이격)은 지금까지와 **한 비트도 다르지 않다** —
/// 그 경우 `|sep|/2 == sep/2` 다.
///
/// # 왜 jagua 함수를 포크하지 않는가
///
/// `import_instance` 는 아이템 정렬·ID 검증·기준 폭 계산까지 한다. 그것을 베껴 오면
/// 상류가 고쳐질 때 한쪽만 고쳐진다 — 이 코드베이스가 명시적으로 경계하는 형태다.
/// 그래서 **호출한 뒤 strip 설정만 고친다.** `SPProblem::new` 과
/// `change_strip_width` 가 둘 다 `base_strip` 에서 컨테이너를 다시 만들므로
/// 이 한 곳만 고치면 전 경로에 전달된다.
/// jagua-rs 1.x `Importer::new` takes no separation; it is applied with
/// `with_min_item_separation`. Keeps the 0.8-style four-argument call sites.
pub fn new_importer(
    cde_config: jagua_rs::collision_detection::CDEConfig,
    simplify_tolerance: Option<f32>,
    min_item_separation: Option<f32>,
    narrow_concavity_cutoff: Option<(f32, f32)>,
) -> jagua_rs::io::import::Importer {
    let importer = jagua_rs::io::import::Importer::new(cde_config, simplify_tolerance, narrow_concavity_cutoff);
    match min_item_separation {
        Some(separation) => importer
            .with_min_item_separation(separation)
            .expect(crate::dmsg!("min_item_separation must be finite and nonnegative")),
        None => importer,
    }
}

pub fn import_spp_instance(
    ext_instance: &ExtSPInstance,
    cde_config: jagua_rs::collision_detection::CDEConfig,
    poly_simpl_tolerance: Option<f32>,
    min_item_separation: Option<f32>,
    narrow_concavity_cutoff_ratio: Option<(f32, f32)>,
) -> anyhow::Result<jagua_rs::probs::spp::entities::SPInstance> {
    import_spp_instance_with_mm(ext_instance, cde_config, poly_simpl_tolerance, min_item_separation, narrow_concavity_cutoff_ratio, None)
}

/// The mm mode replaces (never follows) the area-ratio simplifier. Its source
/// boundary is the explicitly offset contour; overlap and shape error remain
/// independent budgets. Strip containers are rectangles and need no reduction.
pub fn import_spp_instance_with_mm(
    ext_instance: &ExtSPInstance,
    cde_config: jagua_rs::collision_detection::CDEConfig,
    poly_simpl_tolerance: Option<f32>,
    min_item_separation: Option<f32>,
    narrow_concavity_cutoff_ratio: Option<(f32, f32)>,
    max_deviation_mm: Option<f32>,
) -> anyhow::Result<jagua_rs::probs::spp::entities::SPInstance> {
    let token = crate::config::runtime::init_cancel_token();
    import_spp_instance_with_mm_inner(
        ext_instance,
        cde_config,
        poly_simpl_tolerance,
        min_item_separation,
        narrow_concavity_cutoff_ratio,
        max_deviation_mm,
        &|| token.is_some_and(|t| t.is_cancelled()),
    )
}

fn import_spp_instance_with_mm_inner(
    ext_instance: &ExtSPInstance,
    cde_config: jagua_rs::collision_detection::CDEConfig,
    poly_simpl_tolerance: Option<f32>,
    min_item_separation: Option<f32>,
    narrow_concavity_cutoff_ratio: Option<(f32, f32)>,
    max_deviation_mm: Option<f32>,
    cancelled: &(impl Fn() -> bool + Sync),
) -> anyhow::Result<jagua_rs::probs::spp::entities::SPInstance> {
    use jagua_rs::entities::Item;
    use jagua_rs::probs::spp::entities::{SPInstance, Strip};
    use jagua_rs::geometry::shape_modification::ShapeModifyConfig;
    use rayon::prelude::*;
    anyhow::ensure!(!cancelled(), "initialization cancelled");
    if let Some(mm) = max_deviation_mm {
        anyhow::ensure!(mm.is_finite() && mm >= 0.0, "simplification maximum deviation must be finite and nonnegative");
    }
    let mut importer = jagua_rs::io::import::Importer::new(
        cde_config,
        if max_deviation_mm.is_some() { None } else { poly_simpl_tolerance },
        if max_deviation_mm.is_some() { None } else { narrow_concavity_cutoff_ratio },
    );
    if let Some(separation) = min_item_separation {
        importer = importer.with_min_item_separation(separation)?;
    }
    // Match Jagua 0.8.3 import_instance ordering, area accumulation and strip
    // configuration, but check cancellation between expensive item imports.
    #[cfg(feature = "init-profile")]
    crate::init_profile::count("import.items", ext_instance.items.len());
    let mut items = ext_instance.items.par_iter().map(|ext_item| {
    #[cfg(feature = "init-profile")]
    let _profile = crate::init_profile::Scope::new("detail.import_item_parallel");

        anyhow::ensure!(!cancelled(), "initialization cancelled");
        let item = importer.import_item(&ext_item.base, usize::try_from(ext_item.base.id)?)?;
        anyhow::ensure!(!cancelled(), "initialization cancelled");
        Ok((item, usize::try_from(ext_item.demand)?))
    }).collect::<Result<Vec<(std::sync::Arc<Item>, usize)>>>()?;
    #[cfg(feature = "init-profile")]
    {
        crate::init_profile::count("import.rayon_threads", rayon::current_num_threads());
        crate::init_profile::count("import.cd_vertices", items.iter().map(|(item, _)| item.shape_cd().n_vertices()).sum());
    }
    items.sort_by_key(|(item, _)| item.idx());
    items.retain(|(_, demand)| *demand > 0);
    anyhow::ensure!(items.iter().enumerate().all(|(id, (item, _))| item.idx() == id),
        "All items should have consecutive IDs starting from 0");
    anyhow::ensure!(!items.is_empty(), "ExtSPInstance must have at least one item with positive demand");
    let width = items.iter().map(|(item, demand)| item.area() * *demand as f32).sum::<f32>()
        / ext_instance.strip_height;
    anyhow::ensure!(!cancelled(), "initialization cancelled");
    let strip = Strip::new(ext_instance.strip_height, importer.cde_config,
        ShapeModifyConfig { offset: importer.shape_modify_config.offset,
            simplify_tolerance: None, narrow_concavity_cutoff: None }, width)?;
    let mut instance = SPInstance::new(items, strip);
    if let Some(mm) = max_deviation_mm {
        for (item, _) in instance.items.iter_mut() {
            anyhow::ensure!(!cancelled(), "initialization cancelled");
            let mut shape = super::simplify_mm::simplify_shape_mm(item.shape_cd(), jagua_rs::geometry::shape_modification::ShapeModifyMode::Inflate, mm)?;
            shape.generate_surrogate(item.surrogate_config())?;
            *item = std::sync::Arc::new(item.as_ref().clone().with_shape_cd(std::sync::Arc::new(shape)));
        }
    }

    anyhow::ensure!(!cancelled(), "initialization cancelled");
    apply_overlap_strip_offset(&mut instance.base_strip, min_item_separation);
    Ok(instance)
}

/// Reuse fully prepared item geometry while assigning lane-local identities.
/// Shape preparation policy must be identical to the source instance; only the
/// selected item set and strip width change. Each lane keeps its own search state.
pub fn prepared_spp_subset(
    full: &jagua_rs::probs::spp::entities::SPInstance,
    global_ids: &[usize],
    min_item_separation: Option<f32>,
) -> Result<jagua_rs::probs::spp::entities::SPInstance> {
    use jagua_rs::entities::Item;
    use jagua_rs::probs::spp::entities::{SPInstance, Strip};
    anyhow::ensure!(!global_ids.is_empty(), "prepared subset must not be empty");
    let mut seen = std::collections::HashSet::with_capacity(global_ids.len());
    let items = global_ids.iter().enumerate().map(|(local, &global)| {
        anyhow::ensure!(seen.insert(global), "duplicate prepared subset item {global}");
        let (source, demand) = full.items.get(global)
            .with_context(|| format!("unknown prepared subset item {global}"))?;
        anyhow::ensure!(*demand > 0, "prepared subset demand must be positive");
        let item = std::sync::Arc::new(source.as_ref().clone().with_idx(local).with_external_id(local as u64));
        Ok((item, *demand))
    }).collect::<Result<Vec<(std::sync::Arc<Item>, usize)>>>()?;
    // Keep the importer's f32 accumulation order and original-area semantics.
    let width = items.iter().map(|(item, qty)| item.area() * *qty as f32).sum::<f32>()
        / full.base_strip.fixed_height;
    let mut modify = full.base_strip.shape_modify_config;
    modify.offset = min_item_separation.map(|sep| sep / 2.0);
    let mut strip = Strip::new(full.base_strip.fixed_height, full.base_strip.cde_config, modify, width)?;
    apply_overlap_strip_offset(&mut strip, min_item_separation);
    Ok(SPInstance::new(items, strip))
}

fn apply_overlap_strip_offset(
    strip: &mut jagua_rs::probs::spp::entities::Strip,
    min_item_separation: Option<f32>,
) {
    if let Some(sep) = min_item_separation {
        if sep < 0.0 {
            // 겹침 허용. 컨테이너만 반대 방향으로 — 원단은 넓어지지 않는다.
            //
            // 이 인셋만으로는 **원본** 외곽선이 원단 안에 있다고 보증되지 않는다.
            // 침식은 변을 `r` 만큼 들이지만 볼록 꼭짓점은 `r/sin(θ/2)` 만큼 들이므로
            // 침식된 도형이 벽에 닿으면 원본 꼭짓점이 그 차이만큼 밖으로 나간다.
            // 그 초과는 [`crate::fabric_edge`] 가 공개 직전에 국소 보정으로 없앤다 —
            // 여기서 컨테이너를 더 줄이는 방식은 가장 날카로운 꼭짓점 하나가 원반
            // 전체를 줄여 실측에서 최종 효율을 1.5%p 깎았다.
            let deflate = -sep / 2.0;

            // **상한을 둔다.** 컨테이너를 양쪽에서 줄이므로 원단의 짧은 변보다 크게
            // 줄이면 다각형이 비고, jagua 의 `Strip -> Container` 변환이 `unwrap` 에서
            // **패닉한다**. 그 패닉은 C ABI 경계를 넘어가므로 호스트가 죽는다.
            //
            // 실 마커(원단 높이 1,524mm)에서는 도달할 수 없는 상황이지만, 작은
            // 인스턴스나 잘못 입력된 값 하나로 DLL 이 죽는 것은 막아야 한다.
            let shortest = strip.fixed_height.min(strip.width);
            let limit = 0.45 * shortest;
            let applied = if deflate > limit && limit > 0.0 {
                log::warn!(
                    "[IMPORT] 겹침 {:.3}mm 는 이 원단(짧은 변 {:.3})에 비해 너무 크다 — 컨테이너 축소를 {:.3} 로 제한한다. 조각끼리의 겹침은 설정대로 유지되지만 원단 경계 보증은 그만큼 약해진다",
                    -sep,
                    shortest,
                    limit * 2.0
                );
                limit
            } else {
                deflate
            };
            strip.shape_modify_config.offset = Some(applied);
        }
    }
}

pub fn augment_orientations_180(ext_instance: &mut ExtSPInstance) {
    for item in ext_instance.items.iter_mut() {
        if let jagua_rs::io::ext_repr::ExtRotation::Discrete { angles: orients } =
            &mut item.base.orientation.rotation
        {
            let existing = orients.clone();
            for o in existing {
                let flipped = (o + 180.0).rem_euclid(360.0);
                if !orients
                    .iter()
                    .any(|&e| (e.rem_euclid(360.0) - flipped).abs() < 1e-3)
                {
                    orients.push(flipped);
                }
            }
        }
    }
}

pub fn read_spp_input(path: &Path) -> Result<(ExtSPInstance, Option<ExtSPSolution>)> {
    let input_str = fs::read_to_string(path).context("could not read input file")?;
    parse_spp_input(&input_str)
}

/// Parse the established output-first / instance-fallback format from raw text.
pub fn parse_spp_input(input: &str) -> Result<(ExtSPInstance, Option<ExtSPSolution>)> {
    //try parsing a full output (instance + solution)
    match serde_json::from_str::<ExtSPOutput>(input) {
        Ok(ext_output) => Ok((ext_output.instance, Some(ext_output.solution))),
        Err(output_error) => {
            //try parsing just the instance
            let ext_instance = serde_json::from_str::<ExtSPInstance>(input)
                .context("could not parse instance from input file")?;
            // A `solution` the importer cannot read (e.g. missing `strip_width`,
            // `density` or `run_time_sec`) used to vanish without a trace and the
            // run silently started from scratch. Keep the instance-only fallback
            // but say why the placements were dropped.
            if input.contains("\"solution\"") && !input.contains("\"solution\":null") {
                log::warn!(
                    "[INPUT] solution present but not readable; starting without it: {output_error}"
                );
            }
            Ok((ext_instance, None))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn issue9_legacy_raw_text_file_and_string_parity() {
        let base = serde_json::to_value(make_instance(vec![Some(vec![0.0]); 2])).unwrap();
        let mut output = base.clone();
        output["solution"] = serde_json::json!({"strip_width":4.0,"density":0.05,"run_time_sec":7,
            "layout":{"container_id":0,"density":0.05,"placed_items":[
                {"item_id":0,"transformation":{"rotation":0.0,"translation":[1.0,1.0]}},
                {"item_id":1,"transformation":{"rotation":0.0,"translation":[2.0,1.0]}}
            ]}});
        let mut cases = vec![("instance", base.to_string(), true, false),
            ("flattened output", output.to_string(), true, true)];
        for (label, solution) in [("null solution", serde_json::Value::Null),
            ("malformed solution", serde_json::json!({"strip_width":"bad"}))] {
            let mut v = base.clone(); v["solution"] = solution;
            cases.push((label, v.to_string(), true, false));
        }
        let mut v = base.clone(); v["unknown"] = serde_json::json!(true);
        v["items"][0]["OrgID"] = serde_json::json!(9_007_199_254_740_993_i64);
        v["items"][1]["OrgID"] = serde_json::json!(i64::MAX);
        cases.push(("unknown metadata", v.to_string(), true, false));
        let mut v = base.clone(); v["items"][0]["demand"] = serde_json::json!(0);
        cases.push(("zero demand parser", v.to_string(), true, false));
        let mut v = base.clone(); v["items"][0]["demand"] = serde_json::json!(-1);
        cases.push(("negative demand", v.to_string(), false, false));
        let mut v = base.clone(); v["items"][0]["shape"]["type"] = serde_json::json!("invalid");
        cases.push(("invalid shape", v.to_string(), false, false));
        cases.push(("duplicate typed field", base.to_string().replacen(
            "\"strip_height\":", "\"strip_height\":10.0,\"strip_height\":", 1), false, false));
        cases.push(("truncated", "{\"items\":".into(), false, false));
        let path = std::env::temp_dir().join(format!("swarm-issue9-core-{}.json", std::process::id()));
        struct Temp(std::path::PathBuf);
        impl Drop for Temp { fn drop(&mut self) { let _ = std::fs::remove_file(&self.0); } }
        let _temp = Temp(path.clone());
        for (label, text, ok, has_solution) in cases {
            std::fs::write(&path, &text).unwrap();
            let file = read_spp_input(&path);
            let string = parse_spp_input(&text);
            assert_eq!(file.is_ok(), ok, "{label}");
            match (file, string) {
                (Ok((fi, fs)), Ok((si, ss))) => {
                    assert_eq!(serde_json::to_value(fi).unwrap(), serde_json::to_value(si).unwrap(), "{label}");
                    assert_eq!(fs.is_some(), has_solution, "{label}");
                    assert_eq!(serde_json::to_value(fs).unwrap(), serde_json::to_value(&ss).unwrap(), "{label}");
                    if label == "flattened output" { assert_eq!(serde_json::to_value(ss).unwrap(),
                        serde_json::to_value(serde_json::from_value::<ExtSPSolution>(output["solution"].clone()).unwrap()).unwrap()); }
                }
                (Err(fe), Err(se)) => {
                    assert_eq!(format!("{fe:#}"), format!("{se:#}"), "{label}");
                    assert_eq!(se.to_string(), "could not parse instance from input file", "{label}");
                }
                _ => panic!("file/string mismatch: {label}"),
            }
        }
    }
    fn make_instance(orients: Vec<Option<Vec<f32>>>) -> ExtSPInstance {
        let items = orients
            .into_iter()
            .enumerate()
            .map(|(i, o)| jagua_rs::probs::spp::io::ext_repr::ExtItem {
                base: jagua_rs::io::ext_repr::ExtItem {
                    id: i as u64,
                    orientation: jagua_rs::io::ext_repr::ExtOrientation::from_legacy(o.as_deref()),
                    shape: jagua_rs::io::ext_repr::ExtShape::Rectangle {
                        x_min: 0.0,
                        y_min: 0.0,
                        width: 1.0,
                        height: 1.0,
                    },
                    min_quality: None,
                },
                demand: 1,
            })
            .collect();
        ExtSPInstance {
            name: "t".into(),
            min_item_separation: 0.0,
            items,
            strip_height: 10.0,
        }
    }

    #[test]
    fn cancelled_import_stops_between_items() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        let input = make_instance(vec![Some(vec![0.0]); 100]);
        let polls = AtomicUsize::new(0);
        let result = import_spp_instance_with_mm_inner(&input,
            crate::config::DEFAULT_SWARM_CONFIG.cde_config, None, None, None, None,
            &|| polls.fetch_add(1, Ordering::Relaxed) >= 4);
        assert!(result.unwrap_err().to_string().contains("initialization cancelled"));
        // No cancelled partial instance may escape to LBF or certification.
    }

    #[test]
    fn cancellable_import_preserves_upstream_geometry_and_order() {
        // jagua-rs 1.x `import_instance` reads the separation from the instance
        // and rejects negative values, so item geometry is compared with upstream
        // for the nonnegative cases only; the strip keeps swarm's 0.8-style
        // width (item area / height) and offset (separation / 2) for all cases.
        for separation in [None, Some(0.08), Some(-0.08)] {
            let mut input = make_instance(vec![Some(vec![0.0, 180.0]); 4]);
            input.items.reverse();
            let cde = crate::config::DEFAULT_SWARM_CONFIG.cde_config;
            let actual = import_spp_instance_with_mm_inner(&input, cde, Some(0.0001), separation, None, None, &|| false).unwrap();
            let expected_width = actual.items.iter().map(|(item, qty)| item.area() * *qty as f32).sum::<f32>() / input.strip_height;
            assert_eq!(actual.base_strip.width.to_bits(), expected_width.to_bits());
            let mut expected_strip = jagua_rs::probs::spp::entities::Strip::new(
                input.strip_height,
                cde,
                jagua_rs::geometry::shape_modification::ShapeModifyConfig {
                    offset: separation.filter(|s| *s != 0.0).map(|s| s / 2.0),
                    simplify_tolerance: None,
                    narrow_concavity_cutoff: None,
                },
                expected_width,
            ).unwrap();
            apply_overlap_strip_offset(&mut expected_strip, separation);
            assert_eq!(actual.base_strip.shape_modify_config.offset, expected_strip.shape_modify_config.offset);
            assert!(actual.items.iter().enumerate().all(|(idx, (item, _))| item.idx() == idx));
            if separation.is_none_or(|s| s >= 0.0) {
                input.min_item_separation = separation.unwrap_or(0.0);
                let importer = crate::util::io::new_importer(cde, Some(0.0001), None, None);
                let expected = jagua_rs::probs::spp::io::import_instance(&importer, &input).unwrap();
                // Upstream keeps input order; swarm sorts by id. Match by external id.
                assert_eq!(actual.items.len(), expected.items.len());
                for (a, aq) in &actual.items {
                    let (b, bq) = expected.items.iter().find(|(b, _)| b.external_id() == a.external_id()).unwrap();
                    assert_eq!(aq, bq);
                    assert_eq!(a.shape_cd().vertices(), b.shape_cd().vertices());
                    assert_eq!(a.shape_orig().shape.vertices(), b.shape_orig().shape.vertices());
                }
            }
        }
    }

    #[test]
    fn augment_180_adds_flips_without_duplicates() {
        let mut inst = make_instance(vec![
            Some(vec![0.0]),                     // → [0, 180]
            Some(vec![0.0, 180.0]),              // 이미 있음 → 그대로
            Some(vec![0.0, 90.0, 180.0, 270.0]), // 전부 있음 → 그대로
            None,                                // 연속 회전 → 그대로
        ]);
        augment_orientations_180(&mut inst);

        use jagua_rs::io::ext_repr::ExtRotation;
        let angles = |k: usize| inst.items[k].base.orientation.rotation.clone();
        assert_eq!(angles(0), ExtRotation::Discrete { angles: vec![0.0, 180.0] });
        assert_eq!(angles(1), ExtRotation::Discrete { angles: vec![0.0, 180.0] });
        assert_eq!(angles(2), ExtRotation::Discrete { angles: vec![0.0, 90.0, 180.0, 270.0] });
        assert_eq!(angles(3), ExtRotation::Continuous {});
    }
}
