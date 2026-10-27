use crate::config::SlideConfig;
use crate::optimizer::polish::probe::{CollisionProbe, translated};
use crate::optimizer::separator::Separator;
use crate::util::terminator::Terminator;
use itertools::Itertools;
use jagua_rs::entities::{PItemKey};
use jagua_rs::geometry::DTransformation;
use ordered_float::OrderedFloat;

/// 왼쪽 이동이 조각 최소 변의 이 비율보다 짧으면 비스듬한 방향도 시도한다.
const DIAGONAL_TRIGGER_RATIO: f32 = 0.25;

/// 비스듬한 왼쪽 방향 (cos, sin): 15°, 30°, 45°. 위·아래 둘 다 본다.
const DIAGONAL_LEFT: [(f32, f32); 3] = [
    (0.965_925_8, 0.258_819_04),
    (0.866_025_4, 0.5),
    (std::f32::consts::FRAC_1_SQRT_2, std::f32::consts::FRAC_1_SQRT_2),
];

/// 슬라이딩 방향
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SlideDir {
    Left,
    Down,
}

impl SlideDir {
    fn vec(self) -> (f32, f32) {
        match self {
            SlideDir::Left => (-1.0, 0.0),
            SlideDir::Down => (0.0, -1.0),
        }
    }
}

/// 결정론적 슬라이딩(compaction) 패스.
///
/// swarm 본체의 compression은 strip 폭을 통째로 줄인 뒤 separation을 다시 돌리는
/// 확률적 방식이라, 겹치지 않는 아이템은 애초에 이동 대상이 아니다. 그래서 feasible 해에
/// 남은 자잘한 틈이 그대로 방치된다. 이 패스는 그 틈을 직접 회수한다:
/// 각 아이템을 **접촉할 때까지 왼쪽으로**(옵션에 따라 아래로도) 밀어 넣는다.
///
/// 모든 이동은 CDE로 충돌 없음이 검증된 거리만 반영하므로 해의 실행 가능성을 깨뜨리지 않는다.
/// 고정(frozen) 아이템은 건드리지 않는다.
///
/// 반환값: 실제로 이동한 아이템 수
pub fn slide_pass(sep: &mut Separator, cfg: &SlideConfig, term: &impl Terminator) -> usize {
    let mut total_moved = 0;

    for _ in 0..cfg.n_passes.max(1) {
        if term.kill() {
            break;
        }
        let mut moved = slide_along(sep, SlideDir::Left, cfg, term);
        if cfg.slide_down && !term.kill() {
            moved += slide_along(sep, SlideDir::Down, cfg, term);
        }
        total_moved += moved;
        if moved == 0 {
            // 더 이상 밀 곳이 없다
            break;
        }
    }
    total_moved
}

/// 한 방향에 대한 슬라이딩 1회.
fn slide_along(
    sep: &mut Separator,
    dir: SlideDir,
    cfg: &SlideConfig,
    term: &impl Terminator,
) -> usize {
    if sep.has_place_groups() {
        return slide_units_along(sep, dir, cfg, term);
    }
    // 진행 방향에 가까운 아이템부터 밀어야 뒤따르는 아이템이 그 자리를 이어받는다.
    let order: Vec<PItemKey> = sep
        .prob
        .layout()
        .placed_items()
        .iter()
        .filter(|(_, pi)| !sep.frozen_mask.get(pi.item().idx()).copied().unwrap_or(false))
        .sorted_by_key(|(_, pi)| {
            let bbox = pi.shape().bbox();
            match dir {
                SlideDir::Left => OrderedFloat(bbox.x_min),
                SlideDir::Down => OrderedFloat(bbox.y_min),
            }
        })
        .map(|(pk, _)| pk)
        .collect();

    let container = sep.prob.layout().container().outer_cd().bbox();
    let mut moved = 0;

    for pk in order {
        if term.kill() {
            break;
        }
        // 이 아이템이 아직 레이아웃에 있는지 확인 (방어적)
        let Some(pi) = sep.prob.layout().placed_items().get(pk) else {
            continue;
        };
        let (item_id, d_transf, bbox) = (pi.item().idx(), pi.d_transf(), pi.shape().bbox());

        // 컨테이너 벽까지의 거리가 이동 가능한 최대치다.
        let max_dist = match dir {
            SlideDir::Left => bbox.x_min,
            SlideDir::Down => bbox.y_min,
        };
        if max_dist <= 0.0 {
            continue;
        }

        let target = {
            let item = sep.instance.item(item_id);
            let min_dim = f32::min(item.shape_cd().bbox().width(), item.shape_cd().bbox().height());
            let original = if cfg.original_aware {
                sep.original_guard()
            } else {
                None
            };
            let mut probe =
                CollisionProbe::new(&sep.prob.layout(), item, pk).with_original(original);
            let mut slide_to = |dir_vec: (f32, f32), max_dist: f32| {
                let dist = probe.max_slide_with_control(
                    d_transf,
                    dir_vec,
                    max_dist,
                    min_dim * cfg.scan_step_ratio,
                    cfg.n_bisections,
                    &mut || term.kill(),
                );
                probe.settle_original(d_transf, dir_vec, dist, cfg.n_bisections)
            };

            // 진행 방향 성분으로 비교한다: 비스듬히 간 거리는 그만큼 덜 친다.
            let (x0, y0) = d_transf.translation();
            let progress = |dt: DTransformation| {
                let (x, y) = dt.translation();
                match dir {
                    SlideDir::Left => x0 - x,
                    SlideDir::Down => y0 - y,
                }
            };
            let mut best = slide_to(dir.vec(), max_dist);
            let mut best_gain = progress(best);
            if cfg.diagonal
                && dir == SlideDir::Left
                && best_gain < min_dim * DIAGONAL_TRIGGER_RATIO
                && !term.kill()
            {
                for &(cos, sin) in DIAGONAL_LEFT.iter() {
                    for dy in [sin, -sin] {
                        let vertical_room = if dy > 0.0 {
                            container.y_max - bbox.y_max
                        } else {
                            bbox.y_min - container.y_min
                        };
                        let reach = f32::min(max_dist / cos, vertical_room / sin);
                        if reach <= 0.0 || term.kill() {
                            continue;
                        }
                        let candidate = slide_to((-cos, dy), reach);
                        let gain = progress(candidate);
                        if gain > best_gain + 1.0e-4 {
                            best = candidate;
                            best_gain = gain;
                        }
                    }
                }
            }

            if best_gain <= min_dim * cfg.min_move_ratio {
                None
            } else {
                // 접촉 직전에 안전 간격을 남기고 싶다면, 줄인 자리도 실제로 검증한 뒤에만 쓴다.
                // (충돌 여부는 거리에 대해 단조롭지 않아 "덜 가는 쪽"이 항상 안전하지는 않다)
                let safety = min_dim * cfg.safety_ratio;
                let (x, y) = best.translation();
                let travelled = (x - x0).hypot(y - y0);
                let backed_off = (safety > 0.0 && travelled - safety > min_dim * cfg.min_move_ratio)
                    .then(|| {
                        let keep = (travelled - safety) / travelled;
                        DTransformation::new(
                            d_transf.rotation(),
                            (x0 + (x - x0) * keep, y0 + (y - y0) * keep),
                        )
                    })
                    .filter(|&dt| probe.fits(dt));
                Some(backed_off.unwrap_or(best))
            }
        };

        if term.kill() {
            break;
        }
        if let Some(new_dt) = target {
            sep.move_item(pk, new_dt);
            moved += 1;
        }
    }
    moved
}

/// A rigid unit is probed as a whole against external hazards. Internal member
/// poses remain invariant and every member contributes to the wall bound.
fn slide_units_along(
    sep: &mut Separator,
    dir: SlideDir,
    cfg: &SlideConfig,
    term: &impl Terminator,
) -> usize {
    use jagua_rs::collision_detection::hazards::filter::HazKeyFilter;
    let axis_min = |b: jagua_rs::geometry::primitives::Rect| match dir {
        SlideDir::Left => b.x_min,
        SlideDir::Down => b.y_min,
    };
    let mut units: Vec<_> = sep
        .place_groups
        .placed_unit_representatives(&sep.prob.layout())
        .into_iter()
        .filter(|&id| !sep.frozen_mask.get(id).copied().unwrap_or(false))
        .map(|id| {
            let bound = sep
                .place_groups
                .member_item_ids(id)
                .iter()
                .filter_map(|&member| sep.placement_key(member))
                .map(|pk| axis_min(sep.prob.layout().placed_items()[pk].shape().bbox()))
                .fold(f32::INFINITY, f32::min);
            (id, bound)
        })
        .collect();
    units.sort_by_key(|&(id, bound)| (OrderedFloat(bound), id));
    let mut moved = 0;
    for (id, max_dist) in units {
        if term.kill() {
            break;
        }
        let Some(pk) = sep.placement_key(id) else {
            continue;
        };
        let base = sep.prob.layout().placed_items()[pk].d_transf();
        let members = sep.place_groups.member_item_ids(id);
        // Every part of every member is ignored while the unit moves.
        let filter = HazKeyFilter::from_keys(
            members
                .iter()
                .filter_map(|&id| sep.placement_key(id))
                .flat_map(|key| sep.prob.layout().haz_keys(key)),
        );
        let mut buffers: Vec<_> = members
            .iter()
            .map(|&id| (id, crate::util::parts::PartBuffers::new(sep.instance.item(id))))
            .collect();
        let min_dim = members
            .iter()
            .map(|&id| {
                let bbox = sep.instance.item(id).shape_cd().bbox();
                bbox.width().min(bbox.height())
            })
            .fold(f32::INFINITY, f32::min);
        let mut collides = |target| {
            let Ok(targets) =
                sep.place_groups
                    .target_transforms(Some(&sep.prob.layout()), id, target)
            else {
                return true;
            };
            targets.into_iter().any(|(member, dt)| {
                if term.kill() {
                    return true;
                }
                let item = sep.instance.item(member);
                let buffer = &mut buffers.iter_mut().find(|(id, _)| *id == member).unwrap().1;
                buffer
                    .transform(item, &dt.compose())
                    .iter()
                    .any(|shape| sep.prob.layout().cde().detect_poly_collision(shape, &filter))
            })
        };
        let dist = super::probe::max_slide_distance(
            base,
            dir.vec(),
            max_dist,
            min_dim * cfg.scan_step_ratio,
            cfg.n_bisections,
            &mut collides,
            &mut || term.kill(),
        );
        let backed = dist - min_dim * cfg.safety_ratio;
        let dist = if backed > min_dim * cfg.min_move_ratio
            && !collides(translated(base, dir.vec(), backed))
        {
            backed
        } else {
            dist
        };
        if term.kill() {
            break;
        }
        if dist > min_dim * cfg.min_move_ratio {
            sep.move_item(pk, translated(base, dir.vec(), dist));
            moved += 1;
        }
    }
    moved
}
