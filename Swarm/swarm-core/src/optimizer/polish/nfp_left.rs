//! Polish 의 **NFP 왼쪽 재배치** 패스. 설정과 동기는 [`NfpLeftConfig`] 참조.

use crate::config::NfpLeftConfig;
use crate::nfp_lbf::cache::NfpLbfCache;
use crate::optimizer::polish::probe::CollisionProbe;
use crate::optimizer::separator::Separator;
use crate::sample::nfp_sampler::{LeftwardWindow, leftward_line_candidates};
use crate::util::terminator::Terminator;
use itertools::Itertools;
use jagua_rs::entities::{PItemKey};
use ordered_float::OrderedFloat;

/// 왼쪽 조각부터 차례로, 같은 높이 근처의 더 왼쪽 빈자리로 옮긴다.
///
/// 앞 조각이 비운 자리를 뒤 조각이 이어받도록 왼쪽 것부터 본다. 후보는 왼쪽 것부터
/// `max_verify` 개까지 CDE 와 원본 외곽 계약으로 검증하고, 처음 통과한 자리로 옮긴다.
/// 고정 조각과 묶음(place group)은 건드리지 않는다. 반환값은 옮긴 조각 수.
pub fn nfp_left_pass(
    sep: &mut Separator,
    cfg: &NfpLeftConfig,
    cache: &mut NfpLbfCache,
    term: &impl Terminator,
) -> usize {
    if sep.has_place_groups() {
        return 0;
    }
    let order: Vec<PItemKey> = sep
        .prob
        .layout()
        .placed_items()
        .iter()
        .filter(|(_, pi)| !sep.frozen_mask.get(pi.item().idx()).copied().unwrap_or(false))
        .sorted_by_key(|(_, pi)| OrderedFloat(pi.shape().bbox().x_min))
        .map(|(pk, _)| pk)
        .collect();

    let mut moved = 0;
    for pk in order {
        if term.kill() {
            break;
        }
        let Some(pi) = sep.prob.layout().placed_items().get(pk) else {
            continue;
        };
        let item_id = pi.item().idx();
        let target = {
            let item = sep.instance.item(item_id);
            let bbox = item.shape_cd().bbox();
            let window = LeftwardWindow {
                reach: (bbox.width() * cfg.reach_ratio) as f64,
                y_band: (bbox.height() * cfg.y_band_ratio) as f64,
                n_lines_each_side: cfg.n_lines_each_side,
                min_gain: cfg.min_gain_mm as f64,
            };
            let candidates = leftward_line_candidates(
                &sep.prob.layout(),
                item,
                pk,
                cache,
                window,
                &mut || term.kill(),
            );
            let mut probe = CollisionProbe::new(&sep.prob.layout(), item, pk)
                .with_original(sep.original_guard());
            candidates
                .into_iter()
                .take(cfg.max_verify)
                .find(|&dt| probe.fits(dt))
        };
        if term.kill() {
            break;
        }
        if let Some(dt) = target {
            sep.move_item(pk, dt);
            moved += 1;
        }
    }
    moved
}
