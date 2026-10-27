use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::filter::{HazardFilter, PlacedItemFilter};
use jagua_rs::entities::{Item, Layout, PItemKey};
use jagua_rs::geometry::{DTransformation, Transformation};

use crate::original_overlap::OriginalOverlapGuard;

/// 원본 외곽 계약을 어긴 자리에서 비켜 볼 거리(mm). 인증 여유(0.0005mm)보다 크고,
/// 줄어든 도형 사이 간격으로는 무시할 만큼 작다.
const ORIGINAL_NUDGES_MM: [f32; 3] = [0.003, 0.01, 0.03];

/// 비켜 볼 방향: 위, 아래, 뒤쪽 위·아래 대각, 뒤. 앞(왼쪽)으로는 이미 막혀 있다.
const NUDGE_DIRECTIONS: [(f32, f32); 5] = [
    (0.0, 1.0),
    (0.0, -1.0),
    (std::f32::consts::FRAC_1_SQRT_2, std::f32::consts::FRAC_1_SQRT_2),
    (std::f32::consts::FRAC_1_SQRT_2, -std::f32::consts::FRAC_1_SQRT_2),
    (1.0, 0.0),
];

/// 배치된 아이템 하나를 대상으로 "자기 자신을 무시한" 충돌 질의를 반복 수행하기 위한 도우미.
///
/// jagua-rs의 `HazKey`가 `HazardFilter<BasicHazardEntity>`를 구현하므로, 레이아웃에서 아이템을 실제로
/// 제거했다가 다시 넣지 않고도 가상의 위치에 대한 충돌 여부를 물어볼 수 있다.
/// (제거/삽입은 quadtree를 갱신하므로 반복 질의에는 비용이 크다)
pub struct CollisionProbe<'a> {
    layout: &'a Layout,
    item: &'a Item,
    /// 이 아이템이 유발하는 hazard(모든 부분). 질의 시 무시된다.
    self_filter: PlacedItemFilter,
    /// 변환 결과를 담아둘 버퍼 (매 질의마다 새로 할당하지 않기 위함), 부분마다 하나.
    buff: crate::util::parts::PartBuffers,
    /// 디버깅용 평가 카운터 (외부 getter 는 I-17 로 제거됨)
    #[allow(dead_code)]
    n_evals: usize,
    /// 옮기는 조각 자신. 원본 외곽 검사에서 그 현재 자리를 빼는 데 쓴다.
    pk: PItemKey,
    /// 겹침 허용 모드의 원본 외곽 검증기. `None` 이면 CDE 만 본다.
    original: Option<&'a OriginalOverlapGuard>,
}

impl<'a> CollisionProbe<'a> {
    pub fn new(layout: &'a Layout, item: &'a Item, pk: PItemKey) -> Self {
        Self {
            layout,
            item,
            self_filter: PlacedItemFilter(pk),
            buff: crate::util::parts::PartBuffers::new(item),
            n_evals: 0,
            pk,
            original: None,
        }
    }

    /// 원본 외곽 겹침 계약도 함께 보게 한다 ([`Self::original_ok`], [`Self::fits`]).
    pub fn with_original(mut self, guard: Option<&'a OriginalOverlapGuard>) -> Self {
        self.original = guard;
        self
    }

    /// `dt` 에서 원본 외곽이 다른 조각과 허용치 이상 겹치지 않는지. 검증기가 없으면 참.
    pub fn original_ok(&self, dt: DTransformation) -> bool {
        let Some(guard) = self.original else {
            return true;
        };
        guard
            .outline(self.item.idx(), dt)
            .and_then(|outline| {
                guard.outline_allowed_in_layout_excluding(self.layout, &outline, Some(self.pk))
            })
            .unwrap_or(false)
    }

    /// 충돌 도형도, 원본 외곽 계약도 지키는 자리인가.
    pub fn fits(&mut self, dt: DTransformation) -> bool {
        !self.collides(dt) && self.original_ok(dt)
    }

    /// 충돌 도형 기준으로 `dir` 방향 `dist` 만큼 민 자리를 원본 외곽 계약까지 지키는 자리로 고친다.
    ///
    /// 줄어든 도형끼리 맞닿으면 원본 외곽은 허용치를 꽉 채워 겹치고, 인증은 격자 여유만큼
    /// 더 엄격해서 그 자리를 기각한다. 압축이 끝난 배치는 위아래 조각과 거의 맞닿아 있어,
    /// 비스듬히 밀면 그 조각과의 간격이 바로 기준 아래로 떨어진다. 그래서 계약을 어기면
    ///
    /// 1. 그 자리에서 위·아래·뒤로 아주 조금([`ORIGINAL_NUDGES_MM`]) 비켜 본다 — 민 거리를
    ///    거의 그대로 살린다.
    /// 2. 그래도 안 되면 `[0, dist]` 를 이분탐색해 계약을 지키는 가장 먼 거리로 줄인다.
    ///
    /// 출발점은 계약을 지킨다고 본다. 검증기가 없으면 민 자리를 그대로 돌려준다.
    pub fn settle_original(
        &mut self,
        base: DTransformation,
        dir: (f32, f32),
        dist: f32,
        n_bisections: usize,
    ) -> DTransformation {
        let target = translated(base, dir, dist);
        if self.original.is_none() || dist <= 0.0 || self.original_ok(target) {
            return target;
        }
        for &step in ORIGINAL_NUDGES_MM.iter() {
            for &(nx, ny) in NUDGE_DIRECTIONS.iter() {
                let nudged = translated(target, (nx, ny), step);
                if self.fits(nudged) {
                    return nudged;
                }
            }
        }
        let (mut lo, mut hi) = (0.0f32, dist);
        for _ in 0..n_bisections {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            if self.fits(translated(base, dir, mid)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        translated(base, dir, lo)
    }

    // I-17: 사용처 없는 `n_evals()` getter 는 제거했다 (`n_evals` 필드는
    // 디버깅 시 워치 대상으로 남겨 둔다).

    /// `dt` 위치에 아이템을 놓았을 때 (자기 자신을 제외하고) 충돌이 발생하는지.
    /// 컨테이너 경계 이탈도 hazard로 등록되어 있으므로 함께 검출된다.
    pub fn collides(&mut self, dt: DTransformation) -> bool {
        self.n_evals += 1;
        let transf: Transformation = dt.into();
        let filter = self.self_filter;
        self.check(&transf, &filter)
    }

    fn check(&mut self, transf: &Transformation, filter: &impl HazardFilter<BasicHazardEntity>) -> bool {
        let cde = self.layout.cde();
        // 1단계: surrogate(내부 pole/pier) 기반 fail-fast. 충돌이면 실제로도 반드시 충돌.
        // 2단계: 정확한 다각형 검사. 다중 폴리곤 조각은 부분마다 본다.
        crate::util::parts::item_collides(cde, cde, self.item, &mut self.buff, transf, filter)
    }

    /// `base` 위치에서 `dir` 방향으로 충돌 없이 이동할 수 있는 최대 거리를 구한다.
    ///
    /// 충돌 가능 여부는 거리에 대해 단조롭지 않기 때문에(예: 3에서는 막히지만 5에서는
    /// 다시 비어 있을 수 있다) 먼저 `scan_step` 간격으로 전방을 훑어 **최초로 막히는**
    /// 구간을 찾고, 그 구간만 이분탐색한다. 반환값은 항상 실제로 충돌 없음이 검증된 거리다.
    pub fn max_slide(
        &mut self,
        base: DTransformation,
        dir: (f32, f32),
        max_dist: f32,
        scan_step: f32,
        n_bisections: usize,
    ) -> f32 {
        self.max_slide_with_control(base, dir, max_dist, scan_step, n_bisections, &mut || false)
    }

    pub fn max_slide_with_control(
        &mut self,
        base: DTransformation,
        dir: (f32, f32),
        max_dist: f32,
        scan_step: f32,
        n_bisections: usize,
        should_stop: &mut dyn FnMut() -> bool,
    ) -> f32 {
        max_slide_distance(
            base,
            dir,
            max_dist,
            scan_step,
            n_bisections,
            &mut |dt| self.collides(dt),
            should_stop,
        )
    }
}

pub(crate) fn max_slide_distance(
    base: DTransformation,
    dir: (f32, f32),
    max_dist: f32,
    scan_step: f32,
    n_bisections: usize,
    collides: &mut dyn FnMut(DTransformation) -> bool,
    should_stop: &mut dyn FnMut() -> bool,
) -> f32 {
    if max_dist <= 0.0 || !max_dist.is_finite() {
        return 0.0;
    }
    let step = scan_step.max(max_dist / 512.0);

    // 전방 스캔: 마지막으로 비어 있던 거리와 최초로 막힌 거리를 찾는다.
    let mut lo = 0.0f32; // 충돌 없음이 확인된 거리
    let mut hi; // 충돌이 확인된 거리
    let mut t = step;
    loop {
        if should_stop() {
            return lo;
        }
        let probe_t = t.min(max_dist);
        if collides(translated(base, dir, probe_t)) {
            hi = probe_t;
            break;
        }
        lo = probe_t;
        if probe_t >= max_dist {
            // 끝까지 비어 있다
            return max_dist;
        }
        t += step;
    }

    // 이분탐색으로 [lo, hi) 구간을 좁힌다.
    for _ in 0..n_bisections {
        if should_stop() {
            break;
        }
        let mid = 0.5 * (lo + hi);
        if mid <= lo || mid >= hi {
            break;
        }
        if collides(translated(base, dir, mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    lo
}

/// `dt`를 `dir` 방향으로 `dist`만큼 평행이동한 변환.
/// `DTransformation`은 회전 후 평행이동이므로 translation에 그대로 더하면 월드 좌표 이동이 된다.
pub fn translated(dt: DTransformation, dir: (f32, f32), dist: f32) -> DTransformation {
    let (x, y) = dt.translation();
    DTransformation::new(dt.rotation(), (x + dir.0 * dist, y + dir.1 * dist))
}
