use crate::eval::sample_eval::{SampleEval, SampleEvaluator};
use jagua_rs::collision_detection::CDEngine;
use jagua_rs::collision_detection::hazards::BasicHazardEntity;
use jagua_rs::collision_detection::hazards::collector::BasicHazardCollector;
use jagua_rs::collision_detection::hazards::filter::NoFilter;
use jagua_rs::entities::Item;
use jagua_rs::entities::Layout;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::geo_traits::CollidesWith;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::SPolygon;

/// Simple evaluator for the Left-Bottom-Fill constructor.
/// Basically either returns [SampleEval::Invalid] in case of any collision or [SampleEval::Clear] with a loss value
/// that rewards placements that are closer to the left-bottom corner of the container.
pub struct LBFEvaluator<'a> {
    layout: &'a Layout,
    item: &'a Item,
    /// Envelope buffer: the position score reads its POI and bounding box.
    shape_buff: SPolygon,
    /// One buffer per part for the exact collision check.
    parts: crate::util::parts::PartBuffers,
    x_multiplier: f32,
    y_multiplier: f32,
    /// `Some(4·mid_y)` 이면 아래쪽뿐 아니라 **위쪽 모서리에 붙는 것도 같은 값**으로
    /// 친다. `None`(기본)이면 기존 좌·하단 규칙과 한 비트도 다르지 않다.
    ///
    /// # 왜
    ///
    /// 기존 손실은 y 가 작을수록 좋다 — 그래서 모든 조각이 아래부터 쌓인다. 실측
    /// (여성WSjk 156-패널 초기 배치): 하단 모서리에 닿는 조각 16개, 상단 6개로
    /// 2.7배 편향이다. 참조 네스터의 77% 초기 배치는 큰 패널을 위·아래 모서리에
    /// 번갈아 붙여 **프론티어를 둘** 만들고 그 사이를 채운다.
    ///
    /// 값을 `2·(y_min + y_max)` 로 두는 이유는 손실의 y 항이 `poi.y + bbox.y_max`
    /// 라 y 의 **두 배** 스케일이기 때문이다 — `Rect::corners()[0]` 은 최소 모서리가
    /// 아니라 **(x_max, y_max)** 다. 반사 `y → (y_min + y_max) − y` 를 적용하면 poi 는
    /// `(y_min+y_max) − poi.y` 가 되고 bbox 의 **위** 모서리가 아래 모서리에서 오므로
    /// 항이 `2·(y_min+y_max) − poi.y − bbox.y_min` 이 된다. 두 값 중 작은 쪽을 쓰면
    /// "가까운 가로 모서리까지의 거리" 가 되고, 좌측 우선은 그대로 유지된다.
    ///
    /// 여기서 `y_max` 를 쓰면 위쪽 가지가 정확히 **조각 높이만큼** 싸져서, 균형이
    /// 아니라 "큰 조각일수록 위로 몰아붙이는" 편향이 된다.
    double_sided_y: Option<f32>,
    /// 이 x 를 넘는 자리는 유효하지 않다고 본다. `None`(기본)이면 제한 없음.
    ///
    /// 구간 완성(band completion)에 쓴다 — 현재 구간이 위아래로 찰 때까지는
    /// 프론티어를 오른쪽으로 밀지 않는다.
    band_x_max: Option<f32>,
    n_evals: usize,
    exact_cde: super::flat_collision_index::FlatCollisionIndex,
    #[cfg(feature = "init-profile")]
    score_pruned: usize,
}

/// Build an exact-query engine without quadtree children.
///
/// Jagua 0.7.2's optimized `Edge::collides_with_quadrants` can omit one of the
/// quadrants when an edge lies exactly on a node bisector. A depth-zero engine
/// uses the same hazards and exact polygon predicates, but never calls that
/// quadrant-routing optimization. The original layout CDE remains in use for
/// the inexpensive surrogate query.
pub(crate) fn flat_exact_cde(layout: &Layout) -> CDEngine {
    let source = layout.cde();
    let mut config = source.config();
    config.quadtree_depth = 0;
    CDEngine::new(source.bbox(), source.hazards().cloned().collect(), config)
}

/// Rechecks a complete layout with the same depth-zero exact engine used to
/// authorize LBF/profile candidates and counts the colliding hazard classes.
/// Static hazards use the flat CDE; placed-item pairs use one direct symmetric
/// edge/containment test per unordered pair to avoid query/hazard asymmetry.
pub fn flat_layout_collision_counts(
    layout: &Layout,
) -> (usize, usize, usize, Option<(usize, usize)>) {
    flat_layout_collision_counts_with_frozen(layout, &[])
}

pub fn flat_layout_collision_counts_with_frozen(
    layout: &Layout,
    frozen: &[bool],
) -> (usize, usize, usize, Option<(usize, usize)>) {
    flat_layout_collision_counts_with_control(layout, frozen, &|| false).unwrap()
}

/// None means interrupted, never a collision-free partial result.
pub(crate) fn flat_layout_collision_counts_with_control(
    layout: &Layout,
    frozen: &[bool],
    cancelled: &impl Fn() -> bool,
) -> Option<(usize, usize, usize, Option<(usize, usize)>)> {
    flat_layout_collision_counts_with_control_and_groups(
        layout,
        frozen,
        &crate::optimizer::place_groups::PlaceGroups::default(),
        cancelled,
    )
}

pub(crate) fn flat_layout_collision_counts_with_control_and_groups(
    layout: &Layout,
    frozen: &[bool],
    groups: &crate::optimizer::place_groups::PlaceGroups,
    cancelled: &impl Fn() -> bool,
) -> Option<(usize, usize, usize, Option<(usize, usize)>)> {
    if cancelled() {
        return None;
    }
    let cde = flat_exact_cde(layout);
    // Every part of every placed item (the flat engine holds the same hazards).
    let mut dynamic_hazards = Vec::new();
    for (pk, pi) in layout.placed_items().iter() {
        for part in 0..pi.n_parts() {
            let key = cde.haz_key(&pi.part_entity(pk, part)).expect(crate::dmsg!(
                "every placed part must have a cloned flat-CDE hazard"
            ));
            dynamic_hazards.push((key, cde.hazards_map()[key].entity));
        }
    }
    let mut exterior = 0;
    let mut placed_items = 0;
    let mut other = 0;
    let mut first_placed_pair = None;
    for placed in layout.placed_items().values() {
        if cancelled() {
            return None;
        }
        let mut collisions = BasicHazardCollector::new();
        for &(key, entity) in &dynamic_hazards {
            collisions.insert(key, entity);
        }
        for shape in placed.shapes() {
            cde.collect_poly_collisions(shape, &NoFilter, &mut collisions);
        }
        for (_, entity) in collisions.iter() {
            match entity {
                BasicHazardEntity::Exterior => {
                    if !crate::util::frozen::is_frozen(frozen, placed.item().idx()) {
                        exterior += 1;
                    }
                }
                BasicHazardEntity::PlacedItem { .. } => {}
                BasicHazardEntity::Hole { .. } | BasicHazardEntity::InferiorQualityZone { .. } => {
                    other += 1
                }
            }
        }
    }
    let placed = layout.placed_items().values().collect::<Vec<_>>();
    for (index, first) in placed.iter().enumerate() {
        if cancelled() {
            return None;
        }
        for second in placed.iter().skip(index + 1) {
            if groups.same_group(first.item().idx(), second.item().idx()) {
                continue;
            }
            if crate::util::frozen::is_frozen(frozen, first.item().idx())
                && crate::util::frozen::is_frozen(frozen, second.item().idx())
            {
                continue;
            }
            if crate::util::parts::placed_pair_collide(
                first,
                second,
                polygons_collide_bidirectionally,
            ) {
                placed_items += 1;
                first_placed_pair.get_or_insert((first.item().idx(), second.item().idx()));
            }
        }
    }
    Some((exterior, placed_items, other, first_placed_pair))
}

pub fn polygons_collide_bidirectionally(a: &SPolygon, b: &SPolygon) -> bool {
    if !a.bbox().collides_with(&b.bbox()) {
        return false;
    }
    // Size-region solutions are solved in lane-local coordinates and then
    // translated to their global X offset. At large millimetre coordinates a
    // mathematically exact contact can become a sub-ULP overlap after the
    // public f32 transformation is formed. Treat only that numerically
    // indistinguishable overlap as contact; material crossings still use the
    // symmetric edge/containment authority below.
    let overlap_x = a.bbox().x_max.min(b.bbox().x_max) - a.bbox().x_min.max(b.bbox().x_min);
    let overlap_y = a.bbox().y_max.min(b.bbox().y_max) - a.bbox().y_min.max(b.bbox().y_min);
    let scale = [
        a.bbox().x_min,
        a.bbox().x_max,
        a.bbox().y_min,
        a.bbox().y_max,
        b.bbox().x_min,
        b.bbox().x_max,
        b.bbox().y_min,
        b.bbox().y_max,
    ]
    .into_iter()
    .map(f32::abs)
    .fold(1.0f32, f32::max);
    let contact_tolerance = 1.0e-4f32.max(2.0 * f32::EPSILON * scale);
    if overlap_x.min(overlap_y) <= contact_tolerance {
        return false;
    }
    if a.edge_iter().any(|a_edge| {
        b.edge_iter()
            .any(|b_edge| edges_cross_materially(&a_edge, &b_edge, contact_tolerance))
    }) {
        return true;
    }
    if a.collides_with(&b.poi().center) || b.collides_with(&a.poi().center) {
        return true;
    }
    // 변과 변이 «엇갈려» 만나지 않는 정렬된 겹침은 위 두 검사를 모두 빠져나간다. 예: 높이가 같은
    // 직사각형 둘이 같은 y 에서 x 로만 겹치면 모든 교차가 «꼭짓점이 상대 변의 연장선 위» 라
    // `edges_cross_materially` 가 거짓이고, 겹친 폭이 반을 넘지 않으면 POI 도 서로의 밖이다
    // (손 계산: [0,10]² 와 [6,16]x[0,10] 은 4x10mm 가 겹치는데 «충돌 없음» 이었다).
    // 두 bbox 교집합의 한가운데와 그 둘레 네 점이 모두 양쪽 도형 안이면 접촉 허용치보다 큰
    // 면적이 실제로 겹친 것이다. 접촉(겹침 <= 허용치)은 위에서 이미 걸러졌다.
    let reach = (4.0 * contact_tolerance).min(0.25 * overlap_x.min(overlap_y));
    let cx = 0.5 * (a.bbox().x_min.max(b.bbox().x_min) + a.bbox().x_max.min(b.bbox().x_max));
    let cy = 0.5 * (a.bbox().y_min.max(b.bbox().y_min) + a.bbox().y_max.min(b.bbox().y_max));
    [
        (0.0, 0.0),
        (reach, 0.0),
        (-reach, 0.0),
        (0.0, reach),
        (0.0, -reach),
    ]
    .into_iter()
    .all(|(dx, dy)| {
        let probe = jagua_rs::geometry::primitives::Point(cx + dx, cy + dy);
        a.collides_with(&probe) && b.collides_with(&probe)
    })
}

fn edges_cross_materially(
    a: &jagua_rs::geometry::primitives::Edge,
    b: &jagua_rs::geometry::primitives::Edge,
    tolerance: f32,
) -> bool {
    fn signed_area(
        start: jagua_rs::geometry::primitives::Point,
        end: jagua_rs::geometry::primitives::Point,
        point: jagua_rs::geometry::primitives::Point,
    ) -> f64 {
        let dx = f64::from(end.0) - f64::from(start.0);
        let dy = f64::from(end.1) - f64::from(start.1);
        dx * (f64::from(point.1) - f64::from(start.1))
            - dy * (f64::from(point.0) - f64::from(start.0))
    }

    fn materially_opposite(first: f64, second: f64, edge_length: f64, tolerance: f64) -> bool {
        first.is_sign_positive() != second.is_sign_positive()
            && first.abs() > tolerance * edge_length
            && second.abs() > tolerance * edge_length
    }

    let a_len = (f64::from(a.end.0) - f64::from(a.start.0))
        .hypot(f64::from(a.end.1) - f64::from(a.start.1));
    let b_len = (f64::from(b.end.0) - f64::from(b.start.0))
        .hypot(f64::from(b.end.1) - f64::from(b.start.1));
    let a_first = signed_area(a.start, a.end, b.start);
    let a_second = signed_area(a.start, a.end, b.end);
    let b_first = signed_area(b.start, b.end, a.start);
    let b_second = signed_area(b.start, b.end, a.end);
    materially_opposite(a_first, a_second, a_len, f64::from(tolerance))
        && materially_opposite(b_first, b_second, b_len, f64::from(tolerance))
}

impl<'a> LBFEvaluator<'a> {
    pub fn new(layout: &'a Layout, item: &'a Item, x_multiplier: f32, y_multiplier: f32) -> Self {
        Self {
            layout,
            item,
            shape_buff: item.shape_cd().as_ref().clone(),
            parts: crate::util::parts::PartBuffers::new(item),
            x_multiplier,
            y_multiplier,
            double_sided_y: None,
            band_x_max: None,
            n_evals: 0,
            exact_cde: super::flat_collision_index::FlatCollisionIndex::new(layout.cde()),
            #[cfg(feature = "init-profile")]
            score_pruned: 0,
        }
    }

    /// 후보의 오른쪽 끝을 `x` 로 제한한다. [`band_x_max`](Self::band_x_max) 참고.
    pub fn band_x_max(mut self, x: Option<f32>) -> Self {
        self.band_x_max = x;
        self
    }

    /// 위·아래 두 모서리를 같은 값으로 치는 변형. [`double_sided_y`](Self::double_sided_y)
    /// 참고. 컨테이너 높이는 레이아웃에서 읽으므로 호출자가 넘기지 않는다.
    pub fn double_sided(mut self, enabled: bool) -> Self {
        self.double_sided_y = enabled.then(|| {
            let bbox = self.layout.container().outer_cd().bbox();
            2.0 * (bbox.y_min + bbox.y_max)
        });
        self
    }
}

impl<'a> SampleEvaluator for LBFEvaluator<'a> {
    fn evaluate_sample(
        &mut self,
        dt: DTransformation,
        upper_bound: Option<SampleEval>,
    ) -> SampleEval {
        self.n_evals += 1;
        let cde = self.layout.cde();
        let transf = dt.into();
        // The envelope carries the position score.
        self.shape_buff
            .transform_from(&self.item.shape_cd(), &transf);
        // 구간 밖으로 프론티어를 미는 자리는 이 구간에서는 쓸 수 없다.
        if self
            .band_x_max
            .is_some_and(|cap| self.shape_buff.bbox().x_max > cap)
        {
            return SampleEval::Invalid;
        }
        // Compute the identical transformed-geometry score before costly queries.
        let poi = self.shape_buff.poi().center;
        let bbox_corner = self.shape_buff.bbox().corners()[0];
        let from_bottom = poi.1 + bbox_corner.1;
        let y_term = match self.double_sided_y {
            // `y → 2·mid_y − y` 로 뒤집은 같은 항. 아래·위 중
            // 가까운 모서리까지의 거리가 된다.
            Some(fold) => {
                let from_top = fold - poi.1 - self.shape_buff.bbox().y_min;
                from_bottom.min(from_top)
            }
            None => from_bottom,
        };
        let loss = self.x_multiplier * (poi.0 + bbox_corner.0) + self.y_multiplier * y_term;
        let score = SampleEval::Clear { loss };
        // Independent samples cannot enter BestSamples when strictly worse.
        // Descent bypasses this bound because FPA equality is not transitive.
        if loss.is_finite()
            && matches!(upper_bound, Some(SampleEval::Clear { .. }))
            && upper_bound.is_some_and(|bound| score > bound)
        {
            #[cfg(feature = "init-profile")]
            {
                self.score_pruned += 1;
            }
            return SampleEval::Invalid;
        }
        for part in self.item.parts_cd() {
            if cde.detect_surrogate_collision(part.surrogate(), &transf, &NoFilter) {
                return SampleEval::Invalid;
            }
        }
        if self
            .parts
            .transform(self.item, &transf)
            .iter()
            .any(|shape| self.exact_cde.detect_poly_collision(shape, &NoFilter))
        {
            return SampleEval::Invalid;
        }
        score
    }

    fn evaluate_descent_sample(
        &mut self,
        dt: DTransformation,
        _: Option<SampleEval>,
    ) -> SampleEval {
        self.evaluate_sample(dt, None)
    }

    fn n_evals(&self) -> usize {
        self.n_evals
    }
}

#[cfg(feature = "init-profile")]
impl Drop for LBFEvaluator<'_> {
    fn drop(&mut self) {
        crate::init_profile::count("lbf.score_pruned", self.score_pruned);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Historical flat evaluator retained only as a differential test oracle.
    struct ReferenceEvaluator<'a> {
        inner: LBFEvaluator<'a>,
        flat: CDEngine,
    }
    impl<'a> SampleEvaluator for ReferenceEvaluator<'a> {
        fn evaluate_sample(
            &mut self,
            dt: DTransformation,
            _upper_bound: Option<SampleEval>,
        ) -> SampleEval {
            self.inner.n_evals += 1;
            let cde = self.inner.layout.cde();
            let transf = dt.into();
            // Surrogate fast path and exact contour, part by part.
            if crate::util::parts::item_collides(
                cde,
                &self.flat,
                self.inner.item,
                &mut self.inner.parts,
                &transf,
                &NoFilter,
            ) {
                return SampleEval::Invalid; // Some part collides with something
            }
            // The envelope carries the position score.
            self.inner
                .shape_buff
                .transform_from(&self.inner.item.shape_cd(), &transf);
            // 구간 밖으로 프론티어를 미는 자리는 이 구간에서는 쓸 수 없다.
            if self
                .inner
                .band_x_max
                .is_some_and(|cap| self.inner.shape_buff.bbox().x_max > cap)
            {
                return SampleEval::Invalid;
            }
            // No collisions
            let poi = self.inner.shape_buff.poi().center;
            let bbox_corner = self.inner.shape_buff.bbox().corners()[0];
            let from_bottom = poi.1 + bbox_corner.1;
            let y_term = match self.inner.double_sided_y {
                // `y → 2·mid_y − y` 로 뒤집은 같은 항. 아래·위 중
                // 가까운 모서리까지의 거리가 된다.
                Some(fold) => {
                    let from_top = fold - poi.1 - self.inner.shape_buff.bbox().y_min;
                    from_bottom.min(from_top)
                }
                None => from_bottom,
            };
            let loss = self.inner.x_multiplier * (poi.0 + bbox_corner.0)
                + self.inner.y_multiplier * y_term;
            SampleEval::Clear { loss }
        }

        fn n_evals(&self) -> usize {
            self.inner.n_evals
        }
    }

    #[test]
    fn bounded_index_search_preserves_candidates_and_rng_stream() {
        use jagua_rs::probs::spp::{
            entities::{SPPlacement, SPProblem},
            io::ext_repr::ExtSPInstance,
        };
        use rand::{RngExt, SeedableRng, rngs::Xoshiro256PlusPlus};
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name":"differential", "strip_height":100., "items":[
                {"id":0,"demand":1,"allowed_orientations":[0.],"shape":{"type":"simple_polygon","data":[[0.,0.],[30.,0.],[30.,25.],[0.,25.]]}},
                {"id":1,"demand":1,"allowed_orientations":[0.,90.,180.],"shape":{"type":"simple_polygon","data":[[0.,0.],[25.,0.],[25.,8.],[8.,8.],[8.,30.],[0.,30.]]}}
            ]
        })).unwrap();
        let cfg = crate::config::DEFAULT_SWARM_CONFIG;
        let instance =
            crate::util::io::import_spp_instance(&ext, cfg.cde_config, None, None, None).unwrap();
        let mut problem = SPProblem::new(instance.clone()).unwrap();
        problem.change_strip_width(200.).unwrap();
        problem.place_item(SPPlacement {
            item_idx: 0,
            d_transf: DTransformation::new(0., (50., 40.)),
        });
        for seed in 1..=8 {
            for (x_weight, y_weight, double_sided, band) in [
                (1., 1., false, None),
                (2., -1., true, Some(150.)),
                (-1., 1., false, None),
            ] {
                let make = || {
                    LBFEvaluator::new(problem.layout(), instance.item(1), x_weight, y_weight)
                        .double_sided(double_sided)
                        .band_x_max(band)
                };
                let reference = ReferenceEvaluator {
                    inner: make(),
                    flat: flat_exact_cde(problem.layout()),
                };
                let mut before = Xoshiro256PlusPlus::seed_from_u64(seed);
                let mut after = before.clone();
                let a = crate::sample::search::search_placement_with_control(
                    problem.layout(),
                    instance.item(1),
                    None,
                    reference,
                    crate::consts::LBF_SAMPLE_CONFIG,
                    &mut before,
                    &mut || false,
                );
                let b = crate::sample::search::search_placement_with_control(
                    problem.layout(),
                    instance.item(1),
                    None,
                    make(),
                    crate::consts::LBF_SAMPLE_CONFIG,
                    &mut after,
                    &mut || false,
                );
                assert_eq!(a, b, "seed={seed}, weights={x_weight},{y_weight}");
                assert_eq!(
                    before.random::<u64>(),
                    after.random::<u64>(),
                    "RNG stream changed"
                );
            }
        }
    }

    #[test]
    fn bounded_lbf_rejects_strictly_worse_but_keeps_equal_scores() {
        use jagua_rs::probs::spp::{entities::SPProblem, io::ext_repr::ExtSPInstance};
        let ext: ExtSPInstance = serde_json::from_value(serde_json::json!({
            "name":"bounds", "strip_height":100., "items":[{"id":0,"demand":1,
            "allowed_orientations":[0.],"shape":{"type":"simple_polygon","data":[[0.,0.],[10.,0.],[10.,8.],[0.,8.]]}}]
        })).unwrap();
        let cfg = crate::config::DEFAULT_SWARM_CONFIG;
        let instance =
            crate::util::io::import_spp_instance(&ext, cfg.cde_config, None, None, None).unwrap();
        let mut problem = SPProblem::new(instance.clone()).unwrap();
        problem.change_strip_width(100.).unwrap();
        let mut eval = LBFEvaluator::new(problem.layout(), instance.item(0), 1., 1.);
        let dt = DTransformation::new(0., (40., 40.));
        let full = eval.evaluate_sample(dt, None);
        let SampleEval::Clear { loss } = full else {
            panic!("fixture must be clear")
        };
        assert_eq!(
            eval.evaluate_sample(dt, Some(full)),
            full,
            "ties may move coordinate descent"
        );
        assert_eq!(
            eval.evaluate_sample(
                dt,
                Some(SampleEval::Clear {
                    loss: loss.next_down()
                })
            ),
            full,
            "approximate FPA ties must also survive"
        );
        assert_eq!(
            eval.evaluate_sample(dt, Some(SampleEval::Clear { loss: loss - 10. })),
            SampleEval::Invalid,
            "a provably worse candidate should be pruned"
        );
        assert_eq!(
            eval.evaluate_descent_sample(dt, Some(SampleEval::Clear { loss: loss - 10. })),
            full,
            "descent must preserve pairwise FPA ordering"
        );
    }

    /// 양쪽 앵커링의 반사 항이 실제로 대칭인지. 아래에 딱 붙인 조각과 그것을 위아래로
    /// 뒤집어 위에 딱 붙인 조각의 y 항이 같아야 한다. 반사에서 `y_min` 대신 `y_max` 를
    /// 쓰면 정확히 조각 높이만큼 어긋나고, 큰 조각일수록 위로 몰린다.
    #[test]
    fn double_sided_y_term_is_symmetric_about_the_container_midline() {
        let (y_min, y_max) = (0.0f32, 100.0f32);
        let fold = 2.0 * (y_min + y_max);
        // 아래에 붙인 조각: poi 는 높이 h 의 중앙, bbox 는 [0, h].
        let h = 30.0f32;
        let bottom_poi = h * 0.5;
        let bottom_term = bottom_poi + h; // poi.y + bbox.y_max
        // 같은 조각을 위에 붙이면 poi 는 y_max − h/2, bbox 는 [y_max − h, y_max].
        let top_poi = y_max - h * 0.5;
        let top_bbox_y_min = y_max - h;
        let top_term = fold - top_poi - top_bbox_y_min;
        assert!(
            (bottom_term - top_term).abs() < 1.0e-3,
            "bottom {bottom_term} vs top {top_term}"
        );
    }
    use crate::config::DEFAULT_SWARM_CONFIG;
    use jagua_rs::collision_detection::hazards::Hazard;
    use jagua_rs::geometry::primitives::Point;
    use std::sync::Arc;

    fn polygon(points: &[(f32, f32)]) -> SPolygon {
        SPolygon::new(points.iter().copied().map(Point::from).collect()).unwrap()
    }

    fn flat_cde_with_interior_hazard(shape: SPolygon) -> CDEngine {
        let exterior = Arc::new(polygon(&[
            (0.0, 0.0),
            (10_000.0, 0.0),
            (10_000.0, 1_000.0),
            (0.0, 1_000.0),
        ]));
        let mut config = DEFAULT_SWARM_CONFIG.cde_config;
        config.quadtree_depth = 0;
        CDEngine::new(
            exterior.bbox(),
            vec![
                Hazard::new(BasicHazardEntity::Exterior, exterior),
                Hazard::new(BasicHazardEntity::Hole { idx: 0 }, Arc::new(shape)),
            ],
            config,
        )
    }

    #[test]
    fn bidirectional_polygon_authority_covers_edges_containment_and_disjointness() {
        let outer = polygon(&[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]);
        let contained = polygon(&[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)]);
        let crossing = polygon(&[(3.0, -1.0), (5.0, -1.0), (5.0, 1.0), (3.0, 1.0)]);
        let disjoint = polygon(&[(6.0, 0.0), (7.0, 0.0), (7.0, 1.0), (6.0, 1.0)]);

        assert!(polygons_collide_bidirectionally(&outer, &contained));
        assert!(polygons_collide_bidirectionally(&contained, &outer));
        assert!(polygons_collide_bidirectionally(&outer, &crossing));
        assert!(!polygons_collide_bidirectionally(&outer, &disjoint));
    }

    /// 높이가 같은 직사각형이 같은 y 에서 x 로만 겹치면 변끼리 «엇갈려» 만나는 곳이 없고,
    /// 겹친 폭이 반 이하라 POI 도 서로의 밖이다. 그래도 겹침이다.
    #[test]
    fn aligned_rectangles_overlapping_along_collinear_edges_collide() {
        let a = polygon(&[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]);
        let overlapping = polygon(&[(6.0, 0.0), (16.0, 0.0), (16.0, 10.0), (6.0, 10.0)]);
        assert!(polygons_collide_bidirectionally(&a, &overlapping));
        assert!(polygons_collide_bidirectionally(&overlapping, &a));

        // 얇은 겹침(0.05mm)도 허용치(1e-4mm)를 넘으면 겹침이다.
        let sliver = polygon(&[(9.95, 0.0), (19.95, 0.0), (19.95, 10.0), (9.95, 10.0)]);
        assert!(polygons_collide_bidirectionally(&a, &sliver));

        // 맞닿은 것과 허용치 안의 겹침은 여전히 접촉이다.
        let touching = polygon(&[(10.0, 0.0), (20.0, 0.0), (20.0, 10.0), (10.0, 10.0)]);
        let within_tolerance = polygon(&[
            (9.99995, 0.0),
            (19.99995, 0.0),
            (19.99995, 10.0),
            (9.99995, 10.0),
        ]);
        assert!(!polygons_collide_bidirectionally(&a, &touching));
        assert!(!polygons_collide_bidirectionally(&a, &within_tolerance));

        // 한 줄로 쌓인 직사각형(위아래 접촉, 좌우 정렬)은 겹침이 아니다.
        let stacked = polygon(&[(0.0, 10.0), (10.0, 10.0), (10.0, 20.0), (0.0, 20.0)]);
        assert!(!polygons_collide_bidirectionally(&a, &stacked));
    }

    #[test]
    fn wsjk_item_59_and_6_sub_tolerance_sliver_is_treated_as_contact() {
        // Imported `shape_cd` vertices from 여성WSjk_비교용-G_Swarm.json,
        // transformed to the placements that previously passed the one-way
        // item-6 query and left a single infeasible item-item collision.
        let item_59 = polygon(&[
            (2970.9607, 546.4361),
            (2671.518, 545.3551),
            (2609.9607, 539.4361),
            (2547.9607, 531.4361),
            (2555.9607, 501.4361),
            (2560.9607, 465.4361),
            (2557.9607, 435.4361),
            (2547.9607, 399.4361),
            (2627.3608, 388.9493),
            (2693.9607, 386.4361),
            (2970.9607, 386.4361),
        ]);
        let item_6 = polygon(&[
            (3349.9482, 527.3885),
            (3325.9482, 497.3885),
            (3314.9482, 491.3885),
            (3300.9482, 490.3885),
            (3280.9482, 502.3885),
            (3242.1455, 554.0223),
            (3213.9482, 584.3885),
            (3145.9482, 582.3885),
            (2967.9482, 595.3885),
            (2971.9482, 530.3885),
            (2975.9482, 397.3885),
            (3349.9482, 397.3885),
        ]);

        // Jagua 0.7.2's polygon-vs-hazard query is asymmetric for this pair:
        // the later item 6 can miss item 59, while the reverse query sees it.
        let item_59_as_hazard = flat_cde_with_interior_hazard(item_59.clone());
        let item_6_as_hazard = flat_cde_with_interior_hazard(item_6.clone());
        assert!(!item_59_as_hazard.detect_poly_collision(&item_6, &NoFilter));
        assert!(item_6_as_hazard.detect_poly_collision(&item_59, &NoFilter));
        let index_59 =
            super::super::flat_collision_index::FlatCollisionIndex::new(&item_59_as_hazard);
        let index_6 =
            super::super::flat_collision_index::FlatCollisionIndex::new(&item_6_as_hazard);
        assert!(!index_59.detect_poly_collision(&item_6, &NoFilter));
        assert!(index_6.detect_poly_collision(&item_59, &NoFilter));

        // The only penetration is a roughly 0.000045 mm vertex sliver caused
        // by public-coordinate f32 rounding. The public millimetre contract
        // treats it as contact in both query directions; the material crossing
        // fixture above still proves that ordinary overlaps are rejected.
        assert!(!polygons_collide_bidirectionally(&item_59, &item_6));
        assert!(!polygons_collide_bidirectionally(&item_6, &item_59));
    }
}
