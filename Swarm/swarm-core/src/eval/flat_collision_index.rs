//! Conservative BVH broad phase around the existing depth-zero exact predicates.
//! Immutable for one evaluator/layout; no quadrant routing or predicate changes.
use jagua_rs::collision_detection::{
    CDEngine,
    hazards::{BasicHazardEntity, HazardEntity, filter::HazardFilter},
    quadtree::{QTHazPresence, QTHazard},
};
use jagua_rs::geometry::{
    geo_enums::{GeoPosition, GeoRelation},
    geo_traits::{AlmostCollidesWith, CollidesWith},
    primitives::{Rect, SPolygon},
};
use std::sync::Arc;

struct Entry {
    hazard: QTHazard<BasicHazardEntity>,
    shape: Arc<SPolygon>,
}
struct Node {
    bounds: Rect,
    start: usize,
    end: usize,
    children: Option<(usize, usize)>,
}
pub(super) struct FlatCollisionIndex {
    bounds: Rect,
    exterior: Vec<Entry>,
    entries: Vec<Entry>,
    nodes: Vec<Node>,
    #[cfg(test)]
    tested: std::cell::Cell<usize>,
    #[cfg(feature = "init-profile")]
    queries: std::cell::Cell<usize>,
    #[cfg(feature = "init-profile")]
    hazard_checks: std::cell::Cell<usize>,
}
impl FlatCollisionIndex {
    pub(super) fn new(source: &CDEngine) -> Self {
        let mut entries = Vec::new();
        let mut exterior = Vec::new();
        for (key, hazard) in source.hazards_map().iter() {
            let entry = Entry {
                hazard: QTHazard::from_root(source.bbox(), hazard, key),
                shape: hazard.shape.clone(),
            };
            if hazard.entity.scope() == GeoPosition::Exterior {
                exterior.push(entry);
            } else {
                entries.push(entry);
            }
        }
        let mut nodes = Vec::new();
        if !entries.is_empty() {
            Self::build(&mut entries, 0, &mut nodes);
        }
        Self {
            bounds: source.bbox(),
            exterior,
            entries,
            nodes,
            #[cfg(test)]
            tested: std::cell::Cell::new(0),
            #[cfg(feature = "init-profile")]
            queries: std::cell::Cell::new(0),
            #[cfg(feature = "init-profile")]
            hazard_checks: std::cell::Cell::new(0),
        }
    }
    fn build(entries: &mut [Entry], start: usize, nodes: &mut Vec<Node>) -> usize {
        let bounds = entries
            .iter()
            .map(|e| e.shape.bbox())
            .reduce(|a, b| Rect {
                x_min: a.x_min.min(b.x_min),
                y_min: a.y_min.min(b.y_min),
                x_max: a.x_max.max(b.x_max),
                y_max: a.y_max.max(b.y_max),
            })
            .expect("nonempty BVH node");
        let index = nodes.len();
        nodes.push(Node {
            bounds,
            start,
            end: start + entries.len(),
            children: None,
        });
        if entries.len() > 8 {
            let split_x = bounds.width() >= bounds.height();
            let center = |e: &Entry| {
                let b = e.shape.bbox();
                if split_x {
                    f64::from(b.x_min) + f64::from(b.x_max)
                } else {
                    f64::from(b.y_min) + f64::from(b.y_max)
                }
            };
            let mid = entries.len() / 2;
            entries.select_nth_unstable_by(mid, |a, b| center(a).total_cmp(&center(b)));
            let (left, right) = entries.split_at_mut(mid);
            let l = Self::build(left, start, nodes);
            let r = Self::build(right, start + mid, nodes);
            nodes[index].children = Some((l, r));
        }
        index
    }
    pub(super) fn detect_poly_collision(
        &self,
        shape: &SPolygon,
        filter: &impl HazardFilter<BasicHazardEntity>,
    ) -> bool {
        #[cfg(test)]
        self.tested.set(0);
        #[cfg(feature = "init-profile")]
        self.queries.set(self.queries.get() + 1);
        // Preserve the original exact engine's outer-bounds gate even if filtered.
        if self.bounds.relation_to(shape.bbox()) != GeoRelation::Surrounding {
            return true;
        }
        // Exterior containment must always be checked, even for disjoint bboxes.
        if self
            .exterior
            .iter()
            .any(|e| self.check_entry(e, shape, filter))
        {
            return true;
        }
        !self.nodes.is_empty() && self.query(0, shape, filter)
    }
    fn query(
        &self,
        node: usize,
        shape: &SPolygon,
        filter: &impl HazardFilter<BasicHazardEntity>,
    ) -> bool {
        let n = &self.nodes[node];
        // Same tolerant bbox relation used by the exact containment predicate.
        // A strict/disjoint test could otherwise discard a near-contact case.
        if !n.bounds.almost_collides_with(&shape.bbox()) {
            return false;
        }
        if let Some((left, right)) = n.children {
            return self.query(left, shape, filter) || self.query(right, shape, filter);
        }
        self.entries[n.start..n.end].iter().any(|e| {
            e.shape.bbox().almost_collides_with(&shape.bbox()) && self.check_entry(e, shape, filter)
        })
    }
    fn check_entry(
        &self,
        entry: &Entry,
        shape: &SPolygon,
        filter: &impl HazardFilter<BasicHazardEntity>,
    ) -> bool {
        let hazard = &entry.hazard;
        if filter.is_irrelevant(hazard.hkey, &hazard.entity) {
            return false;
        }
        #[cfg(test)]
        self.tested.set(self.tested.get() + 1);
        #[cfg(feature = "init-profile")]
        self.hazard_checks.set(self.hazard_checks.get() + 1);
        match &hazard.presence {
            QTHazPresence::None => false,
            QTHazPresence::Entire => true,
            QTHazPresence::Partial(partial) => {
                shape.edge_iter().any(|edge| partial.collides_with(&edge))
                    || CDEngine::detect_containment_collision(shape, &entry.shape, hazard.entity)
            }
        }
    }
}
#[cfg(feature = "init-profile")]
impl Drop for FlatCollisionIndex {
    fn drop(&mut self) {
        crate::init_profile::count("lbf.index_queries", self.queries.get());
        crate::init_profile::count("lbf.index_hazard_checks", self.hazard_checks.get());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use jagua_rs::{
        collision_detection::hazards::{Hazard, filter::NoFilter},
        geometry::{DTransformation, geo_traits::Transformable, primitives::Point},
    };
    use std::sync::Arc;
    fn rect(x: f32, y: f32, w: f32, h: f32) -> SPolygon {
        SPolygon::new(vec![
            Point(x, y),
            Point(x + w, y),
            Point(x + w, y + h),
            Point(x, y + h),
        ])
        .unwrap()
    }
    fn scene() -> CDEngine {
        let outer = Arc::new(rect(0., 0., 10000., 1000.));
        let mut hazards = vec![Hazard::new(BasicHazardEntity::Exterior, outer.clone())];
        for id in 0..100 {
            hazards.push(Hazard::new(
                BasicHazardEntity::Hole { idx: id },
                Arc::new(rect(100. + id as f32 * 80., 100., 30., 50.)),
            ));
        }
        hazards.push(Hazard::new(
            BasicHazardEntity::InferiorQualityZone { quality: 0, idx: 0 },
            Arc::new(rect(50., 300., 40., 40.)),
        ));
        let mut cfg = crate::config::DEFAULT_SWARM_CONFIG.cde_config;
        cfg.quadtree_depth = 0;
        CDEngine::new(outer.bbox(), hazards, cfg)
    }
    #[test]
    fn filters_preserve_source_keys_and_entity_selection() {
        use jagua_rs::collision_detection::hazards::filter::QualityZoneFilter;
        let source = scene();
        let index = FlatCollisionIndex::new(&source);
        for (key, hazard) in source.hazards_map().iter() {
            let query = hazard.shape.as_ref();
            assert_eq!(
                source.detect_poly_collision(query, &key),
                index.detect_poly_collision(query, &key)
            );
            let quality = QualityZoneFilter { min_quality: 0 };
            assert_eq!(
                source.detect_poly_collision(query, &quality),
                index.detect_poly_collision(query, &quality)
            );
        }
    }

    #[test]
    fn concave_exterior_preserves_inside_notch_and_boundary_checks() {
        let outer = Arc::new(
            SPolygon::new(vec![
                Point(0., 0.),
                Point(100., 0.),
                Point(100., 30.),
                Point(30., 30.),
                Point(30., 100.),
                Point(0., 100.),
            ])
            .unwrap(),
        );
        let mut cfg = crate::config::DEFAULT_SWARM_CONFIG.cde_config;
        cfg.quadtree_depth = 0;
        let source = CDEngine::new(
            outer.bbox(),
            vec![Hazard::new(BasicHazardEntity::Exterior, outer)],
            cfg,
        );
        let index = FlatCollisionIndex::new(&source);
        for x in [0., 10., 20., 29.99999, 30., 50., 90.] {
            for y in [0., 10., 20., 29.99999, 30., 50., 90.] {
                let query = rect(x, y, 10., 10.);
                assert_eq!(
                    source.detect_poly_collision(&query, &NoFilter),
                    index.detect_poly_collision(&query, &NoFilter),
                    "x={x} y={y}"
                );
            }
        }
    }

    #[test]
    fn far_hazards_do_not_reach_exact_checks() {
        let source = scene();
        let index = FlatCollisionIndex::new(&source);
        let query = rect(40., 60., 8., 9.);
        assert!(!index.detect_poly_collision(&query, &NoFilter));
        assert!(
            index.tested.get() <= 2,
            "checked {} hazards for an isolated candidate",
            index.tested.get()
        );
    }
    #[test]
    fn index_matches_flat_at_contacts_containment_and_rotations() {
        let source = scene();
        let index = FlatCollisionIndex::new(&source);
        for x in [
            -0.01, 0., 0.00001, 49.99999, 50., 99.99999, 100., 100.00001, 129.99998, 130., 5000.,
            9990., 9991.,
        ] {
            for y in [
                -0.01, 0., 60., 99.99999, 100., 149.99998, 150., 300., 990., 991.,
            ] {
                for rotation in [0., 0.00001, 0.3, 1.5707964, 3.1415927] {
                    let mut query = rect(0., 0., 10., 10.);
                    query.transform(&DTransformation::new(rotation, (x, y)).compose());
                    assert_eq!(
                        source.detect_poly_collision(&query, &NoFilter),
                        index.detect_poly_collision(&query, &NoFilter),
                        "x={x} y={y} r={rotation}"
                    );
                }
            }
        }
        // A query surrounding multiple hazards must not be discarded by the index.
        let query = rect(90., 90., 200., 80.);
        assert_eq!(
            source.detect_poly_collision(&query, &NoFilter),
            index.detect_poly_collision(&query, &NoFilter)
        );
    }
}
