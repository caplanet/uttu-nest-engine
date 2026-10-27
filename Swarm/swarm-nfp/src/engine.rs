use std::{cell::RefCell, cmp::Ordering, f64::consts::PI};

use crate::backend::{
    GridPath, GridPaths, bounds_scaled, canonical_paths_cmp, canonical_paths_from_polygon_set,
    classify_paths, difference_paths, locate_paths, minkowski_atom_primitives, minkowski_sum_atoms,
    nfp_atom_primitives, nfp_from_atoms, paths_to_polygon_set, quantize_mm, rect_paths,
    reflect_convex_primitives, reflect_paths, transform_polygon_set, translate_paths, union_paths,
};
use crate::prepare::{prepare_from_paths, prepare_polygon_set};
use crate::{
    CompleteNfp, GRID_SCALE, GRID_STEP_MM, Location, MAX_DERIVED_GRID_COORD, NfpError, NfpResult,
    Orientation, PairRelation, Point, PolygonSet, PreparedShape, Ring, Translation,
};

/// Optional gap applied to the complete pair NFP after `fixed + (-moving)`.
#[derive(Debug, Default, Clone, PartialEq)]
pub enum Clearance {
    #[default]
    None,
    /// A caller-supplied convex translation-space kernel that strictly
    /// contains the origin. It need not be symmetric.
    ConvexKernel(crate::Ring),
    /// A conservative polygonal enclosure of a Euclidean disc.
    EuclideanConservative {
        distance_mm: f64,
        max_excess_mm: f64,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct PlacedObstacle<'a> {
    pub shape: &'a PreparedShape,
    pub translation: Translation,
}

#[derive(Debug)]
pub struct FeasibleRequest<'a> {
    pub container: &'a PreparedShape,
    pub moving: &'a PreparedShape,
    pub obstacles: &'a [PlacedObstacle<'a>],
    /// Translation-space kernel applied both to the container complement and
    /// to every placed-obstacle pair NFP.
    pub clearance: Clearance,
}

/// A region canonicalized once for repeated [`NfpEngine::locate_prepared`] queries.
///
/// Holds the same canonical paths [`NfpEngine::locate`] would rebuild per call.
/// `None` means the source region was empty — every point locates as
/// [`Location::Outside`].
#[derive(Debug, Clone)]
pub struct PreparedRegion {
    paths: Option<GridPaths>,
}

impl PreparedRegion {
    /// Whether the source region was empty.
    pub fn is_empty(&self) -> bool {
        self.paths.is_none()
    }
}

/// Stateless, deterministic NFP engine.
#[derive(Debug, Default, Clone)]
pub struct NfpEngine;

impl NfpEngine {
    pub fn new() -> Self {
        Self
    }

    /// Validate, quantize and canonicalize a filled polygon set.
    ///
    /// Input coordinates are anchor-relative millimetres. The local origin is
    /// never changed by preparation, orientation or NFP construction.
    pub fn prepare(&self, input: &PolygonSet) -> NfpResult<PreparedShape> {
        prepare_polygon_set(input, "shape")
    }

    /// Return an immutable oriented copy of a prepared shape.
    ///
    /// `mirror_x` negates local X first. Rotation follows about the anchor at
    /// the origin. Cardinal rotations use integer arithmetic in the backend;
    /// all other microdegree angles are evaluated from the immutable grid
    /// geometry and quantized exactly once.
    pub fn orient(
        &self,
        shape: &PreparedShape,
        orientation: Orientation,
    ) -> NfpResult<PreparedShape> {
        let transformed = transform_polygon_set(&shape.geometry, orientation)?;
        prepare_polygon_set(&transformed, "oriented shape")
    }

    /// Construct the compact regularized contour of `F + (-M)`.
    ///
    /// This is useful for visualization, broad-phase tests and Boolean
    /// subtraction. Clipper rounds derived intersections to the grid, and a
    /// contact-only seam can lie in the topological interior after union. Use
    /// [`Self::complete_pair_nfp`] for an authoritative NFP membership oracle,
    /// or [`Self::classify_pair`] to classify one zero-clearance placement.
    pub fn pair_nfp(&self, fixed: &PreparedShape, moving: &PreparedShape) -> NfpResult<PolygonSet> {
        let paths = self.pair_nfp_paths_with_kernel(fixed, moving, None)?;
        paths_to_polygon_set(&paths)
    }

    /// Construct the complete NFP as exact convex grid primitives plus a
    /// compact Boolean contour.
    ///
    /// The primitive union is authoritative: it retains contact-only seams and
    /// avoids rounding derived Boolean intersections. The compact contour is a
    /// convenient regularized shell/hole view.
    pub fn complete_pair_nfp(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
    ) -> NfpResult<CompleteNfp> {
        self.complete_pair_nfp_with_kernel(fixed, moving, None)
    }

    /// Construct the complete primitive-union NFP after Minkowski dilation by
    /// `clearance`. Its [`CompleteNfp::locate`] result classifies clearance
    /// violation/boundary/outside rather than raw material overlap. A
    /// [`Clearance::EuclideanConservative`] result describes its conservative
    /// polygonal enclosure, including the configured maximum excess.
    pub fn complete_pair_nfp_with_clearance(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
        clearance: &Clearance,
    ) -> NfpResult<CompleteNfp> {
        let kernel = self.prepare_clearance(clearance)?;
        self.complete_pair_nfp_with_kernel(fixed, moving, kernel.as_ref())
    }

    /// Exact number of convex primitives that a complete pair NFP would
    /// construct after applying the requested clearance kernel.
    ///
    /// This validates/prepares the clearance but does not build the pair NFP,
    /// allowing in-process diagnostic clients to reject unsafe workloads
    /// without silently sampling or truncating the exact result.
    pub fn complete_pair_primitive_count(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
        clearance: &Clearance,
    ) -> NfpResult<usize> {
        let base = fixed
            .triangle_count()
            .checked_mul(moving.triangle_count())
            .ok_or(NfpError::PrimitiveCountOverflow {
                operation: "pair NFP",
            })?;
        let clearance_triangles = self
            .prepare_clearance(clearance)?
            .map_or(1, |kernel| kernel.triangle_count());
        base.checked_mul(clearance_triangles)
            .ok_or(NfpError::PrimitiveCountOverflow {
                operation: "pair NFP with clearance",
            })
    }

    /// Classify the actual placed material at one quantized translation.
    ///
    /// Unlike locating the point in a Boolean-unioned NFP polygon, this method
    /// cannot mislabel an internal contact-only seam as overlap.
    pub fn classify_pair(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
        moving_translation: Translation,
    ) -> NfpResult<PairRelation> {
        let dx = quantize_mm(moving_translation.x, "pair classification translation x")?;
        let dy = quantize_mm(moving_translation.y, "pair classification translation y")?;
        let positioned_moving = translate_paths(&moving.canonical_paths, dx, dy)?;
        Ok(classify_paths(
            &fixed.canonical_paths,
            &positioned_moving,
            &fixed.triangles,
            &moving.triangles,
            dx,
            dy,
        ))
    }

    /// Construct `F + (-M)` and dilate the complete result by `clearance`.
    pub fn pair_nfp_with_clearance(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
        clearance: &Clearance,
    ) -> NfpResult<PolygonSet> {
        let kernel = self.prepare_clearance(clearance)?;
        let paths = self.pair_nfp_paths_with_kernel(fixed, moving, kernel.as_ref())?;
        paths_to_polygon_set(&paths)
    }

    /// Compute the inner-fit region for an arbitrary filled polygon container.
    ///
    /// Let `B` be the container bounding rectangle and `D` the anchor domain
    /// obtained by eroding `B` by the moving shape's bounding box. Within `D`,
    /// the regularized grid forbidden region is
    /// `(B - container) + (-moving)`, hence:
    ///
    /// `IFP = D - ((B - container) + (-moving))`.
    ///
    /// An ordinary empty 2-D IFP is a valid result. If the bbox anchor domain
    /// itself collapses to a line or point, the method returns
    /// [`NfpError::DegenerateConfigurationSpace`] because [`PolygonSet`] cannot
    /// represent that potentially valid exact-fit stratum. Line/point strata
    /// created later by subtracting a concave or holey container complement
    /// are likewise regularized away by the 2-D [`PolygonSet`] result model.
    pub fn inner_fit(
        &self,
        container: &PreparedShape,
        moving: &PreparedShape,
    ) -> NfpResult<PolygonSet> {
        let paths = self.inner_fit_paths(container, moving, None)?;
        paths_to_polygon_set(&paths)
    }

    /// Compute an inner-fit region with the same clearance-kernel convention
    /// used by pair NFPs. The moving footprint is effectively expanded before
    /// containment is evaluated.
    pub fn inner_fit_with_clearance(
        &self,
        container: &PreparedShape,
        moving: &PreparedShape,
        clearance: &Clearance,
    ) -> NfpResult<PolygonSet> {
        let kernel = self.prepare_clearance(clearance)?;
        let paths = self.inner_fit_paths(container, moving, kernel.as_ref())?;
        paths_to_polygon_set(&paths)
    }

    /// Compute `IFP - union(translate(NFP(obstacle, moving)))`.
    ///
    /// Obstacle orientation is already baked into each `PreparedShape`; only
    /// its anchor translation is applied here. Empty IFP and feasible regions
    /// are normal successful results. The request's clearance kernel applies
    /// to both the container boundary and every obstacle. The returned set is
    /// the regularized filled 2-D Boolean result; contact along its represented
    /// contours is preserved as [`Location::Boundary`], while isolated line or
    /// point residues created only by obstacle subtraction are not representable
    /// by [`PolygonSet`] and are regularized away.
    pub fn feasible_region(&self, request: FeasibleRequest<'_>) -> NfpResult<PolygonSet> {
        // Validate and prepare clearance even when the IFP or obstacle list is
        // empty, so malformed public input never becomes condition-dependent.
        let kernel = self.prepare_clearance(&request.clearance)?;
        let ifp = self.inner_fit_paths(request.container, request.moving, kernel.as_ref())?;
        if ifp.is_empty() || request.obstacles.is_empty() {
            return paths_to_polygon_set(&ifp);
        }

        let mut forbidden_paths: Vec<GridPath> = Vec::new();
        for obstacle in request.obstacles {
            let pair =
                self.pair_nfp_paths_with_kernel(obstacle.shape, request.moving, kernel.as_ref())?;
            let dx = quantize_mm(obstacle.translation.x, "obstacle translation x")?;
            let dy = quantize_mm(obstacle.translation.y, "obstacle translation y")?;
            let translated = translate_paths(&pair, dx, dy)?;
            forbidden_paths.extend(translated);
        }

        if forbidden_paths.is_empty() {
            return paths_to_polygon_set(&ifp);
        }

        let forbidden = union_paths(GridPaths::new(forbidden_paths))?;
        let feasible = difference_paths(&ifp, &forbidden)?;
        paths_to_polygon_set(&feasible)
    }

    /// Subtract one placed obstacle's NFP from an existing feasible region.
    ///
    /// Computes `region - translate(NFP(obstacle, moving))` and returns the
    /// regularized result, exactly as [`Self::feasible_region`] would for that
    /// single obstacle.
    ///
    /// # Why this exists
    ///
    /// [`Self::feasible_region`] unions **every** obstacle's pair NFP on each
    /// call. A placement loop that calls it once per piece therefore does
    /// `O(n^2)` pair NFPs and, worse, unions a set that grows with `n`. On a
    /// 156-piece garment marker that union starts returning empty or failing
    /// past roughly 26 placed pieces.
    ///
    /// Set difference distributes over the union:
    ///
    /// ```text
    ///     A - (B u C) = (A - B) - C
    /// ```
    ///
    /// so a caller may keep the region and subtract each newly placed obstacle
    /// as it arrives. The result is identical; only the growing union is gone.
    ///
    /// An empty `region` short-circuits: nothing can be removed from nothing.
    pub fn subtract_obstacle(
        &self,
        region: &PolygonSet,
        obstacle: &PlacedObstacle<'_>,
        moving: &PreparedShape,
        clearance: &Clearance,
    ) -> NfpResult<PolygonSet> {
        // Validate clearance even when the region is empty so malformed public
        // input never becomes condition-dependent — same rule as `feasible_region`.
        let kernel = self.prepare_clearance(clearance)?;
        let dx = quantize_mm(obstacle.translation.x, "obstacle translation x")?;
        let dy = quantize_mm(obstacle.translation.y, "obstacle translation y")?;
        if region.is_empty() {
            return Ok(region.clone());
        }
        let pair = self.pair_nfp_paths_with_kernel(obstacle.shape, moving, kernel.as_ref())?;
        let translated = translate_paths(&pair, dx, dy)?;
        if translated.is_empty() {
            return Ok(region.clone());
        }
        let subject = canonical_paths_from_polygon_set(region, "subtract subject")?;
        let result = difference_paths(&subject, &translated)?;
        paths_to_polygon_set(&result)
    }

    /// Locate a quantized world point in the topology of a filled region.
    ///
    /// This canonicalizes arbitrary overlapping/nested public components. For
    /// contact-versus-overlap classification of a shape pair, use
    /// [`Self::classify_pair`] instead.
    pub fn locate(&self, region: &PolygonSet, point: Point) -> NfpResult<Location> {
        let x = quantize_mm(point.x, "point location x")?;
        let y = quantize_mm(point.y, "point location y")?;
        if region.is_empty() {
            return Ok(Location::Outside);
        }
        let canonical = canonical_paths_from_polygon_set(region, "location region")?;
        locate_paths(&canonical, x, y)
    }

    /// Canonicalize a region **once** so repeated location queries skip it.
    ///
    /// [`Self::locate`] re-canonicalizes its `region` on every call. That is fine
    /// for a one-off query, but a placement search locates tens of thousands of
    /// points against the same handful of regions, and the normalization then
    /// dominates: a 40-vertex point-in-polygon test measured ~36 us, almost all
    /// of it rebuilding the canonical paths.
    ///
    /// Prepare the region once with this method and query it with
    /// [`Self::locate_prepared`]. The result is identical to [`Self::locate`] —
    /// only the redundant work is gone.
    pub fn prepare_region(&self, region: &PolygonSet) -> NfpResult<PreparedRegion> {
        if region.is_empty() {
            return Ok(PreparedRegion { paths: None });
        }
        let canonical = canonical_paths_from_polygon_set(region, "location region")?;
        Ok(PreparedRegion {
            paths: Some(canonical),
        })
    }

    /// Locate a point in a region canonicalized by [`Self::prepare_region`].
    ///
    /// Equivalent to [`Self::locate`] on the region it was prepared from.
    pub fn locate_prepared(&self, region: &PreparedRegion, point: Point) -> NfpResult<Location> {
        let x = quantize_mm(point.x, "point location x")?;
        let y = quantize_mm(point.y, "point location y")?;
        match &region.paths {
            None => Ok(Location::Outside),
            Some(paths) => locate_paths(paths, x, y),
        }
    }

    /// Quantize a translation exactly as pair classification and NFP location
    /// do, returning the evaluated millimetre coordinates for diagnostics.
    pub fn quantize_translation(&self, translation: Translation) -> NfpResult<Translation> {
        let x = quantize_mm(translation.x, "translation x")?;
        let y = quantize_mm(translation.y, "translation y")?;
        Ok(Translation::new(
            x as f64 / GRID_SCALE as f64,
            y as f64 / GRID_SCALE as f64,
        ))
    }

    /// Translate a complete region without dropping components or holes.
    pub fn translate_region(
        &self,
        region: &PolygonSet,
        translation: Translation,
    ) -> NfpResult<PolygonSet> {
        let dx = quantize_mm(translation.x, "region translation x")?;
        let dy = quantize_mm(translation.y, "region translation y")?;
        if region.is_empty() {
            return Ok(region.clone());
        }
        let canonical = canonical_paths_from_polygon_set(region, "translated region")?;
        let translated = translate_paths(&canonical, dx, dy)?;
        paths_to_polygon_set(&translated)
    }

    fn pair_nfp_paths_with_kernel(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
        clearance_kernel: Option<&PreparedShape>,
    ) -> NfpResult<GridPaths> {
        let (canonical, reflect) =
            self.pair_nfp_canonical_primitives(fixed, moving, clearance_kernel)?;
        Self::compact_from_canonical(canonical, reflect)
    }

    fn complete_pair_nfp_with_kernel(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
        clearance_kernel: Option<&PreparedShape>,
    ) -> NfpResult<CompleteNfp> {
        let (canonical, reflect) =
            self.pair_nfp_canonical_primitives(fixed, moving, clearance_kernel)?;
        // The exported decomposition is in the caller's requested direction;
        // the compact contour is unioned in the canonical direction and
        // reflected once afterwards, from the same canonical primitives.
        let primitives = if reflect {
            reflect_convex_primitives(&canonical)?
        } else {
            canonical.clone()
        };
        let compact_paths = Self::compact_from_canonical(canonical, reflect)?;
        if compact_paths.is_empty() {
            return Err(NfpError::EmptyNfp);
        }
        Ok(CompleteNfp::new(
            paths_to_polygon_set(&compact_paths)?,
            primitives,
        ))
    }

    /// Union canonical-direction primitives into the compact contour and
    /// restore the requested direction with one final point reflection.
    ///
    /// Clipper must round derived intersections to the grid; reflecting each
    /// primitive before the union could choose the opposite grid point at a
    /// tie and break `NFP(F,M) = -NFP(M,F)`. Reflecting the unioned contour
    /// once is exact, so this is also where the former
    /// reflect-primitives / union / reflect-contour double reflection is
    /// avoided: the primitives arrive already in the canonical direction.
    fn compact_from_canonical(canonical: GridPaths, reflect: bool) -> NfpResult<GridPaths> {
        let canonical_compact = union_paths(canonical)?;
        let compact = if reflect {
            reflect_paths(&canonical_compact)?
        } else {
            canonical_compact
        };
        if compact.is_empty() {
            Err(NfpError::EmptyNfp)
        } else {
            Ok(compact)
        }
    }

    /// Convex NFP primitives in the canonical operand direction.
    ///
    /// Returns `(primitives, reflect)`. When `reflect` is `true`, the
    /// primitives of the *requested* direction are the point reflection of
    /// the returned ones (`reflect_convex_primitives`), and the compact
    /// contour must be unioned as returned and reflected afterwards
    /// ([`Self::compact_from_canonical`]). When `false`, the returned
    /// primitives already are the requested ones.
    fn pair_nfp_canonical_primitives(
        &self,
        fixed: &PreparedShape,
        moving: &PreparedShape,
        clearance_kernel: Option<&PreparedShape>,
    ) -> NfpResult<(GridPaths, bool)> {
        // Every primitive is the filled convex hull of one material-triangle
        // pair's vertex differences. Their implicit union is exact on the
        // input grid and cannot acquire artificial hole rings from a closed
        // boundary sweep.
        // Always generate the base atoms in one canonical operand order. The
        // compact contour can then be unioned in that same direction before
        // any final point reflection, preserving the exact identity
        // NFP(F,M) = -NFP(M,F). Clearance is applied only after the base has
        // been restored to the caller's requested direction.
        let (canonical_base, reversed) =
            match canonical_paths_cmp(&fixed.canonical_paths, &moving.canonical_paths) {
                Ordering::Greater => (
                    nfp_atom_primitives(&moving.triangles, &fixed.triangles)?,
                    true,
                ),
                Ordering::Less | Ordering::Equal => (
                    nfp_atom_primitives(&fixed.triangles, &moving.triangles)?,
                    false,
                ),
            };
        if canonical_base.is_empty() {
            return Err(NfpError::EmptyNfp);
        }

        let Some(kernel) = clearance_kernel else {
            return Ok((canonical_base, reversed));
        };

        // For a centrally symmetric kernel and reversed canonical operands,
        // expand in the canonical direction and let the caller reflect the
        // completed primitives. This makes the exported decomposition itself
        // satisfy NFP_K(F,M) = -NFP_K(M,F), independent of the kernel
        // triangulation's diagonal choices. An asymmetric caller kernel
        // intentionally breaks that identity and must instead be applied
        // after restoring the requested direction, so its primitives are
        // unioned in the requested direction (no final reflection).
        let expand_then_reflect = reversed && self.is_centrally_symmetric(kernel)?;
        let base = if reversed && !expand_then_reflect {
            reflect_convex_primitives(&canonical_base)?
        } else {
            canonical_base
        };
        let base_atoms = base.iter().cloned().collect::<Vec<_>>();
        let expanded = minkowski_atom_primitives(&base_atoms, &kernel.triangles)?;
        if expanded.is_empty() {
            Err(NfpError::EmptyNfp)
        } else {
            Ok((expanded, expand_then_reflect))
        }
    }

    fn is_centrally_symmetric(&self, shape: &PreparedShape) -> NfpResult<bool> {
        let reflected = reflect_paths(&shape.canonical_paths)?;
        Ok(canonical_paths_cmp(&shape.canonical_paths, &reflected) == Ordering::Equal)
    }

    fn inner_fit_paths(
        &self,
        container: &PreparedShape,
        moving: &PreparedShape,
        clearance_kernel: Option<&PreparedShape>,
    ) -> NfpResult<GridPaths> {
        let (c_min_x, c_min_y, c_max_x, c_max_y) = bounds_scaled(&container.canonical_paths)?;
        let (m_min_x, m_min_y, m_max_x, m_max_y) = bounds_scaled(&moving.canonical_paths)?;

        // Pair-clearance convention is `outside + (-moving) + K`, equivalent
        // to testing the effective moving footprint `moving + (-K)`. Its bbox
        // is used for the finite anchor domain so clearance also applies at the
        // outer rectangle, not only at concave container boundaries.
        let (k_min_x, k_min_y, k_max_x, k_max_y) = match clearance_kernel {
            Some(kernel) => bounds_scaled(&kernel.canonical_paths)?,
            None => (0, 0, 0, 0),
        };
        let effective_min_x = m_min_x as i128 - k_max_x as i128;
        let effective_min_y = m_min_y as i128 - k_max_y as i128;
        let effective_max_x = m_max_x as i128 - k_min_x as i128;
        let effective_max_y = m_max_y as i128 - k_min_y as i128;
        let d_min_x = checked_grid(c_min_x as i128 - effective_min_x, "IFP anchor-domain min x")?;
        let d_min_y = checked_grid(c_min_y as i128 - effective_min_y, "IFP anchor-domain min y")?;
        let d_max_x = checked_grid(c_max_x as i128 - effective_max_x, "IFP anchor-domain max x")?;
        let d_max_y = checked_grid(c_max_y as i128 - effective_max_y, "IFP anchor-domain max y")?;

        if d_min_x > d_max_x || d_min_y > d_max_y {
            return Ok(GridPaths::default());
        }
        // A line or point anchor domain may contain valid exact-fit contact,
        // but PolygonSet represents filled 2D regions only. Do not silently
        // report that representational limitation as an ordinary empty IFP.
        if d_min_x == d_max_x || d_min_y == d_max_y {
            return Err(NfpError::DegenerateConfigurationSpace {
                operation: "inner fit",
            });
        }

        let domain = rect_paths(d_min_x, d_min_y, d_max_x, d_max_y)?;
        let container_bbox = rect_paths(c_min_x, c_min_y, c_max_x, c_max_y)?;
        let outside = difference_paths(&container_bbox, &container.canonical_paths)?;
        if outside.is_empty() {
            return Ok(domain);
        }

        let outside_prepared = prepare_from_paths(outside, "container complement")?;
        let forbidden = nfp_from_atoms(&outside_prepared.triangles, &moving.triangles)?;
        if forbidden.is_empty() {
            return Ok(domain);
        }
        let forbidden = match clearance_kernel {
            Some(kernel) => self.dilate_paths(forbidden, kernel, "container-complement NFP")?,
            None => forbidden,
        };
        difference_paths(&domain, &forbidden)
    }

    fn dilate_paths(
        &self,
        paths: GridPaths,
        kernel: &PreparedShape,
        context: &'static str,
    ) -> NfpResult<GridPaths> {
        // True Minkowski sum, not NFP reflection. Triangulation makes the sum
        // distribute exactly over every filled component.
        let prepared = prepare_from_paths(paths, context)?;
        minkowski_sum_atoms(&prepared.triangles, &kernel.triangles)
    }

    /// Prepare the clearance kernel, memoizing the most recent successful
    /// result per thread.
    ///
    /// A placement loop passes the same `Clearance` to every
    /// `subtract_obstacle` / `feasible_region` / `complete_pair_nfp_*` call,
    /// and a Euclidean kernel search prepares and validates up to thousands
    /// of candidate polygons each time. The engine is stateless, so the memo
    /// is a thread-local one-entry cache keyed by value; a different clearance
    /// simply replaces it. Failures are never cached, so invalid input is
    /// re-validated and reported on every call.
    fn prepare_clearance(&self, clearance: &Clearance) -> NfpResult<Option<PreparedShape>> {
        thread_local! {
            static CLEARANCE_MEMO: RefCell<Option<(Clearance, Option<PreparedShape>)>> =
                const { RefCell::new(None) };
        }
        if matches!(clearance, Clearance::None) {
            return Ok(None);
        }
        let cached = CLEARANCE_MEMO.with(|memo| {
            memo.borrow()
                .as_ref()
                .filter(|(key, _)| key == clearance)
                .map(|(_, kernel)| kernel.clone())
        });
        if let Some(kernel) = cached {
            return Ok(kernel);
        }
        let kernel = self.prepare_clearance_uncached(clearance)?;
        CLEARANCE_MEMO.with(|memo| {
            *memo.borrow_mut() = Some((clearance.clone(), kernel.clone()));
        });
        Ok(kernel)
    }

    fn prepare_clearance_uncached(
        &self,
        clearance: &Clearance,
    ) -> NfpResult<Option<PreparedShape>> {
        match clearance {
            Clearance::None => Ok(None),
            Clearance::ConvexKernel(ring) => {
                let kernel =
                    prepare_polygon_set(&PolygonSet::from_outer(ring.clone()), "clearance kernel")?;
                validate_convex_origin_kernel(&kernel)?;
                Ok(Some(kernel))
            }
            Clearance::EuclideanConservative {
                distance_mm,
                max_excess_mm,
            } => self.prepare_euclidean_kernel(*distance_mm, *max_excess_mm),
        }
    }

    fn prepare_euclidean_kernel(
        &self,
        distance_mm: f64,
        max_excess_mm: f64,
    ) -> NfpResult<Option<PreparedShape>> {
        if !distance_mm.is_finite() || distance_mm < 0.0 {
            return Err(NfpError::InvalidClearance);
        }
        if !max_excess_mm.is_finite() || max_excess_mm <= 0.0 {
            return Err(NfpError::InvalidClearanceTolerance);
        }
        if distance_mm == 0.0 {
            return Ok(None);
        }

        // For a regular n-gon circumscribed around radius r, vertex excess is
        // r * (sec(pi/n) - 1). Start at the theoretical minimum side count,
        // then validate the actual quantized grid polygon.
        const MAX_SIDES: usize = 4_096;
        const PAD_STEPS: usize = 8;
        let ratio = (distance_mm / (distance_mm + max_excess_mm)).clamp(0.0, 1.0);
        let angle = ratio.acos();
        let first_n = if angle > 0.0 {
            ((PI / angle).ceil() as usize).max(4)
        } else {
            MAX_SIDES.saturating_add(1)
        };
        let first_even_n = first_n.saturating_add(first_n % 2);
        if first_even_n > MAX_SIDES {
            return Err(NfpError::InvalidClearanceTolerance);
        }

        // A Euclidean distance disc is centrally symmetric. Restrict the
        // conservative regular polygon to an even side count and construct
        // its second half by exact negation, so grid quantization preserves
        // K = -K and therefore swap-negation symmetry of pair NFPs.
        for n in (first_even_n..=MAX_SIDES).step_by(2) {
            let half_step = PI / n as f64;
            let cos_half = half_step.cos();
            if !cos_half.is_finite() || cos_half <= 0.0 {
                continue;
            }
            for pad_step in 0..=PAD_STEPS {
                let pad = pad_step as f64 * (GRID_STEP_MM / PAD_STEPS as f64);
                let vertex_radius = (distance_mm + pad) / cos_half;
                if vertex_radius - distance_mm > max_excess_mm + f64::EPSILON {
                    break;
                }

                let half_vertices = (0..n / 2)
                    .map(|i| {
                        let theta = (2.0 * i as f64 + 1.0) * half_step;
                        Point::new(vertex_radius * theta.cos(), vertex_radius * theta.sin())
                    })
                    .collect::<Vec<_>>();
                let vertices = half_vertices
                    .iter()
                    .copied()
                    .chain(half_vertices.iter().map(|p| Point::new(-p.x, -p.y)));
                let input = PolygonSet::from_outer(Ring::new(vertices));
                let candidate = match prepare_polygon_set(&input, "Euclidean clearance kernel") {
                    Ok(candidate) => candidate,
                    Err(NfpError::TooFewVertices { .. })
                    | Err(NfpError::ZeroAreaRing { .. })
                    | Err(NfpError::SelfIntersection { .. }) => continue,
                    Err(error) => return Err(error),
                };
                if validate_convex_origin_kernel(&candidate).is_err() {
                    continue;
                }
                let Some((min_support, max_radius)) = radial_bounds_mm(&candidate) else {
                    continue;
                };
                let epsilon = 8.0 * f64::EPSILON * distance_mm.max(1.0);
                if min_support + epsilon >= distance_mm
                    && max_radius <= distance_mm + max_excess_mm + epsilon
                {
                    return Ok(Some(candidate));
                }
            }
        }

        Err(NfpError::InvalidClearanceTolerance)
    }
}

fn checked_grid(value: i128, stage: &'static str) -> NfpResult<i64> {
    if value.unsigned_abs() > MAX_DERIVED_GRID_COORD as u128 {
        return Err(NfpError::CoordinateOverflow {
            stage,
            value,
            limit: MAX_DERIVED_GRID_COORD,
        });
    }
    i64::try_from(value).map_err(|_| NfpError::CoordinateOverflow {
        stage,
        value,
        limit: MAX_DERIVED_GRID_COORD,
    })
}

fn validate_convex_origin_kernel(kernel: &PreparedShape) -> NfpResult<()> {
    if kernel.geometry.components.len() != 1 || !kernel.geometry.components[0].holes.is_empty() {
        return Err(NfpError::InvalidClearance);
    }
    let Some(path) = kernel.canonical_paths.first() else {
        return Err(NfpError::InvalidClearance);
    };
    if !is_convex_path(path) || locate_paths(&kernel.canonical_paths, 0, 0)? != Location::Inside {
        return Err(NfpError::InvalidClearance);
    }
    Ok(())
}

fn is_convex_path(path: &GridPath) -> bool {
    let points: Vec<(i64, i64)> = path
        .iter()
        .map(|point| (point.x_scaled(), point.y_scaled()))
        .collect();
    if points.len() < 3 {
        return false;
    }
    let mut sign = 0i8;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        let c = points[(i + 2) % points.len()];
        let ab = (b.0 as i128 - a.0 as i128, b.1 as i128 - a.1 as i128);
        let bc = (c.0 as i128 - b.0 as i128, c.1 as i128 - b.1 as i128);
        let cross = ab.0 * bc.1 - ab.1 * bc.0;
        if cross == 0 {
            continue;
        }
        let current = if cross > 0 { 1 } else { -1 };
        if sign != 0 && sign != current {
            return false;
        }
        sign = current;
    }
    sign != 0
}

/// Return `(minimum edge support, maximum vertex radius)` in millimetres.
fn radial_bounds_mm(kernel: &PreparedShape) -> Option<(f64, f64)> {
    let path = kernel.canonical_paths.first()?;
    let points: Vec<(f64, f64)> = path
        .iter()
        .map(|point| (point.x_scaled() as f64, point.y_scaled() as f64))
        .collect();
    if points.len() < 3 {
        return None;
    }

    let mut min_support = f64::INFINITY;
    let mut max_radius = 0.0f64;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        let dx = b.0 - a.0;
        let dy = b.1 - a.1;
        let edge_len = dx.hypot(dy);
        if edge_len == 0.0 {
            return None;
        }
        let support = (a.0 * b.1 - a.1 * b.0).abs() / edge_len;
        min_support = min_support.min(support);
        max_radius = max_radius.max(a.0.hypot(a.1));
    }

    Some((
        min_support / GRID_SCALE as f64,
        max_radius / GRID_SCALE as f64,
    ))
}
