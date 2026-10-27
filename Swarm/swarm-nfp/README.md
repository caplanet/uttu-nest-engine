# Swarm Complete NFP Kernel

`swarm-nfp` is a standalone polygonal configuration-space kernel. It remains
intentionally disconnected from Swarm's placement optimizer and its
`capswarmcore_100.dll` ABI. A separate diagnostic wrapper,
`swarm-nfp-ffi`, exposes it to the WPF **Complete NFP Lab** through the
independent `capnfp_100.dll`; that laboratory connection does not change
Swarm nesting behaviour.

## What “complete NFP” means here

For two validated, anchor-relative filled polygon sets `F` (fixed) and `M`
(moving), the kernel returns the complete filled forbidden translation set:

```text
NFP(F, M) = F + (-M)
```

- concave shells are preserved;
- holes on either or both inputs are preserved;
- disconnected components on either or both inputs are preserved;
- the result may itself contain multiple components and holes;
- `complete_pair_nfp` retains the result as an exact union of convex
  integer-grid primitives and supplies a compact contour separately;
- `CompleteNfp::locate` is the authoritative NFP membership/contact query;
- Boolean union can place an additional contact-only seam inside the compact
  contour's topological interior when disconnected or multiply contacting
  geometry is involved;
- no convex-hull fallback, boundary sampling, or silent hole removal is used.

“Complete” is relative to the explicit polygon/grid contract below. Curves
must be polygonized by the caller, and invalid or degenerate input rings are
rejected with typed errors.

## Geometry pipeline

1. Quantize caller coordinates once to a `0.001 mm` integer grid.
2. Validate every shell and hole, including self-intersection and hole
   relations.
3. Union input components and reconstruct deterministic shell/hole topology.
4. Triangulate all material, including polygons with holes.
5. Verify the triangles with exact `i128` area and bidirectional Boolean
   equality.
6. For every fixed/moving material-triangle pair, form the exact convex
   Minkowski-difference atom.
7. Retain the atom union as the authoritative `CompleteNfp` representation.
8. Separately Boolean-union and canonicalize the atoms for a compact
   visualization/broad-phase contour.

This filled-material decomposition is deliberate: applying a closed-boundary
Minkowski sweep directly to a polygon with holes can retain artificial rings.

## Public operations

- `prepare`: validate, quantize, union, canonicalize, and cache triangles;
- `orient`: mirror about the local Y axis, then rotate about the local origin;
- `complete_pair_nfp`: authoritative primitive union plus compact contour;
- `pair_nfp`: compact regularized contour of `F + (-M)`;
- `classify_pair`: exact placed-material disjoint/contact/overlap predicate;
- `complete_pair_nfp_with_clearance`: authoritative clearance obstacle;
- `pair_nfp_with_clearance`: dilate the full NFP by a convex translation kernel;
- `inner_fit` / `inner_fit_with_clearance`: arbitrary polygon-container IFP;
- `feasible_region`: IFP minus translated obstacle NFP union;
- `locate`: distinguish `Outside`, `Boundary`, and `Inside`;
- `translate_region`: checked, canonical whole-region translation.

## Coordinate and topology contract

- Unit: millimetres.
- Grid: `1 / 1000 mm` (`GRID_SCALE = 1000`).
- Rounding: nearest grid coordinate, once at preparation or transform input.
- Maximum absolute derived coordinate: `1_000_000_000_000` grid units.
- Shell winding supplied by the caller is ignored; canonical shells are CCW
  and holes are CW.
- Repeated closing vertices, consecutive duplicates, and exactly collinear
  middle vertices are normalized.
- Non-finite coordinates, zero-area rings, self-intersections, holes outside
  their shell, intersecting/nested holes, and coordinate overflow are errors.

The exact primitive count is the product of fixed and moving material-triangle
counts (and also the clearance-kernel triangle count when clearance is used).
This is intentionally complete rather than sampled, but it can be expensive
for highly detailed garments. `CompleteNfp::primitives()` is an inspection
view whose polygons intentionally overlap; do not run it through a normal
polygon union if internal contact seams must remain observable. The individual
views can be serialized for diagnostics, but this crate does not currently
define a durable `CompleteNfp` round-trip persistence format.

The compact `pair_nfp` result does not retain internal contact seams as
separate topology and derived Boolean intersections are rounded to the grid.
Do not infer positive-area overlap from its `Location::Inside` alone. Use
`CompleteNfp::locate`, or call `classify_pair` for a single zero-clearance
placement.

`EuclideanConservative` uses a circumscribed, centrally symmetric even-sided
polygon. It never under-approximates the requested disc, but may conservatively
reject a band up to `max_excess_mm`; its `Boundary` is the polygonal enclosure
limit, not an analytic circle.

`PolygonSet` represents filled two-dimensional regions. If the bounding anchor
domain itself is only a line or point, IFP returns
`DegenerateConfigurationSpace` instead of guessing whether that candidate
stratum is truly feasible. Lower-dimensional strata produced later by a
concave/holey-container subtraction or obstacle subtraction are not
representable and are regularized away; ordinary contact along a represented
region contour remains `Boundary`.

## Build and test

```powershell
cargo check -p swarm-nfp --offline
cargo test -p swarm-nfp --offline
cargo test -p swarm-nfp-ffi --offline
cargo build -p swarm-nfp-ffi --release --offline
```

The implementation uses Clipper2's integer Boolean kernel through the pinned
`clipper2 = 0.6.0` Rust wrapper and `earcutr = 0.4.3` for material
triangulation. The exact-area and Boolean-equality checks guard the
triangulation boundary before it is used for NFP construction.
