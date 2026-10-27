# Changes from upstream jagua-rs 1.1.0

Upstream: https://github.com/JeroenGar/jagua-rs at tag `v1.1.0`
(commit recorded in docs/MULTIPOLYGON_ITEMS_PLAN_2026-10-16.md). License MPL-2.0 (see LICENSE).
Only the `jagua-rs` library sub-crate is vendored; `Cargo.toml` uses concrete versions instead
of upstream's workspace dependencies.

(Each change below is also marked in the source with a `// CAPLANET:` comment.)

## 1. Legacy `allowed_orientations` accepted on items (`io/ext_repr.rs`)

0.8.x item JSON carried `allowed_orientations` (degrees; absent/null = continuous, `[]` or
`[0]` = fixed). Upstream 1.x rejects it. The fork converts it to `orientation` when no
`orientation` object is given, so the C ABI / C# / Java / WPF inputs and `data/input` fixtures
keep working. `ExtOrientation::from_legacy` does the conversion. An item with neither field gets
continuous rotation, as in 0.8.x.

## 2. Collision engine internals re-exposed (`collision_detection/mod.rs`, `cd_engine.rs`)

`quadtree` module, `CDEngine::quadtree`, `CDEngine::get_virtual_root` and
`CDEngine::detect_containment_collision` are `pub` again. swarm-core's specialized collision
pipeline (`eval/specialized_jaguars_pipeline.rs`) walks the quadtree directly.

## 3. `SPProblem::layout_mut` (`probs/spp/entities/problem.rs`)

swarm-core places, removes and swaps layouts directly (frozen items, warm starts, trial moves),
as it could through the public `layout` field in 0.8.x. The accessor keeps that possible; such
callers own the demand bookkeeping themselves.

## 4. `Item::with_idx`, `Item::with_shape_cd`, `Item::with_shape_orig` (`entities/item.rs`)

swarm-core post-processes imported items (lane-local re-indexing, millimetre-bounded
simplification of the collision shape, dropping a negative offset from the original-shape
record). With private fields these consuming setters replace the 0.8.x field writes.

## 5. `Container::set_base_cde` (`entities/container.rs`)

swarm-core widens the collision index root beyond the strip so that locked items outside it
stay registered (`util/frozen.rs`).

## 6. Negative `min_item_separation` accepted (`io/import.rs`)

`Importer::with_min_item_separation` accepts any finite value. A negative separation deflates
items by half of it, which is how swarm expresses an allowed piece overlap (0.8.x behaviour).
Upstream rejects negative values.

## 7. Test-only snapshot mutators (`entities/layout.rs`, `entities/placed_item.rs`, `probs/spp/entities/solution.rs`)

`LayoutSnapshot::placed_items_mut`, `SPSolution::layout_snapshot_mut`, `PlacedItem::set_d_transf`
and `PlacedItem::set_item` (all `#[doc(hidden)]`) let swarm-core's validation tests fabricate
inconsistent snapshots (drifted poses, missing or duplicated members) as they did through the
public fields in 0.8.x. They do not update cached shapes and are not used by optimizers.

Also `Item::with_allowed_orientations` (change 4) and `SPSolution::strip_mut` (change 7).

## 8. `SPProblem::change_strip_width_with` (`probs/spp/entities/problem.rs`)

Same as `change_strip_width`, but the caller may adjust the freshly built container (swarm-core
widens its collision index root, change 5) before the placed items are re-registered into it.
Re-registering a locked item that lies outside the strip into an unwidened index trips the
quadtree debug invariants.

## 9. `import_instance` keeps the importer's separation and the 0.8.x initial width (`probs/spp/io/import.rs`)

Upstream 1.x reads `min_item_separation` from the instance JSON and computes the initial strip
width as `separation + area / (height - separation)`. The fork applies the instance value only
when it is nonzero (swarm configures the separation on the importer, including negative
overlap allowances) and keeps the 0.8.x width `area / height`, which swarm's own importer uses.

Also `Item::with_external_id` (change 4): lane-local items keep `external_id == idx`.

## 10. Multipart items (`entities/item.rs`, `entities/placed_item.rs`, `entities/layout.rs`, `hazards/hazard.rs`, `hazards/filter.rs`, `io/import.rs`)

An item may consist of several disjoint polygons that move as one rigid body
(`ExtShape::MultiPolygon` on items, which upstream rejects). `Item::new_multipart` keeps every
part's original and collision contour in one shared frame (the union is centred once) and sets
`shape_orig()` / `shape_cd()` to the convex hull of the parts, so envelope-based code (bounding
boxes, sampling ranges, diameters) keeps working unchanged. `Item::area()` is the sum of the
parts. A `PlacedItem` carries every transformed part (`shapes()`); the layout registers one
hazard per part, `BasicHazardEntity::PlacedItem` gained a `part` index, `PlacedItemFilter(pk)`
ignores all parts of one placement, `Layout::haz_keys(pk)` lists their keys, and
`is_collision_free` checks every part. Ordinary items have exactly one part that shares the
hull's `Arc`s, so nothing changes for them.

Also (change 10): `SPInstance::item_area` sums `Item::area()` (the parts) instead of the
original-shape record, which is the hull for a multipart item; densities stayed correct.
