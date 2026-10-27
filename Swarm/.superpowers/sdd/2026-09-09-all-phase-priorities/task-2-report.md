# Priority 2: immutable sampler and shape-factor preparation

## Changes

- Added `prepared_geometry.rs`: run-owned `PreparedGeometry` stores each item's owning shape Arc, exact rotation signature, ordered rotation-origin bboxes, and hull-area square root. Continuous proposals use the same 16 f32 angles; discrete order, duplicates and signed zero remain significant.
- `LBFBuilder` and `Separator` prepare once; Separator workers share the same immutable Arc. Controlled plain, top-2, seeded, stochastic top-k and seeded stochastic top-k search paths in LBF/worker/LNS now receive preparation. Searches validate geometry ownership and rotation bits once, then use the same table for focused and container range intersections.
- Public existing sampler/search entry points remain available and construct a local preparation when necessary. Replaced geometry Arcs or changed rotation signatures fall back to fresh preparation. No global cache, RNG draws, proposal filters or f32 sample-range expressions were added or changed.
- CollisionTracker now carries one `ShapeFactor` per placed slot. Separator/LNS tracker construction reuses run-owned roots; cloning/restoring trackers copies the linear factor vector. Geometry replacement checks hull-area bits and refreshes/falls back. No pair-factor table was added to PairMatrix.
- Scalar polygon/container quantification, specialized scalar/SIMD and bounded SIMD evaluation, and rigid group evaluation consume prepared penalties. The pair operation remains `(root1 * root2).sqrt()` and the container's self-penalty uses `(root * root).sqrt()`; loss multiplication order is unchanged. Legacy quantification wrappers remain independent bitwise references.

## Tests and evidence

All Cargo jobs were coordinated with root; no competing Cargo job was started.

- Before source implementation, `prepared_geometry` test target failed to compile only because the planned prepared module, prepared constructors/search variant, and penalty APIs did not yet exist. This was API-absence RED, not an observed runtime behavior failure.
- Production features are `simd capmods adaptive structure-first`.
- Initial focused GREEN: 5 passed, 1 ignored benchmark; sample/RNG bit equality, continuous/discrete/None rotations, signed-zero/duplicate ordering, focused/container/fixed intervals, stale Arc/rotation fallback, scalar/container factor bits, full cached-vs-fresh search result/evalcount/RNG, and tracker losses after rotated moves.
- Combined production regressions: `prepared_geometry`, `explore_code_optimization`, `separation_edge_order`, `p1_constructor`, `algorithm_audit`: 27 passed, 0 failed, 1 ignored benchmark. The existing independent evaluator oracles exercise changing items, rotations, bounds, layouts, cache copies/rollback, constructor feasibility, and cooperative stop.
- `--lib quantify::simd::bounded_tests`: 6 passed, 0 failed. New cached-penalty case compares bit patterns against the independent original SIMD wrapper and adjacent weighted rejection bounds; existing exact/next-down/next-up tests also pass.
- Isolated sampler microbenchmark: `cargo test -p swarm-core --features 'simd capmods adaptive structure-first' --test prepared_geometry prepared_sampler_construction_microbenchmark -- --ignored --nocapture` passed. For 20,000 continuous-16 sampler constructions/samples, legacy setup took 21.913 ms and prepared setup took 3.687 ms (about 5.94 times faster in this one isolated run). Both checksums were 1615403697. The immutable preparation was built once before timing the repeated prepared path.
- Default-feature build/scalar tests: root ran `cargo test -p swarm-core --no-default-features --test prepared_geometry --test initial_incumbent --test rigid_unit_lp --test original_overlap_first --test prepared_subset -- --test-threads=1`; 24 passed, including all five prepared tests, and one ignored benchmark.

## Scope and review

- Source changes are confined to the prepared module/lib declaration, sampler/search, LBF/worker/LNS/Separator wiring, quantify/tracker/SIMD, specialized evaluator and group evaluator. `sep_evaluator.rs` and `PairMatrix` were compared to the supplied backup and were unchanged by this subtask.
- A frozen prechange sampler oracle is retained in `tests/support/legacy_uniform_sampler.rs`; it is deliberately independent of the new preparation path.
- Existing public fresh search and new cached search use the same unchanged search/refinement engine; sampler construction itself is checked against the frozen prechange oracle. There is no duplicated full search engine.
- Non-hot callers of existing public sampler/search wrappers still prepare locally. NFP construction/sampling, raster caches, coordinate descent, pole preparation, and full geometry import are outside this optimization.
- No runtime configuration switch or feature flag was added. No commits, resets, external DLL writes, or subagents.
- No end-to-end garment nesting speedup, Release DLL, FFI, or WPF validation is claimed by this subtask. The microbenchmark isolates preparation, not solver throughput.

## Status

Implementation, production correctness checks, and isolated preparation microbenchmark are complete. Root owns independent review fixes and remaining default-feature verification while this worker proceeds to task10. Task2 source is frozen for handoff; no Cargo job remains running.
