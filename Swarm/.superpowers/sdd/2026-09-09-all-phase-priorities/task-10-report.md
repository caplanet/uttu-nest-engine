# Task 10 — bounded joint Hole Fill

Implemented 2026-09-09 in `swarm-core/src/optimizer/polish/holes.rs`, child `holes/joint.rs`, and new `tests/joint_hole_fill.rs`. Root owns Polish adoption wiring and the existing static-wall test correction.

## Behavior

Both raster and NFP public entrypoints attempt one bounded joint assignment before their existing single-donor follow-up. Select 2–4 rightmost movable units; each has a stay plus up to six translated poses, generated from at most four existing pockets and 96 anchors per pocket. Pocket diversity is preserved. Pairwise candidate CDE conflicts feed a branch-and-bound assignment search capped at 4,096 nodes (the complete maximum tree is 2,801). No new Config/feature/UI switch, full-instance import per candidate, RNG use, or geometry tolerance was introduced.

Singleton identity is PItemKey, retaining repeated copies. Groups share their internal representative; any frozen member pins the whole group. Candidate rotations remain fixed. Geometry checks filter selected members and Exterior only; the existing native `bbox_is_contained_mm` policy checks strip containment, preserving valid exact boundary contact, while item/hole/quality-zone collisions remain CDE checks. Joint candidates use small layouts containing only their member geometry. Final original geometry, frozen and rigid-group certification is authoritative.

The objective includes every movable unit, including unselected units: sorted descending right edges compared lexicographically. Physical strip width must not increase. Root's Polish hook consumes `HoleFillOutcome.certified`, a saved exact checkpoint created before greedy follow-up. Snapshot comparison is independent of the live Separator: O(n) representative aggregation, any-frozen exclusion, singleton copy identity, then descending edge sort. Duplicate external group IDs remain distinct internal units. A shorter physical strip is accepted; equal width uses the joint edge objective.

Joint preparation, candidate checks, pair conflicts, DFS and transactional movement poll cancellation. Before proof, failure or stop restores the complete incoming solution. After completed exact proof, the independent snapshot remains available even when subsequent legacy work stops or mutates live state. The public usize move result counts moved joint units plus legacy individual moves, preserving signatures.

## Evidence

RED: original implementation passed three of the five initial tests and failed the genuine joint-choice fixture and rigid-unit movement test. The greedy pocket trap remained at 19.004221. The fixture has a 6x7 pocket usable by either donor and a 5x4 pocket unusable by the larger donor. Candidate conflicts prevent putting both donors into the same occupied space.

GREEN production features (`simd capmods adaptive structure-first`): initial 5/5; expanded integration set joint_hole_fill 7/7, nfp_hole_placement 3/3, postprocess_phases 10/10. The first existing static-wall regression failed because its group was movable; root explicitly froze one member to preserve the test's static-wall premise.

Actual fixture: initial strip 32; legacy greedy 19.004221; joint raster 11.010111; joint NFP 11.010111; two joint units moved in both paths (42.06% shorter than greedy on this synthetic fixture). This is a quality regression fixture, not a representative throughput benchmark.

Expanded tests cover top/bottom boundary contact; repeated copy identity; rigid relative translation and exact rotation bits; any-frozen group pinning; eight cancellation budgets with full certification/tracker synchronization; both public paths; and real Polish publication and stop preservation. The Polish stop test arms only after the joint proof log, then stops at the third following poll inside the NFP follow-up, requiring Killed plus the exact saved joint width.

Remaining final verification results will be appended below. No native DLL or WPF build was run by this worker; root is coordinating those.

## Final verification

- Production features integrations: joint_hole_fill **7 passed**, nfp_hole_placement **3 passed**, postprocess_phases **10 passed** (20 total).
- No-default-features integrations: joint_hole_fill **7 passed**, postprocess_phases **10 passed** (17 total). nfp_hole_placement has **0 tests** in this configuration due to its existing feature gate; its three tests were verified with production features.
- Production holes unit pass initially had **10 passed / 1 failed**: the legacy narrow-pocket NFP test counted two successive moves (joint prelude then single-donor NFP), where it asserted exactly one. With root authorization, that test now explicitly uses max_candidates=1 to isolate its existing single-donor comparison and preserve the exact-one assertion. This is a test-scope clarification, not a relaxed collision/quality assertion.
- Comparator unit tests independently passed production features: equal-width [100,80,1] to [100,70,20], and internal group identity despite duplicate public IDs, per-group maximum, any-frozen exclusion, repeated singleton count.
- git diff --check passed for owned source/test files (only standard LF-to-CRLF informational warning).

The joint summary uses info level, matching existing Hole Fill summaries; the stop-provenance test therefore works with release_max_level_info as well. No representative real-input runtime improvement is claimed. Candidate/assignment caps intentionally trade search coverage for bounded work; original certification may conservatively reject a whole proposed assignment. Legacy follow-up keeps its previous behavior, while Polish receives the independent certified joint snapshot for cancellation recovery.
- Final no-default-features holes unit run: **11 passed / 0 failed**, including the clarified single-donor NFP test, both snapshot comparator tests, grid/bay cancellation tests, and grouped exact-hole regression. No Cargo process remains owned by this worker; slot returned to root. Root's upcoming production full-core run will reverify the final one-line legacy fixture clarification under production features.
