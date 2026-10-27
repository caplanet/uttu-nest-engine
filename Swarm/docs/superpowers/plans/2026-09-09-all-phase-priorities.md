# Swarm priorities 1, 2, 3, 4, 5, 9, 10 implementation plan

> Execute with subagent-driven-development and focused tests. The user's request authorizes implementation in the current checkout; no extra design approval is required.

**Goal:** Implement the seven selected recommendations with unchanged geometry contracts and measurable validation.

**Architecture:** Keep the existing solver strategies and exact certification. Add immutable prepared data reuse, a fail-fast validation path, constraint-preserving incumbent selection, and bounded joint unit moves for Polish. Do not replace work with configuration changes.

**Tech stack:** Rust, Jagua, Rayon, Clipper2, existing LP backends and C ABI; WPF sample for final artifact integration.

**Spec:** `docs/SWARM_ALL_PHASE_ALGORITHM_PROPOSALS_2026-09-09.md`, priority table rows 1,2,3,4,5,9,10 and corresponding detailed sections.

## Global constraints

- Only AdaptiveAlgorithm2; do not read Uttu.Nest or fable.
- Preserve current dirty files and the last Draft implementation. No auto-rollback, commit, push or external DLL deployment.
- Baseline: `target/all-phase-priorities-before-20260909-173619` (182 source/test/config files and Release DLL).
- Retain full demand, OrgID/local-global mapping, rotations, frozen/frozen and frozen/exterior exceptions, rigid unit transforms, lane bounds and existing overlap mm semantics.
- Stop/error/timeout must leave the last certified complete incumbent, with no callback/GetSolution regression.
- Do not add runtime Config switches or new compile-time features as substitutes for implementation.
- Focused tests before/with implementation. Root controls Cargo build ownership to avoid competing compiles/benchmarks. A worker may request a test slot.
- One implementation subagent at a time; root works on disjoint files. Independent read-only analysis/review is allowed.

## Task 1: Effective NFP cost estimate

Files: `swarm-core/src/nfp_strategy/config.rs`, minimal call-site alignment if required.

- [x] Verify execution precedence against pipeline.rs and add tests for all strategy-selection combinations.
- [x] Compute cost for the selected branch, rather than multiplying mutually exclusive best-fit and Beam factors. Preserve explicit ordering count/time limits.
- [x] Run focused cost/selection tests.

## Task 2: Immutable sampler/penalty preparation

Files: `sample/uniform_sampler.rs`, `sample/search.rs`, sampler callers, `quantify` and evaluator preparation as needed.

- [x] Cache rotation-origin bboxes with exact geometry/rotation ownership; avoid stale pointer-only keys. Share immutable prepared data across focused/container sampler creation and repeated calls.
- [x] Reuse sqrt(hull_area) or pair penalty in run-owned preparation; retain floating operation order and RNG draws. Continuous rotation sampling preserves the existing 16-angle proposal table and projection behavior.
- [x] Compare baseline and prepared samples/loss bit patterns plus RNG state; exercise geometry/rotation replacement and fixed intervals.

## Task 3: First original-overlap violation

Files: `original_overlap.rs`, focused tests.

- [x] Add shared inspection traversal with first/all modes. First mode stops at first violation and avoids eager full pair prewarm. All mode remains for repair/diagnostics.
- [x] Preserve deterministic pair/error ordering, cancellation, cache validity and complete success certification.
- [x] Cover valid/invalid layouts, frozen policy, cached repeat, cancellation and fewer exact calls on early failure.

## Task 4: Reuse full geometry in V2 lanes

Files: `src/grouped_engine.rs`, a focused import/subset helper in `util/io.rs` if needed, tests.

- [x] Build lane-local item identity/demand/rotation records using full-instance prepared geometry Arcs when import settings are equivalent.
- [x] Keep lane-specific strip dimensions and local/global IDs/OrgID/rigid membership; preserve the importer geometry of both area and mm simplification modes.
- [x] Compare sharing against independent import transforms/geometries and exercise lane/hint/frozen/group correctness. Fall back only for concretely incompatible policies.

## Task 5: Certified Init incumbent and partial improvement

Files: `src/ffi_step.rs`, `optimizer/mod.rs`, `initial/attach.rs` or focused helper modules/tests.

- [x] Preserve a certified complete initial incumbent when optional constructors fail, expire, return partial, change frozen/groups or worsen length.
- [x] Accept only certified no-worse complete candidates within the existing phase budget; retain explicit Draft selection as primary when no incumbent yet exists.
- [x] Adapt the separate Initial phase to reuse warm-start/frozen/group constraints and improve movable units rather than constructing unusable all-new candidates.
- [x] Test no-regression, partial/expired candidates, frozen/group preservation and cancellation.

## Task 9: Rigid-unit LP

Files: `optimizer/polish/lp.rs`, LP gate in `optimizer/polish/mod.rs`, dedicated tests.

- [x] One dx/dy pair per rigid unit, fixed rotations; aggregate coefficients sharing variables and omit internal group contacts.
- [x] Union member bounds/trust constraints, pin units containing frozen members, apply one group move through Separator APIs.
- [x] Retain exact post-step certification/backtracking and original incumbent restoration on stop/failure.
- [x] Cover useful multi-member compression, frozen member, internal offsets/rotation, ungrouped equivalence and invalid candidate rollback.

## Task 10: Joint rightmost pocket assignment

Files: `optimizer/polish/holes.rs` and a focused joint search module/tests if needed.

- [x] For 2-4 movable rightmost units, generate bounded pocket proposals and include unchanged positions.
- [x] Search a small joint assignment with candidate-candidate collision checks. Primary objective is global max right edge, ties lexicographically smaller descending right edges.
- [x] Apply and certify as one transaction; preserve rotations/groups/frozen, strip nonincrease, existing hole-fill deadline and stop semantics.
- [x] Regression fixture where one-at-a-time placement wastes the pocket needed by another unit; reject conflicting assignment and restore after cancellation/failure.

## Integration and report

- [x] Per-task code reviews and broad final review against backup diffs.
- [x] Focused/core/FFI regression checks and deterministic microbenchmarks; same-input actual markers where feasible, keeping preparation/certification in total time.
- [x] Rust Release fat LTO and WPF x64 Release builds, DLL hash equality.
- [x] Update the proposal document status and write a report distinguishing measured results, limited experiments and untested production paths.


