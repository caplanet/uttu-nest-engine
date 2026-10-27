# Priority 9: rigid-unit Polish LP

## Implementation

- `swarm-core/src/optimizer/polish/lp.rs`: removed the blanket place-group bypass. Each group uses its representative placement key and one shared `dx, dy` pair, with fixed rotation. Ungrouped placements use their own placement keys, preserving independent demand copies.
- Model construction and solution decoding share `LpVariables`. Every member still contributes its own four container inequalities and external member contacts. Internal unit pairs are excluded before the contact-row cap; model assembly additionally sums duplicate column coefficients and discards identically zero contacts.
- Any frozen member pins the entire unit. Translation decoding emits one representative move per movable unit, using the existing group-aware Separator batch and transactional rollback.
- After surrogate loss reaches zero and strip refitting completes, candidates must have nonincreasing strip width and pass `Separator::solution_is_certified` (including original geometry, rigid poses, and frozen poses). Rejected candidates follow the existing halving backtracking; cancellation restores the previous snapshot. Equal-width certified secondary moves remain allowed.
- `swarm-core/src/optimizer/polish/mod.rs`: enabled grouped LP in both the active-pass predicate and the phase invocation. Retained the predicate signature for existing callers. No changes to slide, holes, or Separator core.
- `swarm-core/tests/rigid_unit_lp.rs`: six integration tests cover real grouped compression, member offsets and fixed rotations, a frozen member pinning its unit while an independent item moves, external collision rejection followed by successful backtracking, ungrouped compression, grouped LP-only phase execution, and cancellation preserving the published incumbent.
- Two focused unit tests in `lp.rs` verify shared member bounds, duplicate coefficient cancellation, and internal contacts not consuming the contact budget.
- With root authorization, removed only the obsolete `(true, true, true)` grouped-LP-disabled tuple from the existing `empty_polish_has_zero_rounds_and_preserves_certified_poses` test. Its remaining cases are unchanged.

## Verification

All Cargo checks use `--features 'simd capmods adaptive structure-first'` and are coordinated with root; no competing Cargo jobs were started.

1. RED: `cargo test -p swarm-core --features 'simd capmods adaptive structure-first' --test rigid_unit_lp -- --nocapture` before production changes: 2 passed, 3 failed for the expected blanket-bypass behavior (grouped direct LP, frozen group plus independent item, grouped LP-only phase).
2. GREEN integration: same command after implementation: 6 passed, 0 failed, runtime 0.27 s; compilation 1 min 03 s. The sixth test was added for publication/cancellation preservation.
3. LP module/model regression: `cargo test -p swarm-core --features 'simd capmods adaptive structure-first' --lib optimizer::polish::lp -- --nocapture`: 20 passed, 0 failed, 3 intentionally ignored measurement/real-marker tests, runtime 0.95 s; compilation 1 min 37 s. The preexisting tests confirm identical ungrouped constraint rows, objective, and solved step, plus deterministic contact generation across worker counts.
4. Existing postprocess integration regression: `cargo test -p swarm-core --features 'simd capmods adaptive structure-first' --test postprocess_phases -- --nocapture`: 10 passed, 0 failed, runtime 0.19 s; compilation 1 min 10 s. This includes independent LP, grouping around hole filling, no-op phase cases, and published-incumbent cancellation.
5. `git diff --check` and a direct comparison against `target/all-phase-priorities-before-20260909-173619` were reviewed. No whitespace errors; Git prints the repository's existing LF-to-CRLF warnings.

## Scope and limitations

- No changes to holes, Separator core, external application code, ABI, or group rotation search. No commits, push, reset, or subagents.
- Translation only is intentional. Grouped objective pull is normalized per movable unit. Existing ungrouped mathematical equivalence tests are included in the LP module regression command.
- Synthetic exact-certified behavior is covered. No garment benchmark, release DLL build, FFI, or WPF validation was performed by this subtask.
- Per-accepted-step exact certification adds geometry-validation work to direct LP passes; performance on large garment layouts is not quantified here.

## Final status

Implementation and scoped verification are complete. The Cargo slot has been released to root. Changes were self-reviewed against the supplied source backup to separate them from preexisting workspace diffs; there are no known unresolved correctness findings for this scope. No further task-9 edits are in progress.
