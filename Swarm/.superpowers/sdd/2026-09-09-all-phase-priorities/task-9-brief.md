# Task 9: rigid-unit Polish LP

Work in C:\Caplanet\AiWorks\AdaptiveAlgorithm2 only. Do not read Uttu/fable. User approved priority 9 of docs/SWARM_ALL_PHASE_ALGORITHM_PROPOSALS_2026-09-09.md, section 9.1.

Own: swarm-core/src/optimizer/polish/lp.rs; LP-specific gate in polish/mod.rs; a NEW focused test file such as swarm-core/tests/rigid_unit_lp.rs. Do not edit holes.rs, Separator core, existing tests/postprocess_phases.rs or other files without coordinating with root.

Implement one dx/dy pair per rigid unit with fixed rotation; map every member's bounds/contacts into the same variables. Aggregate coefficients if endpoints map to the same variables and omit internal rigid contacts. Any frozen member pins the whole unit. Apply a representative unit move once through the existing group-aware Separator path; do not move every member twice. Preserve exact geometry/original certification, strip nonincrease, backtracking, cancellation and best-incumbent restore. Ungrouped inputs retain existing mathematical behavior.

Tests: multiple-member group can actually compress (LP was previously bypassed), offsets/rotations preserved, frozen member pins unit, external collisions rejected, ungrouped regression unchanged. Tests before implementation where practical. Do not run competing Cargo jobs: ask root for the focused test slot after adding red tests, then after implementation. Root is editing config.rs/original_overlap.rs first, disjoint from yours.

Backup exists at target/all-phase-priorities-before-20260909-173619. Preserve all preexisting diffs, no commits/push/reset. Do not spawn subagents. Self-review and write full changes/tests/concerns to task-9-report.md in this same directory; return concise status. If more API changes are necessary, message root with a concrete proposal and continue independent parts.
