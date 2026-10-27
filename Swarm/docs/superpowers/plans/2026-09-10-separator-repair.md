# Separator repair implementation plan

> Use subagent-driven-development for independent modules and review; integrate and benchmark serially.

**Goal:** Add contact-directed search, small-cluster LP repair, and blocker-chain relocation to Explore with independent Config/WPF controls.

**Architecture:** A bounded repair pass runs only after measured stagnation, on private state. Every proposed move is checked against all neighbors using the existing tracker; zero loss additionally requires original-geometry certification. The current synchronous Separator, shrink policy and certified incumbent remain authoritative.

**Tech stack:** Rust, existing geometry and LP solver, C ABI, C#/WPF.

**Spec:** User-approved three-algorithm recommendation in this conversation, with explicit Config and WPF integration.

## Constraints and decisions

- Work in the explicitly requested AdaptiveAlgorithm2 checkout; preserve dirty work. No Uttu.Nest/fable reads, commits or pushes.
- Backup: backups/separator-repair-20260910-171758; source ZIP verified before edits.
- Expose `ExplorationConfig.repair: SeparationRepairConfig` with direction_enabled, cluster_lp_enabled, chain_enabled (bool), stagnation_iterations, max_passes, max_units, max_evaluations, time_budget_ms (u32), max_move_mm (f32).
- Defaults: all algorithms off pending benchmark evidence, stagnation 24 iterations, 2 passes per separation, 4 units, 64 evaluations, 20 ms per pass, 2 mm movement bound. Controls are explicitly opt-in and not silent unused preparation.
- C config uses a new ABI tail block and version bump; update header, managed layout/handshake and tests together. Never enlarge a middle C struct.
- No config/feature switch substitutes for implementation. Disabled path creates no repair state or geometry.
- Cancellation before adoption retains original layout; internal solver time is bounded where supported and checked before/after.

## Tasks

- [x] Back up current source, uncommitted diff and deployed native artifacts; verify ZIP and hashes.
- [x] Contact candidate module: tests for oblique contact, bounded finite translations, degenerate edges and cancellation; implement bounded directional boundary events as proposals only.
- [x] Cluster LP module: test coupled translations, fixed obstacles, slack handling and invalid inputs; use existing pure-Rust LP backend with movement bounds. Exact checker owns feasibility.
- [x] Integration and chain: immutable start snapshot, colliding units plus nearby blockers, one-at-a-time/LP/chain proposals in priority order; accept only lower weighted loss with nonincreasing real loss, and native original certification at zero loss. Test rejection/cancel/locked/group paths.
- [x] Config and WPF: three switches and bounded budgets, native import/export defaults, managed conversion, UI bindings and numeric validation. Test ABI handshake and roundtrip.
- [ ] Independent review; core/native/managed tests; build Release and compare algorithms separately on WSjk/OPS2 using equal-build DLL and same Init/seed/time budget.
- [ ] Choose defaults from evidence; rebuild fresh PGO with CGU1/fat/static CRT, build WPF and verify DLL identity and ABI.
- [ ] Archive changes and experiment results with rollback instructions; report achieved behavior and measured limits.

## Validation rules

Algorithm micro-tests must demonstrate moves that reduce actual collision cost, not merely creation of proposals. Benchmark comparisons include final strip length, certified callback retention and per-pass success/time counts; no guarantee for unseen inputs. Config remains available for all correctly implemented algorithms even if speed gain is marker-dependent; performance-negative algorithms stay off by default.

## Review ledger

- Contact and LP modules independently reviewed: no correctness blocker. Added centered-containment and signed-gap LP tests.
- Fixed unbounded member-pair work in preparation: cached unit bounds, bounded top-K selection, cancellation within member/pair scans.
- Fixed LP body-cap rejection to skip only LP and leave chain fallback available.
- Final independent review found no remaining blocker. Whole-pass time limit remains cooperative: individual snapshot/tracker/exact-certification calls cannot be preempted internally.
- Final core run: 777 passed, 13 pre-existing ignored. Managed: 99 passed. Four integration suites: 11 passed.
