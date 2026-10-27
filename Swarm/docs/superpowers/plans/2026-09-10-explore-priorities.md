# Explore priority improvements implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development for independent tooling/review and execute the tightly coupled solver changes sequentially.

**Goal:** Execute the five user-approved Explore improvements in priority order; retain only changes that pass correctness and representative performance gates against the current PGO build.

**Architecture:** Keep synchronous Separator, candidate arithmetic/order, existing shrink policy, exact certification and Stop incumbent ownership. Benchmark each change separately; archive rejected trials and restore only task-owned changes. Use worker/search-local state; no Config or compile-time feature switches to substitute for optimization.

**Tech Stack:** Rust 1.99 nightly, jagua-rs, Rayon, C ABI 15, C# .NET 10, Python benchmark orchestration.

**Spec:** User-approved proposal in this conversation: collision reset → duplicate candidate evaluation → collision relation updates → failed-pool diversity → bounded joint separation.

## Constraints and evidence

- Work only in `C:/Caplanet/AiWorks/AdaptiveAlgorithm2`; do not read Uttu.Nest/fable source.
- Current source includes unrelated uncommitted work. Preserve it; do not commit or reset the checkout.
- Baseline DLL SHA256 `65689606F2C941F7694EFA381865B8FCFA89313F1F452260ECA8D0A406B3062C`, static CRT, fat LTO, CGU=1, PGO.
- Backup: `target/explore-priorities-20260910-144827/before`, source ZIP and SHA256 manifest. Includes all Git-visible source and current DLL/profile.
- Use snapshot copies inside the allowed workspace for rejected trials and baseline builds. User explicitly chose this source path; no new worktree/branch is required.
- Benchmark processes run sequentially without simultaneous builds/tests. Same input, seed, workers, budgets, geometry policy and compilation settings on both sides. PGO final acceptance uses fresh profiles for changed source.
- Existing reference tests plus deterministic candidate/round comparisons guard f32 summation/order. Full timed comparison is still necessary because ALNS learns elapsed time.

## 0. Backup and measurement

- [x] Copy/hash current source and native artifacts before solver edits.
- [x] Add independent actual-DLL Explore benchmark under `scripts/explore_bench`; native ABI/export checks and exact native collision status, placement counts and monotone certified callback trace.
- [x] Preserve benchmark manifests, input hashes, per-run traces and pair order. Start WSjk/OPS2, seeds 41/42/43, 20 seconds; broaden surviving variants to longer budgets and seeds.

## 1. Sparse collector reset

Files: `eval/specialized_jaguars_pipeline.rs`, `eval/sep_evaluator.rs`, collector tests.

- [x] Add a reference-based reset test covering sparse slots, key generations, removal, duplicate detection, partial early-return and repeated reload.
- [x] Track touched keys in reusable worker-owned storage. Reset only touched entries; retain `SecondaryMap` lookup and exact pending-slot sort order.
- [x] Differential candidate tests: same bits for Clear/Collision, same Invalid decisions over finite/infinite/tied bounds and changed layouts.
- [x] Build and measure as a separate variant. Archive source/DLL and reject reproducible performance regressions.

## 2. Duplicate candidate evaluation

Files: search-local evaluator wrapper and its tests; benchmark-only duplicate diagnostics.

- [x] Measure exact `(x_bits,y_bits,angle_bits)` repetition within one fixed layout/item/weight search. No quantization.
- [x] Closed without introducing a cache: 45,020 measured evaluations had zero exact duplicates. Bound test drafts are archived but were not executed; no cache correctness claim is made.
- [x] Trial a bounded cache only if measured repetition supports it. Cache Invalid only as a rejection threshold, never as exact loss.
- [x] Keep only if cache overhead is repaid on marker workloads; document a rejected trial rather than leave unused preparation in production.

## 3. Collision relation updates

Files: `quantify/tracker.rs`, tracker differential tests.

- [x] Six tracker tests passed for moves, snapshot/COW, mutation, restore, frozen and weight changes. Group integration tests also passed on the final retained state; P3 was rejected before broader testing.
- [x] Trial active-neighbor adjacency while retaining dense pair lookup and original summation order. Direct mutation invalidates/rebuilds adjacency. Copy cost is included in benchmark.
- [x] Compare independently; archive/revert the variant if adjacency maintenance costs more than row clearing saves.

## 4. Failed-pool diversity

Files: `optimizer/explore_mods.rs`, small pool helper/tests.

- [x] Test exact pose-multiset identity independent of placement keys, repeated IDs and signed-zero policy; always retain minimum-loss candidate.
- [x] Trial deduplication at failed-attempt boundaries, followed by limited collision-signature diversity only if needed. Avoid new per-candidate work and preserve original RNG use where possible.
- [x] Compare final lengths, target times and failures; reject marker-specific repeated deterioration.

## 5. Bounded joint separation

Files: dedicated `optimizer/separator/joint.rs`, integration in stagnation path, synthetic tests.

- [x] Construct a discrete-neighborhood single-move trap solved by joint moves (not a proof against the existing continuous sampler); test external-neighbor interactions, locked/grouped members and cancellation rollback.
- [x] Trial a few connected movable units with bounded candidates/nodes; evaluate transactionally against all external neighbors. Keep temporary search states separate from certified public incumbents.
- [x] Trigger only at stagnation boundaries; retain synchronous main loop and current shrink behavior. Do not assume a local win proves same-time superiority.
- [x] Compare independently; archive rejected source without deploying it.

## Final gate

- [x] Review all retained diffs and run relevant core/native tests; no weakening original overlap or constraint checks.
- [x] Fresh PGO build of retained source using the same representative training procedure, verify actual flags, static CRT imports, ABI and exports.
- [x] Paired timed comparison with the backed-up PGO DLL; replay any apparent regression. No blanket claim about unseen markers.
- [x] Build WPF with accepted DLL, verify hash, report changes kept/rejected, measurements and backup/restore paths.

## Rulings

- The five solver steps are sequential; independent benchmark tooling can be delegated while the first step is developed.
- A rejected experiment still completes its priority item. It must be preserved with evidence rather than silently left enabled or substituted by a Config switch.
- Small initial comparisons screen candidates; only final equal-build comparisons justify deployment. Existing PGO coverage counts are not CPU timing percentages.

## Recorded decisions

- P1 retained and deployed after the final PGO comparison and regression replays; measured limits are recorded in the report.
- P2 not introduced: no exact duplicates in the measured fixed-layout searches.
- P3 reverted after repeated same-budget width regressions.
- P4 reverted after repeated same-budget width regressions; the short WSjk runs do not establish a pool-diversity benefit.
- P5 reverted conservatively: longer-run benefit was inconsistent and target observations were later; the one longer-run width regression did not repeat.
- Final retained tests: 754 core + 11 integration + 104 native passed, 13 pre-existing ignored core tests.
- Durable baseline ZIP/DLL also copied and hash-verified under `backups/explore-priorities-20260910-144827`.
- Detailed evidence: `docs/EXPLORE_PRIORITY_EXPERIMENTS_2026-09-10.md`. Final PGO gate completed: P1 retained and deployed; see report for the initial non-reproduced regression and measured limits.
