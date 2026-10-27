# Adaptive shrink, early certified callbacks, and original Place rendering

**Goal:** Implement the approved recommendations from Explore review sections 8–10 and draw Uttu.Nest Places from the original contour.

**Architecture:** Adaptive Explore gets a bounded effort controller at attempt boundaries; the existing synchronous worker selection remains. Completed zero-loss worker snapshots are certified and reported on the listener owner thread while peers finish. Uttu.Nest already uses the original display contour; preserve product code and add actual Skia rendering regression tests.

**Tech stack:** Rust, Rayon, C ABI, C#/WPF, Skia.

**Spec:** `docs/SWARM_EXPLORE_OPTIMIZATION_REVIEW_2026-09-09.md`, sections 8–10; current user request for Uttu.Nest original polygon rendering.

## Constraints

- Work on the user's current dirty checkouts without discarding existing changes; save touched-file backups under `target/adaptive-control-implementation/before`.
- Swarm: `C:/Caplanet/AiWorks/AdaptiveAlgorithm2`. Uttu: `C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest` and necessary dependencies only. Never access fable.
- No new Config switches, ABI fields, or compile-time feature switches.
- Preserve synchronous round winner/tie selection and RNG. Only complete exact-certified layouts may be published. Stop/final/export must preserve published best.
- Existing overlap, locks, groups, rotations, coordinates and original-geometry verification remain authoritative.
- Source changes outside writable root use approved execution. No commits or pushes in this task.

## 1. Shrink controller (root)

- [x] Add tests for late-stage cap, failure backoff to an actually easier target, slow recovery, cancelled observations, finite positive steps, and frozen/representable limits.
- [x] Add a private controller under `swarm-core/src/optimizer/explore_mods/` and integrate at attempt/success/failure boundaries in `explore_mods.rs`.
- [x] Keep the existing base step and density multiplier as proposal. Apply a decreasing time ceiling and a bounded difficulty multiplier. Observe certified successes only; global Stop/deadline is censored.
- [x] Bound individual attempts using the actual remaining deadline and recent effort; wrap Terminator without mutating the phase timeout or converting local expiry to persistent Stop.
- [x] After difficult failure restore last certified snapshot and retry an intermediate width; preserve failed snapshots separately and avoid simultaneous oscillation/relief on that boundary.
- [x] Run focused unit/integration tests before combined validation.

## 2. Early certified callback (independent Rust agent)

- [x] Test owner-thread callback before a blocked peer completes, 1-worker progress, invalid-original suppression, cancellation retention, and unchanged winner selection.
- [x] Add an internal early-certified listener method with existing report fallback; use scoped Rayon work and owner-thread queue processing, no per-round OS threads and no unsafe listener sharing.
- [x] Snapshot only complete zero-loss proposals, certify frozen/group/original constraints, and preserve early certified state through separate() return.
- [x] Integrate FFI JSON eligibility and non-regressing certified checkpoint behavior without ABI changes.

## 3. Uttu.Nest original visual contour (independent Uttu agent)

- [x] Read checkout instructions and trace original contour, buffers, simplification, transform, holes, caching, and Skia usage.
- [x] Back up the touched-file state; verify the existing original contour and placement transform. No product rendering replacement was needed; add Skia bitmap tests and documentation.
- [x] Verify source/diff and build the scoped application. Report if actual UI execution remains unverified.

## 4. Integrated verification (root)

- [x] Review all changes and conflicts against saved pre-task files.
- [x] Run core and FFI tests for cancellation, certification, frozen/groups, controller, and early delivery; SIMD feature coverage; no SIMD kernel changed, and non-SIMD was not rerun.
- [x] Run a bounded before/after experiment with identical inputs and CPU/seed budget, sequentially after builds. Do not claim overall gains from unit tests.
- [x] Build Rust Release with fat LTO and WPF Sample; confirm deployed sample DLL hash. Do not replace Uttu solver DLL as part of its display-only task.
- [x] Update the review with implementation, measured outcomes and limitations; provide backup and build paths.

## Outcome

Implemented and built. All 8 bounded performance outputs certified. WSjk mean length regressed 26.977 mm; OPS2 improved 9.282 mm. Changes and backups remain available for the user's keep/rollback decision. Combined measurement does not isolate callback overhead; UI execution latency remains unmeasured. Full results are in review section 11.

