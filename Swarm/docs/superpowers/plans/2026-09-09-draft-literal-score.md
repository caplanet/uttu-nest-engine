# Draft Init literal scoring implementation plan

**Goal:** Apply the user's ordered attachment/NFP placement flow and literal area formula to the ribbon's Column Draft Init.

**Architecture:** Keep the existing NFP kernel, legal rotations, geometry certification and complete-layout fallback. Separate polygon area/scoring from contact generation, and prepare directional NFP cache entries before binary lookup during placement.

**Tech stack:** Rust, existing swarm-nfp and Jagua geometry, existing C ABI/WPF selector.

**Spec:** User's 2026-09-09 placement pseudocode and explicit reply selecting `avg_s = placed_area / strip_count`. With no additional surface reply, the stated implementation assumption is one SPInstance = one physical fabric surface, strip_count = 1. Columns remain attachment search regions. Exact definitions are in `docs/DRAFT_COLUMN_INIT.md`.

## Constraints

- Work only in AdaptiveAlgorithm2. Do not read Uttu.Nest or fable.
- Preserve the current dirty checkout. Backup: `target/draft-input-formula-before-20260909-170102`.
- No new configuration knob or ABI field.
- Do not substitute weighted efficiency shortfall for the requested area average.
- Maximum 10,000 position evaluations per piece attempt, shared across rotation candidates.
- Never return a partial or uncertified layout as success.

## Steps

- [x] Add tests for literal area/count/reference-area, literal area/strip-count, four weight breakpoints, clipped concave polygon area, invalid denominators and large coordinates. Run red, then implement `swarm-core/src/refplace/column_score.rs`.
- [x] Test directional sorted cache insertion and lookup in `chain.rs`; retain failed/empty entries without treating them as usable boundaries.
- [x] Connect preparation, binary lookup and explicit invalid/held/ready outcomes in `columns.rs`. Preserve independent root seeding and deferred completion so all copies survive.
- [x] Connect literal score to candidate selection using the stated single-fabric mapping. Keep deterministic tie-breaking explicit; do not invent a multiplication by the four-branch weight absent from the final formula.
- [x] Test sequential piece/rotation processing, 10,000 shared cap and report/actual placement consistency. Retain exact validation on every committed position and on the completed layout.
- [x] Diagnose the two pre-existing overlapping-shape test failures before claiming regressions pass. Initial baseline: 6 passed, 2 failed, 2 ignored; both now pass with the original-outline predicate on candidate/refinement positions.
- [x] Run focused core and FFI tests, build native/WPF Release, and update `docs/DRAFT_COLUMN_INIT.md`. Results and three-run real-input comparison: `docs/SWARM_DRAFT_LITERAL_IMPLEMENTATION_2026-09-09.md`.

No commit or push is requested.
