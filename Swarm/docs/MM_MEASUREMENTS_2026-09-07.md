# mm simplification — 2026-09-07

Source: 여성WSjk_비교용-G_Swarm_PLT변환.json; 156 pieces, 1524mm strip height, all rotations [0], original area 7,752,563.7196875mm². Release: simd,capmods,adaptive,structure-first. Compilation excluded. Times are individual measurements on this machine, not guarantees.

## Boundary simplification only

The source for each bound is the contour AFTER the explicit overlap offset, BEFORE simplification. The bound is not a cancellation of overlap against inflated area.

| Bound | Overlap | Vertices before → after | Simplifier time | Max independently sampled boundary deviation |
|---|---:|---:|---:|---:|
| 0.1mm | 0 | 9194 → 8519 | 0.170s | 0.099925mm |
| 0.3mm | 0 | 9194 → 5664 | 0.351s | 0.299940mm |
| 0.5mm | 0 | 9194 → 4729 | 0.380s | 0.499470mm |
| 0.1mm | 0.1mm | 14080 → 8502 | 1.143s | 0.099963mm |
| 0.3mm | 0.1mm | 14080 → 5655 | 1.313s | 0.299784mm |
| 0.5mm | 0.1mm | 14080 → 4669 | 1.330s | 0.499902mm |

GEOS independently checked all 936 Inflate results and another 312 Deflate results, including source/result containment. Deflate at 0.3mm: 9194 → 5652 (0.340s) without overlap; 14080 → 6291 (1.165s) with overlap. Maximum deviations stayed below the bound. Largest unbuffered containment-area residue across both modes was 6.4e-9mm²; the audit explicitly allows 0.0002mm for decimal/f32 tangent representation. Production acceptance uses continuous capsule coverage, not the GEOS sampling check.

## Full public Draft Init

Fresh construction: every placement hint removed. mm mode replaces the importer area-ratio/narrow-pocket operations, and disables Draft's extra 0.005 area-ratio proposal simplification.

| mm bound | Overlap | Width | Original utilization | Init + retrieval | Deadline completion | Failed NFP proposals |
|---|---:|---:|---:|---:|---|---:|
| 0.3 | 0 | 7986.073 | 63.6982% | 9.560s | yes | 29 |
| 0.3 | 0.1 | 8353.053 | 60.8997% | 10.804s | yes | 34 |
| 0.5 | 0 | 7507.0757 | 67.7625% | 7.695s | no | 22 |
| 0.5 | 0.1 | 7474.755 | 68.0555% | 8.980s | no | 9 |

All runs returned 156 pieces and passed source-outline fabric-boundary checks. Independent GEOS checks on original outlines found no overlap beyond the configured overlap plus 0.002mm numerical contact allowance per shape (intersection-area threshold 1e-5mm²).

0.3mm makes a finer NFP contour than the prior aggressive area-ratio proposals. It did NOT improve Draft quality/time here: the 8-second search budget expired and vertical completion was used. 0.5mm completed via NFP proposals without deadline completion and is the WPF input default (option still OFF by default). The reference 78.79% density has not been reproduced.

Logged NFP rejection examples are Clipper Boolean-output topology reconstruction failures (degenerate/self-intersecting output paths), including 49→89, 49→44, 48→85, 50→47 at 0.5mm/0.1mm. The input polygons were independently valid, including after 0.001mm quantization. The kernel failures are NOT bypassed or treated as collision-free; existing exact placement checks remain authoritative. The underlying Boolean kernel was not changed in this task.

## Verification and artifacts

- Three primitive unit tests, three core integration/marker tests, native mm-option/reset test, and nine managed ABI/config tests passed.
- Public API synthetic regressions and four marker configurations passed.
- Release native library and WPF build succeeded; WPF retains 14 pre-existing warnings.
- `mm-geometry.json`, `mm-geometry-audit.json`, `audit_mm_geometry.py`: source/result boundaries and independent checks.
- `mm03-overlap00-solution.json`, `mm03-overlap01-solution.json`, `mm05-overlap00-solution.json`, `mm05-overlap01-solution.json`: public transforms.
- `mm-layout-audit.json`, `audit_mm_layout.py`: independent original-outline validation.
- Configuration/implementation details: `docs/MM_SIMPLIFICATION.md`.
