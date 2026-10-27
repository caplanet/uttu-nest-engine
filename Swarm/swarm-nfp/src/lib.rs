//! Complete polygonal no-fit-polygon (NFP) geometry kernel.
//!
//! The kernel is deliberately independent from Swarm's optimizer, FFI and
//! UI.  It works on filled polygon sets (concave shells, holes and disconnected
//! components), quantizes input once to a 0.001 mm integer grid, and performs
//! all Minkowski and Boolean operations with Clipper2's i64 kernel.
//!
//! # Configuration-space convention
//!
//! For a fixed shape `F` and an anchor-relative moving shape `M`, the pair NFP
//! is the closed forbidden translation region
//!
//! `NFP(F, M) = F + (-M)`.
//!
//! [`NfpEngine::complete_pair_nfp`] retains that set as an exact union of
//! convex integer-grid primitives and also supplies a compact Boolean contour.
//! Use [`CompleteNfp::locate`] for authoritative NFP membership. The compact
//! contour intentionally regularizes internal contact seams and rounds derived
//! intersections back to the grid; use [`NfpEngine::classify_pair`] when
//! directly classifying one zero-clearance placement.

/// 하드닝(C-7): release 에서 panic/expect 메시지를 `""` 로 컴파일 제외. (swarm-core 의 dmsg! 와 동일)
#[macro_export]
macro_rules! dmsg {
    ($s:literal) => {{
        #[cfg(debug_assertions)]
        {
            $s
        }
        #[cfg(not(debug_assertions))]
        {
            ""
        }
    }};
}

mod backend;
mod complete;
mod engine;
mod error;
mod model;
mod prepare;

pub use complete::CompleteNfp;
pub use engine::{Clearance, FeasibleRequest, NfpEngine, PlacedObstacle, PreparedRegion};
pub use error::{NfpError, NfpResult};
pub use model::{
    Bounds, Location, Orientation, PairRelation, Point, Polygon, PolygonSet, Ring, Translation,
};
pub use prepare::PreparedShape;

/// Integer grid units per millimetre.
pub const GRID_SCALE: i64 = 1_000;

/// Size of one integer grid unit in millimetres.
pub const GRID_STEP_MM: f64 = 1.0 / GRID_SCALE as f64;

/// Conservative bound applied to every coordinate produced by the kernel.
///
/// This is intentionally far below Clipper2's i64 hard limit and leaves room
/// for reflection, Minkowski addition, translations and container complements.
pub const MAX_DERIVED_GRID_COORD: i64 = 1_000_000_000_000;
