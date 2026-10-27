use thiserror::Error;

pub type NfpResult<T> = Result<T, NfpError>;

/// Typed failures.  The kernel never replaces invalid geometry with a convex
/// hull and never silently drops holes or disconnected components.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum NfpError {
    #[error("polygon set must contain at least one non-empty component")]
    EmptyShape,

    #[error("{context} contains a non-finite coordinate at ring {ring}, vertex {vertex}")]
    NonFiniteCoordinate {
        context: &'static str,
        ring: usize,
        vertex: usize,
    },

    #[error("{context} ring {ring} has fewer than three grid-distinct vertices")]
    TooFewVertices { context: &'static str, ring: usize },

    #[error("{context} ring {ring} has zero area after 0.001 mm quantization")]
    ZeroAreaRing { context: &'static str, ring: usize },

    #[error("{context} ring {ring} self-intersects")]
    SelfIntersection { context: &'static str, ring: usize },

    #[error("hole {hole} is not strictly contained by component {component}")]
    HoleOutsideShell { component: usize, hole: usize },

    #[error("holes {first} and {second} in component {component} intersect or nest")]
    InvalidHoleRelation {
        component: usize,
        first: usize,
        second: usize,
    },

    #[error("coordinate overflow during {stage}: |{value}| exceeds {limit} grid units")]
    CoordinateOverflow {
        stage: &'static str,
        value: i128,
        limit: i64,
    },

    #[error("invalid orientation: {reason}")]
    InvalidOrientation { reason: &'static str },

    #[error("triangulation failed for component {component}: {reason}")]
    Triangulation { component: usize, reason: String },

    #[error(
        "triangulation area mismatch for component {component}: expected area2 {expected}, got {actual}"
    )]
    TriangulationAreaMismatch {
        component: usize,
        expected: i128,
        actual: i128,
    },

    #[error("polygon Boolean operation {operation} failed: {message}")]
    Boolean {
        operation: &'static str,
        message: String,
    },

    #[error("Minkowski operation produced an invalid or empty forbidden region")]
    EmptyNfp,

    #[error("complete primitive count overflowed usize during {operation}")]
    PrimitiveCountOverflow { operation: &'static str },

    #[error("clearance must be finite and non-negative")]
    InvalidClearance,

    #[error("Euclidean clearance max_excess_mm must be finite and greater than zero")]
    InvalidClearanceTolerance,

    #[error(
        "{operation} has a point/line bounding anchor domain that a filled PolygonSet cannot evaluate; explicit lower-dimensional contact-space evaluation is required"
    )]
    DegenerateConfigurationSpace { operation: &'static str },

    #[error("Clipper output topology could not be reconstructed: {reason}")]
    InvalidOutputTopology { reason: String },
}
