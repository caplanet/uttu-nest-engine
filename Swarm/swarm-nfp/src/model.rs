use serde::{Deserialize, Serialize};

use crate::{NfpError, NfpResult};

/// A world-space point in millimetres.
#[derive(Debug, Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

impl From<(f64, f64)> for Point {
    fn from((x, y): (f64, f64)) -> Self {
        Self { x, y }
    }
}

/// A polygon ring.  The closing point may be omitted or repeated; preparation
/// removes a repeated closing point.  Input winding is ignored.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ring {
    pub vertices: Vec<Point>,
}

impl Ring {
    pub fn new(vertices: impl IntoIterator<Item = impl Into<Point>>) -> Self {
        Self {
            vertices: vertices.into_iter().map(Into::into).collect(),
        }
    }
}

/// One filled component: an outer shell minus zero or more holes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Polygon {
    pub outer: Ring,
    #[serde(default)]
    pub holes: Vec<Ring>,
}

impl Polygon {
    pub fn new(outer: Ring) -> Self {
        Self {
            outer,
            holes: Vec::new(),
        }
    }

    pub fn with_holes(outer: Ring, holes: Vec<Ring>) -> Self {
        Self { outer, holes }
    }
}

/// A filled polygon set.  Components may be disconnected; preparation unions
/// overlapping or touching components into a canonical set.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolygonSet {
    #[serde(default)]
    pub components: Vec<Polygon>,
}

impl PolygonSet {
    pub fn new(components: Vec<Polygon>) -> Self {
        Self { components }
    }

    pub fn from_outer(outer: Ring) -> Self {
        Self::new(vec![Polygon::new(outer)])
    }

    pub fn is_empty(&self) -> bool {
        self.components.is_empty()
    }

    pub fn ring_count(&self) -> usize {
        self.components
            .iter()
            .map(|p| 1usize.saturating_add(p.holes.len()))
            .sum()
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bounds {
    pub min: Point,
    pub max: Point,
}

impl Bounds {
    pub fn width(self) -> f64 {
        self.max.x - self.min.x
    }

    pub fn height(self) -> f64 {
        self.max.y - self.min.y
    }
}

/// Deterministic orientation applied to immutable source geometry.
///
/// Mirroring across the local Y axis is applied first, then rotation about the
/// local origin.  Arbitrary angles are quantized once to the kernel grid.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Orientation {
    pub angle_microdegrees: i64,
    pub mirror_x: bool,
}

impl Orientation {
    pub const IDENTITY: Self = Self {
        angle_microdegrees: 0,
        mirror_x: false,
    };

    pub fn from_degrees_exact(angle_degrees: i64) -> Self {
        // Orientation is periodic. Reducing before multiplication makes this
        // constructor total for every i64 without wrapping or saturation.
        Self {
            angle_microdegrees: angle_degrees.rem_euclid(360) * 1_000_000,
            mirror_x: false,
        }
    }

    pub fn from_degrees(angle_degrees: f64) -> NfpResult<Self> {
        if !angle_degrees.is_finite() {
            return Err(NfpError::InvalidOrientation {
                reason: "angle must be finite",
            });
        }
        let microdegrees = angle_degrees * 1_000_000.0;
        if !microdegrees.is_finite()
            || microdegrees < i64::MIN as f64
            || microdegrees > i64::MAX as f64
        {
            return Err(NfpError::InvalidOrientation {
                reason: "angle exceeds the i64 microdegree range",
            });
        }
        Ok(Self {
            angle_microdegrees: microdegrees.round() as i64,
            mirror_x: false,
        })
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Translation {
    pub x: f64,
    pub y: f64,
}

impl Translation {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Location {
    Outside,
    Boundary,
    Inside,
}

/// Exact material relation for one fixed/moving placement on the kernel grid.
///
/// This is intentionally separate from [`Location`]. Boolean-union topology
/// can turn a contact-only seam between multiple configuration obstacles into
/// the topological interior of the union, while the placed shapes still have
/// zero-area contact only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PairRelation {
    Disjoint,
    Contact,
    Overlap,
}
