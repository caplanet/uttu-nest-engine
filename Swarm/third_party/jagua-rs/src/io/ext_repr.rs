use crate::geometry::DTransformation;
use serde::{Deserialize, Serialize};

/// External representation of an [`Item`](crate::entities::Item).
#[derive(Serialize, Clone)]
pub struct ExtItem {
    /// Unique external identifier of the item.
    pub id: u64,
    /// Required orientation permissions in original item coordinates.
    pub orientation: ExtOrientation,
    /// Shape of the item. Polygons with holes and multipolygons are not supported.
    pub shape: ExtShape,
    /// Minimum required quality in `0..N_QUALITIES`, validated at import.
    /// Missing or null requires full quality. Holes always block placement.
    pub min_quality: Option<usize>,
}

/// Orientation permissions in degrees. Both this object and its rotation field are required.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ExtOrientation {
    /// Permitted rotations, applied after optional reflection.
    pub rotation: ExtRotation,
    /// Optional reflection axes through the original item's local origin,
    /// in degrees counterclockwise from its positive x-axis, before rotation.
    /// 0° is the horizontal x-axis: `(x, y)` becomes `(x, -y)`.
    /// 90° is the vertical y-axis: `(x, y)` becomes `(-x, y)`.
    /// Missing or empty disables reflection.
    /// Choose no reflection or one axis; equivalent axes are normalized modulo 180°.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub reflection_axes: Vec<f32>,
}

/// Explicit rotation modes, validated at import.
/// Lists and stepped expansions are limited to 65,536 angles.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum ExtRotation {
    /// A nonempty list of finite angles in degrees. Zero is not implicit.
    /// Equivalent angles are normalized modulo 360 and deduplicated.
    Discrete { angles: Vec<f32> },
    /// Zero-based increments covering one turn, excluding the duplicate endpoint.
    /// The finite step must be in (0, 360]. In f64 arithmetic, round `360 / step`
    /// to a count and require `abs(count * step - 360) <= 360 * f32::EPSILON`.
    /// Expand by index as `i * 360 / count`, avoiding accumulated rounding error.
    Stepped { step: f32 },
    /// Any rotation angle. No null or empty-list sentinel is used.
    Continuous {},
}

impl<'de> Deserialize<'de> for ExtItem {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        // CAPLANET: the 0.8.x `allowed_orientations` field (degrees; absent = continuous,
        // `[]` or `[0]` = fixed) is still accepted and converted, because existing callers
        // and fixtures send it. `orientation` wins when both are present. Upstream rejects
        // the legacy field. Unrelated metadata such as source filenames is still allowed.
        #[derive(Deserialize)]
        struct Input {
            id: u64,
            #[serde(default)]
            orientation: Option<ExtOrientation>,
            shape: ExtShape,
            min_quality: Option<usize>,
            #[serde(
                default,
                rename = "allowed_orientations",
                alias = "allowed_rotations",
                deserialize_with = "legacy_orientations"
            )]
            legacy: Option<Option<Vec<f32>>>,
        }
        fn legacy_orientations<'de, D: serde::Deserializer<'de>>(
            d: D,
        ) -> Result<Option<Option<Vec<f32>>>, D::Error> {
            Option::<Vec<f32>>::deserialize(d).map(Some)
        }
        let Input {
            id,
            orientation,
            shape,
            min_quality,
            legacy,
        } = Input::deserialize(deserializer)?;
        let orientation = match (orientation, legacy) {
            (Some(orientation), _) => orientation,
            (None, Some(legacy)) => ExtOrientation::from_legacy(legacy.as_deref()),
            // Neither field: continuous rotation, as 0.8.x read an absent list.
            (None, None) => ExtOrientation::from_legacy(None),
        };
        Ok(Self {
            id,
            orientation,
            shape,
            min_quality,
        })
    }
}

impl ExtOrientation {
    /// CAPLANET: converts the 0.8.x `allowed_orientations` list (degrees).
    /// `None` is continuous rotation; an empty list or `[0]` is a fixed orientation.
    #[must_use]
    pub fn from_legacy(allowed_orientations: Option<&[f32]>) -> Self {
        let rotation = match allowed_orientations {
            None => ExtRotation::Continuous {},
            Some(angles) if angles.is_empty() => ExtRotation::Discrete { angles: vec![0.0] },
            Some(angles) => ExtRotation::Discrete {
                angles: angles.to_vec(),
            },
        };
        ExtOrientation {
            rotation,
            reflection_axes: vec![],
        }
    }
}

#[cfg(test)]
mod legacy_orientation_tests {
    use super::*;

    fn item(json: &str) -> ExtItem {
        serde_json::from_str(json).unwrap()
    }
    const SHAPE: &str = r#""shape":{"type":"rectangle","data":{"x_min":0,"y_min":0,"width":2,"height":1}}"#;

    #[test]
    fn legacy_list_becomes_discrete() {
        let it = item(&format!(r#"{{"id":1,"allowed_orientations":[0,90],{SHAPE}}}"#));
        assert_eq!(it.orientation.rotation, ExtRotation::Discrete { angles: vec![0.0, 90.0] });
    }

    #[test]
    fn legacy_empty_and_zero_are_fixed() {
        for list in ["[]", "[0]"] {
            let it = item(&format!(r#"{{"id":1,"allowed_orientations":{list},{SHAPE}}}"#));
            assert_eq!(it.orientation.rotation, ExtRotation::Discrete { angles: vec![0.0] });
        }
    }

    #[test]
    fn legacy_null_or_absent_is_continuous() {
        for json in [
            format!(r#"{{"id":1,"allowed_orientations":null,{SHAPE}}}"#),
            format!(r#"{{"id":1,{SHAPE},"source":"a.dxf"}}"#),
        ] {
            assert_eq!(item(&json).orientation.rotation, ExtRotation::Continuous {});
        }
    }

    #[test]
    fn explicit_orientation_wins() {
        let it = item(&format!(
            r#"{{"id":1,"allowed_orientations":[0,90],"orientation":{{"rotation":{{"mode":"stepped","step":180}}}},{SHAPE}}}"#
        ));
        assert_eq!(it.orientation.rotation, ExtRotation::Stepped { step: 180.0 });
    }
}

/// External representation of a [`Container`](crate::entities::Container).
/// Items can be placed inside containers.
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtContainer {
    /// Unique identifier of the container
    pub id: u64,
    /// Shape of the container
    pub shape: ExtShape,
    /// Zones within the container with varying quality. Holes in the container shape are treated as zones with quality 0.
    #[serde(skip_serializing_if = "Vec::is_empty", default)]
    pub zones: Vec<ExtQualityZone>,
}

/// Various ways to represent a shape
#[derive(Serialize, Deserialize, Clone)]
#[serde(tag = "type", content = "data")]
#[serde(rename_all = "snake_case")]
pub enum ExtShape {
    /// Axis-aligned rectangle. With its left bottom corner at (`x_min`, `y_min`), a width and height
    Rectangle {
        x_min: f32,
        y_min: f32,
        width: f32,
        height: f32,
    },
    /// Polygon with a single outer boundary
    SimplePolygon(ExtSPolygon),
    /// Polygon with a single outer boundary and a set of holes
    Polygon(ExtPolygon),
    /// Multiple disjoint polygons
    MultiPolygon(Vec<ExtPolygon>),
}

/// A polygon represented as an outer boundary and a list of holes
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtPolygon {
    /// The outer boundary of the polygon
    pub outer: ExtSPolygon,
    /// A list of holes in the polygon
    #[serde(default)]
    pub inner: Vec<ExtSPolygon>,
}

/// External representation of a [`SPolygon`](crate::geometry::primitives::SPolygon).
/// A polygon with no holes and no self-intersections.
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtSPolygon(pub Vec<(f32, f32)>);

/// A zone with a specific quality level
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtQualityZone {
    /// The quality level of this zone
    pub quality: usize,
    /// The polygon shape of this zone
    pub shape: ExtShape,
}

/// External representation of a [`Layout`](crate::entities::Layout).
/// A layout consists of a container with items placed in a specific configuration.
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtLayout {
    /// The container that was used
    pub container_id: u64,
    /// The items placed in the container and where they were placed
    pub placed_items: Vec<ExtPlacedItem>,
    /// Some statistics about the layout
    pub density: f32,
}

/// External representation of a [`PlacedItem`](crate::entities::PlacedItem).
/// An item placed in a container with a specific transformation.
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtPlacedItem {
    /// The id of the item in the instance
    pub item_id: u64,
    /// The transformation applied to the item to place it in the container
    pub transformation: ExtTransformation,
}

/// Optional x-axis reflection, followed by rotation and translation.
/// The canonical x-axis reflection is independent of the item's permitted input axes:
/// reflecting across axis `a` then rotating by `r` is exported with rotation `r + 2*a`.
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtTransformation {
    /// Negate the original local y coordinate before rotation. Defaults to false.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub reflected: bool,
    /// The rotation angle in degrees
    pub rotation: f32,
    /// The translation vector (x, y)
    pub translation: (f32, f32),
}

impl From<DTransformation> for ExtTransformation {
    fn from(dt: DTransformation) -> Self {
        ExtTransformation {
            reflected: dt.reflected,
            rotation: dt.rotation().to_degrees(),
            translation: dt.translation(),
        }
    }
}

impl From<ExtTransformation> for DTransformation {
    fn from(ext_dt: ExtTransformation) -> Self {
        DTransformation::new(ext_dt.rotation.to_radians(), ext_dt.translation)
            .with_reflection(ext_dt.reflected)
    }
}
