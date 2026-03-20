//! Core identification types for sketch elements.
//!
//! Mirrors FreeCAD's GeoId / PointPos / GeoElementId system.

/// Geometry identifier within a sketch.
///
/// - `>= 0`: user-created geometry (index into geometry list)
/// - `-1`:  horizontal axis (built-in)
/// - `-2`:  vertical axis (built-in)
/// - `<= -3`: external geometry references (mapped as `-(index + 3)`)
pub type GeoId = i32;

/// Special GeoId constants for the built-in reference geometry.
pub const GEO_ID_H_AXIS: GeoId = -1;
pub const GEO_ID_V_AXIS: GeoId = -2;
/// The origin point shares the H-axis GeoId with `PointPos::Start`.
pub const GEO_ID_ROOT_POINT: GeoId = GEO_ID_H_AXIS;

/// Which sub-element of a geometry is referenced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PointPos {
    /// The edge/curve itself (no specific point).
    None,
    /// Start point of a line or arc.
    Start,
    /// End point of a line or arc.
    End,
    /// Center/midpoint (circle center, arc center, ellipse center).
    Mid,
}

/// Unambiguous reference to a geometric element within a sketch.
///
/// Combines a [`GeoId`] with a [`PointPos`] to identify exactly which
/// sub-element (edge, start, end, center) is being referenced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GeoElementId {
    pub geo_id: GeoId,
    pub pos: PointPos,
}

impl GeoElementId {
    pub fn new(geo_id: GeoId, pos: PointPos) -> Self {
        Self { geo_id, pos }
    }

    pub fn edge(geo_id: GeoId) -> Self {
        Self { geo_id, pos: PointPos::None }
    }

    pub fn start(geo_id: GeoId) -> Self {
        Self { geo_id, pos: PointPos::Start }
    }

    pub fn end(geo_id: GeoId) -> Self {
        Self { geo_id, pos: PointPos::End }
    }

    pub fn mid(geo_id: GeoId) -> Self {
        Self { geo_id, pos: PointPos::Mid }
    }

    /// True if this references an external geometry element.
    pub fn is_external(&self) -> bool {
        self.geo_id <= -3
    }

    /// True if this references one of the built-in axes or origin.
    pub fn is_axis(&self) -> bool {
        self.geo_id == GEO_ID_H_AXIS || self.geo_id == GEO_ID_V_AXIS
    }
}

/// High-level constraint types exposed to the user.
///
/// Each maps to one or more low-level GCS constraints internally.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConstraintType {
    // -- Geometric --
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Tangent,
    Equal,
    PointOnObject,
    Symmetric,
    InternalAlignment,
    Block,

    // -- Dimensional --
    Distance,
    DistanceX,
    DistanceY,
    Angle,
    Radius,
    Diameter,
    SnellsLaw,
    Weight,
}

/// Flags that modify how a geometry element behaves.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GeometryMode {
    /// Construction geometry — visible but not used for extrusion profiles.
    pub construction: bool,
    /// Blocked geometry — all parameters are fixed (like a Block constraint).
    pub blocked: bool,
}

impl Default for GeometryMode {
    fn default() -> Self {
        Self {
            construction: false,
            blocked: false,
        }
    }
}

/// Classification of internal alignment geometry.
///
/// When a complex curve (ellipse, hyperbola, B-spline) is created,
/// helper geometry is generated to represent foci, axes, knots, etc.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InternalType {
    None,
    EllipseMajorDiameter,
    EllipseMinorDiameter,
    EllipseFocus1,
    EllipseFocus2,
    HyperbolaMajor,
    HyperbolaMinor,
    HyperbolaFocus,
    ParabolaFocalAxis,
    ParabolaFocus,
    BSplineControlPoint,
    BSplineKnotPoint,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geo_element_id_constructors() {
        let e = GeoElementId::edge(0);
        assert_eq!(e.geo_id, 0);
        assert_eq!(e.pos, PointPos::None);

        let s = GeoElementId::start(2);
        assert_eq!(s.pos, PointPos::Start);

        let m = GeoElementId::mid(-1);
        assert_eq!(m.geo_id, GEO_ID_H_AXIS);
        assert_eq!(m.pos, PointPos::Mid);
    }

    #[test]
    fn is_external_and_axis() {
        assert!(!GeoElementId::edge(0).is_external());
        assert!(!GeoElementId::edge(-1).is_external());
        assert!(!GeoElementId::edge(-2).is_external());
        assert!(GeoElementId::edge(-3).is_external());
        assert!(GeoElementId::edge(-100).is_external());

        assert!(GeoElementId::edge(-1).is_axis());
        assert!(GeoElementId::edge(-2).is_axis());
        assert!(!GeoElementId::edge(0).is_axis());
        assert!(!GeoElementId::edge(-3).is_axis());
    }

    #[test]
    fn geo_id_constants() {
        assert_eq!(GEO_ID_H_AXIS, -1);
        assert_eq!(GEO_ID_V_AXIS, -2);
        assert_eq!(GEO_ID_ROOT_POINT, GEO_ID_H_AXIS);
    }

    #[test]
    fn geometry_mode_defaults() {
        let m = GeometryMode::default();
        assert!(!m.construction);
        assert!(!m.blocked);
    }

    #[test]
    fn constraint_type_enum_values() {
        // Ensure all variants are distinct
        let types = [
            ConstraintType::Coincident,
            ConstraintType::Horizontal,
            ConstraintType::Vertical,
            ConstraintType::Parallel,
            ConstraintType::Perpendicular,
            ConstraintType::Tangent,
            ConstraintType::Equal,
            ConstraintType::PointOnObject,
            ConstraintType::Symmetric,
            ConstraintType::InternalAlignment,
            ConstraintType::Block,
            ConstraintType::Distance,
            ConstraintType::DistanceX,
            ConstraintType::DistanceY,
            ConstraintType::Angle,
            ConstraintType::Radius,
            ConstraintType::Diameter,
            ConstraintType::SnellsLaw,
            ConstraintType::Weight,
        ];
        assert_eq!(types.len(), 19);
        // No duplicates
        for (i, a) in types.iter().enumerate() {
            for b in types.iter().skip(i + 1) {
                assert_ne!(a, b);
            }
        }
    }

    #[test]
    fn point_pos_equality() {
        assert_eq!(PointPos::Start, PointPos::Start);
        assert_ne!(PointPos::Start, PointPos::End);
        assert_ne!(PointPos::None, PointPos::Mid);
    }

    #[test]
    fn internal_type_variants() {
        assert_ne!(InternalType::None, InternalType::EllipseFocus1);
        assert_eq!(InternalType::BSplineKnotPoint, InternalType::BSplineKnotPoint);
    }
}
