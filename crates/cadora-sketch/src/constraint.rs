//! Sketch-level constraint data model.
//!
//! Each [`SketchConstraint`] represents a user-visible constraint that
//! references geometry via [`GeoElementId`]. During solve, these are
//! translated into one or more low-level GCS constraints.

use crate::types::{ConstraintType, GeoElementId};

/// A sketch-level constraint.
///
/// This is the user-facing constraint representation, equivalent
/// to FreeCAD's `Sketcher::Constraint` class.
#[derive(Debug, Clone)]
pub struct SketchConstraint {
    /// The constraint's semantic type.
    pub constraint_type: ConstraintType,
    /// Referenced geometry elements (0–3 elements depending on type).
    pub elements: Vec<GeoElementId>,
    /// Dimensional value (distance, angle, radius). `None` for purely geometric constraints.
    pub value: Option<f64>,
    /// Whether this constraint drives the solver (true) or is reference-only (false).
    pub is_driving: bool,
    /// Whether this constraint is currently active in the sketch.
    pub is_active: bool,
    /// Internal tag for tracking (assigned by the sketch manager).
    pub(crate) tag: i32,
}

impl SketchConstraint {
    /// Create a geometric constraint (no dimension value).
    pub fn geometric(
        constraint_type: ConstraintType,
        elements: Vec<GeoElementId>,
    ) -> Self {
        Self {
            constraint_type,
            elements,
            value: None,
            is_driving: true,
            is_active: true,
            tag: 0,
        }
    }

    /// Create a dimensional constraint with a value.
    pub fn dimensional(
        constraint_type: ConstraintType,
        elements: Vec<GeoElementId>,
        value: f64,
    ) -> Self {
        Self {
            constraint_type,
            elements,
            value: Some(value),
            is_driving: true,
            is_active: true,
            tag: 0,
        }
    }

    /// Returns true if this is a purely geometric constraint (no value).
    pub fn is_geometric(&self) -> bool {
        matches!(
            self.constraint_type,
            ConstraintType::Coincident
                | ConstraintType::Horizontal
                | ConstraintType::Vertical
                | ConstraintType::Parallel
                | ConstraintType::Perpendicular
                | ConstraintType::Tangent
                | ConstraintType::Equal
                | ConstraintType::PointOnObject
                | ConstraintType::Symmetric
                | ConstraintType::InternalAlignment
                | ConstraintType::Block
        )
    }

    /// Returns true if this is a dimensional constraint.
    pub fn is_dimensional(&self) -> bool {
        !self.is_geometric()
    }

    /// Access the first referenced element, if any.
    pub fn first(&self) -> Option<GeoElementId> {
        self.elements.first().copied()
    }

    /// Access the second referenced element, if any.
    pub fn second(&self) -> Option<GeoElementId> {
        self.elements.get(1).copied()
    }

    /// Access the third referenced element, if any.
    pub fn third(&self) -> Option<GeoElementId> {
        self.elements.get(2).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::PointPos;

    #[test]
    fn geometric_constraint_creation() {
        let c = SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![
                GeoElementId::start(0),
                GeoElementId::end(1),
            ],
        );
        assert_eq!(c.constraint_type, ConstraintType::Coincident);
        assert!(c.is_geometric());
        assert!(!c.is_dimensional());
        assert!(c.value.is_none());
        assert!(c.is_driving);
        assert!(c.is_active);
        assert_eq!(c.elements.len(), 2);
    }

    #[test]
    fn dimensional_constraint_creation() {
        let c = SketchConstraint::dimensional(
            ConstraintType::Distance,
            vec![
                GeoElementId::start(0),
                GeoElementId::start(1),
            ],
            42.0,
        );
        assert_eq!(c.constraint_type, ConstraintType::Distance);
        assert!(c.is_dimensional());
        assert!(!c.is_geometric());
        assert_eq!(c.value, Some(42.0));
    }

    #[test]
    fn element_accessors() {
        let e0 = GeoElementId::new(0, PointPos::Start);
        let e1 = GeoElementId::new(1, PointPos::End);
        let e2 = GeoElementId::new(2, PointPos::Mid);
        let c = SketchConstraint::geometric(
            ConstraintType::Symmetric,
            vec![e0, e1, e2],
        );
        assert_eq!(c.first(), Some(e0));
        assert_eq!(c.second(), Some(e1));
        assert_eq!(c.third(), Some(e2));
    }

    #[test]
    fn element_accessors_none_when_missing() {
        let c = SketchConstraint::geometric(
            ConstraintType::Block,
            vec![GeoElementId::edge(0)],
        );
        assert!(c.first().is_some());
        assert!(c.second().is_none());
        assert!(c.third().is_none());
    }

    #[test]
    fn all_geometric_types() {
        for ct in [
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
        ] {
            let c = SketchConstraint::geometric(ct, vec![]);
            assert!(c.is_geometric(), "{ct:?} should be geometric");
        }
    }

    #[test]
    fn all_dimensional_types() {
        for ct in [
            ConstraintType::Distance,
            ConstraintType::DistanceX,
            ConstraintType::DistanceY,
            ConstraintType::Angle,
            ConstraintType::Radius,
            ConstraintType::Diameter,
            ConstraintType::SnellsLaw,
            ConstraintType::Weight,
        ] {
            let c = SketchConstraint::dimensional(ct, vec![], 1.0);
            assert!(c.is_dimensional(), "{ct:?} should be dimensional");
        }
    }
}
