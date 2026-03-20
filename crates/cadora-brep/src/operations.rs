//! Shape operations: extrusion, revolution, and boolean operations.

use truck_modeling::*;
use crate::shape::Shape;

/// Extrude a planar wire profile along a direction vector to create a solid.
///
/// The wire must be closed and planar. The resulting shape is a prism.
pub fn extrude(wire: &Wire, direction: Vector3) -> Shape {
    // Create a face from the wire
    let face = builder::try_attach_plane(&[wire.clone()]).expect("Wire must be planar and closed");
    extrude_face(&face, direction)
}

/// Extrude a face along a direction vector to create a solid.
pub fn extrude_face(face: &Face, direction: Vector3) -> Shape {
    let solid = builder::tsweep(face, direction);
    Shape::from_solid(solid)
}

/// Extrude a wire profile by a distance along the Z axis.
pub fn extrude_z(wire: &Wire, height: f64) -> Shape {
    extrude(wire, Vector3::new(0.0, 0.0, height))
}

/// Revolve a wire profile around an axis to create a solid.
///
/// `origin` is a point on the axis, `axis` is the direction of the axis,
/// and `angle` is the revolution angle in radians (use TAU for full revolution).
pub fn revolve(wire: &Wire, origin: Point3, axis: Vector3, angle: f64) -> Shape {
    let shell: Shell = builder::rsweep(wire, origin, axis, Rad(angle));
    let solid = Solid::new(vec![shell]);
    Shape::from_solid(solid)
}

/// Revolve a wire profile around the Z axis through the origin.
pub fn revolve_z(wire: &Wire, angle: f64) -> Shape {
    revolve(wire, Point3::origin(), Vector3::new(0.0, 0.0, 1.0), angle)
}

/// Boolean operations on shapes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    /// Union: combine two shapes
    Fuse,
    /// Subtraction: cut tool from base
    Cut,
    /// Intersection: keep only the common volume
    Common,
}

// Note: truck-shapeops (0.4) is not compatible with truck-modeling (0.6).
// Boolean operations will be added when the truck ecosystem versions align,
// or when we add OCCT FFI bindings as an alternative backend.
// For now, we provide the API surface and a placeholder.

/// Perform a boolean operation on two shapes.
///
/// **Note:** Boolean operations require a compatible shape operations backend.
/// Currently returns an error until truck-shapeops is compatible with truck 0.6
/// or OCCT bindings are added.
pub fn boolean(_base: &Shape, _tool: &Shape, _op: BooleanOp) -> std::result::Result<Shape, BooleanError> {
    Err(BooleanError::NotYetImplemented)
}

/// Errors from boolean operations.
#[derive(Debug, Clone)]
pub enum BooleanError {
    /// Boolean operations are not yet implemented with the current backend.
    NotYetImplemented,
    /// The operation failed (e.g., degenerate input).
    OperationFailed(String),
}

impl std::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BooleanError::NotYetImplemented => {
                write!(f, "Boolean operations not yet implemented")
            }
            BooleanError::OperationFailed(msg) => {
                write!(f, "Boolean operation failed: {msg}")
            }
        }
    }
}

impl std::error::Error for BooleanError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::*;
    use approx::assert_relative_eq;

    #[test]
    fn extrude_rectangle() {
        let wire = make_rect_wire(10.0, 5.0);
        let shape = extrude_z(&wire, 3.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.x, 0.0, epsilon = 0.1);
        assert_relative_eq!(bb.max.x, 10.0, epsilon = 0.1);
        assert_relative_eq!(bb.min.y, 0.0, epsilon = 0.1);
        assert_relative_eq!(bb.max.y, 5.0, epsilon = 0.1);
        assert_relative_eq!(bb.min.z, 0.0, epsilon = 0.1);
        assert_relative_eq!(bb.max.z, 3.0, epsilon = 0.1);
    }

    #[test]
    fn extrude_circle() {
        let wire = make_circle_wire(5.0);
        let shape = extrude_z(&wire, 10.0);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.z, 0.0, epsilon = 0.5);
        assert_relative_eq!(bb.max.z, 10.0, epsilon = 0.5);
    }

    #[test]
    fn revolve_rectangle_full() {
        // Revolve a rect profile in the XZ plane around Z to make a cylinder-like shape
        let p0 = Point3::new(3.0, 0.0, 0.0);
        let p1 = Point3::new(5.0, 0.0, 0.0);
        let p2 = Point3::new(5.0, 0.0, 10.0);
        let p3 = Point3::new(3.0, 0.0, 10.0);

        let v0 = builder::vertex(p0);
        let v1 = builder::vertex(p1);
        let v2 = builder::vertex(p2);
        let v3 = builder::vertex(p3);

        let wire = Wire::from(vec![
            builder::line(&v0, &v1),
            builder::line(&v1, &v2),
            builder::line(&v2, &v3),
            builder::line(&v3, &v0),
        ]);

        let shape = revolve_z(&wire, std::f64::consts::TAU);
        let bb = shape.bounding_box();
        assert_relative_eq!(bb.min.z, 0.0, epsilon = 0.5);
        assert_relative_eq!(bb.max.z, 10.0, epsilon = 0.5);
    }

    #[test]
    fn boolean_not_yet_implemented() {
        let s1 = crate::make_box(1.0, 1.0, 1.0);
        let s2 = crate::make_box(1.0, 1.0, 1.0);
        let result = boolean(&s1, &s2, BooleanOp::Fuse);
        assert!(result.is_err());
    }
}
