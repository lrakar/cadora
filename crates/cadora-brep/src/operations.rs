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

/// Default tolerance for boolean operations.
const BOOLEAN_TOLERANCE: f64 = 0.05;

/// Perform a boolean operation on two shapes.
///
/// Uses truck-shapeops for exact BRep boolean operations.
/// The tolerance controls the precision of surface intersection detection.
pub fn boolean(base: &Shape, tool: &Shape, op: BooleanOp) -> std::result::Result<Shape, BooleanError> {
    boolean_with_tolerance(base, tool, op, BOOLEAN_TOLERANCE)
}

/// Perform a boolean operation with a custom tolerance.
pub fn boolean_with_tolerance(
    base: &Shape,
    tool: &Shape,
    op: BooleanOp,
    tolerance: f64,
) -> std::result::Result<Shape, BooleanError> {
    let result = match op {
        BooleanOp::Fuse => {
            truck_shapeops::or(base.solid(), tool.solid(), tolerance)
        }
        BooleanOp::Cut => {
            let mut inverted_tool = tool.solid().clone();
            inverted_tool.not();
            truck_shapeops::and(base.solid(), &inverted_tool, tolerance)
        }
        BooleanOp::Common => {
            truck_shapeops::and(base.solid(), tool.solid(), tolerance)
        }
    };
    result
        .map(Shape::from_solid)
        .ok_or_else(|| BooleanError::OperationFailed("Boolean operation returned None".into()))
}

/// Errors from boolean operations.
#[derive(Debug, Clone)]
pub enum BooleanError {
    /// The operation failed (e.g., degenerate input, tolerance issues).
    OperationFailed(String),
}

impl std::fmt::Display for BooleanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BooleanError::OperationFailed(msg) => {
                write!(f, "Boolean operation failed: {msg}")
            }
        }
    }
}

impl std::error::Error for BooleanError {}

/// Errors from fillet/chamfer (dress-up) operations.
#[derive(Debug, Clone)]
pub enum DressUpError {
    /// The specified edge index is out of range or invalid.
    InvalidEdge(String),
    /// The operation failed geometrically.
    OperationFailed(String),
}

impl std::fmt::Display for DressUpError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DressUpError::InvalidEdge(msg) => write!(f, "Invalid edge: {msg}"),
            DressUpError::OperationFailed(msg) => write!(f, "Dress-up failed: {msg}"),
        }
    }
}

impl std::error::Error for DressUpError {}

/// Chamfer edges of a shape by cutting a flat bevel.
///
/// Each edge is identified by index (in shape traversal order).
/// Works on straight edges between planar faces.
pub fn chamfer(shape: &Shape, edge_indices: &[usize], distance: f64) -> std::result::Result<Shape, DressUpError> {
    let mut result = shape.clone();
    for &idx in edge_indices {
        result = chamfer_single_edge(&result, idx, distance)?;
    }
    Ok(result)
}

/// Fillet edges of a shape with a rounded blend.
///
/// Each edge is identified by index (in shape traversal order).
/// Works on straight edges between planar faces meeting at ~90°.
pub fn fillet(shape: &Shape, edge_indices: &[usize], radius: f64) -> std::result::Result<Shape, DressUpError> {
    let mut result = shape.clone();
    for &idx in edge_indices {
        result = fillet_single_edge(&result, idx, radius)?;
    }
    Ok(result)
}

/// Info about an edge and which face it belongs to.
struct EdgeFaceInfo {
    front: Point3,
    back: Point3,
    face_idx: usize,
    global_edge_idx: usize,
}

/// Collect all edges with face membership info.
fn collect_edges(shape: &Shape) -> Vec<EdgeFaceInfo> {
    let mut result = Vec::new();
    let mut global_idx = 0;
    for shell in shape.solid().boundaries() {
        for (face_idx, face) in shell.face_iter().enumerate() {
            for wire in face.boundaries() {
                for edge in wire.edge_iter() {
                    result.push(EdgeFaceInfo {
                        front: edge.front().point(),
                        back: edge.back().point(),
                        face_idx,
                        global_edge_idx: global_idx,
                    });
                    global_idx += 1;
                }
            }
        }
    }
    result
}

/// Find the two face normals adjacent to an edge.
fn find_edge_adjacent_normals(shape: &Shape, edge_idx: usize) -> std::result::Result<(Point3, Point3, Vector3, Vector3), DressUpError> {
    let edges = collect_edges(shape);
    let target = edges.iter().find(|e| e.global_edge_idx == edge_idx)
        .ok_or_else(|| DressUpError::InvalidEdge(format!("Edge index {edge_idx} out of range")))?;

    let target_front = target.front;
    let target_back = target.back;
    let target_face = target.face_idx;
    let eps = 1e-8;

    // Find the other face that shares this edge (same endpoints)
    let other_face = edges.iter()
        .filter(|e| e.face_idx != target_face)
        .find(|e| {
            let fwd = (e.front - target_front).magnitude() < eps && (e.back - target_back).magnitude() < eps;
            let rev = (e.front - target_back).magnitude() < eps && (e.back - target_front).magnitude() < eps;
            fwd || rev
        })
        .ok_or_else(|| DressUpError::InvalidEdge("No adjacent face found".into()))?;

    // Get face normals via the surface
    let shell = &shape.solid().boundaries()[0];
    let faces: Vec<&Face> = shell.face_iter().collect();
    let n1 = face_outward_normal(faces[target_face]);
    let n2 = face_outward_normal(faces[other_face.face_idx]);

    Ok((target_front, target_back, n1, n2))
}

/// Get the outward normal of a face.
fn face_outward_normal(face: &Face) -> Vector3 {
    let surface = face.oriented_surface();
    surface.normal(0.5, 0.5)
}

/// Chamfer a single edge by constructing and cutting a triangular prism.
///
/// The cutting prism is built with all vertices positioned OUTSIDE the original
/// solid, with a generous margin. This avoids degenerate intersections in
/// truck-shapeops' face-division algorithm that occur when cutting-tool vertices
/// lie exactly on the target solid's faces.
fn chamfer_single_edge(shape: &Shape, edge_idx: usize, distance: f64) -> std::result::Result<Shape, DressUpError> {
    let (start, end, n1, n2) = find_edge_adjacent_normals(shape, edge_idx)?;

    let edge_vec = end - start;
    let edge_len = edge_vec.magnitude();
    let edge_dir = edge_vec / edge_len;

    // Extend the prism well beyond the edge ends
    let ext = distance * 0.5;
    let start_ext = start - edge_dir * ext;
    let extrude_dir = edge_dir * (edge_len + 2.0 * ext);

    // Generous margin: all 3 profile vertices are placed outside the solid.
    // n1, n2 are outward face normals. Moving along n1/n2 goes outside;
    // moving along -n1/-n2 goes inside the solid.
    let margin = distance * 0.3;

    // p0: corner vertex — pushed outward along both normals (outside the corner)
    let p0 = start_ext + n1 * margin + n2 * margin;
    // p1: chamfer vertex on face-1 side — pushed inward past face-1 by (d + margin),
    //     but kept outside face-2 by margin
    let p1 = start_ext - n1 * (distance + margin) + n2 * margin;
    // p2: chamfer vertex on face-2 side — pushed inward past face-2 by (d + margin),
    //     but kept outside face-1 by margin
    let p2 = start_ext - n2 * (distance + margin) + n1 * margin;

    let v0 = builder::vertex(p0);
    let v1 = builder::vertex(p1);
    let v2 = builder::vertex(p2);

    let wire = Wire::from(vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v0),
    ]);

    let face = builder::try_attach_plane(&[wire])
        .map_err(|e| DressUpError::OperationFailed(format!("Cannot create chamfer profile: {e}")))?;

    let cutting_solid = builder::tsweep(&face, extrude_dir);
    let tool = Shape::from_solid(cutting_solid);

    boolean(shape, &tool, BooleanOp::Cut)
        .map_err(|e| DressUpError::OperationFailed(e.to_string()))
}

/// Fillet a single edge by constructing and cutting a profiled shape with a
/// circular-arc face.
///
/// Same margin strategy as chamfer: all profile vertices are outside the solid
/// to avoid degenerate boolean intersections.
fn fillet_single_edge(shape: &Shape, edge_idx: usize, radius: f64) -> std::result::Result<Shape, DressUpError> {
    let (start, end, n1, n2) = find_edge_adjacent_normals(shape, edge_idx)?;

    let edge_vec = end - start;
    let edge_len = edge_vec.magnitude();
    let edge_dir = edge_vec / edge_len;

    let ext = radius * 0.5;
    let start_ext = start - edge_dir * ext;
    let extrude_dir = edge_dir * (edge_len + 2.0 * ext);

    let margin = radius * 0.3;

    // Same vertex placement as chamfer (all outside the solid)
    let p0 = start_ext + n1 * margin + n2 * margin;
    let p1 = start_ext - n1 * (radius + margin) + n2 * margin;
    let p2 = start_ext - n2 * (radius + margin) + n1 * margin;

    // Fillet arc center (offset from edge by radius in each inward direction)
    use cgmath::InnerSpace;
    let arc_center = start_ext - n1 * radius - n2 * radius;
    // Transit point at the midpoint of the ideal fillet quarter-circle
    let mid_dir = (n1 + n2).normalize();
    let transit = arc_center + mid_dir * radius;

    let v0 = builder::vertex(p0);
    let v1 = builder::vertex(p1);
    let v2 = builder::vertex(p2);

    let wire = Wire::from(vec![
        builder::line(&v0, &v1),
        builder::circle_arc(&v1, &v2, transit),
        builder::line(&v2, &v0),
    ]);

    let face = builder::try_attach_plane(&[wire])
        .map_err(|_| DressUpError::OperationFailed("Cannot create fillet profile".into()))?;

    let cutting_solid = builder::tsweep(&face, extrude_dir);
    let tool = Shape::from_solid(cutting_solid);

    boolean(shape, &tool, BooleanOp::Cut)
        .map_err(|e| DressUpError::OperationFailed(e.to_string()))
}

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

    /// Helper: create a box solid using the tsweep chain pattern (vertex→edge→face→solid)
    fn make_test_box(origin: Point3, dx: f64, dy: f64, dz: f64) -> Solid {
        let v = builder::vertex(origin);
        let e = builder::tsweep(&v, Vector3::new(dx, 0.0, 0.0));
        let f = builder::tsweep(&e, Vector3::new(0.0, dy, 0.0));
        builder::tsweep(&f, Vector3::new(0.0, 0.0, dz))
    }

    #[test]
    fn boolean_fuse_two_boxes() {
        // Two overlapping boxes: no coplanar faces
        let s1 = Shape::from_solid(make_test_box(Point3::origin(), 2.0, 2.0, 2.0));
        let s2 = Shape::from_solid(make_test_box(Point3::new(0.5, 0.5, 0.5), 2.0, 2.0, 2.0));
        let fused = boolean(&s1, &s2, BooleanOp::Fuse).expect("Fuse should succeed");
        let bb = fused.bounding_box();
        assert_relative_eq!(bb.min.x, 0.0, epsilon = 0.2);
        assert_relative_eq!(bb.max.x, 2.5, epsilon = 0.2);
        assert_relative_eq!(bb.min.y, 0.0, epsilon = 0.2);
        assert_relative_eq!(bb.max.y, 2.5, epsilon = 0.2);
    }

    #[test]
    fn boolean_cut_box() {
        // Cut a cylinder hole from a cube (same as truck-shapeops example)
        let cube = make_test_box(Point3::origin(), 1.0, 1.0, 1.0);
        let v = builder::vertex(Point3::new(0.5, 0.25, -0.5));
        let w = builder::rsweep(&v, Point3::new(0.5, 0.5, 0.0), Vector3::unit_z(), Rad(7.0));
        let f = builder::try_attach_plane(&[w]).unwrap();
        let mut cylinder = builder::tsweep(&f, Vector3::unit_z() * 2.0);
        cylinder.not();
        let base = Shape::from_solid(cube);
        let tool = Shape::from_solid(cylinder);
        let result = boolean(&base, &tool, BooleanOp::Common).expect("Cut should succeed");
        let bb = result.bounding_box();
        assert_relative_eq!(bb.min.x, 0.0, epsilon = 0.2);
        assert_relative_eq!(bb.max.x, 1.0, epsilon = 0.2);
        assert!(result.face_count() > 6); // More faces than original cube
    }

    #[test]
    fn boolean_common_two_boxes() {
        // Common of two overlapping boxes: no coplanar faces
        let s1 = Shape::from_solid(make_test_box(Point3::origin(), 2.0, 2.0, 2.0));
        let s2 = Shape::from_solid(make_test_box(Point3::new(0.5, 0.5, 0.5), 2.0, 2.0, 2.0));
        let common = boolean(&s1, &s2, BooleanOp::Common).expect("Common should succeed");
        let bb = common.bounding_box();
        // The common volume should be (0.5,0.5,0.5)-(2.0,2.0,2.0) = 1.5×1.5×1.5
        assert_relative_eq!(bb.min.x, 0.5, epsilon = 0.2);
        assert_relative_eq!(bb.max.x, 2.0, epsilon = 0.2);
        assert_relative_eq!(bb.min.y, 0.5, epsilon = 0.2);
        assert_relative_eq!(bb.max.y, 2.0, epsilon = 0.2);
    }

    #[test]
    fn boolean_with_custom_tolerance() {
        let s1 = Shape::from_solid(make_test_box(Point3::origin(), 2.0, 2.0, 2.0));
        let s2 = Shape::from_solid(make_test_box(Point3::new(0.5, 0.5, 0.5), 2.0, 2.0, 2.0));
        let result = boolean_with_tolerance(&s1, &s2, BooleanOp::Fuse, 0.01);
        assert!(result.is_ok());
    }

    #[test]
    fn chamfer_box_edge() {
        let cube = Shape::from_solid(make_test_box(Point3::origin(), 2.0, 2.0, 2.0));
        let original_faces = cube.face_count();
        let result = chamfer(&cube, &[0], 0.3);
        assert!(result.is_ok(), "Chamfer should succeed: {:?}", result.err());
        let chamfered = result.unwrap();
        assert!(chamfered.face_count() > original_faces);
    }

    #[test]
    fn chamfer_preserves_overall_size() {
        let cube = Shape::from_solid(make_test_box(Point3::origin(), 4.0, 4.0, 4.0));
        let result = chamfer(&cube, &[0], 0.5);
        assert!(result.is_ok());
        let bb = result.unwrap().bounding_box();
        // Overall bounding box should be approximately the same
        assert_relative_eq!(bb.max.x, 4.0, epsilon = 0.3);
        assert_relative_eq!(bb.max.y, 4.0, epsilon = 0.3);
        assert_relative_eq!(bb.max.z, 4.0, epsilon = 0.3);
    }

    #[test]
    fn fillet_box_edge() {
        let cube = Shape::from_solid(make_test_box(Point3::origin(), 2.0, 2.0, 2.0));
        let original_faces = cube.face_count();
        let result = fillet(&cube, &[0], 0.3);
        assert!(result.is_ok(), "Fillet should succeed: {:?}", result.err());
        let filleted = result.unwrap();
        // Fillet adds a new curved face
        assert!(filleted.face_count() > original_faces);
    }

    #[test]
    fn chamfer_invalid_edge() {
        let cube = Shape::from_solid(make_test_box(Point3::origin(), 2.0, 2.0, 2.0));
        let result = chamfer(&cube, &[999], 0.3);
        assert!(result.is_err());
    }

    #[test]
    fn dressup_error_display() {
        let e1 = DressUpError::InvalidEdge("test".into());
        assert!(format!("{e1}").contains("Invalid edge"));
        let e2 = DressUpError::OperationFailed("fail".into());
        assert!(format!("{e2}").contains("Dress-up failed"));
    }
}
