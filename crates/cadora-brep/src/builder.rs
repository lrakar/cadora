//! Wire builder for constructing profiles from edges.

use truck_modeling::*;

/// Builds a wire from a sequence of edges.
///
/// Used to construct profiles (e.g., from sketch geometry) that can then
/// be extruded, revolved, etc.
pub struct WireBuilder {
    edges: Vec<Edge>,
}

impl WireBuilder {
    pub fn new() -> Self {
        Self { edges: Vec::new() }
    }

    /// Add a line segment from p0 to p1.
    pub fn line_to(&mut self, from: Point3, to: Point3) -> &mut Self {
        let v0 = builder::vertex(from);
        let v1 = builder::vertex(to);
        self.edges.push(builder::line(&v0, &v1));
        self
    }

    /// Add a circular arc from `from` to `to` passing near `transit`.
    pub fn arc_to(&mut self, from: Point3, to: Point3, transit: Point3) -> &mut Self {
        let v0 = builder::vertex(from);
        let v1 = builder::vertex(to);
        self.edges.push(builder::circle_arc(&v0, &v1, transit));
        self
    }

    /// Build the wire from accumulated edges.
    pub fn build(self) -> Wire {
        Wire::from(self.edges)
    }

    /// Build the wire and create a planar face from it.
    pub fn build_face(self) -> Option<Face> {
        let wire = self.build();
        builder::try_attach_plane(&[wire]).ok()
    }
}

impl Default for WireBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Create a rectangular wire in the XY plane.
pub fn make_rect_wire(width: f64, height: f64) -> Wire {
    let p0 = Point3::new(0.0, 0.0, 0.0);
    let p1 = Point3::new(width, 0.0, 0.0);
    let p2 = Point3::new(width, height, 0.0);
    let p3 = Point3::new(0.0, height, 0.0);

    let v0 = builder::vertex(p0);
    let v1 = builder::vertex(p1);
    let v2 = builder::vertex(p2);
    let v3 = builder::vertex(p3);

    Wire::from(vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        builder::line(&v3, &v0),
    ])
}

/// Create a circular wire in the XY plane centered at origin.
///
/// Uses rsweep of a vertex to create a proper full circle.
pub fn make_circle_wire(radius: f64) -> Wire {
    let v = builder::vertex(Point3::new(radius, 0.0, 0.0));
    builder::rsweep(&v, Point3::origin(), Vector3::unit_z(), Rad(std::f64::consts::TAU))
}

/// Create a planar face from a wire (the wire must be planar and closed).
pub fn make_face_from_wire(wire: &Wire) -> Option<Face> {
    builder::try_attach_plane(&[wire.clone()]).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rect_wire_creates_4_edges() {
        let wire = make_rect_wire(10.0, 5.0);
        assert_eq!(wire.edge_iter().count(), 4);
    }

    #[test]
    fn circle_wire_edge_count() {
        let wire = make_circle_wire(5.0);
        // rsweep with whole revolution divides into 3 arcs
        assert_eq!(wire.edge_iter().count(), 3);
    }

    #[test]
    fn face_from_rect_wire() {
        let wire = make_rect_wire(10.0, 5.0);
        let face = make_face_from_wire(&wire);
        assert!(face.is_some());
    }

    #[test]
    fn face_from_circle_wire() {
        let wire = make_circle_wire(5.0);
        let face = make_face_from_wire(&wire);
        assert!(face.is_some());
    }

    #[test]
    fn wire_builder_triangle() {
        let mut wb = WireBuilder::new();
        wb.line_to(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0))
            .line_to(Point3::new(10.0, 0.0, 0.0), Point3::new(5.0, 8.0, 0.0))
            .line_to(Point3::new(5.0, 8.0, 0.0), Point3::new(0.0, 0.0, 0.0));
        let wire = wb.build();
        assert_eq!(wire.edge_iter().count(), 3);
    }
}
