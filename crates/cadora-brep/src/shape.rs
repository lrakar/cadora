//! Core BRep shape types wrapping truck's topology.

use truck_modeling::*;

/// A 3D point in space.
pub type Vec3 = nalgebra::Vector3<f64>;

/// BRep shape type classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShapeType {
    Vertex,
    Edge,
    Wire,
    Face,
    Shell,
    Solid,
    Compound,
}

/// A BRep solid shape — the primary output of modeling operations.
///
/// Wraps truck's `Solid` type with a simpler API.
#[derive(Debug, Clone)]
pub struct Shape {
    inner: Solid,
}

impl Shape {
    /// Create a Shape from a truck Solid.
    pub fn from_solid(solid: Solid) -> Self {
        Self { inner: solid }
    }

    /// Access the underlying truck Solid.
    pub fn solid(&self) -> &Solid {
        &self.inner
    }

    /// Consume and return the inner truck Solid.
    pub fn into_solid(self) -> Solid {
        self.inner
    }

    /// Get the bounding box of the shape by sampling edge curves and surfaces.
    pub fn bounding_box(&self) -> BoundingBox {
        let mut min = [f64::MAX; 3];
        let mut max = [f64::MIN; 3];

        let mut update = |pt: Point3| {
            min[0] = min[0].min(pt.x);
            min[1] = min[1].min(pt.y);
            min[2] = min[2].min(pt.z);
            max[0] = max[0].max(pt.x);
            max[1] = max[1].max(pt.y);
            max[2] = max[2].max(pt.z);
        };

        for shell in self.inner.boundaries() {
            for face in shell.face_iter() {
                // Sample RevolutedCurve surfaces at interior grid points
                let surface = face.oriented_surface();
                if let Surface::RevolutedCurve(ref rc) = surface {
                    let ent_curve = rc.entity().entity_curve();
                    let curve_range = match ent_curve {
                        Curve::Line(_) => (0.0, 1.0),
                        Curve::BSplineCurve(bsp) => {
                            let kv = bsp.knot_vec();
                            (kv[0], kv[kv.len() - 1])
                        }
                        Curve::NurbsCurve(nbs) => {
                            let kv = nbs.knot_vec();
                            (kv[0], kv[kv.len() - 1])
                        }
                        Curve::IntersectionCurve(_) => (0.0, 1.0),
                    };
                    // Each face from whole_rsweep(DIVISION=3) covers 2π/3
                    let angle_range = std::f64::consts::TAU / 3.0;
                    let steps = 16;

                    if rc.orientation() {
                        // External (u, v) maps to internal (u=curve, v=angle)
                        for ui in 0..=steps {
                            for vi in 0..=steps {
                                let u = curve_range.0
                                    + (curve_range.1 - curve_range.0)
                                        * (ui as f64 / steps as f64);
                                let v = angle_range * (vi as f64 / steps as f64);
                                update(surface.subs(u, v));
                            }
                        }
                    } else {
                        // External (u, v) swapped: u=angle, v=curve
                        for ui in 0..=steps {
                            for vi in 0..=steps {
                                let u = angle_range * (ui as f64 / steps as f64);
                                let v = curve_range.0
                                    + (curve_range.1 - curve_range.0)
                                        * (vi as f64 / steps as f64);
                                update(surface.subs(u, v));
                            }
                        }
                    }
                }

                // Edge vertex positions and curve sampling
                for wire in face.boundaries() {
                    for edge in wire.edge_iter() {
                        update(edge.front().point());
                        update(edge.back().point());

                        let curve = edge.oriented_curve();
                        let (t0, t1) = edge_curve_range(&curve);
                        let n = 32;
                        for i in 1..n {
                            let t = t0 + (t1 - t0) * (i as f64 / n as f64);
                            update(curve.subs(t));
                        }
                    }
                }
            }
        }

        BoundingBox {
            min: Vec3::new(min[0], min[1], min[2]),
            max: Vec3::new(max[0], max[1], max[2]),
        }
    }

    /// Count the number of faces in the shape.
    pub fn face_count(&self) -> usize {
        self.inner
            .boundaries()
            .iter()
            .map(|shell| shell.face_iter().count())
            .sum()
    }

    /// Count the number of edges in the shape.
    pub fn edge_count(&self) -> usize {
        self.inner
            .boundaries()
            .iter()
            .map(|shell| {
                let mut count = 0;
                for face in shell.face_iter() {
                    for wire in face.boundaries() {
                        count += wire.edge_iter().count();
                    }
                }
                count
            })
            .sum()
    }
}

/// Axis-aligned bounding box.
#[derive(Debug, Clone, Copy)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl BoundingBox {
    /// Size of the bounding box along each axis.
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }

    /// Center of the bounding box.
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Volume of the bounding box.
    pub fn volume(&self) -> f64 {
        let s = self.size();
        s.x * s.y * s.z
    }

    /// Check if a point is inside the bounding box.
    pub fn contains(&self, p: &Vec3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }
}

/// Get the parameter range for an edge curve.
pub(crate) fn edge_curve_range(curve: &truck_modeling::Curve) -> (f64, f64) {
    match curve {
        Curve::Line(_) => (0.0, 1.0),
        Curve::BSplineCurve(bsp) => {
            let kv = bsp.knot_vec();
            (kv[0], kv[kv.len() - 1])
        }
        Curve::NurbsCurve(nbs) => {
            let kv = nbs.knot_vec();
            (kv[0], kv[kv.len() - 1])
        }
        Curve::IntersectionCurve(_) => (0.0, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounding_box_properties() {
        let bb = BoundingBox {
            min: Vec3::new(-1.0, -2.0, -3.0),
            max: Vec3::new(1.0, 2.0, 3.0),
        };
        let size = bb.size();
        assert!((size.x - 2.0).abs() < 1e-10);
        assert!((size.y - 4.0).abs() < 1e-10);
        assert!((size.z - 6.0).abs() < 1e-10);

        let center = bb.center();
        assert!((center.x).abs() < 1e-10);
        assert!((center.y).abs() < 1e-10);
        assert!((center.z).abs() < 1e-10);

        assert!((bb.volume() - 48.0).abs() < 1e-10);
        assert!(bb.contains(&Vec3::new(0.0, 0.0, 0.0)));
        assert!(!bb.contains(&Vec3::new(2.0, 0.0, 0.0)));
    }
}
