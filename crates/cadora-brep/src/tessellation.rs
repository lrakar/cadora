//! Tessellation: convert BRep shapes to triangle meshes for rendering.

use crate::shape::Shape;
use truck_modeling::*;

/// A triangle mesh suitable for rendering.
#[derive(Debug, Clone)]
pub struct Mesh {
    /// Vertex positions (x, y, z triples).
    pub positions: Vec<[f64; 3]>,
    /// Triangle indices (3 indices per triangle).
    pub indices: Vec<[usize; 3]>,
    /// Per-vertex normals (x, y, z triples). Same length as positions.
    pub normals: Vec<[f64; 3]>,
}

impl Mesh {
    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.indices.len()
    }

    /// Check if the mesh is empty.
    pub fn is_empty(&self) -> bool {
        self.positions.is_empty() || self.indices.is_empty()
    }

    /// Validate the mesh: all indices in range, normals same length as positions.
    pub fn is_valid(&self) -> bool {
        let n = self.positions.len();
        if self.normals.len() != n {
            return false;
        }
        for tri in &self.indices {
            if tri[0] >= n || tri[1] >= n || tri[2] >= n {
                return false;
            }
        }
        true
    }
}

/// Tessellation parameters.
#[derive(Debug, Clone)]
pub struct TessellationParams {
    /// Number of subdivisions along parameter u for each face.
    pub u_divisions: usize,
    /// Number of subdivisions along parameter v for each face.
    pub v_divisions: usize,
}

impl Default for TessellationParams {
    fn default() -> Self {
        Self {
            u_divisions: 16,
            v_divisions: 16,
        }
    }
}

/// Tessellate a shape into a triangle mesh.
pub fn tessellate(shape: &Shape, params: &TessellationParams) -> Mesh {
    let mut positions = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();

    for shell in shape.solid().boundaries() {
        for face in shell.face_iter() {
            let surface = face.oriented_surface();
            let (urange, vrange) = surface_parameter_range(&surface);

            let u_steps = params.u_divisions;
            let v_steps = params.v_divisions;

            let base_idx = positions.len();

            // Generate grid of vertices
            for vi in 0..=v_steps {
                for ui in 0..=u_steps {
                    let u = urange.0 + (urange.1 - urange.0) * (ui as f64 / u_steps as f64);
                    let v = vrange.0 + (vrange.1 - vrange.0) * (vi as f64 / v_steps as f64);

                    let pt = surface.subs(u, v);
                    positions.push([pt.x, pt.y, pt.z]);

                    // Approximate normal via cross product of partial derivatives
                    let du = 1e-6;
                    let dv = 1e-6;
                    let pt_u = surface.subs(u + du, v);
                    let pt_v = surface.subs(u, v + dv);

                    let deriv_u = Vector3::new(pt_u.x - pt.x, pt_u.y - pt.y, pt_u.z - pt.z);
                    let deriv_v = Vector3::new(pt_v.x - pt.x, pt_v.y - pt.y, pt_v.z - pt.z);
                    let normal = deriv_u.cross(deriv_v);
                    let len = (normal.x * normal.x + normal.y * normal.y + normal.z * normal.z)
                        .sqrt();
                    if len > 1e-14 {
                        normals.push([normal.x / len, normal.y / len, normal.z / len]);
                    } else {
                        normals.push([0.0, 0.0, 1.0]);
                    }
                }
            }

            // Generate triangles from the grid
            for vi in 0..v_steps {
                for ui in 0..u_steps {
                    let i00 = base_idx + vi * (u_steps + 1) + ui;
                    let i10 = i00 + 1;
                    let i01 = i00 + (u_steps + 1);
                    let i11 = i01 + 1;

                    indices.push([i00, i10, i11]);
                    indices.push([i00, i11, i01]);
                }
            }
        }
    }

    Mesh {
        positions,
        indices,
        normals,
    }
}

/// Get parameter range for a surface.
/// For RevolutedCurve surfaces, the u parameter is the curve parameter
/// and v is the rotation angle. We use knot vectors where available.
fn surface_parameter_range(surface: &Surface) -> ((f64, f64), (f64, f64)) {
    match surface {
        Surface::Plane(_) => ((0.0, 1.0), (0.0, 1.0)),
        Surface::BSplineSurface(bsp) => {
            let uknots = bsp.uknot_vec();
            let vknots = bsp.vknot_vec();
            (
                (uknots[0], uknots[uknots.len() - 1]),
                (vknots[0], vknots[vknots.len() - 1]),
            )
        }
        Surface::NurbsSurface(nbs) => {
            let uknots = nbs.uknot_vec();
            let vknots = nbs.vknot_vec();
            (
                (uknots[0], uknots[uknots.len() - 1]),
                (vknots[0], vknots[vknots.len() - 1]),
            )
        }
        Surface::RevolutedCurve(rc) => {
            // u = curve parameter, v = rotation angle
            let curve = rc.entity_curve();
            let u_range = match &curve {
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
            // v is rotation angle; each face from rsweep covers 2π/3
            (u_range, (0.0, std::f64::consts::TAU / 3.0))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::*;

    #[test]
    fn tessellate_box() {
        let shape = make_box(2.0, 3.0, 4.0);
        let params = TessellationParams {
            u_divisions: 4,
            v_divisions: 4,
        };
        let mesh = tessellate(&shape, &params);
        assert!(!mesh.is_empty());
        assert!(mesh.is_valid());
        assert!(mesh.triangle_count() > 0);
        assert!(mesh.vertex_count() > 0);
    }

    #[test]
    fn tessellate_sphere() {
        let shape = make_sphere(5.0);
        let params = TessellationParams::default();
        let mesh = tessellate(&shape, &params);
        assert!(!mesh.is_empty());
        assert!(mesh.is_valid());
        assert!(mesh.triangle_count() > 0);
    }

    #[test]
    fn mesh_validation() {
        let mesh = Mesh {
            positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            indices: vec![[0, 1, 2]],
            normals: vec![[0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
        };
        assert!(mesh.is_valid());
        assert_eq!(mesh.vertex_count(), 3);
        assert_eq!(mesh.triangle_count(), 1);
    }

    #[test]
    fn mesh_invalid_indices() {
        let mesh = Mesh {
            positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            indices: vec![[0, 1, 5]], // index 5 out of range
            normals: vec![[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]],
        };
        assert!(!mesh.is_valid());
    }
}
