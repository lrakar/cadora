//! STL import/export for triangle meshes.
//!
//! Export: Shape → tessellate → PolygonMesh → STL (binary or ASCII)
//! Import: STL file → PolygonMesh → Mesh

use cadora_brep::{Mesh, Shape, TessellationParams};
use std::io::{Read, Write};
use std::path::Path;
use truck_polymesh::stl::{StlFace, StlReader, StlType};

/// Errors that can occur during STL I/O.
#[derive(Debug)]
pub enum StlError {
    /// I/O error.
    Io(std::io::Error),
    /// The STL data is malformed.
    ParseFailed(String),
}

impl std::fmt::Display for StlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "STL I/O error: {e}"),
            Self::ParseFailed(s) => write!(f, "STL parse error: {s}"),
        }
    }
}

impl std::error::Error for StlError {}

impl From<std::io::Error> for StlError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

/// STL output format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StlFormat {
    /// Binary STL (compact, default).
    Binary,
    /// ASCII STL (human-readable).
    Ascii,
}

// ── Export ────────────────────────────────────────────────────────────────────

/// Export a shape to binary STL using default tessellation parameters.
pub fn stl_export<W: Write>(shape: &Shape, writer: &mut W) -> std::result::Result<(), StlError> {
    stl_export_with_params(shape, writer, &TessellationParams::default(), StlFormat::Binary)
}

/// Export a shape to STL with custom tessellation parameters and format.
pub fn stl_export_with_params<W: Write>(
    shape: &Shape,
    writer: &mut W,
    params: &TessellationParams,
    format: StlFormat,
) -> std::result::Result<(), StlError> {
    let mesh = cadora_brep::tessellate(shape, params);
    write_stl_from_mesh(&mesh, writer, format)
}

/// Write a pre-computed mesh to STL.
pub fn write_stl_from_mesh<W: Write>(
    mesh: &Mesh,
    writer: &mut W,
    format: StlFormat,
) -> std::result::Result<(), StlError> {
    let faces = mesh_to_stl_faces(mesh);
    let stl_type = match format {
        StlFormat::Binary => StlType::Binary,
        StlFormat::Ascii => StlType::Ascii,
    };
    truck_polymesh::stl::write(faces, writer, stl_type)
        .map_err(|e| StlError::Io(std::io::Error::new(std::io::ErrorKind::Other, format!("{e}"))))
}

/// Convert our Mesh to a Vec<StlFace> for truck_polymesh::stl::write.
fn mesh_to_stl_faces(mesh: &Mesh) -> Vec<StlFace> {
    mesh.indices
        .iter()
        .map(|tri| {
            let p0 = mesh.positions[tri[0]];
            let p1 = mesh.positions[tri[1]];
            let p2 = mesh.positions[tri[2]];

            // Compute face normal from triangle vertices
            let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
            let nx = e1[1] * e2[2] - e1[2] * e2[1];
            let ny = e1[2] * e2[0] - e1[0] * e2[2];
            let nz = e1[0] * e2[1] - e1[1] * e2[0];
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            let normal = if len > 1e-14 {
                [(nx / len) as f32, (ny / len) as f32, (nz / len) as f32]
            } else {
                [0.0f32, 0.0, 1.0]
            };

            StlFace {
                normal,
                vertices: [
                    [p0[0] as f32, p0[1] as f32, p0[2] as f32],
                    [p1[0] as f32, p1[1] as f32, p1[2] as f32],
                    [p2[0] as f32, p2[1] as f32, p2[2] as f32],
                ],
            }
        })
        .collect()
}

/// Export a shape to an STL file on disk (binary format).
pub fn stl_export_file(shape: &Shape, path: &Path) -> std::result::Result<(), StlError> {
    let mut file = std::fs::File::create(path)?;
    stl_export(shape, &mut file)
}

// ── Import ────────────────────────────────────────────────────────────────────

/// Import an STL file into a Mesh (triangle mesh, not BRep).
pub fn stl_import<R: Read>(reader: R) -> std::result::Result<Mesh, StlError> {
    let stl_reader = StlReader::new(reader, StlType::Automatic)
        .map_err(|e| StlError::ParseFailed(format!("{e}")))?;

    let mut positions = Vec::new();
    let mut indices = Vec::new();
    let mut normals = Vec::new();

    for face_result in stl_reader {
        let face = face_result.map_err(|e| StlError::ParseFailed(format!("{e}")))?;
        let base = positions.len();

        for v in &face.vertices {
            positions.push([v[0] as f64, v[1] as f64, v[2] as f64]);
            normals.push([
                face.normal[0] as f64,
                face.normal[1] as f64,
                face.normal[2] as f64,
            ]);
        }
        indices.push([base, base + 1, base + 2]);
    }

    Ok(Mesh {
        positions,
        indices,
        normals,
    })
}

/// Import an STL file from disk.
pub fn stl_import_file(path: &Path) -> std::result::Result<Mesh, StlError> {
    let file = std::fs::File::open(path)?;
    stl_import(file)
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_modeling::*;
    use truck_modeling::builder;

    fn make_cube() -> Shape {
        let v = builder::vertex(Point3::new(0.0, 0.0, 0.0));
        let e = builder::tsweep(&v, Vector3::unit_x());
        let f = builder::tsweep(&e, Vector3::unit_y());
        let solid = builder::tsweep(&f, Vector3::unit_z());
        Shape::from_solid(solid)
    }

    #[test]
    fn export_binary_stl() {
        let shape = make_cube();
        let mut buf = Vec::new();
        stl_export(&shape, &mut buf).unwrap();
        // Binary STL: 80 byte header + 4 byte count + 50 bytes per face
        assert!(buf.len() > 84, "binary STL too short: {} bytes", buf.len());
    }

    #[test]
    fn export_ascii_stl() {
        let shape = make_cube();
        let mut buf = Vec::new();
        stl_export_with_params(&shape, &mut buf, &TessellationParams::default(), StlFormat::Ascii)
            .unwrap();
        let text = String::from_utf8(buf).unwrap();
        assert!(text.starts_with("solid"));
        assert!(text.contains("facet normal"));
        assert!(text.contains("vertex"));
        assert!(text.contains("endsolid"));
    }

    #[test]
    fn roundtrip_binary_stl() {
        let shape = make_cube();
        let mut buf = Vec::new();
        stl_export(&shape, &mut buf).unwrap();

        let mesh = stl_import(buf.as_slice()).unwrap();
        assert!(mesh.triangle_count() > 0);
        assert!(mesh.vertex_count() > 0);
        assert!(mesh.is_valid());
    }

    #[test]
    fn roundtrip_ascii_stl() {
        let shape = make_cube();
        let mut buf = Vec::new();
        stl_export_with_params(&shape, &mut buf, &TessellationParams::default(), StlFormat::Ascii)
            .unwrap();

        let mesh = stl_import(buf.as_slice()).unwrap();
        assert!(mesh.triangle_count() > 0);
    }

    #[test]
    fn export_file_stl() {
        let shape = make_cube();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.stl");
        stl_export_file(&shape, &path).unwrap();

        let mesh = stl_import_file(&path).unwrap();
        assert!(mesh.triangle_count() > 0);
    }

    #[test]
    fn mesh_normals_unit_length() {
        let shape = make_cube();
        let mut buf = Vec::new();
        stl_export(&shape, &mut buf).unwrap();
        let mesh = stl_import(buf.as_slice()).unwrap();

        for n in &mesh.normals {
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            // Allow some tolerance for f32 → f64 conversion
            assert!((len - 1.0).abs() < 0.01 || len < 1e-10, "normal not unit: {len}");
        }
    }
}
