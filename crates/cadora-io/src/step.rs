//! STEP (ISO 10303-21) import/export via truck-stepio.
//!
//! Export: Shape → CompressedSolid → StepModel → ISO text
//! Import: ISO text → Table → CompressedShell → tessellate → Mesh
//!
//! Note: STEP import returns meshes because the stepio curve/surface types
//! differ from truck-modeling's types. The mesh can be used for visualization.
//! For full BRep import, upstream type unification would be required.

use cadora_brep::{Mesh, Shape};
use std::io::{Read, Write};
use std::path::Path;
use truck_modeling::*;
use truck_stepio::out::{
    CompleteStepDisplay, StepHeaderDescriptor, StepModel, StepModels,
};

/// Errors that can occur during STEP I/O.
#[derive(Debug)]
pub enum StepError {
    /// I/O error reading or writing file.
    Io(std::io::Error),
    /// The STEP data could not be parsed.
    ParseFailed(String),
    /// The parsed topology could not be converted to a solid.
    TopologyError(String),
    /// The shape contains geometry that cannot be exported.
    ExportUnsupported(String),
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "STEP I/O error: {e}"),
            Self::ParseFailed(s) => write!(f, "STEP parse error: {s}"),
            Self::TopologyError(s) => write!(f, "STEP topology error: {s}"),
            Self::ExportUnsupported(s) => write!(f, "STEP export unsupported: {s}"),
        }
    }
}

impl std::error::Error for StepError {}

impl From<std::io::Error> for StepError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

// ── Export ────────────────────────────────────────────────────────────────────

/// Export a single shape to STEP format, writing to `writer`.
pub fn step_export<W: Write>(shape: &Shape, writer: &mut W) -> std::result::Result<(), StepError> {
    step_export_with_header(shape, writer, StepHeaderDescriptor::default())
}

/// Export a single shape to STEP format with a custom header.
pub fn step_export_with_header<W: Write>(
    shape: &Shape,
    writer: &mut W,
    header: StepHeaderDescriptor,
) -> std::result::Result<(), StepError> {
    let compressed = shape.solid().compress();
    let step_model: StepModel<'_, Point3, Curve, Surface> =
        StepModel::from(&compressed);
    let display = CompleteStepDisplay::new(step_model, header);
    write!(writer, "{display}").map_err(|e| StepError::Io(std::io::Error::new(std::io::ErrorKind::Other, e)))
}

/// Export multiple shapes to a single STEP file.
pub fn step_export_many<W: Write>(
    shapes: &[Shape],
    writer: &mut W,
) -> std::result::Result<(), StepError> {
    let compressed: Vec<_> = shapes.iter().map(|s| s.solid().compress()).collect();
    let step_models: StepModels<'_, Point3, Curve, Surface> =
        compressed.iter().collect();
    let display = CompleteStepDisplay::new(step_models, StepHeaderDescriptor::default());
    write!(writer, "{display}").map_err(|e| StepError::Io(std::io::Error::new(std::io::ErrorKind::Other, e)))
}

/// Export a shape to a STEP file on disk.
pub fn step_export_file(shape: &Shape, path: &Path) -> std::result::Result<(), StepError> {
    let mut file = std::fs::File::create(path)?;
    step_export(shape, &mut file)
}

// ── Import ────────────────────────────────────────────────────────────────────

/// Import shapes from STEP text.
///
/// Returns one `Mesh` per shell found in the STEP data.
/// The mesh is tessellated from the parsed B-Rep surfaces.
/// If the file contains no geometry, the result is an empty vec.
pub fn step_import_from_str(step_text: &str) -> std::result::Result<Vec<Mesh>, StepError> {
    use truck_meshalgo::tessellation::{MeshableShape, MeshedShape};

    let table = truck_stepio::r#in::Table::from_step(step_text)
        .ok_or_else(|| StepError::ParseFailed("failed to parse STEP data".into()))?;

    let mut meshes = Vec::new();
    for shell_holder in table.shell.values() {
        let compressed = table.to_compressed_shell(shell_holder).map_err(|e| {
            StepError::TopologyError(format!("{e:?}"))
        })?;

        // Tessellate using truck-meshalgo
        let tessellated = compressed.triangulation(0.01);
        let poly = tessellated.to_polygon();

        let positions: Vec<[f64; 3]> = poly.positions().iter().map(|p| [p.x, p.y, p.z]).collect();
        let mut indices = Vec::new();
        let mut normals = vec![[0.0f64; 3]; positions.len()];

        for tri in poly.faces().triangle_iter() {
            let i0 = tri[0].pos;
            let i1 = tri[1].pos;
            let i2 = tri[2].pos;
            indices.push([i0, i1, i2]);

            // Compute face normal
            let p0 = &positions[i0];
            let p1 = &positions[i1];
            let p2 = &positions[i2];
            let e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            let e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
            let nx = e1[1] * e2[2] - e1[2] * e2[1];
            let ny = e1[2] * e2[0] - e1[0] * e2[2];
            let nz = e1[0] * e2[1] - e1[1] * e2[0];
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            let n = if len > 1e-14 { [nx / len, ny / len, nz / len] } else { [0.0, 0.0, 1.0] };
            for &idx in &[i0, i1, i2] {
                normals[idx][0] += n[0];
                normals[idx][1] += n[1];
                normals[idx][2] += n[2];
            }
        }

        // Normalize accumulated normals
        for n in &mut normals {
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if len > 1e-14 {
                n[0] /= len;
                n[1] /= len;
                n[2] /= len;
            }
        }

        if !indices.is_empty() {
            meshes.push(Mesh { positions, indices, normals });
        }
    }
    Ok(meshes)
}

/// Import meshes from a reader.
pub fn step_import<R: Read>(reader: &mut R) -> std::result::Result<Vec<Mesh>, StepError> {
    let mut text = String::new();
    reader.read_to_string(&mut text)?;
    step_import_from_str(&text)
}

/// Import meshes from a STEP file on disk.
pub fn step_import_file(path: &Path) -> std::result::Result<Vec<Mesh>, StepError> {
    let text = std::fs::read_to_string(path)?;
    step_import_from_str(&text)
}

#[cfg(test)]
mod tests {
    use super::*;
    use truck_modeling::builder;

    fn make_cube() -> Shape {
        let v = builder::vertex(Point3::new(0.0, 0.0, 0.0));
        let e = builder::tsweep(&v, Vector3::unit_x());
        let f = builder::tsweep(&e, Vector3::unit_y());
        let solid = builder::tsweep(&f, Vector3::unit_z());
        Shape::from_solid(solid)
    }

    #[test]
    fn export_produces_step_text() {
        let shape = make_cube();
        let mut buf = Vec::new();
        step_export(&shape, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        assert!(text.contains("ISO-10303-21"));
        assert!(text.contains("HEADER"));
        assert!(text.contains("DATA"));
        assert!(text.contains("END-ISO-10303-21"));
    }

    #[test]
    fn export_contains_geometry() {
        let shape = make_cube();
        let mut buf = Vec::new();
        step_export(&shape, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        // A cube shell should produce faces and edges
        assert!(text.contains("FACE_SURFACE"));
        assert!(text.contains("EDGE_CURVE"));
        assert!(text.contains("VERTEX_POINT"));
    }

    #[test]
    fn roundtrip_cube() {
        let shape = make_cube();
        let mut buf = Vec::new();
        step_export(&shape, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();

        let imported = step_import_from_str(&text).unwrap();
        // Should get at least one mesh back
        assert!(!imported.is_empty(), "roundtrip produced no meshes");
        assert!(imported[0].triangle_count() > 0);
    }

    #[test]
    fn export_many_shapes() {
        let shapes = vec![make_cube(), make_cube()];
        let mut buf = Vec::new();
        step_export_many(&shapes, &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        assert!(text.contains("ISO-10303-21"));
    }

    #[test]
    fn import_invalid_step() {
        let result = step_import_from_str("not a step file");
        assert!(result.is_err());
    }

    #[test]
    fn import_empty_step() {
        // Valid STEP structure but no geometry
        let step = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''), '2;1');\nFILE_NAME('', '', (''), (''), '', '', '');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n";
        let result = step_import_from_str(step).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn export_file_roundtrip() {
        let shape = make_cube();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.step");
        step_export_file(&shape, &path).unwrap();

        let imported = step_import_file(&path).unwrap();
        assert!(!imported.is_empty());
        assert!(imported[0].triangle_count() > 0);
    }
}
