//! `.cadora` project file format.
//!
//! A `.cadora` file is a ZIP archive (similar to FreeCAD's `.FCStd`) containing:
//! - `Document.json` — document metadata, object tree, properties
//! - `shapes/<name>.step` — STEP-encoded BRep shapes for each geometric object
//!
//! This keeps a clean separation between metadata (JSON, human-inspectable) and
//! geometry data (STEP, industry standard).

use cadora_brep::{Mesh, Shape};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io::{Read, Seek, Write};
use std::path::Path;
use zip::write::SimpleFileOptions;

/// Errors that can occur during project file I/O.
#[derive(Debug)]
pub enum ProjectError {
    /// I/O error.
    Io(std::io::Error),
    /// ZIP archive error.
    Zip(zip::result::ZipError),
    /// JSON (de)serialization error.
    Json(serde_json::Error),
    /// STEP export/import error.
    Step(String),
    /// The archive is missing required entries.
    MissingEntry(String),
}

impl std::fmt::Display for ProjectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "project I/O: {e}"),
            Self::Zip(e) => write!(f, "project ZIP: {e}"),
            Self::Json(e) => write!(f, "project JSON: {e}"),
            Self::Step(s) => write!(f, "project STEP: {s}"),
            Self::MissingEntry(s) => write!(f, "missing entry: {s}"),
        }
    }
}

impl std::error::Error for ProjectError {}

impl From<std::io::Error> for ProjectError {
    fn from(e: std::io::Error) -> Self { Self::Io(e) }
}

impl From<zip::result::ZipError> for ProjectError {
    fn from(e: zip::result::ZipError) -> Self { Self::Zip(e) }
}

impl From<serde_json::Error> for ProjectError {
    fn from(e: serde_json::Error) -> Self { Self::Json(e) }
}

// ── Metadata types ───────────────────────────────────────────────────────────

/// Serializable document metadata — the content of `Document.json`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectMeta {
    /// Format version for forward compatibility.
    pub format_version: u32,
    /// Document label / title.
    pub label: String,
    /// Per-object metadata keyed by object name.
    pub objects: Vec<ObjectMeta>,
    /// Arbitrary key-value metadata.
    #[serde(default)]
    pub properties: HashMap<String, serde_json::Value>,
}

/// Metadata for a single document object.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObjectMeta {
    /// Unique object name (e.g. "Pad001").
    pub name: String,
    /// Human-readable label.
    pub label: String,
    /// Type name (e.g. "PartDesign::Pad").
    pub type_name: String,
    /// Whether this object has an associated shape file in the archive.
    pub has_shape: bool,
    /// Serialized properties as JSON.
    #[serde(default)]
    pub properties: HashMap<String, serde_json::Value>,
    /// Dependencies (names of objects this one depends on).
    #[serde(default)]
    pub dependencies: Vec<String>,
}

impl ProjectMeta {
    /// Current format version.
    pub const FORMAT_VERSION: u32 = 1;

    /// Create a new empty project metadata.
    pub fn new(label: impl Into<String>) -> Self {
        Self {
            format_version: Self::FORMAT_VERSION,
            label: label.into(),
            objects: Vec::new(),
            properties: HashMap::new(),
        }
    }
}

// ── Writer ───────────────────────────────────────────────────────────────────

/// Builder for writing `.cadora` project files.
pub struct ProjectWriter<W: Write + Seek> {
    zip: zip::ZipWriter<W>,
    meta: ProjectMeta,
}

impl<W: Write + Seek> ProjectWriter<W> {
    /// Create a new project writer wrapping an output stream.
    pub fn new(writer: W, label: impl Into<String>) -> Self {
        Self {
            zip: zip::ZipWriter::new(writer),
            meta: ProjectMeta::new(label),
        }
    }

    /// Set an arbitrary metadata property.
    pub fn set_property(&mut self, key: impl Into<String>, value: serde_json::Value) {
        self.meta.properties.insert(key.into(), value);
    }

    /// Add an object with its shape to the project file.
    pub fn add_object_with_shape(
        &mut self,
        meta: ObjectMeta,
        shape: &Shape,
    ) -> std::result::Result<(), ProjectError> {
        let shape_path = format!("shapes/{}.step", meta.name);

        // Write shape as STEP into the ZIP
        let mut step_buf = Vec::new();
        crate::step_export(shape, &mut step_buf)
            .map_err(|e| ProjectError::Step(format!("{e}")))?;

        let options = SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Deflated);
        self.zip.start_file(&shape_path, options)?;
        self.zip.write_all(&step_buf)?;

        // Record metadata
        self.meta.objects.push(meta);
        Ok(())
    }

    /// Add an object without a shape (metadata only).
    pub fn add_object(&mut self, meta: ObjectMeta) {
        self.meta.objects.push(meta);
    }

    /// Finalize the archive: write Document.json and close.
    pub fn finish(mut self) -> std::result::Result<W, ProjectError> {
        let json = serde_json::to_string_pretty(&self.meta)?;
        let options = SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Deflated);
        self.zip.start_file("Document.json", options)?;
        self.zip.write_all(json.as_bytes())?;
        Ok(self.zip.finish()?)
    }
}

// ── Reader ───────────────────────────────────────────────────────────────────

/// Result of reading a `.cadora` project file.
pub struct ProjectData {
    /// Document metadata.
    pub meta: ProjectMeta,
    /// Loaded shapes keyed by object name (from STEP data, tessellated).
    pub shapes: HashMap<String, Mesh>,
}

/// Read a `.cadora` project file.
pub fn project_read<R: Read + Seek>(reader: R) -> std::result::Result<ProjectData, ProjectError> {
    let mut archive = zip::ZipArchive::new(reader)?;

    // Read Document.json
    let meta: ProjectMeta = {
        let mut entry = archive
            .by_name("Document.json")
            .map_err(|_| ProjectError::MissingEntry("Document.json".into()))?;
        let mut json_str = String::new();
        entry.read_to_string(&mut json_str)?;
        serde_json::from_str(&json_str)?
    };

    // Load shapes
    let mut shapes = HashMap::new();
    for obj in &meta.objects {
        if !obj.has_shape {
            continue;
        }
        let shape_path = format!("shapes/{}.step", obj.name);
        let mut entry = archive
            .by_name(&shape_path)
            .map_err(|_| ProjectError::MissingEntry(shape_path.clone()))?;
        let mut step_text = String::new();
        entry.read_to_string(&mut step_text)?;
        let imported = crate::step_import_from_str(&step_text)
            .map_err(|e| ProjectError::Step(format!("{e}")))?;
        if let Some(mesh) = imported.into_iter().next() {
            shapes.insert(obj.name.clone(), mesh);
        }
    }

    Ok(ProjectData { meta, shapes })
}

/// Read a `.cadora` project file from disk.
pub fn project_read_file(path: &Path) -> std::result::Result<ProjectData, ProjectError> {
    let file = std::fs::File::open(path)?;
    let reader = std::io::BufReader::new(file);
    project_read(reader)
}

/// Write a project to a file on disk.
pub fn project_write_file(
    path: &Path,
    label: impl Into<String>,
    objects: Vec<(ObjectMeta, Option<&Shape>)>,
) -> std::result::Result<(), ProjectError> {
    let file = std::fs::File::create(path)?;
    let buf_writer = std::io::BufWriter::new(file);
    let mut writer = ProjectWriter::new(buf_writer, label);
    for (meta, shape) in objects {
        match shape {
            Some(s) => writer.add_object_with_shape(meta, s)?,
            None => writer.add_object(meta),
        }
    }
    writer.finish()?;
    Ok(())
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
    fn roundtrip_project_in_memory() {
        let cube = make_cube();

        // Write
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut writer = ProjectWriter::new(&mut buf, "Test Project");
            writer.set_property("author", serde_json::json!("test"));
            writer
                .add_object_with_shape(
                    ObjectMeta {
                        name: "Cube001".into(),
                        label: "My Cube".into(),
                        type_name: "Part::Box".into(),
                        has_shape: true,
                        properties: HashMap::new(),
                        dependencies: Vec::new(),
                    },
                    &cube,
                )
                .unwrap();
            writer
                .add_object(ObjectMeta {
                    name: "Origin".into(),
                    label: "Origin".into(),
                    type_name: "App::Origin".into(),
                    has_shape: false,
                    properties: HashMap::new(),
                    dependencies: Vec::new(),
                });
            writer.finish().unwrap();
        }

        // Read
        buf.set_position(0);
        let data = project_read(buf).unwrap();
        assert_eq!(data.meta.label, "Test Project");
        assert_eq!(data.meta.format_version, ProjectMeta::FORMAT_VERSION);
        assert_eq!(data.meta.objects.len(), 2);
        assert_eq!(data.meta.objects[0].name, "Cube001");
        assert_eq!(data.meta.objects[1].name, "Origin");
        assert!(data.shapes.contains_key("Cube001"));
        assert!(!data.shapes.contains_key("Origin"));
    }

    #[test]
    fn project_metadata_properties() {
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut writer = ProjectWriter::new(&mut buf, "Props Test");
            writer.set_property("version", serde_json::json!("1.0.0"));
            writer.set_property("units", serde_json::json!("mm"));
            writer.finish().unwrap();
        }

        buf.set_position(0);
        let data = project_read(buf).unwrap();
        assert_eq!(data.meta.properties["version"], "1.0.0");
        assert_eq!(data.meta.properties["units"], "mm");
    }

    #[test]
    fn project_object_dependencies() {
        let cube = make_cube();
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let mut writer = ProjectWriter::new(&mut buf, "Deps");
            writer.add_object_with_shape(
                ObjectMeta {
                    name: "Base".into(),
                    label: "Base".into(),
                    type_name: "Part::Box".into(),
                    has_shape: true,
                    properties: HashMap::new(),
                    dependencies: Vec::new(),
                },
                &cube,
            ).unwrap();
            writer.add_object(ObjectMeta {
                name: "Fillet001".into(),
                label: "Fillet".into(),
                type_name: "PartDesign::Fillet".into(),
                has_shape: false,
                properties: HashMap::new(),
                dependencies: vec!["Base".into()],
            });
            writer.finish().unwrap();
        }

        buf.set_position(0);
        let data = project_read(buf).unwrap();
        assert_eq!(data.meta.objects[1].dependencies, vec!["Base"]);
    }

    #[test]
    fn project_file_roundtrip() {
        let cube = make_cube();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.cadora");

        project_write_file(
            &path,
            "File Test",
            vec![(
                ObjectMeta {
                    name: "Box".into(),
                    label: "Box".into(),
                    type_name: "Part::Box".into(),
                    has_shape: true,
                    properties: HashMap::new(),
                    dependencies: Vec::new(),
                },
                Some(&cube),
            )],
        )
        .unwrap();

        let data = project_read_file(&path).unwrap();
        assert_eq!(data.meta.label, "File Test");
        assert!(data.shapes.contains_key("Box"));
    }

    #[test]
    fn project_empty() {
        let mut buf = std::io::Cursor::new(Vec::new());
        {
            let writer = ProjectWriter::new(&mut buf, "Empty");
            writer.finish().unwrap();
        }

        buf.set_position(0);
        let data = project_read(buf).unwrap();
        assert_eq!(data.meta.label, "Empty");
        assert!(data.meta.objects.is_empty());
        assert!(data.shapes.is_empty());
    }
}
