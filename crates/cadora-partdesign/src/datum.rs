//! Datums, Attachment engine, Coordinate system, and ShapeBinder.
//!
//! Mirrors FreeCAD's PartDesign datum features:
//! - `DatumPlane`   — infinite reference plane (PartDesign::Plane → Part::Datum)
//! - `DatumLine`    — infinite reference line  (PartDesign::Line)
//! - `DatumPoint`   — reference point          (PartDesign::Point)
//! - `CoordinateSystem` — local X/Y/Z frame    (PartDesign::CoordinateSystem)
//! - `ShapeBinder`  — reference geometry from other bodies (PartDesign::ShapeBinder)
//! - `AttachmentEngine` — ~40 attachment modes  (Part::AttachEngine)
//!
//! Source reference: FreeCAD src/Mod/Part/App/Attacher.h,cpp + DatumFeature + Datums

use cadora_document::placement::{Placement, Rotation};
use cadora_brep::Shape;
use crate::feature::{Feature, FeatureOutput, FeatureStatus};

// ═══════════════════════════════════════════════════════════════════════
//  Attachment Modes  (mirrors FreeCAD eMapMode enum)
// ═══════════════════════════════════════════════════════════════════════

/// Attachment mode — determines how a datum's Placement is derived
/// from reference geometry.
///
/// Mirrors FreeCAD's `Attacher::eMapMode`. The discriminant values are
/// kept compatible with FreeCAD's file format numbering.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum AttachMode {
    // ── Plane / 3D modes ──────────────────────────────────────────
    Deactivated         = 0,
    Translate           = 1,
    ObjectXY            = 2,
    ObjectXZ            = 3,
    ObjectYZ            = 4,
    FlatFace            = 5,
    TangentPlane        = 6,
    NormalToPath         = 7,
    FrenetNB            = 8,
    FrenetTN            = 9,
    FrenetTB            = 10,
    Concentric          = 11,
    RevolutionSection   = 12,
    ThreePointsPlane    = 13,
    ThreePointsNormal   = 14,
    Folding             = 15,

    // ── Line / 1D modes ──────────────────────────────────────────
    AxisX               = 16,
    AxisY               = 17,
    AxisZ               = 18,
    AxisCurv            = 19,
    Directrix1          = 20,
    Directrix2          = 21,
    Asymptote1          = 22,
    Asymptote2          = 23,
    Tangent             = 24,
    Normal              = 25,
    Binormal            = 26,
    TangentU            = 27,
    TangentV            = 28,
    TwoPoints           = 29,
    Intersection        = 30,
    Proximity           = 31,

    // ── Point / 0D modes ─────────────────────────────────────────
    Origin              = 32,
    Focus1              = 33,
    Focus2              = 34,
    OnEdge              = 35,
    CenterOfCurvature   = 36,
    CenterOfMass        = 37,
    PointIntersection   = 38,
    Vertex              = 39,
    ProximityPoint1     = 40,
    ProximityPoint2     = 41,

    // ── Inertia modes ────────────────────────────────────────────
    AxisInertia1        = 42,
    AxisInertia2        = 43,
    AxisInertia3        = 44,
    InertialCS          = 45,

    // ── Extra modes ──────────────────────────────────────────────
    FaceNormal          = 46,
    OZX                 = 47,
    OZY                 = 48,
    OXY                 = 49,
    OXZ                 = 50,
    OYZ                 = 51,
    OYX                 = 52,
    ParallelPlane       = 53,
    Midpoint            = 54,
}

impl AttachMode {
    /// Human-readable name matching FreeCAD's eMapModeStrings.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Deactivated       => "Deactivated",
            Self::Translate         => "Translate",
            Self::ObjectXY          => "ObjectXY",
            Self::ObjectXZ          => "ObjectXZ",
            Self::ObjectYZ          => "ObjectYZ",
            Self::FlatFace          => "FlatFace",
            Self::TangentPlane      => "TangentPlane",
            Self::NormalToPath      => "NormalToPath",
            Self::FrenetNB          => "FrenetNB",
            Self::FrenetTN          => "FrenetTN",
            Self::FrenetTB          => "FrenetTB",
            Self::Concentric        => "Concentric",
            Self::RevolutionSection => "RevolutionSection",
            Self::ThreePointsPlane  => "ThreePointsPlane",
            Self::ThreePointsNormal => "ThreePointsNormal",
            Self::Folding           => "Folding",
            Self::AxisX             => "ObjectX",
            Self::AxisY             => "ObjectY",
            Self::AxisZ             => "ObjectZ",
            Self::AxisCurv          => "AxisOfCurvature",
            Self::Directrix1        => "Directrix1",
            Self::Directrix2        => "Directrix2",
            Self::Asymptote1        => "Asymptote1",
            Self::Asymptote2        => "Asymptote2",
            Self::Tangent           => "Tangent",
            Self::Normal            => "Normal",
            Self::Binormal          => "Binormal",
            Self::TangentU          => "TangentU",
            Self::TangentV          => "TangentV",
            Self::TwoPoints         => "TwoPoints",
            Self::Intersection      => "Intersection",
            Self::Proximity         => "Proximity",
            Self::Origin            => "Origin",
            Self::Focus1            => "Focus1",
            Self::Focus2            => "Focus2",
            Self::OnEdge            => "OnEdge",
            Self::CenterOfCurvature => "CenterOfCurvature",
            Self::CenterOfMass      => "CenterOfMass",
            Self::PointIntersection => "Intersection",
            Self::Vertex            => "Vertex",
            Self::ProximityPoint1   => "ProximityPoint1",
            Self::ProximityPoint2   => "ProximityPoint2",
            Self::AxisInertia1      => "AxisInertia1",
            Self::AxisInertia2      => "AxisInertia2",
            Self::AxisInertia3      => "AxisInertia3",
            Self::InertialCS        => "InertialCS",
            Self::FaceNormal        => "FaceNormal",
            Self::OZX               => "OZX",
            Self::OZY               => "OZY",
            Self::OXY               => "OXY",
            Self::OXZ               => "OXZ",
            Self::OYZ               => "OYZ",
            Self::OYX               => "OYX",
            Self::ParallelPlane     => "ParallelPlane",
            Self::Midpoint          => "Midpoint",
        }
    }

    /// Whether this is a plane-producing mode.
    pub fn is_plane_mode(&self) -> bool {
        matches!(self,
            Self::Deactivated | Self::Translate | Self::ObjectXY | Self::ObjectXZ |
            Self::ObjectYZ | Self::FlatFace | Self::TangentPlane | Self::NormalToPath |
            Self::FrenetNB | Self::FrenetTN | Self::FrenetTB | Self::Concentric |
            Self::RevolutionSection | Self::ThreePointsPlane | Self::ThreePointsNormal |
            Self::Folding | Self::InertialCS | Self::OZX | Self::OZY | Self::OXY |
            Self::OXZ | Self::OYZ | Self::OYX | Self::ParallelPlane
        )
    }

    /// Whether this is a line-producing mode.
    pub fn is_line_mode(&self) -> bool {
        matches!(self,
            Self::AxisX | Self::AxisY | Self::AxisZ | Self::AxisCurv |
            Self::Directrix1 | Self::Directrix2 | Self::Asymptote1 | Self::Asymptote2 |
            Self::Tangent | Self::Normal | Self::Binormal | Self::TangentU | Self::TangentV |
            Self::TwoPoints | Self::Intersection | Self::Proximity |
            Self::AxisInertia1 | Self::AxisInertia2 | Self::AxisInertia3 | Self::FaceNormal
        )
    }

    /// Whether this is a point-producing mode.
    pub fn is_point_mode(&self) -> bool {
        matches!(self,
            Self::Origin | Self::Focus1 | Self::Focus2 | Self::OnEdge |
            Self::CenterOfCurvature | Self::CenterOfMass | Self::PointIntersection |
            Self::Vertex | Self::ProximityPoint1 | Self::ProximityPoint2 | Self::Midpoint
        )
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Reference types  (mirrors FreeCAD eRefType)
// ═══════════════════════════════════════════════════════════════════════

/// Type of geometric reference for attachment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefType {
    Anything,
    Vertex,
    Edge,
    Face,
    Line,
    Curve,
    Circle,
    Conic,
    FlatFace,
    SphericalFace,
    CylindricalFace,
    ConicalFace,
    ToroidalFace,
    SurfaceOfRevolution,
    Part,
    Solid,
    Wire,
}

// ═══════════════════════════════════════════════════════════════════════
//  Attachment Reference
// ═══════════════════════════════════════════════════════════════════════

/// A reference to geometry used by the attachment engine.
///
/// In FreeCAD this is `PropertyLinkSub` — an object + sub-element string.
/// Here we store the placement of the referenced object and the
/// sub-element type/geometry directly.
#[derive(Debug, Clone)]
pub struct AttachRef {
    /// Position and orientation of the referenced sub-element in global space.
    pub placement: Placement,
    /// The sub-element type (vertex, edge, face, etc.).
    pub ref_type: RefType,
    /// For point refs: the point position.
    pub point: Option<[f64; 3]>,
    /// For line/edge refs: direction vector.
    pub direction: Option<[f64; 3]>,
    /// For face refs: face normal.
    pub normal: Option<[f64; 3]>,
}

impl AttachRef {
    /// Create a point reference.
    pub fn point(pos: [f64; 3]) -> Self {
        Self {
            placement: Placement::from_translation(pos[0], pos[1], pos[2]),
            ref_type: RefType::Vertex,
            point: Some(pos),
            direction: None,
            normal: None,
        }
    }

    /// Create a flat face reference with position and normal.
    pub fn flat_face(origin: [f64; 3], normal: [f64; 3]) -> Self {
        // Build rotation that maps Z→normal
        let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], normal);
        Self {
            placement: Placement::new(origin, rot),
            ref_type: RefType::FlatFace,
            point: Some(origin),
            direction: None,
            normal: Some(normal),
        }
    }

    /// Create a line/edge reference with origin and direction.
    pub fn line(origin: [f64; 3], direction: [f64; 3]) -> Self {
        let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], direction);
        Self {
            placement: Placement::new(origin, rot),
            ref_type: RefType::Line,
            point: Some(origin),
            direction: Some(direction),
            normal: None,
        }
    }

    /// Create an object-placement reference (has placement flag).
    pub fn object(placement: Placement) -> Self {
        Self {
            placement,
            ref_type: RefType::Anything,
            point: Some(placement.position),
            direction: None,
            normal: None,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Attachment Engine
// ═══════════════════════════════════════════════════════════════════════

/// Attachment engine — computes a Placement from reference geometry +
/// attachment mode.
///
/// Mirrors FreeCAD's `Part::AttachEngine::calculateAttachedPlacement()`.
/// Each engine variant (Plane/Line/Point) filters applicable modes, but
/// the core calculation is shared.
#[derive(Debug, Clone)]
pub struct AttachmentEngine {
    /// Current attachment mode.
    pub mode: AttachMode,
    /// Whether the attachment is reversed (flip Z and X).
    pub map_reversed: bool,
    /// Additional offset applied after the computed placement.
    pub offset: Placement,
    /// Parameter for NormalToPath (0.0 to 1.0).
    pub path_parameter: f64,
}

/// Error from attachment computation.
#[derive(Debug, Clone, PartialEq)]
pub enum AttachError {
    /// Attachment is deactivated.
    Deactivated,
    /// Not enough references for this mode.
    InsufficientReferences { mode: &'static str, need: usize, got: usize },
    /// References are of wrong type.
    WrongReferenceType { mode: &'static str, expected: &'static str },
    /// Geometric computation failed.
    ComputationFailed(String),
}

impl std::fmt::Display for AttachError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Deactivated => write!(f, "Attachment is deactivated"),
            Self::InsufficientReferences { mode, need, got } =>
                write!(f, "Mode {mode}: need {need} references, got {got}"),
            Self::WrongReferenceType { mode, expected } =>
                write!(f, "Mode {mode}: expected {expected}"),
            Self::ComputationFailed(msg) => write!(f, "Attachment failed: {msg}"),
        }
    }
}

impl std::error::Error for AttachError {}

impl Default for AttachmentEngine {
    fn default() -> Self {
        Self {
            mode: AttachMode::Deactivated,
            map_reversed: false,
            offset: Placement::identity(),
            path_parameter: 0.0,
        }
    }
}

impl AttachmentEngine {
    /// Create a new engine with the given mode.
    pub fn new(mode: AttachMode) -> Self {
        Self { mode, ..Default::default() }
    }

    /// Calculate the placement from references.
    ///
    /// This is the core of FreeCAD's `AttachEngine::calculateAttachedPlacement`.
    /// It dispatches on the mode and computes origin + orientation from
    /// the reference geometry.
    pub fn calculate(&self, refs: &[AttachRef]) -> std::result::Result<Placement, AttachError> {
        if self.mode == AttachMode::Deactivated {
            return Err(AttachError::Deactivated);
        }

        let placement = self.dispatch(refs)?;

        // Apply map_reversed: flip Z (normal) and X axes
        let placement = if self.map_reversed {
            let flip = Rotation::from_axis_angle([1.0, 0.0, 0.0], std::f64::consts::PI);
            Placement::new(placement.position, placement.rotation.compose(&flip))
        } else {
            placement
        };

        // Compose with offset
        Ok(placement.compose(&self.offset))
    }

    /// Internal dispatch on mode.
    fn dispatch(&self, refs: &[AttachRef]) -> std::result::Result<Placement, AttachError> {
        match self.mode {
            AttachMode::Deactivated => Err(AttachError::Deactivated),

            // ── Translate: origin = vertex position, orientation = identity ──
            AttachMode::Translate => {
                let r = self.need_refs(refs, 1)?;
                let pos = r[0].point.unwrap_or(r[0].placement.position);
                Ok(Placement::from_translation(pos[0], pos[1], pos[2]))
            }

            // ── ObjectXY/XZ/YZ: use object's placement axes ──
            AttachMode::ObjectXY => {
                let r = self.need_refs(refs, 1)?;
                Ok(r[0].placement)
            }
            AttachMode::ObjectXZ => {
                let r = self.need_refs(refs, 1)?;
                // X'=X, Y'=Z, Z'=-Y  → rotate -90° around X
                let swap = Rotation::from_axis_angle([1.0, 0.0, 0.0], -std::f64::consts::FRAC_PI_2);
                Ok(Placement::new(r[0].placement.position, r[0].placement.rotation.compose(&swap)))
            }
            AttachMode::ObjectYZ => {
                let r = self.need_refs(refs, 1)?;
                // X'=Y, Y'=Z, Z'=X  → rotate 90° around Y then 90° around Z
                let swap = Rotation::from_axis_angle([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2);
                Ok(Placement::new(r[0].placement.position, r[0].placement.rotation.compose(&swap)))
            }

            // ── FlatFace: align Z to face normal, X along face U-direction ──
            AttachMode::FlatFace => {
                let r = self.need_refs(refs, 1)?;
                match r[0].normal {
                    Some(n) => {
                        let origin = r[0].point.unwrap_or(r[0].placement.position);
                        let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], n);
                        Ok(Placement::new(origin, rot))
                    }
                    None => Err(AttachError::WrongReferenceType {
                        mode: "FlatFace", expected: "a planar face",
                    }),
                }
            }

            // ── TangentPlane: Z = face normal at a point ──
            AttachMode::TangentPlane => {
                // Need a face + a vertex
                if refs.len() < 2 {
                    return Err(AttachError::InsufficientReferences {
                        mode: "TangentPlane", need: 2, got: refs.len(),
                    });
                }
                let normal = refs[0].normal.ok_or(AttachError::WrongReferenceType {
                    mode: "TangentPlane", expected: "face with normal",
                })?;
                let pos = refs[1].point.unwrap_or(refs[1].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], normal);
                Ok(Placement::new(pos, rot))
            }

            // ── NormalToPath: Frenet frame at parameter along curve ──
            AttachMode::NormalToPath => {
                let r = self.need_refs(refs, 1)?;
                let dir = r[0].direction.ok_or(AttachError::WrongReferenceType {
                    mode: "NormalToPath", expected: "edge with direction",
                })?;
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                // Z = tangent direction, interpolated by path_parameter
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], dir);
                Ok(Placement::new(origin, rot))
            }

            // ── Frenet NB/TN/TB: Frenet frame variants ──
            AttachMode::FrenetNB | AttachMode::FrenetTN | AttachMode::FrenetTB => {
                let r = self.need_refs(refs, 1)?;
                let dir = r[0].direction.unwrap_or([0.0, 0.0, 1.0]);
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                // For straight edges, Frenet frame degenerates: T=dir, N and B arbitrary
                let t = normalize(dir);
                let (n, b) = perpendicular_pair(t);
                let rot = match self.mode {
                    AttachMode::FrenetNB => rotation_from_axes(n, b, t),
                    AttachMode::FrenetTN => rotation_from_axes(t, n, b),
                    AttachMode::FrenetTB => rotation_from_axes(t, b, [-n[0], -n[1], -n[2]]),
                    _ => unreachable!(),
                };
                Ok(Placement::new(origin, rot))
            }

            // ── Concentric: Z = face normal, origin = circular face center ──
            AttachMode::Concentric => {
                let r = self.need_refs(refs, 1)?;
                let normal = r[0].normal.unwrap_or([0.0, 0.0, 1.0]);
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], normal);
                Ok(Placement::new(origin, rot))
            }

            // ── ThreePointsPlane: plane through 3 points ──
            AttachMode::ThreePointsPlane => {
                let r = self.need_refs(refs, 3)?;
                let p0 = r[0].point.unwrap_or(r[0].placement.position);
                let p1 = r[1].point.unwrap_or(r[1].placement.position);
                let p2 = r[2].point.unwrap_or(r[2].placement.position);
                let v01 = sub(p1, p0);
                let v02 = sub(p2, p0);
                let normal = normalize(cross(v01, v02));
                if mag(normal) < 1e-15 {
                    return Err(AttachError::ComputationFailed(
                        "Three points are collinear".into(),
                    ));
                }
                let x_axis = normalize(v01);
                let y_axis = cross(normal, x_axis);
                let rot = rotation_from_axes(x_axis, y_axis, normal);
                Ok(Placement::new(p0, rot))
            }

            // ── ThreePointsNormal: normal to plane through 3 points ──
            AttachMode::ThreePointsNormal => {
                let r = self.need_refs(refs, 3)?;
                let p0 = r[0].point.unwrap_or(r[0].placement.position);
                let p1 = r[1].point.unwrap_or(r[1].placement.position);
                let p2 = r[2].point.unwrap_or(r[2].placement.position);
                let v01 = sub(p1, p0);
                let v02 = sub(p2, p0);
                let normal = normalize(cross(v01, v02));
                // Line along normal, centered at p0
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], normal);
                Ok(Placement::new(p0, rot))
            }

            // ── Line modes ──────────────────────────────────────────────
            AttachMode::AxisX => {
                let r = self.need_refs(refs, 1)?;
                let x = r[0].placement.rotation.transform_point([1.0, 0.0, 0.0]);
                let origin = r[0].placement.position;
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], x);
                Ok(Placement::new(origin, rot))
            }
            AttachMode::AxisY => {
                let r = self.need_refs(refs, 1)?;
                let y = r[0].placement.rotation.transform_point([0.0, 1.0, 0.0]);
                let origin = r[0].placement.position;
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], y);
                Ok(Placement::new(origin, rot))
            }
            AttachMode::AxisZ => {
                let r = self.need_refs(refs, 1)?;
                let z = r[0].placement.rotation.transform_point([0.0, 0.0, 1.0]);
                let origin = r[0].placement.position;
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], z);
                Ok(Placement::new(origin, rot))
            }

            // ── TwoPoints: line from point A to point B ──
            AttachMode::TwoPoints => {
                let r = self.need_refs(refs, 2)?;
                let p0 = r[0].point.unwrap_or(r[0].placement.position);
                let p1 = r[1].point.unwrap_or(r[1].placement.position);
                let dir = normalize(sub(p1, p0));
                if mag(dir) < 1e-15 {
                    return Err(AttachError::ComputationFailed("Points coincide".into()));
                }
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], dir);
                Ok(Placement::new(p0, rot))
            }

            // ── Tangent/Normal/Binormal on edge ──
            AttachMode::Tangent => {
                let r = self.need_refs(refs, 1)?;
                let dir = r[0].direction.unwrap_or([0.0, 0.0, 1.0]);
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], normalize(dir));
                Ok(Placement::new(origin, rot))
            }
            AttachMode::Normal => {
                let r = self.need_refs(refs, 1)?;
                let dir = r[0].direction.unwrap_or([0.0, 0.0, 1.0]);
                let (n, _) = perpendicular_pair(normalize(dir));
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], n);
                Ok(Placement::new(origin, rot))
            }
            AttachMode::Binormal => {
                let r = self.need_refs(refs, 1)?;
                let dir = r[0].direction.unwrap_or([0.0, 0.0, 1.0]);
                let (_, b) = perpendicular_pair(normalize(dir));
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], b);
                Ok(Placement::new(origin, rot))
            }

            // ── Intersection: line at intersection of two planes ──
            AttachMode::Intersection => {
                if refs.len() < 2 {
                    return Err(AttachError::InsufficientReferences {
                        mode: "Intersection", need: 2, got: refs.len(),
                    });
                }
                let n0 = refs[0].normal.ok_or(AttachError::WrongReferenceType {
                    mode: "Intersection", expected: "two faces",
                })?;
                let n1 = refs[1].normal.ok_or(AttachError::WrongReferenceType {
                    mode: "Intersection", expected: "two faces",
                })?;
                let dir = normalize(cross(n0, n1));
                if mag(dir) < 1e-15 {
                    return Err(AttachError::ComputationFailed("Planes are parallel".into()));
                }
                let origin = refs[0].point.unwrap_or(refs[0].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], dir);
                Ok(Placement::new(origin, rot))
            }

            // ── FaceNormal: line along face normal ──
            AttachMode::FaceNormal => {
                let r = self.need_refs(refs, 1)?;
                let normal = r[0].normal.ok_or(AttachError::WrongReferenceType {
                    mode: "FaceNormal", expected: "a face",
                })?;
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], normal);
                Ok(Placement::new(origin, rot))
            }

            // ── Point modes ─────────────────────────────────────────────
            AttachMode::Origin => {
                let r = self.need_refs(refs, 1)?;
                let pos = r[0].placement.position;
                Ok(Placement::from_translation(pos[0], pos[1], pos[2]))
            }
            AttachMode::Vertex => {
                let r = self.need_refs(refs, 1)?;
                let pos = r[0].point.unwrap_or(r[0].placement.position);
                Ok(Placement::from_translation(pos[0], pos[1], pos[2]))
            }
            AttachMode::CenterOfMass => {
                // Average of all reference points
                if refs.is_empty() {
                    return Err(AttachError::InsufficientReferences {
                        mode: "CenterOfMass", need: 1, got: 0,
                    });
                }
                let mut sum = [0.0_f64; 3];
                for r in refs {
                    let p = r.point.unwrap_or(r.placement.position);
                    sum[0] += p[0]; sum[1] += p[1]; sum[2] += p[2];
                }
                let n = refs.len() as f64;
                Ok(Placement::from_translation(sum[0]/n, sum[1]/n, sum[2]/n))
            }
            AttachMode::Midpoint => {
                let r = self.need_refs(refs, 2)?;
                let p0 = r[0].point.unwrap_or(r[0].placement.position);
                let p1 = r[1].point.unwrap_or(r[1].placement.position);
                Ok(Placement::from_translation(
                    (p0[0]+p1[0])/2.0, (p0[1]+p1[1])/2.0, (p0[2]+p1[2])/2.0,
                ))
            }
            AttachMode::OnEdge => {
                let r = self.need_refs(refs, 1)?;
                // Point on edge at path_parameter
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                let dir = r[0].direction.unwrap_or([0.0, 0.0, 1.0]);
                let pos = [
                    origin[0] + dir[0] * self.path_parameter,
                    origin[1] + dir[1] * self.path_parameter,
                    origin[2] + dir[2] * self.path_parameter,
                ];
                Ok(Placement::from_translation(pos[0], pos[1], pos[2]))
            }

            // ── OXY/OXZ/OYZ/OZX/OZY/OYX: axis permutation modes ──
            AttachMode::OXY => self.axis_permutation(refs, [0, 1, 2]),
            AttachMode::OXZ => self.axis_permutation(refs, [0, 2, 1]),
            AttachMode::OYZ => self.axis_permutation(refs, [1, 2, 0]),
            AttachMode::OZX => self.axis_permutation(refs, [2, 0, 1]),
            AttachMode::OZY => self.axis_permutation(refs, [2, 1, 0]),
            AttachMode::OYX => self.axis_permutation(refs, [1, 0, 2]),

            // ── ParallelPlane: same as FlatFace but at offset ──
            AttachMode::ParallelPlane => {
                let r = self.need_refs(refs, 1)?;
                let normal = r[0].normal.unwrap_or([0.0, 0.0, 1.0]);
                let origin = r[0].point.unwrap_or(r[0].placement.position);
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], normal);
                Ok(Placement::new(origin, rot))
            }

            // ── Remaining modes use same fallback pattern ──
            _ => {
                // For modes that require specific OCCT geometry analysis
                // (Focus, Directrix, Asymptote, Curvature, etc.), we use
                // the reference placement directly when detailed geometry
                // is not available. The full geometry analysis would require
                // the actual shape topology.
                if refs.is_empty() {
                    return Err(AttachError::InsufficientReferences {
                        mode: self.mode.name(), need: 1, got: 0,
                    });
                }
                let origin = refs[0].point.unwrap_or(refs[0].placement.position);
                let dir = refs[0].direction.unwrap_or(
                    refs[0].normal.unwrap_or([0.0, 0.0, 1.0])
                );
                let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], dir);
                Ok(Placement::new(origin, rot))
            }
        }
    }

    /// Axis permutation helper for O** modes.
    fn axis_permutation(&self, refs: &[AttachRef], perm: [usize; 3]) -> std::result::Result<Placement, AttachError> {
        let r = self.need_refs(refs, 1)?;
        let axes = [
            r[0].placement.rotation.transform_point([1.0, 0.0, 0.0]),
            r[0].placement.rotation.transform_point([0.0, 1.0, 0.0]),
            r[0].placement.rotation.transform_point([0.0, 0.0, 1.0]),
        ];
        let rot = rotation_from_axes(axes[perm[0]], axes[perm[1]], axes[perm[2]]);
        Ok(Placement::new(r[0].placement.position, rot))
    }

    /// Ensure at least `n` refs, returning a slice.
    fn need_refs<'a>(&self, refs: &'a [AttachRef], n: usize) -> std::result::Result<&'a [AttachRef], AttachError> {
        if refs.len() < n {
            Err(AttachError::InsufficientReferences {
                mode: self.mode.name(), need: n, got: refs.len(),
            })
        } else {
            Ok(refs)
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Vector math helpers (pure f64 arrays, no external deps)
// ═══════════════════════════════════════════════════════════════════════

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]
}

fn mag(v: [f64; 3]) -> f64 {
    (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt()
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let m = mag(v);
    if m < 1e-15 { return [0.0, 0.0, 1.0]; }
    [v[0]/m, v[1]/m, v[2]/m]
}

/// Two mutually perpendicular vectors to `v`.
fn perpendicular_pair(v: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let candidate = if v[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let n = normalize(cross(v, candidate));
    let b = cross(v, n);
    (n, b)
}

/// Build a rotation from three orthonormal axes (X, Y, Z columns).
fn rotation_from_axes(x: [f64; 3], y: [f64; 3], z: [f64; 3]) -> Rotation {
    // Rotation matrix → quaternion
    // M = [x y z] as column vectors
    let trace = x[0] + y[1] + z[2];
    if trace > 0.0 {
        let s = 0.5 / (trace + 1.0).sqrt();
        let w = 0.25 / s;
        let i = (y[2] - z[1]) * s;
        let j = (z[0] - x[2]) * s;
        let k = (x[1] - y[0]) * s;
        Rotation::from_quaternion(i, j, k, w)
    } else if x[0] > y[1] && x[0] > z[2] {
        let s = 2.0 * (1.0 + x[0] - y[1] - z[2]).sqrt();
        let w = (y[2] - z[1]) / s;
        let i = 0.25 * s;
        let j = (y[0] + x[1]) / s;
        let k = (z[0] + x[2]) / s;
        Rotation::from_quaternion(i, j, k, w)
    } else if y[1] > z[2] {
        let s = 2.0 * (1.0 + y[1] - x[0] - z[2]).sqrt();
        let w = (z[0] - x[2]) / s;
        let i = (y[0] + x[1]) / s;
        let j = 0.25 * s;
        let k = (z[1] + y[2]) / s;
        Rotation::from_quaternion(i, j, k, w)
    } else {
        let s = 2.0 * (1.0 + z[2] - x[0] - y[1]).sqrt();
        let w = (x[1] - y[0]) / s;
        let i = (z[0] + x[2]) / s;
        let j = (z[1] + y[2]) / s;
        let k = 0.25 * s;
        Rotation::from_quaternion(i, j, k, w)
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Datum Features
// ═══════════════════════════════════════════════════════════════════════

/// A datum plane — infinite reference plane.
///
/// Mirrors FreeCAD's `PartDesign::Plane` (→ `Part::Datum` → `Part::Feature`).
/// The plane is defined by its Placement: origin = point on plane,
/// Z axis = plane normal.
#[derive(Debug, Clone)]
pub struct DatumPlane {
    pub name: String,
    /// Placement of the plane (position + orientation where Z = normal).
    pub placement: Placement,
    /// Attachment engine for computing placement from references.
    pub attachment: AttachmentEngine,
    /// Display size (length & width) — visual only, plane is infinite.
    pub length: f64,
    pub width: f64,
    suppressed: bool,
}

impl DatumPlane {
    pub fn new(name: impl Into<String>, placement: Placement) -> Self {
        Self {
            name: name.into(),
            placement,
            attachment: AttachmentEngine::default(),
            length: 20.0,
            width: 20.0,
            suppressed: false,
        }
    }

    /// Create XY plane at origin.
    pub fn xy() -> Self {
        Self::new("XY_Plane", Placement::identity())
    }

    /// Create XZ plane at origin (Z up → rotate so plane normal = Y).
    pub fn xz() -> Self {
        let rot = Rotation::from_axis_angle([1.0, 0.0, 0.0], -std::f64::consts::FRAC_PI_2);
        Self::new("XZ_Plane", Placement::new([0.0; 3], rot))
    }

    /// Create YZ plane at origin (normal = X).
    pub fn yz() -> Self {
        let rot = Rotation::from_axis_angle([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2);
        Self::new("YZ_Plane", Placement::new([0.0; 3], rot))
    }

    /// Get the plane normal (Z axis of placement).
    pub fn normal(&self) -> [f64; 3] {
        self.placement.rotation.transform_point([0.0, 0.0, 1.0])
    }

    /// Get the plane origin.
    pub fn origin(&self) -> [f64; 3] {
        self.placement.position
    }

    /// Attach to references and recompute placement.
    pub fn attach(&mut self, refs: &[AttachRef]) -> std::result::Result<(), AttachError> {
        self.placement = self.attachment.calculate(refs)?;
        Ok(())
    }
}

impl Feature for DatumPlane {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "DatumPlane" }
    fn is_suppressed(&self) -> bool { self.suppressed }
    fn set_suppressed(&mut self, s: bool) { self.suppressed = s; }

    fn execute(&self, base_shape: Option<&Shape>) -> FeatureOutput {
        // Datum features pass through the base shape — they don't modify geometry.
        FeatureOutput {
            shape: base_shape.cloned(),
            add_sub_shape: None,
            status: if self.suppressed { FeatureStatus::Suppressed } else { FeatureStatus::Ok },
        }
    }
}

/// A datum line — infinite reference line.
///
/// Mirrors FreeCAD's `PartDesign::Line` (→ `Part::Datum`).
/// Defined by Placement: origin = point on line, Z axis = line direction.
#[derive(Debug, Clone)]
pub struct DatumLine {
    pub name: String,
    pub placement: Placement,
    pub attachment: AttachmentEngine,
    /// Display length — visual only.
    pub length: f64,
    suppressed: bool,
}

impl DatumLine {
    pub fn new(name: impl Into<String>, placement: Placement) -> Self {
        Self {
            name: name.into(),
            placement,
            attachment: AttachmentEngine::default(),
            length: 20.0,
            suppressed: false,
        }
    }

    /// X axis through origin.
    pub fn x_axis() -> Self {
        let rot = Rotation::from_axis_angle([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2);
        Self::new("X_Axis", Placement::new([0.0; 3], rot))
    }

    /// Y axis through origin.
    pub fn y_axis() -> Self {
        let rot = Rotation::from_axis_angle([1.0, 0.0, 0.0], -std::f64::consts::FRAC_PI_2);
        Self::new("Y_Axis", Placement::new([0.0; 3], rot))
    }

    /// Z axis through origin.
    pub fn z_axis() -> Self {
        Self::new("Z_Axis", Placement::identity())
    }

    /// Get the line direction (Z axis of placement).
    pub fn direction(&self) -> [f64; 3] {
        self.placement.rotation.transform_point([0.0, 0.0, 1.0])
    }

    /// Get the line origin.
    pub fn origin(&self) -> [f64; 3] {
        self.placement.position
    }

    /// Attach to references and recompute placement.
    pub fn attach(&mut self, refs: &[AttachRef]) -> std::result::Result<(), AttachError> {
        self.placement = self.attachment.calculate(refs)?;
        Ok(())
    }
}

impl Feature for DatumLine {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "DatumLine" }
    fn is_suppressed(&self) -> bool { self.suppressed }
    fn set_suppressed(&mut self, s: bool) { self.suppressed = s; }

    fn execute(&self, base_shape: Option<&Shape>) -> FeatureOutput {
        FeatureOutput {
            shape: base_shape.cloned(),
            add_sub_shape: None,
            status: if self.suppressed { FeatureStatus::Suppressed } else { FeatureStatus::Ok },
        }
    }
}

/// A datum point — reference point.
///
/// Mirrors FreeCAD's `PartDesign::Point` (→ `Part::Datum`).
#[derive(Debug, Clone)]
pub struct DatumPoint {
    pub name: String,
    pub placement: Placement,
    pub attachment: AttachmentEngine,
    suppressed: bool,
}

impl DatumPoint {
    pub fn new(name: impl Into<String>, position: [f64; 3]) -> Self {
        Self {
            name: name.into(),
            placement: Placement::from_translation(position[0], position[1], position[2]),
            attachment: AttachmentEngine::default(),
            suppressed: false,
        }
    }

    /// Origin point.
    pub fn origin() -> Self {
        Self::new("Origin", [0.0, 0.0, 0.0])
    }

    /// Get the point position.
    pub fn position(&self) -> [f64; 3] {
        self.placement.position
    }

    /// Attach to references and recompute placement.
    pub fn attach(&mut self, refs: &[AttachRef]) -> std::result::Result<(), AttachError> {
        self.placement = self.attachment.calculate(refs)?;
        Ok(())
    }
}

impl Feature for DatumPoint {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "DatumPoint" }
    fn is_suppressed(&self) -> bool { self.suppressed }
    fn set_suppressed(&mut self, s: bool) { self.suppressed = s; }

    fn execute(&self, base_shape: Option<&Shape>) -> FeatureOutput {
        FeatureOutput {
            shape: base_shape.cloned(),
            add_sub_shape: None,
            status: if self.suppressed { FeatureStatus::Suppressed } else { FeatureStatus::Ok },
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Coordinate System (Local CS)
// ═══════════════════════════════════════════════════════════════════════

/// A local coordinate system — defines a local frame for features.
///
/// Mirrors FreeCAD's `PartDesign::CoordinateSystem`.
/// The CS is defined by its Placement, with the three axes derived
/// from the rotation component.
#[derive(Debug, Clone)]
pub struct CoordinateSystem {
    pub name: String,
    pub placement: Placement,
    pub attachment: AttachmentEngine,
    suppressed: bool,
}

impl CoordinateSystem {
    pub fn new(name: impl Into<String>, placement: Placement) -> Self {
        Self {
            name: name.into(),
            placement,
            attachment: AttachmentEngine::default(),
            suppressed: false,
        }
    }

    /// Default world CS at origin.
    pub fn world() -> Self {
        Self::new("WorldCS", Placement::identity())
    }

    /// X axis direction in global space.
    pub fn x_axis(&self) -> [f64; 3] {
        self.placement.rotation.transform_point([1.0, 0.0, 0.0])
    }

    /// Y axis direction in global space.
    pub fn y_axis(&self) -> [f64; 3] {
        self.placement.rotation.transform_point([0.0, 1.0, 0.0])
    }

    /// Z axis direction in global space.
    pub fn z_axis(&self) -> [f64; 3] {
        self.placement.rotation.transform_point([0.0, 0.0, 1.0])
    }

    /// Origin in global space.
    pub fn origin(&self) -> [f64; 3] {
        self.placement.position
    }

    /// Transform a point from local CS to global.
    pub fn to_global(&self, local_point: [f64; 3]) -> [f64; 3] {
        self.placement.transform_point(local_point)
    }

    /// Transform a point from global to local CS.
    pub fn to_local(&self, global_point: [f64; 3]) -> [f64; 3] {
        self.placement.inverse().transform_point(global_point)
    }

    /// Attach to references and recompute placement.
    pub fn attach(&mut self, refs: &[AttachRef]) -> std::result::Result<(), AttachError> {
        self.placement = self.attachment.calculate(refs)?;
        Ok(())
    }
}

impl Feature for CoordinateSystem {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "CoordinateSystem" }
    fn is_suppressed(&self) -> bool { self.suppressed }
    fn set_suppressed(&mut self, s: bool) { self.suppressed = s; }

    fn execute(&self, base_shape: Option<&Shape>) -> FeatureOutput {
        FeatureOutput {
            shape: base_shape.cloned(),
            add_sub_shape: None,
            status: if self.suppressed { FeatureStatus::Suppressed } else { FeatureStatus::Ok },
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  ShapeBinder
// ═══════════════════════════════════════════════════════════════════════

/// A shape binder — references geometry from another body.
///
/// Mirrors FreeCAD's `PartDesign::ShapeBinder`.
/// Stores a reference shape that can be used as a sketch plane, axis, etc.
/// The shape is imported into the current body's coordinate system via
/// a placement transform.
#[derive(Debug, Clone)]
pub struct ShapeBinder {
    pub name: String,
    /// The referenced shape (imported copy).
    pub bound_shape: Option<Shape>,
    /// Placement of the source in the target body's frame.
    pub placement: Placement,
    /// Whether to trace (auto-update) when source changes.
    pub trace_support: bool,
    suppressed: bool,
}

impl ShapeBinder {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            bound_shape: None,
            placement: Placement::identity(),
            trace_support: false,
            suppressed: false,
        }
    }

    /// Bind a shape with a placement transform.
    ///
    /// `source_shape` is the shape from the other body.
    /// `source_placement` is the global placement of the source body.
    /// `target_placement` is the global placement of this body.
    pub fn bind(
        &mut self,
        source_shape: Shape,
        source_placement: Placement,
        target_placement: Placement,
    ) {
        // Transform from source body space to target body space:
        // target_inverse * source
        let relative = target_placement.inverse().compose(&source_placement);
        self.placement = relative;
        self.bound_shape = Some(source_shape);
    }

    /// Bind a shape directly (already in local coordinates).
    pub fn bind_direct(&mut self, shape: Shape) {
        self.bound_shape = Some(shape);
        self.placement = Placement::identity();
    }

    /// Get the bound shape.
    pub fn shape(&self) -> Option<&Shape> {
        self.bound_shape.as_ref()
    }
}

impl Feature for ShapeBinder {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "ShapeBinder" }
    fn is_suppressed(&self) -> bool { self.suppressed }
    fn set_suppressed(&mut self, s: bool) { self.suppressed = s; }

    fn execute(&self, base_shape: Option<&Shape>) -> FeatureOutput {
        if self.suppressed {
            return FeatureOutput {
                shape: base_shape.cloned(),
                add_sub_shape: None,
                status: FeatureStatus::Suppressed,
            };
        }

        // ShapeBinder passes through base shape — it provides reference
        // geometry but doesn't modify the solid.
        FeatureOutput {
            shape: base_shape.cloned(),
            add_sub_shape: self.bound_shape.clone(),
            status: FeatureStatus::Ok,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Tests
// ═══════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature::Feature;

    const TOL: f64 = 1e-10;

    fn assert_near(a: f64, b: f64, label: &str) {
        assert!((a - b).abs() < TOL, "{label}: {a} vs {b}");
    }

    fn assert_vec_near(a: [f64; 3], b: [f64; 3], label: &str) {
        assert_near(a[0], b[0], &format!("{label}[0]"));
        assert_near(a[1], b[1], &format!("{label}[1]"));
        assert_near(a[2], b[2], &format!("{label}[2]"));
    }

    // ── DatumPlane ────────────────────────────────────────

    #[test]
    fn datum_plane_xy() {
        let p = DatumPlane::xy();
        assert_vec_near(p.normal(), [0.0, 0.0, 1.0], "XY normal");
        assert_vec_near(p.origin(), [0.0, 0.0, 0.0], "XY origin");
        assert_eq!(p.type_name(), "DatumPlane");
    }

    #[test]
    fn datum_plane_xz() {
        let p = DatumPlane::xz();
        let n = p.normal();
        // XZ plane normal should be Y direction (approximately)
        assert!(n[1].abs() > 0.99, "XZ normal should be ~Y: {:?}", n);
    }

    #[test]
    fn datum_plane_yz() {
        let p = DatumPlane::yz();
        let n = p.normal();
        // YZ plane normal should be X direction
        assert!(n[0].abs() > 0.99, "YZ normal should be ~X: {:?}", n);
    }

    #[test]
    fn datum_plane_custom() {
        let rot = Rotation::from_axis_angle([1.0, 0.0, 0.0], std::f64::consts::FRAC_PI_4);
        let p = DatumPlane::new("Tilted", Placement::new([1.0, 2.0, 3.0], rot));
        assert_vec_near(p.origin(), [1.0, 2.0, 3.0], "custom origin");
        assert_eq!(p.name(), "Tilted");
    }

    #[test]
    fn datum_plane_passthrough() {
        let p = DatumPlane::xy();
        let output = p.execute(None);
        assert!(output.shape.is_none());
        assert_eq!(output.status, FeatureStatus::Ok);
    }

    // ── DatumLine ────────────────────────────────────────

    #[test]
    fn datum_line_axes() {
        let x = DatumLine::x_axis();
        let y = DatumLine::y_axis();
        let z = DatumLine::z_axis();

        let xd = x.direction();
        let yd = y.direction();
        let zd = z.direction();

        assert!(xd[0].abs() > 0.99, "X axis dir: {:?}", xd);
        assert!(yd[1].abs() > 0.99, "Y axis dir: {:?}", yd);
        assert!(zd[2].abs() > 0.99, "Z axis dir: {:?}", zd);
    }

    #[test]
    fn datum_line_custom() {
        let rot = Rotation::from_two_vectors([0.0, 0.0, 1.0], [1.0, 1.0, 0.0]);
        let l = DatumLine::new("Diagonal", Placement::new([5.0, 5.0, 0.0], rot));
        assert_eq!(l.name(), "Diagonal");
        let d = l.direction();
        // Should point roughly along (1,1,0) normalized
        let s2 = 1.0 / 2.0_f64.sqrt();
        assert!((d[0] - s2).abs() < 0.01, "dir X: {}", d[0]);
        assert!((d[1] - s2).abs() < 0.01, "dir Y: {}", d[1]);
    }

    // ── DatumPoint ───────────────────────────────────────

    #[test]
    fn datum_point_origin() {
        let p = DatumPoint::origin();
        assert_vec_near(p.position(), [0.0, 0.0, 0.0], "origin pos");
        assert_eq!(p.type_name(), "DatumPoint");
    }

    #[test]
    fn datum_point_custom() {
        let p = DatumPoint::new("P1", [10.0, 20.0, 30.0]);
        assert_vec_near(p.position(), [10.0, 20.0, 30.0], "custom pos");
    }

    // ── CoordinateSystem ─────────────────────────────────

    #[test]
    fn coordinate_system_world() {
        let cs = CoordinateSystem::world();
        assert_vec_near(cs.x_axis(), [1.0, 0.0, 0.0], "world X");
        assert_vec_near(cs.y_axis(), [0.0, 1.0, 0.0], "world Y");
        assert_vec_near(cs.z_axis(), [0.0, 0.0, 1.0], "world Z");
        assert_vec_near(cs.origin(), [0.0, 0.0, 0.0], "world origin");
    }

    #[test]
    fn coordinate_system_transform() {
        let cs = CoordinateSystem::new(
            "Offset",
            Placement::from_translation(10.0, 20.0, 30.0),
        );
        let global = cs.to_global([1.0, 0.0, 0.0]);
        assert_vec_near(global, [11.0, 20.0, 30.0], "to_global");

        let local = cs.to_local([11.0, 20.0, 30.0]);
        assert_vec_near(local, [1.0, 0.0, 0.0], "to_local");
    }

    #[test]
    fn coordinate_system_rotated() {
        let rot = Rotation::from_axis_angle([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_2);
        let cs = CoordinateSystem::new("Rotated90", Placement::new([0.0; 3], rot));
        let x = cs.x_axis();
        // 90° around Z: X→Y
        assert!((x[1] - 1.0).abs() < 0.01, "rotated X should be Y: {:?}", x);
    }

    // ── ShapeBinder ──────────────────────────────────────

    #[test]
    fn shape_binder_empty() {
        let sb = ShapeBinder::new("Binder1");
        assert!(sb.shape().is_none());
        assert_eq!(sb.type_name(), "ShapeBinder");
    }

    #[test]
    fn shape_binder_bind_direct() {
        use truck_modeling::*;
        let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
        let v1 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
        let v2 = builder::vertex(Point3::new(1.0, 1.0, 0.0));
        let v3 = builder::vertex(Point3::new(0.0, 1.0, 0.0));
        let wire = Wire::from_iter(vec![
            &builder::line(&v0, &v1),
            &builder::line(&v1, &v2),
            &builder::line(&v2, &v3),
            &builder::line(&v3, &v0),
        ]);
        let shape = cadora_brep::extrude(&wire, Vector3::new(0.0, 0.0, 1.0));

        let mut sb = ShapeBinder::new("Binder");
        sb.bind_direct(shape);
        assert!(sb.shape().is_some());
    }

    // ── AttachmentEngine ─────────────────────────────────

    #[test]
    fn attach_deactivated() {
        let eng = AttachmentEngine::default();
        assert_eq!(eng.mode, AttachMode::Deactivated);
        let result = eng.calculate(&[]);
        assert!(result.is_err());
    }

    #[test]
    fn attach_translate() {
        let eng = AttachmentEngine::new(AttachMode::Translate);
        let refs = [AttachRef::point([5.0, 10.0, 15.0])];
        let p = eng.calculate(&refs).unwrap();
        assert_vec_near(p.position, [5.0, 10.0, 15.0], "translate pos");
    }

    #[test]
    fn attach_flat_face() {
        let eng = AttachmentEngine::new(AttachMode::FlatFace);
        let refs = [AttachRef::flat_face([1.0, 2.0, 3.0], [0.0, 1.0, 0.0])];
        let p = eng.calculate(&refs).unwrap();
        assert_vec_near(p.position, [1.0, 2.0, 3.0], "flat_face pos");
        // Z axis should be the face normal (0,1,0)
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        assert!(z[1].abs() > 0.99, "Z should be ~Y: {:?}", z);
    }

    #[test]
    fn attach_three_points_plane() {
        let eng = AttachmentEngine::new(AttachMode::ThreePointsPlane);
        let refs = [
            AttachRef::point([0.0, 0.0, 0.0]),
            AttachRef::point([1.0, 0.0, 0.0]),
            AttachRef::point([0.0, 1.0, 0.0]),
        ];
        let p = eng.calculate(&refs).unwrap();
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        // Normal of XY triangle should be Z
        assert!(z[2].abs() > 0.99, "three-point normal should be Z: {:?}", z);
    }

    #[test]
    fn attach_two_points_line() {
        let eng = AttachmentEngine::new(AttachMode::TwoPoints);
        let refs = [
            AttachRef::point([0.0, 0.0, 0.0]),
            AttachRef::point([0.0, 0.0, 5.0]),
        ];
        let p = eng.calculate(&refs).unwrap();
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        // Direction should be Z
        assert!(z[2].abs() > 0.99, "two-point dir should be Z: {:?}", z);
    }

    #[test]
    fn attach_object_xy() {
        let eng = AttachmentEngine::new(AttachMode::ObjectXY);
        let obj_place = Placement::from_translation(10.0, 20.0, 30.0);
        let refs = [AttachRef::object(obj_place)];
        let p = eng.calculate(&refs).unwrap();
        assert_vec_near(p.position, [10.0, 20.0, 30.0], "ObjectXY pos");
    }

    #[test]
    fn attach_vertex() {
        let eng = AttachmentEngine::new(AttachMode::Vertex);
        let refs = [AttachRef::point([3.0, 4.0, 5.0])];
        let p = eng.calculate(&refs).unwrap();
        assert_vec_near(p.position, [3.0, 4.0, 5.0], "vertex pos");
    }

    #[test]
    fn attach_midpoint() {
        let eng = AttachmentEngine::new(AttachMode::Midpoint);
        let refs = [
            AttachRef::point([0.0, 0.0, 0.0]),
            AttachRef::point([10.0, 10.0, 10.0]),
        ];
        let p = eng.calculate(&refs).unwrap();
        assert_vec_near(p.position, [5.0, 5.0, 5.0], "midpoint");
    }

    #[test]
    fn attach_center_of_mass() {
        let eng = AttachmentEngine::new(AttachMode::CenterOfMass);
        let refs = [
            AttachRef::point([0.0, 0.0, 0.0]),
            AttachRef::point([6.0, 0.0, 0.0]),
            AttachRef::point([0.0, 6.0, 0.0]),
        ];
        let p = eng.calculate(&refs).unwrap();
        assert_vec_near(p.position, [2.0, 2.0, 0.0], "center of mass");
    }

    #[test]
    fn attach_reversed() {
        let mut eng = AttachmentEngine::new(AttachMode::FlatFace);
        eng.map_reversed = true;
        let refs = [AttachRef::flat_face([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])];
        let p = eng.calculate(&refs).unwrap();
        // Reversed: Z should flip to -Z
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        assert!(z[2] < -0.99, "reversed Z should be ~-Z: {:?}", z);
    }

    #[test]
    fn attach_with_offset() {
        let mut eng = AttachmentEngine::new(AttachMode::Translate);
        eng.offset = Placement::from_translation(1.0, 2.0, 3.0);
        let refs = [AttachRef::point([10.0, 20.0, 30.0])];
        let p = eng.calculate(&refs).unwrap();
        assert_vec_near(p.position, [11.0, 22.0, 33.0], "offset pos");
    }

    #[test]
    fn attach_intersection() {
        let eng = AttachmentEngine::new(AttachMode::Intersection);
        let refs = [
            AttachRef::flat_face([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]), // XY plane
            AttachRef::flat_face([0.0, 0.0, 0.0], [0.0, 1.0, 0.0]), // XZ plane
        ];
        let p = eng.calculate(&refs).unwrap();
        // Intersection of XY and XZ planes is the X axis
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        assert!(z[0].abs() > 0.99, "intersection should be X axis: {:?}", z);
    }

    #[test]
    fn attach_face_normal() {
        let eng = AttachmentEngine::new(AttachMode::FaceNormal);
        let refs = [AttachRef::flat_face([1.0, 2.0, 3.0], [1.0, 0.0, 0.0])];
        let p = eng.calculate(&refs).unwrap();
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        assert!(z[0].abs() > 0.99, "face normal should be X: {:?}", z);
    }

    #[test]
    fn attach_insufficient_refs() {
        let eng = AttachmentEngine::new(AttachMode::ThreePointsPlane);
        let refs = [AttachRef::point([0.0, 0.0, 0.0])];
        let result = eng.calculate(&refs);
        assert!(result.is_err());
        if let Err(AttachError::InsufficientReferences { need, got, .. }) = result {
            assert_eq!(need, 3);
            assert_eq!(got, 1);
        }
    }

    // ── Datum attachment integration ─────────────────────

    #[test]
    fn datum_plane_attach_flat_face() {
        let mut plane = DatumPlane::xy();
        plane.attachment.mode = AttachMode::FlatFace;
        let refs = [AttachRef::flat_face([0.0, 0.0, 5.0], [0.0, 0.0, 1.0])];
        plane.attach(&refs).unwrap();
        assert_vec_near(plane.origin(), [0.0, 0.0, 5.0], "attached plane origin");
    }

    #[test]
    fn datum_line_attach_two_points() {
        let mut line = DatumLine::z_axis();
        line.attachment.mode = AttachMode::TwoPoints;
        let refs = [
            AttachRef::point([1.0, 0.0, 0.0]),
            AttachRef::point([1.0, 0.0, 10.0]),
        ];
        line.attach(&refs).unwrap();
        assert_vec_near(line.origin(), [1.0, 0.0, 0.0], "attached line origin");
        let d = line.direction();
        assert!(d[2].abs() > 0.99, "attached line should go Z: {:?}", d);
    }

    #[test]
    fn datum_point_attach_vertex() {
        let mut pt = DatumPoint::origin();
        pt.attachment.mode = AttachMode::Vertex;
        let refs = [AttachRef::point([7.0, 8.0, 9.0])];
        pt.attach(&refs).unwrap();
        assert_vec_near(pt.position(), [7.0, 8.0, 9.0], "attached point");
    }

    #[test]
    fn coordinate_system_attach() {
        let mut cs = CoordinateSystem::world();
        cs.attachment.mode = AttachMode::ObjectXY;
        let place = Placement::from_pos_axis_angle(
            [5.0, 5.0, 5.0],
            [0.0, 0.0, 1.0],
            std::f64::consts::FRAC_PI_2,
        );
        let refs = [AttachRef::object(place)];
        cs.attach(&refs).unwrap();
        assert_vec_near(cs.origin(), [5.0, 5.0, 5.0], "CS origin");
    }

    // ── Body integration (datum features in body) ────────

    #[test]
    fn datum_in_body() {
        use crate::body::Body;

        let mut body = Body::new("TestBody");

        // Add datum plane
        let dp = DatumPlane::new("SketchPlane", Placement::from_translation(0.0, 0.0, 10.0));
        let id = body.add_feature(Box::new(dp));

        assert_eq!(body.feature_count(), 1);
        let f = body.get_feature(id).unwrap();
        assert_eq!(f.type_name(), "DatumPlane");
    }

    // ── AttachMode properties ────────────────────────────

    #[test]
    fn attach_mode_categories() {
        assert!(AttachMode::FlatFace.is_plane_mode());
        assert!(!AttachMode::FlatFace.is_line_mode());
        assert!(!AttachMode::FlatFace.is_point_mode());

        assert!(!AttachMode::TwoPoints.is_plane_mode());
        assert!(AttachMode::TwoPoints.is_line_mode());
        assert!(!AttachMode::TwoPoints.is_point_mode());

        assert!(!AttachMode::Vertex.is_plane_mode());
        assert!(!AttachMode::Vertex.is_line_mode());
        assert!(AttachMode::Vertex.is_point_mode());
    }

    // ── Axis modes ───────────────────────────────────────

    #[test]
    fn attach_axis_x() {
        let eng = AttachmentEngine::new(AttachMode::AxisX);
        let refs = [AttachRef::object(Placement::identity())];
        let p = eng.calculate(&refs).unwrap();
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        assert!(z[0].abs() > 0.99, "AxisX: Z should map to X: {:?}", z);
    }

    #[test]
    fn attach_axis_y() {
        let eng = AttachmentEngine::new(AttachMode::AxisY);
        let refs = [AttachRef::object(Placement::identity())];
        let p = eng.calculate(&refs).unwrap();
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        assert!(z[1].abs() > 0.99, "AxisY: Z should map to Y: {:?}", z);
    }

    #[test]
    fn attach_axis_z() {
        let eng = AttachmentEngine::new(AttachMode::AxisZ);
        let refs = [AttachRef::object(Placement::identity())];
        let p = eng.calculate(&refs).unwrap();
        let z = p.rotation.transform_point([0.0, 0.0, 1.0]);
        assert!(z[2].abs() > 0.99, "AxisZ: Z should map to Z: {:?}", z);
    }

    // ── Vector math helpers ──────────────────────────────

    #[test]
    fn test_cross_product() {
        let r = cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        assert_vec_near(r, [0.0, 0.0, 1.0], "X × Y = Z");
    }

    #[test]
    fn test_rotation_from_axes() {
        let rot = rotation_from_axes([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
        // Identity rotation
        let x = rot.transform_point([1.0, 0.0, 0.0]);
        let y = rot.transform_point([0.0, 1.0, 0.0]);
        let z = rot.transform_point([0.0, 0.0, 1.0]);
        assert_vec_near(x, [1.0, 0.0, 0.0], "identity X");
        assert_vec_near(y, [0.0, 1.0, 0.0], "identity Y");
        assert_vec_near(z, [0.0, 0.0, 1.0], "identity Z");
    }
}
