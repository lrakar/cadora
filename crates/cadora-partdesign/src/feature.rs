//! Feature definitions: base feature, additive/subtractive, profile-based.
//!
//! Mirrors FreeCAD class hierarchy:
//! Feature → FeatureAddSub → ProfileBased → Pad/Pocket/Revolution/Groove

use cadora_brep::Shape;
use cadora_brep::{
    BooleanOp, ExtrudeMode, RevolveMode,
    extrude, extrude_symmetric, extrude_two_sides,
    revolve, revolve_symmetric, revolve_two_angles,
    boolean, fillet, chamfer, mirror, linear_pattern, polar_pattern,
};
use truck_modeling::*;
use cgmath::InnerSpace;

/// Unique identifier for a feature within a body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FeatureId(pub u64);

/// Whether a feature adds or subtracts material.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddSubType {
    Additive,
    Subtractive,
}

/// Status of a feature after execution.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FeatureStatus {
    /// Successfully computed.
    Ok,
    /// Feature is suppressed — passes through base shape.
    Suppressed,
    /// Execution failed with an error.
    Error(String),
}

/// The output of executing a feature.
#[derive(Debug, Clone)]
pub struct FeatureOutput {
    /// The resulting shape (None if this is the first feature and it failed).
    pub shape: Option<Shape>,
    /// The tool shape (for additive/subtractive features, this is the shape
    /// before the boolean operation).
    pub add_sub_shape: Option<Shape>,
    /// Execution status.
    pub status: FeatureStatus,
}

/// Base trait for all features.
///
/// Mirrors FreeCAD's `PartDesign::Feature` virtual interface.
pub trait Feature: std::fmt::Debug {
    /// Human-readable name of this feature.
    fn name(&self) -> &str;

    /// The type name (e.g., "Pad", "Pocket", "Fillet").
    fn type_name(&self) -> &str;

    /// Execute the feature given the previous feature's shape.
    /// `base_shape` is None for the first feature in the body.
    fn execute(&self, base_shape: Option<&Shape>) -> FeatureOutput;

    /// Whether this feature is suppressed.
    fn is_suppressed(&self) -> bool;

    /// Set suppression state.
    fn set_suppressed(&mut self, suppressed: bool);
}

// ═══════════════════════════════════════════════════════════════════
//  Concrete Feature Types
// ═══════════════════════════════════════════════════════════════════

/// A pad (additive extrusion) feature.
#[derive(Debug, Clone)]
pub struct PadFeature {
    pub name: String,
    /// The profile wire to extrude.
    pub profile: Wire,
    /// Extrusion direction (typically sketch normal).
    pub direction: Vector3,
    /// Extrusion mode.
    pub mode: ExtrudeMode,
    /// Whether reversed.
    pub reversed: bool,
    /// Suppressed.
    suppressed: bool,
}

impl PadFeature {
    pub fn new(name: impl Into<String>, profile: Wire, direction: Vector3, mode: ExtrudeMode) -> Self {
        Self {
            name: name.into(),
            profile,
            direction,
            mode,
            reversed: false,
            suppressed: false,
        }
    }
}

impl Feature for PadFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Pad" }
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

        let dir = if self.reversed {
            Vector3::new(-self.direction.x, -self.direction.y, -self.direction.z)
        } else {
            self.direction
        };

        // Generate the tool shape
        let tool = generate_extrusion(&self.profile, dir, &self.mode);

        match base_shape {
            Some(base) => {
                // Fuse tool with base (additive)
                match boolean(base, &tool, BooleanOp::Fuse) {
                    Ok(result) => FeatureOutput {
                        shape: Some(result),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Ok,
                    },
                    Err(e) => FeatureOutput {
                        shape: Some(base.clone()),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Error(format!("Pad boolean fuse failed: {e}")),
                    },
                }
            }
            None => {
                // First feature — tool IS the result
                FeatureOutput {
                    shape: Some(tool.clone()),
                    add_sub_shape: Some(tool),
                    status: FeatureStatus::Ok,
                }
            }
        }
    }
}

/// A pocket (subtractive extrusion) feature.
#[derive(Debug, Clone)]
pub struct PocketFeature {
    pub name: String,
    pub profile: Wire,
    pub direction: Vector3,
    pub mode: ExtrudeMode,
    pub reversed: bool,
    suppressed: bool,
}

impl PocketFeature {
    pub fn new(name: impl Into<String>, profile: Wire, direction: Vector3, mode: ExtrudeMode) -> Self {
        Self {
            name: name.into(),
            profile,
            direction,
            mode,
            reversed: false,
            suppressed: false,
        }
    }
}

impl Feature for PocketFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Pocket" }
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

        let dir = if self.reversed {
            Vector3::new(-self.direction.x, -self.direction.y, -self.direction.z)
        } else {
            self.direction
        };

        let tool = generate_extrusion(&self.profile, dir, &self.mode);

        match base_shape {
            Some(base) => {
                match boolean(base, &tool, BooleanOp::Cut) {
                    Ok(result) => FeatureOutput {
                        shape: Some(result),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Ok,
                    },
                    Err(e) => FeatureOutput {
                        shape: Some(base.clone()),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Error(format!("Pocket boolean cut failed: {e}")),
                    },
                }
            }
            None => FeatureOutput {
                shape: None,
                add_sub_shape: Some(tool),
                status: FeatureStatus::Error("Pocket requires a base shape".to_string()),
            },
        }
    }
}

/// A revolution (additive) feature.
#[derive(Debug, Clone)]
pub struct RevolutionFeature {
    pub name: String,
    pub profile: Wire,
    pub axis_origin: Point3,
    pub axis_direction: Vector3,
    pub mode: RevolveMode,
    pub reversed: bool,
    suppressed: bool,
}

impl RevolutionFeature {
    pub fn new(
        name: impl Into<String>,
        profile: Wire,
        axis_origin: Point3,
        axis_direction: Vector3,
        mode: RevolveMode,
    ) -> Self {
        Self {
            name: name.into(),
            profile,
            axis_origin,
            axis_direction,
            mode,
            reversed: false,
            suppressed: false,
        }
    }
}

impl Feature for RevolutionFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Revolution" }
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

        let tool = generate_revolution(&self.profile, self.axis_origin, self.axis_direction, &self.mode);

        match base_shape {
            Some(base) => {
                match boolean(base, &tool, BooleanOp::Fuse) {
                    Ok(result) => FeatureOutput {
                        shape: Some(result),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Ok,
                    },
                    Err(e) => FeatureOutput {
                        shape: Some(base.clone()),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Error(format!("Revolution fuse failed: {e}")),
                    },
                }
            }
            None => FeatureOutput {
                shape: Some(tool.clone()),
                add_sub_shape: Some(tool),
                status: FeatureStatus::Ok,
            },
        }
    }
}

/// A groove (subtractive revolution) feature.
#[derive(Debug, Clone)]
pub struct GrooveFeature {
    pub name: String,
    pub profile: Wire,
    pub axis_origin: Point3,
    pub axis_direction: Vector3,
    pub mode: RevolveMode,
    pub reversed: bool,
    suppressed: bool,
}

impl GrooveFeature {
    pub fn new(
        name: impl Into<String>,
        profile: Wire,
        axis_origin: Point3,
        axis_direction: Vector3,
        mode: RevolveMode,
    ) -> Self {
        Self {
            name: name.into(),
            profile,
            axis_origin,
            axis_direction,
            mode,
            reversed: false,
            suppressed: false,
        }
    }
}

impl Feature for GrooveFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Groove" }
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

        let tool = generate_revolution(&self.profile, self.axis_origin, self.axis_direction, &self.mode);

        match base_shape {
            Some(base) => {
                match boolean(base, &tool, BooleanOp::Cut) {
                    Ok(result) => FeatureOutput {
                        shape: Some(result),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Ok,
                    },
                    Err(e) => FeatureOutput {
                        shape: Some(base.clone()),
                        add_sub_shape: Some(tool),
                        status: FeatureStatus::Error(format!("Groove cut failed: {e}")),
                    },
                }
            }
            None => FeatureOutput {
                shape: None,
                add_sub_shape: Some(tool),
                status: FeatureStatus::Error("Groove requires a base shape".to_string()),
            },
        }
    }
}

/// A fillet (dress-up) feature.
#[derive(Debug, Clone)]
pub struct FilletFeature {
    pub name: String,
    /// Edge indices to fillet.
    pub edge_indices: Vec<usize>,
    /// Fillet radius.
    pub radius: f64,
    suppressed: bool,
}

impl FilletFeature {
    pub fn new(name: impl Into<String>, edge_indices: Vec<usize>, radius: f64) -> Self {
        Self { name: name.into(), edge_indices, radius, suppressed: false }
    }
}

impl Feature for FilletFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Fillet" }
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

        let base = match base_shape {
            Some(b) => b,
            None => return FeatureOutput {
                shape: None,
                add_sub_shape: None,
                status: FeatureStatus::Error("Fillet requires a base shape".to_string()),
            },
        };

        match fillet(base, &self.edge_indices, self.radius) {
            Ok(result) => FeatureOutput {
                shape: Some(result),
                add_sub_shape: None,
                status: FeatureStatus::Ok,
            },
            Err(e) => FeatureOutput {
                shape: Some(base.clone()),
                add_sub_shape: None,
                status: FeatureStatus::Error(format!("Fillet failed: {e}")),
            },
        }
    }
}

/// A chamfer (dress-up) feature.
#[derive(Debug, Clone)]
pub struct ChamferFeature {
    pub name: String,
    pub edge_indices: Vec<usize>,
    pub distance: f64,
    suppressed: bool,
}

impl ChamferFeature {
    pub fn new(name: impl Into<String>, edge_indices: Vec<usize>, distance: f64) -> Self {
        Self { name: name.into(), edge_indices, distance, suppressed: false }
    }
}

impl Feature for ChamferFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Chamfer" }
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

        let base = match base_shape {
            Some(b) => b,
            None => return FeatureOutput {
                shape: None,
                add_sub_shape: None,
                status: FeatureStatus::Error("Chamfer requires a base shape".to_string()),
            },
        };

        match chamfer(base, &self.edge_indices, self.distance) {
            Ok(result) => FeatureOutput {
                shape: Some(result),
                add_sub_shape: None,
                status: FeatureStatus::Ok,
            },
            Err(e) => FeatureOutput {
                shape: Some(base.clone()),
                add_sub_shape: None,
                status: FeatureStatus::Error(format!("Chamfer failed: {e}")),
            },
        }
    }
}

/// A mirror feature.
#[derive(Debug, Clone)]
pub struct MirrorFeature {
    pub name: String,
    pub plane_origin: Point3,
    pub plane_normal: Vector3,
    suppressed: bool,
}

impl MirrorFeature {
    pub fn new(name: impl Into<String>, plane_origin: Point3, plane_normal: Vector3) -> Self {
        Self { name: name.into(), plane_origin, plane_normal, suppressed: false }
    }
}

impl Feature for MirrorFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Mirrored" }
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

        let base = match base_shape {
            Some(b) => b,
            None => return FeatureOutput {
                shape: None,
                add_sub_shape: None,
                status: FeatureStatus::Error("Mirror requires a base shape".to_string()),
            },
        };

        let mirrored = mirror(base, self.plane_origin, self.plane_normal);
        match boolean(base, &mirrored, BooleanOp::Fuse) {
            Ok(result) => FeatureOutput {
                shape: Some(result),
                add_sub_shape: Some(mirrored),
                status: FeatureStatus::Ok,
            },
            Err(e) => FeatureOutput {
                shape: Some(base.clone()),
                add_sub_shape: Some(mirrored),
                status: FeatureStatus::Error(format!("Mirror fuse failed: {e}")),
            },
        }
    }
}

/// A linear pattern feature.
#[derive(Debug, Clone)]
pub struct LinearPatternFeature {
    pub name: String,
    pub direction: Vector3,
    pub count: usize,
    pub spacing: f64,
    suppressed: bool,
}

impl LinearPatternFeature {
    pub fn new(name: impl Into<String>, direction: Vector3, count: usize, spacing: f64) -> Self {
        Self { name: name.into(), direction, count, spacing, suppressed: false }
    }
}

impl Feature for LinearPatternFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "LinearPattern" }
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

        let base = match base_shape {
            Some(b) => b,
            None => return FeatureOutput {
                shape: None,
                add_sub_shape: None,
                status: FeatureStatus::Error("LinearPattern requires a base shape".to_string()),
            },
        };

        match linear_pattern(base, self.direction, self.count, self.spacing) {
            Ok(result) => FeatureOutput {
                shape: Some(result),
                add_sub_shape: None,
                status: FeatureStatus::Ok,
            },
            Err(e) => FeatureOutput {
                shape: Some(base.clone()),
                add_sub_shape: None,
                status: FeatureStatus::Error(format!("LinearPattern failed: {e}")),
            },
        }
    }
}

/// A polar pattern feature.
#[derive(Debug, Clone)]
pub struct PolarPatternFeature {
    pub name: String,
    pub axis_origin: Point3,
    pub axis_direction: Vector3,
    pub count: usize,
    pub total_angle: f64,
    suppressed: bool,
}

impl PolarPatternFeature {
    pub fn new(
        name: impl Into<String>,
        axis_origin: Point3,
        axis_direction: Vector3,
        count: usize,
        total_angle: f64,
    ) -> Self {
        Self {
            name: name.into(),
            axis_origin,
            axis_direction,
            count,
            total_angle,
            suppressed: false,
        }
    }
}

impl Feature for PolarPatternFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "PolarPattern" }
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

        let base = match base_shape {
            Some(b) => b,
            None => return FeatureOutput {
                shape: None,
                add_sub_shape: None,
                status: FeatureStatus::Error("PolarPattern requires a base shape".to_string()),
            },
        };

        match polar_pattern(base, self.axis_origin, self.axis_direction, self.count, self.total_angle) {
            Ok(result) => FeatureOutput {
                shape: Some(result),
                add_sub_shape: None,
                status: FeatureStatus::Ok,
            },
            Err(e) => FeatureOutput {
                shape: Some(base.clone()),
                add_sub_shape: None,
                status: FeatureStatus::Error(format!("PolarPattern failed: {e}")),
            },
        }
    }
}

/// Hole type specification.
#[derive(Debug, Clone)]
pub enum HoleType {
    /// Simple cylindrical hole.
    Simple {
        /// Hole diameter.
        diameter: f64,
        /// Hole depth (None = through all).
        depth: Option<f64>,
    },
    /// Countersink hole (conical top + cylindrical bottom).
    Countersink {
        diameter: f64,
        depth: Option<f64>,
        /// Countersink diameter at the surface.
        countersink_diameter: f64,
        /// Countersink angle (total cone angle, typically 90° or 82°).
        countersink_angle: f64,
    },
    /// Counterbore hole (cylindrical recess + smaller through hole).
    Counterbore {
        diameter: f64,
        depth: Option<f64>,
        /// Counterbore diameter.
        counterbore_diameter: f64,
        /// Counterbore depth.
        counterbore_depth: f64,
    },
}

/// A hole feature — revolves a 2D hole profile and cuts from base.
#[derive(Debug, Clone)]
pub struct HoleFeature {
    pub name: String,
    /// Center position of the hole on the face surface.
    pub position: Point3,
    /// Hole axis direction (typically face normal, pointing into material).
    pub direction: Vector3,
    /// Hole specification.
    pub hole_type: HoleType,
    suppressed: bool,
}

impl HoleFeature {
    pub fn new(name: impl Into<String>, position: Point3, direction: Vector3, hole_type: HoleType) -> Self {
        Self {
            name: name.into(),
            position,
            direction,
            hole_type,
            suppressed: false,
        }
    }

    /// Build the 2D profile wire for the hole cross-section,
    /// then revolve it to create the tool solid.
    fn build_tool(&self) -> Shape {
        let dir = self.direction.normalize();
        let large_depth = 1000.0;

        match &self.hole_type {
            HoleType::Simple { diameter, depth } => {
                let r = diameter / 2.0;
                let d = depth.unwrap_or(large_depth);
                // Profile: rectangle from (r, 0) to (0, -d) revolved around axis
                let p0 = self.position;
                let perp = perpendicular_to(dir);
                let pt0 = point_offset(p0, perp, r);
                let pt1 = point_offset(pt0, dir, -d);
                let pt2 = point_offset(p0, dir, -d);
                let pt3 = p0;

                let v0 = builder::vertex(pt0);
                let v1 = builder::vertex(pt1);
                let v2 = builder::vertex(pt2);
                let v3 = builder::vertex(pt3);
                let wire = Wire::from_iter(vec![
                    &builder::line(&v0, &v1),
                    &builder::line(&v1, &v2),
                    &builder::line(&v2, &v3),
                    &builder::line(&v3, &v0),
                ]);
                revolve(&wire, self.position, self.direction, std::f64::consts::TAU)
            }
            HoleType::Countersink { diameter, depth, countersink_diameter, countersink_angle } => {
                let r = diameter / 2.0;
                let r_cs = countersink_diameter / 2.0;
                let d = depth.unwrap_or(large_depth);
                // Countersink depth from geometry
                let cs_depth = (r_cs - r) / (countersink_angle / 2.0).tan();

                // Profile (L-to-R, bottom-to-top):
                // (0, -d) -> (r, -d) -> (r, -cs_depth) -> (r_cs, 0) -> (0, 0)
                let perp = perpendicular_to(dir);
                let p0 = self.position;

                let pt0 = point_offset(p0, dir, -d);
                let pt1 = point_offset(point_offset(p0, dir, -d), perp, r);
                let pt2 = point_offset(point_offset(p0, dir, -cs_depth), perp, r);
                let pt3 = point_offset(p0, perp, r_cs);
                let pt4 = p0;

                let v0 = builder::vertex(pt0);
                let v1 = builder::vertex(pt1);
                let v2 = builder::vertex(pt2);
                let v3 = builder::vertex(pt3);
                let v4 = builder::vertex(pt4);
                let wire = Wire::from_iter(vec![
                    &builder::line(&v0, &v1),
                    &builder::line(&v1, &v2),
                    &builder::line(&v2, &v3),
                    &builder::line(&v3, &v4),
                    &builder::line(&v4, &v0),
                ]);
                revolve(&wire, self.position, self.direction, std::f64::consts::TAU)
            }
            HoleType::Counterbore { diameter, depth, counterbore_diameter, counterbore_depth } => {
                let r = diameter / 2.0;
                let r_cb = counterbore_diameter / 2.0;
                let d = depth.unwrap_or(large_depth);

                // Profile (L-shaped cross-section):
                // (0, -d) -> (r, -d) -> (r, -cb_depth) -> (r_cb, -cb_depth) -> (r_cb, 0) -> (0, 0)
                let perp = perpendicular_to(dir);
                let p0 = self.position;

                let pt0 = point_offset(p0, dir, -d);
                let pt1 = point_offset(point_offset(p0, dir, -d), perp, r);
                let pt2 = point_offset(point_offset(p0, dir, -*counterbore_depth), perp, r);
                let pt3 = point_offset(point_offset(p0, dir, -*counterbore_depth), perp, r_cb);
                let pt4 = point_offset(p0, perp, r_cb);
                let pt5 = p0;

                let v0 = builder::vertex(pt0);
                let v1 = builder::vertex(pt1);
                let v2 = builder::vertex(pt2);
                let v3 = builder::vertex(pt3);
                let v4 = builder::vertex(pt4);
                let v5 = builder::vertex(pt5);
                let wire = Wire::from_iter(vec![
                    &builder::line(&v0, &v1),
                    &builder::line(&v1, &v2),
                    &builder::line(&v2, &v3),
                    &builder::line(&v3, &v4),
                    &builder::line(&v4, &v5),
                    &builder::line(&v5, &v0),
                ]);
                revolve(&wire, self.position, self.direction, std::f64::consts::TAU)
            }
        }
    }
}

impl Feature for HoleFeature {
    fn name(&self) -> &str { &self.name }
    fn type_name(&self) -> &str { "Hole" }
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

        let base = match base_shape {
            Some(b) => b,
            None => return FeatureOutput {
                shape: None,
                add_sub_shape: None,
                status: FeatureStatus::Error("Hole requires a base shape".to_string()),
            },
        };

        let tool = self.build_tool();

        match boolean(base, &tool, BooleanOp::Cut) {
            Ok(result) => FeatureOutput {
                shape: Some(result),
                add_sub_shape: Some(tool),
                status: FeatureStatus::Ok,
            },
            Err(e) => FeatureOutput {
                shape: Some(base.clone()),
                add_sub_shape: Some(tool),
                status: FeatureStatus::Error(format!("Hole cut failed: {e}")),
            },
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Helper functions
// ═══════════════════════════════════════════════════════════════════

/// Compute a vector perpendicular to the given direction.
fn perpendicular_to(dir: Vector3) -> Vector3 {
    let candidate = if dir.x.abs() < 0.9 {
        Vector3::unit_x()
    } else {
        Vector3::unit_y()
    };
    dir.cross(candidate).normalize()
}

/// Offset a point along a direction by a distance.
fn point_offset(p: Point3, dir: Vector3, dist: f64) -> Point3 {
    Point3::new(p.x + dir.x * dist, p.y + dir.y * dist, p.z + dir.z * dist)
}

fn generate_extrusion(profile: &Wire, direction: Vector3, mode: &ExtrudeMode) -> Shape {
    match mode {
        ExtrudeMode::Length(len) => {
            let dir = Vector3::new(
                direction.x * len / direction.magnitude(),
                direction.y * len / direction.magnitude(),
                direction.z * len / direction.magnitude(),
            );
            extrude(profile, dir)
        }
        ExtrudeMode::Symmetric(len) => {
            let dir_norm = Vector3::new(
                direction.x / direction.magnitude(),
                direction.y / direction.magnitude(),
                direction.z / direction.magnitude(),
            );
            extrude_symmetric(profile, dir_norm, *len)
        }
        ExtrudeMode::TwoSides { length1, length2 } => {
            let dir_norm = Vector3::new(
                direction.x / direction.magnitude(),
                direction.y / direction.magnitude(),
                direction.z / direction.magnitude(),
            );
            extrude_two_sides(profile, dir_norm, *length1, *length2)
        }
        ExtrudeMode::ThroughAll => {
            let len = 1000.0; // large distance
            let dir = Vector3::new(
                direction.x * len / direction.magnitude(),
                direction.y * len / direction.magnitude(),
                direction.z * len / direction.magnitude(),
            );
            extrude(profile, dir)
        }
    }
}

fn generate_revolution(profile: &Wire, origin: Point3, axis: Vector3, mode: &RevolveMode) -> Shape {
    match mode {
        RevolveMode::Angle(angle) => revolve(profile, origin, axis, *angle),
        RevolveMode::Symmetric(angle) => revolve_symmetric(profile, origin, axis, *angle),
        RevolveMode::TwoAngles { angle1, angle2 } => {
            revolve_two_angles(profile, origin, axis, *angle1, *angle2)
        }
        RevolveMode::ThroughAll => {
            revolve(profile, origin, axis, std::f64::consts::TAU)
        }
    }
}
