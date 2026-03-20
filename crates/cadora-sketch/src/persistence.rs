//! Serialization, undo/redo, and validation (Phase A7).
//!
//! Provides snapshot-based persistence and undo/redo for sketch state,
//! plus validation methods to detect and clean up invalid state.

use crate::constraint::SketchConstraint;
use crate::external::ExternalFlags;
use crate::geometry::{GeoDef, GeoType};
use crate::sketch::Sketch;
use crate::types::*;

// ===========================================================================
// Serialization data structures
// ===========================================================================

/// Serializable snapshot of a complete sketch state.
#[derive(Debug, Clone)]
pub struct SketchSnapshot {
    pub geometries: Vec<SerializedGeometry>,
    pub constraints: Vec<SerializedConstraint>,
    pub externals: Vec<SerializedExternal>,
}

/// Serializable geometry element.
#[derive(Debug, Clone)]
pub struct SerializedGeometry {
    pub geo_type: String,
    pub params: Vec<f64>,
    pub construction: bool,
    pub blocked: bool,
}

/// Serializable constraint.
#[derive(Debug, Clone)]
pub struct SerializedConstraint {
    pub constraint_type: String,
    pub elements: Vec<(i32, String)>, // (geo_id, point_pos)
    pub value: Option<f64>,
    pub is_driving: bool,
    pub is_active: bool,
}

/// Serializable external geometry reference.
#[derive(Debug, Clone)]
pub struct SerializedExternal {
    pub source_key: String,
    pub geo: SerializedGeometry,
    pub flags: SerializedExternalFlags,
}

/// Serializable external flags.
#[derive(Debug, Clone)]
pub struct SerializedExternalFlags {
    pub defining: bool,
    pub frozen: bool,
    pub detached: bool,
    pub missing: bool,
}

// ===========================================================================
// Serialization implementation
// ===========================================================================

fn geo_type_to_string(geo: &GeoType) -> (&'static str, Vec<f64>) {
    match geo {
        GeoType::Point { x, y } => ("Point", vec![*x, *y]),
        GeoType::Line { x1, y1, x2, y2 } => ("Line", vec![*x1, *y1, *x2, *y2]),
        GeoType::Circle { cx, cy, radius } => ("Circle", vec![*cx, *cy, *radius]),
        GeoType::Arc { cx, cy, radius, start_angle, end_angle } => {
            ("Arc", vec![*cx, *cy, *radius, *start_angle, *end_angle])
        }
        GeoType::Ellipse { cx, cy, fx, fy, radmin } => {
            ("Ellipse", vec![*cx, *cy, *fx, *fy, *radmin])
        }
        GeoType::ArcOfEllipse { cx, cy, fx, fy, radmin, start_angle, end_angle } => {
            ("ArcOfEllipse", vec![*cx, *cy, *fx, *fy, *radmin, *start_angle, *end_angle])
        }
        GeoType::Hyperbola { cx, cy, fx, fy, radmin } => {
            ("Hyperbola", vec![*cx, *cy, *fx, *fy, *radmin])
        }
        GeoType::ArcOfHyperbola { cx, cy, fx, fy, radmin, start_angle, end_angle } => {
            ("ArcOfHyperbola", vec![*cx, *cy, *fx, *fy, *radmin, *start_angle, *end_angle])
        }
        GeoType::Parabola { vx, vy, fx, fy } => ("Parabola", vec![*vx, *vy, *fx, *fy]),
        GeoType::ArcOfParabola { vx, vy, fx, fy, start_angle, end_angle } => {
            ("ArcOfParabola", vec![*vx, *vy, *fx, *fy, *start_angle, *end_angle])
        }
        GeoType::BSpline { .. } => ("BSpline", vec![]), // BSpline needs special handling
    }
}

fn string_to_geo_type(name: &str, params: &[f64]) -> Option<GeoType> {
    match name {
        "Point" if params.len() >= 2 => Some(GeoType::Point { x: params[0], y: params[1] }),
        "Line" if params.len() >= 4 => Some(GeoType::Line {
            x1: params[0], y1: params[1], x2: params[2], y2: params[3],
        }),
        "Circle" if params.len() >= 3 => Some(GeoType::Circle {
            cx: params[0], cy: params[1], radius: params[2],
        }),
        "Arc" if params.len() >= 5 => Some(GeoType::Arc {
            cx: params[0], cy: params[1], radius: params[2],
            start_angle: params[3], end_angle: params[4],
        }),
        "Ellipse" if params.len() >= 5 => Some(GeoType::Ellipse {
            cx: params[0], cy: params[1], fx: params[2], fy: params[3], radmin: params[4],
        }),
        "ArcOfEllipse" if params.len() >= 7 => Some(GeoType::ArcOfEllipse {
            cx: params[0], cy: params[1], fx: params[2], fy: params[3],
            radmin: params[4], start_angle: params[5], end_angle: params[6],
        }),
        "Hyperbola" if params.len() >= 5 => Some(GeoType::Hyperbola {
            cx: params[0], cy: params[1], fx: params[2], fy: params[3], radmin: params[4],
        }),
        "ArcOfHyperbola" if params.len() >= 7 => Some(GeoType::ArcOfHyperbola {
            cx: params[0], cy: params[1], fx: params[2], fy: params[3],
            radmin: params[4], start_angle: params[5], end_angle: params[6],
        }),
        "Parabola" if params.len() >= 4 => Some(GeoType::Parabola {
            vx: params[0], vy: params[1], fx: params[2], fy: params[3],
        }),
        "ArcOfParabola" if params.len() >= 6 => Some(GeoType::ArcOfParabola {
            vx: params[0], vy: params[1], fx: params[2], fy: params[3],
            start_angle: params[4], end_angle: params[5],
        }),
        _ => None,
    }
}

fn constraint_type_to_string(ct: ConstraintType) -> &'static str {
    match ct {
        ConstraintType::Coincident => "Coincident",
        ConstraintType::Horizontal => "Horizontal",
        ConstraintType::Vertical => "Vertical",
        ConstraintType::Parallel => "Parallel",
        ConstraintType::Perpendicular => "Perpendicular",
        ConstraintType::Tangent => "Tangent",
        ConstraintType::Equal => "Equal",
        ConstraintType::PointOnObject => "PointOnObject",
        ConstraintType::Symmetric => "Symmetric",
        ConstraintType::InternalAlignment => "InternalAlignment",
        ConstraintType::Block => "Block",
        ConstraintType::Distance => "Distance",
        ConstraintType::DistanceX => "DistanceX",
        ConstraintType::DistanceY => "DistanceY",
        ConstraintType::Angle => "Angle",
        ConstraintType::Radius => "Radius",
        ConstraintType::Diameter => "Diameter",
        ConstraintType::SnellsLaw => "SnellsLaw",
        ConstraintType::Weight => "Weight",
    }
}

fn string_to_constraint_type(s: &str) -> Option<ConstraintType> {
    match s {
        "Coincident" => Some(ConstraintType::Coincident),
        "Horizontal" => Some(ConstraintType::Horizontal),
        "Vertical" => Some(ConstraintType::Vertical),
        "Parallel" => Some(ConstraintType::Parallel),
        "Perpendicular" => Some(ConstraintType::Perpendicular),
        "Tangent" => Some(ConstraintType::Tangent),
        "Equal" => Some(ConstraintType::Equal),
        "PointOnObject" => Some(ConstraintType::PointOnObject),
        "Symmetric" => Some(ConstraintType::Symmetric),
        "InternalAlignment" => Some(ConstraintType::InternalAlignment),
        "Block" => Some(ConstraintType::Block),
        "Distance" => Some(ConstraintType::Distance),
        "DistanceX" => Some(ConstraintType::DistanceX),
        "DistanceY" => Some(ConstraintType::DistanceY),
        "Angle" => Some(ConstraintType::Angle),
        "Radius" => Some(ConstraintType::Radius),
        "Diameter" => Some(ConstraintType::Diameter),
        "SnellsLaw" => Some(ConstraintType::SnellsLaw),
        "Weight" => Some(ConstraintType::Weight),
        _ => None,
    }
}

fn point_pos_to_string(pos: PointPos) -> &'static str {
    match pos {
        PointPos::None => "None",
        PointPos::Start => "Start",
        PointPos::End => "End",
        PointPos::Mid => "Mid",
    }
}

fn string_to_point_pos(s: &str) -> PointPos {
    match s {
        "Start" => PointPos::Start,
        "End" => PointPos::End,
        "Mid" => PointPos::Mid,
        _ => PointPos::None,
    }
}

// ===========================================================================
// Validation types
// ===========================================================================

/// Issues detected during sketch validation.
#[derive(Debug, Clone)]
pub struct ValidationIssue {
    pub kind: ValidationKind,
    pub description: String,
}

/// Kinds of validation issues.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidationKind {
    /// Constraint references a non-existent geometry.
    InvalidGeoRef,
    /// Geometry has zero-length edge or degenerate shape.
    DegenerateGeometry,
    /// External geometry reference is missing.
    MissingExternal,
}

// ===========================================================================
// Sketch implementation
// ===========================================================================

impl Sketch {
    // -----------------------------------------------------------------------
    // Serialization (Phase A7)
    // -----------------------------------------------------------------------

    /// Create a serializable snapshot of the entire sketch state.
    pub fn save_snapshot(&self) -> SketchSnapshot {
        let geometries = self
            .geometries()
            .map(|(_, gd)| {
                let (name, params) = geo_type_to_string(&gd.geo);
                SerializedGeometry {
                    geo_type: name.to_string(),
                    params,
                    construction: gd.mode.construction,
                    blocked: gd.mode.blocked,
                }
            })
            .collect();

        let constraints = self
            .constraints
            .iter()
            .map(|c| SerializedConstraint {
                constraint_type: constraint_type_to_string(c.constraint_type).to_string(),
                elements: c
                    .elements
                    .iter()
                    .map(|e| (e.geo_id, point_pos_to_string(e.pos).to_string()))
                    .collect(),
                value: c.value,
                is_driving: c.is_driving,
                is_active: c.is_active,
            })
            .collect();

        let externals = self
            .external_refs
            .iter()
            .filter_map(|r| {
                let gd = self.get_geometry(r.geo_id)?;
                let (name, params) = geo_type_to_string(&gd.geo);
                Some(SerializedExternal {
                    source_key: r.source_key.clone(),
                    geo: SerializedGeometry {
                        geo_type: name.to_string(),
                        params,
                        construction: gd.mode.construction,
                        blocked: gd.mode.blocked,
                    },
                    flags: SerializedExternalFlags {
                        defining: r.flags.defining,
                        frozen: r.flags.frozen,
                        detached: r.flags.detached,
                        missing: r.flags.missing,
                    },
                })
            })
            .collect();

        SketchSnapshot {
            geometries,
            constraints,
            externals,
        }
    }

    /// Restore sketch state from a snapshot, replacing all current state.
    pub fn load_snapshot(&mut self, snapshot: &SketchSnapshot) -> bool {
        // Clear current state
        self.store.clear();
        self.constraints.clear();
        self.external_refs.clear();
        self.last_status = None;
        self.last_diagnosis = None;

        // Restore geometries
        for sg in &snapshot.geometries {
            let geo = match string_to_geo_type(&sg.geo_type, &sg.params) {
                Some(g) => g,
                None => return false,
            };
            let mut gd = GeoDef::new(geo);
            gd.mode.construction = sg.construction;
            gd.mode.blocked = sg.blocked;
            self.add_geometry(gd);
        }

        // Restore constraints
        for sc in &snapshot.constraints {
            let ct = match string_to_constraint_type(&sc.constraint_type) {
                Some(t) => t,
                None => return false,
            };
            let elements: Vec<GeoElementId> = sc
                .elements
                .iter()
                .map(|(gid, pos)| GeoElementId::new(*gid, string_to_point_pos(pos)))
                .collect();

            let constraint = if let Some(val) = sc.value {
                let mut c = SketchConstraint::dimensional(ct, elements, val);
                c.is_driving = sc.is_driving;
                c.is_active = sc.is_active;
                c
            } else {
                let mut c = SketchConstraint::geometric(ct, elements);
                c.is_driving = sc.is_driving;
                c.is_active = sc.is_active;
                c
            };
            self.add_constraint(constraint);
        }

        // Restore externals
        for se in &snapshot.externals {
            let geo = match string_to_geo_type(&se.geo.geo_type, &se.geo.params) {
                Some(g) => g,
                None => return false,
            };
            let geo_id = self.add_external(se.source_key.clone(), geo);
            self.set_external_flags(
                geo_id,
                ExternalFlags {
                    defining: se.flags.defining,
                    frozen: se.flags.frozen,
                    detached: se.flags.detached,
                    missing: se.flags.missing,
                },
            );
        }

        true
    }

    // -----------------------------------------------------------------------
    // Undo/Redo (Phase A7)
    // -----------------------------------------------------------------------

    /// Push the current state onto the undo stack.
    ///
    /// Call this before making changes to enable undo.
    pub fn push_undo(&mut self) {
        let snapshot = self.save_snapshot();
        self.undo_stack.push(snapshot);
        // Clear redo stack when new changes are made
        self.redo_stack.clear();
        // Limit stack size
        if self.undo_stack.len() > self.max_undo {
            self.undo_stack.remove(0);
        }
    }

    /// Undo the last operation, restoring the previous state.
    ///
    /// Returns true if undo was performed.
    pub fn undo(&mut self) -> bool {
        if let Some(old_state) = self.undo_stack.pop() {
            // Save current state to redo stack
            let current = self.save_snapshot();
            self.redo_stack.push(current);
            // Restore old state
            self.load_snapshot(&old_state);
            true
        } else {
            false
        }
    }

    /// Redo the last undone operation.
    ///
    /// Returns true if redo was performed.
    pub fn redo(&mut self) -> bool {
        if let Some(new_state) = self.redo_stack.pop() {
            // Save current state to undo stack
            let current = self.save_snapshot();
            self.undo_stack.push(current);
            // Restore the redo state
            self.load_snapshot(&new_state);
            true
        } else {
            false
        }
    }

    /// Returns true if undo is available.
    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    /// Returns true if redo is available.
    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    /// Set the maximum number of undo steps.
    pub fn set_max_undo(&mut self, max: usize) {
        self.max_undo = max;
        while self.undo_stack.len() > max {
            self.undo_stack.remove(0);
        }
    }

    // -----------------------------------------------------------------------
    // Validation (Phase A7)
    // -----------------------------------------------------------------------

    /// Validate the sketch and return any issues found.
    ///
    /// Checks for:
    /// - Constraints referencing non-existent geometry
    /// - Degenerate geometry (zero-length lines, zero-radius circles)
    /// - Missing external geometry references
    pub fn validate(&self) -> Vec<ValidationIssue> {
        let mut issues = Vec::new();

        // Check constraint references
        let geo_count = self.geometry_count() as GeoId;
        let ext_count = self.external_count() as GeoId;

        for (i, c) in self.constraints.iter().enumerate() {
            for e in &c.elements {
                let valid = if e.geo_id >= 0 {
                    e.geo_id < geo_count
                } else if e.geo_id >= -2 {
                    true // axis references are always valid
                } else {
                    // External: -(geo_id + 3) must be < ext_count
                    let ext_idx = -(e.geo_id + 3);
                    ext_idx >= 0 && ext_idx < ext_count
                };

                if !valid {
                    issues.push(ValidationIssue {
                        kind: ValidationKind::InvalidGeoRef,
                        description: format!(
                            "Constraint {} references non-existent geometry {}",
                            i, e.geo_id
                        ),
                    });
                }
            }
        }

        // Check for degenerate geometry
        let tol = 1e-10;
        for (gid, gd) in self.geometries() {
            match &gd.geo {
                GeoType::Line { x1, y1, x2, y2 } => {
                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    if dx * dx + dy * dy < tol * tol {
                        issues.push(ValidationIssue {
                            kind: ValidationKind::DegenerateGeometry,
                            description: format!("Geometry {} is a zero-length line", gid),
                        });
                    }
                }
                GeoType::Circle { radius, .. } | GeoType::Arc { radius, .. } => {
                    if *radius < tol {
                        issues.push(ValidationIssue {
                            kind: ValidationKind::DegenerateGeometry,
                            description: format!(
                                "Geometry {} has zero or negative radius",
                                gid
                            ),
                        });
                    }
                }
                _ => {}
            }
        }

        // Check for missing externals
        for ext_ref in &self.external_refs {
            if ext_ref.flags.missing {
                issues.push(ValidationIssue {
                    kind: ValidationKind::MissingExternal,
                    description: format!(
                        "External reference '{}' is missing",
                        ext_ref.source_key
                    ),
                });
            }
        }

        issues
    }

    /// Remove constraints that reference non-existent geometry.
    ///
    /// Returns the number of constraints removed.
    pub fn cleanup_invalid_constraints(&mut self) -> usize {
        let geo_count = self.geometry_count() as GeoId;
        let ext_count = self.external_count() as GeoId;
        let before = self.constraints.len();

        self.constraints.retain(|c| {
            c.elements.iter().all(|e| {
                if e.geo_id >= 0 {
                    e.geo_id < geo_count
                } else if e.geo_id >= -2 {
                    true
                } else {
                    let ext_idx = -(e.geo_id + 3);
                    ext_idx >= 0 && ext_idx < ext_count
                }
            })
        });

        let removed = before - self.constraints.len();
        if removed > 0 {
            self.last_status = None;
            self.last_diagnosis = None;
        }
        removed
    }

    /// Remove degenerate geometry (zero-length lines, zero-radius circles).
    ///
    /// Also removes constraints referencing removed geometry.
    /// Returns the number of geometries removed.
    pub fn remove_degenerate_geometry(&mut self) -> usize {
        let tol = 1e-10;
        let mut to_remove = Vec::new();

        for (gid, gd) in self.geometries() {
            let degenerate = match &gd.geo {
                GeoType::Line { x1, y1, x2, y2 } => {
                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    dx * dx + dy * dy < tol * tol
                }
                GeoType::Circle { radius, .. } | GeoType::Arc { radius, .. } => {
                    *radius < tol
                }
                _ => false,
            };
            if degenerate {
                to_remove.push(gid);
            }
        }

        // Remove in reverse order to avoid index shifting issues
        to_remove.sort();
        to_remove.reverse();
        for gid in &to_remove {
            self.del_geometry(*gid);
        }

        to_remove.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshot_roundtrip() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0,
            y1: 0.0,
            x2: 10.0,
            y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Circle {
            cx: 5.0,
            cy: 5.0,
            radius: 3.0,
        }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::Radius,
            vec![GeoElementId::edge(1)],
            3.0,
        ));

        let snapshot = sketch.save_snapshot();

        let mut sketch2 = Sketch::new();
        assert!(sketch2.load_snapshot(&snapshot));

        assert_eq!(sketch2.geometry_count(), 2);
        assert_eq!(sketch2.constraint_count(), 2);

        // Verify geometry values
        let g = sketch2.get_geometry(0).unwrap();
        if let GeoType::Line { x2, .. } = &g.geo {
            assert!((*x2 - 10.0).abs() < 1e-10);
        }
    }

    #[test]
    fn undo_redo_basic() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));

        // Save state, then modify
        sketch.push_undo();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 1.0, y: 1.0 }));
        assert_eq!(sketch.geometry_count(), 2);

        // Undo → back to 1 geometry
        assert!(sketch.undo());
        assert_eq!(sketch.geometry_count(), 1);

        // Redo → back to 2
        assert!(sketch.redo());
        assert_eq!(sketch.geometry_count(), 2);
    }

    #[test]
    fn undo_clears_redo_on_new_action() {
        let mut sketch = Sketch::new();
        sketch.push_undo();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));

        sketch.push_undo();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 1.0, y: 1.0 }));

        // Undo once
        sketch.undo();
        assert!(sketch.can_redo());

        // New action clears redo
        sketch.push_undo();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 2.0, y: 2.0 }));
        assert!(!sketch.can_redo());
    }

    #[test]
    fn validate_detects_invalid_refs() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));

        // Add a constraint referencing non-existent geometry
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(0), GeoElementId::mid(5)],
        ));

        let issues = sketch.validate();
        assert!(issues.iter().any(|i| i.kind == ValidationKind::InvalidGeoRef));
    }

    #[test]
    fn validate_detects_degenerate_line() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 5.0,
            y1: 5.0,
            x2: 5.0,
            y2: 5.0,
        }));

        let issues = sketch.validate();
        assert!(issues
            .iter()
            .any(|i| i.kind == ValidationKind::DegenerateGeometry));
    }

    #[test]
    fn cleanup_invalid_constraints() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(0), GeoElementId::mid(99)],
        ));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));

        let removed = sketch.cleanup_invalid_constraints();
        assert_eq!(removed, 1);
        assert_eq!(sketch.constraint_count(), 1);
    }

    #[test]
    fn remove_degenerate_geometry() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0,
            y1: 0.0,
            x2: 10.0,
            y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 5.0,
            y1: 5.0,
            x2: 5.0,
            y2: 5.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Circle {
            cx: 0.0,
            cy: 0.0,
            radius: 0.0,
        }));

        let removed = sketch.remove_degenerate_geometry();
        assert_eq!(removed, 2);
        assert_eq!(sketch.geometry_count(), 1);
    }

    #[test]
    fn snapshot_with_external_geometry() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_external(
            "Part.Edge1".to_string(),
            GeoType::Line {
                x1: 0.0,
                y1: 0.0,
                x2: 10.0,
                y2: 0.0,
            },
        );

        let snapshot = sketch.save_snapshot();
        assert_eq!(snapshot.geometries.len(), 1);
        assert_eq!(snapshot.externals.len(), 1);

        let mut sketch2 = Sketch::new();
        sketch2.load_snapshot(&snapshot);
        assert_eq!(sketch2.geometry_count(), 1);
        assert_eq!(sketch2.external_count(), 1);
    }

    #[test]
    fn validate_missing_external() {
        let mut sketch = Sketch::new();
        let ext_id = sketch.add_external(
            "Part.Edge1".to_string(),
            GeoType::Point { x: 0.0, y: 0.0 },
        );
        sketch.set_external_flags(
            ext_id,
            ExternalFlags {
                missing: true,
                ..ExternalFlags::default()
            },
        );

        let issues = sketch.validate();
        assert!(issues
            .iter()
            .any(|i| i.kind == ValidationKind::MissingExternal));
    }
}
