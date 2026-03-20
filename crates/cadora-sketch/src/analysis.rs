//! Auto-constraint detection and application (Phase A5).
//!
//! Analyses sketch geometry to detect missing coincident, vertical/horizontal,
//! and equality constraints — mirroring FreeCAD's SketchAnalysis.

use crate::constraint::SketchConstraint;
use crate::geometry::GeoType;
use crate::sketch::Sketch;
use crate::types::*;

/// A detected missing constraint, ready to be applied to the sketch.
#[derive(Debug, Clone)]
pub struct DetectedConstraint {
    pub constraint_type: ConstraintType,
    pub first: GeoElementId,
    pub second: GeoElementId,
}

impl Sketch {
    // -----------------------------------------------------------------------
    // Detect missing coincident points (Phase A5)
    // -----------------------------------------------------------------------

    /// Detect pairs of points that are geometrically close but lack a
    /// Coincident constraint.
    ///
    /// `precision` — maximum per-axis distance to consider coincident.
    /// Default FreeCAD value: 1e-4.
    ///
    /// Mirrors FreeCAD `SketchAnalysis::detectMissingPointOnPointConstraints`.
    pub fn detect_missing_coincident(
        &self,
        precision: f64,
    ) -> Vec<DetectedConstraint> {
        // Collect all geometry endpoints (start/end/mid)
        let mut vertices: Vec<(GeoElementId, f64, f64)> = Vec::new();

        for (geo_id, geo_def) in self.store.iter() {
            if geo_def.mode.construction {
                continue;
            }
            match &geo_def.geo {
                GeoType::Point { x, y } => {
                    vertices.push((GeoElementId::mid(geo_id), *x, *y));
                }
                GeoType::Line { x1, y1, x2, y2 } => {
                    vertices.push((GeoElementId::start(geo_id), *x1, *y1));
                    vertices.push((GeoElementId::end(geo_id), *x2, *y2));
                }
                GeoType::Circle { cx, cy, .. } => {
                    vertices.push((GeoElementId::mid(geo_id), *cx, *cy));
                }
                GeoType::Arc { cx, cy, radius, start_angle, end_angle } => {
                    let sx = *cx + radius * start_angle.cos();
                    let sy = *cy + radius * start_angle.sin();
                    let ex = *cx + radius * end_angle.cos();
                    let ey = *cy + radius * end_angle.sin();
                    vertices.push((GeoElementId::start(geo_id), sx, sy));
                    vertices.push((GeoElementId::end(geo_id), ex, ey));
                    vertices.push((GeoElementId::mid(geo_id), *cx, *cy));
                }
                _ => {}
            }
        }

        // Sort by x, then y (enables adjacent-pair detection)
        vertices.sort_by(|a, b| {
            a.1.partial_cmp(&b.1)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
        });

        // Collect existing coincident pairs to avoid duplicates
        let existing: Vec<(GeoElementId, GeoElementId)> = self
            .constraints
            .iter()
            .filter(|c| c.is_active && c.constraint_type == ConstraintType::Coincident)
            .filter_map(|c| Some((c.first()?, c.second()?)))
            .collect();

        let is_already_constrained = |a: GeoElementId, b: GeoElementId| -> bool {
            existing.iter().any(|(e1, e2)| {
                (*e1 == a && *e2 == b) || (*e1 == b && *e2 == a)
            })
        };

        // Find close pairs via sorted-sweep
        let mut detected = Vec::new();
        for i in 0..vertices.len() {
            for j in (i + 1)..vertices.len() {
                let (ei, xi, yi) = vertices[i];
                let (ej, xj, yj) = vertices[j];

                // Since sorted by x, once x-gap exceeds precision we can stop
                if (xj - xi).abs() > precision {
                    break;
                }
                if (yj - yi).abs() > precision {
                    continue;
                }
                // Same geometry — skip
                if ei.geo_id == ej.geo_id {
                    continue;
                }
                if !is_already_constrained(ei, ej) {
                    detected.push(DetectedConstraint {
                        constraint_type: ConstraintType::Coincident,
                        first: ei,
                        second: ej,
                    });
                }
            }
        }

        detected
    }

    // -----------------------------------------------------------------------
    // Detect missing vertical / horizontal (Phase A5)
    // -----------------------------------------------------------------------

    /// Detect line segments that are nearly vertical or horizontal but lack
    /// the corresponding constraint.
    ///
    /// `angle_precision` — maximum deviation angle (radians) from axis.
    /// Default FreeCAD value: π/8 ≈ 22.5°.
    ///
    /// Mirrors FreeCAD `SketchAnalysis::detectMissingVerticalHorizontalConstraints`.
    pub fn detect_missing_vertical_horizontal(
        &self,
        angle_precision: f64,
    ) -> Vec<DetectedConstraint> {
        let mut detected = Vec::new();

        // Collect existing H/V constraints
        let has_h_or_v = |geo_id: GeoId, ct: ConstraintType| -> bool {
            self.constraints.iter().any(|c| {
                c.is_active
                    && c.constraint_type == ct
                    && c.first().is_some_and(|e| e.geo_id == geo_id && e.pos == PointPos::None)
            })
        };

        for (geo_id, geo_def) in self.store.iter() {
            if let GeoType::Line { x1, y1, x2, y2 } = &geo_def.geo {
                let dx = x2 - x1;
                let dy = y2 - y1;
                let len_sq = dx * dx + dy * dy;
                if len_sq < 1e-30 {
                    continue;
                }

                // Check horizontal: |dy/dx| < tan(angle_precision)
                // Equivalently: |dy| < |dx| * tan(angle_precision)
                let tan_prec = angle_precision.tan();
                if dy.abs() < dx.abs() * tan_prec && dx.abs() > 1e-10 {
                    if !has_h_or_v(geo_id, ConstraintType::Horizontal) {
                        detected.push(DetectedConstraint {
                            constraint_type: ConstraintType::Horizontal,
                            first: GeoElementId::edge(geo_id),
                            second: GeoElementId::edge(geo_id), // unused
                        });
                        continue; // don't double-detect
                    }
                }

                // Check vertical: |dx/dy| < tan(angle_precision)
                if dx.abs() < dy.abs() * tan_prec && dy.abs() > 1e-10 {
                    if !has_h_or_v(geo_id, ConstraintType::Vertical) {
                        detected.push(DetectedConstraint {
                            constraint_type: ConstraintType::Vertical,
                            first: GeoElementId::edge(geo_id),
                            second: GeoElementId::edge(geo_id),
                        });
                    }
                }
            }
        }

        detected
    }

    // -----------------------------------------------------------------------
    // Detect missing equality (Phase A5)
    // -----------------------------------------------------------------------

    /// Detect segments / radii that have nearly equal measures but lack an
    /// Equal constraint.
    ///
    /// `precision` — maximum absolute difference to consider equal.
    ///
    /// Mirrors FreeCAD `SketchAnalysis::detectMissingEqualityConstraints`.
    pub fn detect_missing_equality(
        &self,
        precision: f64,
    ) -> Vec<DetectedConstraint> {
        // Collect line lengths
        let mut line_lengths: Vec<(GeoId, f64)> = Vec::new();
        let mut circle_radii: Vec<(GeoId, f64)> = Vec::new();

        for (geo_id, geo_def) in self.store.iter() {
            match &geo_def.geo {
                GeoType::Line { x1, y1, x2, y2 } => {
                    let len = ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
                    line_lengths.push((geo_id, len));
                }
                GeoType::Circle { radius, .. } | GeoType::Arc { radius, .. } => {
                    circle_radii.push((geo_id, *radius));
                }
                _ => {}
            }
        }

        // Collect existing Equal constraints
        let has_equal = |a: GeoId, b: GeoId| -> bool {
            self.constraints.iter().any(|c| {
                c.is_active
                    && c.constraint_type == ConstraintType::Equal
                    && c.first().is_some_and(|e| e.geo_id == a || e.geo_id == b)
                    && c.second().is_some_and(|e| e.geo_id == a || e.geo_id == b)
            })
        };

        let mut detected = Vec::new();

        // Sort by value and find adjacent equals
        line_lengths.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        for w in line_lengths.windows(2) {
            if (w[0].1 - w[1].1).abs() <= precision && !has_equal(w[0].0, w[1].0) {
                detected.push(DetectedConstraint {
                    constraint_type: ConstraintType::Equal,
                    first: GeoElementId::edge(w[0].0),
                    second: GeoElementId::edge(w[1].0),
                });
            }
        }

        circle_radii.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        for w in circle_radii.windows(2) {
            if (w[0].1 - w[1].1).abs() <= precision && !has_equal(w[0].0, w[1].0) {
                detected.push(DetectedConstraint {
                    constraint_type: ConstraintType::Equal,
                    first: GeoElementId::edge(w[0].0),
                    second: GeoElementId::edge(w[1].0),
                });
            }
        }

        detected
    }

    // -----------------------------------------------------------------------
    // Apply detected constraints (Phase A5)
    // -----------------------------------------------------------------------

    /// Apply a list of detected constraints to the sketch.
    ///
    /// Returns the number of constraints actually added.
    pub fn apply_detected_constraints(
        &mut self,
        detected: &[DetectedConstraint],
    ) -> usize {
        let mut count = 0;
        for dc in detected {
            let sc = match dc.constraint_type {
                ConstraintType::Horizontal | ConstraintType::Vertical => {
                    SketchConstraint::geometric(dc.constraint_type, vec![dc.first])
                }
                ConstraintType::Coincident | ConstraintType::Equal => {
                    SketchConstraint::geometric(
                        dc.constraint_type,
                        vec![dc.first, dc.second],
                    )
                }
                _ => continue,
            };
            self.add_constraint(sc);
            count += 1;
        }
        count
    }

    /// Remove all redundant constraints detected by the last solve.
    ///
    /// Returns the number of constraints removed.
    ///
    /// Mirrors FreeCAD `SketchObject::autoRemoveRedundants()`.
    pub fn auto_remove_redundants(&mut self) -> usize {
        let redundant_tags: Vec<i32> = match &self.last_diagnosis {
            Some(diag) => diag.redundant.clone(),
            None => return 0,
        };

        if redundant_tags.is_empty() {
            return 0;
        }

        let before = self.constraints.len();
        self.constraints
            .retain(|c| !redundant_tags.contains(&c.tag));
        let removed = before - self.constraints.len();
        self.last_status = None;
        self.last_diagnosis = None;
        removed
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::GeoDef;

    // -----------------------------------------------------------------------
    // Detect coincident
    // -----------------------------------------------------------------------

    #[test]
    fn detect_missing_coincident_close_points() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 10.0, y1: 0.0, x2: 10.0, y2: 10.0,
        }));

        // end(0) = (10,0) and start(1) = (10,0) → should detect coincident
        let found = sketch.detect_missing_coincident(1e-4);
        assert!(!found.is_empty());
        assert_eq!(found[0].constraint_type, ConstraintType::Coincident);
    }

    #[test]
    fn detect_missing_coincident_ignores_existing() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 10.0, y1: 0.0, x2: 10.0, y2: 10.0,
        }));

        // Already have coincident
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::end(0), GeoElementId::start(1)],
        ));

        let found = sketch.detect_missing_coincident(1e-4);
        assert!(found.is_empty());
    }

    #[test]
    fn detect_missing_coincident_far_points() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 20.0, y1: 0.0, x2: 20.0, y2: 10.0,
        }));

        let found = sketch.detect_missing_coincident(1e-4);
        assert!(found.is_empty());
    }

    // -----------------------------------------------------------------------
    // Detect vertical / horizontal
    // -----------------------------------------------------------------------

    #[test]
    fn detect_horizontal_line() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.01, // nearly horizontal
        }));

        let found = sketch.detect_missing_vertical_horizontal(
            std::f64::consts::PI / 8.0,
        );
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].constraint_type, ConstraintType::Horizontal);
    }

    #[test]
    fn detect_vertical_line() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 0.01, y2: 10.0, // nearly vertical
        }));

        let found = sketch.detect_missing_vertical_horizontal(
            std::f64::consts::PI / 8.0,
        );
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].constraint_type, ConstraintType::Vertical);
    }

    #[test]
    fn detect_neither_v_nor_h() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 10.0, // 45° diagonal
        }));

        let found = sketch.detect_missing_vertical_horizontal(
            std::f64::consts::PI / 8.0,
        );
        assert!(found.is_empty());
    }

    #[test]
    fn detect_vh_skips_existing() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));

        let found = sketch.detect_missing_vertical_horizontal(
            std::f64::consts::PI / 8.0,
        );
        assert!(found.is_empty());
    }

    // -----------------------------------------------------------------------
    // Detect equality
    // -----------------------------------------------------------------------

    #[test]
    fn detect_equal_line_lengths() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0, // length 10
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 5.0, x2: 10.0, y2: 5.0, // length 10
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 10.0, x2: 5.0, y2: 10.0, // length 5 — different
        }));

        let found = sketch.detect_missing_equality(0.01);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].constraint_type, ConstraintType::Equal);
    }

    #[test]
    fn detect_equal_radii() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Circle {
            cx: 0.0, cy: 0.0, radius: 5.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Circle {
            cx: 20.0, cy: 0.0, radius: 5.0,
        }));

        let found = sketch.detect_missing_equality(0.01);
        assert_eq!(found.len(), 1);
        assert_eq!(found[0].constraint_type, ConstraintType::Equal);
    }

    #[test]
    fn detect_equality_ignores_existing() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 5.0, x2: 10.0, y2: 5.0,
        }));

        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Equal,
            vec![GeoElementId::edge(0), GeoElementId::edge(1)],
        ));

        let found = sketch.detect_missing_equality(0.01);
        assert!(found.is_empty());
    }

    // -----------------------------------------------------------------------
    // Apply + remove
    // -----------------------------------------------------------------------

    #[test]
    fn apply_detected_constraints_adds_them() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 10.0, y1: 0.0, x2: 10.0, y2: 10.0,
        }));

        let found = sketch.detect_missing_coincident(1e-4);
        let count = sketch.apply_detected_constraints(&found);
        assert!(count > 0);
        assert!(sketch.constraint_count() > 0);
    }

    #[test]
    fn auto_remove_redundants_cleans_up() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        // Duplicate horizontal constraints → one is redundant
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));

        // Solve to trigger diagnosis
        sketch.solve();
        assert_eq!(sketch.constraint_count(), 2);

        let removed = sketch.auto_remove_redundants();
        assert!(removed >= 1);
    }
}
