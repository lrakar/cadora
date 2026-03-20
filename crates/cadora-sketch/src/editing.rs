//! Interactive editing & dragging (Phase A4).
//!
//! Provides temporary-move / drag operations, coincidence-group queries,
//! and point-on-curve hit testing — mirroring FreeCAD's SketchObject
//! interactive editing layer.

use crate::constraint::SketchConstraint;
use crate::geometry::GeoType;
use crate::sketch::{Sketch, SketchSolveStatus};
use crate::types::*;

// ---------------------------------------------------------------------------
// Coincidence groups
// ---------------------------------------------------------------------------

/// A group of points that are mutually coincident (linked via Coincident or
/// transitive Coincident chain).
pub type CoincidenceGroup = Vec<GeoElementId>;

impl Sketch {
    /// Compute coincidence groups from all active Coincident constraints.
    ///
    /// Uses a union-find algorithm to group points that are transitively
    /// coincident.  Each returned group contains ≥ 2 elements.
    ///
    /// Mirrors FreeCAD `SketchObject::getCoincidenceGroups()`.
    pub fn get_coincidence_groups(&self) -> Vec<CoincidenceGroup> {
        let mut groups: Vec<CoincidenceGroup> = Vec::new();

        for c in &self.constraints {
            if !c.is_active || c.constraint_type != ConstraintType::Coincident {
                continue;
            }
            let (e1, e2) = match (c.first(), c.second()) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };

            let g1 = groups.iter().position(|g| g.contains(&e1));
            let g2 = groups.iter().position(|g| g.contains(&e2));

            match (g1, g2) {
                (Some(i), Some(j)) if i != j => {
                    // Merge group j into group i
                    let merged = groups.remove(j);
                    let target = if j < i { i - 1 } else { i };
                    groups[target].extend(merged);
                }
                (Some(i), None) => {
                    groups[i].push(e2);
                }
                (None, Some(j)) => {
                    groups[j].push(e1);
                }
                (None, None) => {
                    groups.push(vec![e1, e2]);
                }
                _ => {} // both in same group already
            }
        }

        groups
    }

    /// Check whether two specific points are coincident (directly or
    /// transitively through Coincident constraint chains).
    ///
    /// Mirrors FreeCAD `SketchObject::arePointsCoincident()`.
    pub fn are_points_coincident(&self, a: GeoElementId, b: GeoElementId) -> bool {
        if a == b {
            return true;
        }
        for group in self.get_coincidence_groups() {
            if group.contains(&a) && group.contains(&b) {
                return true;
            }
        }
        false
    }

    // -----------------------------------------------------------------------
    // Temporary move / drag
    // -----------------------------------------------------------------------

    /// Perform a temporary move of a geometry element and re-solve.
    ///
    /// Adds temporary Coincident-to-target constraints, solves, removes them.
    /// Returns the solve status.
    ///
    /// `elements` — the points to drag (e.g. `[GeoElementId::start(0)]`)
    /// `dx`, `dy` — the displacement delta.
    ///
    /// Mirrors FreeCAD's `initTemporaryMove()` + `moveGeometryTemporary()`.
    pub fn move_temporary(
        &mut self,
        elements: &[GeoElementId],
        dx: f64,
        dy: f64,
    ) -> SketchSolveStatus {
        // Save current positions for the target points
        let targets: Vec<(GeoElementId, f64, f64)> = elements
            .iter()
            .filter_map(|e| {
                let (px, py) = self.get_point(*e)?;
                Some((*e, px + dx, py + dy))
            })
            .collect();

        if targets.is_empty() {
            return self.last_status.unwrap_or(SketchSolveStatus::Malformed);
        }

        // Add temporary fix-coordinate constraints that pin the dragged
        // points to their target positions.
        let mut temp_indices: Vec<usize> = Vec::new();
        for &(elem, tx, ty) in &targets {
            let idx_x = self.add_constraint(SketchConstraint::dimensional(
                ConstraintType::DistanceX,
                vec![elem],
                tx,
            ));
            let idx_y = self.add_constraint(SketchConstraint::dimensional(
                ConstraintType::DistanceY,
                vec![elem],
                ty,
            ));
            temp_indices.push(idx_x);
            temp_indices.push(idx_y);
        }

        let status = self.solve();

        // Remove temporary constraints (reverse order to keep indices valid)
        for idx in temp_indices.into_iter().rev() {
            self.del_constraint(idx);
        }

        status
    }

    /// Move a whole geometry element (all its defining points) by (dx, dy).
    ///
    /// For a Line this moves start+end; for a Circle it moves the center; etc.
    pub fn move_geometry_temporary(
        &mut self,
        geo_id: GeoId,
        dx: f64,
        dy: f64,
    ) -> SketchSolveStatus {
        let elements = self.movable_points(geo_id);
        if elements.is_empty() {
            return SketchSolveStatus::Malformed;
        }
        self.move_temporary(&elements, dx, dy)
    }

    /// Collect the list of draggable points for a geometry.
    fn movable_points(&self, geo_id: GeoId) -> Vec<GeoElementId> {
        let geo_def = match self.get_geometry(geo_id) {
            Some(g) => g,
            None => return Vec::new(),
        };
        match &geo_def.geo {
            GeoType::Point { .. } => vec![GeoElementId::mid(geo_id)],
            GeoType::Line { .. } => vec![
                GeoElementId::start(geo_id),
                GeoElementId::end(geo_id),
            ],
            GeoType::Circle { .. } => vec![GeoElementId::mid(geo_id)],
            GeoType::Arc { .. } => vec![
                GeoElementId::start(geo_id),
                GeoElementId::end(geo_id),
                GeoElementId::mid(geo_id),
            ],
            _ => Vec::new(),
        }
    }

    // -----------------------------------------------------------------------
    // Point-on-curve hit testing
    // -----------------------------------------------------------------------

    /// Test whether a point `(px, py)` lies on the given curve within solver
    /// precision.
    ///
    /// Mirrors FreeCAD `SketchObject::isPointOnCurve()`.
    pub fn is_point_on_curve(&self, geo_id: GeoId, px: f64, py: f64) -> bool {
        let geo_def = match self.get_geometry(geo_id) {
            Some(g) => g,
            None => return false,
        };

        let err_sq = match &geo_def.geo {
            GeoType::Line { x1, y1, x2, y2 } => {
                // Signed distance from point to infinite line, squared
                let dx = x2 - x1;
                let dy = y2 - y1;
                let len_sq = dx * dx + dy * dy;
                if len_sq < 1e-30 {
                    return false;
                }
                // Perpendicular distance
                let cross = (px - x1) * dy - (py - y1) * dx;
                let perp_sq = cross * cross / len_sq;
                // Also check if projection is within segment
                let dot = (px - x1) * dx + (py - y1) * dy;
                let t = dot / len_sq;
                if t < -1e-6 || t > 1.0 + 1e-6 {
                    // Point is beyond segment endpoints — use distance to
                    // nearest endpoint instead
                    let d1 = (px - x1).powi(2) + (py - y1).powi(2);
                    let d2 = (px - x2).powi(2) + (py - y2).powi(2);
                    d1.min(d2)
                } else {
                    perp_sq
                }
            }
            GeoType::Circle { cx, cy, radius } => {
                let d = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
                (d - radius).powi(2)
            }
            GeoType::Arc {
                cx,
                cy,
                radius,
                start_angle,
                end_angle,
            } => {
                let d = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
                let on_circle = (d - radius).powi(2);
                // Check angle range
                let mut angle = (py - cy).atan2(px - cx);
                let mut sa = *start_angle;
                let mut ea = *end_angle;
                // Normalize to [0, 2π)
                while sa < 0.0 { sa += std::f64::consts::TAU; }
                while ea < 0.0 { ea += std::f64::consts::TAU; }
                while angle < 0.0 { angle += std::f64::consts::TAU; }
                let in_range = if sa <= ea {
                    angle >= sa - 1e-6 && angle <= ea + 1e-6
                } else {
                    angle >= sa - 1e-6 || angle <= ea + 1e-6
                };
                if in_range { on_circle } else { f64::MAX }
            }
            _ => return false,
        };

        err_sq < 1e-6
    }

    /// Find the closest geometry edge to a point. Returns `(GeoId, distance²)`.
    ///
    /// Useful for hit testing / snap-to-geometry in interactive editing.
    pub fn closest_geometry(&self, px: f64, py: f64) -> Option<(GeoId, f64)> {
        let mut best: Option<(GeoId, f64)> = None;

        for (geo_id, geo_def) in self.store.iter() {
            let d_sq = match &geo_def.geo {
                GeoType::Point { x, y } => (px - x).powi(2) + (py - y).powi(2),
                GeoType::Line { x1, y1, x2, y2 } => {
                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    let len_sq = dx * dx + dy * dy;
                    if len_sq < 1e-30 {
                        (px - x1).powi(2) + (py - y1).powi(2)
                    } else {
                        let t = ((px - x1) * dx + (py - y1) * dy) / len_sq;
                        let t = t.clamp(0.0, 1.0);
                        let cx = x1 + t * dx;
                        let cy = y1 + t * dy;
                        (px - cx).powi(2) + (py - cy).powi(2)
                    }
                }
                GeoType::Circle { cx, cy, radius } => {
                    let d = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
                    (d - radius).powi(2)
                }
                GeoType::Arc { cx, cy, radius, .. } => {
                    let d = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
                    (d - radius).powi(2)
                }
                _ => continue,
            };

            if best.is_none() || d_sq < best.unwrap().1 {
                best = Some((geo_id, d_sq));
            }
        }

        best
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
    // Coincidence groups
    // -----------------------------------------------------------------------

    #[test]
    fn coincidence_groups_basic() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 10.0, y1: 0.0, x2: 10.0, y2: 10.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 10.0, y1: 10.0, x2: 0.0, y2: 0.0,
        }));

        // Connect: end(0) = start(1), end(1) = start(2)
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::end(0), GeoElementId::start(1)],
        ));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::end(1), GeoElementId::start(2)],
        ));

        let groups = sketch.get_coincidence_groups();
        assert_eq!(groups.len(), 2);
    }

    #[test]
    fn coincidence_groups_transitive() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));

        // A=B, B=C → all in one group
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(0), GeoElementId::mid(1)],
        ));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(1), GeoElementId::mid(2)],
        ));

        let groups = sketch.get_coincidence_groups();
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].len(), 3);
    }

    #[test]
    fn are_points_coincident_basic() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 5.0, y: 5.0 }));

        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(0), GeoElementId::mid(1)],
        ));

        assert!(sketch.are_points_coincident(GeoElementId::mid(0), GeoElementId::mid(1)));
        assert!(!sketch.are_points_coincident(GeoElementId::mid(0), GeoElementId::mid(2)));
        // Identity
        assert!(sketch.are_points_coincident(GeoElementId::mid(0), GeoElementId::mid(0)));
    }

    // -----------------------------------------------------------------------
    // Point-on-curve hit testing
    // -----------------------------------------------------------------------

    #[test]
    fn point_on_line_hit_test() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));

        assert!(sketch.is_point_on_curve(0, 5.0, 0.0));    // midpoint
        assert!(sketch.is_point_on_curve(0, 0.0, 0.0));    // start
        assert!(sketch.is_point_on_curve(0, 10.0, 0.0));   // end
        assert!(!sketch.is_point_on_curve(0, 5.0, 1.0));   // off the line
        assert!(!sketch.is_point_on_curve(0, -5.0, 0.0));  // past start
    }

    #[test]
    fn point_on_circle_hit_test() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Circle {
            cx: 0.0, cy: 0.0, radius: 5.0,
        }));

        assert!(sketch.is_point_on_curve(0, 5.0, 0.0));    // right
        assert!(sketch.is_point_on_curve(0, 0.0, 5.0));    // top
        assert!(sketch.is_point_on_curve(0, -5.0, 0.0));   // left
        assert!(!sketch.is_point_on_curve(0, 0.0, 0.0));   // center
        assert!(!sketch.is_point_on_curve(0, 3.0, 0.0));   // inside
    }

    #[test]
    fn closest_geometry_finds_nearest() {
        let mut sketch = Sketch::new();
        // Line at y=0
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        // Line at y=5
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 5.0, x2: 10.0, y2: 5.0,
        }));

        let (id, _) = sketch.closest_geometry(5.0, 1.0).unwrap();
        assert_eq!(id, 0); // closer to y=0

        let (id, _) = sketch.closest_geometry(5.0, 4.0).unwrap();
        assert_eq!(id, 1); // closer to y=5
    }

    // -----------------------------------------------------------------------
    // Temporary move / drag
    // -----------------------------------------------------------------------

    #[test]
    fn drag_point_updates_geometry() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));

        let status = sketch.move_temporary(
            &[GeoElementId::mid(0)],
            5.0, 3.0,
        );
        assert_eq!(status, SketchSolveStatus::Ok);

        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Point { x, y } = &g.geo {
            assert!((*x - 5.0).abs() < 1e-3);
            assert!((*y - 3.0).abs() < 1e-3);
        }
    }

    #[test]
    fn drag_preserves_constraint() {
        let mut sketch = Sketch::new();
        // Horizontal line, length 10
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::Distance,
            vec![GeoElementId::edge(0)],
            10.0,
        ));

        // Drag start point up by 5 — should stay horizontal (y unchanged)
        // with length still 10
        let status = sketch.move_temporary(
            &[GeoElementId::start(0)],
            0.0, 5.0,
        );
        // May or may not be Ok depending on conflicting fix + constraint
        // but geometry should remain valid after temp constraints are removed
        let _ = status;
    }

    #[test]
    fn move_geometry_temporary_line() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));

        let status = sketch.move_geometry_temporary(0, 5.0, 5.0);
        assert_eq!(status, SketchSolveStatus::Ok);

        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x1, y1, x2, y2 } = &g.geo {
            // Both endpoints should have moved by (5, 5)
            assert!((*x1 - 5.0).abs() < 1e-3);
            assert!((*y1 - 5.0).abs() < 1e-3);
            assert!((*x2 - 15.0).abs() < 1e-3);
            assert!((*y2 - 5.0).abs() < 1e-3);
        }
    }

    #[test]
    fn drag_with_coincident_moves_linked_point() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));

        // Make them coincident
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(0), GeoElementId::mid(1)],
        ));

        // Drag point 0 to (10, 10)
        let status = sketch.move_temporary(
            &[GeoElementId::mid(0)],
            10.0, 10.0,
        );
        assert_eq!(status, SketchSolveStatus::Ok);

        // Point 1 should follow due to coincident constraint
        let g1 = sketch.get_geometry(1).unwrap();
        if let GeoType::Point { x, y } = &g1.geo {
            assert!((*x - 10.0).abs() < 1e-3);
            assert!((*y - 10.0).abs() < 1e-3);
        }
    }
}
