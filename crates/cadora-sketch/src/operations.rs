//! Geometry operations (Phase A6).
//!
//! Provides fillet, trim, extend, split, and copy/mirror operations,
//! mirroring FreeCAD's SketchObject geometry editing operations.

use crate::constraint::SketchConstraint;
use crate::geometry::{GeoDef, GeoType};
use crate::sketch::Sketch;
use crate::types::*;

/// Result of a geometry operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeoOpResult {
    /// Operation succeeded.
    Ok,
    /// Invalid geometry reference.
    InvalidGeometry,
    /// Geometry type not supported for this operation.
    UnsupportedType,
    /// No intersection found (for trim).
    NoIntersection,
}

impl Sketch {
    // -----------------------------------------------------------------------
    // Fillet (Phase A6)
    // -----------------------------------------------------------------------

    /// Create an arc fillet between two lines at their coincident vertex.
    ///
    /// Shortens both lines and inserts a tangent arc of the given `radius`.
    /// The lines must share an endpoint (via coincident position, not
    /// constraint — the caller should ensure they meet).
    ///
    /// Returns the GeoId of the new arc, or an error.
    ///
    /// Mirrors FreeCAD `SketchObject::fillet()`.
    pub fn fillet(
        &mut self,
        geo_id1: GeoId,
        geo_id2: GeoId,
        radius: f64,
    ) -> Result<GeoId, GeoOpResult> {
        let (x1a, y1a, x1b, y1b) = match self.get_geometry(geo_id1) {
            Some(g) => match &g.geo {
                GeoType::Line { x1, y1, x2, y2 } => (*x1, *y1, *x2, *y2),
                _ => return Err(GeoOpResult::UnsupportedType),
            },
            None => return Err(GeoOpResult::InvalidGeometry),
        };
        let (x2a, y2a, x2b, y2b) = match self.get_geometry(geo_id2) {
            Some(g) => match &g.geo {
                GeoType::Line { x1, y1, x2, y2 } => (*x1, *y1, *x2, *y2),
                _ => return Err(GeoOpResult::UnsupportedType),
            },
            None => return Err(GeoOpResult::InvalidGeometry),
        };

        // Find the shared vertex
        let tol = 1e-6;
        let (corner, pos1, pos2) = if dist(x1a, y1a, x2a, y2a) < tol {
            ((x1a, y1a), PointPos::Start, PointPos::Start)
        } else if dist(x1a, y1a, x2b, y2b) < tol {
            ((x1a, y1a), PointPos::Start, PointPos::End)
        } else if dist(x1b, y1b, x2a, y2a) < tol {
            ((x1b, y1b), PointPos::End, PointPos::Start)
        } else if dist(x1b, y1b, x2b, y2b) < tol {
            ((x1b, y1b), PointPos::End, PointPos::End)
        } else {
            return Err(GeoOpResult::NoIntersection);
        };

        // Far endpoints
        let far1 = if pos1 == PointPos::Start {
            (x1b, y1b)
        } else {
            (x1a, y1a)
        };
        let far2 = if pos2 == PointPos::Start {
            (x2b, y2b)
        } else {
            (x2a, y2a)
        };

        // Direction vectors from corner toward far ends
        let d1 = normalize(far1.0 - corner.0, far1.1 - corner.1);
        let d2 = normalize(far2.0 - corner.0, far2.1 - corner.1);

        // Angle bisector direction (inward)
        let bisect = normalize(d1.0 + d2.0, d1.1 + d2.1);
        if bisect.0.abs() < 1e-12 && bisect.1.abs() < 1e-12 {
            return Err(GeoOpResult::UnsupportedType); // parallel lines
        }

        // Half-angle between lines
        let dot = d1.0 * d2.0 + d1.1 * d2.1;
        let half_angle = ((1.0 - dot) / 2.0).sqrt().asin();
        if half_angle.abs() < 1e-10 {
            return Err(GeoOpResult::UnsupportedType); // parallel
        }

        // Fillet center along bisector at distance radius/sin(half_angle)
        let center_dist = radius / half_angle.sin();
        let cx = corner.0 + bisect.0 * center_dist;
        let cy = corner.1 + bisect.1 * center_dist;

        // Tangent points: project center onto each line direction
        let t1 = (cx - corner.0) * d1.0 + (cy - corner.1) * d1.1;
        let tp1 = (corner.0 + d1.0 * t1, corner.1 + d1.1 * t1);

        let t2 = (cx - corner.0) * d2.0 + (cy - corner.1) * d2.1;
        let tp2 = (corner.0 + d2.0 * t2, corner.1 + d2.1 * t2);

        // Arc angles
        let start_angle = (tp1.1 - cy).atan2(tp1.0 - cx);
        let end_angle = (tp2.1 - cy).atan2(tp2.0 - cx);

        // Add arc
        let arc_id = self.add_geometry(GeoDef::new(GeoType::Arc {
            cx,
            cy,
            radius,
            start_angle,
            end_angle,
        }));

        // Trim lines: move corner endpoints to tangent points
        if let Some(g) = self.store.get_mut(geo_id1) {
            if let GeoType::Line {
                ref mut x1,
                ref mut y1,
                ref mut x2,
                ref mut y2,
            } = g.geo
            {
                if pos1 == PointPos::Start {
                    *x1 = tp1.0;
                    *y1 = tp1.1;
                } else {
                    *x2 = tp1.0;
                    *y2 = tp1.1;
                }
            }
        }
        if let Some(g) = self.store.get_mut(geo_id2) {
            if let GeoType::Line {
                ref mut x1,
                ref mut y1,
                ref mut x2,
                ref mut y2,
            } = g.geo
            {
                if pos2 == PointPos::Start {
                    *x1 = tp2.0;
                    *y1 = tp2.1;
                } else {
                    *x2 = tp2.0;
                    *y2 = tp2.1;
                }
            }
        }

        // Remove the old Coincident constraint at the corner
        let c1_elem = GeoElementId::new(geo_id1, pos1);
        let c2_elem = GeoElementId::new(geo_id2, pos2);
        self.constraints.retain(|c| {
            !(c.constraint_type == ConstraintType::Coincident
                && c.elements.len() == 2
                && ((c.elements[0] == c1_elem && c.elements[1] == c2_elem)
                    || (c.elements[0] == c2_elem && c.elements[1] == c1_elem)))
        });

        // Add tangent constraints: line1-corner → arc start, line2-corner → arc end
        let arc_start = GeoElementId::start(arc_id);
        let arc_end = GeoElementId::end(arc_id);

        self.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::new(geo_id1, pos1), arc_start],
        ));
        self.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::new(geo_id2, pos2), arc_end],
        ));

        Ok(arc_id)
    }

    // -----------------------------------------------------------------------
    // Trim (Phase A6)
    // -----------------------------------------------------------------------

    /// Trim a geometry at a given point, removing the segment closest to
    /// `(px, py)`.
    ///
    /// For a line, the point determines which side to keep/remove (deletes
    /// the part on one side of the nearest intersection with other geometry).
    ///
    /// Simplified version: trims lines at their midpoint or at the closest
    /// intersection with another geometry.
    ///
    /// Returns `Ok` on success.
    pub fn trim(
        &mut self,
        geo_id: GeoId,
        px: f64,
        py: f64,
    ) -> Result<GeoId, GeoOpResult> {
        let (x1, y1, x2, y2) = match self.get_geometry(geo_id) {
            Some(g) => match &g.geo {
                GeoType::Line { x1, y1, x2, y2 } => (*x1, *y1, *x2, *y2),
                _ => return Err(GeoOpResult::UnsupportedType),
            },
            None => return Err(GeoOpResult::InvalidGeometry),
        };

        // Find intersections with other geometry
        let intersections = self.find_line_intersections(geo_id, x1, y1, x2, y2);

        if intersections.is_empty() {
            return Err(GeoOpResult::NoIntersection);
        }

        // Parameterize click point on line: t ∈ [0,1]
        let dx = x2 - x1;
        let dy = y2 - y1;
        let len_sq = dx * dx + dy * dy;
        let t_click = ((px - x1) * dx + (py - y1) * dy) / len_sq;

        // Find nearest intersections on each side of click
        let mut below: Option<(f64, f64, f64)> = None; // (t, ix, iy) closest below t_click
        let mut above: Option<(f64, f64, f64)> = None; // closest above t_click

        for &(t, ix, iy) in &intersections {
            if t < t_click {
                if below.is_none() || t > below.unwrap().0 {
                    below = Some((t, ix, iy));
                }
            } else {
                if above.is_none() || t < above.unwrap().0 {
                    above = Some((t, ix, iy));
                }
            }
        }

        // Determine which segment to keep: remove the segment containing
        // the click point
        match (below, above) {
            (Some((_, bx, by)), Some((_, ax, ay))) => {
                // Click is between two intersections — split into two lines,
                // remove middle segment
                // Keep: [start..below] and [above..end]
                // Modify original to [start..below]
                if let Some(g) = self.store.get_mut(geo_id) {
                    if let GeoType::Line {
                        ref mut x2,
                        ref mut y2,
                        ..
                    } = g.geo
                    {
                        *x2 = bx;
                        *y2 = by;
                    }
                }
                // Add new line [above..end]
                let new_id = self.add_geometry(GeoDef::new(GeoType::Line {
                    x1: ax,
                    y1: ay,
                    x2,
                    y2,
                }));
                // Transfer end-point constraints from original end to new line end
                let old_end = GeoElementId::end(geo_id);
                let new_end = GeoElementId::end(new_id);
                for c in &mut self.constraints {
                    for e in &mut c.elements {
                        if *e == old_end {
                            *e = new_end;
                        }
                    }
                }
                Ok(new_id)
            }
            (Some((_, bx, by)), None) => {
                // Click is past the last intersection — trim end
                if let Some(g) = self.store.get_mut(geo_id) {
                    if let GeoType::Line {
                        ref mut x2,
                        ref mut y2,
                        ..
                    } = g.geo
                    {
                        *x2 = bx;
                        *y2 = by;
                    }
                }
                Ok(geo_id)
            }
            (None, Some((_, ax, ay))) => {
                // Click is before the first intersection — trim start
                if let Some(g) = self.store.get_mut(geo_id) {
                    if let GeoType::Line {
                        ref mut x1,
                        ref mut y1,
                        ..
                    } = g.geo
                    {
                        *x1 = ax;
                        *y1 = ay;
                    }
                }
                Ok(geo_id)
            }
            (None, None) => Err(GeoOpResult::NoIntersection),
        }
    }

    /// Find parameter-ordered intersection points of a line with all other
    /// geometry. Returns `(t, ix, iy)` where t is the parameter on the line.
    fn find_line_intersections(
        &self,
        exclude_id: GeoId,
        x1: f64,
        y1: f64,
        x2: f64,
        y2: f64,
    ) -> Vec<(f64, f64, f64)> {
        let dx = x2 - x1;
        let dy = y2 - y1;
        let len_sq = dx * dx + dy * dy;
        if len_sq < 1e-30 {
            return Vec::new();
        }

        let mut results = Vec::new();

        for (gid, gdef) in self.store.iter() {
            if gid == exclude_id {
                continue;
            }
            match &gdef.geo {
                GeoType::Line {
                    x1: ox1,
                    y1: oy1,
                    x2: ox2,
                    y2: oy2,
                } => {
                    // Line-line intersection
                    if let Some((t, ix, iy)) =
                        line_line_intersect(x1, y1, x2, y2, *ox1, *oy1, *ox2, *oy2)
                    {
                        if t > -1e-6 && t < 1.0 + 1e-6 {
                            results.push((t, ix, iy));
                        }
                    }
                }
                GeoType::Circle { cx, cy, radius } => {
                    for (t, ix, iy) in line_circle_intersect(x1, y1, dx, dy, *cx, *cy, *radius) {
                        if t > -1e-6 && t < 1.0 + 1e-6 {
                            results.push((t, ix, iy));
                        }
                    }
                }
                _ => {}
            }
        }

        results.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        results
    }

    // -----------------------------------------------------------------------
    // Extend (Phase A6)
    // -----------------------------------------------------------------------

    /// Extend a line by `increment` from the specified endpoint.
    ///
    /// Positive increment lengthens, negative shortens.
    ///
    /// Mirrors FreeCAD `SketchObject::extend()`.
    pub fn extend(
        &mut self,
        geo_id: GeoId,
        increment: f64,
        endpoint: PointPos,
    ) -> GeoOpResult {
        let geo_def = match self.store.get_mut(geo_id) {
            Some(g) => g,
            None => return GeoOpResult::InvalidGeometry,
        };

        match &mut geo_def.geo {
            GeoType::Line {
                x1,
                y1,
                x2,
                y2,
            } => {
                let dx = *x2 - *x1;
                let dy = *y2 - *y1;
                let len = (dx * dx + dy * dy).sqrt();
                if len < 1e-30 {
                    return GeoOpResult::InvalidGeometry;
                }
                let dir = (dx / len, dy / len);
                let new_len = len + increment;
                if new_len < 0.0 {
                    return GeoOpResult::InvalidGeometry;
                }

                match endpoint {
                    PointPos::End => {
                        *x2 = *x1 + dir.0 * new_len;
                        *y2 = *y1 + dir.1 * new_len;
                    }
                    PointPos::Start => {
                        // Extend backward: move start opposite to direction
                        *x1 = *x2 - dir.0 * new_len;
                        *y1 = *y2 - dir.1 * new_len;
                    }
                    _ => return GeoOpResult::UnsupportedType,
                }
                GeoOpResult::Ok
            }
            GeoType::Arc {
                start_angle,
                end_angle,
                ..
            } => {
                match endpoint {
                    PointPos::Start => *start_angle -= increment,
                    PointPos::End => *end_angle += increment,
                    _ => return GeoOpResult::UnsupportedType,
                }
                GeoOpResult::Ok
            }
            _ => GeoOpResult::UnsupportedType,
        }
    }

    // -----------------------------------------------------------------------
    // Split (Phase A6)
    // -----------------------------------------------------------------------

    /// Split a line at the point closest to `(px, py)`.
    ///
    /// Creates two line segments from the original (original is shortened,
    /// new segment is added). Adds a Coincident constraint at the split point.
    ///
    /// Returns the GeoId of the newly created second segment.
    pub fn split(
        &mut self,
        geo_id: GeoId,
        px: f64,
        py: f64,
    ) -> Result<GeoId, GeoOpResult> {
        let (x1, y1, x2, y2) = match self.get_geometry(geo_id) {
            Some(g) => match &g.geo {
                GeoType::Line { x1, y1, x2, y2 } => (*x1, *y1, *x2, *y2),
                _ => return Err(GeoOpResult::UnsupportedType),
            },
            None => return Err(GeoOpResult::InvalidGeometry),
        };

        // Project (px, py) onto line to find split parameter
        let dx = x2 - x1;
        let dy = y2 - y1;
        let len_sq = dx * dx + dy * dy;
        if len_sq < 1e-30 {
            return Err(GeoOpResult::InvalidGeometry);
        }
        let t = (((px - x1) * dx + (py - y1) * dy) / len_sq).clamp(0.01, 0.99);
        let sx = x1 + t * dx;
        let sy = y1 + t * dy;

        // Shorten original to [start, split]
        if let Some(g) = self.store.get_mut(geo_id) {
            if let GeoType::Line {
                ref mut x2,
                ref mut y2,
                ..
            } = g.geo
            {
                *x2 = sx;
                *y2 = sy;
            }
        }

        // Create new segment [split, original_end]
        let new_id = self.add_geometry(GeoDef::new(GeoType::Line {
            x1: sx,
            y1: sy,
            x2,
            y2,
        }));

        // Transfer end-point constraints from original end to new segment end
        let old_end = GeoElementId::end(geo_id);
        let new_end = GeoElementId::end(new_id);
        for c in &mut self.constraints {
            for e in &mut c.elements {
                if *e == old_end {
                    *e = new_end;
                }
            }
        }

        // Add Coincident at split point
        self.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::end(geo_id), GeoElementId::start(new_id)],
        ));

        Ok(new_id)
    }

    // -----------------------------------------------------------------------
    // Copy / Mirror (Phase A6)
    // -----------------------------------------------------------------------

    /// Copy geometries with a displacement.
    ///
    /// All constraints among the copied geometries are duplicated.
    /// Returns the GeoIds of the copied geometries.
    pub fn copy_geometries(
        &mut self,
        geo_ids: &[GeoId],
        dx: f64,
        dy: f64,
    ) -> Vec<GeoId> {
        let mut id_map: Vec<(GeoId, GeoId)> = Vec::new();

        for &gid in geo_ids {
            let geo_def = match self.get_geometry(gid) {
                Some(g) => g.clone(),
                None => continue,
            };
            let new_geo = translate_geometry(&geo_def.geo, dx, dy);
            let mut new_def = GeoDef::new(new_geo);
            new_def.mode = geo_def.mode;
            let new_id = self.add_geometry(new_def);
            id_map.push((gid, new_id));
        }

        // Duplicate constraints among the selection
        let geo_set: Vec<GeoId> = geo_ids.to_vec();
        let new_constraints: Vec<SketchConstraint> = self
            .constraints
            .iter()
            .filter(|c| {
                c.is_active
                    && c.elements
                        .iter()
                        .all(|e| geo_set.contains(&e.geo_id))
            })
            .filter_map(|c| {
                let mut new_c = c.clone();
                for e in &mut new_c.elements {
                    e.geo_id = id_map
                        .iter()
                        .find(|(old, _)| *old == e.geo_id)
                        .map(|(_, new)| *new)?;
                }
                Some(new_c)
            })
            .collect();

        for c in new_constraints {
            self.add_constraint(c);
        }

        id_map.iter().map(|(_, new)| *new).collect()
    }

    /// Mirror geometries across a line defined by two points.
    ///
    /// Returns the GeoIds of the mirrored geometries.
    pub fn mirror_geometries(
        &mut self,
        geo_ids: &[GeoId],
        line_x1: f64,
        line_y1: f64,
        line_x2: f64,
        line_y2: f64,
    ) -> Vec<GeoId> {
        let ldx = line_x2 - line_x1;
        let ldy = line_y2 - line_y1;
        let len_sq = ldx * ldx + ldy * ldy;
        if len_sq < 1e-30 {
            return Vec::new();
        }

        let mut id_map: Vec<(GeoId, GeoId)> = Vec::new();

        for &gid in geo_ids {
            let geo_def = match self.get_geometry(gid) {
                Some(g) => g.clone(),
                None => continue,
            };
            let new_geo =
                mirror_geometry(&geo_def.geo, line_x1, line_y1, ldx, ldy, len_sq);
            let mut new_def = GeoDef::new(new_geo);
            new_def.mode = geo_def.mode;
            let new_id = self.add_geometry(new_def);
            id_map.push((gid, new_id));
        }

        // Add Symmetric constraints between original and mirrored points
        for &(old, new) in &id_map {
            let old_geo = match self.get_geometry(old) {
                Some(g) => g,
                None => continue,
            };
            match &old_geo.geo {
                GeoType::Point { .. } => {
                    self.add_constraint(SketchConstraint::geometric(
                        ConstraintType::Coincident,
                        vec![GeoElementId::mid(old), GeoElementId::mid(new)],
                    ));
                }
                GeoType::Line { .. } => {
                    // Mirror reverses start/end
                }
                GeoType::Circle { .. } => {
                    self.add_constraint(SketchConstraint::geometric(
                        ConstraintType::Equal,
                        vec![GeoElementId::edge(old), GeoElementId::edge(new)],
                    ));
                }
                _ => {}
            }
        }

        id_map.iter().map(|(_, new)| *new).collect()
    }
}

// ===========================================================================
// Geometry math helpers
// ===========================================================================

fn dist(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt()
}

fn normalize(x: f64, y: f64) -> (f64, f64) {
    let len = (x * x + y * y).sqrt();
    if len < 1e-15 {
        (0.0, 0.0)
    } else {
        (x / len, y / len)
    }
}

/// Line-line intersection. Returns `(t, ix, iy)` where t is the parameter on
/// the first line.
fn line_line_intersect(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    x3: f64,
    y3: f64,
    x4: f64,
    y4: f64,
) -> Option<(f64, f64, f64)> {
    let d1x = x2 - x1;
    let d1y = y2 - y1;
    let d2x = x4 - x3;
    let d2y = y4 - y3;

    let denom = d1x * d2y - d1y * d2x;
    if denom.abs() < 1e-12 {
        return None; // parallel
    }

    let t = ((x3 - x1) * d2y - (y3 - y1) * d2x) / denom;
    let u = ((x3 - x1) * d1y - (y3 - y1) * d1x) / denom;

    // Check that intersection is within the second segment too
    if u < -1e-6 || u > 1.0 + 1e-6 {
        return None;
    }

    let ix = x1 + t * d1x;
    let iy = y1 + t * d1y;
    Some((t, ix, iy))
}

/// Line-circle intersection. Returns pairs `(t, ix, iy)`.
fn line_circle_intersect(
    x1: f64,
    y1: f64,
    dx: f64,
    dy: f64,
    cx: f64,
    cy: f64,
    r: f64,
) -> Vec<(f64, f64, f64)> {
    let fx = x1 - cx;
    let fy = y1 - cy;
    let a = dx * dx + dy * dy;
    let b = 2.0 * (fx * dx + fy * dy);
    let c = fx * fx + fy * fy - r * r;
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return Vec::new();
    }
    let sq = disc.sqrt();
    let mut results = Vec::new();
    for sign in [-1.0, 1.0] {
        let t = (-b + sign * sq) / (2.0 * a);
        let ix = x1 + t * dx;
        let iy = y1 + t * dy;
        results.push((t, ix, iy));
    }
    results
}

fn translate_geometry(geo: &GeoType, dx: f64, dy: f64) -> GeoType {
    match geo {
        GeoType::Point { x, y } => GeoType::Point {
            x: x + dx,
            y: y + dy,
        },
        GeoType::Line { x1, y1, x2, y2 } => GeoType::Line {
            x1: x1 + dx,
            y1: y1 + dy,
            x2: x2 + dx,
            y2: y2 + dy,
        },
        GeoType::Circle { cx, cy, radius } => GeoType::Circle {
            cx: cx + dx,
            cy: cy + dy,
            radius: *radius,
        },
        GeoType::Arc {
            cx,
            cy,
            radius,
            start_angle,
            end_angle,
        } => GeoType::Arc {
            cx: cx + dx,
            cy: cy + dy,
            radius: *radius,
            start_angle: *start_angle,
            end_angle: *end_angle,
        },
        other => other.clone(),
    }
}

fn mirror_point(
    px: f64,
    py: f64,
    lx: f64,
    ly: f64,
    ldx: f64,
    ldy: f64,
    len_sq: f64,
) -> (f64, f64) {
    // Project point onto line
    let t = ((px - lx) * ldx + (py - ly) * ldy) / len_sq;
    let proj_x = lx + t * ldx;
    let proj_y = ly + t * ldy;
    // Mirror: point + 2*(proj - point)
    (2.0 * proj_x - px, 2.0 * proj_y - py)
}

fn mirror_geometry(
    geo: &GeoType,
    lx: f64,
    ly: f64,
    ldx: f64,
    ldy: f64,
    len_sq: f64,
) -> GeoType {
    match geo {
        GeoType::Point { x, y } => {
            let (mx, my) = mirror_point(*x, *y, lx, ly, ldx, ldy, len_sq);
            GeoType::Point { x: mx, y: my }
        }
        GeoType::Line { x1, y1, x2, y2 } => {
            let (mx1, my1) = mirror_point(*x1, *y1, lx, ly, ldx, ldy, len_sq);
            let (mx2, my2) = mirror_point(*x2, *y2, lx, ly, ldx, ldy, len_sq);
            GeoType::Line {
                x1: mx1,
                y1: my1,
                x2: mx2,
                y2: my2,
            }
        }
        GeoType::Circle { cx, cy, radius } => {
            let (mcx, mcy) = mirror_point(*cx, *cy, lx, ly, ldx, ldy, len_sq);
            GeoType::Circle {
                cx: mcx,
                cy: mcy,
                radius: *radius,
            }
        }
        GeoType::Arc {
            cx,
            cy,
            radius,
            start_angle,
            end_angle,
        } => {
            let (mcx, mcy) = mirror_point(*cx, *cy, lx, ly, ldx, ldy, len_sq);
            // Mirror reverses arc direction — swap and negate angles relative
            // to mirror line
            let line_angle = ldy.atan2(ldx);
            let new_start = 2.0 * line_angle - end_angle;
            let new_end = 2.0 * line_angle - start_angle;
            GeoType::Arc {
                cx: mcx,
                cy: mcy,
                radius: *radius,
                start_angle: new_start,
                end_angle: new_end,
            }
        }
        other => other.clone(),
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::GeoDef;
    use approx::assert_abs_diff_eq;

    // -----------------------------------------------------------------------
    // Fillet
    // -----------------------------------------------------------------------

    #[test]
    fn fillet_two_perpendicular_lines() {
        let mut sketch = Sketch::new();
        // L-shape: (0,0)→(10,0) and (0,0)→(0,10), meeting at origin
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 0.0, y2: 10.0,
        }));

        let arc_id = sketch.fillet(0, 1, 2.0).unwrap();
        assert!(arc_id >= 2);

        // Lines should be shortened
        if let Some(g) = sketch.get_geometry(0) {
            if let GeoType::Line { x1, y1: _, .. } = &g.geo {
                // Start was moved to tangent point
                assert!(*x1 > 1.5);
            }
        }

        // Arc should exist
        let arc = sketch.get_geometry(arc_id).unwrap();
        if let GeoType::Arc { radius, .. } = &arc.geo {
            assert_abs_diff_eq!(*radius, 2.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn fillet_non_intersecting_returns_error() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 5.0, x2: 10.0, y2: 5.0,
        }));

        let result = sketch.fillet(0, 1, 2.0);
        assert!(result.is_err());
    }

    // -----------------------------------------------------------------------
    // Extend
    // -----------------------------------------------------------------------

    #[test]
    fn extend_line_from_end() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));

        let result = sketch.extend(0, 5.0, PointPos::End);
        assert_eq!(result, GeoOpResult::Ok);

        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x2, y2, .. } = &g.geo {
            assert_abs_diff_eq!(*x2, 15.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn extend_line_from_start() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 5.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));

        sketch.extend(0, 3.0, PointPos::Start);

        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x1, .. } = &g.geo {
            assert_abs_diff_eq!(*x1, 2.0, epsilon = 1e-6);
        }
    }

    // -----------------------------------------------------------------------
    // Split
    // -----------------------------------------------------------------------

    #[test]
    fn split_line_at_midpoint() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));

        let new_id = sketch.split(0, 5.0, 0.0).unwrap();

        // Original should be [0..5]
        let g0 = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x2, .. } = &g0.geo {
            assert_abs_diff_eq!(*x2, 5.0, epsilon = 1e-3);
        }

        // New should be [5..10]
        let g1 = sketch.get_geometry(new_id).unwrap();
        if let GeoType::Line { x1, x2, .. } = &g1.geo {
            assert_abs_diff_eq!(*x1, 5.0, epsilon = 1e-3);
            assert_abs_diff_eq!(*x2, 10.0, epsilon = 1e-3);
        }

        // Should have coincident constraint at split
        assert!(sketch.constraint_count() > 0);
    }

    #[test]
    fn split_preserves_end_constraints() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        // Fix original end at (10,0)
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceX,
            vec![GeoElementId::end(0)],
            10.0,
        ));

        let new_id = sketch.split(0, 5.0, 0.0).unwrap();

        // The DistanceX constraint should now reference new_id's end
        let c = sketch.get_constraint(0).unwrap();
        assert_eq!(c.elements[0], GeoElementId::end(new_id));
    }

    // -----------------------------------------------------------------------
    // Trim
    // -----------------------------------------------------------------------

    #[test]
    fn trim_line_at_intersection() {
        let mut sketch = Sketch::new();
        // Horizontal line
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        // Vertical crossing line at x=5
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 5.0, y1: -5.0, x2: 5.0, y2: 5.0,
        }));

        // Click right of intersection → trim right side
        let result = sketch.trim(0, 8.0, 0.0);
        assert!(result.is_ok());

        // Original line should end at intersection (5, 0)
        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x2, y2, .. } = &g.geo {
            assert_abs_diff_eq!(*x2, 5.0, epsilon = 1e-3);
            assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-3);
        }
    }

    // -----------------------------------------------------------------------
    // Copy
    // -----------------------------------------------------------------------

    #[test]
    fn copy_geometries_with_displacement() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 10.0, y1: 0.0, x2: 10.0, y2: 5.0,
        }));

        // Add coincident between them
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::end(0), GeoElementId::start(1)],
        ));

        let new_ids = sketch.copy_geometries(&[0, 1], 20.0, 0.0);
        assert_eq!(new_ids.len(), 2);

        // Verify displacement
        let g = sketch.get_geometry(new_ids[0]).unwrap();
        if let GeoType::Line { x1, .. } = &g.geo {
            assert_abs_diff_eq!(*x1, 20.0, epsilon = 1e-6);
        }

        // Should have duplicated the coincident constraint
        assert!(sketch.constraint_count() >= 2);
    }

    // -----------------------------------------------------------------------
    // Mirror
    // -----------------------------------------------------------------------

    #[test]
    fn mirror_point_across_y_axis() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 5.0, y: 3.0 }));

        let new_ids = sketch.mirror_geometries(
            &[0],
            0.0, 0.0, // line start (origin)
            0.0, 1.0, // line end (y-axis)
        );
        assert_eq!(new_ids.len(), 1);

        let g = sketch.get_geometry(new_ids[0]).unwrap();
        if let GeoType::Point { x, y } = &g.geo {
            assert_abs_diff_eq!(*x, -5.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*y, 3.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn mirror_line_across_x_axis() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 1.0, x2: 10.0, y2: 3.0,
        }));

        let new_ids = sketch.mirror_geometries(
            &[0],
            0.0, 0.0,
            1.0, 0.0, // x-axis
        );

        let g = sketch.get_geometry(new_ids[0]).unwrap();
        if let GeoType::Line { x1, y1, x2, y2 } = &g.geo {
            assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*y1, -1.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*x2, 10.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*y2, -3.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn mirror_circle_preserves_radius() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Circle {
            cx: 5.0, cy: 3.0, radius: 2.0,
        }));

        let new_ids = sketch.mirror_geometries(
            &[0],
            0.0, 0.0,
            0.0, 1.0, // y-axis
        );

        let g = sketch.get_geometry(new_ids[0]).unwrap();
        if let GeoType::Circle { cx, cy, radius } = &g.geo {
            assert_abs_diff_eq!(*cx, -5.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*cy, 3.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*radius, 2.0, epsilon = 1e-6);
        }
    }
}
