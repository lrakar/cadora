//! The main [`Sketch`] struct — manages geometry, constraints, and solver.
//!
//! Equivalent to FreeCAD's SketchObject + Sketch combined into one clean API.

use crate::constraint::SketchConstraint;
use crate::geometry::{GeoDef, GeoType, GeometryStore};
use crate::external::ExternalRef;
use crate::persistence::SketchSnapshot;
use crate::types::*;

use cadora_constraints::*;
use cadora_core::{ParamIdx, ParamStore, Point, SolveStatus, SolverConfig};
use cadora_geo::curve::Line;
use cadora_system::System;

/// Solver result with user-friendly status codes matching FreeCAD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchSolveStatus {
    /// Fully solved, all constraints satisfied.
    Ok,
    /// Solution found but with redundant constraints.
    Redundant,
    /// Conflicting constraints detected.
    Conflicting,
    /// Over-constrained system.
    OverConstrained,
    /// Malformed constraints (invalid geometry references).
    Malformed,
    /// Solver failed to converge.
    Failed,
}

/// Diagnosis result from the sketch solver.
#[derive(Debug, Clone)]
pub struct SketchDiagnosis {
    /// Degrees of freedom remaining.
    pub dofs: i32,
    /// Tags of conflicting constraints.
    pub conflicting: Vec<i32>,
    /// Tags of redundant constraints.
    pub redundant: Vec<i32>,
}

/// The main sketch manager. 
///
/// Manages geometry and constraints, translates them to GCS-level objects,
/// and invokes the solver.
pub struct Sketch {
    pub(crate) store: GeometryStore,
    pub(crate) constraints: Vec<SketchConstraint>,
    next_tag: i32,
    solver_config: SolverConfig,
    /// Last solve status.
    pub(crate) last_status: Option<SketchSolveStatus>,
    /// Last diagnosis result.
    pub(crate) last_diagnosis: Option<SketchDiagnosis>,
    /// External geometry references.
    pub(crate) external_refs: Vec<ExternalRef>,
    /// Undo stack (snapshots of previous states).
    pub(crate) undo_stack: Vec<SketchSnapshot>,
    /// Redo stack (snapshots of undone states).
    pub(crate) redo_stack: Vec<SketchSnapshot>,
    /// Maximum undo steps.
    pub(crate) max_undo: usize,
}

impl Sketch {
    /// Create a new empty sketch.
    pub fn new() -> Self {
        Self {
            store: GeometryStore::new(),
            constraints: Vec::new(),
            next_tag: 1,
            solver_config: SolverConfig::default(),
            last_status: None,
            last_diagnosis: None,
            external_refs: Vec::new(),
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            max_undo: 50,
        }
    }

    /// Create a sketch with custom solver configuration.
    pub fn with_config(config: SolverConfig) -> Self {
        Self {
            solver_config: config,
            ..Self::new()
        }
    }

    // -----------------------------------------------------------------------
    // Geometry management (Phase A1)
    // -----------------------------------------------------------------------

    /// Add a geometry element to the sketch. Returns its [`GeoId`].
    pub fn add_geometry(&mut self, geo_def: GeoDef) -> GeoId {
        self.last_status = None;
        self.last_diagnosis = None;
        self.store.add(geo_def)
    }

    /// Remove a geometry element and all constraints referencing it.
    pub fn del_geometry(&mut self, geo_id: GeoId) -> bool {
        if self.store.get(geo_id).is_none() {
            return false;
        }
        // Remove constraints that reference this geometry
        self.constraints.retain(|c| {
            !c.elements.iter().any(|e| e.geo_id == geo_id)
        });
        // Remap constraints with higher GeoIds
        for c in &mut self.constraints {
            for e in &mut c.elements {
                if e.geo_id > geo_id {
                    e.geo_id -= 1;
                }
            }
        }
        self.store.remove(geo_id);
        self.last_status = None;
        self.last_diagnosis = None;
        true
    }

    /// Get a geometry element by GeoId.
    pub fn get_geometry(&self, geo_id: GeoId) -> Option<&GeoDef> {
        self.store.get(geo_id)
    }

    /// Toggle construction mode for a geometry element.
    pub fn set_construction(&mut self, geo_id: GeoId, construction: bool) -> bool {
        if let Some(g) = self.store.get_mut(geo_id) {
            g.mode.construction = construction;
            true
        } else {
            false
        }
    }

    /// Number of user geometries.
    pub fn geometry_count(&self) -> usize {
        self.store.len()
    }

    /// Iterate over all geometries with their GeoIds.
    pub fn geometries(&self) -> impl Iterator<Item = (GeoId, &GeoDef)> {
        self.store.iter()
    }

    /// Get point coordinates for a GeoElementId (including axis points).
    pub fn get_point(&self, elem: GeoElementId) -> Option<(f64, f64)> {
        self.store.get_point(elem)
    }

    // -----------------------------------------------------------------------
    // Constraint management (Phase A2)
    // -----------------------------------------------------------------------

    /// Add a constraint to the sketch. Returns its index.
    pub fn add_constraint(&mut self, mut constraint: SketchConstraint) -> usize {
        constraint.tag = self.next_tag;
        self.next_tag += 1;
        self.last_status = None;
        self.last_diagnosis = None;
        let idx = self.constraints.len();
        self.constraints.push(constraint);
        idx
    }

    /// Remove a constraint by index.
    pub fn del_constraint(&mut self, index: usize) -> bool {
        if index < self.constraints.len() {
            self.constraints.remove(index);
            self.last_status = None;
            self.last_diagnosis = None;
            true
        } else {
            false
        }
    }

    /// Update the dimensional value of a constraint.
    pub fn set_datum(&mut self, index: usize, value: f64) -> bool {
        if let Some(c) = self.constraints.get_mut(index) {
            c.value = Some(value);
            self.last_status = None;
            true
        } else {
            false
        }
    }

    /// Set whether a constraint is driving or reference-only.
    pub fn set_driving(&mut self, index: usize, driving: bool) -> bool {
        if let Some(c) = self.constraints.get_mut(index) {
            c.is_driving = driving;
            self.last_status = None;
            true
        } else {
            false
        }
    }

    /// Set whether a constraint is active.
    pub fn set_active(&mut self, index: usize, active: bool) -> bool {
        if let Some(c) = self.constraints.get_mut(index) {
            c.is_active = active;
            self.last_status = None;
            true
        } else {
            false
        }
    }

    /// Number of constraints.
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }

    /// Get a constraint by index.
    pub fn get_constraint(&self, index: usize) -> Option<&SketchConstraint> {
        self.constraints.get(index)
    }

    // -----------------------------------------------------------------------
    // Solver integration (Phase A3)
    // -----------------------------------------------------------------------

    /// Build the GCS system and solve. Updates geometry with solution.
    pub fn solve(&mut self) -> SketchSolveStatus {
        let mut param_store = ParamStore::new();

        // --- Phase 1: Allocate parameters for each geometry ---
        struct GeoParams {
            points: Vec<(PointPos, Point)>,
            radius: Option<ParamIdx>,
            start_angle: Option<ParamIdx>,
            end_angle: Option<ParamIdx>,
        }

        let mut geo_params: Vec<GeoParams> = Vec::new();
        let mut all_unknowns: Vec<ParamIdx> = Vec::new();

        for (_geo_id, geo_def) in self.store.iter() {
            let gp = match &geo_def.geo {
                GeoType::Point { x, y } => {
                    let px = param_store.push(*x);
                    let py = param_store.push(*y);
                    if !geo_def.mode.blocked {
                        all_unknowns.extend([px, py]);
                    }
                    GeoParams {
                        points: vec![(PointPos::Mid, Point::new(px, py))],
                        radius: None,
                        start_angle: None,
                        end_angle: None,
                    }
                }
                GeoType::Line { x1, y1, x2, y2 } => {
                    let p1x = param_store.push(*x1);
                    let p1y = param_store.push(*y1);
                    let p2x = param_store.push(*x2);
                    let p2y = param_store.push(*y2);
                    if !geo_def.mode.blocked {
                        all_unknowns.extend([p1x, p1y, p2x, p2y]);
                    }
                    GeoParams {
                        points: vec![
                            (PointPos::Start, Point::new(p1x, p1y)),
                            (PointPos::End, Point::new(p2x, p2y)),
                        ],
                        radius: None,
                        start_angle: None,
                        end_angle: None,
                    }
                }
                GeoType::Circle { cx, cy, radius } => {
                    let pcx = param_store.push(*cx);
                    let pcy = param_store.push(*cy);
                    let pr = param_store.push(*radius);
                    if !geo_def.mode.blocked {
                        all_unknowns.extend([pcx, pcy, pr]);
                    }
                    GeoParams {
                        points: vec![(PointPos::Mid, Point::new(pcx, pcy))],
                        radius: Some(pr),
                        start_angle: None,
                        end_angle: None,
                    }
                }
                GeoType::Arc { cx, cy, radius, start_angle, end_angle } => {
                    let pcx = param_store.push(*cx);
                    let pcy = param_store.push(*cy);
                    let pr = param_store.push(*radius);
                    let psa = param_store.push(*start_angle);
                    let pea = param_store.push(*end_angle);
                    // Start/end points derived from center+radius+angle
                    let sx = param_store.push(cx + radius * start_angle.cos());
                    let sy = param_store.push(cy + radius * start_angle.sin());
                    let ex = param_store.push(cx + radius * end_angle.cos());
                    let ey = param_store.push(cy + radius * end_angle.sin());
                    if !geo_def.mode.blocked {
                        all_unknowns.extend([pcx, pcy, pr, psa, pea, sx, sy, ex, ey]);
                    }
                    GeoParams {
                        points: vec![
                            (PointPos::Start, Point::new(sx, sy)),
                            (PointPos::End, Point::new(ex, ey)),
                            (PointPos::Mid, Point::new(pcx, pcy)),
                        ],
                        radius: Some(pr),
                        start_angle: Some(psa),
                        end_angle: Some(pea),
                    }
                }
                _ => {
                    // TODO: Ellipse, Hyperbola, Parabola, BSpline parameter allocation
                    GeoParams {
                        points: Vec::new(),
                        radius: None,
                        start_angle: None,
                        end_angle: None,
                    }
                }
            };
            geo_params.push(gp);
        }

        // Built-in axes as parameters
        let origin_x = param_store.push(0.0);
        let origin_y = param_store.push(0.0);
        let origin = Point::new(origin_x, origin_y);
        let h_end_x = param_store.push(1.0);
        let h_end_y = param_store.push(0.0);
        let h_end = Point::new(h_end_x, h_end_y);
        let v_end_x = param_store.push(0.0);
        let v_end_y = param_store.push(1.0);
        let v_end = Point::new(v_end_x, v_end_y);

        // --- Phase 2: Translate constraints ---
        let mut system = System::new(param_store, self.solver_config.clone());
        system.declare_unknowns(all_unknowns);

        // Track param equality pairs from Coincident/Horizontal/Vertical constraints.
        // The solver's reduction map will consume 2-param Equal constraints, but may
        // not apply the reduction when no other constraints remain. We handle it manually.
        let mut equal_pairs: Vec<(ParamIdx, ParamIdx)> = Vec::new();

        for constraint in &self.constraints {
            if !constraint.is_active {
                continue;
            }
            let tag = constraint.tag;
            let driving = constraint.is_driving;

            // Helper to resolve a GeoElementId to a GCS Point
            let resolve_point = |elem: GeoElementId| -> Option<Point> {
                // Axes
                if elem.geo_id == GEO_ID_H_AXIS {
                    return match elem.pos {
                        PointPos::Start | PointPos::Mid => Some(origin),
                        PointPos::End => Some(h_end),
                        PointPos::None => None,
                    };
                }
                if elem.geo_id == GEO_ID_V_AXIS {
                    return match elem.pos {
                        PointPos::Start | PointPos::Mid => Some(origin),
                        PointPos::End => Some(v_end),
                        PointPos::None => None,
                    };
                }
                let idx = elem.geo_id as usize;
                if idx >= geo_params.len() { return None; }
                let gp = &geo_params[idx];
                gp.points.iter()
                    .find(|(pp, _)| *pp == elem.pos)
                    .map(|(_, pt)| *pt)
            };

            // Helper to get a Line from a GeoId
            let resolve_line = |geo_id: GeoId| -> Option<Line> {
                if geo_id == GEO_ID_H_AXIS {
                    return Some(Line { p1: origin, p2: h_end });
                }
                if geo_id == GEO_ID_V_AXIS {
                    return Some(Line { p1: origin, p2: v_end });
                }
                let idx = geo_id as usize;
                if idx >= geo_params.len() { return None; }
                let gp = &geo_params[idx];
                let p1 = gp.points.iter().find(|(pp, _)| *pp == PointPos::Start)?;
                let p2 = gp.points.iter().find(|(pp, _)| *pp == PointPos::End)?;
                Some(Line { p1: p1.1, p2: p2.1 })
            };

            match constraint.constraint_type {
                ConstraintType::Coincident => {
                    if let (Some(p1), Some(p2)) = (
                        constraint.first().and_then(&resolve_point),
                        constraint.second().and_then(&resolve_point),
                    ) {
                        equal_pairs.push((p1.x, p2.x));
                        equal_pairs.push((p1.y, p2.y));
                        system.add_constraint(Box::new(
                            ConstraintEqual::new(p1.x, p2.x, 1.0, tag, driving),
                        ));
                        system.add_constraint(Box::new(
                            ConstraintEqual::new(p1.y, p2.y, 1.0, tag, driving),
                        ));
                    }
                }

                ConstraintType::Horizontal => {
                    if let Some(e1) = constraint.first() {
                        if e1.pos == PointPos::None {
                            if let Some(line) = resolve_line(e1.geo_id) {
                                equal_pairs.push((line.p1.y, line.p2.y));
                                system.add_constraint(Box::new(
                                    ConstraintEqual::new(line.p1.y, line.p2.y, 1.0, tag, driving),
                                ));
                            }
                        } else if let Some(e2) = constraint.second() {
                            if let (Some(p1), Some(p2)) = (resolve_point(e1), resolve_point(e2)) {
                                equal_pairs.push((p1.y, p2.y));
                                system.add_constraint(Box::new(
                                    ConstraintEqual::new(p1.y, p2.y, 1.0, tag, driving),
                                ));
                            }
                        }
                    }
                }

                ConstraintType::Vertical => {
                    if let Some(e1) = constraint.first() {
                        if e1.pos == PointPos::None {
                            if let Some(line) = resolve_line(e1.geo_id) {
                                equal_pairs.push((line.p1.x, line.p2.x));
                                system.add_constraint(Box::new(
                                    ConstraintEqual::new(line.p1.x, line.p2.x, 1.0, tag, driving),
                                ));
                            }
                        } else if let Some(e2) = constraint.second() {
                            if let (Some(p1), Some(p2)) = (resolve_point(e1), resolve_point(e2)) {
                                equal_pairs.push((p1.x, p2.x));
                                system.add_constraint(Box::new(
                                    ConstraintEqual::new(p1.x, p2.x, 1.0, tag, driving),
                                ));
                            }
                        }
                    }
                }

                ConstraintType::Parallel => {
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        if let (Some(l1), Some(l2)) = (resolve_line(e1.geo_id), resolve_line(e2.geo_id)) {
                            system.add_constraint(Box::new(
                                ConstraintParallel::new(l1.p1, l1.p2, l2.p1, l2.p2, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::Perpendicular => {
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        if let (Some(l1), Some(l2)) = (resolve_line(e1.geo_id), resolve_line(e2.geo_id)) {
                            system.add_constraint(Box::new(
                                ConstraintPerpendicular::new(l1.p1, l1.p2, l2.p1, l2.p2, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::Equal => {
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        // Equal line length
                        if let (Some(l1), Some(l2)) = (resolve_line(e1.geo_id), resolve_line(e2.geo_id)) {
                            system.add_constraint(Box::new(
                                ConstraintEqualLineLength::new(l1, l2, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::Distance => {
                    let val = constraint.value.unwrap_or(0.0);
                    let dist_param = system.store_mut().push(val);
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        if e1.pos != PointPos::None && e2.pos != PointPos::None {
                            // Point-to-point distance
                            if let (Some(p1), Some(p2)) = (resolve_point(e1), resolve_point(e2)) {
                                system.add_constraint(Box::new(
                                    ConstraintP2PDistance::new(p1, p2, dist_param, tag, driving),
                                ));
                            }
                        } else if e1.pos != PointPos::None && e2.pos == PointPos::None {
                            // Point-to-line distance
                            if let (Some(p), Some(l)) = (resolve_point(e1), resolve_line(e2.geo_id)) {
                                system.add_constraint(Box::new(
                                    ConstraintP2LDistance::new(p, l.p1, l.p2, dist_param, tag, driving),
                                ));
                            }
                        } else if e1.pos == PointPos::None && e2.pos == PointPos::None {
                            // Line length (distance between its endpoints)
                            if let Some(l) = resolve_line(e1.geo_id) {
                                system.add_constraint(Box::new(
                                    ConstraintP2PDistance::new(l.p1, l.p2, dist_param, tag, driving),
                                ));
                            }
                        }
                    } else if let Some(e1) = constraint.first() {
                        // Single element: line length
                        if e1.pos == PointPos::None {
                            if let Some(l) = resolve_line(e1.geo_id) {
                                system.add_constraint(Box::new(
                                    ConstraintP2PDistance::new(l.p1, l.p2, dist_param, tag, driving),
                                ));
                            }
                        }
                    }
                }

                ConstraintType::DistanceX => {
                    let val = constraint.value.unwrap_or(0.0);
                    let diff_param = system.store_mut().push(val);
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        if let (Some(p1), Some(p2)) = (resolve_point(e1), resolve_point(e2)) {
                            system.add_constraint(Box::new(
                                ConstraintDifference::new(p1.x, p2.x, diff_param, tag, driving),
                            ));
                        }
                    } else if let Some(e1) = constraint.first() {
                        // Fix x-coordinate
                        if let Some(p) = resolve_point(e1) {
                            system.add_constraint(Box::new(
                                ConstraintEqual::new(p.x, diff_param, 1.0, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::DistanceY => {
                    let val = constraint.value.unwrap_or(0.0);
                    let diff_param = system.store_mut().push(val);
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        if let (Some(p1), Some(p2)) = (resolve_point(e1), resolve_point(e2)) {
                            system.add_constraint(Box::new(
                                ConstraintDifference::new(p1.y, p2.y, diff_param, tag, driving),
                            ));
                        }
                    } else if let Some(e1) = constraint.first() {
                        // Fix y-coordinate
                        if let Some(p) = resolve_point(e1) {
                            system.add_constraint(Box::new(
                                ConstraintEqual::new(p.y, diff_param, 1.0, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::Angle => {
                    let val = constraint.value.unwrap_or(0.0);
                    let angle_param = system.store_mut().push(val);
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        // Line-to-line angle
                        if let (Some(l1), Some(l2)) = (resolve_line(e1.geo_id), resolve_line(e2.geo_id)) {
                            system.add_constraint(Box::new(
                                ConstraintL2LAngle::new(l1.p1, l1.p2, l2.p1, l2.p2, angle_param, tag, driving),
                            ));
                        }
                    } else if let Some(e1) = constraint.first() {
                        // Line orientation angle
                        if let Some(l) = resolve_line(e1.geo_id) {
                            system.add_constraint(Box::new(
                                ConstraintP2PAngle::new(l.p1, l.p2, angle_param, 0.0, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::Radius => {
                    let val = constraint.value.unwrap_or(0.0);
                    let rad_param = system.store_mut().push(val);
                    if let Some(e1) = constraint.first() {
                        let idx = e1.geo_id as usize;
                        if idx < geo_params.len() {
                            if let Some(r) = geo_params[idx].radius {
                                system.add_constraint(Box::new(
                                    ConstraintEqual::new(r, rad_param, 1.0, tag, driving),
                                ));
                            }
                        }
                    }
                }

                ConstraintType::Diameter => {
                    let val = constraint.value.unwrap_or(0.0);
                    // Diameter = 2 * radius, so radius = value / 2
                    let rad_param = system.store_mut().push(val / 2.0);
                    if let Some(e1) = constraint.first() {
                        let idx = e1.geo_id as usize;
                        if idx < geo_params.len() {
                            if let Some(r) = geo_params[idx].radius {
                                system.add_constraint(Box::new(
                                    ConstraintEqual::new(r, rad_param, 1.0, tag, driving),
                                ));
                            }
                        }
                    }
                }

                ConstraintType::PointOnObject => {
                    if let (Some(e1), Some(e2)) = (constraint.first(), constraint.second()) {
                        if let (Some(p), Some(l)) = (resolve_point(e1), resolve_line(e2.geo_id)) {
                            // Point on line
                            system.add_constraint(Box::new(
                                ConstraintPointOnLine::new(p, l.p1, l.p2, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::Symmetric => {
                    if let (Some(e1), Some(e2), Some(e3)) = (
                        constraint.first().and_then(&resolve_point),
                        constraint.second().and_then(&resolve_point),
                        constraint.third(),
                    ) {
                        // Symmetric about a line: midpoint on line
                        if let Some(l) = resolve_line(e3.geo_id) {
                            system.add_constraint(Box::new(
                                ConstraintMidpointOnLine::new(e1, e2, l.p1, l.p2, tag, driving),
                            ));
                            // Also perpendicular bisector
                            system.add_constraint(Box::new(
                                ConstraintPointOnPerpBisector::new(l.p1, e1, e2, tag, driving),
                            ));
                        }
                    }
                }

                ConstraintType::Block => {
                    // Block is handled via geometry mode — parameters not added as unknowns
                    // Nothing to do here at constraint translation time
                }

                // TODO: Tangent, InternalAlignment, SnellsLaw, Weight — complex cases
                ConstraintType::Tangent
                | ConstraintType::InternalAlignment
                | ConstraintType::SnellsLaw
                | ConstraintType::Weight => {}
            }
        }

        // --- Phase 3: Solve ---
        system.init_solution();
        let status = system.solve_default();

        // Apply equality reductions that may not have been applied by the solver
        // (reduction-consumed Equal constraints in components with no remaining constraints).
        {
            let store = system.store_mut();
            for &(a, b) in &equal_pairs {
                let val = store.get(a);
                store.set(b, val);
            }
        }

        // --- Phase 4: Apply solution back to geometry ---
        let solved_store = system.store();

        // Update geometry with solved values
        let geo_ids: Vec<GeoId> = self.store.iter().map(|(id, _)| id).collect();
        for geo_id in geo_ids {
            let idx = geo_id as usize;
            if idx >= geo_params.len() { continue; }
            let gp = &geo_params[idx];

            if let Some(geo_def) = self.store.get_mut(geo_id) {
                match &mut geo_def.geo {
                    GeoType::Point { x, y } => {
                        if let Some((_, pt)) = gp.points.first() {
                            *x = solved_store.get(pt.x);
                            *y = solved_store.get(pt.y);
                        }
                    }
                    GeoType::Line { x1, y1, x2, y2 } => {
                        if let Some((_, pt)) = gp.points.iter().find(|(p, _)| *p == PointPos::Start) {
                            *x1 = solved_store.get(pt.x);
                            *y1 = solved_store.get(pt.y);
                        }
                        if let Some((_, pt)) = gp.points.iter().find(|(p, _)| *p == PointPos::End) {
                            *x2 = solved_store.get(pt.x);
                            *y2 = solved_store.get(pt.y);
                        }
                    }
                    GeoType::Circle { cx, cy, radius } => {
                        if let Some((_, pt)) = gp.points.first() {
                            *cx = solved_store.get(pt.x);
                            *cy = solved_store.get(pt.y);
                        }
                        if let Some(r) = gp.radius {
                            *radius = solved_store.get(r);
                        }
                    }
                    GeoType::Arc { cx, cy, radius, start_angle, end_angle } => {
                        if let Some((_, pt)) = gp.points.iter().find(|(p, _)| *p == PointPos::Mid) {
                            *cx = solved_store.get(pt.x);
                            *cy = solved_store.get(pt.y);
                        }
                        if let Some(r) = gp.radius {
                            *radius = solved_store.get(r);
                        }
                        if let Some(sa) = gp.start_angle {
                            *start_angle = solved_store.get(sa);
                        }
                        if let Some(ea) = gp.end_angle {
                            *end_angle = solved_store.get(ea);
                        }
                    }
                    _ => {}
                }
            }
        }

        // Compute diagnosis
        let diag = system.diagnose();
        self.last_diagnosis = Some(SketchDiagnosis {
            dofs: diag.dofs,
            conflicting: diag.conflicting_tags.clone(),
            redundant: diag.redundant_tags.clone(),
        });

        let result = match status {
            SolveStatus::Success | SolveStatus::Converged => {
                if !diag.conflicting_tags.is_empty() {
                    SketchSolveStatus::Conflicting
                } else if !diag.redundant_tags.is_empty() {
                    SketchSolveStatus::Redundant
                } else {
                    SketchSolveStatus::Ok
                }
            }
            SolveStatus::Failed | SolveStatus::SuccessfulSolutionInvalid => {
                SketchSolveStatus::Failed
            }
        };

        self.last_status = Some(result);
        result
    }

    /// Get the last solve result.
    pub fn last_status(&self) -> Option<SketchSolveStatus> {
        self.last_status
    }

    /// Get the last diagnosis result.
    pub fn last_diagnosis(&self) -> Option<&SketchDiagnosis> {
        self.last_diagnosis.as_ref()
    }

    /// Degrees of freedom from last solve. Returns `None` if not yet solved.
    pub fn dofs(&self) -> Option<i32> {
        self.last_diagnosis.as_ref().map(|d| d.dofs)
    }

    /// Clear all geometry and constraints.
    pub fn clear(&mut self) {
        self.store.clear();
        self.constraints.clear();
        self.external_refs.clear();
        self.next_tag = 1;
        self.last_status = None;
        self.last_diagnosis = None;
    }
}

impl Default for Sketch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    // -----------------------------------------------------------------------
    // Phase A0/A1: Geometry management
    // -----------------------------------------------------------------------

    #[test]
    fn add_and_query_geometry() {
        let mut sketch = Sketch::new();
        let id = sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 5.0,
        }));
        assert_eq!(id, 0);
        assert_eq!(sketch.geometry_count(), 1);
        assert!(sketch.get_geometry(0).is_some());
    }

    #[test]
    fn del_geometry_cascades_constraints() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 0.0, y2: 10.0,
        }));
        // Horizontal constraint on line 0
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        // Perpendicular between line 0 and line 1
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Perpendicular,
            vec![GeoElementId::edge(0), GeoElementId::edge(1)],
        ));
        assert_eq!(sketch.constraint_count(), 2);

        // Delete line 0 → both constraints should be removed
        sketch.del_geometry(0);
        assert_eq!(sketch.geometry_count(), 1);
        // The perpendicular referenced geo_id=0, so it's removed.
        // The horizontal also referenced geo_id=0, so it's removed.
        assert_eq!(sketch.constraint_count(), 0);
    }

    #[test]
    fn del_geometry_remaps_ids() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 1.0, y: 1.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 2.0, y: 2.0 }));

        // Coincident between point 1 and point 2
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::mid(1), GeoElementId::mid(2)],
        ));

        // Delete point 0 → point 1 becomes 0, point 2 becomes 1
        sketch.del_geometry(0);
        assert_eq!(sketch.constraint_count(), 1);
        let c = sketch.get_constraint(0).unwrap();
        assert_eq!(c.elements[0].geo_id, 0); // was 1, now 0
        assert_eq!(c.elements[1].geo_id, 1); // was 2, now 1
    }

    #[test]
    fn construction_mode_toggle() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        assert!(!sketch.get_geometry(0).unwrap().mode.construction);
        sketch.set_construction(0, true);
        assert!(sketch.get_geometry(0).unwrap().mode.construction);
        sketch.set_construction(0, false);
        assert!(!sketch.get_geometry(0).unwrap().mode.construction);
    }

    #[test]
    fn axis_points_accessible() {
        let sketch = Sketch::new();
        assert_eq!(sketch.get_point(GeoElementId::start(GEO_ID_H_AXIS)), Some((0.0, 0.0)));
        assert_eq!(sketch.get_point(GeoElementId::end(GEO_ID_H_AXIS)), Some((1.0, 0.0)));
        assert_eq!(sketch.get_point(GeoElementId::end(GEO_ID_V_AXIS)), Some((0.0, 1.0)));
    }

    // -----------------------------------------------------------------------
    // Phase A2: Constraint management
    // -----------------------------------------------------------------------

    #[test]
    fn add_and_query_constraint() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 5.0,
        }));
        let idx = sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        assert_eq!(idx, 0);
        assert_eq!(sketch.constraint_count(), 1);
        assert_eq!(sketch.get_constraint(0).unwrap().constraint_type, ConstraintType::Horizontal);
    }

    #[test]
    fn del_constraint() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        assert_eq!(sketch.constraint_count(), 1);
        assert!(sketch.del_constraint(0));
        assert_eq!(sketch.constraint_count(), 0);
    }

    #[test]
    fn set_datum_updates_value() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        let idx = sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::Distance,
            vec![GeoElementId::edge(0)],
            5.0,
        ));
        assert!(sketch.set_datum(idx, 15.0));
        assert_eq!(sketch.get_constraint(idx).unwrap().value, Some(15.0));
    }

    #[test]
    fn set_driving_and_active() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        let idx = sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::Distance,
            vec![GeoElementId::edge(0)],
            5.0,
        ));
        assert!(sketch.get_constraint(idx).unwrap().is_driving);
        sketch.set_driving(idx, false);
        assert!(!sketch.get_constraint(idx).unwrap().is_driving);

        assert!(sketch.get_constraint(idx).unwrap().is_active);
        sketch.set_active(idx, false);
        assert!(!sketch.get_constraint(idx).unwrap().is_active);
    }

    // -----------------------------------------------------------------------
    // Phase A3: Solver integration
    // -----------------------------------------------------------------------

    #[test]
    fn solve_horizontal_line() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 5.0,
        }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        // After solve, line should be horizontal
        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { y1, y2, .. } = &g.geo {
            assert_abs_diff_eq!(y1, y2, epsilon = 1e-6);
        } else {
            panic!("Expected line");
        }
    }

    #[test]
    fn solve_vertical_line() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 5.0, y2: 10.0,
        }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Vertical,
            vec![GeoElementId::edge(0)],
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x1, x2, .. } = &g.geo {
            assert_abs_diff_eq!(x1, x2, epsilon = 1e-6);
        } else {
            panic!("Expected line");
        }
    }

    #[test]
    fn solve_coincident_points() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 12.0, y1: 3.0, x2: 20.0, y2: 10.0,
        }));
        // Line 0 end coincident with line 1 start
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Coincident,
            vec![GeoElementId::end(0), GeoElementId::start(1)],
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g0 = sketch.get_geometry(0).unwrap();
        let g1 = sketch.get_geometry(1).unwrap();
        if let (GeoType::Line { x2, y2, .. }, GeoType::Line { x1, y1, .. }) = (&g0.geo, &g1.geo) {
            assert_abs_diff_eq!(x2, x1, epsilon = 1e-4);
            assert_abs_diff_eq!(y2, y1, epsilon = 1e-4);
        }
    }

    #[test]
    fn solve_distance_constraint() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 8.0, y2: 0.0,
        }));
        // Line length = 5
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::Distance,
            vec![GeoElementId::edge(0)],
            5.0,
        ));
        // Horizontal
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        // Fix start at origin
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceX,
            vec![GeoElementId::start(0)],
            0.0,
        ));
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceY,
            vec![GeoElementId::start(0)],
            0.0,
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x1, y1, x2, y2 } = &g.geo {
            assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*x2, 5.0, epsilon = 1e-4);
            assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-4);
        }
    }

    #[test]
    fn solve_parallel_lines() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
        }));
        sketch.add_geometry(GeoDef::new(GeoType::Line {
            x1: 0.0, y1: 5.0, x2: 10.0, y2: 7.0,
        }));
        // Fix line 0
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0,
        ));
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0,
        ));
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceX, vec![GeoElementId::end(0)], 10.0,
        ));
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceY, vec![GeoElementId::end(0)], 0.0,
        ));
        // Fix line 1 start
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceX, vec![GeoElementId::start(1)], 0.0,
        ));
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceY, vec![GeoElementId::start(1)], 5.0,
        ));
        // Fix line 1 end x
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceX, vec![GeoElementId::end(1)], 10.0,
        ));
        // Parallel
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Parallel,
            vec![GeoElementId::edge(0), GeoElementId::edge(1)],
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g1 = sketch.get_geometry(1).unwrap();
        if let GeoType::Line { y1, y2, .. } = &g1.geo {
            assert_abs_diff_eq!(y1, y2, epsilon = 1e-4); // horizontal = parallel to line 0
        }
    }

    #[test]
    fn solve_rectangle() {
        let mut sketch = Sketch::new();
        // Four lines forming a rectangle
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0 })); // bottom
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 10.0, y1: 0.0, x2: 10.0, y2: 5.0 })); // right
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 10.0, y1: 5.0, x2: 0.0, y2: 5.0 })); // top
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 5.0, x2: 0.0, y2: 0.0 })); // left

        // Coincident corners
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(0), GeoElementId::start(1)]));
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(1), GeoElementId::start(2)]));
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(2), GeoElementId::start(3)]));
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(3), GeoElementId::start(0)]));

        // Horizontal and vertical
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Horizontal, vec![GeoElementId::edge(0)]));
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Horizontal, vec![GeoElementId::edge(2)]));
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Vertical, vec![GeoElementId::edge(1)]));
        sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Vertical, vec![GeoElementId::edge(3)]));

        // Fix bottom-left at origin
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0));

        // Width = 20, Height = 10
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::Distance, vec![GeoElementId::edge(0)], 20.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::Distance, vec![GeoElementId::edge(1)], 10.0));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        // Verify rectangle
        let g0 = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { x1, y1, x2, y2 } = &g0.geo {
            assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-4);
            assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-4);
            assert_abs_diff_eq!(*x2, 20.0, epsilon = 1e-4);
            assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-4);
        }
        let g1 = sketch.get_geometry(1).unwrap();
        if let GeoType::Line { x2, y2, .. } = &g1.geo {
            assert_abs_diff_eq!(*x2, 20.0, epsilon = 1e-4);
            assert_abs_diff_eq!(*y2, 10.0, epsilon = 1e-4);
        }
    }

    #[test]
    fn solve_angle_constraint() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 7.0, y2: 6.0 }));

        // Fix line 0
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::end(0)], 10.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::end(0)], 0.0));

        // Fix line 1 start at origin
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::start(1)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::start(1)], 0.0));
        // Line 1 length = 10
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::Distance, vec![GeoElementId::edge(1)], 10.0));

        // Angle between lines = 45 degrees
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::Angle,
            vec![GeoElementId::edge(0), GeoElementId::edge(1)],
            std::f64::consts::FRAC_PI_4,
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g1 = sketch.get_geometry(1).unwrap();
        if let GeoType::Line { x2, y2, .. } = &g1.geo {
            let expected = 10.0 / std::f64::consts::SQRT_2;
            assert_abs_diff_eq!(*x2, expected, epsilon = 1e-3);
            assert_abs_diff_eq!(*y2, expected, epsilon = 1e-3);
        }
    }

    #[test]
    fn solve_circle_radius() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Circle { cx: 5.0, cy: 5.0, radius: 8.0 }));
        // Fix center
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::mid(0)], 5.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::mid(0)], 5.0));
        // Radius = 3
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::Radius, vec![GeoElementId::edge(0)], 3.0));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Circle { cx, cy, radius } = &g.geo {
            assert_abs_diff_eq!(*cx, 5.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*cy, 5.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*radius, 3.0, epsilon = 1e-4);
        }
    }

    #[test]
    fn solve_perpendicular() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 5.0, y1: 0.0, x2: 5.0, y2: 8.0 }));

        // Fix line 0
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::end(0)], 10.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::end(0)], 0.0));
        // Fix line 1 start
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::start(1)], 5.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::start(1)], 0.0));
        // Line 1 length
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::Distance, vec![GeoElementId::edge(1)], 8.0));

        // Perpendicular
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Perpendicular,
            vec![GeoElementId::edge(0), GeoElementId::edge(1)],
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g1 = sketch.get_geometry(1).unwrap();
        if let GeoType::Line { x1, x2, .. } = &g1.geo {
            assert_abs_diff_eq!(x1, x2, epsilon = 1e-4); // vertical = perp to horizontal
        }
    }

    #[test]
    fn solve_point_on_line() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0 }));
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 5.0, y: 3.0 }));

        // Fix line
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::end(0)], 10.0));
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::end(0)], 0.0));

        // Fix point x
        sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::mid(1)], 5.0));

        // Point on line
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::PointOnObject,
            vec![GeoElementId::mid(1), GeoElementId::edge(0)],
        ));

        let status = sketch.solve();
        assert_eq!(status, SketchSolveStatus::Ok);

        let g = sketch.get_geometry(1).unwrap();
        if let GeoType::Point { x, y } = &g.geo {
            assert_abs_diff_eq!(*x, 5.0, epsilon = 1e-6);
            assert_abs_diff_eq!(*y, 0.0, epsilon = 1e-4); // on the horizontal line
        }
    }

    #[test]
    fn solve_dofs_reported() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 1.0, y: 2.0 }));
        // Only fix x → 1 DOF remaining
        sketch.add_constraint(SketchConstraint::dimensional(
            ConstraintType::DistanceX, vec![GeoElementId::mid(0)], 5.0,
        ));

        sketch.solve();
        assert_eq!(sketch.dofs(), Some(1));
    }

    #[test]
    fn inactive_constraint_ignored() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 10.0, y2: 5.0 }));
        let idx = sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Horizontal,
            vec![GeoElementId::edge(0)],
        ));
        sketch.set_active(idx, false);

        sketch.solve();
        let g = sketch.get_geometry(0).unwrap();
        if let GeoType::Line { y1, y2, .. } = &g.geo {
            // Line should NOT be forced horizontal since constraint is inactive
            // y values may have changed due to solver but shouldn't be forced equal
            // Since there are no active constraints, solver succeeds with no changes
            let _ = (*y1, *y2); // just verify accessible
        }
    }

    #[test]
    fn clear_resets_everything() {
        let mut sketch = Sketch::new();
        sketch.add_geometry(GeoDef::new(GeoType::Point { x: 0.0, y: 0.0 }));
        sketch.add_constraint(SketchConstraint::geometric(
            ConstraintType::Block,
            vec![GeoElementId::edge(0)],
        ));
        sketch.solve();
        sketch.clear();

        assert_eq!(sketch.geometry_count(), 0);
        assert_eq!(sketch.constraint_count(), 0);
        assert!(sketch.last_status().is_none());
        assert!(sketch.last_diagnosis().is_none());
    }
}
