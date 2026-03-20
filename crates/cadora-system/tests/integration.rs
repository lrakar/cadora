//! End-to-end integration tests for the CADORA solver — realistic 2D sketch scenarios.

use approx::assert_abs_diff_eq;
use cadora_constraints::*;
use cadora_core::{Algorithm, ParamIdx, ParamStore, Point, SolveStatus, SolverConfig};
use cadora_geo::curve::Line;
use cadora_system::System;

/// Helper: create a point with coordinates (x, y).
fn mk_point(store: &mut ParamStore, x: f64, y: f64) -> (Point, Vec<ParamIdx>) {
    let px = store.push(x);
    let py = store.push(y);
    (Point::new(px, py), vec![px, py])
}

// ---------------------------------------------------------------------------
// Scenario 1: Axis-aligned rectangle with fixed width and height
// ---------------------------------------------------------------------------
//
// Four points forming a rectangle:
//   p1 -- p2
//   |      |
//   p0 -- p3
//
// Constraints:
//   - p0 is fixed at origin (x=0, y=0)
//   - p1.x = p0.x  (left edge vertical)
//   - p2.y = p1.y  (top edge horizontal)
//   - p3.y = p0.y  (bottom edge horizontal)
//   - p2.x = p3.x  (right edge vertical)
//   - dist(p0, p3) = 5  (width)
//   - dist(p0, p1) = 3  (height)

#[test]
fn rectangle_with_fixed_corner() {
    let mut store = ParamStore::new();

    // Points — start with rough guesses
    let (p0, mut params) = mk_point(&mut store, 0.1, 0.1);
    let (p1, pv) = mk_point(&mut store, 0.2, 2.5);
    params.extend(pv);
    let (p2, pv) = mk_point(&mut store, 4.8, 2.6);
    params.extend(pv);
    let (p3, pv) = mk_point(&mut store, 4.7, 0.2);
    params.extend(pv);

    // Distance params
    let width = store.push(5.0);
    let height = store.push(3.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix p0 at origin
    let zero_x = sys.store_mut().push(0.0);
    let zero_y = sys.store_mut().push(0.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(p0.x, zero_x, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(p0.y, zero_y, 1.0, 2, true)));

    // Vertical left edge: p1.x = p0.x
    sys.add_constraint(Box::new(ConstraintEqual::new(p1.x, p0.x, 1.0, 3, true)));
    // Horizontal top: p2.y = p1.y
    sys.add_constraint(Box::new(ConstraintEqual::new(p2.y, p1.y, 1.0, 4, true)));
    // Horizontal bottom: p3.y = p0.y
    sys.add_constraint(Box::new(ConstraintEqual::new(p3.y, p0.y, 1.0, 5, true)));
    // Vertical right edge: p2.x = p3.x
    sys.add_constraint(Box::new(ConstraintEqual::new(p2.x, p3.x, 1.0, 6, true)));

    // Width: dist(p0, p3) = 5
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(p0, p3, width, 7, true)));
    // Height: dist(p0, p1) = 3
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(p0, p1, height, 8, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    assert_abs_diff_eq!(s.get(p0.x), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p0.y), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p1.x), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p1.y), 3.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p2.x), 5.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p2.y), 3.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p3.x), 5.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p3.y), 0.0, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// Scenario 2: Equilateral triangle via three distance constraints
// ---------------------------------------------------------------------------

#[test]
fn equilateral_triangle() {
    let mut store = ParamStore::new();

    // Three points, rough initial positions
    let (p0, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (p1, pv) = mk_point(&mut store, 4.5, 0.3);
    params.extend(pv);
    let (p2, pv) = mk_point(&mut store, 2.0, 3.5);
    params.extend(pv);

    // Side length = 5
    let side = store.push(5.0);

    // Fix p0 at origin
    let zx = store.push(0.0);
    let zy = store.push(0.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix p0
    sys.add_constraint(Box::new(ConstraintEqual::new(p0.x, zx, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(p0.y, zy, 1.0, 2, true)));

    // Fix p1.y = 0 (base on x-axis)
    sys.add_constraint(Box::new(ConstraintEqual::new(p1.y, zy, 1.0, 3, true)));

    // Three equal sides
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(p0, p1, side, 4, true)));
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(p1, p2, side, 5, true)));
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(p2, p0, side, 6, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // p0 at origin
    assert_abs_diff_eq!(s.get(p0.x), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p0.y), 0.0, epsilon = 1e-6);
    // p1 at (5, 0)
    assert_abs_diff_eq!(s.get(p1.x), 5.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p1.y), 0.0, epsilon = 1e-6);
    // p2 at (2.5, 5*sqrt(3)/2)
    assert_abs_diff_eq!(s.get(p2.x), 2.5, epsilon = 1e-4);
    assert_abs_diff_eq!(s.get(p2.y), 5.0 * 3.0_f64.sqrt() / 2.0, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// Scenario 3: Two independent shapes solved simultaneously
// ---------------------------------------------------------------------------

#[test]
fn two_independent_shapes() {
    let mut store = ParamStore::new();

    // Shape A: horizontal line from (0,0) to (x, 0), distance = 10
    let (a1, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (a2, pv) = mk_point(&mut store, 8.0, 0.5);
    params.extend(pv);
    let ad = store.push(10.0);

    // Shape B: vertical line from (0,0) to (0, y), distance = 7
    let (b1, pv) = mk_point(&mut store, 20.0, 20.0);
    params.extend(pv);
    let (b2, pv) = mk_point(&mut store, 20.5, 26.0);
    params.extend(pv);
    let bd = store.push(7.0);

    let z = store.push(0.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Shape A: fix a1 at origin, a2 on x-axis
    sys.add_constraint(Box::new(ConstraintEqual::new(a1.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(a1.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(a2.y, z, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(a1, a2, ad, 4, true)));

    // Shape B: fix b1, b2 on vertical line
    let b_ref_x = sys.store_mut().push(20.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(b1.x, b_ref_x, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(b1.y, z, 1.0, 6, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(b2.x, b_ref_x, 1.0, 7, true)));
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(b1, b2, bd, 8, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Shape A: line along x-axis of length 10
    assert_abs_diff_eq!(s.get(a1.x), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(a2.x), 10.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(a2.y), 0.0, epsilon = 1e-6);

    // Shape B: vertical line at x=20, length 7
    assert_abs_diff_eq!(s.get(b1.x), 20.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(b2.x), 20.0, epsilon = 1e-6);
    // b2.y should be ±7 from b1.y=0
    assert_abs_diff_eq!(s.get(b2.y).abs(), 7.0, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// Scenario 4: Over-constrained system diagnosis
// ---------------------------------------------------------------------------

#[test]
fn overconstrained_diagnosis() {
    let mut store = ParamStore::new();
    let x = store.push(5.0);

    let t1 = store.push(3.0);
    let t2 = store.push(4.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(vec![x]);

    // Two conflicting constraints: x = 3 AND x = 4
    sys.add_constraint(Box::new(ConstraintEqual::new(x, t1, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(x, t2, 1.0, 2, true)));

    let result = sys.diagnose();
    // 1 param, 2 constraints → DOF should be negative (over-constrained)
    assert!(result.dofs < 0, "Expected negative DOFs, got {}", result.dofs);
    // Should detect conflicting constraints
    assert!(!result.conflicting_tags.is_empty(), "Should have conflicting tags");
}

// ---------------------------------------------------------------------------
// Scenario 5: Under-constrained system diagnosis
// ---------------------------------------------------------------------------

#[test]
fn underconstrained_diagnosis() {
    let mut store = ParamStore::new();
    let (p, params) = mk_point(&mut store, 3.0, 4.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Only constrain x, leave y free
    let target = sys.store_mut().push(5.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(p.x, target, 1.0, 1, true)));

    let result = sys.diagnose();
    assert_eq!(result.dofs, 1, "1 free DOF (y is unconstrained)");
    // y should be in the dependent params
    assert!(
        result.dependent_params.contains(&p.y),
        "p.y should be a dependent (unconstrained) param"
    );
}

// ---------------------------------------------------------------------------
// Scenario 6: Solve then re-solve after changing a constraint
// ---------------------------------------------------------------------------

#[test]
fn re_solve_after_constraint_change() {
    let mut store = ParamStore::new();
    let (p, params) = mk_point(&mut store, 1.0, 1.0);
    let d1 = store.push(3.0);
    let d2 = store.push(4.0);
    let _z = store.push(0.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params.clone());

    // Pin at (3, 4) via distance from origin along each axis
    sys.add_constraint(Box::new(ConstraintEqual::new(p.x, d1, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(p.y, d2, 1.0, 2, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);
    assert_abs_diff_eq!(sys.store().get(p.x), 3.0, epsilon = 1e-8);
    assert_abs_diff_eq!(sys.store().get(p.y), 4.0, epsilon = 1e-8);

    // Now remove tag 2 and add new constraint: y = 10
    sys.clear_by_tag(2);
    let d3 = sys.store_mut().push(10.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(p.y, d3, 1.0, 3, true)));
    sys.declare_unknowns(params);
    sys.init_solution();
    let status2 = sys.solve_default();
    assert_eq!(status2, SolveStatus::Success);
    assert_abs_diff_eq!(sys.store().get(p.x), 3.0, epsilon = 1e-8);
    assert_abs_diff_eq!(sys.store().get(p.y), 10.0, epsilon = 1e-8);
}

// ---------------------------------------------------------------------------
// Scenario 7: Multiple algorithms produce the same result
// ---------------------------------------------------------------------------

#[test]
fn all_algorithms_agree() {
    for alg in [Algorithm::Bfgs, Algorithm::LevenbergMarquardt, Algorithm::DogLeg] {
        let mut store = ParamStore::new();
        let (p, params) = mk_point(&mut store, 1.0, 1.0);
        let tx = store.push(7.0);
        let ty = store.push(11.0);

        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(params);
        sys.add_constraint(Box::new(ConstraintEqual::new(p.x, tx, 1.0, 1, true)));
        sys.add_constraint(Box::new(ConstraintEqual::new(p.y, ty, 1.0, 2, true)));
        sys.init_solution();
        let status = sys.solve_with(true, alg);
        assert_eq!(status, SolveStatus::Success, "Failed with {:?}", alg);
        assert_abs_diff_eq!(sys.store().get(p.x), 7.0, epsilon = 1e-6);
        assert_abs_diff_eq!(sys.store().get(p.y), 11.0, epsilon = 1e-6);
    }
}

// ---------------------------------------------------------------------------
// Scenario 8: Point on line constraint
// ---------------------------------------------------------------------------

#[test]
fn point_on_line() {
    let mut store = ParamStore::new();

    // Line from (0,0) to (10,0) — horizontal
    let (l1, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (l2, pv) = mk_point(&mut store, 10.0, 0.0);
    params.extend(pv);

    // Point that should end up on the line
    let (p, pv) = mk_point(&mut store, 5.0, 3.0);
    params.extend(pv);

    // Fix the x coordinate of p
    let target_x = store.push(5.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix line endpoints — they are known
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l1.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2.y, z, 1.0, 4, true)));

    // Point on line
    sys.add_constraint(Box::new(ConstraintPointOnLine::new(p, l1, l2, 5, true)));
    // Fix p.x
    sys.add_constraint(Box::new(ConstraintEqual::new(p.x, target_x, 1.0, 6, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    assert_abs_diff_eq!(s.get(p.x), 5.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p.y), 0.0, epsilon = 1e-4); // should be on the line (y=0)
}

// ===========================================================================
// FreeCAD Comparison Tests
// ===========================================================================
//
// These tests replicate scenarios from FreeCAD's planegcs test suite
// (TestSketcherSolver.py, planegcs/Constraints.cpp) to verify CADORA
// produces equivalent results.

// ---------------------------------------------------------------------------
// FreeCAD Box: 4 lines forming a rectangle with coincident corners,
// horizontal/vertical constraints, distance constraints, and fixed corner.
// Matches TestSketcherSolver.py CreateBoxSketchSet + testBoxCase
// ---------------------------------------------------------------------------

#[test]
fn freecad_box_rectangle() {
    let mut store = ParamStore::new();

    // 4 corners of the box — initial rough positions (like FreeCAD)
    let (bl, mut params) = mk_point(&mut store, -99.2, -53.2); // bottom-left
    let (tl, pv) = mk_point(&mut store, -99.0, 37.0);          // top-left
    params.extend(pv);
    let (tr, pv) = mk_point(&mut store, 69.4, 37.0);           // top-right
    params.extend(pv);
    let (br, pv) = mk_point(&mut store, 69.4, -53.2);          // bottom-right
    params.extend(pv);

    // Known dimensions
    let width = store.push(168.66);
    let height = store.push(90.16);
    let anchor_x = store.push(90.0);
    let anchor_y = store.push(-50.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Horizontal constraints: top edge, bottom edge
    // top: tl.y = tr.y
    sys.add_constraint(Box::new(ConstraintEqual::new(tl.y, tr.y, 1.0, 1, true)));
    // bottom: bl.y = br.y
    sys.add_constraint(Box::new(ConstraintEqual::new(bl.y, br.y, 1.0, 2, true)));

    // Vertical constraints: left edge, right edge
    // left: bl.x = tl.x
    sys.add_constraint(Box::new(ConstraintEqual::new(bl.x, tl.x, 1.0, 3, true)));
    // right: br.x = tr.x
    sys.add_constraint(Box::new(ConstraintEqual::new(br.x, tr.x, 1.0, 4, true)));

    // Width: distance(tl, tr) = 168.66
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(tl, tr, width, 5, true)));
    // Height: distance(br, tr) = 90.16
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(br, tr, height, 6, true)));

    // Fix bottom-right corner
    sys.add_constraint(Box::new(ConstraintEqual::new(br.x, anchor_x, 1.0, 7, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(br.y, anchor_y, 1.0, 8, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Bottom-right is fixed
    assert_abs_diff_eq!(s.get(br.x), 90.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(br.y), -50.0, epsilon = 1e-6);
    // Top-right: same x, y = br.y + height
    assert_abs_diff_eq!(s.get(tr.x), 90.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(tr.y), -50.0 + 90.16, epsilon = 1e-4);
    // Top-left: x = tr.x - width
    assert_abs_diff_eq!(s.get(tl.x), 90.0 - 168.66, epsilon = 1e-4);
    assert_abs_diff_eq!(s.get(tl.y), -50.0 + 90.16, epsilon = 1e-4);
    // Bottom-left
    assert_abs_diff_eq!(s.get(bl.x), 90.0 - 168.66, epsilon = 1e-4);
    assert_abs_diff_eq!(s.get(bl.y), -50.0, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// FreeCAD Square: 30×30 square at origin
// Matches TestSketcherSolver.py CreateRectangleSketch (corner=0,0, lengths=30,30)
// ---------------------------------------------------------------------------

#[test]
fn freecad_square_at_origin() {
    let mut store = ParamStore::new();

    // 4 corners: initial positions slightly off
    let (bl, mut params) = mk_point(&mut store, 0.1, 0.1);
    let (tl, pv) = mk_point(&mut store, 0.2, 29.8);
    params.extend(pv);
    let (tr, pv) = mk_point(&mut store, 29.8, 29.8);
    params.extend(pv);
    let (br, pv) = mk_point(&mut store, 29.8, 0.2);
    params.extend(pv);

    let side = store.push(30.0);
    let zero = store.push(0.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Horizontal top: tl.y = tr.y
    sys.add_constraint(Box::new(ConstraintEqual::new(tl.y, tr.y, 1.0, 1, true)));
    // Horizontal bottom: bl.y = br.y
    sys.add_constraint(Box::new(ConstraintEqual::new(bl.y, br.y, 1.0, 2, true)));
    // Vertical left: bl.x = tl.x
    sys.add_constraint(Box::new(ConstraintEqual::new(bl.x, tl.x, 1.0, 3, true)));
    // Vertical right: br.x = tr.x
    sys.add_constraint(Box::new(ConstraintEqual::new(br.x, tr.x, 1.0, 4, true)));

    // Fix bottom-left at origin
    sys.add_constraint(Box::new(ConstraintEqual::new(bl.x, zero, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(bl.y, zero, 1.0, 6, true)));

    // Equal side lengths: dist(bl,br) = dist(bl,tl) (square)
    // Width
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(bl, br, side, 7, true)));
    // Height = width (square) via equal constraint on y-distance
    // Alternative: just use same 'side' param for height
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(bl, tl, side, 8, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    assert_abs_diff_eq!(s.get(bl.x), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(bl.y), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(tl.x), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(tl.y), 30.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(tr.x), 30.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(tr.y), 30.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(br.x), 30.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(br.y), 0.0, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// Parallel lines: two lines with a parallel constraint
// ---------------------------------------------------------------------------

#[test]
fn parallel_lines() {
    let mut store = ParamStore::new();

    // Line 1: fixed horizontal at y=0
    let (l1a, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (l1b, pv) = mk_point(&mut store, 10.0, 0.0);
    params.extend(pv);

    // Line 2: starts at (2, 5), end at (8, 7) — slightly non-parallel
    let (l2a, pv) = mk_point(&mut store, 2.0, 5.0);
    params.extend(pv);
    let (l2b, pv) = mk_point(&mut store, 8.0, 7.0);
    params.extend(pv);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix line 1 endpoints
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.y, z, 1.0, 4, true)));

    // Fix l2a.x and l2b.x (only y is free)
    let two = sys.store_mut().push(2.0);
    let eight = sys.store_mut().push(8.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.x, two, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2b.x, eight, 1.0, 6, true)));

    // Parallel constraint
    sys.add_constraint(Box::new(ConstraintParallel::new(l1a, l1b, l2a, l2b, 7, true)));

    // Fix l2a.y to anchor it
    let five = sys.store_mut().push(5.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.y, five, 1.0, 8, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Lines should be parallel: l2b.y should equal l2a.y (both horizontal)
    assert_abs_diff_eq!(s.get(l2a.y), 5.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(l2b.y), 5.0, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// Perpendicular lines: two lines with perpendicular constraint
// ---------------------------------------------------------------------------

#[test]
fn perpendicular_lines() {
    let mut store = ParamStore::new();

    // Line 1: horizontal from (0,0) to (10,0) — fixed
    let (l1a, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (l1b, pv) = mk_point(&mut store, 10.0, 0.0);
    params.extend(pv);

    // Line 2: from (5,1) to (5,8) — nearly vertical but free
    let (l2a, pv) = mk_point(&mut store, 5.0, 1.0);
    params.extend(pv);
    let (l2b, pv) = mk_point(&mut store, 5.0, 8.0);
    params.extend(pv);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix line 1
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.y, z, 1.0, 4, true)));

    // Fix line 2 start
    let five = sys.store_mut().push(5.0);
    let one = sys.store_mut().push(1.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.x, five, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.y, one, 1.0, 6, true)));

    // Perpendicular constraint
    sys.add_constraint(Box::new(ConstraintPerpendicular::new(l1a, l1b, l2a, l2b, 7, true)));

    // Fix the length of line 2
    let len2 = sys.store_mut().push(7.0);
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(l2a, l2b, len2, 8, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // l2 should be vertical: l2b.x = l2a.x = 5.0
    assert_abs_diff_eq!(s.get(l2a.x), 5.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(l2b.x), 5.0, epsilon = 1e-4);
    // Verify dot product of directions is ~0 (perpendicular)
    let dx1 = s.get(l1b.x) - s.get(l1a.x);
    let dy1 = s.get(l1b.y) - s.get(l1a.y);
    let dx2 = s.get(l2b.x) - s.get(l2a.x);
    let dy2 = s.get(l2b.y) - s.get(l2a.y);
    assert_abs_diff_eq!(dx1 * dx2 + dy1 * dy2, 0.0, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// Line-to-line angle constraint
// ---------------------------------------------------------------------------

#[test]
fn l2l_angle_45_degrees() {
    let mut store = ParamStore::new();

    // Line 1: horizontal from origin
    let (l1a, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (l1b, pv) = mk_point(&mut store, 10.0, 0.0);
    params.extend(pv);

    // Line 2: starts at origin, end roughly at 45 degrees
    let (l2a, pv) = mk_point(&mut store, 0.0, 0.0);
    params.extend(pv);
    let (l2b, pv) = mk_point(&mut store, 7.0, 6.5); // rough 45-degree guess
    params.extend(pv);

    let angle = store.push(std::f64::consts::FRAC_PI_4); // 45 degrees

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix line 1 and line 2 start
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.y, z, 1.0, 4, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.x, z, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.y, z, 1.0, 6, true)));

    // Length of line 2 = 10
    let len = sys.store_mut().push(10.0);
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(l2a, l2b, len, 7, true)));

    // Angle between lines = 45 degrees
    sys.add_constraint(Box::new(ConstraintL2LAngle::new(l1a, l1b, l2a, l2b, angle, 8, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Line 2 end should be at (10*cos(45), 10*sin(45)) = (7.071, 7.071)
    let expected = 10.0 * std::f64::consts::FRAC_PI_4.cos();
    assert_abs_diff_eq!(s.get(l2b.x), expected, epsilon = 1e-4);
    assert_abs_diff_eq!(s.get(l2b.y), expected, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// Point-to-line distance
// ---------------------------------------------------------------------------

#[test]
fn point_to_line_distance() {
    let mut store = ParamStore::new();

    // Line: from (0,0) to (10,0) — horizontal, fixed
    let (la, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (lb, pv) = mk_point(&mut store, 10.0, 0.0);
    params.extend(pv);

    // Point at (5, 8) — y is free
    let (p, pv) = mk_point(&mut store, 5.0, 8.0);
    params.extend(pv);

    let dist = store.push(3.0); // should be 3 units from line

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix line and point x
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    let five = sys.store_mut().push(5.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(la.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(la.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(lb.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(lb.y, z, 1.0, 4, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(p.x, five, 1.0, 5, true)));

    // Point-to-line distance = 3
    sys.add_constraint(Box::new(ConstraintP2LDistance::new(p, la, lb, dist, 6, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Point should be at (5, 3) or (5, -3)
    assert_abs_diff_eq!(s.get(p.x), 5.0, epsilon = 1e-6);
    assert_abs_diff_eq!(s.get(p.y).abs(), 3.0, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// Midpoint on line
// ---------------------------------------------------------------------------

#[test]
fn midpoint_on_line_constraint() {
    let mut store = ParamStore::new();

    // Segment from (0,0) to (10,6)
    let (a, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (b, pv) = mk_point(&mut store, 10.0, 6.0);
    params.extend(pv);

    // Line (y-axis): from (5,0) to (5,10) — vertical
    let (la, pv) = mk_point(&mut store, 5.0, 0.0);
    params.extend(pv);
    let (lb, pv) = mk_point(&mut store, 5.0, 10.0);
    params.extend(pv);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix all points except b.y
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    let five = sys.store_mut().push(5.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(a.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(a.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(b.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(la.x, five, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(la.y, z, 1.0, 6, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(lb.x, five, 1.0, 7, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(lb.y, ten, 1.0, 8, true)));

    // Midpoint of (a,b) should be on line (la,lb)
    sys.add_constraint(Box::new(ConstraintMidpointOnLine::new(a, b, la, lb, 9, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Midpoint x = (0 + 10) / 2 = 5, which is on the vertical line x=5
    // The midpoint constraint forces (a.x+b.x)/2 = on line, and (a.y+b.y)/2 on line
    // Since a.x=0, b.x=10, midpoint.x=5; the line is x=5 so this is satisfied
    // b.y is free and solved by the midpoint constraint
    let mid_x = (s.get(a.x) + s.get(b.x)) / 2.0;
    assert_abs_diff_eq!(mid_x, 5.0, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// Tangent circle and line — tangent circumference constraint
// ---------------------------------------------------------------------------

#[test]
fn tangent_circle_and_line() {
    let mut store = ParamStore::new();

    // Circle: center at (5, 5), radius = 3
    let center = Point { x: store.push(5.0), y: store.push(5.0) };
    let radius = store.push(3.0);

    // Horizontal line at y=0: from (0,0) to (10,0)
    let (la, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (lb, pv) = mk_point(&mut store, 10.0, 0.0);
    params.extend(pv);

    // Circle center y is unknown, everything else fixed
    params.push(center.y);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix line, circle center x, and radius
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    let five = sys.store_mut().push(5.0);
    let three = sys.store_mut().push(3.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(la.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(la.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(lb.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(lb.y, z, 1.0, 4, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(center.x, five, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(radius, three, 1.0, 6, true)));

    // Tangency: distance from center to line = radius
    // P2LDistance(center, line) = radius → but P2LDistance uses a stored param
    // We'll use point-to-line distance constraint with the radius as distance param
    sys.add_constraint(Box::new(ConstraintP2LDistance::new(center, la, lb, radius, 7, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Circle should be tangent to y=0 line, so center.y = radius = 3
    assert_abs_diff_eq!(s.get(center.y), 3.0, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// Point-on-perpendicular-bisector
// ---------------------------------------------------------------------------

#[test]
fn point_on_perp_bisector() {
    let mut store = ParamStore::new();

    // Segment from (0, 0) to (10, 0)
    let (a, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (b, pv) = mk_point(&mut store, 10.0, 0.0);
    params.extend(pv);

    // Point that should be on the perpendicular bisector
    let (p, pv) = mk_point(&mut store, 4.0, 7.0);
    params.extend(pv);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix segment and p.y
    let z = sys.store_mut().push(0.0);
    let ten = sys.store_mut().push(10.0);
    let seven = sys.store_mut().push(7.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(a.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(a.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(b.x, ten, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(b.y, z, 1.0, 4, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(p.y, seven, 1.0, 5, true)));

    // Point on perpendicular bisector
    sys.add_constraint(Box::new(ConstraintPointOnPerpBisector::new(p, a, b, 6, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // p.x should be at midpoint x = 5.0
    assert_abs_diff_eq!(s.get(p.x), 5.0, epsilon = 1e-4);
    // Verify equidistant from a and b
    let da = ((s.get(p.x) - s.get(a.x)).powi(2) + (s.get(p.y) - s.get(a.y)).powi(2)).sqrt();
    let db = ((s.get(p.x) - s.get(b.x)).powi(2) + (s.get(p.y) - s.get(b.y)).powi(2)).sqrt();
    assert_abs_diff_eq!(da, db, epsilon = 1e-6);
}

// ---------------------------------------------------------------------------
// Equal line length constraint
// ---------------------------------------------------------------------------

#[test]
fn equal_line_lengths() {
    let mut store = ParamStore::new();

    // Line 1: from (0,0) to (5,0) — length 5
    let (l1a, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (l1b, pv) = mk_point(&mut store, 5.0, 0.0);
    params.extend(pv);

    // Line 2: from (0,3) to (8,3) — length 8, should be adjusted
    let (l2a, pv) = mk_point(&mut store, 0.0, 3.0);
    params.extend(pv);
    let (l2b, pv) = mk_point(&mut store, 8.0, 3.0);
    params.extend(pv);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix line 1 and fix l2a, l2a.y
    let z = sys.store_mut().push(0.0);
    let five = sys.store_mut().push(5.0);
    let three = sys.store_mut().push(3.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1a.y, z, 1.0, 2, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.x, five, 1.0, 3, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l1b.y, z, 1.0, 4, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.x, z, 1.0, 5, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2a.y, three, 1.0, 6, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(l2b.y, three, 1.0, 7, true)));

    // Equal line length
    let line1 = Line { p1: l1a, p2: l1b };
    let line2 = Line { p1: l2a, p2: l2b };
    sys.add_constraint(Box::new(ConstraintEqualLineLength::new(
        line1, line2, 8, true,
    )));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    // Line 2 should have length 5 (same as line 1)
    let len1 = ((s.get(l1b.x) - s.get(l1a.x)).powi(2) + (s.get(l1b.y) - s.get(l1a.y)).powi(2)).sqrt();
    let len2 = ((s.get(l2b.x) - s.get(l2a.x)).powi(2) + (s.get(l2b.y) - s.get(l2a.y)).powi(2)).sqrt();
    assert_abs_diff_eq!(len1, len2, epsilon = 1e-4);
    assert_abs_diff_eq!(len2, 5.0, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// P2P angle constraint
// ---------------------------------------------------------------------------

#[test]
fn p2p_angle_constraint() {
    let mut store = ParamStore::new();

    // Two points  
    let (p1, mut params) = mk_point(&mut store, 0.0, 0.0);
    let (p2, pv) = mk_point(&mut store, 8.0, 6.0);
    params.extend(pv);

    let angle = store.push(std::f64::consts::FRAC_PI_4); // 45 degrees

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix p1 at origin
    let z = sys.store_mut().push(0.0);
    sys.add_constraint(Box::new(ConstraintEqual::new(p1.x, z, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(p1.y, z, 1.0, 2, true)));

    // Distance from p1 to p2 = 10
    let dist = sys.store_mut().push(10.0);
    sys.add_constraint(Box::new(ConstraintP2PDistance::new(p1, p2, dist, 3, true)));

    // Angle from p1 to p2 = 45 degrees
    sys.add_constraint(Box::new(ConstraintP2PAngle::new(p1, p2, angle, 0.0, 4, true)));

    sys.init_solution();
    let status = sys.solve_default();
    assert_eq!(status, SolveStatus::Success);

    let s = sys.store();
    let expected = 10.0 / std::f64::consts::SQRT_2;
    assert_abs_diff_eq!(s.get(p2.x), expected, epsilon = 1e-4);
    assert_abs_diff_eq!(s.get(p2.y), expected, epsilon = 1e-4);
}

// ---------------------------------------------------------------------------
// Diagnosis: fully constrained system has DOF=0
// ---------------------------------------------------------------------------

#[test]
fn diagnosis_fully_constrained() {
    let mut store = ParamStore::new();
    let (p, params) = mk_point(&mut store, 1.0, 2.0);
    let target_x = store.push(5.0);
    let target_y = store.push(7.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);
    sys.add_constraint(Box::new(ConstraintEqual::new(p.x, target_x, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(p.y, target_y, 1.0, 2, true)));

    sys.init_solution();
    sys.solve_default();
    let diag = sys.diagnose();
    assert_eq!(diag.dofs, 0);
    assert!(diag.conflicting_tags.is_empty());
    assert!(diag.redundant_tags.is_empty());
}

// ---------------------------------------------------------------------------
// Multiple algorithms solve same problem consistently
// ---------------------------------------------------------------------------

#[test]
fn all_algorithms_produce_same_result() {
    for alg in [Algorithm::DogLeg, Algorithm::LevenbergMarquardt, Algorithm::Bfgs] {
        let mut store = ParamStore::new();
        let (p, params) = mk_point(&mut store, 1.0, 1.0);
        let target_x = store.push(3.0);
        let target_y = store.push(4.0);

        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(params);

        // x = 3, y = 4, dist(origin, p) = 5
        sys.add_constraint(Box::new(ConstraintEqual::new(p.x, target_x, 1.0, 1, true)));
        sys.add_constraint(Box::new(ConstraintEqual::new(p.y, target_y, 1.0, 2, true)));

        sys.init_solution();
        let status = sys.solve_with(false, alg);
        assert_eq!(status, SolveStatus::Success, "Algorithm {:?} failed", alg);

        let s = sys.store();
        assert_abs_diff_eq!(s.get(p.x), 3.0, epsilon = 1e-6);
        assert_abs_diff_eq!(s.get(p.y), 4.0, epsilon = 1e-6);
    }
}
