//! End-to-end integration tests for the CADORA solver — realistic 2D sketch scenarios.

use approx::assert_abs_diff_eq;
use cadora_constraints::*;
use cadora_core::{Algorithm, ParamIdx, ParamStore, Point, SolveStatus, SolverConfig};
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
