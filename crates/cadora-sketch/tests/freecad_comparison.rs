//! FreeCAD comparison tests for the Sketch Manager.
//!
//! These tests replicate common FreeCAD sketch scenarios and verify
//! that CADORA produces the same results.

use approx::assert_abs_diff_eq;
use cadora_sketch::constraint::SketchConstraint;
use cadora_sketch::geometry::{GeoDef, GeoType};
use cadora_sketch::sketch::{Sketch, SketchSolveStatus};
use cadora_sketch::types::*;

/// Helper: distance between two points.
fn dist(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt()
}

// ============================================================================
// FreeCAD Scenario 1: Fully constrained horizontal line from origin
//
// In FreeCAD:
// 1. Draw a line
// 2. Constrain Horizontal
// 3. Constrain start to origin (Coincident with root point)
// 4. Distance constraint = 10
//
// Expected: line from (0,0) to (10,0), 0 DOF
// ============================================================================
#[test]
fn freecad_horizontal_line_from_origin() {
    let mut sketch = Sketch::new();
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 1.0, y1: 2.0, x2: 8.0, y2: 3.0,
    }));

    // Horizontal
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal,
        vec![GeoElementId::edge(0)],
    ));
    // Start point at origin (coincident with H-axis start = origin)
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Coincident,
        vec![GeoElementId::start(0), GeoElementId::start(GEO_ID_H_AXIS)],
    ));
    // Length = 10
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance,
        vec![GeoElementId::edge(0)],
        10.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    let g = sketch.get_geometry(0).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g.geo {
        assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(*x2, 10.0, epsilon = 1e-3);
        assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-6);
    } else {
        panic!("Expected line geometry");
    }

    assert_eq!(sketch.dofs(), Some(0));
}

// ============================================================================
// FreeCAD Scenario 2: Right triangle
//
// In FreeCAD:
// 1. Draw 3 lines forming a triangle
// 2. Coincident corners
// 3. Horizontal bottom, Vertical left side
// 4. Perpendicular bottom-left
// 5. Fix bottom-left at origin
// 6. Bottom = 30mm, Left = 40mm
//
// Expected: (0,0)→(30,0)→(0,40)→(0,0), hypotenuse = 50
// ============================================================================
#[test]
fn freecad_right_triangle() {
    let mut sketch = Sketch::new();

    // Three lines (approximate positions)
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 0.0, x2: 30.0, y2: 0.0,
    })); // bottom
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 30.0, y1: 0.0, x2: 0.0, y2: 40.0,
    })); // hypotenuse
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 40.0, x2: 0.0, y2: 0.0,
    })); // left side

    // Coincident corners
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Coincident,
        vec![GeoElementId::end(0), GeoElementId::start(1)],
    ));
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Coincident,
        vec![GeoElementId::end(1), GeoElementId::start(2)],
    ));
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Coincident,
        vec![GeoElementId::end(2), GeoElementId::start(0)],
    ));

    // Horizontal bottom, Vertical left
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal,
        vec![GeoElementId::edge(0)],
    ));
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Vertical,
        vec![GeoElementId::edge(2)],
    ));

    // Fix origin
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

    // Dimensions
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance,
        vec![GeoElementId::edge(0)],
        30.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance,
        vec![GeoElementId::edge(2)],
        40.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    // Verify triangle vertices
    let g0 = sketch.get_geometry(0).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g0.geo {
        assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*x2, 30.0, epsilon = 1e-3);
        assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-4);
    }

    let g1 = sketch.get_geometry(1).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g1.geo {
        assert_abs_diff_eq!(*x1, 30.0, epsilon = 1e-3);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*x2, 0.0, epsilon = 1e-3);
        assert_abs_diff_eq!(*y2, 40.0, epsilon = 1e-3);
    }

    // Verify hypotenuse length = 50 (3-4-5 triangle × 10)
    if let GeoType::Line { x1, y1, x2, y2 } = &g1.geo {
        let hyp = dist(*x1, *y1, *x2, *y2);
        assert_abs_diff_eq!(hyp, 50.0, epsilon = 1e-2);
    }
}

// ============================================================================
// FreeCAD Scenario 3: Concentric circles with radius constraints
//
// In FreeCAD:
// 1. Draw two circles
// 2. Coincident centers
// 3. Fix center at (5, 5)
// 4. Inner radius = 3, Outer radius = 7
//
// Expected: both centered at (5,5), r1=3, r2=7
// ============================================================================
#[test]
fn freecad_concentric_circles() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Circle {
        cx: 4.0, cy: 6.0, radius: 5.0,
    }));
    sketch.add_geometry(GeoDef::new(GeoType::Circle {
        cx: 5.5, cy: 4.5, radius: 10.0,
    }));

    // Coincident centers
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Coincident,
        vec![GeoElementId::mid(0), GeoElementId::mid(1)],
    ));

    // Fix center
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX,
        vec![GeoElementId::mid(0)],
        5.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY,
        vec![GeoElementId::mid(0)],
        5.0,
    ));

    // Radii
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Radius,
        vec![GeoElementId::edge(0)],
        3.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Radius,
        vec![GeoElementId::edge(1)],
        7.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    let g0 = sketch.get_geometry(0).unwrap();
    if let GeoType::Circle { cx, cy, radius } = &g0.geo {
        assert_abs_diff_eq!(*cx, 5.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*cy, 5.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*radius, 3.0, epsilon = 1e-4);
    }

    let g1 = sketch.get_geometry(1).unwrap();
    if let GeoType::Circle { cx, cy, radius } = &g1.geo {
        assert_abs_diff_eq!(*cx, 5.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*cy, 5.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*radius, 7.0, epsilon = 1e-4);
    }

    assert_eq!(sketch.dofs(), Some(0));
}

// ============================================================================
// FreeCAD Scenario 4: Two equal-length parallel lines
//
// In FreeCAD:
// 1. Draw two lines
// 2. Both horizontal
// 3. Equal length
// 4. Fix first line fully
// 5. Fix second line start y-coordinate
//
// Expected: second line same length, horizontal, at specified y
// ============================================================================
#[test]
fn freecad_equal_parallel_lines() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 0.0, x2: 15.0, y2: 0.0,
    }));
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 2.0, y1: 8.0, x2: 12.0, y2: 9.0,
    }));

    // Both horizontal
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal,
        vec![GeoElementId::edge(0)],
    ));
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal,
        vec![GeoElementId::edge(1)],
    ));

    // Equal length
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Equal,
        vec![GeoElementId::edge(0), GeoElementId::edge(1)],
    ));

    // Fix line 0 fully
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance, vec![GeoElementId::edge(0)], 15.0,
    ));

    // Fix line 1 start position
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::start(1)], 2.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY, vec![GeoElementId::start(1)], 8.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    // Line 0: (0,0) → (15,0)
    let g0 = sketch.get_geometry(0).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g0.geo {
        assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(dist(*x1, *y1, *x2, *y2), 15.0, epsilon = 1e-3);
    }

    // Line 1: horizontal, starts at (2,8), length = 15
    let g1 = sketch.get_geometry(1).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g1.geo {
        assert_abs_diff_eq!(*x1, 2.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y1, 8.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y1, *y2, epsilon = 1e-4); // horizontal
        assert_abs_diff_eq!(dist(*x1, *y1, *x2, *y2), 15.0, epsilon = 1e-2); // equal length
    }
}

// ============================================================================
// FreeCAD Scenario 5: L-shape with perpendicular lines
//
// In FreeCAD:
// 1. Draw two connected lines forming an L
// 2. Perpendicular constraint
// 3. Horizontal on first line
// 4. Fix corner at origin
// 5. First line length = 20, second = 15
//
// Expected: (−20,0)→(0,0)→(0,15)
// ============================================================================
#[test]
fn freecad_l_shape() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: -18.0, y1: 1.0, x2: 0.0, y2: 0.0,
    }));
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 0.0, x2: 2.0, y2: 13.0,
    }));

    // Coincident at junction
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Coincident,
        vec![GeoElementId::end(0), GeoElementId::start(1)],
    ));

    // Perpendicular
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Perpendicular,
        vec![GeoElementId::edge(0), GeoElementId::edge(1)],
    ));

    // Horizontal first line
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal,
        vec![GeoElementId::edge(0)],
    ));

    // Fix junction at origin
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::end(0)], 0.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY, vec![GeoElementId::end(0)], 0.0,
    ));

    // Lengths
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance, vec![GeoElementId::edge(0)], 20.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance, vec![GeoElementId::edge(1)], 15.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    // Line 0: horizontal, ends at origin, length 20
    let g0 = sketch.get_geometry(0).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g0.geo {
        assert_abs_diff_eq!(*x2, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-4); // horizontal
        assert_abs_diff_eq!(dist(*x1, *y1, *x2, *y2), 20.0, epsilon = 1e-3);
    }

    // Line 1: starts at origin, perpendicular to horizontal → vertical, length 15
    let g1 = sketch.get_geometry(1).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g1.geo {
        assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*x1, *x2, epsilon = 1e-3); // vertical (perp to horiz)
        assert_abs_diff_eq!(dist(*x1, *y1, *x2, *y2), 15.0, epsilon = 1e-3);
    }
}

// ============================================================================
// FreeCAD Scenario 6: Circle inscribed on a line (point-on-object)
//
// In FreeCAD:
// 1. Draw a horizontal line y=0
// 2. Draw a circle
// 3. Circle center on the line (PointOnObject)
// 4. Fix line, set radius
//
// Expected: circle center on the line (y_center = 0)
// ============================================================================
#[test]
fn freecad_circle_center_on_line() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: -10.0, y1: 0.0, x2: 10.0, y2: 0.0,
    }));
    sketch.add_geometry(GeoDef::new(GeoType::Circle {
        cx: 5.0, cy: 3.0, radius: 2.0,
    }));

    // Fix line
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::start(0)], -10.0,
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

    // Circle center on line
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::PointOnObject,
        vec![GeoElementId::mid(1), GeoElementId::edge(0)],
    ));

    // Fix circle x and radius
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::mid(1)], 5.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Radius, vec![GeoElementId::edge(1)], 4.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    let g1 = sketch.get_geometry(1).unwrap();
    if let GeoType::Circle { cx, cy, radius } = &g1.geo {
        assert_abs_diff_eq!(*cx, 5.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*cy, 0.0, epsilon = 1e-4); // ON the line y=0
        assert_abs_diff_eq!(*radius, 4.0, epsilon = 1e-4);
    }

    assert_eq!(sketch.dofs(), Some(0));
}

// ============================================================================
// FreeCAD Scenario 7: Square with equal sides
//
// In FreeCAD:
// 1. Draw 4 lines
// 2. Coincident corners, Horizontal top/bottom, Vertical left/right
// 3. Equal length (all 4 sides)
// 4. Fix bottom-left corner, one side length = 25
//
// Expected: 25×25 square starting at origin
// ============================================================================
#[test]
fn freecad_square_equal_sides() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 0.0, x2: 20.0, y2: 0.0 }));
    sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 20.0, y1: 0.0, x2: 20.0, y2: 20.0 }));
    sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 20.0, y1: 20.0, x2: 0.0, y2: 20.0 }));
    sketch.add_geometry(GeoDef::new(GeoType::Line { x1: 0.0, y1: 20.0, x2: 0.0, y2: 0.0 }));

    // Corners
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(0), GeoElementId::start(1)]));
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(1), GeoElementId::start(2)]));
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(2), GeoElementId::start(3)]));
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Coincident, vec![GeoElementId::end(3), GeoElementId::start(0)]));

    // H/V
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Horizontal, vec![GeoElementId::edge(0)]));
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Horizontal, vec![GeoElementId::edge(2)]));
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Vertical, vec![GeoElementId::edge(1)]));
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Vertical, vec![GeoElementId::edge(3)]));

    // Equal length between adjacent sides (makes it a square)
    // Note: opposite sides of a closed H/V rectangle are already equal,
    // so only adjacent-side equality is needed (matching FreeCAD behavior).
    sketch.add_constraint(SketchConstraint::geometric(ConstraintType::Equal, vec![GeoElementId::edge(0), GeoElementId::edge(1)]));

    // Fix origin
    sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0));
    sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0));

    // Side length = 25
    sketch.add_constraint(SketchConstraint::dimensional(ConstraintType::Distance, vec![GeoElementId::edge(0)], 25.0));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    // Verify all 4 corners
    let g0 = sketch.get_geometry(0).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g0.geo {
        assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-3);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-3);
        assert_abs_diff_eq!(*x2, 25.0, epsilon = 1e-2);
        assert_abs_diff_eq!(*y2, 0.0, epsilon = 1e-3);
    }

    // Verify all sides are equal
    let lengths: Vec<f64> = (0..4)
        .map(|i| {
            let g = sketch.get_geometry(i).unwrap();
            if let GeoType::Line { x1, y1, x2, y2 } = &g.geo {
                dist(*x1, *y1, *x2, *y2)
            } else {
                0.0
            }
        })
        .collect();

    for l in &lengths {
        assert_abs_diff_eq!(*l, 25.0, epsilon = 1e-2);
    }

    assert_eq!(sketch.dofs(), Some(0));
}

// ============================================================================
// FreeCAD Scenario 8: Angle between two lines from same point
//
// In FreeCAD:
// 1. Draw two lines sharing a start point
// 2. Coincident at shared point
// 3. Fix shared point at origin
// 4. First line horizontal, length 10
// 5. Angle between lines = 60°
// 6. Second line length = 10
//
// Expected: Two arms from origin, 60° apart
// ============================================================================
#[test]
fn freecad_angle_from_origin() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
    }));
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 0.0, x2: 5.0, y2: 8.0,
    }));

    // Coincident starts
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Coincident,
        vec![GeoElementId::start(0), GeoElementId::start(1)],
    ));

    // Fix origin
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0,
    ));

    // First line horizontal, length 10
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal, vec![GeoElementId::edge(0)],
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance, vec![GeoElementId::edge(0)], 10.0,
    ));

    // Second line length 10
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance, vec![GeoElementId::edge(1)], 10.0,
    ));

    // Angle between lines = 60° (π/3)
    let angle = std::f64::consts::PI / 3.0;
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Angle,
        vec![GeoElementId::edge(0), GeoElementId::edge(1)],
        angle,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    // Second line end should be at (10*cos(60°), 10*sin(60°)) = (5, 8.66)
    let g1 = sketch.get_geometry(1).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g1.geo {
        assert_abs_diff_eq!(*x1, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y1, 0.0, epsilon = 1e-4);
        let end_len = dist(*x1, *y1, *x2, *y2);
        assert_abs_diff_eq!(end_len, 10.0, epsilon = 1e-3);
        // Angle from horizontal
        let actual_angle = y2.atan2(*x2);
        assert_abs_diff_eq!(actual_angle, angle, epsilon = 1e-2);
    }
}

// ============================================================================
// FreeCAD Scenario 9: Diameter constraint on circle
//
// Expected: circle with diameter = 10 → radius = 5
// ============================================================================
#[test]
fn freecad_diameter_constraint() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Circle {
        cx: 0.0, cy: 0.0, radius: 8.0,
    }));

    // Fix center
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::mid(0)], 0.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY, vec![GeoElementId::mid(0)], 0.0,
    ));

    // Diameter = 10 → radius = 5
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Diameter,
        vec![GeoElementId::edge(0)],
        10.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    let g = sketch.get_geometry(0).unwrap();
    if let GeoType::Circle { cx, cy, radius } = &g.geo {
        assert_abs_diff_eq!(*cx, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*cy, 0.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*radius, 5.0, epsilon = 1e-4);
    }
}

// ============================================================================
// FreeCAD Scenario 10: Re-solve after changing datum
//
// In FreeCAD:
// 1. Create constrained line, solve
// 2. Change distance, re-solve
//
// Expected: geometry updates to new constraint value
// ============================================================================
#[test]
fn freecad_resolve_after_datum_change() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
    }));

    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal, vec![GeoElementId::edge(0)],
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::start(0)], 0.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY, vec![GeoElementId::start(0)], 0.0,
    ));
    let dist_idx = sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::Distance, vec![GeoElementId::edge(0)], 10.0,
    ));

    // First solve
    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);
    if let GeoType::Line { x2, .. } = &sketch.get_geometry(0).unwrap().geo {
        assert_abs_diff_eq!(*x2, 10.0, epsilon = 1e-3);
    }

    // Change distance to 25
    sketch.set_datum(dist_idx, 25.0);
    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);
    if let GeoType::Line { x2, .. } = &sketch.get_geometry(0).unwrap().geo {
        assert_abs_diff_eq!(*x2, 25.0, epsilon = 1e-3);
    }

    // Change distance to 5
    sketch.set_datum(dist_idx, 5.0);
    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);
    if let GeoType::Line { x2, .. } = &sketch.get_geometry(0).unwrap().geo {
        assert_abs_diff_eq!(*x2, 5.0, epsilon = 1e-3);
    }
}

// ============================================================================
// FreeCAD Scenario 11: Block constraint
//
// In FreeCAD:
// 1. Draw a line
// 2. Block it (all params fixed)
// 3. Solve
//
// Expected: line unchanged from initial position
// ============================================================================
#[test]
fn freecad_block_constraint() {
    let mut sketch = Sketch::new();

    let mut geo = GeoDef::new(GeoType::Line {
        x1: 3.0, y1: 7.0, x2: 15.0, y2: 2.0,
    });
    geo.mode.blocked = true;
    sketch.add_geometry(geo);

    // Another unconstrained line
    sketch.add_geometry(GeoDef::new(GeoType::Line {
        x1: 0.0, y1: 0.0, x2: 10.0, y2: 0.0,
    }));

    // Horizontal on second line
    sketch.add_constraint(SketchConstraint::geometric(
        ConstraintType::Horizontal, vec![GeoElementId::edge(1)],
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    // Blocked line should be unchanged
    let g0 = sketch.get_geometry(0).unwrap();
    if let GeoType::Line { x1, y1, x2, y2 } = &g0.geo {
        assert_abs_diff_eq!(*x1, 3.0, epsilon = 1e-10);
        assert_abs_diff_eq!(*y1, 7.0, epsilon = 1e-10);
        assert_abs_diff_eq!(*x2, 15.0, epsilon = 1e-10);
        assert_abs_diff_eq!(*y2, 2.0, epsilon = 1e-10);
    }
}

// ============================================================================
// FreeCAD Scenario 12: DistanceX and DistanceY between two points
//
// In FreeCAD:
// 1. Draw two points
// 2. DistanceX between them = 7
// 3. DistanceY between them = 3
// 4. Fix first point at (2, 1)
//
// Expected: second point at (9, 4) [2+7, 1+3]
// ============================================================================
#[test]
fn freecad_distance_xy_between_points() {
    let mut sketch = Sketch::new();

    sketch.add_geometry(GeoDef::new(GeoType::Point { x: 2.0, y: 1.0 }));
    sketch.add_geometry(GeoDef::new(GeoType::Point { x: 8.0, y: 5.0 }));

    // Fix first point
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX, vec![GeoElementId::mid(0)], 2.0,
    ));
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY, vec![GeoElementId::mid(0)], 1.0,
    ));

    // DistanceX = 7 between points (p2.x - p1.x = 7)
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceX,
        vec![GeoElementId::mid(0), GeoElementId::mid(1)],
        7.0,
    ));
    // DistanceY = 3 between points (p2.y - p1.y = 3)
    sketch.add_constraint(SketchConstraint::dimensional(
        ConstraintType::DistanceY,
        vec![GeoElementId::mid(0), GeoElementId::mid(1)],
        3.0,
    ));

    let status = sketch.solve();
    assert_eq!(status, SketchSolveStatus::Ok);

    let g0 = sketch.get_geometry(0).unwrap();
    if let GeoType::Point { x, y } = &g0.geo {
        assert_abs_diff_eq!(*x, 2.0, epsilon = 1e-4);
        assert_abs_diff_eq!(*y, 1.0, epsilon = 1e-4);
    }

    let g1 = sketch.get_geometry(1).unwrap();
    if let GeoType::Point { x, y } = &g1.geo {
        assert_abs_diff_eq!(*x, 9.0, epsilon = 1e-3);
        assert_abs_diff_eq!(*y, 4.0, epsilon = 1e-3);
    }

    assert_eq!(sketch.dofs(), Some(0));
}
