//! Curve trait and geometry types.
//!
//! Each geometry type stores [`ParamIdx`] references into a [`ParamStore`].

use cadora_core::{ParamIdx, ParamStore, Point};
use cadora_math::DeriVector2;

/// Trait for all parametric curves.
pub trait Curve {
    /// Evaluate the curve at parameter `u`, returning point + derivative.
    /// `du` is the derivative of `u` w.r.t. the differentiation parameter.
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2;

    /// Surface normal at a point on the curve (point-based lookup).
    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2;

    /// All parameter indices this geometry depends on.
    fn params(&self) -> Vec<ParamIdx>;
}

/// A line segment defined by two points.
#[derive(Debug, Clone)]
pub struct Line {
    pub p1: Point,
    pub p2: Point,
}

/// A circle defined by center and radius.
#[derive(Debug, Clone)]
pub struct Circle {
    pub center: Point,
    pub rad: ParamIdx,
}

/// A circular arc: extends circle with start/end angles and endpoint cache.
#[derive(Debug, Clone)]
pub struct Arc {
    pub center: Point,
    pub rad: ParamIdx,
    pub start_angle: ParamIdx,
    pub end_angle: ParamIdx,
    pub start: Point,
    pub end: Point,
}

/// An ellipse defined by center, focus, and semi-minor radius.
#[derive(Debug, Clone)]
pub struct Ellipse {
    pub center: Point,
    pub focus1: Point,
    pub radmin: ParamIdx,
}

/// An arc of ellipse.
#[derive(Debug, Clone)]
pub struct ArcOfEllipse {
    pub ellipse: Ellipse,
    pub start_angle: ParamIdx,
    pub end_angle: ParamIdx,
    pub start: Point,
    pub end: Point,
}

/// A hyperbola defined by center, focus, and semi-minor radius.
#[derive(Debug, Clone)]
pub struct Hyperbola {
    pub center: Point,
    pub focus1: Point,
    pub radmin: ParamIdx,
}

/// An arc of hyperbola.
#[derive(Debug, Clone)]
pub struct ArcOfHyperbola {
    pub hyperbola: Hyperbola,
    pub start_angle: ParamIdx,
    pub end_angle: ParamIdx,
    pub start: Point,
    pub end: Point,
}

/// A parabola defined by vertex and focus.
#[derive(Debug, Clone)]
pub struct Parabola {
    pub vertex: Point,
    pub focus1: Point,
}

/// An arc of parabola.
#[derive(Debug, Clone)]
pub struct ArcOfParabola {
    pub parabola: Parabola,
    pub start_angle: ParamIdx,
    pub end_angle: ParamIdx,
    pub start: Point,
    pub end: Point,
}

/// A NURBS B-spline curve.
#[derive(Debug, Clone)]
pub struct BSpline {
    pub start: Point,
    pub end: Point,
    pub poles: Vec<Point>,
    pub weights: Vec<ParamIdx>,
    pub knots: Vec<ParamIdx>,
    pub mult: Vec<usize>,
    pub degree: usize,
    pub periodic: bool,
    // Flattened knot vector (computed, not stored as params)
    pub flat_knots: Vec<f64>,
}

// Implementations will be added in Phase 1 (Geometry) of the porting pipeline.
// Each geometry type's Curve trait impl will be ported method-by-method from Geo.cpp.
