//! # cadora-constraints
//!
//! All constraint types with error + gradient computation.
//! Port of planegcs `Constraints.h` / `Constraints.cpp`.

use cadora_core::{ParamIdx, ParamStore, Tag};
use cadora_geo::{Ellipse, Hyperbola, Arc, Line, Curve};
use cadora_math::DeriVector2;
use cadora_core::Point;

/// The core constraint trait. Every constraint type implements this.
pub trait Constraint {
    /// Compute the constraint error (residual). Zero when satisfied.
    fn error(&self, store: &ParamStore) -> f64;

    /// Compute partial derivative of error w.r.t. one parameter.
    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64;

    /// All parameter indices this constraint depends on.
    fn params(&self) -> &[ParamIdx];

    /// Constraint tag for identification/grouping.
    fn tag(&self) -> Tag;

    /// Whether this is a driving (active) constraint.
    fn is_driving(&self) -> bool;

    /// Rescale error by geometry size (normalizes Jacobian).
    fn rescale(&mut self, _store: &ParamStore) {}

    /// For driven constraints: set the value parameter to match current geometry.
    fn evaluate(&self, _store: &mut ParamStore) {}
}

// ---------------------------------------------------------------------------
// Equal: param1 == ratio * param2
// ---------------------------------------------------------------------------

pub struct ConstraintEqual {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    pub ratio: f64,
}

impl ConstraintEqual {
    pub fn new(p1: ParamIdx, p2: ParamIdx, ratio: f64, tag: Tag, driving: bool) -> Self {
        Self {
            pvec: vec![p1, p2],
            scale: 1.0,
            tag,
            driving,
            ratio,
        }
    }

    fn param1(&self) -> ParamIdx { self.pvec[0] }
    fn param2(&self) -> ParamIdx { self.pvec[1] }
}

impl Constraint for ConstraintEqual {
    fn error(&self, store: &ParamStore) -> f64 {
        self.scale * (store.get(self.param1()) - self.ratio * store.get(self.param2()))
    }

    fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        if param == self.param1() { deriv += 1.0; }
        if param == self.param2() { deriv -= 1.0; }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let v1 = store.get(self.param1());
        store.set(self.param2(), v1 / self.ratio);
    }
}

// ---------------------------------------------------------------------------
// Difference: param2 - param1 == difference
// ---------------------------------------------------------------------------

pub struct ConstraintDifference {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintDifference {
    pub fn new(p1: ParamIdx, p2: ParamIdx, diff: ParamIdx, tag: Tag, driving: bool) -> Self {
        Self {
            pvec: vec![p1, p2, diff],
            scale: 1.0,
            tag,
            driving,
        }
    }

    fn param1(&self) -> ParamIdx { self.pvec[0] }
    fn param2(&self) -> ParamIdx { self.pvec[1] }
    fn difference(&self) -> ParamIdx { self.pvec[2] }

    fn value(&self, store: &ParamStore) -> f64 {
        store.get(self.param2()) - store.get(self.param1())
    }
}

impl Constraint for ConstraintDifference {
    fn error(&self, store: &ParamStore) -> f64 {
        self.scale * (self.value(store) - store.get(self.difference()))
    }

    fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        if param == self.param1() { deriv -= 1.0; }
        if param == self.param2() { deriv += 1.0; }
        if param == self.difference() { deriv -= 1.0; }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let v = self.value(store);
        store.set(self.difference(), self.scale * v);
    }
}

// ---------------------------------------------------------------------------
// P2PDistance: |p1 - p2| == distance
// ---------------------------------------------------------------------------

pub struct ConstraintP2PDistance {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintP2PDistance {
    pub fn new(p1: Point, p2: Point, dist: ParamIdx, tag: Tag, driving: bool) -> Self {
        Self {
            pvec: vec![p1.x, p1.y, p2.x, p2.y, dist],
            scale: 1.0,
            tag,
            driving,
        }
    }

    fn p1x(&self) -> ParamIdx { self.pvec[0] }
    fn p1y(&self) -> ParamIdx { self.pvec[1] }
    fn p2x(&self) -> ParamIdx { self.pvec[2] }
    fn p2y(&self) -> ParamIdx { self.pvec[3] }
    fn distance(&self) -> ParamIdx { self.pvec[4] }

    fn value(&self, store: &ParamStore) -> f64 {
        let dx = store.get(self.p1x()) - store.get(self.p2x());
        let dy = store.get(self.p1y()) - store.get(self.p2y());
        (dx * dx + dy * dy).sqrt()
    }
}

impl Constraint for ConstraintP2PDistance {
    fn error(&self, store: &ParamStore) -> f64 {
        self.scale * (self.value(store) - store.get(self.distance()))
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        if param == self.p1x() || param == self.p1y()
            || param == self.p2x() || param == self.p2y()
        {
            let dx = store.get(self.p1x()) - store.get(self.p2x());
            let dy = store.get(self.p1y()) - store.get(self.p2y());
            let d = (dx * dx + dy * dy).sqrt();
            if param == self.p1x() { deriv += dx / d; }
            if param == self.p1y() { deriv += dy / d; }
            if param == self.p2x() { deriv -= dx / d; }
            if param == self.p2y() { deriv -= dy / d; }
        }
        if param == self.distance() { deriv -= 1.0; }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let v = self.value(store);
        store.set(self.distance(), v);
    }
}

// ---------------------------------------------------------------------------
// P2PAngle: atan2 of direction (p1→p2) w.r.t. angle+da == 0
// ---------------------------------------------------------------------------

pub struct ConstraintP2PAngle {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    pub da: f64,
}

impl ConstraintP2PAngle {
    pub fn new(
        p1: Point, p2: Point, angle: ParamIdx,
        da: f64, tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![p1.x, p1.y, p2.x, p2.y, angle],
            scale: 1.0,
            tag,
            driving,
            da,
        }
    }

    fn p1x(&self) -> ParamIdx { self.pvec[0] }
    fn p1y(&self) -> ParamIdx { self.pvec[1] }
    fn p2x(&self) -> ParamIdx { self.pvec[2] }
    fn p2y(&self) -> ParamIdx { self.pvec[3] }
    fn angle(&self) -> ParamIdx { self.pvec[4] }
}

impl Constraint for ConstraintP2PAngle {
    fn error(&self, store: &ParamStore) -> f64 {
        let dx = store.get(self.p2x()) - store.get(self.p1x());
        let dy = store.get(self.p2y()) - store.get(self.p1y());
        let a = store.get(self.angle()) + self.da;
        let ca = a.cos();
        let sa = a.sin();
        let x = dx * ca + dy * sa;
        let y = -dx * sa + dy * ca;
        self.scale * y.atan2(x)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        if param == self.p1x() || param == self.p1y()
            || param == self.p2x() || param == self.p2y()
        {
            let dx = store.get(self.p2x()) - store.get(self.p1x());
            let dy = store.get(self.p2y()) - store.get(self.p1y());
            let a = store.get(self.angle()) + self.da;
            let ca = a.cos();
            let sa = a.sin();
            let x = dx * ca + dy * sa;
            let y = -dx * sa + dy * ca;
            let r2 = dx * dx + dy * dy;
            let ddx = -y / r2;
            let ddy = x / r2;
            if param == self.p1x() { deriv += -ca * ddx + sa * ddy; }
            if param == self.p1y() { deriv += -sa * ddx - ca * ddy; }
            if param == self.p2x() { deriv += ca * ddx - sa * ddy; }
            if param == self.p2y() { deriv += sa * ddx + ca * ddy; }
        }
        if param == self.angle() { deriv -= 1.0; }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let dx = store.get(self.p2x()) - store.get(self.p1x());
        let dy = store.get(self.p2y()) - store.get(self.p1y());
        store.set(self.angle(), dy.atan2(dx) - self.da);
    }
}

// ---------------------------------------------------------------------------
// P2LDistance: distance from point to line
// ---------------------------------------------------------------------------

pub struct ConstraintP2LDistance {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintP2LDistance {
    pub fn new(
        p: Point, l_p1: Point, l_p2: Point, dist: ParamIdx,
        tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![p.x, p.y, l_p1.x, l_p1.y, l_p2.x, l_p2.y, dist],
            scale: 1.0,
            tag,
            driving,
        }
    }

    fn p0x(&self) -> ParamIdx { self.pvec[0] }
    fn p0y(&self) -> ParamIdx { self.pvec[1] }
    fn p1x(&self) -> ParamIdx { self.pvec[2] }
    fn p1y(&self) -> ParamIdx { self.pvec[3] }
    fn p2x(&self) -> ParamIdx { self.pvec[4] }
    fn p2y(&self) -> ParamIdx { self.pvec[5] }
    fn distance(&self) -> ParamIdx { self.pvec[6] }

    fn value(&self, store: &ParamStore) -> f64 {
        let (x0, y0) = (store.get(self.p0x()), store.get(self.p0y()));
        let (x1, y1) = (store.get(self.p1x()), store.get(self.p1y()));
        let (x2, y2) = (store.get(self.p2x()), store.get(self.p2y()));
        let dx = x2 - x1;
        let dy = y2 - y1;
        let d = (dx * dx + dy * dy).sqrt();
        let area = -x0 * dy + y0 * dx + x1 * y2 - x2 * y1;
        (area / d).abs()
    }
}

impl Constraint for ConstraintP2LDistance {
    fn error(&self, store: &ParamStore) -> f64 {
        self.scale * (self.value(store) - store.get(self.distance()))
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        if param == self.p0x() || param == self.p0y()
            || param == self.p1x() || param == self.p1y()
            || param == self.p2x() || param == self.p2y()
        {
            let (x0, y0) = (store.get(self.p0x()), store.get(self.p0y()));
            let (x1, y1) = (store.get(self.p1x()), store.get(self.p1y()));
            let (x2, y2) = (store.get(self.p2x()), store.get(self.p2y()));
            let dx = x2 - x1;
            let dy = y2 - y1;
            let d2 = dx * dx + dy * dy;
            let d = d2.sqrt();
            let area = -x0 * dy + y0 * dx + x1 * y2 - x2 * y1;
            if param == self.p0x() { deriv += (y1 - y2) / d; }
            if param == self.p0y() { deriv += (x2 - x1) / d; }
            if param == self.p1x() { deriv += ((y2 - y0) * d + (dx / d) * area) / d2; }
            if param == self.p1y() { deriv += ((x0 - x2) * d + (dy / d) * area) / d2; }
            if param == self.p2x() { deriv += ((y0 - y1) * d - (dx / d) * area) / d2; }
            if param == self.p2y() { deriv += ((x1 - x0) * d - (dy / d) * area) / d2; }
            if area < 0.0 { deriv *= -1.0; }
        }
        if param == self.distance() { deriv -= 1.0; }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let v = self.value(store);
        store.set(self.distance(), v);
    }
}

// ---------------------------------------------------------------------------
// PointOnLine: signed distance from point to line == 0
// ---------------------------------------------------------------------------

pub struct ConstraintPointOnLine {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintPointOnLine {
    pub fn new(p: Point, l_p1: Point, l_p2: Point, tag: Tag, driving: bool) -> Self {
        Self {
            pvec: vec![p.x, p.y, l_p1.x, l_p1.y, l_p2.x, l_p2.y],
            scale: 1.0,
            tag,
            driving,
        }
    }

    fn p0x(&self) -> ParamIdx { self.pvec[0] }
    fn p0y(&self) -> ParamIdx { self.pvec[1] }
    fn p1x(&self) -> ParamIdx { self.pvec[2] }
    fn p1y(&self) -> ParamIdx { self.pvec[3] }
    fn p2x(&self) -> ParamIdx { self.pvec[4] }
    fn p2y(&self) -> ParamIdx { self.pvec[5] }
}

impl Constraint for ConstraintPointOnLine {
    fn error(&self, store: &ParamStore) -> f64 {
        let (x0, y0) = (store.get(self.p0x()), store.get(self.p0y()));
        let (x1, y1) = (store.get(self.p1x()), store.get(self.p1y()));
        let (x2, y2) = (store.get(self.p2x()), store.get(self.p2y()));
        let dx = x2 - x1;
        let dy = y2 - y1;
        let d = (dx * dx + dy * dy).sqrt();
        let area = -x0 * dy + y0 * dx + x1 * y2 - x2 * y1;
        self.scale * area / d
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        if param == self.p0x() || param == self.p0y()
            || param == self.p1x() || param == self.p1y()
            || param == self.p2x() || param == self.p2y()
        {
            let (x0, y0) = (store.get(self.p0x()), store.get(self.p0y()));
            let (x1, y1) = (store.get(self.p1x()), store.get(self.p1y()));
            let (x2, y2) = (store.get(self.p2x()), store.get(self.p2y()));
            let dx = x2 - x1;
            let dy = y2 - y1;
            let d2 = dx * dx + dy * dy;
            let d = d2.sqrt();
            let area = -x0 * dy + y0 * dx + x1 * y2 - x2 * y1;
            if param == self.p0x() { deriv += (y1 - y2) / d; }
            if param == self.p0y() { deriv += (x2 - x1) / d; }
            if param == self.p1x() { deriv += ((y2 - y0) * d + (dx / d) * area) / d2; }
            if param == self.p1y() { deriv += ((x0 - x2) * d + (dy / d) * area) / d2; }
            if param == self.p2x() { deriv += ((y0 - y1) * d - (dx / d) * area) / d2; }
            if param == self.p2y() { deriv += ((x1 - x0) * d - (dy / d) * area) / d2; }
        }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// PointOnPerpBisector: proj of (p-p1) and (p-p2) onto line direction sum to 0
// ---------------------------------------------------------------------------

pub struct ConstraintPointOnPerpBisector {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintPointOnPerpBisector {
    pub fn new(p: Point, l_p1: Point, l_p2: Point, tag: Tag, driving: bool) -> Self {
        Self {
            pvec: vec![p.x, p.y, l_p1.x, l_p1.y, l_p2.x, l_p2.y],
            scale: 1.0,
            tag,
            driving,
        }
    }
}

impl Constraint for ConstraintPointOnPerpBisector {
    fn error(&self, store: &ParamStore) -> f64 {
        let p0 = Point::new(self.pvec[0], self.pvec[1]);
        let p1 = Point::new(self.pvec[2], self.pvec[3]);
        let p2 = Point::new(self.pvec[4], self.pvec[5]);
        let p0v = DeriVector2::from_point(store, p0, None);
        let p1v = DeriVector2::from_point(store, p1, None);
        let p2v = DeriVector2::from_point(store, p2, None);
        let d1 = p0v.sub(&p1v);
        let d2 = p0v.sub(&p2v);
        let big_d = p2v.sub(&p1v).normalized();
        let (proj1, _) = d1.dot(&big_d);
        let (proj2, _) = d2.dot(&big_d);
        self.scale * (proj1 + proj2)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let p0 = Point::new(self.pvec[0], self.pvec[1]);
        let p1 = Point::new(self.pvec[2], self.pvec[3]);
        let p2 = Point::new(self.pvec[4], self.pvec[5]);
        let p0v = DeriVector2::from_point(store, p0, Some(param));
        let p1v = DeriVector2::from_point(store, p1, Some(param));
        let p2v = DeriVector2::from_point(store, p2, Some(param));
        let d1 = p0v.sub(&p1v);
        let d2 = p0v.sub(&p2v);
        let big_d = p2v.sub(&p1v).normalized();
        let (_, dproj1) = d1.dot(&big_d);
        let (_, dproj2) = d2.dot(&big_d);
        self.scale * (dproj1 + dproj2)
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// Parallel: cross product of two line directions == 0
// ---------------------------------------------------------------------------

pub struct ConstraintParallel {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintParallel {
    pub fn new(l1_p1: Point, l1_p2: Point, l2_p1: Point, l2_p2: Point, tag: Tag, driving: bool) -> Self {
        let mut c = Self {
            pvec: vec![l1_p1.x, l1_p1.y, l1_p2.x, l1_p2.y, l2_p1.x, l2_p1.y, l2_p2.x, l2_p2.y],
            scale: 1.0,
            tag,
            driving,
        };
        c.scale = 1.0; // will be set by rescale
        c
    }
}

impl Constraint for ConstraintParallel {
    fn error(&self, store: &ParamStore) -> f64 {
        let dx1 = store.get(self.pvec[0]) - store.get(self.pvec[2]);
        let dy1 = store.get(self.pvec[1]) - store.get(self.pvec[3]);
        let dx2 = store.get(self.pvec[4]) - store.get(self.pvec[6]);
        let dy2 = store.get(self.pvec[5]) - store.get(self.pvec[7]);
        self.scale * (dx1 * dy2 - dy1 * dx2)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let dx1 = store.get(self.pvec[0]) - store.get(self.pvec[2]);
        let dy1 = store.get(self.pvec[1]) - store.get(self.pvec[3]);
        let dx2 = store.get(self.pvec[4]) - store.get(self.pvec[6]);
        let dy2 = store.get(self.pvec[5]) - store.get(self.pvec[7]);
        let mut deriv = 0.0;
        if param == self.pvec[0] { deriv += dy2; }   // l1p1x → dy2
        if param == self.pvec[2] { deriv -= dy2; }   // l1p2x → -dy2
        if param == self.pvec[1] { deriv -= dx2; }   // l1p1y → -dx2
        if param == self.pvec[3] { deriv += dx2; }   // l1p2y → dx2
        if param == self.pvec[4] { deriv -= dy1; }   // l2p1x → -dy1
        if param == self.pvec[6] { deriv += dy1; }   // l2p2x → dy1
        if param == self.pvec[5] { deriv += dx1; }   // l2p1y → dx1
        if param == self.pvec[7] { deriv -= dx1; }   // l2p2y → -dx1
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn rescale(&mut self, store: &ParamStore) {
        let dx1 = store.get(self.pvec[0]) - store.get(self.pvec[2]);
        let dy1 = store.get(self.pvec[1]) - store.get(self.pvec[3]);
        let dx2 = store.get(self.pvec[4]) - store.get(self.pvec[6]);
        let dy2 = store.get(self.pvec[5]) - store.get(self.pvec[7]);
        self.scale = 1.0 / ((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2)).sqrt();
    }
}

// ---------------------------------------------------------------------------
// Perpendicular: dot product of two line directions == 0
// ---------------------------------------------------------------------------

pub struct ConstraintPerpendicular {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintPerpendicular {
    pub fn new(l1_p1: Point, l1_p2: Point, l2_p1: Point, l2_p2: Point, tag: Tag, driving: bool) -> Self {
        Self {
            pvec: vec![l1_p1.x, l1_p1.y, l1_p2.x, l1_p2.y, l2_p1.x, l2_p1.y, l2_p2.x, l2_p2.y],
            scale: 1.0,
            tag,
            driving,
        }
    }
}

impl Constraint for ConstraintPerpendicular {
    fn error(&self, store: &ParamStore) -> f64 {
        let dx1 = store.get(self.pvec[0]) - store.get(self.pvec[2]);
        let dy1 = store.get(self.pvec[1]) - store.get(self.pvec[3]);
        let dx2 = store.get(self.pvec[4]) - store.get(self.pvec[6]);
        let dy2 = store.get(self.pvec[5]) - store.get(self.pvec[7]);
        self.scale * (dx1 * dx2 + dy1 * dy2)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let dx1 = store.get(self.pvec[0]) - store.get(self.pvec[2]);
        let dy1 = store.get(self.pvec[1]) - store.get(self.pvec[3]);
        let dx2 = store.get(self.pvec[4]) - store.get(self.pvec[6]);
        let dy2 = store.get(self.pvec[5]) - store.get(self.pvec[7]);
        let mut deriv = 0.0;
        if param == self.pvec[0] { deriv += dx2; }
        if param == self.pvec[2] { deriv -= dx2; }
        if param == self.pvec[1] { deriv += dy2; }
        if param == self.pvec[3] { deriv -= dy2; }
        if param == self.pvec[4] { deriv += dx1; }
        if param == self.pvec[6] { deriv -= dx1; }
        if param == self.pvec[5] { deriv += dy1; }
        if param == self.pvec[7] { deriv -= dy1; }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn rescale(&mut self, store: &ParamStore) {
        let dx1 = store.get(self.pvec[0]) - store.get(self.pvec[2]);
        let dy1 = store.get(self.pvec[1]) - store.get(self.pvec[3]);
        let dx2 = store.get(self.pvec[4]) - store.get(self.pvec[6]);
        let dy2 = store.get(self.pvec[5]) - store.get(self.pvec[7]);
        self.scale = 1.0 / ((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2)).sqrt();
    }
}

// ---------------------------------------------------------------------------
// L2LAngle: angle between two lines
// ---------------------------------------------------------------------------

pub struct ConstraintL2LAngle {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintL2LAngle {
    pub fn new(
        l1_p1: Point, l1_p2: Point, l2_p1: Point, l2_p2: Point,
        angle: ParamIdx, tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![
                l1_p1.x, l1_p1.y, l1_p2.x, l1_p2.y,
                l2_p1.x, l2_p1.y, l2_p2.x, l2_p2.y,
                angle,
            ],
            scale: 1.0,
            tag,
            driving,
        }
    }

    fn angle(&self) -> ParamIdx { self.pvec[8] }
}

impl Constraint for ConstraintL2LAngle {
    fn error(&self, store: &ParamStore) -> f64 {
        let dx1 = store.get(self.pvec[2]) - store.get(self.pvec[0]);
        let dy1 = store.get(self.pvec[3]) - store.get(self.pvec[1]);
        let dx2 = store.get(self.pvec[6]) - store.get(self.pvec[4]);
        let dy2 = store.get(self.pvec[7]) - store.get(self.pvec[5]);
        let a = dy1.atan2(dx1) + store.get(self.angle());
        let ca = a.cos();
        let sa = a.sin();
        let x2 = dx2 * ca + dy2 * sa;
        let y2 = -dx2 * sa + dy2 * ca;
        self.scale * y2.atan2(x2)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        // Line 1 endpoint derivatives affect atan2(dy1,dx1) term
        if param == self.pvec[0] || param == self.pvec[1]
            || param == self.pvec[2] || param == self.pvec[3]
        {
            let dx1 = store.get(self.pvec[2]) - store.get(self.pvec[0]);
            let dy1 = store.get(self.pvec[3]) - store.get(self.pvec[1]);
            let r2 = dx1 * dx1 + dy1 * dy1;
            if param == self.pvec[0] { deriv += -dy1 / r2; }  // -l1p1x
            if param == self.pvec[1] { deriv += dx1 / r2; }   // -l1p1y
            if param == self.pvec[2] { deriv += dy1 / r2; }   // l1p2x
            if param == self.pvec[3] { deriv += -dx1 / r2; }  // l1p2y
        }
        // Line 2 endpoint derivatives — same structure as P2PAngle
        if param == self.pvec[4] || param == self.pvec[5]
            || param == self.pvec[6] || param == self.pvec[7]
        {
            let dx1 = store.get(self.pvec[2]) - store.get(self.pvec[0]);
            let dy1 = store.get(self.pvec[3]) - store.get(self.pvec[1]);
            let dx2 = store.get(self.pvec[6]) - store.get(self.pvec[4]);
            let dy2 = store.get(self.pvec[7]) - store.get(self.pvec[5]);
            let a = dy1.atan2(dx1) + store.get(self.angle());
            let ca = a.cos();
            let sa = a.sin();
            let x = dx2 * ca + dy2 * sa;
            let y = -dx2 * sa + dy2 * ca;
            let r2 = dx2 * dx2 + dy2 * dy2;
            let ddx = -y / r2;
            let ddy = x / r2;
            if param == self.pvec[4] { deriv += -ca * ddx + sa * ddy; }
            if param == self.pvec[5] { deriv += -sa * ddx - ca * ddy; }
            if param == self.pvec[6] { deriv += ca * ddx - sa * ddy; }
            if param == self.pvec[7] { deriv += sa * ddx + ca * ddy; }
        }
        if param == self.angle() { deriv -= 1.0; }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let dx1 = store.get(self.pvec[2]) - store.get(self.pvec[0]);
        let dy1 = store.get(self.pvec[3]) - store.get(self.pvec[1]);
        let dx2 = store.get(self.pvec[6]) - store.get(self.pvec[4]);
        let dy2 = store.get(self.pvec[7]) - store.get(self.pvec[5]);
        let a1 = dy1.atan2(dx1);
        let ca = a1.cos();
        let sa = a1.sin();
        let x = dx2 * ca + dy2 * sa;
        let y = -dx2 * sa + dy2 * ca;
        store.set(self.angle(), y.atan2(x));
    }
}

// ---------------------------------------------------------------------------
// MidpointOnLine: midpoint of line1 lies on line2
// ---------------------------------------------------------------------------

pub struct ConstraintMidpointOnLine {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintMidpointOnLine {
    pub fn new(
        l1_p1: Point, l1_p2: Point, l2_p1: Point, l2_p2: Point,
        tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![
                l1_p1.x, l1_p1.y, l1_p2.x, l1_p2.y,
                l2_p1.x, l2_p1.y, l2_p2.x, l2_p2.y,
            ],
            scale: 1.0,
            tag,
            driving,
        }
    }
}

impl Constraint for ConstraintMidpointOnLine {
    fn error(&self, store: &ParamStore) -> f64 {
        let x0 = (store.get(self.pvec[0]) + store.get(self.pvec[2])) / 2.0;
        let y0 = (store.get(self.pvec[1]) + store.get(self.pvec[3])) / 2.0;
        let (x1, y1) = (store.get(self.pvec[4]), store.get(self.pvec[5]));
        let (x2, y2) = (store.get(self.pvec[6]), store.get(self.pvec[7]));
        let dx = x2 - x1;
        let dy = y2 - y1;
        let d = (dx * dx + dy * dy).sqrt();
        let area = -x0 * dy + y0 * dx + x1 * y2 - x2 * y1;
        self.scale * area / d
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        let x0 = (store.get(self.pvec[0]) + store.get(self.pvec[2])) / 2.0;
        let y0 = (store.get(self.pvec[1]) + store.get(self.pvec[3])) / 2.0;
        let (x1, y1) = (store.get(self.pvec[4]), store.get(self.pvec[5]));
        let (x2, y2) = (store.get(self.pvec[6]), store.get(self.pvec[7]));
        let dx = x2 - x1;
        let dy = y2 - y1;
        let d2 = dx * dx + dy * dy;
        let d = d2.sqrt();
        let area = -x0 * dy + y0 * dx + x1 * y2 - x2 * y1;

        // l1p1, l1p2 contribute through midpoint (factor 1/2)
        if param == self.pvec[0] { deriv += (y1 - y2) / (2.0 * d); }
        if param == self.pvec[1] { deriv += (x2 - x1) / (2.0 * d); }
        if param == self.pvec[2] { deriv += (y1 - y2) / (2.0 * d); }
        if param == self.pvec[3] { deriv += (x2 - x1) / (2.0 * d); }
        // l2p1, l2p2 — same as PointOnLine gradient on line endpoints
        if param == self.pvec[4] { deriv += ((y2 - y0) * d + (dx / d) * area) / d2; }
        if param == self.pvec[5] { deriv += ((x0 - x2) * d + (dy / d) * area) / d2; }
        if param == self.pvec[6] { deriv += ((y0 - y1) * d - (dx / d) * area) / d2; }
        if param == self.pvec[7] { deriv += ((x1 - x0) * d - (dy / d) * area) / d2; }

        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ===========================================================================
// Phase 3 — Geometric Constraints
// ===========================================================================

// ---------------------------------------------------------------------------
// TangentCircumf: two circles/arcs tangent to each other
// error = d² - (r1 ± r2)²   (external/internal tangency)
// Singularity guard for concentric case (d²<1e-14 → error = r1-r2)
// ---------------------------------------------------------------------------

pub struct ConstraintTangentCircumf {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    pub internal: bool,
}

impl ConstraintTangentCircumf {
    pub fn new(
        c1: Point, c2: Point, rad1: ParamIdx, rad2: ParamIdx,
        internal: bool, tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![c1.x, c1.y, c2.x, c2.y, rad1, rad2],
            scale: 1.0,
            tag,
            driving,
            internal,
        }
    }

    fn c1x(&self) -> ParamIdx { self.pvec[0] }
    fn c1y(&self) -> ParamIdx { self.pvec[1] }
    fn c2x(&self) -> ParamIdx { self.pvec[2] }
    fn c2y(&self) -> ParamIdx { self.pvec[3] }
    fn r1(&self) -> ParamIdx { self.pvec[4] }
    fn r2(&self) -> ParamIdx { self.pvec[5] }
}

impl Constraint for ConstraintTangentCircumf {
    fn error(&self, store: &ParamStore) -> f64 {
        let dx = store.get(self.c1x()) - store.get(self.c2x());
        let dy = store.get(self.c1y()) - store.get(self.c2y());
        let d_sq = dx * dx + dy * dy;
        if d_sq < 1e-14 {
            return self.scale * (store.get(self.r1()) - store.get(self.r2()));
        }
        let r1 = store.get(self.r1());
        let r2 = store.get(self.r2());
        if self.internal {
            self.scale * (d_sq - (r1 - r2) * (r1 - r2))
        } else {
            self.scale * (d_sq - (r1 + r2) * (r1 + r2))
        }
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let dx = store.get(self.c1x()) - store.get(self.c2x());
        let dy = store.get(self.c1y()) - store.get(self.c2y());
        let d_sq = dx * dx + dy * dy;
        if d_sq < 1e-14 {
            let mut deriv = 0.0;
            if param == self.r1() { deriv = 1.0; }
            else if param == self.r2() { deriv = -1.0; }
            return self.scale * deriv;
        }
        let r1 = store.get(self.r1());
        let r2 = store.get(self.r2());
        let mut deriv = 0.0;
        if param == self.c1x() { deriv += 2.0 * dx; }
        if param == self.c1y() { deriv += 2.0 * dy; }
        if param == self.c2x() { deriv -= 2.0 * dx; }
        if param == self.c2y() { deriv -= 2.0 * dy; }
        if self.internal {
            if param == self.r1() { deriv += 2.0 * (r2 - r1); }
            if param == self.r2() { deriv += 2.0 * (r1 - r2); }
        } else {
            if param == self.r1() { deriv -= 2.0 * (r1 + r2); }
            if param == self.r2() { deriv -= 2.0 * (r1 + r2); }
        }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// PointOnEllipse: |P-F1| + |P-F2| == 2a (focal distance formula)
// ---------------------------------------------------------------------------

pub struct ConstraintPointOnEllipse {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintPointOnEllipse {
    pub fn new(
        p: Point, center: Point, focus1: Point, radmin: ParamIdx,
        tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![p.x, p.y, center.x, center.y, focus1.x, focus1.y, radmin],
            scale: 1.0,
            tag,
            driving,
        }
    }
}

impl Constraint for ConstraintPointOnEllipse {
    fn error(&self, store: &ParamStore) -> f64 {
        let (x0, y0) = (store.get(self.pvec[0]), store.get(self.pvec[1]));
        let (xc, yc) = (store.get(self.pvec[2]), store.get(self.pvec[3]));
        let (xf1, yf1) = (store.get(self.pvec[4]), store.get(self.pvec[5]));
        let b = store.get(self.pvec[6]);
        let dist_f1 = ((x0 - xf1).powi(2) + (y0 - yf1).powi(2)).sqrt();
        let dist_f2 = ((x0 + xf1 - 2.0 * xc).powi(2) + (y0 + yf1 - 2.0 * yc).powi(2)).sqrt();
        let a = (b * b + (xf1 - xc).powi(2) + (yf1 - yc).powi(2)).sqrt();
        self.scale * (dist_f1 + dist_f2 - 2.0 * a)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (x0, y0) = (store.get(self.pvec[0]), store.get(self.pvec[1]));
        let (xc, yc) = (store.get(self.pvec[2]), store.get(self.pvec[3]));
        let (xf1, yf1) = (store.get(self.pvec[4]), store.get(self.pvec[5]));
        let b = store.get(self.pvec[6]);
        let d1 = ((x0 - xf1).powi(2) + (y0 - yf1).powi(2)).sqrt();
        let d2 = ((x0 + xf1 - 2.0 * xc).powi(2) + (y0 + yf1 - 2.0 * yc).powi(2)).sqrt();
        let da = (b * b + (xf1 - xc).powi(2) + (yf1 - yc).powi(2)).sqrt();
        let mut deriv = 0.0;
        if param == self.pvec[0] {
            deriv += (x0 - xf1) / d1 + (x0 + xf1 - 2.0 * xc) / d2;
        }
        if param == self.pvec[1] {
            deriv += (y0 - yf1) / d1 + (y0 + yf1 - 2.0 * yc) / d2;
        }
        if param == self.pvec[4] {
            deriv += -(x0 - xf1) / d1 - 2.0 * (xf1 - xc) / da + (x0 + xf1 - 2.0 * xc) / d2;
        }
        if param == self.pvec[5] {
            deriv += -(y0 - yf1) / d1 - 2.0 * (yf1 - yc) / da + (y0 + yf1 - 2.0 * yc) / d2;
        }
        if param == self.pvec[2] {
            deriv += 2.0 * (xf1 - xc) / da - 2.0 * (x0 + xf1 - 2.0 * xc) / d2;
        }
        if param == self.pvec[3] {
            deriv += 2.0 * (yf1 - yc) / da - 2.0 * (y0 + yf1 - 2.0 * yc) / d2;
        }
        if param == self.pvec[6] {
            deriv += -2.0 * b / da;
        }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// PointOnHyperbola: |P-F2| - |P-F1| == 2a
// ---------------------------------------------------------------------------

pub struct ConstraintPointOnHyperbola {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintPointOnHyperbola {
    pub fn new(
        p: Point, center: Point, focus1: Point, radmin: ParamIdx,
        tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![p.x, p.y, center.x, center.y, focus1.x, focus1.y, radmin],
            scale: 1.0,
            tag,
            driving,
        }
    }
}

impl Constraint for ConstraintPointOnHyperbola {
    fn error(&self, store: &ParamStore) -> f64 {
        let (x0, y0) = (store.get(self.pvec[0]), store.get(self.pvec[1]));
        let (xc, yc) = (store.get(self.pvec[2]), store.get(self.pvec[3]));
        let (xf1, yf1) = (store.get(self.pvec[4]), store.get(self.pvec[5]));
        let b = store.get(self.pvec[6]);
        let dist_f1 = ((x0 - xf1).powi(2) + (y0 - yf1).powi(2)).sqrt();
        let dist_f2 = ((x0 + xf1 - 2.0 * xc).powi(2) + (y0 + yf1 - 2.0 * yc).powi(2)).sqrt();
        let a = ((xf1 - xc).powi(2) + (yf1 - yc).powi(2) - b * b).sqrt();
        self.scale * (-dist_f1 + dist_f2 - 2.0 * a)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (x0, y0) = (store.get(self.pvec[0]), store.get(self.pvec[1]));
        let (xc, yc) = (store.get(self.pvec[2]), store.get(self.pvec[3]));
        let (xf1, yf1) = (store.get(self.pvec[4]), store.get(self.pvec[5]));
        let b = store.get(self.pvec[6]);
        let d1 = ((x0 - xf1).powi(2) + (y0 - yf1).powi(2)).sqrt();
        let d2 = ((x0 + xf1 - 2.0 * xc).powi(2) + (y0 + yf1 - 2.0 * yc).powi(2)).sqrt();
        let da = ((xf1 - xc).powi(2) + (yf1 - yc).powi(2) - b * b).sqrt();
        let mut deriv = 0.0;
        if param == self.pvec[0] {
            deriv += -(x0 - xf1) / d1 + (x0 + xf1 - 2.0 * xc) / d2;
        }
        if param == self.pvec[1] {
            deriv += -(y0 - yf1) / d1 + (y0 + yf1 - 2.0 * yc) / d2;
        }
        if param == self.pvec[4] {
            deriv += (x0 - xf1) / d1 - 2.0 * (xf1 - xc) / da + (x0 + xf1 - 2.0 * xc) / d2;
        }
        if param == self.pvec[5] {
            deriv += (y0 - yf1) / d1 - 2.0 * (yf1 - yc) / da + (y0 + yf1 - 2.0 * yc) / d2;
        }
        if param == self.pvec[2] {
            deriv += 2.0 * (xf1 - xc) / da - 2.0 * (x0 + xf1 - 2.0 * xc) / d2;
        }
        if param == self.pvec[3] {
            deriv += 2.0 * (yf1 - yc) / da - 2.0 * (y0 + yf1 - 2.0 * yc) / d2;
        }
        if param == self.pvec[6] {
            deriv += 2.0 * b / da;
        }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// PointOnParabola: |P-F| - 2*focal - proj(P-F onto xdir) == 0
// Uses DeriVector2 errorgrad pattern.
// ---------------------------------------------------------------------------

pub struct ConstraintPointOnParabola {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintPointOnParabola {
    pub fn new(
        p: Point, focus: Point, vertex: Point,
        tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![p.x, p.y, focus.x, focus.y, vertex.x, vertex.y],
            scale: 1.0,
            tag,
            driving,
        }
    }

    fn errorgrad_impl(&self, store: &ParamStore, deriv_param: Option<ParamIdx>) -> (f64, f64) {
        let p_pt = Point::new(self.pvec[0], self.pvec[1]);
        let focus_pt = Point::new(self.pvec[2], self.pvec[3]);
        let vertex_pt = Point::new(self.pvec[4], self.pvec[5]);

        let focus = DeriVector2::from_point(store, focus_pt, deriv_param);
        let vertex = DeriVector2::from_point(store, vertex_pt, deriv_param);
        let point = DeriVector2::from_point(store, p_pt, deriv_param);

        let focalvect = focus.sub(&vertex);
        let xdir = focalvect.normalized();
        let point_to_focus = point.sub(&focus);

        let (focal, dfocal) = focalvect.length();
        let (pf, dpf) = point_to_focus.length();
        let (proj, dproj) = point_to_focus.dot(&xdir);

        (pf - 2.0 * focal - proj, dpf - 2.0 * dfocal - dproj)
    }
}

impl Constraint for ConstraintPointOnParabola {
    fn error(&self, store: &ParamStore) -> f64 {
        let (err, _) = self.errorgrad_impl(store, None);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, grad) = self.errorgrad_impl(store, Some(param));
        self.scale * grad
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// EllipseTangentLine: mirror F1 against line, |F1m - F2| == 2*radmaj
// ---------------------------------------------------------------------------

pub struct ConstraintEllipseTangentLine {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    line: Line,
    ellipse: Ellipse,
}

impl ConstraintEllipseTangentLine {
    pub fn new(line: Line, ellipse: Ellipse, tag: Tag, driving: bool) -> Self {
        let mut pvec = Vec::new();
        // Line params: p1.x, p1.y, p2.x, p2.y
        pvec.push(line.p1.x);
        pvec.push(line.p1.y);
        pvec.push(line.p2.x);
        pvec.push(line.p2.y);
        // Ellipse params: center.x, center.y, focus1.x, focus1.y, radmin
        pvec.push(ellipse.center.x);
        pvec.push(ellipse.center.y);
        pvec.push(ellipse.focus1.x);
        pvec.push(ellipse.focus1.y);
        pvec.push(ellipse.radmin);
        Self {
            pvec,
            scale: 1.0,
            tag,
            driving,
            line,
            ellipse,
        }
    }

    fn errorgrad_impl(&self, store: &ParamStore, param: Option<ParamIdx>) -> (f64, f64) {
        let p1 = DeriVector2::from_point(store, self.line.p1, param);
        let p2 = DeriVector2::from_point(store, self.line.p2, param);
        let f1 = DeriVector2::from_point(store, self.ellipse.focus1, param);
        let c = DeriVector2::from_point(store, self.ellipse.center, param);
        let f2 = c.lin_combi(2.0, &f1, -1.0); // 2*c - f1

        // line normal (normalized)
        let nl = p2.sub(&p1).rotate90ccw().normalized();

        // distance F1 to line
        let (dist_f1_l, ddist_f1_l) = f1.sub(&p1).dot(&nl);

        // mirror F1 against line: f1m = f1 - 2*dist*nl
        let f1m = f1.add(&nl.mul_scalar(-2.0 * dist_f1_l, -2.0 * ddist_f1_l));

        // distance from f1m to f2
        let (dist_f1m_f2, ddist_f1m_f2) = f2.sub(&f1m).length();

        // major radius
        let (b, db) = DeriVector2::from_scalar(store, self.ellipse.radmin, param);
        let (radmaj, dradmaj) = self.ellipse.get_rad_maj_from_dv(&c, &f1, b, db);

        (dist_f1m_f2 - 2.0 * radmaj, ddist_f1m_f2 - 2.0 * dradmaj)
    }
}

impl Constraint for ConstraintEllipseTangentLine {
    fn error(&self, store: &ParamStore) -> f64 {
        let (err, _) = self.errorgrad_impl(store, None);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, grad) = self.errorgrad_impl(store, Some(param));
        self.scale * grad
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// InternalAlignmentPoint2Ellipse: aligning special ellipse points
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EllipseAlignmentType {
    PositiveMajorX,
    PositiveMajorY,
    NegativeMajorX,
    NegativeMajorY,
    PositiveMinorX,
    PositiveMinorY,
    NegativeMinorX,
    NegativeMinorY,
    Focus2X,
    Focus2Y,
}

pub struct ConstraintInternalAlignmentPoint2Ellipse {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    ellipse: Ellipse,
    point: Point,
    pub alignment_type: EllipseAlignmentType,
}

impl ConstraintInternalAlignmentPoint2Ellipse {
    pub fn new(
        ellipse: Ellipse, p: Point,
        alignment_type: EllipseAlignmentType,
        tag: Tag, driving: bool,
    ) -> Self {
        let mut pvec = Vec::new();
        pvec.push(p.x);
        pvec.push(p.y);
        pvec.push(ellipse.center.x);
        pvec.push(ellipse.center.y);
        pvec.push(ellipse.focus1.x);
        pvec.push(ellipse.focus1.y);
        pvec.push(ellipse.radmin);
        Self {
            pvec,
            scale: 1.0,
            tag,
            driving,
            ellipse,
            point: p,
            alignment_type,
        }
    }

    fn errorgrad_impl(&self, store: &ParamStore, param: Option<ParamIdx>) -> (f64, f64) {
        let c = DeriVector2::from_point(store, self.ellipse.center, param);
        let f1 = DeriVector2::from_point(store, self.ellipse.focus1, param);
        let emaj = f1.sub(&c).normalized();
        let emin = emaj.rotate90ccw();
        let pv = DeriVector2::from_point(store, self.point, param);
        let (b, db) = DeriVector2::from_scalar(store, self.ellipse.radmin, param);
        let (a, da) = self.ellipse.get_rad_maj_from_dv(&c, &f1, b, db);

        let (poa, by_y) = match self.alignment_type {
            EllipseAlignmentType::PositiveMajorX => (c.add(&emaj.mul_scalar(a, da)), false),
            EllipseAlignmentType::PositiveMajorY => (c.add(&emaj.mul_scalar(a, da)), true),
            EllipseAlignmentType::NegativeMajorX => (c.add(&emaj.mul_scalar(-a, -da)), false),
            EllipseAlignmentType::NegativeMajorY => (c.add(&emaj.mul_scalar(-a, -da)), true),
            EllipseAlignmentType::PositiveMinorX => (c.add(&emin.mul_scalar(b, db)), false),
            EllipseAlignmentType::PositiveMinorY => (c.add(&emin.mul_scalar(b, db)), true),
            EllipseAlignmentType::NegativeMinorX => (c.add(&emin.mul_scalar(-b, -db)), false),
            EllipseAlignmentType::NegativeMinorY => (c.add(&emin.mul_scalar(-b, -db)), true),
            EllipseAlignmentType::Focus2X => (c.lin_combi(2.0, &f1, -1.0), false),
            EllipseAlignmentType::Focus2Y => (c.lin_combi(2.0, &f1, -1.0), true),
        };

        if by_y {
            (pv.y - poa.y, pv.dy - poa.dy)
        } else {
            (pv.x - poa.x, pv.dx - poa.dx)
        }
    }
}

impl Constraint for ConstraintInternalAlignmentPoint2Ellipse {
    fn error(&self, store: &ParamStore) -> f64 {
        let (err, _) = self.errorgrad_impl(store, None);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, grad) = self.errorgrad_impl(store, Some(param));
        self.scale * grad
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// InternalAlignmentPoint2Hyperbola: aligning special hyperbola points
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HyperbolaAlignmentType {
    PositiveMajorX,
    PositiveMajorY,
    NegativeMajorX,
    NegativeMajorY,
    PositiveMinorX,
    PositiveMinorY,
    NegativeMinorX,
    NegativeMinorY,
}

pub struct ConstraintInternalAlignmentPoint2Hyperbola {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    hyperbola: Hyperbola,
    point: Point,
    pub alignment_type: HyperbolaAlignmentType,
}

impl ConstraintInternalAlignmentPoint2Hyperbola {
    pub fn new(
        hyperbola: Hyperbola, p: Point,
        alignment_type: HyperbolaAlignmentType,
        tag: Tag, driving: bool,
    ) -> Self {
        let mut pvec = Vec::new();
        pvec.push(p.x);
        pvec.push(p.y);
        pvec.push(hyperbola.center.x);
        pvec.push(hyperbola.center.y);
        pvec.push(hyperbola.focus1.x);
        pvec.push(hyperbola.focus1.y);
        pvec.push(hyperbola.radmin);
        Self {
            pvec,
            scale: 1.0,
            tag,
            driving,
            hyperbola,
            point: p,
            alignment_type,
        }
    }

    fn errorgrad_impl(&self, store: &ParamStore, param: Option<ParamIdx>) -> (f64, f64) {
        let c = DeriVector2::from_point(store, self.hyperbola.center, param);
        let f1 = DeriVector2::from_point(store, self.hyperbola.focus1, param);
        let emaj = f1.sub(&c).normalized();
        let emin = emaj.rotate90ccw();
        let pv = DeriVector2::from_point(store, self.point, param);
        let (b, db) = DeriVector2::from_scalar(store, self.hyperbola.radmin, param);
        let (a, da) = self.hyperbola.get_rad_maj_from_dv(&c, &f1, b, db);

        let (poa, by_y) = match self.alignment_type {
            HyperbolaAlignmentType::PositiveMajorX => (c.add(&emaj.mul_scalar(a, da)), false),
            HyperbolaAlignmentType::PositiveMajorY => (c.add(&emaj.mul_scalar(a, da)), true),
            HyperbolaAlignmentType::NegativeMajorX => (c.add(&emaj.mul_scalar(-a, -da)), false),
            HyperbolaAlignmentType::NegativeMajorY => (c.add(&emaj.mul_scalar(-a, -da)), true),
            HyperbolaAlignmentType::PositiveMinorX => {
                let pa = c.add(&emaj.mul_scalar(a, da));
                (pa.add(&emin.mul_scalar(b, db)), false)
            }
            HyperbolaAlignmentType::PositiveMinorY => {
                let pa = c.add(&emaj.mul_scalar(a, da));
                (pa.add(&emin.mul_scalar(b, db)), true)
            }
            HyperbolaAlignmentType::NegativeMinorX => {
                let pa = c.add(&emaj.mul_scalar(a, da));
                (pa.add(&emin.mul_scalar(-b, -db)), false)
            }
            HyperbolaAlignmentType::NegativeMinorY => {
                let pa = c.add(&emaj.mul_scalar(a, da));
                (pa.add(&emin.mul_scalar(-b, -db)), true)
            }
        };

        if by_y {
            (pv.y - poa.y, pv.dy - poa.dy)
        } else {
            (pv.x - poa.x, pv.dx - poa.dx)
        }
    }
}

impl Constraint for ConstraintInternalAlignmentPoint2Hyperbola {
    fn error(&self, store: &ParamStore) -> f64 {
        let (err, _) = self.errorgrad_impl(store, None);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, grad) = self.errorgrad_impl(store, Some(param));
        self.scale * grad
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// EqualMajorAxesConic: a1 == a2 for two major-radius conics
// Stores ellipse/hyperbola params inline; uses get_rad_maj_from_dv.
// ---------------------------------------------------------------------------

pub struct ConstraintEqualMajorAxesConic {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    e1: EqualMajorAxesConicData,
    e2: EqualMajorAxesConicData,
}

#[derive(Clone)]
enum EqualMajorAxesConicData {
    Ellipse(Ellipse),
    Hyperbola(Hyperbola),
}

impl EqualMajorAxesConicData {
    fn get_rad_maj(&self, store: &ParamStore, param: Option<ParamIdx>) -> (f64, f64) {
        match self {
            EqualMajorAxesConicData::Ellipse(e) => e.get_rad_maj(store, param),
            EqualMajorAxesConicData::Hyperbola(h) => h.get_rad_maj(store, param),
        }
    }

    fn push_params(&self, pvec: &mut Vec<ParamIdx>) {
        match self {
            EqualMajorAxesConicData::Ellipse(e) => {
                pvec.push(e.center.x); pvec.push(e.center.y);
                pvec.push(e.focus1.x); pvec.push(e.focus1.y);
                pvec.push(e.radmin);
            }
            EqualMajorAxesConicData::Hyperbola(h) => {
                pvec.push(h.center.x); pvec.push(h.center.y);
                pvec.push(h.focus1.x); pvec.push(h.focus1.y);
                pvec.push(h.radmin);
            }
        }
    }
}

impl ConstraintEqualMajorAxesConic {
    pub fn new_ellipse_ellipse(e1: Ellipse, e2: Ellipse, tag: Tag, driving: bool) -> Self {
        let d1 = EqualMajorAxesConicData::Ellipse(e1);
        let d2 = EqualMajorAxesConicData::Ellipse(e2);
        let mut pvec = Vec::new();
        d1.push_params(&mut pvec);
        d2.push_params(&mut pvec);
        Self { pvec, scale: 1.0, tag, driving, e1: d1, e2: d2 }
    }

    pub fn new_hyperbola_hyperbola(h1: Hyperbola, h2: Hyperbola, tag: Tag, driving: bool) -> Self {
        let d1 = EqualMajorAxesConicData::Hyperbola(h1);
        let d2 = EqualMajorAxesConicData::Hyperbola(h2);
        let mut pvec = Vec::new();
        d1.push_params(&mut pvec);
        d2.push_params(&mut pvec);
        Self { pvec, scale: 1.0, tag, driving, e1: d1, e2: d2 }
    }
}

impl Constraint for ConstraintEqualMajorAxesConic {
    fn error(&self, store: &ParamStore) -> f64 {
        let (a1, _) = self.e1.get_rad_maj(store, None);
        let (a2, _) = self.e2.get_rad_maj(store, None);
        self.scale * (a2 - a1)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, da1) = self.e1.get_rad_maj(store, Some(param));
        let (_, da2) = self.e2.get_rad_maj(store, Some(param));
        self.scale * (da2 - da1)
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// EqualFocalDistance: |v1-f1| == |v2-f2| (parabola focal distances)
// ---------------------------------------------------------------------------

pub struct ConstraintEqualFocalDistance {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintEqualFocalDistance {
    pub fn new(
        focus1: Point, vertex1: Point,
        focus2: Point, vertex2: Point,
        tag: Tag, driving: bool,
    ) -> Self {
        Self {
            pvec: vec![
                focus1.x, focus1.y, vertex1.x, vertex1.y,
                focus2.x, focus2.y, vertex2.x, vertex2.y,
            ],
            scale: 1.0,
            tag,
            driving,
        }
    }

    fn errorgrad_impl(&self, store: &ParamStore, param: Option<ParamIdx>) -> (f64, f64) {
        let f1 = DeriVector2::from_point(store, Point::new(self.pvec[0], self.pvec[1]), param);
        let v1 = DeriVector2::from_point(store, Point::new(self.pvec[2], self.pvec[3]), param);
        let f2 = DeriVector2::from_point(store, Point::new(self.pvec[4], self.pvec[5]), param);
        let v2 = DeriVector2::from_point(store, Point::new(self.pvec[6], self.pvec[7]), param);

        let (focal1, dfocal1) = v1.sub(&f1).length();
        let (focal2, dfocal2) = v2.sub(&f2).length();

        (focal2 - focal1, dfocal2 - dfocal1)
    }
}

impl Constraint for ConstraintEqualFocalDistance {
    fn error(&self, store: &ParamStore) -> f64 {
        let (err, _) = self.errorgrad_impl(store, None);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, grad) = self.errorgrad_impl(store, Some(param));
        self.scale * grad
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// CurveValue: p.coord == curve.value(u).coord
// Uses a boxed Curve trait object for polymorphism.
// ---------------------------------------------------------------------------

pub struct ConstraintCurveValue {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    point: Point,
    u_param: ParamIdx,
    pcoord_is_y: bool,
    curve: Box<dyn Curve>,
}

impl ConstraintCurveValue {
    pub fn new(
        p: Point, pcoord_is_y: bool, curve: Box<dyn Curve>, u: ParamIdx,
        tag: Tag, driving: bool,
    ) -> Self {
        let mut pvec = vec![p.x, p.y, u];
        pvec.extend(curve.params());
        Self {
            pvec,
            scale: 1.0,
            tag,
            driving,
            point: p,
            u_param: u,
            pcoord_is_y,
            curve,
        }
    }

    fn errorgrad_impl(&self, store: &ParamStore, param: Option<ParamIdx>) -> (f64, f64) {
        let u = store.get(self.u_param);
        let du = if param == Some(self.u_param) { 1.0 } else { 0.0 };
        let p_to = self.curve.value(store, u, du, param);
        let p_from = DeriVector2::from_point(store, self.point, param);
        let err_vec = p_from.sub(&p_to);
        if self.pcoord_is_y {
            (err_vec.y, err_vec.dy)
        } else {
            (err_vec.x, err_vec.dx)
        }
    }
}

impl Constraint for ConstraintCurveValue {
    fn error(&self, store: &ParamStore) -> f64 {
        let (err, _) = self.errorgrad_impl(store, None);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, grad) = self.errorgrad_impl(store, Some(param));
        self.scale * grad
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// ArcLength: r * (endAngle - startAngle) == distance
// ---------------------------------------------------------------------------

pub struct ConstraintArcLength {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    arc: Arc,
}

impl ConstraintArcLength {
    pub fn new(arc: Arc, dist: ParamIdx, tag: Tag, driving: bool) -> Self {
        let pvec = vec![
            dist, arc.center.x, arc.center.y, arc.rad,
            arc.start_angle, arc.end_angle,
            arc.start.x, arc.start.y, arc.end.x, arc.end.y,
        ];
        Self {
            pvec,
            scale: 1.0,
            tag,
            driving,
            arc,
        }
    }

    fn distance(&self) -> ParamIdx { self.pvec[0] }

    fn normalized_angles(&self, store: &ParamStore) -> (f64, f64) {
        let mut start = store.get(self.arc.start_angle);
        let mut end = store.get(self.arc.end_angle);
        let two_pi = 2.0 * std::f64::consts::PI;
        while start < 0.0 { start += two_pi; }
        while end < start { end += two_pi; }
        (start, end)
    }
}

impl Constraint for ConstraintArcLength {
    fn error(&self, store: &ParamStore) -> f64 {
        let rad = store.get(self.arc.rad);
        let (start, end) = self.normalized_angles(store);
        self.scale * (rad * (end - start) - store.get(self.distance()))
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        if param == self.distance() {
            return self.scale * -1.0;
        }
        let rad = store.get(self.arc.rad);
        let (start, end) = self.normalized_angles(store);
        let d_rad = if param == self.arc.rad { 1.0 } else { 0.0 };
        let d_start = if param == self.arc.start_angle { 1.0 } else { 0.0 };
        let d_end = if param == self.arc.end_angle { 1.0 } else { 0.0 };
        self.scale * (rad * (d_end - d_start) + d_rad * (end - start))
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let rad = store.get(self.arc.rad);
        let (start, end) = self.normalized_angles(store);
        store.set(self.distance(), (end - start) * rad);
    }
}

// ---------------------------------------------------------------------------
// EqualLineLength: |l1| == |l2| with surrogate gradient ±1e-10
// ---------------------------------------------------------------------------

pub struct ConstraintEqualLineLength {
    pvec: Vec<ParamIdx>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
    l1: Line,
    l2: Line,
}

impl ConstraintEqualLineLength {
    pub fn new(l1: Line, l2: Line, tag: Tag, driving: bool) -> Self {
        let pvec = vec![
            l1.p1.x, l1.p1.y, l1.p2.x, l1.p2.y,
            l2.p1.x, l2.p1.y, l2.p2.x, l2.p2.y,
        ];
        Self {
            pvec,
            scale: 1.0,
            tag,
            driving,
            l1,
            l2,
        }
    }

    fn errorgrad_impl(&self, store: &ParamStore, param: Option<ParamIdx>) -> (f64, f64) {
        let p1 = DeriVector2::from_point(store, self.l1.p1, param);
        let p2 = DeriVector2::from_point(store, self.l1.p2, param);
        let p3 = DeriVector2::from_point(store, self.l2.p1, param);
        let p4 = DeriVector2::from_point(store, self.l2.p2, param);

        let v1 = p1.sub(&p2);
        let v2 = p3.sub(&p4);

        let (length1, dlength1) = v1.length();
        let (length2, dlength2) = v2.length();

        let err = length2 - length1;
        let mut grad = dlength2 - dlength1;

        // Surrogate gradient to avoid zero gradient when line is vertical/horizontal
        if let Some(p) = param {
            if grad.abs() < 1e-10 {
                let surrogate = 1e-10_f64;
                if p == self.l1.p1.x { grad = if v1.x > 0.0 { surrogate } else { -surrogate }; }
                if p == self.l1.p1.y { grad = if v1.y > 0.0 { surrogate } else { -surrogate }; }
                if p == self.l1.p2.x { grad = if v1.x > 0.0 { -surrogate } else { surrogate }; }
                if p == self.l1.p2.y { grad = if v1.y > 0.0 { -surrogate } else { surrogate }; }
                if p == self.l2.p1.x { grad = if v2.x > 0.0 { surrogate } else { -surrogate }; }
                if p == self.l2.p1.y { grad = if v2.y > 0.0 { surrogate } else { -surrogate }; }
                if p == self.l2.p2.x { grad = if v2.x > 0.0 { -surrogate } else { surrogate }; }
                if p == self.l2.p2.y { grad = if v2.y > 0.0 { -surrogate } else { surrogate }; }
            }
        }

        (err, grad)
    }
}

impl Constraint for ConstraintEqualLineLength {
    fn error(&self, store: &ParamStore) -> f64 {
        let (err, _) = self.errorgrad_impl(store, None);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (_, grad) = self.errorgrad_impl(store, Some(param));
        self.scale * grad
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn equal_constraint_satisfied() {
        let mut store = ParamStore::new();
        let a = store.push(5.0);
        let b = store.push(5.0);
        let c = ConstraintEqual::new(a, b, 1.0, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-15);
    }

    #[test]
    fn equal_constraint_error() {
        let mut store = ParamStore::new();
        let a = store.push(5.0);
        let b = store.push(3.0);
        let c = ConstraintEqual::new(a, b, 1.0, 0, true);
        assert_abs_diff_eq!(c.error(&store), 2.0, epsilon = 1e-15);
    }

    #[test]
    fn equal_gradient_fd() {
        let mut store = ParamStore::new();
        let a = store.push(5.0);
        let b = store.push(3.0);
        let c = ConstraintEqual::new(a, b, 1.0, 0, true);
        let g = c.grad(&store, a);
        let eps = 1e-8;
        let e0 = c.error(&store);
        store.set(a, 5.0 + eps);
        let e1 = c.error(&store);
        assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-6);
    }

    #[test]
    fn difference_constraint() {
        let mut store = ParamStore::new();
        let a = store.push(3.0);
        let b = store.push(7.0);
        let d = store.push(4.0);
        let c = ConstraintDifference::new(a, b, d, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-15);
    }

    #[test]
    fn p2p_distance_satisfied() {
        let mut store = ParamStore::new();
        let p1 = Point::new(store.push(0.0), store.push(0.0));
        let p2 = Point::new(store.push(3.0), store.push(4.0));
        let d = store.push(5.0);
        let c = ConstraintP2PDistance::new(p1, p2, d, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-15);
    }

    #[test]
    fn p2p_distance_gradient_fd() {
        let mut store = ParamStore::new();
        let p1 = Point::new(store.push(0.0), store.push(0.0));
        let p2 = Point::new(store.push(3.0), store.push(4.0));
        let d = store.push(4.0);
        let c = ConstraintP2PDistance::new(p1, p2, d, 0, true);
        for &param in &[p1.x, p1.y, p2.x, p2.y, d] {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5,);
        }
    }

    #[test]
    fn p2p_angle_satisfied() {
        let mut store = ParamStore::new();
        let p1 = Point::new(store.push(0.0), store.push(0.0));
        let p2 = Point::new(store.push(1.0), store.push(0.0));
        let a = store.push(0.0); // angle = 0 for horizontal right
        let c = ConstraintP2PAngle::new(p1, p2, a, 0.0, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-15);
    }

    #[test]
    fn point_on_line_satisfied() {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(1.5), store.push(2.0));
        let l1 = Point::new(store.push(0.0), store.push(0.0));
        let l2 = Point::new(store.push(3.0), store.push(4.0));
        let c = ConstraintPointOnLine::new(p, l1, l2, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn point_on_line_gradient_fd() {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(1.0), store.push(2.5));
        let l1 = Point::new(store.push(0.0), store.push(0.0));
        let l2 = Point::new(store.push(3.0), store.push(4.0));
        let c = ConstraintPointOnLine::new(p, l1, l2, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    #[test]
    fn parallel_satisfied() {
        let mut store = ParamStore::new();
        let l1p1 = Point::new(store.push(0.0), store.push(0.0));
        let l1p2 = Point::new(store.push(1.0), store.push(1.0));
        let l2p1 = Point::new(store.push(0.0), store.push(1.0));
        let l2p2 = Point::new(store.push(2.0), store.push(3.0));
        let c = ConstraintParallel::new(l1p1, l1p2, l2p1, l2p2, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn perpendicular_satisfied() {
        let mut store = ParamStore::new();
        let l1p1 = Point::new(store.push(0.0), store.push(0.0));
        let l1p2 = Point::new(store.push(1.0), store.push(0.0));
        let l2p1 = Point::new(store.push(0.0), store.push(0.0));
        let l2p2 = Point::new(store.push(0.0), store.push(1.0));
        let c = ConstraintPerpendicular::new(l1p1, l1p2, l2p1, l2p2, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-15);
    }

    #[test]
    fn perp_bisector_satisfied() {
        let mut store = ParamStore::new();
        // Midpoint of (0,0)-(4,0) is (2,0). Point (2,5) is on perp bisector.
        let p = Point::new(store.push(2.0), store.push(5.0));
        let l1 = Point::new(store.push(0.0), store.push(0.0));
        let l2 = Point::new(store.push(4.0), store.push(0.0));
        let c = ConstraintPointOnPerpBisector::new(p, l1, l2, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn perp_bisector_gradient_fd() {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(2.5), store.push(3.0));
        let l1 = Point::new(store.push(0.0), store.push(0.0));
        let l2 = Point::new(store.push(4.0), store.push(0.0));
        let c = ConstraintPointOnPerpBisector::new(p, l1, l2, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    #[test]
    fn l2l_angle_satisfied() {
        let mut store = ParamStore::new();
        // Two lines: L1 along x-axis, L2 at 45 degrees
        let l1p1 = Point::new(store.push(0.0), store.push(0.0));
        let l1p2 = Point::new(store.push(1.0), store.push(0.0));
        let l2p1 = Point::new(store.push(0.0), store.push(0.0));
        let l2p2 = Point::new(store.push(1.0), store.push(1.0));
        let a = store.push(std::f64::consts::FRAC_PI_4);
        let c = ConstraintL2LAngle::new(l1p1, l1p2, l2p1, l2p2, a, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn l2l_angle_gradient_fd() {
        let mut store = ParamStore::new();
        let l1p1 = Point::new(store.push(0.0), store.push(0.0));
        let l1p2 = Point::new(store.push(2.0), store.push(1.0));
        let l2p1 = Point::new(store.push(0.0), store.push(0.0));
        let l2p2 = Point::new(store.push(1.0), store.push(3.0));
        let a = store.push(0.5);
        let c = ConstraintL2LAngle::new(l1p1, l1p2, l2p1, l2p2, a, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    #[test]
    fn midpoint_on_line_satisfied() {
        let mut store = ParamStore::new();
        // L1: (0,0)-(4,0), midpoint = (2,0). L2: (0,0)-(3,0) along x-axis.
        let l1p1 = Point::new(store.push(0.0), store.push(0.0));
        let l1p2 = Point::new(store.push(4.0), store.push(0.0));
        let l2p1 = Point::new(store.push(0.0), store.push(0.0));
        let l2p2 = Point::new(store.push(3.0), store.push(0.0));
        let c = ConstraintMidpointOnLine::new(l1p1, l1p2, l2p1, l2p2, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn midpoint_on_line_gradient_fd() {
        let mut store = ParamStore::new();
        let l1p1 = Point::new(store.push(1.0), store.push(2.0));
        let l1p2 = Point::new(store.push(5.0), store.push(3.0));
        let l2p1 = Point::new(store.push(0.0), store.push(0.0));
        let l2p2 = Point::new(store.push(4.0), store.push(1.0));
        let c = ConstraintMidpointOnLine::new(l1p1, l1p2, l2p1, l2p2, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    // ===== Phase 3: Geometric constraint tests =====

    #[test]
    fn tangent_circumf_external_satisfied() {
        let mut store = ParamStore::new();
        // Circle 1 at (0,0) r=3, Circle 2 at (5,0) r=2 → distance=5 = r1+r2
        let c1 = Point::new(store.push(0.0), store.push(0.0));
        let c2 = Point::new(store.push(5.0), store.push(0.0));
        let r1 = store.push(3.0);
        let r2 = store.push(2.0);
        let c = ConstraintTangentCircumf::new(c1, c2, r1, r2, false, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn tangent_circumf_internal_satisfied() {
        let mut store = ParamStore::new();
        // Circle 1 at (0,0) r=5, Circle 2 at (2,0) r=3 → d=2, r1-r2=2 → d²=(r1-r2)²
        let c1 = Point::new(store.push(0.0), store.push(0.0));
        let c2 = Point::new(store.push(2.0), store.push(0.0));
        let r1 = store.push(5.0);
        let r2 = store.push(3.0);
        let c = ConstraintTangentCircumf::new(c1, c2, r1, r2, true, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn tangent_circumf_gradient_fd() {
        let mut store = ParamStore::new();
        let c1 = Point::new(store.push(0.0), store.push(0.0));
        let c2 = Point::new(store.push(4.0), store.push(3.0));
        let r1 = store.push(2.0);
        let r2 = store.push(1.5);
        let c = ConstraintTangentCircumf::new(c1, c2, r1, r2, false, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-4);
        }
    }

    #[test]
    fn point_on_ellipse_satisfied() {
        let mut store = ParamStore::new();
        // Ellipse: center (0,0), focus1 (3,0), rmin=4 → a = sqrt(16+9)=5
        // Point on ellipse: (5,0) → |P-F1|=2, |P-F2|=8 → sum=10=2*5 ✓
        let p = Point::new(store.push(5.0), store.push(0.0));
        let center = Point::new(store.push(0.0), store.push(0.0));
        let focus1 = Point::new(store.push(3.0), store.push(0.0));
        let rmin = store.push(4.0);
        let c = ConstraintPointOnEllipse::new(p, center, focus1, rmin, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn point_on_ellipse_gradient_fd() {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(4.0), store.push(2.0));
        let center = Point::new(store.push(0.0), store.push(0.0));
        let focus1 = Point::new(store.push(3.0), store.push(0.0));
        let rmin = store.push(4.0);
        let c = ConstraintPointOnEllipse::new(p, center, focus1, rmin, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    #[test]
    fn point_on_hyperbola_satisfied() {
        let mut store = ParamStore::new();
        // Hyperbola: center (0,0), focus1 (5,0), rmin=3 → a = sqrt(25-9)=4
        // Point (4,0): |P-F2|=9, |P-F1|=1 → diff=8=2*4 ✓
        let p = Point::new(store.push(4.0), store.push(0.0));
        let center = Point::new(store.push(0.0), store.push(0.0));
        let focus1 = Point::new(store.push(5.0), store.push(0.0));
        let rmin = store.push(3.0);
        let c = ConstraintPointOnHyperbola::new(p, center, focus1, rmin, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn point_on_hyperbola_gradient_fd() {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(5.0), store.push(1.0));
        let center = Point::new(store.push(0.0), store.push(0.0));
        let focus1 = Point::new(store.push(5.0), store.push(0.0));
        let rmin = store.push(3.0);
        let c = ConstraintPointOnHyperbola::new(p, center, focus1, rmin, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-4);
        }
    }

    #[test]
    fn point_on_parabola_gradient_fd() {
        let mut store = ParamStore::new();
        // Parabola: vertex (0,0), focus (1,0). Point (1,2): pf=sqrt(1+4)=√5,
        // focal=1, proj=0/1=0 → pf - 2 - 0 = √5 - 2 ≈ 0.236
        let p = Point::new(store.push(1.0), store.push(2.0));
        let focus = Point::new(store.push(1.0), store.push(0.0));
        let vertex = Point::new(store.push(0.0), store.push(0.0));
        let c = ConstraintPointOnParabola::new(p, focus, vertex, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-4);
        }
    }

    #[test]
    fn ellipse_tangent_line_gradient_fd() {
        let mut store = ParamStore::new();
        let line = cadora_geo::Line {
            p1: Point::new(store.push(5.0), store.push(-1.0)),
            p2: Point::new(store.push(5.0), store.push(1.0)),
        };
        let ellipse = cadora_geo::Ellipse {
            center: Point::new(store.push(0.0), store.push(0.0)),
            focus1: Point::new(store.push(3.0), store.push(0.0)),
            radmin: store.push(4.0),
        };
        let c = ConstraintEllipseTangentLine::new(line, ellipse, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-4);
        }
    }

    #[test]
    fn equal_major_axes_conic_satisfied() {
        let mut store = ParamStore::new();
        // Two ellipses with same major radius
        let e1 = cadora_geo::Ellipse {
            center: Point::new(store.push(0.0), store.push(0.0)),
            focus1: Point::new(store.push(3.0), store.push(0.0)),
            radmin: store.push(4.0),
        };
        let e2 = cadora_geo::Ellipse {
            center: Point::new(store.push(10.0), store.push(0.0)),
            focus1: Point::new(store.push(13.0), store.push(0.0)),
            radmin: store.push(4.0),
        };
        let c = ConstraintEqualMajorAxesConic::new_ellipse_ellipse(e1, e2, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn equal_focal_distance_gradient_fd() {
        let mut store = ParamStore::new();
        let f1 = Point::new(store.push(1.0), store.push(0.0));
        let v1 = Point::new(store.push(0.0), store.push(0.0));
        let f2 = Point::new(store.push(3.0), store.push(1.0));
        let v2 = Point::new(store.push(1.0), store.push(1.0));
        let c = ConstraintEqualFocalDistance::new(f1, v1, f2, v2, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    #[test]
    fn curve_value_satisfied() {
        let mut store = ParamStore::new();
        // Line from (0,0) to (4,6). At u=0.5, value = (2,3).
        let line = cadora_geo::Line {
            p1: Point::new(store.push(0.0), store.push(0.0)),
            p2: Point::new(store.push(4.0), store.push(6.0)),
        };
        let p = Point::new(store.push(2.0), store.push(3.0)); // x-coord test
        let u = store.push(0.5);
        let c = ConstraintCurveValue::new(p, false, Box::new(line), u, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn curve_value_gradient_fd() {
        let mut store = ParamStore::new();
        let line = cadora_geo::Line {
            p1: Point::new(store.push(0.0), store.push(0.0)),
            p2: Point::new(store.push(4.0), store.push(6.0)),
        };
        let p = Point::new(store.push(2.5), store.push(3.0));
        let u = store.push(0.5);
        let c = ConstraintCurveValue::new(p, false, Box::new(line), u, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    #[test]
    fn arc_length_satisfied() {
        let mut store = ParamStore::new();
        // Arc: center (0,0), rad=2, angles 0→π/2 → length = 2 * π/2 = π
        let arc = cadora_geo::Arc {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(2.0),
            start_angle: store.push(0.0),
            end_angle: store.push(std::f64::consts::FRAC_PI_2),
            start: Point::new(store.push(2.0), store.push(0.0)),
            end: Point::new(store.push(0.0), store.push(2.0)),
        };
        let d = store.push(std::f64::consts::PI);
        let c = ConstraintArcLength::new(arc, d, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn arc_length_gradient_fd() {
        let mut store = ParamStore::new();
        let arc = cadora_geo::Arc {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(3.0),
            start_angle: store.push(0.5),
            end_angle: store.push(2.0),
            start: Point::new(store.push(0.0), store.push(0.0)),
            end: Point::new(store.push(0.0), store.push(0.0)),
        };
        let d = store.push(4.0);
        let c = ConstraintArcLength::new(arc, d, 0, true);
        // Only test differentiable params: rad, start_angle, end_angle, distance
        let test_params = [c.pvec[0], c.pvec[3], c.pvec[4], c.pvec[5]];
        for &param in &test_params {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-5);
        }
    }

    #[test]
    fn equal_line_length_satisfied() {
        let mut store = ParamStore::new();
        let l1 = cadora_geo::Line {
            p1: Point::new(store.push(0.0), store.push(0.0)),
            p2: Point::new(store.push(3.0), store.push(4.0)),
        };
        let l2 = cadora_geo::Line {
            p1: Point::new(store.push(1.0), store.push(1.0)),
            p2: Point::new(store.push(4.0), store.push(5.0)),
        };
        let c = ConstraintEqualLineLength::new(l1, l2, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn equal_line_length_gradient_fd() {
        let mut store = ParamStore::new();
        let l1 = cadora_geo::Line {
            p1: Point::new(store.push(0.0), store.push(0.0)),
            p2: Point::new(store.push(3.0), store.push(4.0)),
        };
        let l2 = cadora_geo::Line {
            p1: Point::new(store.push(1.0), store.push(1.0)),
            p2: Point::new(store.push(5.0), store.push(2.0)),
        };
        let c = ConstraintEqualLineLength::new(l1, l2, 0, true);
        for &param in c.params() {
            let g = c.grad(&store, param);
            let e0 = c.error(&store);
            let orig = store.get(param);
            let eps = 1e-8;
            store.set(param, orig + eps);
            let e1 = c.error(&store);
            store.set(param, orig);
            assert_abs_diff_eq!(g, (e1 - e0) / eps, epsilon = 1e-4);
        }
    }
}
