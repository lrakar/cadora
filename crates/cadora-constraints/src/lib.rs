//! # cadora-constraints
//!
//! All constraint types with error + gradient computation.
//! Port of planegcs `Constraints.h` / `Constraints.cpp`.

use cadora_core::{ParamIdx, ParamStore, Tag};
use cadora_geo::{Ellipse, Hyperbola, Arc, Line, Circle, Curve};
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

    /// Whether this constraint is an internal-alignment constraint (excluded from conflict reporting).
    fn is_internal_alignment(&self) -> bool { false }
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
    fn is_internal_alignment(&self) -> bool { true }
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
    fn is_internal_alignment(&self) -> bool { true }
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

// ===========================================================================
// Phase 4 — Advanced Constraints
// ===========================================================================

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// `atan2(y,x)` – based angle between two 2-D vectors.
/// Equivalent to rotating v1→align with x-axis, then measuring atan2(v2_rot).
fn vector_angle_helper(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let a = y1.atan2(x1);
    let (sa, ca) = a.sin_cos();
    let x = x2 * ca + y2 * sa;
    let y = -x2 * sa + y2 * ca;
    y.atan2(x)
}

/// Computes d(atan2(n.y, n.x))/d(param) from a DeriVector2 `n`.
#[inline]
fn datan2_d(n: &DeriVector2) -> f64 {
    let len2 = n.x * n.x + n.y * n.y;
    if len2 < 1e-30 { return 0.0; }
    (-n.dx * n.y + n.dy * n.x) / len2
}

// ---------------------------------------------------------------------------
// AngleViaPoint: angle between normals of two curves at a single point
// ---------------------------------------------------------------------------

pub struct ConstraintAngleViaPoint {
    pvec: Vec<ParamIdx>,
    crv1: Box<dyn Curve>,
    crv2: Box<dyn Curve>,
    poa: Point,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintAngleViaPoint {
    pub fn new(
        crv1: Box<dyn Curve>,
        crv2: Box<dyn Curve>,
        angle: ParamIdx,
        poa: Point,
        tag: Tag,
        driving: bool,
    ) -> Self {
        let mut pvec = vec![angle, poa.x, poa.y];
        pvec.extend(crv1.params());
        pvec.extend(crv2.params());
        Self { pvec, crv1, crv2, poa, scale: 1.0, tag, driving }
    }

    #[inline]
    fn angle_idx(&self) -> ParamIdx { self.pvec[0] }
}

impl Constraint for ConstraintAngleViaPoint {
    fn error(&self, store: &ParamStore) -> f64 {
        let ang = store.get(self.angle_idx());
        let n1 = self.crv1.normal_at_point(store, self.poa, None);
        let n2 = self.crv2.normal_at_point(store, self.poa, None);
        let (sa, ca) = ang.sin_cos();
        let n1r_x = n1.x * ca - n1.y * sa;
        let n1r_y = n1.x * sa + n1.y * ca;
        let err = (-n2.x * n1r_y + n2.y * n1r_x).atan2(n2.x * n1r_x + n2.y * n1r_y);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        if !self.pvec.contains(&param) { return 0.0; }
        let mut deriv = 0.0;
        if param == self.angle_idx() { deriv -= 1.0; }
        let n1 = self.crv1.normal_at_point(store, self.poa, Some(param));
        let n2 = self.crv2.normal_at_point(store, self.poa, Some(param));
        deriv -= datan2_d(&n1);
        deriv += datan2_d(&n2);
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// AngleViaTwoPoints: angle between normals of two curves at two different points
// ---------------------------------------------------------------------------

pub struct ConstraintAngleViaTwoPoints {
    pvec: Vec<ParamIdx>,
    crv1: Box<dyn Curve>,
    crv2: Box<dyn Curve>,
    poa1: Point,
    poa2: Point,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintAngleViaTwoPoints {
    pub fn new(
        crv1: Box<dyn Curve>,
        crv2: Box<dyn Curve>,
        angle: ParamIdx,
        poa1: Point,
        poa2: Point,
        tag: Tag,
        driving: bool,
    ) -> Self {
        let mut pvec = vec![angle, poa1.x, poa1.y, poa2.x, poa2.y];
        pvec.extend(crv1.params());
        pvec.extend(crv2.params());
        Self { pvec, crv1, crv2, poa1, poa2, scale: 1.0, tag, driving }
    }

    #[inline]
    fn angle_idx(&self) -> ParamIdx { self.pvec[0] }
}

impl Constraint for ConstraintAngleViaTwoPoints {
    fn error(&self, store: &ParamStore) -> f64 {
        let ang = store.get(self.angle_idx());
        let n1 = self.crv1.normal_at_point(store, self.poa1, None);
        let n2 = self.crv2.normal_at_point(store, self.poa2, None);
        let (sa, ca) = ang.sin_cos();
        let n1r_x = n1.x * ca - n1.y * sa;
        let n1r_y = n1.x * sa + n1.y * ca;
        let err = (-n2.x * n1r_y + n2.y * n1r_x).atan2(n2.x * n1r_x + n2.y * n1r_y);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        if !self.pvec.contains(&param) { return 0.0; }
        let mut deriv = 0.0;
        if param == self.angle_idx() { deriv -= 1.0; }
        let n1 = self.crv1.normal_at_point(store, self.poa1, Some(param));
        let n2 = self.crv2.normal_at_point(store, self.poa2, Some(param));
        deriv -= datan2_d(&n1);
        deriv += datan2_d(&n2);
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let n1 = self.crv1.normal_at_point(store, self.poa1, None);
        let n2 = self.crv2.normal_at_point(store, self.poa2, None);
        store.set(self.angle_idx(), vector_angle_helper(n1.x, n1.y, n2.x, n2.y));
    }
}

// ---------------------------------------------------------------------------
// AngleViaPointAndParam: crv1 normal at param, crv2 normal at point
// ---------------------------------------------------------------------------

pub struct ConstraintAngleViaPointAndParam {
    pvec: Vec<ParamIdx>,
    crv1: Box<dyn Curve>,
    crv2: Box<dyn Curve>,
    poa: Point,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintAngleViaPointAndParam {
    pub fn new(
        crv1: Box<dyn Curve>,
        crv2: Box<dyn Curve>,
        angle: ParamIdx,
        poa: Point,
        cparam: ParamIdx,
        tag: Tag,
        driving: bool,
    ) -> Self {
        // pvec layout: [angle, poa.x, poa.y, cparam, ...crv1 params, ...crv2 params]
        let mut pvec = vec![angle, poa.x, poa.y, cparam];
        pvec.extend(crv1.params());
        pvec.extend(crv2.params());
        Self { pvec, crv1, crv2, poa, scale: 1.0, tag, driving }
    }

    #[inline] fn angle_idx(&self) -> ParamIdx { self.pvec[0] }
    #[inline] fn cparam_idx(&self) -> ParamIdx { self.pvec[3] }
}

impl Constraint for ConstraintAngleViaPointAndParam {
    fn error(&self, store: &ParamStore) -> f64 {
        let ang = store.get(self.angle_idx());
        let u = store.get(self.cparam_idx());
        let n1 = self.crv1.normal_at_param(store, u, None);
        let n2 = self.crv2.normal_at_point(store, self.poa, None);
        let (sa, ca) = ang.sin_cos();
        let n1r_x = n1.x * ca - n1.y * sa;
        let n1r_y = n1.x * sa + n1.y * ca;
        let err = (-n2.x * n1r_y + n2.y * n1r_x).atan2(n2.x * n1r_x + n2.y * n1r_y);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        if !self.pvec.contains(&param) { return 0.0; }
        let mut deriv = 0.0;
        if param == self.angle_idx() { deriv -= 1.0; }
        let u = store.get(self.cparam_idx());
        let n1 = self.crv1.normal_at_param(store, u, Some(param));
        let n2 = self.crv2.normal_at_point(store, self.poa, Some(param));
        deriv -= datan2_d(&n1);
        deriv += datan2_d(&n2);
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let u = store.get(self.cparam_idx());
        let n1 = self.crv1.normal_at_param(store, u, None);
        let n2 = self.crv2.normal_at_point(store, self.poa, None);
        store.set(self.angle_idx(), vector_angle_helper(n1.x, n1.y, n2.x, n2.y));
    }
}

// ---------------------------------------------------------------------------
// AngleViaPointAndTwoParams: both normals at curve parameters
// ---------------------------------------------------------------------------

pub struct ConstraintAngleViaPointAndTwoParams {
    pvec: Vec<ParamIdx>,
    crv1: Box<dyn Curve>,
    crv2: Box<dyn Curve>,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintAngleViaPointAndTwoParams {
    pub fn new(
        crv1: Box<dyn Curve>,
        crv2: Box<dyn Curve>,
        angle: ParamIdx,
        poa: Point,
        cparam1: ParamIdx,
        cparam2: ParamIdx,
        tag: Tag,
        driving: bool,
    ) -> Self {
        // pvec: [angle, poa.x, poa.y, cparam1, cparam2, ...crv1, ...crv2]
        let mut pvec = vec![angle, poa.x, poa.y, cparam1, cparam2];
        pvec.extend(crv1.params());
        pvec.extend(crv2.params());
        Self { pvec, crv1, crv2, scale: 1.0, tag, driving }
    }

    #[inline] fn angle_idx(&self) -> ParamIdx { self.pvec[0] }
    #[inline] fn cparam1_idx(&self) -> ParamIdx { self.pvec[3] }
    #[inline] fn cparam2_idx(&self) -> ParamIdx { self.pvec[4] }
}

impl Constraint for ConstraintAngleViaPointAndTwoParams {
    fn error(&self, store: &ParamStore) -> f64 {
        let ang = store.get(self.angle_idx());
        let u1 = store.get(self.cparam1_idx());
        let u2 = store.get(self.cparam2_idx());
        let n1 = self.crv1.normal_at_param(store, u1, None);
        let n2 = self.crv2.normal_at_param(store, u2, None);
        let (sa, ca) = ang.sin_cos();
        let n1r_x = n1.x * ca - n1.y * sa;
        let n1r_y = n1.x * sa + n1.y * ca;
        let err = (-n2.x * n1r_y + n2.y * n1r_x).atan2(n2.x * n1r_x + n2.y * n1r_y);
        self.scale * err
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        if !self.pvec.contains(&param) { return 0.0; }
        let mut deriv = 0.0;
        if param == self.angle_idx() { deriv -= 1.0; }
        let u1 = store.get(self.cparam1_idx());
        let u2 = store.get(self.cparam2_idx());
        let n1 = self.crv1.normal_at_param(store, u1, Some(param));
        let n2 = self.crv2.normal_at_param(store, u2, Some(param));
        deriv -= datan2_d(&n1);
        deriv += datan2_d(&n2);
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }

    fn evaluate(&self, store: &mut ParamStore) {
        let u1 = store.get(self.cparam1_idx());
        let u2 = store.get(self.cparam2_idx());
        let n1 = self.crv1.normal_at_param(store, u1, None);
        let n2 = self.crv2.normal_at_param(store, u2, None);
        store.set(self.angle_idx(), vector_angle_helper(n1.x, n1.y, n2.x, n2.y));
    }
}

// ---------------------------------------------------------------------------
// Snell: n1*sin(θ1) = n2*sin(θ2) at refraction point
// ---------------------------------------------------------------------------

pub struct ConstraintSnell {
    pvec: Vec<ParamIdx>,
    ray1: Box<dyn Curve>,
    ray2: Box<dyn Curve>,
    boundary: Box<dyn Curve>,
    poa: Point,
    flipn1: bool,
    flipn2: bool,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintSnell {
    pub fn new(
        ray1: Box<dyn Curve>,
        ray2: Box<dyn Curve>,
        boundary: Box<dyn Curve>,
        n1: ParamIdx,
        n2: ParamIdx,
        poa: Point,
        flipn1: bool,
        flipn2: bool,
        tag: Tag,
        driving: bool,
    ) -> Self {
        let mut pvec = vec![n1, n2, poa.x, poa.y];
        pvec.extend(ray1.params());
        pvec.extend(ray2.params());
        pvec.extend(boundary.params());
        Self { pvec, ray1, ray2, boundary, poa, flipn1, flipn2, scale: 1.0, tag, driving }
    }

    #[inline] fn n1_idx(&self) -> ParamIdx { self.pvec[0] }
    #[inline] fn n2_idx(&self) -> ParamIdx { self.pvec[1] }

    fn errorgrad_impl(&self, store: &ParamStore, deriv_param: Option<ParamIdx>) -> (f64, f64) {
        let tang1 = self.ray1.normal_at_point(store, self.poa, deriv_param)
            .rotate90cw().normalized();
        let tang2 = self.ray2.normal_at_point(store, self.poa, deriv_param)
            .rotate90cw().normalized();
        let tang_b = self.boundary.normal_at_point(store, self.poa, deriv_param)
            .rotate90cw().normalized();

        let (mut sin1, mut dsin1) = tang1.dot(&tang_b);
        let (mut sin2, mut dsin2) = tang2.dot(&tang_b);

        if self.flipn1 { sin1 = -sin1; dsin1 = -dsin1; }
        if self.flipn2 { sin2 = -sin2; dsin2 = -dsin2; }

        let n1v = store.get(self.n1_idx());
        let n2v = store.get(self.n2_idx());
        let dn1 = if deriv_param == Some(self.n1_idx()) { 1.0 } else { 0.0 };
        let dn2 = if deriv_param == Some(self.n2_idx()) { 1.0 } else { 0.0 };

        let err = n1v * sin1 - n2v * sin2;
        let grad = dn1 * sin1 + n1v * dsin1 - dn2 * sin2 - n2v * dsin2;
        (err, grad)
    }
}

impl Constraint for ConstraintSnell {
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
// WeightedLinearCombination: q * sum(w_i*f_i) = sum(p_i * w_i * f_i)
//   pvec = [q, p_1..p_n, w_1..w_n]
// ---------------------------------------------------------------------------

pub struct ConstraintWeightedLinearCombination {
    pvec: Vec<ParamIdx>,
    factors: Vec<f64>,
    numpoles: usize,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintWeightedLinearCombination {
    pub fn new(
        numpoles: usize,
        all_params: Vec<ParamIdx>,  // [q, p_1..p_n, w_1..w_n]
        factors: Vec<f64>,
        tag: Tag,
        driving: bool,
    ) -> Self {
        assert_eq!(all_params.len(), 2 * numpoles + 1);
        assert_eq!(factors.len(), numpoles);
        Self { pvec: all_params, factors, numpoles, scale: 1.0, tag, driving }
    }

    #[inline] fn thepoint(&self) -> ParamIdx { self.pvec[0] }
    #[inline] fn poleat(&self, i: usize) -> ParamIdx { self.pvec[1 + i] }
    #[inline] fn weightat(&self, i: usize) -> ParamIdx { self.pvec[1 + self.numpoles + i] }
}

impl Constraint for ConstraintWeightedLinearCombination {
    fn error(&self, store: &ParamStore) -> f64 {
        let q = store.get(self.thepoint());
        let mut sum = 0.0;
        let mut wsum = 0.0;
        for i in 0..self.numpoles {
            let wc = store.get(self.weightat(i)) * self.factors[i];
            wsum += wc;
            sum += store.get(self.poleat(i)) * wc;
        }
        self.scale * (q * wsum - sum)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        if param == self.thepoint() {
            let mut wsum = 0.0;
            for i in 0..self.numpoles {
                wsum += store.get(self.weightat(i)) * self.factors[i];
            }
            return self.scale * wsum;
        }
        for i in 0..self.numpoles {
            if param == self.poleat(i) {
                return self.scale * -(store.get(self.weightat(i)) * self.factors[i]);
            }
            if param == self.weightat(i) {
                let q = store.get(self.thepoint());
                let p = store.get(self.poleat(i));
                return self.scale * (q - p) * self.factors[i];
            }
        }
        0.0
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// CenterOfGravity: center = sum(p_i * w_i)
//   pvec = [center, p_1..p_n]
// ---------------------------------------------------------------------------

pub struct ConstraintCenterOfGravity {
    pvec: Vec<ParamIdx>,
    weights: Vec<f64>,
    numpoints: usize,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintCenterOfGravity {
    pub fn new(
        all_params: Vec<ParamIdx>,  // [center, p_1..p_n]
        weights: Vec<f64>,
        tag: Tag,
        driving: bool,
    ) -> Self {
        let numpoints = all_params.len() - 1;
        assert_eq!(weights.len(), numpoints);
        Self { pvec: all_params, weights, numpoints, scale: 1.0, tag, driving }
    }

    #[inline] fn thecenter(&self) -> ParamIdx { self.pvec[0] }
    #[inline] fn pointat(&self, i: usize) -> ParamIdx { self.pvec[1 + i] }
}

impl Constraint for ConstraintCenterOfGravity {
    fn error(&self, store: &ParamStore) -> f64 {
        let c = store.get(self.thecenter());
        let mut sum = 0.0;
        for i in 0..self.numpoints {
            sum += store.get(self.pointat(i)) * self.weights[i];
        }
        self.scale * (c - sum)
    }

    fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
        let mut deriv = 0.0;
        if param == self.thecenter() {
            deriv = 1.0;
        }
        for i in 0..self.numpoints {
            if param == self.pointat(i) {
                deriv = -self.weights[i];
            }
        }
        self.scale * deriv
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// SlopeAtBSplineKnot: constrains slope at a C1+ knot to be parallel to a line
//   pvec = [polexs..., poleys..., weights..., lp1x, lp1y, lp2x, lp2y]
// ---------------------------------------------------------------------------

pub struct ConstraintSlopeAtBSplineKnot {
    pvec: Vec<ParamIdx>,
    factors: Vec<f64>,
    slopefactors: Vec<f64>,
    numpoles: usize,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintSlopeAtBSplineKnot {
    /// Creates the constraint given pre-computed factors and slopefactors.
    /// `pvec` layout: [pole_x_0..pole_x_n, pole_y_0..pole_y_n, w_0..w_n, lp1x, lp1y, lp2x, lp2y]
    pub fn new(
        pvec: Vec<ParamIdx>,
        factors: Vec<f64>,
        slopefactors: Vec<f64>,
        numpoles: usize,
        tag: Tag,
        driving: bool,
    ) -> Self {
        assert_eq!(pvec.len(), 3 * numpoles + 4);
        assert_eq!(factors.len(), numpoles);
        assert_eq!(slopefactors.len(), numpoles);
        Self { pvec, factors, slopefactors, numpoles, scale: 1.0, tag, driving }
    }

    #[inline] fn polexat(&self, i: usize) -> ParamIdx { self.pvec[i] }
    #[inline] fn poleyat(&self, i: usize) -> ParamIdx { self.pvec[self.numpoles + i] }
    #[inline] fn weightat(&self, i: usize) -> ParamIdx { self.pvec[2 * self.numpoles + i] }
    #[inline] fn linep1x(&self) -> ParamIdx { self.pvec[3 * self.numpoles] }
    #[inline] fn linep1y(&self) -> ParamIdx { self.pvec[3 * self.numpoles + 1] }
    #[inline] fn linep2x(&self) -> ParamIdx { self.pvec[3 * self.numpoles + 2] }
    #[inline] fn linep2y(&self) -> ParamIdx { self.pvec[3 * self.numpoles + 3] }

    fn compute_slopes(&self, store: &ParamStore) -> (f64, f64) {
        let mut xsum = 0.0;
        let mut xslopesum = 0.0;
        let mut ysum = 0.0;
        let mut yslopesum = 0.0;
        let mut wsum = 0.0;
        let mut wslopesum = 0.0;

        for i in 0..self.numpoles {
            let wc = store.get(self.weightat(i)) * self.factors[i];
            let wsc = store.get(self.weightat(i)) * self.slopefactors[i];
            wsum += wc;
            xsum += store.get(self.polexat(i)) * wc;
            ysum += store.get(self.poleyat(i)) * wc;
            wslopesum += wsc;
            xslopesum += store.get(self.polexat(i)) * wsc;
            yslopesum += store.get(self.poleyat(i)) * wsc;
        }
        let sx = wsum * xslopesum - wslopesum * xsum;
        let sy = wsum * yslopesum - wslopesum * ysum;
        (sx, sy)
    }

    fn line_dir(&self, store: &ParamStore) -> (f64, f64) {
        let lx = store.get(self.linep2x()) - store.get(self.linep1x());
        let ly = store.get(self.linep2y()) - store.get(self.linep1y());
        let len = (lx * lx + ly * ly).sqrt();
        (lx / len, ly / len)
    }
}

impl Constraint for ConstraintSlopeAtBSplineKnot {
    fn error(&self, store: &ParamStore) -> f64 {
        let (sx, sy) = self.compute_slopes(store);
        let (dirx, diry) = self.line_dir(store);
        self.scale * (sx * diry - sy * dirx)
    }

    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
        let (dirx, diry) = self.line_dir(store);
        let lx = store.get(self.linep2x()) - store.get(self.linep1x());
        let ly = store.get(self.linep2y()) - store.get(self.linep1y());
        let len_sq = lx * lx + ly * ly;
        let len = len_sq.sqrt();

        // Check pole params
        for i in 0..self.numpoles {
            if param == self.polexat(i) {
                let mut wsum = 0.0;
                let mut wslopesum = 0.0;
                for j in 0..self.numpoles {
                    wsum += store.get(self.weightat(j)) * self.factors[j];
                    wslopesum += store.get(self.weightat(j)) * self.slopefactors[j];
                }
                let result = (wsum * self.slopefactors[i] - wslopesum * self.factors[i]) * diry;
                return self.scale * result;
            }
            if param == self.poleyat(i) {
                let mut wsum = 0.0;
                let mut wslopesum = 0.0;
                for j in 0..self.numpoles {
                    wsum += store.get(self.weightat(j)) * self.factors[j];
                    wslopesum += store.get(self.weightat(j)) * self.slopefactors[j];
                }
                let result = -(wsum * self.slopefactors[i] - wslopesum * self.factors[i]) * dirx;
                return self.scale * result;
            }
            if param == self.weightat(i) {
                let mut xsum = 0.0;
                let mut xslopesum = 0.0;
                let mut ysum = 0.0;
                let mut yslopesum = 0.0;
                for j in 0..self.numpoles {
                    let wc = store.get(self.weightat(j)) * self.factors[j];
                    let wsc = store.get(self.weightat(j)) * self.slopefactors[j];
                    let pxi = store.get(self.polexat(i));
                    let pyi = store.get(self.poleyat(i));
                    xsum += wc * (store.get(self.polexat(j)) - pxi);
                    xslopesum += wsc * (store.get(self.polexat(j)) - pxi);
                    ysum += wc * (store.get(self.poleyat(j)) - pyi);
                    yslopesum += wsc * (store.get(self.poleyat(j)) - pyi);
                }
                let result = (self.factors[i] * xslopesum - self.slopefactors[i] * xsum) * diry
                    - (self.factors[i] * yslopesum - self.slopefactors[i] * ysum) * dirx;
                return self.scale * result;
            }
        }

        // Line params
        let (sx, sy) = self.compute_slopes(store);
        let len_cubed = len_sq * len;

        if param == self.linep1x() {
            let d_dirx = ly * ly / len_cubed;
            let d_diry = -(lx * ly) / len_cubed;
            return self.scale * (sx * (-d_diry) - sy * (-d_dirx));
        }
        if param == self.linep2x() {
            let d_dirx = ly * ly / len_cubed;
            let d_diry = -(lx * ly) / len_cubed;
            return self.scale * (sx * d_diry - sy * d_dirx);
        }
        if param == self.linep1y() {
            let d_dirx = -(lx * ly) / len_cubed;
            let d_diry = lx * lx / len_cubed;
            return self.scale * (sx * (-d_diry) - sy * (-d_dirx));
        }
        if param == self.linep2y() {
            let d_dirx = -(lx * ly) / len_cubed;
            let d_diry = lx * lx / len_cubed;
            return self.scale * (sx * d_diry - sy * d_dirx);
        }

        0.0
    }

    fn rescale(&mut self, store: &ParamStore) {
        let mut sx = 0.0;
        let mut sy = 0.0;
        for i in 0..self.numpoles {
            sx += store.get(self.polexat(i)) * self.slopefactors[i];
            sy += store.get(self.poleyat(i)) * self.slopefactors[i];
        }
        let len = (sx * sx + sy * sy).sqrt();
        if len > 1e-30 {
            self.scale = 1.0 / len;
        }
    }

    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { self.tag }
    fn is_driving(&self) -> bool { self.driving }
}

// ---------------------------------------------------------------------------
// C2CDistance: distance between two circles
// ---------------------------------------------------------------------------

pub struct ConstraintC2CDistance {
    pvec: Vec<ParamIdx>,
    c1: Circle,
    c2: Circle,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintC2CDistance {
    pub fn new(c1: Circle, c2: Circle, distance: ParamIdx, tag: Tag, driving: bool) -> Self {
        let mut pvec = vec![distance];
        pvec.extend(c1.params());
        pvec.extend(c2.params());
        Self { pvec, c1, c2, scale: 1.0, tag, driving }
    }

    #[inline] fn distance_idx(&self) -> ParamIdx { self.pvec[0] }

    fn errorgrad_impl(&self, store: &ParamStore, deriv_param: Option<ParamIdx>) -> (f64, f64) {
        let ct1 = DeriVector2::from_point(store, self.c1.center, deriv_param);
        let ct2 = DeriVector2::from_point(store, self.c2.center, deriv_param);
        let v = ct1.sub(&ct2);
        let (len, dlen) = v.length();

        let r1 = store.get(self.c1.rad);
        let r2 = store.get(self.c2.rad);
        let d = store.get(self.distance_idx());

        if len >= r1 && len >= r2 {
            // Outer case
            let err = len - (r2 + r1 + d);
            let drad = if deriv_param == Some(self.c2.rad)
                || deriv_param == Some(self.c1.rad)
                || deriv_param == Some(self.distance_idx())
            { -1.0 } else { 0.0 };
            (err, dlen + drad)
        } else {
            // Inner case
            let (big_rad_idx, small_rad_idx) = if r1 >= r2 {
                (self.c1.rad, self.c2.rad)
            } else {
                (self.c2.rad, self.c1.rad)
            };
            let big_r = r1.max(r2);
            let small_r = r1.min(r2);
            let small_span = small_r + len + d;
            let err = big_r - small_span;

            let mut drad = 0.0;
            if deriv_param == Some(big_rad_idx) {
                drad = 1.0;
            } else if deriv_param == Some(small_rad_idx) {
                drad = -1.0;
            } else if deriv_param == Some(self.distance_idx()) {
                drad = if d < 0.0 { 1.0 } else { -1.0 };
            }

            let grad = if len > 1e-13 { -dlen + drad } else { drad };
            (err, grad)
        }
    }
}

impl Constraint for ConstraintC2CDistance {
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

    fn evaluate(&self, store: &mut ParamStore) {
        let dx = store.get(self.c1.center.x) - store.get(self.c2.center.x);
        let dy = store.get(self.c1.center.y) - store.get(self.c2.center.y);
        let cdist = (dx * dx + dy * dy).sqrt();
        let r1 = store.get(self.c1.rad);
        let r2 = store.get(self.c2.rad);
        let (small_r, big_r) = if r1 < r2 { (r1, r2) } else { (r2, r1) };
        let d = if cdist > big_r && cdist > small_r {
            cdist - big_r - small_r
        } else {
            big_r - small_r - cdist
        };
        store.set(self.distance_idx(), d);
    }
}

// ---------------------------------------------------------------------------
// C2LDistance: distance between a circle and a line
// ---------------------------------------------------------------------------

pub struct ConstraintC2LDistance {
    pvec: Vec<ParamIdx>,
    circle: Circle,
    line: Line,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintC2LDistance {
    pub fn new(circle: Circle, line: Line, distance: ParamIdx, tag: Tag, driving: bool) -> Self {
        let mut pvec = vec![distance];
        pvec.extend(circle.params());
        pvec.extend(line.params());
        Self { pvec, circle, line, scale: 1.0, tag, driving }
    }

    #[inline] fn distance_idx(&self) -> ParamIdx { self.pvec[0] }

    /// Center-to-line perpendicular distance and its derivative.
    fn center_to_line_dist(&self, store: &ParamStore, deriv_param: Option<ParamIdx>) -> (f64, f64) {
        let ct = DeriVector2::from_point(store, self.circle.center, deriv_param);
        let p1 = DeriVector2::from_point(store, self.line.p1, deriv_param);
        let p2 = DeriVector2::from_point(store, self.line.p2, deriv_param);
        let v_line = p2.sub(&p1);
        let v_p1ct = ct.sub(&p1);

        let (area, darea_raw) = v_line.cross_z(&v_p1ct);
        let (length, dlength) = v_line.length();

        let h = area.abs() / length;
        let darea = if area < 0.0 { -darea_raw } else { darea_raw };
        let dh = (darea - h * dlength) / length;
        (h, dh)
    }

    fn errorgrad_impl(&self, store: &ParamStore, deriv_param: Option<ParamIdx>) -> (f64, f64) {
        let (h, dh) = self.center_to_line_dist(store, deriv_param);
        let r = store.get(self.circle.rad);
        let d = store.get(self.distance_idx());

        if h < r {
            let err = r - d.abs() - h;
            let grad = if deriv_param == Some(self.distance_idx())
                || deriv_param == Some(self.circle.rad)
            { -1.0 } else { -dh };
            (err, grad)
        } else {
            let err = r + d.abs() - h;
            let grad = if deriv_param == Some(self.distance_idx())
                || deriv_param == Some(self.circle.rad)
            { 1.0 } else { -dh };
            (err, grad)
        }
    }
}

impl Constraint for ConstraintC2LDistance {
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

    fn evaluate(&self, store: &mut ParamStore) {
        let (h, _) = self.center_to_line_dist(store, None);
        let r = store.get(self.circle.rad);
        let d = if h < r { r - h } else { h - r };
        store.set(self.distance_idx(), d);
    }
}

// ---------------------------------------------------------------------------
// P2CDistance: distance from a point to a circle
// ---------------------------------------------------------------------------

pub struct ConstraintP2CDistance {
    pvec: Vec<ParamIdx>,
    circle: Circle,
    pt: Point,
    scale: f64,
    pub tag: Tag,
    pub driving: bool,
}

impl ConstraintP2CDistance {
    pub fn new(pt: Point, circle: Circle, distance: ParamIdx, tag: Tag, driving: bool) -> Self {
        let mut pvec = vec![distance];
        pvec.extend(circle.params());
        pvec.push(pt.x);
        pvec.push(pt.y);
        Self { pvec, circle, pt, scale: 1.0, tag, driving }
    }

    #[inline] fn distance_idx(&self) -> ParamIdx { self.pvec[0] }

    /// Distance from center to point and its derivative.
    fn center_to_point_dist(&self, store: &ParamStore, deriv_param: Option<ParamIdx>) -> (f64, f64) {
        let ct = DeriVector2::from_point(store, self.circle.center, deriv_param);
        let p = DeriVector2::from_point(store, self.pt, deriv_param);
        let v = ct.sub(&p);
        v.length()
    }

    fn errorgrad_impl(&self, store: &ParamStore, deriv_param: Option<ParamIdx>) -> (f64, f64) {
        let (len, dlen) = self.center_to_point_dist(store, deriv_param);
        let r = store.get(self.circle.rad);
        let d = store.get(self.distance_idx());

        if len < r {
            // Point inside circle
            let err = r - d - len;
            let grad = if deriv_param == Some(self.distance_idx()) {
                -1.0
            } else if deriv_param == Some(self.circle.rad) {
                1.0
            } else {
                -dlen
            };
            (err, grad)
        } else {
            let err = r + d - len;
            let grad = if deriv_param == Some(self.distance_idx()) {
                1.0
            } else if deriv_param == Some(self.circle.rad) {
                1.0
            } else {
                -dlen
            };
            (err, grad)
        }
    }
}

impl Constraint for ConstraintP2CDistance {
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

    fn evaluate(&self, store: &mut ParamStore) {
        let (h, _) = self.center_to_point_dist(store, None);
        let r = store.get(self.circle.rad);
        let d = if h < r { r - h } else { h - r };
        store.set(self.distance_idx(), d);
    }
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

    // ===================================================================
    // Phase 4 tests
    // ===================================================================

    #[test]
    fn angle_via_point_satisfied() {
        // Two circles: tangent at (3,0). The angle between normals should be π.
        let mut store = ParamStore::new();
        let c1 = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(3.0),
        };
        let c2 = cadora_geo::Circle {
            center: Point::new(store.push(6.0), store.push(0.0)),
            rad: store.push(3.0),
        };
        let poa = Point::new(store.push(3.0), store.push(0.0));
        let ang = store.push(std::f64::consts::PI);
        let c = ConstraintAngleViaPoint::new(Box::new(c1), Box::new(c2), ang, poa, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn angle_via_point_gradient_fd() {
        let mut store = ParamStore::new();
        let c1 = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(3.0),
        };
        let c2 = cadora_geo::Circle {
            center: Point::new(store.push(5.0), store.push(1.0)),
            rad: store.push(2.0),
        };
        let poa = Point::new(store.push(2.5), store.push(0.5));
        let ang = store.push(1.0);
        let c = ConstraintAngleViaPoint::new(Box::new(c1), Box::new(c2), ang, poa, 0, true);
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
    fn angle_via_two_points_gradient_fd() {
        let mut store = ParamStore::new();
        let c1 = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(3.0),
        };
        let c2 = cadora_geo::Circle {
            center: Point::new(store.push(5.0), store.push(1.0)),
            rad: store.push(2.0),
        };
        let poa1 = Point::new(store.push(2.5), store.push(0.5));
        let poa2 = Point::new(store.push(4.0), store.push(2.0));
        let ang = store.push(0.7);
        let c = ConstraintAngleViaTwoPoints::new(Box::new(c1), Box::new(c2), ang, poa1, poa2, 0, true);
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
    fn angle_via_two_points_evaluate() {
        let mut store = ParamStore::new();
        let c1 = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(3.0),
        };
        let c2 = cadora_geo::Circle {
            center: Point::new(store.push(5.0), store.push(0.0)),
            rad: store.push(2.0),
        };
        let poa1 = Point::new(store.push(1.0), store.push(1.0));
        let poa2 = Point::new(store.push(4.0), store.push(1.0));
        let ang = store.push(0.0);
        let c = ConstraintAngleViaTwoPoints::new(Box::new(c1), Box::new(c2), ang, poa1, poa2, 0, true);
        c.evaluate(&mut store);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn snell_satisfied() {
        // Two rays and a flat boundary all through a point.
        // n1*sin(θ1) = n2*sin(θ2) for straight lines.
        let mut store = ParamStore::new();
        // Boundary: horizontal line through origin
        let boundary = cadora_geo::Line {
            p1: Point::new(store.push(-1.0), store.push(0.0)),
            p2: Point::new(store.push(1.0), store.push(0.0)),
        };
        // ray1: 45° from normal → sin(45°) = √2/2
        let ray1 = cadora_geo::Line {
            p1: Point::new(store.push(-1.0), store.push(-1.0)),
            p2: Point::new(store.push(0.0), store.push(0.0)),
        };
        // ray2: 30° from normal → sin(30°) = 0.5
        let angle2 = std::f64::consts::FRAC_PI_6;
        let ray2 = cadora_geo::Line {
            p1: Point::new(store.push(0.0), store.push(0.0)),
            p2: Point::new(store.push(angle2.sin()), store.push(angle2.cos())),
        };
        let poa = Point::new(store.push(0.0), store.push(0.0));
        // n1*sin(45) = n2*sin(30)  →  n1*0.7071 = n2*0.5  →  n1/n2 = 0.5/0.7071
        let sin45 = std::f64::consts::FRAC_PI_4.sin();
        let sin30 = 0.5;
        let n1 = store.push(sin30);
        let n2 = store.push(sin45);
        let c = ConstraintSnell::new(
            Box::new(ray1), Box::new(ray2), Box::new(boundary),
            n1, n2, poa, false, false, 0, true,
        );
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn snell_gradient_fd() {
        let mut store = ParamStore::new();
        let boundary = cadora_geo::Line {
            p1: Point::new(store.push(-1.0), store.push(0.0)),
            p2: Point::new(store.push(1.0), store.push(0.0)),
        };
        let ray1 = cadora_geo::Line {
            p1: Point::new(store.push(-1.0), store.push(-1.0)),
            p2: Point::new(store.push(0.0), store.push(0.0)),
        };
        let ray2 = cadora_geo::Line {
            p1: Point::new(store.push(0.0), store.push(0.0)),
            p2: Point::new(store.push(1.0), store.push(2.0)),
        };
        let poa = Point::new(store.push(0.0), store.push(0.0));
        let n1 = store.push(1.5);
        let n2 = store.push(1.0);
        let c = ConstraintSnell::new(
            Box::new(ray1), Box::new(ray2), Box::new(boundary),
            n1, n2, poa, false, false, 0, true,
        );
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
    fn weighted_linear_combination_satisfied() {
        // 3 poles at x=1,2,3 with weights 1,1,1 and factors [0.25, 0.5, 0.25]
        // q = (1*1*0.25 + 2*1*0.5 + 3*1*0.25) / (1*0.25 + 1*0.5 + 1*0.25)
        // q = (0.25 + 1.0 + 0.75) / 1.0 = 2.0
        let mut store = ParamStore::new();
        let q = store.push(2.0);
        let p0 = store.push(1.0);
        let p1 = store.push(2.0);
        let p2 = store.push(3.0);
        let w0 = store.push(1.0);
        let w1 = store.push(1.0);
        let w2 = store.push(1.0);
        let c = ConstraintWeightedLinearCombination::new(
            3,
            vec![q, p0, p1, p2, w0, w1, w2],
            vec![0.25, 0.5, 0.25],
            0, true,
        );
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn weighted_linear_combination_gradient_fd() {
        let mut store = ParamStore::new();
        let q = store.push(2.5);
        let p0 = store.push(1.0);
        let p1 = store.push(3.0);
        let p2 = store.push(5.0);
        let w0 = store.push(0.3);
        let w1 = store.push(0.5);
        let w2 = store.push(0.2);
        let c = ConstraintWeightedLinearCombination::new(
            3,
            vec![q, p0, p1, p2, w0, w1, w2],
            vec![0.25, 0.5, 0.25],
            0, true,
        );
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
    fn center_of_gravity_satisfied() {
        let mut store = ParamStore::new();
        // center = 0.3*1.0 + 0.7*3.0 = 0.3 + 2.1 = 2.4
        let center = store.push(2.4);
        let p0 = store.push(1.0);
        let p1 = store.push(3.0);
        let c = ConstraintCenterOfGravity::new(vec![center, p0, p1], vec![0.3, 0.7], 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn center_of_gravity_gradient_fd() {
        let mut store = ParamStore::new();
        let center = store.push(3.0);
        let p0 = store.push(1.0);
        let p1 = store.push(4.0);
        let p2 = store.push(2.0);
        let c = ConstraintCenterOfGravity::new(
            vec![center, p0, p1, p2], vec![0.2, 0.5, 0.3], 0, true,
        );
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
    fn slope_at_bspline_knot_gradient_fd() {
        // Simple test: 2 poles, factors and slopefactors are manual
        let mut store = ParamStore::new();
        let numpoles = 2;
        // pole x coords
        let px0 = store.push(0.0);
        let px1 = store.push(3.0);
        // pole y coords
        let py0 = store.push(0.0);
        let py1 = store.push(4.0);
        // weights
        let w0 = store.push(1.0);
        let w1 = store.push(1.0);
        // line p1 and p2
        let lp1x = store.push(0.0);
        let lp1y = store.push(0.0);
        let lp2x = store.push(3.0);
        let lp2y = store.push(4.0);

        let pvec = vec![px0, px1, py0, py1, w0, w1, lp1x, lp1y, lp2x, lp2y];
        let factors = vec![0.5, 0.5];
        let slopefactors = vec![-1.0, 1.0];
        let c = ConstraintSlopeAtBSplineKnot::new(pvec, factors, slopefactors, numpoles, 0, true);

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
    fn c2c_distance_outer_satisfied() {
        let mut store = ParamStore::new();
        // c1 at origin r=2, c2 at (5,0) r=1, distance = 5-2-1 = 2
        let c1 = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(2.0),
        };
        let c2 = cadora_geo::Circle {
            center: Point::new(store.push(5.0), store.push(0.0)),
            rad: store.push(1.0),
        };
        let d = store.push(2.0);
        let c = ConstraintC2CDistance::new(c1, c2, d, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn c2c_distance_gradient_fd() {
        let mut store = ParamStore::new();
        let c1 = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(2.0),
        };
        let c2 = cadora_geo::Circle {
            center: Point::new(store.push(5.0), store.push(1.0)),
            rad: store.push(1.5),
        };
        let d = store.push(1.0);
        let c = ConstraintC2CDistance::new(c1, c2, d, 0, true);
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
    fn c2l_distance_satisfied() {
        let mut store = ParamStore::new();
        // Circle at (0, 3) r=1, line along x-axis.
        // h = 3, d = h - r = 2
        let circle = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(3.0)),
            rad: store.push(1.0),
        };
        let line = cadora_geo::Line {
            p1: Point::new(store.push(-5.0), store.push(0.0)),
            p2: Point::new(store.push(5.0), store.push(0.0)),
        };
        let d = store.push(2.0);
        let c = ConstraintC2LDistance::new(circle, line, d, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn c2l_distance_gradient_fd() {
        let mut store = ParamStore::new();
        let circle = cadora_geo::Circle {
            center: Point::new(store.push(1.0), store.push(4.0)),
            rad: store.push(1.5),
        };
        let line = cadora_geo::Line {
            p1: Point::new(store.push(-2.0), store.push(1.0)),
            p2: Point::new(store.push(5.0), store.push(1.0)),
        };
        let d = store.push(0.5);
        let c = ConstraintC2LDistance::new(circle, line, d, 0, true);
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
    fn p2c_distance_satisfied() {
        let mut store = ParamStore::new();
        // Circle at origin r=3, point at (5,0), distance = 5-3 = 2
        let circle = cadora_geo::Circle {
            center: Point::new(store.push(0.0), store.push(0.0)),
            rad: store.push(3.0),
        };
        let pt = Point::new(store.push(5.0), store.push(0.0));
        let d = store.push(2.0);
        let c = ConstraintP2CDistance::new(pt, circle, d, 0, true);
        assert_abs_diff_eq!(c.error(&store), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn p2c_distance_gradient_fd() {
        let mut store = ParamStore::new();
        let circle = cadora_geo::Circle {
            center: Point::new(store.push(1.0), store.push(2.0)),
            rad: store.push(3.0),
        };
        let pt = Point::new(store.push(5.0), store.push(5.0));
        let d = store.push(1.0);
        let c = ConstraintP2CDistance::new(pt, circle, d, 0, true);
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
    fn vector_angle_helper_sanity() {
        // Angle between (1,0) and (0,1) should be π/2
        let a = vector_angle_helper(1.0, 0.0, 0.0, 1.0);
        assert_abs_diff_eq!(a, std::f64::consts::FRAC_PI_2, epsilon = 1e-12);
        // Angle between (1,0) and (1,0) should be 0
        let b = vector_angle_helper(1.0, 0.0, 1.0, 0.0);
        assert_abs_diff_eq!(b, 0.0, epsilon = 1e-12);
    }
}
