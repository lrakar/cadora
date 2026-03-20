//! # cadora-constraints
//!
//! All constraint types with error + gradient computation.
//! Port of planegcs `Constraints.h` / `Constraints.cpp`.

use cadora_core::{ParamIdx, ParamStore, Tag};
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
}
