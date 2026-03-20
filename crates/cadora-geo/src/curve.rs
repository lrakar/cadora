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

    /// Surface normal at curve parameter `u`.
    /// Default: evaluates curve at `u` to get a point, then calls `normal_at_point`.
    /// The point coordinates are NOT differentiated — only curve params carry derivatives.
    fn normal_at_param(
        &self,
        store: &ParamStore,
        u: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let pt_dv = self.value(store, u, 0.0, None);
        // Clone store and push temporary point coords so they won't match any deriv_param.
        let mut tmp = store.clone();
        let px = tmp.push(pt_dv.x);
        let py = tmp.push(pt_dv.y);
        let at = Point::new(px, py);
        self.normal_at_point(&tmp, at, deriv_param)
    }

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

// ---------------------------------------------------------------------------
// Line implementation
// ---------------------------------------------------------------------------

impl Curve for Line {
    /// `p1 + (p2 - p1) * u`  — line parametrised so u=0 → p1, u=1 → p2.
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let p1v = DeriVector2::from_point(store, self.p1, deriv_param);
        let p2v = DeriVector2::from_point(store, self.p2, deriv_param);
        let line_vec = p2v.sub(&p1v);
        p1v.add(&line_vec.mul_scalar(u, du))
    }

    /// Normal = (p2 - p1).rotate90ccw  — independent of `at`.
    fn normal_at_point(
        &self,
        store: &ParamStore,
        _at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let p1v = DeriVector2::from_point(store, self.p1, deriv_param);
        let p2v = DeriVector2::from_point(store, self.p2, deriv_param);
        p2v.sub(&p1v).rotate90ccw()
    }

    fn params(&self) -> Vec<ParamIdx> {
        vec![self.p1.x, self.p1.y, self.p2.x, self.p2.y]
    }
}

// ---------------------------------------------------------------------------
// Circle implementation
// ---------------------------------------------------------------------------

impl Curve for Circle {
    /// `center + r*(cos(u), sin(u))`.
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let cv = DeriVector2::from_point(store, self.center, deriv_param);
        let (r, dr) = DeriVector2::from_scalar(store, self.rad, deriv_param);
        // basis vectors scaled by radius
        let ex = DeriVector2::new(r, dr, 0.0, 0.0);
        let ey = ex.rotate90ccw();
        let co = u.cos();
        let dco = du * (-u.sin());
        let si = u.sin();
        let dsi = du * u.cos();
        cv.add(&ex.mul_scalar(co, dco).add(&ey.mul_scalar(si, dsi)))
    }

    /// Normal points inward: `center - p`.
    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let cv = DeriVector2::from_point(store, self.center, deriv_param);
        let pv = DeriVector2::from_point(store, at, deriv_param);
        cv.sub(&pv)
    }

    fn params(&self) -> Vec<ParamIdx> {
        vec![self.center.x, self.center.y, self.rad]
    }
}

// ---------------------------------------------------------------------------
// Arc implementation (inherits Circle's value/normal logic)
// ---------------------------------------------------------------------------

impl Curve for Arc {
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        // Arc uses the same parametric formula as Circle
        let circle = Circle {
            center: self.center,
            rad: self.rad,
        };
        circle.value(store, u, du, deriv_param)
    }

    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let circle = Circle {
            center: self.center,
            rad: self.rad,
        };
        circle.normal_at_point(store, at, deriv_param)
    }

    fn params(&self) -> Vec<ParamIdx> {
        vec![
            self.center.x,
            self.center.y,
            self.rad,
            self.start.x,
            self.start.y,
            self.end.x,
            self.end.y,
            self.start_angle,
            self.end_angle,
        ]
    }
}

// ---------------------------------------------------------------------------
// Ellipse helpers
// ---------------------------------------------------------------------------

impl Ellipse {
    /// Compute major radius from pre-built DeriVector2 values.
    /// C++ hack: uses vector-length formula `sqrt(b² + cf²)` where cf = |focus1 - center|.
    pub fn get_rad_maj_from_dv(
        &self,
        center: &DeriVector2,
        f1: &DeriVector2,
        b: f64,
        db: f64,
    ) -> (f64, f64) {
        let (cf, dcf) = f1.sub(center).length();
        // hack vector: (b, cf) — its length = sqrt(b² + cf²) = a
        let hack = DeriVector2::new(b, db, cf, dcf);
        hack.length()
    }

    /// Compute major radius from the param store.
    pub fn get_rad_maj(
        &self,
        store: &ParamStore,
        deriv_param: Option<ParamIdx>,
    ) -> (f64, f64) {
        let c = DeriVector2::from_point(store, self.center, deriv_param);
        let f1 = DeriVector2::from_point(store, self.focus1, deriv_param);
        let (b, db) = DeriVector2::from_scalar(store, self.radmin, deriv_param);
        self.get_rad_maj_from_dv(&c, &f1, b, db)
    }
}

// ---------------------------------------------------------------------------
// Ellipse Curve implementation
// ---------------------------------------------------------------------------

impl Curve for Ellipse {
    /// `center + a_vec*cos(u) + b_vec*sin(u)` where a_vec,b_vec are major/minor axis vectors.
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let c = DeriVector2::from_point(store, self.center, deriv_param);
        let f1 = DeriVector2::from_point(store, self.focus1, deriv_param);

        let emaj = f1.sub(&c).normalized();
        let emin = emaj.rotate90ccw();
        let (b, db) = DeriVector2::from_scalar(store, self.radmin, deriv_param);
        let (a, da) = self.get_rad_maj_from_dv(&c, &f1, b, db);

        let a_vec = emaj.mul_scalar(a, da);
        let b_vec = emin.mul_scalar(b, db);

        let co = u.cos();
        let dco = -u.sin() * du;
        let si = u.sin();
        let dsi = u.cos() * du;

        a_vec.mul_scalar(co, dco).add(&b_vec.mul_scalar(si, dsi)).add(&c)
    }

    /// Normal = normalized(f1-p) + normalized(f2-p), where f2 = 2*center - f1.
    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let cv = DeriVector2::from_point(store, self.center, deriv_param);
        let f1v = DeriVector2::from_point(store, self.focus1, deriv_param);
        let pv = DeriVector2::from_point(store, at, deriv_param);

        let f2v = cv.lin_combi(2.0, &f1v, -1.0);
        let pf1 = f1v.sub(&pv);
        let pf2 = f2v.sub(&pv);

        pf1.normalized().add(&pf2.normalized())
    }

    fn params(&self) -> Vec<ParamIdx> {
        vec![self.center.x, self.center.y, self.focus1.x, self.focus1.y, self.radmin]
    }
}

// ---------------------------------------------------------------------------
// ArcOfEllipse Curve implementation
// ---------------------------------------------------------------------------

impl Curve for ArcOfEllipse {
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        self.ellipse.value(store, u, du, deriv_param)
    }

    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        self.ellipse.normal_at_point(store, at, deriv_param)
    }

    fn params(&self) -> Vec<ParamIdx> {
        let mut p = self.ellipse.params();
        p.extend_from_slice(&[
            self.start.x, self.start.y,
            self.end.x, self.end.y,
            self.start_angle, self.end_angle,
        ]);
        p
    }
}

// ---------------------------------------------------------------------------
// Hyperbola helpers
// ---------------------------------------------------------------------------

impl Hyperbola {
    /// `a = sqrt(cf² - b²)` where cf = |focus1 - center|.
    pub fn get_rad_maj_from_dv(
        &self,
        center: &DeriVector2,
        f1: &DeriVector2,
        b: f64,
        db: f64,
    ) -> (f64, f64) {
        let (cf, dcf) = f1.sub(center).length();
        let a = (cf * cf - b * b).sqrt();
        let da = (dcf * cf - db * b) / a;
        (a, da)
    }

    pub fn get_rad_maj(
        &self,
        store: &ParamStore,
        deriv_param: Option<ParamIdx>,
    ) -> (f64, f64) {
        let c = DeriVector2::from_point(store, self.center, deriv_param);
        let f1 = DeriVector2::from_point(store, self.focus1, deriv_param);
        let (b, db) = DeriVector2::from_scalar(store, self.radmin, deriv_param);
        self.get_rad_maj_from_dv(&c, &f1, b, db)
    }
}

// ---------------------------------------------------------------------------
// Hyperbola Curve implementation
// ---------------------------------------------------------------------------

impl Curve for Hyperbola {
    /// `center + a_vec*cosh(u) + b_vec*sinh(u)`.
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let c = DeriVector2::from_point(store, self.center, deriv_param);
        let f1 = DeriVector2::from_point(store, self.focus1, deriv_param);

        let emaj = f1.sub(&c).normalized();
        let emin = emaj.rotate90ccw();
        let (b, db) = DeriVector2::from_scalar(store, self.radmin, deriv_param);
        let (a, da) = self.get_rad_maj_from_dv(&c, &f1, b, db);

        let a_vec = emaj.mul_scalar(a, da);
        let b_vec = emin.mul_scalar(b, db);

        let co = u.cosh();
        let dco = u.sinh() * du;
        let si = u.sinh();
        let dsi = u.cosh() * du;

        a_vec.mul_scalar(co, dco).add(&b_vec.mul_scalar(si, dsi)).add(&c)
    }

    /// Normal: normalized(-pf1) + normalized(pf2), note the negation vs ellipse.
    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let cv = DeriVector2::from_point(store, self.center, deriv_param);
        let f1v = DeriVector2::from_point(store, self.focus1, deriv_param);
        let pv = DeriVector2::from_point(store, at, deriv_param);

        let f2v = cv.lin_combi(2.0, &f1v, -1.0);
        let pf1 = f1v.sub(&pv).neg(); // differs from ellipse: negated
        let pf2 = f2v.sub(&pv);

        pf1.normalized().add(&pf2.normalized())
    }

    fn params(&self) -> Vec<ParamIdx> {
        vec![self.center.x, self.center.y, self.focus1.x, self.focus1.y, self.radmin]
    }
}

// ---------------------------------------------------------------------------
// ArcOfHyperbola Curve implementation
// ---------------------------------------------------------------------------

impl Curve for ArcOfHyperbola {
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        self.hyperbola.value(store, u, du, deriv_param)
    }

    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        self.hyperbola.normal_at_point(store, at, deriv_param)
    }

    fn params(&self) -> Vec<ParamIdx> {
        let mut p = self.hyperbola.params();
        p.extend_from_slice(&[
            self.start.x, self.start.y,
            self.end.x, self.end.y,
            self.start_angle, self.end_angle,
        ]);
        p
    }
}

// ---------------------------------------------------------------------------
// Parabola Curve implementation
// ---------------------------------------------------------------------------

impl Curve for Parabola {
    /// `vertex + u²/(4f) * xdir + u * ydir` where f = |focus - vertex|.
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let c = DeriVector2::from_point(store, self.vertex, deriv_param);
        let f1 = DeriVector2::from_point(store, self.focus1, deriv_param);
        let fv = f1.sub(&c);
        let (f, df) = fv.length();
        let xdir = fv.normalized();
        let ydir = xdir.rotate90ccw();

        // u²/(4f) along xdir  +  u along ydir
        let dirx = xdir
            .mul_scalar(u, du)
            .mul_scalar(u, du)
            .div_scalar(4.0 * f, 4.0 * df);
        let diry = ydir.mul_scalar(u, du);

        c.add(&dirx.add(&diry))
    }

    /// Normal: normalized(vertex-focus) - normalized(focus-p).
    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let cv = DeriVector2::from_point(store, self.vertex, deriv_param);
        let f1v = DeriVector2::from_point(store, self.focus1, deriv_param);
        let pv = DeriVector2::from_point(store, at, deriv_param);

        cv.sub(&f1v).normalized().sub(&f1v.sub(&pv).normalized())
    }

    fn params(&self) -> Vec<ParamIdx> {
        vec![self.vertex.x, self.vertex.y, self.focus1.x, self.focus1.y]
    }
}

// ---------------------------------------------------------------------------
// ArcOfParabola Curve implementation
// ---------------------------------------------------------------------------

impl Curve for ArcOfParabola {
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        du: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        self.parabola.value(store, u, du, deriv_param)
    }

    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        self.parabola.normal_at_point(store, at, deriv_param)
    }

    fn params(&self) -> Vec<ParamIdx> {
        let mut p = self.parabola.params();
        p.extend_from_slice(&[
            self.start.x, self.start.y,
            self.end.x, self.end.y,
            self.start_angle, self.end_angle,
        ]);
        p
    }
}

// ---------------------------------------------------------------------------
// BSpline helpers
// ---------------------------------------------------------------------------

impl BSpline {
    /// de Boor evaluation: compute `spline(x)` from coefficient vector `d` at knot span `k`.
    /// `d` is mutated in-place (de Boor triangular table).
    pub fn spline_value(x: f64, k: usize, p: usize, d: &mut [f64], flat_knots: &[f64]) -> f64 {
        for r in 1..=p {
            let mut j = p;
            while j >= r {
                let alpha = (x - flat_knots[j + k - p])
                    / (flat_knots[j + 1 + k - r] - flat_knots[j + k - p]);
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
                if j == r {
                    break;
                }
                j -= 1;
            }
        }
        if p < d.len() { d[p] } else { 0.0 }
    }

    /// Get the basis function value B_i(x) at knot span k.
    pub fn get_lin_comb_factor(&self, x: f64, k: usize, i: usize, p: usize) -> f64 {
        let mut d = vec![0.0; p + 1];
        let idx = i as isize + p as isize - k as isize;
        if idx < 0 || idx > p as isize {
            return 0.0;
        }
        d[idx as usize] = 1.0;
        Self::spline_value(x, k, p, &mut d, &self.flat_knots)
    }

    /// Setup the flattened knot vector from knots + multiplicities.
    pub fn setup_flat_knots(&mut self, store: &ParamStore) {
        self.flat_knots.clear();
        for (i, &m) in self.mult.iter().enumerate() {
            let kval = store.get(self.knots[i]);
            for _ in 0..m {
                self.flat_knots.push(kval);
            }
        }
        if self.periodic {
            let period = store.get(*self.knots.last().unwrap()) - store.get(self.knots[0]);
            let c = (self.degree + 1).saturating_sub(self.mult[0]);
            let n = self.flat_knots.len();
            let last_m = *self.mult.last().unwrap();
            let first_m = self.mult[0];

            let front_new: Vec<f64> = self.flat_knots[n - last_m - c..n - last_m].to_vec();
            let back_new: Vec<f64> = self.flat_knots[first_m..first_m + c].to_vec();

            self.flat_knots.extend_from_slice(&back_new);
            let mut with_front = front_new;
            with_front.append(&mut self.flat_knots);
            self.flat_knots = with_front;

            for i in 0..c {
                self.flat_knots[i] -= period;
                let len = self.flat_knots.len();
                self.flat_knots[len - 1 - i] += period;
            }
        }
    }

    /// Evaluate homogeneous coords (xw, yw, w) and their derivatives.
    pub fn value_homogeneous(
        &self,
        store: &ParamStore,
        u: f64,
    ) -> (f64, f64, f64, f64, f64, f64) {
        let startpole = self.find_start_pole(store, u);
        let np = self.degree + 1;

        let polexat = |i: usize| store.get(self.poles[(startpole + i) % self.poles.len()].x);
        let poleyat = |i: usize| store.get(self.poles[(startpole + i) % self.poles.len()].y);
        let weightat = |i: usize| store.get(self.weights[(startpole + i) % self.weights.len()]);

        let mut d = vec![0.0; np];

        // xw
        for i in 0..np { d[i] = polexat(i) * weightat(i); }
        let xw = Self::spline_value(u, startpole + self.degree, self.degree, &mut d, &self.flat_knots);

        // yw
        for i in 0..np { d[i] = poleyat(i) * weightat(i); }
        let yw = Self::spline_value(u, startpole + self.degree, self.degree, &mut d, &self.flat_knots);

        // w
        for i in 0..np { d[i] = weightat(i); }
        let w = Self::spline_value(u, startpole + self.degree, self.degree, &mut d, &self.flat_knots);

        // slopes
        let mut sd = vec![0.0; np - 1];
        let k = startpole + self.degree;

        for i in 1..np {
            sd[i - 1] = (polexat(i) * weightat(i) - polexat(i - 1) * weightat(i - 1))
                / (self.flat_knots[startpole + i + self.degree] - self.flat_knots[startpole + i]);
        }
        let dxwdu = self.degree as f64 * Self::spline_value(u, k, self.degree - 1, &mut sd, &self.flat_knots);

        for i in 1..np {
            sd[i - 1] = (poleyat(i) * weightat(i) - poleyat(i - 1) * weightat(i - 1))
                / (self.flat_knots[startpole + i + self.degree] - self.flat_knots[startpole + i]);
        }
        let dywdu = self.degree as f64 * Self::spline_value(u, k, self.degree - 1, &mut sd, &self.flat_knots);

        for i in 1..np {
            sd[i - 1] = (weightat(i) - weightat(i - 1))
                / (self.flat_knots[startpole + i + self.degree] - self.flat_knots[startpole + i]);
        }
        let dwdu = self.degree as f64 * Self::spline_value(u, k, self.degree - 1, &mut sd, &self.flat_knots);

        (xw, yw, w, dxwdu, dywdu, dwdu)
    }

    fn find_start_pole(&self, store: &ParamStore, u: f64) -> usize {
        let mut startpole = 0;
        for j in 1..self.mult.len() {
            if store.get(self.knots[j]) <= u {
                startpole += self.mult[j];
            } else {
                break;
            }
        }
        if !self.periodic && startpole >= self.poles.len() {
            startpole = self.poles.len() - self.degree - 1;
        }
        startpole
    }
}

// ---------------------------------------------------------------------------
// BSpline Curve implementation (value-only — no derivative propagation yet,
// matching C++ which ignores du/derivparam in BSpline::Value)
// ---------------------------------------------------------------------------

impl Curve for BSpline {
    fn value(
        &self,
        store: &ParamStore,
        u: f64,
        _du: f64,
        _deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let (xw, yw, w, dxwdu, dywdu, dwdu) = self.value_homogeneous(store, u);
        DeriVector2::new(
            xw / w,
            (w * dxwdu - dwdu * xw) / (w * w),
            yw / w,
            (w * dywdu - dwdu * yw) / (w * w),
        )
    }

    fn normal_at_point(
        &self,
        store: &ParamStore,
        at: Point,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        // For non-periodic splines with endpoint multiplicity > degree,
        // check if the point is at start or end.
        if !self.mult.is_empty()
            && self.mult[0] > self.degree
            && *self.mult.last().unwrap() > self.degree
        {
            let at_x = store.get(at.x);
            let at_y = store.get(at.y);
            let sx = store.get(self.start.x);
            let sy = store.get(self.start.y);
            if at_x == sx && at_y == sy {
                let endpt = DeriVector2::from_point(store, self.poles[1], deriv_param);
                let spt = DeriVector2::from_point(store, self.poles[0], deriv_param);
                return endpt.sub(&spt).rotate90ccw();
            }
            let ex = store.get(self.end.x);
            let ey = store.get(self.end.y);
            if at_x == ex && at_y == ey {
                let n = self.poles.len();
                let endpt = DeriVector2::from_point(store, self.poles[n - 1], deriv_param);
                let spt = DeriVector2::from_point(store, self.poles[n - 2], deriv_param);
                return endpt.sub(&spt).rotate90ccw();
            }
        }
        DeriVector2::zero()
    }

    /// Full CalculateNormal port from FreeCAD Geo.cpp.
    ///
    /// Computes the tangent at parameter `u` using NURBS homogeneous coordinates,
    /// then differentiates w.r.t. `deriv_param` (pole, weight, or u itself).
    /// When `deriv_param` matches the curve parameter, computes the second derivative (curvature).
    /// Returns `tangent.rotate90ccw()`.
    fn normal_at_param(
        &self,
        store: &ParamStore,
        u: f64,
        deriv_param: Option<ParamIdx>,
    ) -> DeriVector2 {
        let startpole = self.find_start_pole(store, u);
        let np = self.degree + 1;

        let polexat = |i: usize| self.poles[(startpole + i) % self.poles.len()].x;
        let poleyat = |i: usize| self.poles[(startpole + i) % self.poles.len()].y;
        let weight_idx = |i: usize| self.weights[(startpole + i) % self.weights.len()];

        let px = |i: usize| store.get(polexat(i));
        let py = |i: usize| store.get(poleyat(i));
        let wt = |i: usize| store.get(weight_idx(i));

        let (xsum, ysum, wsum, xslopesum, yslopesum, wslopesum) =
            self.value_homogeneous(store, u);

        // Tangent = (w * d(xw)/du - dw/du * xw,  w * d(yw)/du - dw/du * yw)
        let mut result = DeriVector2::new(
            wsum * xslopesum - wslopesum * xsum,
            0.0,
            wsum * yslopesum - wslopesum * ysum,
            0.0,
        );

        if let Some(dparam) = deriv_param {
            // Check if deriv_param matches a pole or weight in the active range
            let mut found_pole_or_weight = false;
            for i in 0..np {
                let is_px = dparam == polexat(i);
                let is_py = dparam == poleyat(i);
                let is_w = dparam == weight_idx(i);
                if !is_px && !is_py && !is_w {
                    continue;
                }

                // Compute basis function factor and slope factor for pole i
                let mut d = vec![0.0; np];
                d[i] = 1.0;
                let factor = Self::spline_value(
                    u, startpole + self.degree, self.degree, &mut d, &self.flat_knots,
                );

                let mut sd = vec![0.0; np - 1];
                if i > 0 {
                    sd[i - 1] = 1.0
                        / (self.flat_knots[startpole + i + self.degree]
                            - self.flat_knots[startpole + i]);
                }
                if i < np - 1 {
                    sd[i] = -1.0
                        / (self.flat_knots[startpole + i + 1 + self.degree]
                            - self.flat_knots[startpole + i + 1]);
                }
                let slopefactor = Self::spline_value(
                    u, startpole + self.degree, self.degree - 1, &mut sd, &self.flat_knots,
                );

                if is_px {
                    result.dx = self.degree as f64
                        * wt(i) * (wsum * slopefactor - wslopesum * factor);
                } else if is_py {
                    result.dy = self.degree as f64
                        * wt(i) * (wsum * slopefactor - wslopesum * factor);
                } else {
                    // weight derivative:
                    // d/d(w_i) T_x = factor*(xslopesum - wslopesum*px_i) + degree*slopefactor*(wsum*px_i - xsum)
                    let deg = self.degree as f64;
                    result.dx = factor * (xslopesum - wslopesum * px(i))
                        + deg * slopefactor * (wsum * px(i) - xsum);
                    result.dy = factor * (yslopesum - wslopesum * py(i))
                        + deg * slopefactor * (wsum * py(i) - ysum);
                }
                found_pole_or_weight = true;
                break;
            }

            // If deriv_param is NOT a pole/weight, check if it's the curve parameter
            // In that case compute second derivative (curvature direction)
            if !found_pole_or_weight {
                // Second derivative via double differences
                let k = startpole + self.degree;

                // d²w/du²
                let mut sd = vec![0.0; np - 1];
                for i in 1..np {
                    sd[i - 1] = (wt(i) - wt(i - 1))
                        / (self.flat_knots[startpole + i + self.degree]
                            - self.flat_knots[startpole + i]);
                }
                let mut ssd = vec![0.0; np.saturating_sub(2)];
                for i in 1..np.saturating_sub(1) {
                    ssd[i - 1] = (sd[i] - sd[i - 1])
                        / (self.flat_knots[startpole + i + self.degree]
                            - self.flat_knots[startpole + i]);
                }
                let wslopeslopesum = if self.degree >= 2 {
                    self.degree as f64
                        * (self.degree as f64 - 1.0)
                        * Self::spline_value(u, k, self.degree - 2, &mut ssd, &self.flat_knots)
                } else {
                    0.0
                };

                // d²(xw)/du²
                for i in 1..np {
                    sd[i - 1] = (px(i) * wt(i) - px(i - 1) * wt(i - 1))
                        / (self.flat_knots[startpole + i + self.degree]
                            - self.flat_knots[startpole + i]);
                }
                let mut ssd = vec![0.0; np.saturating_sub(2)];
                for i in 1..np.saturating_sub(1) {
                    ssd[i - 1] = (sd[i] - sd[i - 1])
                        / (self.flat_knots[startpole + i + self.degree]
                            - self.flat_knots[startpole + i]);
                }
                let xslopeslopesum = if self.degree >= 2 {
                    self.degree as f64
                        * (self.degree as f64 - 1.0)
                        * Self::spline_value(u, k, self.degree - 2, &mut ssd, &self.flat_knots)
                } else {
                    0.0
                };

                // d²(yw)/du²
                for i in 1..np {
                    sd[i - 1] = (py(i) * wt(i) - py(i - 1) * wt(i - 1))
                        / (self.flat_knots[startpole + i + self.degree]
                            - self.flat_knots[startpole + i]);
                }
                let mut ssd = vec![0.0; np.saturating_sub(2)];
                for i in 1..np.saturating_sub(1) {
                    ssd[i - 1] = (sd[i] - sd[i - 1])
                        / (self.flat_knots[startpole + i + self.degree]
                            - self.flat_knots[startpole + i]);
                }
                let yslopeslopesum = if self.degree >= 2 {
                    self.degree as f64
                        * (self.degree as f64 - 1.0)
                        * Self::spline_value(u, k, self.degree - 2, &mut ssd, &self.flat_knots)
                } else {
                    0.0
                };

                result.dx = wsum * xslopeslopesum - wslopeslopesum * xsum;
                result.dy = wsum * yslopeslopesum - wslopeslopesum * ysum;
            }
        }

        result.rotate90ccw()
    }

    fn params(&self) -> Vec<ParamIdx> {
        let mut p = Vec::new();
        for pole in &self.poles {
            p.push(pole.x);
            p.push(pole.y);
        }
        p.extend_from_slice(&self.weights);
        p.extend_from_slice(&self.knots);
        p.push(self.start.x);
        p.push(self.start.y);
        p.push(self.end.x);
        p.push(self.end.y);
        p
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f64::consts::PI;

    // Helper: create a line from (0,0) to (3,4)
    fn make_line() -> (ParamStore, Line) {
        let mut s = ParamStore::new();
        let p1 = Point::new(s.push(0.0), s.push(0.0));
        let p2 = Point::new(s.push(3.0), s.push(4.0));
        (s, Line { p1, p2 })
    }

    #[test]
    fn line_value_at_endpoints() {
        let (s, line) = make_line();
        let v0 = line.value(&s, 0.0, 0.0, None);
        assert_abs_diff_eq!(v0.x, 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(v0.y, 0.0, epsilon = 1e-15);
        let v1 = line.value(&s, 1.0, 0.0, None);
        assert_abs_diff_eq!(v1.x, 3.0, epsilon = 1e-15);
        assert_abs_diff_eq!(v1.y, 4.0, epsilon = 1e-15);
    }

    #[test]
    fn line_value_midpoint() {
        let (s, line) = make_line();
        let v = line.value(&s, 0.5, 0.0, None);
        assert_abs_diff_eq!(v.x, 1.5, epsilon = 1e-15);
        assert_abs_diff_eq!(v.y, 2.0, epsilon = 1e-15);
    }

    #[test]
    fn line_normal() {
        let (s, line) = make_line();
        let dummy = Point::new(line.p1.x, line.p1.y); // ignored for line
        let n = line.normal_at_point(&s, dummy, None);
        // (3,4).rotate90ccw = (-4, 3)
        assert_abs_diff_eq!(n.x, -4.0, epsilon = 1e-15);
        assert_abs_diff_eq!(n.y, 3.0, epsilon = 1e-15);
    }

    #[test]
    fn line_value_derivative_fd() {
        let (mut s, line) = make_line();
        let dp = line.p2.x; // differentiate w.r.t. p2.x
        let v = line.value(&s, 0.5, 0.0, Some(dp));
        let eps = 1e-8;
        s.set(dp, 3.0 + eps);
        let vp = line.value(&s, 0.5, 0.0, None);
        assert_abs_diff_eq!(v.dx, (vp.x - v.x) / eps, epsilon = 1e-6);
        assert_abs_diff_eq!(v.dy, (vp.y - v.y) / eps, epsilon = 1e-6);
    }

    // Helper: circle centered at (1,2) with radius 3
    fn make_circle() -> (ParamStore, Circle) {
        let mut s = ParamStore::new();
        let center = Point::new(s.push(1.0), s.push(2.0));
        let rad = s.push(3.0);
        (s, Circle { center, rad })
    }

    #[test]
    fn circle_value_at_0() {
        let (s, circ) = make_circle();
        let v = circ.value(&s, 0.0, 0.0, None);
        // center + (r,0) = (4, 2)
        assert_abs_diff_eq!(v.x, 4.0, epsilon = 1e-15);
        assert_abs_diff_eq!(v.y, 2.0, epsilon = 1e-15);
    }

    #[test]
    fn circle_value_at_pi_half() {
        let (s, circ) = make_circle();
        let v = circ.value(&s, PI / 2.0, 0.0, None);
        // center + (0, r) = (1, 5)
        assert_abs_diff_eq!(v.x, 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v.y, 5.0, epsilon = 1e-12);
    }

    #[test]
    fn circle_normal_points_inward() {
        let (s, circ) = make_circle();
        // Normal at point (4,2) should point toward center: (1-4, 2-2) = (-3, 0)
        let mut s2 = s.clone();
        let at = Point::new(s2.push(4.0), s2.push(2.0));
        let n = circ.normal_at_point(&s2, at, None);
        assert_abs_diff_eq!(n.x, -3.0, epsilon = 1e-15);
        assert_abs_diff_eq!(n.y, 0.0, epsilon = 1e-15);
    }

    #[test]
    fn circle_value_derivative_fd() {
        let (mut s, circ) = make_circle();
        let dp = circ.rad;
        let v = circ.value(&s, 1.0, 0.0, Some(dp));
        let eps = 1e-8;
        s.set(dp, 3.0 + eps);
        let vp = circ.value(&s, 1.0, 0.0, None);
        assert_abs_diff_eq!(v.dx, (vp.x - v.x) / eps, epsilon = 1e-6);
        assert_abs_diff_eq!(v.dy, (vp.y - v.y) / eps, epsilon = 1e-6);
    }

    // Helper: ellipse centered at origin, focus at (3,0), radmin=4 → rad_maj=5
    fn make_ellipse() -> (ParamStore, Ellipse) {
        let mut s = ParamStore::new();
        let center = Point::new(s.push(0.0), s.push(0.0));
        let focus1 = Point::new(s.push(3.0), s.push(0.0));
        let radmin = s.push(4.0);
        (s, Ellipse { center, focus1, radmin })
    }

    #[test]
    fn ellipse_rad_maj() {
        let (s, ell) = make_ellipse();
        let (a, _) = ell.get_rad_maj(&s, None);
        // a = sqrt(3² + 4²) = 5
        assert_abs_diff_eq!(a, 5.0, epsilon = 1e-12);
    }

    #[test]
    fn ellipse_value_at_0() {
        let (s, ell) = make_ellipse();
        let v = ell.value(&s, 0.0, 0.0, None);
        // center + a*cos(0)*emaj = (5, 0)
        assert_abs_diff_eq!(v.x, 5.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn ellipse_value_at_pi_half() {
        let (s, ell) = make_ellipse();
        let v = ell.value(&s, PI / 2.0, 0.0, None);
        // center + b*sin(pi/2)*emin = (0, 4)
        assert_abs_diff_eq!(v.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v.y, 4.0, epsilon = 1e-12);
    }

    #[test]
    fn ellipse_value_derivative_fd() {
        let (mut s, ell) = make_ellipse();
        let dp = ell.radmin;
        let u = 1.0;
        let v = ell.value(&s, u, 0.0, Some(dp));
        let eps = 1e-8;
        s.set(dp, 4.0 + eps);
        let vp = ell.value(&s, u, 0.0, None);
        assert_abs_diff_eq!(v.dx, (vp.x - v.x) / eps, epsilon = 1e-5);
        assert_abs_diff_eq!(v.dy, (vp.y - v.y) / eps, epsilon = 1e-5);
    }

    #[test]
    fn parabola_value_at_0() {
        let mut s = ParamStore::new();
        let vertex = Point::new(s.push(0.0), s.push(0.0));
        let focus1 = Point::new(s.push(1.0), s.push(0.0));
        let para = Parabola { vertex, focus1 };
        let v = para.value(&s, 0.0, 0.0, None);
        // u=0 → vertex
        assert_abs_diff_eq!(v.x, 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(v.y, 0.0, epsilon = 1e-15);
    }

    #[test]
    fn parabola_value_derivative_fd() {
        let mut s = ParamStore::new();
        let vertex = Point::new(s.push(0.0), s.push(0.0));
        let focus1 = Point::new(s.push(1.0), s.push(0.0));
        let para = Parabola { vertex, focus1 };
        let dp = focus1.x;
        let u = 2.0;
        let v = para.value(&s, u, 0.0, Some(dp));
        let eps = 1e-8;
        s.set(dp, 1.0 + eps);
        let vp = para.value(&s, u, 0.0, None);
        assert_abs_diff_eq!(v.dx, (vp.x - v.x) / eps, epsilon = 1e-5);
        assert_abs_diff_eq!(v.dy, (vp.y - v.y) / eps, epsilon = 1e-5);
    }

    #[test]
    fn hyperbola_value_at_0() {
        let mut s = ParamStore::new();
        let center = Point::new(s.push(0.0), s.push(0.0));
        let focus1 = Point::new(s.push(5.0), s.push(0.0));
        let radmin = s.push(3.0); // a = sqrt(25-9) = 4
        let hyp = Hyperbola { center, focus1, radmin };
        let v = hyp.value(&s, 0.0, 0.0, None);
        // cosh(0)=1, sinh(0)=0 → center + a*emaj = (4, 0)
        assert_abs_diff_eq!(v.x, 4.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn hyperbola_value_derivative_fd() {
        let mut s = ParamStore::new();
        let center = Point::new(s.push(0.0), s.push(0.0));
        let focus1 = Point::new(s.push(5.0), s.push(0.0));
        let radmin = s.push(3.0);
        let hyp = Hyperbola { center, focus1, radmin };
        let dp = radmin;
        let u = 0.5;
        let v = hyp.value(&s, u, 0.0, Some(dp));
        let eps = 1e-8;
        s.set(dp, 3.0 + eps);
        let vp = hyp.value(&s, u, 0.0, None);
        assert_abs_diff_eq!(v.dx, (vp.x - v.x) / eps, epsilon = 1e-5);
        assert_abs_diff_eq!(v.dy, (vp.y - v.y) / eps, epsilon = 1e-5);
    }

    // -----------------------------------------------------------------------
    // BSpline tests
    // -----------------------------------------------------------------------

    /// Create a cubic Bezier curve (degree 3, 4 control points, all weights=1).
    /// Poles: (0,0), (1,2), (3,2), (4,0), knots: [0, 1], mult: [4, 4]
    fn make_cubic_bezier() -> (ParamStore, BSpline) {
        let mut s = ParamStore::new();
        let poles = vec![
            Point::new(s.push(0.0), s.push(0.0)),
            Point::new(s.push(1.0), s.push(2.0)),
            Point::new(s.push(3.0), s.push(2.0)),
            Point::new(s.push(4.0), s.push(0.0)),
        ];
        let weights: Vec<ParamIdx> = (0..4).map(|_| s.push(1.0)).collect();
        let knots = vec![s.push(0.0), s.push(1.0)];
        let mult = vec![4, 4];
        let start = poles[0];
        let end = poles[3];
        let mut bsp = BSpline {
            start, end, poles, weights, knots, mult,
            degree: 3, periodic: false, flat_knots: vec![],
        };
        bsp.setup_flat_knots(&s);
        (s, bsp)
    }

    #[test]
    fn bspline_cubic_bezier_endpoints() {
        let (s, bsp) = make_cubic_bezier();
        let v0 = bsp.value(&s, 0.0, 0.0, None);
        assert_abs_diff_eq!(v0.x, 0.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v0.y, 0.0, epsilon = 1e-12);
        let v1 = bsp.value(&s, 1.0, 0.0, None);
        assert_abs_diff_eq!(v1.x, 4.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v1.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn bspline_cubic_bezier_midpoint() {
        let (s, bsp) = make_cubic_bezier();
        // For uniform-weight cubic Bezier: B(0.5) = 0.125*P0 + 0.375*P1 + 0.375*P2 + 0.125*P3
        // x = 0.125*0 + 0.375*1 + 0.375*3 + 0.125*4 = 0 + 0.375 + 1.125 + 0.5 = 2.0
        // y = 0.125*0 + 0.375*2 + 0.375*2 + 0.125*0 = 0 + 0.75 + 0.75 + 0 = 1.5
        let v = bsp.value(&s, 0.5, 0.0, None);
        assert_abs_diff_eq!(v.x, 2.0, epsilon = 1e-12);
        assert_abs_diff_eq!(v.y, 1.5, epsilon = 1e-12);
    }

    #[test]
    fn bspline_nurbs_weighted() {
        // Raise weight of P1 to 2.0 — should pull curve toward P1
        let (mut s, bsp) = make_cubic_bezier();
        s.set(bsp.weights[1], 2.0);
        let v_uniform = DeriVector2::new(2.0, 0.0, 1.5, 0.0); // from test above
        let v_weighted = bsp.value(&s, 0.5, 0.0, None);
        // With higher weight on P1=(1,2), curve should shift toward P1
        assert!(v_weighted.x < v_uniform.x, "weighted x should be pulled left");
        assert!(v_weighted.y > v_uniform.y, "weighted y should be pulled up");
    }

    #[test]
    fn bspline_normal_at_param_tangent_direction() {
        let (s, bsp) = make_cubic_bezier();
        // At u=0, tangent is in direction P1-P0 = (1,2). Normal = (-2,1) (rotated 90 CCW)
        let n = bsp.normal_at_param(&s, 0.0, None);
        let scale = (n.x * n.x + n.y * n.y).sqrt();
        assert!(scale > 1e-10, "normal should be nonzero");
        assert_abs_diff_eq!(n.x / scale, -2.0 / 5.0_f64.sqrt(), epsilon = 1e-6);
        assert_abs_diff_eq!(n.y / scale, 1.0 / 5.0_f64.sqrt(), epsilon = 1e-6);
    }

    #[test]
    fn bspline_normal_derivative_wrt_pole_fd() {
        let (mut s, bsp) = make_cubic_bezier();
        let dp = bsp.poles[1].x; // differentiate w.r.t. pole1.x
        let u = 0.3;
        let n = bsp.normal_at_param(&s, u, Some(dp));
        let eps = 1e-7;
        let n0 = bsp.normal_at_param(&s, u, None);
        s.set(dp, s.get(dp) + eps);
        let n1 = bsp.normal_at_param(&s, u, None);
        assert_abs_diff_eq!(n.dx, (n1.x - n0.x) / eps, epsilon = 1e-4);
        assert_abs_diff_eq!(n.dy, (n1.y - n0.y) / eps, epsilon = 1e-4);
    }

    #[test]
    fn bspline_normal_derivative_wrt_weight_fd() {
        let (mut s, mut bsp) = make_cubic_bezier();
        s.set(bsp.weights[1], 2.0); // non-trivial weight
        bsp.setup_flat_knots(&s);
        let dp = bsp.weights[1];
        let u = 0.5;
        let n = bsp.normal_at_param(&s, u, Some(dp));
        let eps = 1e-7;
        let n0 = bsp.normal_at_param(&s, u, None);
        s.set(dp, s.get(dp) + eps);
        let n1 = bsp.normal_at_param(&s, u, None);
        assert_abs_diff_eq!(n.dx, (n1.x - n0.x) / eps, epsilon = 1e-4);
        assert_abs_diff_eq!(n.dy, (n1.y - n0.y) / eps, epsilon = 1e-4);
    }

    #[test]
    fn bspline_periodic_setup_flat_knots() {
        // Periodic cubic with knots [0,1,2,3], mult [1,1,1,1]
        let mut s = ParamStore::new();
        let poles: Vec<Point> = (0..3).map(|_| Point::new(s.push(0.0), s.push(0.0))).collect();
        let weights: Vec<ParamIdx> = (0..3).map(|_| s.push(1.0)).collect();
        let knots = vec![s.push(0.0), s.push(1.0), s.push(2.0), s.push(3.0)];
        let mut bsp = BSpline {
            start: poles[0], end: poles[2],
            poles, weights, knots,
            mult: vec![1, 1, 1, 1],
            degree: 3, periodic: true, flat_knots: vec![],
        };
        bsp.setup_flat_knots(&s);
        // Periodic padding adds degree+1-mult[0] = 3 knots on each side
        assert!(bsp.flat_knots.len() > 4, "periodic padding should extend knot vector");
    }
}
