//! # cadora-math
//!
//! Forward-mode automatic differentiation via `DeriVector2`.
//! Direct port of planegcs `DeriVector2` from `Geo.h` / `Geo.cpp`.

use cadora_core::{ParamIdx, ParamStore, Point};

/// 2D vector carrying both value and derivative with respect to a single parameter.
///
/// This is the core differentiation primitive. When constructed from a [`Point`],
/// `dx`/`dy` are set to 1.0 or 0.0 depending on whether `deriv_param` matches
/// that coordinate's [`ParamIdx`]. All operations propagate derivatives via chain rule.
#[derive(Debug, Clone, Copy)]
pub struct DeriVector2 {
    pub x: f64,
    pub dx: f64,
    pub y: f64,
    pub dy: f64,
}

impl DeriVector2 {
    pub fn new(x: f64, dx: f64, y: f64, dy: f64) -> Self {
        Self { x, dx, y, dy }
    }

    pub fn zero() -> Self {
        Self {
            x: 0.0,
            dx: 0.0,
            y: 0.0,
            dy: 0.0,
        }
    }

    /// Construct from a Point, seeding derivative for `deriv_param`.
    /// If `deriv_param` is `None`, all derivatives are zero (value-only mode).
    pub fn from_point(store: &ParamStore, p: Point, deriv_param: Option<ParamIdx>) -> Self {
        let x = store.get(p.x);
        let y = store.get(p.y);
        match deriv_param {
            Some(dp) => Self {
                x,
                dx: if p.x == dp { 1.0 } else { 0.0 },
                y,
                dy: if p.y == dp { 1.0 } else { 0.0 },
            },
            None => Self {
                x,
                dx: 0.0,
                y,
                dy: 0.0,
            },
        }
    }

    /// Construct from a single scalar parameter (used for radius, angle, etc.).
    /// Returns DeriVector2 with the value in x, zero in y.
    pub fn from_scalar(
        store: &ParamStore,
        param: ParamIdx,
        deriv_param: Option<ParamIdx>,
    ) -> (f64, f64) {
        let val = store.get(param);
        let dval = match deriv_param {
            Some(dp) if dp == param => 1.0,
            _ => 0.0,
        };
        (val, dval)
    }

    /// Length and its derivative. Handles zero-length vectors safely.
    pub fn length(&self) -> (f64, f64) {
        let l = (self.x * self.x + self.y * self.y).sqrt();
        let dl = if l > 0.0 {
            (self.x * self.dx + self.y * self.dy) / l
        } else {
            // planegcs returns 1.0 for dlength when l == 0
            // This prevents division by zero in normalization
            1.0
        };
        (l, dl)
    }

    /// Returns normalized vector (unit length) with propagated derivative.
    /// When length is zero, returns zero vector with raw derivatives unchanged
    /// (matches planegcs `getNormalized` behavior).
    pub fn normalized(&self) -> Self {
        let (l, dl) = self.length();
        if l == 0.0 {
            // planegcs: DeriVector2(0, dx, 0, dy) — preserves derivatives
            Self {
                x: 0.0,
                dx: self.dx,
                y: 0.0,
                dy: self.dy,
            }
        } else {
            self.div_scalar(l, dl)
        }
    }

    /// Dot product: (value, derivative).
    pub fn dot(&self, other: &Self) -> (f64, f64) {
        let val = self.x * other.x + self.y * other.y;
        let dval =
            self.dx * other.x + self.x * other.dx + self.dy * other.y + self.y * other.dy;
        (val, dval)
    }

    /// Cross product Z-component: (value, derivative).
    /// This is `self.x * other.y - self.y * other.x`.
    pub fn cross_z(&self, other: &Self) -> (f64, f64) {
        let val = self.x * other.y - self.y * other.x;
        let dval =
            self.dx * other.y + self.x * other.dy - self.dy * other.x - self.y * other.dx;
        (val, dval)
    }

    /// Add two DeriVector2 values.
    pub fn add(&self, other: &Self) -> Self {
        Self {
            x: self.x + other.x,
            dx: self.dx + other.dx,
            y: self.y + other.y,
            dy: self.dy + other.dy,
        }
    }

    /// Subtract: self - other.
    pub fn sub(&self, other: &Self) -> Self {
        Self {
            x: self.x - other.x,
            dx: self.dx - other.dx,
            y: self.y - other.y,
            dy: self.dy - other.dy,
        }
    }

    /// Multiply by scalar with derivative.
    pub fn mul_scalar(&self, s: f64, ds: f64) -> Self {
        Self {
            x: self.x * s,
            dx: self.dx * s + self.x * ds,
            y: self.y * s,
            dy: self.dy * s + self.y * ds,
        }
    }

    /// Divide by scalar with derivative (quotient rule).
    pub fn div_scalar(&self, d: f64, dd: f64) -> Self {
        let inv = 1.0 / d;
        let dinv = -dd / (d * d);
        self.mul_scalar(inv, dinv)
    }

    /// Rotate 90° counter-clockwise.
    pub fn rotate90ccw(&self) -> Self {
        Self {
            x: -self.y,
            dx: -self.dy,
            y: self.x,
            dy: self.dx,
        }
    }

    /// Rotate 90° clockwise.
    pub fn rotate90cw(&self) -> Self {
        Self {
            x: self.y,
            dx: self.dy,
            y: -self.x,
            dy: -self.dx,
        }
    }

    /// Negate.
    pub fn neg(&self) -> Self {
        Self {
            x: -self.x,
            dx: -self.dx,
            y: -self.y,
            dy: -self.dy,
        }
    }

    /// Squared length (avoids the sqrt).
    pub fn length_sq(&self) -> (f64, f64) {
        self.dot(self)
    }

    /// Multiply by a constant (no derivative on the multiplier).
    /// Equivalent to C++ `DeriVector2::mult(double val)`.
    pub fn scale(&self, val: f64) -> Self {
        Self {
            x: self.x * val,
            dx: self.dx * val,
            y: self.y * val,
            dy: self.dy * val,
        }
    }

    /// Linear combination: `self * m1 + v2 * m2` (constant multipliers).
    /// Equivalent to C++ `DeriVector2::linCombi(m1, v2, m2)`.
    pub fn lin_combi(&self, m1: f64, other: &Self, m2: f64) -> Self {
        Self {
            x: self.x * m1 + other.x * m2,
            dx: self.dx * m1 + other.dx * m2,
            y: self.y * m1 + other.y * m2,
            dy: self.dy * m1 + other.dy * m2,
        }
    }
}

// ---------------------------------------------------------------------------
// Operator implementations for ergonomics
// ---------------------------------------------------------------------------

impl std::ops::Add for DeriVector2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        DeriVector2::add(&self, &rhs)
    }
}

impl std::ops::Sub for DeriVector2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        DeriVector2::sub(&self, &rhs)
    }
}

impl std::ops::Neg for DeriVector2 {
    type Output = Self;
    fn neg(self) -> Self {
        DeriVector2::neg(&self)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn make_store_and_point() -> (ParamStore, Point) {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(3.0), store.push(4.0));
        (store, p)
    }

    #[test]
    fn from_point_no_deriv() {
        let (store, p) = make_store_and_point();
        let v = DeriVector2::from_point(&store, p, None);
        assert_eq!(v.x, 3.0);
        assert_eq!(v.y, 4.0);
        assert_eq!(v.dx, 0.0);
        assert_eq!(v.dy, 0.0);
    }

    #[test]
    fn from_point_deriv_x() {
        let (store, p) = make_store_and_point();
        let v = DeriVector2::from_point(&store, p, Some(p.x));
        assert_eq!(v.dx, 1.0);
        assert_eq!(v.dy, 0.0);
    }

    #[test]
    fn from_point_deriv_y() {
        let (store, p) = make_store_and_point();
        let v = DeriVector2::from_point(&store, p, Some(p.y));
        assert_eq!(v.dx, 0.0);
        assert_eq!(v.dy, 1.0);
    }

    #[test]
    fn length_3_4_5() {
        let v = DeriVector2::new(3.0, 1.0, 4.0, 0.0);
        let (l, dl) = v.length();
        assert_abs_diff_eq!(l, 5.0, epsilon = 1e-15);
        // dl = (3*1 + 4*0) / 5 = 0.6
        assert_abs_diff_eq!(dl, 0.6, epsilon = 1e-15);
    }

    #[test]
    fn length_zero_vector() {
        let v = DeriVector2::new(0.0, 1.0, 0.0, 0.0);
        let (l, dl) = v.length();
        assert_eq!(l, 0.0);
        assert_eq!(dl, 1.0); // planegcs behavior
    }

    #[test]
    fn normalized_unit() {
        let v = DeriVector2::new(3.0, 0.0, 4.0, 0.0);
        let n = v.normalized();
        assert_abs_diff_eq!(n.x, 0.6, epsilon = 1e-15);
        assert_abs_diff_eq!(n.y, 0.8, epsilon = 1e-15);
    }

    #[test]
    fn dot_product() {
        let a = DeriVector2::new(1.0, 0.0, 2.0, 0.0);
        let b = DeriVector2::new(3.0, 0.0, 4.0, 0.0);
        let (val, _) = a.dot(&b);
        assert_abs_diff_eq!(val, 11.0, epsilon = 1e-15);
    }

    #[test]
    fn cross_product_z() {
        let a = DeriVector2::new(1.0, 0.0, 0.0, 0.0);
        let b = DeriVector2::new(0.0, 0.0, 1.0, 0.0);
        let (val, _) = a.cross_z(&b);
        assert_abs_diff_eq!(val, 1.0, epsilon = 1e-15);
    }

    #[test]
    fn add_sub() {
        let a = DeriVector2::new(1.0, 0.5, 2.0, 0.3);
        let b = DeriVector2::new(3.0, 0.1, 4.0, 0.2);
        let sum = a + b;
        assert_abs_diff_eq!(sum.x, 4.0, epsilon = 1e-15);
        assert_abs_diff_eq!(sum.dx, 0.6, epsilon = 1e-15);
        let diff = a - b;
        assert_abs_diff_eq!(diff.x, -2.0, epsilon = 1e-15);
    }

    #[test]
    fn rotate90() {
        let v = DeriVector2::new(1.0, 0.0, 0.0, 0.0);
        let ccw = v.rotate90ccw();
        assert_abs_diff_eq!(ccw.x, 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(ccw.y, 1.0, epsilon = 1e-15);
        let cw = v.rotate90cw();
        assert_abs_diff_eq!(cw.x, 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(cw.y, -1.0, epsilon = 1e-15);
    }

    #[test]
    fn mul_div_scalar() {
        let v = DeriVector2::new(2.0, 1.0, 3.0, 0.0);
        let scaled = v.mul_scalar(2.0, 0.0);
        assert_abs_diff_eq!(scaled.x, 4.0, epsilon = 1e-15);
        assert_abs_diff_eq!(scaled.dx, 2.0, epsilon = 1e-15);

        let divided = v.div_scalar(2.0, 0.0);
        assert_abs_diff_eq!(divided.x, 1.0, epsilon = 1e-15);
        assert_abs_diff_eq!(divided.dx, 0.5, epsilon = 1e-15);
    }

    // Cross-validation test: compare analytical gradient vs finite difference
    #[test]
    fn gradient_cross_validation_length() {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(3.0), store.push(4.0));

        // Analytical derivative of length w.r.t. p.x
        let v = DeriVector2::from_point(&store, p, Some(p.x));
        let (l, dl_analytical) = v.length();

        // Finite difference
        let eps = 1e-8;
        store.set(p.x, 3.0 + eps);
        let v_plus = DeriVector2::from_point(&store, p, None);
        let (l_plus, _) = v_plus.length();
        let dl_fd = (l_plus - l) / eps;

        assert_abs_diff_eq!(dl_analytical, dl_fd, epsilon = 1e-6);
    }

    #[test]
    fn gradient_cross_validation_dot() {
        let mut store = ParamStore::new();
        let p1 = Point::new(store.push(1.0), store.push(2.0));
        let p2 = Point::new(store.push(3.0), store.push(4.0));

        // Analytical d(p1·p2)/d(p1.x)
        let v1 = DeriVector2::from_point(&store, p1, Some(p1.x));
        let v2 = DeriVector2::from_point(&store, p2, Some(p1.x));
        let (val, dval_analytical) = v1.dot(&v2);

        // Finite difference
        let eps = 1e-8;
        store.set(p1.x, 1.0 + eps);
        let v1p = DeriVector2::from_point(&store, p1, None);
        let v2p = DeriVector2::from_point(&store, p2, None);
        let (val_plus, _) = v1p.dot(&v2p);
        let dval_fd = (val_plus - val) / eps;

        assert_abs_diff_eq!(dval_analytical, dval_fd, epsilon = 1e-6);
    }

    #[test]
    fn gradient_cross_validation_cross_z() {
        let mut store = ParamStore::new();
        let p1 = Point::new(store.push(2.0), store.push(3.0));
        let p2 = Point::new(store.push(5.0), store.push(7.0));

        // Analytical d(p1×p2)/d(p1.y)
        let v1 = DeriVector2::from_point(&store, p1, Some(p1.y));
        let v2 = DeriVector2::from_point(&store, p2, Some(p1.y));
        let (val, dval_analytical) = v1.cross_z(&v2);

        let eps = 1e-8;
        store.set(p1.y, 3.0 + eps);
        let v1p = DeriVector2::from_point(&store, p1, None);
        let v2p = DeriVector2::from_point(&store, p2, None);
        let (val_plus, _) = v1p.dot(&v2p); // intentionally dot — we compute cross below
        let _ = val_plus;
        let (cross_plus, _) = v1p.cross_z(&v2p);
        let dval_fd = (cross_plus - val) / eps;

        assert_abs_diff_eq!(dval_analytical, dval_fd, epsilon = 1e-6);
    }

    #[test]
    fn gradient_cross_validation_normalized() {
        // Test d(norm(v).x)/d(p.x) and d(norm(v).y)/d(p.x)
        let mut store = ParamStore::new();
        let p = Point::new(store.push(3.0), store.push(4.0));

        let v = DeriVector2::from_point(&store, p, Some(p.x));
        let n = v.normalized();
        let nx_analytical = n.dx;
        let ny_analytical = n.dy;

        let eps = 1e-8;
        store.set(p.x, 3.0 + eps);
        let vp = DeriVector2::from_point(&store, p, None);
        let np = vp.normalized();
        let nx_fd = (np.x - n.x) / eps;
        let ny_fd = (np.y - n.y) / eps;

        assert_abs_diff_eq!(nx_analytical, nx_fd, epsilon = 1e-6);
        assert_abs_diff_eq!(ny_analytical, ny_fd, epsilon = 1e-6);
    }

    #[test]
    fn gradient_cross_validation_mul_scalar() {
        // f(t) = v * t where v = (3,4) and t is a parameter
        let mut store = ParamStore::new();
        let p = Point::new(store.push(3.0), store.push(4.0));
        let t_idx = store.push(2.5);

        // d/dt of (v * t)
        let v = DeriVector2::from_point(&store, p, Some(t_idx));
        let (t, dt) = DeriVector2::from_scalar(&store, t_idx, Some(t_idx));
        let result = v.mul_scalar(t, dt);

        let eps = 1e-8;
        store.set(t_idx, 2.5 + eps);
        let vp = DeriVector2::from_point(&store, p, None);
        let (tp, _) = DeriVector2::from_scalar(&store, t_idx, None);
        let result_plus = vp.mul_scalar(tp, 0.0);

        let dx_fd = (result_plus.x - result.x) / eps;
        let dy_fd = (result_plus.y - result.y) / eps;

        assert_abs_diff_eq!(result.dx, dx_fd, epsilon = 1e-6);
        assert_abs_diff_eq!(result.dy, dy_fd, epsilon = 1e-6);
    }

    #[test]
    fn scale_and_lin_combi() {
        let a = DeriVector2::new(1.0, 0.5, 2.0, 0.3);
        let b = DeriVector2::new(3.0, 0.1, 4.0, 0.2);

        // scale(2) should give same result as mul_scalar(2, 0)
        let s = a.scale(2.0);
        let m = a.mul_scalar(2.0, 0.0);
        assert_abs_diff_eq!(s.x, m.x, epsilon = 1e-15);
        assert_abs_diff_eq!(s.dx, m.dx, epsilon = 1e-15);
        assert_abs_diff_eq!(s.y, m.y, epsilon = 1e-15);
        assert_abs_diff_eq!(s.dy, m.dy, epsilon = 1e-15);

        // lin_combi: a*3 + b*(-2)
        let lc = a.lin_combi(3.0, &b, -2.0);
        assert_abs_diff_eq!(lc.x, 1.0 * 3.0 + 3.0 * (-2.0), epsilon = 1e-15);
        assert_abs_diff_eq!(lc.dx, 0.5 * 3.0 + 0.1 * (-2.0), epsilon = 1e-15);
        assert_abs_diff_eq!(lc.y, 2.0 * 3.0 + 4.0 * (-2.0), epsilon = 1e-15);
        assert_abs_diff_eq!(lc.dy, 0.3 * 3.0 + 0.2 * (-2.0), epsilon = 1e-15);
    }

    #[test]
    fn length_sq_cross_validation() {
        let v = DeriVector2::new(3.0, 1.0, 4.0, 0.0);
        let (lsq, dlsq) = v.length_sq();
        assert_abs_diff_eq!(lsq, 25.0, epsilon = 1e-15);
        // d(x^2+y^2)/dp = 2*x*dx + 2*y*dy = 2*3*1 + 2*4*0 = 6
        assert_abs_diff_eq!(dlsq, 6.0, epsilon = 1e-15);
    }
}
