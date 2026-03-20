//! # cadora-constraints
//!
//! All 36 constraint types with error + gradient computation.
//! Port of planegcs `Constraints.h` / `Constraints.cpp`.

use cadora_core::{ParamIdx, ParamStore, Tag};

/// The core constraint trait. Every constraint type implements this.
pub trait Constraint {
    /// Compute the constraint error (residual). Zero when satisfied.
    fn error(&self, store: &ParamStore) -> f64;

    /// Compute partial derivative of error w.r.t. one parameter.
    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64;

    /// Combined error + gradient (allows shared computation).
    /// Default implementation calls error() and grad() separately.
    fn error_grad(&self, store: &ParamStore, param: ParamIdx) -> (f64, f64) {
        (self.error(store), self.grad(store, param))
    }

    /// All parameter indices this constraint depends on.
    fn params(&self) -> &[ParamIdx];

    /// Constraint tag for identification/grouping.
    fn tag(&self) -> Tag;

    /// Whether this is a driving (active) constraint.
    fn is_driving(&self) -> bool;

    /// Maximum step size in a given direction (for line search clamping).
    fn max_step(&self, _store: &ParamStore, _dir: &[f64]) -> f64 {
        f64::MAX
    }

    /// Rescale error by geometry size (normalizes Jacobian).
    fn rescale(&mut self, _store: &ParamStore) {}

    /// For driven constraints: set the value parameter to match current geometry.
    fn evaluate(&self, _store: &mut ParamStore) {}
}

// Constraint implementations will be added in Phases 2-4 of the porting pipeline.
// Each of the 36 types will be ported individually from Constraints.cpp.
