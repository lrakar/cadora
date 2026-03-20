//! # cadora-core
//!
//! Central types for the CADORA constraint solver.
//! Replaces the raw `double*` parameter model from planegcs with safe indexed storage.

use std::collections::HashMap;
use std::fmt;

// ---------------------------------------------------------------------------
// ParamIdx — typed index into the parameter store
// ---------------------------------------------------------------------------

/// Index into a [`ParamStore`]. Replaces the raw `double*` pattern from C++ planegcs.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ParamIdx(pub usize);

impl fmt::Debug for ParamIdx {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "P({})", self.0)
    }
}

// ---------------------------------------------------------------------------
// ParamStore — central parameter storage
// ---------------------------------------------------------------------------

/// Owns all solver parameter values. Geometry and constraints reference parameters
/// by [`ParamIdx`] rather than raw pointers.
#[derive(Debug, Clone)]
pub struct ParamStore {
    values: Vec<f64>,
}

impl ParamStore {
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            values: Vec::with_capacity(cap),
        }
    }

    /// Allocate a new parameter with the given initial value. Returns its index.
    pub fn push(&mut self, val: f64) -> ParamIdx {
        let idx = ParamIdx(self.values.len());
        self.values.push(val);
        idx
    }

    #[inline]
    pub fn get(&self, idx: ParamIdx) -> f64 {
        self.values[idx.0]
    }

    #[inline]
    pub fn set(&mut self, idx: ParamIdx, val: f64) {
        self.values[idx.0] = val;
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Get a slice of all values (for bulk reads by solvers).
    pub fn as_slice(&self) -> &[f64] {
        &self.values
    }

    /// Get a mutable slice (for bulk writes by solvers).
    pub fn as_mut_slice(&mut self) -> &mut [f64] {
        &mut self.values
    }

    /// Snapshot current values (equivalent to C++ `setReference()`).
    pub fn snapshot(&self) -> Vec<f64> {
        self.values.clone()
    }

    /// Restore from a snapshot (equivalent to C++ `resetToReference()`).
    pub fn restore(&mut self, snapshot: &[f64]) {
        assert_eq!(snapshot.len(), self.values.len(), "snapshot size mismatch");
        self.values.copy_from_slice(snapshot);
    }
}

impl Default for ParamStore {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Point — two parameter indices
// ---------------------------------------------------------------------------

/// A 2D point represented as two parameter indices.
/// Equivalent to planegcs `struct Point { double *x; double *y; }` but safe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Point {
    pub x: ParamIdx,
    pub y: ParamIdx,
}

impl Point {
    pub fn new(x: ParamIdx, y: ParamIdx) -> Self {
        Self { x, y }
    }
}

// ---------------------------------------------------------------------------
// Constraint tag
// ---------------------------------------------------------------------------

/// Constraint tag. Positive = user constraint. Negative = auxiliary (e.g. drag).
/// Zero = untagged.
pub type Tag = i32;

// ---------------------------------------------------------------------------
// Solver result
// ---------------------------------------------------------------------------

/// Outcome of a solve attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveStatus {
    Success,
    Converged,
    Failed,
    SuccessfulSolutionInvalid,
}

// ---------------------------------------------------------------------------
// Solver algorithm selection
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Algorithm {
    Bfgs,
    LevenbergMarquardt,
    DogLeg,
}

impl Default for Algorithm {
    fn default() -> Self {
        Self::DogLeg
    }
}

// ---------------------------------------------------------------------------
// DogLeg Gauss step variant
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DogLegGaussStep {
    FullPivLu,
    LeastNormFullPivLu,
    LeastNormLdlt,
}

impl Default for DogLegGaussStep {
    fn default() -> Self {
        Self::FullPivLu
    }
}

// ---------------------------------------------------------------------------
// QR algorithm selection
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QrAlgorithm {
    DenseQr,
    SparseQr,
}

impl Default for QrAlgorithm {
    fn default() -> Self {
        Self::SparseQr
    }
}

// ---------------------------------------------------------------------------
// Debug mode
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DebugMode {
    NoDebug,
    Minimal,
    IterationLevel,
}

impl Default for DebugMode {
    fn default() -> Self {
        Self::Minimal
    }
}

// ---------------------------------------------------------------------------
// Solver constants (from C++ #defines)
// ---------------------------------------------------------------------------

/// Rough convergence threshold — used for initial fast-check.
/// C++: `#define XconvergenceRough 1e-8`
pub const X_CONVERGENCE_ROUGH: f64 = 1e-8;

/// Threshold below which a squared-error sum is considered "small".
/// C++: `#define smallF 1e-20`
pub const SMALL_F: f64 = 1e-20;

// ---------------------------------------------------------------------------
// Solver configuration
// ---------------------------------------------------------------------------

/// All solver tunables. Mirrors planegcs `System` fields.
/// Defaults match the verified C++ constructor values.
#[derive(Debug, Clone)]
pub struct SolverConfig {
    pub max_iter: usize,
    pub max_iter_redundant: usize,
    pub convergence: f64,
    pub convergence_redundant: f64,
    pub sketch_size_multiplier: bool,
    pub sketch_size_multiplier_redundant: bool,

    // LM-specific
    pub lm_eps: f64,
    pub lm_eps1: f64,
    pub lm_tau: f64,
    pub lm_eps_redundant: f64,
    pub lm_eps1_redundant: f64,
    pub lm_tau_redundant: f64,

    // DogLeg-specific
    pub dl_tolg: f64,
    pub dl_tolx: f64,
    pub dl_tolf: f64,
    pub dl_tolg_redundant: f64,
    pub dl_tolx_redundant: f64,
    pub dl_tolf_redundant: f64,
    pub dog_leg_gauss_step: DogLegGaussStep,

    // QR diagnosis
    pub qr_algorithm: QrAlgorithm,
    pub qr_pivot_threshold: f64,
    pub auto_choose_algorithm: bool,
    pub auto_qr_threshold: usize,

    // Debug
    pub debug_mode: DebugMode,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iter: 100,
            max_iter_redundant: 100,
            convergence: 1e-10,
            convergence_redundant: 1e-10,
            sketch_size_multiplier: false,
            sketch_size_multiplier_redundant: false,

            lm_eps: 1e-10,
            lm_eps1: 1e-50,
            lm_tau: 1e-3,
            lm_eps_redundant: 1e-10,
            lm_eps1_redundant: 1e-50,
            lm_tau_redundant: 1e-3,

            dl_tolg: 1e-80,
            dl_tolx: 1e-80,
            dl_tolf: 1e-10,
            dl_tolg_redundant: 1e-80,
            dl_tolx_redundant: 1e-80,
            dl_tolf_redundant: 1e-10,
            dog_leg_gauss_step: DogLegGaussStep::default(),

            qr_algorithm: QrAlgorithm::default(),
            qr_pivot_threshold: 1e-13,
            auto_choose_algorithm: true,
            auto_qr_threshold: 1000,

            debug_mode: DebugMode::default(),
        }
    }
}

// ---------------------------------------------------------------------------
// Reduction map (equality elimination)
// ---------------------------------------------------------------------------

/// Maps a replaced parameter to the parameter it was merged into.
/// Equivalent to planegcs `MAP_pD_pD reductionmap`.
pub type ReductionMap = HashMap<ParamIdx, ParamIdx>;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_store_push_and_get() {
        let mut store = ParamStore::new();
        let a = store.push(3.14);
        let b = store.push(2.71);
        assert_eq!(store.get(a), 3.14);
        assert_eq!(store.get(b), 2.71);
        assert_eq!(store.len(), 2);
    }

    #[test]
    fn param_store_set() {
        let mut store = ParamStore::new();
        let a = store.push(1.0);
        store.set(a, 42.0);
        assert_eq!(store.get(a), 42.0);
    }

    #[test]
    fn param_store_snapshot_restore() {
        let mut store = ParamStore::new();
        let a = store.push(1.0);
        let b = store.push(2.0);
        let snap = store.snapshot();
        store.set(a, 99.0);
        store.set(b, 99.0);
        store.restore(&snap);
        assert_eq!(store.get(a), 1.0);
        assert_eq!(store.get(b), 2.0);
    }

    #[test]
    fn point_construction() {
        let mut store = ParamStore::new();
        let p = Point::new(store.push(1.0), store.push(2.0));
        assert_eq!(store.get(p.x), 1.0);
        assert_eq!(store.get(p.y), 2.0);
    }

    #[test]
    fn solver_config_defaults() {
        let cfg = SolverConfig::default();
        assert_eq!(cfg.max_iter, 100);
        assert_eq!(cfg.max_iter_redundant, 100);
        assert_eq!(cfg.convergence, 1e-10);
        assert_eq!(cfg.convergence_redundant, 1e-10);
        assert!(!cfg.sketch_size_multiplier);
        assert!(!cfg.sketch_size_multiplier_redundant);
        assert_eq!(cfg.auto_qr_threshold, 1000);
        assert_eq!(cfg.qr_algorithm, QrAlgorithm::SparseQr);
        assert_eq!(cfg.debug_mode, DebugMode::Minimal);
        assert_eq!(cfg.dog_leg_gauss_step, DogLegGaussStep::FullPivLu);
        // Redundant-specific solver params mirror normal defaults
        assert_eq!(cfg.lm_eps_redundant, cfg.lm_eps);
        assert_eq!(cfg.lm_eps1_redundant, cfg.lm_eps1);
        assert_eq!(cfg.lm_tau_redundant, cfg.lm_tau);
        assert_eq!(cfg.dl_tolg_redundant, cfg.dl_tolg);
        assert_eq!(cfg.dl_tolx_redundant, cfg.dl_tolx);
        assert_eq!(cfg.dl_tolf_redundant, cfg.dl_tolf);
    }

    #[test]
    fn solver_constants() {
        assert_eq!(X_CONVERGENCE_ROUGH, 1e-8);
        assert_eq!(SMALL_F, 1e-20);
    }
}
