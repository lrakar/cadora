//! # cadora-subsystem
//!
//! Partitioned subsystem: parameter scoping, Jacobian and gradient evaluation.
//! Port of planegcs `SubSystem.h` / `SubSystem.cpp`.

use std::collections::{HashMap, HashSet};

use cadora_constraints::Constraint;
use cadora_core::{ParamIdx, ParamStore};

/// A partitioned sub-problem: a subset of constraints and the free parameters
/// they depend on, with an isolated working copy of the parameter store.
///
/// Mirrors the C++ `GCS::SubSystem` class.  The working store (`self.store`)
/// is a clone of the global store and may be freely mutated by solver
/// algorithms without affecting the original values until `apply_solution()`
/// is called.
#[allow(dead_code)]
pub struct SubSystem {
    constraints: Vec<Box<dyn Constraint>>,
    /// Ordered list of free parameter indices (after intersection with
    /// constraint params and optional reduction).
    param_list: Vec<ParamIdx>,
    /// Map from ParamIdx → position in `param_list`.
    idx_map: HashMap<ParamIdx, usize>,
    /// Working copy of the parameter store (solver isolation).
    store: ParamStore,
    /// Constraint index → local param indices used by that constraint.
    c2p: Vec<Vec<usize>>,
    /// Local param index → constraint indices that use it.
    p2c: Vec<Vec<usize>>,
    /// For each representative param, the list of ALL original ParamIdx values
    /// that map to that slot (including the representative itself).
    /// Used by set_params to keep merged params in sync.
    slot_members: Vec<Vec<ParamIdx>>,
}

impl SubSystem {
    /// Create a subsystem from constraints and a set of free parameters.
    pub fn new(
        constraints: Vec<Box<dyn Constraint>>,
        free_params: &[ParamIdx],
        global_store: &ParamStore,
    ) -> Self {
        Self::new_with_reduction(constraints, free_params, &HashMap::new(), global_store)
    }

    /// Create a subsystem with an optional parameter reduction map.
    ///
    /// `reduction` maps a dependent ParamIdx to the ParamIdx it should be
    /// merged with.  Multiple params mapping to the same target share a single
    /// slot in the working vector.
    pub fn new_with_reduction(
        constraints: Vec<Box<dyn Constraint>>,
        free_params: &[ParamIdx],
        reduction: &HashMap<ParamIdx, ParamIdx>,
        global_store: &ParamStore,
    ) -> Self {
        // Collect all params referenced by the constraints.
        let mut constraint_param_set = HashSet::new();
        for c in &constraints {
            for &p in c.params() {
                constraint_param_set.insert(p);
            }
        }

        // Intersect free_params with constraint params (preserving order).
        let relevant: Vec<ParamIdx> = free_params
            .iter()
            .filter(|p| constraint_param_set.contains(p))
            .copied()
            .collect();

        // Build param_list applying reduction.
        let mut param_list: Vec<ParamIdx> = Vec::new();
        let mut idx_map: HashMap<ParamIdx, usize> = HashMap::new();

        if reduction.is_empty() {
            for &p in &relevant {
                let local_idx = param_list.len();
                idx_map.insert(p, local_idx);
                param_list.push(p);
            }
        } else {
            let mut target_idx: HashMap<ParamIdx, usize> = HashMap::new();
            for &p in &relevant {
                if let Some(&target) = reduction.get(&p) {
                    if let Some(&existing) = target_idx.get(&target) {
                        idx_map.insert(p, existing);
                    } else {
                        let local_idx = param_list.len();
                        param_list.push(target);
                        target_idx.insert(target, local_idx);
                        idx_map.insert(p, local_idx);
                    }
                } else if let Some(&existing) = target_idx.get(&p) {
                    idx_map.insert(p, existing);
                } else {
                    let local_idx = param_list.len();
                    param_list.push(p);
                    target_idx.insert(p, local_idx);
                    idx_map.insert(p, local_idx);
                }
            }
        }

        // Clone the global store for isolated solver work.
        let store = global_store.clone();

        // Build adjacency lists.
        let csize = constraints.len();
        let psize = param_list.len();
        let mut c2p = vec![Vec::new(); csize];
        let mut p2c = vec![Vec::new(); psize];

        for (ci, c) in constraints.iter().enumerate() {
            let mut seen = HashSet::new();
            for &p in c.params() {
                if let Some(&local_j) = idx_map.get(&p) {
                    if seen.insert(local_j) {
                        c2p[ci].push(local_j);
                        p2c[local_j].push(ci);
                    }
                }
            }
        }

        // Build slot_members: for each slot, all original ParamIdx that map to it.
        let mut slot_members = vec![Vec::new(); psize];
        for (&p, &slot) in &idx_map {
            slot_members[slot].push(p);
        }

        Self {
            constraints,
            param_list,
            idx_map,
            store,
            c2p,
            p2c,
            slot_members,
        }
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    pub fn p_size(&self) -> usize {
        self.param_list.len()
    }

    pub fn c_size(&self) -> usize {
        self.constraints.len()
    }

    pub fn param_list(&self) -> &[ParamIdx] {
        &self.param_list
    }

    pub fn store(&self) -> &ParamStore {
        &self.store
    }

    pub fn store_mut(&mut self) -> &mut ParamStore {
        &mut self.store
    }

    pub fn constraints(&self) -> &[Box<dyn Constraint>] {
        &self.constraints
    }

    // -----------------------------------------------------------------------
    // Parameter transfer (mirrors C++ getParams / setParams)
    // -----------------------------------------------------------------------

    /// Read working-store values for the subsystem's params into a dense vector.
    pub fn get_params(&self) -> Vec<f64> {
        self.param_list
            .iter()
            .map(|&p| self.store.get(p))
            .collect()
    }

    /// Write a dense vector into the working store's subsystem params.
    /// When reduction is active, also updates all merged params that share
    /// the same slot so that constraint evaluations see consistent values.
    pub fn set_params(&mut self, x: &[f64]) {
        assert_eq!(x.len(), self.param_list.len());
        for (i, _) in self.param_list.iter().enumerate() {
            let val = x[i];
            // Set all params that map to this slot
            for &member in &self.slot_members[i] {
                self.store.set(member, val);
            }
        }
    }

    /// Copy current global-store values into the working store (C++ redirectParams copy step).
    /// Also propagates representative values to all merged params.
    pub fn sync_from(&mut self, global_store: &ParamStore) {
        for (i, &p) in self.param_list.iter().enumerate() {
            let val = global_store.get(p);
            for &member in &self.slot_members[i] {
                self.store.set(member, val);
            }
        }
    }

    /// Copy solution from working store back to the global store (C++ applySolution).
    /// Writes all merged params so the global store is consistent.
    pub fn apply_solution(&self, global_store: &mut ParamStore) {
        for (i, _) in self.param_list.iter().enumerate() {
            let val = self.store.get(self.param_list[i]);
            for &member in &self.slot_members[i] {
                global_store.set(member, val);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Error / Residual / Jacobian / Gradient
    // -----------------------------------------------------------------------

    /// Total error: 0.5 × Σ error_i².
    pub fn error(&self) -> f64 {
        let mut err = 0.0;
        for c in &self.constraints {
            let e = c.error(&self.store);
            err += e * e;
        }
        err * 0.5
    }

    /// Residual vector  r[i] = constraint_i.error(store).
    pub fn calc_residual(&self) -> Vec<f64> {
        self.constraints
            .iter()
            .map(|c| c.error(&self.store))
            .collect()
    }

    /// Residual vector and total error in one pass.
    pub fn calc_residual_and_error(&self) -> (Vec<f64>, f64) {
        let r = self.calc_residual();
        let err = r.iter().map(|ri| ri * ri).sum::<f64>() * 0.5;
        (r, err)
    }

    /// Jacobian matrix  J[i,j] = constraint_i.grad(store, param_list[j]).
    ///
    /// Returns a flat row-major buffer of size `c_size() × p_size()`.
    pub fn calc_jacobi(&self) -> Vec<f64> {
        let csize = self.constraints.len();
        let psize = self.param_list.len();
        let mut j = vec![0.0; csize * psize];
        for col in 0..psize {
            let p = self.param_list[col];
            for row in 0..csize {
                j[row * psize + col] = self.constraints[row].grad(&self.store, p);
            }
        }
        j
    }

    /// Gradient vector  grad[j] = Σ_i  r_i · J_{i,j}.
    ///
    /// Uses the sparse p2c adjacency for efficiency (matches C++ implementation).
    pub fn calc_grad(&self) -> Vec<f64> {
        let psize = self.param_list.len();
        let mut grad = vec![0.0; psize];
        for j in 0..psize {
            let p = self.param_list[j];
            for &ci in &self.p2c[j] {
                let c = &self.constraints[ci];
                grad[j] += c.error(&self.store) * c.grad(&self.store, p);
            }
        }
        grad
    }

    /// Maximum step along direction `xdir` that keeps constraints valid.
    ///
    /// Currently returns a large default; per-constraint `maxStep` hooks can
    /// be added when arc/angle wrapping constraints are ported.
    pub fn max_step(&self, _xdir: &[f64]) -> f64 {
        1e10
    }

    /// Consume the subsystem and return its owned constraints.
    pub fn into_constraints(self) -> Vec<Box<dyn Constraint>> {
        self.constraints
    }
}

/// Quadratic-interpolation line search along direction `xdir`.
///
/// Port of the C++ `lineSearch(SubSystem*, VectorXd&)` free function.
/// Modifies `subsys` working store; on return the store holds the best point.
pub fn line_search(subsys: &mut SubSystem, xdir: &[f64]) -> f64 {
    let alpha_max = subsys.max_step(xdir);
    let x0 = subsys.get_params();
    let psize = x0.len();

    let set_alpha = |subsys: &mut SubSystem, alpha: f64| {
        let x: Vec<f64> = (0..psize).map(|i| x0[i] + alpha * xdir[i]).collect();
        subsys.set_params(&x);
    };

    // f1 at alpha1 = 0 (initial)
    let alpha1 = 0.0;
    let f1 = subsys.error();

    // f2 at alpha2 = 1
    let mut alpha2 = 1.0;
    set_alpha(subsys, alpha2);
    let mut f2 = subsys.error();

    // f3 at alpha3 = 2
    let mut alpha3 = 2.0;
    set_alpha(subsys, alpha3);
    let mut f3 = subsys.error();

    // Bracket the minimum: f1 > f2 < f3
    while f2 > f1 || f2 > f3 {
        if f2 > f1 {
            alpha3 = alpha2;
            f3 = f2;
            alpha2 *= 0.5;
            set_alpha(subsys, alpha2);
            f2 = subsys.error();
        } else if f2 > f3 {
            if alpha3 >= alpha_max {
                break;
            }
            alpha2 = alpha3;
            f2 = f3;
            alpha3 *= 2.0;
            set_alpha(subsys, alpha3);
            f3 = subsys.error();
        }
    }

    // Quadratic interpolation for the minimum
    let denom = 3.0 * (f1 - 2.0 * f2 + f3);
    let mut alpha_star = if denom.abs() > 1e-30 {
        alpha2 + ((alpha2 - alpha1) * (f1 - f3)) / denom
    } else {
        alpha2
    };

    // Clamp to bracket
    if alpha_star >= alpha3 || alpha_star <= alpha1 {
        alpha_star = alpha2;
    }
    if alpha_star > alpha_max {
        alpha_star = alpha_max;
    }
    if alpha_star.is_nan() {
        alpha_star = 0.0;
    }

    set_alpha(subsys, alpha_star);
    alpha_star
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    // A simple constraint for testing: error = x - target
    struct ParamTarget {
        pvec: Vec<ParamIdx>,
        target: f64,
        tag: i32,
    }

    impl ParamTarget {
        fn new(p: ParamIdx, target: f64) -> Self {
            Self {
                pvec: vec![p],
                target,
                tag: 0,
            }
        }
    }

    impl Constraint for ParamTarget {
        fn error(&self, store: &ParamStore) -> f64 {
            store.get(self.pvec[0]) - self.target
        }
        fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] { 1.0 } else { 0.0 }
        }
        fn params(&self) -> &[ParamIdx] {
            &self.pvec
        }
        fn tag(&self) -> i32 { self.tag }
        fn is_driving(&self) -> bool { true }
    }

    // constraint: px + py = target
    struct SumTarget {
        pvec: Vec<ParamIdx>,
        target: f64,
    }

    impl SumTarget {
        fn new(px: ParamIdx, py: ParamIdx, target: f64) -> Self {
            Self {
                pvec: vec![px, py],
                target,
            }
        }
    }

    impl Constraint for SumTarget {
        fn error(&self, store: &ParamStore) -> f64 {
            store.get(self.pvec[0]) + store.get(self.pvec[1]) - self.target
        }
        fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] || param == self.pvec[1] { 1.0 } else { 0.0 }
        }
        fn params(&self) -> &[ParamIdx] {
            &self.pvec
        }
        fn tag(&self) -> i32 { 0 }
        fn is_driving(&self) -> bool { true }
    }

    #[test]
    fn subsystem_basic_sizes() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);
        let z = store.push(3.0);

        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 5.0));
        let c2: Box<dyn Constraint> = Box::new(ParamTarget::new(y, 7.0));

        // Only x and y are free; z is not used by any constraint
        let ss = SubSystem::new(vec![c1, c2], &[x, y, z], &store);
        assert_eq!(ss.p_size(), 2); // z filtered out
        assert_eq!(ss.c_size(), 2);
    }

    #[test]
    fn subsystem_get_set_params() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);

        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 0.0));
        let c2: Box<dyn Constraint> = Box::new(ParamTarget::new(y, 0.0));

        let mut ss = SubSystem::new(vec![c1, c2], &[x, y], &store);
        let p = ss.get_params();
        assert_eq!(p, vec![1.0, 2.0]);

        ss.set_params(&[10.0, 20.0]);
        assert_eq!(ss.get_params(), vec![10.0, 20.0]);
        // Global store should be untouched
        assert_eq!(store.get(x), 1.0);
        assert_eq!(store.get(y), 2.0);
    }

    #[test]
    fn subsystem_error_and_residual() {
        let mut store = ParamStore::new();
        let x = store.push(3.0);
        let y = store.push(5.0);

        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 1.0)); // error = 2
        let c2: Box<dyn Constraint> = Box::new(ParamTarget::new(y, 2.0)); // error = 3

        let ss = SubSystem::new(vec![c1, c2], &[x, y], &store);
        let r = ss.calc_residual();
        assert_eq!(r, vec![2.0, 3.0]);
        // error = 0.5*(4+9) = 6.5
        assert_abs_diff_eq!(ss.error(), 6.5);
    }

    #[test]
    fn subsystem_calc_jacobi() {
        let mut store = ParamStore::new();
        let x = store.push(3.0);
        let y = store.push(5.0);

        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 1.0));
        let c2: Box<dyn Constraint> = Box::new(SumTarget::new(x, y, 0.0));

        let ss = SubSystem::new(vec![c1, c2], &[x, y], &store);
        let j = ss.calc_jacobi(); // 2×2 row-major: [[1,0],[1,1]]
        assert_eq!(j, vec![1.0, 0.0, 1.0, 1.0]);
    }

    #[test]
    fn subsystem_calc_grad() {
        let mut store = ParamStore::new();
        let x = store.push(3.0);
        let y = store.push(5.0);

        // c1: error = x - 1 = 2, grad_x = 1
        // c2: error = x + y - 0 = 8, grad_x = 1, grad_y = 1
        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 1.0));
        let c2: Box<dyn Constraint> = Box::new(SumTarget::new(x, y, 0.0));

        let ss = SubSystem::new(vec![c1, c2], &[x, y], &store);
        let grad = ss.calc_grad();
        // grad_x = r1*J1x + r2*J2x = 2*1 + 8*1 = 10
        // grad_y = r2*J2y = 8*1 = 8
        assert_eq!(grad, vec![10.0, 8.0]);
    }

    #[test]
    fn subsystem_apply_solution() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);

        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 0.0));
        let c2: Box<dyn Constraint> = Box::new(ParamTarget::new(y, 0.0));

        let mut ss = SubSystem::new(vec![c1, c2], &[x, y], &store);
        ss.set_params(&[42.0, 99.0]);
        ss.apply_solution(&mut store);
        assert_eq!(store.get(x), 42.0);
        assert_eq!(store.get(y), 99.0);
    }

    #[test]
    fn subsystem_sync_from() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);

        let c: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 0.0));
        let mut ss = SubSystem::new(vec![c], &[x], &store);

        // Modify global store, then sync
        store.set(x, 123.0);
        assert_eq!(ss.get_params(), vec![1.0]); // still old
        ss.sync_from(&store);
        assert_eq!(ss.get_params(), vec![123.0]); // now synced
    }

    #[test]
    fn subsystem_reduction() {
        let mut store = ParamStore::new();
        let x = store.push(5.0);
        let y = store.push(5.0);

        // Two constraints, one on x, one on y
        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 3.0));
        let c2: Box<dyn Constraint> = Box::new(ParamTarget::new(y, 3.0));

        // Reduce y → x (they share the same slot)
        let mut reduction = HashMap::new();
        reduction.insert(y, x);

        let ss = SubSystem::new_with_reduction(vec![c1, c2], &[x, y], &reduction, &store);
        assert_eq!(ss.p_size(), 1); // x and y merged into one slot
        assert_eq!(ss.c_size(), 2);
    }

    #[test]
    fn line_search_descent() {
        let mut store = ParamStore::new();
        let x = store.push(5.0); // target = 0.0

        let c: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 0.0));
        let mut ss = SubSystem::new(vec![c], &[x], &store);

        let err_before = ss.error(); // 0.5*25 = 12.5
        // Search in the negative gradient direction
        let grad = ss.calc_grad(); // [5.0]
        let xdir: Vec<f64> = grad.iter().map(|g| -g).collect();

        let alpha = line_search(&mut ss, &xdir);
        assert!(alpha > 0.0);
        let err_after = ss.error();
        assert!(err_after < err_before);
    }

    #[test]
    fn line_search_converges_for_linear() {
        // For a single linear constraint x - target = 0, the minimum of
        // f(α) = 0.5*(x0 - α*grad - target)² should be reached exactly.
        let mut store = ParamStore::new();
        let x = store.push(10.0);

        let c: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 0.0));
        let mut ss = SubSystem::new(vec![c], &[x], &store);
        let xdir = vec![-10.0]; // step towards target
        line_search(&mut ss, &xdir);
        assert_abs_diff_eq!(ss.error(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn calc_residual_and_error_consistent() {
        let mut store = ParamStore::new();
        let x = store.push(3.0);
        let c: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 1.0));
        let ss = SubSystem::new(vec![c], &[x], &store);
        let (r, err) = ss.calc_residual_and_error();
        assert_eq!(r, vec![2.0]);
        assert_abs_diff_eq!(err, 2.0);
    }

    #[test]
    fn jacobi_grad_consistency() {
        // Verify: grad = J^T * r
        let mut store = ParamStore::new();
        let x = store.push(3.0);
        let y = store.push(5.0);

        let c1: Box<dyn Constraint> = Box::new(ParamTarget::new(x, 1.0));
        let c2: Box<dyn Constraint> = Box::new(SumTarget::new(x, y, 0.0));

        let ss = SubSystem::new(vec![c1, c2], &[x, y], &store);
        let r = ss.calc_residual();
        let j = ss.calc_jacobi();
        let grad = ss.calc_grad();
        let psize = ss.p_size();
        let csize = ss.c_size();

        // Compute J^T * r manually
        for col in 0..psize {
            let mut val = 0.0;
            for row in 0..csize {
                val += j[row * psize + col] * r[row];
            }
            assert_abs_diff_eq!(val, grad[col], epsilon = 1e-12);
        }
    }
}
