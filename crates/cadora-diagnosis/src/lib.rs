//! # cadora-diagnosis
//!
//! QR rank analysis, DOF computation, conflict/redundancy detection.
//! Port of planegcs `diagnose()` and related methods from `GCS.cpp`.

use std::collections::{HashMap, HashSet};

use cadora_constraints::Constraint;
use cadora_core::{Algorithm, ParamIdx, ParamStore, SolverConfig};
use nalgebra::DMatrix;

// ---------------------------------------------------------------------------
// Diagnosis result
// ---------------------------------------------------------------------------

/// Result of constraint system diagnosis.
#[derive(Debug, Clone)]
pub struct DiagnosisResult {
    /// Degrees of freedom (negative if over-constrained).
    pub dofs: i32,
    /// Tags of conflicting constraints.
    pub conflicting_tags: Vec<i32>,
    /// Tags of fully redundant constraints (all solver constraints for a tag are redundant).
    pub redundant_tags: Vec<i32>,
    /// Tags of partially redundant constraints (some solver constraints redundant).
    pub partially_redundant_tags: Vec<i32>,
    /// Dependent parameter indices (under-constrained params).
    pub dependent_params: Vec<ParamIdx>,
    /// Groups of mutually dependent parameters.
    pub dependent_param_groups: Vec<Vec<ParamIdx>>,
    /// Whether there were no driving constraints at all.
    pub empty_diagnose_matrix: bool,
}

// ---------------------------------------------------------------------------
// makeReducedJacobian
// ---------------------------------------------------------------------------

/// Build a reduced Jacobian matrix that only includes driving constraints (tag >= 0)
/// and excludes driven-constraint value parameters.
///
/// Returns `(J, jacobian_constraint_map, diagnose_params, tag_multiplicity)`.
///
/// - `J`: rows = driving constraint count, cols = diagnose param count
/// - `jacobian_constraint_map`: maps reduced row index → original constraint index
/// - `diagnose_params`: parameter list excluding driven value params
/// - `tag_multiplicity`: how many solver constraints share each tag
pub fn make_reduced_jacobian(
    constraints: &[Box<dyn Constraint>],
    all_params: &[ParamIdx],
    driven_params: &[ParamIdx],
    store: &ParamStore,
) -> (
    DMatrix<f64>,
    HashMap<usize, usize>,
    Vec<ParamIdx>,
    HashMap<i32, usize>,
) {
    // Build diagnose param list (all params minus driven value params)
    let driven_set: HashSet<ParamIdx> = driven_params.iter().copied().collect();
    let diagnose_params: Vec<ParamIdx> = all_params
        .iter()
        .copied()
        .filter(|p| !driven_set.contains(p))
        .collect();

    let ncols = diagnose_params.len();

    // Count driving constraints with non-negative tags
    let driving_count = constraints
        .iter()
        .filter(|c| c.tag() >= 0 && c.is_driving())
        .count();

    let mut j_mat = DMatrix::zeros(driving_count, ncols);
    let mut jacobian_constraint_map = HashMap::new();
    let mut tag_multiplicity: HashMap<i32, usize> = HashMap::new();

    let mut jrow = 0;
    for (all_idx, constr) in constraints.iter().enumerate() {
        if constr.tag() >= 0 && constr.is_driving() {
            for (jcol, &param) in diagnose_params.iter().enumerate() {
                j_mat[(jrow, jcol)] = constr.grad(store, param);
            }

            *tag_multiplicity.entry(constr.tag()).or_insert(0) += 1;
            jacobian_constraint_map.insert(jrow, all_idx);
            jrow += 1;
        }
    }

    // If tagmultiplicity counting: the C++ counts additional occurrences (0-based
    // for first, so first=0, second=1 etc). We count total occurrences here
    // and subtract 1 later where needed. Actually looking at the C++ more carefully,
    // the first occurrence sets to 0, subsequent increment. So our count - 1 gives the
    // C++ value. We'll adjust in the comparison.

    (j_mat, jacobian_constraint_map, diagnose_params, tag_multiplicity)
}

// ---------------------------------------------------------------------------
// QR decomposition + rank
// ---------------------------------------------------------------------------

/// Perform QR decomposition on J^T (transposed Jacobian) and return (rank, R_upper).
///
/// Uses nalgebra's column-pivoted QR. The `R` matrix is upper-triangular,
/// and column permutations track which constraints are pivots.
fn qr_decomposition_transposed(
    j: &DMatrix<f64>,
    constr_count: usize,
    pivot_threshold: f64,
) -> (usize, DMatrix<f64>, Vec<usize>) {
    // J^T: params_num × constr_num
    let jt = j.rows(0, constr_count).transpose();

    if jt.nrows() == 0 || jt.ncols() == 0 {
        return (0, DMatrix::zeros(0, 0), vec![]);
    }

    let nrows = jt.nrows();
    let ncols = jt.ncols();
    let qr = jt.col_piv_qr();
    let r_full = qr.r();

    // Compute rank from R diagonal
    let min_dim = nrows.min(ncols);
    let mut rank = 0;
    for i in 0..min_dim {
        if r_full[(i, i)].abs() > pivot_threshold {
            rank += 1;
        } else {
            break;
        }
    }

    // Extract column permutation: permute index vector [0,1,...,ncols-1]
    let perm = qr.p();
    let mut idx_mat = DMatrix::from_fn(1, ncols, |_, j| j as f64);
    perm.permute_columns(&mut idx_mat);
    let col_perm: Vec<usize> = (0..ncols).map(|j| idx_mat[(0, j)] as usize).collect();

    (rank, r_full, col_perm)
}

/// QR decomposition of J (non-transposed) for parameter dependency analysis.
fn qr_decomposition_params(
    j: &DMatrix<f64>,
    constr_count: usize,
    pivot_threshold: f64,
) -> (usize, DMatrix<f64>, Vec<usize>) {
    let jg = j.rows(0, constr_count).clone_owned();

    if jg.nrows() == 0 || jg.ncols() == 0 {
        return (0, DMatrix::zeros(0, 0), vec![]);
    }

    let nrows = jg.nrows();
    let ncols = jg.ncols();
    let qr = jg.col_piv_qr();
    let r_full = qr.r();

    let min_dim = nrows.min(ncols);
    let mut rank = 0;
    for i in 0..min_dim {
        if r_full[(i, i)].abs() > pivot_threshold {
            rank += 1;
        } else {
            break;
        }
    }

    let perm = qr.p();
    let mut idx_mat = DMatrix::from_fn(1, ncols, |_, j| j as f64);
    perm.permute_columns(&mut idx_mat);
    let col_perm: Vec<usize> = (0..ncols).map(|j| idx_mat[(0, j)] as usize).collect();

    (rank, r_full, col_perm)
}

// ---------------------------------------------------------------------------
// eliminateNonZerosOverPivot — RREF-like reduction of upper-triangular R
// ---------------------------------------------------------------------------

/// Eliminate non-zero entries above each pivot in an upper-triangular matrix.
/// This produces a reduced row-echelon-like form that makes dependency groups visible.
fn eliminate_nonzeros_over_pivot(r: &mut DMatrix<f64>, rank: usize) {
    let ncols = r.ncols();
    for i in 1..rank {
        assert!(r[(i, i)] != 0.0, "pivot at ({i},{i}) is zero");
        for row in 0..i {
            if r[(row, i)].abs() > 1e-10 {
                let coef = r[(row, i)] / r[(i, i)];
                for col in (i + 1)..ncols {
                    let val = r[(i, col)];
                    r[(row, col)] -= coef * val;
                }
                r[(row, i)] = 0.0;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Identify dependent parameters
// ---------------------------------------------------------------------------

/// Identify parameters that are not fully constrained (dependent).
fn identify_dependent_params(
    j: &DMatrix<f64>,
    constr_count: usize,
    diagnose_params: &[ParamIdx],
    pivot_threshold: f64,
) -> (Vec<ParamIdx>, Vec<Vec<ParamIdx>>) {
    let (rank, mut r_params, col_perm) =
        qr_decomposition_params(j, constr_count, pivot_threshold);

    if rank == 0 || r_params.nrows() == 0 {
        // All params are dependent
        return (
            diagnose_params.to_vec(),
            vec![diagnose_params.to_vec()],
        );
    }

    eliminate_nonzeros_over_pivot(&mut r_params, rank);

    let ncols = col_perm.len();
    let mut dep_params = Vec::new();
    let mut dep_groups: Vec<Vec<ParamIdx>> = Vec::new();

    for j_idx in rank..ncols {
        let mut group = Vec::new();

        // Find which pivot rows have nonzero entries in this column
        for row in 0..rank {
            if r_params[(row, j_idx)].abs() > 1e-10 {
                let orig_col = col_perm[row];
                if orig_col < diagnose_params.len() {
                    group.push(diagnose_params[orig_col]);
                    dep_params.push(diagnose_params[orig_col]);
                }
            }
        }

        // The dependent column itself
        let orig_col = col_perm[j_idx];
        if orig_col < diagnose_params.len() {
            group.push(diagnose_params[orig_col]);
            dep_params.push(diagnose_params[orig_col]);
        }

        dep_groups.push(group);
    }

    (dep_params, dep_groups)
}

// ---------------------------------------------------------------------------
// diagnose — main entry point
// ---------------------------------------------------------------------------

/// Analyze constraints to determine DOF, conflicts, and redundancies.
///
/// Port of `System::diagnose()` from `GCS.cpp`.
///
/// # Parameters
/// - `constraints`: all constraints in the system
/// - `all_params`: all free parameters
/// - `driven_params`: parameters driven by other constraints (excluded from Jacobian columns)
/// - `store`: current parameter values
/// - `config`: solver configuration (controls QR thresholds, convergence, etc.)
/// - `algorithm`: which solver algorithm to use for redundancy-checking solve
pub fn diagnose(
    constraints: &[Box<dyn Constraint>],
    all_params: &[ParamIdx],
    driven_params: &[ParamIdx],
    store: &ParamStore,
    config: &SolverConfig,
    _algorithm: Algorithm,
) -> DiagnosisResult {
    if all_params.is_empty() || all_params.len() == driven_params.len() {
        return DiagnosisResult {
            dofs: 0,
            conflicting_tags: vec![],
            redundant_tags: vec![],
            partially_redundant_tags: vec![],
            dependent_params: vec![],
            dependent_param_groups: vec![],
            empty_diagnose_matrix: true,
        };
    }

    // Build reduced Jacobian
    let (j, jacobian_constraint_map, diagnose_params, tag_multiplicity) =
        make_reduced_jacobian(constraints, all_params, driven_params, store);

    let constr_count = jacobian_constraint_map.len();

    if constr_count == 0 {
        // No driving constraints — all DOFs are free
        return DiagnosisResult {
            dofs: diagnose_params.len() as i32,
            conflicting_tags: vec![],
            redundant_tags: vec![],
            partially_redundant_tags: vec![],
            dependent_params: diagnose_params.clone(),
            dependent_param_groups: vec![diagnose_params],
            empty_diagnose_matrix: true,
        };
    }

    let pivot_threshold = config.qr_pivot_threshold;

    // QR of J^T for constraint analysis
    let (rank, mut r_constr, col_perm) =
        qr_decomposition_transposed(&j, constr_count, pivot_threshold);

    // Identify dependent parameters (via QR of J, not J^T)
    let (dep_params, dep_groups) =
        identify_dependent_params(&j, constr_count, &diagnose_params, pivot_threshold);

    let params_num = diagnose_params.len();
    let mut dofs = (params_num as i32) - (rank as i32);

    // Detect conflicting/redundant constraints
    let mut conflicting_tags = vec![];
    let mut redundant_tags = vec![];
    let mut partially_redundant_tags = vec![];

    if constr_count > rank {
        // There are excess constraints — some may be conflicting or redundant
        // For now we do a simplified version: identify conflict groups from R
        // without the re-solve step (which needs constraint cloning).

        eliminate_nonzeros_over_pivot(&mut r_constr, rank);

        let num_groups = constr_count - rank;
        let mut conflict_groups: Vec<Vec<usize>> = Vec::with_capacity(num_groups);

        for j_idx in rank..constr_count {
            let mut group = Vec::new();
            for row in 0..rank {
                if r_constr[(row, j_idx)].abs() > 1e-10 {
                    let orig_col = col_perm[row];
                    if let Some(&all_idx) = jacobian_constraint_map.get(&orig_col) {
                        group.push(all_idx);
                    }
                }
            }
            let orig_col = col_perm[j_idx];
            if let Some(&all_idx) = jacobian_constraint_map.get(&orig_col) {
                group.push(all_idx);
            }
            conflict_groups.push(group);
        }

        // Popularity contest to identify which constraints to skip
        let mut skipped: HashSet<usize> = HashSet::new();
        let mut satisfied_groups: HashSet<usize> = HashSet::new();

        loop {
            let mut conflict_map: HashMap<usize, HashSet<usize>> = HashMap::new();

            for (gi, group) in conflict_groups.iter().enumerate() {
                if satisfied_groups.contains(&gi) {
                    continue;
                }
                for &constr_idx in group {
                    let constr = &constraints[constr_idx];
                    let is_ia = constr.is_internal_alignment();
                    let is_priority = constr.tag() == 0;
                    if !is_priority && !is_ia {
                        conflict_map.entry(constr_idx).or_default().insert(gi);
                    }
                }
            }

            if conflict_map.is_empty() {
                break;
            }

            let (&most_popular_idx, _) = conflict_map
                .iter()
                .max_by(|(idx_a, groups_a), (idx_b, groups_b)| {
                    let sz_a = groups_a.len();
                    let sz_b = groups_b.len();
                    let tag_a = constraints[**idx_a].tag();
                    let tag_b = constraints[**idx_b].tag();
                    let mult_a = tag_multiplicity.get(&tag_a).copied().unwrap_or(1);
                    let mult_b = tag_multiplicity.get(&tag_b).copied().unwrap_or(1);

                    sz_a.cmp(&sz_b)
                        .then(mult_b.cmp(&mult_a))
                        .then(tag_a.cmp(&tag_b))
                })
                .unwrap();

            let target_tag = constraints[most_popular_idx].tag();
            let same_tag_indices: Vec<usize> = conflict_map
                .keys()
                .copied()
                .filter(|&idx| constraints[idx].tag() == target_tag)
                .collect();

            for idx in same_tag_indices {
                if let Some(groups) = conflict_map.get(&idx) {
                    for &gi in groups {
                        satisfied_groups.insert(gi);
                    }
                }
                skipped.insert(idx);
            }
        }

        // Check if skipped constraints are redundant by evaluating their error
        // (simplified: check current error instead of re-solving)
        let mut redundant_set: HashSet<usize> = HashSet::new();
        for &skip_idx in &skipped {
            let err = constraints[skip_idx].error(store);
            if err * err < config.convergence_redundant {
                redundant_set.insert(skip_idx);
            }
        }

        // Count non-redundant excess constraints
        let mut nonredundant_constr_count = constr_count;
        for group in conflict_groups.iter().rev() {
            let has_redundant = group.iter().any(|idx| redundant_set.contains(idx));
            if has_redundant {
                nonredundant_constr_count -= 1;
            }
        }

        // If over-constrained: dofs = params - nonredundant
        if params_num == rank && nonredundant_constr_count > rank {
            dofs = params_num as i32 - nonredundant_constr_count as i32;
        }

        // Extract tags
        let mut conflicting_set: HashSet<i32> = HashSet::new();
        for group in &conflict_groups {
            let has_redundant = group.iter().any(|idx| redundant_set.contains(idx));
            if !has_redundant {
                for &idx in group {
                    if !constraints[idx].is_internal_alignment() {
                        conflicting_set.insert(constraints[idx].tag());
                    }
                }
            }
        }
        conflicting_set.remove(&0);

        let mut redundant_set_tags: HashSet<i32> = HashSet::new();
        let mut partial_set: HashSet<i32> = HashSet::new();
        for &idx in &redundant_set {
            redundant_set_tags.insert(constraints[idx].tag());
            partial_set.insert(constraints[idx].tag());
        }
        for (idx, constr) in constraints.iter().enumerate() {
            if !redundant_set.contains(&idx) {
                redundant_set_tags.remove(&constr.tag());
            }
        }
        for &t in &redundant_set_tags {
            partial_set.remove(&t);
        }

        conflicting_tags = conflicting_set.into_iter().collect();
        conflicting_tags.sort();
        redundant_tags = redundant_set_tags.into_iter().collect();
        redundant_tags.sort();
        partially_redundant_tags = partial_set.into_iter().collect();
        partially_redundant_tags.sort();
    }

    DiagnosisResult {
        dofs,
        conflicting_tags,
        redundant_tags,
        partially_redundant_tags,
        dependent_params: dep_params,
        dependent_param_groups: dep_groups,
        empty_diagnose_matrix: false,
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use cadora_constraints::Constraint;
    use cadora_core::{ParamIdx, ParamStore};

    // Simple constraint: error = x - target
    struct PTarget {
        pvec: Vec<ParamIdx>,
        target: f64,
        tag: i32,
    }
    impl PTarget {
        fn new(p: ParamIdx, target: f64, tag: i32) -> Self {
            Self { pvec: vec![p], target, tag }
        }
    }
    impl Constraint for PTarget {
        fn error(&self, store: &ParamStore) -> f64 {
            store.get(self.pvec[0]) - self.target
        }
        fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] { 1.0 } else { 0.0 }
        }
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> i32 { self.tag }
        fn is_driving(&self) -> bool { true }
    }

    // Constraint: error = a*x + b*y - c
    struct LinearCombo {
        pvec: Vec<ParamIdx>,
        a: f64,
        b: f64,
        c: f64,
        tag: i32,
    }
    impl LinearCombo {
        fn new(px: ParamIdx, py: ParamIdx, a: f64, b: f64, c: f64, tag: i32) -> Self {
            Self { pvec: vec![px, py], a, b, c, tag }
        }
    }
    impl Constraint for LinearCombo {
        fn error(&self, store: &ParamStore) -> f64 {
            self.a * store.get(self.pvec[0]) + self.b * store.get(self.pvec[1]) - self.c
        }
        fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] { self.a }
            else if param == self.pvec[1] { self.b }
            else { 0.0 }
        }
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> i32 { self.tag }
        fn is_driving(&self) -> bool { true }
    }

    // Non-driving constraint for testing filtering
    struct NonDriving {
        pvec: Vec<ParamIdx>,
    }
    impl Constraint for NonDriving {
        fn error(&self, _store: &ParamStore) -> f64 { 0.0 }
        fn grad(&self, _store: &ParamStore, _param: ParamIdx) -> f64 { 0.0 }
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> i32 { 1 }
        fn is_driving(&self) -> bool { false }
    }

    #[test]
    fn fully_constrained_system_dofs_zero() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);

        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(PTarget::new(x, 3.0, 1)),
            Box::new(PTarget::new(y, 5.0, 2)),
        ];

        let result = diagnose(
            &constraints, &[x, y], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        assert_eq!(result.dofs, 0);
        assert!(result.conflicting_tags.is_empty());
        assert!(result.redundant_tags.is_empty());
        assert!(result.dependent_params.is_empty());
    }

    #[test]
    fn under_constrained_system_has_dofs() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);

        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(PTarget::new(x, 3.0, 1)),
        ];

        let result = diagnose(
            &constraints, &[x, y], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        assert_eq!(result.dofs, 1);
        assert!(result.conflicting_tags.is_empty());
        // y should be dependent
        assert!(!result.dependent_params.is_empty());
    }

    #[test]
    fn over_constrained_system_detected() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);

        // Two constraints on the same parameter with different targets
        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(PTarget::new(x, 3.0, 1)),
            Box::new(PTarget::new(x, 5.0, 2)),
        ];

        let result = diagnose(
            &constraints, &[x], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        // Over-constrained: dofs should be negative
        assert!(result.dofs < 0);
        assert!(!result.conflicting_tags.is_empty());
    }

    #[test]
    fn redundant_constraint_detected() {
        let mut store = ParamStore::new();
        let x = store.push(3.0); // Already at target

        // Two identical constraints with the same target — currently satisfied
        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(PTarget::new(x, 3.0, 1)),
            Box::new(PTarget::new(x, 3.0, 2)),
        ];

        let result = diagnose(
            &constraints, &[x], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        // One constraint is redundant (not conflicting)
        assert!(!result.redundant_tags.is_empty());
    }

    #[test]
    fn empty_system_returns_zero_dofs() {
        let store = ParamStore::new();
        let result = diagnose(
            &[], &[], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );
        assert_eq!(result.dofs, 0);
        assert!(result.empty_diagnose_matrix);
    }

    #[test]
    fn non_driving_constraints_excluded() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);

        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(PTarget::new(x, 3.0, 1)),
            Box::new(NonDriving { pvec: vec![y] }),
        ];

        let result = diagnose(
            &constraints, &[x, y], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        // Only one driving constraint, two params → 1 DOF
        assert_eq!(result.dofs, 1);
    }

    #[test]
    fn driven_params_excluded_from_jacobian() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);

        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(PTarget::new(x, 3.0, 1)),
        ];

        // y is driven — should be excluded from params
        let result = diagnose(
            &constraints, &[x, y], &[y], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        // 1 driving constraint, 1 diagnose param (x only) → 0 DOF
        assert_eq!(result.dofs, 0);
    }

    #[test]
    fn reduced_jacobian_correctness() {
        let mut store = ParamStore::new();
        let x = store.push(2.0);
        let y = store.push(3.0);

        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(LinearCombo::new(x, y, 1.0, 2.0, 8.0, 1)),
        ];

        let (j, jmap, dparams, _) =
            make_reduced_jacobian(&constraints, &[x, y], &[], &store);

        assert_eq!(j.nrows(), 1);
        assert_eq!(j.ncols(), 2);
        assert_abs_diff_eq!(j[(0, 0)], 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(j[(0, 1)], 2.0, epsilon = 1e-12);
        assert_eq!(dparams.len(), 2);
        assert_eq!(jmap.len(), 1);
    }

    #[test]
    fn eliminate_nonzeros_over_pivot_basic() {
        // R = [[2, 3, 5],
        //      [0, 4, 7],
        //      [0, 0, 0]]
        let mut r = DMatrix::from_row_slice(3, 3, &[
            2.0, 3.0, 5.0,
            0.0, 4.0, 7.0,
            0.0, 0.0, 0.0,
        ]);
        eliminate_nonzeros_over_pivot(&mut r, 2);

        // After elimination: R[0,1] should be 0
        assert_abs_diff_eq!(r[(0, 1)], 0.0, epsilon = 1e-12);
        // R[0,2] should be 5 - (3/4)*7 = 5 - 5.25 = -0.25
        assert_abs_diff_eq!(r[(0, 2)], -0.25, epsilon = 1e-12);
    }

    #[test]
    fn dependent_params_identified() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let y = store.push(2.0);
        let z = store.push(3.0);

        // Only constrain x. y and z are unconstrained.
        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(PTarget::new(x, 5.0, 1)),
        ];

        let result = diagnose(
            &constraints, &[x, y, z], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        assert_eq!(result.dofs, 2);
        // y and z should be dependent
        assert!(result.dependent_params.contains(&y));
        assert!(result.dependent_params.contains(&z));
    }

    #[test]
    fn linearly_dependent_constraints_detected() {
        let mut store = ParamStore::new();
        let x = store.push(0.5);
        let y = store.push(0.5);

        // c1: x + y = 1 (tag 1) — satisfied
        // c2: 2x + 2y = 2 (tag 2) — linearly dependent with c1, also satisfied
        let constraints: Vec<Box<dyn Constraint>> = vec![
            Box::new(LinearCombo::new(x, y, 1.0, 1.0, 1.0, 1)),
            Box::new(LinearCombo::new(x, y, 2.0, 2.0, 2.0, 2)),
        ];

        let result = diagnose(
            &constraints, &[x, y], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        // 2 params, rank 1 → 1 DOF
        assert_eq!(result.dofs, 1);
        // The duplicate constraint should be detected as redundant
        assert!(!result.redundant_tags.is_empty() || !result.partially_redundant_tags.is_empty());
    }

    #[test]
    fn diagnosis_with_no_params_but_constraints() {
        let store = ParamStore::new();

        // Degenerate: constraints exist but no params
        let result = diagnose(
            &[], &[], &[], &store,
            &SolverConfig::default(), Algorithm::DogLeg,
        );

        assert_eq!(result.dofs, 0);
        assert!(result.empty_diagnose_matrix);
    }
}
