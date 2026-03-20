//! # cadora-system
//!
//! System orchestrator: graph partitioning, initSolution, solve, diagnose.
//! Port of planegcs `System` class from `GCS.h` / `GCS.cpp`.

use std::collections::{HashMap, HashSet};

use petgraph::unionfind::UnionFind;

use cadora_constraints::Constraint;
use cadora_core::{
    Algorithm, ParamIdx, ParamStore, ReductionMap, SolveStatus, SolverConfig,
};
use cadora_diagnosis::{diagnose, DiagnosisResult};
use cadora_solvers::{solve, solve_sqp};
use cadora_subsystem::SubSystem;

// ---------------------------------------------------------------------------
// Component info (partitioning metadata)
// ---------------------------------------------------------------------------

/// Metadata for one connected component of the constraint graph.
struct ComponentInfo {
    /// Parameters in this component.
    params: Vec<ParamIdx>,
    /// Indices into `System::constraints` for driving constraints (tag >= 0, is_driving).
    driving_indices: Vec<usize>,
    /// Indices into `System::constraints` for non-driving constraints (!is_driving).
    nondriving_indices: Vec<usize>,
    /// Parameter reduction map built from Equal constraints in this component.
    reduction: ReductionMap,
}

// ---------------------------------------------------------------------------
// System
// ---------------------------------------------------------------------------

/// The main constraint system orchestrator.
///
/// Port of `GCS::System` from FreeCAD's planegcs.  Manages constraints and
/// parameters, partitions the system into connected components, builds
/// reduction maps from equality constraints, and dispatches to solvers and
/// diagnosis.
pub struct System {
    /// Central parameter store (ground truth).
    store: ParamStore,
    /// Solver configuration.
    config: SolverConfig,
    /// Default algorithm for solve dispatch.
    algorithm: Algorithm,

    // -- Constraint & parameter management --
    /// All constraints.  `Option` enables temporary extraction during solve.
    constraints: Vec<Option<Box<dyn Constraint>>>,
    /// Declared unknown parameters.
    param_list: Vec<ParamIdx>,
    /// Parameters that belong to non-driving constraints only.
    driven_param_set: HashSet<ParamIdx>,

    // -- Initialization state --
    is_init: bool,
    has_diagnosis: bool,

    // -- Partitioned data (populated by init_solution) --
    components: Vec<ComponentInfo>,

    // -- Reference snapshot for resetToReference --
    reference: Option<Vec<f64>>,

    // -- Cached diagnosis --
    diagnosis_result: Option<DiagnosisResult>,
}

impl System {
    /// Create a new empty system.
    pub fn new(store: ParamStore, config: SolverConfig) -> Self {
        Self {
            store,
            config,
            algorithm: Algorithm::default(),
            constraints: Vec::new(),
            param_list: Vec::new(),
            driven_param_set: HashSet::new(),
            is_init: false,
            has_diagnosis: false,
            components: Vec::new(),
            reference: None,
            diagnosis_result: None,
        }
    }

    /// Set the default solver algorithm.
    pub fn set_algorithm(&mut self, alg: Algorithm) {
        self.algorithm = alg;
    }

    /// Access the parameter store.
    pub fn store(&self) -> &ParamStore {
        &self.store
    }

    /// Mutable access to the parameter store.
    pub fn store_mut(&mut self) -> &mut ParamStore {
        &mut self.store
    }

    /// Access the solver configuration.
    pub fn config(&self) -> &SolverConfig {
        &self.config
    }

    /// Mutable access to the solver configuration.
    pub fn config_mut(&mut self) -> &mut SolverConfig {
        &mut self.config
    }

    /// Whether the system has been initialized (init_solution called).
    pub fn is_initialized(&self) -> bool {
        self.is_init
    }

    // -----------------------------------------------------------------------
    // Parameter management
    // -----------------------------------------------------------------------

    /// Declare the unknown parameters for the system.
    /// Equivalent to C++ `declareUnknowns`.
    pub fn declare_unknowns(&mut self, params: Vec<ParamIdx>) {
        self.param_list = params;
        self.is_init = false;
    }

    /// Get the unknown parameter list.
    pub fn param_list(&self) -> &[ParamIdx] {
        &self.param_list
    }

    // -----------------------------------------------------------------------
    // Constraint management
    // -----------------------------------------------------------------------

    /// Add a constraint to the system.
    ///
    /// Returns the index of the added constraint.
    /// Invalidates initialization and diagnosis.
    pub fn add_constraint(&mut self, constraint: Box<dyn Constraint>) -> usize {
        self.is_init = false;
        if constraint.tag() >= 0 {
            self.has_diagnosis = false;
        }
        if !constraint.is_driving() {
            for &p in constraint.params() {
                self.driven_param_set.insert(p);
            }
        }
        let idx = self.constraints.len();
        self.constraints.push(Some(constraint));
        idx
    }

    /// Remove all constraints with the given tag.
    pub fn clear_by_tag(&mut self, tag_id: i32) {
        self.constraints.retain(|opt| {
            opt.as_ref().is_some_and(|c| c.tag() != tag_id)
        });
        self.is_init = false;
        self.has_diagnosis = false;
    }

    /// Number of constraints.
    pub fn constraint_count(&self) -> usize {
        self.constraints.iter().filter(|c| c.is_some()).count()
    }

    // -----------------------------------------------------------------------
    // Graph partitioning & reduction
    // -----------------------------------------------------------------------

    /// Build connected components of the constraint-parameter bipartite graph.
    ///
    /// Uses union-find (equivalent to C++ boost::connected_components).
    /// Returns: (component_id_per_constraint, component_id_per_param, num_components)
    fn partition_graph(&self) -> (Vec<usize>, HashMap<ParamIdx, usize>, usize) {
        // Assign IDs:  params first [0..plist.len()), then constraints [plist.len()..)
        let n_params = self.param_list.len();
        let active_constraints: Vec<(usize, &Box<dyn Constraint>)> = self.constraints.iter()
            .enumerate()
            .filter_map(|(i, opt)| opt.as_ref().map(|c| (i, c)))
            .collect();
        let n_total = n_params + active_constraints.len();

        if n_total == 0 {
            return (vec![], HashMap::new(), 0);
        }

        let mut uf = UnionFind::<usize>::new(n_total);

        // Map each ParamIdx to its position in param_list
        let param_pos: HashMap<ParamIdx, usize> = self.param_list.iter()
            .enumerate()
            .map(|(i, &p)| (p, i))
            .collect();

        // Union each constraint with its parameters
        for (ci, (_orig_idx, c)) in active_constraints.iter().enumerate() {
            let constraint_node = n_params + ci;
            for &p in c.params() {
                if let Some(&pi) = param_pos.get(&p) {
                    uf.union(pi, constraint_node);
                }
            }
        }

        // Extract labels
        let labels = uf.into_labeling();

        // Remap labels to contiguous 0..num_components
        let mut label_map: HashMap<usize, usize> = HashMap::new();
        let mut next_id = 0usize;
        let mut constraint_components = Vec::with_capacity(active_constraints.len());
        let mut param_components: HashMap<ParamIdx, usize> = HashMap::new();

        for (i, &p) in self.param_list.iter().enumerate() {
            let comp = *label_map.entry(labels[i]).or_insert_with(|| {
                let id = next_id;
                next_id += 1;
                id
            });
            param_components.insert(p, comp);
        }

        for (ci, _) in active_constraints.iter().enumerate() {
            let comp = *label_map.entry(labels[n_params + ci]).or_insert_with(|| {
                let id = next_id;
                next_id += 1;
                id
            });
            constraint_components.push(comp);
        }

        (constraint_components, param_components, next_id)
    }

    /// Build reduction maps from Equal constraints.
    ///
    /// For each driving Equal constraint with ratio == 1.0, merges the two
    /// parameters (the second is replaced by the first).
    ///
    /// Returns `(reduction_maps, consumed_constraint_indices)`.
    fn build_reduction_maps(
        &self,
        num_components: usize,
        constraint_components: &[usize],
    ) -> (Vec<ReductionMap>, HashSet<usize>) {
        let mut reductions = vec![ReductionMap::new(); num_components];
        let mut consumed = HashSet::new();

        let active: Vec<&Box<dyn Constraint>> = self.constraints.iter()
            .filter_map(|opt| opt.as_ref())
            .collect();

        // Build initial one-to-one reduction from Equal constraints.
        // Uses union-find-style chaining: if a→b and b→c, then a→c and b→c.
        let mut reduced_to: HashMap<ParamIdx, ParamIdx> = HashMap::new();

        // Only reduce params that are actually unknowns
        let unknown_set: HashSet<ParamIdx> = self.param_list.iter().copied().collect();

        for (ci, c) in active.iter().enumerate() {
            if !c.is_driving() || c.tag() < 0 {
                continue;
            }
            let params = c.params();
            if params.len() != 2 {
                continue;
            }
            // Both params must be unknowns for reduction to apply
            if !unknown_set.contains(&params[0]) || !unknown_set.contains(&params[1]) {
                continue;
            }
            // Check if the constraint has equality-like gradients:
            // grad(p1) = +scale, grad(p2) = -scale for some nonzero scale.
            let g1 = c.grad(&self.store, params[0]);
            let g2 = c.grad(&self.store, params[1]);
            if (g1 + g2).abs() > 1e-12 || g1.abs() < 1e-12 {
                continue; // Not an equality constraint
            }

            let comp = constraint_components[ci];

            // Follow chains to find the ultimate representative
            let mut p_kept = params[0];
            while let Some(&target) = reduced_to.get(&p_kept) {
                p_kept = target;
            }
            let mut p_replaced = params[1];
            while let Some(&target) = reduced_to.get(&p_replaced) {
                p_replaced = target;
            }

            if p_kept != p_replaced {
                reduced_to.insert(p_replaced, p_kept);
                reductions[comp].insert(p_replaced, p_kept);
                consumed.insert(ci);
            }
        }

        // Flatten chains: ensure all entries point directly to the representative
        for reduction in &mut reductions {
            let keys: Vec<ParamIdx> = reduction.keys().copied().collect();
            for k in keys {
                let mut target = reduction[&k];
                while let Some(&next) = reduction.get(&target) {
                    if next == target { break; }
                    target = next;
                }
                reduction.insert(k, target);
            }
        }

        (reductions, consumed)
    }

    // -----------------------------------------------------------------------
    // initSolution
    // -----------------------------------------------------------------------

    /// Initialize the solution: partition the constraint graph into connected
    /// components, build reduction maps from equality constraints, and prepare
    /// component metadata for solve dispatch.
    ///
    /// Port of C++ `System::initSolution()`.
    pub fn init_solution(&mut self) {
        // Save reference snapshot
        self.reference = Some(self.store.snapshot());

        // Get active constraint indices
        let active_indices: Vec<usize> = self.constraints.iter()
            .enumerate()
            .filter_map(|(i, opt)| opt.as_ref().map(|_| i))
            .collect();

        // Partition
        let (constraint_components, param_components, num_components) =
            self.partition_graph();

        // Build reduction maps (also returns indices of consumed Equal constraints)
        let (reductions, consumed_indices) =
            self.build_reduction_maps(num_components, &constraint_components);

        // Build component info
        let mut components: Vec<ComponentInfo> = (0..num_components)
            .map(|_| ComponentInfo {
                params: Vec::new(),
                driving_indices: Vec::new(),
                nondriving_indices: Vec::new(),
                reduction: ReductionMap::new(),
            })
            .collect();

        // Assign params to components
        for (&p, &comp) in &param_components {
            if comp < components.len() {
                components[comp].params.push(p);
            }
        }
        // Sort params for deterministic ordering
        for comp in &mut components {
            comp.params.sort();
        }

        // Assign constraints to components, excluding those consumed by reduction
        for (ci, &orig_idx) in active_indices.iter().enumerate() {
            if ci < constraint_components.len() && !consumed_indices.contains(&ci) {
                let comp_id = constraint_components[ci];
                let c = self.constraints[orig_idx].as_ref().unwrap();
                if c.is_driving() {
                    components[comp_id].driving_indices.push(orig_idx);
                } else {
                    components[comp_id].nondriving_indices.push(orig_idx);
                }
            }
        }

        // Assign reduction maps
        for (i, reduction) in reductions.into_iter().enumerate() {
            if i < components.len() {
                components[i].reduction = reduction;
            }
        }

        self.components = components;
        self.is_init = true;
    }

    // -----------------------------------------------------------------------
    // solve
    // -----------------------------------------------------------------------

    /// Solve the constraint system using the default algorithm.
    pub fn solve_default(&mut self) -> SolveStatus {
        self.solve_with(true, self.algorithm)
    }

    /// Solve with explicit parameters.
    ///
    /// Equivalent to C++ `System::solve(params, isFine, alg, isRedundant)`.
    pub fn solve_params(
        &mut self,
        params: Vec<ParamIdx>,
        is_fine: bool,
        alg: Algorithm,
    ) -> SolveStatus {
        self.declare_unknowns(params);
        self.init_solution();
        self.solve_with(is_fine, alg)
    }

    /// Main solve dispatch.  Iterates over connected components and solves
    /// each one.
    ///
    /// Port of C++ `System::solve(bool isFine, Algorithm alg, bool isRedundant)`.
    pub fn solve_with(&mut self, _is_fine: bool, alg: Algorithm) -> SolveStatus {
        if !self.is_init {
            return SolveStatus::Failed;
        }

        let mut worst = SolveStatus::Success;

        // Process each connected component independently.
        // Components share no parameters, so solving one does not affect others.
        let num_components = self.components.len();
        for comp_idx in 0..num_components {
            let has_driving = !self.components[comp_idx].driving_indices.is_empty();
            let has_nondriving = !self.components[comp_idx].nondriving_indices.is_empty();

            let status = if has_driving && has_nondriving {
                self.solve_dual_component(comp_idx, alg)
            } else if has_driving {
                self.solve_single_component(comp_idx, alg, true)
            } else if has_nondriving {
                self.solve_single_component(comp_idx, alg, false)
            } else {
                SolveStatus::Success
            };

            if (status as u8) > (worst as u8) {
                worst = status;
            }
        }

        worst
    }

    /// Solve a single-system component (either driving or non-driving constraints).
    fn solve_single_component(
        &mut self,
        comp_idx: usize,
        alg: Algorithm,
        is_driving: bool,
    ) -> SolveStatus {
        let comp = &self.components[comp_idx];
        let indices = if is_driving {
            comp.driving_indices.clone()
        } else {
            comp.nondriving_indices.clone()
        };
        let params = comp.params.clone();
        let reduction = comp.reduction.clone();

        // Take constraints out
        let subsys_constraints: Vec<Box<dyn Constraint>> = indices.iter()
            .map(|&i| self.constraints[i].take().unwrap())
            .collect();

        // Create SubSystem
        let mut subsys = SubSystem::new_with_reduction(
            subsys_constraints,
            &params,
            &reduction,
            &self.store,
        );

        // Solve
        let status = solve(&mut subsys, &self.config, alg, false);

        // Apply solution back to global store
        subsys.apply_solution(&mut self.store);

        // Put constraints back
        let returned = subsys.into_constraints();
        for (c, &idx) in returned.into_iter().zip(indices.iter()) {
            self.constraints[idx] = Some(c);
        }

        status
    }

    /// Solve a dual-system component (driving = equality constraints,
    /// non-driving = objective to minimize).
    ///
    /// Uses SQP to minimize non-driving objective subject to driving equality constraints.
    fn solve_dual_component(
        &mut self,
        comp_idx: usize,
        _alg: Algorithm,
    ) -> SolveStatus {
        let comp = &self.components[comp_idx];
        let driving_indices = comp.driving_indices.clone();
        let nondriving_indices = comp.nondriving_indices.clone();
        let params = comp.params.clone();
        let reduction = comp.reduction.clone();

        // Take constraints out for both subsystems
        let driving_constraints: Vec<Box<dyn Constraint>> = driving_indices.iter()
            .map(|&i| self.constraints[i].take().unwrap())
            .collect();
        let nondriving_constraints: Vec<Box<dyn Constraint>> = nondriving_indices.iter()
            .map(|&i| self.constraints[i].take().unwrap())
            .collect();

        // Create both SubSystems
        let mut subsys_a = SubSystem::new_with_reduction(
            driving_constraints,
            &params,
            &reduction,
            &self.store,
        );
        let mut subsys_b = SubSystem::new_with_reduction(
            nondriving_constraints,
            &params,
            &reduction,
            &self.store,
        );

        // Solve using SQP
        let status = solve_sqp(&mut subsys_a, &mut subsys_b, &self.config, false);

        // Apply solution from subsys_a (it holds the authoritative params)
        subsys_a.apply_solution(&mut self.store);

        // Put constraints back
        let returned_a = subsys_a.into_constraints();
        for (c, &idx) in returned_a.into_iter().zip(driving_indices.iter()) {
            self.constraints[idx] = Some(c);
        }
        let returned_b = subsys_b.into_constraints();
        for (c, &idx) in returned_b.into_iter().zip(nondriving_indices.iter()) {
            self.constraints[idx] = Some(c);
        }

        status
    }

    // -----------------------------------------------------------------------
    // Diagnosis
    // -----------------------------------------------------------------------

    /// Run diagnosis on the constraint system.
    ///
    /// Returns DOF count, conflicting/redundant constraints, dependent parameters.
    pub fn diagnose(&mut self) -> DiagnosisResult {
        // Collect all constraints temporarily
        let owned: Vec<Box<dyn Constraint>> = self.constraints.iter_mut()
            .filter_map(|opt| opt.take())
            .collect();

        let driven_params: Vec<ParamIdx> = self.driven_param_set.iter().copied().collect();

        let result = diagnose(
            &owned,
            &self.param_list,
            &driven_params,
            &self.store,
            &self.config,
            self.algorithm,
        );

        // Put constraints back (order preserved by filter_map iteration order)
        // We need to put them back in the right slots.
        // Since we took them in order (filter_map preserves iteration order),
        // we put them back in the same slots.
        let mut owned_iter = owned.into_iter();
        for opt in &mut self.constraints {
            if opt.is_none() {
                // This slot was taken
                if let Some(c) = owned_iter.next() {
                    *opt = Some(c);
                }
            }
        }

        self.has_diagnosis = true;
        self.diagnosis_result = Some(result.clone());
        result
    }

    /// Get the cached diagnosis result (if available).
    pub fn diagnosis_result(&self) -> Option<&DiagnosisResult> {
        self.diagnosis_result.as_ref()
    }

    // -----------------------------------------------------------------------
    // Reset / clear
    // -----------------------------------------------------------------------

    /// Clear all subsystem/partitioning state (requires re-init before solve).
    pub fn clear_sub_systems(&mut self) {
        self.is_init = false;
        self.components.clear();
    }

    /// Clear everything: constraints, params, diagnosis state.
    pub fn clear(&mut self) {
        self.constraints.clear();
        self.param_list.clear();
        self.driven_param_set.clear();
        self.is_init = false;
        self.has_diagnosis = false;
        self.components.clear();
        self.reference = None;
        self.diagnosis_result = None;
    }

    /// Reset parameter values to the reference snapshot (if available).
    pub fn reset_to_reference(&mut self) {
        if let Some(ref snap) = self.reference {
            self.store.restore(snap);
        }
    }

    /// Undo the last solution by restoring the reference snapshot.
    pub fn undo_solution(&mut self) {
        self.reset_to_reference();
    }

    /// Remove a constraint at a given slot index.
    /// Invalidates initialization state, requiring `init_solution()` before the next solve.
    pub fn remove_constraint(&mut self, slot: usize) {
        if slot < self.constraints.len() {
            self.constraints[slot] = None;
            self.is_init = false;
            self.has_diagnosis = false;
            self.components.clear();
        }
    }

    /// Evaluate all driven (non-driving) constraints, updating their value parameters
    /// to reflect the current geometry.
    pub fn evaluate_driven_constraints(&mut self) {
        for opt in &self.constraints {
            if let Some(c) = opt {
                if !c.is_driving() {
                    c.evaluate(&mut self.store);
                }
            }
        }
    }

    /// Rescale a specific constraint by re-reading the current geometry.
    pub fn rescale_constraint(&mut self, slot: usize) {
        if slot < self.constraints.len() {
            if let Some(c) = &mut self.constraints[slot] {
                c.rescale(&self.store);
            }
        }
    }

    /// Calculate the constraint error for all constraints with a given tag.
    pub fn calculate_constraint_error_by_tag(&self, tag_id: i32) -> f64 {
        let mut sum = 0.0;
        let mut count = 0usize;
        for opt in &self.constraints {
            if let Some(c) = opt {
                if c.tag() == tag_id {
                    let e = c.error(&self.store);
                    sum += e * e;
                    count += 1;
                }
            }
        }
        if count == 1 {
            sum.sqrt().copysign(
                self.constraints.iter()
                    .filter_map(|o| o.as_ref())
                    .find(|c| c.tag() == tag_id)
                    .unwrap()
                    .error(&self.store),
            )
        } else if count > 1 {
            (sum / count as f64).sqrt()
        } else {
            0.0
        }
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
    use cadora_core::{ParamIdx, ParamStore, Tag};

    // -- Test constraint: error = x - target --------------------------------
    struct ParamTarget {
        pvec: Vec<ParamIdx>,
        target: f64,
        tag: Tag,
        driving: bool,
    }

    impl ParamTarget {
        fn new(p: ParamIdx, target: f64, tag: Tag) -> Self {
            Self {
                pvec: vec![p],
                target,
                tag,
                driving: true,
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
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> Tag { self.tag }
        fn is_driving(&self) -> bool { self.driving }
    }

    // -- Test constraint: error = x - y (equality) -------------------------
    struct EqualParams {
        pvec: Vec<ParamIdx>,
        tag: Tag,
    }

    impl EqualParams {
        fn new(p1: ParamIdx, p2: ParamIdx, tag: Tag) -> Self {
            Self { pvec: vec![p1, p2], tag }
        }
    }

    impl Constraint for EqualParams {
        fn error(&self, store: &ParamStore) -> f64 {
            store.get(self.pvec[0]) - store.get(self.pvec[1])
        }
        fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] { 1.0 }
            else if param == self.pvec[1] { -1.0 }
            else { 0.0 }
        }
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> Tag { self.tag }
        fn is_driving(&self) -> bool { true }
    }

    // -- Test constraint: nonlinear x^2 = target ----------------------------
    struct ParamSquaredTarget {
        pvec: Vec<ParamIdx>,
        target: f64,
        tag: Tag,
    }

    impl ParamSquaredTarget {
        fn new(p: ParamIdx, target: f64, tag: Tag) -> Self {
            Self { pvec: vec![p], target, tag }
        }
    }

    impl Constraint for ParamSquaredTarget {
        fn error(&self, store: &ParamStore) -> f64 {
            let x = store.get(self.pvec[0]);
            x * x - self.target
        }
        fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] {
                2.0 * store.get(self.pvec[0])
            } else {
                0.0
            }
        }
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> Tag { self.tag }
        fn is_driving(&self) -> bool { true }
    }

    #[test]
    fn empty_system_succeeds() {
        let store = ParamStore::new();
        let mut sys = System::new(store, SolverConfig::default());
        sys.init_solution();
        assert_eq!(sys.solve_default(), SolveStatus::Success);
    }

    #[test]
    fn single_constraint_solve() {
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x]);
        sys.add_constraint(Box::new(ParamTarget::new(x, 5.0, 1)));
        sys.init_solution();
        let status = sys.solve_default();
        assert_eq!(status, SolveStatus::Success);
        assert_abs_diff_eq!(sys.store().get(x), 5.0, epsilon = 1e-8);
    }

    #[test]
    fn two_independent_components() {
        // Two unrelated constraints form separate components
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let y = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x, y]);
        sys.add_constraint(Box::new(ParamTarget::new(x, 3.0, 1)));
        sys.add_constraint(Box::new(ParamTarget::new(y, 7.0, 2)));
        sys.init_solution();
        let status = sys.solve_default();
        assert_eq!(status, SolveStatus::Success);
        assert_abs_diff_eq!(sys.store().get(x), 3.0, epsilon = 1e-8);
        assert_abs_diff_eq!(sys.store().get(y), 7.0, epsilon = 1e-8);
    }

    #[test]
    fn coupled_constraints_single_component() {
        // Two constraints sharing a parameter form one component
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let y = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x, y]);
        // x = 2 and x = y → both should converge to 2
        sys.add_constraint(Box::new(ParamTarget::new(x, 2.0, 1)));
        sys.add_constraint(Box::new(EqualParams::new(x, y, 2)));
        sys.init_solution();
        let status = sys.solve_default();
        assert_eq!(status, SolveStatus::Success);
        assert_abs_diff_eq!(sys.store().get(x), 2.0, epsilon = 1e-8);
        assert_abs_diff_eq!(sys.store().get(y), 2.0, epsilon = 1e-8);
    }

    #[test]
    fn equality_reduction() {
        // Equal constraint should reduce parameters
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let y = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x, y]);
        sys.add_constraint(Box::new(EqualParams::new(x, y, 1)));
        sys.add_constraint(Box::new(ParamTarget::new(x, 4.0, 2)));
        sys.init_solution();

        // After init, the reduction map should merge x and y
        assert!(!sys.components.is_empty());
        let comp = &sys.components[0];
        // The reduction should map y → x (or x → y)
        assert!(!comp.reduction.is_empty());

        let status = sys.solve_default();
        assert_eq!(status, SolveStatus::Success);
        assert_abs_diff_eq!(sys.store().get(x), 4.0, epsilon = 1e-8);
        assert_abs_diff_eq!(sys.store().get(y), 4.0, epsilon = 1e-8);
    }

    #[test]
    fn nonlinear_solve() {
        let mut store = ParamStore::new();
        let x = store.push(5.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x]);
        sys.add_constraint(Box::new(ParamSquaredTarget::new(x, 9.0, 1)));
        sys.init_solution();
        let status = sys.solve_default();
        assert_eq!(status, SolveStatus::Success);
        assert_abs_diff_eq!(sys.store().get(x), 3.0, epsilon = 1e-6);
    }

    #[test]
    fn clear_by_tag_removes_constraints() {
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.add_constraint(Box::new(ParamTarget::new(x, 5.0, 1)));
        sys.add_constraint(Box::new(ParamTarget::new(x, 10.0, 2)));
        assert_eq!(sys.constraint_count(), 2);
        sys.clear_by_tag(1);
        assert_eq!(sys.constraint_count(), 1);
    }

    #[test]
    fn solve_with_multiple_algorithms() {
        for alg in [Algorithm::Bfgs, Algorithm::LevenbergMarquardt, Algorithm::DogLeg] {
            let mut store = ParamStore::new();
            let x = store.push(0.0);
            let mut sys = System::new(store, SolverConfig::default());
            sys.declare_unknowns(vec![x]);
            sys.add_constraint(Box::new(ParamTarget::new(x, 7.0, 1)));
            sys.init_solution();
            let status = sys.solve_with(true, alg);
            assert_eq!(status, SolveStatus::Success, "Failed with {:?}", alg);
            assert_abs_diff_eq!(sys.store().get(x), 7.0, epsilon = 1e-8);
        }
    }

    #[test]
    fn solve_params_shorthand() {
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.add_constraint(Box::new(ParamTarget::new(x, 3.0, 1)));
        let status = sys.solve_params(vec![x], true, Algorithm::DogLeg);
        assert_eq!(status, SolveStatus::Success);
        assert_abs_diff_eq!(sys.store().get(x), 3.0, epsilon = 1e-8);
    }

    #[test]
    fn diagnosis_basic() {
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let y = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x, y]);
        // Only one constraint on two unknowns → 1 DOF
        sys.add_constraint(Box::new(ParamTarget::new(x, 3.0, 1)));
        let result = sys.diagnose();
        assert_eq!(result.dofs, 1);
    }

    #[test]
    fn clear_resets_everything() {
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x]);
        sys.add_constraint(Box::new(ParamTarget::new(x, 5.0, 1)));
        sys.init_solution();
        sys.solve_default();
        sys.clear();
        assert_eq!(sys.constraint_count(), 0);
        assert!(sys.param_list().is_empty());
        assert!(!sys.is_initialized());
    }

    #[test]
    fn reset_to_reference() {
        let mut store = ParamStore::new();
        let x = store.push(10.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x]);
        sys.add_constraint(Box::new(ParamTarget::new(x, 0.0, 1)));
        sys.init_solution();
        sys.solve_default();
        // x should now be ~0
        assert_abs_diff_eq!(sys.store().get(x), 0.0, epsilon = 1e-8);
        // Reset to reference should bring it back to 10
        sys.reset_to_reference();
        assert_abs_diff_eq!(sys.store().get(x), 10.0, epsilon = 1e-8);
    }

    #[test]
    fn undo_solution() {
        let mut store = ParamStore::new();
        let x = store.push(5.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x]);
        sys.add_constraint(Box::new(ParamTarget::new(x, 0.0, 1)));
        sys.init_solution();
        sys.solve_default();
        assert_abs_diff_eq!(sys.store().get(x), 0.0, epsilon = 1e-8);
        sys.undo_solution();
        assert_abs_diff_eq!(sys.store().get(x), 5.0, epsilon = 1e-8);
    }

    #[test]
    fn remove_constraint_and_re_solve() {
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let mut sys = System::new(store, SolverConfig::default());
        sys.declare_unknowns(vec![x]);
        let slot0 = sys.add_constraint(Box::new(ParamTarget::new(x, 3.0, 1)));
        let slot1 = sys.add_constraint(Box::new(ParamTarget::new(x, 7.0, 2)));
        sys.init_solution();
        sys.solve_default();
        // With conflicting constraints, result is somewhere between 3 and 7
        // Remove the second constraint and re-solve → should converge to 3
        sys.remove_constraint(slot1);
        sys.declare_unknowns(vec![x]);
        sys.init_solution();
        sys.solve_default();
        assert_abs_diff_eq!(sys.store().get(x), 3.0, epsilon = 1e-8);
        let _ = slot0; // used to verify add_constraint returns index
    }

    #[test]
    fn calculate_constraint_error_by_tag_test() {
        let mut store = ParamStore::new();
        let x = store.push(5.0); // not at target
        let mut sys = System::new(store, SolverConfig::default());
        // x should be at 3 → error = 5-3=2
        sys.add_constraint(Box::new(ParamTarget::new(x, 3.0, 42)));
        let err = sys.calculate_constraint_error_by_tag(42);
        assert_abs_diff_eq!(err, 2.0, epsilon = 1e-10);
        // No constraints with tag 99
        assert_abs_diff_eq!(sys.calculate_constraint_error_by_tag(99), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn rescale_constraint_test() {
        let mut store = ParamStore::new();
        let x = store.push(1.0);
        let mut sys = System::new(store, SolverConfig::default());
        let slot = sys.add_constraint(Box::new(ParamTarget::new(x, 5.0, 1)));
        // Rescale should not panic
        sys.rescale_constraint(slot);
    }
}
