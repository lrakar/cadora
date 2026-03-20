# CADORA Solver Architecture

## Overview

CADORA is a Rust rewrite of FreeCAD's planegcs 2D geometric constraint solver.
The solver takes a set of geometric constraints and parameters, then finds
parameter values that satisfy all constraints simultaneously.

```
User builds sketch
        │
        ▼
┌─────────────────────────────────────────────────────┐
│                  cadora-system                       │
│  System: add constraints, declare unknowns, solve    │
│                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │
│  │ init_solution │  │    solve     │  │  diagnose  │ │
│  │  • partition  │  │  • per-comp  │  │  • DOF     │ │
│  │  • reduce     │  │  • dispatch  │  │  • conflicts│ │
│  └──────┬───────┘  └──────┬───────┘  └─────┬──────┘ │
└─────────┼──────────────────┼───────────────┼────────┘
          │                  │               │
          ▼                  ▼               ▼
  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
  │cadora-subsystem│ │cadora-solvers│ │cadora-diagnosis│
  │  SubSystem    │  │  BFGS       │  │  QR rank     │
  │  • Jacobian   │  │  LM         │  │  RREF        │
  │  • gradient   │  │  DogLeg     │  │  conflict    │
  │  • line search│  │  qp_eq      │  │  detection   │
  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘
         │                 │                  │
         ▼                 ▼                  ▼
  ┌──────────────────────────────────────────────────┐
  │              cadora-constraints                   │
  │  34 constraint types (Equal, P2PDistance, etc.)   │
  │  trait Constraint { error, grad, params, tag }    │
  └──────────────────────┬───────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         ▼               ▼               ▼
  ┌────────────┐  ┌────────────┐  ┌────────────┐
  │ cadora-geo │  │ cadora-math│  │ cadora-core│
  │ 10 geometry│  │ DeriVector2│  │ ParamStore │
  │ types      │  │ forward AD │  │ ParamIdx   │
  └────────────┘  └────────────┘  │ SolverConfig│
                                  └────────────┘
```

## Crate Dependency Graph

```
cadora-system
 ├── cadora-subsystem
 │    ├── cadora-constraints
 │    │    ├── cadora-geo
 │    │    │    ├── cadora-math
 │    │    │    │    └── cadora-core
 │    │    │    └── cadora-core
 │    │    ├── cadora-math
 │    │    └── cadora-core
 │    └── cadora-core
 ├── cadora-solvers
 │    ├── cadora-subsystem
 │    └── cadora-core
 ├── cadora-diagnosis
 │    ├── cadora-constraints
 │    ├── cadora-subsystem
 │    ├── cadora-solvers
 │    └── cadora-core
 └── cadora-core
```

## Core Concepts

### ParamStore & ParamIdx

All solver parameters live in a single `ParamStore` (a `Vec<f64>`).
Parameters are referenced by `ParamIdx(usize)` — a typed index that
replaces the raw `double*` pointers used in planegcs. This eliminates
pointer aliasing bugs and enables safe concurrent access patterns.

```rust
let mut store = ParamStore::new();
let x = store.push(0.0);  // ParamIdx(0)
let y = store.push(5.0);  // ParamIdx(1)
store.set(x, 3.14);
assert_eq!(store.get(x), 3.14);
```

### Constraints

Every constraint implements the `Constraint` trait:

```rust
pub trait Constraint {
    fn error(&self, store: &ParamStore) -> f64;
    fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64;
    fn params(&self) -> &[ParamIdx];
    fn tag(&self) -> Tag;
    fn is_driving(&self) -> bool;
}
```

- `error()` returns the scalar residual (zero when satisfied)
- `grad()` returns ∂error/∂param for a specific parameter
- `params()` lists all parameters this constraint depends on
- `tag()` identifies the constraint for diagnosis/removal
- `is_driving()` distinguishes equality constraints from objectives

### DeriVector2

Forward-mode automatic differentiation for 2D vectors. Used by geometry
and constraint implementations to compute exact analytical gradients.

```rust
let v = DeriVector2::new(store, point, deriv_param);
// v.x, v.dx, v.y, v.dy are value and derivative components
let len = v.length(store);  // returns (value, derivative) of ||v||
```

## Solve Pipeline

### 1. Build the System

```rust
let mut sys = System::new(store, SolverConfig::default());
sys.declare_unknowns(param_list);
sys.add_constraint(Box::new(ConstraintP2PDistance::new(p0, p1, dist, 1, true)));
// ... add more constraints
```

### 2. Initialize (init_solution)

`init_solution()` performs three steps:

1. **Graph Partitioning:** Uses `petgraph::UnionFind` to find connected
   components of the constraint-parameter bipartite graph. Independent
   sketch regions become separate components solved in isolation.

2. **Equality Reduction:** Scans for `Equal` constraints with ratio 1.0
   between unknown parameters. Merges them via union-find so that
   `x = y` eliminates one unknown, reducing the system size.

3. **Component Metadata:** Builds `ComponentInfo` structs listing each
   component's parameters, driving constraints, non-driving constraints,
   and reduction map.

### 3. Solve

`solve_with(is_fine, algorithm)` iterates over components:

- **Single-system components** (only driving or only non-driving
  constraints): Builds a `SubSystem`, calls the selected solver, applies
  the solution back to the global store.

- **Dual-system components** (both driving and non-driving): Currently
  solves driving constraints first, then non-driving. Full SQP dispatch
  is planned.

The `SubSystem` creates an isolated working copy of the parameter store.
Solvers modify only this copy; `apply_solution()` writes results back to
the global store when the solve succeeds.

### 4. Diagnose

`diagnose()` computes:

- **Degrees of Freedom (DOF):** rank(J) vs number of unknowns
- **Conflicting constraints:** Tags of constraints that cannot be
  simultaneously satisfied (detected via QR rank analysis + RREF)
- **Redundant constraints:** Tags of constraints that add no information
- **Dependent parameters:** Parameters that are fully determined

## Solver Algorithms

### BFGS (Broyden-Fletcher-Goldfarb-Shanno)

Quasi-Newton method using approximate inverse Hessian with rank-2 updates.
Uses line search for step size. Best for smooth, well-conditioned problems.

### Levenberg-Marquardt (LM)

Damped Gauss-Newton for nonlinear least squares. Interpolates between
gradient descent (high damping) and Gauss-Newton (low damping) based on
the gain ratio. Robust for over-determined systems.

### DogLeg

Trust-region method combining steepest descent and Gauss-Newton steps.
The "dog-leg" path interpolates between the two when the Gauss-Newton
step exceeds the trust region. Three Gauss-step variants:
- `FullPivLu`: Direct solve (square) or normal equations (non-square)
- `LeastNormFullPivLu`: Minimum-norm solution via J·J^T
- `LeastNormLdlt`: Same using Cholesky-like factorization

## Parameter Reduction

When Equal constraints merge parameters (e.g., `p0.x = p1.x`), the
SubSystem maps both to a single "slot" in the working vector. The
`slot_members` field tracks all original params per slot, ensuring:

- `set_params()` updates ALL merged params in the working store
- `calc_jacobi()` sums gradients over all slot members (chain rule)
- `apply_solution()` writes back to all merged params in the global store

This reduction decreases the effective system size and improves solver
convergence.

## Testing Strategy

- **Unit tests:** Each crate has focused tests for its public API
  (146 tests total across 8 crates)
- **Integration tests:** End-to-end sketch scenarios in
  `cadora-system/tests/integration.rs` (8 scenarios: rectangle,
  triangle, multi-component, over/under-constrained, re-solve, etc.)
- **Benchmarks:** Criterion benchmarks in
  `cadora-system/benches/solver_bench.rs` measuring solve time across
  chain, grid, and multi-component configurations

## Key Differences from planegcs

| Aspect | planegcs (C++) | CADORA (Rust) |
|--------|---------------|---------------|
| Memory safety | Raw double* pointers | ParamIdx + ParamStore |
| Linear algebra | Eigen (header-only) | nalgebra + faer |
| Graph algorithms | Boost.Graph | petgraph |
| Testing | None | 146 unit + 8 integration tests |
| Architecture | Monolithic GCS.cpp (4800 lines) | 8 focused crates |
| Constraint ownership | Shared pointers | Vec<Option<Box<dyn Constraint>>> |
| Thread safety | std::async | rayon-ready (Send + Sync) |
