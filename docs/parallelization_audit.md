# Parallelization Audit

## Summary

CADORA declares `rayon` as a dependency for work-stealing parallelism.
This audit evaluates where parallelization adds value vs. overhead.

## Findings

### 1. Parallel Component Solving (OPPORTUNITY — future work)

When a sketch has multiple disconnected regions (components), each can be
solved independently. The current `System::solve_with` loop processes them
sequentially. Parallelizing this with `rayon::scope` or extracting all
component data up-front then using `par_iter` would scale linearly with
the number of independent groups.

**Why not implemented now:** The current architecture uses `&mut self` for
constraint extraction (`Option::take`), which prevents sharing across
threads. A refactor to extract all component constraints into owned
`SubSystem` objects first, then solve in parallel, then write back, would
enable this. The independent_triangles benchmark shows linear scaling at
~7µs per triangle — parallel solving would help for sketches with 10+
independent regions.

### 2. Inner-Loop Parallelism (NOT RECOMMENDED)

Operations like Jacobian computation, residual evaluation, and gradient
computation operate on small matrices (typical CAD sketches have 5-200
params). rayon's thread pool creation and work distribution overhead
(~1-5µs) exceeds the computation time for these small systems.

**Jacobian (calc_jacobi):** O(constraints × params) — typically < 10,000
entries. Sequential loop completes in < 1µs for typical sketches.

**Residual (calc_residual):** O(constraints) — trivial cost.

**Gradient (calc_grad):** Uses sparse p2c adjacency, already efficient.

### 3. Solver Internals (NOT RECOMMENDED)

BFGS, LM, and DogLeg solvers use nalgebra for matrix operations. nalgebra
internally uses rayon for large matrix multiplications (> 256×256), so
these benefit from parallelism automatically at scale without manual
intervention.

### 4. Diagnosis (MARGINAL)

QR decomposition and RREF in diagnosis could benefit from parallel column
operations for very large systems (1000+ constraints), but typical usage
is well under this threshold.

## Recommendations

1. **Short-term:** Keep sequential component solving; the overhead of
   parallelization doesn't justify the complexity for typical sketch sizes.
2. **Medium-term:** When sketch sizes routinely exceed 50+ independent
   components, refactor `solve_with` to extract all SubSystems first, then
   use `rayon::par_iter` for solving.
3. **Long-term:** nalgebra's internal parallelism handles large matrix
   operations; no manual SIMD or threading needed in solver internals.
