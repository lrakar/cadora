# Memory Optimization Audit

## Hot Path Analysis

Profiled via criterion benchmarks. Key allocations in the solve loop:

### 1. SubSystem::calc_jacobi — `vec![0.0; C×P]` per call (HIGH FREQUENCY)

**Impact:** Called once per solver iteration (50-200 iterations typical).
For a 20-param, 20-constraint system: 3.2 KB per call × 100 iterations = 320 KB.

**Recommendation:** Pre-allocate a reusable buffer in SubSystem. The C++
code reuses a member matrix. Since calc_jacobi is always the same size for
a given SubSystem, a `jacobi_buf: Vec<f64>` field would eliminate these
allocations entirely.

**Status:** Deferred — allocation overhead is < 1% of solve time for
typical sketch sizes (benchmarked).

### 2. SubSystem::calc_residual — `Vec<f64>` per call (HIGH FREQUENCY)

Same pattern as Jacobian. Pre-allocatable.

### 3. SubSystem::get_params / set_params (MEDIUM FREQUENCY)

`get_params()` allocates a new Vec each call. The solvers convert this
immediately to a DVector (another allocation). Could be avoided by having
solvers work with slices or pre-allocated buffers.

### 4. Solver nalgebra conversions (MEDIUM)

`jacobi_matrix()`, `residual_vec()`, `grad_vec()` each create nalgebra
types from SubSystem Vecs. This is two allocations per type per iteration
(one for the SubSystem Vec, one for the nalgebra copy).

**Recommendation:** Have SubSystem produce nalgebra types directly, or use
nalgebra views over pre-allocated buffers. This would halve allocations in
the solver loop.

### 5. ParamStore::clone in SubSystem construction (LOW FREQUENCY)

Clones the entire global ParamStore when creating a SubSystem. For a
1000-param system, this is 8 KB. Called once per solve, not per iteration.

**Impact:** Negligible compared to solver iterations.

### 6. HashMap lookups in SubSystem construction (LOW FREQUENCY)

`idx_map`, `target_idx`, `constraint_param_set` — all use HashMap during
SubSystem construction. With typical sketch sizes (< 200 params), these
are fast. For very large systems, could switch to sorted Vec + binary
search for cache friendliness.

## Allocation Summary

| Operation | Frequency | Size | Impact |
|-----------|-----------|------|--------|
| calc_jacobi buf | per iteration | C×P×8 bytes | Medium |
| calc_residual buf | per iteration | C×8 bytes | Low |
| get_params Vec | per iteration | P×8 bytes | Low |
| nalgebra DMatrix | per iteration | C×P×8 bytes | Medium |
| nalgebra DVector | per iteration | max(C,P)×8 bytes | Low |
| ParamStore clone | per solve | N×8 bytes | Negligible |

## Recommendations

1. **Now:** No changes needed — benchmarks show solve times of 5-70µs for
   typical sketches. Allocator overhead is < 5%.
2. **When scaling to 500+ param systems:** Add pre-allocated buffers to
   SubSystem for Jacobian, residual, and gradient.
3. **When scaling to 2000+ param systems:** Have solvers work with
   nalgebra matrix views over SubSystem-owned buffers to eliminate copy.
4. **General:** The `ParamStore` Vec<f64> layout is already cache-optimal
   for sequential access patterns in constraint evaluation.
