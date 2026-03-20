# CADORA

**Open-source CAD with Fusion 360 UX — offline-first.**

CADORA is a ground-up open-source CAD application starting with a modern rewrite of FreeCAD's planegcs 2D geometric constraint solver in Rust.

## Solver Architecture

The solver is a modular Cargo workspace:

| Crate | Purpose |
|-------|---------|
| `cadora-core` | Parameter store, indices, solver enums, configuration |
| `cadora-math` | DeriVector2 forward-mode AD, vector math |
| `cadora-geo` | Geometry primitives (Line, Circle, Arc, Ellipse, BSpline, ...) |
| `cadora-constraints` | All 36 constraint types with error + gradient |
| `cadora-subsystem` | Partitioned subsystem: parameter scoping, Jacobian |
| `cadora-solvers` | BFGS, Levenberg-Marquardt, DogLeg, line search, SQP |
| `cadora-diagnosis` | QR rank analysis, DOF, conflict/redundancy detection |
| `cadora-system` | System orchestrator: graph partitioning, solve, diagnose |

## Key Design Decisions vs planegcs

| planegcs (C++) | CADORA (Rust) |
|----------------|---------------|
| Raw `double*` pointers | Safe `ParamIdx` indices into `ParamStore` |
| Eigen (header-only C++) | `nalgebra` (dense) + `faer` (sparse) |
| Boost.Graph | `petgraph` |
| `std::async` | `rayon` work-stealing |
| No unit tests | Every module tested + cross-validated against C++ |
| Global mutable state | Immutable parameter store passed by reference |

## Progress

Open `docs/progress.html` in a browser to track porting progress.

## Building

```bash
cargo build --workspace
cargo test --workspace
```

## License

LGPL-2.1-or-later
