# CADORA

**Open-source CAD with Fusion 360 UX — offline-first.**

CADORA is a ground-up open-source CAD application starting with a modern rewrite of FreeCAD's planegcs 2D geometric constraint solver in Rust.

## Solver Architecture

The solver is a modular Cargo workspace with 8 crates:

| Crate | Purpose |
|-------|---------|
| `cadora-core` | Parameter store, indices, solver enums, configuration |
| `cadora-math` | DeriVector2 forward-mode AD, vector math |
| `cadora-geo` | Geometry primitives (Line, Circle, Arc, Ellipse, BSpline, ...) |
| `cadora-constraints` | All 34 constraint types with error + gradient |
| `cadora-subsystem` | Partitioned subsystem: parameter scoping, Jacobian |
| `cadora-solvers` | BFGS, Levenberg-Marquardt, DogLeg, QP |
| `cadora-diagnosis` | QR rank analysis, DOF, conflict/redundancy detection |
| `cadora-system` | System orchestrator: graph partitioning, solve, diagnose |

See [docs/architecture.md](docs/architecture.md) for detailed design documentation.

## Quick Start

```rust
use cadora_core::{ParamStore, Point, SolverConfig};
use cadora_constraints::*;
use cadora_system::System;

let mut store = ParamStore::new();
let p0 = Point::new(store.push(0.0), store.push(0.0));
let p1 = Point::new(store.push(4.5), store.push(0.3));
let dist = store.push(5.0);

let mut sys = System::new(store, SolverConfig::default());
sys.declare_unknowns(vec![p1.x, p1.y]);
sys.add_constraint(Box::new(ConstraintP2PDistance::new(p0, p1, dist, 1, true)));

sys.init_solution();
sys.solve_default();
// p1 is now exactly 5.0 units from p0
```

## Key Design Decisions vs planegcs

| planegcs (C++) | CADORA (Rust) |
|----------------|---------------|
| Raw `double*` pointers | Safe `ParamIdx` indices into `ParamStore` |
| Eigen (header-only C++) | `nalgebra` (dense) + `faer` (sparse) |
| Boost.Graph | `petgraph` |
| `std::async` | `rayon` work-stealing |
| No unit tests | 146 unit tests + 8 integration tests |
| Monolithic GCS.cpp (4800 lines) | 8 focused crates |

## Progress

Open `docs/progress.html` in a browser to track porting progress.

## Building

```bash
cargo build --workspace
cargo test --workspace
```

## Benchmarks

```bash
cargo bench -p cadora-system --bench solver_bench
```

## Documentation

- [Architecture Guide](docs/architecture.md)
- [Parallelization Audit](docs/parallelization_audit.md)
- [Memory Audit](docs/memory_audit.md)
- [Progress Tracker](docs/progress.html) (open in browser)

## License

LGPL-2.1-or-later
