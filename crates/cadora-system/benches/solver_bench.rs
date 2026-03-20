//! Performance benchmarks for the CADORA solver.
//!
//! Run with: `cargo bench -p cadora-system`

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};

use cadora_constraints::*;
use cadora_core::{Algorithm, ParamIdx, ParamStore, Point, SolverConfig};
use cadora_system::System;

fn mk_point(store: &mut ParamStore, x: f64, y: f64) -> (Point, Vec<ParamIdx>) {
    let px = store.push(x);
    let py = store.push(y);
    (Point::new(px, py), vec![px, py])
}

/// Build a chain of N points connected by distance constraints.
/// Point 0 is fixed at origin, each subsequent point is distance 1 from the previous.
fn build_chain(n: usize) -> System {
    let mut store = ParamStore::new();
    let mut points = Vec::new();
    let mut params = Vec::new();

    for i in 0..n {
        let (p, pv) = mk_point(&mut store, i as f64 + 0.1, 0.1);
        points.push(p);
        params.extend(pv);
    }

    let dist_val = store.push(1.0);
    let zero = store.push(0.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix first point at origin
    sys.add_constraint(Box::new(ConstraintEqual::new(points[0].x, zero, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(points[0].y, zero, 1.0, 2, true)));

    // Chain distance constraints
    let mut tag = 3;
    for i in 0..n - 1 {
        sys.add_constraint(Box::new(ConstraintP2PDistance::new(
            points[i], points[i + 1], dist_val, tag, true,
        )));
        tag += 1;
    }

    // Fix all points on x-axis for a well-determined system
    for i in 1..n {
        sys.add_constraint(Box::new(ConstraintEqual::new(points[i].y, zero, 1.0, tag, true)));
        tag += 1;
    }

    sys
}

/// Build an NxN grid of points with distance constraints.
fn build_grid(side: usize) -> System {
    let mut store = ParamStore::new();
    let mut points = Vec::new();
    let mut params = Vec::new();

    for i in 0..side {
        for j in 0..side {
            let (p, pv) = mk_point(&mut store, j as f64 + 0.05, i as f64 + 0.05);
            points.push(p);
            params.extend(pv);
        }
    }

    let dist_val = store.push(1.0);
    let zero = store.push(0.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(params);

    // Fix first point
    sys.add_constraint(Box::new(ConstraintEqual::new(points[0].x, zero, 1.0, 1, true)));
    sys.add_constraint(Box::new(ConstraintEqual::new(points[0].y, zero, 1.0, 2, true)));
    // Fix second point on x-axis
    sys.add_constraint(Box::new(ConstraintEqual::new(points[1].y, zero, 1.0, 3, true)));

    let mut tag = 4;

    // Horizontal edges
    for i in 0..side {
        for j in 0..side - 1 {
            let idx = i * side + j;
            sys.add_constraint(Box::new(ConstraintP2PDistance::new(
                points[idx], points[idx + 1], dist_val, tag, true,
            )));
            tag += 1;
        }
    }

    // Vertical edges
    for i in 0..side - 1 {
        for j in 0..side {
            let idx = i * side + j;
            sys.add_constraint(Box::new(ConstraintP2PDistance::new(
                points[idx], points[idx + side], dist_val, tag, true,
            )));
            tag += 1;
        }
    }

    sys
}

/// Build N independent triangles (multi-component benchmark).
fn build_independent_triangles(n: usize) -> System {
    let mut store = ParamStore::new();
    let mut all_params = Vec::new();
    let mut triangles = Vec::new();

    for i in 0..n {
        let offset = i as f64 * 20.0;
        let (p0, pv) = mk_point(&mut store, offset + 0.1, 0.1);
        all_params.extend(&pv);
        let (p1, pv) = mk_point(&mut store, offset + 4.8, 0.2);
        all_params.extend(&pv);
        let (p2, pv) = mk_point(&mut store, offset + 2.3, 3.5);
        all_params.extend(&pv);
        triangles.push((p0, p1, p2));
    }

    let side = store.push(5.0);
    let zero = store.push(0.0);

    let mut sys = System::new(store, SolverConfig::default());
    sys.declare_unknowns(all_params);

    let mut tag = 1;
    for (i, &(p0, p1, p2)) in triangles.iter().enumerate() {
        let base_x_val = sys.store_mut().push(i as f64 * 20.0);

        // Fix p0
        sys.add_constraint(Box::new(ConstraintEqual::new(p0.x, base_x_val, 1.0, tag, true)));
        tag += 1;
        sys.add_constraint(Box::new(ConstraintEqual::new(p0.y, zero, 1.0, tag, true)));
        tag += 1;
        sys.add_constraint(Box::new(ConstraintEqual::new(p1.y, zero, 1.0, tag, true)));
        tag += 1;

        // Three equal sides
        sys.add_constraint(Box::new(ConstraintP2PDistance::new(p0, p1, side, tag, true)));
        tag += 1;
        sys.add_constraint(Box::new(ConstraintP2PDistance::new(p1, p2, side, tag, true)));
        tag += 1;
        sys.add_constraint(Box::new(ConstraintP2PDistance::new(p2, p0, side, tag, true)));
        tag += 1;
    }

    sys
}

fn bench_solve_chain(c: &mut Criterion) {
    let mut group = c.benchmark_group("chain");
    for n in [5, 10, 20, 50] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_batched(
                || {
                    let mut sys = build_chain(n);
                    sys.init_solution();
                    sys
                },
                |mut sys| {
                    let status = sys.solve_with(true, Algorithm::DogLeg);
                    black_box(status);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

fn bench_solve_grid(c: &mut Criterion) {
    let mut group = c.benchmark_group("grid");
    for side in [3, 4, 5] {
        group.bench_with_input(BenchmarkId::from_parameter(side), &side, |b, &side| {
            b.iter_batched(
                || {
                    let mut sys = build_grid(side);
                    sys.init_solution();
                    sys
                },
                |mut sys| {
                    let status = sys.solve_with(true, Algorithm::DogLeg);
                    black_box(status);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

fn bench_algorithms_compare(c: &mut Criterion) {
    let mut group = c.benchmark_group("algorithm_compare");
    for alg in [Algorithm::Bfgs, Algorithm::LevenbergMarquardt, Algorithm::DogLeg] {
        group.bench_with_input(BenchmarkId::from_parameter(format!("{:?}", alg)), &alg, |b, &alg| {
            b.iter_batched(
                || {
                    let mut sys = build_chain(20);
                    sys.init_solution();
                    sys
                },
                |mut sys| {
                    let status = sys.solve_with(true, alg);
                    black_box(status);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

fn bench_multicomponent(c: &mut Criterion) {
    let mut group = c.benchmark_group("independent_triangles");
    for n in [1, 5, 10, 20] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_batched(
                || {
                    let mut sys = build_independent_triangles(n);
                    sys.init_solution();
                    sys
                },
                |mut sys| {
                    let status = sys.solve_default();
                    black_box(status);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

fn bench_init_solution(c: &mut Criterion) {
    let mut group = c.benchmark_group("init_solution");
    for n in [10, 20, 50] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_batched(
                || build_chain(n),
                |mut sys| {
                    sys.init_solution();
                    black_box(&sys);
                },
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_solve_chain,
    bench_solve_grid,
    bench_algorithms_compare,
    bench_multicomponent,
    bench_init_solution,
);
criterion_main!(benches);
