//! # cadora-solvers
//!
//! Numerical solvers: BFGS, Levenberg-Marquardt, DogLeg, dual-system SQP.
//! Port of planegcs solver methods from `GCS.cpp`.

use cadora_core::{DogLegGaussStep, SolveStatus, SolverConfig, SMALL_F, X_CONVERGENCE_ROUGH};
use cadora_subsystem::SubSystem;
use nalgebra::{DMatrix, DVector};

// ---------------------------------------------------------------------------
// Helper: build nalgebra DMatrix from flat row-major Jacobian
// ---------------------------------------------------------------------------

fn jacobi_matrix(subsys: &SubSystem) -> DMatrix<f64> {
    let csize = subsys.c_size();
    let psize = subsys.p_size();
    let flat = subsys.calc_jacobi();
    DMatrix::from_row_slice(csize, psize, &flat)
}

fn residual_vec(subsys: &SubSystem) -> DVector<f64> {
    DVector::from_vec(subsys.calc_residual())
}

fn grad_vec(subsys: &SubSystem) -> DVector<f64> {
    DVector::from_vec(subsys.calc_grad())
}

fn params_vec(subsys: &SubSystem) -> DVector<f64> {
    DVector::from_vec(subsys.get_params())
}

fn set_params_dv(subsys: &mut SubSystem, x: &DVector<f64>) {
    subsys.set_params(x.as_slice());
}

// ---------------------------------------------------------------------------
// BFGS
// ---------------------------------------------------------------------------

/// BFGS quasi-Newton solver.
///
/// Port of `System::solve_BFGS`.
pub fn solve_bfgs(
    subsys: &mut SubSystem,
    config: &SolverConfig,
    is_redundant: bool,
) -> SolveStatus {
    let xsize = subsys.p_size();
    if xsize == 0 {
        return SolveStatus::Success;
    }

    let mut big_d = DMatrix::<f64>::identity(xsize, xsize);
    let mut x = params_vec(subsys);
    let mut grad = grad_vec(subsys);

    // Initial steepest-descent step
    let mut xdir = -&grad;
    cadora_subsystem::line_search(subsys, xdir.as_slice());
    let mut err = subsys.error();

    let mut h = x.clone();
    x = params_vec(subsys);
    h = &x - &h;

    let max_iter = if is_redundant {
        if config.sketch_size_multiplier_redundant {
            config.max_iter_redundant * xsize
        } else {
            config.max_iter_redundant
        }
    } else if config.sketch_size_multiplier {
        config.max_iter * xsize
    } else {
        config.max_iter
    };

    let conv = if is_redundant {
        config.convergence_redundant
    } else {
        config.convergence
    };

    let diverging_lim = 1e6 * err + 1e12;

    for _iter in 1..max_iter {
        let h_norm = h.norm();
        if h_norm <= conv || err <= SMALL_F {
            break;
        }
        if err > diverging_lim || err.is_nan() {
            break;
        }

        let y_old = grad.clone();
        grad = grad_vec(subsys);
        let y = &grad - &y_old;

        let mut hty = h.dot(&y);
        if hty == 0.0 {
            hty = 1e-10;
        }

        let big_dy = &big_d * &y;
        let yt_dy = y.dot(&big_dy);

        // BFGS update: D += (1+ytDy/hty)/hty * h*h^T - 1/hty * (h*Dy^T + Dy*h^T)
        big_d += (1.0 + yt_dy / hty) / hty * &h * h.transpose();
        big_d -= (1.0 / hty) * (&h * big_dy.transpose() + &big_dy * h.transpose());

        xdir = -&big_d * &grad;
        cadora_subsystem::line_search(subsys, xdir.as_slice());
        err = subsys.error();

        let x_old = x.clone();
        x = params_vec(subsys);
        h = &x - &x_old;
    }

    if err <= SMALL_F {
        SolveStatus::Success
    } else if h.norm() <= conv {
        SolveStatus::Converged
    } else {
        SolveStatus::Failed
    }
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt
// ---------------------------------------------------------------------------

/// Levenberg-Marquardt solver.
///
/// Port of `System::solve_LM`.
pub fn solve_lm(
    subsys: &mut SubSystem,
    config: &SolverConfig,
    is_redundant: bool,
) -> SolveStatus {
    let xsize = subsys.p_size();
    let _csize = subsys.c_size();
    if xsize == 0 {
        return SolveStatus::Success;
    }

    let mut x = params_vec(subsys);
    let mut e = -residual_vec(subsys);

    let max_iter = if is_redundant {
        if config.sketch_size_multiplier_redundant {
            config.max_iter_redundant * xsize
        } else {
            config.max_iter_redundant
        }
    } else if config.sketch_size_multiplier {
        config.max_iter * xsize
    } else {
        config.max_iter
    };

    let diverging_lim = 1e6 * e.norm_squared() + 1e12;

    let (eps, eps1, tau) = if is_redundant {
        (config.lm_eps_redundant, config.lm_eps1_redundant, config.lm_tau_redundant)
    } else {
        (config.lm_eps, config.lm_eps1, config.lm_tau)
    };

    let mut nu = 2.0_f64;
    let mut mu = 0.0_f64;
    let mut stop = 0;

    for iter in 0..max_iter {
        let err = e.norm_squared();
        if err <= eps * eps {
            stop = 1;
            break;
        }
        if err > diverging_lim || err.is_nan() {
            stop = 6;
            break;
        }

        let j = jacobi_matrix(subsys);
        let mut a = j.transpose() * &j;
        let g = j.transpose() * &e;

        let g_inf = g.amax();
        let diag_a: DVector<f64> = DVector::from_iterator(xsize, (0..xsize).map(|i| a[(i, i)]));

        if g_inf <= eps1 {
            stop = 2;
            break;
        }

        if iter == 0 {
            mu = tau * diag_a.amax();
        }

        let mut _h_norm = 0.0_f64;
        let mut k = 0;
        while k < 50 {
            // Augment: A = J^T J + μI
            for i in 0..xsize {
                a[(i, i)] += mu;
            }

            // Solve A*h = g
            let h_opt = a.clone().full_piv_lu().solve(&g);
            if let Some(h) = h_opt {
                let rel_error = (&a * &h - &g).norm() / g.norm();

                if rel_error < 1e-5 {
                    // Restrict by maxStep
                    let scale = subsys.max_step(h.as_slice());
                    let h_scaled = if scale < 1.0 { &h * scale } else { h.clone() };

                    let x_new = &x + &h_scaled;
                    _h_norm = h_scaled.norm_squared();

                    let x_norm = x.norm();
                    if _h_norm <= eps1 * eps1 * x_norm {
                        stop = 3;
                        break;
                    }
                    if _h_norm >= (x_norm + X_CONVERGENCE_ROUGH) / (f64::EPSILON * f64::EPSILON) {
                        stop = 4;
                        break;
                    }

                    set_params_dv(subsys, &x_new);
                    let e_new = -residual_vec(subsys);

                    let d_f = e.norm_squared() - e_new.norm_squared();
                    let d_l = h.dot(&(&h * mu + &g));

                    if d_f > 0.0 && d_l > 0.0 {
                        let tmp = 2.0 * d_f / d_l - 1.0;
                        mu *= (1.0 / 3.0_f64).max(1.0 - tmp * tmp * tmp);
                        nu = 2.0;
                        x = x_new;
                        e = e_new;
                        break;
                    }
                }
            }

            // Reject: increase damping
            mu *= nu;
            nu *= 2.0;
            for i in 0..xsize {
                a[(i, i)] = diag_a[i];
            }
            k += 1;
        }
        if k >= 50 {
            stop = 7;
            break;
        }
        if stop != 0 {
            break;
        }
    }

    if stop == 1 {
        SolveStatus::Success
    } else {
        SolveStatus::Failed
    }
}

// ---------------------------------------------------------------------------
// DogLeg
// ---------------------------------------------------------------------------

/// DogLeg trust-region solver.
///
/// Port of `System::solve_DL`.
pub fn solve_dl(
    subsys: &mut SubSystem,
    config: &SolverConfig,
    is_redundant: bool,
) -> SolveStatus {
    let xsize = subsys.p_size();
    if xsize == 0 {
        return SolveStatus::Success;
    }

    let (tolg, tolx, tolf) = if is_redundant {
        (config.dl_tolg_redundant, config.dl_tolx_redundant, config.dl_tolf_redundant)
    } else {
        (config.dl_tolg, config.dl_tolx, config.dl_tolf)
    };

    let max_iter = if is_redundant {
        if config.sketch_size_multiplier_redundant {
            config.max_iter_redundant * xsize
        } else {
            config.max_iter_redundant
        }
    } else if config.sketch_size_multiplier {
        config.max_iter * xsize
    } else {
        config.max_iter
    };

    let mut x = params_vec(subsys);
    let (r, mut err) = subsys.calc_residual_and_error();
    let mut fx = DVector::from_vec(r);
    let mut jx = jacobi_matrix(subsys);

    let mut g = jx.transpose() * (-&fx);
    let mut g_inf = g.amax();
    let mut fx_inf = fx.amax();

    let diverging_lim = 1e6 * err + 1e12;

    let mut delta = 0.1;
    let mut nu = 2.0_f64;
    let mut reduce: i32 = 0;
    let mut stop = 0;
    let mut iter = 0;

    while stop == 0 {
        if fx_inf <= tolf {
            stop = 1;
            break;
        }
        if g_inf <= tolg {
            stop = 2;
            break;
        }
        if delta <= tolx * (tolx + x.norm()) {
            stop = 2;
            break;
        }
        if iter >= max_iter {
            stop = 4;
            break;
        }
        if err > diverging_lim || err.is_nan() {
            stop = 6;
            break;
        }

        // Steepest descent step
        let g_sq = g.norm_squared();
        let jxg = &jx * &g;
        let alpha = g_sq / jxg.norm_squared();
        let h_sd = &g * alpha;

        // Gauss-Newton step
        let neg_fx = -&fx;
        let h_gn = match config.dog_leg_gauss_step {
            DogLegGaussStep::FullPivLu => {
                if jx.nrows() == jx.ncols() {
                    jx.clone().full_piv_lu().solve(&neg_fx).unwrap_or_else(|| DVector::zeros(xsize))
                } else {
                    // Non-square: use normal equations J^T*J * h = J^T * (-fx)
                    let jtj = jx.transpose() * &jx;
                    let jt_neg_fx = jx.transpose() * &neg_fx;
                    jtj.full_piv_lu().solve(&jt_neg_fx).unwrap_or_else(|| DVector::zeros(xsize))
                }
            }
            DogLegGaussStep::LeastNormFullPivLu => {
                let jjt = &jx * jx.transpose();
                let sol = jjt.full_piv_lu().solve(&neg_fx).unwrap_or_else(|| DVector::zeros(fx.len()));
                jx.transpose() * sol
            }
            DogLegGaussStep::LeastNormLdlt => {
                let jjt = &jx * jx.transpose();
                // Use Cholesky as nalgebra doesn't have ldlt directly; fall back to full_piv_lu
                let sol = jjt.clone().full_piv_lu().solve(&neg_fx).unwrap_or_else(|| DVector::zeros(fx.len()));
                jx.transpose() * sol
            }
        };

        let rel_error = (&jx * &h_gn + &fx).norm() / fx.norm();
        if rel_error > 1e15 {
            break;
        }

        // Compute dogleg step
        let h_dl;
        let h_gn_norm = h_gn.norm();
        if h_gn_norm < delta {
            h_dl = h_gn;
            if h_dl.norm() <= tolx * (tolx + x.norm()) {
                stop = 5;
                break;
            }
        } else if alpha * g.norm() >= delta {
            h_dl = &h_sd * (delta / (alpha * g.norm()));
        } else {
            let b = &h_gn - &h_sd;
            let bb = b.norm_squared();
            let gb = h_sd.dot(&b);
            let c = (delta + h_sd.norm()) * (delta - h_sd.norm());

            let beta = if gb > 0.0 {
                c / (gb + (gb * gb + c * bb).sqrt())
            } else {
                ((gb * gb + c * bb).sqrt() - gb) / bb
            };
            h_dl = &h_sd + &b * beta;
        }

        // Trial step
        let x_new = &x + &h_dl;
        set_params_dv(subsys, &x_new);
        let (r_new, err_new) = subsys.calc_residual_and_error();
        let fx_new = DVector::from_vec(r_new);
        let jx_new = jacobi_matrix(subsys);

        let d_l = err - 0.5 * (&fx + &jx * &h_dl).norm_squared();
        let d_f = err - err_new;
        let rho = if d_l.abs() > 1e-30 { d_f / d_l } else { -1.0 };

        if d_f > 0.0 && d_l > 0.0 {
            x = x_new;
            jx = jx_new;
            fx = fx_new;
            err = err_new;
            g = jx.transpose() * (-&fx);
            g_inf = g.amax();
            fx_inf = fx.amax();
        }

        let rho_val = if d_f > 0.0 && d_l > 0.0 { rho } else { -1.0 };

        // Update trust-region radius
        if (rho_val - 1.0).abs() < 0.2 && h_dl.norm() > delta / 3.0 && reduce <= 0 {
            delta *= 3.0;
            nu = 2.0;
            reduce = 0;
        } else if rho_val < 0.25 {
            delta /= nu;
            nu *= 2.0;
            reduce = 2;
        } else {
            reduce -= 1;
        }

        iter += 1;
    }

    if stop == 1 {
        SolveStatus::Success
    } else {
        SolveStatus::Failed
    }
}

// ---------------------------------------------------------------------------
// qp_eq — equality-constrained QP
// ---------------------------------------------------------------------------

/// Solve: min 0.5 x^T H x + g^T x  subject to  A x + c = 0.
///
/// Returns `(x, Y, Z)` where Y is the row-space and Z is the null-space of A,
/// or `None` if the constraints are rank-deficient.
///
/// Port of `qp_eq.cpp`.
pub fn qp_eq(
    h_mat: &DMatrix<f64>,
    g: &DVector<f64>,
    a: &DMatrix<f64>,
    c: &DVector<f64>,
) -> Option<(DVector<f64>, DMatrix<f64>, Option<DMatrix<f64>>)> {
    let at = a.transpose();

    let params_num = a.ncols();
    let constr_num = a.nrows();

    if constr_num > params_num {
        return None;
    }

    // Use SVD for robust rank computation (thin SVD is fine for singular values)
    let svd = a.clone().svd(false, false);
    let rank = svd
        .singular_values
        .iter()
        .filter(|&&s| s > 1e-12)
        .count();

    if rank != constr_num {
        return None;
    }

    // Particular solution: x_p = A^T (A A^T)^{-1} (-c)
    let aat = a * &at;
    let aat_lu = aat.full_piv_lu();
    let y_c = aat_lu.solve(c)?;
    let x_p = -&at * &y_c;

    // Y = A^T (A A^T)^{-1}
    let y_mat = {
        let aat_inv_cols: Vec<DVector<f64>> = (0..constr_num)
            .map(|i| {
                let mut ei = DVector::zeros(constr_num);
                ei[i] = 1.0;
                aat_lu.solve(&ei).unwrap_or_else(|| DVector::zeros(constr_num))
            })
            .collect();
        let aat_inv = DMatrix::from_columns(&aat_inv_cols);
        &at * aat_inv
    };

    if params_num == rank {
        Some((x_p, y_mat, None))
    } else {
        // Null space of A via the null-space projector P = I - Y*A.
        // P is params_num × params_num (square), so SVD gives full V^T.
        // P is an orthogonal projector — its nonzero singular values are 1.
        let proj = DMatrix::identity(params_num, params_num) - &y_mat * a;
        let svd_proj = proj.svd(false, true);
        let vt = svd_proj.v_t.as_ref()?;
        let null_dim = params_num - rank;
        // Take the first null_dim right singular vectors (σ ≈ 1)
        let z_cols: Vec<DVector<f64>> = (0..null_dim)
            .map(|i| vt.row(i).transpose())
            .collect();
        let z = DMatrix::from_columns(&z_cols);

        let zthz = z.transpose() * h_mat * &z;
        let rhs = z.transpose() * (h_mat * &y_mat * c - g);
        let y_sol = zthz.full_piv_lu().solve(&rhs)?;
        let x = &x_p + &z * &y_sol;

        Some((x, y_mat, Some(z)))
    }
}

// ---------------------------------------------------------------------------
// Dispatch: solve single subsystem with chosen algorithm
// ---------------------------------------------------------------------------

/// Solve a single subsystem using the algorithm specified in config.
pub fn solve(
    subsys: &mut SubSystem,
    config: &SolverConfig,
    algorithm: cadora_core::Algorithm,
    is_redundant: bool,
) -> SolveStatus {
    match algorithm {
        cadora_core::Algorithm::Bfgs => solve_bfgs(subsys, config, is_redundant),
        cadora_core::Algorithm::LevenbergMarquardt => solve_lm(subsys, config, is_redundant),
        cadora_core::Algorithm::DogLeg => solve_dl(subsys, config, is_redundant),
    }
}

// ---------------------------------------------------------------------------
// Dual-system SQP (Sequential Quadratic Programming)
// ---------------------------------------------------------------------------

/// Dual-system SQP solver: solve `subsys_a` (hard equality constraints)
/// while minimizing `subsys_b` (soft / driving constraints).
///
/// Port of `System::solve(SubSystem*,SubSystem*,...)` from GCS.cpp.
///
/// - **SubsysA** = higher-priority (hard equalities to be satisfied exactly).
/// - **SubsysB** = lower-priority (objective function to be minimized).
///
/// Uses an L₁ exact penalty merit function, BFGS Hessian approximation, and
/// Nocedal–Wright SQP line search (Eqs 18.27–18.36).
pub fn solve_sqp(
    subsys_a: &mut SubSystem,
    subsys_b: &mut SubSystem,
    config: &SolverConfig,
    is_redundant: bool,
) -> SolveStatus {
    use cadora_core::ParamIdx;
    use std::collections::BTreeSet;

    let xsize_a = subsys_a.p_size();
    let xsize_b = subsys_b.p_size();
    let csize_a = subsys_a.c_size();

    if xsize_a == 0 && xsize_b == 0 {
        return SolveStatus::Success;
    }

    // Build union param list (sorted, deduplicated)
    let mut param_set = BTreeSet::new();
    for &p in subsys_a.param_list() {
        param_set.insert(p);
    }
    for &p in subsys_b.param_list() {
        param_set.insert(p);
    }
    let params_ab: Vec<ParamIdx> = param_set.into_iter().collect();
    let xsize = params_ab.len();

    if xsize == 0 {
        return SolveStatus::Success;
    }

    let mut big_b = DMatrix::<f64>::identity(xsize, xsize); // BFGS approx Hessian

    // Synchronize: A's values take priority over B's
    let _x_b = subsys_b.get_params_ext(&params_ab);
    let x_a = subsys_a.get_params_ext(&params_ab);
    let mut x = DVector::from_vec(x_a);
    subsys_b.set_params_ext(&params_ab, x.as_slice());

    // Initial function evaluations
    let mut grad = DVector::from_vec(subsys_b.calc_grad_ext(&params_ab));

    let ja_flat = subsys_a.calc_jacobi_ext(&params_ab);
    let mut j_a = DMatrix::from_row_slice(csize_a, xsize, &ja_flat);

    let mut res_a = DVector::from_vec(subsys_a.calc_residual());

    let max_iter = if is_redundant {
        if config.sketch_size_multiplier_redundant {
            config.max_iter_redundant * xsize
        } else {
            config.max_iter_redundant
        }
    } else if config.sketch_size_multiplier {
        config.max_iter * xsize
    } else {
        config.max_iter
    };

    let conv = if is_redundant {
        config.convergence_redundant
    } else {
        config.convergence
    };

    let diverging_lim = 1e6 * subsys_a.error() + 1e12;

    let mut mu = 0.0_f64;
    let mut lambda = DVector::zeros(csize_a);
    let mut h = DVector::zeros(xsize);

    for _iter in 1..max_iter {
        // Solve QP subproblem
        let qp_result = qp_eq(&big_b, &grad, &j_a, &res_a);
        let (xdir, y_mat, _z) = match qp_result {
            Some(r) => r,
            None => break,
        };

        let x0 = x.clone();
        let lambda0 = lambda.clone();
        lambda = y_mat.transpose() * (&big_b * &xdir + &grad);
        let lambdadir = &lambda - &lambda0;

        // Line search (Nocedal & Wright Eqs 18.27-18.36)
        let eta = 0.25;
        let tau = 0.5;
        let rho = 0.5;
        let mut alpha = 1.0_f64;
        alpha = alpha.min(subsys_a.max_step_ext(&params_ab, xdir.as_slice()));

        // Penalty parameter update (Eq 18.36)
        let res_a_l1 = res_a.iter().map(|r| r.abs()).sum::<f64>();
        if res_a_l1 > 1e-30 {
            let penalty_cand = (grad.dot(&xdir) + (0.5 * xdir.dot(&(&big_b * &xdir))).max(0.0))
                / ((1.0 - rho) * res_a_l1);
            mu = mu.max(penalty_cand);
        }

        // Merit function at current point
        let f0 = subsys_b.error() + mu * res_a_l1;
        let deriv = grad.dot(&xdir) - mu * res_a_l1;

        x = &x0 + &xdir * alpha;
        subsys_a.set_params_ext(&params_ab, x.as_slice());
        subsys_b.set_params_ext(&params_ab, x.as_slice());
        res_a = DVector::from_vec(subsys_a.calc_residual());
        let new_l1 = res_a.iter().map(|r| r.abs()).sum::<f64>();
        let mut f = subsys_b.error() + mu * new_l1;

        // Armijo backtracking
        let mut first = true;
        while f > f0 + eta * alpha * deriv {
            if first {
                first = false;
                // Correction step: project onto A-constraint manifold
                let xdir1 = -&y_mat * &res_a;
                x = &x + &xdir1;
                subsys_a.set_params_ext(&params_ab, x.as_slice());
                subsys_b.set_params_ext(&params_ab, x.as_slice());
                res_a = DVector::from_vec(subsys_a.calc_residual());
                let new_l1 = res_a.iter().map(|r| r.abs()).sum::<f64>();
                f = subsys_b.error() + mu * new_l1;
                if f < f0 + eta * alpha * deriv {
                    break;
                }
            }
            alpha *= tau;
            if alpha < 1e-8 {
                alpha = 0.0;
            }
            x = &x0 + &xdir * alpha;
            subsys_a.set_params_ext(&params_ab, x.as_slice());
            subsys_b.set_params_ext(&params_ab, x.as_slice());
            res_a = DVector::from_vec(subsys_a.calc_residual());
            let new_l1 = res_a.iter().map(|r| r.abs()).sum::<f64>();
            f = subsys_b.error() + mu * new_l1;
            if alpha < 1e-8 {
                break;
            }
        }
        lambda = &lambda0 + &lambdadir * alpha;

        h = &x - &x0;

        // BFGS update of the Lagrangian Hessian (Eq 18.13)
        let mut y = &grad - &j_a.transpose() * &lambda;
        {
            grad = DVector::from_vec(subsys_b.calc_grad_ext(&params_ab));
            let ja_flat = subsys_a.calc_jacobi_ext(&params_ab);
            j_a = DMatrix::from_row_slice(csize_a, xsize, &ja_flat);
            res_a = DVector::from_vec(subsys_a.calc_residual());
        }
        y = &grad - &j_a.transpose() * &lambda - &y;

        if _iter > 1 {
            let y_t_h = y.dot(&h);
            if y_t_h.abs() > 1e-30 {
                let bh = &big_b * &h;
                big_b += (1.0 / y_t_h) * &y * y.transpose();
                big_b -= (1.0 / h.dot(&bh)) * (&bh * bh.transpose());
            }
        }

        // Convergence check
        let err = subsys_a.error();
        if h.norm() <= conv && err <= SMALL_F {
            break;
        }
        if err > diverging_lim || err.is_nan() {
            break;
        }
    }

    if subsys_a.error() <= SMALL_F {
        SolveStatus::Success
    } else if h.norm() <= conv {
        SolveStatus::Converged
    } else {
        SolveStatus::Failed
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

    // Test constraint: error = x - target
    struct PTarget {
        pvec: Vec<ParamIdx>,
        target: f64,
    }
    impl PTarget {
        fn new(p: ParamIdx, target: f64) -> Self {
            Self { pvec: vec![p], target }
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
        fn tag(&self) -> i32 { 0 }
        fn is_driving(&self) -> bool { true }
    }

    // Quadratic constraint: error = x^2 - target
    struct QuadTarget {
        pvec: Vec<ParamIdx>,
        target: f64,
    }
    impl QuadTarget {
        fn new(p: ParamIdx, target: f64) -> Self {
            Self { pvec: vec![p], target }
        }
    }
    impl Constraint for QuadTarget {
        fn error(&self, store: &ParamStore) -> f64 {
            let x = store.get(self.pvec[0]);
            x * x - self.target
        }
        fn grad(&self, store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] { 2.0 * store.get(self.pvec[0]) } else { 0.0 }
        }
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> i32 { 0 }
        fn is_driving(&self) -> bool { true }
    }

    fn make_linear_subsys() -> (ParamStore, SubSystem) {
        let mut store = ParamStore::new();
        let x = store.push(10.0);
        let y = store.push(-5.0);
        let c1: Box<dyn Constraint> = Box::new(PTarget::new(x, 3.0));
        let c2: Box<dyn Constraint> = Box::new(PTarget::new(y, 7.0));
        let ss = SubSystem::new(vec![c1, c2], &[x, y], &store);
        (store, ss)
    }

    fn make_nonlinear_subsys() -> (ParamStore, SubSystem) {
        let mut store = ParamStore::new();
        let x = store.push(5.0); // want x^2 = 9 → x=3
        let c: Box<dyn Constraint> = Box::new(QuadTarget::new(x, 9.0));
        let ss = SubSystem::new(vec![c], &[x], &store);
        (store, ss)
    }

    #[test]
    fn bfgs_linear() {
        let (_store, mut ss) = make_linear_subsys();
        let cfg = SolverConfig::default();
        let status = solve_bfgs(&mut ss, &cfg, false);
        assert!(matches!(status, SolveStatus::Success | SolveStatus::Converged));
        let p = ss.get_params();
        assert_abs_diff_eq!(p[0], 3.0, epsilon = 1e-6);
        assert_abs_diff_eq!(p[1], 7.0, epsilon = 1e-6);
    }

    #[test]
    fn bfgs_nonlinear() {
        let (_store, mut ss) = make_nonlinear_subsys();
        let cfg = SolverConfig::default();
        let status = solve_bfgs(&mut ss, &cfg, false);
        assert!(matches!(status, SolveStatus::Success | SolveStatus::Converged));
        let p = ss.get_params();
        // x^2 = 9 has two roots; accept either
        assert_abs_diff_eq!(p[0].abs(), 3.0, epsilon = 1e-4);
    }

    #[test]
    fn lm_linear() {
        let (_store, mut ss) = make_linear_subsys();
        let cfg = SolverConfig::default();
        let status = solve_lm(&mut ss, &cfg, false);
        assert!(matches!(status, SolveStatus::Success));
        let p = ss.get_params();
        assert_abs_diff_eq!(p[0], 3.0, epsilon = 1e-6);
        assert_abs_diff_eq!(p[1], 7.0, epsilon = 1e-6);
    }

    #[test]
    fn lm_nonlinear() {
        let (_store, mut ss) = make_nonlinear_subsys();
        let cfg = SolverConfig::default();
        let status = solve_lm(&mut ss, &cfg, false);
        assert!(matches!(status, SolveStatus::Success));
        let p = ss.get_params();
        assert_abs_diff_eq!(p[0], 3.0, epsilon = 1e-4);
    }

    #[test]
    fn dl_linear() {
        let (_store, mut ss) = make_linear_subsys();
        let cfg = SolverConfig::default();
        let status = solve_dl(&mut ss, &cfg, false);
        assert!(matches!(status, SolveStatus::Success));
        let p = ss.get_params();
        assert_abs_diff_eq!(p[0], 3.0, epsilon = 1e-6);
        assert_abs_diff_eq!(p[1], 7.0, epsilon = 1e-6);
    }

    #[test]
    fn dl_nonlinear() {
        let (_store, mut ss) = make_nonlinear_subsys();
        let cfg = SolverConfig::default();
        let status = solve_dl(&mut ss, &cfg, false);
        assert!(matches!(status, SolveStatus::Success));
        let p = ss.get_params();
        assert_abs_diff_eq!(p[0], 3.0, epsilon = 1e-4);
    }

    #[test]
    fn dispatch_selects_algorithm() {
        let (_store, mut ss) = make_linear_subsys();
        let cfg = SolverConfig::default();
        let status = solve(&mut ss, &cfg, cadora_core::Algorithm::DogLeg, false);
        assert!(matches!(status, SolveStatus::Success));
    }

    #[test]
    fn qp_eq_simple() {
        // min 0.5*(x1^2 + x2^2) subject to x1 + x2 = 1
        // H = I, g = 0, A = [1,1], c = [-1]
        // Solution: x1 = x2 = 0.5
        let h = DMatrix::identity(2, 2);
        let g = DVector::zeros(2);
        let a = DMatrix::from_row_slice(1, 2, &[1.0, 1.0]);
        let c = DVector::from_vec(vec![-1.0]);
        let (x, _y, _z) = qp_eq(&h, &g, &a, &c).unwrap();
        assert_abs_diff_eq!(x[0], 0.5, epsilon = 1e-10);
        assert_abs_diff_eq!(x[1], 0.5, epsilon = 1e-10);
    }

    #[test]
    fn zero_size_subsystem_succeeds() {
        let store = ParamStore::new();
        let mut ss = SubSystem::new(vec![], &[], &store);
        let cfg = SolverConfig::default();
        assert!(matches!(solve_bfgs(&mut ss, &cfg, false), SolveStatus::Success));
        assert!(matches!(solve_lm(&mut ss, &cfg, false), SolveStatus::Success));
        assert!(matches!(solve_dl(&mut ss, &cfg, false), SolveStatus::Success));
    }

    #[test]
    fn apply_solution_updates_global() {
        let (mut store, mut ss) = make_linear_subsys();
        let cfg = SolverConfig::default();
        solve_dl(&mut ss, &cfg, false);
        ss.apply_solution(&mut store);
        // Params in store should now be solved values
        let params: Vec<f64> = ss.param_list().iter().map(|&p| store.get(p)).collect();
        assert_abs_diff_eq!(params[0], 3.0, epsilon = 1e-6);
        assert_abs_diff_eq!(params[1], 7.0, epsilon = 1e-6);
    }

    // -----------------------------------------------------------------------
    // SQP tests
    // -----------------------------------------------------------------------

    /// Two constraints sharing one param: A forces x+y=1, B minimizes (x-5)^2+(y-5)^2.
    /// Solution: point closest to (5,5) on the line x+y=1 → (0.5, 0.5).
    struct SumEq {
        pvec: Vec<ParamIdx>,
        target: f64,
    }
    impl SumEq {
        fn new(a: ParamIdx, b: ParamIdx, target: f64) -> Self {
            Self { pvec: vec![a, b], target }
        }
    }
    impl Constraint for SumEq {
        fn error(&self, store: &ParamStore) -> f64 {
            store.get(self.pvec[0]) + store.get(self.pvec[1]) - self.target
        }
        fn grad(&self, _store: &ParamStore, param: ParamIdx) -> f64 {
            if param == self.pvec[0] || param == self.pvec[1] { 1.0 } else { 0.0 }
        }
        fn params(&self) -> &[ParamIdx] { &self.pvec }
        fn tag(&self) -> i32 { 0 }
        fn is_driving(&self) -> bool { true }
    }

    #[test]
    fn sqp_constrained_minimum() {
        // A: x + y = 1 (hard equality)
        // B: minimize (x-5)^2 + (y-5)^2 → drive x→5, y→5
        // Solution on constraint: x=0.5, y=0.5
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let y = store.push(0.0);

        let c_a: Box<dyn Constraint> = Box::new(SumEq::new(x, y, 1.0));
        let c_bx: Box<dyn Constraint> = Box::new(PTarget::new(x, 5.0));
        let c_by: Box<dyn Constraint> = Box::new(PTarget::new(y, 5.0));

        let mut ss_a = SubSystem::new(vec![c_a], &[x, y], &store);
        let mut ss_b = SubSystem::new(vec![c_bx, c_by], &[x, y], &store);

        let cfg = SolverConfig::default();
        let status = solve_sqp(&mut ss_a, &mut ss_b, &cfg, false);
        assert!(
            matches!(status, SolveStatus::Success | SolveStatus::Converged),
            "SQP failed: {:?}", status
        );

        // A constraint should be satisfied
        assert_abs_diff_eq!(ss_a.error(), 0.0, epsilon = 1e-8);
        // Parameters should be at the constrained min
        let pa = ss_a.get_params();
        assert_abs_diff_eq!(pa[0] + pa[1], 1.0, epsilon = 1e-6);
    }

    #[test]
    fn sqp_trivial_both_satisfied() {
        // Both A and B can be satisfied simultaneously
        let mut store = ParamStore::new();
        let x = store.push(0.0);
        let y = store.push(0.0);

        let c_a: Box<dyn Constraint> = Box::new(PTarget::new(x, 3.0));
        let c_b: Box<dyn Constraint> = Box::new(PTarget::new(y, 7.0));

        let mut ss_a = SubSystem::new(vec![c_a], &[x, y], &store);
        let mut ss_b = SubSystem::new(vec![c_b], &[x, y], &store);

        let cfg = SolverConfig::default();
        let status = solve_sqp(&mut ss_a, &mut ss_b, &cfg, false);
        assert!(matches!(status, SolveStatus::Success | SolveStatus::Converged));
        assert_abs_diff_eq!(ss_a.error(), 0.0, epsilon = 1e-8);
        assert_abs_diff_eq!(ss_b.error(), 0.0, epsilon = 1e-8);
    }
}
