use cadora_core::{Algorithm, ParamStore, SolverConfig, SolveStatus};
use cadora_subsystem::SubSystem;
use cadora_constraints::Constraint;
use cadora_core::{ParamIdx, Tag};

struct PT { pvec: Vec<ParamIdx>, target: f64 }
impl PT {
    fn new(p: ParamIdx, t: f64) -> Self { Self { pvec: vec![p], target: t } }
}
impl Constraint for PT {
    fn error(&self, store: &ParamStore) -> f64 { store.get(self.pvec[0]) - self.target }
    fn grad(&self, _: &ParamStore, p: ParamIdx) -> f64 { if p == self.pvec[0] { 1.0 } else { 0.0 } }
    fn params(&self) -> &[ParamIdx] { &self.pvec }
    fn tag(&self) -> Tag { 1 }
    fn is_driving(&self) -> bool { true }
}

fn main() {
    let mut store = ParamStore::new();
    let x = store.push(0.0);
    let c: Box<dyn Constraint> = Box::new(PT::new(x, 7.0));
    let mut subsys = SubSystem::new(vec![c], &[x], &store);
    let config = SolverConfig::default();
    let status = cadora_solvers::solve(&mut subsys, &config, Algorithm::LevenbergMarquardt, false);
    println!("LM status: {:?}", status);
    subsys.apply_solution(&mut store);
    println!("x = {}", store.get(x));
}
