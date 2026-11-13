use super::super::system::Body;
use super::super::{
    BodyFreedom, BodyId, Joint3d, JointId, JointRedundancy, JointResidual, SolveError,
    SolveOptions, SolveReport, Target3d, TargetReport,
};
use super::equations::{self, Evaluation};
use super::prepare::{self, BODY_UNKNOWNS, Component};
use crate::geometry::transform::Rigid;
use crate::solver::numerical::{self, Factor, Problem};
use nalgebra::DVector;
use std::collections::BTreeMap;

/// Solves every component into world poses; the caller commits only a complete success.
pub(in crate::solver::assembly) fn solve(
    bodies: &BTreeMap<BodyId, Body>,
    joints: &BTreeMap<JointId, Joint3d>,
    targets: &[Target3d],
    options: &SolveOptions,
) -> Result<(BTreeMap<BodyId, Rigid>, SolveReport), SolveError> {
    let mut poses = BTreeMap::new();
    let mut report = SolveReport::default();
    for component in prepare::prepare(bodies, joints, targets) {
        let mut attempt = numerical::restore(&component, component.initial.clone(), options);
        report.iterations += attempt.iterations;
        if !attempt.feasible {
            return Err(failure(
                &component,
                &attempt.evaluation,
                component.scale,
                options,
            ));
        }
        if !component.targets.is_empty() {
            let work = numerical::improve_targets(&component, &mut attempt, options);
            report.iterations += work.restore_iterations;
            report.target_iterations += work.iterations;
            report.target_budget_exhausted |= work.budget_exhausted;
        }
        let solved: BTreeMap<BodyId, Rigid> = component
            .entries
            .iter()
            .map(|entry| (entry.id, component.pose(entry, &attempt.x)))
            .collect();
        // Check every joint again on the poses as published, in the caller's units.
        let physical = equations::world(&component, &solved);
        if !physical.feasible(1., options) {
            return Err(failure(&component, &physical, 1., options));
        }
        let evaluation = equations::permanent(&component, &attempt.x);
        let factor = Factor::new(&evaluation.jacobian, options.rank_tolerance);
        report.mobility += component.columns() - factor.rank();
        report.redundancy += evaluation.values.len() - factor.rank();
        for entry in component.entries.iter().filter(|entry| entry.owned) {
            let degrees = entry.column.map_or(0, |column| {
                let slots: Vec<_> = (column..column + BODY_UNKNOWNS)
                    .map(|c| Some((c, 1.)))
                    .collect();
                factor.freedom(&slots)
            });
            report.freedoms.push(BodyFreedom {
                body: entry.id,
                degrees,
            });
        }
        let dependent = numerical::dependent_rows(&evaluation, options.rank_tolerance);
        for ((joint, _), equations) in evaluation.groups.iter().zip(dependent) {
            if equations > 0 {
                report.redundancies.push(JointRedundancy {
                    joint: *joint,
                    equations,
                });
            }
        }
        report.residuals.extend(residuals(&physical, 1., options));
        for (index, target) in &component.targets {
            let achieved = solved[&target.body].apply(target.point);
            let distance = (achieved - target.position).norm();
            report.targets.push(TargetReport {
                index: *index,
                achieved,
                distance,
                reached: distance <= options.target_tolerance,
            });
        }
        for entry in component.entries.iter().filter(|entry| entry.owned) {
            poses.insert(entry.id, solved[&entry.id]);
        }
    }
    report.targets.sort_by_key(|r| r.index);
    Ok((poses, report))
}

impl Problem for Component {
    type Key = JointId;
    fn scale(&self) -> f64 {
        self.scale
    }
    fn valid(&self, x: &DVector<f64>) -> bool {
        x.iter().all(|v| v.is_finite())
    }
    fn admissible(&self, _: &DVector<f64>) -> bool {
        true
    }
    fn permanent(&self, x: &DVector<f64>) -> Evaluation {
        equations::permanent(self, x)
    }
    fn objectives(&self, x: &DVector<f64>) -> Evaluation {
        equations::objectives(self, x)
    }
}

fn failure(
    component: &Component,
    evaluation: &Evaluation,
    scale: f64,
    options: &SolveOptions,
) -> SolveError {
    SolveError::NonConvergence {
        joints: component.joints.iter().map(|(id, _)| *id).collect(),
        residuals: residuals(evaluation, scale, options),
    }
}

fn residuals(evaluation: &Evaluation, scale: f64, options: &SolveOptions) -> Vec<JointResidual> {
    numerical::residuals(scale, evaluation, options)
        .into_iter()
        .map(|(joint, values, max_tolerance_ratio)| JointResidual {
            joint,
            values,
            max_tolerance_ratio,
        })
        .collect()
}
