use super::super::{
    Constraint2d, ConstraintId, ConstraintResidual, GeometryId, PrimitiveFreedom, Redundancy,
    RedundancyKind, SolveError, SolveOptions, SolveReport, TangencyMode, Target2d, TargetReport,
};
use super::super::{
    constraints::StoredConstraint,
    primitives::{Param, Primitive},
};
use super::{
    evaluate::{self, Evaluation},
    prepare::{self, Component},
};
use crate::solver::numerical::{self, Factor, Problem};
use Constraint2d::*;
use nalgebra::DVector;
use std::collections::BTreeMap;

/// Solves components into owned candidate geometry; the caller commits only a complete success.
pub(crate) fn solve(
    geometry: &BTreeMap<GeometryId, Primitive>,
    constraints: &BTreeMap<ConstraintId, StoredConstraint>,
    targets: &[Target2d],
    options: &SolveOptions,
) -> Result<(BTreeMap<GeometryId, Primitive>, SolveReport), SolveError> {
    let mut candidate = geometry.clone();
    // A fixed curve retains discrete orientation as well as continuous parameters.
    for constraint in constraints.values() {
        if let Constraint2d::Fixed(id) = constraint.definition
            && let Some((sense, axis)) = constraint.captured_frame
            && let Some(primitive) = candidate.get_mut(&id)
        {
            primitive.sense = sense;
            primitive.circle_axis = axis;
        }
    }
    let components = prepare::prepare(&candidate, constraints, targets, options.linear_reduction);
    let mut report = SolveReport::default();
    for component in &components {
        check_conflicts(component, options)?;
        let mut attempt = numerical::restore(component, component.initial.clone(), options);
        report.iterations += attempt.iterations;
        if !attempt.feasible {
            return Err(SolveError::NonConvergence {
                constraints: component.constraints.iter().map(|(k, _)| *k).collect(),
                residuals: residuals(component.scale, &attempt.evaluation, options),
            });
        }
        if !component.targets.is_empty() {
            let work = numerical::improve_targets(component, &mut attempt, options);
            report.iterations += work.restore_iterations;
            report.target_iterations += work.iterations;
            report.target_budget_exhausted |= work.budget_exhausted;
        }
        let mut world = DVector::zeros(component.columns.len());
        for entry in &component.entries {
            let values = component.physical_values(entry, &attempt.x);
            for (i, value) in values.iter().enumerate() {
                world[entry.offset + i] = *value;
            }
            if let Some(primitive) = candidate.get_mut(&entry.key) {
                primitive.values = values;
            }
        }
        let physical = evaluate::hard_world(component, &world);
        // Check all original relations independently of rank reduction and secondary objectives.
        let final_evaluation = evaluate::hard(component, &attempt.x);
        if !physical.feasible(1., options)
            || component
                .entries
                .iter()
                .any(|entry| !candidate[&entry.key].valid())
            || !final_evaluation.feasible(component.scale, options)
            || !contacts_valid(component, &attempt.x)
            || !component.valid(&attempt.x)
        {
            return Err(SolveError::NonConvergence {
                constraints: component.constraints.iter().map(|(k, _)| *k).collect(),
                residuals: residuals(1., &physical, options),
            });
        }
        let factor = Factor::new(&final_evaluation.jacobian, options.rank_tolerance);
        report.degrees_of_freedom += attempt.x.len() - factor.rank();
        report.freedoms.extend(freedoms(component, &factor));
        report.residuals.extend(residuals(1., &physical, options));
        report.redundancies.extend(redundancies(
            component,
            &final_evaluation,
            options.rank_tolerance,
        ));
        for (key, c) in &component.constraints {
            if matches!(c.definition, Constraint2d::TangentCircles { .. }) {
                report.tangencies.push((*key, c.tangency));
            }
        }
        for (index, target) in &component.targets {
            let achieved = evaluate::target_point_world(component, &world, target);
            let distance = (achieved - target.position).norm();
            report.targets.push(TargetReport {
                index: *index,
                achieved,
                distance,
                reached: distance <= options.target_tolerance,
            });
        }
    }
    report.targets.sort_by_key(|r| r.index);
    Ok((candidate, report))
}

impl Problem for Component {
    type Key = ConstraintId;
    fn scale(&self) -> f64 {
        self.scale
    }
    fn valid(&self, x: &DVector<f64>) -> bool {
        Component::valid(self, x)
    }
    fn admissible(&self, x: &DVector<f64>) -> bool {
        contacts_valid(self, x)
    }
    fn permanent(&self, x: &DVector<f64>) -> Evaluation {
        evaluate::hard(self, x)
    }
    fn objectives(&self, x: &DVector<f64>) -> Evaluation {
        evaluate::soft(self, x)
    }
}

/// How freely each primitive's own parameters can still move.
fn freedoms(component: &Component, factor: &Factor) -> Vec<PrimitiveFreedom> {
    component
        .entries
        .iter()
        .map(|entry| {
            let slots: Vec<_> = (0..entry.primitive.kind.layout().len())
                .map(|slot| {
                    let slot = entry.offset + slot;
                    component.columns[slot].map(|column| (column, component.multipliers[slot]))
                })
                .collect();
            PrimitiveFreedom {
                geometry: entry.key,
                degrees: factor.freedom(&slots),
            }
        })
        .collect()
}

fn residuals(
    scale: f64,
    evaluation: &Evaluation,
    options: &SolveOptions,
) -> Vec<ConstraintResidual> {
    numerical::residuals(scale, evaluation, options)
        .into_iter()
        .map(|(constraint, values, max_tolerance_ratio)| ConstraintResidual {
            constraint,
            values,
            max_tolerance_ratio,
        })
        .collect()
}

/// Internal contact must have a unique contact point, excluding coincident circles.
fn contacts_valid(component: &Component, x: &DVector<f64>) -> bool {
    component.constraints.iter().all(|(_, c)| {
        let Constraint2d::TangentCircles { first, second, .. } = c.definition else {
            return true;
        };
        let a = |param| component.parameter(first.0, param, x);
        let b = |param| component.parameter(second.0, param, x);
        let distinct_centers = a(Param::X) != b(Param::X) || a(Param::Y) != b(Param::Y);
        match c.tangency {
            TangencyMode::FirstContainsSecond => {
                a(Param::Radius) > b(Param::Radius) && distinct_centers
            }
            TangencyMode::SecondContainsFirst => {
                b(Param::Radius) > a(Param::Radius) && distinct_centers
            }
            _ => true,
        }
    })
}

/// Warns about structural duplicates and local row dependence without claiming nonlinear redundancy.
fn redundancies(component: &Component, evaluation: &Evaluation, tolerance: f64) -> Vec<Redundancy> {
    let dependent = numerical::dependent_rows(evaluation, tolerance);
    let mut output = vec![];
    for (index, (key, range)) in evaluation.groups.iter().enumerate() {
        let c = &component.constraints[index].1;
        let duplicate = component.constraints[..index].iter().any(|(_, p)| {
            p.definition == c.definition
                && p.captured == c.captured
                && p.captured_frame == c.captured_frame
                && p.side == c.side
                && p.tangency == c.tangency
        });
        let dependent = dependent[index]
            .saturating_sub(component.eliminated_rows.get(key).copied().unwrap_or(0));
        let kind = if duplicate {
            Some(RedundancyKind::Duplicate)
        } else if dependent == range.len() {
            Some(RedundancyKind::LocalDependency)
        } else if dependent > 0 {
            Some(RedundancyKind::PartialLocalDependency)
        } else {
            None
        };
        if let Some(kind) = kind {
            output.push(Redundancy {
                constraint: *key,
                kind,
            });
        }
    }
    output
}

/// Establishes simple contradictory scalar requirements; other failures remain non-convergence.
fn check_conflicts(component: &Component, options: &SolveOptions) -> Result<(), SolveError> {
    for (i, (key, a)) in component.constraints.iter().enumerate() {
        for (other, b) in &component.constraints[..i] {
            if contradictory(a, b, options) {
                return Err(SolveError::Conflict {
                    constraints: vec![*other, *key],
                });
            }
        }
    }
    Ok(())
}
fn contradictory(a: &StoredConstraint, b: &StoredConstraint, options: &SolveOptions) -> bool {
    let linear = 2. * options.linear_tolerance;
    match (&a.definition, &b.definition) {
        (Length(a, x), Length(b, y)) if a == b => (x - y).abs() > linear,
        (Radius(a, x), Radius(b, y)) if a == b => (x - y).abs() > linear,
        (Distance(a, b, x), Distance(c, d, y)) if (a == c && b == d) || (a == d && b == c) => {
            (x - y).abs() > linear
        }
        (DistanceToLine(a, b, x), DistanceToLine(c, d, y)) if a == c && b == d => {
            (x - y).abs() > linear
        }
        (OffsetX(a, b, x), OffsetX(c, d, y)) | (OffsetY(a, b, x), OffsetY(c, d, y))
            if a == c && b == d =>
        {
            (x - y).abs() > linear
        }
        (FixedPoint(p), FixedPoint(q)) if p == q => a
            .captured
            .iter()
            .zip(&b.captured)
            .any(|(x, y)| (x - y).abs() > linear),
        (Fixed(first), Fixed(second)) if first == second => a.captured_frame != b.captured_frame,
        _ => false,
    }
}
