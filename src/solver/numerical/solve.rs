use super::super::{
    Constraint2d, ConstraintResidual, Redundancy, RedundancyKind, SolveError, SolveOptions,
    SolveReport, TangencyMode, Target2d, TargetReport,
};
use super::super::{
    constraints::StoredConstraint,
    keys::{ConstraintSlot, GeometrySlot},
    primitives::Primitive,
};
use super::{
    evaluate::{self, Evaluation},
    prepare::{self, Component},
};
use nalgebra::{DMatrix, DVector};
use slotmap::SlotMap;

/// Solves components into owned candidate geometry; the caller commits only a complete success.
pub(crate) fn solve(
    owner: u64,
    geometry: &SlotMap<GeometrySlot, Primitive>,
    constraints: &SlotMap<ConstraintSlot, StoredConstraint>,
    targets: &[Target2d],
    options: &SolveOptions,
) -> Result<(SlotMap<GeometrySlot, Primitive>, SolveReport), SolveError> {
    let mut candidate = geometry.clone();
    // A fixed curve retains discrete orientation as well as continuous parameters.
    for (_, constraint) in constraints {
        if let Constraint2d::Fixed(key) = constraint.definition
            && let Some((sense, axis)) = constraint.captured_frame
        {
            candidate[key.slot].sense = sense;
            candidate[key.slot].circle_axis = axis;
        }
    }
    let components = prepare::prepare(
        owner,
        &candidate,
        constraints,
        targets,
        options.linear_reduction,
    );
    let mut report = SolveReport::default();
    for component in &components {
        check_conflicts(component, options)?;
        let mut attempt = restore(component, component.initial.clone(), options);
        report.iterations += attempt.iterations;
        if !attempt.feasible {
            return Err(SolveError::NonConvergence {
                constraints: component.constraints.iter().map(|(k, _)| *k).collect(),
                residuals: residuals(component.scale, &attempt.evaluation, options),
            });
        }
        if !component.targets.is_empty() {
            improve_targets(component, &mut attempt, options, &mut report);
        }
        let mut world = DVector::zeros(component.columns.len());
        for entry in &component.entries {
            let values = component.physical_values(entry, &attempt.x);
            for (i, value) in values.iter().enumerate() {
                world[entry.offset + i] = *value;
            }
            candidate[entry.key.slot].values = values;
        }
        let physical = evaluate::hard_world(component, &world);
        // Check all original relations independently of rank reduction and secondary objectives.
        let final_evaluation = evaluate::hard(component, &attempt.x);
        if !physical.feasible(1., options)
            || component
                .entries
                .iter()
                .any(|entry| !candidate[entry.key.slot].valid())
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

struct Attempt {
    x: DVector<f64>,
    evaluation: Evaluation,
    iterations: usize,
    feasible: bool,
}

/// Damped minimum-norm corrections with adaptive step bounds and domain rejection.
fn restore(component: &Component, mut x: DVector<f64>, options: &SolveOptions) -> Attempt {
    let mut evaluation = evaluate::hard(component, &x);
    let mut damping = 1e-6_f64;
    let mut radius = 1_f64;
    let mut iterations = 0;
    for _ in 0..options.max_iterations {
        if !component.valid(&x) || !evaluation.finite() {
            break;
        }
        if evaluation.feasible(component.scale, options) {
            break;
        }
        iterations += 1;
        let factor = Factor::new(&evaluation.jacobian, options.rank_tolerance);
        let mut step = factor.correction(&(-&evaluation.values), damping);
        let norm = step.norm();
        if !norm.is_finite() || norm < 1e-15 {
            break;
        }
        if norm > radius {
            step *= radius / norm;
        }
        let trial = &x + step;
        if !component.valid(&trial) {
            damping = (damping * 10.).min(1e12);
            radius *= 0.5;
            continue;
        }
        let next = evaluate::hard(component, &trial);
        if next.finite() && next.values.norm() < evaluation.values.norm() {
            x = trial;
            evaluation = next;
            damping = (damping * 0.2).max(1e-14);
            radius = (radius * 2.).min(1e6);
        } else {
            damping = (damping * 10.).min(1e12);
            radius *= 0.5;
        }
    }
    let feasible = component.valid(&x)
        && evaluation.feasible(component.scale, options)
        && contacts_valid(component, &x);
    Attempt {
        x,
        evaluation,
        iterations,
        feasible,
    }
}

/// Projects positional objectives onto the permanent Jacobian's null space, then restores feasibility.
fn improve_targets(
    component: &Component,
    best: &mut Attempt,
    options: &SolveOptions,
    report: &mut SolveReport,
) {
    let mut radius = 1_f64;
    for iteration in 0..options.max_target_iterations {
        let soft = evaluate::soft(component, &best.x);
        if !soft.finite() {
            return;
        }
        let factor = Factor::new(&best.evaluation.jacobian, options.rank_tolerance);
        let mut projected = soft.jacobian.clone();
        for row in 0..projected.nrows() {
            let v = projected.row(row).transpose();
            let v = factor.null_projection(&v);
            projected.row_mut(row).copy_from(&v.transpose());
        }
        let target_factor = Factor::new(&projected, options.rank_tolerance);
        let mut step = factor.null_projection(&target_factor.correction(&(-&soft.values), 1e-10));
        let norm = step.norm();
        if !norm.is_finite() || norm < 1e-11 {
            return;
        }
        if norm > radius {
            step *= radius / norm;
        }
        let old_error = soft.values.norm();
        let mut accepted = false;
        let mut fraction = 1_f64;
        for _ in 0..16 {
            let trial = &best.x + &step * fraction;
            if component.valid(&trial) {
                let restored = restore(component, trial, options);
                report.iterations += restored.iterations;
                if restored.feasible {
                    let error = evaluate::soft(component, &restored.x).values.norm();
                    if error < old_error - 1e-13 * old_error.max(1e-6) {
                        *best = restored;
                        accepted = true;
                        break;
                    }
                }
            }
            fraction *= 0.5;
        }
        report.target_iterations += 1;
        if !accepted {
            return;
        }
        radius = (radius * 1.5).min(1e6);
        if iteration + 1 == options.max_target_iterations {
            report.target_budget_exhausted = true;
        }
    }
}

/// Thin SVD supplies minimum-norm corrections and projections even at deficient rank.
struct Factor {
    u: DMatrix<f64>,
    vt: DMatrix<f64>,
    singular: DVector<f64>,
    threshold: f64,
    columns: usize,
}
impl Factor {
    fn new(j: &DMatrix<f64>, tolerance: f64) -> Self {
        if j.nrows() == 0 || j.ncols() == 0 {
            return Self {
                u: DMatrix::zeros(j.nrows(), 0),
                vt: DMatrix::zeros(0, j.ncols()),
                singular: DVector::zeros(0),
                threshold: tolerance,
                columns: j.ncols(),
            };
        }
        let svd = j.clone().svd(true, true);
        let threshold = tolerance * svd.singular_values.max().max(1.);
        Self {
            u: svd.u.expect("requested left singular vectors"),
            vt: svd.v_t.expect("requested right singular vectors"),
            singular: svd.singular_values,
            threshold,
            columns: j.ncols(),
        }
    }
    fn rank(&self) -> usize {
        self.singular
            .iter()
            .filter(|s| **s > self.threshold)
            .count()
    }
    fn correction(&self, rhs: &DVector<f64>, damping: f64) -> DVector<f64> {
        let mut output = DVector::zeros(self.columns);
        for i in 0..self.singular.len() {
            let s = self.singular[i];
            if s <= self.threshold {
                continue;
            }
            let coefficient = self.u.column(i).dot(rhs) * s / (s * s + damping);
            for j in 0..self.columns {
                output[j] += self.vt[(i, j)] * coefficient;
            }
        }
        output
    }
    fn null_projection(&self, v: &DVector<f64>) -> DVector<f64> {
        let mut output = v.clone();
        for i in 0..self.singular.len() {
            if self.singular[i] <= self.threshold {
                continue;
            }
            let coefficient = self.vt.row(i).transpose().dot(v);
            for j in 0..self.columns {
                output[j] -= coefficient * self.vt[(i, j)];
            }
        }
        output
    }
}

fn residuals(
    scale: f64,
    evaluation: &Evaluation,
    options: &SolveOptions,
) -> Vec<ConstraintResidual> {
    evaluation
        .groups
        .iter()
        .map(|(key, range)| {
            let mut values = vec![];
            let mut max_ratio = 0_f64;
            for row in range.clone() {
                let angular = evaluation.angular[row];
                let physical = evaluation.values[row] * if angular { 1. } else { scale };
                let tolerance = if angular {
                    options.angular_tolerance
                } else {
                    options.linear_tolerance
                };
                let ratio = physical.abs() / tolerance;
                max_ratio = if ratio.is_finite() {
                    max_ratio.max(ratio)
                } else {
                    f64::INFINITY
                };
                values.push(physical);
            }
            ConstraintResidual {
                constraint: *key,
                values,
                max_tolerance_ratio: max_ratio,
            }
        })
        .collect()
}

/// Internal contact must have a unique contact point, excluding coincident circles.
fn contacts_valid(component: &Component, x: &DVector<f64>) -> bool {
    component.constraints.iter().all(|(_, c)| {
        let Constraint2d::TangentCircles { first, second, .. } = c.definition else {
            return true;
        };
        let a_offset = component.entry(first.0).offset;
        let b_offset = component.entry(second.0).offset;
        let a: Vec<_> = (0..3).map(|i| component.value(a_offset + i, x)).collect();
        let b: Vec<_> = (0..3).map(|i| component.value(b_offset + i, x)).collect();
        match c.tangency {
            TangencyMode::FirstContainsSecond => a[2] > b[2] && (a[0] != b[0] || a[1] != b[1]),
            TangencyMode::SecondContainsFirst => b[2] > a[2] && (a[0] != b[0] || a[1] != b[1]),
            _ => true,
        }
    })
}

/// Warns about structural duplicates and local row dependence without claiming nonlinear redundancy.
fn redundancies(component: &Component, evaluation: &Evaluation, tolerance: f64) -> Vec<Redundancy> {
    let mut basis: Vec<DVector<f64>> = vec![];
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
        let mut dependent = 0_usize;
        for row in range.clone() {
            let mut v = evaluation.jacobian.row(row).transpose();
            let norm = v.norm();
            for _ in 0..2 {
                for b in &basis {
                    let projection = b.dot(&v);
                    v -= b * projection;
                }
            }
            let remaining = v.norm();
            if remaining <= tolerance * norm.max(1.) {
                dependent += 1;
            } else {
                basis.push(v / remaining);
            }
        }
        dependent =
            dependent.saturating_sub(component.eliminated_rows.get(key).copied().unwrap_or(0));
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
    use Constraint2d::*;
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
