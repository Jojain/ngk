//! Always-on work counters for the geometric solvers.
//!
//! The solvers are reached through several layers of dispatch, so the only
//! practical place to answer "where did the time go" is inside the solvers
//! themselves. Each counter is a thread-local integer increment on a path that
//! already does floating-point work, which is cheap enough to leave enabled
//! rather than gate behind a feature that is never on when it is wanted.
//!
//! Counters are cumulative per thread. A caller measures one operation by
//! taking a [`SolverCounters::snapshot`] before and after and subtracting with
//! [`SolverCounters::since`].

use std::cell::Cell;

thread_local! {
    static COUNTERS: Cell<SolverCounters> = const { Cell::new(SolverCounters::ZERO) };
}

/// Cumulative solver work observed on the current thread.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SolverCounters {
    /// Surface/surface queries entered, however they were answered.
    pub surface_surface_calls: u64,
    /// Surface/surface queries answered in closed form by the analytic table.
    pub surface_surface_analytic_calls: u64,
    /// Curve/surface queries entered.
    pub curve_surface_calls: u64,
    /// Curve/surface queries answered in closed form by the analytic table.
    pub curve_surface_analytic_calls: u64,
    /// 3D curve/curve queries entered.
    pub curve_curve_calls: u64,
    /// 3D curve/curve queries answered in closed form by the analytic table.
    pub curve_curve_analytic_calls: u64,
    /// 2D curve/curve queries entered.
    pub curve_curve_2d_calls: u64,
    /// Embedding nodes visited across every bounded search.
    pub subdivision_nodes: u64,
    /// Corrector steps taken while marching a surface/surface branch.
    pub trace_steps: u64,
    /// Newton iterations run by any corrector.
    pub newton_iterations: u64,
    /// Bézier decompositions built for a surface.
    pub prepared_surfaces_built: u64,
    /// Bézier decompositions built for a curve.
    pub prepared_curves_built: u64,
    /// Branch fits attempted, including retraced ones.
    pub branch_fits: u64,
}

impl SolverCounters {
    const ZERO: Self = Self {
        surface_surface_calls: 0,
        surface_surface_analytic_calls: 0,
        curve_surface_calls: 0,
        curve_surface_analytic_calls: 0,
        curve_curve_calls: 0,
        curve_curve_analytic_calls: 0,
        curve_curve_2d_calls: 0,
        subdivision_nodes: 0,
        trace_steps: 0,
        newton_iterations: 0,
        prepared_surfaces_built: 0,
        prepared_curves_built: 0,
        branch_fits: 0,
    };

    /// Returns the counters accumulated on this thread so far.
    pub fn snapshot() -> Self {
        COUNTERS.with(Cell::get)
    }

    /// Returns the work done since `earlier` was taken.
    pub fn since(self, earlier: Self) -> Self {
        Self {
            surface_surface_calls: self.surface_surface_calls - earlier.surface_surface_calls,
            surface_surface_analytic_calls: self.surface_surface_analytic_calls
                - earlier.surface_surface_analytic_calls,
            curve_surface_calls: self.curve_surface_calls - earlier.curve_surface_calls,
            curve_surface_analytic_calls: self.curve_surface_analytic_calls
                - earlier.curve_surface_analytic_calls,
            curve_curve_calls: self.curve_curve_calls - earlier.curve_curve_calls,
            curve_curve_analytic_calls: self.curve_curve_analytic_calls
                - earlier.curve_curve_analytic_calls,
            curve_curve_2d_calls: self.curve_curve_2d_calls - earlier.curve_curve_2d_calls,
            subdivision_nodes: self.subdivision_nodes - earlier.subdivision_nodes,
            trace_steps: self.trace_steps - earlier.trace_steps,
            newton_iterations: self.newton_iterations - earlier.newton_iterations,
            prepared_surfaces_built: self.prepared_surfaces_built - earlier.prepared_surfaces_built,
            prepared_curves_built: self.prepared_curves_built - earlier.prepared_curves_built,
            branch_fits: self.branch_fits - earlier.branch_fits,
        }
    }

    /// Returns whether no work at all was recorded.
    pub fn is_zero(&self) -> bool {
        *self == Self::ZERO
    }
}

/// Applies `update` to this thread's counters.
#[inline]
fn record(update: impl FnOnce(&mut SolverCounters)) {
    COUNTERS.with(|counters| {
        let mut current = counters.get();
        update(&mut current);
        counters.set(current);
    });
}

/// Counts one surface/surface query.
#[inline]
pub(crate) fn count_surface_surface_call() {
    record(|counters| counters.surface_surface_calls += 1);
}

/// Counts one surface/surface query answered in closed form.
#[inline]
pub(crate) fn count_surface_surface_analytic_call() {
    record(|counters| counters.surface_surface_analytic_calls += 1);
}

/// Counts one curve/surface query.
#[inline]
pub(crate) fn count_curve_surface_call() {
    record(|counters| counters.curve_surface_calls += 1);
}

/// Counts one curve/surface query answered in closed form.
#[inline]
pub(crate) fn count_curve_surface_analytic_call() {
    record(|counters| counters.curve_surface_analytic_calls += 1);
}

/// Counts one 3D curve/curve query.
#[inline]
pub(crate) fn count_curve_curve_call() {
    record(|counters| counters.curve_curve_calls += 1);
}

/// Counts one 3D curve/curve query answered in closed form.
#[inline]
pub(crate) fn count_curve_curve_analytic_call() {
    record(|counters| counters.curve_curve_analytic_calls += 1);
}

/// Counts one 2D curve/curve query.
#[inline]
pub(crate) fn count_curve_curve_2d_call() {
    record(|counters| counters.curve_curve_2d_calls += 1);
}

/// Counts one embedding node visit.
#[inline]
pub(crate) fn count_subdivision_node() {
    record(|counters| counters.subdivision_nodes += 1);
}

/// Counts one branch trace step.
#[inline]
pub(crate) fn count_trace_step() {
    record(|counters| counters.trace_steps += 1);
}

/// Counts `iterations` Newton iterations.
#[inline]
pub(crate) fn count_newton_iterations(iterations: usize) {
    record(|counters| counters.newton_iterations += iterations as u64);
}

/// Counts one surface Bézier decomposition.
#[inline]
pub(crate) fn count_prepared_surface() {
    record(|counters| counters.prepared_surfaces_built += 1);
}

/// Counts one curve Bézier decomposition.
#[inline]
pub(crate) fn count_prepared_curve() {
    record(|counters| counters.prepared_curves_built += 1);
}

/// Counts one branch fit attempt.
#[inline]
pub(crate) fn count_branch_fit() {
    record(|counters| counters.branch_fits += 1);
}
