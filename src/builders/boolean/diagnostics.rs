//! Observable work and unresolved geometry from Boolean preparation.

use std::cell::Cell;
use std::cmp::Reverse;
use std::time::Duration;

use crate::geometry::{IntersectionIncompleteReason, SolverCounters};
use crate::topology::shape_keys::FaceKey;

thread_local! {
    /// Trim domains built on this thread, cumulative.
    ///
    /// `FaceTrimDomain::new` is reached from seven call sites across four
    /// modules, none of which carries the diagnostics, so the count is
    /// collected the same way the solver counters are.
    static TRIM_DOMAINS_BUILT: Cell<u64> = const { Cell::new(0) };
}

/// Counts one trim domain construction.
pub(super) fn count_trim_domain_built() {
    TRIM_DOMAINS_BUILT.with(|built| built.set(built.get() + 1));
}

/// Returns the trim domains built on this thread so far.
pub(super) fn trim_domains_built() -> u64 {
    TRIM_DOMAINS_BUILT.with(Cell::get)
}

/// Wall-clock time attributed to each Boolean stage.
///
/// Wall clock rather than a sample profile: the point is to say which stage a
/// slow or hanging Boolean is in, which no aggregate can answer.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BooleanStageTimings {
    pub vertex_contacts: Duration,
    pub edge_contacts: Duration,
    pub edge_face_contacts: Duration,
    pub face_contacts: Duration,
    pub imprint_normalization: Duration,
    pub network: Duration,
    pub splitting: Duration,
    pub classification: Duration,
    pub assembly: Duration,
}

impl BooleanStageTimings {
    /// Returns the total time attributed to the stages measured.
    pub fn total(&self) -> Duration {
        self.vertex_contacts
            + self.edge_contacts
            + self.edge_face_contacts
            + self.face_contacts
            + self.imprint_normalization
            + self.network
            + self.splitting
            + self.classification
            + self.assembly
    }

    /// Returns each stage's name and elapsed time, longest first.
    pub fn ranked(&self) -> Vec<(&'static str, Duration)> {
        let mut stages = vec![
            ("vertex_contacts", self.vertex_contacts),
            ("edge_contacts", self.edge_contacts),
            ("edge_face_contacts", self.edge_face_contacts),
            ("face_contacts", self.face_contacts),
            ("imprint_normalization", self.imprint_normalization),
            ("network", self.network),
            ("splitting", self.splitting),
            ("classification", self.classification),
            ("assembly", self.assembly),
        ];
        stages.sort_by_key(|(_, elapsed)| Reverse(*elapsed));
        stages
    }
}

/// Work counters and coverage limitations retained with the contact plan.
#[derive(Debug, Clone, Default)]
pub struct BooleanDiagnostics {
    pub tolerances: super::BooleanTolerances,
    pub fragments: usize,
    pub components: usize,
    pub classification_rays: usize,
    pub candidate_pairs_tested: usize,
    pub candidate_pairs_pruned: usize,
    pub edge_face_pairs_tested: usize,
    pub edge_face_pairs_pruned: usize,
    pub branches_found: usize,
    pub branches_uncertified: usize,
    pub spans: usize,
    pub events: usize,
    pub regions: usize,
    /// Trim domains built.
    ///
    /// Contact computation builds one per face and reuses it, so a count above
    /// the number of participating faces means a domain was built outside that
    /// cache -- during splitting or classification, which hold no cache.
    pub trim_domains_built: u64,
    /// Solver work this Boolean is responsible for.
    pub solver: SolverCounters,
    /// Wall-clock time per stage.
    pub stages: BooleanStageTimings,
    /// Candidate overlap is not proof of a coincident trimmed region.
    pub unresolved_overlaps: Vec<(FaceKey, FaceKey)>,
    pub coverage: Vec<IntersectionIncompleteReason>,
}

impl BooleanDiagnostics {
    /// Renders the stage and solver profile as one human-readable block.
    ///
    /// Written for a bench harness or a failing repro rather than for a log:
    /// it answers "which stage, and how much solver work" in one read.
    pub fn profile(&self) -> String {
        let mut report = format!("total {:?}\n", self.stages.total());
        for (stage, elapsed) in self.stages.ranked() {
            if !elapsed.is_zero() {
                report.push_str(&format!("  {stage:<22} {elapsed:?}\n"));
            }
        }
        report.push_str(&format!(
            "  surface/surface calls  {} ({} analytic)\n  \
             curve/surface calls    {} ({} analytic)\n  \
             curve/curve calls      {} 3D ({} analytic), {} 2D\n  \
             subdivision nodes      {}\n  \
             trace steps            {}\n  \
             newton iterations      {}\n  \
             branch fits            {}\n  \
             prepared surfaces      {}\n  \
             prepared curves        {}\n  \
             trim domains           {}\n",
            self.solver.surface_surface_calls,
            self.solver.surface_surface_analytic_calls,
            self.solver.curve_surface_calls,
            self.solver.curve_surface_analytic_calls,
            self.solver.curve_curve_calls,
            self.solver.curve_curve_analytic_calls,
            self.solver.curve_curve_2d_calls,
            self.solver.subdivision_nodes,
            self.solver.trace_steps,
            self.solver.newton_iterations,
            self.solver.branch_fits,
            self.solver.prepared_surfaces_built,
            self.solver.prepared_curves_built,
            self.trim_domains_built,
        ));
        report
    }
}
