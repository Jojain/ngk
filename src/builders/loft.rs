//! Skinning a sequence of sections.
//!
//! A loft takes an ordered sequence of `N` sections and builds the shape
//! passing through all of them. Where an extrusion sweeps along a vector and a
//! revolution sweeps around an axis, a loft sweeps *between stated sections*
//! with no generating motion at all — so where those two derive every face
//! from one source edge, a loft is handed sections authored independently and
//! has to *establish* the correspondence before it can build anything.
//!
//! # Correspondence
//!
//! Three obligations, in order:
//!
//! 1. **One parametrization per section.** [`ProfileCurve`] reduces each to a
//!    traversal over `[0, 1]`, which is what makes sections with unlike edge
//!    counts comparable at all.
//! 2. **One breakpoint set.** The union of every section's vertex parameters,
//!    merged within tolerance, cuts every section into the same number of
//!    pieces. Those pieces are the loft's **columns**, and each becomes one
//!    face. A section is cut at positions that came from other sections; that
//!    is the point.
//! 3. **One direction and one seam.** [`agree_directions`] and [`align_seams`]
//!    settle both against section 0 rather than pairwise, because `N` locally
//!    consistent pairings can still spiral.
//!
//! # An intermediate section is not a boundary
//!
//! There is one face per column, spanning all `N` sections, so an intermediate
//! section contributes no vertex, no edge and no face: a loft is smooth where
//! it crosses one, and an edge recorded there would assert a discontinuity the
//! geometry does not have. What it does contribute is its **breakpoints** —
//! a corner in an intermediate section is a real crease, and splitting at it
//! puts that crease on a column boundary, which is a rail, which is an edge.
//! Skip it and the crease sits in a face interior where nothing records it.
//!
//! # The kind of section decides the kind of result
//!
//! [`OpenSection`], [`ClosedSection`] and [`CappedSection`] are three types
//! rather than three cases of one, and [`add_loft`] takes a slice of one of
//! them. Two things follow, and both are the point:
//!
//! - **A mixed run cannot be spelled.** Lofting an open section into a closed
//!   run is a request with no meaning, and it is now unwritable rather than
//!   reported.
//! - **The result follows the input.** Wires loft to a [`SheetKey`] and faces
//!   to a [`SolidKey`], through [`LoftSection::Output`], so no caller unwraps
//!   a result whose shape it already knows.
//!
//! Each is built by a constructor that checks what its type promises, so a
//! [`ClosedSection`] is made only from a [`Closed`] profile and a
//! [`CappedSection`] only from a face a loft can actually cap.

use std::collections::{HashMap, HashSet};

use nalgebra::Vector3;
use thiserror::Error;

use crate::builders::errors::ModelEditFailure;
use crate::builders::faces::{reverse_face_winding, split_face_edge_staged};
use crate::builders::scaffold::cut_between_loops;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Axis2, Curve, Degree, LINEAR_TOLERANCE, NurbsCurve, NurbsError, NurbsSurface, Point2, Point3,
    PointCoincidence, Surface, TrimmedCurve, TrimmedCurve2, make_compatible,
};
use crate::model::Model;
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, LoopDefinition, LoopKind, ProfileAttr, SheetAttr, SolidAttr, VertexAttr,
};
use crate::topology::closed::{Closeable, Closed};
use crate::topology::edge::Edge;
use crate::topology::edit::{ModelEdit, ModelEditError};
use crate::topology::embedding::EntityOwner;
use crate::topology::face::Face;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::{DefaultPayload, Payload};
use crate::topology::profile::Profile;
use crate::topology::profile_curve::{
    ProfileCurve, ProfileCurveError, agree_directions, align_seams,
};
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use crate::topology::vertex::Vertex;

mod sealed {
    pub trait Sealed {}
    impl Sealed for super::OpenSection {}
    impl Sealed for super::ClosedSection {}
    impl Sealed for super::CappedSection {}
}

/// An open wire to loft through. The sheet stays open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OpenSection(ProfileKey);

impl OpenSection {
    /// Reads an open profile as a section.
    ///
    /// Refuses a closed profile. An open section's two ends are where the
    /// sheet stops, and a closed profile has no ends to stop at — what it
    /// wants is a [`ClosedSection`], which closes the ring instead.
    pub fn new<P: Payload>(profile: &Profile<'_, P>) -> Result<Self, LoftError> {
        if profile.is_closed() {
            return Err(LoftError::ClosedProfileAsOpenSection);
        }
        Ok(Self(profile.key()))
    }
}

/// A closed wire to loft through. The ring between the last and the first
/// column closes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClosedSection(ProfileKey);

impl ClosedSection {
    /// Reads a closed profile as a section.
    ///
    /// Infallible: [`Closed`] already carries the proof, so there is nothing
    /// left here to check.
    pub fn new<P: Payload>(profile: &Closed<Profile<'_, P>>) -> Self {
        Self(profile.key())
    }
}

/// A face to loft through. The ring closes and the two end faces become caps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CappedSection(FaceKey);

impl CappedSection {
    /// Reads a face as a section, cap included.
    ///
    /// Refuses a face with holes. A face with a hole lofted to another with a
    /// hole has a correspondence problem between the holes on top of the one
    /// between the outer loops, and a cardinality question when the counts
    /// differ. Lofting the outer loops alone would leave each cap with a hole
    /// nothing walls in, so the solid would be open where the shell reports
    /// itself closed.
    ///
    /// Planarity is *not* required: a cap keeps its own face's surface, and a
    /// section is a closed curve in space whether or not one plane holds it.
    pub fn new<P: Payload>(face: &Face<'_, P>) -> Result<Self, LoftError> {
        let inner = face.inner_loops().len();
        if inner > 0 {
            return Err(LoftError::FaceHasInnerLoops { count: inner });
        }
        if face.outer_loop().is_none() {
            return Err(LoftError::FaceHasNoOuterLoop);
        }
        Ok(Self(face.key()))
    }
}

/// One kind of section, and what a loft through sections of that kind makes.
///
/// Sealed: the three kinds are the three the algorithm has tails for, and a
/// fourth would need a tail of its own rather than an implementation of this.
pub trait LoftSection: Copy + sealed::Sealed {
    /// What a loft through sections of this kind produces.
    type Output;

    /// Whether the ring between the last and the first column closes.
    const CLOSES_RING: bool;

    /// The traversal this section contributes, read out of `model`.
    fn traversal<'m, P: Payload>(
        self,
        model: &'m Model<P>,
        index: usize,
    ) -> Result<ProfileCurve<'m, P>, LoftError>;

    /// Readies this section's entity for the columns about to meet it.
    ///
    /// `cuts` are the points where a column boundary lands on it. Only an end
    /// section is prepared, and only a face has anything to prepare.
    fn prepare_end<P: Payload>(
        self,
        edit: &mut ModelEdit<'_, P>,
        index: usize,
        cuts: &[Point3],
    ) -> Result<(), LoftError>;

    /// Closes the run at its two ends and registers the result.
    fn finish<P: Payload>(
        edit: &mut ModelEdit<'_, P>,
        columns: &LoftColumns,
        ends: [Self; 2],
    ) -> Result<Self::Output, LoftError>;
}

impl LoftSection for OpenSection {
    type Output = SheetKey;
    const CLOSES_RING: bool = false;

    fn traversal<'m, P: Payload>(
        self,
        model: &'m Model<P>,
        index: usize,
    ) -> Result<ProfileCurve<'m, P>, LoftError> {
        wire_traversal(model, index, self.0)
    }

    fn prepare_end<P: Payload>(
        self,
        _edit: &mut ModelEdit<'_, P>,
        _index: usize,
        _cuts: &[Point3],
    ) -> Result<(), LoftError> {
        Ok(())
    }

    fn finish<P: Payload>(
        edit: &mut ModelEdit<'_, P>,
        columns: &LoftColumns,
        _ends: [Self; 2],
    ) -> Result<SheetKey, LoftError> {
        Ok(columns.register_sheet(edit))
    }
}

impl LoftSection for ClosedSection {
    type Output = SheetKey;
    const CLOSES_RING: bool = true;

    fn traversal<'m, P: Payload>(
        self,
        model: &'m Model<P>,
        index: usize,
    ) -> Result<ProfileCurve<'m, P>, LoftError> {
        wire_traversal(model, index, self.0)
    }

    fn prepare_end<P: Payload>(
        self,
        _edit: &mut ModelEdit<'_, P>,
        _index: usize,
        _cuts: &[Point3],
    ) -> Result<(), LoftError> {
        Ok(())
    }

    fn finish<P: Payload>(
        edit: &mut ModelEdit<'_, P>,
        columns: &LoftColumns,
        _ends: [Self; 2],
    ) -> Result<SheetKey, LoftError> {
        Ok(columns.register_sheet(edit))
    }
}

impl LoftSection for CappedSection {
    type Output = SolidKey;
    const CLOSES_RING: bool = true;

    fn traversal<'m, P: Payload>(
        self,
        model: &'m Model<P>,
        index: usize,
    ) -> Result<ProfileCurve<'m, P>, LoftError> {
        let face = model
            .face(self.0)
            .ok_or(LoftError::MissingSection { index })?;
        let boundary = face
            .outer_loop()
            .ok_or(LoftError::MissingOuterLoop { index })?;
        ProfileCurve::from_loop(&boundary).map_err(|source| LoftError::Section { index, source })
    }

    fn prepare_end<P: Payload>(
        self,
        edit: &mut ModelEdit<'_, P>,
        index: usize,
        cuts: &[Point3],
    ) -> Result<(), LoftError> {
        for cut in cuts {
            cut_cap_at(edit, self.0, index, *cut)?;
        }
        Ok(())
    }

    fn finish<P: Payload>(
        edit: &mut ModelEdit<'_, P>,
        columns: &LoftColumns,
        ends: [Self; 2],
    ) -> Result<SolidKey, LoftError> {
        let caps = [ends[0].0, ends[1].0];
        for (slot, cap) in caps.into_iter().enumerate() {
            columns.sew_cap(edit, cap, slot)?;
        }
        columns.orient_shell(edit, caps);

        // Contextual, like an extruded shell: the dart must keep the outward
        // orientation just established for the bottom cap.
        let shell = edit.face_attr_unchecked(caps[0]).outer_unchecked();
        if edit.sheet_key(shell).is_none() {
            edit.add_sheet(SheetAttr::new(shell));
        }
        Ok(edit.add_solid(SolidAttr::new(shell, None)))
    }
}

/// One traversal of a profile section, in the sense the profile is walked in.
fn wire_traversal<P: Payload>(
    model: &Model<P>,
    index: usize,
    key: ProfileKey,
) -> Result<ProfileCurve<'_, P>, LoftError> {
    let profile = model
        .profile(key)
        .ok_or(LoftError::MissingSection { index })?;
    ProfileCurve::from_profile(&profile).map_err(|source| LoftError::Section { index, source })
}

/// How a loft reads its sections.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LoftOptions {
    /// The degree of the skin across the sections.
    ///
    /// `None` takes `min(3, N - 1)`, the highest degree the section count
    /// supports up to a cubic. A degree of `1` gives a surface linear in `v`
    /// between consecutive sections — the multi-section generalization of a
    /// ruled surface, and what a CAD user means by a "ruled" rather than
    /// "smooth" loft. Nothing branches on it: the two are one construction
    /// with a different degree.
    pub v_degree: Option<Degree>,
}

impl LoftOptions {
    /// Straight between consecutive sections.
    pub fn ruled() -> Self {
        Self {
            v_degree: Some(Degree::new(1).expect("1 is a valid degree")),
        }
    }

    /// The degree to skin `sections` sections at.
    fn degree(self, sections: usize) -> Result<Degree, NurbsError> {
        match self.v_degree {
            Some(degree) if degree.get() < sections => Ok(degree),
            Some(degree) => Err(NurbsError::SkinningDegreeTooHigh {
                degree: degree.get(),
                sections,
            }),
            None => Degree::new(3.min(sections - 1)),
        }
    }
}

/// How a loft fails.
#[derive(Debug, Clone, Error, PartialEq)]
pub enum LoftError {
    /// A closed profile was offered as an open section.
    #[error("a closed profile has no ends to leave open; loft it as a closed section")]
    ClosedProfileAsOpenSection,
    /// A face offered as a section has holes.
    #[error("a face with {count} inner loops has holes a loft cannot correspond")]
    FaceHasInnerLoops { count: usize },
    /// A face offered as a section has no outer boundary to read.
    #[error("a face with no outer loop names no section")]
    FaceHasNoOuterLoop,
    /// Fewer than two sections were given; there is nothing to skin between.
    #[error("a loft needs at least 2 sections, got {got}")]
    TooFewSections { got: usize },
    /// A run of profiles is neither all open nor all closed.
    ///
    /// [`add_loft`] cannot be handed a mixed run at all — the section types
    /// see to that. This is for a caller that infers the kind from the
    /// profiles themselves, where inference is what has no answer.
    #[error("profile {index} is not closed the way the run's first profile is")]
    MixedProfiles { index: usize },
    /// A section's entity is no longer in the model.
    #[error("section {index} names no registered entity")]
    MissingSection { index: usize },
    /// A face section lost the outer boundary it was built from.
    #[error("section {index} is a face with no outer loop")]
    MissingOuterLoop { index: usize },
    /// A section could not be read as one traversal.
    #[error("section {index} is not one traversal")]
    Section {
        index: usize,
        #[source]
        source: ProfileCurveError,
    },
    /// One column's curves could not be skinned.
    ///
    /// Named by column rather than by loft as a whole: with fifty columns a
    /// bare failure is unactionable.
    #[error("column {column} of {sections} sections could not be skinned")]
    Column {
        column: usize,
        sections: usize,
        #[source]
        source: NurbsError,
    },
    /// A cap's boundary could not be cut where a column meets it.
    #[error("the cap of section {index} could not be cut at {point:?}")]
    CapNotCuttable { index: usize, point: Point3 },
    /// A column has no counterpart on a cap to be sewn to.
    #[error("column {column} has no matching edge on the cap at end {slot}")]
    CapEdgeMissing { column: usize, slot: usize },
    /// Two darts that should have joined would not.
    #[error("darts {first:?} and {second:?} are not sewable in dimension {dim:?}")]
    SewFailed { dim: Dim, first: Dart, second: Dart },
    /// A dart that should have carried an edge did not.
    #[error("dart {dart:?} carries no edge to sew")]
    MissingEdge { dart: Dart },
    /// The transaction could not be carried out.
    #[error("loft model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for LoftError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}

/// Builds the shape passing through every section of `sections`, in order.
///
/// The section type decides the result: [`OpenSection`] and [`ClosedSection`]
/// give a [`SheetKey`], [`CappedSection`] a [`SolidKey`]. See the
/// [module documentation](self) for how correspondence between sections is
/// established.
pub fn add_loft<S: LoftSection, P: DefaultPayload>(
    g: &mut Model<P>,
    sections: &[S],
    options: LoftOptions,
) -> Result<S::Output, LoftError> {
    let plan = LoftPlan::read(g, sections, options)?;
    g.transaction(|edit| plan.build(edit))
}

/// Everything the loft needs from the model, read before anything is touched.
///
/// The sections arrive as typed views borrowing the model, and building
/// mutates it, so the reading happens once and what it derives travels as
/// owned geometry. That is also what lets the cap faces be cut at the column
/// boundaries: the cuts are named by the points they land on, which survive
/// the edits that finding them could not.
struct LoftPlan<S: LoftSection> {
    /// The first and last section, which are the two that close the run.
    ends: [S; 2],
    /// One entry per column, each holding that column's curve on every
    /// section, in section order.
    columns: Vec<Vec<NurbsCurve>>,
    /// Where each column starts and ends on the first and last section, which
    /// is what a cap edge is matched against.
    cap_seams: [Vec<(Point3, Point3)>; 2],
    /// The corner at the start of the first and last traversal, where one is.
    ///
    /// A wrapping band's boundary is one closed edge per end section, and an
    /// edge is marked or unmarked by whether a corner sits on it. Read here
    /// because the traversal that knows is gone by the time the edge is built.
    end_corners: [Option<Point3>; 2],
    /// The direction the loft runs, from the first section to the last.
    advance: Vector3<f64>,
    /// Whether the first section's traversal turns the way `advance` points.
    ///
    /// A lateral face carries `dS/du x dS/dv`, which is `tangent x advance`,
    /// and that is the outward direction exactly when the traversal turns
    /// right-handed about the run. So this one sign decides the winding of
    /// every lateral face at once.
    laterals_face_outward: bool,
    degree_v: Degree,
    /// Whether the run's one column closes onto itself.
    ///
    /// A closed run whose sections share no breakpoint — a pair of circles —
    /// has one column whose two ends are the same curve. Building it as a quad
    /// and sewing those ends together would record a seam edge where the
    /// surface has no crease at all, so the column is built as a **band**
    /// instead: two closed boundary loops, no rails, and nothing between them
    /// but the face.
    wraps: bool,
}

impl<S: LoftSection> LoftPlan<S> {
    /// Reads the sections and works out every column's geometry.
    fn read<P: Payload>(
        model: &Model<P>,
        sections: &[S],
        options: LoftOptions,
    ) -> Result<Self, LoftError> {
        let (Some(first), Some(last)) = (sections.first(), sections.last()) else {
            return Err(LoftError::TooFewSections { got: 0 });
        };
        if sections.len() < 2 {
            return Err(LoftError::TooFewSections {
                got: sections.len(),
            });
        }

        let mut traversals = sections
            .iter()
            .enumerate()
            .map(|(index, section)| section.traversal(model, index))
            .collect::<Result<Vec<_>, _>>()?;
        agree_directions(&mut traversals);
        align_seams(&mut traversals).map_err(|source| LoftError::Section { index: 0, source })?;

        let union = breakpoint_union(&traversals);
        let pieces = traversals
            .iter()
            .enumerate()
            .map(|(index, traversal)| {
                traversal
                    .subdivided(&union)
                    .map_err(|source| LoftError::Section { index, source })
            })
            .collect::<Result<Vec<_>, _>>()?;

        let degree_v = options
            .degree(sections.len())
            .map_err(|source| LoftError::Column {
                column: 0,
                sections: sections.len(),
                source,
            })?;
        let columns = (0..union.len() - 1)
            .map(|column| compatible_column(&pieces, column, sections.len()))
            .collect::<Result<Vec<_>, _>>()?;

        let ends = [0, traversals.len() - 1];
        let advance = center(&pieces[ends[1]]) - center(&pieces[ends[0]]);
        Ok(Self {
            ends: [*first, *last],
            cap_seams: ends.map(|end| seams(&pieces[end])),
            end_corners: ends.map(|end| start_corner(&traversals[end])),
            advance,
            laterals_face_outward: traversals[0].turning_normal().dot(&advance) > 0.0,
            wraps: S::CLOSES_RING && columns.len() == 1,
            columns,
            degree_v,
        })
    }

    /// Builds the columns, sews them, and finishes according to the kind.
    fn build<P: Payload>(self, edit: &mut ModelEdit<'_, P>) -> Result<S::Output, LoftError> {
        self.prepare_ends(edit)?;

        let mut faces = Vec::with_capacity(self.columns.len());
        for (column, curves) in self.columns.iter().enumerate() {
            let skin = NurbsSurface::skinned(curves, self.degree_v).map_err(|source| {
                LoftError::Column {
                    column,
                    sections: curves.len(),
                    source,
                }
            })?;
            faces.push(if self.wraps {
                add_loft_band_face(edit, column, skin, self.end_corners)?
            } else {
                add_loft_quad_face(edit, column, skin)?
            });
        }

        for pair in faces.windows(2) {
            if let (Some(before), Some(after)) = (pair[0].rails, pair[1].rails) {
                sew_edges(edit, before.end, after.start)?;
            }
        }
        // One column closes the ring onto itself, but only where it was built
        // as a quad: a band has no rail to sew, because it is already closed.
        if S::CLOSES_RING
            && let (Some(last), Some(first)) = (
                faces.last().and_then(|face| face.rails),
                faces.first().and_then(|face| face.rails),
            )
        {
            sew_edges(edit, last.end, first.start)?;
        }

        let columns = LoftColumns {
            faces,
            cap_seams: self.cap_seams,
            advance: self.advance,
            laterals_face_outward: self.laterals_face_outward,
        };
        S::finish(edit, &columns, self.ends)
    }

    /// Cuts each end section's entity where the columns will meet it.
    ///
    /// A wrapping run has no interior column boundary, so the one place its
    /// seams name is where the sections close — which bounds nothing and must
    /// not be cut onto a cap.
    fn prepare_ends<P: Payload>(&self, edit: &mut ModelEdit<'_, P>) -> Result<(), LoftError> {
        for (slot, section) in self.ends.into_iter().enumerate() {
            let cuts: Vec<Point3> = if self.wraps {
                Vec::new()
            } else {
                self.cap_seams[slot]
                    .iter()
                    .map(|(start, _)| *start)
                    .collect()
            };
            let index = if slot == 0 { 0 } else { self.columns.len() };
            section.prepare_end(edit, index, &cuts)?;
        }
        Ok(())
    }
}

/// The column faces a loft built, and what closing its two ends takes.
///
/// Opaque by design: it exists so each [`LoftSection`] kind can state its own
/// tail, and carries nothing a caller outside this module can act on.
pub struct LoftColumns {
    faces: Vec<LoftColumnFace>,
    cap_seams: [Vec<(Point3, Point3)>; 2],
    advance: Vector3<f64>,
    laterals_face_outward: bool,
}

impl LoftColumns {
    /// Registers the lateral faces as one sheet.
    fn register_sheet<P: Payload>(&self, edit: &mut ModelEdit<'_, P>) -> SheetKey {
        let dart = self.faces[0].anchor;
        edit.add_sheet(SheetAttr::new(dart))
    }

    /// Sews every column's section edge onto the cap it came from.
    ///
    /// `slot` is `0` for the first section and `1` for the last.
    fn sew_cap<P: Payload>(
        &self,
        edit: &mut ModelEdit<'_, P>,
        cap: FaceKey,
        slot: usize,
    ) -> Result<(), LoftError> {
        for (column, face) in self.faces.iter().enumerate() {
            let (start, end) = self.cap_seams[slot][column];
            let cap_dart = cap_dart_between(edit, cap, start, end)
                .ok_or(LoftError::CapEdgeMissing { column, slot })?;
            let side = if slot == 0 {
                face.first_section
            } else {
                face.last_section
            };
            sew_edges(edit, cap_dart, side)?;
        }
        Ok(())
    }

    /// Makes every face of the shell point away from the material.
    ///
    /// The bottom cap must face against the run and the top cap with it; the
    /// laterals all agree with one another by construction, so one sign turns
    /// them all.
    fn orient_shell<P: Payload>(&self, edit: &mut ModelEdit<'_, P>, caps: [FaceKey; 2]) {
        if !self.laterals_face_outward {
            for face in &self.faces {
                reverse_face_winding(edit, face.key);
            }
        }
        if cap_normal_along_run(edit, caps[0], self.advance) > 0.0 {
            reverse_face_winding(edit, caps[0]);
        }
        if cap_normal_along_run(edit, caps[1], self.advance) < 0.0 {
            reverse_face_winding(edit, caps[1]);
        }
    }
}

/// The breakpoints of every section, merged into one set of column bounds.
///
/// The merge tolerance is not [`LINEAR_TOLERANCE`], because a fraction is not
/// a distance. Each section derives one from its own length, and the coarsest
/// governs: two corners closer than that are indistinguishable in space on the
/// shortest section, and admitting both would give every column a hairline
/// neighbour.
///
/// `0` and `1` are always bounds, whatever the sections said, since they are
/// where the run begins and ends.
fn breakpoint_union<P: Payload>(traversals: &[ProfileCurve<'_, P>]) -> Vec<Fraction> {
    let tolerance = traversals
        .iter()
        .map(ProfileCurve::merge_tolerance)
        .fold(0.0, f64::max);

    let mut all = vec![0.0, 1.0];
    all.extend(
        traversals
            .iter()
            .flat_map(ProfileCurve::breakpoints)
            .map(|fraction| fraction.value()),
    );
    all.sort_by(f64::total_cmp);

    let mut bounds: Vec<f64> = Vec::new();
    let mut cluster: Vec<f64> = Vec::new();
    for value in all {
        if cluster
            .first()
            .is_some_and(|first| value - first > tolerance)
        {
            bounds.push(cluster.iter().sum::<f64>() / cluster.len() as f64);
            cluster.clear();
        }
        cluster.push(value);
    }
    bounds.push(cluster.iter().sum::<f64>() / cluster.len() as f64);

    // The two ends are exact, not averages: a column that began a hair past
    // `0` would leave a sliver of every section outside the loft.
    let last = bounds.len() - 1;
    bounds[0] = 0.0;
    bounds[last] = 1.0;
    bounds.into_iter().map(Fraction::new).collect()
}

/// The `N` curves of one column, brought onto a common degree and knot vector.
fn compatible_column(
    pieces: &[Vec<TrimmedCurve>],
    column: usize,
    sections: usize,
) -> Result<Vec<NurbsCurve>, LoftError> {
    let named = |source| LoftError::Column {
        column,
        sections,
        source,
    };
    let mut curves = pieces
        .iter()
        .map(|section| match section[column].to_curve().map_err(named)? {
            Curve::Nurbs(curve) => Ok(curve),
            other => other.to_nurbs().map_err(named),
        })
        .collect::<Result<Vec<_>, _>>()?;
    make_compatible(&mut curves).map_err(named)?;
    Ok(curves)
}

/// Where each column starts and ends on one section.
fn seams(pieces: &[TrimmedCurve]) -> Vec<(Point3, Point3)> {
    pieces
        .iter()
        .map(|piece| (piece.start(), piece.end()))
        .collect()
}

/// The point of the corner a traversal begins on, where it begins on one.
fn start_corner<P: Payload>(traversal: &ProfileCurve<'_, P>) -> Option<Point3> {
    traversal
        .spans()
        .first()
        .filter(|span| span.starts_at_corner())
        .map(|_| traversal.point_at(Fraction::START))
}

/// The mean of a section's column bounds, as a stand-in for its position.
fn center(pieces: &[TrimmedCurve]) -> Point3 {
    let sum = pieces
        .iter()
        .fold(Vector3::zeros(), |acc, piece| acc + piece.start().coords);
    Point3::from(sum / pieces.len() as f64)
}

/// The two rails a column shares with its neighbours.
#[derive(Debug, Clone, Copy)]
struct ColumnRails {
    /// The rail at `u = 0`, at the column's `(0, 0)` corner.
    start: Dart,
    /// The rail at `u = 1`, at the column's `(1, 0)` corner.
    end: Dart,
}

/// One column's face, with the darts every later sew needs.
struct LoftColumnFace {
    key: FaceKey,
    /// The dart on the first section's boundary, where the cap's edge sits.
    first_section: Dart,
    /// The dart on the last section's boundary, where the cap's edge sits.
    last_section: Dart,
    /// The rails, absent on a column that closes onto itself.
    rails: Option<ColumnRails>,
    /// A boundary dart, for rooting the sheet.
    anchor: Dart,
}

/// The four corners of the `[0, 1]²` domain box, in loop order.
///
/// A skinned surface's parameterization *is* the quad, so the sides joining
/// these are a column face's pcurves with no fitting at all.
const BOX_CORNERS: [Point2; 4] = [
    Point2::new(0.0, 0.0),
    Point2::new(1.0, 0.0),
    Point2::new(1.0, 1.0),
    Point2::new(0.0, 1.0),
];

/// Adds the quad face one column makes.
///
/// Eight darts, four vertices, four edges, one profile, one face — the shape
/// every swept quad in this crate has. What is particular to a loft is where
/// the four boundary curves come from: the two section rows and the two rails
/// are all **isocurves of the skin itself**, extracted exactly. A rail is
/// curved whenever there are more than two sections, and one derived any other
/// way would not lie on the two faces that share it.
fn add_loft_quad_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    column: usize,
    skin: NurbsSurface,
) -> Result<LoftColumnFace, LoftError> {
    let named = |source| LoftError::Column {
        column,
        sections: skin.control_points().nv(),
        source,
    };
    let boundary = [
        Curve::Nurbs(skin.isocurve_v(0.0).map_err(named)?),
        Curve::Nurbs(skin.isocurve_u(1.0).map_err(named)?),
        Curve::Nurbs(skin.isocurve_v(1.0).map_err(named)?.reversed()),
        Curve::Nurbs(skin.isocurve_u(0.0).map_err(named)?.reversed()),
    ];
    let corners = BOX_CORNERS.map(|corner| skin.point_at(corner.x, corner.y));

    let darts: Vec<Dart> = (0..8).map(|_| edit.add_dart()).collect();
    for i in 0..4 {
        edit.link(Dim::Zero, darts[2 * i], darts[2 * i + 1])?;
    }
    for i in 0..4 {
        edit.link(Dim::One, darts[2 * i + 1], darts[(2 * i + 2) % darts.len()])?;
    }
    for i in 0..4 {
        let dart = edit.cell_representative(darts[2 * i], Dim::Zero);
        edit.add_vertex(VertexAttr::new(dart, corners[i]));
    }
    for i in 0..4 {
        edit.add_edge(EdgeAttr::new(darts[2 * i], boundary[i].clone()));
    }

    let pcurves = (0..4)
        .map(|i| {
            (
                darts[2 * i],
                TrimmedCurve2::segment(BOX_CORNERS[i], BOX_CORNERS[(i + 1) % 4]),
            )
        })
        .collect::<HashMap<_, _>>();
    edit.add_profile(ProfileAttr::new(darts[0]));
    let key = edit.add_face(FaceAttr::with_pcurves(
        Surface::Nurbs(skin),
        darts[0],
        Vec::new(),
        pcurves,
    ));

    Ok(LoftColumnFace {
        key,
        first_section: darts[0],
        // The loop runs the last section's edge from `(1, 1)` to `(0, 1)`, so
        // its *second* dart is the one sitting where the cap's edge dart sits.
        last_section: darts[5],
        rails: Some(ColumnRails {
            start: darts[7],
            end: darts[2],
        }),
        anchor: darts[0],
    })
}

/// Adds the band face a column that closes onto itself makes.
///
/// The column's two ends are the same curve, so a quad would need a rail
/// there and that rail would be a seam: an edge asserting a crease the surface
/// does not have. The band has none. It is bounded by two closed loops — one
/// per end section, each running the whole of `u` — joined by a scaffold cut
/// so the face still occupies one 2-cell. The pair is
/// [`LoopKind::Wrapping`] because neither loop closes in parameter space: each
/// is a straight run across the domain that closes only on the surface.
///
/// A boundary loop is a *marked* closed edge where its section began on a
/// corner and an *unmarked* one where it did not, so a circle handed in
/// without a vertex comes back out without one.
fn add_loft_band_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    column: usize,
    skin: NurbsSurface,
    corners: [Option<Point3>; 2],
) -> Result<LoftColumnFace, LoftError> {
    let named = |source| LoftError::Column {
        column,
        sections: skin.control_points().nv(),
        source,
    };
    // Each loop runs the way the face's boundary does, so the band lies to the
    // left of both: the first section forward in `u`, the last one back.
    let boundary = [
        Curve::Nurbs(skin.isocurve_v(0.0).map_err(named)?),
        Curve::Nurbs(skin.isocurve_v(1.0).map_err(named)?.reversed()),
    ];
    let pcurves = [
        TrimmedCurve2::segment(BOX_CORNERS[0], BOX_CORNERS[1]),
        TrimmedCurve2::segment(BOX_CORNERS[2], BOX_CORNERS[3]),
    ];

    let mut loops = [[Dart::new(0); 2]; 2];
    for (slot, darts) in loops.iter_mut().enumerate() {
        let first = edit.add_dart();
        let second = edit.add_dart();
        edit.link(Dim::Zero, first, second)?;
        edit.link(Dim::One, first, second)?;
        let edge = edit.add_edge(EdgeAttr::new(first, boundary[slot].clone()));
        match corners[slot] {
            Some(point) => {
                edit.add_vertex(VertexAttr::new(first, point));
            }
            // Nothing meets where this loop closes, so the 0-cell there is
            // interior to the edge rather than a corner of the shape.
            None => edit.own_cell(Dim::Zero, first, EntityOwner::Edge(edge)),
        }
        edit.add_profile(ProfileAttr::new(first));
        *darts = [first, second];
    }
    let seeds = [loops[0][0], loops[1][0]];

    let key = edit.add_face(FaceAttr::with_loops(
        Surface::Nurbs(skin),
        vec![
            LoopDefinition::from_kind(seeds[0], LoopKind::Wrapping { axis: Axis2::U }),
            LoopDefinition::from_kind(seeds[1], LoopKind::Wrapping { axis: Axis2::U }),
        ],
        HashMap::from([
            (seeds[0], pcurves[0].clone()),
            (seeds[1], pcurves[1].clone()),
        ]),
    ));
    cut_between_loops(edit, key, seeds[0], seeds[1])?;

    Ok(LoftColumnFace {
        key,
        first_section: seeds[0],
        // A shell is consistently wound when neighbours run their shared edge
        // opposite ways, so the cap's own dart has to meet the *other* dart of
        // this loop — as it meets the quad's second dart rather than its seed.
        last_section: loops[1][1],
        rails: None,
        anchor: seeds[0],
    })
}

/// Joins two boundary darts across the edge they share.
///
/// Total over all three shapes of edge, which is what a loft needs and the
/// other sweeps do not: a band's boundary can be an unmarked closed edge, and
/// there is no vertex at either end of one to reconcile. Each pairing that
/// *does* name two vertices is merged, and a closed edge naming the same
/// vertex twice is merged once.
fn sew_edges<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    survivor: Dart,
    removed: Dart,
) -> Result<(), LoftError> {
    let edge_at = |edit: &ModelEdit<'_, P>, dart: Dart| {
        Edge::from_dart(edit, dart)
            .map(|edge| edge.key())
            .ok_or(LoftError::MissingEdge { dart })
    };
    let survivor_edge = edge_at(edit, survivor)?;
    let removed_edge = edge_at(edit, removed)?;
    let pairs: Vec<(VertexKey, VertexKey)> = [
        (survivor, removed),
        (
            edit.alpha(Dim::Zero, survivor),
            edit.alpha(Dim::Zero, removed),
        ),
    ]
    .into_iter()
    .filter_map(|(keep, drop)| {
        Some((
            Vertex::from_dart(edit, keep)?.key(),
            Vertex::from_dart(edit, drop)?.key(),
        ))
    })
    .collect();

    // Read before the sew, while the two 0-cells are still apart.
    let closes_on_an_edge = [survivor, removed]
        .into_iter()
        .any(|dart| edge_owned_closure(edit, dart).is_some());

    edit.sew(Dim::Two, survivor, removed)
        .map_err(|_| LoftError::SewFailed {
            dim: Dim::Two,
            first: survivor,
            second: removed,
        })?;

    if survivor_edge != removed_edge {
        edit.merge_edges_into(survivor_edge, removed_edge);
        if closes_on_an_edge {
            reown_closure(edit, survivor, survivor_edge);
        }
    }
    let mut merged: HashSet<VertexKey> = HashSet::new();
    for (keep, drop) in pairs {
        if keep != drop && merged.insert(drop) {
            edit.merge_vertices_into(keep, drop);
        }
    }
    Ok(())
}

/// The edge owning the 0-cell at `dart`, where an edge owns it.
///
/// A closed edge with no corner records where it closes by owning that
/// 0-cell, which is what makes it interior to the edge rather than a vertex.
fn edge_owned_closure<P: Payload>(edit: &ModelEdit<'_, P>, dart: Dart) -> Option<EdgeKey> {
    if Vertex::from_dart(edit, dart).is_some() {
        return None;
    }
    edit.orbit(dart, edit.orbit_indices(Dim::Zero))
        .find_map(
            |anchor| match edit.embedding().owner_at(Dim::Zero, anchor) {
                Some(EntityOwner::Edge(key)) => Some(key),
                _ => None,
            },
        )
}

/// Hands the closure point of two merged closed edges to the one that lived.
///
/// Sewing two corner-free closed edges brings their two closure 0-cells onto
/// one orbit, and merging the edges leaves that orbit claimed by the key that
/// went away as well as by the one that stayed. Two claims on one cell is
/// what commit refuses, so the orbit is unlabelled and relabelled once.
fn reown_closure<P: Payload>(edit: &mut ModelEdit<'_, P>, dart: Dart, edge: EdgeKey) {
    edit.disown_cell(Dim::Zero, dart);
    edit.own_cell(Dim::Zero, dart, EntityOwner::Edge(edge));
}

/// Cuts a cap's boundary at `point`, unless a corner is already there.
fn cut_cap_at<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    cap: FaceKey,
    index: usize,
    point: Point3,
) -> Result<(), LoftError> {
    // Read the cut out of the model before touching it: the view that finds
    // the edge borrows what the split mutates.
    let target: Option<(EdgeKey, Fraction)> = {
        let model = edit.model();
        let face = Face::new(model, cap);
        if face
            .vertices()
            .iter()
            .any(|vertex| vertex.point().coincides(point, LINEAR_TOLERANCE))
        {
            return Ok(());
        }
        face.edges().into_iter().find_map(|edge| {
            let section = model.edge_unchecked(edge.key()).trimmed_curve();
            section
                .contains(point, LINEAR_TOLERANCE)
                .then(|| (edge.key(), section.parameter_at(point)))
        })
    };
    let Some((edge, parameter)) = target else {
        return Err(LoftError::CapNotCuttable { index, point });
    };
    split_face_edge_staged(edit, cap, edge, parameter)
        .map(|_| ())
        .map_err(|_| LoftError::CapNotCuttable { index, point })
}

/// The cap's boundary dart running from `start` to `end`.
///
/// The cap's loop may run either way round relative to the columns — a section
/// handed to the loft backwards is turned round by direction agreement, and
/// the face it came from is not — so the edge is found by its two ends and
/// then read from the one the column starts at.
fn cap_dart_between<P: Payload>(
    model: &Model<P>,
    cap: FaceKey,
    start: Point3,
    end: Point3,
) -> Option<Dart> {
    Face::new(model, cap).loops().into_iter().find_map(|loop_| {
        loop_.darts().find_map(|dart| {
            let here = model.point_at_dart(dart)?;
            let partner = model.alpha(Dim::Zero, dart);
            let there = model.point_at_dart(partner)?;
            if here.coincides(start, LINEAR_TOLERANCE) && there.coincides(end, LINEAR_TOLERANCE) {
                return Some(dart);
            }
            (here.coincides(end, LINEAR_TOLERANCE) && there.coincides(start, LINEAR_TOLERANCE))
                .then_some(partner)
        })
    })
}

/// How much a cap's normal runs with the loft.
fn cap_normal_along_run<P: Payload>(model: &Model<P>, cap: FaceKey, advance: Vector3<f64>) -> f64 {
    Face::new(model, cap).normal_at(0.0, 0.0).dot(&advance)
}
