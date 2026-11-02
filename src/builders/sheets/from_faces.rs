//! Joining loose faces into one sheet.

use std::collections::{BTreeSet, HashSet};

use crate::builders::errors::SheetFromFacesError;
use crate::builders::faces::{FaceSewError, SewMerges, Traversal, sew_faces_along};
use crate::geometry::parameter::Fraction;
use crate::geometry::{LINEAR_TOLERANCE, Point3, PointCoincidence, TrimmedCurve};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::SheetAttr;
use crate::topology::edit::EditKey;
use crate::topology::gmap::Dim;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SheetKey};

/// Joins `faces` into one sheet, regardless of their supplied order.
///
/// Two faces are joined along an edge where each runs a boundary edge covering
/// the same span within [`LINEAR_TOLERANCE`]: the same ends, and each edge's
/// midpoint on the other. Edges that only partly overlap are not joined. The
/// faces must be loose -- no edge of theirs joined to another face yet -- and
/// must end up as one connected sheet, wound consistently, with no edge run by
/// more than two of them. The sheet is oriented like the first face.
///
/// A single face is a sheet of one face. Failure leaves the model unchanged.
pub fn add_sheet_from_faces<P: Payload>(
    g: &mut Model<P>,
    faces: &[FaceKey],
) -> Result<SheetKey, SheetFromFacesError> {
    g.transaction(|edit| add_sheet_from_faces_edit(edit, faces))
}

/// One face's run along one of its boundary edges.
struct BoundaryUse {
    face: FaceKey,
    edge: EdgeKey,
    curve: TrimmedCurve,
}

impl BoundaryUse {
    fn start(&self) -> Point3 {
        self.curve.point_at(Fraction::new(0.0))
    }

    fn end(&self) -> Point3 {
        self.curve.point_at(Fraction::new(1.0))
    }

    fn middle(&self) -> Point3 {
        self.curve.point_at(Fraction::new(0.5))
    }

    fn passes_through(&self, point: Point3) -> bool {
        self.curve
            .point_at(self.curve.parameter_at(point))
            .coincides(point, LINEAR_TOLERANCE)
    }

    /// Whether both runs cover the same span, in either direction.
    fn covers_same_span(&self, other: &Self) -> bool {
        let (a0, a1, b0, b1) = (self.start(), self.end(), other.start(), other.end());
        let ends = (a0.coincides(b0, LINEAR_TOLERANCE) && a1.coincides(b1, LINEAR_TOLERANCE))
            || (a0.coincides(b1, LINEAR_TOLERANCE) && a1.coincides(b0, LINEAR_TOLERANCE));
        ends && other.passes_through(self.middle()) && self.passes_through(other.middle())
    }
}

/// Joins loose faces into one sheet inside the caller's transaction.
pub(crate) fn add_sheet_from_faces_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    faces: &[FaceKey],
) -> Result<SheetKey, SheetFromFacesError> {
    let faces = faces
        .iter()
        .copied()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    let &first = faces.first().ok_or(SheetFromFacesError::NoFaces)?;
    let mut uses = Vec::new();
    for &face in &faces {
        let view = edit
            .face(face)
            .ok_or(SheetFromFacesError::MissingFace { face })?;
        for edge in view
            .loops()
            .into_iter()
            .flat_map(|boundary| boundary.edges())
        {
            if !edit.is_free(edge.dart(), Dim::Two) {
                return Err(SheetFromFacesError::FaceNotLoose { face });
            }
            uses.push(BoundaryUse {
                face,
                edge: edge.key(),
                curve: edge.trimmed_curve(),
            });
        }
    }

    let mut pairs = Vec::new();
    for (index, run) in uses.iter().enumerate() {
        let mut partners = uses
            .iter()
            .enumerate()
            .filter(|&(other, candidate)| other != index && run.covers_same_span(candidate));
        let Some((partner, _)) = partners.next() else {
            continue;
        };
        if partners.next().is_some() {
            return Err(SheetFromFacesError::NonManifoldEdge {
                point: run.middle(),
            });
        }
        if index < partner {
            pairs.push((index, partner));
        }
    }

    let mut merges = SewMerges::default();
    for (a, b) in pairs {
        let (a, b) = (&uses[a], &uses[b]);
        let traversal = sew_faces_along(
            edit,
            (a.edge, a.face),
            (b.edge, b.face),
            LINEAR_TOLERANCE,
            &mut merges,
        )
        .map_err(|error| match error {
            FaceSewError::Mismatch => SheetFromFacesError::JoinFailed {
                first: a.face,
                second: b.face,
            },
            FaceSewError::EdgeSplit(error) => error.into(),
            FaceSewError::ModelEdit(error) => error.into(),
        })?;
        if traversal == Traversal::Aligned {
            return Err(SheetFromFacesError::InconsistentOrientation {
                first: a.face,
                second: b.face,
            });
        }
    }

    let components = connected_components(edit, &faces);
    if components > 1 {
        return Err(SheetFromFacesError::Disconnected { components });
    }
    // A face copied out of a one-face sheet brings that registration along;
    // the sheet it now belongs to replaces it.
    let registered = faces
        .iter()
        .filter_map(|&face| edit.sheet_key(edit.face_unchecked(face).dart()))
        .collect::<HashSet<_>>();
    for sheet in registered {
        edit.remove_sheet(sheet);
    }
    let root = edit.face_unchecked(first).dart();
    Ok(edit.add_sheet_derived_from(
        faces.iter().map(|&face| EditKey::Face(face)).collect(),
        SheetAttr::new(root),
    ))
}

/// How many groups `faces` fall into when joined across their shared edges.
fn connected_components<P: Payload>(model: &Model<P>, faces: &[FaceKey]) -> usize {
    let mut remaining = faces.iter().copied().collect::<BTreeSet<_>>();
    let mut components = 0;
    while let Some(&seed) = remaining.first() {
        components += 1;
        let mut pending = vec![seed];
        while let Some(face) = pending.pop() {
            if !remaining.remove(&face) {
                continue;
            }
            for edge in model.face_unchecked(face).edges() {
                pending.extend(edge.faces().into_iter().map(|face| face.key()));
            }
        }
    }
    components
}
