//! The three-dimensional Boolean domain: solids bounded by faces.
//!
//! A solid's fragments are its faces, two fragments meet across an edge, and
//! the section a Boolean computes is realized as edges on both boundaries.
//! Locating a point against the whole operand is a ray cast, which is the one
//! expensive answer in the family — see [`super::classify::SolidRayCaster`].

use std::collections::HashSet;

use nalgebra::Vector3;

use crate::geometry::Point3;
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

use super::classify::{SolidRayCaster, probe};
use super::domain::{BooleanDomain, RelativeLocation, Selection};
use super::{
    BooleanError, BooleanLineage, BooleanOperandPreparation, BooleanOperation, BooleanOptions,
    BooleanSide, BooleanTolerances,
};

/// Marker for the solid domain; carries no state of its own.
pub(crate) struct SolidDomain;

impl BooleanDomain for SolidDomain {
    type Fragment = FaceKey;
    type Boundary = EdgeKey;
    type Classifier<'a, P: Payload + 'a> = SolidRayCaster<'a, P>;

    fn fragments(lineage: &BooleanLineage) -> Vec<(FaceKey, FaceKey)> {
        lineage
            .faces
            .iter()
            .flat_map(|(&source, faces)| faces.iter().map(move |&face| (source, face)))
            .collect()
    }

    fn boundaries<P: Payload>(map: &Model<P>, fragment: FaceKey) -> Vec<EdgeKey> {
        map.face_unchecked(fragment)
            .edges()
            .into_iter()
            .map(|edge| edge.key())
            .collect()
    }

    fn incident<P: Payload>(map: &Model<P>, boundary: EdgeKey) -> Vec<FaceKey> {
        map.edge(boundary)
            .map(|edge| edge.faces().into_iter().map(|face| face.key()).collect())
            .unwrap_or_default()
    }

    /// Every section edge, on both sides, bars the walk.
    ///
    /// A span's two sides are still separate edges at this point — sewing is
    /// what later fuses them — so both are listed, and a fragment is grouped
    /// only with fragments the section did not cut it away from.
    fn barriers<P: Payload>(
        _map: &Model<P>,
        preparation: &BooleanOperandPreparation,
    ) -> HashSet<EdgeKey> {
        preparation
            .span_edges
            .values()
            .flatten()
            .flatten()
            .copied()
            .collect()
    }

    fn probe<P: Payload>(
        map: &Model<P>,
        fragment: FaceKey,
        tolerances: BooleanTolerances,
    ) -> Result<(Point3, Vector3<f64>), BooleanError> {
        let (point, uv) = probe(map, fragment, tolerances)?;
        let normal = *map.face_unchecked(fragment).normal_at(uv.x, uv.y);
        Ok((point, normal))
    }

    /// The regularized boundary table, retaining A's copy of same-oriented
    /// coincidence.
    ///
    /// Every rule is about which side of the *other* solid a piece of boundary
    /// falls on, because the result's boundary is assembled from exactly those
    /// pieces. A difference borrows the second operand's boundary and must
    /// turn it around, which is the one reversal in the family.
    fn keeps(
        operation: BooleanOperation,
        side: BooleanSide,
        location: RelativeLocation,
    ) -> Selection {
        use BooleanOperation::{Difference, Intersection, Union};
        use BooleanSide::{First, Second};
        use RelativeLocation::{Inside, OnBoundaryOpposite, OnBoundarySame, Outside};
        let keep = matches!(
            (operation, side, location),
            (Union, _, Outside)
                | (Union, First, OnBoundarySame)
                | (Intersection, _, Inside)
                | (Intersection, First, OnBoundarySame)
                | (Difference, First, Outside | OnBoundaryOpposite)
                | (Difference, Second, Inside)
        );
        match (keep, operation == Difference && side == Second) {
            (true, true) => Selection::KeepReversed,
            (true, false) => Selection::Keep,
            (false, _) => Selection::Drop,
        }
    }

    fn classifier<'a, P: Payload>(
        map: &'a Model<P>,
        lineage: &BooleanLineage,
        options: BooleanOptions,
        tolerances: BooleanTolerances,
    ) -> Result<SolidRayCaster<'a, P>, BooleanError> {
        SolidRayCaster::new(
            map,
            lineage.faces.values().flatten().copied(),
            options,
            tolerances,
        )
    }
}
