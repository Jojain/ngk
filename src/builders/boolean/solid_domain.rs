//! The three-dimensional Boolean domain: solids bounded by faces.
//!
//! A solid's fragments are its faces, two fragments meet across an edge, and
//! the section a Boolean computes is realized as edges on both boundaries.
//! Locating a point against the whole operand is a ray cast, which is the one
//! expensive answer in the family — see [`super::classify::SolidRayCaster`].

use nalgebra::Vector3;

use crate::geometry::Point3;
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;

use super::classify::{SolidRayCaster, probe};
use super::domain::{BooleanDomain, Membership, RelativeLocation};
use super::{BooleanError, BooleanLineage, BooleanOptions, BooleanTolerances};

/// Marker for the solid domain; carries no state of its own.
pub(crate) struct SolidDomain;

impl BooleanDomain for SolidDomain {
    type Fragment = FaceKey;
    type Classifier<'a, P: Payload + 'a> = SolidRayCaster<'a, P>;

    fn fragments(lineage: &BooleanLineage) -> Vec<(FaceKey, FaceKey)> {
        lineage
            .faces
            .iter()
            .flat_map(|(&source, faces)| faces.iter().map(move |&face| (source, face)))
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

    /// A piece of boundary has the other solid on both of its sides, on
    /// neither, or -- where the two boundaries coincide -- on exactly one,
    /// which their relative facing names.
    fn membership(location: RelativeLocation) -> Membership {
        let (front, behind, coincident) = match location {
            RelativeLocation::Inside => (true, true, false),
            RelativeLocation::Outside => (false, false, false),
            RelativeLocation::OnBoundarySame => (false, true, true),
            RelativeLocation::OnBoundaryOpposite => (true, false, true),
        };
        Membership {
            front,
            behind,
            coincident,
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
