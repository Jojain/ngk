//! The Boolean domain for faces on one shared support surface.
//!
//! Both domains read the same field, `BooleanLineage::faces`, and both get
//! `FaceKey`s out of it. What differs is what those faces *are*.
//!
//! `operand_cells` returns a solid's boundary faces, so for a solid operand the
//! fragments are pieces of the solid's boundary. It returns a face operand as
//! itself, so here the fragments are pieces of **that face** — pieces of its
//! area, which splitting cut it into where another operand's boundary crossed
//! it.
//!
//! Two overlapping squares on one plane make this concrete. With A the square
//! [0,2]x[0,2] and B the square [1,3]x[1,3], splitting replaces each of them
//! with two faces: A becomes [1,2]x[1,2] plus an L-shape, and B becomes its own
//! [1,2]x[1,2] plus the other L-shape. Both copies of the overlap are present,
//! under different `FaceKey`s, which is why [`PlanarDomain::membership`] calls
//! an overlap a coincidence, and selection keeps at most one of them.
//!
//! That is also why the solid reading cannot be reused. It takes a fragment
//! inside another operand to have that operand on both of its sides, because a
//! piece of boundary inside a solid lies in its interior. Applied here it would
//! drop A's copy of the overlap, and the union of two faces would come back
//! missing the area the two share.
//!
//! Nothing here is specific to a plane. Containment is decided in the support
//! surface's own parameter space, so two faces on one cylinder or one sphere
//! are as good a pair as two faces on one plane; what admission requires is
//! that both operands sit on the *same* support surface, not that it is flat.

use nalgebra::Vector3;

use crate::geometry::{Point2, Point3, Surface};
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;

use super::domain::{BooleanDomain, Membership, OperandClassifier, RelativeLocation};
use super::solid_domain::SolidDomain;
use super::trim::FaceTrimDomain;
use super::{BooleanCell, BooleanError, BooleanLineage, BooleanOptions, BooleanTolerances};

/// Marker for the face-pair domain; carries no state of its own.
pub(crate) struct PlanarDomain;

/// Answers whether a point lies inside one face operand.
///
/// The operand is no longer one face by the time this is built: splitting has
/// replaced it with the faces listed in its `BooleanLineage::faces`. Together
/// those cover the same area the original face did, so the question is whether
/// the point falls in any one of them. The edges where those faces meet each
/// other are interior to the operand, and nothing here treats them as boundary.
pub(crate) struct PlanarOperandClassifier {
    /// The support surface all of this operand's faces share.
    surface: Surface,
    /// The trim of each face the operand's split left behind.
    trims: Vec<FaceTrimDomain>,
}

impl PlanarOperandClassifier {
    /// Whether `uv` lies strictly inside any of this operand's faces.
    fn contains(&self, uv: Point2) -> bool {
        self.trims.iter().any(|trim| trim.contains(uv))
    }
}

impl OperandClassifier<FaceKey> for PlanarOperandClassifier {
    /// Locates one face fragment's probe point against this operand.
    ///
    /// `probe` picks a point interior to the fragment, chosen for clearance
    /// from every edge of it, so the point never lands on this operand's
    /// boundary. The answer is therefore always `Inside` or `Outside`, which is
    /// why [`PlanarDomain::membership`] has no `OnBoundary` reading of its own.
    ///
    /// `normal` goes unused: this is a closed-form winding query, with no ray
    /// to cast and no second orientation to compare against.
    fn locate(
        &self,
        point: Point3,
        _normal: Vector3<f64>,
        source: FaceKey,
    ) -> Result<RelativeLocation, BooleanError> {
        let uv = self
            .surface
            .param_at(point)
            .map_err(|_| BooleanError::MissingGeometry {
                cell: BooleanCell::Face(source),
            })?;
        Ok(match self.contains(uv) {
            true => RelativeLocation::Inside,
            false => RelativeLocation::Outside,
        })
    }
}

impl BooleanDomain for PlanarDomain {
    type Fragment = FaceKey;
    type Classifier<'a, P: Payload + 'a> = PlanarOperandClassifier;

    // Fragments are faces, and faces are bounded by edges, here exactly as for
    // a solid — so these read the same cells out of the same places. Only
    // `classifier` and `membership` differ, and they differ because the faces
    // mean something else.

    fn fragments(lineage: &BooleanLineage) -> Vec<(FaceKey, FaceKey)> {
        SolidDomain::fragments(lineage)
    }

    fn probe<P: Payload>(
        map: &Model<P>,
        fragment: FaceKey,
        tolerances: BooleanTolerances,
    ) -> Result<(Point3, Vector3<f64>), BooleanError> {
        SolidDomain::probe(map, fragment, tolerances)
    }

    fn classifier<P: Payload>(
        map: &Model<P>,
        lineage: &BooleanLineage,
        _options: BooleanOptions,
        tolerances: BooleanTolerances,
    ) -> Result<PlanarOperandClassifier, BooleanError> {
        let mut trims = Vec::new();
        let mut surface = None;
        for &face in lineage.faces.values().flatten() {
            let view = map.face(face).ok_or(BooleanError::MissingGeometry {
                cell: BooleanCell::Face(face),
            })?;
            surface.get_or_insert_with(|| view.surface().clone());
            trims.push(FaceTrimDomain::new(&view, tolerances.parameter)?);
        }
        Ok(PlanarOperandClassifier {
            surface: surface.ok_or(BooleanError::EmptyResult)?,
            trims,
        })
    }

    /// A face fragment is a piece of area, not of boundary, so it is read as
    /// a sheet with nothing in front of it and its area behind it.
    ///
    /// The other operand then either covers that area or does not. Where it
    /// does, the area is present twice — once among each operand's faces — so
    /// the location is also a coincidence, and selection keeps one copy. In
    /// front of the sheet nothing belongs to any operand, so no operation is
    /// true there, and nothing is ever reversed: a face already bounds its own
    /// area the right way round, unlike a boundary face borrowed from another
    /// solid.
    fn membership(location: RelativeLocation) -> Membership {
        match location {
            RelativeLocation::Inside => Membership {
                front: false,
                behind: true,
                coincident: true,
            },
            RelativeLocation::Outside => Membership {
                front: false,
                behind: false,
                coincident: false,
            },
            RelativeLocation::OnBoundarySame | RelativeLocation::OnBoundaryOpposite => {
                unreachable!("a face operand's classifier answers only inside or outside")
            }
        }
    }
}
