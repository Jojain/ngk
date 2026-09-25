//! The Boolean domain for two faces on one shared support surface.
//!
//! Both domains read the same field, `BooleanLineage::faces`, and both get
//! `FaceKey`s out of it. What differs is what those faces *are*.
//!
//! `operand_cells` returns a solid's boundary faces, so for a solid operand the
//! fragments are pieces of the solid's boundary. It returns a face operand as
//! itself, so here the fragments are pieces of **that face** — pieces of its
//! area, which splitting cut it into where the other operand's boundary crossed
//! it.
//!
//! Two overlapping squares on one plane make this concrete. With A the square
//! [0,2]x[0,2] and B the square [1,3]x[1,3], splitting replaces each of them
//! with two faces: A becomes [1,2]x[1,2] plus an L-shape, and B becomes its own
//! [1,2]x[1,2] plus the other L-shape. Both copies of the overlap are present,
//! under different `FaceKey`s, which is why every rule in
//! [`PlanarDomain::keeps`] keeps at most one of them.
//!
//! That is also why the solid table cannot be reused. It keeps boundary pieces
//! lying outside the other operand and drops the ones inside, because an inside
//! piece would fall in the interior of the answer rather than on its boundary.
//! Applied here it would drop A's copy of the overlap, and the union of two
//! faces would come back missing the area the two share.
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

use super::domain::{BooleanDomain, OperandClassifier, RelativeLocation, Selection};
use super::solid_domain::SolidDomain;
use super::trim::FaceTrimDomain;
use super::{
    BooleanCell, BooleanError, BooleanLineage, BooleanOperation, BooleanOptions, BooleanSide,
    BooleanTolerances,
};

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
    /// why [`PlanarDomain::keeps`] has no `OnBoundary` rows to write.
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
    // a solid — so these four read the same cells out of the same places. Only
    // `classifier` and `keeps` differ, and they differ because the faces mean
    // something else.

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

    /// Which faces of which operand the answer is made of.
    ///
    /// Where the two operands overlap, that area is present twice — once among
    /// the first operand's faces and once among the second's — so every rule
    /// keeps at most one of the two. The first operand is the one kept, which
    /// is the convention the solid table already follows for coincident
    /// boundary.
    ///
    /// - **Union**: all of the first operand, plus the second's faces that lie
    ///   outside the first.
    /// - **Intersection**: the first operand's faces that lie inside the
    ///   second. The second's copies of that same area are dropped.
    /// - **Difference**: the first operand's faces that lie outside the second,
    ///   and none of the second's.
    ///
    /// Nothing is reversed. A face already bounds its own area the right way
    /// round, unlike a boundary face borrowed from the other solid.
    fn keeps(
        operation: BooleanOperation,
        side: BooleanSide,
        location: RelativeLocation,
    ) -> Selection {
        use BooleanOperation::{Difference, Intersection, Union};
        use BooleanSide::{First, Second};
        use RelativeLocation::{Inside, Outside};
        let keep = matches!(
            (operation, side, location),
            (Union, First, _)
                | (Union, Second, Outside)
                | (Intersection, First, Inside)
                | (Difference, First, Outside)
        );
        match keep {
            true => Selection::Keep,
            false => Selection::Drop,
        }
    }
}
