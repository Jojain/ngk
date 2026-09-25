//! The local surgery one vertex treatment writes.

use nalgebra::Vector3;

use super::super::errors::BlendError;
use super::super::network::{BlendNetwork, NetworkVertex, RingSlot};
use super::super::section::{EdgeSection, loop_darts};
use crate::geometry::{LINEAR_TOLERANCE, Plane, Point3, Surface, TrimmedCurve};
use crate::model::Model;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// One vertex's treatment, written in the surgery's vocabulary with ids local
/// to the vertex.
///
/// A treatment says where the vertex's pieces go (`corners`), which curves
/// join them (`joints`), how each blend ending here is closed between its two
/// rails (`ends`), and which faces it adds (`patches`). Anything a treatment
/// can express this way the executor can build, which is what lets a new
/// treatment be added without touching it.
#[derive(Debug, Default)]
pub(crate) struct VertexBlend {
    pub(crate) corners: Vec<Point3>,
    pub(crate) joints: Vec<LocalJoint>,
    pub(crate) ends: Vec<BlendEnd>,
    pub(crate) patches: Vec<Patch>,
    /// Unselected edges this treatment cuts back, and the point each now ends at.
    pub(crate) landings: Vec<(EdgeKey, Point3)>,
}

/// A curve between two of a treatment's corners.
#[derive(Debug)]
pub(crate) struct LocalJoint {
    pub(crate) corners: [usize; 2],
    pub(crate) curve: TrimmedCurve,
    /// The face corner the joint is inserted into: the face, and the incoming
    /// edge's dart the joint follows.
    pub(crate) insertion: Option<(FaceKey, Dart)>,
}

/// How one blend ending at this vertex is closed.
#[derive(Debug)]
pub(crate) struct BlendEnd {
    /// The network edge, and which of its ends this vertex is.
    pub(crate) edge: usize,
    pub(crate) end: usize,
    /// The corner each network side's rail stops at.
    pub(crate) rails: [usize; 2],
    /// The joints from side 0's corner to side 1's, each with whether the
    /// walk runs against the joint's own direction.
    pub(crate) chain: Vec<(usize, bool)>,
}

/// A face a treatment adds, bounded by its joints.
#[derive(Debug)]
pub(crate) struct Patch {
    pub(crate) surface: Surface,
    /// The joints walked once round, each with whether the walk runs against it.
    pub(crate) boundary: Vec<(usize, bool)>,
}

impl VertexBlend {
    pub(crate) fn corner(&mut self, point: Point3) -> usize {
        self.corners.push(point);
        self.corners.len() - 1
    }

    pub(crate) fn joint(&mut self, from: usize, to: usize, curve: TrimmedCurve) -> usize {
        self.joints.push(LocalJoint {
            corners: [from, to],
            curve,
            insertion: None,
        });
        self.joints.len() - 1
    }

    /// Inserts a joint into `slot`'s face corner, between the corner on the
    /// slot's edge and the corner on the next slot's edge.
    ///
    /// The joint runs the way the face walks its loop, so `curve` is asked
    /// for the direction it will be walked in.
    pub(crate) fn insert<P: Payload>(
        &mut self,
        model: &Model<P>,
        slot: &RingSlot,
        corners: [usize; 2],
        curve: impl Fn(Point3, Point3) -> Result<TrimmedCurve, BlendError>,
    ) -> Result<usize, BlendError> {
        let walked = loop_darts(model, slot.face);
        let (after, from, to) = if walked.contains(&model.alpha(Dim::Zero, slot.darts[0])) {
            (slot.darts[0], corners[0], corners[1])
        } else if walked.contains(&model.alpha(Dim::Zero, slot.darts[1])) {
            (slot.darts[1], corners[1], corners[0])
        } else {
            return Err(BlendError::InconsistentSurgery {
                reason: "a face's corner is not on its loop",
            });
        };
        let curve = curve(self.corners[from], self.corners[to])?;
        self.joints.push(LocalJoint {
            corners: [from, to],
            curve,
            insertion: Some((slot.face, after)),
        });
        Ok(self.joints.len() - 1)
    }

    /// Closes the blend of network edge `edge` at this vertex, walking
    /// `joints` from side 0's corner to side 1's.
    pub(crate) fn end(
        &mut self,
        edge: usize,
        end: usize,
        rails: [usize; 2],
        joints: &[usize],
    ) -> Result<(), BlendError> {
        let chain = self.path(rails[0], rails[1], joints)?;
        self.ends.push(BlendEnd {
            edge,
            end,
            rails,
            chain,
        });
        Ok(())
    }

    /// Adds a patch bounded by `joints`, walked round from `start`.
    pub(crate) fn patch(
        &mut self,
        surface: Surface,
        start: usize,
        joints: &[usize],
    ) -> Result<(), BlendError> {
        let boundary = self.path(start, start, joints)?;
        self.patches.push(Patch { surface, boundary });
        Ok(())
    }

    /// Orders `joints` into a walk from corner `from` to corner `to`.
    fn path(
        &self,
        from: usize,
        to: usize,
        joints: &[usize],
    ) -> Result<Vec<(usize, bool)>, BlendError> {
        let mut remaining = joints.to_vec();
        let mut walk = Vec::with_capacity(joints.len());
        let mut at = from;
        while !remaining.is_empty() {
            let Some(position) = remaining
                .iter()
                .position(|&joint| self.joints[joint].corners.contains(&at))
            else {
                return Err(BlendError::InconsistentSurgery {
                    reason: "a treatment's joints do not chain",
                });
            };
            let joint = remaining.swap_remove(position);
            let [first, second] = self.joints[joint].corners;
            let reversed = first != at;
            at = if reversed { first } else { second };
            walk.push((joint, reversed));
        }
        if at != to {
            return Err(BlendError::InconsistentSurgery {
                reason: "a treatment's joints do not reach the rail they close",
            });
        }
        Ok(walk)
    }
}

/// What every treatment reads about the vertex it treats.
pub(crate) struct VertexContext<'a, P: Payload> {
    pub(crate) model: &'a Model<P>,
    pub(crate) network: &'a BlendNetwork,
    pub(crate) sections: &'a [EdgeSection],
    /// The vertex's index in the network.
    pub(crate) index: usize,
    pub(crate) vertex: &'a NetworkVertex,
}

impl<P: Payload> VertexContext<'_, P> {
    pub(crate) fn unsupported(&self, reason: &'static str) -> BlendError {
        BlendError::UnsupportedVertex {
            vertex: self.vertex.key,
            reason,
        }
    }

    pub(crate) fn does_not_fit(&self, reason: &'static str) -> BlendError {
        BlendError::VertexDoesNotFit {
            vertex: self.vertex.key,
            reason,
        }
    }

    /// The selected edge of a slot: its network index and section.
    pub(crate) fn section(&self, slot: &RingSlot) -> Result<(usize, &EdgeSection), BlendError> {
        let edge = slot.selected.ok_or(BlendError::InconsistentSurgery {
            reason: "a treatment read a section off an unselected edge",
        })?;
        Ok((edge, &self.sections[edge]))
    }

    /// Which side of network edge `edge` lies on `face`.
    pub(crate) fn side_on(&self, edge: usize, face: FaceKey) -> Result<usize, BlendError> {
        self.network.edges[edge]
            .side_on(face)
            .ok_or(BlendError::InconsistentSurgery {
                reason: "a blend's face is missing from the ring around its end",
            })
    }

    /// Which end of network edge `edge` this vertex is.
    pub(crate) fn end_of(&self, edge: usize) -> usize {
        self.network.edges[edge].end_at(self.index)
    }

    /// The unit direction network edge `edge` leaves this vertex in.
    pub(crate) fn leaving(&self, edge: usize) -> Vector3<f64> {
        let span = &self.network.edges[edge].span;
        let along = (span.end() - span.start()).normalize();
        if self.end_of(edge) == 0 {
            along
        } else {
            -along
        }
    }

    /// The support plane of a face, or a refusal when it is not planar.
    pub(crate) fn plane(&self, face: FaceKey) -> Result<Plane, BlendError> {
        match &self.model.face_attr_unchecked(face).surface {
            Surface::Plane(plane) => Ok(plane.clone()),
            _ => Err(self.unsupported("a face around it is not planar")),
        }
    }

    /// A face's outward normal, read in its stored orientation.
    pub(crate) fn normal(&self, face: FaceKey) -> Vector3<f64> {
        *self.model.face_unchecked(face).normal_at(0.0, 0.0)
    }

    /// An unselected straight edge at this vertex: the direction it leaves
    /// the vertex in and its length.
    pub(crate) fn straight(&self, edge: EdgeKey) -> Result<(Vector3<f64>, f64), BlendError> {
        let span = self.model.edge_unchecked(edge).trimmed_curve();
        if !matches!(span.curve(), crate::geometry::Curve::Line(_)) {
            return Err(self.unsupported("an edge around it is not straight"));
        }
        let (start, end) = (span.start(), span.end());
        let (near, far) = if (start - self.vertex.point).norm() <= (end - self.vertex.point).norm()
        {
            (start, end)
        } else {
            (end, start)
        };
        let direction = far - near;
        Ok((direction.normalize(), direction.norm()))
    }

    /// Where the rail of network edge `edge`'s `side` meets the unselected
    /// straight edge `target`, refused unless it falls strictly inside it.
    pub(crate) fn land(
        &self,
        edge: usize,
        side: usize,
        target: EdgeKey,
    ) -> Result<Point3, BlendError> {
        let (direction, length) = self.straight(target)?;
        let point = self.sections[edge]
            .land(side, self.vertex.point, self.vertex.point, direction)
            .ok_or_else(|| self.unsupported("a rail does not meet the edge beside it"))?;
        self.inside(point, direction, length)?;
        Ok(point)
    }

    /// Refuses a point that is not strictly inside the edge leaving the vertex
    /// along `direction` for `length`.
    pub(crate) fn inside(
        &self,
        point: Point3,
        direction: Vector3<f64>,
        length: f64,
    ) -> Result<(), BlendError> {
        let along = (point - self.vertex.point).dot(&direction);
        let slack = LINEAR_TOLERANCE.sqrt();
        if along <= slack || along >= length - slack {
            return Err(self.does_not_fit("a rail runs off the edge beside it"));
        }
        Ok(())
    }
}
