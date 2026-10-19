use std::cmp::Ordering;

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use super::frame::Frame;
use super::utils::Point3;
use crate::geometry::transform::Rigid;

/// One end of a box's range along a frame axis, for picking a [`BBox::corner`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bound {
    Min,
    Max,
}

/// An oriented 3D bounding box: a frame and a range along each of its axes.
///
/// The frame is kept as given, origin included; the box is the set of points
/// whose coordinates in that frame lie between `min` and `max`. A box built in
/// [`Frame::xyz`] is therefore an axis-aligned box whose corners are the
/// world-space minimum and maximum, read without any round trip through a
/// centre and a size. A box always contains something: constructors over no
/// points return `None`.
#[derive(Debug, Clone, PartialEq)]
pub struct BBox {
    frame: Frame,
    min: Vector3<f64>,
    max: Vector3<f64>,
}

impl BBox {
    /// A box in `frame` spanning `min` to `max`, both in frame coordinates.
    ///
    /// # Panics
    ///
    /// When `min` exceeds `max` along any axis, or either is not finite.
    pub fn new(frame: Frame, min: Vector3<f64>, max: Vector3<f64>) -> Self {
        assert!(
            (0..3).all(|axis| min[axis].is_finite()
                && max[axis].is_finite()
                && min[axis] <= max[axis]),
            "a box's ranges must be finite and ordered, got {min:?} to {max:?}"
        );
        Self { frame, min, max }
    }

    /// Builds an oriented bounding box from a point cloud, or `None` when there
    /// are no points.
    ///
    /// The frame's axes are the principal axes of the point covariance, longest
    /// spread first, and its origin is the centroid. A single point gets the
    /// world axes at that point.
    pub fn from_points(points: impl IntoIterator<Item = Point3>) -> Option<Self> {
        let points = points.into_iter().collect::<Vec<_>>();
        let frame = match points.as_slice() {
            [] => return None,
            [point] => Frame::from_xy(*point, Vector3::x(), Vector3::y()),
            _ => Self::principal_frame(&points),
        };
        Self::from_points_in_frame(frame, points)
    }

    /// Builds the box of `points` in `frame`, or `None` when there are no points.
    ///
    /// Analytic supports use their natural frame so extrema in parameter space
    /// produce an exact oriented box rather than a sampled PCA approximation.
    pub fn from_points_in_frame(
        frame: Frame,
        points: impl IntoIterator<Item = Point3>,
    ) -> Option<Self> {
        let mut points = points.into_iter();
        let first = frame.coordinates_of(points.next()?);
        let mut bbox = Self {
            frame,
            min: first,
            max: first,
        };
        for point in points {
            bbox.extend(point);
        }
        Some(bbox)
    }

    /// Returns this box under a rigid motion.
    ///
    /// Exact: the frame moves and the ranges along its axes do not change.
    pub fn moved(&self, r: &Rigid) -> Self {
        Self {
            frame: self.frame.moved(r),
            min: self.min,
            max: self.max,
        }
    }

    /// Returns the frame the box's ranges are measured in.
    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    /// Returns the corner at the lower end of every range.
    ///
    /// In [`Frame::xyz`] this is the component-wise world minimum; in any other
    /// frame it is the corner at the frame-local minimum.
    pub fn min(&self) -> Point3 {
        self.frame.point_at(self.min)
    }

    /// Returns the corner at the upper end of every range.
    ///
    /// In [`Frame::xyz`] this is the component-wise world maximum; in any other
    /// frame it is the corner at the frame-local maximum.
    pub fn max(&self) -> Point3 {
        self.frame.point_at(self.max)
    }

    /// Returns the box size along each frame axis.
    pub fn size(&self) -> Vector3<f64> {
        self.max - self.min
    }

    /// Returns the box size along the frame's x axis.
    pub fn x_size(&self) -> f64 {
        self.max.x - self.min.x
    }

    /// Returns the box size along the frame's y axis.
    pub fn y_size(&self) -> f64 {
        self.max.y - self.min.y
    }

    /// Returns the box size along the frame's z axis.
    pub fn z_size(&self) -> f64 {
        self.max.z - self.min.z
    }

    /// Returns the centre of the box.
    pub fn center(&self) -> Point3 {
        self.frame.point_at((self.min + self.max) / 2.0)
    }

    /// Returns the length of the box diagonal.
    pub fn diagonal_length(&self) -> f64 {
        self.size().norm()
    }

    /// Returns a box grown by `tolerance` at both ends of every range.
    pub fn expanded(&self, tolerance: f64) -> Self {
        let delta = Vector3::repeat(tolerance.max(0.0));
        Self {
            frame: self.frame.clone(),
            min: self.min - delta,
            max: self.max + delta,
        }
    }

    /// Returns true when `point` lies inside this box within `tolerance`.
    pub fn contains_point(&self, point: Point3, tolerance: f64) -> bool {
        let tolerance = tolerance.max(0.0);
        let local = self.frame.coordinates_of(point);
        (0..3).all(|axis| {
            self.min[axis] - tolerance <= local[axis] && local[axis] <= self.max[axis] + tolerance
        })
    }

    /// Returns true when two oriented boxes overlap within `tolerance`.
    pub fn intersects(&self, other: &Self, tolerance: f64) -> bool {
        let tolerance = tolerance.max(0.0);
        let half_a = self.size() / 2.0 + Vector3::repeat(tolerance);
        let half_b = other.size() / 2.0 + Vector3::repeat(tolerance);
        let axes_a = frame_axes(&self.frame);
        let axes_b = frame_axes(&other.frame);

        let mut rotation = [[0.0; 3]; 3];
        let mut abs_rotation = [[0.0; 3]; 3];
        for i in 0..3 {
            for (j, axis_b) in axes_b.iter().enumerate() {
                rotation[i][j] = axes_a[i].dot(axis_b);
                abs_rotation[i][j] = rotation[i][j].abs() + f64::EPSILON;
            }
        }

        let center_delta = other.center() - self.center();
        let translation = Vector3::new(
            center_delta.dot(&axes_a[0]),
            center_delta.dot(&axes_a[1]),
            center_delta.dot(&axes_a[2]),
        );

        for i in 0..3 {
            let radius_a = half_a[i];
            let radius_b = half_b.x * abs_rotation[i][0]
                + half_b.y * abs_rotation[i][1]
                + half_b.z * abs_rotation[i][2];
            if translation[i].abs() > radius_a + radius_b {
                return false;
            }
        }

        for (j, axis_b) in axes_b.iter().enumerate() {
            let radius_a = half_a.x * abs_rotation[0][j]
                + half_a.y * abs_rotation[1][j]
                + half_a.z * abs_rotation[2][j];
            let radius_b = half_b[j];
            let distance = center_delta.dot(axis_b).abs();
            if distance > radius_a + radius_b {
                return false;
            }
        }

        for i in 0..3 {
            for j in 0..3 {
                let next_i = (i + 1) % 3;
                let last_i = (i + 2) % 3;
                let next_j = (j + 1) % 3;
                let last_j = (j + 2) % 3;
                let radius_a = half_a[next_i] * abs_rotation[last_i][j]
                    + half_a[last_i] * abs_rotation[next_i][j];
                let radius_b = half_b[next_j] * abs_rotation[i][last_j]
                    + half_b[last_j] * abs_rotation[i][next_j];
                let distance = (translation[last_i] * rotation[next_i][j]
                    - translation[next_i] * rotation[last_i][j])
                    .abs();
                if distance > radius_a + radius_b {
                    return false;
                }
            }
        }

        true
    }

    /// Grows the ranges to contain one more point; the frame does not change.
    pub fn extend(&mut self, point: Point3) {
        let local = self.frame.coordinates_of(point);
        self.min = self.min.inf(&local);
        self.max = self.max.sup(&local);
    }

    /// Grows this box to contain `other`; this box's frame does not change.
    ///
    /// When both boxes share a frame the ranges merge directly, so the result
    /// is the exact smallest box of both. Otherwise this box grows to `other`'s
    /// corners: it contains `other`, but bounds `other`'s geometry only as
    /// tightly as `other` itself does, seen from this frame.
    pub fn merge(&mut self, other: &Self) {
        if self.frame == other.frame {
            self.min = self.min.inf(&other.min);
            self.max = self.max.sup(&other.max);
        } else {
            for corner in other.corners() {
                self.extend(corner);
            }
        }
    }

    /// Returns the corner at the given end of each of the frame's ranges.
    pub fn corner(&self, x: Bound, y: Bound, z: Bound) -> Point3 {
        let pick = |bound, axis: usize| match bound {
            Bound::Min => self.min[axis],
            Bound::Max => self.max[axis],
        };
        self.frame
            .point_at(Vector3::new(pick(x, 0), pick(y, 1), pick(z, 2)))
    }

    /// Returns the eight corners of the box, x slowest and z fastest, each from
    /// its minimum to its maximum: `corners()[0]` is [`min`](Self::min) and
    /// `corners()[7]` is [`max`](Self::max).
    pub fn corners(&self) -> [Point3; 8] {
        use Bound::{Max, Min};
        [
            self.corner(Min, Min, Min),
            self.corner(Min, Min, Max),
            self.corner(Min, Max, Min),
            self.corner(Min, Max, Max),
            self.corner(Max, Min, Min),
            self.corner(Max, Min, Max),
            self.corner(Max, Max, Min),
            self.corner(Max, Max, Max),
        ]
    }

    // The principal axes of at least two points, at their centroid.
    fn principal_frame(points: &[Point3]) -> Frame {
        let centroid = Self::centroid(points);
        let covariance = Self::covariance(points, centroid);
        let eigens = SymmetricEigen::new(covariance);
        let mut axes = [
            (
                eigens.eigenvalues[0],
                eigens.eigenvectors.column(0).into_owned(),
            ),
            (
                eigens.eigenvalues[1],
                eigens.eigenvectors.column(1).into_owned(),
            ),
            (
                eigens.eigenvalues[2],
                eigens.eigenvectors.column(2).into_owned(),
            ),
        ];

        axes.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(Ordering::Equal));

        Frame::from_xy(centroid, axes[0].1, axes[1].1)
    }

    // Computes the point cloud centroid, the origin of the covariance
    // measurement and of the principal frame.
    fn centroid(points: &[Point3]) -> Point3 {
        let sum: Vector3<f64> = points.iter().map(|point| point.coords).sum();
        Point3::from(sum / points.len() as f64)
    }

    // Builds the covariance matrix used by PCA.
    //
    // Each point contributes `offset * offset.transpose()`, an outer product:
    // - diagonal terms measure spread along world x, y, and z;
    // - off-diagonal terms measure how pairs of coordinates vary together.
    //
    // The eigenvectors of this symmetric matrix are the principal directions of
    // the point cloud. Sorting eigenvalues from largest to smallest gives the
    // longest, middle, and shortest spread directions for the oriented frame.
    fn covariance(points: &[Point3], centroid: Point3) -> Matrix3<f64> {
        let mut covariance = Matrix3::zeros();
        for point in points {
            let offset = point - centroid;
            covariance += offset * offset.transpose();
        }
        covariance / points.len() as f64
    }
}

fn frame_axes(frame: &Frame) -> [Vector3<f64>; 3] {
    [
        *frame.x_dir.as_ref(),
        *frame.y_dir.as_ref(),
        *frame.z_dir.as_ref(),
    ]
}
