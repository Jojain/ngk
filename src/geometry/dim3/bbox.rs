use std::cmp::Ordering;

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use super::frame::Frame;
use super::utils::Point3;

/// A 3D bounding box that may be empty.
///
/// Non-empty boxes are oriented: their `frame` stores the center and axes, and
/// `size` stores the box length along each frame axis. `from_points` chooses the
/// initial frame with PCA so sampled point clouds can produce an oriented box.
#[derive(Clone)]
pub enum BBox {
    /// A box with no contained geometry.
    Empty,
    /// A centered oriented box with extents along its frame axes.
    NonEmpty {
        /// Center and orientation of the box.
        frame: Frame,
        /// Full box length along `frame.x_dir`, `frame.y_dir`, and `frame.z_dir`.
        size: Vector3<f64>,
    },
}

impl BBox {
    /// Creates an empty box suitable for incremental construction with `extend`.
    pub fn empty() -> Self {
        Self::Empty
    }

    /// Builds an oriented bounding box from a point cloud.
    ///
    /// Empty input returns `BBox::Empty`. A single point returns a non-empty
    /// zero-size box centered on that point. For larger point clouds, the frame
    /// orientation is estimated from the principal axes of the point covariance.
    pub fn from_points(points: impl IntoIterator<Item = Point3>) -> Self {
        let points = points.into_iter().collect::<Vec<_>>();
        Self::fit_points(&points)
    }

    /// Builds a box in a caller-selected frame and expands it over `points`.
    ///
    /// Analytic supports use their natural frame so extrema in parameter space
    /// produce an exact oriented box rather than a sampled PCA approximation.
    pub fn from_points_in_frame(frame: Frame, points: impl IntoIterator<Item = Point3>) -> Self {
        let points = points.into_iter().collect::<Vec<_>>();
        if points.is_empty() {
            Self::Empty
        } else {
            Self::fit_points_in_frame(&points, frame)
        }
    }

    /// Returns true when the box contains no points.
    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Empty)
    }

    /// Returns the negative local-space corner of the box.
    ///
    /// For an oriented box this is not necessarily the component-wise minimum in
    /// world coordinates. It is the corner at `(-x_size/2, -y_size/2, -z_size/2)`
    /// in the box frame.
    pub fn min(&self) -> Option<Point3> {
        match self {
            Self::Empty => None,
            Self::NonEmpty { frame, size } => Some(frame.point_at(-size / 2.0)),
        }
    }

    /// Returns the positive local-space corner of the box.
    ///
    /// For an oriented box this is not necessarily the component-wise maximum in
    /// world coordinates. It is the corner at `(x_size/2, y_size/2, z_size/2)` in
    /// the box frame.
    pub fn max(&self) -> Option<Point3> {
        match self {
            Self::Empty => None,
            Self::NonEmpty { frame, size } => Some(frame.point_at(size / 2.0)),
        }
    }

    /// Returns the centered frame of a non-empty box.
    pub fn frame(&self) -> Option<&Frame> {
        match self {
            Self::Empty => None,
            Self::NonEmpty { frame, .. } => Some(frame),
        }
    }

    /// Returns the full box size along each local frame axis.
    pub fn size(&self) -> Vector3<f64> {
        match self {
            Self::Empty => Vector3::zeros(),
            Self::NonEmpty { size, .. } => *size,
        }
    }

    /// Returns the full box size along the local x axis.
    pub fn x_size(&self) -> f64 {
        self.size().x
    }

    /// Returns the full box size along the local y axis.
    pub fn y_size(&self) -> f64 {
        self.size().y
    }

    /// Returns the full box size along the local z axis.
    pub fn z_size(&self) -> f64 {
        self.size().z
    }

    /// Returns half the box size along the local x axis.
    pub fn x_half_size(&self) -> f64 {
        self.x_size() / 2.
    }

    /// Returns half the box size along the local y axis.
    pub fn y_half_size(&self) -> f64 {
        self.y_size() / 2.
    }

    /// Returns half the box size along the local z axis.
    pub fn z_half_size(&self) -> f64 {
        self.z_size() / 2.
    }

    /// Returns the center of a non-empty box.
    pub fn center(&self) -> Option<Point3> {
        match self {
            Self::Empty => None,
            Self::NonEmpty { frame, .. } => Some(frame.origin),
        }
    }

    /// Returns the length of the box diagonal.
    pub fn diagonal_length(&self) -> f64 {
        self.size().norm()
    }

    /// Returns a box expanded by `tolerance` in every local direction.
    pub fn expanded(&self, tolerance: f64) -> Self {
        match self {
            Self::Empty => Self::Empty,
            Self::NonEmpty { frame, size } => {
                let delta = 2.0 * tolerance.max(0.0);
                Self::NonEmpty {
                    frame: frame.clone(),
                    size: size + Vector3::repeat(delta),
                }
            }
        }
    }

    /// Returns true when `point` lies inside this box within `tolerance`.
    pub fn contains_point(&self, point: Point3, tolerance: f64) -> bool {
        match self {
            Self::Empty => false,
            Self::NonEmpty { frame, size } => {
                let tolerance = tolerance.max(0.0);
                let half_size = *size / 2.0 + Vector3::repeat(tolerance);
                let local = frame.coordinates_of(point);
                local.x >= -half_size.x
                    && local.x <= half_size.x
                    && local.y >= -half_size.y
                    && local.y <= half_size.y
                    && local.z >= -half_size.z
                    && local.z <= half_size.z
            }
        }
    }

    /// Returns true when two oriented boxes overlap within `tolerance`.
    pub fn intersects(&self, other: &Self, tolerance: f64) -> bool {
        let (
            Self::NonEmpty {
                frame: frame_a,
                size: size_a,
            },
            Self::NonEmpty {
                frame: frame_b,
                size: size_b,
            },
        ) = (self, other)
        else {
            return false;
        };

        let tolerance = tolerance.max(0.0);
        let half_a = *size_a / 2.0 + Vector3::repeat(tolerance);
        let half_b = *size_b / 2.0 + Vector3::repeat(tolerance);
        let axes_a = frame_axes(frame_a);
        let axes_b = frame_axes(frame_b);

        let mut rotation = [[0.0; 3]; 3];
        let mut abs_rotation = [[0.0; 3]; 3];
        for i in 0..3 {
            for (j, axis_b) in axes_b.iter().enumerate() {
                rotation[i][j] = axes_a[i].dot(axis_b);
                abs_rotation[i][j] = rotation[i][j].abs() + f64::EPSILON;
            }
        }

        let center_delta = frame_b.origin - frame_a.origin;
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

    /// Expands the box to contain one more point.
    ///
    /// Extending an existing non-empty box preserves its current frame
    /// orientation and grows the local extents. It does not recompute PCA,
    /// because the box does not store the original point cloud.
    pub fn extend(&mut self, point: Point3) {
        match self {
            Self::Empty => {
                *self = Self::from_point(point);
            }
            Self::NonEmpty { frame, size } => {
                let half_size = *size / 2.0;
                let local = frame.coordinates_of(point);
                let min = Vector3::new(
                    (-half_size.x).min(local.x),
                    (-half_size.y).min(local.y),
                    (-half_size.z).min(local.z),
                );
                let max = Vector3::new(
                    half_size.x.max(local.x),
                    half_size.y.max(local.y),
                    half_size.z.max(local.z),
                );
                let center_local = (min + max) / 2.0;
                let new_center = frame.point_at(center_local);

                frame.origin = new_center;
                *size = max - min;
            }
        }
    }

    /// Expands this box to contain another box.
    ///
    /// If both boxes are non-empty, the receiver keeps its current frame and is
    /// extended by the other box corners.
    pub fn merge(&mut self, other: &Self) {
        match (&mut *self, other) {
            (_, Self::Empty) => {}
            (this @ Self::Empty, _) => {
                *this = other.clone();
            }
            (Self::NonEmpty { .. }, Self::NonEmpty { .. }) => {
                for corner in other.corners().expect("non-empty boxes have corners") {
                    self.extend(corner);
                }
            }
        }
    }

    /// Returns the union of this box and another box.
    pub fn union(mut self, other: &Self) -> Self {
        self.merge(other);
        self
    }

    fn fit_points(points: &[Point3]) -> Self {
        match points {
            [] => Self::Empty,
            [point] => Self::from_point(*point),
            _ => {
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

                let frame = Frame::from_xy(centroid, axes[0].1, axes[1].1);
                Self::fit_points_in_frame(points, frame)
            }
        }
    }

    fn from_point(point: Point3) -> Self {
        Self::NonEmpty {
            frame: Frame::from_xy(point, Vector3::x(), Vector3::y()),
            size: Vector3::zeros(),
        }
    }

    fn fit_points_in_frame(points: &[Point3], frame: Frame) -> Self {
        let mut min = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        for point in points {
            let local = frame.coordinates_of(*point);
            min = Vector3::new(min.x.min(local.x), min.y.min(local.y), min.z.min(local.z));
            max = Vector3::new(max.x.max(local.x), max.y.max(local.y), max.z.max(local.z));
        }

        let center = frame.point_at((min + max) / 2.0);

        Self::NonEmpty {
            frame: Frame::from_xy(center, frame.x_dir, frame.y_dir),
            size: max - min,
        }
    }

    // Computes the point cloud centroid. PCA uses this as the origin of the
    // covariance measurement, and the final box center is fitted afterward from
    // the min/max projected coordinates.
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

    /// Returns the eight corners of a non-empty box.
    pub fn corners(&self) -> Option<[Point3; 8]> {
        match self {
            Self::Empty => None,
            Self::NonEmpty { frame, size } => {
                let half_size = *size / 2.0;
                let signs = [
                    Vector3::new(-1.0, -1.0, -1.0),
                    Vector3::new(-1.0, -1.0, 1.0),
                    Vector3::new(-1.0, 1.0, -1.0),
                    Vector3::new(-1.0, 1.0, 1.0),
                    Vector3::new(1.0, -1.0, -1.0),
                    Vector3::new(1.0, -1.0, 1.0),
                    Vector3::new(1.0, 1.0, -1.0),
                    Vector3::new(1.0, 1.0, 1.0),
                ];

                Some(signs.map(|sign| frame.point_at(sign.component_mul(&half_size))))
            }
        }
    }
}

impl Default for BBox {
    fn default() -> Self {
        Self::empty()
    }
}

fn frame_axes(frame: &Frame) -> [Vector3<f64>; 3] {
    [
        *frame.x_dir.as_ref(),
        *frame.y_dir.as_ref(),
        *frame.z_dir.as_ref(),
    ]
}
