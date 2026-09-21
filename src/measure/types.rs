use std::marker::PhantomData;

use nalgebra::{Matrix3, SymmetricEigen, Vector3};

use crate::geometry::{Axis3, Point3};

/// Marker for one-dimensional geometric extent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Length;

/// Marker for two-dimensional geometric extent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Area;

/// Marker for three-dimensional geometric extent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Volume;

/// A second-moment tensor about a property's centroid.
#[derive(Debug, Clone, Copy)]
pub struct Inertia<D> {
    tensor: Matrix3<f64>,
    measure: f64,
    dimension: PhantomData<D>,
}

impl<D> Inertia<D> {
    pub(crate) fn new(tensor: Matrix3<f64>, measure: f64) -> Self {
        Self {
            tensor,
            measure,
            dimension: PhantomData,
        }
    }

    /// Returns the symmetric inertia tensor in world coordinates.
    pub fn tensor(self) -> Matrix3<f64> {
        self.tensor
    }

    /// Returns the second moment about a unit axis through the centroid.
    pub fn moment_about(&self, axis: Axis3) -> f64 {
        axis.direction.dot(&(self.tensor * *axis.direction))
    }

    /// Returns the radius of gyration about a unit axis through the centroid.
    pub fn radius_of_gyration(&self, axis: Axis3) -> f64 {
        (self.moment_about(axis) / self.measure).sqrt()
    }

    /// Returns the principal moments and their orthonormal axes.
    pub fn principal(&self) -> PrincipalInertia<D> {
        let eigen = SymmetricEigen::new(self.tensor);
        PrincipalInertia {
            moments: eigen.eigenvalues,
            axes: eigen.eigenvectors,
            dimension: PhantomData,
        }
    }
}

/// Principal second moments and the corresponding orthonormal axes.
#[derive(Debug, Clone, Copy)]
pub struct PrincipalInertia<D> {
    moments: Vector3<f64>,
    axes: Matrix3<f64>,
    dimension: PhantomData<D>,
}

impl<D> PrincipalInertia<D> {
    /// Returns the principal moments as the columns of a vector.
    pub fn moments(self) -> Vector3<f64> {
        self.moments
    }

    /// Returns the principal axes as columns of an orthonormal matrix.
    pub fn axes(self) -> Matrix3<f64> {
        self.axes
    }
}

/// Length, centroid and centroidal inertia of a curve or profile.
#[derive(Debug, Clone, Copy)]
pub struct LinearProperties {
    pub length: f64,
    pub centroid: Point3,
    pub inertia: Inertia<Length>,
}

/// Area, centroid and centroidal inertia of a face or sheet.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceProperties {
    pub area: f64,
    pub centroid: Point3,
    pub inertia: Inertia<Area>,
}

/// Volume, centroid and centroidal inertia of a solid.
#[derive(Debug, Clone, Copy)]
pub struct VolumeProperties {
    pub volume: f64,
    pub centroid: Point3,
    pub inertia: Inertia<Volume>,
}
