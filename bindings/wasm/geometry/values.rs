use nalgebra::{UnitVector3, Vector3};
use wasm_bindgen::prelude::*;

use crate::geometry::axis::Axis3;
use crate::geometry::{Frame, Point3};

pub(crate) fn point(value: Point3) -> WasmPoint3 {
    WasmPoint3 { inner: value }
}

/// Converts a kernel vector to its JavaScript exploration value.
pub(crate) fn vector(value: Vector3<f64>) -> WasmVector3 {
    WasmVector3 { inner: value }
}

pub(crate) fn unit_vector(value: UnitVector3<f64>) -> WasmVector3 {
    vector(value.into_inner())
}

#[wasm_bindgen(js_name = Point3)]
#[derive(Clone)]
pub struct WasmPoint3 {
    pub(crate) inner: Point3,
}

#[wasm_bindgen]
impl WasmPoint3 {
    /// Creates a three-dimensional point.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        point(Point3::new(x, y, z))
    }

    /// Returns the x coordinate.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x
    }

    /// Returns the y coordinate.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y
    }

    /// Returns the z coordinate.
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z
    }

    /// Returns `[x, y, z]`.
    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        vec![self.inner.x, self.inner.y, self.inner.z]
    }
}

/// Three-dimensional vector returned by JavaScript exploration methods.
#[wasm_bindgen(js_name = Vector3)]
#[derive(Clone)]
pub struct WasmVector3 {
    pub(crate) inner: Vector3<f64>,
}

#[wasm_bindgen]
impl WasmVector3 {
    /// Creates a three-dimensional vector.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        vector(Vector3::new(x, y, z))
    }

    /// Returns the x component.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x
    }

    /// Returns the y component.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y
    }

    /// Returns the z component.
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z
    }

    /// Returns `[x, y, z]`.
    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        vec![self.inner.x, self.inner.y, self.inner.z]
    }
}

/// A directed three-dimensional line used to place and orient geometry.
#[wasm_bindgen(js_name = Axis3)]
#[derive(Clone)]
pub struct WasmAxis3 {
    axis: Axis3,
}

#[wasm_bindgen]
impl WasmAxis3 {
    /// Creates an axis from an origin and direction.
    #[wasm_bindgen(constructor)]
    pub fn new(origin: &WasmPoint3, direction: &WasmVector3) -> Self {
        Self {
            axis: Axis3::new(origin.inner, direction.inner),
        }
    }

    /// Creates an axis through two points.
    #[wasm_bindgen(js_name = fromPoints)]
    pub fn from_points(start: &WasmPoint3, end: &WasmPoint3) -> Self {
        Self {
            axis: Axis3::from_points(start.inner, end.inner),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn origin(&self) -> WasmPoint3 {
        point(self.axis.origin)
    }

    #[wasm_bindgen(getter)]
    pub fn direction(&self) -> WasmVector3 {
        unit_vector(self.axis.direction)
    }

    /// Orthogonally projects a point onto the axis.
    pub fn project(&self, point_to_project: &WasmPoint3) -> WasmPoint3 {
        point(self.axis.project(point_to_project.inner))
    }
}

/// An orthonormal three-dimensional placement frame.
#[wasm_bindgen(js_name = Frame)]
#[derive(Clone)]
pub struct WasmFrame {
    pub(crate) inner: Frame,
}

#[wasm_bindgen]
impl WasmFrame {
    /// Creates a frame from its origin and local x/y directions.
    #[wasm_bindgen(constructor)]
    pub fn new(origin: &WasmPoint3, x_dir: &WasmVector3, y_dir: &WasmVector3) -> Self {
        Self {
            inner: Frame::from_xy(origin.inner, x_dir.inner, y_dir.inner),
        }
    }

    /// Returns the world XY frame.
    pub fn xyz() -> Self {
        Self {
            inner: Frame::xyz(),
        }
    }

    #[wasm_bindgen(js_name = fromXY)]
    pub fn from_xy(origin: &WasmPoint3, x_dir: &WasmVector3, y_dir: &WasmVector3) -> Self {
        Self::new(origin, x_dir, y_dir)
    }

    #[wasm_bindgen(js_name = fromXZ)]
    pub fn from_xz(origin: &WasmPoint3, x_dir: &WasmVector3, z_dir: &WasmVector3) -> Self {
        Self {
            inner: Frame::from_xz(origin.inner, x_dir.inner, z_dir.inner),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn origin(&self) -> WasmPoint3 {
        point(self.inner.origin)
    }

    #[wasm_bindgen(getter, js_name = xDir)]
    pub fn x_dir(&self) -> WasmVector3 {
        unit_vector(self.inner.x_dir)
    }

    #[wasm_bindgen(getter, js_name = yDir)]
    pub fn y_dir(&self) -> WasmVector3 {
        unit_vector(self.inner.y_dir)
    }

    #[wasm_bindgen(getter, js_name = zDir)]
    pub fn z_dir(&self) -> WasmVector3 {
        unit_vector(self.inner.z_dir)
    }

    #[wasm_bindgen(getter, js_name = xAxis)]
    pub fn x_axis(&self) -> WasmAxis3 {
        WasmAxis3 {
            axis: self.inner.x_axis(),
        }
    }

    #[wasm_bindgen(getter, js_name = yAxis)]
    pub fn y_axis(&self) -> WasmAxis3 {
        WasmAxis3 {
            axis: self.inner.y_axis(),
        }
    }

    #[wasm_bindgen(getter, js_name = zAxis)]
    pub fn z_axis(&self) -> WasmAxis3 {
        WasmAxis3 {
            axis: self.inner.z_axis(),
        }
    }

    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, coordinates: &WasmVector3) -> WasmPoint3 {
        point(self.inner.point_at(coordinates.inner))
    }

    #[wasm_bindgen(js_name = coordinatesOf)]
    pub fn coordinates_of(&self, point_to_locate: &WasmPoint3) -> WasmVector3 {
        vector(self.inner.coordinates_of(point_to_locate.inner))
    }
}
