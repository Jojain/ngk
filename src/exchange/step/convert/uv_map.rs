//! The change of parameters between a STEP surface and the NGK surface it
//! maps to.
//!
//! Most surfaces share STEP's parameterization exactly, but not all of them
//! do, and every difference is orientation- or scale-bearing: getting one
//! wrong inverts the face normal and surfaces much later, far from the cause,
//! as a failed orientation validation over the whole solid.
//!
//! Every such difference is affine and axis-aligned — a transposition, a
//! uniform or per-axis scale, a shift — so **one value covers all of them**,
//! and the orientation question stops being a case analysis and becomes the
//! sign of a determinant. A surface conversion states its map once, as data,
//! and nothing downstream reasons about a sign by hand.
//!
//! | surface | swap | scale |
//! |---|---|---|
//! | plane, cylinder, sphere, torus, B-spline | no | `(1, 1)` |
//! | cone(α) | no | `(1, 1/cos α)` |
//! | surface of revolution | **yes** | `(1, 1)` |
//!
//! The direction is **STEP → NGK**: [`apply`](UvMap::apply) takes the
//! parameters a STEP entity is written in and returns the ones the NGK surface
//! answers to. [`inverted`](UvMap::inverted) is the same kind of value going
//! the other way, so a writer needs no second implementation.

use nalgebra::{Vector2, vector};

use crate::geometry::{
    Circle2, ControlPolygon2, Curve2, Ellipse2, HPoint2, Line2, NurbsCurve2, NurbsError, Point2,
    TrimmedCurve2,
};

/// An affine, axis-aligned change of surface parameters, STEP → NGK.
///
/// Applied in the order the fields are written: transpose, then scale, then
/// shift. That order is what makes [`inverted`](Self::inverted) another value
/// of the same shape rather than a composition of two.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UvMap {
    /// Whether the two parameter directions exchange roles.
    swap: bool,
    /// What each direction is multiplied by, after any swap.
    scale: Vector2<f64>,
    /// What each direction is shifted by, after the scale.
    offset: Vector2<f64>,
}

impl Default for UvMap {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl UvMap {
    /// The map of a surface NGK parameterizes exactly as STEP does.
    pub const IDENTITY: Self = Self {
        swap: false,
        scale: vector![1.0, 1.0],
        offset: vector![0.0, 0.0],
    };

    /// The map that exchanges the two parameter directions.
    ///
    /// A transposition is orientation-reversing in 2D, so it flips the
    /// boundary's signed area *and* the surface normal together.
    pub const TRANSPOSED: Self = Self {
        swap: true,
        ..Self::IDENTITY
    };

    /// The map that rescales each direction independently.
    pub const fn scaled(u: f64, v: f64) -> Self {
        Self {
            swap: false,
            scale: vector![u, v],
            offset: vector![0.0, 0.0],
        }
    }

    /// Returns this map with `offset` added to its result.
    pub fn shifted(self, offset: Vector2<f64>) -> Self {
        Self {
            offset: self.offset + offset,
            ..self
        }
    }

    /// Converts STEP parameters to the NGK surface's own.
    pub fn apply(&self, parameters: Point2) -> Point2 {
        let transposed = self.transpose(parameters.coords);
        Point2::from(
            vector![self.scale.x * transposed.x, self.scale.y * transposed.y] + self.offset,
        )
    }

    /// Converts NGK parameters back to the ones a STEP entity is written in.
    pub fn unapply(&self, parameters: Point2) -> Point2 {
        self.inverted().apply(parameters)
    }

    /// Returns the map from NGK parameters back to STEP's.
    ///
    /// Another value of the same shape, so writing needs no second
    /// implementation of anything here — a pcurve going out is the same
    /// [`map_pcurve`](Self::map_pcurve) called on this.
    pub fn inverted(&self) -> Self {
        let reciprocal = vector![1.0 / self.scale.x, 1.0 / self.scale.y];
        let shift = vector![-self.offset.x / self.scale.x, -self.offset.y / self.scale.y];
        Self {
            swap: self.swap,
            scale: self.transpose(reciprocal),
            offset: self.transpose(shift),
        }
    }

    /// Returns whether the map turns a boundary's winding inside out.
    ///
    /// The sign of the linear part's determinant, asked once so that no caller
    /// has to reason about which flips cancel: a transposition reverses
    /// orientation, a negative scale in one direction reverses it, and two
    /// negatives cancel.
    pub fn reverses_orientation(&self) -> bool {
        self.swap != (self.scale.x * self.scale.y < 0.0)
    }

    /// Returns whether the map's linear part preserves angles.
    ///
    /// A similarity carries a circle to a circle; anything else turns one into
    /// an ellipse, which is why a pcurve under a non-uniform scale cannot keep
    /// its analytic type.
    fn is_similarity(&self) -> bool {
        self.scale.x.abs() == self.scale.y.abs()
    }

    /// Converts a parameter curve from STEP's parameter space to NGK's.
    ///
    /// The support keeps its analytic type under every map for a line, and
    /// under a similarity for a circle or an ellipse. A non-uniform scale
    /// carries a circle to an ellipse whose axes are *not* the mapped ones, so
    /// there the support is demoted to NURBS rather than misreported.
    ///
    /// **What a demotion preserves is the point set, the two endpoints and the
    /// direction of travel — not the speed.** An affine map commutes with the
    /// rational basis, so the mapped control points describe the image exactly;
    /// but the section is carried over as its own NURBS, whose parameter is
    /// projective where a conic's is angular, so the same fraction no longer
    /// reaches the same point. That is what a `TrimmedCurve2` can hold: it is
    /// a support and an interval, with nowhere to put a conic reparameteriz-
    /// ation. It costs nothing here because a face's parameter curves are read
    /// as 2D geometry — for a winding, a domain polygon, a corner, a
    /// containment test — and never paired fraction-for-fraction against the
    /// edge's 3D curve.
    pub fn map_pcurve(&self, pcurve: &TrimmedCurve2) -> Result<TrimmedCurve2, NurbsError> {
        if let Some(support) = self.map_support(pcurve.curve()) {
            return Ok(TrimmedCurve2::new(support, pcurve.interval()));
        }
        // `to_nurbs` cuts the span down to a curve that *is* the section, so
        // the result spans the whole of what it returns and no interval of the
        // original parameterization survives to be misread.
        let section = pcurve.to_nurbs()?;
        Ok(TrimmedCurve2::whole(Curve2::Nurbs(
            self.map_nurbs(&section)?,
        )))
    }

    /// Maps a support that keeps its analytic type, or declines.
    ///
    /// Declining is not failure: it means the mapped point set is no longer
    /// of that type, and the caller converts instead.
    fn map_support(&self, support: &Curve2) -> Option<Curve2> {
        match support {
            // A line's parameter is affine and so is the map, so the two
            // compose without touching the parameter at all.
            Curve2::Line(line) => Some(Curve2::Line(Line2::new(
                self.apply(line.origin()),
                self.linear(line.derivative_at(0.0, 1)),
            ))),
            Curve2::Circle(circle) if self.is_similarity() => {
                let center = self.apply(circle.center());
                let x_dir = self.linear(*circle.x_dir());
                let mapped = Circle2::new(center, x_dir, circle.radius() * self.scale.x.abs());
                Some(Curve2::Circle(self.oriented(mapped, Circle2::reversed)))
            }
            Curve2::Ellipse(ellipse) if self.is_similarity() => {
                let center = self.apply(ellipse.center());
                let x_dir = self.linear(*ellipse.x_dir());
                let factor = self.scale.x.abs();
                let mapped = Ellipse2::new(
                    center,
                    x_dir,
                    ellipse.major_radius() * factor,
                    ellipse.minor_radius() * factor,
                );
                Some(Curve2::Ellipse(self.oriented(mapped, Ellipse2::reversed)))
            }
            Curve2::Circle(_) | Curve2::Ellipse(_) => None,
            Curve2::Nurbs(nurbs) => Some(Curve2::Nurbs(self.map_nurbs(nurbs).ok()?)),
        }
    }

    /// Maps a NURBS support by carrying its control points across.
    ///
    /// Weights and knots are untouched: the map is affine, so it commutes with
    /// the rational basis and the curve's own parameterization is preserved
    /// exactly rather than approximated.
    fn map_nurbs(&self, nurbs: &NurbsCurve2) -> Result<NurbsCurve2, NurbsError> {
        let points = nurbs
            .control_points()
            .as_slice()
            .iter()
            .map(|point| HPoint2::from_cartesian(self.apply(point.to_cartesian()), point.weight()))
            .collect();
        NurbsCurve2::new(
            nurbs.degree(),
            ControlPolygon2::new(points)?,
            nurbs.knots().clone(),
        )
    }

    /// Restores a conic's traversal sense after an orientation-reversing map.
    ///
    /// A conic built from a centre, a start direction and a radius is
    /// counter-clockwise by construction, but the map may have turned the
    /// plane over — in which case the same angular parameter now runs the
    /// other way, and the support has to say so or every trimmed span it
    /// carries reads backwards.
    fn oriented<T>(&self, conic: T, reverse: impl Fn(&T) -> T) -> T {
        if self.reverses_orientation() {
            reverse(&conic)
        } else {
            conic
        }
    }

    /// Applies the map's linear part, which carries directions rather than
    /// positions and so ignores the offset.
    fn linear(&self, vector: Vector2<f64>) -> Vector2<f64> {
        let transposed = self.transpose(vector);
        vector![self.scale.x * transposed.x, self.scale.y * transposed.y]
    }

    fn transpose(&self, vector: Vector2<f64>) -> Vector2<f64> {
        if self.swap {
            vector![vector.y, vector.x]
        } else {
            vector
        }
    }
}
