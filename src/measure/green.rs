//! Integrals over a face's trimmed parameter domain, read off its boundary.
//!
//! Green's theorem turns an integral over the trimmed domain `D` into one
//! along its boundary:
//!
//! ```text
//! ∬_D g(u, v) du dv  =  ∮_∂D G(u, v) dv,      G(u, v) = ∫_{u_ref}^{u} g(s, v) ds
//! ```
//!
//! The outer integral runs along every piece of every loop, weighted by how
//! fast the piece climbs in `v`; at each of its points the inner integral runs
//! in `u` from the reference line to the boundary, at that point's `v`. Any
//! reference line gives the same answer. The one at the left of the face's
//! own parameter extent keeps the inner ranges as short as the face is wide,
//! whatever the support's own domain -- a plane has none.
//!
//! The loops come from [`UnwrappedFaceDomain`], which places a loop running
//! across a periodic support's cut as one continuous curve and closes ring and
//! cap faces with straight runs across the cut and along a collapsed row. Those
//! runs are integrated like any other piece. A seam or bridge is run twice in
//! opposite directions and cancels itself; nothing here special-cases one.
//!
//! The loops wind the way the face points: counter-clockwise in `(u, v)` when
//! the face's normal is `dS/du × dS/dv`, clockwise when it is reversed. The
//! integral therefore comes out signed by the face's sense, which is what a
//! flux wants and what an area has to undo.
//!
//! A face that nothing encloses -- a whole sphere or torus -- covers its
//! support's parameter rectangle, minus any hole. The rectangle is integrated
//! as one more loop, wound by the face's sense.

use nalgebra::{SVector, Vector2, Vector3};

use crate::geometry::parameter::{Fraction, NativeParam};
use crate::geometry::{Curve, Curve2, Interval, Point2, Point3, Surface, TrimmedCurve2};
use crate::topology::face::Face;
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::unwrapped_face_domain::{UnwrappedFaceDomain, UnwrappedFaceDomainLoop};

use super::integrate::MeasureError;
use super::quadrature::{Integral, Quadrature, Unconverged, integrate};

/// A face's trimmed parameter domain, as the boundary pieces that enclose it.
pub(crate) struct FaceDomain<'a> {
    surface: &'a Surface,
    pieces: Vec<Piece>,
    /// Where every inner integral starts.
    reference_u: f64,
}

/// One stretch of a closed boundary loop in parameter space.
enum Piece {
    /// A straight run with no pcurve: across a cut, along a collapsed row, or
    /// round a parameter rectangle.
    Segment { start: Point2, end: Point2 },
    /// A boundary edge's pcurve, moved by whole periods to where the loop
    /// runs through it.
    Pcurve {
        curve: TrimmedCurve2,
        offset: Vector2<f64>,
    },
}

impl Piece {
    /// The point at `fraction` and the derivative with respect to it.
    fn at(&self, fraction: Fraction) -> (Point2, Vector2<f64>) {
        match self {
            Piece::Segment { start, end } => {
                (start + (end - start) * fraction.value(), end - start)
            }
            Piece::Pcurve { curve, offset } => (
                curve.point_at(fraction) + offset,
                curve.derivative_at(fraction, 1),
            ),
        }
    }

    /// Fractions where the piece's `v` crosses one of the support's `v`
    /// breaks.
    ///
    /// `v` is sampled a few times along every stretch between `smooth`
    /// breakpoints, and each crossing bracketed by two samples is refined by
    /// bisection. A crossing and its return between the same two samples is
    /// missed, and left to the adaptive rule to find.
    fn v_crossings(&self, surface: &Surface, smooth: &[Fraction]) -> Vec<Fraction> {
        const SAMPLES_PER_STRETCH: usize = 4;
        let ends = std::iter::once(0.0)
            .chain(smooth.iter().map(|fraction| fraction.value()))
            .chain(std::iter::once(1.0))
            .collect::<Vec<_>>();
        let mut samples = ends
            .windows(2)
            .flat_map(|pair| {
                (0..SAMPLES_PER_STRETCH).map(move |step| {
                    pair[0] + (pair[1] - pair[0]) * step as f64 / SAMPLES_PER_STRETCH as f64
                })
            })
            .collect::<Vec<_>>();
        samples.push(1.0);
        let v_at = |fraction: f64| self.at(Fraction::new(fraction)).0.y;
        let values = samples
            .iter()
            .map(|fraction| v_at(*fraction))
            .collect::<Vec<_>>();
        let low = values.iter().copied().fold(f64::INFINITY, f64::min);
        let high = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        // A level piece crosses nothing; a non-finite one has no rows to find.
        if low >= high || !low.is_finite() || !high.is_finite() {
            return Vec::new();
        }
        let rows = v_breaks(surface, Interval::new(low, high))
            .into_iter()
            .map(|row| row.value())
            .collect::<Vec<_>>();
        let mut crossings = Vec::new();
        for (fractions, values) in samples.windows(2).zip(values.windows(2)) {
            let [(start, before), (end, after)] =
                [(fractions[0], values[0]), (fractions[1], values[1])];
            for row in &rows {
                if before == *row {
                    crossings.push(Fraction::new(start));
                } else if (before - row) * (after - row) < 0.0 {
                    let (mut low, mut high) = (start, end);
                    loop {
                        let middle = 0.5 * (low + high);
                        if middle == low || middle == high {
                            break;
                        }
                        match (v_at(middle) - row) * (before - row) > 0.0 {
                            true => low = middle,
                            false => high = middle,
                        }
                    }
                    crossings.push(Fraction::new(0.5 * (low + high)));
                }
            }
        }
        crossings
    }

    /// Whether the piece runs at one `v`, and so contributes nothing.
    fn is_level(&self) -> bool {
        match self {
            Piece::Segment { start, end } => start.y == end.y,
            Piece::Pcurve { curve, .. } => {
                matches!(curve.curve(), Curve2::Line(_)) && curve.start().y == curve.end().y
            }
        }
    }

    /// Fractions in `(0, 1)` where the integrand may lose smoothness: the
    /// pcurve's own knots and quarter turns, and wherever the piece crosses
    /// one of the support's `v` breaks.
    ///
    /// The second kind matters as much as the first. Every inner integral
    /// runs at the `v` its outer point sits at, so a support that is only
    /// twice differentiable across a knot row hands the outer integral a kink
    /// wherever the piece crosses that row -- and a fitted surface can have a
    /// hundred rows.
    fn breaks(&self, surface: &Surface) -> Vec<Fraction> {
        let mut fractions = match self {
            Piece::Pcurve { curve, .. } => {
                let interval = curve.interval();
                let native = match curve.curve() {
                    Curve2::Line(_) => Vec::new(),
                    Curve2::Circle(_) | Curve2::Ellipse(_) => quarter_turns(interval),
                    Curve2::Nurbs(nurbs) => distinct_knots(nurbs.knots().as_slice(), interval),
                };
                native
                    .into_iter()
                    .map(|parameter| interval.fraction_of(parameter))
                    .collect::<Vec<_>>()
            }
            Piece::Segment { .. } => Vec::new(),
        };
        normalize_fractions(&mut fractions);
        let crossings = self.v_crossings(surface, &fractions);
        fractions.extend(crossings);
        normalize_fractions(&mut fractions);
        fractions
    }
}

/// Keeps the fractions strictly inside `(0, 1)`, ascending and distinct.
fn normalize_fractions(fractions: &mut Vec<Fraction>) {
    fractions.retain(|fraction| fraction.value() > 0.0 && fraction.value() < 1.0);
    fractions.sort_by(|a, b| a.value().total_cmp(&b.value()));
    fractions.dedup();
}

impl<'a> FaceDomain<'a> {
    /// Reads `face`'s trimmed parameter domain off its boundary.
    pub(crate) fn of_face<P: Payload>(face: &'a Face<'_, P>) -> Result<Self, MeasureError> {
        let surface = face.surface();
        let unwrapped = UnwrappedFaceDomain::of_face(face).map_err(|source| {
            MeasureError::UnreadableBoundary {
                face: face.key(),
                source,
            }
        })?;
        let (enclosing, holes) = unwrapped
            .loops()
            .split_first()
            .expect("an unwrapped domain always has an enclosing loop, if an empty one");
        if !enclosing.is_empty() {
            let mut pieces = Vec::new();
            for loop_ in unwrapped.loops() {
                push_loop(&mut pieces, loop_);
            }
            return Ok(Self::new(surface, pieces, unwrapped.bounds().0.x));
        }

        // Nothing encloses the face, so it runs out to its support's own
        // parameter rectangle and is wound by its sense -- or, where it has
        // holes, against them, since a Boolean flips such a face by rewinding
        // its holes rather than by turning the view.
        let (u, v) = surface.domain();
        if !u.is_finite() || !v.is_finite() {
            return Err(MeasureError::UnboundedFace { face: face.key() });
        }
        let mut pieces = Vec::new();
        for hole in holes {
            push_loop(&mut pieces, hole);
        }
        let counter_clockwise = match pieces.is_empty() {
            true => face.sense() == Orientation::Same,
            false => {
                let hole = Self::new(surface, std::mem::take(&mut pieces), u.start.value());
                let winding = hole.parameter_area();
                pieces = hole.pieces;
                winding < 0.0
            }
        };
        let corners = [
            Point2::new(u.start.value(), v.start.value()),
            Point2::new(u.end.value(), v.start.value()),
            Point2::new(u.end.value(), v.end.value()),
            Point2::new(u.start.value(), v.end.value()),
        ];
        for index in 0..4 {
            let (start, end) = (corners[index], corners[(index + 1) % 4]);
            pieces.push(match counter_clockwise {
                true => Piece::Segment { start, end },
                false => Piece::Segment {
                    start: end,
                    end: start,
                },
            });
        }
        Ok(Self::new(surface, pieces, u.start.value()))
    }

    fn new(surface: &'a Surface, pieces: Vec<Piece>, reference_u: f64) -> Self {
        let pieces = pieces
            .into_iter()
            .filter(|piece| !piece.is_level())
            .collect();
        Self {
            surface,
            pieces,
            reference_u,
        }
    }

    /// Signed area of the domain in parameter space, positive when its loops
    /// wind counter-clockwise.
    fn parameter_area(&self) -> f64 {
        self.integrate(Quadrature::Fixed, |_, _, _| SVector::<f64, 1>::new(1.0))
            .map(|integral| integral.value[0])
            .unwrap_or(0.0)
    }

    /// Points spread over the face, for sizing it: a few along every piece.
    pub(crate) fn sample_points(&self) -> impl Iterator<Item = Point3> + '_ {
        const SAMPLES: [f64; 4] = [0.0, 0.25, 0.5, 0.75];
        self.pieces.iter().flat_map(move |piece| {
            SAMPLES.iter().map(move |fraction| {
                let (uv, _) = piece.at(Fraction::new(*fraction));
                self.surface.point_at(uv.x, uv.y)
            })
        })
    }

    /// Integrates `integrand` over the domain against `du dv`.
    ///
    /// `integrand` receives the surface point and its two partials, so a
    /// caller builds an area element from `|S_u × S_v|` or a flux from
    /// `S_u × S_v` itself. The result is signed by the loops' winding.
    pub(crate) fn integrate<const N: usize>(
        &self,
        quadrature: Quadrature,
        integrand: impl Fn(Point3, Vector3<f64>, Vector3<f64>) -> SVector<f64, N>,
    ) -> Result<Integral<N>, Unconverged> {
        // The inner integrals are smoothed by the outer one, so they are held
        // tighter than it to keep their own error out of its estimate.
        let inner_quadrature = match quadrature {
            Quadrature::Fixed => Quadrature::Fixed,
            Quadrature::Adaptive(tolerance) => Quadrature::Adaptive(0.1 * tolerance),
        };
        let inner = |u: f64, v: f64| {
            let breaks = u_breaks(self.surface, self.reference_u, u);
            integrate(inner_quadrature, &breaks, |s| {
                let (point, du, dv) = self.surface.point_and_derivative_at(s, v);
                Ok(integrand(point, du, dv))
            })
        };

        // Every piece is one unit of a single outer parameter, so one adaptive
        // integral spends its panels wherever the face needs them.
        let mut breaks = vec![0.0];
        for (index, piece) in self.pieces.iter().enumerate() {
            let base = index as f64;
            breaks.extend(
                piece
                    .breaks(self.surface)
                    .into_iter()
                    .map(|fraction| base + fraction.value()),
            );
            breaks.push(base + 1.0);
        }
        integrate(quadrature, &breaks, |along| {
            let index = (along.floor() as usize).min(self.pieces.len() - 1);
            let (uv, derivative) = self.pieces[index].at(Fraction::new(along - index as f64));
            if derivative.y == 0.0 {
                return Ok(SVector::zeros());
            }
            Ok(inner(uv.x, uv.y)?.value * derivative.y)
        })
    }
}

/// Appends one placed loop's pieces: for each pcurve, the straight runs
/// through the corners it turns through, then the pcurve itself.
fn push_loop(pieces: &mut Vec<Piece>, loop_: &UnwrappedFaceDomainLoop) {
    let curves = loop_.curves();
    let Some(last) = curves.last() else {
        return;
    };
    let mut previous = last.end();
    for curve in curves {
        for corner in curve.corners().iter().copied().chain([curve.start()]) {
            if corner != previous {
                pieces.push(Piece::Segment {
                    start: previous,
                    end: corner,
                });
            }
            previous = corner;
        }
        pieces.push(Piece::Pcurve {
            curve: curve.curve().clone(),
            offset: curve.offset(),
        });
        previous = curve.end();
    }
}

/// The inner integral's breakpoints from `from` to `to`, in that order.
///
/// They feed the quadrature directly, so they come back as the plain values
/// the support is evaluated at.
fn u_breaks(surface: &Surface, from: f64, to: f64) -> Vec<f64> {
    let span = Interval::new(from, to);
    let mut values = match surface {
        Surface::Plane(_) => Vec::new(),
        Surface::Cylinder(_) | Surface::Sphere(_) | Surface::Cone(_) | Surface::Torus(_) => {
            quarter_turns(span)
        }
        Surface::Ruled(ruled) => curve_breaks(ruled.curve(), span),
        Surface::Revolution(revolution) => curve_breaks(revolution.curve(), span),
        Surface::Nurbs(nurbs) => distinct_knots(nurbs.knots_u().as_slice(), span),
    };
    if to < from {
        values.reverse();
    }
    let mut breaks = Vec::with_capacity(values.len() + 2);
    breaks.push(from);
    breaks.extend(values.into_iter().map(|parameter| parameter.value()));
    breaks.push(to);
    breaks
}

/// The support's `v` breaks strictly inside `span`, ascending.
fn v_breaks(surface: &Surface, span: Interval) -> Vec<NativeParam> {
    match surface {
        Surface::Torus(_) | Surface::Revolution(_) => quarter_turns(span),
        Surface::Nurbs(nurbs) => distinct_knots(nurbs.knots_v().as_slice(), span),
        Surface::Plane(_)
        | Surface::Cylinder(_)
        | Surface::Sphere(_)
        | Surface::Cone(_)
        | Surface::Ruled(_) => Vec::new(),
    }
}

/// A 3D curve's breaks strictly inside `span` of its native parameter,
/// ascending.
pub(crate) fn curve_breaks(curve: &Curve, span: Interval) -> Vec<NativeParam> {
    match curve {
        Curve::Line(_) => Vec::new(),
        Curve::Circle(_) | Curve::Ellipse(_) | Curve::Helix(_) => quarter_turns(span),
        Curve::Nurbs(nurbs) => distinct_knots(nurbs.knots().as_slice(), span),
    }
}

/// Multiples of a quarter turn strictly inside `span`, ascending.
///
/// An angle-parameterized integrand is analytic everywhere, but a whole turn
/// in one panel is a long way for a polynomial rule to reach.
fn quarter_turns(span: Interval) -> Vec<NativeParam> {
    const QUARTER: f64 = std::f64::consts::FRAC_PI_2;
    let span = span.ordered();
    let (low, high) = (span.start.value(), span.end.value());
    let first = (low / QUARTER).floor() as i64 + 1;
    (first..)
        .map(|index| index as f64 * QUARTER)
        .take_while(|value| *value < high)
        .filter(|value| *value > low)
        .map(NativeParam::new)
        .collect()
}

/// Distinct knot values strictly inside `span`, ascending.
fn distinct_knots(knots: &[f64], span: Interval) -> Vec<NativeParam> {
    let span = span.ordered();
    let (low, high) = (span.start.value(), span.end.value());
    let mut values = knots
        .iter()
        .copied()
        .filter(|knot| *knot > low && *knot < high)
        .collect::<Vec<_>>();
    values.dedup();
    values.into_iter().map(NativeParam::new).collect()
}
