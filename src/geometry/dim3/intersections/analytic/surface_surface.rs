//! Closed-form sections for recognized analytic surface pairs.
//!
//! Each entry returns the exact 3D section, plus a pcurve on each support that
//! is exact where a closed form exists and a measured fit otherwise. Both
//! pcurves are written over the section's own normalized domain, so the three
//! curves are synchronized by construction rather than by matching afterwards.

use std::f64::consts::TAU;

use nalgebra::{UnitVector3, Vector2, Vector3};

use super::{AnalyticSection, AnalyticSurfaceIntersection, PcurveFidelity};
use crate::geometry::axis::Axis3;
use crate::geometry::counters::count_surface_surface_analytic_call;
use crate::geometry::dim3::intersections::error::IntersectionError;
use crate::geometry::dim3::intersections::options::IntersectionOptions;
use crate::geometry::{
    Curve, Curve2, Cylinder, Ellipse, Frame, Interval, Line, NurbsCurve2, Plane, Point2, Point3,
    Sphere, Surface, TrimmedCurve, TrimmedCurve2,
};

/// Samples used for the first fitting attempt of one pcurve.
///
/// A section is a conic and the interpolant is cubic, so this is already far
/// denser than the fit needs; the doubling below exists for the eccentric
/// cases where it is not.
const INITIAL_FIT_SAMPLES: usize = 33;

/// Fitting attempts before a pcurve is reported at whatever it reached.
const MAX_FIT_REFINEMENTS: usize = 3;

/// Narrowest piece a split may leave, as a fraction of the section.
///
/// Not a tolerance on the geometry but on the *splitting*: below this a piece
/// carries no length worth a separate section, and both the section and its
/// pcurves refuse to be trimmed to it.
const MIN_PIECE_SPAN: f64 = 1.0e-6;

/// The window every section is written over.
///
/// Pcurves are normalized to it. An unbounded ruling keeps its own
/// arc-length parameter and is merely *described* over one unit of it: both
/// its pcurves are straight, and `Line2` extrapolates, so the description is
/// exact everywhere rather than only inside the window.
const SECTION_DOMAIN: Interval = Interval {
    start: 0.0,
    end: 1.0,
};

/// Intersects two surfaces in closed form, or declines the pair.
///
/// `None` means the pair is not in the table -- or is, but reached a case with
/// no representation in [`Curve`] -- and the caller must fall back to the
/// general solver. `Ok(AnalyticSurfaceIntersection::Empty)` is a proof of
/// disjointness, not an absence of results.
pub fn intersect_analytic_surfaces(
    a: &Surface,
    b: &Surface,
    options: IntersectionOptions,
) -> Option<Result<AnalyticSurfaceIntersection, IntersectionError>> {
    if !options.validate() {
        return Some(Err(IntersectionError::InvalidOptions));
    }
    let result = match (a, b) {
        (Surface::Plane(first), Surface::Plane(second)) => plane_plane(first, second, options),
        (Surface::Plane(plane), Surface::Sphere(sphere)) => {
            plane_sphere(plane, sphere, false, options)
        }
        (Surface::Sphere(sphere), Surface::Plane(plane)) => {
            plane_sphere(plane, sphere, true, options)
        }
        (Surface::Plane(plane), Surface::Cylinder(cylinder)) => {
            plane_cylinder(plane, cylinder, false, options)
        }
        (Surface::Cylinder(cylinder), Surface::Plane(plane)) => {
            plane_cylinder(plane, cylinder, true, options)
        }
        (Surface::Sphere(first), Surface::Sphere(second)) => sphere_sphere(first, second, options),
        _ => return None,
    };
    match result {
        Ok(Some(intersection)) => {
            count_surface_surface_analytic_call();
            Some(Ok(intersection))
        }
        // The pair is in the table but this configuration has no closed form
        // the curve types can carry, so the caller still needs the solver.
        Ok(None) => None,
        Err(error) => Some(Err(error)),
    }
}

/// Two planes meet in a line, are the same plane, or are parallel and apart.
fn plane_plane(
    a: &Plane,
    b: &Plane,
    options: IntersectionOptions,
) -> Result<Option<AnalyticSurfaceIntersection>, IntersectionError> {
    let (first_normal, second_normal) = (*a.normal(), *b.normal());
    let cross = first_normal.cross(&second_normal);
    let cross_squared = cross.norm_squared();
    if cross_squared <= options.angular_tolerance.powi(2) {
        let distance = (b.origin() - a.origin()).dot(&first_normal);
        return Ok(Some(if distance.abs() <= options.linear_tolerance {
            AnalyticSurfaceIntersection::Coincident
        } else {
            AnalyticSurfaceIntersection::Empty
        }));
    }

    let direction = cross.normalize();
    let first_offset = first_normal.dot(&a.origin().coords);
    let second_offset = second_normal.dot(&b.origin().coords);
    let origin = Point3::from(
        (first_offset * second_normal.cross(&cross) + second_offset * cross.cross(&first_normal))
            / cross_squared,
    );
    let curve = Curve::Line(Line::new(Axis3::new(origin, direction)));
    // A plane's parameters are Cartesian, so the image of a line is a line and
    // `Line2` extrapolates: this pcurve is exact over the whole unbounded
    // section, not only over the unit window it is written from.
    let plane_line = |plane: &Plane| {
        TrimmedCurve2::segment(
            plane.parameter_at(origin),
            plane.parameter_at(origin + direction),
        )
    };
    Ok(Some(AnalyticSurfaceIntersection::Sections(vec![
        AnalyticSection {
            curve: TrimmedCurve::new(curve, SECTION_DOMAIN),
            pcurve_a: plane_line(a),
            pcurve_b: plane_line(b),
            fidelity: PcurveFidelity::Exact,
        },
    ])))
}

/// A plane cuts a sphere in a circle, touches it, or misses it.
fn plane_sphere(
    plane: &Plane,
    sphere: &Sphere,
    swapped: bool,
    options: IntersectionOptions,
) -> Result<Option<AnalyticSurfaceIntersection>, IntersectionError> {
    let normal = *plane.normal();
    let centre = sphere.frame().origin;
    let distance = (centre - plane.origin()).dot(&normal);
    let section_centre = centre - normal * distance;
    let overlap = sphere.radius() - distance.abs();
    if overlap < -options.linear_tolerance {
        return Ok(Some(AnalyticSurfaceIntersection::Empty));
    }
    if overlap <= options.linear_tolerance {
        return Ok(Some(AnalyticSurfaceIntersection::TangentPoint(
            section_centre,
        )));
    }

    let radius = (sphere.radius() * sphere.radius() - distance * distance).sqrt();
    let section = circle_section(section_centre, plane.x_dir(), plane.normal(), radius);
    Ok(Some(AnalyticSurfaceIntersection::Sections(sections_for(
        section,
        Interval::new(0.0, TAU),
        supports(
            &Surface::Plane(plane.clone()),
            &Surface::Sphere(sphere.clone()),
            swapped,
        ),
        options,
    )?)))
}

/// Two spheres meet in a circle, touch at a point, coincide, or miss.
fn sphere_sphere(
    a: &Sphere,
    b: &Sphere,
    options: IntersectionOptions,
) -> Result<Option<AnalyticSurfaceIntersection>, IntersectionError> {
    let offset = b.frame().origin - a.frame().origin;
    let separation = offset.norm();
    if separation <= options.linear_tolerance {
        return Ok(Some(
            if (a.radius() - b.radius()).abs() <= options.linear_tolerance {
                AnalyticSurfaceIntersection::Coincident
            } else {
                AnalyticSurfaceIntersection::Empty
            },
        ));
    }
    // The section lies in the plane perpendicular to the centre line, at the
    // distance from the first centre where both radical equations agree.
    let axis = UnitVector3::new_normalize(offset);
    let distance = (separation * separation + a.radius() * a.radius() - b.radius() * b.radius())
        / (2.0 * separation);
    let centre = a.frame().origin + *axis * distance;
    let overlap = a.radius() - distance.abs();
    if separation > a.radius() + b.radius() + options.linear_tolerance
        || separation < (a.radius() - b.radius()).abs() - options.linear_tolerance
        || overlap < -options.linear_tolerance
    {
        return Ok(Some(AnalyticSurfaceIntersection::Empty));
    }
    if overlap <= options.linear_tolerance {
        return Ok(Some(AnalyticSurfaceIntersection::TangentPoint(centre)));
    }

    let radius = (a.radius() * a.radius() - distance * distance).sqrt();
    let reference = perpendicular_to(axis);
    let section = circle_section(centre, reference, axis, radius);
    Ok(Some(AnalyticSurfaceIntersection::Sections(sections_for(
        section,
        Interval::new(0.0, TAU),
        [Surface::Sphere(a.clone()), Surface::Sphere(b.clone())],
        options,
    )?)))
}

/// A plane cuts a cylinder in an ellipse, a circle, a line pair, or nothing.
fn plane_cylinder(
    plane: &Plane,
    cylinder: &Cylinder,
    swapped: bool,
    options: IntersectionOptions,
) -> Result<Option<AnalyticSurfaceIntersection>, IntersectionError> {
    let normal = plane.normal();
    let axis = cylinder.axis();
    let alignment = normal.dot(&axis);
    let supports = supports(
        &Surface::Plane(plane.clone()),
        &Surface::Cylinder(cylinder.clone()),
        swapped,
    );

    if alignment.abs() <= options.angular_tolerance {
        // The plane runs along the axis: the section is the pair of rulings at
        // the plane's own distance from the axis, or one where it is tangent.
        let offset = plane.origin() - cylinder.origin();
        let signed = offset.dot(&normal);
        if signed.abs() >= cylinder.radius - options.linear_tolerance {
            if signed.abs() > cylinder.radius + options.linear_tolerance {
                return Ok(Some(AnalyticSurfaceIntersection::Empty));
            }
            // A tangent ruling is a section of positive length, unlike a
            // tangent point, so it is reported as one.
            let foot = cylinder.origin() + *normal * signed;
            let mut sections = Vec::new();
            sections.extend(sections_for(
                ruling_section(foot, axis),
                SECTION_DOMAIN,
                supports.clone(),
                options,
            )?);
            return Ok(Some(AnalyticSurfaceIntersection::Sections(sections)));
        }
        let half_chord = (cylinder.radius * cylinder.radius - signed * signed).sqrt();
        let along = UnitVector3::new_normalize(axis.cross(&normal));
        let centre = cylinder.origin() + *normal * signed;
        let mut sections = Vec::new();
        for side in [1.0_f64, -1.0] {
            sections.extend(sections_for(
                ruling_section(centre + *along * (side * half_chord), axis),
                SECTION_DOMAIN,
                supports.clone(),
                options,
            )?);
        }
        return Ok(Some(AnalyticSurfaceIntersection::Sections(sections)));
    }

    // The plane crosses every ruling, so the section is a bounded conic: a
    // circle when the plane is square to the axis, an ellipse otherwise.
    let height = (plane.origin() - cylinder.origin()).dot(&normal) / alignment;
    let centre = cylinder.origin() + *axis * height;
    if (alignment.abs() - 1.0).abs() <= options.angular_tolerance {
        let section = circle_section(centre, cylinder.x_dir(), axis, cylinder.radius);
        return Ok(Some(AnalyticSurfaceIntersection::Sections(sections_for(
            section,
            Interval::new(0.0, TAU),
            supports,
            options,
        )?)));
    }

    // Along the plane's line of steepest ascent the ruling is met obliquely,
    // stretching the radius by 1/|cos|; square to it the radius is unchanged.
    let minor_dir = UnitVector3::new_normalize(axis.cross(&normal));
    let major_dir = UnitVector3::new_normalize(normal.cross(&minor_dir));
    let ellipse = Ellipse::new(
        Frame::from_xy(centre, major_dir, minor_dir),
        cylinder.radius / alignment.abs(),
        cylinder.radius,
    );
    let section = Curve::Ellipse(ellipse);
    Ok(Some(AnalyticSurfaceIntersection::Sections(sections_for(
        section,
        Interval::new(0.0, TAU),
        supports,
        options,
    )?)))
}

/// Maps a normalized piece onto a support curve's native parameter interval.
fn trim_section(interval: Interval, piece: Interval) -> Interval {
    Interval::new(interval.at(piece.start), interval.at(piece.end))
}

/// Orders the two supports the way the caller passed them.
fn supports(first: &Surface, second: &Surface, swapped: bool) -> [Surface; 2] {
    if swapped {
        [second.clone(), first.clone()]
    } else {
        [first.clone(), second.clone()]
    }
}

/// Returns a full-circle support in a plane.
fn circle_section(
    centre: Point3,
    x_dir: UnitVector3<f64>,
    normal: UnitVector3<f64>,
    radius: f64,
) -> Curve {
    Curve::circle(Plane::new(centre, x_dir, normal), radius)
}

/// Returns a cylinder ruling as an unbounded line through `point`.
fn ruling_section(point: Point3, axis: UnitVector3<f64>) -> Curve {
    Curve::Line(Line::new(Axis3::new(point, axis)))
}

/// Returns any unit vector perpendicular to `axis`.
fn perpendicular_to(axis: UnitVector3<f64>) -> UnitVector3<f64> {
    let reference = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    UnitVector3::new_normalize(axis.cross(&reference))
}

/// Writes one section's pcurves on both supports, splitting it at any seam.
///
/// The section arrives with its own normalized domain. Each support is asked
/// for a closed-form pcurve first; when none exists, or the candidate does not
/// reproduce the section, the pcurve is interpolated from the section's own
/// samples inverted through the support's closed-form projection.
///
/// A pcurve whose periodic parameter leaves one period is split rather than
/// wrapped, because a face trim classifies in `[0, 2pi]` and a loop that
/// leaves it is dropped rather than understood.
fn sections_for(
    curve: Curve,
    interval: Interval,
    supports: [Surface; 2],
    options: IntersectionOptions,
) -> Result<Vec<AnalyticSection>, IntersectionError> {
    let domain = SECTION_DOMAIN;
    let traces = [
        SectionTrace::build(&curve, interval, &supports[0], domain, options)?,
        SectionTrace::build(&curve, interval, &supports[1], domain, options)?,
    ];
    let mut cuts = vec![0.0_f64, 1.0];
    for trace in &traces {
        cuts.extend(trace.seam_crossings());
        cuts.extend(trace.degeneracy_crossings());
    }
    // Cuts arrive from two traces that can locate the same feature -- a
    // section leaving a pole right at the seam -- a rounding error apart.
    // Merging at the solver's parameter tolerance would keep both and leave a
    // piece too narrow for either the section or its pcurves to be trimmed to.
    for cut in &mut cuts {
        if *cut < MIN_PIECE_SPAN {
            *cut = 0.0;
        } else if *cut > 1.0 - MIN_PIECE_SPAN {
            *cut = 1.0;
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= MIN_PIECE_SPAN);

    let mut sections = Vec::new();
    for window in cuts.windows(2) {
        let piece = Interval::new(window[0], window[1]);
        if piece.end - piece.start <= MIN_PIECE_SPAN {
            continue;
        }
        let curve_interval = if cuts.len() == 2 {
            interval
        } else {
            trim_section(interval, piece)
        };
        let pcurve_a = traces[0].pcurve_over(piece, options)?;
        let pcurve_b = traces[1].pcurve_over(piece, options)?;
        sections.push(AnalyticSection {
            curve: TrimmedCurve::new(curve.clone(), curve_interval),
            pcurve_a: pcurve_a.0,
            pcurve_b: pcurve_b.0,
            fidelity: pcurve_a.1.combined(pcurve_b.1),
        });
    }
    Ok(sections)
}

/// One section inverted onto one support, before any pcurve is written.
struct SectionTrace {
    surface: Surface,
    /// The whole section, kept so samples are inverted rather than interpolated.
    curve: Curve,
    /// Native support interval corresponding to normalized parameters 0 and 1.
    interval: Interval,
    /// Section parameters at which the support was sampled.
    parameters: Vec<f64>,
    /// Support parameters, with the periodic direction unwrapped to be continuous.
    uv: Vec<Point2>,
    /// Period of the support's `u` direction, when it has one.
    period: Option<f64>,
    /// The exact pcurve, when the support admits one for this section.
    exact: Option<TrimmedCurve2>,
}

impl SectionTrace {
    fn build(
        curve: &Curve,
        interval: Interval,
        surface: &Surface,
        domain: Interval,
        options: IntersectionOptions,
    ) -> Result<Self, IntersectionError> {
        let period = match surface.periodicity() {
            crate::geometry::SurfacePeriodicity::UPeriodic(period)
            | crate::geometry::SurfacePeriodicity::UVPeriodic(period, _) => Some(period),
            _ => None,
        };
        let parameters = (0..INITIAL_FIT_SAMPLES)
            .map(|index| {
                domain.start
                    + (domain.end - domain.start) * index as f64 / (INITIAL_FIT_SAMPLES - 1) as f64
            })
            .collect::<Vec<_>>();
        let mut uv = Vec::with_capacity(parameters.len());
        for &parameter in &parameters {
            uv.push(surface.param_at(curve.point_at(interval.at(parameter)))?);
        }
        unwrap_periodic(&mut uv, period);
        let mut trace = Self {
            surface: surface.clone(),
            curve: curve.clone(),
            interval,
            parameters,
            uv,
            period,
            exact: None,
        };
        trace.exact = trace.exact_candidate(options);
        Ok(trace)
    }

    fn point_at(&self, parameter: f64) -> Point3 {
        self.curve.point_at(self.interval.at(parameter))
    }

    /// Returns a closed-form pcurve for the whole section, when one exists.
    ///
    /// Candidates are proposed from the trace and then *verified* against the
    /// section, so a candidate built with the wrong orientation is rejected
    /// rather than silently returned. Only a straight trace can be exact here:
    /// a section's image in a plane is already the section itself, and on a
    /// quadric only an aligned section keeps one parameter constant.
    fn exact_candidate(&self, options: IntersectionOptions) -> Option<TrimmedCurve2> {
        if let Surface::Plane(plane) = &self.surface {
            return plane_pcurve(plane, &self.curve, self.interval, options);
        }
        let first = *self.uv.first()?;
        let last = *self.uv.last()?;
        let candidate = TrimmedCurve2::segment(first, last);
        self.verifies(&candidate, options).then_some(candidate)
    }

    /// Whether a candidate pcurve reproduces the section within tolerance.
    fn verifies(&self, candidate: &TrimmedCurve2, options: IntersectionOptions) -> bool {
        self.deviation(candidate, Interval::new(0.0, 1.0)) <= options.linear_tolerance
    }

    /// A non-finite sample counts as infinitely far — see [`worst_distance`].
    /// Largest distance between the section and a pcurve lifted back onto the support.
    fn deviation(&self, pcurve: &TrimmedCurve2, piece: Interval) -> f64 {
        let samples = 4 * INITIAL_FIT_SAMPLES;
        (0..=samples)
            .map(|index| {
                let local = index as f64 / samples as f64;
                let uv = pcurve.point_at(local);
                let expected = self.point_at(piece.at(local));
                (self.surface.point_at(uv.x, uv.y) - expected).norm()
            })
            .fold(0.0_f64, worst_distance)
    }

    /// Section parameters at which the unwrapped `u` crosses a period boundary.
    fn seam_crossings(&self) -> Vec<f64> {
        let Some(period) = self.period else {
            return Vec::new();
        };
        if self.exact.is_some() && self.within_one_period(period) {
            return Vec::new();
        }
        let mut crossings = Vec::new();
        for window in self.uv.windows(2).enumerate() {
            let (index, pair) = window;
            let (start, end) = (pair[0].x, pair[1].x);
            let (low, high) = if start <= end {
                (start, end)
            } else {
                (end, start)
            };
            // Half-open on purpose. A sample landing exactly on a boundary is
            // the common case -- a section symmetric about the seam has one --
            // and a closed test makes both neighbouring segments decline it,
            // losing the crossing entirely.
            let first = (low / period).floor() as i64 + 1;
            let last = (high / period).floor() as i64;
            for step in first..=last {
                let boundary = step as f64 * period;
                if (end - start).abs() <= f64::EPSILON {
                    continue;
                }
                let fraction = (boundary - start) / (end - start);
                if (0.0..=1.0).contains(&fraction) {
                    let a = self.parameters[index];
                    let b = self.parameters[index + 1];
                    crossings.push(self.refined_crossing(a, b, a + (b - a) * fraction, boundary));
                }
            }
        }
        crossings
    }

    /// Sharpens a seam crossing located on the sampled polyline.
    ///
    /// The polyline puts the crossing within a sample spacing of the truth,
    /// which is not close enough: a piece that starts slightly early carries a
    /// pcurve that begins just outside the period, exactly the condition the
    /// splitting exists to prevent. Bisecting on the inverted section closes
    /// that gap without another inversion pass over the whole curve.
    fn refined_crossing(&self, low: f64, high: f64, seed: f64, boundary: f64) -> f64 {
        let Ok(seed_value) = self.uv_at(seed) else {
            return seed;
        };
        let (mut low, mut high) = (low, high);
        let increasing = self
            .uv_at(high)
            .map(|end| end.x >= seed_value.x)
            .unwrap_or(true);
        for _ in 0..40 {
            let middle = 0.5 * (low + high);
            let Ok(value) = self.uv_at(middle) else {
                return middle;
            };
            if (value.x < boundary) == increasing {
                low = middle;
            } else {
                high = middle;
            }
        }
        0.5 * (low + high)
    }

    /// Inverts the section at one parameter, on the trace's unwrapped branch.
    ///
    /// `closest_parameter` folds longitude back into one period, so the raw
    /// answer would jump wherever the trace was unwrapped. Shifting it by
    /// whole periods towards the trace keeps every sample on one branch while
    /// still being an exact inversion rather than an interpolation.
    fn uv_at(&self, parameter: f64) -> Result<Point2, IntersectionError> {
        let raw = self.surface.param_at(self.point_at(parameter))?;
        let Some(period) = self.period else {
            return Ok(raw);
        };
        let reference = self.interpolated_uv(parameter).x;
        Ok(Point2::new(
            raw.x + ((reference - raw.x) / period).round() * period,
            raw.y,
        ))
    }

    /// Section parameters where the support's parameterization collapses.
    ///
    /// A sphere's pole is one point carrying every longitude, so a section
    /// running through it has a genuine discontinuity that unwrapping cannot
    /// remove and no interpolation can follow. Cut there and each side becomes
    /// a meridian, which is not merely fittable but exact.
    ///
    /// The collapse is found as a zero of the `u` parameter line's speed,
    /// which is what "the parameterization collapses in `u`" means, so this
    /// needs no per-surface knowledge beyond the support's own degeneracy test.
    fn degeneracy_crossings(&self) -> Vec<f64> {
        if self.period.is_none() || self.uv.len() < 3 {
            return Vec::new();
        }
        let speeds = self
            .parameters
            .iter()
            .map(|&parameter| self.u_speed(parameter))
            .collect::<Vec<_>>();
        let fastest = speeds.iter().copied().fold(0.0_f64, f64::max);
        if fastest <= f64::EPSILON {
            return Vec::new();
        }
        let mut crossings = Vec::new();
        for index in 1..speeds.len() - 1 {
            // A real collapse is a deep dip. Requiring one keeps a section of
            // merely varying speed -- every non-aligned section has that --
            // from being cut at each of its slower samples.
            let dips = speeds[index] < 0.25 * fastest
                && speeds[index] <= speeds[index - 1]
                && speeds[index] <= speeds[index + 1];
            if !dips {
                continue;
            }
            let parameter =
                self.slowest_between(self.parameters[index - 1], self.parameters[index + 1]);
            if let Ok(uv) = self.uv_at(parameter)
                && self.surface.is_degenerate_at(uv.x, uv.y)
            {
                crossings.push(parameter);
            }
        }
        crossings
    }

    /// Speed of the support's `u` parameter line under the section at `parameter`.
    fn u_speed(&self, parameter: f64) -> f64 {
        let Ok(uv) = self.uv_at(parameter) else {
            return f64::INFINITY;
        };
        let step = 1.0e-6;
        (self.surface.point_at(uv.x + step, uv.y) - self.surface.point_at(uv.x - step, uv.y)).norm()
            / (2.0 * step)
    }

    /// Locates the slowest point of the `u` parameter line inside a window.
    fn slowest_between(&self, low: f64, high: f64) -> f64 {
        let (mut low, mut high) = (low, high);
        for _ in 0..60 {
            let first = low + (high - low) / 3.0;
            let second = high - (high - low) / 3.0;
            if self.u_speed(first) < self.u_speed(second) {
                high = second;
            } else {
                low = first;
            }
        }
        0.5 * (low + high)
    }

    /// Whether the whole trace already sits inside one period.
    fn within_one_period(&self, period: f64) -> bool {
        let (min, max) = self
            .uv
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), point| {
                (min.min(point.x), max.max(point.x))
            });
        min >= -f64::EPSILON && max <= period + f64::EPSILON
    }

    /// Writes this support's pcurve over one piece of the section.
    fn pcurve_over(
        &self,
        piece: Interval,
        options: IntersectionOptions,
    ) -> Result<(TrimmedCurve2, PcurveFidelity), IntersectionError> {
        if let Some(exact) = &self.exact {
            let trimmed = if piece.start <= options.parameter_tolerance
                && piece.end >= 1.0 - options.parameter_tolerance
            {
                exact.clone()
            } else {
                exact.sub(piece)
            };
            return Ok((
                shifted_into_period(trimmed, self.period),
                PcurveFidelity::Exact,
            ));
        }
        // Exactness is a property of the piece, not of the whole section: a
        // section cut at a pole has meridian pieces with closed forms even
        // though the section it came from had none.
        if let Some(exact) = self.exact_piece(piece, options) {
            return Ok((
                shifted_into_period(exact, self.period),
                PcurveFidelity::Exact,
            ));
        }
        self.fitted_over(piece, options)
    }

    /// Returns a closed-form pcurve for one piece, when the piece admits one.
    ///
    /// The candidate is built from interior samples rather than endpoints. An
    /// endpoint may sit on a degeneracy -- a pole reports longitude zero
    /// whatever meridian reaches it -- which would place the line nowhere near
    /// the piece it is meant to describe.
    fn exact_piece(&self, piece: Interval, options: IntersectionOptions) -> Option<TrimmedCurve2> {
        let sub_interval = trim_section(self.interval, piece);
        if let Surface::Plane(plane) = &self.surface {
            return plane_pcurve(plane, &self.curve, sub_interval, options);
        }
        let span = piece.end - piece.start;
        let first = self.uv_at(piece.start + span * 0.25).ok()?;
        let second = self.uv_at(piece.start + span * 0.75).ok()?;
        let direction = (second - first) * 2.0;
        let candidate = TrimmedCurve2::segment(first - direction * 0.25, second + direction * 0.25);
        (self.deviation(&candidate, piece) <= options.linear_tolerance).then_some(candidate)
    }

    /// Interpolates this support's pcurve over one piece of the section.
    fn fitted_over(
        &self,
        piece: Interval,
        options: IntersectionOptions,
    ) -> Result<(TrimmedCurve2, PcurveFidelity), IntersectionError> {
        let mut samples = INITIAL_FIT_SAMPLES;
        let mut best: Option<(TrimmedCurve2, f64)> = None;
        for _ in 0..MAX_FIT_REFINEMENTS {
            let (points, parameters) = self.resample(piece, samples)?;
            let fitted = NurbsCurve2::interpolate_with_parameters(&points, &parameters)?;
            let span = fitted.domain();
            let curve = TrimmedCurve2::new(Curve2::Nurbs(fitted), span);
            // The measured error is against the section itself, in model units,
            // so it means the same thing as any other linear tolerance here.
            let deviation = self.fit_deviation(&curve, piece);
            let improved = best.as_ref().is_none_or(|(_, best)| deviation < *best);
            if improved {
                best = Some((curve, deviation));
            }
            if deviation <= options.fit_tolerance {
                break;
            }
            samples = samples * 2 - 1;
        }
        let (curve, deviation) = best.expect("at least one fit is attempted");
        Ok((
            shifted_into_period(curve, self.period),
            PcurveFidelity::Fitted { deviation },
        ))
    }

    /// Samples the support parameters over one piece, at normalized positions.
    fn resample(
        &self,
        piece: Interval,
        samples: usize,
    ) -> Result<(Vec<Point2>, Vec<f64>), IntersectionError> {
        let mut points = Vec::with_capacity(samples);
        let mut parameters = Vec::with_capacity(samples);
        for index in 0..samples {
            let local = index as f64 / (samples - 1) as f64;
            points.push(self.uv_at(piece.start + (piece.end - piece.start) * local)?);
            parameters.push(local);
        }
        Ok((points, parameters))
    }

    /// Reads the trace's unwrapped parameters, interpolated between samples.
    ///
    /// This is only ever a *branch guide*: it says which period a parameter
    /// belongs to, never what the parameter is. Fitting against it directly
    /// would fit the sampled polyline instead of the section, capping accuracy
    /// at the polyline's own sagitta however dense the fit became.
    fn interpolated_uv(&self, parameter: f64) -> Point2 {
        match self
            .parameters
            .binary_search_by(|probe| probe.total_cmp(&parameter))
        {
            Ok(index) => self.uv[index],
            Err(0) => self.uv[0],
            Err(index) if index >= self.uv.len() => self.uv[self.uv.len() - 1],
            Err(index) => {
                let (a, b) = (self.parameters[index - 1], self.parameters[index]);
                let fraction = if (b - a).abs() <= f64::EPSILON {
                    0.0
                } else {
                    (parameter - a) / (b - a)
                };
                let (start, end) = (self.uv[index - 1], self.uv[index]);
                start + (end - start) * fraction
            }
        }
    }

    /// Largest departure of a fitted pcurve from the section, in model units.
    ///
    /// Measured after lifting the pcurve back onto the support, so the number
    /// is a distance in the model rather than in a parameter space whose scale
    /// varies across the surface.
    fn fit_deviation(&self, pcurve: &TrimmedCurve2, piece: Interval) -> f64 {
        let samples = 4 * INITIAL_FIT_SAMPLES;
        (0..=samples)
            .map(|index| {
                let local = index as f64 / samples as f64;
                let uv = pcurve.point_at(local);
                let expected = self.point_at(piece.at(local));
                (self.surface.point_at(uv.x, uv.y) - expected).norm()
            })
            .fold(0.0_f64, worst_distance)
    }
}

/// Folds sampled distances into the worst one, treating a non-finite sample as
/// infinitely far.
///
/// `f64::max` ignores `NaN`, so folding with it alone would report a candidate
/// that evaluates nowhere -- a straight pcurve proposed for a trace that closes
/// on itself, say -- as a perfect match and let it through verification.
fn worst_distance(worst: f64, distance: f64) -> f64 {
    if distance.is_finite() {
        worst.max(distance)
    } else {
        f64::INFINITY
    }
}

/// Translates a pcurve by whole periods until it lies inside `[0, period]`.
///
/// Unwrapping made the trace continuous; this puts the continuous result back
/// where a trim domain expects to find it. The piece has already been split so
/// that it spans no boundary, so its midpoint names the band it belongs to --
/// its endpoints do not, since one of them may sit exactly on a boundary and
/// name the wrong side.
fn shifted_into_period(curve: TrimmedCurve2, period: Option<f64>) -> TrimmedCurve2 {
    let Some(period) = period else {
        return curve;
    };
    let shift = -(curve.point_at(0.5).x / period).floor() * period;
    if shift == 0.0 {
        return curve;
    }
    curve
        .translated(Vector2::new(shift, 0.0))
        .unwrap_or(curve.clone())
}

/// Returns a section's exact pcurve in a plane's Cartesian parameters.
///
/// A plane's parameterization is an isometry, so a section's image is the
/// section itself with the same parameterization -- a line stays a line, a
/// circle a circle, an ellipse an ellipse.
fn plane_pcurve(
    plane: &Plane,
    curve: &Curve,
    interval: Interval,
    options: IntersectionOptions,
) -> Option<TrimmedCurve2> {
    let project = |point: Point3| plane.parameter_at(point);
    let candidate = match curve {
        Curve::Line(line) => TrimmedCurve2::segment(
            project(line.point_at(interval.start)),
            project(line.point_at(interval.end)),
        ),
        Curve::Circle(circle) => {
            let centre = project(circle.plane().origin());
            let start = project(circle.point_at(interval.start));
            // Anchoring the support on the section's own start puts its angle
            // at zero there, so the span is just the signed sweep from it.
            TrimmedCurve2::arc(
                centre,
                start - centre,
                circle.radius(),
                signed_sweep(plane, circle.plane().normal(), interval.delta()),
            )
        }
        Curve::Ellipse(ellipse) => {
            let centre = project(ellipse.frame().origin);
            let start = project(ellipse.point_at(interval.start));
            TrimmedCurve2::ellipse_arc(
                centre,
                start - centre,
                ellipse.major_radius(),
                ellipse.minor_radius(),
                signed_sweep(plane, ellipse.frame().z_dir, interval.delta()),
            )
        }
        _ => return None,
    };
    // A conic's sense in the plane's parameters follows whether the section's
    // own normal agrees with the plane's, which the sign above encodes; the
    // check keeps a mistake there from reaching a caller.
    let samples = 16;
    let worst = (0..=samples)
        .map(|index| {
            let local = index as f64 / samples as f64;
            let uv = candidate.point_at(local);
            (plane.point_at(uv.x, uv.y) - curve.point_at(interval.at(local))).norm()
        })
        .fold(0.0_f64, f64::max);
    (worst <= options.linear_tolerance).then_some(candidate)
}

/// Returns a sweep signed by whether the section turns with the plane's normal.
fn signed_sweep(plane: &Plane, section_normal: UnitVector3<f64>, sweep: f64) -> f64 {
    if section_normal.dot(&plane.normal()) >= 0.0 {
        sweep
    } else {
        -sweep
    }
}

/// Makes a periodic parameter continuous across the seam.
///
/// `closest_parameter` folds longitude into one period, so a section crossing
/// the seam comes back as a jump. Interpolating that jump would cut a chord
/// across the whole parameter domain, so it is undone before anything is fitted.
fn unwrap_periodic(uv: &mut [Point2], period: Option<f64>) {
    let Some(period) = period else {
        return;
    };
    let mut shift = 0.0;
    for index in 1..uv.len() {
        let previous = uv[index - 1].x;
        let mut current = uv[index].x + shift;
        while current - previous > 0.5 * period {
            shift -= period;
            current -= period;
        }
        while previous - current > 0.5 * period {
            shift += period;
            current += period;
        }
        uv[index].x = current;
    }
}
