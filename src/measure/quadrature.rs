//! One-dimensional Gauss quadrature over a list of breakpoints.
//!
//! Every measure reduces to nested one-dimensional integrals of smooth,
//! vector-valued integrands, and this file evaluates them. The integrand is
//! smooth between consecutive breakpoints -- a knot of the curve or surface it
//! follows, a quarter turn of a conic -- and possibly less so across them, so
//! no panel ever straddles one.
//!
//! Two rules share that contract:
//!
//! - [`Quadrature::Fixed`] runs eight-point Gauss–Legendre once per panel. It
//!   is exact for polynomials of degree fifteen and asks nothing about error,
//!   which is what a sign check wants.
//! - [`Quadrature::Adaptive`] runs Gauss–Kronrod 7–15 and keeps bisecting the
//!   panel with the largest error until every component's error is within the
//!   relative tolerance of the largest component's magnitude. The error of a
//!   panel is taken as the gap between its Kronrod and embedded Gauss
//!   estimates, which bounds the error of the seven-point rule; the
//!   fifteen-point estimate that is kept is far closer than that.
//!
//! Measuring every component against the largest is what lets a component that
//! cancels to nothing -- a first moment about a centre of symmetry, a face's
//! offset from a plane it lies in -- count as converged once its error is small
//! beside the rest. It asks the caller to make the components commensurate,
//! which for moments means taking offsets in units of the shape's size.

use nalgebra::SVector;

/// How an integral is evaluated.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Quadrature {
    /// Eight-point Gauss–Legendre on every panel between breakpoints.
    Fixed,
    /// Gauss–Kronrod 7–15, bisected until each component's error is within
    /// this fraction of the largest integral of a component's magnitude.
    Adaptive(f64),
}

/// An integral that could not reach its tolerance within the panel budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Unconverged;

/// A vector integral with the integral of each component's magnitude.
///
/// The magnitude is the scale a relative tolerance is measured against: an
/// integral that cancels to zero still has a size to be accurate relative to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Integral<const N: usize> {
    pub(crate) value: SVector<f64, N>,
    pub(crate) magnitude: SVector<f64, N>,
}

impl<const N: usize> Integral<N> {
    pub(crate) fn zero() -> Self {
        Self {
            value: SVector::zeros(),
            magnitude: SVector::zeros(),
        }
    }
}

impl<const N: usize> std::ops::AddAssign for Integral<N> {
    fn add_assign(&mut self, other: Self) {
        self.value += other.value;
        self.magnitude += other.magnitude;
    }
}

/// The most bisections one adaptive integral may make, on top of the panels
/// its breakpoints start it with.
///
/// Smooth integrands meet `1e-10` in a handful of bisections per breakpoint
/// interval; needing this many means the integrand is not smooth where the
/// breakpoints say it is, and the answer would not be what was asked for. A
/// fitted pcurve can start an integral with hundreds of knot spans, which is
/// why the budget does not count them.
const MAX_BISECTIONS: usize = 400;

/// Integrates `integrand` from the first breakpoint to the last.
///
/// `breaks` must be ordered, either way: a decreasing list integrates
/// backwards and negates the result, which is what an antiderivative read from
/// a reference line to a point on the other side of it needs. Fewer than two
/// breakpoints, or coincident ones, contribute nothing.
pub(crate) fn integrate<const N: usize>(
    quadrature: Quadrature,
    breaks: &[f64],
    mut integrand: impl FnMut(f64) -> Result<SVector<f64, N>, Unconverged>,
) -> Result<Integral<N>, Unconverged> {
    let panels = breaks
        .windows(2)
        .filter(|pair| pair[0] != pair[1])
        .map(|pair| (pair[0], pair[1]));
    match quadrature {
        Quadrature::Fixed => {
            let mut total = Integral::zero();
            for (start, end) in panels {
                total += gauss_legendre_8(start, end, &mut integrand)?;
            }
            Ok(total)
        }
        Quadrature::Adaptive(tolerance) => {
            let mut panels = panels
                .map(|(start, end)| Panel::evaluate(start, end, &mut integrand))
                .collect::<Result<Vec<_>, _>>()?;
            adapt(&mut panels, tolerance, &mut integrand)?;
            let mut total = Integral::zero();
            for panel in &panels {
                total += panel.integral;
            }
            Ok(total)
        }
    }
}

/// Bisects the worst panel until every component meets `tolerance`.
fn adapt<const N: usize>(
    panels: &mut Vec<Panel<N>>,
    tolerance: f64,
    integrand: &mut impl FnMut(f64) -> Result<SVector<f64, N>, Unconverged>,
) -> Result<(), Unconverged> {
    let mut bisections = 0;
    loop {
        let mut error = SVector::<f64, N>::zeros();
        let mut magnitude = SVector::<f64, N>::zeros();
        for panel in panels.iter() {
            error += panel.error;
            magnitude += panel.integral.magnitude;
        }
        // Rounding sets a floor no bisection gets under.
        let budget = tolerance.max(50.0 * f64::EPSILON) * magnitude.max();
        if error.max() <= budget {
            return Ok(());
        }
        if bisections == MAX_BISECTIONS {
            return Err(Unconverged);
        }
        bisections += 1;
        let worst = panels
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.error.max().total_cmp(&b.error.max()))
            .map(|(index, _)| index)
            .expect("a failed tolerance has a panel to blame");
        let Panel { start, end, .. } = panels[worst];
        let middle = 0.5 * (start + end);
        if middle == start || middle == end {
            return Err(Unconverged);
        }
        panels[worst] = Panel::evaluate(start, middle, integrand)?;
        panels.push(Panel::evaluate(middle, end, integrand)?);
    }
}

/// One Gauss–Kronrod panel and its error estimate.
#[derive(Debug, Clone, Copy)]
struct Panel<const N: usize> {
    start: f64,
    end: f64,
    integral: Integral<N>,
    error: SVector<f64, N>,
}

impl<const N: usize> Panel<N> {
    fn evaluate(
        start: f64,
        end: f64,
        integrand: &mut impl FnMut(f64) -> Result<SVector<f64, N>, Unconverged>,
    ) -> Result<Self, Unconverged> {
        let centre = 0.5 * (start + end);
        let half = 0.5 * (end - start);
        let mut kronrod = SVector::<f64, N>::zeros();
        let mut gauss = SVector::<f64, N>::zeros();
        let mut magnitude = SVector::<f64, N>::zeros();
        for (index, (node, weight)) in KRONROD_NODES.iter().zip(KRONROD_WEIGHTS).enumerate() {
            let samples = if *node == 0.0 {
                [integrand(centre)?, SVector::zeros()]
            } else {
                [
                    integrand(centre - half * node)?,
                    integrand(centre + half * node)?,
                ]
            };
            let sum = samples[0] + samples[1];
            kronrod += sum * weight;
            magnitude += (samples[0].abs() + samples[1].abs()) * weight;
            if index % 2 == 1 {
                gauss += sum * GAUSS_7_WEIGHTS[index / 2];
            }
        }
        Ok(Self {
            start,
            end,
            integral: Integral {
                value: kronrod * half,
                magnitude: magnitude * half.abs(),
            },
            error: ((kronrod - gauss) * half).abs(),
        })
    }
}

/// Eight-point Gauss–Legendre over one panel.
fn gauss_legendre_8<const N: usize>(
    start: f64,
    end: f64,
    integrand: &mut impl FnMut(f64) -> Result<SVector<f64, N>, Unconverged>,
) -> Result<Integral<N>, Unconverged> {
    let centre = 0.5 * (start + end);
    let half = 0.5 * (end - start);
    let mut value = SVector::<f64, N>::zeros();
    let mut magnitude = SVector::<f64, N>::zeros();
    for (node, weight) in GAUSS_8_NODES.iter().zip(GAUSS_8_WEIGHTS) {
        for sample in [
            integrand(centre - half * node)?,
            integrand(centre + half * node)?,
        ] {
            value += sample * weight;
            magnitude += sample.abs() * weight;
        }
    }
    Ok(Integral {
        value: value * half,
        magnitude: magnitude * half.abs(),
    })
}

/// Positive nodes of the eight-point Gauss–Legendre rule on `[-1, 1]`.
const GAUSS_8_NODES: [f64; 4] = [
    0.183_434_642_495_649_8,
    0.525_532_409_916_329,
    0.796_666_477_413_626_7,
    0.960_289_856_497_536_2,
];

const GAUSS_8_WEIGHTS: [f64; 4] = [
    0.362_683_783_378_362,
    0.313_706_645_877_887_3,
    0.222_381_034_453_374_5,
    0.101_228_536_290_376_3,
];

/// Non-negative nodes of the fifteen-point Kronrod rule on `[-1, 1]`, from
/// the outside in; the odd-indexed ones are the seven-point Gauss rule's.
const KRONROD_NODES: [f64; 8] = [
    0.991_455_371_120_812_6,
    0.949_107_912_342_758_5,
    0.864_864_423_359_769_1,
    0.741_531_185_599_394_4,
    0.586_087_235_467_691_1,
    0.405_845_151_377_397_2,
    0.207_784_955_007_898_5,
    0.0,
];

const KRONROD_WEIGHTS: [f64; 8] = [
    0.022_935_322_010_529_22,
    0.063_092_092_629_978_55,
    0.104_790_010_322_250_2,
    0.140_653_259_715_525_9,
    0.169_004_726_639_267_9,
    0.190_350_578_064_785_4,
    0.204_432_940_075_298_9,
    0.209_482_141_084_727_8,
];

/// Weights of the seven-point Gauss rule at the odd-indexed Kronrod nodes.
const GAUSS_7_WEIGHTS: [f64; 4] = [
    0.129_484_966_168_869_7,
    0.279_705_391_489_276_7,
    0.381_830_050_505_118_9,
    0.417_959_183_673_469_4,
];

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector1;

    fn scalar(f: impl Fn(f64) -> f64) -> impl FnMut(f64) -> Result<Vector1<f64>, Unconverged> {
        move |x| Ok(Vector1::new(f(x)))
    }

    /// `x^degree` integrated over `[a, b]` in closed form.
    fn monomial(degree: i32, a: f64, b: f64) -> f64 {
        (b.powi(degree + 1) - a.powi(degree + 1)) / f64::from(degree + 1)
    }

    #[test]
    fn eight_point_gauss_is_exact_to_degree_fifteen() {
        for degree in 0..=15 {
            let value = integrate(Quadrature::Fixed, &[-0.5, 1.5], scalar(|x| x.powi(degree)))
                .expect("a fixed rule always finishes")
                .value[0];
            let exact = monomial(degree, -0.5, 1.5);
            assert!(
                (value - exact).abs() <= 1e-14 * exact.abs().max(1.0),
                "degree {degree}: {value} against {exact}"
            );
        }
        let value = integrate(Quadrature::Fixed, &[-0.5, 1.5], scalar(|x| x.powi(16)))
            .expect("a fixed rule always finishes")
            .value[0];
        assert!(
            (value - monomial(16, -0.5, 1.5)).abs() > 1e-12,
            "degree sixteen is past what eight points integrate exactly"
        );
    }

    #[test]
    fn one_kronrod_panel_is_exact_to_degree_twenty_three_and_its_gauss_rule_to_thirteen() {
        let rule = |degree: i32| {
            Panel::evaluate(-0.5, 1.5, &mut scalar(|x| x.powi(degree)))
                .expect("an infallible integrand evaluates")
        };
        for degree in 0..=23 {
            let panel = rule(degree);
            let exact = monomial(degree, -0.5, 1.5);
            assert!(
                (panel.integral.value[0] - exact).abs() <= 1e-13 * exact.abs().max(1.0),
                "degree {degree}: {} against {exact}",
                panel.integral.value[0]
            );
            if degree <= 13 {
                assert!(
                    panel.error[0] <= 1e-13 * exact.abs().max(1.0),
                    "the seven-point rule is exact at degree {degree}"
                );
            }
        }
        assert!(
            rule(14).error[0] > 1e-12,
            "degree fourteen is past seven points"
        );
    }

    #[test]
    fn the_kronrod_estimate_bounds_the_true_error_on_smooth_integrands() {
        // An integrand, the interval it is integrated over, and the integral.
        type Case = (fn(f64) -> f64, f64, f64, f64);
        let cases: [Case; 4] = [
            (f64::exp, 0.0, 3.0, 3f64.exp() - 1.0),
            (|x| 1.0 / (1.0 + x * x), -4.0, 4.0, 2.0 * 4f64.atan()),
            (|x| (5.0 * x).cos(), 0.0, 2.0, (10f64).sin() / 5.0),
            (|x| (1.0 + x).sqrt(), 0.0, 8.0, 2.0 / 3.0 * (27.0 - 1.0)),
        ];
        for (f, a, b, exact) in cases {
            let panel = Panel::evaluate(a, b, &mut scalar(f)).expect("evaluates");
            let true_error = (panel.integral.value[0] - exact).abs();
            assert!(
                true_error <= panel.error[0].max(1e-15 * exact.abs()),
                "on [{a}, {b}] the estimate {} should bound the error {true_error}",
                panel.error[0]
            );
            let adaptive = integrate(Quadrature::Adaptive(1e-12), &[a, b], scalar(f))
                .expect("a smooth integrand converges")
                .value[0];
            assert!(
                (adaptive - exact).abs() <= 1e-12 * exact.abs(),
                "on [{a}, {b}] adaptive gave {adaptive} against {exact}"
            );
        }
    }

    #[test]
    fn breakpoints_keep_panels_off_a_kink_and_a_decreasing_list_integrates_backwards() {
        let kink = |x: f64| (x - 0.3).abs();
        let exact = (0.3f64.powi(2) + 0.7f64.powi(2)) / 2.0;
        let split = integrate(Quadrature::Fixed, &[0.0, 0.3, 1.0], scalar(kink))
            .expect("fixed")
            .value[0];
        assert!((split - exact).abs() <= 1e-15);
        let backwards = integrate(Quadrature::Fixed, &[1.0, 0.3, 0.0], scalar(kink))
            .expect("fixed")
            .value[0];
        assert!((backwards + exact).abs() <= 1e-15);
        // Without the breakpoint the adaptive rule still gets there, by
        // bisecting towards the kink.
        let adaptive = integrate(Quadrature::Adaptive(1e-10), &[0.0, 1.0], scalar(kink))
            .expect("a kink converges by bisection")
            .value[0];
        assert!((adaptive - exact).abs() <= 1e-10 * exact);
    }

    #[test]
    fn an_integrand_that_never_settles_is_refused() {
        let noisy = |x: f64| (1.0 / (x + 1e-300)).sin();
        assert_eq!(
            integrate(Quadrature::Adaptive(1e-12), &[0.0, 1.0], scalar(noisy)),
            Err(Unconverged)
        );
    }
}
