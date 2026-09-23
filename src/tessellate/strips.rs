//! Cutting a face's parameter-space loops into strips between parallel lines.
//!
//! Two jobs share one cutter, because both are "cut every loop where it
//! crosses a line `origin + k * width`, then close the pieces up again along
//! those lines":
//!
//! - **Folding** ([`fold_into_period`]). The unwrapped domain writes each loop
//!   as one continuous polyline, and a loop may run over several periods of a
//!   periodic axis: a band winding round a cylinder is a long parallelogram,
//!   one turn per period. Such a face is not a polygon in any one period, and
//!   no bridge makes it one — a hole several periods long, cut out of a wall one
//!   period wide, overlaps its own outer loop on every other branch. So every
//!   piece is shifted back into the one period `[origin, origin + width]`.
//! - **Slicing** ([`cut_into_slabs`]). A face far longer than the scale its
//!   support bends on — a wall running eight turns of a helix — ear-clips into
//!   triangles spanning the whole of it, and refining those down to that scale
//!   shrinks them in every direction at once. Cut into slabs a support-scale
//!   wide first, each piece clips into triangles already the right length.
//!   Here the pieces stay where they are.
//!
//! Which stretches of a line belong to the face is asked of the face itself —
//! in the quotient when folding, because a point on the line is the same
//! surface point whichever branch it is read on. What comes out is a set of
//! ordinary polygons with holes, each of which the polygon path meshes as it
//! would any other face.

use std::collections::BTreeMap;

use crate::geometry::Point2;

use super::face::{point_in_polygon, signed_area};

/// One connected piece of a cut face: an enclosing loop and its holes.
pub(super) struct Strip {
    pub(super) outer: Vec<Point2>,
    pub(super) holes: Vec<Vec<Point2>>,
}

/// The lines a cut was made along.
///
/// A stretch of boundary lying on one of them was put there by the cut, not by
/// the face, so it bounds nothing a neighbouring face shares.
#[derive(Debug, Clone, Copy)]
pub(super) struct CutLines {
    axis: usize,
    origin: f64,
    width: f64,
}

impl CutLines {
    /// Whether the segment `a`–`b` runs along one of the lines.
    ///
    /// Exact, because the cutter snaps every point it puts on a line to the
    /// very value computed here.
    pub(super) fn hold(&self, a: Point2, b: Point2) -> bool {
        let value = a[self.axis];
        let turn = ((value - self.origin) / self.width).round();
        value == b[self.axis] && value == self.origin + turn * self.width
    }
}

/// Folds `outer` and `holes` into `[cut, cut + period]` along `axis`.
///
/// Returns `None` where the pieces will not close into loops, which is a
/// boundary that crosses itself on the surface rather than one this can mend.
pub(super) fn fold_into_period(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    axis: usize,
    cut: f64,
    period: f64,
    tolerance: f64,
) -> Option<(Vec<Strip>, CutLines)> {
    let cutter = Cutter {
        face: Face::new(outer, holes, axis, Some(period)),
        axis,
        origin: cut,
        width: period,
        fold: true,
        tolerance,
    };
    Some((cutter.run()?, cutter.lines()))
}

/// Cuts `outer` and `holes` into slabs `width` wide along `axis`.
///
/// The lines start half a slab before the face does, so that none of them
/// lands on a side the face already has at its extreme -- the edge of a
/// parameter rectangle, which a neighbouring face shares.
pub(super) fn cut_into_slabs(
    outer: &[Point2],
    holes: &[Vec<Point2>],
    axis: usize,
    width: f64,
    tolerance: f64,
) -> Option<(Vec<Strip>, CutLines)> {
    let face = Face::new(outer, holes, axis, None);
    let cutter = Cutter {
        axis,
        origin: face.extent.0 - 0.5 * width,
        width,
        fold: false,
        tolerance,
        face,
    };
    Some((cutter.run()?, cutter.lines()))
}

/// The face being cut, able to say which points lie in it.
struct Face {
    /// The enclosing loop, counter-clockwise, then each hole, clockwise.
    loops: Vec<Vec<Point2>>,
    axis: usize,
    /// The period along `axis`, when points are asked about in the quotient.
    period: Option<f64>,
    extent: (f64, f64),
}

impl Face {
    fn new(outer: &[Point2], holes: &[Vec<Point2>], axis: usize, period: Option<f64>) -> Self {
        let mut loops = vec![wound(outer, true)];
        loops.extend(holes.iter().map(|hole| wound(hole, false)));
        let extent = loops
            .iter()
            .flatten()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), point| {
                (min.min(point[axis]), max.max(point[axis]))
            });
        Self {
            loops,
            axis,
            period,
            extent,
        }
    }

    /// Whether `point` lies in the face.
    ///
    /// On a period, enclosed on some branch and in a hole on none — the same
    /// reading the Boolean's trim takes, for the same reason: a hole need not
    /// be written on the branch its enclosure is.
    fn contains(&self, point: Point2) -> bool {
        let turns = match self.period {
            Some(period) => {
                let first = ((self.extent.0 - point[self.axis]) / period).floor() as i64 - 1;
                let last = ((self.extent.1 - point[self.axis]) / period).ceil() as i64 + 1;
                (first..=last).map(|turn| turn as f64 * period).collect()
            }
            None => vec![0.0],
        };
        let (outer, holes) = self
            .loops
            .split_first()
            .expect("a face always has its enclosing loop");
        let mut enclosed = false;
        for shift in turns {
            let mut image = point;
            image[self.axis] += shift;
            if holes.iter().any(|hole| point_in_polygon(hole, image)) {
                return false;
            }
            enclosed |= point_in_polygon(outer, image);
        }
        enclosed
    }
}

/// Which line of its window a piece ends on.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    /// The window's lower line, where the face lies on the increasing side.
    Start,
    /// The window's upper line, where the face lies on the decreasing side.
    End,
}

/// A directed run of boundary, from its first point to its last.
struct Edge {
    points: Vec<Point2>,
}

impl Edge {
    fn start(&self) -> Point2 {
        self.points[0]
    }

    fn end(&self) -> Point2 {
        *self.points.last().expect("an edge has two points")
    }
}

/// What one window between two neighbouring lines collects.
#[derive(Default)]
struct Window {
    edges: Vec<Edge>,
    closed: Vec<Vec<Point2>>,
}

/// The lines being cut at, and where the pieces go.
struct Cutter {
    face: Face,
    axis: usize,
    origin: f64,
    width: f64,
    /// Whether every piece is shifted into window `0`.
    fold: bool,
    tolerance: f64,
}

impl Cutter {
    fn lines(&self) -> CutLines {
        CutLines {
            axis: self.axis,
            origin: self.origin,
            width: self.width,
        }
    }

    fn run(&self) -> Option<Vec<Strip>> {
        let mut windows = BTreeMap::<i64, Window>::new();
        for boundary in &self.face.loops {
            self.cut_loop(boundary, &mut windows);
        }
        let mut strips = Vec::new();
        for (&index, window) in &mut windows {
            for side in [Side::Start, Side::End] {
                self.close_along(index, side, &mut window.edges);
            }
            let mut loops = std::mem::take(&mut window.closed);
            loops.extend(chain(std::mem::take(&mut window.edges), self.tolerance)?);
            strips.extend(regions(loops)?);
        }
        Some(strips)
    }

    /// The window a coordinate falls in.
    fn turn(&self, value: f64) -> i64 {
        ((value - self.origin) / self.width).floor() as i64
    }

    /// The line a coordinate sits on, if it sits on one.
    fn on_line(&self, value: f64) -> bool {
        let turn = ((value - self.origin) / self.width).round();
        (value - (self.origin + turn * self.width)).abs() <= self.tolerance
    }

    /// The window a piece lying in window `turn` is placed in, and the shift
    /// that carries it there.
    fn placement(&self, turn: i64) -> (i64, f64) {
        match self.fold {
            true => (0, -(turn as f64) * self.width),
            false => (turn, 0.0),
        }
    }

    /// The coordinate of window `index`'s line on `side`.
    fn line(&self, index: i64, side: Side) -> f64 {
        let offset = match side {
            Side::Start => 0,
            Side::End => 1,
        };
        // Spelled as `CutLines::hold` spells it, so the two agree to the bit.
        self.origin + ((index + offset) as f64) * self.width
    }

    /// Cuts one loop at every line it crosses and files each piece under the
    /// window it is placed in.
    ///
    /// A stretch of the loop that runs *along* a line is dropped. Folding, it
    /// is where the unwrapped domain closed a loop across its own cut, which
    /// says nothing about the face; slicing, it is a side of the face that
    /// happens to lie on a line. Either way the stretches of line the face
    /// owns are put back by [`Self::close_along`], once per window.
    fn cut_loop(&self, boundary: &[Point2], windows: &mut BTreeMap<i64, Window>) {
        const ALONG_A_LINE: i64 = i64::MIN;
        let mut runs: Vec<(i64, Vec<Point2>)> = Vec::new();
        let count = boundary.len();
        for index in 0..count {
            let (a, b) = (boundary[index], boundary[(index + 1) % count]);
            for (from, to) in self.split(a, b) {
                let middle = (from[self.axis] + to[self.axis]) * 0.5;
                if self.on_line(from[self.axis])
                    && self.on_line(to[self.axis])
                    && self.on_line(middle)
                {
                    runs.push((ALONG_A_LINE, Vec::new()));
                    continue;
                }
                let turn = self.turn(middle);
                match runs.last_mut() {
                    Some((current, points)) if *current == turn => points.push(to),
                    _ => runs.push((turn, vec![from, to])),
                }
            }
        }
        runs.retain(|(turn, points)| *turn != ALONG_A_LINE && points.len() >= 2);

        // The walk started partway along whatever run holds the loop's first
        // point, so that run's two halves are rejoined.
        if runs.len() > 1 {
            let last = runs.len() - 1;
            if runs[0].0 == runs[last].0 && runs[last].1.last() == Some(&runs[0].1[0]) {
                let (_, head) = runs.remove(0);
                runs.last_mut()
                    .expect("more than one run")
                    .1
                    .extend(head.into_iter().skip(1));
            }
        }

        if runs.len() == 1 && !self.on_line(runs[0].1[0][self.axis]) {
            let (turn, mut points) = runs.pop().expect("one run");
            let (index, shift) = self.placement(turn);
            points.pop();
            for point in &mut points {
                point[self.axis] += shift;
            }
            windows.entry(index).or_default().closed.push(points);
            return;
        }
        for (turn, mut points) in runs {
            let (index, shift) = self.placement(turn);
            for point in &mut points {
                point[self.axis] += shift;
            }
            // Snapped onto the line exactly, so that the stretches added along
            // it meet the pieces bit for bit.
            for end in [0, points.len() - 1] {
                let (start, finish) = (self.line(index, Side::Start), self.line(index, Side::End));
                let value = points[end][self.axis];
                points[end][self.axis] = if (value - start).abs() <= (value - finish).abs() {
                    start
                } else {
                    finish
                };
            }
            windows
                .entry(index)
                .or_default()
                .edges
                .push(Edge { points });
        }
    }

    /// The segment `a`–`b`, split at every line strictly between them.
    fn split(&self, a: Point2, b: Point2) -> Vec<(Point2, Point2)> {
        let (from, to) = (a[self.axis], b[self.axis]);
        let (low, high) = (from.min(to), from.max(to));
        let first = ((low - self.origin) / self.width).floor() as i64 + 1;
        let last = ((high - self.origin) / self.width).ceil() as i64 - 1;
        let mut cuts = (first..=last)
            .map(|turn| self.origin + turn as f64 * self.width)
            .filter(|line| line - low > self.tolerance && high - line > self.tolerance)
            .map(|line| ((line - from) / (to - from), line))
            .collect::<Vec<_>>();
        cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut points = vec![a];
        points.extend(cuts.into_iter().map(|(t, line)| {
            let mut point = a + (b - a) * t;
            point[self.axis] = line;
            point
        }));
        points.push(b);
        points.windows(2).map(|pair| (pair[0], pair[1])).collect()
    }

    /// Adds the stretches of one of window `index`'s lines that bound the face.
    ///
    /// Every piece ending on the line marks a place the face may start or stop
    /// owning it, and between two neighbouring marks the face either owns the
    /// whole stretch or none of it. Which, the face answers from a point just
    /// inside the window. A stretch is walked with the face on its left.
    fn close_along(&self, index: i64, side: Side, edges: &mut Vec<Edge>) {
        let line = self.line(index, side);
        let other = 1 - self.axis;
        let mut marks = edges
            .iter()
            .flat_map(|edge| [edge.start(), edge.end()])
            .filter(|point| point[self.axis] == line)
            .map(|point| point[other])
            .collect::<Vec<_>>();
        marks.sort_by(f64::total_cmp);
        marks.dedup_by(|a, b| (*a - *b).abs() <= self.tolerance);

        let inward = match side {
            Side::Start => self.width * 1.0e-6,
            Side::End => -self.width * 1.0e-6,
        };
        let at = |along: f64| {
            let mut point = Point2::origin();
            point[self.axis] = line;
            point[other] = along;
            point
        };
        // With the face on the left, a stretch of the start line runs towards
        // lower values of the other axis when the cut axis is `u`, and towards
        // higher ones when it is `v`, since swapping the axes mirrors the
        // plane; the end line runs the other way in both.
        let descending = (side == Side::Start) == (self.axis == 0);
        for pair in marks.windows(2) {
            let mut probe = at((pair[0] + pair[1]) * 0.5);
            probe[self.axis] += inward;
            if !self.face.contains(probe) {
                continue;
            }
            let (low, high) = (at(pair[0]), at(pair[1]));
            let points = if descending {
                vec![high, low]
            } else {
                vec![low, high]
            };
            edges.push(Edge { points });
        }
    }
}

/// Joins directed edges end to start into closed loops.
fn chain(edges: Vec<Edge>, tolerance: f64) -> Option<Vec<Vec<Point2>>> {
    let mut used = vec![false; edges.len()];
    let mut loops = Vec::new();
    for seed in 0..edges.len() {
        if used[seed] {
            continue;
        }
        used[seed] = true;
        let origin = edges[seed].start();
        let mut points = edges[seed].points.clone();
        let mut end = edges[seed].end();
        while (end - origin).norm() > tolerance {
            let next = (0..edges.len())
                .find(|&index| !used[index] && (edges[index].start() - end).norm() <= tolerance)?;
            used[next] = true;
            points.extend(edges[next].points.iter().skip(1));
            end = edges[next].end();
        }
        points.pop();
        loops.push(points);
    }
    Some(loops)
}

/// Sorts closed loops into enclosing loops and the holes each one carries.
fn regions(loops: Vec<Vec<Point2>>) -> Option<Vec<Strip>> {
    let (outers, holes): (Vec<_>, Vec<_>) = loops
        .into_iter()
        .filter(|boundary| boundary.len() >= 3)
        .partition(|boundary| signed_area(boundary) > 0.0);
    let mut strips = outers
        .into_iter()
        .map(|outer| Strip {
            outer,
            holes: Vec::new(),
        })
        .collect::<Vec<_>>();
    for hole in holes {
        let owner = strips
            .iter_mut()
            .filter(|strip| point_in_polygon(&strip.outer, hole[0]))
            .min_by(|a, b| signed_area(&a.outer).total_cmp(&signed_area(&b.outer)))?;
        owner.holes.push(hole);
    }
    Some(strips)
}

/// `boundary` wound counter-clockwise when `ccw`, clockwise otherwise.
fn wound(boundary: &[Point2], ccw: bool) -> Vec<Point2> {
    let mut boundary = boundary.to_vec();
    if (signed_area(&boundary) > 0.0) != ccw {
        boundary.reverse();
    }
    boundary
}

#[cfg(test)]
mod tests {
    use super::{Strip, cut_into_slabs, fold_into_period};
    use crate::geometry::Point2;
    use crate::tessellate::face::signed_area;
    use std::f64::consts::TAU;

    /// A band winding `turns` times round a cylinder of unit pitch: a
    /// parallelogram in `(u, v)`, rising one unit of `v` per period of `u`.
    fn band(turns: f64) -> Vec<Point2> {
        let (start, end) = (0.5, 0.5 + turns * TAU);
        let rise = |u: f64| 1.0 + (u - start) / TAU;
        vec![
            Point2::new(start, rise(start) - 0.2),
            Point2::new(end, rise(end) - 0.2),
            Point2::new(end, rise(end) + 0.2),
            Point2::new(start, rise(start) + 0.2),
        ]
    }

    fn area(regions: &[Strip]) -> f64 {
        regions
            .iter()
            .map(|region| {
                signed_area(&region.outer).abs()
                    - region
                        .holes
                        .iter()
                        .map(|hole| signed_area(hole).abs())
                        .sum::<f64>()
            })
            .sum()
    }

    fn assert_within_period(regions: &[Strip]) {
        for point in regions
            .iter()
            .flat_map(|region| region.outer.iter().chain(region.holes.iter().flatten()))
        {
            assert!(
                (-1e-9..=TAU + 1e-9).contains(&point.x),
                "{point:?} is outside the period"
            );
        }
    }

    #[test]
    fn a_band_folds_into_one_period_with_its_area() {
        let band = band(3.5);
        let regions = fold_into_period(&band, &[], 0, 0.0, TAU, 1e-9)
            .expect("the band folds")
            .0;

        assert_within_period(&regions);
        let expected = signed_area(&band).abs();
        assert!(
            (area(&regions) - expected).abs() <= 1e-9 * expected,
            "folded area {} against {expected}",
            area(&regions)
        );
    }

    #[test]
    fn a_wall_keeps_its_area_less_a_band_cut_out_of_it() {
        let band = band(3.5);
        let wall = vec![
            Point2::new(0.0, 6.0),
            Point2::new(0.0, 0.0),
            Point2::new(TAU, 0.0),
            Point2::new(TAU, 6.0),
        ];
        let regions = fold_into_period(&wall, std::slice::from_ref(&band), 0, 0.0, TAU, 1e-9)
            .expect("the wall folds")
            .0;

        assert_within_period(&regions);
        let expected = TAU * 6.0 - signed_area(&band).abs();
        assert!(
            (area(&regions) - expected).abs() <= 1e-9 * expected,
            "folded area {} against {expected}",
            area(&regions)
        );
    }

    #[test]
    fn slabs_cover_the_face_they_cut_and_nothing_more() {
        let band = band(3.5);
        let wall = vec![
            Point2::new(-1.0, 0.0),
            Point2::new(30.0, 0.0),
            Point2::new(30.0, 6.0),
            Point2::new(-1.0, 6.0),
        ];
        let strips = cut_into_slabs(&wall, std::slice::from_ref(&band), 0, 0.7, 1e-9)
            .expect("the wall cuts")
            .0;

        assert!(strips.len() > 40, "{} slabs", strips.len());
        for strip in &strips {
            let (low, high) = strip
                .outer
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), point| {
                    (low.min(point.x), high.max(point.x))
                });
            assert!(high - low <= 0.7 + 1e-9, "a slab {} wide", high - low);
        }
        let expected = 31.0 * 6.0 - signed_area(&band).abs();
        assert!(
            (area(&strips) - expected).abs() <= 1e-9 * expected,
            "sliced area {} against {expected}",
            area(&strips)
        );
    }
}
