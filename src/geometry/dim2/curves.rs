use super::utils::Point2;

#[derive(Clone)]
pub enum Curve2 {
    Line(Line2),
    Polyline(Polyline2),
}

impl Curve2 {
    pub fn point_at(&self, t: f64) -> Point2 {
        match self {
            Curve2::Line(line) => line.point_at(t),
            Curve2::Polyline(polyline) => polyline.point_at(t),
        }
    }

    pub fn sample(&self, segments: usize) -> Vec<Point2> {
        match self {
            Curve2::Line(line) => line.sample(segments),
            Curve2::Polyline(polyline) => polyline.sample(segments),
        }
    }

    pub fn reversed(&self) -> Self {
        match self {
            Curve2::Line(line) => Curve2::Line(line.reversed()),
            Curve2::Polyline(polyline) => Curve2::Polyline(polyline.reversed()),
        }
    }
}

#[derive(Clone)]
pub struct Line2 {
    pub start: Point2,
    pub end: Point2,
}

impl Line2 {
    pub fn new(start: Point2, end: Point2) -> Self {
        Self { start, end }
    }

    pub fn point_at(&self, t: f64) -> Point2 {
        self.start + (self.end - self.start) * t
    }

    pub fn sample(&self, segments: usize) -> Vec<Point2> {
        let segments = segments.max(1);
        (0..=segments)
            .map(|i| self.point_at(i as f64 / segments as f64))
            .collect()
    }

    pub fn reversed(&self) -> Self {
        Self {
            start: self.end,
            end: self.start,
        }
    }
}

#[derive(Clone)]
pub struct Polyline2 {
    pub points: Vec<Point2>,
}

impl Polyline2 {
    pub fn new(points: Vec<Point2>) -> Self {
        Self { points }
    }

    pub fn point_at(&self, t: f64) -> Point2 {
        match self.points.as_slice() {
            [] => Point2::origin(),
            [point] => *point,
            points => {
                let t = t.clamp(0.0, 1.0);
                let segment_count = points.len() - 1;
                let scaled = t * segment_count as f64;
                let i = scaled.floor().min((segment_count - 1) as f64) as usize;
                let local_t = scaled - i as f64;
                points[i] + (points[i + 1] - points[i]) * local_t
            }
        }
    }

    pub fn sample(&self, segments: usize) -> Vec<Point2> {
        if self.points.len() <= 1 {
            return self.points.clone();
        }

        let segments = segments.max(1);
        (0..=segments)
            .map(|i| self.point_at(i as f64 / segments as f64))
            .collect()
    }

    pub fn reversed(&self) -> Self {
        let mut points = self.points.clone();
        points.reverse();
        Self { points }
    }
}
