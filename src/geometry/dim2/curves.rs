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

    pub fn split_at(&self, t: f64) -> (Self, Self) {
        match self {
            Curve2::Line(line) => {
                let (first, second) = line.split_at(t);
                (Curve2::Line(first), Curve2::Line(second))
            }
            Curve2::Polyline(polyline) => {
                let (first, second) = polyline.split_at(t);
                (Curve2::Polyline(first), Curve2::Polyline(second))
            }
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

    pub fn split_at(&self, t: f64) -> (Self, Self) {
        let point = self.point_at(t.clamp(0.0, 1.0));
        (Self::new(self.start, point), Self::new(point, self.end))
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

    pub fn split_at(&self, t: f64) -> (Self, Self) {
        match self.points.as_slice() {
            [] => (Self::new(Vec::new()), Self::new(Vec::new())),
            [point] => (Self::new(vec![*point]), Self::new(vec![*point])),
            points => {
                let t = t.clamp(0.0, 1.0);
                let segment_count = points.len() - 1;
                let scaled = t * segment_count as f64;
                let segment = scaled.floor().min((segment_count - 1) as f64) as usize;
                let local_t = scaled - segment as f64;
                let split = points[segment] + (points[segment + 1] - points[segment]) * local_t;

                let mut first = points[..=segment].to_vec();
                if first.last().is_none_or(|point| *point != split) {
                    first.push(split);
                }

                let mut second = Vec::with_capacity(points.len() - segment);
                second.push(split);
                second.extend_from_slice(&points[segment + 1..]);

                (Self::new(first), Self::new(second))
            }
        }
    }
}
