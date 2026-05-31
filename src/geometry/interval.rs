#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Interval {
    pub start: f64,
    pub end: f64,
}

impl Interval {
    pub fn new(start: f64, end: f64) -> Self {
        Self { start, end }
    }

    pub fn ordered(self) -> Self {
        if self.start <= self.end {
            self
        } else {
            Self::new(self.end, self.start)
        }
    }

    pub fn length(self) -> f64 {
        (self.end - self.start).abs()
    }

    pub fn contains(self, value: f64, tolerance: f64) -> bool {
        let ordered = self.ordered();
        value >= ordered.start - tolerance && value <= ordered.end + tolerance
    }

    pub fn is_degenerate(self, tolerance: f64) -> bool {
        self.length() <= tolerance
    }

    pub fn intersects(self, other: Self, tolerance: f64) -> bool {
        self.intersection(other, tolerance).is_some()
    }

    pub fn intersection(self, other: Self, tolerance: f64) -> Option<Self> {
        let a = self.ordered();
        let b = other.ordered();
        let start = a.start.max(b.start);
        let end = a.end.min(b.end);
        if start <= end {
            return Some(Self::new(start, end));
        }
        if start - end <= tolerance {
            let midpoint = 0.5 * (start + end);
            return Some(Self::new(midpoint, midpoint));
        }
        None
    }
}
