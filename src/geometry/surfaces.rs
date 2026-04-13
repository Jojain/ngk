

struct Plane {
    origin: Point,
    x_dir: Vector,
    normal: Vector,
}

impl Plane {
    pub fn new(origin: Point, x_dir: Vector, normal: Vector) -> Self {
        Self { origin, x_dir, normal }
    }
}



