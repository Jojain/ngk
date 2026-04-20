#[derive(Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
pub struct Dart(usize);

impl Dart {
    pub fn new(id: usize) -> Self {
        Self(id)
    }
    pub fn id(&self) -> usize {
        self.0
    }
}

pub struct IsolatedDart(pub Dart);

impl IsolatedDart {
    pub fn new(dart: Dart) -> Self {
        Self(dart)
    }
    pub fn id(&self) -> usize {
        self.0.id()
    }
}
