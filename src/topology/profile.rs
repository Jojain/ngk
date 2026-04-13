use super::gmap::{Dart, GMap};

pub struct ProfileRef<'a> {
    gmap: &'a GMap<'a>,
    dart: Dart,
}

impl<'a> ProfileRef<'a> {
    pub fn new(gmap: &'a GMap<'a>, dart: Dart) -> Self {
        Self { gmap, dart }
    }
}

pub struct ClosedProfileRef<'a> {
    gmap: &'a GMap<'a>,
    dart: Dart,
}

impl<'a> ClosedProfileRef<'a> {
    pub fn new(gmap: &'a GMap<'a>, dart: Dart) -> Self {
        Self { gmap, dart }
    }
}
