use super::gmap::{Dart, GMap};

pub struct SheetRef<'a> {
    gmap: &'a GMap,
    dart: Dart,
}

impl<'a> SheetRef<'a> {
    pub fn new(gmap: &'a GMap, dart: Dart) -> Self {
        Self { gmap, dart }
    }
}
