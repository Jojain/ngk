use super::super::{ConstraintKey, GeometryKey, Target2d};
use super::super::{
    constraints::StoredConstraint,
    keys::{ConstraintSlot, GeometrySlot},
    primitives::{Kind, Primitive},
};
use crate::geometry::dim2::utils::Point2;
use nalgebra::DVector;
use slotmap::SlotMap;
use std::collections::HashMap;

pub(super) struct Entry {
    pub key: GeometryKey,
    pub primitive: Primitive,
    pub offset: usize,
}
pub(super) struct Component {
    pub entries: Vec<Entry>,
    pub lookup: HashMap<GeometryKey, usize>,
    pub constraints: Vec<(ConstraintKey, StoredConstraint)>,
    pub targets: Vec<(usize, Target2d)>,
    pub origin: Point2,
    pub scale: f64,
    pub initial: DVector<f64>,
    pub columns: Vec<Option<usize>>,
    pub offsets: Vec<f64>,
    pub multipliers: Vec<f64>,
    pub eliminated_rows: HashMap<ConstraintKey, usize>,
}

/// Separates independent constraint components without relying on spatial proximity.
pub(super) fn prepare(
    owner: u64,
    geometry: &SlotMap<GeometrySlot, Primitive>,
    constraints: &SlotMap<ConstraintSlot, StoredConstraint>,
    targets: &[Target2d],
    reduce: bool,
) -> Vec<Component> {
    let primitives: Vec<_> = geometry
        .iter()
        .map(|(slot, p)| {
            (
                GeometryKey {
                    owner,
                    slot,
                    serial: p.serial,
                },
                p.clone(),
            )
        })
        .collect();
    let indices: HashMap<_, _> = primitives
        .iter()
        .enumerate()
        .map(|(i, (key, _))| (*key, i))
        .collect();
    let mut parents: Vec<_> = (0..primitives.len()).collect();
    for (_, constraint) in constraints {
        let keys = constraint.definition.geometries();
        let first = indices[&keys[0]];
        for key in &keys[1..] {
            let a = root(&mut parents, first);
            let b = root(&mut parents, indices[key]);
            parents[b] = a;
        }
    }
    let mut groups: Vec<Vec<(GeometryKey, Primitive)>> = vec![];
    let mut group_for_root = HashMap::new();
    let mut group_for_key = HashMap::new();
    for (i, (key, p)) in primitives.into_iter().enumerate() {
        let root = root(&mut parents, i);
        let group = *group_for_root.entry(root).or_insert_with(|| {
            groups.push(vec![]);
            groups.len() - 1
        });
        groups[group].push((key, p));
        group_for_key.insert(key, group);
    }
    let mut output: Vec<_> = groups.into_iter().map(Component::new).collect();
    for (slot, c) in constraints {
        let group = group_for_key[&c.definition.geometries()[0]];
        output[group].constraints.push((
            ConstraintKey {
                owner,
                slot,
                serial: c.serial,
            },
            c.clone(),
        ));
    }
    for (i, t) in targets.iter().enumerate() {
        output[group_for_key[&t.geometry()]]
            .targets
            .push((i, t.clone()));
    }
    if reduce {
        for component in &mut output {
            super::reduce::reduce(component);
        }
    }
    output
}
fn root(parents: &mut [usize], mut i: usize) -> usize {
    while parents[i] != i {
        parents[i] = parents[parents[i]];
        i = parents[i];
    }
    i
}
impl Component {
    fn new(primitives: Vec<(GeometryKey, Primitive)>) -> Self {
        let origin = Point2::new(primitives[0].1.values[0], primitives[0].1.values[1]);
        let mut scale = 0_f64;
        for (_, p) in &primitives {
            scale = scale.max((p.values[0] - origin.x).hypot(p.values[1] - origin.y));
            match p.kind {
                Kind::Segment => {
                    scale = scale.max((p.values[2] - p.values[0]).hypot(p.values[3] - p.values[1]))
                }
                Kind::Circle | Kind::Arc => scale = scale.max(p.values[2]),
                Kind::Point => {}
            }
        }
        if scale == 0. || !scale.is_finite() {
            scale = 1.;
        }
        let mut entries = vec![];
        let mut lookup = HashMap::new();
        let mut values = vec![];
        for (key, p) in primitives {
            let offset = values.len();
            for (i, v) in p.values.iter().enumerate() {
                values.push(normalize(*v, p.kind, i, origin, scale));
            }
            lookup.insert(key, entries.len());
            entries.push(Entry {
                key,
                primitive: p,
                offset,
            });
        }
        let count = values.len();
        Self {
            entries,
            lookup,
            constraints: vec![],
            targets: vec![],
            origin,
            scale,
            initial: DVector::from_vec(values),
            columns: (0..count).map(Some).collect(),
            offsets: vec![0.; count],
            multipliers: vec![1.; count],
            eliminated_rows: HashMap::new(),
        }
    }
    pub fn entry(&self, key: GeometryKey) -> &Entry {
        &self.entries[self.lookup[&key]]
    }
    pub fn normalized(&self, value: f64, kind: Kind, index: usize) -> f64 {
        normalize(value, kind, index, self.origin, self.scale)
    }
    pub fn value(&self, index: usize, x: &DVector<f64>) -> f64 {
        self.offsets[index]
            + self.columns[index].map_or(0., |column| x[column] * self.multipliers[index])
    }
    pub fn physical_values(&self, entry: &Entry, x: &DVector<f64>) -> Vec<f64> {
        (0..entry.primitive.values.len())
            .map(|i| {
                let value = self.value(entry.offset + i, x);
                match coordinate(entry.primitive.kind, i) {
                    Some(0) => value * self.scale + self.origin.x,
                    Some(_) => value * self.scale + self.origin.y,
                    None if entry.primitive.kind == Kind::Arc && i >= 3 => value,
                    None => value * self.scale,
                }
            })
            .collect()
    }
    /// Rejects trial geometry outside the supported domain before evaluating relations.
    pub fn valid(&self, x: &DVector<f64>) -> bool {
        if !x.iter().all(|v| v.is_finite()) {
            return false;
        }
        self.entries.iter().all(|e| {
            let p: Vec<_> = (0..e.primitive.values.len())
                .map(|i| self.value(e.offset + i, x))
                .collect();
            match e.primitive.kind {
                Kind::Point => true,
                Kind::Segment => (p[2] - p[0]).hypot(p[3] - p[1]) > 1e-14,
                Kind::Circle => p[2] > 0.,
                Kind::Arc => p[2] > 0. && p[4] > 0. && p[4] < std::f64::consts::TAU,
            }
        })
    }
}
fn coordinate(kind: Kind, index: usize) -> Option<usize> {
    if index < 2 || kind == Kind::Segment {
        Some(index % 2)
    } else {
        None
    }
}
fn normalize(value: f64, kind: Kind, index: usize, origin: Point2, scale: f64) -> f64 {
    match coordinate(kind, index) {
        Some(0) => (value - origin.x) / scale,
        Some(_) => (value - origin.y) / scale,
        None if kind == Kind::Arc && index >= 3 => value,
        None => value / scale,
    }
}
