use super::super::{ConstraintKey, GeometryKey, Target2d};
use super::super::{
    constraints::StoredConstraint,
    keys::{ConstraintSlot, GeometrySlot},
    primitives::{Kind, Param, Primitive, Quantity},
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
    /// The seed as solver unknowns, one per column.
    pub initial: DVector<f64>,
    /// Per parameter slot (see [`Component::slot`]), the normalized value is
    /// `offsets + multipliers * x[columns]`, or the constant `offsets` when
    /// the column is `None`.
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
        let first = &primitives[0].1;
        let origin = Point2::new(first.get(Param::X), first.get(Param::Y));
        let mut scale = 0_f64;
        for (_, p) in &primitives {
            scale = scale.max((p.get(Param::X) - origin.x).hypot(p.get(Param::Y) - origin.y));
            match p.kind {
                Kind::Segment => {
                    scale = scale.max(
                        (p.get(Param::EndX) - p.get(Param::X))
                            .hypot(p.get(Param::EndY) - p.get(Param::Y)),
                    )
                }
                Kind::Circle | Kind::Arc => scale = scale.max(p.get(Param::Radius)),
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
            for (param, v) in p.kind.layout().iter().zip(&p.values) {
                values.push(normalize(*v, param.quantity(), origin, scale));
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
    /// A parameter's position among the component's parameter slots, which
    /// lay every primitive's values end to end.
    pub fn slot(&self, key: GeometryKey, param: Param) -> usize {
        self.entry(key).offset + param.index()
    }
    pub fn normalized(&self, value: f64, param: Param) -> f64 {
        normalize(value, param.quantity(), self.origin, self.scale)
    }
    /// A slot's normalized value at the unknowns `x`.
    pub fn value(&self, slot: usize, x: &DVector<f64>) -> f64 {
        self.offsets[slot]
            + self.columns[slot].map_or(0., |column| x[column] * self.multipliers[slot])
    }
    pub fn parameter(&self, key: GeometryKey, param: Param, x: &DVector<f64>) -> f64 {
        self.value(self.slot(key, param), x)
    }
    pub fn physical_values(&self, entry: &Entry, x: &DVector<f64>) -> Vec<f64> {
        entry
            .primitive
            .kind
            .layout()
            .iter()
            .map(|param| {
                let value = self.value(entry.offset + param.index(), x);
                match param.quantity() {
                    Quantity::X => value * self.scale + self.origin.x,
                    Quantity::Y => value * self.scale + self.origin.y,
                    Quantity::Length => value * self.scale,
                    Quantity::Angle => value,
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
            let v = |param: Param| self.value(e.offset + param.index(), x);
            match e.primitive.kind {
                Kind::Point => true,
                Kind::Segment => {
                    (v(Param::EndX) - v(Param::X)).hypot(v(Param::EndY) - v(Param::Y)) > 1e-14
                }
                Kind::Circle => v(Param::Radius) > 0.,
                Kind::Arc => {
                    v(Param::Radius) > 0.
                        && v(Param::Sweep) > 0.
                        && v(Param::Sweep) < std::f64::consts::TAU
                }
            }
        })
    }
}
fn normalize(value: f64, quantity: Quantity, origin: Point2, scale: f64) -> f64 {
    match quantity {
        Quantity::X => (value - origin.x) / scale,
        Quantity::Y => (value - origin.y) / scale,
        Quantity::Length => value / scale,
        Quantity::Angle => value,
    }
}
