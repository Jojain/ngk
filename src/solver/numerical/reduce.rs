use super::super::{Constraint2d, PointRef};
use super::super::{keys::PointRole, primitives::Kind};
use super::prepare::Component;
use nalgebra::DVector;
use std::collections::HashMap;

/// Reduces affine coordinate equalities without changing any public point identity.
pub(super) fn reduce(component: &mut Component) {
    let original = component.initial.clone();
    let count = component.initial.len();
    let mut aliases = Aliases::new(count);
    let mut directions = HashMap::new();
    for (key, c) in &component.constraints {
        let before = aliases.merges;
        use Constraint2d::*;
        match &c.definition {
            Coincident(a, b) | Distance(a, b, 0.) => {
                if let (Some(a), Some(b)) =
                    (point_indices(component, *a), point_indices(component, *b))
                {
                    aliases.join(a[0], b[0], 0.);
                    aliases.join(a[1], b[1], 0.);
                }
            }
            Horizontal(line) => {
                let offset = component.entry(line.0).offset;
                aliases.join(offset + 1, offset + 3, 0.);
                directions.insert(line.0, (0, c.side));
            }
            Vertical(line) => {
                let offset = component.entry(line.0).offset;
                aliases.join(offset, offset + 2, 0.);
                directions.insert(line.0, (1, c.side));
            }
            OffsetX(a, b, value) | OffsetY(a, b, value) => {
                if let (Some(a), Some(b)) =
                    (point_indices(component, *a), point_indices(component, *b))
                {
                    let axis = usize::from(matches!(c.definition, OffsetY(..)));
                    aliases.join(a[axis], b[axis], value / component.scale);
                }
            }
            Concentric(a, b) => {
                let a = component.entry(a.0).offset;
                let b = component.entry(b.0).offset;
                aliases.join(a, b, 0.);
                aliases.join(a + 1, b + 1, 0.);
            }
            FixedPoint(p) => {
                if let Some(indices) = point_indices(component, *p) {
                    aliases.join(
                        count,
                        indices[0],
                        (c.captured[0] - component.origin.x) / component.scale,
                    );
                    aliases.join(
                        count,
                        indices[1],
                        (c.captured[1] - component.origin.y) / component.scale,
                    );
                }
            }
            Fixed(g) => {
                let entry = component.entry(*g);
                for (i, value) in c.captured.iter().enumerate() {
                    aliases.join(
                        count,
                        entry.offset + i,
                        component.normalized(*value, entry.primitive.kind, i),
                    );
                }
            }
            Radius(g, value) => {
                let entry = component.entry(g.0);
                aliases.join(count, entry.offset + 2, value / component.scale);
            }
            EqualRadius(a, b) => {
                aliases.join(
                    component.entry(a.0).offset + 2,
                    component.entry(b.0).offset + 2,
                    0.,
                );
            }
            _ => {}
        }
        component
            .eliminated_rows
            .insert(*key, aliases.merges - before);
    }
    // An aligned segment's positive length is an affine endpoint offset on its selected branch.
    for (key, c) in &component.constraints {
        if let Constraint2d::Length(line, length) = c.definition
            && let Some(&(axis, side)) = directions.get(&line.0)
        {
            let before = aliases.merges;
            let offset = component.entry(line.0).offset;
            aliases.join(
                offset + axis,
                offset + axis + 2,
                side * length / component.scale,
            );
            *component.eliminated_rows.entry(*key).or_default() += aliases.merges - before;
        }
    }
    let mut group_for_root = HashMap::new();
    let mut groups: Vec<(f64, usize)> = vec![];
    for i in 0..count {
        let (root, offset) = aliases.find(i);
        component.offsets[i] = offset;
        if root == count {
            component.columns[i] = None;
            continue;
        }
        let group = *group_for_root.entry(root).or_insert_with(|| {
            groups.push((0., 0));
            groups.len() - 1
        });
        component.columns[i] = Some(group);
        groups[group].0 += component.initial[i] - offset;
        groups[group].1 += 1;
    }
    // Multiplicity scaling preserves minimum motion in the unreduced parameter metric.
    let values = groups
        .iter()
        .map(|(sum, n)| sum / (*n as f64).sqrt())
        .collect();
    for i in 0..count {
        if let Some(column) = component.columns[i] {
            component.multipliers[i] = 1. / (groups[column].1 as f64).sqrt();
        }
    }
    component.initial = DVector::from_vec(values);
    // Projection onto affine equalities can collapse a valid segment. Keep its original
    // seed and full parameter space so the nonlinear iteration can rotate it instead.
    if !component.valid(&component.initial) {
        component.initial = original;
        component.columns = (0..count).map(Some).collect();
        component.offsets.fill(0.);
        component.multipliers.fill(1.);
        component.eliminated_rows.clear();
    }
}

fn point_indices(component: &Component, point: PointRef) -> Option<[usize; 2]> {
    let entry = component.entry(point.geometry);
    match (entry.primitive.kind, point.role) {
        (Kind::Arc, PointRole::Start | PointRole::End) => None,
        (Kind::Segment, PointRole::End) => Some([entry.offset + 2, entry.offset + 3]),
        _ => Some([entry.offset, entry.offset + 1]),
    }
}

/// Weighted union-find where each edge stores x_child - x_parent.
struct Aliases {
    parents: Vec<usize>,
    offsets: Vec<f64>,
    anchor: usize,
    merges: usize,
}
impl Aliases {
    fn new(count: usize) -> Self {
        Self {
            parents: (0..=count).collect(),
            offsets: vec![0.; count + 1],
            anchor: count,
            merges: 0,
        }
    }
    fn find(&mut self, index: usize) -> (usize, f64) {
        let parent = self.parents[index];
        if parent == index {
            return (index, 0.);
        }
        let (root, offset) = self.find(parent);
        self.offsets[index] += offset;
        self.parents[index] = root;
        (root, self.offsets[index])
    }
    /// Imposes x_b - x_a = delta. Cycles remain explicit equations for final validation.
    fn join(&mut self, a: usize, b: usize, delta: f64) {
        let (ra, da) = self.find(a);
        let (rb, db) = self.find(b);
        if ra == rb {
            return;
        }
        if rb == self.anchor {
            self.parents[ra] = rb;
            self.offsets[ra] = db - da - delta;
        } else {
            self.parents[rb] = ra;
            self.offsets[rb] = da + delta - db;
        }
        self.merges += 1;
    }
}
