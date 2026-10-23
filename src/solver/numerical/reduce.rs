use super::super::primitives::Param;
use super::super::{Constraint2d, PointRef};
use super::prepare::Component;
use Constraint2d::*;
use nalgebra::DVector;
use std::collections::HashMap;

/// Reduces affine coordinate equalities without changing any public point identity.
pub(super) fn reduce(component: &mut Component) {
    let original = component.initial.clone();
    let count = component.initial.len();
    let mut aliases = Aliases::new(count);
    // Aligned segments: the parameters their length runs along, and its sign.
    let mut aligned = HashMap::new();
    for (key, c) in &component.constraints {
        let before = aliases.merges;
        let slot = |key, param| component.slot(key, param);
        match &c.definition {
            Coincident(a, b) | Distance(a, b, 0.) => {
                if let (Some(a), Some(b)) = (point_slots(component, *a), point_slots(component, *b))
                {
                    aliases.join(a.x, b.x, 0.);
                    aliases.join(a.y, b.y, 0.);
                }
            }
            Horizontal(line) => {
                aliases.join(slot(line.0, Param::Y), slot(line.0, Param::EndY), 0.);
                aligned.insert(line.0, (Param::X, Param::EndX, c.side));
            }
            Vertical(line) => {
                aliases.join(slot(line.0, Param::X), slot(line.0, Param::EndX), 0.);
                aligned.insert(line.0, (Param::Y, Param::EndY, c.side));
            }
            OffsetX(a, b, value) | OffsetY(a, b, value) => {
                if let (Some(a), Some(b)) = (point_slots(component, *a), point_slots(component, *b))
                {
                    let (a, b) = if matches!(c.definition, OffsetX(..)) {
                        (a.x, b.x)
                    } else {
                        (a.y, b.y)
                    };
                    aliases.join(a, b, value / component.scale);
                }
            }
            Concentric(a, b) => {
                aliases.join(slot(a.0, Param::X), slot(b.0, Param::X), 0.);
                aliases.join(slot(a.0, Param::Y), slot(b.0, Param::Y), 0.);
            }
            FixedPoint(p) => {
                if let Some(p) = point_slots(component, *p) {
                    aliases.fix(p.x, component.normalized(c.captured[0], Param::X));
                    aliases.fix(p.y, component.normalized(c.captured[1], Param::Y));
                }
            }
            Fixed(g) => {
                let layout = component.entry(*g).primitive.kind.layout();
                for (param, value) in layout.iter().zip(&c.captured) {
                    aliases.fix(slot(*g, *param), component.normalized(*value, *param));
                }
            }
            Radius(g, value) => {
                aliases.fix(slot(g.0, Param::Radius), value / component.scale);
            }
            EqualRadius(a, b) => {
                aliases.join(slot(a.0, Param::Radius), slot(b.0, Param::Radius), 0.);
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
            && let Some(&(start, end, side)) = aligned.get(&line.0)
        {
            let before = aliases.merges;
            aliases.join(
                component.slot(line.0, start),
                component.slot(line.0, end),
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
        if root == aliases.anchor {
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

/// The slots of a stored point's coordinates.
struct PointSlots {
    x: usize,
    y: usize,
}
/// An arc's endpoints are evaluated, not stored, so they have no slots.
fn point_slots(component: &Component, point: PointRef) -> Option<PointSlots> {
    let (x, y) = component
        .entry(point.geometry)
        .primitive
        .kind
        .point_params(point.role)?;
    Some(PointSlots {
        x: component.slot(point.geometry, x),
        y: component.slot(point.geometry, y),
    })
}

/// Weighted union-find where each edge stores x_child - x_parent. The extra
/// `anchor` element stands for zero: whatever joins it becomes a constant.
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
    /// Imposes x_slot = value.
    fn fix(&mut self, slot: usize, value: f64) {
        self.join(self.anchor, slot, value);
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
