use super::super::system::Body;
use super::super::{BodyId, Joint3d, JointId, Target3d};
use crate::geometry::dim3::frame::Frame;
use crate::geometry::dim3::utils::Point3;
use crate::geometry::transform::Rigid;
use nalgebra::{DVector, Matrix3, Unit, Vector3};
use std::collections::{BTreeMap, HashMap};

/// The number of unknowns of a free body: a translation and a rotation vector.
pub(super) const BODY_UNKNOWNS: usize = 6;

/// A body inside one component.
pub(super) struct Entry {
    pub id: BodyId,
    /// The pose the body starts from: its seed, or its fixing.
    pub start: Rigid,
    /// The orientation the rotation unknowns turn from.
    pub rotation: Matrix3<f64>,
    /// The world translation the body starts from.
    pub translation: Vector3<f64>,
    /// The first of the body's six unknowns; `None` for a fixed body, which is constant.
    pub column: Option<usize>,
    /// Whether this component reports the body. A fixed body another component's
    /// joints hold is borrowed there as a constant.
    pub owned: bool,
}

/// Bodies whose poses depend on each other, the joints between them, and the targets
/// on them. Free bodies are linked by joints; a fixed body links nothing, being constant.
pub(super) struct Component {
    pub entries: Vec<Entry>,
    lookup: HashMap<BodyId, usize>,
    pub joints: Vec<(JointId, Joint3d)>,
    pub targets: Vec<(usize, Target3d)>,
    /// Translation unknowns are `(translation - origin) / scale`.
    pub origin: Vector3<f64>,
    pub scale: f64,
    pub initial: DVector<f64>,
}

impl Component {
    pub fn entry(&self, body: BodyId) -> &Entry {
        &self.entries[self.lookup[&body]]
    }
    /// Unknowns in normalized units: translations relative to `origin` over `scale`,
    /// rotation vectors in radians from each entry's `rotation`.
    pub fn columns(&self) -> usize {
        self.initial.len()
    }
    /// The world pose of an entry at the unknowns `x`.
    pub fn pose(&self, entry: &Entry, x: &DVector<f64>) -> Rigid {
        let Some(column) = entry.column else {
            return entry.start;
        };
        let translation =
            self.origin + Vector3::new(x[column], x[column + 1], x[column + 2]) * self.scale;
        let turn =
            nalgebra::Rotation3::new(Vector3::new(x[column + 3], x[column + 4], x[column + 5]));
        rigid(turn.matrix() * entry.rotation, translation)
    }
}

/// Splits the assembly into components that solve independently.
pub(super) fn prepare(
    bodies: &BTreeMap<BodyId, Body>,
    joints: &BTreeMap<JointId, Joint3d>,
    targets: &[Target3d],
) -> Vec<Component> {
    let ids: Vec<BodyId> = bodies.keys().copied().collect();
    let index: HashMap<BodyId, usize> = ids.iter().enumerate().map(|(i, id)| (*id, i)).collect();
    let free = |id: &BodyId| bodies[id].fixed.is_none();
    let mut parents: Vec<usize> = (0..ids.len()).collect();
    for joint in joints.values() {
        let [a, b] = joint.bodies();
        if free(&a) && free(&b) {
            let ra = root(&mut parents, index[&a]);
            let rb = root(&mut parents, index[&b]);
            parents[rb] = ra;
        }
    }
    let mut group_of_root = HashMap::new();
    let mut members: Vec<Vec<BodyId>> = vec![];
    let mut group_of_body = HashMap::new();
    for (i, id) in ids.iter().enumerate() {
        let r = root(&mut parents, i);
        let group = *group_of_root.entry(r).or_insert_with(|| {
            members.push(vec![]);
            members.len() - 1
        });
        members[group].push(*id);
        group_of_body.insert(*id, group);
    }
    let mut grouped_joints: Vec<Vec<(JointId, Joint3d)>> = vec![vec![]; members.len()];
    for (id, joint) in joints {
        let [a, b] = joint.bodies();
        let anchor = if free(&a) || !free(&b) { a } else { b };
        grouped_joints[group_of_body[&anchor]].push((*id, joint.clone()));
    }
    let mut grouped_targets: Vec<Vec<(usize, Target3d)>> = vec![vec![]; members.len()];
    for (i, target) in targets.iter().enumerate() {
        grouped_targets[group_of_body[&target.body]].push((i, target.clone()));
    }
    members
        .into_iter()
        .zip(grouped_joints)
        .zip(grouped_targets)
        .map(|((owned, joints), targets)| Component::new(bodies, owned, joints, targets))
        .collect()
}

fn root(parents: &mut [usize], mut i: usize) -> usize {
    while parents[i] != i {
        parents[i] = parents[parents[i]];
        i = parents[i];
    }
    i
}

impl Component {
    fn new(
        bodies: &BTreeMap<BodyId, Body>,
        owned: Vec<BodyId>,
        joints: Vec<(JointId, Joint3d)>,
        targets: Vec<(usize, Target3d)>,
    ) -> Self {
        let mut ids: Vec<(BodyId, bool)> = owned.iter().map(|id| (*id, true)).collect();
        for (_, joint) in &joints {
            for body in joint.bodies() {
                if !ids.iter().any(|(id, _)| *id == body) {
                    ids.push((body, false));
                }
            }
        }
        let mut entries = vec![];
        let mut lookup = HashMap::new();
        let mut columns = 0;
        for (id, owned) in ids {
            let body = &bodies[&id];
            let start = body.fixed.unwrap_or(body.seed);
            let (rotation, translation) = parts(start);
            let column = body.fixed.is_none().then(|| {
                columns += BODY_UNKNOWNS;
                columns - BODY_UNKNOWNS
            });
            lookup.insert(id, entries.len());
            entries.push(Entry {
                id,
                start,
                rotation,
                translation,
                column,
                owned,
            });
        }
        let origin = entries[0].translation;
        let mut scale = 0_f64;
        for entry in &entries {
            scale = scale.max((entry.translation - origin).norm());
        }
        for (_, joint) in &joints {
            for attachment in [&joint.first, &joint.second] {
                let entry = &entries[lookup[&attachment.body]];
                let world = entry.rotation * attachment.frame.origin.coords + entry.translation;
                scale = scale.max((world - origin).norm());
            }
        }
        if scale == 0. || !scale.is_finite() {
            scale = 1.;
        }
        let mut initial = DVector::zeros(columns);
        for entry in &entries {
            if let Some(column) = entry.column {
                let normalized = (entry.translation - origin) / scale;
                initial.rows_mut(column, 3).copy_from(&normalized);
            }
        }
        Self {
            entries,
            lookup,
            joints,
            targets,
            origin,
            scale,
            initial,
        }
    }
}

/// A pose as its rotation matrix and its translation.
pub(super) fn parts(pose: Rigid) -> (Matrix3<f64>, Vector3<f64>) {
    let rotation = Matrix3::from_columns(&[
        pose.apply_vector(Vector3::x()),
        pose.apply_vector(Vector3::y()),
        pose.apply_vector(Vector3::z()),
    ]);
    (rotation, pose.apply(Point3::origin()).coords)
}

/// The pose with this rotation matrix and translation.
fn rigid(rotation: Matrix3<f64>, translation: Vector3<f64>) -> Rigid {
    let to = Frame {
        origin: Point3::from(translation),
        x_dir: Unit::new_normalize(rotation.column(0).into_owned()),
        y_dir: Unit::new_normalize(rotation.column(1).into_owned()),
        z_dir: Unit::new_normalize(rotation.column(2).into_owned()),
    };
    Rigid::between_frames(&Frame::xyz(), &to)
}
