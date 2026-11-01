//! What a Boolean reached, recorded stage by stage, for looking at a failure.
//!
//! A Boolean runs in one transaction, so a failure rolls the model back and
//! leaves nothing but the error: the sections it traced, the faces it had
//! already split and the face it could not are all gone. A trace keeps them.
//! Each stage holds what that stage produced -- a copy of the model as it then
//! stood, the curves it traced or imprinted, the points it met at, and the
//! faces it singled out -- so a failure can be looked at where it happened
//! rather than reconstructed from its message.
//!
//! Recording copies the model, so it is done only when asked for: a disabled
//! trace records nothing and costs nothing. Each copy is completed with
//! default payloads where the edit had not assigned one yet, so it can be
//! handled -- a face isolated from it, sent to a viewer -- like any committed
//! model; recording therefore asks for a payload with a default at every
//! dimension.

use crate::geometry::{Point3, TrimmedCurve};
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;

/// The stages one Boolean went through, in order.
#[derive(Clone)]
pub struct BooleanTrace<P: Payload> {
    /// Completes a recorded model's payloads, when recording at all.
    complete: Option<fn(&mut Model<P>)>,
    stages: Vec<BooleanTraceStage<P>>,
}

/// What one stage of a Boolean produced.
#[derive(Clone)]
pub struct BooleanTraceStage<P: Payload> {
    /// What the stage did, or how it failed.
    pub name: String,
    /// The model as the stage left it, when the stage changed it.
    pub model: Option<Model<P>>,
    /// Named groups of faces of `model` the stage singled out.
    pub faces: Vec<(String, Vec<FaceKey>)>,
    /// Named groups of curves the stage traced or imprinted.
    pub curves: Vec<(String, Vec<TrimmedCurve>)>,
    /// Points the stage met at.
    pub points: Vec<Point3>,
}

impl<P: Payload> BooleanTraceStage<P> {
    /// A stage with nothing recorded but its name.
    pub(crate) fn named(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            model: None,
            faces: Vec::new(),
            curves: Vec::new(),
            points: Vec::new(),
        }
    }

    /// The stage with a copy of `model` as it now stands.
    pub(crate) fn with_model(mut self, model: &Model<P>) -> Self {
        self.model = Some(model.clone());
        self
    }

    pub(crate) fn with_faces(mut self, name: impl Into<String>, faces: Vec<FaceKey>) -> Self {
        self.faces.push((name.into(), faces));
        self
    }

    pub(crate) fn with_curves(
        mut self,
        name: impl Into<String>,
        curves: Vec<TrimmedCurve>,
    ) -> Self {
        self.curves.push((name.into(), curves));
        self
    }

    pub(crate) fn with_points(mut self, points: Vec<Point3>) -> Self {
        self.points.extend(points);
        self
    }
}

impl<P: Payload> BooleanTrace<P> {
    /// A trace that records every stage.
    pub fn recording() -> Self
    where
        P::V: Default,
        P::E: Default,
        P::Profile: Default,
        P::F: Default,
        P::Sheet: Default,
        P::S: Default,
    {
        Self {
            complete: Some(Model::fill_missing_data),
            stages: Vec::new(),
        }
    }

    /// A trace that records nothing.
    pub(crate) fn disabled() -> Self {
        Self {
            complete: None,
            stages: Vec::new(),
        }
    }

    /// The stages recorded, in the order the Boolean reached them.
    pub fn stages(&self) -> &[BooleanTraceStage<P>] {
        &self.stages
    }

    /// Records the stage `build` describes, building it only when recording.
    pub(crate) fn record(&mut self, build: impl FnOnce() -> BooleanTraceStage<P>) {
        let Some(complete) = self.complete else {
            return;
        };
        let mut stage = build();
        if let Some(model) = &mut stage.model {
            complete(model);
        }
        self.stages.push(stage);
    }
}
