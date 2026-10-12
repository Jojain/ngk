use nalgebra::{Unit, Vector3};
use thiserror::Error;

use crate::builders;
use crate::builders::blend::{BlendSelection, BlendTarget};
use crate::builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, boolean};
use crate::builders::sweep::{SweepFrame, SweepOptions, SweepTransition};
use crate::geometry::Axis3;
use crate::modeling::faces::{FaceBooleanResult, copy_face};
use crate::modeling::import::import_tools;
use crate::modeling::solids::copy_solid;
use crate::topology::ModelEditError;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::shape::{Shape, SolidTag};

use super::explore::{ExploreError, SharedFace, SharedModel, SharedSolid};

#[derive(Debug, Error)]
pub(crate) enum SelectionError {
    #[error("blend selections must belong to the same model")]
    DifferentModels,
}

/// Collects stable entity keys with their owning model identity.
#[derive(Clone, Default)]
pub(crate) struct SharedBlendTarget<P: Payload = StandardPayload> {
    model: Option<SharedModel<P>>,
    target: BlendTarget,
}

impl<P: Payload> SharedBlendTarget<P> {
    pub(crate) fn new() -> Self {
        Self {
            model: None,
            target: BlendTarget::new(),
        }
    }

    pub(crate) fn add(
        &mut self,
        model: SharedModel<P>,
        selection: BlendSelection,
    ) -> Result<(), SelectionError> {
        if self
            .model
            .as_ref()
            .is_some_and(|owner| !owner.ptr_eq(&model))
        {
            return Err(SelectionError::DifferentModels);
        }
        self.model = Some(model);
        self.target = self.target.clone().with(selection);
        Ok(())
    }

    pub(crate) fn for_model(&self, model: &SharedModel<P>) -> Result<BlendTarget, SelectionError> {
        if self
            .model
            .as_ref()
            .is_some_and(|owner| !owner.ptr_eq(model))
        {
            return Err(SelectionError::DifferentModels);
        }
        Ok(self.target.clone())
    }
}

#[derive(Debug, Error)]
pub(crate) enum SweepOptionsError {
    #[error("unknown sweep frame {0}; expected parallel, frenet, or axial")]
    Frame(String),
    #[error("axial sweep frame requires an axis")]
    MissingAxis,
    #[error("an axis can only be supplied for the axial sweep frame")]
    UnexpectedAxis,
    #[error("unknown sweep transition {0}; expected smooth, straight, or rounded")]
    Transition(String),
}

/// Reads the same sweep configuration in Python and WASM.
pub(crate) fn sweep_options(
    frame: &str,
    axis: Option<Axis3>,
    transition: &str,
    samples_per_segment: usize,
) -> Result<SweepOptions, SweepOptionsError> {
    let frame = match (frame, axis) {
        ("parallel", None) => SweepFrame::Parallel,
        ("frenet", None) => SweepFrame::Frenet,
        ("axial", Some(axis)) => SweepFrame::Axial(axis),
        ("axial", None) => return Err(SweepOptionsError::MissingAxis),
        ("parallel" | "frenet", Some(_)) => return Err(SweepOptionsError::UnexpectedAxis),
        (other, _) => return Err(SweepOptionsError::Frame(other.to_owned())),
    };
    let transition = match transition {
        "smooth" => SweepTransition::Smooth,
        "straight" => SweepTransition::Straight,
        "rounded" => SweepTransition::Rounded,
        other => return Err(SweepOptionsError::Transition(other.to_owned())),
    };
    Ok(SweepOptions {
        frame,
        transition,
        samples_per_segment,
    })
}

#[derive(Debug, Error)]
pub(crate) enum BindingBooleanError {
    #[error(transparent)]
    Handle(#[from] ExploreError),
    #[error(transparent)]
    Import(#[from] ModelEditError),
    #[error(transparent)]
    Boolean(#[from] BooleanError),
}

/// Evaluates one solid Boolean for both bindings.
///
/// A binding never mutates the model its handles share, so the Boolean runs in
/// a copy of the target's model, kept whole: every other entity of it is still
/// there, under the same key. A tool held in that same model is used in place
/// by its key; any other tool contributes only its own solid, copied in.
pub(crate) fn solid_boolean<P: Payload>(
    target: &SharedSolid<P>,
    tools: &[SharedSolid<P>],
    operation: BooleanOperation,
) -> Result<Shape<SolidTag, P>, BindingBooleanError> {
    target.view()?;
    let shared = target.model();
    let mut model = shared.model().clone();
    let views = tools
        .iter()
        .map(|tool| Ok((tool, tool.view()?)))
        .collect::<Result<Vec<_>, ExploreError>>()?;
    let keys = import_tools(
        &mut model,
        &views,
        |(a, _), (b, _)| a.same_entity(b),
        |edit, (tool, view)| match tool.model().ptr_eq(&shared) {
            true => tool.key(),
            false => copy_solid(edit, view.clone()),
        },
    )?;
    let result = boolean(
        &mut model,
        target.key(),
        &keys,
        operation,
        BooleanOptions::default(),
    )?;
    Ok(Shape::new(model, result.solid))
}

/// Evaluates one face Boolean for both bindings, on the model rules of
/// [`solid_boolean`].
pub(crate) fn face_boolean<P: Payload>(
    target: &SharedFace<P>,
    tools: &[SharedFace<P>],
    operation: BooleanOperation,
) -> Result<FaceBooleanResult<P>, BindingBooleanError> {
    target.view()?;
    let shared = target.model();
    let mut model = shared.model().clone();
    let views = tools
        .iter()
        .map(|tool| Ok((tool, tool.view()?)))
        .collect::<Result<Vec<_>, ExploreError>>()?;
    let keys = import_tools(
        &mut model,
        &views,
        |(a, _), (b, _)| a.same_entity(b),
        |edit, (tool, view)| match tool.model().ptr_eq(&shared) {
            true => tool.key(),
            false => copy_face(edit, view.clone()),
        },
    )?;
    let result = builders::boolean::face_boolean(
        &mut model,
        target.key(),
        &keys,
        operation,
        BooleanOptions::default(),
    )?;
    Ok(FaceBooleanResult::new(model, result.faces))
}

#[derive(Debug, Error)]
pub(crate) enum DirectionError {
    #[error("extrusion direction must contain three finite coordinates")]
    NonFinite,
    #[error("extrusion direction must be non-zero")]
    Zero,
}

/// Validates and normalizes an extrusion direction for both bindings.
pub(crate) fn extrusion_direction(
    direction: Vector3<f64>,
) -> Result<Unit<Vector3<f64>>, DirectionError> {
    if direction.iter().any(|value| !value.is_finite()) {
        return Err(DirectionError::NonFinite);
    }
    Unit::try_new(direction, 0.0).ok_or(DirectionError::Zero)
}
