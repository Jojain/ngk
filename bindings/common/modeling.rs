use nalgebra::{Unit, Vector3};
use thiserror::Error;

use crate::builders::blend::{BlendSelection, BlendTarget};
use crate::builders::sweep::{SweepFrame, SweepOptions, SweepTransition};
use crate::geometry::Axis3;
use crate::topology::payload::{Payload, StandardPayload};

use super::explore::SharedModel;

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
