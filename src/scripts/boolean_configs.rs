use nalgebra::Vector3;

use crate::geometry::{Plane, Point3};
use crate::modeling::boolean::{
    BooleanOperation, boolean_difference, boolean_intersection, boolean_union,
};
use crate::modeling::{faces, solids, sweep};
use crate::topology::payload::StandardPayload;
use crate::topology::shape::{Shape, SolidTag};
use crate::viz::{ScriptResult, Style, VizHints};

const OBJECT_SIZE: f64 = 1.5;
const TOOL_SIZE: f64 = 1.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanConfig {
    UnionOverlap,
    IntersectionOverlap,
    DifferenceOverlap,
}

impl BooleanConfig {
    pub fn from_id(id: &str) -> Result<Self, String> {
        match id {
            "union-overlap" => Ok(Self::UnionOverlap),
            "intersection-overlap" => Ok(Self::IntersectionOverlap),
            "difference-overlap" => Ok(Self::DifferenceOverlap),
            _ => Err(format!("unknown boolean configuration: {id}")),
        }
    }

    fn operation(self) -> BooleanOperation {
        match self {
            Self::UnionOverlap => BooleanOperation::Union,
            Self::IntersectionOverlap => BooleanOperation::Intersection,
            Self::DifferenceOverlap => BooleanOperation::Difference,
        }
    }

    fn tool_origin(self) -> Point3 {
        match self {
            Self::UnionOverlap | Self::IntersectionOverlap | Self::DifferenceOverlap => {
                Point3::new(0.65, 0.0, 0.0)
            }
        }
    }

    fn result_color(self) -> &'static str {
        match self {
            Self::UnionOverlap => "#56b870",
            Self::IntersectionOverlap => "#5aa9e6",
            Self::DifferenceOverlap => "#e6864a",
        }
    }
}

pub fn build_by_id(id: &str) -> Result<ScriptResult, String> {
    build(BooleanConfig::from_id(id)?)
}

pub fn build(config: BooleanConfig) -> Result<ScriptResult, String> {
    let object = solids::block(OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE)
        .map_err(|err| format!("failed to build boolean object block: {err:?}"))?;
    let tool = shifted_block(config.tool_origin(), TOOL_SIZE, TOOL_SIZE, TOOL_SIZE)?;
    let result = match config.operation() {
        BooleanOperation::Union => boolean_union(&object, &tool),
        BooleanOperation::Intersection => boolean_intersection(&object, &tool),
        BooleanOperation::Difference => boolean_difference(&object, &tool),
    }
    .map_err(|err| format!("failed to build boolean result: {err:?}"))?;

    let mut hints = VizHints::new();
    for (key, _) in result.map().iter_faces() {
        hints.face(
            key,
            Style::default()
                .color(config.result_color())
                .opacity(0.82)
                .double_sided(false),
        );
    }

    Ok(ScriptResult::from_gmap_with_hints(result.map(), &hints))
}

pub fn run() -> Result<ScriptResult, String> {
    build(BooleanConfig::UnionOverlap)
}

fn shifted_block(
    origin: Point3,
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> Result<Shape<SolidTag, StandardPayload>, String> {
    let base = faces::rectangle(
        Plane::from_xy(origin, Vector3::x(), Vector3::y()),
        x_size,
        y_size,
    )
    .map_err(|err| format!("failed to build boolean tool base face: {err:?}"))?;

    sweep::extrude_face(base, Vector3::new(0.0, 0.0, z_size))
        .map_err(|err| format!("failed to extrude boolean tool block: {err:?}"))
}
