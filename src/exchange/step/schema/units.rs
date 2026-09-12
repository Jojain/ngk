//! The document's unit block, and what it converts to.
//!
//! STEP states its own units: metres with an SI prefix, or an inch defined as
//! a conversion from millimetres; radians, or degrees defined the same way.
//! NGK holds no units at all — a `Point3` is a length in whatever the caller
//! meant — so the exchange layer picks one and converts at the boundary (D10).
//!
//! The one picked is **millimetres and radians**, matching what NGK's own
//! models and its reference kernel both use, and what
//! [`write_context`](super::product::write_context) declares on the way out.
//!
//! A file with no unit block at all is read as millimetres rather than
//! refused: product structure is where vendor files diverge most, and a
//! missing context should not cost the geometry.

use super::super::part21::{EntityId, Instance, Record, StepExchange, Value};
use super::resolver::SchemaError;

/// What one document's numbers mean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Units {
    /// Multiply a file length by this to get millimetres.
    pub length: f64,
    /// Multiply a file plane angle by this to get radians.
    pub angle: f64,
    /// The document's declared position tolerance, in millimetres.
    ///
    /// What the writing kernel means by "these two positions are the same".
    /// NGK's own `LINEAR_TOLERANCE` is three orders tighter than the `1e-6` to
    /// `1e-7` such files carry, so this — not the global constant — is what
    /// vertex merging and edge stitching are measured against.
    pub uncertainty: f64,
}

impl Default for Units {
    /// Millimetres, radians, and the tolerance files in the wild declare.
    fn default() -> Self {
        Self {
            length: 1.0,
            angle: 1.0,
            uncertainty: 1.0e-7,
        }
    }
}

impl Units {
    /// Converts a file length to millimetres.
    pub fn to_mm(&self, length: f64) -> f64 {
        length * self.length
    }

    /// Converts a file plane angle to radians.
    pub fn to_radians(&self, angle: f64) -> f64 {
        angle * self.angle
    }
}

/// Reads the 3D representation context's units and uncertainty.
///
/// The context is found by sweeping for it rather than by walking down from
/// `SHAPE_DEFINITION_REPRESENTATION`, for the same reason the B-Rep roots are
/// (§5): the path down differs between vendors and none of it is needed.
pub fn read_units(exchange: &StepExchange) -> Result<Units, SchemaError> {
    let Some(instance) = geometric_context(exchange) else {
        return Ok(Units::default());
    };

    let mut units = Units::default();
    if let Some(assigned) = instance.record("GLOBAL_UNIT_ASSIGNED_CONTEXT") {
        for id in reference_list(assigned, 0) {
            let Some(unit) = exchange.get(id) else {
                continue;
            };
            if unit.is("LENGTH_UNIT") {
                units.length = length_scale(exchange, unit)?;
            } else if unit.is("PLANE_ANGLE_UNIT") {
                units.angle = angle_scale(exchange, unit)?;
            }
        }
    }

    if let Some(assigned) = instance.record("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT")
        && let Some(id) = reference_list(assigned, 0).into_iter().next()
        && let Some(measure) = exchange.get(id)
        && let Some(record) = measure.record("UNCERTAINTY_MEASURE_WITH_UNIT")
        && let Some(value) = typed_value(record.param(0))
    {
        units.uncertainty = value * units.length;
    }

    Ok(units)
}

/// Finds the 3D geometric representation context, if the file declares one.
///
/// A file may carry several — every `PCURVE`'s definitional representation
/// sits in a 2D one — so the dimension count is what tells them apart.
fn geometric_context(exchange: &StepExchange) -> Option<&Instance> {
    exchange
        .instances_of("GEOMETRIC_REPRESENTATION_CONTEXT")
        .find(|instance| {
            instance
                .record("GEOMETRIC_REPRESENTATION_CONTEXT")
                .and_then(|record| record.param(0))
                .and_then(Value::as_integer)
                == Some(3)
        })
}

/// Returns how many millimetres one of this unit is.
fn length_scale(exchange: &StepExchange, unit: &Instance) -> Result<f64, SchemaError> {
    if let Some(si) = unit.record("SI_UNIT") {
        // The base SI length is the metre, which is a thousand millimetres.
        let name = enumeration(si.param(1));
        if !name.is_some_and(|name| name.eq_ignore_ascii_case("METRE")) {
            return Err(unreadable(unit, format!("{:?} is not a length", name)));
        }
        return Ok(1000.0 * prefix_factor(enumeration(si.param(0)))?);
    }
    conversion_scale(exchange, unit)
}

/// Returns how many radians one of this unit is.
fn angle_scale(exchange: &StepExchange, unit: &Instance) -> Result<f64, SchemaError> {
    if let Some(si) = unit.record("SI_UNIT") {
        let name = enumeration(si.param(1));
        if !name.is_some_and(|name| name.eq_ignore_ascii_case("RADIAN")) {
            return Err(unreadable(unit, format!("{:?} is not a plane angle", name)));
        }
        return prefix_factor(enumeration(si.param(0)));
    }
    conversion_scale(exchange, unit)
}

/// Resolves a `CONVERSION_BASED_UNIT` — an inch, or a degree — to its base.
///
/// The conversion factor is a `MEASURE_WITH_UNIT` naming both a number and the
/// unit that number is in, so this recurses one level: a degree is `0.01745…`
/// *radians*, and the radian's own scale still has to be applied.
fn conversion_scale(exchange: &StepExchange, unit: &Instance) -> Result<f64, SchemaError> {
    let converted = unit.record("CONVERSION_BASED_UNIT").ok_or_else(|| {
        unreadable(
            unit,
            "neither an SI nor a conversion-based unit".to_string(),
        )
    })?;
    let factor_id = converted
        .param(1)
        .and_then(Value::as_reference)
        .ok_or_else(|| unreadable(unit, "conversion factor is not a reference".to_string()))?;
    let factor = exchange
        .get(factor_id)
        .and_then(|instance| instance.record("MEASURE_WITH_UNIT"))
        .ok_or_else(|| unreadable(unit, "conversion factor is not a measure".to_string()))?;

    let value = typed_value(factor.param(0))
        .ok_or_else(|| unreadable(unit, "conversion factor carries no number".to_string()))?;
    let base_id = factor
        .param(1)
        .and_then(Value::as_reference)
        .ok_or_else(|| unreadable(unit, "conversion factor names no unit".to_string()))?;
    let base = exchange
        .get(base_id)
        .ok_or_else(|| unreadable(unit, format!("conversion factor names {base_id}")))?;

    let base_scale = if base.is("PLANE_ANGLE_UNIT") {
        angle_scale(exchange, base)?
    } else {
        length_scale(exchange, base)?
    };
    Ok(value * base_scale)
}

/// Returns the multiplier an `SI_UNIT` prefix stands for.
fn prefix_factor(prefix: Option<&str>) -> Result<f64, SchemaError> {
    let Some(prefix) = prefix else {
        return Ok(1.0);
    };
    let factor = match prefix.to_ascii_uppercase().as_str() {
        "EXA" => 1.0e18,
        "PETA" => 1.0e15,
        "TERA" => 1.0e12,
        "GIGA" => 1.0e9,
        "MEGA" => 1.0e6,
        "KILO" => 1.0e3,
        "HECTO" => 1.0e2,
        "DECA" => 1.0e1,
        "DECI" => 1.0e-1,
        "CENTI" => 1.0e-2,
        "MILLI" => 1.0e-3,
        "MICRO" => 1.0e-6,
        "NANO" => 1.0e-9,
        "PICO" => 1.0e-12,
        "FEMTO" => 1.0e-15,
        "ATTO" => 1.0e-18,
        _ => return Ok(1.0),
    };
    Ok(factor)
}

/// Reads the number out of a `LENGTH_MEASURE(1.)`-style typed parameter.
fn typed_value(param: Option<&Value>) -> Option<f64> {
    let record = param?.as_typed()?;
    let value = record.param(0)?;
    value
        .as_real()
        .or_else(|| value.as_integer().map(|integer| integer as f64))
}

fn enumeration(param: Option<&Value>) -> Option<&str> {
    param?.as_enum()
}

fn reference_list(record: &Record, index: usize) -> Vec<EntityId> {
    record
        .param(index)
        .and_then(Value::as_list)
        .map(|values| values.iter().filter_map(Value::as_reference).collect())
        .unwrap_or_default()
}

fn unreadable(unit: &Instance, detail: String) -> SchemaError {
    SchemaError::UnreadableUnit {
        id: unit.id,
        line: unit.line,
        detail,
    }
}
