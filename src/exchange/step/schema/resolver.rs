//! Resolving references and reading attributes by position.
//!
//! Every read of a STEP file is "follow this `#N`, check it is the entity I
//! expect, and take its third attribute as a real". Doing that inline turns
//! each mapping function into a pile of `Option` handling that loses the one
//! thing a user needs — *which* entity was wrong, and on which line. This
//! module does it once.

use thiserror::Error;

use super::super::part21::{EntityId, Instance, Record, StepExchange, Value};
use super::units::{Units, read_units};

/// The file said something the schema does not allow, or nothing at all.
///
/// Every variant names the entity and the source line it was found at — a
/// STEP error a user cannot locate in their file is not actionable (§7).
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SchemaError {
    /// A reference resolving to nothing.
    #[error("{from} on line {line} refers to {to}, which is not defined")]
    DanglingReference {
        /// The instance holding the reference.
        from: EntityId,
        /// The name referred to.
        to: EntityId,
        /// The line `from` starts on.
        line: u32,
    },

    /// An entity of the wrong kind where a specific one was required.
    #[error("{id} on line {line} is a {found}, but a {expected} was expected")]
    WrongEntity {
        /// The instance met.
        id: EntityId,
        /// Its line.
        line: u32,
        /// What was needed.
        expected: &'static str,
        /// What was there, as the file spells it.
        found: String,
    },

    /// A complex instance where a simple one was required.
    ///
    /// A complex instance is read through its keyword, so reaching one by
    /// position means the caller expected a plain entity.
    #[error("{id} on line {line} is a complex instance of {records} records")]
    ComplexInstance {
        /// The instance met.
        id: EntityId,
        /// Its line.
        line: u32,
        /// How many records it carries.
        records: usize,
    },

    /// An attribute past the end of the record, or holding the wrong type.
    #[error("{id} on line {line}: {keyword} attribute {index} is not {expected}")]
    BadAttribute {
        /// The instance met.
        id: EntityId,
        /// Its line.
        line: u32,
        /// The record's keyword.
        keyword: String,
        /// The 0-based attribute position.
        index: usize,
        /// What was needed there.
        expected: &'static str,
    },

    /// A unit the exchange layer cannot convert to millimetres or radians.
    #[error("{id} on line {line}: {detail}")]
    UnreadableUnit {
        /// The unit instance met.
        id: EntityId,
        /// Its line.
        line: u32,
        /// What about it could not be read.
        detail: String,
    },
}

/// One record, and where in the file it came from.
///
/// Carried together so that reading an attribute can fail with a message
/// naming a position, rather than the caller having to re-find it.
#[derive(Debug, Clone, Copy)]
pub struct Entity<'a> {
    /// The instance this record belongs to.
    pub id: EntityId,
    /// The 1-based line that instance starts on.
    pub line: u32,
    record: &'a Record,
}

impl<'a> Entity<'a> {
    /// Returns the keyword this record carries, as the file spells it.
    pub fn keyword(&self) -> &'a str {
        &self.record.keyword
    }

    /// Reports whether this record carries `keyword`, ignoring ASCII case.
    pub fn is(&self, keyword: &str) -> bool {
        self.record.is(keyword)
    }

    /// Returns this record's attributes, in schema order.
    pub fn params(&self) -> &'a [Value] {
        &self.record.params
    }

    fn param(&self, index: usize, expected: &'static str) -> Result<&'a Value, SchemaError> {
        self.record
            .param(index)
            .ok_or_else(|| self.bad(index, expected))
    }

    fn bad(&self, index: usize, expected: &'static str) -> SchemaError {
        SchemaError::BadAttribute {
            id: self.id,
            line: self.line,
            keyword: self.record.keyword.clone(),
            index,
            expected,
        }
    }

    /// Reads attribute `index` as a reference to another instance.
    pub fn reference(&self, index: usize) -> Result<EntityId, SchemaError> {
        self.param(index, "a reference")?
            .as_reference()
            .ok_or_else(|| self.bad(index, "a reference"))
    }

    /// Reads attribute `index` as a reference, or `None` when it is unset.
    pub fn optional_reference(&self, index: usize) -> Result<Option<EntityId>, SchemaError> {
        match self.record.param(index) {
            None | Some(Value::Null) | Some(Value::Derived) => Ok(None),
            Some(Value::Ref(id)) => Ok(Some(*id)),
            Some(_) => Err(self.bad(index, "a reference or $")),
        }
    }

    /// Reads attribute `index` as a real.
    ///
    /// An integer is accepted where a real is wanted: Part 21 distinguishes
    /// them by spelling, and a vendor writing `1` for a magnitude means the
    /// number rather than a change of type.
    pub fn real(&self, index: usize) -> Result<f64, SchemaError> {
        numeric(self.param(index, "a real")?).ok_or_else(|| self.bad(index, "a real"))
    }

    /// Reads attribute `index` as an integer.
    pub fn integer(&self, index: usize) -> Result<i64, SchemaError> {
        self.param(index, "an integer")?
            .as_integer()
            .ok_or_else(|| self.bad(index, "an integer"))
    }

    /// Reads attribute `index` as an enumeration name, without its dots.
    pub fn enumeration(&self, index: usize) -> Result<&'a str, SchemaError> {
        self.param(index, "an enumeration")?
            .as_enum()
            .ok_or_else(|| self.bad(index, "an enumeration"))
    }

    /// Reads attribute `index` as a `.T.` / `.F.` boolean.
    pub fn boolean(&self, index: usize) -> Result<bool, SchemaError> {
        let name = self.enumeration(index)?;
        if name.eq_ignore_ascii_case("T") {
            Ok(true)
        } else if name.eq_ignore_ascii_case("F") {
            Ok(false)
        } else {
            Err(self.bad(index, ".T. or .F."))
        }
    }

    /// Reads attribute `index` as an aggregate.
    pub fn list(&self, index: usize) -> Result<&'a [Value], SchemaError> {
        self.param(index, "a list")?
            .as_list()
            .ok_or_else(|| self.bad(index, "a list"))
    }

    /// Reads attribute `index` as a list of references.
    pub fn references(&self, index: usize) -> Result<Vec<EntityId>, SchemaError> {
        self.list(index)?
            .iter()
            .map(|value| {
                value
                    .as_reference()
                    .ok_or_else(|| self.bad(index, "a list of references"))
            })
            .collect()
    }

    /// Reads attribute `index` as a list of reals of exactly `N` elements.
    ///
    /// This is what a `CARTESIAN_POINT`'s coordinates and a `DIRECTION`'s
    /// ratios are, and checking the arity here is what stops a 2D entity
    /// reaching a 3D reader unnoticed.
    pub fn reals<const N: usize>(&self, index: usize) -> Result<[f64; N], SchemaError> {
        let values = self.list(index)?;
        if values.len() != N {
            return Err(self.bad(index, "a list of the expected length"));
        }
        let mut coordinates = [0.0; N];
        for (slot, value) in coordinates.iter_mut().zip(values) {
            *slot = numeric(value).ok_or_else(|| self.bad(index, "a list of reals"))?;
        }
        Ok(coordinates)
    }

    /// Reads attribute `index` as a typed parameter such as `LENGTH_MEASURE(1.)`.
    pub fn typed(&self, index: usize) -> Result<Entity<'a>, SchemaError> {
        let record = self
            .param(index, "a typed parameter")?
            .as_typed()
            .ok_or_else(|| self.bad(index, "a typed parameter"))?;
        Ok(Entity {
            id: self.id,
            line: self.line,
            record,
        })
    }
}

fn numeric(value: &Value) -> Option<f64> {
    value
        .as_real()
        .or_else(|| value.as_integer().map(|integer| integer as f64))
}

/// An instance table plus the units it is expressed in.
///
/// Everything above L2 reaches the file through this. Normalization happens
/// here rather than further up (D10): the scales are read once, so a length
/// reaches [`convert`](super::super::convert) already in millimetres and an
/// angle already in radians, and no NGK type ever holds an inch or a degree.
#[derive(Debug, Clone, Copy)]
pub struct Resolver<'a> {
    exchange: &'a StepExchange,
    units: Units,
}

impl<'a> Resolver<'a> {
    /// Reads the document's unit block and prepares to resolve against it.
    ///
    /// `uncertainty` overrides what the file declares, for a caller that knows
    /// its own accuracy budget better than the writer did (D10).
    pub fn new(exchange: &'a StepExchange, uncertainty: Option<f64>) -> Result<Self, SchemaError> {
        let mut units = read_units(exchange)?;
        if let Some(override_value) = uncertainty {
            units.uncertainty = override_value;
        }
        Ok(Self { exchange, units })
    }

    /// Returns the units this document is expressed in.
    pub fn units(&self) -> Units {
        self.units
    }

    /// Returns the exchange structure being read.
    pub fn exchange(&self) -> &'a StepExchange {
        self.exchange
    }

    /// Resolves a reference, naming the instance that dangles.
    pub fn instance(&self, from: &Entity<'_>, id: EntityId) -> Result<&'a Instance, SchemaError> {
        self.exchange.get(id).ok_or(SchemaError::DanglingReference {
            from: from.id,
            to: id,
            line: from.line,
        })
    }

    /// Returns an instance's sole record, refusing a complex instance.
    pub fn entity(&self, instance: &'a Instance) -> Result<Entity<'a>, SchemaError> {
        instance
            .simple()
            .map(|record| Entity {
                id: instance.id,
                line: instance.line,
                record,
            })
            .ok_or(SchemaError::ComplexInstance {
                id: instance.id,
                line: instance.line,
                records: instance.records.len(),
            })
    }

    /// Returns an instance's record for `keyword`, simple or complex alike.
    pub fn record(
        &self,
        instance: &'a Instance,
        keyword: &'static str,
    ) -> Result<Entity<'a>, SchemaError> {
        instance
            .record(keyword)
            .map(|record| Entity {
                id: instance.id,
                line: instance.line,
                record,
            })
            .ok_or_else(|| SchemaError::WrongEntity {
                id: instance.id,
                line: instance.line,
                expected: keyword,
                found: found_name(instance),
            })
    }

    /// Follows a reference and returns the referent's sole record.
    pub fn follow(&self, from: &Entity<'_>, id: EntityId) -> Result<Entity<'a>, SchemaError> {
        self.entity(self.instance(from, id)?)
    }

    /// Follows a reference and requires the referent to carry `keyword`.
    pub fn follow_typed(
        &self,
        from: &Entity<'_>,
        id: EntityId,
        keyword: &'static str,
    ) -> Result<Entity<'a>, SchemaError> {
        self.record(self.instance(from, id)?, keyword)
    }
}

/// Names what an instance actually is, for a "wrong entity" message.
fn found_name(instance: &Instance) -> String {
    instance
        .records
        .iter()
        .map(|record| record.keyword.as_str())
        .collect::<Vec<_>>()
        .join(" ")
}
