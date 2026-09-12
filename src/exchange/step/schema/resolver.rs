//! Resolving references, and walking a record's attributes in order.
//!
//! [`Attributes`] is a borrowed [`Record`] that remembers where in the file it
//! came from and how far a read has walked it. It knows nothing about what
//! entity it holds — that is [`entities`](super::entities)' job — so its
//! accessors answer "the next attribute, as a reference" rather than "the edge
//! geometry".
//!
//! Everything here carries an [`Origin`], because a STEP error a user cannot
//! find in their file is not actionable.

use std::fmt;
use std::ops::Deref;

use thiserror::Error;

use super::super::part21::{EntityId, Instance, Record, StepExchange, Value};
use super::entities::{Entity, Measure};
use super::units::{Units, read_units};

/// Where in a file something was found.
///
/// The instance name and the line travel together because neither locates
/// anything alone: `#1234` is what a user searches for, and the line is what
/// their editor jumps to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Origin {
    /// The instance name.
    pub id: EntityId,
    /// The 1-based source line the instance starts on.
    pub line: u32,
}

impl Origin {
    /// Where `instance` is, for an error that has to name a position.
    pub fn of(instance: &Instance) -> Self {
        Self {
            id: instance.id,
            line: instance.line,
        }
    }
}

impl fmt::Display for Origin {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{} on line {}", self.id, self.line)
    }
}

/// The file said something the schema does not allow, or nothing at all.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SchemaError {
    /// A reference resolving to nothing.
    #[error("{from} refers to {to}, which is not defined")]
    DanglingReference {
        /// The instance holding the reference.
        from: Origin,
        /// The name referred to.
        to: EntityId,
    },

    /// An entity of the wrong kind where a specific one was required.
    #[error("{origin} is a {found}, but {expected} was expected")]
    WrongEntity {
        /// Where it was.
        origin: Origin,
        /// What was needed, as a list when several spellings are accepted.
        expected: String,
        /// What was there, as the file spells it.
        found: String,
    },

    /// A complex instance where a simple one was required.
    #[error("{origin} is a complex instance of {records} records")]
    ComplexInstance {
        /// Where it was.
        origin: Origin,
        /// How many records it carries.
        records: usize,
    },

    /// An attribute past the end of the record, or holding the wrong type.
    #[error("{origin}: {keyword} attribute {index} is not {expected}")]
    BadAttribute {
        /// Where it was.
        origin: Origin,
        /// The record's keyword.
        keyword: String,
        /// The 0-based attribute position.
        index: usize,
        /// What was needed there.
        expected: &'static str,
    },

    /// A unit the exchange layer cannot convert to millimetres or radians.
    #[error("{origin}: {detail}")]
    UnreadableUnit {
        /// Where it was.
        origin: Origin,
        /// What about it could not be read.
        detail: String,
    },
}

/// One record, where it came from, and how far a read has walked it.
///
/// Copy, so decoding an entity takes its own cursor by value and no caller
/// has to thread a `&mut` or reset anything.
#[derive(Debug, Clone, Copy)]
pub struct Attributes<'a> {
    /// Where in the file this record is.
    pub origin: Origin,
    record: &'a Record,
    next: usize,
}

impl<'a> Attributes<'a> {
    /// Points a fresh cursor at `record`, said to have come from `origin`.
    ///
    /// A [`Resolver`] builds these while following references. Constructing
    /// one directly is for reading a record that was never in a file — an
    /// entity's own output, checked against its input.
    pub fn new(origin: Origin, record: &'a Record) -> Self {
        Self {
            origin,
            record,
            next: 0,
        }
    }

    /// Returns the keyword this record carries, as the file spells it.
    pub fn keyword(&self) -> &'a str {
        &self.record.keyword
    }

    /// Reports whether this record carries `keyword`, ignoring ASCII case.
    pub fn is(&self, keyword: &str) -> bool {
        self.record.is(keyword)
    }

    /// Reports whether this record is one `T` can be decoded from.
    pub fn holds<T: Entity>(&self) -> bool {
        T::KEYWORDS.iter().any(|keyword| self.is(keyword))
    }

    /// Decodes this record as `T`, or declines a record that is not one.
    ///
    /// `None` means the keyword is not `T`'s and the caller should try the
    /// next candidate; `Some(Err(..))` means it *is* `T` and is malformed.
    /// That is the crate's analytic-first dispatch convention, and it is what
    /// lets a surface reader try plane, then cylinder, then cone in turn.
    pub fn decode<T: Entity>(&self) -> Option<Result<T, SchemaError>> {
        self.holds::<T>().then(|| T::read(self.rewound()))
    }

    /// Returns this record with its cursor back at the first attribute.
    pub fn rewound(&self) -> Self {
        Self { next: 0, ..*self }
    }

    /// Consumes the decorative name every geometric entity carries.
    ///
    /// Its content is not returned: NGK writes `''` and reads nothing from it.
    /// This exists so that a read walks *every* attribute in order and never
    /// counts a skip.
    pub fn name(&mut self) -> Result<(), SchemaError> {
        self.take("a name")?;
        Ok(())
    }

    /// Consumes an attribute the schema redeclares and derives, written `*`.
    pub fn derived(&mut self) -> Result<(), SchemaError> {
        self.take("a derived attribute")?;
        Ok(())
    }

    /// Reads the next attribute as a reference to another instance.
    pub fn reference(&mut self) -> Result<EntityId, SchemaError> {
        let index = self.next;
        self.take("a reference")?
            .as_reference()
            .ok_or_else(|| self.bad(index, "a reference"))
    }

    /// Reads the next attribute as a reference, or `None` when it is unset.
    ///
    /// An attribute past the end of the record also reads as `None`, so a
    /// writer that truncates its trailing optionals still parses.
    pub fn optional_reference(&mut self) -> Result<Option<EntityId>, SchemaError> {
        let index = self.next;
        match self.take_optional() {
            None | Some(Value::Null) | Some(Value::Derived) => Ok(None),
            Some(Value::Ref(id)) => Ok(Some(*id)),
            Some(_) => Err(self.bad(index, "a reference or $")),
        }
    }

    /// Reads the next attribute as a real.
    ///
    /// An integer is accepted where a real is wanted: Part 21 distinguishes
    /// them by spelling, and a writer emitting `1` for a magnitude means the
    /// number rather than a change of type.
    pub fn real(&mut self) -> Result<f64, SchemaError> {
        let index = self.next;
        numeric(self.take("a real")?).ok_or_else(|| self.bad(index, "a real"))
    }

    /// Reads the next attribute as an integer.
    pub fn integer(&mut self) -> Result<i64, SchemaError> {
        let index = self.next;
        self.take("an integer")?
            .as_integer()
            .ok_or_else(|| self.bad(index, "an integer"))
    }

    /// Reads the next attribute as a string, decoded from its escapes.
    pub fn text(&mut self) -> Result<String, SchemaError> {
        let index = self.next;
        self.take("a string")?
            .as_text()
            .map(str::to_string)
            .ok_or_else(|| self.bad(index, "a string"))
    }

    /// Reads the next attribute as an enumeration name, without its dots.
    pub fn enumeration(&mut self) -> Result<&'a str, SchemaError> {
        let index = self.next;
        self.take("an enumeration")?
            .as_enum()
            .ok_or_else(|| self.bad(index, "an enumeration"))
    }

    /// Reads the next attribute as an enumeration, or `None` when it is `$`.
    pub fn optional_enumeration(&mut self) -> Result<Option<&'a str>, SchemaError> {
        let index = self.next;
        match self.take_optional() {
            None | Some(Value::Null) | Some(Value::Derived) => Ok(None),
            Some(Value::Enum(name)) => Ok(Some(name.as_str())),
            Some(_) => Err(self.bad(index, "an enumeration or $")),
        }
    }

    /// Reads the next attribute as a `.T.` / `.F.` boolean.
    pub fn boolean(&mut self) -> Result<bool, SchemaError> {
        let index = self.next;
        let name = self.enumeration()?;
        if name.eq_ignore_ascii_case("T") {
            Ok(true)
        } else if name.eq_ignore_ascii_case("F") {
            Ok(false)
        } else {
            Err(self.bad(index, ".T. or .F."))
        }
    }

    /// Reads the next attribute as an aggregate.
    pub fn list(&mut self) -> Result<&'a [Value], SchemaError> {
        let index = self.next;
        self.take("a list")?
            .as_list()
            .ok_or_else(|| self.bad(index, "a list"))
    }

    /// Reads the next attribute as a list of references.
    pub fn references(&mut self) -> Result<Vec<EntityId>, SchemaError> {
        let index = self.next;
        self.list()?
            .iter()
            .map(|value| {
                value
                    .as_reference()
                    .ok_or_else(|| self.bad(index, "a list of references"))
            })
            .collect()
    }

    /// Reads the next attribute as a list of integers of any length.
    ///
    /// For a count per element, such as a knot's multiplicity, where the
    /// length is whatever the entity it belongs to says it is.
    pub fn integers(&mut self) -> Result<Vec<i64>, SchemaError> {
        let index = self.next;
        self.list()?
            .iter()
            .map(|value| {
                value
                    .as_integer()
                    .ok_or_else(|| self.bad(index, "a list of integers"))
            })
            .collect()
    }

    /// Reads the next attribute as a list of reals of any length.
    ///
    /// Unlike [`Self::reals`] there is no arity to check: a knot vector or a
    /// weight list is as long as its entity's other attributes make it, which
    /// is a cross-attribute agreement the entity checks rather than the cursor.
    pub fn real_list(&mut self) -> Result<Vec<f64>, SchemaError> {
        let index = self.next;
        self.list()?
            .iter()
            .map(|value| numeric(value).ok_or_else(|| self.bad(index, "a list of reals")))
            .collect()
    }

    /// Reads the next attribute as a list of reals of exactly `N` elements.
    ///
    /// Checking the arity here is what stops a 2D entity reaching a 3D reader
    /// unnoticed.
    pub fn reals<const N: usize>(&mut self) -> Result<[f64; N], SchemaError> {
        let index = self.next;
        let values = self.list()?;
        if values.len() != N {
            return Err(self.bad(index, "a list of the expected length"));
        }
        let mut coordinates = [0.0; N];
        for (slot, value) in coordinates.iter_mut().zip(values) {
            *slot = numeric(value).ok_or_else(|| self.bad(index, "a list of reals"))?;
        }
        Ok(coordinates)
    }

    /// Reads the next attribute as a typed quantity such as
    /// `LENGTH_MEASURE(25.4)`.
    pub fn measure(&mut self) -> Result<Measure, SchemaError> {
        let index = self.next;
        let record = self
            .take("a measure")?
            .as_typed()
            .ok_or_else(|| self.bad(index, "a measure"))?;
        let value = record
            .param(0)
            .and_then(numeric)
            .ok_or_else(|| self.bad(index, "a measure carrying a number"))?;
        Ok(Measure {
            kind: record.keyword.clone(),
            value,
        })
    }

    /// Advances past the next attribute, or fails naming what was wanted.
    fn take(&mut self, expected: &'static str) -> Result<&'a Value, SchemaError> {
        let index = self.next;
        let value = self
            .record
            .param(index)
            .ok_or_else(|| self.bad(index, expected))?;
        self.next += 1;
        Ok(value)
    }

    /// Advances past the next attribute, tolerating the end of the record.
    fn take_optional(&mut self) -> Option<&'a Value> {
        let value = self.record.param(self.next);
        self.next += 1;
        value
    }

    fn bad(&self, index: usize, expected: &'static str) -> SchemaError {
        SchemaError::BadAttribute {
            origin: self.origin,
            keyword: self.record.keyword.clone(),
            index,
            expected,
        }
    }
}

/// Something read out of the file, and where it was read from.
///
/// An entity forgets its own position as soon as it is decoded — the schema
/// says nothing about instance names — but a report entry or an error raised
/// *after* the decode still has to name it, so the two are kept together.
/// Dereferences to the entity, so `face.same_sense` reads through.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Located<T> {
    /// Where in the file it was.
    pub origin: Origin,
    /// What was there.
    pub entity: T,
}

impl<T> Deref for Located<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.entity
    }
}

fn numeric(value: &Value) -> Option<f64> {
    value
        .as_real()
        .or_else(|| value.as_integer().map(|integer| integer as f64))
}

/// An instance table plus the units it is expressed in.
///
/// Everything above this reaches the file through it. The unit scales are
/// read once, here, so that a length reaches [`convert`] already in
/// millimetres and an angle already in radians, and no NGK type ever holds an
/// inch or a degree.
///
/// [`convert`]: super::super::convert
#[derive(Debug, Clone, Copy)]
pub struct Resolver<'a> {
    exchange: &'a StepExchange,
    units: Units,
}

impl<'a> Resolver<'a> {
    /// Reads the document's unit block and prepares to resolve against it.
    ///
    /// `uncertainty` overrides what the file declares, for a caller that knows
    /// its own accuracy budget better than the writer did.
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
    pub fn instance(&self, from: Origin, id: EntityId) -> Result<&'a Instance, SchemaError> {
        self.exchange
            .get(id)
            .ok_or(SchemaError::DanglingReference { from, to: id })
    }

    /// Follows a reference and decodes what it points at as `T`.
    pub fn read<T: Entity>(&self, from: Origin, id: EntityId) -> Result<Located<T>, SchemaError> {
        self.decode(self.instance(from, id)?)
    }

    /// Decodes an instance the caller already has, such as one swept for by
    /// keyword rather than followed from a reference.
    pub fn decode<T: Entity>(&self, instance: &'a Instance) -> Result<Located<T>, SchemaError> {
        let attributes = self.select::<T>(instance)?;
        Ok(Located {
            origin: attributes.origin,
            entity: T::read(attributes)?,
        })
    }

    /// Follows a reference without deciding what it is.
    ///
    /// For declining dispatch, which has to look at the keyword before it
    /// knows which entity to decode into.
    pub fn attributes(&self, from: Origin, id: EntityId) -> Result<Attributes<'a>, SchemaError> {
        self.simple(self.instance(from, id)?)
    }

    /// Returns an instance's sole record, refusing a complex instance.
    pub fn simple(&self, instance: &'a Instance) -> Result<Attributes<'a>, SchemaError> {
        let origin = origin_of(instance);
        instance
            .simple()
            .map(|record| Attributes::new(origin, record))
            .ok_or(SchemaError::ComplexInstance {
                origin,
                records: instance.records.len(),
            })
    }

    /// Returns the record of `instance` that `T` can be decoded from.
    ///
    /// Searches every record, so a complex instance — which is what AP214
    /// uses for the unit block and the rational B-spline forms — is reached
    /// the same way a simple one is.
    fn select<T: Entity>(&self, instance: &'a Instance) -> Result<Attributes<'a>, SchemaError> {
        let origin = origin_of(instance);
        T::KEYWORDS
            .iter()
            .find_map(|keyword| instance.record(keyword))
            .map(|record| Attributes::new(origin, record))
            .ok_or_else(|| SchemaError::WrongEntity {
                origin,
                expected: expected_name::<T>(),
                found: found_name(instance),
            })
    }
}

fn origin_of(instance: &Instance) -> Origin {
    Origin::of(instance)
}

/// Names the keywords an entity accepts, for a "wrong entity" message.
fn expected_name<T: Entity>() -> String {
    match T::KEYWORDS {
        [only] => (*only).to_string(),
        many => format!("one of {}", many.join(", ")),
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
