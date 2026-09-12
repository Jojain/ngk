//! The Part 21 value model: tokens, records, instances, and the instance table.
//!
//! This module is ISO 10303-**21** and nothing else. It does not know what an
//! `ADVANCED_FACE` is, which application protocol produced a file, or that a
//! geometric kernel exists. That ignorance is the point: the same table serves
//! AP203, AP214, AP242 and even IFC, and it is the only part of the STEP stack
//! a third-party crate could ever supply.

use std::collections::HashMap;
use std::fmt;

use thiserror::Error;

/// A `#N` entity instance name.
///
/// Displays with its `#`, so an error message that interpolates one is already
/// in the form a user can search their file for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EntityId(pub u64);

impl fmt::Display for EntityId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "#{}", self.0)
    }
}

/// One Part 21 parameter.
///
/// `Real` and `Integer` are genuinely distinct in the exchange structure — a
/// real always carries a `.` and an integer never does — so they stay distinct
/// here rather than collapsing into one numeric variant.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// An integer literal, written without a decimal point.
    Integer(i64),
    /// A real literal, written with a decimal point.
    Real(f64),
    /// A string literal, already decoded from its Part 21 escapes.
    Text(String),
    /// An enumeration name, stored without its enclosing dots: `.T.` is `T`.
    Enum(String),
    /// A reference to another instance in the same data section.
    Ref(EntityId),
    /// `$` — an unset optional attribute.
    Null,
    /// `*` — an attribute redeclared and derived by a subtype.
    Derived,
    /// A parenthesised aggregate: list, set, bag or array.
    List(Vec<Value>),
    /// A typed parameter such as `PARAMETER_VALUE(0.)`, used where the schema
    /// declares a `SELECT` over defined types.
    Typed(Box<Record>),
}

impl Value {
    /// Returns the integer this value holds, if it is one.
    pub fn as_integer(&self) -> Option<i64> {
        match self {
            Self::Integer(value) => Some(*value),
            _ => None,
        }
    }

    /// Returns the real this value holds, if it is one.
    pub fn as_real(&self) -> Option<f64> {
        match self {
            Self::Real(value) => Some(*value),
            _ => None,
        }
    }

    /// Returns the decoded string this value holds, if it is one.
    pub fn as_text(&self) -> Option<&str> {
        match self {
            Self::Text(text) => Some(text),
            _ => None,
        }
    }

    /// Returns the enumeration name this value holds, without its dots.
    pub fn as_enum(&self) -> Option<&str> {
        match self {
            Self::Enum(name) => Some(name),
            _ => None,
        }
    }

    /// Returns the instance this value refers to, if it is a reference.
    pub fn as_reference(&self) -> Option<EntityId> {
        match self {
            Self::Ref(id) => Some(*id),
            _ => None,
        }
    }

    /// Returns the elements of this aggregate, if it is one.
    pub fn as_list(&self) -> Option<&[Value]> {
        match self {
            Self::List(values) => Some(values),
            _ => None,
        }
    }

    /// Returns the record of this typed parameter, if it is one.
    pub fn as_typed(&self) -> Option<&Record> {
        match self {
            Self::Typed(record) => Some(record),
            _ => None,
        }
    }

    /// Reports whether this value is `$`.
    pub fn is_null(&self) -> bool {
        matches!(self, Self::Null)
    }

    /// Reports whether this value is `*`.
    pub fn is_derived(&self) -> bool {
        matches!(self, Self::Derived)
    }
}

/// A keyword applied to an ordered parameter list.
///
/// This is both an entity in a data section and a typed parameter inside one;
/// the syntax does not distinguish them and neither does this type.
#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    /// The entity or defined-type name, as spelled in the file.
    pub keyword: String,
    /// The attribute values, in schema order.
    pub params: Vec<Value>,
}

impl Record {
    /// Builds a record from a keyword and its parameters.
    pub fn new(keyword: impl Into<String>, params: Vec<Value>) -> Self {
        Self {
            keyword: keyword.into(),
            params,
        }
    }

    /// Reports whether this record carries `keyword`, ignoring ASCII case.
    ///
    /// Part 21 spells keywords in upper case, but comparing case-insensitively
    /// costs nothing and keeps one class of vendor deviation from mattering.
    pub fn is(&self, keyword: &str) -> bool {
        self.keyword.eq_ignore_ascii_case(keyword)
    }

    /// Returns the parameter at `index`, if the record has one.
    pub fn param(&self, index: usize) -> Option<&Value> {
        self.params.get(index)
    }
}

/// A `#N = ...;` entry of a data section.
///
/// A simple instance holds one record and a complex instance holds several, so
/// `#5 = A(..);` and `#5 = (A(..) B(..));` are the same type and every consumer
/// reads both through [`Instance::record`]. That is what keeps complex
/// instances — which AP203 requires for the rational B-spline forms and for the
/// unit block — a non-event rather than a special case.
#[derive(Debug, Clone, PartialEq)]
pub struct Instance {
    /// The instance name this entry defines.
    pub id: EntityId,
    /// The records it is built from: one when simple, several when complex.
    pub records: Vec<Record>,
    /// The 1-based source line the instance starts on.
    ///
    /// Carried so every later layer can name a position the way a user reads
    /// their file, rather than as a byte offset.
    pub line: u32,
}

impl Instance {
    /// Returns the sole record of a simple instance.
    ///
    /// Returns `None` for a complex instance, whose records must be selected by
    /// keyword instead.
    pub fn simple(&self) -> Option<&Record> {
        match self.records.as_slice() {
            [record] => Some(record),
            _ => None,
        }
    }

    /// Returns this instance's record for `keyword`, simple or complex alike.
    pub fn record(&self, keyword: &str) -> Option<&Record> {
        self.records.iter().find(|record| record.is(keyword))
    }

    /// Reports whether this instance carries a record for `keyword`.
    pub fn is(&self, keyword: &str) -> bool {
        self.record(keyword).is_some()
    }

    /// The keywords this instance is spelled with, for a message about it.
    ///
    /// One for a simple instance. A complex one is the intersection of every
    /// type it names, so naming only the first would describe a different
    /// entity — usually a supertype carrying no attributes at all.
    pub fn spelling(&self) -> String {
        self.records
            .iter()
            .map(|record| record.keyword.as_str())
            .collect::<Vec<_>>()
            .join(" ")
    }
}

/// Two instances in one exchange structure claim the same name.
///
/// This breaks the instance table rather than any one entity, so it is refused
/// at construction instead of being reported and carried.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
#[error("entity {id} on line {line} was already defined on line {first_line}")]
pub struct DuplicateEntityId {
    /// The name claimed twice.
    pub id: EntityId,
    /// The line of the second definition.
    pub line: u32,
    /// The line of the first definition.
    pub first_line: u32,
}

/// A reference to an instance the exchange structure does not define.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DanglingReference {
    /// The instance holding the reference.
    pub from: EntityId,
    /// The name referred to, which nothing defines.
    pub to: EntityId,
    /// The line `from` starts on.
    pub line: u32,
}

/// One Part 21 exchange structure: a header and an instance table.
///
/// Instances are kept in file order and indexed by name, so a consumer can
/// either sweep for the entities it cares about or resolve references.
#[derive(Debug, Clone, PartialEq)]
pub struct StepExchange {
    header: Vec<Record>,
    instances: Vec<Instance>,
    index: HashMap<EntityId, usize>,
}

impl StepExchange {
    /// Builds an exchange structure from a header and a data section.
    ///
    /// Fails if two instances claim the same name, since the table could not
    /// then answer a reference.
    pub fn new(header: Vec<Record>, instances: Vec<Instance>) -> Result<Self, DuplicateEntityId> {
        let mut index: HashMap<EntityId, usize> = HashMap::with_capacity(instances.len());
        for (position, instance) in instances.iter().enumerate() {
            if let Some(&first) = index.get(&instance.id) {
                return Err(DuplicateEntityId {
                    id: instance.id,
                    line: instance.line,
                    first_line: instances[first].line,
                });
            }
            index.insert(instance.id, position);
        }
        Ok(Self {
            header,
            instances,
            index,
        })
    }

    /// Returns the header records, in file order.
    pub fn header(&self) -> &[Record] {
        &self.header
    }

    /// Returns the header record for `keyword`, such as `FILE_SCHEMA`.
    pub fn header_record(&self, keyword: &str) -> Option<&Record> {
        self.header.iter().find(|record| record.is(keyword))
    }

    /// Returns every instance, in file order.
    pub fn instances(&self) -> &[Instance] {
        &self.instances
    }

    /// Returns the instance named `id`.
    pub fn get(&self, id: EntityId) -> Option<&Instance> {
        self.index
            .get(&id)
            .map(|&position| &self.instances[position])
    }

    /// Returns every instance carrying a record for `keyword`, in file order.
    pub fn instances_of<'a>(&'a self, keyword: &'a str) -> impl Iterator<Item = &'a Instance> {
        self.instances
            .iter()
            .filter(move |instance| instance.is(keyword))
    }

    /// Returns every reference that resolves to nothing.
    ///
    /// Left as a query rather than a parse failure: which dangling references
    /// are fatal is a schema question, and a file whose unreachable corner is
    /// broken should still yield the solids it does define.
    pub fn dangling_references(&self) -> Vec<DanglingReference> {
        let mut dangling = Vec::new();
        for instance in &self.instances {
            for record in &instance.records {
                self.collect_dangling(instance, &record.params, &mut dangling);
            }
        }
        dangling
    }

    fn collect_dangling(
        &self,
        instance: &Instance,
        params: &[Value],
        dangling: &mut Vec<DanglingReference>,
    ) {
        for param in params {
            match param {
                Value::Ref(id) if self.get(*id).is_none() => dangling.push(DanglingReference {
                    from: instance.id,
                    to: *id,
                    line: instance.line,
                }),
                Value::List(values) => self.collect_dangling(instance, values, dangling),
                Value::Typed(record) => self.collect_dangling(instance, &record.params, dangling),
                _ => {}
            }
        }
    }
}
