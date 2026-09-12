//! Accumulating instances into an exchange structure.
//!
//! Sits between L1 and the mapping layers: it knows that a STEP file is a list
//! of named instances, but nothing about what any of them mean. Both L3 and L4
//! emit through it, which is what lets a surface and a face allocate names from
//! one counter without either knowing about the other.

use std::collections::HashMap;
use std::fmt::Write as _;

use super::part21::{EntityId, Instance, Record, StepExchange, Value};

/// Allocates instance names and accumulates records in emission order.
///
/// The distinction that matters here is [`add`](Self::add) versus
/// [`add_shared`](Self::add_shared) — see those. Getting it wrong is silent:
/// sharing a `VERTEX_POINT` welds two corners of a solid together, and not
/// sharing a `CARTESIAN_POINT` merely makes the file bigger.
#[derive(Debug, Default)]
pub struct InstanceBuilder {
    instances: Vec<Instance>,
    shared: HashMap<String, EntityId>,
}

impl InstanceBuilder {
    /// Returns an empty builder.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a record under a fresh name, never shared.
    ///
    /// For everything that *has identity*: a `VERTEX_POINT` is a corner of the
    /// solid, not a position, so two corners that happen to coincide must stay
    /// two instances. The same goes for `EDGE_CURVE`, `ADVANCED_FACE` and the
    /// product-structure entities.
    pub fn add(&mut self, record: Record) -> EntityId {
        self.push(vec![record])
    }

    /// Adds a complex instance: several records under one fresh name.
    ///
    /// AP214 requires this for the unit block and the document context, where
    /// one name carries `NAMED_UNIT`, `SI_UNIT` and the measure type at once.
    pub fn add_complex(&mut self, records: Vec<Record>) -> EntityId {
        self.push(records)
    }

    /// Adds a record, reusing an identical one already emitted.
    ///
    /// For everything that is *a value*: a `CARTESIAN_POINT` is a position and
    /// two equal positions are the same entity. This is what keeps a file from
    /// being dominated by repeated points and directions — a box names eight
    /// corners rather than twenty-four.
    ///
    /// Records are matched on their exact content, reals compared by bits, so
    /// a shared record is one that would have been written identically.
    pub fn add_shared(&mut self, record: Record) -> EntityId {
        let key = dedup_key(&record);
        if let Some(&existing) = self.shared.get(&key) {
            return existing;
        }
        let id = self.push(vec![record]);
        self.shared.insert(key, id);
        id
    }

    /// Returns the name the next instance will be given.
    ///
    /// Lets an entity that must refer to itself, or to a cycle it is part of,
    /// reserve its own name before its parameters can be built.
    pub fn peek_next_id(&self) -> EntityId {
        EntityId(self.instances.len() as u64 + 1)
    }

    /// Finishes into an exchange structure carrying `header`.
    ///
    /// Infallible by construction: names are handed out from one counter, so
    /// the duplicate an exchange structure can otherwise be refused for cannot
    /// arise here.
    pub fn finish(self, header: Vec<Record>) -> StepExchange {
        StepExchange::new(header, self.instances)
            .expect("names are allocated sequentially, so no two instances can share one")
    }

    fn push(&mut self, records: Vec<Record>) -> EntityId {
        let id = self.peek_next_id();
        self.instances.push(Instance {
            id,
            records,
            // Instances the writer emits have no source line; it is `parse`'s
            // job to fill this, not ours.
            line: 0,
        });
        id
    }
}

/// Renders a record into a key equal exactly when two records are identical.
///
/// Two properties carry the whole safety of sharing, and both are deliberate.
///
/// **Reals go in as bits, never within a tolerance.** Two positions share an
/// instance only when they are the same `f64` to the last bit — which is also
/// exactly when the writer would spell them identically, since `{:?}` is
/// shortest-round-trip and so distinguishes every distinct value. Arithmetic
/// error can therefore only cause *under*-sharing: two positions a hair apart
/// get two instances and the file is slightly larger. It can never merge two
/// entities that differ.
///
/// **Every variable-length string is length-prefixed.** Without that the
/// encoding is not uniquely decodable: a text containing the separator can
/// impersonate a parameter boundary, so `K('x\u{1},ty')` and `K('x','y')`
/// produce one key and two unrelated entities silently collapse into one.
/// Reading a length before the bytes makes that impossible rather than
/// unlikely.
fn dedup_key(record: &Record) -> String {
    let mut key = String::new();
    write_key_record(&mut key, record);
    key
}

fn write_key_record(key: &mut String, record: &Record) {
    write_key_str(key, 'k', &record.keyword);
    key.push('(');
    for param in &record.params {
        write_key_value(key, param);
        key.push(',');
    }
    key.push(')');
}

fn write_key_value(key: &mut String, value: &Value) {
    match value {
        Value::Integer(integer) => {
            let _ = write!(key, "i{integer}");
        }
        Value::Real(real) => {
            let _ = write!(key, "r{}", real.to_bits());
        }
        Value::Text(text) => write_key_str(key, 't', text),
        Value::Enum(name) => write_key_str(key, 'e', name),
        Value::Ref(id) => {
            let _ = write!(key, "#{}.", id.0);
        }
        Value::Null => key.push('$'),
        Value::Derived => key.push('*'),
        Value::List(values) => {
            key.push('[');
            for value in values {
                write_key_value(key, value);
                key.push(',');
            }
            key.push(']');
        }
        Value::Typed(record) => write_key_record(key, record),
    }
}

/// Writes a string whose own bytes can never be read as structure.
fn write_key_str(key: &mut String, tag: char, text: &str) {
    let _ = write!(key, "{tag}{}:{text}", text.len());
}
