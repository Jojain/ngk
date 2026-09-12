//! Writing an [`Exchange`] back out as Part 21 text.
//!
//! No crate supplies Part 21 *writing*, so this side is ours regardless of
//! what reads the file. It carries the real-formatting guard described in
//! [`format_real`], which is the cheapest way in this feature to produce a
//! file that looks correct and is not.

use std::io::Write;

use thiserror::Error;

use super::value::{Instance, Record, StepExchange, Value};

/// A value with no Part 21 representation, or a failure of the sink.
#[derive(Debug, Error)]
pub enum WriteError {
    /// The sink rejected the bytes.
    #[error("failed to write the exchange structure")]
    Io(#[from] std::io::Error),

    /// Part 21 has no spelling for an infinity or a NaN. Writing one would
    /// produce a file no reader can parse, so it is refused at the source.
    #[error("{value} has no Part 21 representation: a real literal must be finite")]
    NonFiniteReal {
        /// The offending value.
        value: f64,
    },
}

/// Writes an exchange structure as Part 21 text.
pub fn write_exchange(sink: &mut impl Write, exchange: &StepExchange) -> Result<(), WriteError> {
    writeln!(sink, "ISO-10303-21;")?;

    writeln!(sink, "HEADER;")?;
    for record in exchange.header() {
        write_record(sink, record)?;
        writeln!(sink, ";")?;
    }
    writeln!(sink, "ENDSEC;")?;

    writeln!(sink, "DATA;")?;
    for instance in exchange.instances() {
        write_instance(sink, instance)?;
    }
    writeln!(sink, "ENDSEC;")?;

    writeln!(sink, "END-ISO-10303-21;")?;
    Ok(())
}

/// Writes an exchange structure to a string.
pub fn exchange_to_string(exchange: &StepExchange) -> Result<String, WriteError> {
    let mut bytes = Vec::new();
    write_exchange(&mut bytes, exchange)?;
    // Every byte written above came from `str` formatting, so this cannot fail.
    Ok(String::from_utf8(bytes).expect("Part 21 output is written as UTF-8 text"))
}

fn write_instance(sink: &mut impl Write, instance: &Instance) -> Result<(), WriteError> {
    write!(sink, "{} = ", instance.id)?;
    match instance.records.as_slice() {
        [record] => write_record(sink, record)?,
        records => {
            write!(sink, "(")?;
            for record in records {
                write_record(sink, record)?;
            }
            write!(sink, ")")?;
        }
    }
    writeln!(sink, ";")?;
    Ok(())
}

fn write_record(sink: &mut impl Write, record: &Record) -> Result<(), WriteError> {
    write!(sink, "{}", record.keyword)?;
    write_params(sink, &record.params)
}

fn write_params(sink: &mut impl Write, params: &[Value]) -> Result<(), WriteError> {
    write!(sink, "(")?;
    for (position, param) in params.iter().enumerate() {
        if position > 0 {
            write!(sink, ",")?;
        }
        write_value(sink, param)?;
    }
    write!(sink, ")")?;
    Ok(())
}

fn write_value(sink: &mut impl Write, value: &Value) -> Result<(), WriteError> {
    match value {
        Value::Integer(integer) => write!(sink, "{integer}")?,
        Value::Real(real) => write!(sink, "{}", format_real(*real)?)?,
        Value::Text(text) => write!(sink, "'{}'", encode_text(text))?,
        Value::Enum(name) => write!(sink, ".{name}.")?,
        Value::Ref(id) => write!(sink, "{id}")?,
        Value::Null => write!(sink, "$")?,
        Value::Derived => write!(sink, "*")?,
        Value::List(values) => write_params(sink, values)?,
        Value::Typed(record) => write_record(sink, record)?,
    }
    Ok(())
}

/// Formats a real so that Part 21 reads back the same value *and the same
/// type*.
///
/// `format!("{}", 1.0f64)` yields `"1"`, which is an **integer** in Part 21 and
/// changes the parsed type of the attribute — a corruption that produces a file
/// looking entirely correct. The guard is twofold: take the shortest
/// round-tripping form from `{:?}`, then make sure a `.` is present *in the
/// mantissa*, since `1e300` must be written `1.E300` and never `1e300.`.
pub fn format_real(value: f64) -> Result<String, WriteError> {
    if !value.is_finite() {
        return Err(WriteError::NonFiniteReal { value });
    }

    let shortest = format!("{value:?}");
    let (mantissa, exponent) = match shortest.split_once(['e', 'E']) {
        Some((mantissa, exponent)) => (mantissa, Some(exponent)),
        None => (shortest.as_str(), None),
    };

    let mut formatted = String::with_capacity(shortest.len() + 2);
    formatted.push_str(mantissa);
    if !formatted.contains('.') {
        formatted.push('.');
    }
    if let Some(exponent) = exponent {
        formatted.push('E');
        formatted.push_str(exponent);
    }
    Ok(formatted)
}

/// Encodes a string for a Part 21 literal.
///
/// The inverse of [`decode_text`](super::parse::decode_text): `'` doubles, `\`
/// doubles, printable ASCII passes through, and everything else becomes one
/// `\X2\....\X0\` run of UTF-16 code units. Adjacent non-ASCII characters share
/// a run rather than each opening their own.
pub fn encode_text(text: &str) -> String {
    let mut encoded = String::with_capacity(text.len() + 2);
    let mut run: Vec<u16> = Vec::new();

    for character in text.chars() {
        match character {
            '\'' => {
                flush_run(&mut run, &mut encoded);
                encoded.push_str("''");
            }
            '\\' => {
                flush_run(&mut run, &mut encoded);
                encoded.push_str("\\\\");
            }
            character if character.is_ascii_graphic() || character == ' ' => {
                flush_run(&mut run, &mut encoded);
                encoded.push(character);
            }
            character => {
                let mut buffer = [0u16; 2];
                run.extend_from_slice(character.encode_utf16(&mut buffer));
            }
        }
    }

    flush_run(&mut run, &mut encoded);
    encoded
}

fn flush_run(run: &mut Vec<u16>, encoded: &mut String) {
    if run.is_empty() {
        return;
    }
    encoded.push_str("\\X2\\");
    for unit in run.drain(..) {
        encoded.push_str(&format!("{unit:04X}"));
    }
    encoded.push_str("\\X0\\");
}
