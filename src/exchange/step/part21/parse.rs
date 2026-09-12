//! Reading Part 21 text into an [`Exchange`].
//!
//! Two grammar rules carry most of the awkwardness and are handled explicitly
//! below: a real **always** contains a `.` while an integer never does, and
//! `.T.` must not be read as a real — so a `.` is dispatched on the character
//! that follows it. String decoding is deliberately lenient: an escape this
//! parser does not recognize is passed through literally rather than failing,
//! because a malformed `\P?\` in one vendor string must not sink an otherwise
//! good import.

use thiserror::Error;
use winnow::ascii::{Caseless, digit1, multispace1};
use winnow::combinator::{alt, cut_err, delimited, opt, preceded, repeat, separated, terminated};
use winnow::error::{ContextError, ErrMode, StrContext, StrContextValue};
use winnow::stream::Location;
use winnow::token::{one_of, take_until, take_while};
use winnow::{LocatingSlice, ModalResult, Parser};

use super::value::{DuplicateEntityId, EntityId, StepExchange, Instance, Record, Value};

/// The input stream: text, with byte offsets tracked so an error and every
/// instance can name the line it sits on.
type Input<'a> = LocatingSlice<&'a str>;

/// A file that is not a well-formed Part 21 exchange structure.
///
/// Every variant names a line, because a STEP error a user cannot locate in
/// their file is not actionable.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum SyntaxError {
    /// The text does not parse at the position given.
    #[error("line {line}, column {column}: {expected}")]
    Malformed {
        /// 1-based source line.
        line: u32,
        /// 1-based column, in bytes.
        column: u32,
        /// What the parser was looking for there.
        expected: String,
    },

    /// Two instances claim the same name.
    #[error(transparent)]
    DuplicateEntityId(#[from] DuplicateEntityId),
}

/// Reads Part 21 text into an exchange structure.
pub fn parse_exchange(source: &str) -> Result<StepExchange, SyntaxError> {
    let (header, parsed) = exchange_file
        .parse(LocatingSlice::new(source))
        .map_err(|error| {
            let (line, column) = line_and_column(source, error.offset());
            let expected = describe(error.inner());
            SyntaxError::Malformed {
                line,
                column,
                expected,
            }
        })?;

    let instances = parsed
        .into_iter()
        .map(|instance| Instance {
            id: instance.id,
            records: instance.records,
            line: line_and_column(source, instance.offset).0,
        })
        .collect();

    Ok(StepExchange::new(header, instances)?)
}

/// Renders a parser context as the "expected ..." half of a message.
///
/// Only the innermost context is kept. The outer ones are the enclosing
/// constructs — "a DATA section" around every defect in a file — and listing
/// them buries the one expectation that names the defect.
fn describe(error: &ContextError) -> String {
    match error.context().next() {
        Some(context) => context.to_string(),
        None => "not a well-formed Part 21 exchange structure".to_string(),
    }
}

/// Converts a byte offset into a 1-based line and column.
fn line_and_column(source: &str, offset: usize) -> (u32, u32) {
    let offset = offset.min(source.len());
    let consumed = &source[..offset];
    let line = consumed.bytes().filter(|byte| *byte == b'\n').count() + 1;
    let column = match consumed.rfind('\n') {
        Some(newline) => offset - newline,
        None => offset + 1,
    };
    (line as u32, column as u32)
}

/// An instance as parsed, before its byte offset is resolved to a line.
struct ParsedInstance {
    id: EntityId,
    records: Vec<Record>,
    offset: usize,
}

fn exchange_file(input: &mut Input<'_>) -> ModalResult<(Vec<Record>, Vec<ParsedInstance>)> {
    skip_trivia(input)?;
    word("ISO-10303-21")
        .context(StrContext::Expected(StrContextValue::Description(
            "the ISO-10303-21 start tag",
        )))
        .parse_next(input)?;
    symbol(';').parse_next(input)?;
    let header = header_section(input)?;
    let sections: Vec<Vec<ParsedInstance>> = repeat(1.., data_section)
        .context(StrContext::Expected(StrContextValue::Description(
            "a DATA section",
        )))
        .parse_next(input)?;
    word("END-ISO-10303-21")
        .context(StrContext::Expected(StrContextValue::Description(
            "the END-ISO-10303-21 end tag",
        )))
        .parse_next(input)?;
    symbol(';').parse_next(input)?;
    Ok((header, sections.into_iter().flatten().collect()))
}

fn header_section(input: &mut Input<'_>) -> ModalResult<Vec<Record>> {
    word("HEADER")
        .context(StrContext::Expected(StrContextValue::Description(
            "a HEADER section",
        )))
        .parse_next(input)?;
    symbol(';').parse_next(input)?;
    let records = repeat(0.., terminated(record, symbol(';'))).parse_next(input)?;
    end_of_section(input)?;
    Ok(records)
}

fn data_section(input: &mut Input<'_>) -> ModalResult<Vec<ParsedInstance>> {
    word("DATA").parse_next(input)?;
    // Part 21 edition 3 allows a parameter list on the section header.
    opt(parameter_list).parse_next(input)?;
    symbol(';').parse_next(input)?;
    let instances = repeat(0.., instance).parse_next(input)?;
    end_of_section(input)?;
    Ok(instances)
}

fn end_of_section(input: &mut Input<'_>) -> ModalResult<()> {
    word("ENDSEC")
        .context(StrContext::Expected(StrContextValue::Description(
            "ENDSEC to close the section",
        )))
        .parse_next(input)?;
    symbol(';').parse_next(input)
}

fn instance(input: &mut Input<'_>) -> ModalResult<ParsedInstance> {
    let offset = input.current_token_start();
    let id = entity_id(input)?;
    symbol('=').parse_next(input)?;
    // Past the `=` this can only be an instance, so failures below are cut
    // rather than backtracked. Otherwise the enclosing `repeat` swallows the
    // failure and the error degrades into "expected ENDSEC" at the start of
    // the instance, naming neither the defect nor its column.
    let records = cut_err(alt((
        // A complex instance: several records under one name.
        delimited(symbol('('), repeat(1.., record), symbol(')')),
        record.map(|record| vec![record]),
    )))
    .context(StrContext::Expected(StrContextValue::Description(
        "an entity record after '='",
    )))
    .parse_next(input)?;
    cut_err(symbol(';'))
        .context(StrContext::Expected(StrContextValue::Description(
            "';' to close the instance",
        )))
        .parse_next(input)?;
    Ok(ParsedInstance {
        id,
        records,
        offset,
    })
}

fn record(input: &mut Input<'_>) -> ModalResult<Record> {
    let keyword = keyword(input)?;
    let params = parameter_list(input)?;
    Ok(Record { keyword, params })
}

fn parameter_list(input: &mut Input<'_>) -> ModalResult<Vec<Value>> {
    // An opening parenthesis commits: nothing else in the grammar can follow
    // it, so a malformed list is reported where it breaks rather than being
    // retried as something else.
    delimited(
        symbol('('),
        separated(0.., value, symbol(',')),
        cut_err(symbol(')')).context(StrContext::Expected(StrContextValue::Description(
            "')' to close the parameter list",
        ))),
    )
    .parse_next(input)
}

fn value(input: &mut Input<'_>) -> ModalResult<Value> {
    // Every alternative is distinguished by its first character, so the order
    // here is for readability rather than for disambiguation.
    alt((
        symbol('$').value(Value::Null),
        symbol('*').value(Value::Derived),
        entity_id.map(Value::Ref),
        text.map(Value::Text),
        enumeration.map(Value::Enum),
        number,
        parameter_list.map(Value::List),
        record.map(|record| Value::Typed(Box::new(record))),
    ))
    .context(StrContext::Expected(StrContextValue::Description(
        "a Part 21 parameter",
    )))
    .parse_next(input)
}

fn entity_id(input: &mut Input<'_>) -> ModalResult<EntityId> {
    let id = preceded('#', digit1).parse_to::<u64>().parse_next(input)?;
    skip_trivia(input)?;
    Ok(EntityId(id))
}

/// Reads a numeric literal, classifying it by the Part 21 rule that a real
/// always carries a `.` and an integer never does.
///
/// An exponent also forces a real. A conforming file never writes one without
/// a `.`, so this only ever rescues a malformed literal — it cannot misread a
/// legal one.
fn number(input: &mut Input<'_>) -> ModalResult<Value> {
    let literal = (
        opt(one_of(['+', '-'])),
        digit1,
        opt(('.', opt(digit1))),
        opt((one_of(['e', 'E']), opt(one_of(['+', '-'])), digit1)),
    )
        .take()
        .parse_next(input)?;
    skip_trivia(input)?;

    if literal.contains(['.', 'e', 'E']) {
        // An out-of-range literal parses to an infinity, which Part 21 cannot
        // spell and the kernel must never see. Refuse it here rather than let
        // it travel.
        match literal.parse::<f64>() {
            Ok(parsed) if parsed.is_finite() => Ok(Value::Real(parsed)),
            _ => Err(ErrMode::Backtrack(ContextError::new())),
        }
    } else {
        match literal.parse::<i64>() {
            Ok(parsed) => Ok(Value::Integer(parsed)),
            Err(_) => Err(ErrMode::Backtrack(ContextError::new())),
        }
    }
}

/// Reads a string literal.
///
/// This finds the literal's extent and nothing more: a doubled quote is kept
/// doubled and handed to [`decode_text`], so that decoding lives in one place
/// and stays the exact inverse of
/// [`encode_text`](super::write::encode_text).
fn text(input: &mut Input<'_>) -> ModalResult<String> {
    '\''.parse_next(input)?;
    let mut raw = String::new();
    loop {
        let chunk: &str = take_while(0.., |character| character != '\'').parse_next(input)?;
        raw.push_str(chunk);
        // An opening quote commits: a string that runs to the end of the file
        // is a common vendor defect and should be named as one.
        cut_err('\'')
            .context(StrContext::Expected(StrContextValue::Description(
                "a closing quote",
            )))
            .parse_next(input)?;
        if opt('\'').parse_next(input)?.is_none() {
            break;
        }
        // A doubled quote is part of the literal, not the end of it.
        raw.push_str("''");
    }
    skip_trivia(input)?;
    Ok(decode_text(&raw))
}

/// Reads an enumeration name, which is what a `.` followed by a letter or
/// digit always introduces.
fn enumeration(input: &mut Input<'_>) -> ModalResult<String> {
    let name = delimited(
        '.',
        take_while(1.., |character: char| {
            character.is_ascii_alphanumeric() || character == '_'
        }),
        '.',
    )
    .parse_next(input)?;
    skip_trivia(input)?;
    Ok(name.to_string())
}

fn keyword(input: &mut Input<'_>) -> ModalResult<String> {
    let keyword = (
        opt('!'),
        one_of(|character: char| character.is_ascii_alphabetic() || character == '_'),
        take_while(0.., |character: char| {
            character.is_ascii_alphanumeric() || character == '_'
        }),
    )
        .take()
        .parse_next(input)?;
    skip_trivia(input)?;
    Ok(keyword.to_string())
}

/// Matches one punctuation character and the trivia after it.
fn symbol<'a>(character: char) -> impl Parser<Input<'a>, (), ErrMode<ContextError>> {
    move |input: &mut Input<'a>| {
        let mut punctuation = character;
        punctuation.parse_next(input)?;
        skip_trivia(input)
    }
}

/// Matches one keyword, ignoring ASCII case, and the trivia after it.
fn word<'a>(text: &'static str) -> impl Parser<Input<'a>, (), ErrMode<ContextError>> {
    move |input: &mut Input<'a>| {
        let mut literal = Caseless(text);
        literal.parse_next(input)?;
        skip_trivia(input)
    }
}

/// Consumes whitespace and `/* */` comments.
fn skip_trivia(input: &mut Input<'_>) -> ModalResult<()> {
    let _: () = repeat(0.., alt((multispace1.void(), comment))).parse_next(input)?;
    Ok(())
}

fn comment(input: &mut Input<'_>) -> ModalResult<()> {
    ("/*", take_until(0.., "*/"), "*/").void().parse_next(input)
}

/// Decodes the body of a Part 21 string literal.
///
/// The exact inverse of [`encode_text`](super::write::encode_text): a doubled
/// quote collapses to one, and the backslash escapes are resolved. An
/// unrecognized escape — a `\P?\` page directive, or anything malformed — is
/// passed through literally, so one bad string costs its own fidelity and
/// nothing else.
pub fn decode_text(raw: &str) -> String {
    let characters: Vec<char> = raw.chars().collect();
    let mut decoded = String::with_capacity(raw.len());
    let mut index = 0;
    while index < characters.len() {
        if characters[index] == '\'' && characters.get(index + 1) == Some(&'\'') {
            decoded.push('\'');
            index += 2;
            continue;
        }
        if characters[index] != '\\' {
            decoded.push(characters[index]);
            index += 1;
            continue;
        }
        match decode_escape(&characters, index) {
            Some((text, next)) => {
                decoded.push_str(&text);
                index = next;
            }
            None => {
                decoded.push('\\');
                index += 1;
            }
        }
    }
    decoded
}

/// Decodes the escape starting at `start`, returning its text and the index
/// just past it, or `None` if it is not one this parser knows.
fn decode_escape(characters: &[char], start: usize) -> Option<(String, usize)> {
    match characters.get(start + 1)? {
        // `\\` is one backslash.
        '\\' => Some((String::from('\\'), start + 2)),
        // `\S\c` is `c` shifted into the upper half of the code page.
        'S' => {
            let marker = characters.get(start + 2)?;
            if *marker != '\\' {
                return None;
            }
            let shifted = (*characters.get(start + 3)? as u32).checked_add(0x80)?;
            Some((char::from_u32(shifted)?.to_string(), start + 4))
        }
        'X' => match characters.get(start + 2)? {
            // `\X\HH` is one byte of the current code page.
            '\\' => {
                let digits: String = characters.get(start + 3..start + 5)?.iter().collect();
                let code = u32::from_str_radix(&digits, 16).ok()?;
                Some((char::from_u32(code)?.to_string(), start + 5))
            }
            // `\X2\....\X0\` is a run of UTF-16 code units.
            '2' => decode_extended(characters, start, 4),
            // `\X4\........\X0\` is a run of code points.
            '4' => decode_extended(characters, start, 8),
            _ => None,
        },
        _ => None,
    }
}

/// Decodes an `\X2\`/`\X4\` run of fixed-width hex groups up to its `\X0\`.
fn decode_extended(characters: &[char], start: usize, width: usize) -> Option<(String, usize)> {
    if *characters.get(start + 3)? != '\\' {
        return None;
    }
    let mut index = start + 4;
    let mut units: Vec<u32> = Vec::new();
    loop {
        if *characters.get(index)? == '\\' {
            let terminator: String = characters.get(index..index + 4)?.iter().collect();
            if !terminator.eq_ignore_ascii_case("\\X0\\") {
                return None;
            }
            index += 4;
            break;
        }
        let group: String = characters.get(index..index + width)?.iter().collect();
        if !group.chars().all(|character| character.is_ascii_hexdigit()) {
            return None;
        }
        units.push(u32::from_str_radix(&group, 16).ok()?);
        index += width;
    }

    let text = if width == 4 {
        let code_units: Vec<u16> = units.iter().map(|unit| *unit as u16).collect();
        String::from_utf16(&code_units).ok()?
    } else {
        units
            .iter()
            .map(|unit| char::from_u32(*unit))
            .collect::<Option<String>>()?
    };
    Some((text, index))
}
