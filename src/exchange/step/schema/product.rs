//! Product structure, units and the document context.
//!
//! This is the boilerplate that turns a heap of geometry into something
//! another CAD system will open, and it is confined to this module so that no
//! other layer learns which application protocol the file is written in.
//!
//! The schema written is **AP214** (`AUTOMOTIVE_DESIGN`). The entity list is
//! the same one AP203 needs — only `FILE_SCHEMA` and the application-context
//! description differ — and AP214 is what OpenCascade writes by default, so
//! it is the spelling most readers are known to accept.
//!
//! The read side needs none of this: it finds `MANIFOLD_SOLID_BREP` directly
//! rather than walking down from `SHAPE_DEFINITION_REPRESENTATION`, because
//! product structure is where vendor files diverge most.

use crate::geometry::Frame;

use super::super::builder::InstanceBuilder;
use super::super::convert::placement::write_unshared_placement;
use super::super::options::StepWriteOptions;
use super::super::part21::{EntityId, Record, Value};
use super::entities::{self, Entity};

/// The AP214 schema name, as `FILE_SCHEMA` must spell it.
const SCHEMA_NAME: &str = "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }";

/// Builds the `HEADER` section.
pub fn write_header(options: &StepWriteOptions) -> Vec<Record> {
    vec![
        Record::new(
            "FILE_DESCRIPTION",
            vec![
                Value::List(vec![Value::Text(options.description.clone())]),
                Value::Text("2;1".to_string()),
            ],
        ),
        Record::new(
            "FILE_NAME",
            vec![
                Value::Text(options.product_name.clone()),
                Value::Text(options.timestamp.clone()),
                Value::List(vec![Value::Text(options.author.clone())]),
                Value::List(vec![Value::Text(options.organization.clone())]),
                Value::Text(originating_system()),
                Value::Text(originating_system()),
                Value::Text(String::new()),
            ],
        ),
        Record::new(
            "FILE_SCHEMA",
            vec![Value::List(vec![Value::Text(SCHEMA_NAME.to_string())])],
        ),
    ]
}

fn originating_system() -> String {
    format!("ngk {}", env!("CARGO_PKG_VERSION"))
}

/// The document context every geometric representation is placed in.
///
/// Returned so the representation can name it, and so the uncertainty it
/// carries stays next to the units it is measured in.
#[derive(Debug, Clone, Copy)]
pub struct DocumentContext {
    /// The `(GEOMETRIC_REPRESENTATION_CONTEXT(3) ...)` complex instance.
    pub context: EntityId,
}

/// Writes the unit block and the 3D representation context.
///
/// Lengths are millimetres, matching the convention NGK's own models and its
/// reference kernel both use. NGK holds no units of its own, so this is
/// a declaration about the file rather than a conversion of anything.
pub fn write_context(builder: &mut InstanceBuilder, options: &StepWriteOptions) -> DocumentContext {
    // `SI_UNIT` and `UNCERTAINTY_MEASURE_WITH_UNIT` go through the entity
    // types because the reader reads them: the two directions state that
    // attribute order once between them rather than twice. The rest of this
    // module is written and never read, so it builds records directly.
    let length = builder.add_complex(vec![
        Record::new("LENGTH_UNIT", Vec::new()),
        Record::new("NAMED_UNIT", vec![Value::Derived]),
        entities::SiUnit {
            prefix: Some("MILLI".to_string()),
            name: "METRE".to_string(),
        }
        .record(),
    ]);
    let angle = builder.add_complex(vec![
        Record::new("NAMED_UNIT", vec![Value::Derived]),
        Record::new("PLANE_ANGLE_UNIT", Vec::new()),
        entities::SiUnit {
            prefix: None,
            name: "RADIAN".to_string(),
        }
        .record(),
    ]);
    let solid_angle = builder.add_complex(vec![
        Record::new("NAMED_UNIT", vec![Value::Derived]),
        entities::SiUnit {
            prefix: None,
            name: "STERADIAN".to_string(),
        }
        .record(),
        Record::new("SOLID_ANGLE_UNIT", Vec::new()),
    ]);

    let uncertainty = builder.add_entity(&entities::UncertaintyMeasureWithUnit {
        value_component: entities::Measure::length(options.uncertainty),
        unit_component: length,
        name: "distance_accuracy_value".to_string(),
        description: "confusion accuracy".to_string(),
    });

    let context = builder.add_complex(vec![
        Record::new("GEOMETRIC_REPRESENTATION_CONTEXT", vec![Value::Integer(3)]),
        Record::new(
            "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT",
            vec![Value::List(vec![Value::Ref(uncertainty)])],
        ),
        Record::new(
            "GLOBAL_UNIT_ASSIGNED_CONTEXT",
            vec![Value::List(vec![
                Value::Ref(length),
                Value::Ref(angle),
                Value::Ref(solid_angle),
            ])],
        ),
        Record::new(
            "REPRESENTATION_CONTEXT",
            vec![
                Value::Text("Context #1".to_string()),
                Value::Text("3D Context with UNIT and UNCERTAINTY".to_string()),
            ],
        ),
    ]);

    DocumentContext { context }
}

/// Wraps already-written B-Rep roots in the product structure that names them.
///
/// `breps` are the solid instances to publish, whichever `MANIFOLD_SOLID_BREP`
/// subtype each was written as; they are all placed in one product, which is as
/// much assembly structure as a single product can express.
pub fn write_product(
    builder: &mut InstanceBuilder,
    options: &StepWriteOptions,
    context: DocumentContext,
    breps: &[EntityId],
) {
    let application = builder.add(Record::new(
        "APPLICATION_CONTEXT",
        vec![Value::Text(
            "core data for automotive mechanical design processes".to_string(),
        )],
    ));
    builder.add(Record::new(
        "APPLICATION_PROTOCOL_DEFINITION",
        vec![
            Value::Text("international standard".to_string()),
            Value::Text("automotive_design".to_string()),
            Value::Integer(2000),
            Value::Ref(application),
        ],
    ));

    let product_context = builder.add(Record::new(
        "PRODUCT_CONTEXT",
        vec![
            Value::Text(String::new()),
            Value::Ref(application),
            Value::Text("mechanical".to_string()),
        ],
    ));
    let product = builder.add(Record::new(
        "PRODUCT",
        vec![
            Value::Text(options.product_name.clone()),
            Value::Text(options.product_name.clone()),
            Value::Text(String::new()),
            Value::List(vec![Value::Ref(product_context)]),
        ],
    ));
    let formation = builder.add(Record::new(
        "PRODUCT_DEFINITION_FORMATION",
        vec![
            Value::Text(String::new()),
            Value::Text(String::new()),
            Value::Ref(product),
        ],
    ));
    let definition_context = builder.add(Record::new(
        "PRODUCT_DEFINITION_CONTEXT",
        vec![
            Value::Text("part definition".to_string()),
            Value::Ref(application),
            Value::Text("design".to_string()),
        ],
    ));
    let definition = builder.add(Record::new(
        "PRODUCT_DEFINITION",
        vec![
            Value::Text("design".to_string()),
            Value::Text(String::new()),
            Value::Ref(formation),
            Value::Ref(definition_context),
        ],
    ));
    let definition_shape = builder.add(Record::new(
        "PRODUCT_DEFINITION_SHAPE",
        vec![
            Value::Text(String::new()),
            Value::Text(String::new()),
            Value::Ref(definition),
        ],
    ));

    // The representation's own placement: the world origin the solids are
    // already expressed in, so it is the identity frame rather than a
    // transform anything has to be applied through.
    //
    // Written unshared, though its coordinates are ordinary values. This is
    // the representation's datum rather than a position being reused, and a
    // face whose plane happens to sit at the origin would otherwise hand its
    // `PLANE` the very instance the representation lists as an item of itself.
    // OpenCascade accepts that, but its own files keep the two apart, so we do
    // too rather than rely on every reader being as tolerant.
    let origin = write_unshared_placement(builder, &Frame::xyz());
    let mut items = vec![Value::Ref(origin)];
    items.extend(breps.iter().map(|brep| Value::Ref(*brep)));
    let representation = builder.add(Record::new(
        "ADVANCED_BREP_SHAPE_REPRESENTATION",
        vec![
            Value::Text(String::new()),
            Value::List(items),
            Value::Ref(context.context),
        ],
    ));

    builder.add(Record::new(
        "SHAPE_DEFINITION_REPRESENTATION",
        vec![Value::Ref(definition_shape), Value::Ref(representation)],
    ));
    builder.add(Record::new(
        "PRODUCT_RELATED_PRODUCT_CATEGORY",
        vec![
            Value::Text("part".to_string()),
            Value::Null,
            Value::List(vec![Value::Ref(product)]),
        ],
    ));
}
