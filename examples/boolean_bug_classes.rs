//! Sends a few cases of each known Boolean bug class to the debug viewer.
//!
//! ```text
//! cargo run --release --example boolean_bug_classes -- [--class A] [--case 2] [--label before]
//! ```
//!
//! A class is a family of configurations that break one assumption of the
//! Boolean pipeline, the way coincident contact regions or an edge lying on
//! the other operand's surface do. Each case is one member of a family: the
//! reproduction of an issue, or a sibling the same assumption fails on.
//!
//! Every case becomes one entry of the viewer's timeline, named
//! `<class><n> · <title> · <op> · <label>`. It holds both operands, the result
//! when there is one, and every stage the Boolean reached, as its trace
//! recorded it: the imprints each side found, the sections they were joined
//! into, the model after imprinting, the faces kept and dropped, and -- when
//! the Boolean fails -- the model as it stood at that moment, with the face it
//! could not split and that face's imprints. The label tells two runs apart:
//! run it once before a fix and once after, and the timeline holds both.
//!
//! Start the viewer first (`npm run debug` in `visualization/`): this talks to
//! it on `127.0.0.1:3941`, or whatever `NGK_DEBUG_VIEWER_PORT` names.

use std::error::Error;
use std::panic::AssertUnwindSafe;

use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanOperation, BooleanTrace, BooleanTraceStage};
use ngk::builders::edges::add_edge;
use ngk::geometry::{
    Frame, Interval, LINEAR_TOLERANCE, Plane, Point3, PointCoincidence, TrimmedCurve,
};
use ngk::model::Model;
use ngk::modeling::{faces, loft, solids, sweep};
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape::{FaceTag, Shape, SolidTag};
use ngk::topology::shape_keys::FaceKey;
use ngk::viz::debug_viewer::{
    DebugDisplay, DebugNode, DebugStyle, DebugViewerError, DebugViewerOptions, DebugViewerPayload,
    payload_for_display, send_payload,
};

type Solid = Shape<SolidTag>;
type Face = Shape<FaceTag>;

/// The two operands of one case, built afresh for each run.
enum Operands {
    Solids(Box<dyn Fn() -> (Solid, Solid)>),
    Faces(Box<dyn Fn() -> (Face, Face)>),
}

struct Case {
    class: char,
    title: &'static str,
    operation: BooleanOperation,
    operands: Operands,
}

fn main() -> Result<(), Box<dyn Error>> {
    let arguments = Arguments::parse()?;
    let mut counts = std::collections::HashMap::new();
    for case in cases() {
        let index = counts.entry(case.class).or_insert(0);
        *index += 1;
        if arguments.class.is_some_and(|class| class != case.class)
            || arguments.case.is_some_and(|wanted| wanted != *index)
        {
            continue;
        }
        let name = format!(
            "{}{} · {} · {} · {}",
            case.class,
            index,
            case.title,
            operation_name(case.operation),
            arguments.label
        );
        let outcome = send_case(&case, &name)?;
        println!("{name}: {outcome}");
    }
    Ok(())
}

fn cases() -> Vec<Case> {
    let up = Vector3::z();
    let at = move |x: f64, y: f64, z: f64| Frame::from_xz(Point3::new(x, y, z), Vector3::x(), up);
    let square = move |corner: f64, z: f64, size: f64| {
        faces::square(
            Plane::new(Point3::new(corner, corner, z), Vector3::x(), up),
            size,
        )
        .expect("square")
    };
    let rectangle = move |width: f64, height: f64, x: f64, y: f64| {
        faces::rectangle(
            Plane::new(Point3::new(x, y, 0.0), Vector3::x(), up),
            width,
            height,
        )
        .expect("rectangle")
    };
    let tube = move || {
        solids::cut(
            solids::block_at(at(30.0, 15.0, 10.0), 40.0, 30.0, 8.0).expect("tube block"),
            [&solids::block_at(at(45.0, 25.0, 10.0), 10.0, 10.0, 8.0).expect("tube hole")],
        )
        .expect("tube")
    };
    let solids_case = |class, title, operation, build: Box<dyn Fn() -> (Solid, Solid)>| Case {
        class,
        title,
        operation,
        operands: Operands::Solids(build),
    };
    vec![
        // A. Coincident contacts: the operands share a region of one surface.
        solids_case(
            'A',
            "tube with a hole standing on a plate (#29)",
            BooleanOperation::Union,
            Box::new(move || (solids::block(100.0, 60.0, 10.0).expect("plate"), tube())),
        ),
        solids_case(
            'A',
            "straight loft standing on an extruded box (#39)",
            BooleanOperation::Union,
            Box::new(move || {
                let box_ = sweep::extrude_face(square(0.0, 0.0, 4.0), Vector3::new(0.0, 0.0, 5.0))
                    .expect("box");
                let lofted = loft::loft(
                    &[&square(0.0, 5.0, 4.0), &square(0.0, 10.0, 4.0)],
                    Default::default(),
                )
                .expect("loft");
                (box_, lofted)
            }),
        ),
        solids_case(
            'A',
            "small block flush with a block's top edge",
            BooleanOperation::Union,
            Box::new(move || {
                (
                    solids::block(10.0, 10.0, 10.0).expect("block"),
                    solids::block_at(at(0.0, 3.0, 10.0), 4.0, 4.0, 4.0).expect("small block"),
                )
            }),
        ),
        Case {
            class: 'A',
            title: "rectangle flush with a rectangle's side, inside (#31)",
            operation: BooleanOperation::Union,
            operands: Operands::Faces(Box::new(move || {
                (
                    rectangle(10.0, 10.0, 0.0, 0.0),
                    rectangle(4.0, 4.0, 0.0, 3.0),
                )
            })),
        },
        // B. An edge of one operand lies on the other's surface.
        solids_case(
            'B',
            "cylinder axis on a block edge, ends coplanar (#3)",
            BooleanOperation::Difference,
            Box::new(|| {
                (
                    solids::block(30.0, 20.0, 10.0).expect("block"),
                    solids::cylinder(4.0, 10.0).expect("cylinder"),
                )
            }),
        ),
        solids_case(
            'B',
            "cylinder axis on a block edge, top above the block (#37)",
            BooleanOperation::Difference,
            Box::new(move || {
                (
                    solids::block(30.0, 20.0, 10.0).expect("block"),
                    solids::cylinder_at(at(0.0, 0.0, 5.0), 4.0, 10.0).expect("cylinder"),
                )
            }),
        ),
        solids_case(
            'B',
            "cylinder axis on a block edge, fused (#37)",
            BooleanOperation::Union,
            Box::new(|| {
                (
                    solids::block(30.0, 20.0, 10.0).expect("block"),
                    solids::cylinder(4.0, 10.0).expect("cylinder"),
                )
            }),
        ),
        solids_case(
            'B',
            "cylinder axis on a block face, ends coplanar",
            BooleanOperation::Difference,
            Box::new(move || {
                (
                    solids::block(30.0, 20.0, 10.0).expect("block"),
                    solids::cylinder_at(at(15.0, 0.0, 0.0), 4.0, 10.0).expect("cylinder"),
                )
            }),
        ),
        solids_case(
            'B',
            "cylinder tangent to a block face",
            BooleanOperation::Union,
            Box::new(move || {
                (
                    solids::block(30.0, 20.0, 10.0).expect("block"),
                    solids::cylinder_at(at(15.0, -4.0, 2.0), 4.0, 6.0).expect("cylinder"),
                )
            }),
        ),
        solids_case(
            'B',
            "lying cylinder whose wall holds a block's top edge",
            BooleanOperation::Union,
            Box::new(|| {
                // The wall's circle passes through the edge's corner, and its
                // disc bites into the block below and behind the edge.
                let frame =
                    Frame::from_xz(Point3::new(-5.0, 1.0, 11.0), Vector3::y(), Vector3::x());
                (
                    solids::block(30.0, 20.0, 10.0).expect("block"),
                    solids::cylinder_at(frame, 2.0_f64.sqrt(), 40.0).expect("cylinder"),
                )
            }),
        ),
    ]
}

/// Runs one case and sends it to the viewer, returning how it ended.
fn send_case(case: &Case, name: &str) -> Result<String, Box<dyn Error>> {
    let mut nodes = Vec::new();
    let outcome = match &case.operands {
        Operands::Solids(build) => {
            let (target, tool) = build();
            let operands = operand_nodes(&target, &tool)?;
            let (result, trace) = solids::traced(target, [&tool], case.operation);
            let outcome = match &result {
                Ok(shape) => {
                    // The result says it all; the operands stay at hand.
                    nodes.extend(operands.into_iter().map(DebugNode::hidden));
                    nodes.extend(node("result", shape)?);
                    format!("ok, volume {:?}", shape.solid().volume())
                }
                Err(error) => {
                    nodes.extend(operands);
                    format!("failed: {}", chain(error))
                }
            };
            nodes.push(trace_node(&trace)?);
            outcome
        }
        Operands::Faces(build) => {
            let (target, tool) = build();
            nodes.extend(operand_nodes(&target, &tool)?);
            let combined = match case.operation {
                BooleanOperation::Union => faces::fuse(target, [&tool]),
                BooleanOperation::Difference => faces::cut(target, [&tool]),
                BooleanOperation::Intersection => faces::intersect(target, [&tool]),
            };
            match combined {
                Ok(result) => {
                    nodes.extend(node("result", &result.faces())?);
                    format!("ok, {} face(s)", result.faces().len())
                }
                Err(error) => format!("failed: {}", chain(&error)),
            }
        }
    };
    let options = DebugViewerOptions {
        name: name.to_owned(),
        ..Default::default()
    };
    let payload = DebugViewerPayload {
        kind: "ngk.debug.v4".to_owned(),
        name: name.to_owned(),
        nodes: vec![DebugNode::group(outcome.clone(), nodes)],
    };
    send_payload(&payload, &options)?;
    Ok(outcome)
}

/// The operands, see-through so whatever a stage singles out shows inside
/// them: the target blue, the tool orange.
fn operand_nodes<T: DebugDisplay>(
    target: &T,
    tool: &T,
) -> Result<Vec<DebugNode>, DebugViewerError> {
    let mut nodes = styled(node("target", target)?, style("#4a7bc8", Some(0.25), None));
    nodes.extend(styled(
        node("tool", tool)?,
        style("#e0913a", Some(0.25), None),
    ));
    Ok(nodes)
}

/// One stage per group, each holding what that stage recorded.
///
/// One stage arrives checked, the last that singles anything out -- the face
/// that would not split and its imprints, the contacts that would not join,
/// or the faces kept when nothing failed -- since a stage holding only a model
/// shows nothing the operands do not. Every other is a click away.
fn trace_node(trace: &BooleanTrace<StandardPayload>) -> Result<DebugNode, DebugViewerError> {
    let singles_out = |stage: &BooleanTraceStage<StandardPayload>| {
        !stage.faces.is_empty() || !stage.curves.is_empty()
    };
    let last = trace
        .stages()
        .iter()
        .rposition(singles_out)
        .unwrap_or(trace.stages().len().saturating_sub(1));
    let stages = trace
        .stages()
        .iter()
        .enumerate()
        .map(|(index, stage)| {
            let node = stage_node(index, stage)?;
            Ok(if index == last { node } else { node.hidden() })
        })
        .collect::<Result<Vec<_>, DebugViewerError>>()?;
    Ok(DebugNode::group("trace", stages))
}

/// How each kind of group a stage records is drawn.
///
/// The stage's model is a faint grey backdrop, so what the stage singles out
/// stands out against it: the face that failed in solid red, kept faces green
/// and dropped ones faint red, the first operand's imprints red and the
/// second's green, joined sections and a failed face's imprints yellow, and
/// the points they meet at magenta.
fn stage_style(group: &str) -> DebugStyle {
    match group {
        "model" => style("#8a8a95", Some(0.12), None),
        "face" => style("#ff3b3b", Some(1.0), None),
        "kept" => style("#3ccf6e", Some(0.85), None),
        "dropped" => style("#ff6b6b", Some(0.3), None),
        "first imprints" => style("#ff3b3b", None, Some(4.0)),
        "second imprints" => style("#3ccf6e", None, Some(4.0)),
        "points" => style("#ff4fd8", None, None),
        _ => style("#ffd23b", None, Some(4.0)),
    }
}

fn style(color: &str, opacity: Option<f64>, width: Option<f64>) -> DebugStyle {
    DebugStyle {
        color: Some(color.to_owned()),
        opacity,
        width,
    }
}

fn styled(nodes: Vec<DebugNode>, style: DebugStyle) -> Vec<DebugNode> {
    nodes
        .into_iter()
        .map(|node| node.styled(style.clone()))
        .collect()
}

fn stage_node(
    index: usize,
    stage: &BooleanTraceStage<StandardPayload>,
) -> Result<DebugNode, DebugViewerError> {
    let mut children = Vec::new();
    if let Some(model) = &stage.model {
        children.extend(styled(node("model", model)?, stage_style("model")));
        for (name, keys) in &stage.faces {
            let faces = keys
                .iter()
                .map(|key| face_node(model, *key))
                .collect::<Result<Vec<_>, _>>()?;
            children.push(
                DebugNode::group(name.as_str(), faces.into_iter().flatten().collect())
                    .styled(stage_style(name)),
            );
        }
    }
    for (name, curves) in &stage.curves {
        children.extend(styled(
            node(name, &curves_model(curves))?,
            stage_style(name),
        ));
    }
    if !stage.points.is_empty() {
        children.extend(styled(
            node("points", &stage.points)?,
            stage_style("points"),
        ));
    }
    Ok(DebugNode::group(
        format!("{index}. {}", stage.name),
        children,
    ))
}

/// One face of a recorded model, or its outline when it cannot be copied out.
///
/// A model recorded where a split failed can hold the face it was splitting
/// half done: its loops no longer close over the darts it covers, and copying
/// it out of the model trips over the darts it reaches but does not own. Its
/// edges are still there, so it is drawn as their curves, and says so.
fn face_node(
    model: &Model<StandardPayload>,
    key: FaceKey,
) -> Result<Vec<DebugNode>, DebugViewerError> {
    let Some(face) = model.face(key) else {
        return Ok(Vec::new());
    };
    let name = format!("{key:?}");
    let quiet = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let isolated = std::panic::catch_unwind(AssertUnwindSafe(|| node(&name, &face)));
    let outline = match &isolated {
        Ok(_) => None,
        Err(_) => std::panic::catch_unwind(AssertUnwindSafe(|| {
            face.loops()
                .iter()
                .flat_map(|boundary| boundary.edges())
                .map(|edge| edge.trimmed_curve())
                .collect::<Vec<_>>()
        }))
        .ok(),
    };
    std::panic::set_hook(quiet);
    match (isolated, outline) {
        (Ok(nodes), _) => nodes,
        (Err(_), Some(curves)) => node(
            &format!("{name} (outline: the face is half split)"),
            &curves_model(&curves),
        ),
        (Err(_), None) => Ok(vec![DebugNode::group(
            format!("{name} (unreadable: the face is half split)"),
            Vec::new(),
        )]),
    }
}

/// Draws trimmed curves as edges, so the viewer shows each piece rather than
/// its whole support.
///
/// An edge needs two distinct ends, so a closed curve is drawn as its two
/// halves.
fn curves_model(curves: &[TrimmedCurve]) -> Model<StandardPayload> {
    let mut model = Model::new();
    for curve in curves {
        let pieces = if curve.start().coincides(curve.end(), LINEAR_TOLERANCE) {
            vec![
                curve.sub(Interval::new(0.0, 0.5)),
                curve.sub(Interval::new(0.5, 1.0)),
            ]
        } else {
            vec![curve.clone()]
        };
        for piece in pieces {
            let drawn = piece
                .to_curve()
                .map_err(|error| error.to_string())
                .and_then(|support| {
                    add_edge(&mut model, piece.start(), piece.end(), support)
                        .map_err(|error| error.to_string())
                });
            if let Err(error) = drawn {
                eprintln!("could not draw a curve: {error}");
            }
        }
    }
    model
}

/// The viewer nodes for one value, under `name`.
fn node<T: DebugDisplay + ?Sized>(
    name: &str,
    value: &T,
) -> Result<Vec<DebugNode>, DebugViewerError> {
    let options = DebugViewerOptions {
        name: name.to_owned(),
        ..Default::default()
    };
    Ok(payload_for_display(value, &options)?.nodes)
}

/// An error and every cause under it, outermost first.
fn chain(error: &dyn Error) -> String {
    let mut text = error.to_string();
    let mut source = error.source();
    while let Some(cause) = source {
        text.push_str(" <- ");
        text.push_str(&cause.to_string());
        source = cause.source();
    }
    text
}

fn operation_name(operation: BooleanOperation) -> &'static str {
    match operation {
        BooleanOperation::Union => "fuse",
        BooleanOperation::Difference => "cut",
        BooleanOperation::Intersection => "intersect",
    }
}

struct Arguments {
    class: Option<char>,
    case: Option<usize>,
    label: String,
}

impl Arguments {
    fn parse() -> Result<Self, Box<dyn Error>> {
        let mut parsed = Self {
            class: None,
            case: None,
            label: "now".to_owned(),
        };
        let mut arguments = std::env::args().skip(1);
        while let Some(flag) = arguments.next() {
            let value = arguments.next().ok_or(format!("{flag} needs a value"))?;
            match flag.as_str() {
                "--class" => parsed.class = value.chars().next().map(|c| c.to_ascii_uppercase()),
                "--case" => parsed.case = Some(value.parse()?),
                "--label" => parsed.label = value,
                _ => return Err(format!("unknown flag {flag}").into()),
            }
        }
        Ok(parsed)
    }
}
