//! Lofting between owned sections.
//!
//! The builder works on one model and names its sections by key, but a user
//! authors sections one at a time and each arrives owning a model of its own.
//! [`loft`] copies every section into one fresh model and hands the result
//! back as a single owned [`Shape`].
//!
//! # One function, and the kind of section still decides the result
//!
//! There is one public entry point rather than one per section kind, and the
//! kind travels in the *type of the sections it is given*: a slice of
//! profiles lofts to a [`SheetTag`] shape, a slice of faces to a
//! [`SolidTag`] one, through [`LoftInput::Output`]. Two things follow, and
//! both are the point:
//!
//! - **A mixed run cannot be spelled.** Every element of a slice has one
//!   type, so a profile among faces is a compile error rather than a runtime
//!   refusal. An enum of section kinds would have made that run writable and
//!   moved the refusal to run time; a homogeneous slice never admits it.
//! - **The result follows the input.** No caller unwraps a result whose shape
//!   it already stated by what it passed in.
//!
//! So the delegation happens *inside*: the caller says `loft(&sections, ..)`
//! and never picks a tail, while the dispatch that picks one is resolved
//! statically from the section type.
//!
//! # What is still inferred
//!
//! [`loft`] also infers whether a run of profiles is open or closed, which
//! [`add_loft`] deliberately does not: its three section types make a mixed
//! run unwritable there. Inference has to face the case those types rule out,
//! so a run that is neither all open nor all closed is refused here by name.
//! That one stays dynamic because closedness is a property of the geometry a
//! user authored, not of the type they named.

use crate::builders::loft::{
    CappedSection, ClosedSection, LoftError, LoftOptions, OpenSection, add_loft,
};
use crate::model::{Cell2, Model};
use crate::topology::closed::{Closeable, Closed};
use crate::topology::face::Face;
use crate::topology::payload::{DefaultPayload, Payload};
use crate::topology::shape::{FaceTag, ProfileTag, Shape, ShapeKind, SheetTag, SolidTag};
use crate::topology::shape_keys::{FaceKey, ProfileKey};

mod sealed {
    pub trait Sealed {}
    impl Sealed for crate::topology::shape::ProfileTag {}
    impl Sealed for crate::topology::shape::FaceTag {}
}

/// A shape kind a loft can run through, and what a run of it makes.
///
/// Sealed: the two kinds are the two the builder has sections for, and a
/// third would need a section type of its own rather than an implementation
/// of this.
pub trait LoftInput: ShapeKind + Sized + sealed::Sealed {
    /// The kind of shape a loft through sections of this kind produces.
    type Output: ShapeKind;

    /// Skins the sections, copying each into one fresh model.
    ///
    /// Called through [`loft`], which is the name callers use.
    fn loft_sections<P: DefaultPayload>(
        sections: &[&Shape<Self, P>],
        options: LoftOptions,
    ) -> Result<Shape<Self::Output, P>, LoftError>;
}

/// Skins an ordered sequence of sections into one shape.
///
/// Profiles loft to a sheet: closed profiles close the ring between the last
/// and first column, open ones leave the sheet open, and which of the two
/// applies is read off the profiles themselves, so a run mixing them is
/// refused — it names no shape.
///
/// Faces loft to a solid, the two end faces becoming the caps. An
/// intermediate face is consumed for its outer loop alone, since there is no
/// cap in the middle of a loft.
pub fn loft<K: LoftInput, P: DefaultPayload>(
    sections: &[&Shape<K, P>],
    options: LoftOptions,
) -> Result<Shape<K::Output, P>, LoftError> {
    K::loft_sections(sections, options)
}

impl LoftInput for ProfileTag {
    type Output = SheetTag;

    fn loft_sections<P: DefaultPayload>(
        sections: &[&Shape<Self, P>],
        options: LoftOptions,
    ) -> Result<Shape<SheetTag, P>, LoftError> {
        let mut model = Model::new();
        let keys = merge_profiles(&mut model, sections)?;

        let sheet = if closedness(&model, &keys)? {
            let sections = keys
                .iter()
                .map(|key| {
                    Closed::new(model.profile_unchecked(*key))
                        .map(|closed| ClosedSection::new(&closed))
                        .ok_or(LoftError::ClosedProfileAsOpenSection)
                })
                .collect::<Result<Vec<_>, _>>()?;
            add_loft(&mut model, &sections, options)?
        } else {
            let sections = keys
                .iter()
                .map(|key| OpenSection::new(&model.profile_unchecked(*key)))
                .collect::<Result<Vec<_>, _>>()?;
            add_loft(&mut model, &sections, options)?
        };
        Ok(Shape::new(model, sheet))
    }
}

impl LoftInput for FaceTag {
    type Output = SolidTag;

    fn loft_sections<P: DefaultPayload>(
        sections: &[&Shape<Self, P>],
        options: LoftOptions,
    ) -> Result<Shape<SolidTag, P>, LoftError> {
        let mut model = Model::new();
        let keys = model.transaction(|edit| {
            let mut keys = Vec::with_capacity(sections.len());
            for section in sections {
                let dart = edit.merge(section.face());
                keys.push(
                    *edit
                        .attribute::<Cell2>(dart)
                        .expect("a copied face keeps its registration"),
                );
            }
            Ok::<Vec<FaceKey>, LoftError>(keys)
        })?;

        let sections = keys
            .iter()
            .map(|key| CappedSection::new(&Face::new(&model, *key)))
            .collect::<Result<Vec<_>, _>>()?;
        let solid = add_loft(&mut model, &sections, options)?;
        Ok(Shape::new(model, solid))
    }
}

/// Copies every profile into `model`, keeping the order they were given in.
fn merge_profiles<P: DefaultPayload>(
    model: &mut Model<P>,
    sections: &[&Shape<ProfileTag, P>],
) -> Result<Vec<ProfileKey>, LoftError> {
    model.transaction(|edit| {
        let mut keys = Vec::with_capacity(sections.len());
        for section in sections {
            let dart = edit.merge(section.profile());
            keys.push(
                edit.profile_key(dart)
                    .expect("a copied profile keeps its registration"),
            );
        }
        Ok::<Vec<ProfileKey>, LoftError>(keys)
    })
}

/// Whether the run is closed, refusing one that is neither all nor none.
fn closedness<P: Payload>(model: &Model<P>, keys: &[ProfileKey]) -> Result<bool, LoftError> {
    let mut closed = None;
    for (index, key) in keys.iter().enumerate() {
        let is_closed = model.profile_unchecked(*key).is_closed();
        match closed {
            None => closed = Some(is_closed),
            Some(first) if first != is_closed => {
                return Err(LoftError::MixedProfiles { index });
            }
            Some(_) => {}
        }
    }
    closed.ok_or(LoftError::TooFewSections { got: 0 })
}
