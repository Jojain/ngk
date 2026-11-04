//! A drum of radius 8 about z from its floor z = 0 up to the saddle z = 10 + xy/8. The saddle is
//! the bilinear patch on (−10, −10, 22.5), (10, −10, −2.5), (−10, 10, −2.5), (10, 10, 22.5): the
//! ruled loft, linear both ways, between its two edges y = −10 and y = 10. A cylinder from z = 0
//! to 30 is split by that sheet and the part under it is kept.
use crate::bench::p;
use crate::{Refusal, Solid};
use ngk::builders::loft::LoftOptions;
use ngk::modeling::{self, loft, profiles, solids};

pub fn saddle() -> Result<ngk::topology::shape::Shape<ngk::topology::shape::SheetTag>, Refusal> {
    let near = profiles::polyline(&[p(-10.0, -10.0, 22.5), p(10.0, -10.0, -2.5)])?;
    let far = profiles::polyline(&[p(-10.0, 10.0, -2.5), p(10.0, 10.0, 22.5)])?;
    Ok(loft::loft(&[&near, &far], LoftOptions::ruled())?)
}

/// The part of `drum` under the saddle.
pub fn under_saddle(drum: Solid) -> Result<Solid, Refusal> {
    let parts = modeling::split(&drum, &saddle()?)?;
    // The part under the saddle is the one whose centroid is lower.
    let (front, back) = (parts.front.solid().volume_properties()?, parts.back.solid().volume_properties()?);
    Ok(if front.centroid.z < back.centroid.z { parts.front } else { parts.back })
}

pub fn build() -> Result<Solid, Refusal> {
    under_saddle(solids::cylinder(8.0, 30.0)?)
}
