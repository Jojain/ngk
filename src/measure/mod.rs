//! Geometric measures and mass properties for NGK topology views.
//!
//! Every measure is an integral reduced to one-dimensional ones. A solid's
//! volume integrals become flux integrals over its faces by the divergence
//! theorem; each face's integral over its trimmed parameter domain becomes a
//! line integral along its pcurves by Green's theorem; and that line integral,
//! with the inner integral it carries, is evaluated by Gauss quadrature split
//! at every knot and quarter turn. Curves are integrated directly along their
//! 3D spans. No mesh is built, so whatever has a readable boundary measures.
//!
//! # What is exact, and what bounds the rest
//!
//! The public properties are integrated adaptively until each moment's error
//! estimate is within `1e-10` of the largest moment's magnitude, offsets taken
//! in units of the shape's size. The estimate is the gap between a Kronrod
//! rule and the Gauss rule inside it, which bounds the Gauss rule's error; the
//! Kronrod value that is kept is far closer, and on the analytic primitives
//! the result agrees with the closed forms to rounding.
//!
//! - **Exact, up to rounding:** wherever the integrand is a polynomial within
//!   the degree the fifteen-point rule integrates exactly. That is every
//!   moment of a planar face with straight edges, whose area element is
//!   constant, and the volume moments of a low-degree polynomial patch with
//!   polynomial pcurves.
//! - **To the tolerance:** everything else. A curved face's area element is a
//!   square root, and conics and rational patches make the volume integrand
//!   rational, but both are analytic between breakpoints and converge
//!   spectrally.
//! - **To the pcurves:** the domain integrated over is the one the pcurves
//!   trim. Where they are fitted -- intersection curves after a Boolean,
//!   splines read from a file -- a pcurve that strays `δ` from its edge moves
//!   the area by about `δ` times the edge's length, and no tolerance here
//!   reaches past that.
//!
//! An integral that does not settle within its bisection budget is refused
//! with [`MeasureError::FaceUnconverged`] or [`MeasureError::EdgeUnconverged`]
//! rather than returned at whatever accuracy it reached.
//!
//! Validation and Boolean shell assembly ask only for the sign and rough size
//! of a shell's volume, and run the same face integral at a fixed low order.

mod green;
mod integrate;
mod quadrature;
mod types;

pub use types::{
    Area, Inertia, Length, LinearProperties, PrincipalInertia, SurfaceProperties, Volume,
    VolumeProperties,
};

pub(crate) use integrate::{
    combine_surface_properties, face_surface_properties, linear_properties_for_edge,
    linear_properties_for_edges, sheet_surface_properties, signed_shell_volume,
    signed_volume_for_faces, solid_volume_properties,
};

pub use integrate::MeasureError;
