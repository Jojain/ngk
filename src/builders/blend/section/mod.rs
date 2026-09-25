//! Each selected edge's blend geometry, independent of how its ends are treated.
//!
//! A section is the surface the blend face lies on and, on each of the edge's
//! two faces, the rail the face now ends at. It is solved from a table keyed
//! by the two faces' supports, the edge's curve and the law, closed form
//! first — the same shape as the analytic intersection table — and nothing
//! downstream needs to know which row answered: vertex treatments read the
//! [`SectionForm`] to answer in closed form and refuse a form they do not know.
//!
//! Every pair of faces and every edge curve the closed forms leave has one
//! general row: the cross-section is solved at samples along the edge —
//! the ball touching both faces for a fillet, the points a set distance from
//! the edge for a chamfer — and skinned into a NURBS blend.

mod contact;
mod crease;
mod edge_section;
mod lines;
mod planar;
mod revolved;
mod swept;
mod translated;

pub(crate) use crease::loop_darts;
pub(crate) use edge_section::{EdgeSection, SectionForm, solve_section};
pub(crate) use lines::{is_straight, line_line};
