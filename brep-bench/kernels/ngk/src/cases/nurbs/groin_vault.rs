//! The common part of the barrel-vaulted block of `nurbs/vault-tangent-plane` and the same block
//! turned a quarter turn about z: a groin vault.
use super::vault_tangent_plane::vault;
use crate::bench::{common, deg};
use crate::{Refusal, Solid};
use ngk::geometry::Axis3;

pub fn build() -> Result<Solid, Refusal> {
    let turned = vault()?.rotated(Axis3::z(), deg(90.0));
    common(vault()?, &turned)
}
