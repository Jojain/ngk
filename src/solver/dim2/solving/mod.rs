mod curve;
mod evaluate;
mod prepare;
mod reduce;
mod solve;
mod vector;

pub(crate) use solve::solve;

#[cfg(test)]
#[path = "../../../../tests/solver/derivatives.rs"]
mod derivatives;
