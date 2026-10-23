mod curve;
mod differentiate;
mod evaluate;
mod prepare;
mod reduce;
mod solve;

pub(crate) use solve::solve;

#[cfg(test)]
#[path = "../../../tests/solver/derivatives.rs"]
mod derivatives;
