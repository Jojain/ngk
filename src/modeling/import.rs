//! Copying an operation's secondary inputs into its primary input's model.

use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::{ModelEdit, ModelEditError};

/// Copies each tool into `model` in one transaction and returns, for every
/// tool, the key of its copy.
///
/// A tool passed more than once — `same` says when two are the same — is
/// copied once, and every place it was passed gets that one copy's key, so the
/// operation sees one operand where the caller named one. `copy` merges one
/// tool and returns the key it now has.
pub(crate) fn import_tools<P: Payload, T, K: Copy>(
    model: &mut Model<P>,
    tools: &[T],
    same: impl Fn(&T, &T) -> bool,
    copy: impl Fn(&mut ModelEdit<'_, P>, &T) -> K,
) -> Result<Vec<K>, ModelEditError> {
    model.transaction(|edit| {
        let mut keys: Vec<K> = Vec::with_capacity(tools.len());
        for (index, tool) in tools.iter().enumerate() {
            let key = match tools[..index]
                .iter()
                .position(|earlier| same(earlier, tool))
            {
                Some(earlier) => keys[earlier],
                None => copy(edit, tool),
            };
            keys.push(key);
        }
        Ok(keys)
    })
}
