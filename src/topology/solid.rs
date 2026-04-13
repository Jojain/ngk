use super::shell::ShellRef;

pub type SolidId = usize;

#[derive(Clone)]
pub struct Solid<'a> {
    id: SolidId,
    outer: ShellRef<'a>,
    inners: Option<Vec<ShellRef<'a>>>,
}

impl<'a> Solid<'a> {
    pub fn new(id: SolidId, outer: ShellRef<'a>, inners: Option<Vec<ShellRef<'a>>>) -> Self {
        Self { id, outer, inners }
    }

    pub fn shells(&self) -> Vec<&ShellRef<'a>> {
        let mut shells = vec![&self.outer];
        if let Some(inners) = &self.inners {
            shells.extend(inners.iter().map(|inner| inner));
        }
        shells
    }
}
