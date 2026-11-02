from collections.abc import Sequence

from ..topology import Edge, Face, Model, Profile, Sheet, Solid

class FaceBooleanResult:
    @property
    def model(self) -> Model: ...
    def faces(self) -> list[Face]: ...

# Every Boolean takes one target and one tool or a sequence of tools, and
# evaluates the whole set at once: cut = target - (t1 | ... | tn),
# fuse = target | t1 | ... | tn, intersect = target & t1 & ... & tn. The order
# of the tools does not matter. The result lives in a copy of the target's
# model, kept whole, so the target's model is never mutated and every other
# entity of it keeps its key. A tool from that same model is used in place; any
# other tool contributes only its own solid or face. A tool passed twice is one
# tool.
#
# A face Boolean with no area left (a face cut by one covering it, disjoint
# faces intersected) returns a result whose faces() is empty.
def fuse(target: Solid, tools: Solid | Sequence[Solid]) -> Solid: ...
def cut(target: Solid, tools: Solid | Sequence[Solid]) -> Solid: ...
def intersect(target: Solid, tools: Solid | Sequence[Solid]) -> Solid: ...
def fuse_faces(target: Face, tools: Face | Sequence[Face]) -> FaceBooleanResult: ...
def cut_faces(target: Face, tools: Face | Sequence[Face]) -> FaceBooleanResult: ...
def intersect_faces(target: Face, tools: Face | Sequence[Face]) -> FaceBooleanResult: ...

class SplitResult[T: (Solid, Sheet, Face, Profile, Edge)]:
    @property
    def model(self) -> Model: ...
    # The part on the side the sheet's normals point to.
    @property
    def front(self) -> T: ...
    # The part behind the sheet.
    @property
    def back(self) -> T: ...

# Cuts the target along the sheet, into two parts of the target's own kind:
# any shape but a vertex, which has no side to fall on. The sheet must reach
# past the target everywhere it crosses it; a sheet that misses the target,
# stops inside it or leaves one side empty raises, as does a side falling in
# several pieces. A solid or a sheet is cut in one copy of its model; a face,
# a profile or an edge in a model of its own, since it may be part of
# something larger that cutting it in place would tear. The inputs are
# unchanged.
def split[T: (Solid, Sheet, Face, Profile, Edge)](target: T, tool: Sheet) -> SplitResult[T]: ...
