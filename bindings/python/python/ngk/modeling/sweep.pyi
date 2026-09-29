from ..geometry import Axis, Vector
from ..topology import Edge, Face, Profile, Sheet, Solid


def extrude_profile(profile: Profile, direction: Vector) -> Sheet: ...
def extrude_face(face: Face, direction: Vector) -> Solid: ...


class SweepOptions:
    def __init__(
        self,
        frame: str = "parallel",
        axis: Axis | None = None,
        transition: str = "smooth",
        samples_per_segment: int = 8,
    ) -> None: ...


def face_along_edge(face: Face, spine: Edge, options: SweepOptions | None = None) -> Solid: ...
def face_along_profile(face: Face, spine: Profile, options: SweepOptions | None = None) -> Solid: ...
