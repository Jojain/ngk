from os import PathLike

from ..topology import Solid

def stl_to_bytes(solid: Solid, segments: int = 64) -> bytes: ...
def write_stl(solid: Solid, path: str | PathLike[str] | None = None, segments: int = 64) -> str: ...

__all__ = [
    "stl_to_bytes",
    "write_stl",
]
