from os import PathLike

from ..core import StepImport
from ..topology import Solid

def write_step(solid: Solid, path: str | PathLike[str] | None = None, name: str | None = None) -> str: ...
def step_to_string(solid: Solid, name: str | None = None) -> str: ...
def read_step(path: str | PathLike[str], strict: bool = False) -> StepImport: ...
def step_from_string(text: str, strict: bool = False) -> StepImport: ...

__all__ = [
    "StepImport",
    "read_step",
    "step_from_string",
    "step_to_string",
    "write_step",
]
