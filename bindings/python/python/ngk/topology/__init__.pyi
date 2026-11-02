from ..core import (
    Edge as Edge,
    Face as Face,
    Loop as Loop,
    Profile as Profile,
    Shell as Shell,
    Sheet as Sheet,
    Solid as Solid,
    Vertex as Vertex,
)

type Shape = Vertex | Edge | Profile | Face | Sheet | Solid

__all__: list[str]
