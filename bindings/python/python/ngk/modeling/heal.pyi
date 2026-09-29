from ..topology import Solid

class HealingOptions:
    def __init__(
        self,
        remove_redundant_vertices: bool = True,
        remove_redundant_edges: bool = True,
        remove_seams: bool = True,
        remove_filled_inner_loops: bool = True,
        linear_tolerance: float | None = None,
        angular_tolerance: float | None = None,
        max_iterations: int = 16,
    ) -> None: ...
    @staticmethod
    def seams_only() -> HealingOptions: ...

class HealingReport:
    @property
    def changes(self) -> int: ...
    @property
    def iterations(self) -> int: ...
    @property
    def skipped(self) -> list[str]: ...

class HealingResult:
    @property
    def solid(self) -> Solid: ...
    @property
    def report(self) -> HealingReport: ...

def solid(shape: Solid, options: HealingOptions | None = None) -> HealingResult: ...
