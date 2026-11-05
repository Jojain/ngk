#!/usr/bin/env python3
"""Run the pinned OCCT cases with the local OCP bindings, including OCP 7.9.

OCP 8 names its collections through OCP.collections; OCP 7.9 exposes the same
OCCT collection types in TColStd, TColgp and TopTools. Aliases translate those
imports without changing the case implementations or substituting geometry.
"""
from pathlib import Path
import importlib.util
import runpy
import sys
import types

import OCP

if len(sys.argv) > 1 and sys.argv[1] == "--version":
    print(f"OCCT via OCP {OCP.__version__} (pinned brep-bench cases)")
    sys.exit(0)

if importlib.util.find_spec("OCP.collections") is None:
    from OCP.TColStd import TColStd_Array1OfInteger, TColStd_Array1OfReal, TColStd_Array2OfReal
    from OCP.TColgp import TColgp_Array1OfPnt, TColgp_Array2OfPnt
    from OCP.TopTools import TopTools_ListOfShape

    collections = types.ModuleType("OCP.collections")
    collections.Array1_double = TColStd_Array1OfReal
    collections.Array1_int = TColStd_Array1OfInteger
    collections.Array2_double = TColStd_Array2OfReal
    collections.Array1_gp_Pnt = TColgp_Array1OfPnt
    collections.Array2_gp_Pnt = TColgp_Array2OfPnt
    collections.List_TopoDS_Shape = TopTools_ListOfShape
    sys.modules["OCP.collections"] = collections

runner = Path(__file__).resolve().parent / ".upstream" / "kernels" / "occt" / "run.py"
runpy.run_path(str(runner), run_name="__main__")
