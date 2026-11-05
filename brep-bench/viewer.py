#!/usr/bin/env python3
"""Prepare the dedicated NGK / OCCT comparison page from real harness outputs.

python brep-bench/viewer.py --refresh  # run NGK and the OCP reference, then export
python brep-bench/viewer.py            # export the latest local runs only
"""
import argparse
import ast
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from urllib.parse import quote

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
UPSTREAM = HERE / ".upstream"
OUTPUT = HERE / "viewer-data"


def read_json(path):
    """Read an optional result without substituting another run or kernel."""
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def export_sources(kernel, case_id, copied):
    """Copy the case implementation and its local construction helpers as static text."""
    category, name = case_id.split("/")
    filename = name.replace("-", "_")
    source_root = HERE / "kernels" / "ngk" / "src" if kernel == "ngk" else UPSTREAM / "kernels" / "occt"
    suffix = ".rs" if kernel == "ngk" else ".py"
    case_path = source_root / "cases" / category / (filename + suffix)
    pending = [case_path]
    if kernel == "ngk":
        pending.append(source_root / "bench.rs")
    files, visited = [], set()
    while pending:
        path = pending.pop(0)
        if path in visited or not path.is_file():
            continue
        visited.add(path)
        relative = path.relative_to(source_root)
        target_relative = Path("source") / kernel / relative
        target = OUTPUT / target_relative
        if target not in copied:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
            copied.add(target)
        files.append({"path": relative.as_posix(), "url": quote(target_relative.as_posix(), safe="/"),
                      "language": "rust" if kernel == "ngk" else "python"})
        if kernel == "occt":
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                if isinstance(node, ast.ImportFrom) and node.module:
                    pending.append(source_root / (node.module.split(".")[0] + ".py"))
                elif isinstance(node, ast.Import):
                    pending.extend(source_root / (item.name.split(".")[0] + ".py") for item in node.names)
    return files


def run_reference(python, timeout, env):
    """Register and run the pinned suite's OCCT implementation through OCP."""
    registry_path = UPSTREAM / "kernels.json"
    registry = read_json(registry_path) or {"kernels": []}
    runner = str(HERE / "occt_reference.py")
    reference = {
        "name": "occt", "reference": True,
        "command": [str(python), runner],
        "version": [str(python), runner, "--version"],
        "source": "kernels/occt/cases/{category}/{name_snake}.py",
    }
    registry["kernels"] = [k for k in registry["kernels"] if k["name"] != "occt"] + [reference]
    registry_path.write_text(json.dumps(registry, indent=2), encoding="utf-8")
    subprocess.run([str(python), str(UPSTREAM / "harness" / "run.py"), "--kernel", "occt",
                    "--timeout", str(timeout)], env=env, check=True, cwd=ROOT)


def export():
    """Export all cases and only meshes confirmed by their corresponding result row."""
    sys.path.insert(0, str(UPSTREAM / "harness"))
    from suite import cases

    OUTPUT.mkdir(parents=True, exist_ok=True)
    runs = {name: read_json(UPSTREAM / "results" / f"{name}.json") for name in ("occt", "ngk")}
    indexed = {name: {row["case"]: row for row in (run or {}).get("cases", [])}
               for name, run in runs.items()}
    baseline = read_json(HERE / "results" / "ngk.json")
    baseline_rows = {row["case"]: row for row in (baseline or {}).get("cases", [])}
    entries = []
    copied_sources = set()
    for case in cases():
        cid = case["id"]
        results = {}
        for name in runs:
            row = indexed[name].get(cid)
            result = dict(row) if row else {"outcome": "missing", "why": "This case has not been run."}
            result.pop("case", None)
            result["meshUrl"] = None
            result["sourceFiles"] = export_sources(name, cid, copied_sources)
            source = UPSTREAM / "results" / "meshes" / name / (cid + ".stl")
            # A refused run can leave an older STL on disk. Never show that stale geometry.
            if row and row.get("mesh") and source.is_file():
                relative = Path("meshes") / name / (cid + ".stl")
                target = OUTPUT / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
                result["meshUrl"] = quote(relative.as_posix(), safe="/")
            results[name] = result
        entries.append({
            "id": cid, "title": case["title"], "category": cid.split("/")[0],
            "description": case["description"], "expect": case["expect"],
            "baseline": baseline_rows.get(cid, {}).get("outcome"), "results": results,
        })
    manifest = {
        "schema": 1, "generatedAt": datetime.now(timezone.utc).isoformat(),
        "upstreamCommit": (HERE / "UPSTREAM").read_text().strip(),
        "kernels": {name: {"label": "OCCT / OCP" if name == "occt" else "NGK",
                           "version": (run or {}).get("version"),
                           "run": (run or {}).get("run"),
                           "tally": (run or {}).get("tally", {})} for name, run in runs.items()},
        "cases": entries,
    }
    temporary = OUTPUT / "manifest.tmp"
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(OUTPUT / "manifest.json")
    print(f"Exported {len(entries)} cases to {OUTPUT}")
    for name in runs:
        meshes = sum(bool(c["results"][name]["meshUrl"]) for c in entries)
        print(f"  {name}: {meshes} meshes; {(runs[name] or {}).get('tally', 'not run')}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--refresh", action="store_true", help="run NGK and OCCT before exporting")
    parser.add_argument("--run-reference", action="store_true", help="run OCCT before exporting")
    local_python = ROOT / ".venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    parser.add_argument("--reference-python", type=Path,
                        default=local_python if local_python.is_file() else Path(sys.executable),
                        help="Python with cadquery-ocp installed")
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args()
    if args.refresh or args.run_reference:
        temporary = ROOT / "target" / "benchmark-temp"
        temporary.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, TEMP=str(temporary), TMP=str(temporary), PYTHONUTF8="1")
        if args.refresh:
            subprocess.run([sys.executable, str(HERE / "gate.py"), "--timeout", str(args.timeout)],
                           env=env, check=True, cwd=ROOT)
        run_reference(args.reference_python.resolve(), args.timeout, env)
    if not (UPSTREAM / "harness" / "suite.py").exists():
        parser.error("Run python brep-bench/gate.py first to fetch the pinned benchmark.")
    export()


if __name__ == "__main__":
    main()
