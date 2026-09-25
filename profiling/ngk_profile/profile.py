#!/usr/bin/env python3
"""Profiles a Python script or a Rust Cargo example, with Rust frames named.

Builds the Python extension or Rust example with the `profiling` Cargo profile
(release optimisation, symbols kept) and records a run of the target.

`maturin develop` builds `dev` by default, and `[profile.dev]` is `opt-level = 0`
-- a profile of that build measures bounds checks and un-inlined nalgebra
accessors, not the kernel. This script never records that build.

Two recorders, because they see different things:

* `samply` records native frames and opens the Firefox Profiler: call tree,
  inverted call tree, flame graph, and a timeline that can be range-selected to
  one phase of the run. Rust frames are named down to their generic arguments.
  This is the one for "where does the kernel spend its time".
* `py-spy` walks the CPython frames, so hot lines of the .py itself are named.
  Its `--native` mode resolves Windows DLLs from the export table rather than
  the PDB, so on Windows it cannot see into the kernel -- all of ngk arrives as
  one `PyInit__ngk` frame. Use it for Python scripts, not Rust examples.

Recording needs the privilege to profile the system: on Windows samply traces
through ETW and raises a UAC prompt per run unless the terminal is already
elevated, or the account holds "Profile system performance"
(SeSystemProfilePrivilege, granted in secpol.msc). Only viewing a recording is
unprivileged.

Only the Windows path is exercised here; the POSIX branches are written from
samply's and uv's documented behaviour. On Linux samply records through `perf`,
which usually wants `kernel.perf_event_paranoid` at 1 or lower.

Installed as the `profile` command by `uv tool install --editable ./profiling`.

Examples
--------
    profile tie_plate            # one run of the tie_plate example
    profile tie_plate 20         # twenty runs in the one process
    profile bindings/python/examples/explore_block.py
    profile examples/curved_support.rs
    profile curved_support 20    # twenty executions in one recording
    profile curved_support --no-open
    profile tie_plate --tool py-spy --skip-build
"""

import os
import shutil
import subprocess
import sys
import time
from enum import Enum
from pathlib import Path
from typing import Annotated

import typer

from .fix_debug_paths import absolutize_file

# .../profiling/ngk_profile/profile.py -> the repo. An editable install keeps
# __file__ inside the checkout, so the command always drives the repo it was
# installed from.
REPO_ROOT = Path(__file__).resolve().parents[2]
IS_WINDOWS = os.name == "nt"

# samply honours --main-thread-only on Windows and macOS only. It matters: an
# ETW trace of every thread on the machine runs to hundreds of MB for a run of a
# few seconds, and takes longer to post-process than the run itself took.
MAIN_THREAD_ONLY = sys.platform in ("win32", "darwin")

# Microsoft's public symbol server names the Windows frames. Without it ntdll
# arrives as `fun_26e50` rather than `RtlpLowFragHeapAllocFromContext`, which is
# the difference between "a third of the run is somewhere in ntdll" and "a third
# of the run is the heap". PDBs land in samply's cache, so only the first look
# at a given Windows build pays for it.
WINDOWS_SYMBOL_SERVER = "https://msdl.microsoft.com/download/symbols"
app = typer.Typer(help=__doc__, no_args_is_help=True)


class Profiler(str, Enum):
    """The supported recording backends."""

    samply = "samply"
    py_spy = "py-spy"


def venv_paths(venv):
    """Returns (python, bin_dir) for a virtualenv on this platform."""
    bin_dir = venv / ("Scripts" if IS_WINDOWS else "bin")
    return bin_dir / ("python.exe" if IS_WINDOWS else "python"), bin_dir


def run(argv, **kwargs):
    """Runs a command, failing the script with its exit code if it fails."""
    printable = " ".join(str(a) for a in argv)
    completed = subprocess.run([str(a) for a in argv], check=False, **kwargs)
    if completed.returncode != 0:
        raise SystemExit(f"failed ({completed.returncode}): {printable}")
    return completed


def tool_env():
    """PATH with the places cargo and uv install their binaries."""
    env = os.environ.copy()
    home = Path.home()
    extra = [home / ".cargo" / "bin", home / ".local" / "bin"]
    env["PATH"] = os.pathsep.join([str(p) for p in extra] + [env.get("PATH", "")])
    return env


def find_tool(name, env):
    """Resolves a tool to a full path, or says how to install it.

    Windows resolves a child's executable against the *parent's* PATH, not the
    one handed to it, so a tool in a directory this script added -- `py-spy` in
    uv's `~/.local/bin`, which uv itself warns is off PATH -- is not found by
    name alone. Looking it up here is what makes the added entries count.
    """
    found = shutil.which(name, path=env["PATH"])
    if found is None:
        hint = {
            "samply": "cargo install samply",
            "py-spy": "uv tool install py-spy",
            "maturin": "uv tool install maturin",
        }.get(name, f"install {name}")
        raise SystemExit(f"{name} not found on PATH; install it with: {hint}")
    return found


EXAMPLES_DIR = REPO_ROOT / "bindings" / "python" / "examples"
RUST_EXAMPLES_DIR = REPO_ROOT / "examples"


def resolve_script(name):
    """Accepts a Python script or Rust Cargo example and returns its absolute path.

    Absolute paths keep Python scripts valid when the recording runs from the
    repository root instead of the directory where the command was typed.
    Rust files must be direct Cargo examples under `examples/`.
    """
    candidates = (
        Path(name),
        REPO_ROOT / name,
        EXAMPLES_DIR / name,
        EXAMPLES_DIR / f"{name}.py",
        RUST_EXAMPLES_DIR / name,
        RUST_EXAMPLES_DIR / f"{name}.rs",
    )
    for candidate in candidates:
        if not candidate.is_file():
            continue
        script = candidate.resolve()
        if script.suffix == ".py":
            return script
        if script.suffix == ".rs" and script.parent == RUST_EXAMPLES_DIR.resolve():
            return script
        raise SystemExit(f"unsupported script {script}; Rust files must be in {RUST_EXAMPLES_DIR}")

    known = sorted(
        [p.stem for p in EXAMPLES_DIR.glob("*.py")]
        + [p.stem for p in RUST_EXAMPLES_DIR.glob("*.rs")]
    )
    raise SystemExit(f"no script {name!r}; examples are: {', '.join(known)}")


@app.command()
def profile(
    script_name: Annotated[
        str,
        typer.Argument(help="Python script or Rust example path or bare name"),
    ],
    repeat: Annotated[
        int,
        typer.Argument(help="Python main() calls or Rust executable runs", min=1),
    ] = 1,
    tool: Annotated[Profiler, typer.Option(help="recording backend")] = Profiler.samply,
    rate: Annotated[int, typer.Option(help="samples per second", min=1)] = 1000,
    skip_build: Annotated[bool, typer.Option(help="do not rebuild the target")] = False,
    no_open: Annotated[bool, typer.Option(help="leave the recording on disk")] = False,
) -> None:
    """Build and record one Python script or Rust Cargo example."""
    script = resolve_script(script_name)
    is_rust = script.suffix == ".rs"
    if is_rust and tool is Profiler.py_spy:
        raise SystemExit("py-spy only profiles Python scripts; use samply for Rust examples")

    env = tool_env()

    env.setdefault("UV_CACHE_DIR", str(REPO_ROOT / ".uv-cache"))
    env.setdefault("UV_PYTHON_INSTALL_DIR", str(REPO_ROOT / ".uv-python"))

    if is_rust:
        executable = REPO_ROOT / "target" / "profiling" / "examples" / (
            script.stem + (".exe" if IS_WINDOWS else "")
        )
        if not skip_build:
            run(
                [find_tool("cargo", env), "build", "--profile", "profiling", "--example", script.stem],
                cwd=REPO_ROOT, env=env,
            )
        elif not executable.is_file():
            raise SystemExit(f"no executable at {executable}; build it without --skip-build")
        target = [executable]
    else:
        venv = REPO_ROOT / ".venv"
        env["VIRTUAL_ENV"] = str(venv)
        python, _ = venv_paths(venv)
        if not python.exists():
            raise SystemExit(f"no interpreter at {python}; run build.ps1 or create .venv first")
        if not skip_build:
            run([find_tool("maturin", env), "develop", "--profile", "profiling"], cwd=REPO_ROOT, env=env)
        if repeat > 1:
            target = [python, Path(__file__).resolve().parent / "runner.py", script, str(repeat)]
        else:
            target = [python, script]

    out_dir = REPO_ROOT / "target" / "profiles"
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{script.stem}-{time.strftime('%Y%m%d-%H%M%S')}"

    if tool is Profiler.samply:
        out = out_dir / f"{stem}.json.gz"

        # --save-only records without holding the shell on a local server, which
        # leaves the profile on disk to be patched before anything reads it.
        samply = find_tool("samply", env)
        record = [samply, "record", "--rate", str(rate), "--save-only", "--no-open"]
        if is_rust and repeat > 1:
            record += ["--iteration-count", str(repeat)]
        if MAIN_THREAD_ONLY:
            record.append("--main-thread-only")
        record += ["--output", out, "--", *target]
        run(record, cwd=REPO_ROOT, env=env)

        fixed = absolutize_file(out)
        print(f"resolved {fixed} local debug-info path(s) in {out.name}")

        # samply exits 0 even when the target exits early, so a missing local
        # PDB path warrants checking the recording before trusting its frames.
        if IS_WINDOWS and fixed == 0:
            print()
            if is_rust:
                print("warning: no local PDB path was resolved; check the target output")
                print("         and Rust symbols in the recording.")
            else:
                print("warning: no ngk.pdb in the recording -- the target never loaded")
                print("         the kernel. Check the lines above for how the run")
                print("         failed; a script path is resolved against the repo")
                print("         root, not the directory you typed the command in.")

        load = [samply, "load"]
        if IS_WINDOWS:
            load += ["--windows-symbol-server", WINDOWS_SYMBOL_SERVER]
        load.append(out)

        print(f"\nRecorded {out}")
        if no_open:
            # Printed with the plain tool name rather than the resolved path, so
            # the line is something to read and retype.
            hint = " ".join(["samply", *(str(a) for a in load[1:])])
            print(f"View it with:  {hint}")
        else:
            print("Opening the Firefox Profiler; Ctrl+C here when you are done looking.")
            run(load, cwd=REPO_ROOT, env=env)
    else:
        out = out_dir / f"{stem}.speedscope.json"
        spy_python = python

        if IS_WINDOWS:
            # py-spy cannot read the uv trampoline at .venv\Scripts\python.exe,
            # so it runs the interpreter that one launches, with the venv's paths
            # supplied directly rather than through the trampoline. POSIX venvs
            # symlink the interpreter, which py-spy reads without help.
            spy_python = subprocess.run(
                [str(python), "-c", "import sys; print(sys._base_executable)"],
                capture_output=True, text=True, check=True,
            ).stdout.strip()
            env["PYTHONPATH"] = os.pathsep.join(
                [str(venv / "Lib" / "site-packages"), str(REPO_ROOT / "bindings" / "python" / "python")]
            )

        run(
            [find_tool("py-spy", env), "record", "--rate", str(rate), "--format", "speedscope",
             "--output", out, "--", spy_python, *target],
            cwd=REPO_ROOT, env=env,
        )

        print(f"\nRecorded {out}")
        print("Drop it on https://speedscope.app (it renders in the page; nothing is uploaded).")


def main() -> None:
    """Run the profiling command-line application."""
    app()


if __name__ == "__main__":
    main()
