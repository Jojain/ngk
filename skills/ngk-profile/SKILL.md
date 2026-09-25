---
name: ngk-profile
description: Use when explaining, running, or troubleshooting NGK's `profile` command for Python scripts or Rust Cargo examples, including repeat counts and saved sampling profiles.
---

# NGK profile tool

Use the repository's `profile` command to sample an NGK workload. Its current
implementation is in `profiling/ngk_profile/profile.py`; check `profile --help`
or that file when exact behavior matters.

## Setup and commands

Run from the NGK repository root in PowerShell. If the command is not installed,
install the editable tool once with `uv tool install --editable ./profiling`.
The command builds an optimized target with debug symbols before recording.
Python targets need the repository `.venv` and `maturin`; native recordings need
`samply`. The optional `py-spy` backend needs `py-spy` installed.

```powershell
profile tie_plate
profile tie_plate 20
profile bindings/python/examples/explore_block.py --tool py-spy
profile examples/curved_support.rs 20
profile curved_support 20 --no-open
```

- A bare Python name resolves under `bindings/python/examples/`; an explicit
  `.py` path also works. With a repeat count above one, the script must define
  `main()`, which is called repeatedly in **one Python process**.
- A Rust file must be directly under `examples/` as a Cargo example. A bare
  name also resolves there. The repeat count uses samply's
  `--iteration-count`: it runs the executable that many times in **separate
  processes within one recording**. Files under `src/scripts/` are registry
  entries, not runnable targets for this command.
- `samply` is the default and shows native Rust frames, including Rust code
  called from Python. `--tool py-spy` shows Python frames and is only valid for
  Python scripts; it does not resolve NGK's Rust internals on Windows.
- `--skip-build` reuses the existing profiling build. Use it only when that
  target was already built and is current. `--no-open` saves the recording
  without launching the viewer. `--rate` changes samples per second.

## Reading a recording

Native recordings are saved as `target/profiles/*.json.gz`; the command opens
the Firefox Profiler by default or prints a `samply load` command with
`--no-open`. `py-spy` saves `*.speedscope.json` in the same directory. For a
short run, select the workload interval in the profiler before attributing
startup cost to the kernel.

On Windows, samply records through ETW and may request profiling privileges.
The command also repairs local PDB paths in native recordings. If it reports
no resolved PDB path, inspect the target output and symbol names before
trusting the call tree. A successful build or launched profiler is not evidence
that the workload produced a usable recording; report the saved artifact and
whether its Rust frames are resolved.
