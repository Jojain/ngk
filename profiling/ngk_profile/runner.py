"""Runs an example's `main()` repeatedly, so a sampling profiler gets samples.

A single `tie_plate.py` run is well under a second once the kernel is built
optimised, and a good part of that is interpreter startup and the `import ngk`.
Repeating `main()` in one process pushes the kernel work far enough above that
floor to dominate the profile; the startup cost stays a fixed one-off that a
range selection in the profiler UI can exclude.

Used by `profile` when repeating a Python script's main().
"""

import runpy
import sys


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: profile_runner.py <script.py> <count>")

    script, count = sys.argv[1], int(sys.argv[2])

    # `run_path` executes the module body under a name that is not "__main__",
    # so the script's own entry guard stays shut and `main()` runs only here.
    namespace = runpy.run_path(script)
    entry = namespace.get("main")
    if entry is None:
        raise SystemExit(f"{script} defines no main() to repeat")

    for _ in range(count):
        entry()


if __name__ == "__main__":
    main()
