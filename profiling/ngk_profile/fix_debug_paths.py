"""Rewrites a samply profile's relative debug-info names into absolute paths.

Cargo links MSVC binaries with `/PDBALTPATH:%_PDB%`, so the debug directory of
`_ngk.pyd` records the bare name `ngk.pdb` rather than a path. samply resolves a
`debugPath` only when it is absolute, so it finds no PDB, falls back to the
export table -- of which a cdylib has exactly one, `PyInit__ngk` -- and the whole
kernel arrives in the profiler as `fun_1a8b0` and friends.

Every module whose `debugPath` is a bare filename gets it resolved against the
directory its binary was loaded from, which is where cargo and maturin both
leave the PDB. Modules whose symbols live elsewhere (Windows' own DLLs, CPython)
already carry an absolute path and are left alone, as are ELF and Mach-O builds,
which carry their debug info in the binary and so name no separate file. That
makes this a no-op everywhere except the case it exists for.
"""

import gzip
import json
import os
import sys


def absolutize(profile):
    """Returns the number of modules whose debugPath was made absolute."""
    fixed = 0
    for lib in profile.get("libs", []):
        debug_path = lib.get("debugPath") or ""
        if not debug_path or os.path.dirname(debug_path):
            continue
        candidate = os.path.join(os.path.dirname(lib.get("path") or ""), debug_path)
        if os.path.exists(candidate):
            lib["debugPath"] = os.path.abspath(candidate)
            fixed += 1
    return fixed


def absolutize_file(path):
    """Patches a recorded profile in place, returning how many paths it fixed."""
    path = str(path)
    opener = gzip.open if path.endswith(".gz") else open

    with opener(path, "rt", encoding="utf-8") as handle:
        profile = json.load(handle)

    fixed = absolutize(profile)

    with opener(path, "wt", encoding="utf-8") as handle:
        json.dump(profile, handle)

    return fixed


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: fix_debug_paths.py <profile.json.gz>")

    path = sys.argv[1]
    print(f"resolved {absolutize_file(path)} local debug-info path(s) in {os.path.basename(path)}")


if __name__ == "__main__":
    main()
