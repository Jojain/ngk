#!/usr/bin/env python3
"""Run brep-bench on this NGK tree and fail on any regression.

    python3 brep-bench/gate.py                 # the whole suite, against results/ngk.json
    python3 brep-bench/gate.py --case boolean/ # part of it
    python3 brep-bench/gate.py --update        # the whole suite, then make it the new baseline

brep-bench (https://github.com/bmander/brep-bench) is fetched at the commit in `UPSTREAM` into
`.upstream/`, which is not tracked. Its harness and cases are used as they are; only its
`kernels.json` is replaced by one naming NGK alone, pointing at the runner in `kernels/ngk`, which
is built against this tree. A case is a regression when the baseline graded it `pass` (or `built`)
and this run does not. Standard library only (Python 3.11 or later).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
UPSTREAM = os.path.join(HERE, ".upstream")
RUNNER = os.path.join(HERE, "kernels", "ngk")
BASELINE = os.path.join(HERE, "results", "ngk.json")
GOOD = {"pass", "built"}
LINE = re.compile(r"^ngk\s+(\S+)\s+(\S+)\s*(.*)$")


def run(cmd, **kw):
    print("+", " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=True, **kw)


def fetch():
    """`.upstream/` at the pinned commit, cloned once and moved when the pin moves."""
    with open(os.path.join(HERE, "UPSTREAM")) as f:
        commit = f.read().strip()
    if not os.path.isdir(os.path.join(UPSTREAM, ".git")):
        run(["git", "clone", "--quiet", "https://github.com/bmander/brep-bench", UPSTREAM])
    head = subprocess.run(["git", "-C", UPSTREAM, "rev-parse", "HEAD"], capture_output=True,
                          text=True).stdout.strip()
    if head != commit:
        run(["git", "-C", UPSTREAM, "fetch", "--quiet", "origin"])
        run(["git", "-C", UPSTREAM, "checkout", "--quiet", "--force", commit])


def build():
    run(["cargo", "build", "--release", "--manifest-path", os.path.join(RUNNER, "Cargo.toml")])
    exe = os.path.join(RUNNER, "target", "release", "brep-bench-ngk")
    if os.name == "nt":
        exe += ".exe"
    entry = {
        "name": "ngk",
        "kernel": "https://github.com/Jojain/ngk",
        "license": "MIT",
        "backend": "Rust",
        "frontend": "Rust",
        "command": [exe],
        "version": [exe, "--version"],
        "source": os.path.join(RUNNER, "src", "cases", "{category}", "{name_snake}.rs"),
    }
    with open(os.path.join(UPSTREAM, "kernels.json"), "w") as f:
        json.dump({"kernels": [entry]}, f, indent=2)


def bench(case, timeout):
    """Each case's outcome and reason, as the harness graded them."""
    cmd = [sys.executable, os.path.join(UPSTREAM, "harness", "run.py"), "--kernel", "ngk",
           "--timeout", str(timeout)]
    if case:
        cmd += ["--case", case]
    print("+", " ".join(cmd), flush=True)
    out = subprocess.run(cmd, capture_output=True, text=True, cwd=UPSTREAM)
    if out.returncode != 0:
        sys.stderr.write(out.stdout + out.stderr)
        raise SystemExit("the harness failed")
    rows = {}
    for line in out.stdout.splitlines():
        m = LINE.match(line)
        if m and "/" in m.group(2):
            rows[m.group(2)] = (m.group(1), m.group(3).strip())
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="", help="only the cases whose id starts with this")
    ap.add_argument("--update", action="store_true", help="make this run the new baseline")
    ap.add_argument("--timeout", type=float, default=60)
    args = ap.parse_args()
    if args.update and args.case:
        raise SystemExit("--update needs the whole suite")
    if not shutil.which("cargo"):
        raise SystemExit("cargo not found")

    fetch()
    build()
    rows = bench(args.case, args.timeout)

    with open(BASELINE) as f:
        before = {r["case"]: r for r in json.load(f)["cases"]}
    regressed, improved, changed = [], [], []
    for cid, (outcome, why) in sorted(rows.items()):
        old = before.get(cid, {}).get("outcome", "missing")
        if old in GOOD and outcome not in GOOD:
            regressed.append((cid, old, outcome, why))
        elif old not in GOOD and outcome in GOOD:
            improved.append((cid, old, outcome, why))
        elif old != outcome:
            changed.append((cid, old, outcome, why))

    tally = {}
    for outcome, _ in rows.values():
        tally[outcome] = tally.get(outcome, 0) + 1
    print(f"\n{len(rows)} cases: {tally}")
    for title, group in (("now passing", improved), ("changed outcome", changed),
                         ("REGRESSED", regressed)):
        if group:
            print(f"\n{title} ({len(group)}):")
            for cid, old, new, why in group:
                print(f"  {cid}: {old} -> {new}" + (f"  ({why})" if why and new not in GOOD else ""))

    if args.update:
        shutil.copy(os.path.join(UPSTREAM, "results", "ngk.json"), BASELINE)
        print(f"\nbaseline updated: {os.path.relpath(BASELINE)}")
    if regressed:
        raise SystemExit(f"\n{len(regressed)} case(s) regressed")


if __name__ == "__main__":
    main()
