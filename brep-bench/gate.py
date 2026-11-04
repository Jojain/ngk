#!/usr/bin/env python3
"""Run brep-bench on this NGK tree and fail on any regression.

    python3 brep-bench/gate.py                 # the whole suite, against results/ngk.json
    python3 brep-bench/gate.py --case boolean/ # part of it
    python3 brep-bench/gate.py --update        # the whole suite, then make it the new baseline

brep-bench (https://github.com/bmander/brep-bench) is fetched at the commit in `UPSTREAM` into
`.upstream/`, which is not tracked. Its harness and cases are used as they are; only its
`kernels.json` is replaced by one naming NGK alone, pointing at the runner in `kernels/ngk`, which
is built against this tree. A case is a regression when the baseline graded it `pass` (or `built`)
and this run does not, or when a whole run no longer reports it. The run ends with the cases that
moved either way and a tally; the exit status is 1 on a regression. `--update` writes the baseline
without timings or dates, its cases in id order, so the same tree gives the same file; it refuses
to run over a regression unless `--accept-regressions` says so. Standard library only (Python 3.11
or later).
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
UPSTREAM_URL = os.environ.get("BREP_BENCH_URL", "https://github.com/bmander/brep-bench")
ORDER = ["pass", "built", "wrong", "refused", "crashed"]


def run(cmd, **kw):
    print("+", " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=True, **kw)


def fetch():
    """`.upstream/` at the pinned commit: fetched by hash, nothing else of the history."""
    with open(os.path.join(HERE, "UPSTREAM")) as f:
        commit = f.read().strip()
    if not os.path.isdir(os.path.join(UPSTREAM, ".git")):
        os.makedirs(UPSTREAM, exist_ok=True)
        run(["git", "-C", UPSTREAM, "init", "--quiet"])
        run(["git", "-C", UPSTREAM, "remote", "add", "origin", UPSTREAM_URL])
    head = subprocess.run(["git", "-C", UPSTREAM, "rev-parse", "HEAD"], capture_output=True,
                          text=True).stdout.strip()
    if head != commit:
        run(["git", "-C", UPSTREAM, "fetch", "--quiet", "--depth", "1", "origin", commit])
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


def tally_of(outcomes):
    counts = {}
    for outcome in outcomes:
        counts[outcome] = counts.get(outcome, 0) + 1
    known = [k for k in ORDER if k in counts]
    return {k: counts[k] for k in known + sorted(set(counts) - set(ORDER))}


def stable_why(why):
    """A reason without what differs between runs and machines: a panic's place is relative to
    the tree, and the key of the cell a message names depends on the order the kernel visited
    cells in."""
    why = re.sub(r"panicked at \S*?(?=src/)", "panicked at ", why)
    return re.sub(r"(\w+Key)\(\d+v\d+\)", r"\1(_)", why)


def write_baseline():
    """`results/ngk.json` from the harness's own result file: its cases in id order, without how
    long each took, when it ran or which version built it, none of which a diff should carry."""
    fresh = os.path.join(UPSTREAM, "results", "ngk.json")
    if not os.path.exists(fresh):
        raise SystemExit("the harness wrote no result file")
    with open(fresh) as f:
        result = json.load(f)
    cases = []
    for c in sorted(result["cases"], key=lambda c: c["case"]):
        c = {k: v for k, v in c.items() if k != "seconds"}
        c["why"] = stable_why(c.get("why", ""))
        cases.append(c)
    out = {
        "kernel": result["kernel"],
        "measured": result["measured"],
        "tally": tally_of(c["outcome"] for c in cases),
        "cases": cases,
    }
    with open(BASELINE, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="", help="only the cases whose id starts with this")
    ap.add_argument("--update", action="store_true", help="make this run the new baseline")
    ap.add_argument("--accept-regressions", action="store_true",
                    help="with --update, write the baseline although a case regressed")
    ap.add_argument("--timeout", type=float, default=60)
    args = ap.parse_args()
    if args.update and args.case:
        raise SystemExit("--update needs the whole suite")
    if not shutil.which("cargo"):
        raise SystemExit("cargo not found")

    fetch()
    build()
    if os.path.exists(os.path.join(UPSTREAM, "results", "ngk.json")):
        os.remove(os.path.join(UPSTREAM, "results", "ngk.json"))
    rows = bench(args.case, args.timeout)

    with open(BASELINE) as f:
        before = {r["case"]: r for r in json.load(f)["cases"]}
    wanted = {cid for cid in before if cid.startswith(args.case)}
    regressed, improved, changed = [], [], []
    for cid in sorted(wanted | set(rows)):
        old = before.get(cid, {}).get("outcome", "new")
        outcome, why = rows.get(cid, ("missing", "the harness reported no result"))
        if old in GOOD and outcome not in GOOD:
            regressed.append((cid, old, outcome, why))
        elif old not in GOOD and outcome in GOOD:
            improved.append((cid, old, outcome, why))
        elif old != outcome:
            changed.append((cid, old, outcome, why))

    print(f"\nbaseline ({len(wanted)} cases): {tally_of(before[c]['outcome'] for c in wanted)}")
    print(f"this run ({len(rows)} cases): {tally_of(o for o, _ in rows.values())}")
    for title, group in (("now passing", improved), ("changed outcome", changed),
                         ("REGRESSED", regressed)):
        if group:
            print(f"\n{title} ({len(group)}):")
            for cid, old, new, why in group:
                print(f"  {cid}: {old} -> {new}" + (f"  ({why})" if why and new not in GOOD else ""))
    if not (improved or changed or regressed):
        print("no case moved")

    if args.update:
        if regressed and not args.accept_regressions:
            raise SystemExit(f"\n{len(regressed)} case(s) regressed: baseline not updated")
        write_baseline()
        print(f"\nbaseline updated: {os.path.relpath(BASELINE)}")
    if regressed:
        raise SystemExit(f"\n{len(regressed)} case(s) regressed")


if __name__ == "__main__":
    main()
