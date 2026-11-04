# brep-bench

[brep-bench](https://github.com/bmander/brep-bench) is a neutral conformance suite for B-rep
kernels: 231 cases described in words, each with its volume and area in closed form, graded the
same way for every kernel (volume and area within 1e-6, and the written triangles closed,
consistently oriented and bounding the expected volume). Results of every kernel:
https://bmander.github.io/brep-bench/

This folder holds NGK's runner and its implementation of every case, and the gate that runs the
suite on this tree:

```
python3 brep-bench/gate.py                  # whole suite, fails on any regression
python3 brep-bench/gate.py --case boolean/  # part of it
python3 brep-bench/gate.py --update         # whole suite, then make it the new baseline
```

- `kernels/ngk/`: the runner (`brep-bench-ngk CASE OUT.stl`), one file per case under
  `src/cases/<category>/<name>.rs`, shared helpers in `src/bench.rs`, and `workarounds.toml`, the
  cases built by working around a feature NGK lacks. It builds against this tree.
- `results/ngk.json`: the baseline the gate compares with. Update it (`--update`) in the change
  that moves cases forward.
- `UPSTREAM`: the brep-bench commit whose harness and cases are used. The gate clones it into
  `.upstream/` (not tracked) and gives it a `kernels.json` naming NGK alone.

The gate runs in CI (`.github/workflows/brep-bench.yml`) on pull requests touching `src/` and on
pushes to `master`. The work it tracks is planned in `plan/robustness/`.
