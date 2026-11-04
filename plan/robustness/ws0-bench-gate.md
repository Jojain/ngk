# WS 0: brep-bench and the probe corpus as a regression gate

**Owns:** `brep-bench/`, CI workflow. **Depends on:** nothing. **Size:** small.

## Why first

Every other workstream proves its fix on a family of cases and must not break
what already passes. The bench gives 231 independently specified cases with
closed-form answers and a strict grader; the probes in `probes/` give the
reduced reproductions each class was diagnosed with.

## Tasks

1. Keep the runner in `brep-bench/kernels/ngk` building against this tree
   (`ngk = { path = "../../.." }`), with a clone of upstream brep-bench's
   `harness/` and `cases/` vendored or fetched by a script at a pinned commit.
2. A script that runs the whole suite and compares against the last
   committed `brep-bench/results/ngk.json`: it fails when a case that passed
   no longer does, and prints the cases that moved either way.
3. CI: run it on every push to main and on pull requests touching `src/`
   (about four minutes in release). Commit the new `results/ngk.json` when a
   change moves cases forward.
4. Drop `probes/boolean_probe.rs` into the runner as a case file and expose
   each probe by name (`PROBE=<name>`), so a workstream can rerun the exact
   reproductions this plan was written from. Then turn each family listed in
   a workstream file into ordinary `cargo test` tests next to the code it
   exercises; the probe file is a starting point, not the tests.
5. Fix the runner itself where it hid information: refusals now carry the
   whole error chain and a Boolean's coverage diagnostics (done on
   2026-11-04); keep that.
6. When the bench passes a case that needed a workaround, drop the
   workaround (`workarounds.toml`) so the board reflects the kernel.

## Done when

The gate runs in CI and is red on any regression; each workstream's pull
request states the bench cases it moved.
