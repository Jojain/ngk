# WS 0: brep-bench and the probe corpus as a regression gate

**Status:** landed. The gate (`brep-bench/gate.py`, run in CI by
`.github/workflows/brep-bench.yml`) and the reproduction suite
(`tests/robustness.rs`, one module per workstream) exist; `brep-bench/README.md`
says how to run both. What is left is upkeep, below.

**Owns:** `brep-bench/`, CI workflow, `tests/robustness*`, `tests/support/robust.rs`.
**Depends on:** nothing. **Size:** small.

## What each workstream does with it

- Every workstream starts by un-ignoring the reproductions of its class in
  `tests/robustness/<area>.rs` (red first), and is done when they are green. The
  `#[ignore]` reason names the class and the workstream file it belongs to; the
  change that fixes the class deletes the attribute, and re-reads the reasons
  of the tests next to it.
- A reproduction that needs an operation the suite cannot spell yet (a loft
  with an explicit pairing, a section stored as a procedural curve, a face on a
  given NURBS surface such as the saddle drum or the groin vault) is added by
  the workstream that adds the operation.
- Every workstream's pull request runs `python3 brep-bench/gate.py`, and when
  cases moved forward commits the new `brep-bench/results/ngk.json` with
  `--update`, stating the cases that moved.
- WS A's mesh check, `assert_closed_mesh` in `tests/support/robust.rs`, is the
  check to apply to every solid a test builds; the ignored tessellation tests
  are its first uses.

## Upkeep

1. When the bench passes a case that needed a workaround, drop the workaround
   (`brep-bench/kernels/ngk/workarounds.toml`) so the board reflects the kernel.
2. Move `brep-bench/UPSTREAM` to a newer brep-bench commit when its cases
   change, and run `--update` in the same change, so a moved pin is never read
   as a regression.
3. Keep the refusals' full error chains and the Boolean's coverage diagnostics
   in the runner (`brep-bench/kernels/ngk/src/bench.rs`): they are what makes a
   refused case explain itself.
4. Turn the remaining families listed in the workstream files into tests as
   their operations appear, in the module of their workstream. Not yet written:
   the loft pairing options (WS B5), the helix's start angle (WS B4), the
   saddle face, groin vault and lofted wall of WS E (they need a face on a
   NURBS surface), the family cross-products of WS F and G beyond the cases in
   the modules, and WS C's property test of random builder sequences.

## Done when

The gate runs in CI and is red on any regression; each workstream's pull
request states the bench cases it moved.
