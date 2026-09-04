---
name: test-first-workflow
description: Use when Codex adds, changes, or reviews tests; writes regression tests for known bugs; follows red-green implementation; names tests; or decides whether a test should assert current broken behavior versus desired behavior.
---

# Test First Workflow

## Core Rule

Always write tests as the behavior the code should provide, not as the current known-broken behavior.

When a bug is known:

1. Add or update the test to assert the desired correct behavior.
2. Run the focused test and confirm it fails for the expected reason.
3. Fix the implementation.
4. Run the focused test again and confirm it passes.
5. Run the relevant broader test suite.

Never add a test that asserts a validation error, panic, or wrong output only because that is what the current broken code does. Only assert errors when the error is the intended API behavior.

## Test Scope Exclusions

Do not create or update tests for visualization code or exploration scripts. This includes frontend visualization directories such as `visualization/`, Rust script modules such as `src/scripts/`, and mirrored script tests such as `tests/scripts/`.

Validate visualization and script changes with the relevant build, typecheck, lint, or direct execution checks instead. If tests were previously added specifically for visualization or scripts, remove them rather than maintaining them. This exclusion does not apply to underlying core behavior in builders, topology, geometry, modeling, tessellation, or other non-visualization library modules.

## Test Names

Give tests names that describe the stable expected behavior or invariant.

Avoid names that describe an implementation bug, a temporary failing state, or the fact that a test is expected to fail before the fix.

Prefer:

```rust
block_solid_orientation_validation_requires_outward_face_normals
```

Avoid:

```rust
block_solid_builds_closed_shell_but_fails_outward_orientation_validation
```

## Reporting

When following this workflow, state the red result before implementing the fix, including the focused command and the failure reason. After the fix, state the green result and any broader validation that was run.
