---
status: accepted
date: 2026-11-05
---

# Panics are not recoverable modeling errors

## Context

The edit-safety implementation caught panics around the transaction's operation
closure. A panic after a recorded read fault became that fault's error; other
panics triggered rollback before resuming unwinding. Commit ran outside this
handler, so policy panics did not receive the same treatment.

Inferring that a recorded read fault caused every subsequent panic hides the
actual failure. Extending the handler to commit would extend recovery rather
than repair the invalid read or mutation that made recovery necessary.

## Decision

NGK transactions do not catch panics. They call the operation directly and
preserve the original panic, including when a read fault has been recorded.
Commit and payload policy follow the same rule.

Recoverable conditions use `Result`: invalid inputs, unsupported configurations,
failed validation and policy refusals restore the transaction-start snapshot.
A panic identifies a defect to investigate and correct through preconditions,
capability boundaries, explicit fallible APIs or invariant-preserving edits.

## Consequences

- Rollback and model reuse are not guaranteed after a panic.
- No panic handler or unwind-recovery guard is added to provide such a promise.
- Tests use `#[should_panic]` to check propagation; they do not assert recovery
  or continued use of a model after a panic.
- Recorded read faults still fail a transaction when the operation returns.
  They never convert a panic into `ModelEditError`.
- The operator and deferred-block decisions in ADR 0003 remain in force.
