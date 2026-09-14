# Periodic supports: closed NURBS, and curves that repeat without closing

Status: **Proposed** — nothing here is needed by current code. It is written
down so the two features that will need it are designed for rather than
discovered, and so the one thing that must *not* be done in the meantime is on
the record.

## The conflation

`Periodicity::Periodic(P)` is reported today by `Circle` and `Ellipse` only.
Every NURBS reports `Periodicity::None`, in both dimensions. That is currently
honest, and the reason is one line in `NurbsCurve2::point_at`:

```rust
let parameter = self.clamp_parameter(parameter);
```

Evaluation **clamps**. Hand it a parameter past the end of the knot domain and
it silently returns the endpoint. So `Periodic` cannot be declared for a NURBS
without making the declaration false.

Underneath sit three different statements that the single enum currently blurs:

| Statement | Question it answers | Who answers today |
|---|---|---|
| **closed** | do the ends meet? | `Curve::is_closed()` — geometric, cheap, already right |
| **periodic parameterization** | may I evaluate outside the domain and get the right point? | `Periodicity` — a promise about the *evaluator*, not about the shape |
| **repeating** | does the shape repeat along the parameter without the points repeating? | nothing — no support needs it yet |

Periodic implies closed: if `point_at(t + P) == point_at(t)` for every `t`, any
window of length `P` closes on itself. The converse holds for every support the
kernel has, since any closed curve *can* be given a wrapping parameterization.
So closed and periodic pick out the same shapes today — they are still not the
same claim, and the two features below separate them.

## Gap 1 — a closed NURBS span cannot cross its own seam

A `TrimmedCurve`/`TrimmedCurve2` is a support plus an `Interval`. Expressing
"from 3π/2 round to π/2" needs the interval `[3π/2, 5π/2]`, which leaves the
knot domain and clamps. So **any span on a closed NURBS that crosses where the
curve closes is unrepresentable**, and a caller that wants one gets the
complementary arc or an endpoint.

This is not obscure. A circle stays a `Circle` in 3D, but as a *pcurve* it is
fitted and arrives as a closed rational quadratic — so every circular face
boundary has a closed NURBS underneath it in parameter space.

Nothing is known to need a seam-crossing span right now. Confirm that before
starting: if a producer already wants one, it is presumably choosing the wrong
arc silently today, and that is a correctness bug rather than a feature.

**The ordering matters, and getting it wrong is worse than doing nothing:**

1. Make `NurbsCurve2` / `NurbsCurve` evaluation **wrap** into the domain for a
   closed curve rather than clamping.
2. Add `Curve2::is_closed()`, mirroring the 3D `Curve::is_closed()` the 2D side
   is missing — the claim that "2D mirrors 3D exactly" is not true here today.
3. *Then* derive periodicity from closedness rather than storing it beside:
   `Curve2::Nurbs(c) => if c.is_closed() { Periodic(c.domain().delta()) } else { None }`.

Declaring periodicity before step 1 converts a loud rejection
(`SplitPointNotOnPcurve`) into silently wrong geometry. Do not reorder.

### The seam derivative

A closed NURBS is C⁰ where it closes but generally not C¹, so `derivative_at`
is two-valued there: a left derivative approaching the domain end and a right
derivative leaving the start. For a circle they agree; for a curve with a kink
they do not, and no single answer is correct.

The resolution is to **ask the span, not the support**. A `TrimmedCurve2` knows
which branch it covers and which way it runs. Mid-span the question is not
ambiguous at all — both one-sided derivatives lie inside the span. Only a span
*endpoint* is ambiguous, and there the caller wants the derivative pointing into
the span, which the span knows and the support cannot. `parameter_slack`
already sidesteps this by evaluating at the span's midpoint.

So: wrap evaluation one-sided into `[start, end)`, and let the trimmed curve own
endpoint derivatives by working from its interior.

A kink where a curve closes is a **corner**, and a corner is a *mark* — see the
edge vocabulary in `AGENTS.md`. An edge whose seam is kinked should be marked
there, and then nothing asks for a derivative across it. The pathological case
is the one the topology should already be recording.

## Gap 2 — a helix repeats but never closes

`point_at(t + 2π)` on a helix is not the same point; it is translated along the
axis. The parameterization has a period, the point set has none. A helix is the
concrete case where **periodic and closed come apart**, and it is not exotic
here: sweeps and revolutions are exactly where one would arrive.

When a helix support lands, `Periodicity` has to stop meaning "the points
repeat". Either it gains a variant for a shape that repeats under a transform,
or helices carry the repeat separately and report `Periodicity::None`. Deciding
that is part of adding the support, not before.

Until then, treat `Periodic(P)` as meaning **the points repeat** — that is what
`TrimmedCurve::native_parameter_at` assumes when it folds a raw parameter onto
the branch nearest a span, and a helix would break that silently.

## What to do now

Nothing, beyond not painting over it:

- Do not report `Periodic` for a NURBS while evaluation clamps.
- Do not add a second field beside `is_closed()` saying the same thing; derive.
- Do not assume `Periodic(P)` and "closed" are interchangeable in new code, even
  though every support today satisfies both or neither.
