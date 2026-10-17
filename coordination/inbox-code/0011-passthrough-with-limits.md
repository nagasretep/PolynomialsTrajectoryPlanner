# 0011 — Exact pass-through with kinematic limits

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

Run after request 0010.

## Goal

The 1D exact pass-through builder can produce the shortest feasible trajectory when kinematic limits are active, using one common scale factor λ on the reference durations, and verifies limits when the durations are prescribed. Failures are explicit.

## Reference documents

- `theory/TrajectoryPassingThroughConstraintPoints.md`, *Pass-Through With Kinematic Limits* (scaling law, closed form, reference values) and *Global Minimum-Snap Formulation In 1D*
- `theory/TrajectoryConstraints.md`, *Shortest Feasible Duration* (general case: duration search, caveats)
- `design/07-implementation-plan.md` A8 (proportional rule), A9

## Scope

- In scope:
  1. **Builder API.** The Core pass-through builder gets an optional `KinematicLimits? limits`. Behaviour:
     - *Durations prescribed* (user-specified durations, or a prescribed total with the proportional rule): keep them, solve, verify every segment with `VerifyLimits`; a violation raises `KinematicLimitsViolatedException` naming the segment index and the limit.
     - *Durations automatic, limits active*: reference durations from the strategy (proportional rule of A8 with a reference total of 1 s, or the equal split if all distances are zero). Solve once. **Boundary derivatives all zero (rest ends):** compute the peaks P_k(1) of orders 1..4 of the reference solution with the existing extrema routine, then λ* = max over active limits of (P_k/L_k)^(1/k); durations = λ*·reference; **solve again** at the scaled durations and verify (do not just rescale the coefficients; the second solve doubles as a check of the scaling law). **Non-zero boundary derivatives:** use the general duration search of `ShortestFeasibleDuration` on λ (each evaluation = one solve), starting from the zero-boundary value; explicit failure if the search fails.
     - *Orders with P_k = 0 impose nothing*; if no limit is active, or all P_k are zero, the limits determine nothing: use the reference durations unscaled (mirrors the `null` outcome of `ShortestFeasibleDuration.Find`).
     - The result should expose the chosen λ and the final durations (a small result type or properties), so the report can show T per segment.
  2. A reusable helper for the peaks of a whole trajectory by order, if one does not exist (the 0009 `Sample`/extrema code may already provide it).
  3. Tests (below). No App/UI change (that is R10); only if a minimal App hook is needed for the tests, say so in the note.
- Out of scope: 3D, non-uniform duration allocation, UI.

## Acceptance criteria

- [ ] For the reference values in the theory section the computed peaks (1e-5 relative, the theory values come from 4001-point sampling), λ*, durations and totals match, and the peaks at λ* equal the limit for the binding order (1e-9 relative with exact extrema).
- [ ] Scaling law: for several data sets, degrees (5, 7, 9) and orders m, peaks at λ = 0.5 and 2 equal λ^(-k)·P_k(1) (1e-9 relative).
- [ ] The final trajectory passes `VerifyLimits` for every active limit; each limit binding in turn is tested.
- [ ] Non-zero boundary derivative case: result feasible, search failure explicit, comparison with the zero-boundary λ* documented in a test.
- [ ] Prescribed durations: kept and verified; violation reported with segment index.
- [ ] `dotnet build` free of new warnings; all tests pass; note states the test count.

## Expected tests

- Reference data 1 and 2 of the theory (rest ends, d = 9, m = 4), all four limits and the (v, a) case.
- Degrees 3, 5, 7 with m = default; a cubic (P_4 = 0) must not break the formula.
- N = 30 waypoints with a duration ratio of 100: runs, feasible, λ* finite (conditioning stays acceptable thanks to the scaling of 0006).
- All waypoints equal (nothing determined), no active limit, a single limit.
- A case with overshoot (non-monotone position between waypoints) where the peak is not at a node.
- Non-zero start velocity case.

## Notes for the author (educational)

Explain in the return note: why scaling all durations by λ scales the k-th derivative by λ^(-k) and leaves the optimal *shape* unchanged (and why this fails with non-zero boundary derivatives), why one solve plus a formula replaces a search, and what is still not optimal (individual durations are not re-allocated: a sequence with one very long segment keeps the same proportions).

Return note: `coordination/inbox-docs/0011-passthrough-with-limits.md`, ending with a "Suggested commits" section.
