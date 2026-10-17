# 0006 — Numerical conditioning and two duration fixes

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

## Goal

Three small, well-defined changes that came out of your notes 0004 and 0005 (D16, D17 in `roadmap/discrepancies.md`). No new features.

## Reference documents

- `theory/TrajectoryPassingThroughConstraintPoints.md` — *Properties Worth Knowing*, bullet **Conditioning** (updated).
- `theory/TrajectoryConstraints.md` — *Shortest Feasible Duration* (lower bounds for all orders; the zero-length case).
- Your notes `coordination/inbox-docs/0004-…` (finding 3, open question 1) and `0005-…` (open question 1).

## Tasks

1. **Diagonal scaling in the pass-through solve (D16).** In `PassThroughProblem1D.Solve`, scale before factorizing: $s_i = 1/\sqrt{H_{ii}}$, $\tilde{H} = S H S$, $\tilde{f} = S f$; Cholesky-solve $\tilde{H} z = -\tilde{f}$; $y = S z$. Keep `H`, `F` as they are (tests use them). Cowork's prototype on a comparable case (30 segments, duration ratio 100) reduced cond from $3 \cdot 10^{15}$ to $2 \cdot 10^{9}$ for degree 9, $m = 5$, and from $4 \cdot 10^{13}$ to $1 \cdot 10^{6}$ for $m = 4$. Report your own numbers (cond of $H$ and of $\tilde{H}$) in the note.
2. **`Cost(y)` (your open question 1 of 0004).** Keep it public for the optimality tests, mark it in the XML comment as unreliable for large constants (cancellation) and name `SmoothnessCost.Of` as the one for reports. Do not remove it.
3. **Zero-length rest-to-rest segment (D17, your open question 1 of 0005).** When the limits determine nothing (no active limit, or a rest-to-rest segment with $\Delta = 0$), `ShortestFeasibleDuration` must not invent a 1 ms duration. Make "not determined by the limits" an explicit outcome (for example a `TryFind` returning false, or a nullable result; your choice, state why). `ConstraintAwareDuration` and the App then use the prescribed or default duration (for `ConstraintAwareDuration` a constructor parameter with the fallback duration; the App keeps its default of 1.0 s). Keep the 1 ms floor only as protection against degenerate numbers in the search, not as an answer.
4. **Theory alignment of the start bound.** Your start bound already uses the change of the boundary derivative of order $k-1$ divided by $L_k$; the theory now says the same. Nothing to do except confirming in the note that the code matches the text.

## Out of scope

Constant velocity (D8 awaits the author), blending validation (R7), the consolidation.

## Acceptance criteria

- [ ] All earlier tests pass (173 + new).
- [ ] N = 30 test of request 0004: for degree 9, $m = 5$ the **exact cost** `SmoothnessCost.Of(segments, m)` of the scaled solution is not larger than that of the unscaled solution (within $10^{-12}$ relative), and the optimality perturbation test still passes. Add a test that fixes this comparison (keep the unscaled solve available in the test, for instance via the public `H` and `F`).
- [ ] Zero-length rest-to-rest with active limits and no active limit: the outcome is "not determined", and the callers use the fallback duration (tests for `ShortestFeasibleDuration`, `ConstraintAwareDuration`, and the App path if you can reach it as you did for 0005).
- [ ] `dotnet build` and `dotnet test` succeed.
- [ ] Return note `coordination/inbox-docs/0006-numerics-and-duration-fixes.md` (with *Suggested commits*) and `coordination/STATUS.md` updated.
