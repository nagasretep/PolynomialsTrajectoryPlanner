# 0002 — Exact minimum-snap pass-through solver (M7)

Status: open · From: Cowork · To: Code · Date: 2026-10-17

## Goal

Replace the finite-difference heuristic in `PassThroughSolver1D` with the exact global solve defined in `theory/TrajectoryPassingThroughConstraintPoints.md`, section *Global Minimum-Snap Formulation In 1D*. Along the way, give the project a single duration mechanism (findings D1, D2, D6 of `roadmap/discrepancies.md`; item 4 and 6 of your note 0001).

## Reference documents

- `theory/TrajectoryPassingThroughConstraintPoints.md` — the formulation, the table of orders $m$, the reference values (read the whole new section first).
- `theory/PolynomialCoefficientDetermination.md` — normalized-time matrix (it provides $\mathbf{B}$).
- `design/07-implementation-plan.md` — Decision 1 (durations), Decision 5 and appendix A7.
- `roadmap/discrepancies.md` — D1, D2, D6.
- Your note: `coordination/inbox-docs/0001-baseline-review.md`, items 4 and 5.

## Scope

1. **Solver.** In `Core/PassThrough`: build $\mathbf{B}$ (inverse of the normalized-time matrix for the degree; reuse the existing matrix factory or the existing coefficient solver, whichever is simpler and consistent with the code), the matrix $\mathbf{Q}$ for order $m$, the segment matrices $\mathbf{R}_k$, assemble $\mathbf{H}$ and $\mathbf{f}$ (dense is fine), solve $\mathbf{H}\mathbf{y} = -\mathbf{f}$ with a Cholesky factorization (Math.NET; do not invert), rebuild the node states and the segments through the existing `PolynomialSegment1D`. Degrees 3, 5, 7, 9.
2. **Criterion.** Replace `ISmoothnessCriterion` (branch selector plus unused cost) by something that carries the derivative order $m$ (validate $1 \le m \le p$, $p = (d+1)/2$). Defaults: $m = 2, 3, 4$ for degrees $3, 5, 7$ and $m = 4$ (snap) for degree 9; degree 9 with $m = 5$ must also work (the author may switch to it). The exact cost of a solution as a quadratic form (the formula for $J_k$) should be available as a function, for tests and reports; the trapezoid sampling goes away.
3. **Remove the heuristic** from the production path (`SolveFiniteDifferenceHeuristic`, `SolveMinimumSnapInterior`, minimum-jerk branch). **Before removing it**, compute for $x = (0, 1, 3, 2)$, $T = (0.7, 1.3, 0.9)$, degree 9, $m = 4$: the exact cost $J$ of the new solution and the cost of the old solver's solution, and report both in your return note (Cowork's prototype gives roughly $2.2 \cdot 10^{4}$ for the exact solution; the old one should be dozens of times larger).
4. **One duration mechanism.** Wire `IDurationStrategy1D` into the builders and delete the duplicate in `PassThrough/DurationStrategies` (or the reverse; one place only, probably `Services`). `AutoDerivedProportionalDuration` must follow Decision 1: total time split **proportionally to $\lvert \Delta x \rvert$** between consecutive waypoints, with a small floor so that equal consecutive waypoints do not give a zero duration (keep your existing floor if reasonable and say which). Say in the note whether this changes any current behaviour.
5. **3D.** If `TrajectoryBuilder3D` exposes pass-through, make it use the new solver per axis with **common durations**. With common durations the 3D cost is the sum of the three axis costs, so the component-wise solve is exactly optimal; add a test that each 3D component equals the 1D solve on the same data.

## Out of scope

UI changes; `DurationSearcher` behaviour (decision D5 pending); blending; constant velocity; moving App logic into Core.

## Acceptance criteria

- [ ] Reference values in the theory section are reproduced (tolerance $10^{-6}$ on the states, relative $10^{-9}$ on $J$): $(0,1,2)$ for all four degrees, and $(0,1,3,2)$ for degree 9 with $m = 4$.
- [ ] Clamped-spline relation holds for degree 3, $m = 2$, with several waypoints and equal durations.
- [ ] Exact passage at every waypoint; zero derivatives at the global ends; continuity of the derivatives up to order $p-1$ at every interior node (checked by evaluating neighbouring segments).
- [ ] Optimality: the gradient $\mathbf{H}\mathbf{y} + \mathbf{f}$ at the solution is below $10^{-7}$ in relative terms, and random small perturbations of the interior derivatives always increase $J$.
- [ ] Degree 9 with $m = 5$: derivatives continuous across the waypoints up to the 8th (tolerance of your choosing, stated in the test).
- [ ] Scaling: multiplying all positions by $\lambda$ multiplies all node derivatives by $\lambda$.
- [ ] Edge cases: one segment (no unknowns, rest-to-rest), two segments, durations $\le 0$ rejected, fewer than two waypoints rejected.
- [ ] `dotnet build` and `dotnet test` succeed; all previous tests still pass (or are updated with a reason stated in the note).
- [ ] Return note `coordination/inbox-docs/0002-minimum-snap-solver.md` (template) and `coordination/STATUS.md` updated.

## Notes for the author (educational)

In the return note, please explain in plain words the path from the theory to the code: which variable in the code is $\mathbf{y}$, $\mathbf{H}$, $\mathbf{f}$, $\mathbf{R}_k$, $\mathbf{B}$, and why the constraints disappear (they are built into the parametrization). Show the before/after cost comparison of item 3 and, if easy, one before/after plot description (overshoot, peak jerk).
