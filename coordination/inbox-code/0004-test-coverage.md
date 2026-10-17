# 0004 — Test coverage for the untested areas

Status: open · From: Cowork · To: Code · Date: 2026-10-17

## Goal

Close the gaps listed in your note 0001 (item 3): Blending, degree 7, `Trajectories`, extrema analysis for degrees above 3, and the larger pass-through case. Tests only; if a test reveals a defect, do not fix it silently: write it in the note (and mark the test as expected-to-fail with a reason if that is the honest state).

## Reference documents

- `theory/BlendingSegments.md` (sections *Sampling The Boundary States For The Blend*, *Building The Blend Segment In 1D*, *Choosing The Blend Duration*)
- `theory/PolynomialCoefficientDetermination.md`, `theory/NinthDegreePolynomialTheory.md`, `theory/TrajectoryConstraints.md`
- `roadmap/discrepancies.md` (D4, D7)

## Tests to write

1. **Blending (`BlendBuilder1D`).**
   - The blend segment starts and ends with exactly the states sampled from the neighbours (position … snap for degree 9), and the truncated neighbours join it with continuity of all those derivatives.
   - Inherited duration: $T_b$ equals the sum of the removed portions, as the code states it; absolute blend positions map to the right $\tau$ (check by evaluating the neighbour at the returned $\tau$).
   - The blend does not pass through the nominal join (distance from the join is positive for a non-degenerate window).
   - Invalid input: a position outside the segment, window reversed or empty.
   - Document in the test names which behaviour is today's implementation and not theory (blend degree fixed at 9, $\tau$ clamp), so that the consolidation can change them knowingly (D7).
2. **Degree-7 coefficients.** Endpoint conditions (position, velocity, acceleration, jerk) at both ends, as already done for degrees 3, 5 and 9; plus the rest-to-rest case.
3. **`Trajectory1D` / `Trajectory3D`.** `TotalDuration`; evaluation at $t = 0$, exactly at an interior node (no gap, no overlap, consistent value from either side), at $t$ = total, and outside the range (state the behaviour); `FindSegment` with a very short segment; continuity of the evaluated position and velocity across all nodes of a multi-segment trajectory.
4. **Extrema analysis with analytic answers.** For a rest-to-rest segment of distance $\Delta$ and duration $T$, the peak velocity is $c_d\, \Delta / T$ with $c_3 = 3/2$, $c_5 = 15/8$, $c_7 = 35/16$, $c_9 = 315/128$ (the velocity is $c\,\tau^{p-1}(1-\tau)^{p-1}$ times a constant, maximal at the midpoint; for $d=9$: $x'(\tau) = 630\,\tau^4 (1-\tau)^4\, \Delta / T$). Check the value and the location ($t = T/2$) for each degree; check that the acceleration peak scales as $\Delta / T^2$ (run with two durations). Also a violated-limit case for `VerifyLimits`.
5. **Duration strategies.** `UserSpecifiedDuration` count mismatch; `ConstraintAwareDuration` per-segment result satisfies the limits (and the failure reporting of request 0003).
6. **Pass-through, larger case.** $N = 30$ segments with strongly different durations (a ratio of at least 100 between the longest and the shortest): the solve succeeds, the gradient at the solution is small relative to the cost, continuity and exact passage hold. If the Cholesky factorization fails or the accuracy degrades, report the numbers; do not hide it.

## Acceptance criteria

- [ ] All new tests pass, or each failing one is marked with a reason and listed in the note as a finding.
- [ ] `dotnet build` and `dotnet test` succeed.
- [ ] Return note `coordination/inbox-docs/0004-test-coverage.md` with a table "Core folder → tests" updated from your note 0001, and `coordination/STATUS.md` updated.
