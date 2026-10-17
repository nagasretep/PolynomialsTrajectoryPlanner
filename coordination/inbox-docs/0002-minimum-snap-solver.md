# 0002 — Exact minimum-snap pass-through solver (return note)

Status: closed (reviewed by Cowork 2026-10-17) · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0002-minimum-snap-solver.md`

## What was done

- **Exact solver** in `Core/PassThrough` replacing the finite-difference heuristic (which is deleted: `SolveFiniteDifferenceHeuristic`, `SolveMinimumSnapInterior`, `SolveMinimumJerkInterior`, `MinimumSnapCriterion`, `MinimumJerkCriterion`, `SolveWithAutoDurations`, `DurationStrategies`). Degrees 3, 5, 7, 9.
- **New files**: `SmoothnessCriterion.cs` (order *m*), `SmoothnessCost.cs` (exact cost, matrix Q), `PassThroughProblem1D.cs` (H, f, constant, solve), and a much shorter `PassThroughSolver1D.cs`.
- **One duration mechanism** in `Services/IDurationStrategy.cs`, now used by both builders.
- **Tests**: 43 new tests in `Core.Tests/PassThrough/MinimumSnapSolverTests.cs`; one existing test adjusted (see below). `dotnet build`: 0 errors (same 3 NU1701 warnings); `dotnet test`: **67/67 passed** (24 old + 43 new).

## Why (choices made)

### From the theory to the code (for the author)

The theory says: put the unknown interior derivatives in a vector **y**, express the cost as a quadratic form in y, and solve one linear system. In the code:

| Theory | Where in the code | What it is |
|---|---|---|
| **y** | `Vector<double>` returned by `PassThroughProblem1D.Solve()` | The derivatives of order 1 … p−1 at each *interior* waypoint, node after node: for degree 9 and N−1 interior nodes, 4 numbers per node (v, a, j, s). Layout: index `(node−1)(p−1) + (order−1)`. |
| **B** | local `b` in `PassThroughProblem1D.Build` | Inverse of the normalized-time matrix (`NormalizedTimeMatrixFactory`), the 10×10 matrix that turns end-point values into polynomial coefficients. Same matrix the single-segment solver already uses, just inverted once (it depends on the degree only; it is small, so an explicit inverse is fine here). |
| **Q** | `SmoothnessCost.BuildQ(degree, m)` | The matrix such that cᵀQc = ∫₀¹ (dᵐx/dτᵐ)² dτ, using the formula in the theory (falling factorials / (i+j−2m+1)). |
| **Δₖ** | local `delta` | The diagonal scaling by powers of Tₖ that converts physical derivatives into normalized-time ones. |
| **Rₖ** | the expression `scale · delta · (BᵀQB) · delta` inside the loop over segments | The cost of segment k as a quadratic form in its 2p end-point numbers **zₖ**. |
| **zₖ = Gₖy + gₖ** | arrays `unknownIndex` / `knownValue` | For each of the 2p entries of zₖ: either a known number (a waypoint position, or a 0 for a derivative at the global start/end) with `knownValue`, or "this is component *i* of y" with `unknownIndex`. This is Gₖ and gₖ written as two small arrays instead of 0/1 matrices. |
| **H**, **f**, const | `PassThroughProblem1D.H`, `.F`, `.Constant` | Accumulated segment by segment: where both row and column entries are unknown → into H; unknown × known → into f; known × known → into the constant. |
| H y = −f | `H.Cholesky().Solve(−F)` | Cholesky factorization (H is symmetric positive definite), no inverse of H. |

**Why the constraints disappear**: the constraints of the problem (pass through every waypoint, start and end at rest, continuity of position…snap at the nodes) are not equations the solver has to enforce. They are used to *define* the unknowns: each node has one shared state, the positions are the waypoints (known), the end derivatives are 0 (known), so what is left free is exactly y. A segment starts from the state of node k and ends at the state of node k+1, so continuity is automatic. J then depends on y alone and is minimized without constraints.

Once y is known, `PassThroughSolver1D.Solve` builds the node states and creates each segment through the existing `PolynomialSegment1D` (so the coefficients of every segment still come from the single-segment solver).

### Choices not dictated by the theory

1. **Criterion**: `SmoothnessCriterion` is a `readonly record struct` with just the order *m*, validated against 1 ≤ m ≤ p. `DefaultForDegree`: m = p for degrees 3, 5, 7 and **4 for degree 9** (A7, pending author decision). `new PassThroughSolver1D()` with no argument takes the per-degree default; the builders have `WithSmoothnessCriterion(...)` to override. `ISmoothnessCriterion` no longer exists.
2. **Exact cost as a function**: `SmoothnessCost.Of(segments, m)` / `OfSegment`, evaluated from the segment coefficients (cᵀQc·T^(1−2m)), independent of the quadratic form in `PassThroughProblem1D`; a test checks that both agree to 1e-8 relative.
3. **`PassThroughProblem1D` is public** (H, F, Constant, Cost, Gradient, Solve) so the optimality tests can inspect the problem; it is also the natural place for a later report ("cost of this trajectory").
4. **Explicit inverse of the normalized matrix** (B), Cholesky for H (as asked).
5. **Floor of the proportional duration**: kept the existing absolute floor of 0.05 s (`AutoDerivedProportionalDuration.DefaultMinSegmentDuration`, constructor parameter). Consequence, documented in the XML comment: when the floor is active the sum of the durations can exceed the requested total. A relative floor would be more scale-independent, but changes behaviour more; open question 2.
6. **`IDurationStrategy1D`** got a second member `ComputeDurations(waypoints, degree)` for waypoint sequences (the per-segment `ComputeDuration` is kept unchanged). `UserSpecifiedDuration` validates the count; `AutoDerivedProportionalDuration` is proportional to |Δx| (and has an overload for `Point3D[]` using the Euclidean distance); `ConstraintAwareDuration.ComputeDurations` throws `NotSupportedException` (pass-through interior states are unknown before durations are fixed; `DurationSearcher` was out of scope). Builder precedence for pass-through: explicit `durations` argument, else `WithDurationStrategy(...)`, else proportional with `totalDuration`.

## Item 3: cost before and after (old heuristic vs exact)

Data: x = (0, 1, 3, 2), T = (0.7, 1.3, 0.9), degree 9, m = 4. Measured by sampling 3001 points of the whole trajectory.

| | old heuristic | exact (m = 4) | exact (m = 5, for reference) |
|---|---|---|---|
| snap cost J₄ | **981 993.99** | **22 094.42** | 35 839.92 |
| max position | 3.058 | 3.674 | 4.185 |
| peak velocity | 3.07 | 4.47 | 5.70 |
| peak acceleration | 14.6 | 9.63 | 14.5 |
| peak jerk | 130.2 | 33.0 | 49.5 |
| peak snap | 1327 | 256 | 210 |

- The new cost is **44.4 times smaller**; Cowork's estimate (~2.2·10⁴) is confirmed (22 094).
- In plain words: the old solution fixed the node velocity by a central difference, set a = j = 0 at every node and took an arbitrary snap, forcing the polynomials to "make up" the difference inside each segment, which costs large jerk and snap peaks. The exact solution lets the nodes carry the acceleration, jerk and snap they need, and the peaks fall by a factor 4 to 5.
- **Honest caveat for the author**: minimum snap does *not* minimize overshoot or speed. Here the exact trajectory goes higher than the old one (max position 3.67 vs 3.06, waypoints peak at 3) and has a higher peak velocity (4.47 vs 3.07). It is smoother in the snap sense, not "tighter". This is what the theory section "Overshoot" warns about: kinematic limits must still be verified afterwards.

## Acceptance criteria — where verified

| Criterion | Test(s) |
|---|---|
| Reference values (0,1,2), all four degrees | `ThreeWaypoints_InteriorVelocityMatchesReference`, `…Degree9Snap_JerkAndCostMatchReference` (J = 4032 at 1e-9 rel.), `…Degree3_CostIsSix` |
| Reference (0,1,3,2), d = 9, m = 4 | `FourWaypoints_Degree9Snap_NodeStatesMatchReference` (states to 1e-6; J compared to 10977.31 within ±0.01 because the document gives it with 7 digits only, not 1e-9) |
| Clamped spline, degree 3, m = 2 | `Degree3_Order2_SatisfiesClampedSplineRelation` (7 waypoints, h = 0.8, 1e-9) |
| Passage, zero ends, continuity to p−1 | `Solution_PassesExactlyStartsAndEndsAtRest_AndIsContinuous` (all four degrees, unequal durations) |
| Optimality (gradient < 1e-7 relative, 100 random perturbations increase J) | `Solution_IsTheMinimizer` (degrees 3/5/7/9 and 9 with m = 5) |
| Degree 9, m = 5 continuous up to the 8th derivative | `Degree9_Order5_IsContinuousUpToTheEighthDerivative` (relative 1e-6) and, as a contrast, `Degree9_Snap_HasJumpsInTheFifthDerivative` |
| Scaling by λ | `Scaling_AllPositionsByLambda_ScalesAllNodeDerivatives` |
| Edge cases | one segment (4 degrees), repeated waypoint, <2 waypoints, count mismatch, durations ≤ 0 or NaN, m outside 1…p, unsupported degree |
| 3D component = 1D solve | `Builder3D_EachComponentEqualsThe1DSolveWithTheSameDurations` |
| Durations | `AutoDerivedProportional_*`, `Builder1D_UsesConfiguredDurationStrategy`, `Builder1D_ExplicitDurationsWinOverTheStrategy` |

The only existing test touched is `PassThroughSolverTests.Solve_ThreeWaypoints_CreatesValidSegments`: `new PassThroughSolver1D(new MinimumSnapCriterion())` became `new PassThroughSolver1D(SmoothnessCriterion.MinimumSnap)` (the type no longer exists); assertions unchanged.

## Behaviour changes to be aware of

- **Pass-through results change** everywhere (heuristic → exact), by definition. The App does not use pass-through, so the UI is unchanged (the App project builds untouched).
- **Durations**: `DurationStrategies.ProportionalToDistance(3D)` and the new `AutoDerivedProportionalDuration` compute the same numbers (|Δ|-proportional, 0.05 s floor, equal split when all points coincide), so default pass-through durations are **unchanged**. What changed is `AutoDerivedProportionalDuration` itself: its waypoint-sequence form is now proportional (Decision 1), while its legacy per-segment `ComputeDuration` is still an equal split (nothing uses it).
- New validation: durations must be positive and finite; `Solve` rejects unsupported degrees and bad *m* with exceptions (before: zero/negative durations failed deep inside `PolynomialSegment1D`).
- `PassThroughSolver1D.SolveWithAutoDurations` removed (no users).

## Discrepancies between theory and code

None found for the formulation: the code reproduces all reference values of the theory section, which also validates the section's formulas (Q, Rₖ, H, f). Two small points on the text:
1. The theory's *Which Order m* table uses m = p for degree 9 as "natural", while the default for degree 9 is snap by Decision 5; the code follows Decision 5 and takes m as a parameter, as the text says.
2. The cost J(0,1,3,2) is given to 7 digits (10977.31), so the 1e-9 relative acceptance cannot be applied to it; I used ±0.01.

## Test results

```
dotnet build PolynomialsTrajectoryPlanner.slnx → 0 errors, 3 warnings (NU1701, unchanged)
dotnet test src\TrajectoryPlanner.Core.Tests    → Passed! Failed: 0, Passed: 67, Skipped: 0, Total: 67
```

## Open questions

1. **Degree 9: snap or crackle?** With m = 5 the trajectory is C⁸ at the waypoints but, on the data above, its J₄ is 62 % higher and it overshoots more (max 4.19) and is faster (peak velocity 5.70) while peak snap is lower (210 vs 256). Numbers may help the author's decision.
2. **Floor of proportional durations**: absolute 0.05 s (current) or relative to the total? With a tiny total (say 0.1 s over 5 segments) the absolute floor dominates and the total is not respected.
3. `AutoDerivedProportionalDuration.ComputeDuration` (per segment) is still an equal split and has no users; delete it from the interface, or keep? I'd delete it in the consolidation (roadmap R7).
4. `ConstraintAwareDuration` cannot serve pass-through. If the author wants "pass-through with kinematic limits", the natural approach is a loop around the solver (scale all durations, solve, verify limits with `ExtremaAnalyzer`), which belongs with the D5 decision.
5. Cost scaling: H has a condition number around 10⁶–10⁷ for the tested cases (as in the theory); no problem found up to the tested sizes, but I did not test dozens of segments with very different durations. A future test could cover N = 30.

## Cowork's replies to the open questions (2026-10-17)

1. **Snap or crackle**: author decision; numbers collected in `roadmap/discrepancies.md`. Default stays snap.
2. **Floor of proportional durations**: relative floor, total preserved — rule in `design/07` A8; implemented in request 0003.
3. **Per-segment `ComputeDuration`**: delete in the consolidation (R7), agreed.
4. **Pass-through with limits**: agreed, a loop around the solver. For rest ends and fixed duration ratios the time scale needed is closed-form (peak of order $k$ scales as $\lambda^{-k}$, see D5 in `roadmap/discrepancies.md`); it belongs to roadmap R11, after the author's decision on D5.
5. **N = 30 conditioning**: added to request 0004.
