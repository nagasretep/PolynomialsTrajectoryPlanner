# 0011 — Exact pass-through with kinematic limits (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0011-passthrough-with-limits.md`

## Summary

Implemented as specified; tests **331 (Core.Tests, was 296) + 63 (App.Tests, unchanged) = 394**, all passing. `dotnet build`: 0 errors; warnings only the known NU1701. No App/UI change.

## What was done

**New `Core/PassThrough/PassThroughPlanner1D.cs`** (`Plan(...)` → `PassThroughResult(Segments, Durations, Scale λ, ScaledByLimits)` with `TotalDuration`):
- *Prescribed durations*: kept, solved, verified; a violation raises `KinematicLimitsViolatedException` with the **segment index and the limit kind** ("Pass-through segment 1 (duration 1.3 s) violates the velocity limit: peak 3.9 exceeds 3.").
- *Automatic, limits active*: reference durations = A8 proportional rule with a 1 s total (or an explicit `referenceDurations` argument, see below; equal split if all distances are zero).
  - **Zero boundary derivatives**: solve the reference once, peaks P_k(1) of orders 1–4 with the extrema routine, `λ* = max over active limits with P_k > 0 of (P_k/L_k)^(1/k)`, **second solve at λ*·reference** (not a rescale of coefficients), verification. The second solve doubles as the check of the scaling law: the binding peak equals the limit to 1e-9 in the tests.
  - **Non-zero boundary derivatives**: search on λ starting from the zero-boundary λ* (the reference solved with the boundary derivatives set to zero): geometric steps ×1.05 up while infeasible, ÷1.05 down while feasible, then bisection to 1e-13 relative; each evaluation is one solve. Range [λ0/1000, 1000·λ0]; failure → `DurationSearchFailedException` ("The pass-through: no common duration scale between … satisfies …").
  - *Nothing determined* (no active limit, or all P_k zero): reference durations unscaled, `ScaledByLimits = false`, `Scale = 1` (mirrors the `null` of `ShortestFeasibleDuration.Find`). Orders with P_k = 0 impose nothing (cubic + snap limit tested).
  - A small safety loop (up to 5 steps of ×(1+10⁻⁹) on λ) repairs a result that exceeds a limit only by rounding (the closed form lands exactly on the limit and `VerifyLimits` has a 10⁻¹⁰ absolute slack). In the test runs I did not instrument whether it fired; it is there as protection and then the final `VerifyOrThrow` is strict.
- **`ExtremaAnalyzer.PeakAbsolute(segments, order)`**: the reusable helper for the peak of a whole trajectory by order (the 0009 `Sample` is a sampler, not an extrema routine, so it did not provide this).
- **Builder**: `TrajectoryBuilder1D.AddSegmentsFromWaypointsPassThrough(waypoints, durations, totalDuration, degree, limits, startBoundary, endBoundary)`. `totalDuration` is now `double?` (was `double = 1.0`) so that "not given" can be told from a prescribed total. Durations are **prescribed** if `durations` is given, a duration strategy was set, or `totalDuration` is given (verify only); otherwise **automatic** (scaled with limits, reference 1 s without limits, exactly the earlier behaviour). `LastPassThroughResult` exposes λ and the final durations (for the report).
- **Non-zero boundary derivatives in the solver** (needed for the "non-zero start velocity" tests; the solver previously hard-wired zero ends): `PassThroughProblem1D.Build` and `PassThroughSolver1D.Solve` take optional start/end `State1D` with the prescribed derivatives of orders 1…p−1 (positions come from the waypoints). They enter only the vector `f` and the constant (the known part g_k), not `H`, so the conditioning work of 0006 is unaffected. Default (null) = rest, so all earlier behaviour and tests are unchanged.
- **`referenceDurations`** optional argument of `Plan` (automatic mode only; mutually exclusive with prescribed durations): the proportions T̄_k to scale. The theory defines λ with respect to reference durations "any proportional rule", and the reference values of the theory (data 2: 0.7, 1.3, 0.9) are not what the A8 rule gives, so this was needed to reproduce them. Not exposed on the builder (an easy addition if wanted).

## Acceptance criteria — where verified (`PassThrough/PassThroughWithLimitsTests.cs`, 35 cases)

| Criterion | Test(s) and numbers |
|---|---|
| Theory reference values: peaks, λ*, durations, totals, peaks at λ* | `ReferenceData1_*`: P = 2.275, 4.094571, 15.4, 74.005522 (1e-5); λ* = 1.168271, T = (1.168271, 1.168271), total 2.336542; at λ*: 1.947322, **3.0 (binding, 1e-9)**, 9.658063, 39.727378 (the code gives 39.727413: 9·10⁻⁷ relative to the sampled prototype value). `ReferenceData2_*`: P = 4.47151, 9.626191, 33.013296, 255.962162; equal-duration peaks 3.490773, 8.369728, 24.329164, 129.585159; λ* = 1.490503, T = (1.043352, 1.937654, 1.341453), total 4.32246, **v = 3.0 binding (1e-9)**, a = 4.332999 |
| Each limit binding in turn | `EachLimitBindingInTurn_*` (v, a, j, s on data 1: its peak equals the limit to 1e-9, others respected) and the (v, a) case of data 2 |
| Scaling law at λ = 0.5 and 2 | `ScalingLaw_*`: degrees 5, 7, 9 (m = 3, 4, 4, 5), three data sets, all orders: 1e-9 relative |
| Passes `VerifyLimits` | every automatic test calls `Respects` |
| Non-zero boundary derivative, comparison with zero-boundary λ* | `NonZeroStartVelocity_*`: zero-boundary λ* = 1.4905; start velocity **along** the motion (+1.5): **1.1908**; **against** it (−1.5): **5.9005**; the result is the feasibility boundary (durations 0.01 % shorter violate a limit); search failure: `BoundaryVelocityAboveTheVelocityLimit_ReportsASearchFailure` |
| Prescribed durations | `PrescribedDurations_*` (kept, λ = 1), violation message names the segment and the limit, builder variants |
| Degrees 3, 5, 7; cubic with P₄ = 0 | `LowerDegrees_*`, `Cubic_SnapLimitAlone_…` (imposes nothing), `Cubic_SnapLimitTogetherWithVelocity_…` |
| N = 30, ratio 100 | `ThirtySegments…`: runs, feasible, λ finite, durations = λ·reference |
| All waypoints equal; no limit; single limit | `AllWaypointsEqual_*` (equal split 1/3 each, `ScaledByLimits` false), `NoActiveLimit_*`, `ASingleLimit_IsEnough` |
| Peak between nodes | `WhenThePeakIsBetweenNodes_*`: waypoints (0, 2, 2.2, 5, 4.9, 6): true velocity peak 4.0 (binding), max node velocity 3.28 |
| Non-zero start velocity | as above, plus `BoundaryDerivatives_AreHonouredBySolverAndPlanner` (v, a at the start, v at the end, continuity of the interior node up to snap) |

Time: the non-zero-boundary search for data 2 took ~9 ms (about 40 solves of a 3-segment problem).

## For the author (educational)

- **Why scaling all durations by λ scales the k-th derivative by λ⁻ᵏ and keeps the optimal shape.** If every segment is stretched in time by the same factor, the trajectory x(t) becomes x(t/λ): the same curve traversed more slowly. One time derivative brings a factor 1/λ each (chain rule), so velocity scales as λ⁻¹, acceleration as λ⁻², jerk as λ⁻³, snap as λ⁻⁴. The smoothness cost ∫(x⁽ᵐ⁾)² dt becomes λ^(1−2m) times the old one for *every* admissible curve, so the ranking of curves does not change: the old minimiser, stretched, is the new minimiser. That is why the shape is unchanged. It needs the boundary conditions to scale with it: zero derivatives stay zero, but a prescribed start velocity of 1.5 is a physical number that does not become 1.5/λ when the trajectory is stretched; the stretched curve would no longer satisfy the condition, the "same shape" argument fails and the problem has to be solved again for each λ (the search).
- **Why one solve plus a formula replaces a search.** The scaling law gives the peaks for every λ from the peaks at λ = 1: P_k(λ) = λ⁻ᵏ P_k(1). The feasibility condition P_k(1) λ⁻ᵏ ≤ L_k can be solved for λ in closed form, order by order, and the answer is the largest of the four. No iteration is needed; the second solve is only a check. In the data 1 example the acceleration decides: (4.0946/3)^(1/2) = 1.1683, larger than the other three.
- **What is still not optimal.** All segments share the same λ, so the *proportions* of the durations never change. A sequence with one very long segment keeps it long, even if the limit binds in a short one: the short segment is slowed and the long one is slowed by the same factor, although the long one had plenty of room. A truly shortest trajectory would let each duration vary (a nonlinear optimisation); that is not part of this baseline.
- **An observation from the numbers.** A start velocity against the direction of travel (−1.5 for waypoints going up) multiplies the total duration by almost 4 (λ 5.9 vs 1.49): the trajectory must first stop and reverse, and doing that within the acceleration/velocity limits dominates. With a start velocity along the motion the sequence is *faster* than from rest (1.19).

## Discrepancies between theory and code

None for the formulation. Two remarks for the theory text: (1) in the reference values the "reference durations" (0.7, 1.3, 0.9) are not those of the A8 rule; the code's default automatic mode uses A8 with a 1 s total (a different λ, the same final durations when the same proportions are used); (2) the sampled peak 39.727378 vs the exact 39.727413 (see above): the theory says "to about 10⁻⁶ relative", which is met (9·10⁻⁷).

## Open questions

1. Should `referenceDurations` be exposed on the builder (and later in the UI, R10) so that a user can give proportions without fixing the scale?
2. The non-zero-boundary search starts from the zero-boundary λ*; the lower bound from the necessary conditions for the first/last segment (theory) was not implemented. It is not needed for correctness (the search covers [λ0/1000, 1000 λ0]) but it could shorten the search.
3. 3D pass-through with limits (component-wise with common durations needs a common λ across axes: the maximum of the three λ*; out of scope here).

## Suggested commits

1. `code: pass-through solver accepts non-zero boundary derivatives; ExtremaAnalyzer.PeakAbsolute` — `src/TrajectoryPlanner.Core/PassThrough/PassThroughProblem1D.cs`, `src/TrajectoryPlanner.Core/PassThrough/PassThroughSolver1D.cs`, `src/TrajectoryPlanner.Core/Constraints/ExtremaAnalyzer.cs`
2. `code: exact pass-through with kinematic limits (common scale λ), builder support` — `src/TrajectoryPlanner.Core/PassThrough/PassThroughPlanner1D.cs`, `src/TrajectoryPlanner.Core/Services/TrajectoryBuilder1D.cs`, `src/TrajectoryPlanner.Core.Tests/PassThrough/PassThroughWithLimitsTests.cs`
3. `coord: close request 0011` — `coordination/inbox-code/0011-passthrough-with-limits.md`, `coordination/inbox-docs/0011-passthrough-with-limits.md`, `coordination/STATUS.md`

(Requests 0009 and 0010 are still uncommitted in the working directory at the time of writing; their files are listed in their own notes.)
