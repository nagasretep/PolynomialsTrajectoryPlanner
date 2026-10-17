# 0005 — Shortest feasible duration with limits (return note)

Status: closed (reviewed by Cowork 2026-10-17) · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0005-shortest-feasible-duration.md`

## What was done

- New `Core/Constraints/ShortestFeasibleDuration.cs` (static class) implementing *Shortest Feasible Duration* of `theory/TrajectoryConstraints.md`:
  - `Find(degree, start, end, limits)`: dispatches to the **closed form** when all prescribed boundary derivatives of order 1 … p−1 are zero (|·| ≤ 1e-12), otherwise to the **search**;
  - `RestToRest(degree, distance, limits)`: T* = max_k (|Δ| c_k / L_k)^(1/k), with c_k = peak of |x⁽ᵏ⁾| of the unit profile (Δ = 1, T = 1) computed by `ExtremaAnalyzer`; for Δ = 0 returns `MinimumDuration`;
  - `Search(...)`: public too, so that tests can compare it with the closed form;
  - `Feasibility(...)`: F(T) = max_k P_k(T)/L_k − 1.
- **Callers**: `ConstraintAwareDuration` now returns the shortest feasible duration (its `initialGuess` constructor parameter is gone: it has no meaning any more); the App's "auto duration with limits" for plain polynomial segments uses it. A prescribed duration stays *verify only* (error message "Segment n violates the active kinematic limits at the specified duration", as before). With no active limit nothing is searched (the App keeps its default of 1.0 s; `Find` throws `InvalidOperationException`).
- **Reported to the user**: the Coefficients tab of the App now has a first row `T [s]` for every segment (prescribed or computed duration). Failure reaches the user as the message of 0003.
- **Not changed**: `DurationSearcher.FindValidDuration` (legacy doubling search) stays, because constant-velocity transitions (App and Core) still call it and constant-velocity durations are out of scope of this request. Candidate for removal in the consolidation.

## Search parameters (choices)

| Parameter | Value | Why |
|---|---|---|
| Geometric ratio ρ | **1.05** | as suggested; a feasible window narrower than 5 % can be skipped (caveat in the theory) |
| Start | max(1 ms, \|Δx\|/v_max, \|Δx⁽ᵏ⁻¹⁾\|/L_k for each active limit with prescribed boundary x⁽ᵏ⁻¹⁾) | necessary conditions (mean ≤ peak), so nothing shorter can be feasible. The velocity one is the one the theory names; the others are the same argument for acceleration (change of boundary velocity / a_max), etc. |
| Cap | 10⁶ × max(start, 1 s) | enough for any realistic case; the closed form has no cap |
| Feasibility tolerance | F ≤ 10⁻⁹ | relative; consistent with the 10⁻¹⁰ absolute used by `VerifyLimits` |
| Bisection | between the last infeasible and the first feasible duration, until relative width 10⁻¹³ (max 100 steps) | |
| Floor | 1 ms (`MinimumDuration`), also for the closed form | the two paths must agree; also the answer for a zero-length rest-to-rest segment, where every duration is feasible |

If the starting duration is already feasible it is returned (it is a necessary lower bound, hence the shortest). Cost: the search path took ~2 ms in a typical non-rest case and ~5 ms until it reports a failure (about 280 evaluations).

## Educational note: why the rest-to-rest case has a closed form and the general case does not

*Rest-to-rest*: every boundary derivative of order ≥ 1 is zero, so the whole profile is one fixed shape stretched by the distance and by time: x(t) = x₀ + Δ·φ(t/T). Differentiating k times gives x⁽ᵏ⁾ = Δ φ⁽ᵏ⁾(t/T) / Tᵏ, therefore P_k(T) = |Δ| c_k T⁻ᵏ, where c_k is a number that depends on the degree only. Example, degree 9, Δ = 1, v_max = 1: c₁ = 315/128 = 2.4609, so P₁(T) = 2.4609/T and the limit P₁ ≤ 1 gives T* = 2.4609 s immediately (the tests also find it by search: 2.4609375). Two limits at once simply take the larger T*.

*General case*: with a non-zero boundary velocity the shape is no longer a pure stretch, because the prescribed velocity is a physical value that does not scale with T. Example, degree 7, x: 0 → 3, v: +1 → −0.5, v_max = 2, a_max = 3: here P_k(T) is not a power law; the search gives T* = 2.98605 s, and there is no formula for it (a feasible duration is also not guaranteed in general: a prescribed boundary velocity of 10 against v_max = 1 is infeasible at *every* T, which is now reported as an error). The theory's remark that F(T) is not necessarily monotone is why the search starts from a lower bound and takes the first feasible duration, rather than bisecting over an arbitrary interval.

## Behaviour change (item 4)

Same segment, auto duration with a velocity limit, degree 9, Δ = 1 (checked by running the App's `PrototypeTrajectoryService` and the legacy method):

| | legacy `FindValidDuration` | now |
|---|---|---|
| v_max = 1 | 2.4609375 | 2.4609375 (T* = 315/128; same because the legacy first guess, 1 s, was infeasible, so it doubled and bisected) |
| v_max = 10 | **1.0 s** (the first guess is returned if it is feasible) | **0.24609 s** (T* = 315/(128·10)) |
| non-rest example above (degree 7) | 2.986051483 | 2.986051479 |

So results differ only where the legacy initial guess (1 s in the App) was already feasible: the new one is shorter, as the author decided. Plots of such segments get faster; peak values touch the limits.

## Acceptance criteria — where verified

| Criterion | Test(s) in `Constraints/ShortestFeasibleDurationTests.cs` |
|---|---|
| Rest-to-rest velocity limit, degrees 3/5/7/9, c₁ = 3/2, 15/8, 35/16, 315/128; closed form = search | `RestToRest_VelocityLimit_IsC1TimesDistanceOverVmax_FromBothPaths` (1e-6 relative), `Degree9_UnitDistanceUnitVelocity_Is315Over128` |
| Acceleration-only and jerk-only: closed form = search; √Δ and Δ^(1/3) scaling | `RestToRest_SingleHigherOrderLimit_ClosedFormAndSearchAgree` (orders 2, 3, 4), `…AccelerationOnly_ScalesAsSqrtOfDistance`, `…JerkOnly_ScalesAsCubeRootOfDistance` |
| Several limits: the larger wins | `RestToRest_SeveralLimits_TheLargestIndividualDurationWins` |
| Non-zero boundary velocity: F(T*) ≈ 0, F > 0 slightly below and on a fine grid below, not longer than any feasible grid duration | `NonZeroBoundaryVelocity_ResultIsTheBoundaryBetweenInfeasibleAndFeasible` (degrees 5/7/9), `…_IsNotLongerThanAnyFeasibleDurationOnTheGrid` |
| Infeasible: failure, not a wrong duration | `BoundaryVelocityAboveTheVelocityLimit_ReportsFailure`, `BoundaryAccelerationAboveTheAccelerationLimit_ReportsFailure` |
| Prescribed duration with violated limit: reported, not changed | App behaviour, **not in `Core.Tests`** (the App has no tests): verified by running `PrototypeTrajectoryService.Compute` from a throwaway console project in the scratchpad (not part of the repository): auto duration → 2.4609375 and row `T [s]`; prescribed 1 s with v_max = 1 → "Segment 1 violates the active kinematic limits at the specified duration."; boundary velocity 10 vs v_max 1 → "Segment 1: No duration between 1 s and 1E+06 s satisfies the active kinematic limits…"; no limit → 1 s. I did **not** run the WPF window itself. |
| Others | no limit → `InvalidOperationException`; non-positive limit → `ArgumentException`; zero distance → `MinimumDuration`; direction independence; result feasible and any 0.01 % shorter infeasible; `ConstraintAwareDuration` = shortest; legacy-vs-new example |

## Tests

`dotnet build`: 0 errors (3 NU1701 warnings, unchanged). `dotnet test`: **173/173 passed** (137 before; 36 new). One existing test adapted: `ConstraintAware_PerSegmentResult_SatisfiesTheLimits` (0004) no longer passes an initial guess to `ConstraintAwareDuration`.

## Discrepancies between theory and code

None. Two details beyond the text that Cowork may want in the theory: (1) the lower bounds for orders other than velocity (change of the boundary derivative of order k−1 divided by L_k); (2) the 1 ms floor and the zero-length rest-to-rest case (T* = 0 mathematically, 1 ms in the code).

## Open questions

1. Zero-length rest-to-rest segment with active limits: returns 1 ms; in the App this gives a nearly instantaneous segment. Is it better to fall back to the application's default duration (1 s) in that case?
2. Constant-velocity transitions still use the legacy search; to be aligned in the consolidation (they would then also get shortest durations).
3. Pass-through with limits (R11) can reuse the scaling remark of the theory (common factor on all durations).

## Cowork review (2026-10-17)

Accepted; the behaviour change (0.246 s instead of 1 s for $v_{max} = 10$) is what the author decided. Replies:

1. **Zero-length rest-to-rest**: the limits determine nothing there; fall back to the prescribed/default duration, not 1 ms (request 0006; the theory now says so).
2. **Constant-velocity transitions** keep the legacy search until D8 is decided; they are aligned in the consolidation.
3. **Beyond the text**: the lower bounds for orders other than velocity are now in the theory (*Shortest Feasible Duration*, search step 1); the 1 ms floor is an implementation protection, not an answer.

## Suggested commits

1. `code: shortest feasible duration (closed form + search), use it for auto duration with limits, show T in the coefficient report` — `src/TrajectoryPlanner.Core/Constraints/ShortestFeasibleDuration.cs`, `src/TrajectoryPlanner.Core/Services/IDurationStrategy.cs` (note: also holds the 0003 proportional-duration changes), `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs` (also holds the 0003 label/failure changes), `src/TrajectoryPlanner.Core.Tests/Constraints/ShortestFeasibleDurationTests.cs`, `src/TrajectoryPlanner.Core.Tests/Constraints/ExtremaAndDurationTests.cs`
2. `coord: close requests 0003-0005` — `coordination/inbox-code/0003-…`, `0004-…`, `0005-…`, `coordination/inbox-docs/0003-…`, `0004-…`, `0005-…`, `coordination/STATUS.md`

If the author prefers one commit per request, `IDurationStrategy.cs` and `PrototypeTrajectoryService.cs` contain changes of both 0003 and 0005 (use `git add -p`).
