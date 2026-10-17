# 0007 — Constant-velocity transitions and tests (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0007-constant-velocity-transitions.md`

## What was done

- **`ConstantVelocityTrajectory1D` (Core) rewritten** (the polynomial/linear pieces are the same as before):
  - With limits active, entry and exit each take `ShortestFeasibleDuration.Find(...)` of their own transition (independently). The legacy `DurationSearcher.CreateSegmentWithAutoDuration` / `FindValidDuration` are no longer used by this class.
  - **Explicit failures**: speed above the velocity limit → new `KinematicLimitsViolatedException` ("The constant speed … exceeds the velocity limit …: no transition can repair it"), raised first, also when the durations are prescribed; transition search failure → `DurationSearchFailedException` whose message names "entry transition"/"exit transition"; a prescribed duration that violates a limit → `KinematicLimitsViolatedException`. Every piece is re-verified with `ExtremaAnalyzer.VerifyLimits` before the object exists, so a trajectory that violates the limits cannot be returned.
  - **Optional `entryDuration` / `exitDuration`**: kept as given, only verified; non-positive → `ArgumentException`.
  - **Without limits and without prescribed durations**: unchanged rule `max(0.5·T_cv, 0.1 s)`, now the public static `DefaultTransitionDuration(T_cv)` documented as provisional. The same value is the fallback when `Find` returns `null` ("not determined", 0006).
  - **New constructor taking `State1D startState, State1D endState`** (full boundary states: acceleration, jerk, snap), because the App's constant-velocity segments carry them; the old `(x0, xf, v0, vf, …)` signature remains and delegates (existing callers and tests unchanged).
  - **Small fixes found on the way** (all in this class): (1) `vCv` is now taken as a speed magnitude; before, a negative `vCv` combined with the direction sign gave a velocity opposite to the travel direction; (2) `vCv = 0` with different positions used to give `cvDuration = ∞` without an exception (the request says "existing exception": it only existed for equal positions); both cases now throw `ArgumentException`; (3) the old test `limits != KinematicLimits.None` compared references (`None` creates a new object each time), so the limits branch always ran; now "limits active" means at least one limit is set; (4) `EvaluateState` at the very end could request a local time a few ulps beyond the exit duration and throw; it is clamped.
- **`ShortestFeasibleDuration.FeasibilityTolerance` set to 0** (was 1·10⁻⁹ relative). Reason: `ExtremaAnalyzer.VerifyLimits` accepts only 10⁻¹⁰ absolute slack, so a duration that was feasible within a relative 10⁻⁹ could fail the verification that now runs on every piece. With 0 the result is the shortest duration with peak ≤ limit; the effect on durations is ≤ 10⁻⁹ relative (all 0005/0006 tests unchanged and passing).
- **App**: `PrototypeTrajectoryService.BuildConstantVelocitySegments` no longer builds anything itself: it parses the inputs, applies the App-level rule for a prescribed total duration and calls the Core class; Core exceptions are prefixed with "Segment n:". Removed from the App: its own construction of entry/stretch/exit, the legacy-search helper `FindDurationOrExplain` and the use of `DurationSearcher` (the constant-velocity stretch is read from `ConstantVelocity.ToPolynomialSegments()`). The App's rule for a **prescribed total duration** is unchanged: remaining time `T_tot − T_cv` split equally, passed to Core as prescribed entry/exit (so only verified against limits), commented as a provisional App-level choice.
- **Legacy `FindValidDuration`** is no longer used by any production code. Remaining users: only `CreateSegmentWithAutoDuration` (itself unused) and tests (`SegmentAndConstraintsTests`, `CleanupBatchTests`, `ShortestFeasibleDurationTests` — the "legacy vs new" comparison). I did not delete it because that would remove tests; it is a candidate for removal in the consolidation (R7), with those tests.

## Tests (new file `Core.Tests/ConstantVelocity/ConstantVelocityTrajectoryTests.cs`, 40 cases)

| Requested | Test(s) |
|---|---|
| Rest-to-rest, degrees 3/5/7/9, both directions: durations, continuity, total = T_in + T_cv + T_out | `RestToRest_ContinuousAtBothJunctions_AndTotalDurationIsTheSum` (8 cases); junction check compares orders 0 … p−1 at 1e-9, plus zero a/j/s of the stretch |
| Non-zero boundary velocities, including opposite to the stretch | `NonZeroBoundaryVelocities_IncludingOppositeToTheStretch` (3 cases), `FullBoundaryStates_AreHonoured` |
| Each limit binding in turn; durations = shortest feasible (1e-9) | `WithLimits_TransitionsTakeTheirShortestFeasibleDuration_…` (v, a, j, s for degree 9; a for 7 and 5; v for 5): also checks F ≈ 0 (binding) and that every piece passes `VerifyLimits`; `Limits_CanBeCombined_AllRespected`; `EntryAndExit_AreSizedIndependently_…` |
| `vCv` over the velocity limit; impossible transition limits | `ConstantSpeedAboveTheVelocityLimit_*` (2), `ImpossibleLimitsForATransition_*` (entry, exit, message names the piece) |
| Prescribed durations kept and verified | `PrescribedDurations_AreKeptAsGiven_*` (2), `PrescribedEntryDurationViolatingALimit_IsReportedNotRepaired`, `OnePrescribedAndOneComputedDuration`, non-positive rejected |
| No limits: today's rule | `Today_WithoutLimitsEachTransitionTakesMaxOfHalfTheStretchAndTenthOfASecond` (3 cases incl. the 0.1 s floor and a negative direction) |
| Degenerate input | equal positions, zero speed, negative speed taken as magnitude |
| Evaluation | at the very end, `GetAllSegments` |

`dotnet build`: 0 errors, no new warnings (the 3 NU1701 as before; the App compiled to a separate output folder because the app was running). `dotnet test`: **228/228 passed** (188 + 40).

App path (throwaway console project in the scratchpad calling `PrototypeTrajectoryService.Compute`, degree 9, x 0→100, stretch 20→80 at speed 15; the WPF window was not run):

| input | result |
|---|---|
| no limits, no duration | total 8 s: entry 2, stretch 4, exit 2 (today's rule) |
| prescribed total 20 s | entry 8, stretch 4, exit 8 (equal split) |
| prescribed total 3 s | "duration must be greater than the constant-velocity stretch duration (4.0000)" |
| a_max = 6, no duration | entry = exit = 13.0113 s, total 30.02 s |
| v_max = 10 (speed 15) | "Segment 1: The constant speed 15 exceeds the velocity limit 10: no transition can repair it." |
| prescribed total 8 s with a_max = 2 | "Segment 1: The entry transition (duration 2 s) violates the active kinematic limits." |

## For the author (educational)

- **Why the two transitions are sized independently.** Each transition joins two *fixed* states: the outer one (given by the neighbouring trajectory or a rest state) and the inner one (position, constant velocity, zero acceleration/jerk/snap, dictated by the stretch). Its polynomial is therefore completely determined once its duration is chosen, and nothing the exit does can change the entry. So the entry's shortest feasible duration is a one-segment problem, the exit's another, and the total is their sum plus T_cv.
- **Why the stretch is the one thing the limits cannot change.** Its duration is |Δx|/v_cv and its velocity is v_cv, both fixed by the user. Making a transition longer or shorter never alters them, so if v_cv > v_max no choice of the other pieces can make the trajectory legal.
- **Exception versus a trajectory that merely looks feasible.** Before, a failed search returned a number, and the plotted curve looked normal while its peak velocity exceeded the limit. Now the object is only created if every piece passed the extrema check; otherwise nothing is returned and the message says which piece and why. In the App this appears in the status bar and message box instead of a plot that silently breaks the limits.
- **A surprise worth knowing** (a_max = 6 above): the transition takes 13 s although the stretch is only 4 s. The entry must cover 20 units while arriving at speed 15 *with zero acceleration, jerk and snap* (the stretch's state), a back-loaded profile that, for degree 9, needs a long, gentle build-up to respect 6 units/s². It is the shortest duration that satisfies the limit with this boundary condition, not an error. With a lower degree (fewer constraints at the junction) or a longer lead-in distance the transition is shorter.

## Discrepancies between theory and code

None. The code implements Rule 1 and the documented provisional rules as written. Observation for the theory: in 1D a transition's boundary states can only coincide if the entry starts exactly at the stretch with the same velocity and zero a/j/s, in which case the polynomial is still non-trivial (it must leave and return), so the "not determined by the limits" fallback of the text (item 5) is in practice never reached in this class; it exists as a safety net.

## Open questions

1. Rule for the prescribed total duration: equal split is kept (provisional). With the 13 s example above, an equal split of a short remaining time can easily violate limits; a split proportional to the shortest feasible durations would be a natural third option for the author to consider.
2. Remove `DurationSearcher.FindValidDuration` / `CreateSegmentWithAutoDuration` and their tests in the consolidation?
3. Limits on the stretch other than velocity are trivially satisfied (a = j = s = 0); only the velocity limit is checked for the stretch.

## Suggested commits

1. `code: constant-velocity transitions use the shortest feasible duration, explicit limit failures, App uses the Core class` — `src/TrajectoryPlanner.Core/ConstantVelocity/ConstantVelocityTrajectory1D.cs`, `src/TrajectoryPlanner.Core/Constraints/KinematicLimitsViolatedException.cs`, `src/TrajectoryPlanner.Core/Constraints/ShortestFeasibleDuration.cs` (feasibility tolerance 0; this file also holds the 0006 changes), `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs` (also holds the 0006 default-duration change)
2. `code: constant-velocity test suite` — `src/TrajectoryPlanner.Core.Tests/ConstantVelocity/ConstantVelocityTrajectoryTests.cs`
3. `coord: close request 0007` — `coordination/inbox-code/0007-constant-velocity-transitions.md`, `coordination/inbox-docs/0007-constant-velocity-transitions.md`, `coordination/STATUS.md`

Note: if request 0006 is not yet committed, its files (`PassThroughProblem1D.cs`, `IDurationStrategy.cs`, the 0006 tests) are still in the working directory and belong to the 0006 commit (see its note).
