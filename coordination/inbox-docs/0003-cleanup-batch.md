# 0003 — Cleanup batch (return note)

Status: closed (reviewed by Cowork 2026-10-17) · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0003-cleanup-batch.md`

## What was done

1. **Coefficient labels (D9).** `PrototypeTrajectoryService` now labels the report rows `c0 … cd`. I looked at the code only; I did **not** run the app. Other `a_i`-style names: none that denote polynomial coefficients. `a0`, `af` in `PolynomialCoefficientSolver` and `PolynomialSegment1D.CreateDegree5/7/9` are the initial/final **acceleration** (one letter per derivative, as in `v0, vf, j0, jf, s0, sf`), which is a different symbol from `a_i` coefficients; renaming them would be misleading. `PolynomialCoefficients` uses neutral names. No XAML header uses `a0…`.
2. **`DurationSearcher` failure (D5, bug part).** New `DurationSearchFailedException : InvalidOperationException` (property `LastDurationTried`). `FindValidDuration` throws it when no duration satisfies the limits after the maximum number of doublings; the search itself (initial guess returned if valid, doubling, bisection) is **unchanged**. *Why an exception rather than a flag*: the two other callers (`CreateSegmentWithAutoDuration`, `ConstraintAwareDuration`) return plain numbers/segments; an exception cannot be ignored by mistake, a success flag could. Callers: `ConstantVelocityTrajectory1D` and `ConstraintAwareDuration` let it propagate; the App wraps it (`FindDurationOrExplain`) into an `InvalidOperationException` that names the segment ("Segment 2 entry transition: No duration up to … satisfies the active kinematic limits …"), which the existing `catch` in `MainWindow` already shows in the status bar and a message box.
3. **`Trajectory1D.SampleTimes` (D12).** Loop by index, so a repeated segment object gets the right start time; `samplesPerSegment < 2` throws `ArgumentOutOfRangeException` **at the call** (the method is split in a checking wrapper and an iterator, otherwise the exception would only appear on enumeration). Output is unchanged for normal use (each segment from its start to its end inclusive, so node times appear twice).
4. **`Symbols.Epsilon` (D13).** Deleted; nothing used it, including the tests.
5. **Proportional durations (D14).** `AutoDerivedProportionalDuration` implements `design/07` A8: weights `max(d_i, φ·mean(d))`, `T_i = T_tot·w_i/Σw`, φ = 0.05 as constructor parameter `relativeFloor` (constant `DefaultRelativeFloor`), total preserved (checked to 1e-12), equal split if the mean distance ≤ 1e-12, Euclidean distance in 3D. The absolute 0.05 s floor and `MinSegmentDuration` are gone. Per-segment `ComputeDuration` untouched (equal split, no users; D15).

## Why (choices made)

- Exception type deriving from `InvalidOperationException` so existing `catch (InvalidOperationException)` paths and the App's generic `catch` keep working.
- "Total exactly" is floating-point exact to 1e-12, not bit-exact.

## Behaviour changes

- Pass-through with **automatic durations** changes, as expected: previously `max(0.05, T·d/Σd)`; now the A8 rule (they coincide when no segment is short; they differ when a segment is nearly zero-length or the total is small, where the old sum could exceed the total).
- A search that previously returned an unchecked, limit-violating duration now throws.

## Tests

- Added `Segments/CleanupBatchTests.cs`: 15 tests (SampleTimes ×5 incl. repeated object and rejection of 1/0/−3 samples; failure reporting ×4: impossible velocity limit with huge distance, boundary velocity above the limit, `ConstraintAwareDuration` propagation, plus the achievable case still working; proportional durations ×6: sum, proportionality, zero-distance segment gets the floor share, floor parameter, tiny total respected, all-zero equal split, 3D Euclidean).
- Removed one test of `MinimumSnapSolverTests` (the old absolute-floor test), replaced by the proportional tests above.
- `dotnet build`: 0 errors (3 NU1701 warnings, unchanged). `dotnet test`: **81/81 passed**.

## Discrepancies between theory and code

None new.

## Open questions

None.

## Cowork review (2026-10-17)

Accepted. D5 (failure part), D9, D12, D13, D14 closed in `roadmap/discrepancies.md`. Choice of an exception for the search failure: agreed.

## Suggested commits

1. `code: report duration-search failure, fix SampleTimes, relative floor for proportional durations, c_i labels, drop unused Epsilon` — `src/TrajectoryPlanner.Core/Constraints/DurationSearchFailedException.cs`, `…/Constraints/DurationSearcher.cs`, `…/Services/IDurationStrategy.cs`, `…/Trajectories/Trajectory1D.cs`, `…/Symbols/Constants.cs`, `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs`, `src/TrajectoryPlanner.Core.Tests/Segments/CleanupBatchTests.cs`, `src/TrajectoryPlanner.Core.Tests/PassThrough/MinimumSnapSolverTests.cs`
2. `coord: close request 0003` — `coordination/inbox-code/0003-cleanup-batch.md`, `coordination/inbox-docs/0003-cleanup-batch.md`, `coordination/STATUS.md`
