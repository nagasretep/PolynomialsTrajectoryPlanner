# 0017 — Core: 3D trajectory level (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0017-core-3d-trajectory-level.md`

## Summary

Implemented as specified. Tests: Core.Tests 400 → **428** (+28), App.Tests **101** (unchanged), **529 total**, all passing. `dotnet build`: 0 errors, no warnings other than the known NU1701. No App/UI change, no 1D change. The only edit to an existing test is a cast in `PassThrough/MinimumSnapSolverTests.cs` (`Trajectory3D.Segments` now holds `ISegment3D`, so the test takes the segment as `PolynomialSegment3D` to read its axes).

## What was done

**1. Common interface** — `Segments/ISegment3D.cs`: `Duration`, `EvaluateState(t)`, `EvaluatePosition(t)`, `EvaluateDerivative(t, order)`. Implemented by `PolynomialSegment3D` (no code change, the members already existed) and by `CurvedConstantSpeedStretch3D` (same). `Trajectory3D` takes `IEnumerable<ISegment3D>` (a list of `PolynomialSegment3D` still converts, so old callers compile) and gained `Parts`, `StartTimeOf(i)` and `Sample`.

**2. `TrajectoryBuilder3D`** (existing API kept: `WithDefaultDegree`, `WithSmoothnessCriterion`, `AddRestToRest`, `AddSegmentsFromWaypointsPassThrough`, `Build`):
- `AddSegment(start, end, duration?, degree?, limits?)` through the new `SegmentFactory3D` (mirror of `SegmentFactory1D`): a given duration is only verified (`KinematicLimitsViolatedException`); null with limits = the multi-component shortest feasible duration; **null without limits = `SegmentFactory1D.DefaultDuration`, 1 s, the rule of the 1D builder.**
- `AddBlendBetween(i, i+1, τ_start, τ_end, …)` and the overload with two absolute `Point3D`s, through `BlendBuilder3D.BuildBlendedSegmentPair`; as in 1D the indices are positions in the current list and the pair is replaced by truncated preceding + blend + truncated following. Only plain segments can be blended (a blend, an entry/exit or a stretch as neighbour → `InvalidOperationException`; non-consecutive → `ArgumentException`).
- `AddConstantSpeed(segmentIndex, specification, speed, degree?, limits?, entryDuration?, exitDuration?, totalDuration?)`: replaces a plain segment by entry + stretch + exit, **always via `ConstantSpeedTrajectory3D`** (also for a straight stretch). The outer states of the transitions are the segment's own start and end states; the transitions' degree is `degree`, by default that of the segment. `StretchSpecification` (new) has `FromParameterFractions`, `FromPoints`, `FromArcLengthFractions`.
- **Part descriptors**: `builder.Parts` / `trajectory.Parts`: `TrajectoryPart3D(Kind {Segment, Blend, CvEntry, CvStretch, CvExit}, OriginalSegmentIndex, SecondSegmentIndex, StartTime, Duration, Segment, InheritedDuration, AdjustedByLimits, StretchLength, StretchSpeed, AdmissibleSpeed, BindingLimit)` plus `EndTime` and a `Label` ("Segment 2", "Blend 1-2", "Entry 3", "Stretch 3", "Exit 3", numbered from 1). `OriginalSegmentIndex` counts the segments in the order they were added (for a blend, the preceding one; the following is in `SecondSegmentIndex`); the pieces from the same original share the number. For a stretch with no active limit, `AdmissibleSpeed` is null.

**3. Analysis** — `Trajectories/TrajectoryAnalyzer3D.cs`:
- `Trajectory3D.Sample(count, maxOrder)` → `TrajectorySamples3D`: `count` uniform times (ends included) **plus the junction times that are not already among them**, sorted, without duplicates (1e-12). The number of samples is therefore `count` plus at most (pieces − 1); a sample exactly at a junction belongs to the piece that starts there (continuity makes the value the same). `maxOrder` 0–4.
- `Extrema(trajectory)` → `TrajectoryExtrema3D.Of(axis, order)` for axis 0–2, order 0–4: `AxisExtremum(Minimum, MinimumTime, Maximum, MaximumTime, MaxAbsolute, MaxAbsoluteTime, MaxAbsolutePart)`, times on the trajectory's time axis; plus `MaxSpeed`/`MaxSpeedTime`. Polynomial pieces: `ExtremaAnalyzer.FindExtrema` (exact). Stretch: new `CurvedConstantSpeedStretch3D.ComponentExtrema(order)` (grid of 1000 intervals of the path parameter, golden-section refinement of the best minimum and maximum of each component, times from the arc length at the optimum). Max speed: polynomial pieces scan + golden section; a stretch has the constant `Speed`.
- `LimitReport(trajectory, limits)` → one `LimitReportRow3D(Quantity, Axis, Peak, Limit, PercentOfLimit, PartIndex, PartLabel, Time)` per active limit and axis (never throws; `Violated` = peak above the limit by more than 1e-9 relative + 1e-10 absolute). `VerifyLimits(trajectory, limits)` returns the same rows or throws `KinematicLimitsViolatedException` for the worst violation: "Segment 2 violates the velocity limit on axis x: peak 3.5 exceeds 2 (175.00 % of the limit) at t = 3.6 s."
- `PathLength(trajectory)`: polynomial pieces by `ArcLengthPath` over the whole piece (`requireRegular: false`, the quadrature of 0016, valid with rest points), stretches by their own length.

## Acceptance criteria — where verified (`Services/TrajectoryBuilder3DTests.cs` 16, `Trajectories/TrajectoryAnalyzer3DTests.cs` 12)

| Criterion | Test |
|---|---|
| `Trajectory3D` holds a curved stretch; evaluation and sampling through the interface | `TheTrajectoryWithTheCurvedStretch_EvaluatesLikeTheStretchItself` (orders 0–4 at 21 times, 1e-9), `MixedTrajectory_IsContinuousAtEveryJunction…` (six pieces: segment, blend, segment, entry, stretch, exit; orders 0–4 at all five junctions, 1e-9), `SampleTimes_ContainEveryJunction…` |
| Existing tests unchanged | all 400 + 101 pass; the one cast above |
| Along an axis the 3D builder equals the 1D builder (1e-9) | `AlongOneAxis_TheSegmentEquals…` (prescribed duration and shortest feasible duration), `…TheBlendEquals…` (with a limit), `…TheConstantSpeedStretchEquals…` (entry/exit coefficients, T_cv, total, with limits) |
| Null duration with limits = multi-component shortest feasible | `NullDurationWithLimits_…` (also equals the 1D segment of the largest distance); without limits: 1 s |
| Extrema equal a dense sampling | `Extrema_AgreeWithADenseSampling`: 30 000 samples, all axes and orders 0–4, min/max/max-abs; the exact value is never below the sampled one (1e-9) and agrees to 1e-6 (orders 0–2), 1e-4 (3), 1e-3 (4) relative to (1 + peak); the reported time reproduces the value. `MaxSpeed_…` 1e-6 |
| Limit report names binding axis and part; violated limit names part, axis, quantity | `LimitReport_ShowsTheBindingAxisAndPart` ("Segment 2", axis x, percent = 100·peak/limit), `AViolatedLimit_NamesPartAxisAndQuantity` ("Segment 2", "axis x", "velocity"), `ALimitedMixedTrajectory_PassesTheWholeVerification`, `LimitPeaksOfTheStretch_AreItsOwn` (P₂ = 2 of the parabola, at the start) |
| Path length | straight segment = chord (1e-10); theory parabola 1.478943 (and 1e-12 relative to the closed form); a mixed trajectory = sum of its pieces (1e-12) and = a 60 000-point polyline (1e-4) |
| Builder behaviour | part kinds/originals/start times/labels, blend inherited duration and flag, stretch length/speed/admissible speed and binding limit, the three specifications, blend errors, constant-speed errors, prescribed total |

## Design choices to know

- **Interface placement**: `ISegment3D` lives in `Segments` next to `PolynomialSegment3D`; it is deliberately minimal (the four members the trajectory needs). Things only a polynomial has (`XAxis`, coefficients, `StartState`) stay off it; the analyzer does a type switch (polynomial, stretch, and a generic sampled fallback for any other future piece).
- **Descriptors travel with the trajectory** (`Trajectory3D.Parts`), not only with the builder, so that the analysis can say "Blend 1-2" in a message; a trajectory built by hand gets default descriptors (all `Segment`, numbered in order).
- **Part index vs original index**: `PartIndex` in the report is the position in `Parts`/`Segments` (the table row), `OriginalSegmentIndex` is the numbering the user knows. The label uses the original numbering.
- **Stretch replaces the whole segment**: the original segment's portions before and after the stretch are not kept; the entry and exit transitions, which start and end at the segment's own boundary states, replace them (as the theory says: the path is preserved on the stretch only).
- **One `KinematicLimits` for all three axes**, as requested.

## For the author (educational)

- **Why a common interface rather than a special case in `Trajectory3D`.** The trajectory only needs to ask a piece four questions: how long are you, and what are your position, derivative and state at time t? Any object that can answer them can be a piece. With a special case (`if (piece is the curved stretch) …`) every function that walks the trajectory (evaluation, sampling, extrema, limits, length) would need its own branch, and a third kind of piece would mean editing all of them again. With the interface, evaluation and sampling are written once and keep working; only the analysis that needs exact knowledge (peaks of a polynomial versus peaks of a non-polynomial stretch) switches on the type, in one place.
- **Why the builder returns part descriptors.** The builder is the only place that knows what each piece *is*: it created the blend, it knows which two segments it came from, whether the limits changed its duration, what the admissible speed of the stretch is. Rebuilding that from the list of segments afterwards would mean guessing (a short segment next to a stretch looks like any other segment). If the user interface derived it, the same inference would be written again in every client and could disagree with the builder. The descriptor is the builder's statement of fact, passed on unchanged.
- **How the peak of a polynomial part is found without sampling.** The extrema of a smooth function are at the ends of the interval or where its derivative vanishes. For a polynomial the derivative is another polynomial, so its zeros are the real roots of a polynomial of lower degree, which can be found reliably; evaluating the quantity at those roots and at the ends gives the exact extrema, with no grid that might step over a narrow peak. Sampling is used only where there is no polynomial to differentiate: the curved stretch, whose derivatives are not polynomials in time, and the speed ‖v‖ (the square root of a polynomial), which is scanned and refined.

## Discrepancies between theory and code

None.

## Open questions

1. `AddConstantSpeed` replaces the whole segment. If the portions of the original segment outside the stretch should be kept (the stretch carved out of the middle, with the transitions as additional pieces), that is a different construction not in the theory; I did not do it.
2. The transitions' default degree is that of the segment (`degree ?? segment.Degree`), not the builder's default degree; say if the other is wanted.
3. Per-axis limits and several stretches in one segment are out of scope, as listed. 3D pass-through with limits (R9c) is the next Core piece for 3D mode.
4. Cost: `Extrema` on a trajectory with a stretch evaluates about 1 000 jets for each of the 3 × 5 (axis, order) pairs plus refinements (≈ 0.1 s); the UI should call it once per Compute, not per mouse move. `Sample` costs one inversion per sample inside a stretch.

## Suggested commits

1. `code: ISegment3D; Trajectory3D holds polynomial segments and curved stretches, with part descriptors and sampling` — `src/TrajectoryPlanner.Core/Segments/ISegment3D.cs`, `src/TrajectoryPlanner.Core/Segments/PolynomialSegment3D.cs`, `src/TrajectoryPlanner.Core/Trajectories/Trajectory3D.cs`, `src/TrajectoryPlanner.Core/Trajectories/TrajectoryPart3D.cs`, `src/TrajectoryPlanner.Core.Tests/PassThrough/MinimumSnapSolverTests.cs`
2. `code: TrajectoryBuilder3D with segments, blends, constant-speed stretches and part descriptors` — `src/TrajectoryPlanner.Core/Services/TrajectoryBuilder3D.cs`, `src/TrajectoryPlanner.Core/Services/SegmentFactory3D.cs`, `src/TrajectoryPlanner.Core/ConstantVelocity/StretchSpecification.cs`, `src/TrajectoryPlanner.Core/ConstantVelocity/ConstantSpeedTrajectory3D.cs` (also `ComponentExtrema` and `ISegment3D`), `src/TrajectoryPlanner.Core.Tests/Services/TrajectoryBuilder3DTests.cs`
3. `code: whole-trajectory analysis in 3D (extrema, limit report, path length)` — `src/TrajectoryPlanner.Core/Trajectories/TrajectoryAnalyzer3D.cs`, `src/TrajectoryPlanner.Core.Tests/Trajectories/TrajectoryAnalyzer3DTests.cs`
4. `coord: close request 0017` — `coordination/inbox-code/0017-core-3d-trajectory-level.md`, `coordination/inbox-docs/0017-core-3d-trajectory-level.md`, `coordination/STATUS.md`

(0013–0016 may still be uncommitted; `ConstantSpeedTrajectory3D.cs` belongs to 0016 and 0017 together. `roadmap/`, `theory/`, `design/` untouched.)
