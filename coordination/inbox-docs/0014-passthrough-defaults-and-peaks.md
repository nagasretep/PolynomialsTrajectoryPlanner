# 0014 — Pass-through: uniform default durations and peaks against limits (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0014-passthrough-defaults-and-peaks.md`

## Summary

Both points done. Tests: Core.Tests 336 → **338** (+2), App.Tests 92 → **100** (+8), **438 total**, all passing. `dotnet build`: 0 errors, no new warnings (NU1701 as before; the xUnit2000 hints in `PassThroughWithLimitsTests.cs`, lines ~134-136, date from 0011 and are not touched here). Blending mode, 3D and the Core algorithms are unchanged; the only Core addition is the accessor asked for.

## 1. Uniform default durations

With blank segment durations and no total, the reference durations are now **always** the A8 proportional rule with a total of `number of segments × 1 s`, with and without limits.

- Without limits the planner uses them as they are (`referenceDurations` → `Solve(reference)`, λ = 1): the "each segment takes 1 s" path (`Enumerable.Repeat`) is gone.
- With limits the planner scales the same reference by λ.
- Where the rule lives: it already lived in the Core (`AutoDerivedProportionalDuration`, which the service calls for the total-given case too), so it is not duplicated. The App only chooses the total (`segmentCount × ReferenceDurationPerSegment`, constant renamed from `FallbackSegmentDuration`) and passes the result as `referenceDurations`. I did not change the planner's own default (A8 with a 1 s total) because that would change `Plan(...)` for its other callers and tests; the service passes the reference explicitly.
- Result: (0, 1, 3) without limits → durations 2/3 s and 4/3 s (ratio 1 : 2), total 2 s. With a limit, the same proportions are scaled (test: ratio equal, `λ·T_ref` = result). (0, 1, 2) unchanged: total 2 s, interior velocity 2.275, and with the four reference limits λ = 1.168271 as in 0013.

Behaviour change to know: for **unequally** spaced waypoints without limits the durations used to be 1 s each and are now proportional to the distances.

## 2. Peaks against the limits

- **Core accessor** `ExtremaAnalyzer.PeakAbsoluteWithLocation(segments, order)` → `(Peak, SegmentIndex (zero-based), LocalTime)`; first segment wins a tie; the peak is the same value as `PeakAbsolute`. Tests: `Constraints/PeakLocationTests.cs` (2): peak and segment of two rest-to-rest moves of equal distance and different duration (315/128 at the shorter one, time near its middle, equal to `PeakAbsolute`); a negative excursion is reported as an absolute value.
- **Service**: `PrototypeComputationResult` has a new last member `LimitPeaks` (list of `LimitPeakRow(Quantity, Peak, Limit, PercentOfLimit, Segment)`). For every **active** limit the degree supports it has one row: peak over the whole trajectory, the limit, `100 · peak / limit`, and the segment numbered from 1. Empty list when no limit is active.
- **UI**: the Summary tab has a second table, "Peaks against the limits", shown only when there are rows (so not when no limit is active). Numbers are bound with `StringFormat`, which follows the current culture (0012).
- Check (test `LimitedCase_…`, data of the 0013 acceptance): acceleration at 100 % (1e-4 on the percentage; the underlying Core check is 1e-9 in 0011), velocity, jerk and snap below 100 %, all segments in 1…2.

## Tests added (App.Tests/PassThroughDefaultsAndPeaksTests.cs, 8)

Unequal spacing without limits (ratio 1 : 2, total 2); equal spacing unchanged (2 s, 2.275); same proportions before scaling with a limit; no limit → empty peaks list; binding limit at 100 % and others below, segments from 1; the peak segment is the one with the large distance (waypoints 0, 0.1, 2.1 with 1 s each); a limit the degree does not support is not listed (degree 3, snap); `it-IT` (decimal comma in input, the summary text, peak percentage 100).

One test of mine was wrong at first and was corrected, not the code: with durations 2 s and 0.8 s over equal distances I assumed the peak lies in the shorter segment; the peak was in segment 1, because the optimizer chooses a high velocity at the interior node (see below). I replaced it with a case where the distance, not the duration, differs.

## For the author (educational)

- **Why equal durations on unequally spaced waypoints are a bad default.** The trajectory has to cover 1 unit in the first segment and 2 units in the second, and with equal durations the second segment needs twice the average speed. The minimum-snap solution cannot stay smooth *and* do that, so it compensates: it raises the velocity at the interior waypoint and brakes hard afterwards, so the large derivatives (acceleration, jerk, snap) appear around that node, and the plot shows an uneven profile. If the time is distributed in proportion to the distances, the average speed is the same in both segments and the optimizer has nothing to compensate; the derivatives are smaller and the shape is calm. This is also why limits are reached later (shorter total) with the proportional rule.
- **What "one second per segment in total" means.** The rule A8 only fixes *proportions*; it needs a total to turn them into seconds. The total chosen is "one second for each segment", i.e. N − 1 seconds for N waypoints: for equally spaced waypoints every segment lasts exactly 1 s (as before), for unequal spacing the same total is shared out according to the distances (0.67 s and 1.33 s for 1 and 2 units). Without limits these are the durations; with limits they are only the first guess that λ stretches or shrinks.
- **How to read the peaks table.** "% of limit" is the peak divided by the limit. After λ has been applied, the quantity that decided the timetable is at 100 %; the others are below, and their margin shows how far they were from being the problem. "Segment" is where the peak occurs, counted from 1 like the waypoint table, so it tells where the trajectory works hardest.

## Discrepancies between theory and code

None. The first open question of 0013 (reference proportions) is closed by this request.

## Open questions

1. `Plan(...)` still defaults to A8 with a 1 s reference total when no `referenceDurations` is given; the App now passes N × 1 s explicitly. If the planner default should also become "1 s per segment", that is a Core change (all callers) not requested here.
2. The peak table lists only limits that are active. Listing also the peaks of the quantities without limit (informational) would be possible; the Extrema tab already shows them.

## Suggested commits

1. `code: Core — ExtremaAnalyzer.PeakAbsoluteWithLocation (peak and segment of a whole trajectory)` — `src/TrajectoryPlanner.Core/Constraints/ExtremaAnalyzer.cs`, `src/TrajectoryPlanner.Core.Tests/Constraints/PeakLocationTests.cs`
2. `code: App — one default reference rule for pass-through durations; peaks against the limits in the Summary` — `src/TrajectoryPlanner.App/PrototypeTrajectoryService.PassThrough.cs`, `src/TrajectoryPlanner.App/PrototypeModels.cs`, `src/TrajectoryPlanner.App/MainWindow.xaml`, `src/TrajectoryPlanner.App/MainWindow.xaml.cs`, `src/TrajectoryPlanner.App.Tests/PassThroughDefaultsAndPeaksTests.cs`
3. `coord: close request 0014` — `coordination/inbox-code/0014-passthrough-defaults-and-peaks.md`, `coordination/inbox-docs/0014-passthrough-defaults-and-peaks.md`, `coordination/STATUS.md`

(The uncommitted changes under `roadmap/`, `theory/` and `design/` are Cowork's and not part of these commits; the code of 0013 may still be uncommitted too.)
