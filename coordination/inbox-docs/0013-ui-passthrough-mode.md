# 0013 — UI: pass-through mode in 1D (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0013-ui-passthrough-mode.md`

## Summary

The App has a second mode, **Pass-through**, selected at the top of the window (`Blending | Pass-through`). Tests: Core.Tests **336** (unchanged count), App.Tests **68 → 92** (+24), **428 total**, all passing. `dotnet build`: 0 errors; warnings only the known NU1701. Blending mode is unchanged (its 68 tests pass untouched). The window itself was not clicked through (no UI automation); a smoke test loads the real `MainWindow` and switches mode, see the hand-check list.

## What was done

**Service** (`App/PrototypeTrajectoryService.PassThrough.cs`, the service class is now `partial`): `ComputePassThrough(PassThroughInputModel)` parses the table, end conditions and limits, calls `PassThroughPlanner1D.Plan` (0011), and builds the same `PrototypeComputationResult` as blending mode, with three new optional trailing members: `WaypointMarkers`, `WaypointStates`, `SummaryRows` (null in blending mode). The plot sampling was extracted into a private `SampleSeries` shared by both modes. No duration, λ or limit algorithm is in the App.

How the user's inputs map to the planner (the only decisions in the App):

| Inputs | Planner call |
|---|---|
| all segment durations filled | `prescribedDurations` (kept, verified; violation = error naming segment and limit) |
| total given, durations blank | the Core class `AutoDerivedProportionalDuration(total)` (rule A8) → `prescribedDurations` (verify only) |
| nothing given, no active limit | 1 s per segment (`FallbackSegmentDuration`) → `prescribedDurations` |
| nothing given, limits active | `referenceDurations` = A8 proportions with total = number of segments → the planner computes λ and the shortest feasible sequence |

Input errors (all `InvalidOperationException`, message names the field): fewer than two waypoints; non-numeric text (`InputNumber`, so the decimal comma works under `it-IT`); durations partly filled (names the first blank row); durations together with a total; non-positive duration/total/limit; smoothness order outside 1…p.

**Inputs** (`PassThroughInputModel`, `WaypointInputModel` in `PrototypeModels.cs`): degree; m (default snap for degree 9, else p; reset to the default when the degree changes; dropdown 1…p); start/end conditions (rest checkbox, else v, a, j, s as the degree supports: v from degree 3, a from 5, j from 7, s from 9); waypoint table with Add/Remove; total duration; four limits (those the degree supports; a limit the degree does not support is ignored, as in blending mode).

**UI** (`MainWindow`): radio buttons `Blending | Pass-through`; each mode has its own input model, so switching deletes nothing; each mode also keeps its last result and re-shows it on return. The toolbar shows the buttons of the active mode (Add/Remove Segment vs Add/Remove Waypoint). The same five stacked plots, cursor table and status bar are used by both modes; the waypoints are markers on the position plot. Two report tabs appear only in pass-through mode: **Summary** (segment durations, total, λ when computed, m, exact cost `SmoothnessCost.Of`) and **Waypoints** (time and state x, v, a, j, s at every waypoint). **Extrema** has one row per quantity over the whole trajectory (segment "Trajectory", times on the global axis); **Coefficients** has T and c0…cd for every segment.

**Notice** (status bar, as in 0012): when the limits determined the durations: `Limits applied: all segment durations were multiplied by λ = 1.168271; total duration 2.3365 s.` Infeasible limits and violated prescribed durations are errors (status bar + message box), and no trajectory is shown.

**Core, two small changes (message text only)**: the violation message of `PassThroughPlanner1D` numbered the segments from 0 ("segment 0"), while the table of the UI starts at 1. It now says `segment {k + 1}`; the one Core test that matched `segment [0-2]` was adjusted to `[1-3]`. No algorithm changed.

## Acceptance criteria — where verified (`App.Tests/PassThroughServiceTests.cs`, 23 cases; `MainWindowSmokeTests`, 1)

| Criterion | Test and numbers |
|---|---|
| (0, 1, 2), degree 9, snap, no limits: total 2 s, interior velocity 2.275 | `DefaultInputs_ReproduceTheTheoryReference_RestToRest`: total 2, durations (1, 1), v at waypoint 2 = 2.275 (1e-6) |
| limits v ≤ 2, a ≤ 3, j ≤ 10, s ≤ 50: λ* = 1.168271, total 2.336542, acceleration at the limit | `ReferenceLimits_GiveTheShortestFeasibleSequence`: λ = 1.168271, total 2.336542 (1e-5), peak acceleration 3.0 (1e-6), the others respected, notice present |
| prescribed durations violating a limit: error naming segment and limit, no trajectory | `PrescribedDurationsViolatingALimit_…` (message contains "segment 1" and "velocity"); `TotalDurationViolatingALimit_IsAnError` |
| switching mode keeps both modes' data; blending unchanged | `MainWindowSmokeTests` (edits both inputs, switches back and forth, data intact); the 68 earlier App tests pass |
| build, tests, count, hand-check list | 0 errors, 428 tests, list below |

Other cases covered: prescribed durations kept; total split by A8 (distances 1, 2 → 2 s, 4 s); durations + total error; partly filled durations error naming the row; last-row duration ignored; fewer than two waypoints; non-numeric position names "Waypoint 2 position"; non-positive duration; m outside 1…p; default m per degree and the list 1…p; non-zero end velocity (1.5, with a = 0); non-zero start v and a for degree 7; degree 3 (2 plotted quantities); unsupported limit ignored; waypoint markers, exact cost row, 5 whole-trajectory extrema rows and 22 coefficient rows; `it-IT` decimal commas in input and in the output text (summary values and notice), and the invariant decimal point still accepted under `it-IT`.

## For the author (educational)

- **How the second mode reuses the chart and report code.** Both modes end in the same object, `PrototypeComputationResult`: sampled series per quantity, extrema rows, coefficient rows, functions to evaluate the trajectory at the cursor time, the total duration and the notices. `MainWindow` knows how to draw that and nothing else about where it came from (`ShowResult`). Pass-through only adds optional extras (markers, waypoint states, summary lines) that the window shows if present. So the plots, the cursor table, the report tables and the status bar exist once.
- **Why the mode logic is in the service and not in `MainWindow`.** The window is hard to test (it needs a screen); the service is a plain class that the tests call directly with text inputs, as the author's hand would type them. Everything that can be wrong, parsing, the choice between prescribed and automatic durations, the error messages, the numbers in the report, therefore has a test. `MainWindow` is left with what only a window can do: show and hide panels, switch mode, draw.
- **What λ means in plain words.** Without durations the program needs a time for every segment. It starts from a plausible *proportion* between them (here: equal, or in proportion to the distances) and then asks "how much must I stretch or shrink the whole timetable so that the fastest-moving point just touches the strictest limit?". That single stretching factor is λ. λ = 1.17 means "every segment lasts 17 % longer than the first guess, and no segment can be shorter without breaking a limit". It is a shortest *common-factor* timetable, not the shortest possible one: the proportions never change (see note 0011).
- **A choice to know about.** With limits and no durations the first-guess proportions are the A8 distances (total = number of segments); with no limits they are simply 1 s each (design/08). For equally spaced waypoints these coincide; for unequal spacing the two cases give different proportions. See open question 1.

## Hand-check list for the author (what the tests cannot see)

1. Start the App on the Italian setup: the **mode selector** switches the left panel, the toolbar buttons and the two extra report tabs (Summary, Waypoints) between the modes; the data typed in each mode is still there after switching back, and the last result is re-shown.
2. Pass-through defaults (0, 1, 2; degree 9): Compute gives total 2 s and the Waypoints tab shows v = 2.275 at waypoint 2. With the four limits of the request: total 2.3365, the status bar shows the λ notice.
3. Waypoint table: edit a position or a duration and click Compute **without leaving the cell** (the grid is committed before computing: check that the last typed value is used); Add Waypoint appends a row with the previous position + 1; Remove Waypoint is disabled at two waypoints; the Duration cell of the last row is greyed out.
4. Decimal comma: typing `0,5` is accepted; tables, status bar and cursor show the comma (0012); **ScottPlot tick labels are still unchecked**.
5. Markers: the waypoints appear as dots on the position plot, on the curve.
6. Error cases show the message and no trajectory: one duration filled out of two; durations and total both filled; a limit that the prescribed durations violate (message names the segment and the limit).
7. Changing the degree resets m to its default and shows/hides the a, j, s fields and limits.

## Discrepancies between theory and code

None for the maths. One for the text of the design: `design/08` says the Notices include "λ applied" and the errors; implemented as written (the λ notice is a status-bar line; the λ value is also in the Summary tab). Not specified there and chosen here: the reference proportions with limits (open question 1), and that the duration of the last waypoint row is ignored (greyed out).

## Open questions

1. **Reference proportions.** With limits and blank durations the code scales A8 proportions (total = number of segments); without limits it uses 1 s each. Should both cases use the same proportions (A8 always, or equal always)? With equal always, λ would be comparable between runs, but a very uneven table would waste time on the short moves; A8 always would contradict "each segment takes 1 s".
2. `referenceDurations` is still not exposed to the user (open question 1 of note 0011): with the table already there, a second column "Weight" would be natural; not done (out of scope).
3. Whether the Coefficients tab or the Summary tab should also list the per-segment peaks against the limits (useful to see which limit was binding); not requested.
4. Mode switching shows the stored result of a mode even if its inputs were edited afterwards (the plots then lag the inputs until Compute). A "stale" marker would be possible; not done.
5. ScottPlot tick-label culture (from 0012) still unchecked.

## Suggested commits

1. `code: Core — pass-through violation message numbers segments from 1` — `src/TrajectoryPlanner.Core/PassThrough/PassThroughPlanner1D.cs`, `src/TrajectoryPlanner.Core.Tests/PassThrough/PassThroughWithLimitsTests.cs`
2. `code: App — pass-through mode: inputs, service entry point, mode selector, summary and waypoint reports` — `src/TrajectoryPlanner.App/MainWindow.xaml`, `src/TrajectoryPlanner.App/MainWindow.xaml.cs`, `src/TrajectoryPlanner.App/PrototypeModels.cs`, `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs`, `src/TrajectoryPlanner.App/PrototypeTrajectoryService.PassThrough.cs`
3. `code: App.Tests — pass-through service cases and a window smoke test` — `src/TrajectoryPlanner.App.Tests/PassThroughServiceTests.cs`, `src/TrajectoryPlanner.App.Tests/MainWindowSmokeTests.cs`
4. `coord: close request 0013` — `coordination/inbox-code/0013-ui-passthrough-mode.md`, `coordination/inbox-docs/0013-ui-passthrough-mode.md`, `coordination/STATUS.md`

(`design/08-ui-first-prototype.md` has an uncommitted change by Cowork, not part of these commits.)
