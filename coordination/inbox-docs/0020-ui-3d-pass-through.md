# 0020 — UI: pass-through in 3D, fourth mode (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0020-ui-3d-pass-through.md`

## Summary

The App has a fourth mode, **3D pass-through**. Tests: Core.Tests **450** (unchanged), App.Tests 122 → **136** (+14 service tests; the window smoke test extended to the fourth mode), **586 total**, all passing. `dotnet build`: 0 errors, no warnings other than the known NU1701. No Core change. The three other modes are unchanged (their tests pass untouched).

**Reference document missing**: the request points to `design/08-ui-first-prototype.md`, section *3D Pass-Through Mode (decided 2026-10-18)*. That section is **not in the file** (it has *Pass-Through Mode* and *3D Mode* only). I followed the request's scope and, for what it does not say, the 1D *Pass-Through Mode* section (inputs, blank-duration rule, notices). Please add the section or tell me where it is.

## What was done

**Inputs** (`App/PassThrough3DModels.cs`): `PassThrough3DInputModel` (degree; smoothness order m with the 1D defaults, reset when the degree changes; start/end "at rest" or vectors v, a, j, s as the degree uses, empty box = 0; a waypoint table of `Waypoint3DInputModel` rows X, Y, Z, optional duration, the last row's duration greyed out; optional total; four limits, one value each for every axis). The **default inputs are the reference example of the theory** ((0,0,0), (1,0,0), (1,3,0), (1,3,4); the limit values 2 / 3 / 10 / 50 pre-filled but not ticked), so the example is one click away.

**Service** (`App/Prototype3DTrajectoryService.PassThrough.cs`, the 3D service is now `partial`): `ComputePassThrough(PassThrough3DInputModel)` parses and validates the fields, calls `TrajectoryBuilder3D.AddSegmentsFromWaypointsPassThrough` (Core 0019) and reads `LastPassThroughResult` for durations, reference durations, λ and the binding pair; nothing about durations, scaling or limits is computed in the App. Input errors: "Waypoint 2: the Y component is empty.", "Waypoint 2, Z component, must be a valid number …", "Segment durations are partly filled (1 of 3; waypoint 2 is blank) …", "… both given …", "At least two waypoints are required.", "Smoothness order: …". Core failures are prefixed "Cannot build the trajectory: …" (as in the 1D pass-through), e.g. "… Pass-through segment 1 (duration 0.5 s) violates the velocity limit on axis x: peak … exceeds 2."

**Presentation**: the report-building part of the 3D service (series, path pieces, endpoints, parts, extrema, coefficients, summary) was made into one shared method that both 3D modes call; the 3D mode passes its own limit rows and notices, the pass-through its own. The result type is the same `Prototype3DComputationResult`, with two new optional members: `Waypoints` (number, time, point) and `WaypointStates` (vector state rows).
- Window: radio button *3D pass-through*; Add/Remove Waypoint serve both pass-through modes; a new left panel; the 3D presentation (three curves + norm, part boundaries, Path tab, 3D cursor table, Parts / Extrema 3D / Peaks vs limits tabs) is shown in both 3D modes. Waypoints are open circles on the position plot (one per axis, in the axis colour) and dark-green open circles on the three projections. New report tab **Waypoint states** (time, quantity, X, Y, Z at every waypoint for the orders the waypoint states share, 0 … p−1).
- Summary (first tab shown in this mode): segment durations, reference durations (when λ was applied), **common factor λ**, **binding limit** ("velocity on axis x"), smoothness order m, **smoothness cost J**, then total duration, path length, parts, largest speed.
- **Cost**: the Core `SmoothnessCost.Of` per axis, summed in the service (the 3D cost is the sum over the axes, theory *Common Durations …*, "Decoupling"); no Core extension was needed.
- Peaks vs limits: the Core `LimitReport` over the whole trajectory (one set of limits in this mode), binding first.
- Notice (status bar) when λ was applied: "Limits applied: all segment durations were multiplied by λ = 4.990502 (binding: velocity on axis x); total duration 14.9715 s." Violations by prescribed durations or totals and impossible limits are errors (status bar + message box), no trajectory shown.

## Acceptance criteria — where verified (`App.Tests/PassThrough3DServiceTests.cs`, 14; `MainWindowSmokeTests`)

| Criterion | Test |
|---|---|
| Reference example: λ = 4.990502, total 14.971506, binding velocity on x, in the report and in the status bar | `ReferenceExample_WithLimits_…` (summary rows, notice, peaks table: 12 rows, velocity X at 100 % first); smoke test: the window's status bar starts "Computed 5 plotted quantities across 14.9715" and contains "binding: velocity on axis x" |
| Without limits: 3 s, 0.375 / 1.125 / 1.5 | `ReferenceExample_WithoutLimits_…` |
| Waypoints marked; the path passes through them | `TheWaypointsAreMarked_…` (times 0, 1.871438, 7.485753, 14.971506; the trajectory is at each waypoint to 1e-9); the drawing runs in the smoke test; the look is a hand check |
| Four modes keep their data; others unchanged | smoke test (round trip over the four modes, results and inputs kept); the earlier App tests pass unchanged |
| Other listed tests | prescribed durations violating a limit (segment, axis, quantity), total duration (chord split 2 / 6 / 8), partly filled durations, durations + total, fewer than two waypoints, empty / non-numeric coordinate (row and axis), smoothness order above p, non-zero end velocity vector (honoured, limits respected, λ applied), along one axis = the 1D pass-through service (total, every coefficient of X, λ text), `it-IT` vectors and output, waypoint states, cost = sum over the axes |

## Hand checks for the author (window)

1. **Mode selector** with four entries; switching keeps every mode's inputs and last result (Blending, Pass-through, 3D, 3D pass-through).
2. **Reference example**: choose *3D pass-through* (the waypoints are already the theory's), tick the four limits (2 / 3 / 10 / 50), Compute. Expected: status bar "… across 14,9715 seconds … λ = 4,990502 (binding: velocity on axis x) …"; Summary: segment durations 1,871438 / 5,614315 / 7,485753, reference durations 0,375 / 1,125 / 1,5, λ 4,990502, binding "velocity on axis x"; Peaks vs limits: first row Velocity X 100 %; velocity plot: the red X curve touches 2 in the first segment.
3. Untick the limits: total 3 s, durations 0,375 / 1,125 / 1,5, no λ row.
4. **Path tab**: XY shows the L-shaped path from (0,0) to (1,0) to (1,3), XZ and YZ the climb to z = 4; the dark-green circles sit on the path at the four waypoints; the black cursor dot follows the time cursor.
5. **Position plot**: open circles on the three curves at the waypoint times.
6. Waypoint table: edit a coordinate and Compute without leaving the cell (the value is used); Add Waypoint appends (previous X + 1, same Y, Z); Remove is disabled at two waypoints; the last row's duration is grey.
7. Errors: one duration filled; durations + total; an empty Y; prescribed durations 0,5 / 6 / 8 with the limits (message names segment 1, axis x, velocity).
8. Decimal comma everywhere (ScottPlot tick labels still unchecked).

## For the author (educational)

- **How the fourth mode reuses the 3D presentation.** Both 3D modes produce the same result object: curves per quantity with X, Y, Z and the norm, the path cut into pieces, part boundaries, report rows and a function that gives the state at any time. The window draws that object the same way whichever mode made it; the pass-through adds only waypoint markers and a waypoint-state table. Even the service shares its report code between the two 3D modes; only how the trajectory is *built* differs (segments, blends and stretches in one; waypoints and λ in the other).
- **What the binding quantity and axis tell the user.** All segments are stretched by one factor λ until every axis respects every limit. The pair "velocity on axis x" names the single constraint that stops λ from being smaller: x is the axis that moves fastest relative to its limit, and only it is at 100 %; everything else has margin (the Peaks vs limits table shows how much). To make the whole motion faster one must relieve that pair — a higher velocity limit on x, or more time for the segment where x moves (here the first, short segment).
- **Why one scale factor applies to all axes.** The axes must reach every waypoint *at the same instant*, so they share the durations; stretching the durations stretches time for all three at once. A stretched trajectory p(t/λ) runs through exactly the same points in the same order: the path in space is unchanged, only the clock runs slower, and every derivative of order k shrinks by λ^k on every axis.

## Discrepancies between theory and code

None with the theory. The missing design section is reported above.

## Open questions

1. The design section *3D Pass-Through Mode* should be added to `design/08` (or the reference corrected).
2. With equal limits on all axes the binding axis is simply the one with the largest motion relative to its time share; per-axis limits (out of scope) would make the pair more informative.

## Suggested commits

1. `code: App — 3D pass-through mode: inputs, service entry point, shared 3D report, window` — `src/TrajectoryPlanner.App/PassThrough3DModels.cs`, `src/TrajectoryPlanner.App/Prototype3DTrajectoryService.cs`, `src/TrajectoryPlanner.App/Prototype3DTrajectoryService.PassThrough.cs`, `src/TrajectoryPlanner.App/Prototype3DModels.cs`, `src/TrajectoryPlanner.App/MainWindow.xaml`, `src/TrajectoryPlanner.App/MainWindow.xaml.cs`
2. `code: App.Tests — 3D pass-through service tests; window smoke test over four modes` — `src/TrajectoryPlanner.App.Tests/PassThrough3DServiceTests.cs`, `src/TrajectoryPlanner.App.Tests/MainWindowSmokeTests.cs`
3. `coord: close request 0020` — `coordination/inbox-code/0020-ui-3d-pass-through.md`, `coordination/inbox-docs/0020-ui-3d-pass-through.md`, `coordination/STATUS.md`

(0013–0019 may still be uncommitted. `roadmap/`, `theory/`, `design/` untouched.)
