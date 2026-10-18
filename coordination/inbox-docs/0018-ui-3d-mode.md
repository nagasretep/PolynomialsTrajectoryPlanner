# 0018 — UI: 3D mode in the App (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0018-ui-3d-mode.md`

## Summary

The App has a third mode, **3D**. Tests: Core.Tests 428 → **429** (+1 regression test), App.Tests 101 → **122** (+21 service tests; the window smoke test extended), **551 total**, all passing. `dotnet build`: 0 errors, no warnings other than the known NU1701. The two 1D modes are unchanged (their tests pass untouched). The window was not clicked through by hand; the smoke test loads the real window, computes a 3D trajectory, draws the time plots and the projections, moves the cursor and switches modes back and forth. The hand-check list is below.

## What was done

**Inputs** (`App/Prototype3DModels.cs`): `VectorInput` (X | Y | Z text boxes) and `Segment3DInputModel`: degree; start/end position (required), velocity, acceleration, jerk, snap (as the degree supports; empty box = 0); duration (blank = automatic); four limits, one value each, applied to every axis; **constant-speed stretch** (speed, specification selector *Points | ParameterFractions | ArcLengthFractions*, start/end points or fractions, optional entry, exit or total duration); **blend** (selector *ParameterFractions | Points*, initial value toward the previous segment, final value toward the next).

**Service** (`App/Prototype3DTrajectoryService.cs`, sibling of `PrototypeTrajectoryService`): parses, then only calls the Core:
1. `TrajectoryBuilder3D.AddSegment` for every segment (duration and limits passed as given).
2. Blends, first to last pair, with `AddBlendBetween(…, Point3D, Point3D, limits: MostRestrictive(neighbours))`. A value given as a fraction is turned into the point of the **original** segment at that fraction (one Core evaluation), so that a segment blended on both sides keeps the meaning "a place on the segment as entered", even though the second blend works on what the first left of it. Indices are tracked as pieces are inserted.
3. Constant-speed stretches, last to first, with `AddConstantSpeed` (always `ConstantSpeedTrajectory3D`).
4. `TrajectoryAnalyzer3D`: `Sample` (≥ 600 samples, junctions included), `Extrema` once, `LimitReport`, `PathLength`.

Input errors name the field: "Segment 1 start position: the Y component is empty.", "Segment 1 end velocity, Z component, must be a valid number …", "… must be a fraction between 0 and 1"; the rule of this step: "Segment 2 has a constant-speed stretch, so it cannot be blended with a neighbour: remove the blend values of segment(s) 1, 2 toward it, or the stretch."; a blend with one value only: "requires both values" (as in 1D). Core failures are prefixed with the segment or blend ("Segment 1: The constant speed 1.8 exceeds the admissible speed 1.70998 of this stretch: the jerk limit binds …").

**Peaks versus limits with different limits per segment**: the Core's `LimitReport` takes one set of limits. The service calls it once per run of pieces that share their limits (a segment's pieces: that segment's limits; a blend: the most restrictive of its neighbours), on a `Trajectory3D` of those pieces with their descriptors, and shifts the times back to the global axis. Rows are sorted with the highest percentage first. With one segment (or equal limits everywhere) this is exactly the Core report on the whole trajectory (tested number by number).

**Window** (`MainWindow`): mode selector `Blending | Pass-through | 3D` (the mode is now an enum; Add/Remove Segment serve blending and 3D); a third left panel with the 3D segment tabs (a `VectorBoxes` template draws every X | Y | Z row); the chart area is a tab control: **Time** (the same five stacked plots and cursor table; in 3D three curves per quantity in red/green/blue, a "Show the magnitude (norm)" checkbox adding a dashed black curve, dotted grey lines at the part boundaries, a cursor table with X, Y, Z, |.|) and **Path** (3D only: XY, XZ, YZ with `SquareUnits`, path coloured by part kind: blue segment, orange blend, grey entry/exit, thick red stretch; black squares at segment ends; a black dot that follows the cursor time). Report tabs in 3D: **Parts**, **Extrema 3D**, **Peaks vs limits**, **Coefficients** (per part and axis), **Summary** (total duration, path length, number of parts, largest speed). Each mode keeps its inputs and its last result.

**Core changes (small, said as requested)**:
1. `TrajectoryAnalyzer3D.DescribeWorstViolation(piece, limits)` and its use in `SegmentFactory3D` and the 3D constant-velocity transitions: a prescribed duration that violates a limit now says which ("Segment 1: The segment violates the active kinematic limits at the specified duration (0.5 s): the velocity limit on axis x (peak 11.8125 above 2).").
2. **Bug fix in 0017's analysis**: grids computed as `Duration · k / n` can exceed `Duration` by one ulp, which a polynomial segment rejects (`ArgumentOutOfRangeException`); found by the parabola test of this request (stretch duration 0.7394714287722989). Clamped in `TrajectoryAnalyzer3D` (max speed and the sampled fallbacks), `ArcLengthPath.Speed`, `ArcLengthDerivatives.Compute` and the stretch's extrema. Regression test `Analysis_DoesNotStepBeyondTheEndOfAPiece_ForAnyDuration` (200 random durations plus that stretch).

## Acceptance criteria — where verified

| Criterion | Test (`App.Tests/Prototype3DServiceTests.cs` unless said) |
|---|---|
| Two segments and a blend: continuous, adjustment reported, parts Segment / Blend / Segment | `TwoSegmentsWithABlend_…` (kinds, label "Blend 1-2", one notice "Blend 1-2: duration adjusted from …", all peaks ≤ 100 %), `ABlendIsContinuous_AndByPointsEqualsByFractions` |
| Parabola: stretch at speed 1 lasts L/v = 1.478943 s; above v_adm refused with the Core message | `TheParabola_StretchAtSpeedOne_…` (stretch duration and length 1.478943; total 2 × 1.478943, see hand checks), `ASpeedAboveTheAdmissibleSpeed_…` ("admissible speed 1.70998", "jerk") |
| Projections with the stretch distinct; cursor marker moves | in the window (hand check); the smoke test runs the drawing and the cursor update; `ACurvedStretch_…` checks the stretch has its own path piece |
| Peaks table = Core `LimitReport` | `ThePeaksTable_IsTheCoreLimitReport` (every row: peak, percentage, part; binding first at 100 %) |
| 1D modes unchanged, build, tests | 1D tests pass unchanged; `MainWindowSmokeTests` (modes keep their data) |
| Other listed tests | one segment of each degree (`OneSegmentOfEachDegree…` ×4), blank duration with/without limits, prescribed duration violating a limit (names axis and quantity), curved stretch by points / τ / arc length giving the same trajectory (1e-8), point off the path, rest point, blend + stretch rule, partly filled vector row, non-numeric box, `it-IT` vectors, report extrema/path length/endpoints, no segments |

## Hand checks for the author (window)

1. **Mode selector**: Blending and Pass-through look and work as before; switching to 3D and back keeps the data of every mode, and each shows its last result again.
2. **Parabola example** (curved): 3D mode, one segment, degree 9, duration 1; start position (0, 0, 0), start velocity (1, 0, 0), start acceleration (0, 2, 0); end position (1, 1, 0), end velocity (1, 2, 0), end acceleration (0, 2, 0) (that is the path (t, t², 0)). Constant-speed stretch: enable, speed 1, *ParameterFractions*, 0 and 1. Expected: Parts = Entry 1, Stretch 1, Exit 1; stretch duration and length **1.478943**; total **2.957886 s** (entry and exit cover no distance, so each takes T_cv / 2 = 0.739471 s); on the Path tab the stretch is the thick red parabola in XY, a straight line in XZ and YZ (z = 0); the velocity plot shows |v| = 1 on the stretch with the norm checkbox on. Then tick maximum acceleration 8 and jerk 20 and set the speed to 1.8: message "Segment 1: The constant speed 1.8 exceeds the admissible speed 1.70998 of this stretch: the jerk limit binds …".
3. **Blend**: two segments (Add Segment copies the end of the previous one as the start); on segment 1 *Final blend value* 0.7, on segment 2 *Initial blend value* 0.3: Parts shows Segment 1, Blend 1-2, Segment 2; orange piece on the projections; set an acceleration limit on both to see "duration adjusted" in the status bar (if the inherited blend respects it, no notice is expected).
4. **Cursor**: moving over a time plot moves the vertical line on all plots, updates the X/Y/Z/|.| table and the black dot on the three projections (switch to the Path tab after moving the mouse).
5. **Equal scales** on the projections (a circle looks round), legends readable, part boundaries visible as thin dotted lines.
6. **Errors**: empty Y of a start position; a stretch point slightly off the path (message with the distance); a stretch from fraction 0 on a rest-to-rest segment (rest point); blend values on a segment that has a stretch (names the segments).
7. **Decimal comma** in the vector boxes and in the tables (0012); ScottPlot tick labels still unchecked.

## For the author (educational)

- **How the third mode reuses the chart and report code.** The window has one set of five stacked plots, one cursor mechanism, one status bar and one report tab control. Each mode hands the window a result object and the window draws it: the 1D modes a series per quantity, the 3D mode three series per quantity (plus the norm). The plotting loop, the cursor line, the clamp of the cursor time, the "keep the last result per mode" logic and the table styles are the same; the 3D mode adds what has no 1D counterpart (the Path tab, three new report tabs) and hides what does not apply to it.
- **Why the 3D service holds no geometry or limit logic.** Everything that decides *what the trajectory is* (durations, shortest feasible durations, blend windows and their limits, the arc-length stretch, the admissible speed, which part violates what) is in the Core, where it is tested once and shared by every client. The service only translates text into the builder's arguments and the builder's answers into rows and curves. If it computed, say, its own admissible speed or its own blend duration, the App and the Core could disagree, and the App's version would have no Core tests behind it.
- **What the part descriptors give the UI.** The builder says for each piece what it is (segment, blend, entry, stretch, exit), which original segment it comes from, when it starts and how long it lasts, and the facts the user wants to see (inherited blend duration and whether the limits changed it; stretch length, speed, admissible speed and the binding limit). The UI shows these directly in the Parts table, uses the start times for the boundary lines, the kinds for the colours of the path, and the labels in the error messages, without reconstructing anything.
- **Why projections instead of a 3D view.** A 3D view on a flat screen hides depth: a point that seems to lie on a curve may be far behind it, and judging distances or angles needs rotating the view. Three projections on equal scales show every coordinate exactly, can be read like technical drawings, are drawn by the same chart library as the time plots, and need no rotation control; the stretch and the blends are as visible there as in a 3D view.

## Discrepancies between theory and code

None. Two interpretations, both reversible: (1) a blend value given as a fraction refers to the segment as entered (see above); (2) the request's "(not both a total and a transition duration, as in 1D)": the Core rule of 1D is "a total together with both transition durations is a conflict" (a total with one of them is allowed and splits the rest); the App passes the values through and the Core decides, so the 1D rule applies.

## Open questions

1. Blending a segment toward a neighbour with a stretch, and several stretches per segment, are refused / out of scope; the Core would need blends between non-plain pieces.
2. The design mentions "waypoint markers"; the 3D mode has no waypoints, so the markers are the segment start and end points.
3. `Extrema` with a stretch takes about 0.1 s; it is called once per Compute. Sampling (≥ 600 points, one inversion per stretch sample) is fast enough for Compute; nothing is recomputed on mouse moves.

## Suggested commits

1. `code: Core — 3D violation messages name the axis and quantity; clamp analysis grids at the end of a piece` — `src/TrajectoryPlanner.Core/Trajectories/TrajectoryAnalyzer3D.cs`, `src/TrajectoryPlanner.Core/Services/SegmentFactory3D.cs`, `src/TrajectoryPlanner.Core/ConstantVelocity/ConstantVelocityTrajectory3D.cs`, `src/TrajectoryPlanner.Core/ConstantVelocity/ConstantSpeedTrajectory3D.cs`, `src/TrajectoryPlanner.Core/ArcLength/ArcLengthPath.cs`, `src/TrajectoryPlanner.Core/ArcLength/ArcLengthDerivatives.cs`, `src/TrajectoryPlanner.Core.Tests/Trajectories/TrajectoryAnalyzer3DTests.cs`
2. `code: App — 3D mode: vector inputs, 3D service, time plots with three curves, path projections, 3D report tabs` — `src/TrajectoryPlanner.App/Prototype3DModels.cs`, `src/TrajectoryPlanner.App/Prototype3DTrajectoryService.cs`, `src/TrajectoryPlanner.App/MainWindow.xaml`, `src/TrajectoryPlanner.App/MainWindow.xaml.cs`
3. `code: App.Tests — 3D service tests; window smoke test computes in 3D` — `src/TrajectoryPlanner.App.Tests/Prototype3DServiceTests.cs`, `src/TrajectoryPlanner.App.Tests/MainWindowSmokeTests.cs`
4. `coord: close request 0018` — `coordination/inbox-code/0018-ui-3d-mode.md`, `coordination/inbox-docs/0018-ui-3d-mode.md`, `coordination/STATUS.md`

(0013–0017 may still be uncommitted. `roadmap/`, `theory/`, `design/` untouched.)
