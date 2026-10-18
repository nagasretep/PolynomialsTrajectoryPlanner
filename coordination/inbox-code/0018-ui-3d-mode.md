# 0018 — UI: 3D mode in the App

Status: issued · From: Cowork · To: Code · Date: 2026-10-18

## Goal

The App has a third mode, **3D**, in which the author enters 3D segments by vectors (three boxes X | Y | Z per quantity), optionally with blends and a constant-speed stretch (curved or straight), and sees the trajectory in stacked time plots with three curves per quantity and in three path projections, with a report of parts, peaks against limits and notices.

## Reference documents

- `design/08-ui-first-prototype.md`, *3D Mode (decided 2026-10-18)* (read it fully; it is the specification of inputs and outputs)
- Notes `0013` (how the second mode was added; reuse its pattern), `0012` (`UiCulture`, notices), `0010` (`InputNumber`), `0015`, `0016`, `0017` (Core API: `TrajectoryBuilder3D`, `StretchSpecification`, `Trajectory3D.Parts`, `TrajectoryAnalyzer3D`, `TrajectorySamples3D`)
- `theory/ConstantVelocitySegment.md`, *Curved 3D Constant-Speed Stretch*; `theory/BlendingSegments.md`

## Scope

1. **Mode selector**: `Blending | Pass-through | 3D`. Each mode keeps its data when the user switches; the two 1D modes behave exactly as today.
2. **Segment tabs in 3D** with add / remove as in blending mode:
   - **Degree** 3 / 5 / 7 / 9; **boundary conditions** at start and end that the degree needs (position, velocity, plus acceleration, jerk, snap as the degree supports, as in `BoundaryConditions.md`), each a row of three boxes `X | Y | Z`. Empty boxes of the derivative rows mean zero; position boxes are required.
   - **Constraints**: duration (optional), maximum velocity, acceleration, jerk, snap (those the degree supports), one value each, applied to every axis (`KinematicLimits`). Duration blank: shortest feasible with limits, 1 s without (Core rule, 0017).
   - **Constant-speed stretch** (optional, collapsible like in 1D): speed; specification selector *Points | Path-parameter fractions | Arc-length fractions* with the start and end entered accordingly (points as `X|Y|Z` rows, fractions as numbers in [0, 1]); optional entry duration, exit duration or total duration (not both a total and a transition duration, as in 1D). Built with `TrajectoryBuilder3D.AddConstantSpeed`.
   - **Blend** (optional, collapsible): initial and final blend value toward the previous / next segment, as path-parameter fractions of the segment or as points (selector like above). Built with `AddBlendBetween`; limits of a blend = `KinematicLimits.MostRestrictive` of its neighbours (as in the 1D service); duration inherited, adjusted by limits when needed (notice).
   - **Rule of this step**: a segment that has a constant-speed stretch cannot also be blended with a neighbour, and its neighbours cannot blend toward it (the Core blends plain segments only). Report it as an input error that names the segments.
3. **Service**: a 3D entry point in the App service layer (a sibling class of `PrototypeTrajectoryService` is fine) that takes the input models, builds the trajectory with `TrajectoryBuilder3D`, calls `TrajectoryAnalyzer3D` (`Sample`, `Extrema` once per Compute, `LimitReport`, `PathLength`) and returns series, report tables and `Notices` of the same kind as the other modes. **No duration, limit or geometry logic in the App**; all input errors name the field (segment number, quantity, axis) and come from `InputNumber` or from the Core exceptions (`KinematicLimitsViolatedException`, `DurationSearchFailedException`, `PathNotRegularException`, `ArgumentException`).
4. **Time plots**: the same stacked plots and shared cursor as the other modes, each quantity with three curves (X, Y, Z in fixed distinct colours), a checkbox to add the magnitude (norm) curve, part boundaries marked (thin vertical lines), cursor table with X, Y, Z (and the norm when shown). Quantities shown: position up to the highest order any part supplies; where a part does not supply an order, show what the Core returns (zero), as the 1D plots do.
5. **Path area**: a second tab of the chart area with the projections XY, XZ, YZ (equal axis scale in each), the path coloured by part kind (segment, blend, entry / stretch / exit), segment start and end markers, and a marker following the cursor time. No 3D rotation view.
6. **Report** (left column, same table styles): parts table (label, kind, start, duration; blend: inherited duration and whether adjusted; stretch: length, speed, $v_{adm}$ and binding limit); extrema per axis and order (min, max, max-abs with time); peaks versus limits per active limit and axis (peak, limit, percentage, part), with the binding one first; total duration and path length. Numbers follow the current culture (0012).
7. **Notices**: blend duration adjusted by limits, speed above $v_{adm}$ (error, from the Core message), transition search failure (error), stretch point off the path (error), singular path point (error), limits violated by a prescribed duration (error naming part, axis, quantity).
8. **Tests** (App.Tests, service level): one segment of each degree with vector input; limits and null duration; a two-segment blend with limit adjustment notice; a curved stretch by points, by τ fractions and by arc-length fractions giving the same trajectory; stretch with speed above $v_{adm}$ error; blend plus stretch on the same segment input error; point off the path error; partly filled vector row error naming segment, quantity and axis; `it-IT` parsing of the vectors; the 1D modes unchanged (existing tests pass). The window itself cannot be tested: list the hand checks for the author in the note (include one curved example with numbers I can compare).

Out of scope: 3D pass-through, saving and loading, editing on the plot, per-axis limits, 3D rotation view, several stretches in one segment, any change to the Core (if a small accessor is missing, add it and say so).

## Acceptance criteria

- [ ] 3D mode with two segments and a blend: continuous trajectory, blend adjusted by limits reported, parts table lists Segment / Blend / Segment.
- [ ] Parabola example (segment from (0,0,0) to (1,1,0) with a curved path of the user's choice, or the theory parabola data if it can be entered): stretch at speed 1 has duration $L/v = 1.478943$ s, and a speed above $v_{adm}$ is refused with the Core message.
- [ ] The path area shows the three projections with the stretch visibly distinct; the cursor marker moves with the cursor.
- [ ] Peaks table agrees with the Core `LimitReport` (same numbers).
- [ ] Blending and pass-through modes unchanged; `dotnet build` free of new warnings; all tests pass; the note states the test count and the hand-check list.

## Notes for the author (educational)

Explain in the return note how the third mode reuses the chart and report code, why the 3D service holds no geometry or limit logic, what the part descriptors give the UI, and why path projections are used instead of a 3D view.

Return note: `coordination/inbox-docs/0018-ui-3d-mode.md`, ending with a "Suggested commits" section. Do not touch `roadmap/`, `theory/` or `design/`; targeted edits only in `coordination/STATUS.md`.
