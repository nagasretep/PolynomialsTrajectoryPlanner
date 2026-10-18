# Version 1.0 Verification

Companion of `05-success-criteria.md`. For every criterion: how it is verified, the evidence Cowork found on 2026-10-18, what the author still has to do, and a column for the result (write the date and your initials when done). Version 1.0 is reached when every row is closed.

## 1. Commands to run on the laptop

Run from the repository root in a PowerShell window:

```
dotnet build -c Release
dotnet test -c Release
dotnet test -c Release --filter Category=Speed --logger "console;verbosity=detailed"
powershell -ExecutionPolicy Bypass -File tools\build-theory.ps1
powershell -ExecutionPolicy Bypass -File tools\build-manual.ps1
```

Expected: the build has 0 errors and only the known `NU1701` warning; all tests pass (600 at the time of request 0021: 445 Core + 155 App); the three speed groups print medians far below their limits; two PDFs appear in `build/`. If a PDF build asks for a missing font, pass `-MainFont "<a font on the machine>"`; if `lmodern.sty` is missing see `roadmap/06-theory-library-roadmap.md`.

## 2. Table of criteria

| Id | How verified | Evidence found (2026-10-18) | Author to do | Result |
|---|---|---|---|---|
| N1 | automated tests | endpoint tests in `PolynomialEvaluationTests`, `Degree7Tests`, `SegmentAndConstraintsTests`, `TrajectoryIntegrationTests`; pass-through and 3D tests check the waypoints to 1e-9 | `dotnet test` green | |
| N2 | automated tests | junction continuity (orders 0-4, 1e-9) in `BlendBuilderTests`, `BlendBuilder3DTests`, `ConstantVelocityTrajectoryTests`, `ConstantVelocityTrajectory3DTests`, `ConstantSpeedTrajectory3DTests`, `TrajectoryAnalyzer3DTests`, pass-through tests | `dotnet test` green | |
| N3 | automated tests | limit verification and explicit failures in `ShortestFeasibleDurationTests`, `PassThroughWithLimitsTests`, `PassThroughPlanner3DTests`, `BlendBuilderLimitsAndDegreeTests`, App service tests (messages name segment, axis, quantity) | `dotnet test` green | |
| N4 | automated tests | **checked by Cowork**: every reference table of `theory/` has a test, see section 3 | none beyond `dotnet test` | |
| N5 | automated tests | `MinimumSnapSolverTests` (cost 10977.31 and 4032 against the theory, gradient zero at the optimum); `PassThroughPlanner3DTests` and `PassThrough3DServiceTests` (one axis equals the 1D result) | `dotnet test` green | |
| S1 | timing tests | median 0.4 / 1.3 / 2.6 / 4.4 ms for degrees 3 / 5 / 7 / 9 against 100 ms (Debug and Release alike, Ryzen 9 8940HX) | run the Speed filter on your laptop; write the numbers in the Result column | |
| S2 | timing tests | median 7 ms (1D) and 25 ms (3D) for 30 waypoints against 1 s | same | |
| S3 | timing tests | median 166 ms against 1 s | same | |
| Q1 | `dotnet test` | 600 passing in the report of 0021 | run it, check the count in `coordination/STATUS.md` | |
| Q2 | `dotnet build` | 0 errors, only `NU1701` in every report | run it | |
| Q3 | Cowork review | matching of algorithms to tests, section 4; done by test file and by reference value, not by reading every assertion | accept, or ask Cowork for a line-by-line review | |
| Q4 | Code request | done in 0021: list of removed and internalised members in `coordination/inbox-docs/0021-save-load-speed-cleanup.md` | none | |
| A1 | author hand check | script in section 5 below | run the script | |
| A2 | tests + hand check | error cases of the four modes are in the App service tests; hand-check items 6 and 7 of section 5 | run items 6 and 7 | |
| A3 | tests + hand check | `ProjectFileTests` (18) and the window test; the four hand checks of 0021 were done by the author on 2026-10-18 and gave the expected result | none | closed 2026-10-18 |
| A4 | author follows the manual | `manual/UserManual.md`: the four worked examples; the numbers of the 1D blending example (peaks 2.4609 / 9.3720 / 78.75 / 622.53) were computed by Cowork outside the program and have not yet been seen in the window | follow the manual's examples in the program and report any difference | |
| D1 | Cowork review | all eight documents are *Reviewed 2026-10-18* in `roadmap/06-theory-library-roadmap.md`; the notation collisions and the open questions were fixed; no discrepancy of kind theory is open (D25, D26, D27 closed; D18 is accepted, low priority) | none | |
| D2 | build script | the book builds into 101 pages with table of contents, no literal math and no missing characters (checked in the Cowork environment with a different font) | run `tools\build-theory.ps1` and open the PDF; check a few cross references and formulas | |
| D3 | Cowork review | see section 6: the decisions of 2026-10-18 were missing from the list and have been added | none | |
| D4 | author sign-off | questions and reference answers: `06-understanding-questions.md`, `06-understanding-answers.md` | complete and sign the checklist in `05-success-criteria.md` | |

## 3. N4: reference tables of the theory and their tests

| Reference in `theory/` | Values | Test file |
|---|---|---|
| Pass-through, waypoints (0, 1, 2), rest ends | interior velocity 1.5 / 1.875 / 2.1875 / 2.275 for degrees 3 / 5 / 7 / 9 | `MinimumSnapSolverTests` |
| Pass-through, waypoints (0, 1, 3, 2), degree 9, snap | node derivatives (3.038429, ..., -1.142124, ...), cost 10977.31 | `MinimumSnapSolverTests` |
| Closed form with limits, (0, 1, 2) | peaks 2.275 / 4.094571 / 15.4 / 74.005522, $\lambda^\star = 1.168271$ | `PassThroughWithLimitsTests` |
| Closed form with limits, (0, 1, 3, 2) | peaks 4.47151 / 9.626191 / 33.013296 / 255.962162, $\lambda^\star = 1.490503$, snap 39.727378 | `PassThroughWithLimitsTests` |
| Same waypoints, durations (1, 1, 1) | peaks 3.490773 / 8.369728 / 24.329164 / 129.585159 | `PassThroughWithLimitsTests` |
| Common durations in 3D | axis peaks (9.981004, 38.207532, 207.293297, 2904.831335, ...), $\lambda^\star = 4.990502$, durations 1.871438 / 5.614315 / 7.485753, total 14.971506 | `PassThroughPlanner3DTests` |
| Parabola $(t, t^2, 0)$ | length 1.478943, derivatives with respect to arc length at both ends (0.447214, 0.894427, 0.157419, 0.24064, ...), peaks, $v_{adm} = 1.709976$ | `ConstantSpeedTrajectory3DTests`, `TrajectoryAnalyzer3DTests` |

The text of criterion N4 used to mention a "13 s example"; no such example exists in the theory, so the criterion was corrected to the real list above.

## 4. Q3: algorithms of the theory and their tests

| Algorithm (theory document) | Test files |
|---|---|
| Polynomial evaluation, derivatives, coefficient determination (degrees 3, 5, 7, 9) | `PolynomialEvaluationTests`, `Degree7Tests`, `SegmentAndConstraintsTests`, `SymbolsAndPointsTests` |
| Extrema and peak location | `PeakLocationTests`, `ExtremaAndDurationTests`, `SegmentAndConstraintsTests` |
| Shortest feasible duration (closed form and search) | `ShortestFeasibleDurationTests` |
| Constant velocity, 1D, duration rules | `ConstantVelocityTrajectoryTests`, `ConstantVelocityTotalDurationTests` |
| Constant velocity, 3D straight and curved | `ConstantVelocityTrajectory3DTests`, `ConstantSpeedTrajectory3DTests` |
| Blending, 1D and 3D, limits and degree | `BlendBuilderTests`, `BlendBuilderLimitsAndDegreeTests`, `BlendDurationInformationTests`, `BlendBuilder3DTests` |
| Exact pass-through, 1D, with limits, 3D, large N | `MinimumSnapSolverTests`, `PassThroughWithLimitsTests`, `PassThroughPlanner3DTests`, `LargePassThroughTests` |
| Trajectory level in 3D (builder, analysis) | `TrajectoryBuilder3DTests`, `TrajectoryAnalyzer3DTests` |
| Application services and project file | `PrototypeTrajectoryServiceTests`, `PassThroughServiceTests`, `Prototype3DServiceTests`, `PassThrough3DServiceTests`, `ProjectFileTests`, `InputNumberTests`, `MainWindowSmokeTests` |

## 5. A1 and A2: hand-check script (about 15 minutes)

Tick each step when the window shows what is written. The numbers come from the notes of requests 0013, 0018, 0020 and the manual. Use decimal point or comma as you prefer.

1. **Modes.** Four entries in the selector; switch among them: every mode keeps its inputs and its last result.
2. **Blending, one segment.** Degree 9, from 0 to 1, duration 1, nothing else: the Extrema table gives velocity 2.4609, acceleration 9.3720, jerk 78.75, snap 622.53 (maximum absolute values).
3. **Blending, two segments.** Add a segment from 1 to 3; final blend value 0.7 on the first, initial blend value 1.3 on the second: the plots have no stop at 1 and the report shows a blend part. Set a blend position outside a segment: an error that names the blend and the range.
4. **Pass-through, defaults.** Points 0, 1, 2, degree 9: total 2 s, velocity at the second waypoint 2.275. Tick the four limits (2 / 3 / 10 / 50): total 2.3365, status bar notice about $\lambda$ = 1.168271, acceleration at 100 %.
5. **Waypoint tables.** Edit a value and press Compute without leaving the cell: the new value is used. Add Waypoint appends a row; Remove Waypoint is disabled at two rows; the last row's duration is grey.
6. **Errors in pass-through.** One duration filled out of two; durations and total together; a prescribed duration that violates a limit: each shows a message that names the field, segment and limit, and no trajectory.
7. **Errors in 3D.** An empty Y of a start position; a stretch point slightly off the path; blend values on a segment that has a stretch; a speed above the admissible one: each message names the field, segment, axis or limit.
8. **3D curved stretch.** The parabola of the manual (start (0, 0, 0) velocity (1, 0, 0) acceleration (0, 2, 0); end (1, 1, 0) velocity (1, 2, 0) acceleration (0, 2, 0); degree 9, duration 1; stretch at speed 1 over fractions 0 and 1): stretch 1.478943 s, total 2.957886 s. With acceleration 8 and jerk 20 and speed 1.8: refused, admissible speed 1.70998, jerk binds.
9. **3D blend and cursor.** Two segments with blend values: orange blend on the projections; moving the mouse over a plot moves the cursor on all plots, the table and the dot on the Path tab; the projections have equal scales.
10. **3D pass-through.** The reference waypoints with the four limits (2 / 3 / 10 / 50): total 14.9715 s, $\lambda$ = 4.990502, binding "velocity on axis x", first row of Peaks vs limits Velocity X at 100 %, waypoints marked on the projections. Without limits: total 3 s, durations 0.375 / 1.125 / 1.5.
11. **Decimal comma** in input boxes and in output tables, status bar and cursor. Check the axis labels of the ScottPlot charts (still unverified).
12. **Save and Open** (already done on 2026-10-18): nothing to repeat unless you change the program.

## 6. D3: decisions are justified

`roadmap/decisions/` contains one record (`ADR-0001-repository-restructure.md`). The decisions of the author about the algorithms are recorded with their reason in `roadmap/discrepancies.md`, section *Decisions taken by the author*, and in the documents they concern (`design/07-implementation-plan.md`, `design/08-ui-first-prototype.md` and the theory). That list ended on 2026-10-17: the decisions of 2026-10-18 (3D input by vector; common durations with the chord rule; one common $\lambda$; the success criteria) were only in the documents. They have been added to the list on 2026-10-18. Every discrepancy D1-D27 is closed with its resolution in the table, except D18 (degree-3 feasibility is not monotone in the duration), which is accepted with low priority: results are always feasible and failures are explicit.
