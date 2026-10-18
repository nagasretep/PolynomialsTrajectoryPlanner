# Project status

Owner: whoever closes a task. Last updated: 2026-10-18 (R13).

## Current phase

Reorganization done; requests 0001-0005 done and reviewed; 0006 (numerics, duration fixes) and 0007 (constant-velocity transitions and tests) done by Code and reviewed by Cowork (committed by the author). 0008 (constant-velocity duration rules 2 and 3) done by Code and reviewed by Cowork. 0009 (R7 consolidation) done by Code and reviewed by Cowork. 0010 (blend duration rule, number parsing) done by Code and reviewed by Cowork. 0011-0014 (pass-through with limits, UI pass-through, defaults) and 0015-0020 (3D Core and UI: blending, constant speed, trajectory level, pass-through) done by Code and reviewed by Cowork. R12 (criteria) and R13 (theory review) done. Next: R14-R15 and cleanup (Code), then R16 manual (Cowork).

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | build + tests verified 2026-10-17 (173/173 pass) |
| M2 WPF shell | implemented | chart-first, ScottPlot; now four modes (blending, pass-through, 3D, 3D pass-through) |
| M3 Degrees 3/5/7 | implemented | degree 7 now tested (0004) |
| M4 Constraints | implemented | extrema, limit verification; shortest feasible duration (closed form + search) for auto duration with limits (0005); failure of the search is reported (0003); constant-velocity transitions still use the legacy search |
| M5 Constant velocity | implemented | 1D, 3D straight and 3D curved (0015, 0016), Trajectory3D-level builder (0017); Rules 1-3 for the transition durations in Core (0007, 0008) |
| M6 Blending | implemented | 1D and 3D (0015); selectable degree, limit verification and input validation in the Core (0009); duration rule with limits to adjust (0010) |
| M7 Exact pass-through | implemented | exact global solve (m = 4 snap default for degree 9), 67/67 tests; reviewed by Cowork 2026-10-17 (reference values and code consistent) |
| M8 3D extension | implemented | Core and UI: 3D blending, straight and curved constant speed, 3D builder and analysis, 3D pass-through with common durations, four modes in the App (0015-0020) |

## In progress

- None. Next request for Code to be written (R14, R15, cleanup).

## Done

- 0020 UI pass-through in 3D (Code) — closed 2026-10-18; note `coordination/inbox-docs/0020-ui-3d-pass-through.md`. 586 tests (450 Core + 136 App). Fourth mode `3D pass-through` (waypoint table X, Y, Z with optional durations, optional total, limits, reference example as default inputs); the design section it referred to is now in `design/08-ui-first-prototype.md`.

- 0019 Core 3D pass-through (Code) — closed 2026-10-18; note `coordination/inbox-docs/0019-core-3d-pass-through.md`. 572 tests (450 Core + 122 App). `PassThroughPlanner3D` (chord rule, common λ, binding quantity and axis); the algorithm of `PassThroughPlanner1D` moved into one shared `PassThroughEngine` (1D tests unchanged); `TrajectoryBuilder3D.AddSegmentsFromWaypointsPassThrough` uses it (default now 1 s per segment instead of a 1 s total).

- 0018 UI 3D mode (Code) — closed 2026-10-18; note `coordination/inbox-docs/0018-ui-3d-mode.md`. 551 tests (429 Core + 122 App). Third mode `Blending | Pass-through | 3D` with vector inputs, blends, constant-speed stretch; `Prototype3DTrajectoryService`; time plots with X/Y/Z (+ norm) and part boundaries, Path tab with XY/XZ/YZ projections; Parts, Extrema 3D, Peaks vs limits tabs. Core: violation messages name axis and quantity; an end-of-piece rounding bug in the 0017 analysis fixed. Hand checks for the author in the note.

- 0017 Core 3D trajectory level (Code) — closed 2026-10-18; note `coordination/inbox-docs/0017-core-3d-trajectory-level.md`. 529 tests (428 Core + 101 App). `ISegment3D` (polynomial segment and curved stretch), `Trajectory3D` with `Parts` and `Sample`, `TrajectoryBuilder3D` (segment, blend, constant speed, part descriptors), `TrajectoryAnalyzer3D` (extrema, limit report/verification, path length).

- 0016 Core 3D curved constant-speed stretch (Code) — closed 2026-10-18; note `coordination/inbox-docs/0016-core-3d-curved-constant-speed.md`. 501 tests (400 Core + 101 App). Series arithmetic for arc-length derivatives, Gauss–Legendre arc length with safeguarded Newton inversion, `ConstantSpeedTrajectory3D` (three specifications, full-jet transitions, admissible speed); the straight case through the general code equals 0015.
- 0015 Core 3D blending and straight constant velocity (Code) — closed 2026-10-18; note `coordination/inbox-docs/0015-core-3d-blending-and-straight-constant-velocity.md`. 473 tests at that point. `BlendBuilder3D`, `ConstantVelocityTrajectory3D` (straight), multi-component `ShortestFeasibleDuration.Find`, shared `TransitionDurationRules`; Core pass-through default reference 1 s per segment (D23); peaks table lists unlimited quantities (D24).

- 0014 pass-through defaults and peaks (Code) — closed 2026-10-18; note `coordination/inbox-docs/0014-passthrough-defaults-and-peaks.md`. 438 tests (338 Core + 100 App). One default rule (A8 proportions, 1 s per segment in total) with and without limits; Summary lists the peak against each active limit with its segment; `ExtremaAnalyzer.PeakAbsoluteWithLocation`.

- 0013 UI pass-through mode in 1D (Code) — closed 2026-10-18; note `coordination/inbox-docs/0013-ui-passthrough-mode.md`. 428 tests (336 Core + 92 App). Mode selector `Blending | Pass-through`, service entry point `ComputePassThrough`, Summary and Waypoints report tabs, waypoint markers, λ notice; Core violation message numbers segments from 1. Hand-check list for the author in the note.

- 0012 report polish (Code) — closed 2026-10-17; note `coordination/inbox-docs/0012-report-polish.md`. 404 tests (336 Core + 68 App). Blend result exposes inherited duration and `AdjustedByLimits`; status bar shows the adjustment notice; WPF default Language set to the current culture so tables, status bar and cursor use one format (ScottPlot tick labels not checked).
- 0011 pass-through with kinematic limits (Code) — closed 2026-10-17; note `coordination/inbox-docs/0011-passthrough-with-limits.md`. 394 tests at that point. `PassThroughPlanner1D`: common scale λ on reference durations (closed form + second solve for rest ends, search for non-zero boundary derivatives), prescribed durations verified with segment index; solver accepts non-zero boundary derivatives; `ExtremaAnalyzer.PeakAbsolute`.

- 0010 blend duration rule and number parsing (Code) — closed 2026-10-17; note `coordination/inbox-docs/0010-blend-duration-and-parsing.md`. 359 tests (296 Core + 63 App). Blend keeps the inherited duration when feasible (D19); user numbers parse without thousands separators, current culture then invariant (D20). Reported: outputs mix en-US (XAML tables) and current culture (status bar).

- 0009 consolidation R7 (Code) — closed 2026-10-17; note `coordination/inbox-docs/0009-consolidation-r7.md`. 317 tests (291 Core + 26 new App.Tests, a Windows-only project that tests the service without a window). Core blending has selectable degree, limit verification and input validation; the App is a thin layer; `DurationSearcher` deleted. Behaviour changes on purpose: blend degree follows the neighbours, blends respect limits (shortest feasible), out-of-range blend positions are errors.

- 0008 constant-velocity duration rules 2 and 3 (Code) — closed 2026-10-17; note `coordination/inbox-docs/0008-constant-velocity-duration-rules.md`. 256/256 tests. Finding: for degree 3 feasibility is not monotone in the transition duration, so the proportional split can be refused for a larger total (explicit error); the Rule 3 sentence "respects the limits whenever any split can" needs a qualification.

- 0007 constant-velocity transitions and tests (Code) — closed 2026-10-17; note `coordination/inbox-docs/0007-constant-velocity-transitions.md`. 228/228 tests (40 new). `FeasibilityTolerance` now 0 so results pass `VerifyLimits`; `FindValidDuration` no longer used by production code.
- 0006 numerics and duration fixes (Code) — closed 2026-10-17; note `coordination/inbox-docs/0006-numerics-and-duration-fixes.md`. Diagonal scaling cuts cond(H) (d = 9, m = 5) from 5e15 to 2e11; `ShortestFeasibleDuration.Find` returns null when limits determine nothing. The strict 1e-12 cost criterion is not reachable (rounding noise ~1e-9), see the note.

- 0005 shortest feasible duration (Code) — closed 2026-10-17; note `coordination/inbox-docs/0005-shortest-feasible-duration.md`. 173/173 tests (36 new). Behaviour change: shorter auto durations when the old first guess (1 s) was already feasible.
- 0004 test coverage (Code) — closed 2026-10-17; note `coordination/inbox-docs/0004-test-coverage.md`. 137/137 at that point (56 new); no defect found; N = 30 pass-through solves fine (cond(H) up to 5e15 for d = 9, m = 5).
- 0003 cleanup batch (Code) — closed 2026-10-17; note `coordination/inbox-docs/0003-cleanup-batch.md`. D5 (failure signalling), D9, D12, D13, D14 done.

- 0002 exact minimum-snap solver (Code) — closed 2026-10-17; return note `coordination/inbox-docs/0002-minimum-snap-solver.md`. Build OK, 67/67 tests (24 old + 43 new). Cost on the reference data fell 44x versus the old heuristic. D1, D2, D6 addressed in code.
- 0001 baseline review (Code) — closed 2026-10-17; reviewed by Cowork the same day. Findings in `roadmap/discrepancies.md`.

## Blocked / to review

- No blocking decision. Decided on 2026-10-17: degree-9 default order = snap; auto duration with limits = shortest feasible duration; constant-velocity transitions with limits = shortest feasible duration each (D8). Also decided (D8): transitions without limits use the mean-velocity rule; a prescribed total duration is split in proportion to the shortest feasible durations (request 0008).
- Uncommitted work in the author's working directory: the code of 0010 (and any of 0008/0009 not yet committed) and the Cowork documents of this round.
- NU1701 (`SkiaSharp.Views.WPF`): transitive via `ScottPlot.WPF`, judged probably harmless; no action.

## Next requests for Code

R12 (success criteria) and R13 (theory library review, PDF build script) done by Cowork. Next request for Code: R14 save/load, R15 speed tests and cleanup of unused public members (to be written). Afterwards R16 user manual (Cowork). See `roadmap/ROADMAP.md`.
