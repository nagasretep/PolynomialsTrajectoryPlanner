# Project status

Owner: whoever closes a task. Last updated: 2026-10-18.

## Current phase

Reorganization done; requests 0001-0005 done and reviewed; 0006 (numerics, duration fixes) and 0007 (constant-velocity transitions and tests) done by Code and reviewed by Cowork (committed by the author). 0008 (constant-velocity duration rules 2 and 3) done by Code and reviewed by Cowork. 0009 (R7 consolidation) done by Code and reviewed by Cowork. 0010 (blend duration rule, number parsing) done by Code and reviewed by Cowork. 0011, 0012 and 0013 (UI pass-through mode) done by Code. 0014 and 0015 done by Code and reviewed by Cowork. Next: 0016 (R9b), then R10b.

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | build + tests verified 2026-10-17 (173/173 pass) |
| M2 WPF shell | implemented | chart-first, ScottPlot, 1D, blending only |
| M3 Degrees 3/5/7 | implemented | degree 7 now tested (0004) |
| M4 Constraints | implemented | extrema, limit verification; shortest feasible duration (closed form + search) for auto duration with limits (0005); failure of the search is reported (0003); constant-velocity transitions still use the legacy search |
| M5 Constant velocity | partial | 1D and 3D straight stretches; Rules 1-3 for the transition durations implemented in Core, App uses the Core class (0007, 0008); 3D missing |
| M6 Blending | partial | 1D only; selectable degree, limit verification and input validation in the Core (0009); duration rule with limits to adjust (0010) |
| M7 Exact pass-through | implemented | exact global solve (m = 4 snap default for degree 9), 67/67 tests; reviewed by Cowork 2026-10-17 (reference values and code consistent) |
| M8 3D extension | partial | component-wise wrapping; 3D blending and straight-stretch constant velocity done (0015); curved constant speed in request 0016 |

## In progress

- 0016 Core 3D curved constant-speed stretch (Code) — issued; theory in `theory/ConstantVelocitySegment.md`

## Done

- 0015 Core 3D blending and straight constant velocity (Code) — closed 2026-10-18; note `coordination/inbox-docs/0015-core-3d-blending-and-straight-constant-velocity.md`. 473 tests (372 Core + 101 App). `BlendBuilder3D`/`BlendSegment3D`, `ConstantVelocityTrajectory3D` on straight stretches (curved → `NotSupportedException`), multi-component `ShortestFeasibleDuration.Find`, shared `TransitionDurationRules`; Core pass-through default reference now 1 s per segment (D23); peaks table lists unlimited quantities (D24).
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

`coordination/inbox-code/0016-core-3d-curved-constant-speed.md` (R9b). Afterwards R10b (3D UI), see `roadmap/ROADMAP.md`.
