# Project status

Owner: whoever closes a task. Last updated: 2026-10-17.

## Current phase

Reorganization done; requests 0001-0005 done and reviewed; 0006 (numerics, duration fixes) and 0007 (constant-velocity transitions and tests) done by Code, awaiting Cowork review. Next: R7 consolidation (`roadmap/ROADMAP.md`).

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | build + tests verified 2026-10-17 (173/173 pass) |
| M2 WPF shell | implemented | chart-first, ScottPlot, 1D, blending only |
| M3 Degrees 3/5/7 | implemented | degree 7 now tested (0004) |
| M4 Constraints | implemented | extrema, limit verification; shortest feasible duration (closed form + search) for auto duration with limits (0005); failure of the search is reported (0003); constant-velocity transitions still use the legacy search |
| M5 Constant velocity | partial | 1D only; transitions use the shortest feasible duration, explicit limit failures, App uses the Core class (0007); 3D missing; no-limits and prescribed-total rules still provisional (D8) |
| M6 Blending | partial | 1D only; tested (0004), today's behaviour pinned by `Today_…` tests (D7) |
| M7 Exact pass-through | implemented | exact global solve (m = 4 snap default for degree 9), 67/67 tests; reviewed by Cowork 2026-10-17 (reference values and code consistent) |
| M8 3D extension | partial | component-wise wrapping; no 3D constant velocity or blending |

## In progress

- (none; next for Code: R7 consolidation)

## Done

- 0007 constant-velocity transitions and tests (Code) — closed 2026-10-17; note `coordination/inbox-docs/0007-constant-velocity-transitions.md`. 228/228 tests (40 new). `FeasibilityTolerance` now 0 so results pass `VerifyLimits`; `FindValidDuration` no longer used by production code.
- 0006 numerics and duration fixes (Code) — closed 2026-10-17; note `coordination/inbox-docs/0006-numerics-and-duration-fixes.md`. Diagonal scaling cuts cond(H) (d = 9, m = 5) from 5e15 to 2e11; `ShortestFeasibleDuration.Find` returns null when limits determine nothing. The strict 1e-12 cost criterion is not reachable (rounding noise ~1e-9), see the note.

- 0005 shortest feasible duration (Code) — closed 2026-10-17; note `coordination/inbox-docs/0005-shortest-feasible-duration.md`. 173/173 tests (36 new). Behaviour change: shorter auto durations when the old first guess (1 s) was already feasible.
- 0004 test coverage (Code) — closed 2026-10-17; note `coordination/inbox-docs/0004-test-coverage.md`. 137/137 at that point (56 new); no defect found; N = 30 pass-through solves fine (cond(H) up to 5e15 for d = 9, m = 5).
- 0003 cleanup batch (Code) — closed 2026-10-17; note `coordination/inbox-docs/0003-cleanup-batch.md`. D5 (failure signalling), D9, D12, D13, D14 done.

- 0002 exact minimum-snap solver (Code) — closed 2026-10-17; return note `coordination/inbox-docs/0002-minimum-snap-solver.md`. Build OK, 67/67 tests (24 old + 43 new). Cost on the reference data fell 44x versus the old heuristic. D1, D2, D6 addressed in code.
- 0001 baseline review (Code) — closed 2026-10-17; reviewed by Cowork the same day. Findings in `roadmap/discrepancies.md`.

## Blocked / to review

- No blocking decision. Decided on 2026-10-17: degree-9 default order = snap; auto duration with limits = shortest feasible duration; constant-velocity transitions with limits = shortest feasible duration each (D8). Non-blocking, open for the author: default transition duration without limits, split of a prescribed total duration (theory: `ConstantVelocitySegment.md`).
- Uncommitted work in the author's working directory: changes of 0006 and 0007 (code and coordination); `ShortestFeasibleDuration.cs`, `IDurationStrategy.cs` and `PrototypeTrajectoryService.cs` mix both (see the suggested commits in the notes).
- NU1701 (`SkiaSharp.Views.WPF`): transitive via `ScottPlot.WPF`, judged probably harmless; no action.

## Next requests for Code

`coordination/inbox-code/0006-numerics-and-duration-fixes.md`, then `0007-constant-velocity-transitions.md`; afterwards the consolidation R7 (`roadmap/ROADMAP.md`).
