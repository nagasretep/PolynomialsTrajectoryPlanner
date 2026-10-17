# Project status

Owner: whoever closes a task. Last updated: 2026-10-17.

## Current phase

Reorganization done; baseline reviewed (0001); exact minimum-snap solver (0002); cleanup batch (0003), test coverage (0004) and shortest feasible duration (0005) done by Code and reviewed by Cowork (2026-10-17). Next: request 0006 (numerics, duration fixes); then, after the author's decision on D8, constant-velocity theory and tests, then the R7 consolidation.

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | build + tests verified 2026-10-17 (173/173 pass) |
| M2 WPF shell | implemented | chart-first, ScottPlot, 1D, blending only |
| M3 Degrees 3/5/7 | implemented | degree 7 now tested (0004) |
| M4 Constraints | implemented | extrema, limit verification; shortest feasible duration (closed form + search) for auto duration with limits (0005); failure of the search is reported (0003); constant-velocity transitions still use the legacy search |
| M5 Constant velocity | partial | 1D only; App and Core have diverging implementations (D8, D11) |
| M6 Blending | partial | 1D only; tested (0004), today's behaviour pinned by `Today_…` tests (D7) |
| M7 Exact pass-through | implemented | exact global solve (m = 4 snap default for degree 9), 67/67 tests; reviewed by Cowork 2026-10-17 (reference values and code consistent) |
| M8 3D extension | partial | component-wise wrapping; no 3D constant velocity or blending |

## In progress

- 0006 numerics and duration fixes (Code) — open

## Done

- 0005 shortest feasible duration (Code) — closed 2026-10-17; note `coordination/inbox-docs/0005-shortest-feasible-duration.md`. 173/173 tests (36 new). Behaviour change: shorter auto durations when the old first guess (1 s) was already feasible.
- 0004 test coverage (Code) — closed 2026-10-17; note `coordination/inbox-docs/0004-test-coverage.md`. 137/137 at that point (56 new); no defect found; N = 30 pass-through solves fine (cond(H) up to 5e15 for d = 9, m = 5).
- 0003 cleanup batch (Code) — closed 2026-10-17; note `coordination/inbox-docs/0003-cleanup-batch.md`. D5 (failure signalling), D9, D12, D13, D14 done.

- 0002 exact minimum-snap solver (Code) — closed 2026-10-17; return note `coordination/inbox-docs/0002-minimum-snap-solver.md`. Build OK, 67/67 tests (24 old + 43 new). Cost on the reference data fell 44x versus the old heuristic. D1, D2, D6 addressed in code.
- 0001 baseline review (Code) — closed 2026-10-17; reviewed by Cowork the same day. Findings in `roadmap/discrepancies.md`.

## Blocked / to review

- Author decision pending (D8): approve or change the proposed rule for constant-velocity transition durations (end of `roadmap/discrepancies.md`). Decided earlier on 2026-10-17: degree-9 default order = snap; auto duration with limits = shortest feasible duration (theory: `TrajectoryConstraints.md`, *Shortest Feasible Duration*).
- Uncommitted work in the author's working directory: all changes of 0003, 0004 and 0005 (code and coordination); `IDurationStrategy.cs` and `PrototypeTrajectoryService.cs` mix 0003 and 0005 (see the suggested commits in the notes).
- NU1701 (`SkiaSharp.Views.WPF`): transitive via `ScottPlot.WPF`, judged probably harmless; no action.

## Next requests for Code

`coordination/inbox-code/0006-numerics-and-duration-fixes.md`; afterwards constant-velocity tests (after D8) and the consolidation R7 (`roadmap/ROADMAP.md`).
