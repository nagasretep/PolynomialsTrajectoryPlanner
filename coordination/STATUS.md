# Project status

Owner: whoever closes a task. Last updated: 2026-10-17.

## Current phase

Reorganization done; baseline reviewed (0001); exact minimum-snap solver implemented and reviewed by Cowork (0002). Next: cleanup batch (0003), then test coverage (0004).

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | build + tests verified 2026-10-17 (24/24 pass) |
| M2 WPF shell | implemented | chart-first, ScottPlot, 1D, blending only |
| M3 Degrees 3/5/7 | implemented | degree-7 coefficients untested |
| M4 Constraints | implemented | extrema, limit verification, duration search (see D5) |
| M5 Constant velocity | partial | 1D only; App and Core have diverging implementations (D8, D11) |
| M6 Blending | partial | 1D only; no tests (D7) |
| M7 Exact pass-through | implemented | exact global solve (m = 4 snap default for degree 9), 67/67 tests; reviewed by Cowork 2026-10-17 (reference values and code consistent) |
| M8 3D extension | partial | component-wise wrapping; no 3D constant velocity or blending |

## In progress

- (none; next for Code: 0003 cleanup batch, then 0004 test coverage)

## Done

- 0002 exact minimum-snap solver (Code) — closed 2026-10-17; return note `coordination/inbox-docs/0002-minimum-snap-solver.md`. Build OK, 67/67 tests (24 old + 43 new). Cost on the reference data fell 44x versus the old heuristic. D1, D2, D6 addressed in code.
- 0001 baseline review (Code) — closed 2026-10-17; reviewed by Cowork the same day. Findings in `roadmap/discrepancies.md`.

## Blocked / to review

- Author decisions pending (do not block 0003/0004): degree-9 default order (snap or crackle, numbers in `roadmap/discrepancies.md`); meaning of "auto duration with limits" (D5).
- Uncommitted work in the author's working directory: all changes of 0002 (code and coordination) — the author commits (see `CLAUDE.md`).
- NU1701 (`SkiaSharp.Views.WPF`): transitive via `ScottPlot.WPF`, judged probably harmless; no action.

## Next requests for Code

`coordination/inbox-code/0003-cleanup-batch.md`, then `0004-test-coverage.md`; afterwards R7 consolidation (`roadmap/ROADMAP.md`).
