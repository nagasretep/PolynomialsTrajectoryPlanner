# Project status

Owner: whoever closes a task. Last updated: 2026-10-17.

## Current phase

Reorganization done; baseline reviewed (request 0001 closed). Next: exact minimum-snap solver (request 0002, open).

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | build + tests verified 2026-10-17 (24/24 pass) |
| M2 WPF shell | implemented | chart-first, ScottPlot, 1D, blending only |
| M3 Degrees 3/5/7 | implemented | degree-7 coefficients untested |
| M4 Constraints | implemented | extrema, limit verification, duration search (see D5) |
| M5 Constant velocity | partial | 1D only; App and Core have diverging implementations (D8, D11) |
| M6 Blending | partial | 1D only; no tests (D7) |
| M7 Exact pass-through | **in progress** | heuristic today; exact solve specified in theory, request 0002 |
| M8 3D extension | partial | component-wise wrapping; no 3D constant velocity or blending |

## In progress

- 0002 exact minimum-snap solver (Code) — open

## Done

- 0001 baseline review (Code) — closed 2026-10-17; reviewed by Cowork the same day. Findings in `roadmap/discrepancies.md`.

## Blocked / to review

- Author decisions pending (do not block 0002): degree-9 default order (snap or crackle); meaning of "auto duration with limits". See `roadmap/discrepancies.md`.
- NU1701 (`SkiaSharp.Views.WPF`): transitive via `ScottPlot.WPF`, judged probably harmless; no action.

## Next requests for Code

See `roadmap/ROADMAP.md` (R5 tests, R6 cleanup, R7 consolidation).
