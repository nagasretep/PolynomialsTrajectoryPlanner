# Project status

Owner: whoever closes a task. Last updated: 2026-10-17.

## Current phase

Repository reorganized (2026-10-17, ADR-0001) and baseline verified; first request to Code open (`coordination/inbox-code/0001-baseline-review.md`).

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | build + tests verified 2026-10-17 (24/24 pass) |
| M2 WPF shell | implemented | chart-first, ScottPlot, 1D, blending only |
| M3 Degrees 3/5/7 | implemented | |
| M4 Constraints | implemented | extrema, limit verification, duration search |
| M5 Constant velocity | partial | 1D only |
| M6 Blending | partial | 1D only |
| M7 Exact pass-through | provisional | `PassThroughSolver1D` is a heuristic, not the minimum-snap solve of Decision 5 |
| M8 3D extension | partial | component-wise wrapping; no 3D constant velocity or blending |

## In progress

- 0001 baseline review (Code) — open

## Blocked / to review

- Build warning NU1701 (`SkiaSharp.Views.WPF` restored for .NET Framework) — to be assessed in 0001.
- Only 24 tests for M1–M8: coverage per area unknown — to be inventoried in 0001.

## Next requests for Code

See `roadmap/ROADMAP.md`.
