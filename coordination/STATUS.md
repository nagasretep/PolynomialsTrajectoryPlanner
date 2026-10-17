# Project status

Owner: whoever closes a task. Last updated: 2026-10-17.

## Current phase

Repository reorganization and coordination setup (Cowork ↔ Code). See `roadmap/decisions/ADR-0001-repository-restructure.md`.

## Milestones (from `design/07-implementation-plan.md` and `design/08-ui-first-prototype.md`)

| Milestone | State | Notes |
|---|---|---|
| M1 Core degree-9 engine (1D) | implemented | to be re-verified by build + tests |
| M2 WPF shell | implemented | chart-first, ScottPlot, 1D, blending only |
| M3 Degrees 3/5/7 | implemented | |
| M4 Constraints | implemented | extrema, limit verification, duration search |
| M5 Constant velocity | partial | 1D only |
| M6 Blending | partial | 1D only |
| M7 Exact pass-through | provisional | `PassThroughSolver1D` is a heuristic, not the minimum-snap solve of Decision 5 |
| M8 3D extension | partial | component-wise wrapping; no 3D constant velocity or blending |

## In progress

- (none yet)

## Blocked / to review

- Baseline not yet verified: build and tests have not been run since the reorganization.

## Next requests for Code

See `roadmap/ROADMAP.md`.
