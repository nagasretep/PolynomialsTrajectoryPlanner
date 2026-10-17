# Roadmap

Owner: Cowork. Milestone status (M1–M8) is in `coordination/STATUS.md`; the theory ↔ code findings are in `roadmap/discrepancies.md`. Items marked "proposed" become real when the author confirms them. Last updated: 2026-10-17.

## Priority order

| # | Item | Owner | Depends on | Output / state |
|---|---|---|---|---|
| R1 ✅ | Baseline hygiene: `.gitignore`, untrack build output, restructure | author / Cowork | – | done 2026-10-17 |
| R2 ✅ | Verify baseline; inventory; NU1701 assessment | Code | R1 | request 0001 closed |
| R3 | Theory review against the code (discrepancy register) | Cowork | R2 | in progress: D4, D7 (code side), D8 open |
| R4 ✅ | Exact minimum-snap pass-through solver (M7) + single duration mechanism | Code | R3 | request 0002 closed; 67/67 tests |
| R5 | Cleanup batch: `c_i` labels, `DurationSearcher` failure signalling, `SampleTimes`, `Epsilon`, proportional-duration floor | Code | – | **request 0003 issued** |
| R6 | Test coverage: Blending, degree 7, `Trajectories`, extrema analytic checks, N = 30 pass-through | Code | R5 | **request 0004 issued** |
| R7 | Consolidation: the App uses the Core builders, constant-velocity, blend (selectable degree, limit check) and limit-validation code | Code | R5, R6, D8 | proposed 0005 |
| R8 | Constant-velocity transition rule written in theory (D8), after author input | Cowork | author | – |
| R9 | 3D constant velocity and 3D blending | Code (theory exists in `theory/ConstantVelocitySegment.md`) | R7 | proposed |
| R10 | UI: exact pass-through and 3D input (open points in `design/08`) | Code | R7, R9 | proposed |
| R11 | Pass-through with kinematic limits (time-scaling loop around the solver, D5) | Cowork (theory) + Code | author decision D5 | proposed |
| R12 | Measurable success criteria (suggestion in `spec/05-success-criteria.md`) | Cowork, author decides | – | proposed |
| R13 | Theory library from Draft v1 to reviewed; PDF build into `build/` | Cowork | R3 | proposed |

## Decisions waiting for the author

1. Degree 9 default order for exact pass-through: snap ($m = 4$) or crackle ($m = 5$) — numbers in `roadmap/discrepancies.md`. Default until decided: snap; changing it later is a one-line change.
2. "Auto duration with limits": shortest feasible duration or initial guess if valid (D5). Blocks the behavioural part of R11 and any change to `DurationSearcher`.

## Open points carried from the existing documents

- Decision 2 representation (`BoundaryCondition1D/3D`) to be revisited when persistence/UI needs it (`design/07`).
- Project save/load, refined chart styling, differential cursors: out of scope for now (`design/08`).
- Future topics (`spec/04-future-topics.md`): orientation, time scaling, multi-axis synchronization, numerical stability — not scheduled.
