# Roadmap

Owner: Cowork. Milestone status (M1–M8) is in `coordination/STATUS.md`; the theory ↔ code findings are in `roadmap/discrepancies.md`. Items marked "proposed" become real when the author confirms them. Last updated: 2026-10-17.

## Priority order

| # | Item | Owner | Depends on | Output / state |
|---|---|---|---|---|
| R1 ✅ | Baseline hygiene: `.gitignore`, untrack build output, restructure | author / Cowork | – | done 2026-10-17 |
| R2 ✅ | Verify baseline; inventory; NU1701 assessment | Code | R1 | request 0001 closed |
| R3 | Theory review against the code (discrepancy register) | Cowork | R2 | in progress: D4, D7 (code side) open |
| R4 ✅ | Exact minimum-snap pass-through solver (M7) + single duration mechanism | Code | R3 | request 0002 closed; 67/67 tests |
| R5 ✅ | Cleanup batch | Code | – | request 0003 closed |
| R6 ✅ | Test coverage (blending, degree 7, trajectories, extrema, N = 30) | Code | R5 | request 0004 closed; no defect found |
| R6b ✅ | Shortest feasible duration with limits | Code | R5 | request 0005 closed |
| R6c ✅ | Numerics and duration fixes: diagonal scaling in the pass-through solve, zero-length segment, `Cost` documentation | Code | R6b | request 0006 closed; 188/188 |
| R7 | Consolidation: the App uses the Core builders, constant-velocity, blend (selectable degree, limit check, input validation) and limit-validation code; remove legacy `FindValidDuration` and per-segment `ComputeDuration` | Code | R6c, R8 | proposed |
| R8 | Constant-velocity transitions: rules 1 (done, 0007), 2 and 3 decided 2026-10-17; implementation in request 0008 | Code | author | **request 0008 issued** |
| R9 | 3D constant velocity and 3D blending | Code (theory exists in `theory/ConstantVelocitySegment.md`) | R7 | proposed |
| R10 | UI: exact pass-through and 3D input (open points in `design/08`) | Code | R7, R9 | proposed |
| R11 | Pass-through with kinematic limits (common time-scaling factor around the solver; closed form for rest ends, D5) | Cowork (theory) + Code | R6b | proposed |
| R12 | Measurable success criteria (suggestion in `spec/05-success-criteria.md`) | Cowork, author decides | – | proposed |
| R13 | Theory library from Draft v1 to reviewed; PDF build into `build/` | Cowork | R3 | proposed |

## Decisions taken by the author

1. Degree 9 default order for exact pass-through: **snap** ($m = 4$), 2026-10-17.
2. Auto duration with limits: **shortest feasible duration**, 2026-10-17.

No decision is pending.

## Open points carried from the existing documents

- Decision 2 representation (`BoundaryCondition1D/3D`) to be revisited when persistence/UI needs it (`design/07`).
- Project save/load, refined chart styling, differential cursors: out of scope for now (`design/08`).
- Future topics (`spec/04-future-topics.md`): orientation, time scaling, multi-axis synchronization, numerical stability — not scheduled.
