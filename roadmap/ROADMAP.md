# Roadmap

Owner: Cowork. Milestone status (M1–M8) is in `coordination/STATUS.md`; the theory ↔ code findings are in `roadmap/discrepancies.md`. Items marked "proposed" become real when the author confirms them. Last updated: 2026-10-17.

## Priority order

| # | Item | Owner | Depends on | Output / state |
|---|---|---|---|---|
| R1 ✅ | Baseline hygiene: `.gitignore`, untrack build output, restructure | author / Cowork | – | done 2026-10-17 |
| R2 ✅ | Verify baseline; inventory; NU1701 assessment | Code | R1 | request 0001 closed; build OK, 24/24 tests |
| R3 | Theory review against the code (discrepancy register) | Cowork | R2 | in progress: D1 formulation written; D4, D7, D8 open |
| R4 | Exact minimum-snap pass-through solver (M7) + single duration mechanism | Code (theory done) | R3 (D1) | **request 0002 issued** |
| R5 | Test request: Blending, degree-7 coefficients, `Trajectories`, duration strategies | Code | R4 | proposed 0003 (24 tests today, none for these areas) |
| R6 | Cleanup batch: `a_i` → `c_i` labels in the App, `DurationSearcher` failure signalling, `SampleTimes` robustness, `Epsilon`, `using` leftovers | Code | – | proposed 0004 |
| R7 | Consolidation: the App uses the Core builders, constant-velocity, blend and limit-validation code instead of its own copies | Code | R4, D8 | proposed 0005 |
| R8 | Constant-velocity transition rule written in theory (D8), after author input | Cowork | author | – |
| R9 | 3D constant velocity and 3D blending | Code (theory exists in `theory/ConstantVelocitySegment.md`) | R7 | proposed |
| R10 | UI: exact pass-through and 3D input (open points in `design/08`) | Code | R4, R7, R9 | proposed |
| R11 | Measurable success criteria (suggestion in `spec/05-success-criteria.md`) | Cowork, author decides | – | proposed |
| R12 | Theory library from Draft v1 to reviewed; PDF build into `build/` | Cowork | R3 | proposed |

## Decisions waiting for the author

1. Degree 9 default order for exact pass-through: snap ($m = 4$) or crackle ($m = 5$) — see `roadmap/discrepancies.md`. Does not block R4.
2. "Auto duration with limits": shortest feasible duration or initial guess if valid (D5). Blocks the `DurationSearcher` part of R6.

## Open points carried from the existing documents

- Decision 2 representation (`BoundaryCondition1D/3D`) to be revisited when persistence/UI needs it (`design/07`).
- Project save/load, refined chart styling, differential cursors: out of scope for now (`design/08`).
- Future topics (`spec/04-future-topics.md`): orientation, time scaling, multi-axis synchronization, numerical stability — not scheduled.
