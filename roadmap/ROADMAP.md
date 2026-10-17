# Roadmap

Owner: Cowork. Status of the milestones M1–M8 is tracked in `coordination/STATUS.md`; this file holds priorities and the plan. Items below are **proposed** and become real when the author confirms them.

## Priority order

| # | Item | Owner | Depends on | Output |
|---|---|---|---|---|
| R1 | Baseline hygiene: `.gitignore`, untrack build output, one baseline commit | author / Cowork | – | clean history point |
| R2 | Verify baseline: `dotnet build` and `dotnet test` green | Code | R1 | note in `inbox-docs/` with results |
| R3 | Review theory library Draft v1 against the code (find discrepancies) | Cowork | R2 | list of discrepancies, requests for Code |
| R4 | Exact minimum-snap pass-through solver (M7), replacing the heuristic | Code (theory: Cowork) | R3 | updated `PassThrough/`, tests, update to `theory/TrajectoryPassingThroughConstraintPoints.md` |
| R5 | 3D constant velocity and 3D blending (M5/M6 in 3D) | Code (theory: Cowork) | R3 | `Core` extension + tests |
| R6 | UI: exact pass-through and 3D input (open points in `design/08`) | Code | R4, R5 | updated `design/08` |
| R7 | Measurable success criteria (suggestion in `spec/05-success-criteria.md`) | Cowork, author decides | – | amendment of `spec/05` |
| R8 | Theory library from Draft v1 to reviewed, PDF build in `build/` | Cowork | R3 | reviewed documents |

## Open points carried from the existing documents

- Decision 2 representation (`BoundaryCondition1D/3D`) to be revisited when persistence/UI needs it (`design/07`).
- Where orchestration lives: `PrototypeTrajectoryService` is in `App`, the builders (`TrajectoryBuilder1D/3D`) are in `Core` — decide whether the prototype service should move into `Core` (R3 output).
- Project save/load, refined chart styling, differential cursors: out of scope for now (`design/08`).
- Future topics (`spec/04-future-topics.md`): orientation, time scaling, multi-axis synchronization, numerical stability — not scheduled.
