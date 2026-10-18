# Roadmap

Owner: Cowork. Milestone status (M1–M8) is in `coordination/STATUS.md`; the theory ↔ code findings are in `roadmap/discrepancies.md`. Items marked "proposed" become real when the author confirms them. Last updated: 2026-10-17.

## Priority order

| # | Item | Owner | Depends on | Output / state |
|---|---|---|---|---|
| R1 ✅ | Baseline hygiene: `.gitignore`, untrack build output, restructure | author / Cowork | – | done 2026-10-17 |
| R2 ✅ | Verify baseline; inventory; NU1701 assessment | Code | R1 | request 0001 closed |
| R3 | Theory review against the code (discrepancy register) | Cowork | R2 | in progress: no open theory/code discrepancy except the low-priority D18 |
| R4 ✅ | Exact minimum-snap pass-through solver (M7) + single duration mechanism | Code | R3 | request 0002 closed; 67/67 tests |
| R5 ✅ | Cleanup batch | Code | – | request 0003 closed |
| R6 ✅ | Test coverage (blending, degree 7, trajectories, extrema, N = 30) | Code | R5 | request 0004 closed; no defect found |
| R6b ✅ | Shortest feasible duration with limits | Code | R5 | request 0005 closed |
| R6c ✅ | Numerics and duration fixes: diagonal scaling in the pass-through solve, zero-length segment, `Cost` documentation | Code | R6b | request 0006 closed; 188/188 |
| R7 ✅ | Consolidation: the App is a thin layer over the Core; blending with degree, limits and validation; legacy removed | Code | R6c, R8 | request 0009 closed; 317/317 |
| R7b ✅ | Blend duration rule (inherited if feasible), number parsing culture | Code | R7 | request 0010 closed; 359/359 |
| R7c ✅ | Report polish: blend adjustment notice (D22), one output culture (D21) | Code | R7b | request 0012 closed; 404/404 |
| R8 ✅ | Constant-velocity transitions: rules 1–3 decided and implemented | Code | author | requests 0007 and 0008 closed; 256/256 |
| R9a | Core 3D: blending (component-wise, common duration), constant velocity on straight subpaths | Code | R7 | **request 0015 issued** |
| R9b | Curved 3D constant-speed stretch: theory written 2026-10-18; Core in request 0016 | Cowork (theory) / Code | R9a | **theory done; request 0016 issued (after 0015)** |
| R10a ✅ | UI: pass-through mode in 1D | Code | R7, R11 | request 0013 closed; 428 tests. Follow-up 0014 (default durations, peaks vs limits) issued |
| R10b | UI: 3D input (axis-by-axis or vector, to decide) | Code | R9, R10a | proposed |
| R11 ✅ | Pass-through with kinematic limits: common scale factor λ, closed form for rest ends, search otherwise | Code | R7b | request 0011 closed; 394/394 |
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
