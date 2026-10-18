# 0020 — UI: pass-through in 3D (fourth mode)

Status: issued · From: Cowork · To: Code · Date: 2026-10-18

## Goal

The App has a fourth mode, **3D pass-through**: the author enters a table of 3D waypoints (vector input), degree, smoothness order, end conditions, optional durations or total and optional limits, and sees the exact pass-through trajectory with the shortest feasible common durations when limits are active.

## Reference documents

- `design/08-ui-first-prototype.md`, *3D Pass-Through Mode (decided 2026-10-18)* (read it fully)
- `theory/TrajectoryPassingThroughConstraintPoints.md`, *Common Durations And Kinematic Limits In 3D*
- Notes `0013` (the 1D pass-through mode: reuse its inputs, parsing and notices), `0018` (the 3D mode: reuse its result presentation), `0019` (Core: `PassThroughPlanner3D`, `TrajectoryBuilder3D.AddSegmentsFromWaypointsPassThrough` with `LastPassThroughResult`)

## Scope

1. **Mode selector** `Blending | Pass-through | 3D | 3D pass-through`; every mode keeps its data; the other three behave as today.
2. **Inputs** exactly as in the design section: degree; smoothness order $m$ with the 1D defaults; end conditions (rest or vectors $v, a, j, s$ as the degree uses, `X | Y | Z` rows, empty box = 0); waypoint table with columns X, Y, Z and the optional segment duration, add / remove rows, at least two waypoints; optional total duration; optional limits (one value per quantity for all axes). Input errors (partly filled duration column, fewer than two waypoints, durations plus total, non-numeric text, empty waypoint coordinate) name the field, row and axis, using `InputNumber`.
3. **Service**: a 3D pass-through entry point in the App service layer (sibling of `Prototype3DTrajectoryService`, or an extension of it if cleaner) that builds the request for the Core planner of 0019 and returns the same result type as the 3D mode (series, parts, report tables, `Notices`) plus the pass-through items below. No duration, scaling or limit logic in the App.
4. **Outputs**: reuse the 3D presentation of 0018 (X / Y / Z time plots with the norm option and part boundaries, the Path tab with the three projections, cursor table); waypoints marked on the projections and on the position plot; report: segment durations and total; $\lambda$ (when computed) with the binding quantity and axis; the vector state ($v, a, j, s$ as the degree uses) at every waypoint; the total smoothness cost (sum over the axes; use the Core cost function, extended per component if needed, and say so); peaks versus limits per axis (use the Core `LimitReport`); extrema; coefficients per segment. Notices as in the design section. All numbers follow the current culture.
5. **Tests** (App.Tests, service level): the reference example of the theory ((0,0,0), (1,0,0), (1,3,0), (1,3,4), durations blank, limits $v \le 2$, $a \le 3$, $j \le 10$, $s \le 50$: $\lambda = 4.990502$, total 14.971506, binding velocity on $x$); same data without limits (total 3 s, reference durations 0.375, 1.125, 1.5); prescribed durations violating a limit (error with segment, axis, quantity); total duration; partly filled durations error; fewer than two waypoints; non-zero end velocity vector; the 1D case along one axis agrees with the 1D service; `it-IT` parsing of the vectors; the other three modes unchanged (existing tests pass). The window cannot be tested: list the hand checks in the note, including the reference example with its numbers.

## Out of scope

Blends or constant-speed stretches inside the pass-through, saving and loading, editing on the plot, per-axis limits, any change to the Core (if a small accessor is missing, add it and say so).

## Acceptance criteria

- [ ] Reference example: $\lambda^\star = 4.990502$, total 14.971506 s, binding quantity velocity on axis $x$ shown in the report and in the status bar.
- [ ] Without limits: total 3 s with durations 0.375 / 1.125 / 1.5.
- [ ] Waypoints marked on the path projections; the path passes through them.
- [ ] Switching among the four modes keeps all data; the three existing modes unchanged; `dotnet build` free of new warnings; all tests pass; the note states the test count and the hand-check list.

## Notes for the author (educational)

Explain in the return note how the fourth mode reuses the 3D presentation, what the binding quantity and axis tell the user (which axis limits the speed of the whole motion), and why the same scale factor applies to all axes (the path shape is preserved, only the time scale changes).

Return note: `coordination/inbox-docs/0020-ui-3d-pass-through.md`, ending with a "Suggested commits" section. Do not touch `roadmap/`, `theory/` or `design/`; targeted edits only in `coordination/STATUS.md`.
