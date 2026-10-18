# 0013 — UI: pass-through mode in 1D

Status: open · From: Cowork · To: Code · Date: 2026-10-17

## Goal

The App has a second mode, **Pass-through**, in which the author enters a waypoint table, degree, smoothness order, end conditions and optional limits, and sees the exact pass-through trajectory, with the shortest feasible durations when limits are active.

## Reference documents

- `design/08-ui-first-prototype.md`, *Pass-Through Mode (decided 2026-10-17)* (inputs, outputs, rules; read it fully)
- `theory/TrajectoryPassingThroughConstraintPoints.md`, *Pass-Through With Kinematic Limits*
- Notes `0011` (planner API) and `0012` (notices, `UiCulture`)

## Scope

- In scope:
  1. **Mode selector** `Blending | Pass-through` at the top of the window. Each mode keeps its data when the user switches. Blending mode behaves exactly as today.
  2. **Pass-through inputs** exactly as in the design section: degree; smoothness order $m$ (from 1 to $p$, defaults as stated: snap for degree 9, else $p$); end conditions (rest or values of $v, a, j, s$ as the degree uses, at both global ends); waypoint table (position, optional segment duration) with add/remove; optional total duration; optional global limits. Input errors (partly filled duration column, fewer than two waypoints, durations plus total both given, non-numeric text) are shown with a message naming the field, using `InputNumber` (0010).
  3. **Service**: `PrototypeTrajectoryService` (or a sibling class if cleaner) gets a pass-through entry point that builds the request for the Core planner of 0011 and returns the same kind of result as the blending mode (series, report tables, `Notices`). The App stays a thin layer: no duration or limit logic in the App.
  4. **Outputs**: the same stacked plots and cursor table as blending mode, over the whole trajectory, with the waypoints marked on the position plot; report with segment durations and total, $\lambda$ when computed, the state $(v, a, j, s)$ at every waypoint, the exact smoothness cost (`SmoothnessCost.Of`), extrema over the whole trajectory per quantity, coefficients of every segment. Notices as in the design section. All numbers follow the current culture (0012).
  5. Reuse existing components (tabs, table styles, chart panel) rather than duplicating them; if you must restructure `MainWindow`, keep the blending mode's behaviour and tests intact.
  6. Tests in `App.Tests`: service-level cases for the new entry point (rest-to-rest reference data of the theory with and without limits, prescribed durations, total duration, mixed durations error, fewer than two waypoints, non-zero end velocity, input parsing under `it-IT`). The window itself cannot be tested; list in the note what the author should check by hand.
- Out of scope: 3D, blend or constant velocity inside pass-through, save/load, editing on the plot, any change to the Core algorithms (if the Core needs a small accessor for the report, add it and say so).

## Acceptance criteria

- [ ] With waypoints (0, 1, 2), durations blank, no limits, degree 9, snap: total 2 s, interior velocity 2.275 (compare with the theory reference values).
- [ ] With limits v ≤ 2, a ≤ 3, j ≤ 10, s ≤ 50 on the same data: $\lambda^{\star} = 1.168271$, total 2.336542, acceleration at the limit.
- [ ] Prescribed durations violating a limit: error naming segment and limit; no trajectory shown.
- [ ] Switching mode keeps both modes' data; blending mode unchanged (existing App tests pass).
- [ ] `dotnet build` free of new warnings; all tests pass; the note states the test count and the hand-check list for the author.

## Notes for the author (educational)

Explain in the return note how the second mode reuses the same chart and report code, why the service keeps the mode logic out of `MainWindow`, and what the common factor $\lambda$ means for the user in plain words.

Return note: `coordination/inbox-docs/0013-ui-passthrough-mode.md`, ending with a "Suggested commits" section.
