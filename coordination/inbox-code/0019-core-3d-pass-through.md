# 0019 — Core: 3D pass-through with common durations and limits

Status: closed · From: Cowork · To: Code · Date: 2026-10-18

## Goal

The Core has an exact pass-through planner for 3D waypoints, with the same duration rules and the same kinematic-limits handling as `PassThroughPlanner1D` (0011, 0014), so that a later request can add pass-through to the 3D mode of the UI.

## Reference documents

- `theory/TrajectoryPassingThroughConstraintPoints.md`, *Common Durations And Kinematic Limits In 3D (decided 2026-10-18)* (read it fully, including the reference values) and *Pass-Through With Kinematic Limits*
- Notes `0011`, `0014` (planner API, A8 default, report), `0015`/`0017` (multi-component `ShortestFeasibleDuration`, `TrajectoryBuilder3D`, `TrajectoryAnalyzer3D`)

## Scope

1. **`PassThroughPlanner3D`** (name free) with the same shape as `PassThroughPlanner1D.Plan`: waypoints as `Point3D[]`, degree, smoothness criterion, `KinematicLimits?` (one value per quantity, applied to each axis), optional prescribed durations (common to the axes), optional start / end boundary states as `State3D` (non-prescribed derivatives zero). Result analogous to `PassThroughResult`: the `PolynomialSegment3D`s, the final durations, the reference durations, $\lambda$ (null when not computed), whether the limits adjusted anything, and the **binding quantity and axis**.
2. **Computation**: for given durations, three scalar solves of the existing 1D solver (reuse; do not copy), same durations.
3. **Durations**: prescribed durations kept and verified (error naming segment, axis, quantity; `KinematicLimitsViolatedException` style of 0017); a prescribed total uses the A8 proportional rule with **Euclidean chords**; with nothing prescribed: A8 with a total of one second per segment (same default as 1D, 0014), and with active limits the shortest feasible common scale $\lambda$ on those reference durations. Equal consecutive waypoints (zero chord) and all-equal waypoints are handled exactly as the 1D planner does; state in the note what that is.
4. **Limits**: zero boundary derivatives: the closed form $\lambda^\star = \max_{k,a} (P_{k,a}(1)/L_k)^{1/k}$ from one solve per axis at $\lambda=1$ plus the second solve at $\lambda^\star$ (as 1D); non-zero boundary derivatives: the existing $\lambda$ search with $F$ = maximum over axes (generalise the 1D search once, no second copy). The peaks come from `ExtremaAnalyzer`. Constant axes impose nothing; explicit failure as in 1D when the search fails.
5. **`TrajectoryBuilder3D.AddSegmentsFromWaypointsPassThrough`** (already present) is rewired to the planner; its old behaviour without limits is kept where it was correct (check that its tests stay green; if a test expectation encodes a different default, say so in the note).
6. **Tests** (Core.Tests): along one axis the 3D planner equals the 1D planner (durations, coefficients, $\lambda$, 1e-9) with and without limits and with a non-zero boundary velocity; the reference example of the theory (waypoints $(0,0,0),(1,0,0),(1,3,0),(1,3,4)$): reference durations $(0.375, 1.125, 1.5)$, $\lambda^\star = 4.990502$, total $14.971506$, binding quantity velocity on axis $x$, peaks at $\lambda^\star$ to $10^{-5}$; scaling law $P_k(\lambda) = \lambda^{-k}P_k(1)$ per axis to $10^{-8}$; continuity of the state at every waypoint up to the order the degree allows (1e-9); prescribed duration violation names segment, axis, quantity; total prescribed; all-equal waypoints; a constant axis; a non-zero boundary velocity vector solved by search and verified by `TrajectoryAnalyzer3D.LimitReport`; degrees 3, 5, 7, 9.

## Out of scope

UI (a later request), blends or constant-speed stretches inside the pass-through, per-axis limits, optimisation of the duration proportions, any change to the 1D behaviour.

## Acceptance criteria

- [ ] Reference example reproduced as listed (reference durations, $\lambda^\star$, total, binding axis, peaks).
- [ ] Along one axis identical to the 1D planner (1e-9).
- [ ] The $\lambda$ search exists once for 1D and 3D; the 1D tests pass unchanged.
- [ ] `dotnet build` without new warnings; all tests pass; the note gives the test count and the zero-chord rule.

## Notes for the author (educational)

Explain why the 3D optimum is three independent 1D optima tied only by the durations, why one scale factor $\lambda$ scales the whole vector trajectory (and what it does to the path shape: nothing), and why the chord rule can give a large $\lambda$ when one axis does all its motion in a short segment.

Return note: `coordination/inbox-docs/0019-core-3d-pass-through.md`, ending with a "Suggested commits" section. Do not touch `roadmap/`, `theory/` or `design/`; targeted edits only in `coordination/STATUS.md`.
