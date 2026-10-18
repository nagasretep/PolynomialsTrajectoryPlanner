# 0015 — Core 3D: blending and constant velocity on straight subpaths

Status: closed · From: Cowork · To: Code · Date: 2026-10-18

## Goal

The Core can blend two consecutive 3D segments and build a 3D constant-velocity trajectory when the selected subpath is a straight line, both with kinematic limits handled like their 1D counterparts. A genuinely curved subpath is reported as not supported yet (theory for it is still to be written by Cowork). Two small items left by 0014 are closed as well.

## Reference documents

- `theory/BlendingSegments.md`, 3D sections (*3D Blending Problem*, *Recovering The Corresponding Parameters In 3D*, *Sampling The Boundary States In 3D*, *How The 3D Blend Is Actually Computed*, *Absolute Vs Percentage Specification In 3D*) and *Decisions For The 1D Baseline* (duration rule, limits, degree)
- `theory/ConstantVelocitySegment.md`, *Decided 3D Interpretation*, *3D Straight-Chord Alternative*, *3D Transition Targets*, *Durations Of The Entry And Exit Transitions In 1D* (rules 1–3 carry over)
- `theory/TrajectoryConstraints.md` (component-wise treatment of limits; *Shortest Feasible Duration*)

## Scope

### Part A — 3D blending

1. A 3D blend builder mirroring `BlendBuilder1D`: two neighbouring 3D segments, blend window given by the **local parameter fractions** $\tau$ (primary entry point), optionally by **absolute points** on the neighbours. For an absolute point, find the parameter by minimizing the distance to the segment (scan + refinement as for `TauForPosition`); if the distance at the optimum exceeds a tolerance (say $10^{-9}$ times the segment's extent), reject with `ArgumentException` naming the blend and the distance. Empty window and a window removing a whole neighbour are rejected as in 1D.
2. Boundary states sampled from the neighbours (vector states up to the order the blend degree needs); coefficients component-wise with **one common duration** $T_b$ for the three components. Degree: selectable, default rule as in 1D (neighbours' degree if equal, else 9).
3. Duration with limits as decided for 1D: inherited if it passes `VerifyLimits` on all three components, otherwise the shortest feasible duration, where feasibility is the maximum over the components and the active limits (extend `ShortestFeasibleDuration` to a multi-component state if needed, or add a thin wrapper; do not duplicate the search). Prescribed duration: only verified. Limits of a blend: most restrictive of the neighbours. Expose inherited duration, chosen duration and `AdjustedByLimits` like `BlendSegment1D`.

### Part B — 3D constant velocity on a straight subpath

1. A 3D constant-velocity trajectory (entry transition, constant-velocity stretch, exit transition) for a stretch given by two points $\mathbf{p}_{cv}^{start}$, $\mathbf{p}_{cv}^{end}$ and a speed $v_{cv} > 0$ (direction from start to end; $T_{cv} = \lVert\Delta\mathbf{p}\rVert / v_{cv}$; constant velocity vector $v_{cv}\,\mathbf{n}$).
2. The transitions are ordinary 3D segments with vector boundary states (outer end given, inner end $(\mathbf{p}, v_{cv}\mathbf{n}, 0, 0, 0)$), common duration for the three components. Durations as in 1D: Rule 1 (limits active: shortest feasible, multi-component), Rule 2 (mean-velocity rule with the Euclidean distance and speeds $\lVert\mathbf{v}_b\rVert$ and $v_{cv}$), Rule 3 (prescribed total split in proportion to the shortest feasible durations), prescribed durations only verified. Velocity limit: the constant speed is checked component-wise against the per-axis velocity limit ($\lvert v_{cv} n_i\rvert \le v_{max}$ for every axis), explicit failure otherwise.
3. **Straight subpath detection.** When the stretch is delimited by points on a given 3D segment, the subpath is straight if the segment's path between them is collinear within a stated tolerance; if it is not, raise `NotSupportedException` with a message that says the curved multi-axis case is not implemented yet. If the stretch is given only by two points (no underlying segment), nothing to check.
4. Same exceptions as 1D (`KinematicLimitsViolatedException`, `DurationSearchFailedException`, `ArgumentException` for degenerate input).

### Part C — two small items from 0014

1. The default reference durations of the Core pass-through planner (`Plan(...)` without `referenceDurations`) use the A8 rule with a total of one second per segment, like the App (D23). Update the affected tests and say which.
2. The pass-through Summary table of peaks (App) also lists the quantities that have no active limit (limit column empty, percentage empty), after the limited ones (D24).

- Out of scope: curved 3D constant velocity (arc-length), 3D UI, 3D pass-through, any change to 1D behaviour except Part C.1.

## Acceptance criteria

- [ ] 3D blend between two straight segments along different directions: blend continuous in position, velocity, … up to the order the degree needs with both neighbours (1e-9); components computed independently equal the 1D blend of each component with the same duration.
- [ ] 3D blend with an active limit: inherited kept when feasible, shortest feasible otherwise, whole result passes `VerifyLimits` per component; adjustment flag correct.
- [ ] Absolute point off the segment is rejected; on the segment gives the same result as the $\tau$ entry point (1e-9).
- [ ] 3D constant velocity on a straight line along an axis equals the 1D construction (durations and coefficients, 1e-9); along a diagonal, the stretch has speed $v_{cv}$ exactly and transitions are continuous at both junctions.
- [ ] Rules 1–3 hold in 3D with the same tests as 1D adapted (Euclidean distances); speed above a per-axis velocity limit fails explicitly.
- [ ] Curved subpath: `NotSupportedException` with a clear message.
- [ ] Part C done; `dotnet build` free of new warnings; all tests pass; the note states the count.

## Expected tests

- The 1D tests of blending and constant velocity re-run in 3D (axis-aligned cases must reproduce the 1D numbers).
- Diagonal cases; opposite directions; limits binding on one axis only; a prescribed total; a curved segment for the `NotSupportedException`.

## Notes for the author (educational)

Explain in the return note why component-wise computation with a common duration is equivalent to the vector blend, why the straight-subpath case is exactly the straight-chord construction, and why the curved case needs a different time law (non-polynomial).

Return note: `coordination/inbox-docs/0015-core-3d-blending-and-straight-constant-velocity.md`, ending with "Suggested commits". Remember: small targeted edits to `STATUS.md` only; do not touch `roadmap/`, `theory/`, `design/`.
