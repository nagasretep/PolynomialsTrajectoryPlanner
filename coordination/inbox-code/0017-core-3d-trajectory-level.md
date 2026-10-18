# 0017 — Core: 3D trajectory level (common segment interface, builder, analysis)

Status: closed · From: Cowork · To: Code · Date: 2026-10-18

## Goal

The Core offers a complete 3D trajectory API at the trajectory level, so that the 3D mode of the UI (request 0018, after this one) is a thin layer as in 1D. Today `Trajectory3D` holds only `PolynomialSegment3D`, so it cannot hold the curved constant-speed stretch of 0016, and there is no 3D builder, no whole-trajectory analysis, no limit verification of a whole 3D trajectory.

## Reference documents

- `design/08-ui-first-prototype.md`, *3D Mode (decided 2026-10-18)* (what the UI will need)
- Notes `0015` and `0016` (blending 3D, `ConstantVelocityTrajectory3D`, `ConstantSpeedTrajectory3D`, `CurvedConstantSpeedStretch3D`, `TrajectorySamples3D`)
- `Services/TrajectoryBuilder1D.cs` and `Trajectories/Trajectory1D.cs` (the shape to mirror)

## Scope

1. **Common segment interface.** An interface (suggested name `ISegment3D`) with `Duration`, `EvaluateState(t)`, `EvaluatePosition(t)`, `EvaluateDerivative(t, order)`, implemented by `PolynomialSegment3D` and by the curved stretch. `Trajectory3D` takes `IEnumerable<ISegment3D>`; adapt callers; all existing tests stay green. Evaluation beyond the orders a segment supports behaves as today.
2. **`TrajectoryBuilder3D`**, mirroring `TrajectoryBuilder1D` where it applies:
   - `AddSegment(start, end, duration?, degree?, limits?)`: a null duration with limits means the shortest feasible duration (multi-component `ShortestFeasibleDuration.Find`); a null duration without limits follows the rule of the 1D builder (use the same, and say which in the note); a given duration is only verified against the limits.
   - `AddBlendBetween(precedingIndex, followingIndex, start, end, degree?, duration?, limits?)` through `BlendBuilder3D` (start and end as τ fractions or as absolute points: two overloads).
   - `AddConstantSpeed(segmentIndex, specification, speed, degree?, limits?, entryDuration?, exitDuration?, totalDuration?)` which replaces the segment by entry transition + stretch + exit transition built with `ConstantSpeedTrajectory3D` (always that class, also for straight stretches; its equality with the straight construction was shown in 0016). The specification covers points, τ fractions and arc-length fractions, as the factories of 0016.
   - `Build()` returns the `Trajectory3D`; the builder also exposes an ordered list of **part descriptors** (kind: Segment, Blend, CvEntry, CvStretch, CvExit; index of the original segment; start time; duration; for a blend the inherited duration and `AdjustedByLimits`; for a stretch its length, speed and the admissible speed with its binding limit) so the UI does not have to reconstruct it.
3. **Analysis of a `Trajectory3D`**:
   - `Sample(count, maxOrder)` returning `TrajectorySamples3D` over the whole duration, same call shape as `Trajectory1D.Sample`, with the part boundaries included among the sample times (so a junction is never skipped).
   - Per axis and per order 0–4: minimum, maximum and the maximum of the absolute value, each with the time where it occurs; exact for polynomial parts (existing `ExtremaAnalyzer`), from the stretch's own peak routine for stretches. Plus the maximum of the magnitude of the velocity over the trajectory (sampled, with refinement).
   - `VerifyLimits(trajectory, limits)`: per axis and quantity, the peak against the limit, the part and the time where the peak occurs; an exception naming part, axis and quantity when violated (same style as `KinematicLimitsViolatedException`), plus a non-throwing report of peak, limit and percentage for the UI table.
   - Total path length (sum of arc lengths; polynomial parts by the quadrature of 0016, which works on any polynomial segment).
4. **Tests** (Core.Tests): builder with a mixed trajectory (segment, blend, curved stretch) is continuous up to the order the degrees allow at every junction (1e-9); the 3D builder along one axis equals the 1D builder (durations and coefficients, 1e-9) for segment, blend and constant velocity; null duration with limits equals the multi-component shortest feasible duration; sample times contain every junction; extrema equal an independent dense sampling (1e-6); `VerifyLimits` reports the binding axis and part; a violated limit names part, axis and quantity; path length of a straight segment equals the chord and of the parabola of the theory equals 1.478943; `Trajectory3D` with the curved stretch evaluates the same state as the stretch itself at its times.

## Out of scope

3D pass-through (Core planner is 1D; a separate theory note and request will follow), UI, save/load, per-axis limits (one `KinematicLimits` applied to every axis), several constant-speed stretches in one segment.

## Acceptance criteria

- [ ] `Trajectory3D` contains a trajectory with a curved stretch; evaluating and sampling it works through the common interface.
- [ ] All existing tests pass unchanged except constructor adaptations; new tests as listed; `dotnet build` without new warnings.
- [ ] Along an axis the 3D builder reproduces the 1D builder to 1e-9 for segment, blend and constant velocity.
- [ ] Peaks and limit report agree with dense sampling to 1e-6; junction times are in the samples.
- [ ] The note states the test count, the null-duration rule without limits, and any design choice about the interface.

## Notes for the author (educational)

Explain why a common interface (rather than a special case in `Trajectory3D`) is the right way to hold two kinds of segment, why the builder returns part descriptors instead of the UI deriving them, and how the peak of the whole trajectory is found without sampling for polynomial parts.

Return note: `coordination/inbox-docs/0017-core-3d-trajectory-level.md`, ending with a "Suggested commits" section. Do not touch `roadmap/`, `theory/` or `design/`; make targeted edits in `coordination/STATUS.md` only.
