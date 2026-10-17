# 0007 — Constant-velocity transitions and tests

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

Run after request 0006 (it uses the explicit "not determined by the limits" outcome of `ShortestFeasibleDuration`).

## Goal

In Core, the entry and exit transitions of `ConstantVelocityTrajectory1D` take their durations from the same mechanism as ordinary segments (shortest feasible duration) when limits are active, failures are explicit, and the constant-velocity feature has a real test suite. The App starts using the Core class instead of its own copy.

## Reference documents

- `theory/ConstantVelocitySegment.md`, *Durations Of The Entry And Exit Transitions In 1D* (rule 1 decided; the other rules provisional)
- `theory/TrajectoryConstraints.md`, *Shortest Feasible Duration*
- `roadmap/discrepancies.md`: D8, D11

## Scope

- In scope:
  1. `ConstantVelocityTrajectory1D`: with limits active, each transition uses `ShortestFeasibleDuration` (entry and exit independently). Remove the use of the legacy `DurationSearcher.CreateSegmentWithAutoDuration` / `FindValidDuration` from this class.
  2. Explicit failure: if `vCv` exceeds the velocity limit, or a transition search fails, throw a clear exception (reuse `DurationSearchFailedException` where it fits); never return a trajectory that violates the limits.
  3. Optional constructor parameters for user-specified entry/exit durations (only verified against the limits, never altered).
  4. Without limits and without prescribed durations: keep today's rule (`max(0.5·T_cv, 0.1 s)`) and document it in the code as provisional (theory marks it open). Do not change it.
  5. If a transition has coincident boundary states ("not determined by the limits"), use the fallback duration of 0006.
  6. App: `PrototypeTrajectoryService` builds the constant-velocity case through the Core class (first step of R7 for this feature). Keep the App's behaviour for a prescribed total duration (remaining time split equally) as an App-level choice, documented as provisional.
  7. Constant-velocity test suite.
- Out of scope: 3D constant velocity, blending changes, UI changes, the rest of R7, any change to the no-limits default rule.

## Acceptance criteria

- [ ] With limits active, entry and exit durations equal `ShortestFeasibleDuration` of the corresponding transition (±1e-9 relative) and the full trajectory respects the limits (checked by the existing extrema verification).
- [ ] `vCv` above the velocity limit raises an explicit exception.
- [ ] Continuity at both junctions: position, velocity, acceleration, jerk and snap match the constant-velocity states (1e-9).
- [ ] The App no longer contains its own constant-velocity construction.
- [ ] `dotnet build` clean of new warnings; all tests pass (previous 173 plus the new ones).
- [ ] Legacy `FindValidDuration` is still present only if some other code needs it; list in the note who still uses it.

## Expected tests

- Rest-to-rest ends, several degrees (3/5/7/9), positive and negative direction: durations, continuity, total duration = T_in + T_cv + T_out.
- Non-zero boundary velocities (including opposite direction to `vCv`).
- Limits active: each limit (velocity, acceleration, jerk, snap) binding in turn; durations match the shortest feasible duration.
- `vCv` over the velocity limit; impossible limits for a transition (explicit failure).
- Prescribed durations: kept as given, verified.
- No limits: today's rule, pinned by a `Today_…` test.
- Degenerate input: equal `xCvStart`/`xCvEnd`, zero `vCv` (existing exception).

## Notes for the author (educational)

Explain in the return note: why the two transitions can be sized independently (their boundary states are both fixed), why the constant-velocity part is the one thing the limits cannot change, and how the exception differs from a trajectory that merely looks feasible.

Return note: `coordination/inbox-docs/0007-constant-velocity-transitions.md`, ending with a "Suggested commits" section.
