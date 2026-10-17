# 0008 — Constant-velocity duration rules 2 and 3

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

## Goal

`ConstantVelocityTrajectory1D` implements the two duration rules the author decided on 2026-10-17: the mean-velocity rule for transitions without limits (Rule 2), and the prescribed total duration split in proportion to the shortest feasible durations (Rule 3). The App stops doing its own equal split.

## Reference documents

- `theory/ConstantVelocitySegment.md`, *Durations Of The Entry And Exit Transitions In 1D* (Rules 1–3, all decided)
- `theory/TrajectoryConstraints.md`, *Shortest Feasible Duration*
- `roadmap/discrepancies.md`: D8

## Scope

- In scope:
  1. **Rule 2.** Without limits and without prescribed duration: `T = 2·|Δx| / (|v_b| + v_cv)`, `Δx` = distance covered by the transition, `v_b` = velocity at the outer end. If `Δx = 0` use the old fallback `max(0.5·T_cv, 0.1 s)`. `DefaultTransitionDuration` is replaced or reworked accordingly (keep a public, documented way to get the value).
  2. **Rule 3.** New optional constructor parameter `totalDuration` in `ConstantVelocityTrajectory1D` (both constructors). `R = totalDuration − T_cv` must be positive (`ArgumentException` otherwise, message with `T_cv`). Weights `s_in`, `s_out`: shortest feasible durations if limits are active (use `Find ?? Rule 2 value` when `null`), Rule 2 durations otherwise. `T_in = R·s_in/(s_in+s_out)`, `T_out = R − T_in`. With limits active and `R < s_in + s_out`: explicit failure (`KinematicLimitsViolatedException` or a more suitable existing exception) whose message states the minimum feasible total `T_cv + s_in + s_out`.
  3. **Combinations.** `totalDuration` with exactly one prescribed transition duration: the other transition takes the remainder (must be positive; verified against the limits). `totalDuration` with both transition durations prescribed: `ArgumentException`. A prescribed single duration without `totalDuration`: as today.
  4. **App.** `PrototypeTrajectoryService` passes the prescribed total duration to the Core constructor and loses its equal-split rule. Its existing message for a total shorter than the stretch may stay or be replaced by the Core message.
  5. Tests, and update of the `Today_…` test that pins the old no-limits rule.
- Out of scope: 3D, blending, the rest of R7, any change to Rule 1.

## Acceptance criteria

- [ ] Rule 2 values match the formula (1e-12) in several cases: rest ends, non-zero outer velocity, negative direction, `Δx = 0` fallback.
- [ ] With a prescribed total, `T_in + T_cv + T_out` equals the total (1e-12), and the ratio `T_in/T_out` equals `s_in/s_out`.
- [ ] With limits active and `R ≥ s_in + s_out`, the trajectory respects the limits; with `R < s_in + s_out` the failure is explicit and the message contains the minimum feasible total.
- [ ] One prescribed transition + total: the other takes the remainder; both prescribed + total: rejected.
- [ ] The App contains no duration split of its own.
- [ ] `dotnet build` clean of new warnings; all tests pass (228 plus the new ones).

## Expected tests

- Rule 2: formula, all four cases above, both transitions.
- Rule 3: without limits (weights = Rule 2 values), with limits (weights = shortest feasible), `R` exactly equal to `s_in + s_out` (both transitions at their minimum, feasible), `R` slightly below (failure), total ≤ `T_cv` (rejected).
- Combinations with prescribed durations.
- A case where the old equal split would violate a limit and the proportional split does not (e.g. very different entry/exit distances with an acceleration limit).

## Notes for the author (educational)

Explain in the return note why the proportional split respects the limits whenever any split can (`T_in ≥ s_in` and `T_out ≥ s_out` hold exactly when `R ≥ s_in + s_out`), and what changes for the user compared with the equal split. Mention anything that breaks the assumption that feasibility only improves when a transition gets longer; if a case violates it, report it.

Return note: `coordination/inbox-docs/0008-constant-velocity-duration-rules.md`, ending with a "Suggested commits" section.
