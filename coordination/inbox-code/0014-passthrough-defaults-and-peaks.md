# 0014 — Pass-through: uniform default durations and peaks against limits

Status: closed · From: Cowork · To: Code · Date: 2026-10-18

## Goal

Two follow-ups from the review of 0013: (1) the default durations of the pass-through mode are the same rule with and without limits; (2) the Summary shows, for each active limit, the peak reached against the limit.

## Reference documents

- `design/08-ui-first-prototype.md`, *Pass-Through Mode* (updated 2026-10-18: bullets on blank durations and on the numeric report)
- `design/07-implementation-plan.md` A8 (proportional rule); note `coordination/inbox-docs/0013-ui-passthrough-mode.md` (first open question)

## Scope

- In scope:
  1. **Uniform default.** With blank segment durations and no total, the reference durations are always the A8 proportional rule with a total of one second per segment (number of segments × 1 s), with and without limits. Without limits they are the durations used. Remove the "each segment takes 1 s" path. Where the rule lives (App or Core) follows the thin-layer principle: it must not be duplicated; if the Core planner already does it for the limits case, make the no-limits case use the same code.
  2. **Peaks against limits.** In pass-through mode, when at least one limit is active, the Summary lists for each active limit: quantity, peak over the whole trajectory, limit, percentage of the limit, and the segment (numbered from 1) where the peak occurs. With `λ` applied, the binding limit is at 100 %. The Core accessor (peak and its segment) is added if it does not exist (`ExtremaAnalyzer`), with a test.
  3. Tests: equally and unequally spaced waypoints without limits (durations proportional to distance, total = number of segments); the 0013 reference data unchanged (equal spacing: total 2 s, interior velocity 2.275); peaks table in a limited case (binding limit at 100 % ± 1e-6, others below), culture formatting under `it-IT`.
- Out of scope: blending mode, 3D, any change to the algorithms of the Core other than the accessor.

## Acceptance criteria

- [ ] Waypoints (0, 1, 3) without limits and durations: durations in ratio 1 : 2, total 2 s (A8 with φ as implemented); with limits the reference durations are the same before scaling.
- [ ] The Summary shows the peaks table when limits are active and none when no limit is active.
- [ ] `dotnet build` free of new warnings; all tests pass; the note states the count.

## Notes for the author (educational)

Explain in the return note why equal durations on unequally spaced waypoints would produce a fast first segment and a slow second one (the optimizer compensates with large derivatives), and what the one-second-per-segment total means.

Return note: `coordination/inbox-docs/0014-passthrough-defaults-and-peaks.md`, ending with "Suggested commits".
