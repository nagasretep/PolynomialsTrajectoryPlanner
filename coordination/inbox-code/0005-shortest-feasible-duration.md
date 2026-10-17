# 0005 — Shortest feasible duration with limits

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

## Goal

Implement the author's decision of 2026-10-17: when a segment's duration is **not** prescribed and limits are active, compute the **shortest duration that satisfies all active limits**. Run after 0003 (failure signalling) and 0004 (tests).

## Reference documents

- `theory/TrajectoryConstraints.md` — section *Shortest Feasible Duration* (feasibility function, closed form, search algorithm), and the updated *Generic Constraint-Imposition Algorithm*.
- `design/07-implementation-plan.md` — appendix A9.
- `roadmap/discrepancies.md` — D5.

## Scope

1. **Search.** Change `DurationSearcher` (or add a clearly named method and make it the default for auto duration) to follow the theory: feasibility $F(T) = \max_k P_k(T)/L_k - 1$ over the **active** limits, geometric growth from a lower starting duration (for an active velocity limit start no lower than $\lvert \Delta x \rvert / v_{max}$), bisection to $T^{\star}$, explicit failure (as defined in 0003) when nothing feasible exists up to the cap. A geometric ratio around 1.05 is suggested; state your choice and the cap.
2. **Closed form.** When all prescribed boundary derivatives of order $1 \dots p-1$ are zero, compute $T^{\star} = \max_k (\lvert \Delta \rvert c_k / L_k)^{1/k}$ with $c_k$ taken from the unit profile (distance 1, duration 1, via `ExtremaAnalyzer`). Use it as the fast path; keep the search for the general case.
3. **Callers.** `ConstraintAwareDuration` and the App's "auto duration with limits" use the new behaviour. A prescribed duration stays "verify only" (a violation is reported, never repaired). With no active limit, nothing is searched: the existing default/prescribed duration applies. Show the computed duration to the user in the App's report.
4. **Behaviour change note.** State in the return note how results differ from the old behaviour on one example (for instance degree 9, $\Delta = 1$, $v_{max} = 1$: old result versus $T^{\star} = 315/128 \approx 2.4609$).

## Out of scope

Pass-through with limits (roadmap R11); blend and constant-velocity durations; UI redesign beyond showing the computed duration.

## Acceptance criteria

- [ ] Rest-to-rest, each degree 3/5/7/9, velocity limit only: $T^{\star} = c_1 \lvert \Delta \rvert / v_{max}$ with $c_1 = 3/2,\ 15/8,\ 35/16,\ 315/128$ (relative tolerance $10^{-6}$); the same result from the search path and from the closed form.
- [ ] Acceleration-only and jerk-only limits: closed form and search agree; $T^{\star}$ scales as $\sqrt{\lvert \Delta \rvert}$ and $\lvert \Delta \rvert^{1/3}$ respectively.
- [ ] Several limits at once: the larger of the individual $T^{\star}$ wins.
- [ ] Non-zero boundary velocity case: $F(T^{\star}) \approx 0$ and $F(T) > 0$ for $T$ slightly below $T^{\star}$ (check on a fine grid); the result is not larger than any feasible duration on the grid.
- [ ] Infeasible case (for example a prescribed boundary velocity above the velocity limit): failure is reported, not a wrong duration.
- [ ] Prescribed duration with violated limit: reported, not changed.
- [ ] `dotnet build` and `dotnet test` succeed; earlier tests pass or are updated with a reason stated in the note.
- [ ] Return note `coordination/inbox-docs/0005-shortest-feasible-duration.md` (with *Suggested commits*) and `coordination/STATUS.md` updated.

## Notes for the author (educational)

Explain in the note why the rest-to-rest case has a closed form while the general case does not, using one numeric example of each.
