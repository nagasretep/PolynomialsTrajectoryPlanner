# 0008 — Constant-velocity duration rules 2 and 3 (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0008-constant-velocity-duration-rules.md`

## What was done

- **Rule 2** in `ConstantVelocityTrajectory1D`: without limits and without prescribed durations, `T = 2·|Δx| / (|v_b| + v_cv)` per transition (`Δx` = distance between the outer position and the stretch end; `v_b` = outer velocity, absolute value, so an outer velocity against the travel counts as its speed, as the theory says). If `Δx = 0`: the old fallback `max(0.5·T_cv, 0.1 s)`. `DefaultTransitionDuration` is replaced by the public, documented `MeanVelocityDuration(distance, outerVelocity, speed, constantVelocityDuration)`.
- **Rule 3**: new optional parameter `totalDuration` in both constructors. `R = total − T_cv` must be positive (`ArgumentException` whose message contains `T_cv`). Weights = shortest feasible durations if limits are active (`Find ?? Rule 2` value when `null`), Rule 2 durations otherwise; `T_in = R·s_in/(s_in+s_out)`, `T_out = R − T_in`. With limits and `R < s_in + s_out` → `KinematicLimitsViolatedException` stating the minimum feasible total (`T_cv + s_in + s_out`, with the three terms). The comparison has a relative tolerance of 10⁻¹² so that `R` computed as "total − T_cv" with `total = T_cv + s_in + s_out` is accepted.
- **Combinations**: total + one prescribed transition → the other takes the remainder (must be positive, else `ArgumentException`; then verified against the limits like any piece); total + both prescribed → `ArgumentException`; one prescribed without total → as before, the other by Rule 1/2.
- **App**: `PrototypeTrajectoryService` passes `totalDuration` straight to Core. Its own equal split (and the "remaining time" check) are gone; the message for a total not above the stretch is now Core's ("The total duration 3 must exceed the constant-velocity stretch duration T_cv = 4"), prefixed with "Segment n:". `ArgumentException` from Core is converted to the same `InvalidOperationException` as the other Core errors, only around this call.
- Rule 1 untouched.

## Behaviour changes for the user

- **No limits, no duration**: transitions are now `2Δx/(|v_b|+v_cv)` instead of `max(0.5·T_cv, 0.1)`. App example (x 0→100, stretch 20→80 at 15, rest ends): entry/exit 2.667 s each instead of 2.0 s, total 9.33 s instead of 8.0 s.
- **Prescribed total**: the transitions are no longer always equal. Example (stretch 10→60 at 15, entry covers 10 units, exit 40): the exit gets four times the entry (the Rule 2 ratio), instead of half each. With limits the ratio is the ratio of the shortest feasible durations.
- **Prescribed total too short for the limits**: before, the equal split typically ended in "the entry transition violates the limits" with no indication of what total would work; now the message gives the minimum feasible total (App, `a_max = 2`, total 8 s: "minimum feasible total is 91.995 s (stretch 4 s + entry 43.9975 s + exit 43.9975 s)").

## Tests (256/256 pass; 228 before)

- `ConstantVelocityTrajectoryTests`: the old `Today_…` test (no-limits rule) replaced by `Rule2_MeanVelocityFormula` (6 cases: rest ends, moving start, end velocity against travel, start velocity against travel, negative direction, other speed; 1e-12), `Rule2_ZeroDistanceTransition_*` (fallback and 0.1 s floor), `MeanVelocityDuration_IsPublicAndFollowsTheFormula`.
- New `ConstantVelocityTotalDurationTests` (21 cases): total exact (1e-12) and ratio = weight ratio without limits (weights = Rule 2) and with limits (degrees 9/7/5, slack 1.0001 … 3); `R = s_in + s_out` feasible with both transitions at their minimum; `R` 10⁻⁶ below → failure with the minimum total in the message; total ≤ `T_cv` rejected (4 values); combinations (prescribed entry, prescribed exit, remainder not positive, remainder checked against limits, both prescribed → conflict, single prescribed without total, full-state constructor); a case where the old equal split (passed to Core as prescribed durations) is rejected and the proportional split of the same total respects the limits (precondition asserted: `R/2 < max(s_in, s_out)`).
- `dotnet build`: 0 errors, no new warnings (the App compiled to a separate folder; the author's copy of the app was probably running). The App path was exercised with a throwaway console project (scratchpad), the WPF window was not run.

## For the author (educational)

**Why the proportional split respects the limits whenever any split can** (under the assumption below). With limits active each transition has a smallest admissible duration, `s_in` and `s_out`, and anything at least that long is admissible. Any split of the time left `R` into `T_in + T_out` needs `T_in ≥ s_in` and `T_out ≥ s_out`; adding these gives `R ≥ s_in + s_out`. So if `R` is smaller than the sum no split exists at all (and the message says what total is needed), and if it is at least the sum, taking `T_in = R·s_in/(s_in+s_out)` gives `T_in = s_in·R/(s_in+s_out) ≥ s_in` and likewise for the exit. The equal split gives each transition `R/2`, which is below the minimum of the more demanding one whenever that minimum exceeds `R/2`, even though a feasible split exists. For the user: the same total now yields unequal transitions (the one that must cover more distance or change speed more gets more time), and a total that cannot work is refused with the number that would work.

**Where the assumption "longer is never worse" breaks (reported as requested).** I scanned entry and exit transitions (degrees 3/5/7/9; limits a=6, v=16, v=20, v=15.5, j=20, s=100; rest start, stretch at 15, 20 units to cover) from `s` up to 200 × `s`. Only **degree 3 with the acceleration limit** violates it: for a_max = 6, rest start, 20 units, arriving at 15, the entry is feasible for T in [2.6235, 2.7635], infeasible between about 2.76 and 7.5 s (peak acceleration up to 25 % above the limit around 4 s), and feasible again for longer durations (a long creep followed by a final ramp). Mechanism: with fixed distance and fixed arrival speed, lengthening T first lowers the mean speed so the polynomial must ramp harder at the end, and only for much longer T does the whole profile become gentle. Consequences for Rule 3: a total just above the minimum works; a larger total can make the proportional split land in the infeasible middle range and be rejected (`Today_Degree3_FeasibleDurationsAreNotAnUpwardClosedSet…` pins this: slack 1.05 works, slack 2.0 is refused with "The entry transition (duration 5.25 s) violates the active kinematic limits") although other splits of the same total are feasible. The error is explicit, never a wrong trajectory, so the guarantee "no silently wrong result" holds; only the claim "the proportional split works whenever any split can" is false for degree 3 in such cases. The same non-monotony is what the theory warns about for `F(T)` in *Shortest Feasible Duration*. For degrees 5, 7, 9 I found no case in this scan (not a proof). A fix, if wanted, would be a feasible-set-aware split (search for `T_in` in the feasible set, not only the proportional one); I did not implement it (out of scope).

## Discrepancies between theory and code

None for Rules 1–3 as written. The caveat above is a theory-level observation: the sentence "a proportional split respects the limits whenever any split can" in *Rule 3* holds under monotone feasibility and should be qualified (degree 3, and in principle any case where `F(T)` is not monotone).

## Open questions

1. Qualify the Rule 3 sentence in the theory, or ask for the feasible-set-aware split?
2. In 1D the fallback for `Δx = 0` is reachable only when the outer position equals the stretch start/end; kept as specified.

## Suggested commits

1. `code: constant-velocity Rule 2 (mean-velocity duration) and Rule 3 (prescribed total, proportional split); App passes the total to Core` — `src/TrajectoryPlanner.Core/ConstantVelocity/ConstantVelocityTrajectory1D.cs`, `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs` (this file also holds the 0006/0007 changes if those are not committed yet)
2. `code: tests for constant-velocity Rules 2 and 3` — `src/TrajectoryPlanner.Core.Tests/ConstantVelocity/ConstantVelocityTrajectoryTests.cs`, `src/TrajectoryPlanner.Core.Tests/ConstantVelocity/ConstantVelocityTotalDurationTests.cs`
3. `coord: close request 0008` — `coordination/inbox-code/0008-constant-velocity-duration-rules.md`, `coordination/inbox-docs/0008-constant-velocity-duration-rules.md`, `coordination/STATUS.md`
