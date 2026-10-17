# Theory ↔ code discrepancy register

Owner: Cowork. Source: Code's return note `coordination/inbox-docs/0001-baseline-review.md` (item 6) plus Cowork's own review. Last updated: 2026-10-17.

Classes: **code** = the code must change; **theory** = a theory/design document must change or be completed; **accepted** = difference is acceptable, documented; **decision** = the author must decide.

| ID | Where | Finding | Class | Resolution | Status |
|---|---|---|---|---|---|
| D1 | Decision 5 / `TrajectoryPassingThrough…` | Pass-through interior derivatives come from finite differences; the minimum-snap branch even leaves $a = j = 0$ | code (+ theory gap) | Theory gap closed: explicit formulation added (2026-10-17). Code: request 0002 | in progress |
| D2 | Decision 5 architecture note | `ISmoothnessCriterion` only selects a branch; `ComputeCost` never called | code | Criterion = derivative order $m$ (design/07 A7); request 0002 | in progress |
| D3 | Decision 4 | Math.NET only used for linear solve; root finding hand-written; no integration | design update | design/07 A6 records reality; no code change | closed |
| D4 | `TrajectoryConstraints` | Extrema: scan + bisection **and** 201-point sampling (superset of the documented candidates) | accepted | Add an implementation note to `TrajectoryConstraints.md` (Cowork, low priority) | open |
| D5 | `TrajectoryConstraints` | `DurationSearcher` returns the initial guess if it already satisfies the limits (not the shortest feasible duration); silent failure after 50 doublings | decision + code | The document literally describes "increase if violated", so the code matches the text. Decision for the author: should "auto duration with limits" mean the **shortest feasible duration**? Note: for a rest-to-rest segment (all boundary derivatives zero) the peak of derivative order $k$ scales as $T^{-k}$, so the shortest duration is closed-form: $T = \max_k \left( \Delta x\, P_k / L_k \right)^{1/k}$ with $P_k$ the peak for $T = 1$, unit distance. Failure signalling is a plain bug (backlog 0004) | open |
| D6 | Decision 1 | `AutoDerivedProportionalDuration` splits equally instead of proportionally to $\lvert \Delta x \rvert$; two unrelated duration mechanisms; strategies unused | code | Single duration mechanism wired into the builders; request 0002 | in progress |
| D7 | `BlendingSegments` | Blend degree hard-coded to 9; blend position clamped to $\tau \in [0.001, 0.999]$; no limit verification of the blend | theory check | Cowork to check `BlendingSegments.md` for what it prescribes (R3) and then raise a request or amend the text | open |
| D8 | `ConstantVelocitySegment` | Entry/exit transition **durations** are not specified by the theory; App and Core use different rules | theory gap | Confirmed: the document gives the target states of the transitions (baseline step 7) but no duration rule. Cowork to write the rule after the author's input; consolidation of the two implementations follows | open |
| D9 | `SymbolsAndNomenclature` | App coefficient report uses labels `a0 … a9` instead of `c_i` | code | Backlog 0004 | open |
| D10 | Decision 3 | Absolute positions in the UI; Core also exposes $\tau$-based entry points | accepted | Both conventions are fine; Core $\tau$ entry points are internal | closed |
| D11 | `PrototypeTrajectoryService` | App re-implements duration choice, constant velocity, blend point search (copy of a private Core method), limit validation, sampling | code | Consolidation request (roadmap R7), after the solver | open |
| D12 | `Trajectory1D.SampleTimes` | $O(n^2)$ `IndexOf`, wrong for repeated segment objects, division by zero for 1 sample | code | Backlog 0004 | open |
| D13 | Core | `Symbols.Epsilon` unused (literal tolerances used instead) | code | Backlog 0004: use it or remove it | open |

## Decisions waiting for the author

1. **Degree 9: snap or crackle** as the default smoothness order for exact pass-through (D1). Default until decided: snap ($m = 4$), as in Decision 5. The code takes $m$ as a parameter, so the answer does not block request 0002.
2. **Auto duration with limits** (D5): shortest feasible duration, or keep "initial guess if valid".
