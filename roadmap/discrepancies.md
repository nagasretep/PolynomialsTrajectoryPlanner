# Theory ↔ code discrepancy register

Owner: Cowork. Sources: Code's return notes `coordination/inbox-docs/0001-…` and `0002-…`, plus Cowork's own review. Last updated: 2026-10-17.

Classes: **code** = the code must change; **theory** = a theory/design document must change or be completed; **accepted** = difference is acceptable, documented; **decision** = the author must decide.

| ID | Where | Finding | Class | Resolution | Status |
|---|---|---|---|---|---|
| D1 | Decision 5 / `TrajectoryPassingThrough…` | Pass-through interior derivatives came from finite differences | code + theory gap | Formulation written in theory; exact solver implemented and verified (request 0002: reference values reproduced, 67/67 tests) | **closed** |
| D2 | Decision 5 architecture note | `ISmoothnessCriterion` only selected a branch | code | Replaced by `SmoothnessCriterion` (derivative order $m$) | **closed** |
| D3 | Decision 4 | Math.NET only used for dense linear algebra; root finding hand-written; no integration | design update | `design/07` A6 records the reality; Cholesky is now a second use | **closed** |
| D4 | `TrajectoryConstraints` | Extrema: scan + bisection **and** 201-point sampling (superset of the documented candidates) | accepted | Add an implementation note to `TrajectoryConstraints.md` (Cowork, low priority) | open |
| D5 | `TrajectoryConstraints` | `DurationSearcher` returns the initial guess if it already satisfies the limits (not the shortest feasible duration); silent failure after 50 doublings | decision + code | Text and code agree ("increase if violated"). **Author decides** whether auto duration means the shortest feasible duration. Closed form when the problem scales in time (rest-to-rest segment, or pass-through with fixed duration ratios and rest ends): the optimal trajectory for durations $\lambda T$ is the same shape in stretched time, so the peak of derivative order $k$ scales as $\lambda^{-k}$ and the shortest feasible scale is $\lambda^{\star} = \max_k \left( P_k / L_k \right)^{1/k}$, with $P_k$ the peak for $\lambda = 1$ and $L_k$ the limit. Not valid with non-zero boundary derivatives. Failure signalling is a plain bug (request 0003) | open |
| D6 | Decision 1 | Duration strategies unused; equal split called "proportional"; two mechanisms | code | One mechanism wired into both builders, proportional to $\lvert \Delta x \rvert$ (request 0002). Floor rule revised, see D14 | **closed** (floor: D14) |
| D7 | `BlendingSegments` | Blend degree hard-coded to 9; blend not verified against limits; blend position clamped to $\tau \in [0.001, 0.999]$ | code | Theory (Baseline steps 7 and 10) prescribes a **selectable blend degree**, sampling only the derivative orders the degree supports, and verification of the blend against active constraints with duration adjustment. The $\tau$ clamp is an implementation detail: accepted (a blend that starts exactly on an endpoint is degenerate). To be done in the consolidation (R7) | open |
| D8 | `ConstantVelocitySegment` | Entry/exit transition **durations** not specified by theory; App and Core use different rules | theory gap | Confirmed: baseline step 7 gives the target states of the transitions but no duration rule. Cowork writes the rule with the author's input (R8); consolidation follows | open |
| D9 | `SymbolsAndNomenclature` | App coefficient report labels `a0 … a9` instead of `c_i` | code | Request 0003 | open |
| D10 | Decision 3 | Absolute positions in the UI; Core also exposes $\tau$-based entry points | accepted | Both conventions fine | **closed** |
| D11 | `PrototypeTrajectoryService` | App re-implements duration choice, constant velocity, blend point search, limit validation, sampling | code | Consolidation (R7), after D8 | open |
| D12 | `Trajectory1D.SampleTimes` | $O(n^2)$ `IndexOf`, wrong for repeated segment objects, division by zero for 1 sample | code | Request 0003 | open |
| D13 | Core | `Symbols.Epsilon` unused | code | Request 0003 (delete unless a test needs it) | open |
| D14 | Decision 1 | Proportional-duration floor: absolute 0.05 s can make the sum exceed the requested total | design update + code | Rule recorded in `design/07` A8 (relative floor, total preserved); implementation in request 0003 | open |
| D15 | `AutoDerivedProportionalDuration.ComputeDuration` | Per-segment member is an unused equal split | code | Removed in the consolidation (R7) together with the interface cleanup | open |

## Decisions waiting for the author

1. **Degree 9: snap or crackle** as the default order for exact pass-through. Default until decided: snap ($m = 4$), as in Decision 5. Code's measurements on $x = (0,1,3,2)$, $T = (0.7, 1.3, 0.9)$:

   | | snap ($m=4$) | crackle ($m=5$) |
   |---|---|---|
   | snap cost $J_4$ | 22 094 | 35 840 |
   | max position (waypoints peak at 3) | 3.67 | 4.19 |
   | peak velocity | 4.47 | 5.70 |
   | peak jerk | 33.0 | 49.5 |
   | peak snap | 256 | 210 |
   | continuity at waypoints | up to snap; crackle jumps | up to the 8th derivative |

   Cowork's view: keep snap. On this example it overshoots less, has lower peak velocity and jerk, and the extra continuity of crackle would buy derivatives the project does not use. It is a requirement question, not a mathematical one, so the choice is yours.
2. **Auto duration with limits** (D5): shortest feasible duration, or keep "initial guess if valid".
