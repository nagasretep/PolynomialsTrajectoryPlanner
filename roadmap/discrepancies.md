# Theory ↔ code discrepancy register

Owner: Cowork. Sources: Code's return notes `coordination/inbox-docs/0001-…` and `0002-…`, plus Cowork's own review. Last updated: 2026-10-17.

Classes: **code** = the code must change; **theory** = a theory/design document must change or be completed; **accepted** = difference is acceptable, documented; **decision** = the author must decide.

| ID | Where | Finding | Class | Resolution | Status |
|---|---|---|---|---|---|
| D1 | Decision 5 / `TrajectoryPassingThrough…` | Pass-through interior derivatives came from finite differences | code + theory gap | Formulation written in theory; exact solver implemented and verified (request 0002: reference values reproduced, 67/67 tests) | **closed** |
| D2 | Decision 5 architecture note | `ISmoothnessCriterion` only selected a branch | code | Replaced by `SmoothnessCriterion` (derivative order $m$) | **closed** |
| D3 | Decision 4 | Math.NET only used for dense linear algebra; root finding hand-written; no integration | design update | `design/07` A6 records the reality; Cholesky is now a second use | **closed** |
| D4 | `TrajectoryConstraints` | Extrema: scan + bisection **and** 201-point sampling (superset of the documented candidates) | accepted | Add an implementation note to `TrajectoryConstraints.md` (Cowork, low priority) | open |
| D5 | `TrajectoryConstraints` | Auto duration with limits was not the shortest; silent failure | decision + code | Decided (shortest feasible duration); theory written; failure signalling (0003) and shortest-duration search with closed form (0005) done and verified (173/173). Two refinements remain: zero-length rest-to-rest segment returns 1 ms where the limits determine nothing (request 0006); constant-velocity transitions still use the legacy search (with D8) | **closed** (refinements: D17, D8) |
| D6 | Decision 1 | Duration strategies unused; equal split called "proportional"; two mechanisms | code | One mechanism wired into both builders (0002); floor rule A8 implemented (0003) | **closed** |
| D7 | `BlendingSegments` | Blend degree hard-coded to 9 (even between degree-3 neighbours); no limit verification; blend position clamped to $\tau \in [0.001, 0.999]$; out-of-range absolute position silently clamped; empty window accepted | code | Tests pin today's behaviour (`Today_…`, request 0004). Theory (Baseline steps 7 and 10) prescribes a **selectable blend degree** and limit verification with duration adjustment; the consolidation (R7) must also **reject** an absolute position outside the segment and an empty window. The $\tau$ clamp is accepted | open |
| D8 | `ConstantVelocitySegment` | Entry/exit transition **durations** not specified by theory; App and Core use different rules; constant velocity has one smoke test | theory gap + code | All three rules decided and written in the theory; Rules 2 and 3 to implement (request 0008) | open (code) |
| D9 | `SymbolsAndNomenclature` | App coefficient report labels `a0 … a9` | code | Labels now `c0 … cd` (0003; read in code, not seen in the running app). `a0`/`af` in the solvers are initial/final acceleration: correct, unchanged | **closed** |
| D10 | Decision 3 | Absolute positions in the UI; Core also exposes $\tau$-based entry points | accepted | Both conventions fine | **closed** |
| D11 | `PrototypeTrajectoryService` | App re-implements duration choice, constant velocity, blend point search, limit validation, sampling | code | Consolidation (R7), after D8 | open |
| D12 | `Trajectory1D.SampleTimes` | $O(n^2)$ `IndexOf`, repeated-segment bug, 1 sample | code | Fixed and tested (0003) | **closed** |
| D13 | Core | `Symbols.Epsilon` unused | code | Deleted (0003) | **closed** |
| D14 | Decision 1 | Absolute floor in proportional durations | design update + code | A8 rule implemented and tested (0003) | **closed** |
| D15 | `AutoDerivedProportionalDuration.ComputeDuration` | Per-segment member is an unused equal split | code | Removed in the consolidation (R7) together with the interface cleanup | open |
| D16 | `PassThroughProblem1D` | cond(H) up to $5 \cdot 10^{15}$ (d = 9, m = 5, 30 segments, duration ratio 100): near the limit of double precision; `Cost(y)` loses ~6 digits | code | Diagonal scaling implemented (cond 5e15 → 2e11 for d = 9, m = 5); `Cost(y)` documented as unreliable (0006) | **closed** |
| D17 | `ShortestFeasibleDuration` | Zero-length rest-to-rest segment returns 1 ms | code | `Find` returns null when the limits determine nothing; callers use a fallback (0006) | **closed** |

## Decisions taken by the author

1. **Degree 9 default order for exact pass-through: snap** ($m = 4$), 2026-10-17. Crackle ($m = 5$) remains a parameter.
2. **Auto duration with limits: the shortest feasible duration**, 2026-10-17 (D5).
3. **Constant-velocity transitions with limits active: each transition uses the shortest feasible duration** (same as ordinary segments), 2026-10-17 (D8).

4. **Constant-velocity transitions without limits: mean-velocity rule; prescribed total duration: split in proportion to the shortest feasible durations**, 2026-10-17 (D8).

No decision is pending.
