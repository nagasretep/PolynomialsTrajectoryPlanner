# Theory ↔ code discrepancy register

Owner: Cowork. Sources: Code's return notes `coordination/inbox-docs/0001-…` and `0002-…`, plus Cowork's own review. Last updated: 2026-10-17.

Classes: **code** = the code must change; **theory** = a theory/design document must change or be completed; **accepted** = difference is acceptable, documented; **decision** = the author must decide.

| ID | Where | Finding | Class | Resolution | Status |
|---|---|---|---|---|---|
| D1 | Decision 5 / `TrajectoryPassingThrough…` | Pass-through interior derivatives came from finite differences | code + theory gap | Formulation written in theory; exact solver implemented and verified (request 0002: reference values reproduced, 67/67 tests) | **closed** |
| D2 | Decision 5 architecture note | `ISmoothnessCriterion` only selected a branch | code | Replaced by `SmoothnessCriterion` (derivative order $m$) | **closed** |
| D3 | Decision 4 | Math.NET only used for dense linear algebra; root finding hand-written; no integration | design update | `design/07` A6 records the reality; Cholesky is now a second use | **closed** |
| D4 | `TrajectoryConstraints` | Extrema: scan + bisection **and** 201-point sampling (superset of the documented candidates) | accepted | Implementation note added to `TrajectoryConstraints.md` | **closed** |
| D5 | `TrajectoryConstraints` | Auto duration with limits was not the shortest; silent failure | decision + code | Decided (shortest feasible duration); theory written; failure signalling (0003) and shortest-duration search with closed form (0005) done and verified (173/173). Two refinements remain: zero-length rest-to-rest segment returns 1 ms where the limits determine nothing (request 0006); constant-velocity transitions still use the legacy search (with D8) | **closed** (refinements: D17, D8) |
| D6 | Decision 1 | Duration strategies unused; equal split called "proportional"; two mechanisms | code | One mechanism wired into both builders (0002); floor rule A8 implemented (0003) | **closed** |
| D7 | `BlendingSegments` | Blend degree hard-coded to 9 (even between degree-3 neighbours); no limit verification; blend position clamped to $\tau \in [0.001, 0.999]$; out-of-range absolute position silently clamped; empty window accepted | code | Selectable degree, limit verification, input validation, duration rule: all implemented (0009, 0010) | **closed** |
| D8 | `ConstantVelocitySegment` | Entry/exit transition **durations** not specified by theory; App and Core use different rules; constant velocity has one smoke test | theory gap + code | All rules decided and implemented (0007, 0008); 256/256 tests | **closed** |
| D9 | `SymbolsAndNomenclature` | App coefficient report labels `a0 … a9` | code | Labels now `c0 … cd` (0003; read in code, not seen in the running app). `a0`/`af` in the solvers are initial/final acceleration: correct, unchanged | **closed** |
| D10 | Decision 3 | Absolute positions in the UI; Core also exposes $\tau$-based entry points | accepted | Both conventions fine | **closed** |
| D11 | `PrototypeTrajectoryService` | App re-implements duration choice, constant velocity, blend point search, limit validation, sampling | code | App is a thin layer over the Core (0009); service tests in `App.Tests` | **closed** |
| D12 | `Trajectory1D.SampleTimes` | $O(n^2)$ `IndexOf`, repeated-segment bug, 1 sample | code | Fixed and tested (0003) | **closed** |
| D13 | Core | `Symbols.Epsilon` unused | code | Deleted (0003) | **closed** |
| D14 | Decision 1 | Absolute floor in proportional durations | design update + code | A8 rule implemented and tested (0003) | **closed** |
| D15 | `AutoDerivedProportionalDuration.ComputeDuration` | Per-segment member is an unused equal split | code | Removed (0009) | **closed** |
| D16 | `PassThroughProblem1D` | cond(H) up to $5 \cdot 10^{15}$ (d = 9, m = 5, 30 segments, duration ratio 100): near the limit of double precision; `Cost(y)` loses ~6 digits | code | Diagonal scaling implemented (cond 5e15 → 2e11 for d = 9, m = 5); `Cost(y)` documented as unreliable (0006) | **closed** |
| D17 | `ShortestFeasibleDuration` | Zero-length rest-to-rest segment returns 1 ms | code | `Find` returns null when the limits determine nothing; callers use a fallback (0006) | **closed** |
| D18 | `ShortestFeasibleDuration`, constant-velocity Rule 3 | For degree 3 with an acceleration limit (rest start, non-zero arrival velocity) feasibility is not monotone in the duration: the search can return a later feasible duration (window about 5 % wide, like the step) and the proportional split can be refused although another split is feasible | theory + code | Theory qualified (2026-10-17); behaviour accepted: results are always feasible, failures explicit. Possible refinement: finer scan near the lower bound; among feasible splits take the one closest to proportional | accepted (low priority) |
| D19 | `BlendBuilder1D` | With limits active the blend is always the shortest feasible; the author decided: inherited duration if feasible, shortest feasible only on violation | code | Implemented and tested (0010): inherited duration kept if feasible, shortest feasible otherwise | **closed** |
| D20 | `PrototypeTrajectoryService` | `double.TryParse` without culture: on Italian regional settings a decimal point may be read as a thousands separator | code | `InputNumber`: current culture then invariant, no thousands separators; tested with it-IT and en-US (0010) | **closed** |
| D21 | App output | XAML tables always show a decimal point (WPF default culture en-US) while the status bar and cursor follow the current culture (comma on an Italian PC) | code | All displayed numbers follow the current culture (`UiCulture`, WPF `Language`); tested under it-IT (0012). To check by the author on the Italian PC: tables, status bar, cursor, ScottPlot axis labels | **closed** |
| D22 | `BlendBuilder1D`, report | When the inherited blend duration violates the limits, the blend duration and the trajectory total change without telling the user | code | `BlendSegment1D` exposes inherited and chosen duration and `AdjustedByLimits`; the service reports a notice in the status bar (0012) | **closed** |
| D23 | `PassThroughPlanner1D.Plan` | The default reference durations (A8 with total 1 s) differ from the App rule (A8 with total = number of segments × 1 s); without limits the total matters | code | Uniform default in the Core (request 0015, part C) | closed (request 0015, 2026-10-18) |
| D24 | App Summary | Peaks table lists only the quantities with an active limit | code | Also list the unlimited quantities (request 0015, part C) | closed (request 0015, 2026-10-18) |
| D25 | `ConstantVelocitySegment` (3D) | Curved multi-axis constant-speed stretch: the transitions to the non-polynomial arc-length segment are not specified ("dedicated blending/reparameterization design choices in later documents") | theory gap | Cowork to write the theory; Core raises `NotSupportedException` meanwhile (request 0015) | closed (theory 2026-10-18, Core request 0016) |

## Decisions taken by the author

1. **Degree 9 default order for exact pass-through: snap** ($m = 4$), 2026-10-17. Crackle ($m = 5$) remains a parameter.
2. **Auto duration with limits: the shortest feasible duration**, 2026-10-17 (D5).
3. **Constant-velocity transitions with limits active: each transition uses the shortest feasible duration** (same as ordinary segments), 2026-10-17 (D8).

4. **Constant-velocity transitions without limits: mean-velocity rule; prescribed total duration: split in proportion to the shortest feasible durations**, 2026-10-17 (D8).

5. **Blend duration with limits: inherited if feasible, otherwise the shortest feasible**, 2026-10-17 (D19).

Conventions adopted by Cowork on Code's proposals (revisable): first crossing for an overshooting segment, blend limits = most restrictive of the neighbours, default blend degree = neighbours' degree if equal else 9, unused public members kept until R9.

No decision is pending.

To verify by the author when running the App on the Italian PC (0012): decimal comma in the tables as in the status bar and cursor; the axis labels of the ScottPlot charts (not touched, culture unverified).
| D26 | `ConstantVelocitySegment.md` reference values | P4x reference 18.187075 was a sampled maximum; true maximum 18.187110 | theory | Theory corrected 2026-10-18 | closed |
| D27 | theory library (all documents) | One letter with two meanings (λ fraction vs λ duration scale; σ direction sign vs parametric speed; φ profile vs orientation angle); Open Questions already settled by the implementation or by 2026-10-17/18 decisions; stale paragraphs | theory | Renamed (fractions η, parametric speed ν, profile g), table of later symbols added to `SymbolsAndNomenclature`, Open Questions marked resolved, stale paragraphs rewritten (R13, 2026-10-18) | closed |
