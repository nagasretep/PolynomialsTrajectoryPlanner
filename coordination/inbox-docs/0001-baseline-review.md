# 0001 — Baseline review of the code (return note)

Status: closed (reviewed by Cowork 2026-10-17) · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0001-baseline-review.md`

## What was done

Read-only review of `src/` (Core, App, Core.Tests) against the request's seven items. **No source file, package version or project file was changed.** Files written: this note, and the status updates in `coordination/STATUS.md` and the request header.

Verified on 2026-10-17: `dotnet build PolynomialsTrajectoryPlanner.slnx` → 0 errors, 3 warnings (all the same NU1701, reported once per build pass); `dotnet test src\TrajectoryPlanner.Core.Tests` → 24/24 passed. I did not launch the WPF app, so runtime behaviour of the Skia assembly (item 1) is not verified by me.

---

## 1. Warning NU1701 (`SkiaSharp.Views.WPF 3.119.0`)

- **Origin**: transitive. `ScottPlot.WPF 5.1.59` → `SkiaSharp.Views.WPF 3.119.0` (also pulls `OpenTK 4.9.4`, `OpenTK.GLWpfControl 4.3.3`). Confirmed in `project.assets.json` and `dotnet list package --include-transitive`. Nothing in `src/` references SkiaSharp directly.
- **Cause**: that SkiaSharp package ships only `net462`+ (.NET Framework) assemblies, so NuGet restores them with the "may not be fully compatible" fallback to net8.0-windows.
- **Assessment**: most likely harmless — WPF assemblies built for .NET Framework generally load on modern .NET, and this is the dependency chain ScottPlot.WPF itself ships with. Caveat: not verified at runtime by me. The author's report that the prototype runs and plots is the practical evidence.
- **Action taken**: none (to silence it I would have to suppress NU1701 for the whole App project, which also hides any future genuine incompatibility; I judged that not "trivial and neutral").
- **Options**: (a) leave as is; (b) `<NoWarn>$(NoWarn);NU1701</NoWarn>` in the App csproj; (c) upgrade `ScottPlot.WPF` when a release depends on a net8-targeted SkiaSharp view package. I suggest (a) now, (c) opportunistically.

## 2. Core inventory

| Folder / namespace | Public types | Purpose | Theory document |
|---|---|---|---|
| `Symbols` | `DerivativeOrder` (enum), `TrajectoryConstants` | Derivative-order enum; supported degrees {3,5,7,9}, epsilon | `SymbolsAndNomenclature` |
| `Points` | `Point1D`, `Point3D` | Position values (3D with distance/arithmetic) | `SymbolsAndNomenclature`, `BoundaryConditions` |
| `States` | `State1D`, `State3D` | Kinematic state (position … snap) at an instant | `BoundaryConditions` |
| `BoundaryConditions` | `BoundaryCondition1D`, `BoundaryCondition3D`, `BoundaryEndpoint`, `BoundaryConditionValidator` | Typed boundary conditions; checks count/orders per degree | `BoundaryConditions` |
| `Polynomials` | `PolynomialCoefficients`, `PolynomialEvaluator`, `NormalizedTimeMatrixFactory` | Normalized-time coefficients, Horner/canonical evaluation, derivative coefficients, hard-coded matrices for degrees 3/5/7/9 | `NinthDegreePolynomialTheory`, `PolynomialCoefficientDetermination` |
| `CoefficientDetermination` | `PolynomialCoefficientSolver` | Solves the (degree+1)² system for coefficients (Math.NET dense solve) | `PolynomialCoefficientDetermination` |
| `Segments` | `PolynomialSegment1D`, `PolynomialSegment3D` | One polynomial segment: states + duration → coefficients, evaluation | `NinthDegreePolynomialTheory`, `PolynomialCoefficientDetermination` |
| `Trajectories` | `Trajectory1D`, `Trajectory3D` | Ordered segments, global-time evaluation | (multi-segment parts of `TrajectoryPassingThrough…`, `BlendingSegments`) |
| `Constraints` | `KinematicLimits`, `ExtremaResult`, `ExtremaAnalyzer`, `DurationSearcher` | Extrema per derivative order, limit verification, duration recompute-and-verify | `TrajectoryConstraints` |
| `Blending` | `BlendBuilder1D` | Blend segment between two segments; truncation of neighbours | `BlendingSegments` |
| `ConstantVelocity` | `ConstantVelocityTrajectory1D`, `LinearConstantVelocitySegment` | Entry transition + constant-speed stretch + exit transition (1D) | `ConstantVelocitySegment` |
| `PassThrough` | `ISmoothnessCriterion`, `MinimumSnapCriterion`, `MinimumJerkCriterion`, `PassThroughSolver1D`, `DurationStrategies` (static) | Exact pass-through of waypoints (see item 4) | `TrajectoryPassingThroughConstraintPoints` |
| `Services` | `TrajectoryBuilder1D`, `TrajectoryBuilder3D`, `IDurationStrategy1D`, `DurationMode`, `UserSpecifiedDuration`, `AutoDerivedProportionalDuration`, `ConstraintAwareDuration` | Fluent builders; duration strategies | `design/07` Decision 1 and A-appendix (no single theory doc) |

Folders without a clear theory counterpart or with oddities:
- `Services`: orchestration only, no theory (by design, `design/07`). But `IDurationStrategy1D` and its three implementations are **not used by any builder or by the App** (dead code today). `AutoDerivedProportionalDuration` returns `total / segments` — equal split, not "proportional to distance" as Decision 1 says for pass-through (the correct proportional rule lives in `PassThrough/DurationStrategies`, a second, unrelated duration mechanism).
- `PassThrough/DurationStrategies` lives in the PassThrough folder but is a duration helper; arguably belongs with `Services`.
- `Symbols.Epsilon` is defined but, as far as I saw, unused (code uses literal tolerances: 1e-10, 1e-12, 1e-8, 1e-6).

## 3. Test coverage by area (24 tests)

| Core folder | Tests that exercise it | Judgement |
|---|---|---|
| `Symbols` | 2 (`DerivativeOrder_HasCorrectIntegerValues`, `SupportedDegrees_AreCorrect`) | smoke |
| `Points` | 2 (`Point1D_/Point3D_BasicOperations`) | smoke |
| `States` | 1 (`State1D_DefaultZero`); `State3D` only indirectly | smoke |
| `BoundaryConditions` | 2 (validator: required count, derivative levels). No test of `ValidateConditions1D/3D` failure paths, nor of `BoundaryCondition1D/3D` | thin |
| `Polynomials` | 3 (Horner = canonical, first-derivative coeffs, derivative beyond degree) | real, small |
| `CoefficientDetermination` | 4 (degree 3, 5, 9 endpoint conditions; degree-3 symmetry). **Degree 7 has no test** | real |
| `Segments` | 2 (degree-9 rest-to-rest, degree-3 ramp). `PolynomialSegment3D` only via builder test | thin |
| `Trajectories` | indirect only, through builder tests. `FindSegment` boundary handling untested | none direct |
| `Constraints` | 3 (one extremum case on degree 3; `VerifyLimits` happy path; `DurationSearcher` degree 3). No degree ≥ 5 extrema, no violated-limit case, no `ConstraintAwareDuration` | smoke |
| `Blending` | **0** | none |
| `ConstantVelocity` | 1 (speed and acceleration at the middle of the stretch). Entry/exit transitions, limits branch untested | smoke |
| `PassThrough` | 1 (positions at waypoints, zero end velocity, 3 waypoints) + 1 builder test. **Nothing checks interior derivatives or optimality** | smoke |
| `Services` | 3 (1D multi-segment, 1D pass-through coverage, 3D rest-to-rest). `IDurationStrategy1D` implementations untested | smoke |

Areas with none: Blending, `Trajectories` (direct), duration strategies, degree-7 coefficients, 3D pass-through. The App layer has no tests (`TrajectoryPlanner.Core.Tests` tests Core only, per CLAUDE.md), so everything in `PrototypeTrajectoryService` is untested.

## 4. `PassThroughSolver1D` today vs minimum snap

**Inputs**: waypoint positions `x_0 … x_N`, one duration per segment, polynomial degree. `SolveWithAutoDurations` splits a total time proportionally to |Δx| (min 0.05 s). The builders call `Solve` with `new MinimumSnapCriterion()`.

**What it computes**:
1. Start and end states of the whole trajectory: v = a = j = s = 0 (matches the theory).
2. Interior waypoint states are **not obtained by optimisation**. The `ISmoothnessCriterion` object is used only to choose a branch via `DerivativeWeight`; its `ComputeCost` (a 100-sample trapezoid of ∫(x⁽⁴⁾)² or ∫(x⁽³⁾)²) is **never called** by the solver. Both branches end in `SolveFiniteDifferenceHeuristic`.
3. Heuristic, in sequence, for each interior node *i* (times `t_i` cumulated from durations):
   - `v_i = (x_{i+1} − x_{i−1}) / (t_{i+1} − t_{i−1})` (central difference over positions);
   - `a_i = (v_{i+1} − v_{i−1}) / (t_{i+1} − t_{i−1})`, `j_i` likewise from `a`, `s_i` likewise from `j` — each derivative is a central difference of the previous one, with the end values (zero) included in the stencil.
4. **Minimum-snap branch quirk** (the one the builders use): `SolveMinimumSnapInterior` passes throw-away arrays for acceleration and jerk, so the real `a_i` and `j_i` stay **0** at all interior nodes, while `s_i` is computed from the discarded jerk chain and is generally non-zero (for three waypoints it is also 0). The result is an inconsistent state: v from finite differences, a = j = 0, s ≠ 0 — not what any optimality condition would give. The minimum-jerk branch fills all of v, a, j, s from the finite-difference chain.
5. Each segment is then a normal degree-d polynomial between consecutive node states (`PolynomialSegment1D`), so position and the derivatives that the degree supports are continuous at the nodes by construction (shared states). Degree 3/5/7 simply ignore the higher derivatives.

**Heuristic vs not**: the exact-passage and the continuity-by-shared-state parts are exact and correct. *Everything* about the interior v, a, j, s is a heuristic. Note that it does not even guarantee anything about smoothness: the trajectory can overshoot between waypoints.

**Difference from minimum snap** (`TrajectoryPassingThroughConstraintPoints`, Decision 5): minimum snap treats the interior derivatives (or equivalently the 10 coefficients per segment) as unknowns and minimises J = Σ_k ∫ (x_k⁽⁴⁾(t))² dt subject to: waypoint positions, zero v/a/j/s at the global ends, and continuity of v, a, j, s (for degree 9, up to C⁴/snap per the doc) at interior nodes. Because J is quadratic and the constraints are linear, the problem is a single linear system (equality-constrained QP / KKT) for all segments at once — "global" because moving one waypoint's velocity changes the neighbouring segments. In practical terms: minimum snap gives the *smoothest possible* path through the waypoints in the snap-energy sense; the current solver gives *a* smooth-looking path through them, whose quality depends on the waypoint spacing. The gap is the whole M7 milestone: build the segment-wise Q (snap-cost) matrix, the continuity/constraint matrix, and solve — probably another use for Math.NET dense solves.

## 5. Where logic lives

`PrototypeTrajectoryService` (App) does **not use** `TrajectoryBuilder1D`, `ConstantVelocityTrajectory1D`, `PassThroughSolver1D` or the duration strategies at all. It re-implements the following, each of which already exists or could exist in Core:

| Step in the App | Core counterpart / could move to Core |
|---|---|
| Choosing segment duration: user value / `DurationSearcher` / default 1.0 | `IDurationStrategy1D` (`UserSpecified`, `ConstraintAware`) — exists, unused |
| Building start/end `State1D` from UI values with per-degree masking (`SupportsAcceleration` etc.) | `State1D`/validator — a "state for degree" factory could live in Core |
| Constant-velocity entry/stretch/exit construction (own duration rules, splits remaining time in two equal halves when the user fixes total duration) | `ConstantVelocityTrajectory1D` — exists but has a different rule set (fixed 0.5·T_cv or limits-based), so **two diverging implementations** |
| Blend: root-finding of τ for a given absolute position (`FindTauForPosition`, an exact copy of the private method in `BlendBuilder1D`), then `BuildBlendedSegmentPair` with blend degree hard-coded to 9 | `BlendBuilder1D.BuildBlendSegment(…, Point1D, Point1D, …)` already accepts absolute positions; `TrajectoryBuilder1D.AddBlendBetween` takes τ values only |
| Validating a fixed-duration segment against limits (`ValidateSegmentAgainstLimits`) | belongs in `Constraints` (builder option "validate") |
| Concatenating segments, evaluation, sampling at `max(300, 120·n)` points, per-derivative series | `Trajectory1D` + a Core sampling helper (`SampleTimes` exists but is per-segment) |
| Extrema / coefficient report rows | Core could return a report record; the UI formatting (`ExtremaReportRow`, `CoefficientReportRow`) can stay |
| Pass-through | Not exposed in the App at all (the UI is blending-only, per STATUS) |

Calculation-related code in the UI layer, concretely: `FindTauForPosition` (numerical Newton iteration), the constant-velocity timing rules, blend orchestration, limit validation, and sampling. `MainWindow.xaml.cs` itself only binds and plots (no math beyond clamping the cursor time). Text → number parsing (`ParseRequiredDouble`) is legitimately UI-side.

## 6. Theory / code discrepancies (listed, not resolved)

1. **`design/07` Decision 5 / `TrajectoryPassingThrough…` §"Recommended 1D Baseline"**: minimum snap with interior derivatives as global unknowns ↔ code uses finite differences (and the minimum-snap branch gives a = j = 0, s ≠ 0). See item 4.
2. **`design/07` Decision 5 architecture note**: `ISmoothnessCriterion` is meant to drive the solve ↔ it is a branch selector plus an unused cost function.
3. **`design/07` Decision 4**: Math.NET for root-finding (e.g. Brent) and numerical integration ↔ see item 7.
4. **`TrajectoryConstraints`** (stationary points = roots of the next-derivative polynomial): code finds roots by scanning 100 points for sign changes + bisection **and** additionally samples 201 points of τ ∈ [0,1], taking the min/max over all candidates. Result is a superset of the theory's candidate set; a double root without sign change would be caught only by the sampling. Not wrong, but not the documented algorithm.
5. **`TrajectoryConstraints`** (recompute-and-verify): `DurationSearcher` returns the *initial guess* if it already satisfies the limits (no search for the shortest valid duration); only when the guess fails does it double until valid and then bisect in [T/2, T]. If 50 doublings fail it returns the last duration **without** signalling failure. The App starts from 1.0 s, so with limits and no fixed duration, "auto" can return 1.0 s even if much shorter is feasible.
6. **`design/07` Decision 1** (pass-through default: proportional to waypoint spacing; blending: inherited from removed portions): `AutoDerivedProportionalDuration` splits equally (not by spacing); the blend default (`T = remaining preceding + consumed following`) matches the "inherited" rule only partly (it's the sum of the removed portions in *normalised-time × duration*, which is consistent, but the App cannot override it and never verifies the blend against limits, whereas `BlendingSegments` §"limits" says to verify and adjust).
7. **`BlendingSegments`**: blend degree is hard-coded to 9 in the App (and the default in Core), regardless of the neighbouring segments' degrees; blend positions are clamped to τ ∈ [0.001, 0.999] (initial [0.01, 0.99]), so a blend cannot start at or end exactly on a segment endpoint.
8. **`ConstantVelocitySegment`**: the constant stretch is a degree-3 polynomial with equal end velocities (a straight line). The theory says an exact CV subinterval cannot live in a single degree-9 polynomial, consistent with this; but the entry/exit durations are policy choices not specified in a theory document (see item 5: App and Core differ). I have not checked the document line by line for these rules — **Cowork, please confirm**.
9. **`SymbolsAndNomenclature`** (coefficients are `c_i`): the App's coefficient report prints rows named `a0 … a9` (`PrototypeTrajectoryService`, `CoefficientReportRow(label, $"a{i}", …)`) — the legacy `a_i` naming the CLAUDE.md rule forbids. Core's `PolynomialCoefficients` uses neutral names (`NormalizedCoefficients`, indexer), so the issue is App-only. Not changed here (no behavioural change allowed in this request); trivial to fix in a later request.
10. **Decision 3 (absolute vs percentage)**: matches in the App (absolute positions); Core API for blending also exposes τ-based entry points, which is not a contradiction but means both conventions exist.

## 7. Math.NET Numerics (Decision 4)

Partially used, and **not** for what Decision 4 describes:
- Used: `MathNet.Numerics.LinearAlgebra` dense LU solve in `PolynomialCoefficientSolver.SolveLinearSystem` (the (d+1)×(d+1) boundary-condition system). That is the only functional use.
- `PassThroughSolver1D.cs` has a `using MathNet.Numerics.LinearAlgebra;` that is never used.
- **Root finding**: hand-written — 100-point scan for sign changes + 100-iteration bisection (`ExtremaAnalyzer.FindStationaryPoints/TryBisection`) and a hand-written Newton iteration in `FindTauForPosition` (`BlendBuilder1D` and the App copy). No Brent, no `MathNet.Numerics.RootFinding`.
- **Integration**: none anywhere. The only integral-like code is the trapezoidal snap/jerk cost in `ISmoothnessCriterion` implementations (uncalled). There is no arc-length integration because 3D constant velocity (the case Decision 4 reserves it for) is not implemented.

## Test results

```
dotnet build PolynomialsTrajectoryPlanner.slnx   → 0 errors, 3 warnings (NU1701 ×3, same package)
dotnet test src\TrajectoryPlanner.Core.Tests     → Passed! Failed: 0, Passed: 24, Skipped: 0, Total: 24
```

## Open questions

1. Should the dead code (`IDurationStrategy1D` family, `ISmoothnessCriterion.ComputeCost`, unused `Epsilon`) be wired in or removed? I'd wire the duration strategies into the builders as part of the pass-through request.
2. Next request suggestion: exact minimum-snap solver (M7) **before** moving App logic into Core, since the App does not use pass-through today; and a test request for Blending / degree 7 / `Trajectories`.
3. Decide whether `DurationSearcher` should search for the *shortest* feasible duration (item 6.5) — this changes behaviour, so I did not touch it.
4. Minor robustness issues seen while reading, not in scope: `Trajectory1D.SampleTimes` uses `IndexOf(seg)` (O(n²), wrong if the same segment object appears twice) and divides by zero when `samplesPerSegment = 1`.
