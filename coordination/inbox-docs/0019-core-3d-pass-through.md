# 0019 — Core: 3D pass-through with common durations and limits (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0019-core-3d-pass-through.md`

## Summary

Implemented as specified. Tests: Core.Tests 429 → **450** (+21), App.Tests **122** (unchanged), **572 total**, all passing. `dotnet build`: 0 errors, no warnings other than the known NU1701. No App change, no 1D behaviour change: the 1D tests (planner, builder, App service) pass **unchanged**.

## What was done

- **One algorithm for 1D and 3D**: the body of `PassThroughPlanner1D.Plan` (closed form, λ search, safety step, verification) moved into the internal `PassThrough/PassThroughEngine.cs`, which works on *components* that share their durations: 1D is one component, 3D three. For fixed durations each component is one call of the existing `PassThroughSolver1D` (reused, not copied). The closed form takes the maximum over components and active limits of (P_k,c / L_k)^(1/k) and remembers the maximiser; the search's F is the maximum over components; constant components impose nothing (peak 0). `PassThroughPlanner1D.Plan` is now argument checking + the reference rule + one engine call; its messages are identical (no axis in 1D).
- **`PassThrough/PassThroughPlanner3D.cs`**: `Plan(waypoints: Point3D[], degree, criterion, limits, prescribedDurations, totalDuration, referenceDurations, startBoundary: State3D?, endBoundary: State3D?)` → `PassThroughResult3D(Segments: PolynomialSegment3D[], Durations, ReferenceDurations (null when prescribed), Scale (null when not computed), ScaledByLimits, BindingQuantity, BindingAxis)` + `TotalDuration`. The three scalar solutions are combined into `PolynomialSegment3D`s through their node states.
  - prescribed durations: kept, verified; violation "Pass-through segment 1 (duration 0.5 s) violates the velocity limit on axis x: peak … exceeds 2.";
  - prescribed total: the A8 rule on Euclidean chords (`AutoDerivedProportionalDuration.ComputeDurations(Point3D[])`, already present), only verified;
  - nothing prescribed: the chord rule with one second per segment, scaled by λ with active limits (closed form for rest ends, search otherwise; failure → `DurationSearchFailedException`).
  - Binding pair: for rest ends the maximiser of the closed form; after a search the largest P/L at the final λ.
- **Zero-chord rule** (identical to 1D, it is the same `AutoDerivedProportionalDuration`): weights w_k = max(Δ_k, φ·mean(Δ)), φ = 0.05, so a zero chord (two equal consecutive waypoints) gets the floor share φ·mean instead of a zero duration; if **all** chords are zero the split is equal (1 s each by default). With all waypoints equal every axis is constant, every peak is zero and the limits determine nothing (`ScaledByLimits = false`, `Scale = null`). Tested: `AZeroChord_GetsTheFloorShareOfTheRule`, `AllWaypointsEqual_…`.
- **`TrajectoryBuilder3D.AddSegmentsFromWaypointsPassThrough`** now calls the planner and gained `limits`, `startBoundary`, `endBoundary` and `LastPassThroughResult` (as the 1D builder). **Default changed**: `totalDuration` was `double = 1.0` (a 1 s *total* split by the chords); it is now `double? = null`, and with nothing given the planner's default applies (**one second per segment**, D23, like the 1D builder since 0015). No existing test encoded the old default (the two 3D callers pass explicit durations); calls with `totalDuration:` keep working and mean "prescribed total".

## Acceptance criteria — where verified (`PassThrough/PassThroughPlanner3DTests.cs`, 21 cases)

| Criterion | Test and numbers |
|---|---|
| Reference example | `ReferenceExample_ReferenceDurationsAndPeaksAtLambdaOne`: reference durations (0.375, 1.125, 1.5); the 12 peaks P_k,a(1) of the table to 1e-5 relative. `ReferenceExample_WithLimits_…`: λ* = 4.990502, total 14.971506 (1e-6), durations (1.871438, 5.614315, 7.485753), binding velocity on x, peaks at λ* 2.0 (1e-9), 1.534124, 1.667833, 4.683214 (1e-5) |
| Along one axis = 1D (1e-9) | `AlongOneAxis_EqualsThe1DPlanner` ×4: without/with limits, with/without a start velocity 1.5 (search): durations, λ, all coefficients |
| Scaling law per axis | `ScalingLaw_PerAxis` (λ = 0.5, 2): 1e-8 relative, every axis and order |
| Continuity, degrees 3/5/7/9 | `EveryWaypointIsHit_…` ×4: waypoints hit, orders 0..p−1 continuous at every interior waypoint (1e-9), limits respected |
| Prescribed / total / all-equal / constant axis / zero chord | named tests above; `AConstantAxis_ImposesNothing` (z constant: zero peak, not binding) |
| Non-zero boundary velocity vector by search, verified by `LimitReport` | `ANonZeroBoundaryVelocityVector_…`: boundary velocities honoured (1e-9), no row violated, the top row at 100 % (1e-6) on the reported axis and quantity, 0.01 % shorter durations violate; `ABoundaryVelocityAboveTheLimit_FailsExplicitly` |
| Search exists once; 1D unchanged | `PassThroughEngine`; all 1D tests green without edits |
| Builder | `Builder_UsesThePlanner_…` (3 s default, λ case 14.971506, binding x, three parts) |

## For the author (educational)

- **Why the 3D optimum is three 1D optima tied only by the durations.** The cost being minimised is the integral of the squared m-th derivative of the vector, which is the sum of the three integrals of the components' squared m-th derivatives. The conditions (pass through every waypoint, end conditions) also speak about each component separately. A sum of three independent terms, each with its own constraints, is smallest when each term is smallest. The only thing the components share is *when* the waypoints are reached, i.e. the durations; once those are fixed, the problem falls apart into three 1D problems.
- **Why one λ scales the whole vector trajectory, and what it does to the path.** Stretching all durations by λ replaces every component x_a(t) by x_a(t/λ): each component runs through the same values more slowly, so the vector p(t/λ) visits the same points in the same order. The path — the curve in space — is unchanged; only the timetable changes, and every derivative of order k is divided by λ^k on every axis. That is why a single number can make every axis respect every limit: the most demanding (quantity, axis) pair decides λ, and all the others are then within their limits too.
- **Why the chord rule can give a large λ.** The chord rule gives each segment time in proportion to its length in space, not to what each axis does in it. In the reference example the first segment is short (length 1) but it contains the whole motion of x: x must accelerate and stop within the short reference time 0.375 s, so its velocity peak is large (≈ 10 at λ = 1) and only stretching *everything* by 5 brings it under 2. The other segments, already slow, are slowed down as well. A smarter proportion (giving the first segment more time) would need a nonlinear optimisation of the proportions, which is not part of the baseline.

## Discrepancies between theory and code

None. The reference values are reproduced to the stated precision (P₄ₓ = 2904.831335 to 1e-5 as well; the sampled/true-maximum difference seen in 0016 does not appear here because the polynomial peaks are exact in the code and the theory sampled 20001 points per segment).

## Open questions

1. The UI step (pass-through in the 3D mode) can use `TrajectoryBuilder3D.AddSegmentsFromWaypointsPassThrough` with `LastPassThroughResult` for λ and the binding pair, and `TrajectoryAnalyzer3D` for the rest of the report.
2. `referenceDurations` (custom proportions) exists in the 3D planner as in 1D; it is not exposed in the builder (also as in 1D).

## Suggested commits

1. `code: one pass-through engine for one or several components; PassThroughPlanner1D on top of it` — `src/TrajectoryPlanner.Core/PassThrough/PassThroughEngine.cs`, `src/TrajectoryPlanner.Core/PassThrough/PassThroughPlanner1D.cs`
2. `code: 3D pass-through with common durations and limits; builder uses it` — `src/TrajectoryPlanner.Core/PassThrough/PassThroughPlanner3D.cs`, `src/TrajectoryPlanner.Core/Services/TrajectoryBuilder3D.cs`, `src/TrajectoryPlanner.Core.Tests/PassThrough/PassThroughPlanner3DTests.cs`
3. `coord: close request 0019` — `coordination/inbox-code/0019-core-3d-pass-through.md`, `coordination/inbox-docs/0019-core-3d-pass-through.md`, `coordination/STATUS.md`

(0013–0018 may still be uncommitted. `roadmap/`, `theory/`, `design/` untouched.)
