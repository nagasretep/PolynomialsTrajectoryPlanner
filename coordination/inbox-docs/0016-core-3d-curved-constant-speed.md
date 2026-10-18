# 0016 — Core 3D: constant speed along a curved subpath (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-18 · Request: `coordination/inbox-code/0016-core-3d-curved-constant-speed.md`

## Summary

Implemented as specified. Tests: Core.Tests 372 → **400** (+28), App.Tests **101** (unchanged), **501 total**, all passing. `dotnet build`: 0 errors, no warnings other than the known NU1701. No App/UI change, no 1D change.

## What was done

**New folder `Core/ArcLength/`**
- `TaylorSeries`: truncated-series arithmetic on `double[]` (multiply, derivative, square root, reciprocal).
- `ArcLengthDerivatives.Compute(segment, t0, maxOrder = 4)` → `(σ, D¹p … D⁴p)`: expands each component to order `maxOrder` around `t0` from the segment's derivatives, forms the series of σ², σ and 1/σ once, then applies D = (differentiate, multiply by 1/σ) `maxOrder` times. Works for every degree (a degree-5 curve has non-zero p_sss); the transitions then use only the orders their degree needs. σ(t0) below 1e-12 → `PathNotRegularException` (an `ArgumentException`).
- `ArcLengthPath(segment, t_s, t_e, requireRegular = true)`: `Length`, `LengthAt(t)` (composite 16-point Gauss–Legendre, nodes computed at start-up by Newton on the Legendre polynomial; panels doubled until two values agree to a relative 1e-13), `TimeAtLength(ℓ)` (safeguarded Newton with bisection fallback, |ℓ(t*) − ℓ*| ≤ 1e-13 L; ℓ ≤ 0 → t_s and ℓ ≥ L → t_e exactly). Regularity: σ on a 400-interval grid plus golden-section refinement of the smallest sample; `PathNotRegularException` if σ_min ≤ 1e-9 σ_max on the subpath (a constant `RegularityTolerance`; a rest point has σ = 0 exactly, a near-rest start with σ ~ 1e-6 σ_max passes). `requireRegular: false` exists only to measure a whole segment whose ends are rest points (for arc-length fractions).

**New `Core/ConstantVelocity/ConstantSpeedTrajectory3D.cs`**
- `CurvedConstantSpeedStretch3D(ArcLengthPath, speed)`: `Duration = L/v`, `StartState`/`EndState` = `(p, v D p, v² D²p, v³ D³p, v⁴ D⁴p)`, `EvaluateState/Derivative/Position(t)` via the inversion (`d^k p/dt^k = v^k D^k p(t*)`), `ComponentPeaks()` (P_k per component: grid of 1000 intervals, then golden-section refinement of the best sample of each component), `Peaks()`, `AdmissibleSpeed(limits)` = `(v_adm, binding limit name)`.
- `ConstantSpeedTrajectory3D`: entry transition (`PolynomialSegment3D`) + stretch + exit transition. Constructor takes the segment and the subpath as segment times; factories for the three specifications: `FromParameterFractions` (τ), `FromPoints` (closest point via `BlendBuilder3D.TauForPoint`, rejected beyond 1e-9 of the extent), `FromArcLengthFractions` (inversion over the whole segment). Transitions have the stretch's **full** boundary state (non-zero a/j/s on a curve); their durations use the 0015 machinery (`TransitionDurationRules`, multi-component `ShortestFeasibleDuration.Find`, Euclidean distance and speed norms in Rule 2); `EvaluateState/Position/Derivative(t)`; `Sample(count, maxOrder)` → new `TrajectorySamples3D(Times, Point3D[order][i])`, the 3D counterpart of `TrajectorySamples` and the same call shape as `Trajectory1D.Sample`.
- **Limits**: v_cv > v_adm (1 + 1e-12) → `KinematicLimitsViolatedException` stating v_adm and the binding limit ("…exceeds the admissible speed 1.70998 of this stretch: the jerk limit binds. The time law … is fixed … so no transition can repair it."); transitions verified per axis (`VerifyLimits`); a total below the minimum feasible one fails as in 0015.
- 0015 interplay: I did not make `ConstantVelocityTrajectory3D.OnSegment` return the new type. It stays the straight-chord API; its `NotSupportedException` message for a curved subpath no longer says "not implemented yet" but points to `ConstantSpeedTrajectory3D` (one assertion of the 0015 test adapted). `Weight`/`Verify` of the 0015 class became `internal` and are shared.

## Acceptance criteria — where verified (`ConstantVelocity/ConstantSpeedTrajectory3DTests.cs`, 28 cases)

| Criterion | Test and numbers |
|---|---|
| Parabola reference: derivatives, length, peaks, v_adm | `ParabolaDerivatives_AtTheStart/AtTheEnd`: exact to 1e-9 at t = 0 and for p_s, p_ss at t = 1; p_sss, p_ssss at t = 1 to the six digits the theory prints (1e-6, 1e-5). `ParabolaLength`: relative 1e-12 of (2√5 + asinh 2)/4. `ParabolaPeaks`: all eight component peaks to 1e-5 **except P₄ₓ**, see Discrepancies. `ParabolaAdmissibleSpeed`: 1.709976, jerk binds |
| Independent check of the derivatives on a space curve | `SpaceCurve_DerivativeOfOrderK…` (degree 5, three parameters): D^(k+1)p equals the central difference of D^k p divided by σ (1e-6), p_s = ṗ/σ (1e-12, unit norm), p_ss from the closed form (1e-10) |
| Inversion | `Inversion_RoundTripsRandomParameters`: 200 random t*, ℓ⁻¹(ℓ(t*)) = t* to 1e-12; t_e at ℓ = L exactly. `…StronglyVaryingSpeed`: rest-to-rest path on [0.04, 0.96] with σ_max/σ_min > 10³, ℓ(ℓ⁻¹(ℓ*)) = ℓ* to 1e-12 L (see remark) |
| Constant speed, on the path, T_cv = L/v, inversion at T_cv | `TheStretch_HasConstantSpeed…`: speed 1.7 to 1e-9 at 61 times; every position is on the polynomial path (TauForPoint accepts it, 1e-9 of the extent) and the travelled length equals v t; `T_cv` → t_e to 1e-12 |
| Stretch derivatives v^k D^k p; centripetal | `TheStretchDerivatives_…` (1e-9, v·a = 0 to 1e-9); `TheTimeDerivatives…FiniteDifferences` (derivatives of order k+1 = difference of order k) |
| Junction continuity, degrees 5, 7, 9 | `TheJunctions_…` ×3: orders 0..2, 0..3, 0..4 at both junctions to 1e-9; non-zero acceleration on the curve; degree 3: position and velocity |
| Straight segment through the general code = 0015 | `AStraightSegment_…` (with and without limits): durations to 1e-9, a/j/s at the junction below 1e-9, states of the two trajectories equal at 41 times, orders 0–4 (1e-8) |
| Specifications give the same stretch | `ParametersPointsAndLengthFractions_…` (durations 1e-8, boundary jets 1e-7); point off the path, reversed or out-of-range fractions rejected |
| v > v_adm, rest point, limits on transitions, rule 3 | `ASpeedAboveTheAdmissibleSpeed…` (message has "1.70998" and "jerk"), `ASubpathThroughARestPoint…`, `Rule1_…` (entry equals the multi-component Find, pieces pass `VerifyLimits`, stretch sampled within the caps), `Rule3_…` (exact total, same proportions, too-short total fails), `Rule2_…` |

## Discrepancies between theory and code

1. **Reference P₄ₓ = 18.187075** (theory, "sampling, 2001 points"). The code, which refines the grid maximum, gives **18.187110**: the true maximum is 3.5·10⁻⁵ above the sampled one. The criterion (peaks to 1e-5) cannot hold for this one value against a *sampled* reference, so the test checks (a) the refined value to 1e-4 and ≥ the reference, and (b) an independent 2001-point sampling of D⁴p to 1e-5 of 18.187075 (it matches). The other seven peaks agree to 1e-5 directly. Suggest the theory say "true maxima" or give 18.18711.
2. The strongly-varying-σ inversion test checks the *length* round trip (ℓ(ℓ⁻¹(ℓ*)) = ℓ*) to 1e-12 L, not t* to 1e-12: where σ is 10⁻³–10⁻⁴ of its maximum, a length tolerance of 10⁻¹³ L corresponds to an error 10³–10⁴ times larger in t (inherent in inverting a nearly flat ℓ). The t-round trip to 1e-12 holds where σ is moderate (parabola test).

## Findings worth knowing

- **Limits can make the transitions impossible even when the stretch is admissible.** With a_max = 8, j_max = 20 and v_cv = 1.7 (just below v_adm) the stretch passes, but the boundary jerk is almost the limit (v³·4 = 19.65), and a degree-9 transition ending at that jerk cannot stay under 20 for any duration: the entry search reports `DurationSearchFailedException` ("The entry transition: no duration … satisfies the active kinematic limits"). That is the existing, explicit behaviour (the same as a boundary velocity near the velocity limit). The test of v_adm therefore checks the stretch alone just below v_adm and the transitions only with looser limits.
- Cost: `ComponentPeaks` takes 1001 jet evaluations plus 60-step refinements for 12 (order, axis) pairs; it is computed once and cached. Each `Stretch.EvaluateState` is one inversion (several quadratures) plus one jet; sampling 300–400 points is instant, but a UI that resamples at mouse speed may want a cache (not needed now).

## For the author (educational)

- **The derivative with respect to arc length.** Time can be replaced by the distance s travelled along the curve as the independent variable. Then p_s = dp/ds is the unit tangent (moving one unit of length moves the point by one unit along the curve), and p_ss = d²p/ds² measures how fast that direction turns: its length is the curvature κ and it points to the centre of the bend. These are *geometric* quantities, independent of how fast the curve is run. At constant speed v the distance is s = v t, so one time derivative is just v times one s-derivative: velocity = v p_s, acceleration = v² p_ss, jerk = v³ p_sss, snap = v⁴ p_ssss.
- **Why constant speed still has acceleration.** Speed is the *length* of the velocity vector; acceleration is the change of the whole vector, and on a curve the direction changes even if the length does not. The change is perpendicular to the motion, toward the centre of the bend, with magnitude v²κ: the centripetal acceleration. Only on a straight line (κ = 0) is constant speed also zero acceleration, which is why the straight case reduced to a constant velocity vector with zero a, j, s.
- **Why the speed limit cannot be repaired by the transitions.** The transitions are free pieces whose durations can be stretched, but the stretch is not: its shape is the original curve and its time law is fixed by the single number v_cv. Its accelerations grow like v², jerk like v³, snap like v⁴ with coefficients (the peaks P_k of the geometry) that no timetable change can touch. So for a given curve there is a largest admissible speed, v_adm = min_k (L_k/P_k)^(1/k); above it the stretch itself breaks a limit and the program says so (with the limit that binds) instead of silently changing the path or the speed.
- **How series arithmetic avoids long formulas.** D³p and D⁴p written symbolically are very long expressions in the derivatives of p. Instead, around one parameter value we write each component as a short polynomial (its Taylor series, 5 terms) and do the algebra *on the series*: products, square root, reciprocal are a few lines of loops. Each application of D = (d/dt)/σ differentiates the series (losing one term) and multiplies by the series of 1/σ; after four applications the constant term of the series is exactly D⁴p. The computer does the symbolic bookkeeping that would otherwise be done by hand, and the check against the closed form for D²p and against finite differences for the rest confirms it.

## Open questions

1. Should `ConstantVelocityTrajectory3D.OnSegment` (straight, 0015) be kept next to `ConstantSpeedTrajectory3D`, or removed now that the general class reproduces it (tests show equality to 1e-9)? I kept both: the straight one is simpler and has no numerical integration.
2. Several consecutive stretches and the `Trajectory3D`-level builder are out of scope; the stretch is not a `PolynomialSegment3D`, so `Trajectory3D` cannot hold the curved trajectory yet (a common interface or an evaluation delegate would be needed when R10b starts).
3. `RegularityTolerance = 1e-9` (σ_min/σ_max) is my value; the theory only asks for σ > 0.

## Suggested commits

1. `code: arc-length derivatives by series arithmetic, arc length and inversion` — `src/TrajectoryPlanner.Core/ArcLength/TaylorSeries.cs`, `src/TrajectoryPlanner.Core/ArcLength/ArcLengthDerivatives.cs`, `src/TrajectoryPlanner.Core/ArcLength/ArcLengthPath.cs`
2. `code: constant speed along a curved subpath (stretch, transitions, limits, sampling)` — `src/TrajectoryPlanner.Core/ConstantVelocity/ConstantSpeedTrajectory3D.cs`, `src/TrajectoryPlanner.Core/ConstantVelocity/ConstantVelocityTrajectory3D.cs`, `src/TrajectoryPlanner.Core/Trajectories/TrajectorySamples.cs`
3. `code: tests for the curved constant-speed trajectory` — `src/TrajectoryPlanner.Core.Tests/ConstantVelocity/ConstantSpeedTrajectory3DTests.cs`, `src/TrajectoryPlanner.Core.Tests/ConstantVelocity/ConstantVelocityTrajectory3DTests.cs`
4. `coord: close request 0016` — `coordination/inbox-code/0016-core-3d-curved-constant-speed.md`, `coordination/inbox-docs/0016-core-3d-curved-constant-speed.md`, `coordination/STATUS.md`

(The work of 0013–0015 may still be uncommitted. `roadmap/`, `theory/`, `design/` untouched.)
