# 0004 — Test coverage for the untested areas (return note)

Status: closed (reviewed by Cowork 2026-10-17) · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0004-test-coverage.md`

## What was done

56 new tests in five new files; no change to `src/TrajectoryPlanner.Core` or the App. `dotnet build`: 0 errors (3 NU1701 warnings, unchanged). `dotnet test`: **137/137 passed** (81 before this request). No test had to be marked expected-to-fail, and no defect was found; but several tests pin behaviour that the theory does not prescribe (see *Findings*).

| Item | File | Tests |
|---|---|---|
| 1 Blending | `Blending/BlendBuilderTests.cs` | 16 cases |
| 2 Degree 7 | `Polynomials/Degree7Tests.cs` | 3 |
| 3 Trajectories | `Trajectories/TrajectoryTests.cs` | 8 (1D) + 4 (3D) |
| 4 Extrema | `Constraints/ExtremaAndDurationTests.cs` (`ExtremaAnalyticTests`) | 11 cases |
| 5 Duration strategies | same file (`DurationStrategyTests`) | 8 cases |
| 6 Large pass-through | `PassThrough/LargePassThroughTests.cs` | 6 cases |

## Findings

1. **Blending, "invalid input" (D7).** Today only τ outside [0, 1] is rejected (`ArgumentOutOfRangeException`). Not rejected, and pinned by `Today_…` tests:
   - an absolute blend position **outside** the segment (e.g. x = 5 on a segment ending at 2) is silently clamped (τ clamp), no error;
   - an **empty window** (τ = 1 on the preceding and τ = 0 on the following segment) is accepted; the blend duration is floored at 0.01 s;
   - a "reversed" window cannot be detected in τ terms; there is no check for it.
   Also pinned: blend degree 9 by default (even between degree-3 neighbours); absolute positions can never give τ exactly 0 or 1 (clamp [0.001, 0.999]); inherited duration floor 0.01 s. All of these are in tests named `Today_…`, so the consolidation can change them knowingly.
2. **Blend does not pass through the nominal join.** Verified on a 1D corner (velocity reversal at x = 2, blend between τ = 0.8 and 0.2): the blend stays more than 1e-3 away from the join. In a *monotone* 1D path the blend necessarily crosses the join *position*, so the test uses a reversal; the theory's statement (section *1D Blend Does Not Need To Pass Through The Nominal Join*) is meaningful for corners.
3. **Pass-through at N = 30 (item 6).** 30 segments, durations log-uniform in [0.05, 5] with the extremes forced to exactly 0.05 and 5.0 (ratio 100), positions a random walk. Cholesky succeeded for all degrees and orders; accuracy (measured 2026-10-17):

   | degree, m | cond(H) | relative gradient | J | quadratic form vs exact J (relative) |
   |---|---|---|---|---|
   | 3, 2 | 3·10¹ | 4·10⁻¹⁷ | 4.7·10⁴ | 5·10⁻¹⁶ |
   | 5, 3 | 1·10⁶ | 4·10⁻¹⁶ | 7.8·10⁶ | 2·10⁻¹² |
   | 7, 4 | 4·10¹⁰ | 3·10⁻¹⁴ | 7.2·10⁹ | 5·10⁻⁹ |
   | 9, 4 | 6·10¹³ | 3·10⁻¹² | 8.9·10⁹ | 9·10⁻⁷ |
   | 9, 5 | 5·10¹⁵ | 6·10⁻¹² | 1.6·10¹³ | 2·10⁻⁵ |

   Passage through the waypoints and continuity of the derivatives up to order p−1 at all 29 interior nodes hold to 1e-6 relative in every case. **What degrades** is not the solution but the *evaluation of J through the quadratic form* `PassThroughProblem1D.Cost(y)`: the constant term is up to 6·10¹⁹ against J = 1.6·10¹³ (cancellation, ~6 digits lost for d = 9, m = 5). `SmoothnessCost.Of(segments, m)` (the exact integral from the coefficients) does not have this problem, and should be the one used for reports. The test tolerance for the agreement of the two is therefore tied to the constant (`1e-11·const + 1e-9·J`), stated in the test.
   Condition numbers around 5·10¹⁵ for d = 9, m = 5 are close to the limit of double precision (about 10¹⁶); with larger ratios of durations or more segments the solve for m = 5 could start failing. Not observed up to the sizes tested; an `ArithmeticException`/non-positive-definite error from the Cholesky factorization would be the symptom.
4. **`Trajectory1D.FindSegment` at an interior node** returns the *following* segment with local time 0 (and the last segment at the total duration); a tolerance of 1e-12 above the total duration is accepted, anything beyond, or negative time, throws `ArgumentOutOfRangeException`. Pinned in `OutsideTheRange_…`.

## Table: Core folder → tests (update of note 0001, item 3; total now 137)

| Core folder | Tests that exercise it | Judgement now |
|---|---|---|
| `Symbols` | 2 (foundation) | smoke (Epsilon removed in 0003) |
| `Points` | 2 (foundation) + 3D usage in several | smoke |
| `States` | 1 + indirect everywhere | smoke |
| `BoundaryConditions` | 2 | thin (failure paths of `ValidateConditions*` still untested) |
| `Polynomials` | 3 evaluator + 3 degree-7 + indirect | good |
| `CoefficientDetermination` | 4 + 3 degree-7 (all four degrees now covered) | good |
| `Segments` | 2 + degree-7 ×3 + many indirect; `PolynomialSegment3D` via 3D builder/trajectory | fair |
| `Trajectories` | 8 (1D) + 4 (3D) + `SampleTimes` ×5 | good |
| `Constraints` | 3 old + 11 analytic extrema/limits + 4 failure + 4 `ConstraintAware` | good for rest-to-rest; non-zero boundary derivatives and degree-5/7 extrema with nonzero boundaries not covered |
| `Blending` | 16 (incl. `Today_…`) | fair (1D only; 3D blending does not exist) |
| `ConstantVelocity` | 1 | **still smoke** (not in this request) |
| `PassThrough` | 43 (request 0002) + 6 large-case | good |
| `Services` | builders 3 + 0002 duration/builder tests ×~6 + strategy tests 8 + proportional ×6 | fair |
| App (`PrototypeTrajectoryService`) | 0 (outside `Core.Tests` by rule) | none |

## Why (choices made)

- Blending fixtures use degree 9 and a velocity-reversing corner so that all five derivatives (position … snap) are non-trivial.
- Rest-to-rest peak velocity is checked for the four degrees against 3/2, 15/8, 35/16, 315/128 at relative 1e-9; location T/2 at 1e-6 (the maximum is found from the sampling/root candidates, not analytically).
- "Outside the range" for trajectories documents the actual exception, because the request asked to state the behaviour.

## Discrepancies between theory and code

None new. The `Today_…` blend tests are the executable form of D7.

## Open questions

1. Should `PassThroughProblem1D.Cost` (cancellation) be removed from the public API, or documented as unreliable for large constants? I would keep it for the optimality tests and use `SmoothnessCost.Of` for reports.
2. Do you want a test request for `ConstantVelocity` (still one smoke test) before the consolidation?

## Cowork review (2026-10-17)

Accepted; the `Today_…` tests are the executable form of D7 for the consolidation.

1. **`Cost(y)`**: keep, document as unreliable for large constants; reports use `SmoothnessCost.Of` (request 0006).
2. **ConstantVelocity tests**: not yet. The transition durations are not specified by the theory (D8; a rule is proposed to the author), so tests would pin behaviour that is about to change. They come right after the rule is decided.
3. **Conditioning** ($5 \cdot 10^{15}$): a diagonal scaling brings it down by 4–6 orders of magnitude; request 0006.

## Suggested commits

1. `code: tests for blending, degree 7, trajectories, extrema, duration strategies, large pass-through` — `src/TrajectoryPlanner.Core.Tests/Blending/BlendBuilderTests.cs`, `…/Polynomials/Degree7Tests.cs`, `…/Trajectories/TrajectoryTests.cs`, `…/Constraints/ExtremaAndDurationTests.cs`, `…/PassThrough/LargePassThroughTests.cs`
2. `coord: close request 0004` — `coordination/inbox-code/0004-test-coverage.md`, `coordination/inbox-docs/0004-test-coverage.md`, `coordination/STATUS.md`
