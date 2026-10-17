# 0006 — Numerical conditioning and two duration fixes (return note)

Status: open · From: Code · To: Cowork · Date: 2026-10-17 · Request: `coordination/inbox-code/0006-numerics-and-duration-fixes.md`

## What was done

1. **Diagonal scaling (D16).** `PassThroughProblem1D.Solve()` now solves `(S H S) z = −S f` with Cholesky and returns `y = S z`, `S = diag(1/√H_ii)`. `H`, `F`, `Cost`, `Gradient` unchanged. New public helper `DiagonalScale()` (returns the vector of the s_i) so that tests and reports can build `S H S` themselves.
2. **`Cost(y)`.** Kept public; its XML comment now says it is unreliable for large constants (cancellation, measured up to 4·10⁶ × J) and names `SmoothnessCost.Of` as the function for reports.
3. **"Not determined by the limits" (D17).** `ShortestFeasibleDuration.Find` and `RestToRest` now return `double?`: **`null`** when there is no active limit or when the segment is rest-to-rest with Δ = 0. *Why nullable and not `TryFind(out …)`*: the callers need one expression, `Find(...) ?? fallback`; and the other outcomes keep their own signal (impossible limits still throw `DurationSearchFailedException`; invalid limits `ArgumentException`), so `null` has exactly one meaning. The 1 ms `MinimumDuration` is now only the floor of the *start* of the search (documented as such); the closed form is no longer clamped (a 1 µm move gives its true tiny T*, tested).
   - `ConstraintAwareDuration(limits, fallbackDuration = 1.0)`: new `FallbackDuration` (validated > 0), used when `Find` returns `null`.
   - App: `ShortestDurationOrExplain` returns `double?`; `Compute` uses `?? DefaultDuration` (new constant, 1.0 s, same value as before).
4. **Start bound — confirmation.** The code matches the theory text (`TrajectoryConstraints.md`, search step 1): for every active limit L_k whose boundary derivative of order k−1 is prescribed, T ≥ |b_f − b_0| / L_k (for k = 1 the positions), largest of these and a small minimum. The only addition not literally in the text is the guard "order k−1 ≤ p−1" (for a degree whose state does not prescribe that derivative, e.g. a snap limit with degree 3, the bound is skipped); it is implied by "prescribed".

## Numbers requested (task 1)

Data: the N = 30 case of request 0004 (log-uniform durations 0.05–5, ratio exactly 100; two random seeds). Condition numbers, `cond(H)` → `cond(S H S)`:

| degree, m | seed 2026 | seed 7 |
|---|---|---|
| 5, 3 | 1.1·10⁶ → 1.3·10³ | 1.5·10⁶ → 8.1·10² |
| 7, 4 | 4.1·10¹⁰ → 1.2·10⁷ | 7.0·10¹⁰ → 3.5·10⁶ |
| 9, 4 | 5.9·10¹³ → **2.1·10⁷** | 2.8·10¹³ → 6.1·10⁶ |
| 9, 5 | 5.5·10¹⁵ → **2.3·10¹¹** | 6.8·10¹⁵ → 3.7·10¹⁰ |

The improvement is 4 to 5 orders of magnitude for degree 9 (Cowork's prototype: 3·10¹⁵ → 2·10⁹ and 4·10¹³ → 1·10⁶; my scaled numbers are somewhat larger, 2·10¹¹ and 2·10⁷, on a different random case; same direction and order of improvement).

## Acceptance criterion on the exact cost: what I measured (please read)

The request asks that `SmoothnessCost.Of` of the scaled solution be **not larger than** that of the unscaled one "within 10⁻¹² relative". Measured relative difference (scaled − unscaled)/unscaled:

| degree, m | seed 2026 | seed 7 |
|---|---|---|
| 5, 3 | −1.3·10⁻¹⁵ | −9·10⁻¹⁶ |
| 7, 4 | +7·10⁻¹⁶ | −1.7·10⁻¹⁴ |
| 9, 4 | **+1.3·10⁻¹²** | −6.8·10⁻¹² |
| 9, 5 | **−1.5·10⁻⁹** | **+1.6·10⁻⁹** |

So the criterion as written (10⁻¹²) is **met for degrees 5 and 7, marginally missed for d = 9, m = 4 (1.3·10⁻¹²) and not met for d = 9, m = 5, where the sign changes with the data**. My reading: the two solutions differ in y by about 10⁻⁶ relative for m = 5 (10⁻¹⁰–10⁻¹¹ for m = 4), the cost is flat to first order at the minimum, so such a difference in y cannot change J by 10⁻⁹; the 10⁻⁹ is the rounding noise of evaluating J itself from coefficients of size ~10⁶–10¹⁰. The residual of the normal equations is identical to all digits shown (e.g. 6.1·10⁻¹² both). So I cannot say from J which solution is the better one; the better conditioning is the reliable gain (y is determined to about 10⁻¹⁶ × 10¹¹ instead of ×10¹⁵). I did not tune the tolerance to hide this: the committed test `ScaledSolve_HasAMuchSmallerConditionNumber_AndANotLargerExactCost` asserts cond(S H S) < cond(H)/100 and scaled cost ≤ unscaled × (1 + 10⁻⁸), with the above explanation in the comment. If you want the stricter criterion, the measurement would need a higher-precision cost evaluation (e.g. extended precision), which I consider out of proportion.

The optimality test `Solution_IsTheMinimizer` (gradient < 10⁻⁷, 100 random perturbations increase J) still passes for all five (degree, m) combinations with the scaled solve.

## Tests

- Added: `ScaledSolve_HasAMuchSmallerConditionNumber_AndANotLargerExactCost` (4 cases), `ScaledSolve_SatisfiesTheNormalEquationsOfTheOriginalSystem` (`LargePassThroughTests.cs`); in `ShortestFeasibleDurationTests`: zero distance → `null` (4 degrees), very short distance still answered (1.5·10⁻⁶ for degree 3), zero-length with non-zero boundary velocity still determined, no active limit → `null`; in `ExtremaAndDurationTests`: `ConstraintAwareFallbackTests` (zero length → fallback, no limit → default 1 s, determined case ignores fallback, invalid fallback rejected). Replaced: the two old tests that expected a 1 ms answer / an exception.
- `dotnet test`: **188/188 passed** (173 + 15). `dotnet build` of Core and tests: 0 errors. The App project: 0 errors when compiled to a separate output folder; building it in place failed once only because the author's copy of the app was running and had locked `TrajectoryPlanner.Core.dll` (MSB3021), which is not a code problem.
- App path (as for 0005, throwaway console project in the scratchpad calling `PrototypeTrajectoryService.Compute`; the WPF window itself was not run): zero-length segment with an active velocity limit → `T [s]` row = 1; no limit → 1; normal case → 2.4609375.

## Behaviour changes

- Pass-through results are numerically the same minimizer, with differences at the 10⁻⁶ relative level in y for d = 9, m = 5 on badly scaled problems (see above); invisible in normal cases.
- A zero-length rest-to-rest segment with limits and no prescribed duration now gets the default duration (1.0 s in the App, `FallbackDuration` in the strategy) instead of 1 ms; before 0005 the legacy search gave 1.0 s as well.
- `ShortestFeasibleDuration.Find/RestToRest` return type changed to `double?` and `ConstraintAwareDuration` has a new optional constructor parameter (compatible for existing calls).

## Discrepancies between theory and code

None.

## Open questions

1. Strict 10⁻¹² criterion on the exact cost: see above; do you accept the 10⁻⁸ bound with the explanation, or do you want an extended-precision check?
2. The conditioning after scaling is still 2·10¹¹ for d = 9, m = 5 at ratio 100; with ratio 10⁴ or hundreds of segments a block-wise (per-node) scaling or a banded solve might be needed. Not needed for the project sizes so far.

## Suggested commits

1. `code: diagonal scaling in the pass-through solve; "not determined" outcome for shortest duration with fallback` — `src/TrajectoryPlanner.Core/PassThrough/PassThroughProblem1D.cs`, `src/TrajectoryPlanner.Core/Constraints/ShortestFeasibleDuration.cs`, `src/TrajectoryPlanner.Core/Services/IDurationStrategy.cs`, `src/TrajectoryPlanner.App/PrototypeTrajectoryService.cs`, `src/TrajectoryPlanner.Core.Tests/PassThrough/LargePassThroughTests.cs`, `src/TrajectoryPlanner.Core.Tests/Constraints/ShortestFeasibleDurationTests.cs`, `src/TrajectoryPlanner.Core.Tests/Constraints/ExtremaAndDurationTests.cs`
2. `coord: close request 0006` — `coordination/inbox-code/0006-numerics-and-duration-fixes.md`, `coordination/inbox-docs/0006-numerics-and-duration-fixes.md`, `coordination/STATUS.md`
