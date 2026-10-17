# 0003 — Cleanup batch

Status: open · From: Cowork · To: Code · Date: 2026-10-17

## Goal

Fix the small, well-understood defects found in your notes 0001 and 0002, without changing any numerical behaviour of the working features. Run this request **before** 0004 (tests), so that the new tests exercise the corrected code.

## Reference documents

- `roadmap/discrepancies.md` — D5 (failure signalling only), D9, D12, D13, D14.
- `design/07-implementation-plan.md` — appendix A8 (proportional duration rule).
- Your notes `coordination/inbox-docs/0001-…` (items 6.5, 6.9, open question 4) and `0002-…` (open question 2).

## Tasks

1. **Coefficient labels (D9).** In the App coefficient report use `c0 … cd` (not `a0 … a9`), following `theory/SymbolsAndNomenclature.md`. Check for any other `a_i`-style coefficient naming in the App and Core and list it in the note.
2. **`DurationSearcher` failure (D5, bug part only).** When no duration satisfying the limits is found after the maximum number of doublings, the failure must be visible to the caller (for example a result with a success flag, or an exception; your choice, say which and why). Do **not** change when the initial guess is returned or how the search proceeds: that is an author decision still pending. Update the callers (App, `ConstraintAwareDuration`) so that the failure reaches the user as a message instead of a silent wrong duration.
3. **`Trajectory1D.SampleTimes` (D12).** Remove the `IndexOf`-based loop (use the index), make it correct when the same segment object appears twice, and reject `samplesPerSegment < 2` with a clear exception.
4. **`Symbols.Epsilon` (D13).** Delete it if nothing needs it. If a test needs it, keep it and say so. Do not replace the literal tolerances in the code.
5. **Proportional durations (D14).** Change `AutoDerivedProportionalDuration` to the rule of `design/07` A8: weights $w_i = \max(\Delta_i, \varphi \bar{\Delta})$, durations $T_i = T_{tot} w_i / \sum w$, $\varphi = 0.05$ as a constructor parameter, total preserved exactly, equal split if all distances are zero, Euclidean distance in 3D. Remove the absolute 0.05 s floor. Leave the per-segment `ComputeDuration` as it is (it goes away in the consolidation).

## Out of scope

Anything that changes results of existing features (except the durations in task 5, which only affects pass-through with automatic durations); `DurationSearcher` search behaviour; blending; constant velocity.

## Acceptance criteria

- [ ] `dotnet build` and `dotnet test` succeed; previous tests still pass (or are updated with a reason stated in the note).
- [ ] New tests: `SampleTimes` (repeated segment object; one sample rejected); `DurationSearcher` failure is reported (a limit that cannot be met, e.g. a tiny maximum velocity with a huge distance and the search cap); proportional durations (sum equals total exactly; weights proportional to distance; zero-distance segment gets the floor share; all-zero distances give an equal split; 3D Euclidean).
- [ ] The App shows `c0 … cd` in the coefficient report (state in the note that you looked at the code only if you did not run the app).
- [ ] Return note `coordination/inbox-docs/0003-cleanup-batch.md` and `coordination/STATUS.md` updated.
