# 0010 — Blend duration rule and number parsing

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

## Goal

Two small changes after the review of 0009: (1) with limits active, a blend keeps its inherited duration when that is feasible and takes the shortest feasible duration only when it is not (author's decision, D19); (2) the UI parses numbers without the thousands-separator ambiguity on Italian regional settings (D20).

## Reference documents

- `theory/BlendingSegments.md`, *Decisions For The 1D Baseline (2026-10-17)*
- `roadmap/discrepancies.md`: D19, D20; note `coordination/inbox-docs/0009-consolidation-r7.md` (odd thing 4 and open question 5)

## Scope

- In scope:
  1. **Blend duration.** `BlendBuilder1D`: with no prescribed duration and limits active, compute the inherited duration (as without limits, with its floor); if the blend with that duration passes `VerifyLimits`, keep it; otherwise use `ShortestFeasibleDuration.Find` (and `?? inherited` when it returns null); verify as now; errors as now. A prescribed duration is only verified (unchanged). Without limits nothing changes.
  2. Tests: the tests of 0009 that expect the shortest feasible blend when the inherited one was feasible change deliberately (say which in the note); add: inherited feasible → kept (total duration unchanged), inherited infeasible → shortest feasible, still-infeasible limits → explicit error naming the blend.
  3. **Number parsing in the UI layer** (`ParseRequiredDouble`, `ParsePositiveDouble`, and any other `double.TryParse` of user text): `NumberStyles.Float` **without** `AllowThousands`, first with the current culture, and if that fails with the invariant culture. Effect: on an Italian PC "1,5" and "1.5" both give 1.5; "1.234,5" and "1,234.5" are rejected with a clear message; on an English PC "1.5" works and "1,5" is rejected. Keep the number output as it is today; if outputs use a different culture than the inputs, report it in the note without changing it.
  4. Tests for the parsing with explicit cultures (`it-IT`, `en-US`, invariant) for the cases above, in `App.Tests`; the tests must not depend on the machine's culture (set it explicitly and restore it).
- Out of scope: R9–R11, unused public members, other UI changes.

## Acceptance criteria

- [ ] Blend: inherited duration kept when feasible (total trajectory duration unchanged); otherwise the shortest feasible; the whole trajectory passes `VerifyLimits`.
- [ ] Parsing: the cases listed above pass under `it-IT` and `en-US`.
- [ ] `dotnet build` free of new warnings; all tests pass; the note states which tests were rewritten and why.

## Expected tests

- Blend with `v_max = 8` on the 0009 example: inherited 0.643 s is feasible → kept (the shortest feasible result of 0009 was 0.100 s). A tighter limit that makes the inherited blend infeasible → shortest feasible.
- Parsing matrix per culture.

## Notes for the author (educational)

Explain in the return note why the culture of `double.TryParse` matters (the same text means different numbers in different cultures; "1.5" can be 15 where the dot is a thousands separator) and why rejecting thousands separators removes the ambiguity.

Return note: `coordination/inbox-docs/0010-blend-duration-and-parsing.md`, ending with a "Suggested commits" section.
